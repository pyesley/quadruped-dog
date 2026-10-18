#!/usr/bin/env python3
"""
Interactive jog for two moteus motors (ids 1 and 2).

  1. Prompts you for a target position (revolutions) for each motor and
     moves there.  Enter on an empty line keeps the motor where it is.
  2. Drops into "jog" mode where single keys nudge each motor by --step
     (0.2 rev by default) -- no Enter needed:

         motor 1:   a = -0.2      d = +0.2
         motor 2:   j = -0.2      l = +0.2

         space      print current positions
         p          type new target positions again
         q / Ctrl-C stop motors and quit

  From a real terminal keys act instantly.  From an IDE run console (e.g.
  PyCharm) stdin is not a tty, so type the key and press Enter.

  Whenever a move finishes (both motors report trajectory complete) the
  measured positions are printed as feedback.

Same mechanics as step.py: the position command is re-sent every cycle
(~100 Hz) so the moteus watchdog never trips, `velocity_limit`/`accel_limit`
shape the move, and arrival comes from the TRAJECTORY_COMPLETE register.
The keyboard is polled from inside that same loop (terminal in cbreak mode,
no threads), so the CAN stream never pauses while you type.

Run:
  DRY RUN (no hardware; a fake transport simulates the motors):
      python3 jog.py --dry-run

  LIVE:
      python3 jog.py
      python3 jog.py --step 0.1 --vel 2.0 --accel 20.0
"""

import argparse
import asyncio
import contextlib
import math
import os
import select
import sys
import termios
import tty

import moteus
import moteus.multiplex as mp


MOTOR_IDS = (1, 2)

# key -> (motor id, direction)
JOG_KEYS = {"a": (1, -1), "d": (1, +1),
            "j": (2, -1), "l": (2, +1)}

# Replies right after a target change can still carry the previous
# trajectory's "complete" flag (moteus's own wait_complete skips 2).
SETTLE_CYCLES = 3


class Jog:
    def __init__(self, args):
        self.args = args
        self.transport = (_FakeTransport(args.rate) if args.dry_run
                          else moteus.Fdcanusb())

        qr = moteus.QueryResolution()
        qr.trajectory_complete = mp.INT8          # off by default
        self.ctrl = {mid: moteus.Controller(id=mid, transport=self.transport,
                                            query_resolution=qr)
                     for mid in MOTOR_IDS}

        self.target = {}                          # empty => not commanded yet
        self.actual = {mid: None for mid in MOTOR_IDS}
        self.awaiting_arrival = False
        self.cycles_since_change = 0

        self.pending = {}                         # values typed so far
        self.prompt = None                        # _Prompt while typing targets
        self.echo = os.isatty(sys.stdin.fileno()) # cbreak => we echo ourselves
        self.quit = False

    # ------------------------------------------------------------------ CAN
    async def can_cycle(self):
        """Send this cycle's commands to both motors and digest the replies."""
        if self.target:
            cmds = [self.ctrl[m].make_position(position=self.target[m],
                                               velocity=0.0,
                                               velocity_limit=self.args.vel,
                                               accel_limit=self.args.accel,
                                               query=True)
                    for m in MOTOR_IDS]
        else:
            cmds = [self.ctrl[m].make_query() for m in MOTOR_IDS]

        results = await self.transport.cycle(cmds)
        by_id = {r.id: r for r in results}

        for m in MOTOR_IDS:
            r = by_id.get(m)
            if r is None:
                continue
            _check_fault(r, m)
            p = r.values.get(moteus.Register.POSITION)
            if p is not None:
                self.actual[m] = p

        self.cycles_since_change += 1
        if (self.awaiting_arrival
                and self.prompt is None
                and self.cycles_since_change > SETTLE_CYCLES
                and all(_complete(by_id.get(m)) for m in MOTOR_IDS)):
            self.awaiting_arrival = False
            print(f"[✓] arrived   {self.positions_str()}")

    async def stop(self):
        if not self.args.dry_run:
            await self.transport.cycle([c.make_stop() for c in self.ctrl.values()])

    # ----------------------------------------------------------------- keys
    def handle_key(self, ch):
        if self.prompt is not None:
            self._handle_prompt_key(ch)
            return

        if ch in JOG_KEYS:
            m, sign = JOG_KEYS[ch]
            self.set_target(m, self.target[m] + sign * self.args.step)
            print(f"    motor {m} target -> {self.target[m]:+.3f}")
        elif ch == " ":
            print(f"    now   {self.positions_str()}")
        elif ch == "p":
            self.start_prompt()
        elif ch == "q":
            self.quit = True

    def set_target(self, m, value):
        self.target[m] = round(value, 4)
        self.cycles_since_change = 0
        self.awaiting_arrival = True

    def positions_str(self):
        parts = []
        for m in MOTOR_IDS:
            a = self.actual[m]
            s = f"motor {m}: {a:+.3f}" if a is not None else f"motor {m}:   n/a "
            if m in self.target:
                s += f" (cmd {self.target[m]:+.3f})"
            parts.append(s)
        return "   ".join(parts)

    # --------------------------------------------------------------- prompt
    def start_prompt(self):
        self.awaiting_arrival = False   # new targets coming; old arrival moot
        self.pending = {}
        self.prompt = self._prompt_for(MOTOR_IDS[0])

    def _prompt_for(self, m):
        return _Prompt(m, self._default_for(m), self.echo)

    def _default_for(self, m):
        return self.target.get(m, self.actual[m])

    def _handle_prompt_key(self, ch):
        line = self.prompt.feed(ch)
        if line is None:
            return                                    # still typing
        m = self.prompt.motor
        text = line.strip()
        try:
            value = self.prompt.default if text == "" else float(text)
        except ValueError:
            value = None
        if value is None:
            print(f"    '{text}' is not a number, try again")
            self.prompt = self._prompt_for(m)
            return

        self.pending[m] = value
        nxt = m + 1
        if nxt in MOTOR_IDS:
            self.prompt = self._prompt_for(nxt)
            return

        # All motors entered: commit both targets in the same cycle so the
        # CAN loop never sees a half-filled target dict.
        self.prompt = None
        for k in MOTOR_IDS:
            self.set_target(k, self.pending[k])
        self.pending = {}
        print(f"[*] moving to  " + "   ".join(
            f"motor {k}: {self.target[k]:+.3f}" for k in MOTOR_IDS))
        print(self.help_str())

    def help_str(self):
        s = self.args.step
        return (f"[jog]  motor 1: a {-s:+.1f} / d {s:+.1f}    "
                f"motor 2: j {-s:+.1f} / l {s:+.1f}    "
                f"space=positions  p=new targets  q=quit")


class _Prompt:
    """A tiny line editor.  In a real terminal we run in cbreak mode (no
    echo, no line buffering) so we echo keystrokes ourselves; when stdin is a
    pipe/IDE console the console already shows what was typed, so echo=False."""

    def __init__(self, motor, default, echo):
        self.motor = motor
        self.default = default
        self.echo = echo
        self.buf = ""
        d = f"{default:+.3f}" if default is not None else "?"
        sys.stdout.write(f"motor {motor} position [{d}]: ")
        sys.stdout.flush()

    def feed(self, ch):
        """Returns the finished line on Enter, else None."""
        if ch in ("\n", "\r"):
            self._echo("\n")
            return self.buf
        if ch in ("\x7f", "\b"):
            if self.buf:
                self.buf = self.buf[:-1]
                self._echo("\b \b")
        elif ch.isprintable():
            self.buf += ch
            self._echo(ch)
        return None

    def _echo(self, text):
        if self.echo:
            sys.stdout.write(text)
            sys.stdout.flush()


# -------------------------------------------------------------- main loop
async def run(args):
    jog = Jog(args)
    period = 1.0 / args.rate
    fd = sys.stdin.fileno()

    print(f"[*] Jog: step={args.step} rev, vel_limit={args.vel} rev/s, "
          f"accel_limit={args.accel} rev/s^2, rate={args.rate} Hz "
          f"({'DRY RUN' if args.dry_run else 'LIVE'}).")

    await jog.stop()                    # clean state; clears faults

    # A few query cycles so we know where the motors are before asking.
    for _ in range(5):
        await jog.can_cycle()
        await asyncio.sleep(period)
    print(f"[*] now   {jog.positions_str()}")
    jog.start_prompt()

    try:
        with _cbreak(fd):
            while not jog.quit:
                await jog.can_cycle()
                for ch in _pending_keys(fd):
                    if ch is None:              # stdin closed (EOF)
                        jog.quit = True
                        break
                    jog.handle_key(ch)
                await asyncio.sleep(period)
    finally:
        # Runs on 'q', Ctrl-C (asyncio cancels us) and errors alike.
        print("\n[*] Stopping motors ...")
        await jog.stop()
        print("[✓] Stopped.")


def _pending_keys(fd):
    """Every key currently waiting on stdin, without blocking.  Yields None
    once if stdin has hit EOF."""
    while select.select([fd], [], [], 0)[0]:
        data = os.read(fd, 64)
        if not data:
            yield None
            return
        yield from data.decode(errors="ignore")


@contextlib.contextmanager
def _cbreak(fd):
    """No line buffering, no echo -- but Ctrl-C still works.  No-op when stdin
    is not a terminal (e.g. piped input for testing)."""
    if not os.isatty(fd):
        yield
        return
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _complete(result):
    if result is None:
        return False
    return bool(result.values.get(moteus.Register.TRAJECTORY_COMPLETE, 0))


def _check_fault(result, mid):
    mode = result.values.get(moteus.Register.MODE)
    if mode in (moteus.Mode.FAULT, moteus.Mode.TIMEOUT):
        fault = result.values.get(moteus.Register.FAULT, 0)
        raise RuntimeError(f"motor {mid}: mode={mode} fault={fault}")


# ---------------------------------------------------------------------------
# Fake transport for --dry-run.  It decodes the outgoing CAN frame with
# moteus's own multiplex parser, so it really sees the commanded position and
# velocity limit, and moves a pretend motor toward it at that speed.
# ---------------------------------------------------------------------------
class _FakeResult:
    def __init__(self, id_, position, complete):
        self.id = id_
        self.values = {
            moteus.Register.MODE: moteus.Mode.POSITION,
            moteus.Register.POSITION: position,
            moteus.Register.TRAJECTORY_COMPLETE: 1 if complete else 0,
        }


class _FakeTransport:
    def __init__(self, rate):
        self.dt = 1.0 / rate
        self.pos = {1: 8.3, 2: 0.15}          # pretend start pose

    async def cycle(self, commands, **kwargs):
        results = []
        for cmd in commands:
            mid = cmd.destination
            target, vel = None, 1.0
            for sf in mp.parse_frame(cmd.data):
                if sf.type != mp.SubframeType.WRITE:
                    continue
                if sf.register == moteus.Register.COMMAND_POSITION:
                    target = sf.value
                elif sf.register == moteus.Register.COMMAND_VELOCITY_LIMIT:
                    vel = sf.value
            complete = True
            if target is not None and not math.isnan(target):
                step = vel * self.dt
                err = target - self.pos[mid]
                if abs(err) <= step:
                    self.pos[mid] = target
                else:
                    self.pos[mid] += math.copysign(step, err)
                    complete = False
            results.append(_FakeResult(mid, self.pos[mid], complete))
        return results


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", type=float, default=0.2,
                    help="jog increment per key press, rev (default 0.2)")
    ap.add_argument("--vel", type=float, default=4.0,
                    help="velocity limit for moves, rev/s (default 4.0)")
    ap.add_argument("--accel", type=float, default=40.0,
                    help="acceleration limit for moves, rev/s^2 (default 40.0)")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="command update rate in Hz (default 100)")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware/serial; simulate the motors")
    args = ap.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
