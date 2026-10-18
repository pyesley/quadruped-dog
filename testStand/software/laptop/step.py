#!/usr/bin/env python3
"""
Step two moteus motors (ids 1 and 2) back and forth between two poses
(POSE_1 / POSE_2 below).

Sequence:
  go to pose 1 -> hold DWELL s -> go to pose 2 -> hold DWELL s -> pose 1 -> ...

Speed of the move
-----------------
Two numbers set how quickly a move happens: the velocity limit (rev/s) and
the acceleration limit (rev/s^2).  There are three places to set them:

  1. In this file:   VEL / ACCEL just below, next to the poses.
  2. Command line:   python3 step.py --vel 50 --accel 20
  3. While running, with single keys (no Enter needed):

         +  /  -       velocity limit   x1.25  /  /1.25
         ]  /  [       accel limit      x1.25  /  /1.25
         space         print the current limits
         q / Ctrl-C    stop motors and quit

     A key press takes effect right away, even mid-move: moteus re-plans its
     trapezoid from the current speed each time a command carries new limits.
     From a real terminal keys act instantly.  From an IDE run console (e.g.
     PyCharm) stdin is not a tty, so type the key and press Enter.

  Note that for a short move the accel limit is usually what sets the move
  time: with accel 20 rev/s^2 a 2.8 rev move only ever reaches
  sqrt(20 * 2.8) ~= 7.5 rev/s, so a velocity limit above that changes
  nothing.  Raise accel (']') to make such a move faster.

How it works (companion to sine_two_motors.py):

  * Unlike the sine, this IS point-to-point motion, so we let the controller
    run its own trapezoidal trajectory: `velocity_limit` / `accel_limit` set
    how "quickly" the move is, and `velocity=0.0` means "come to rest at the
    target" (a non-zero `velocity` would keep drifting after arrival).

  * We still stream the command every loop (~100 Hz) -- including during the
    hold -- because moteus has a watchdog: if it hears nothing for ~100 ms it
    faults and goes limp.  Streaming the same target is what "hold position"
    means.  The keyboard is polled from inside that same loop (terminal in
    cbreak mode, no threads, same as jog.py) so the CAN stream never pauses.

  * Both motors are commanded in ONE CAN cycle, so they start together.  To
    make them also FINISH together, each motor's velocity/accel limits are
    scaled by (its distance / the longest distance).  Scaling v, a and d by the
    same factor leaves the trapezoid's timing unchanged (t_accel = v/a, and
    the profile just shrinks in amplitude), so the short move takes exactly
    as long as the long one.  VEL/ACCEL apply to the longest move.

  * Arrival is detected with the TRAJECTORY_COMPLETE register (not sent by
    default, so we enable it in the QueryResolution).  Both motors must report
    complete before the hold timer starts.

Run:
  DRY RUN (no hardware/serial -- exercises the state machine and timing):
      python3 step.py --dry-run

  LIVE (motors move):
      python3 step.py
      python3 step.py --vel 50 --accel 20 --dwell 2.0

  With the dashboard (dash.py) running, stream position/velocity/torque to it:
      python3 step.py --telemetry 127.0.0.1:9871
"""

import argparse
import asyncio
import contextlib
import math
import os
import json
import select
import socket
import sys
import termios
import time
import tty

import moteus
import moteus.multiplex as mp


# motor id -> position (revolutions)
POSE_1 = {1: -1.195, 2: 0.095}
POSE_2 = {1: -3.99, 2: 1.932}
POSES = [("pose 1", POSE_1), ("pose 2", POSE_2)]

# How fast to move between the poses.  These are the defaults for --vel /
# --accel / --dwell; the +/- and ]/[ keys change the first two while running.
VEL = 50.0      # velocity limit for the longest motor's move, rev/s
ACCEL = 20.0    # acceleration limit for the longest motor's move, rev/s^2
DWELL = 2.0     # seconds to hold at each pose

MOTOR_IDS = (1, 2)

# Protective trips, checked every cycle.  These sit *below* the controller's own
# servo.max_current_A = 9 and servo.fault_temperature = 75 so that we abort the
# run rather than letting the hardware fault out.  --max-current 0 / --max-temp 0
# turn them off.
MAX_CURRENT_A = 8.0
MAX_TEMP_C = 70.0

# Each + - ] [ press multiplies or divides a limit by this ...
ADJUST_FACTOR = 1.25
# ... but never below these: a 0 velocity_limit would mean "never move".
MIN_VEL = 0.05      # rev/s
MIN_ACCEL = 0.5     # rev/s^2

# The reply to the very first command(s) after a target change can still carry
# the *previous* trajectory's "complete" flag.  moteus's own
# set_position_wait_complete() skips 2 replies for the same reason.
SETTLE_CYCLES = 3

# A motor with (almost) no distance to cover still gets this fraction of the
# limits rather than zero, so we never hand moteus a 0 velocity/accel limit.
MIN_SCALE = 0.02

KEY_HELP = ("[keys]  + / -  velocity limit x1.25 or /1.25     "
            "] / [  accel limit x1.25 or /1.25     "
            "space  show limits     q  quit")


# moteus fw/error.h -- what the FAULT register means.  Without this a trip just
# prints a number and you are left guessing which limit you hit.
FAULT_NAMES = {
    0: "no fault", 1: "DMA stream transfer", 2: "DMA stream FIFO",
    3: "UART overrun", 4: "UART framing", 5: "UART noise",
    6: "UART buffer overrun", 7: "UART parity",
    32: "calibration fault", 33: "motor driver fault",
    34: "OVER VOLTAGE (regen pushed the bus past servo.max_voltage)",
    35: "encoder fault", 36: "motor not configured", 37: "pwm cycle overrun",
    38: "OVER TEMPERATURE (servo.fault_temperature, 75 C on these boards)",
    39: "start outside limit", 40: "under voltage", 41: "config changed",
    42: "theta invalid", 43: "position invalid", 44: "driver enable fault",
    45: "stop position deprecated", 46: "timing violation",
    47: "bemf feedforward without accel limit", 48: "invalid limits",
    49: "position control error", 50: "velocity control error",
    # 96..105 are NOT faults.  They are reported while running to say what is
    # currently limiting the output.  The controller stays in its control mode.
    96: "limiting: max velocity", 97: "limiting: max power",
    98: "limiting: max voltage", 99: "limiting: servo.max_current_A",
    100: "limiting: board temperature", 101: "limiting: motor temperature",
    102: "limiting: max torque", 103: "limiting: position bounds",
    104: "limiting: FLUX BRAKING active (bus above max_voltage - margin)",
    105: "limiting: field weakening active",
}

# Codes in this range mean "something is capping the output right now", not
# "the controller has faulted".  Treating them as faults cries wolf.
LIMIT_CODES = range(96, 106)


def is_limit(code):
    return int(code or 0) in LIMIT_CODES


def fault_str(code):
    code = int(code or 0)
    return f"{code} ({FAULT_NAMES.get(code, 'unknown')})"


def _sync_scale(dists):
    """Per-motor factor (0..1] for the velocity/accel limits so every move
    lasts as long as the longest one.  `dists` is motor id ->
    |target - current| in rev."""
    d_max = max(dists.values())
    return {mid: 1.0 if d_max < 1e-6 else max(d / d_max, MIN_SCALE)
            for mid, d in dists.items()}


def _limits_str(scale, vel, accel):
    return "  ".join(f"m{m}: v={vel * k:.2f} a={accel * k:.1f}"
                     for m, k in scale.items())


def _update_actual(actual, results):
    for r in results:
        p = r.values.get(moteus.Register.POSITION)
        if p is not None:
            actual[r.id] = p


class _Telemetry:
    """Fire-and-forget UDP JSON of every reply, for dash.py. Never blocks and
    never raises: losing a telemetry packet must not disturb the CAN loop.

    Keys per motor: pos/vel/tq (rev, rev/s, N.m), q/d (phase current, A),
    pw (electrical power, W), volt (bus V), temp / mtemp (board / motor degC),
    mode, flt (fault code), home (0 relative, 1 rotor, 2 output)."""

    def __init__(self, hostport):
        host, port = hostport.rsplit(":", 1)
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.t0 = time.time()

    def send(self, results, pose, state, vel_lim=None, accel_lim=None, extra=None):
        R = moteus.Register
        m = {}
        for r in results:
            v = r.values
            m[str(r.id)] = {"pos": v.get(R.POSITION),
                            "vel": v.get(R.VELOCITY),
                            "tq": v.get(R.TORQUE),
                            "q": v.get(R.Q_CURRENT),
                            "d": v.get(R.D_CURRENT),
                            "pw": v.get(R.POWER),
                            "mode": int(v.get(R.MODE, 0) or 0),
                            "temp": v.get(R.TEMPERATURE),
                            "mtemp": v.get(R.MOTOR_TEMPERATURE),
                            "volt": v.get(R.VOLTAGE),
                            "flt": int(v.get(R.FAULT, 0) or 0),
                            "home": int(v.get(R.HOME_STATE, 0) or 0)}
        now = time.time()
        pkt = {"t": now - self.t0, "tw_send": now, "state": state,
               "vel_lim": vel_lim, "accel_lim": accel_lim,
               "cmd": {str(k): v for k, v in pose.items()}, "m": m}
        if extra:                       # e.g. hop.py's control-panel status
            pkt.update(extra)
        try:
            self.sock.sendto(json.dumps(pkt).encode(), self.addr)
        except OSError:
            pass


async def run_step(args):
    transport = (_FakeTransport(args.rate) if args.dry_run
                 else moteus.Fdcanusb())

    telemetry = _Telemetry(args.telemetry) if args.telemetry else None

    # Ask each controller for more than the default mode/pos/vel/torque/
    # voltage/temperature/fault: TRAJECTORY_COMPLETE to detect arrival, and
    # the currents/power so the dashboard can watch how close we are to
    # servo.max_current_A.  Every extra register costs CAN bytes, so this is
    # also what sets the ceiling on --rate.
    qr = moteus.QueryResolution()
    qr.trajectory_complete = mp.INT8
    qr.q_current = mp.F32
    qr.d_current = mp.F32
    qr.power = mp.F32
    qr.motor_temperature = mp.F32
    qr.home_state = mp.INT8

    ctrl = {mid: moteus.Controller(id=mid, transport=transport,
                                   query_resolution=qr)
            for mid in MOTOR_IDS}

    # Clean, known state (clears faults, stops any prior motion).
    if not args.dry_run:
        await transport.cycle([c.make_stop() for c in ctrl.values()])

    vel, accel = args.vel, args.accel       # the live limits; keys change them
    period = 1.0 / args.rate
    print(f"[*] Step: vel_limit={vel} rev/s, accel_limit={accel} "
          f"rev/s^2, dwell={args.dwell}s, rate={args.rate} Hz "
          f"({'DRY RUN' if args.dry_run else 'LIVE'}). Ctrl-C to stop.")
    print(f"[*] Trips: max phase current "
          f"{str(args.max_current) + ' A' if args.max_current else 'off'}, "
          f"max temperature "
          f"{str(args.max_temp) + ' C' if args.max_temp else 'off'}"
          + (f", telemetry -> {args.telemetry}" if args.telemetry else ""))
    print(KEY_HELP)

    # Where are the motors now?  Needed to time the very first move.
    actual = {}
    for _ in range(5):
        results = await transport.cycle([c.make_query() for c in ctrl.values()])
        _update_actual(actual, results)
        await asyncio.sleep(period)

    def start_move(pose):
        """Per-motor scale factors for a move from `actual` to `pose`."""
        if all(m in actual for m in MOTOR_IDS):
            dists = {m: abs(pose[m] - actual[m]) for m in MOTOR_IDS}
            scale = _sync_scale(dists)
        else:   # a motor never replied; can't scale, use the raw limits
            scale = {m: 1.0 for m in MOTOR_IDS}
        print(f"[*] -> {name}  {pose}   ({_limits_str(scale, vel, accel)})")
        return scale

    def handle_key(ch):
        """Apply one key press.  Returns True when the user asked to quit."""
        nonlocal vel, accel
        if ch in ("+", "="):                # '=' is '+' without shift
            vel *= ADJUST_FACTOR
        elif ch == "-":
            vel = max(vel / ADJUST_FACTOR, MIN_VEL)
        elif ch == "]":
            accel *= ADJUST_FACTOR
        elif ch == "[":
            accel = max(accel / ADJUST_FACTOR, MIN_ACCEL)
        elif ch == "q":
            return True
        elif ch != " ":
            return False                    # Enter, unknown keys: ignore
        print(f"[*] vel_limit={vel:.2f} rev/s  accel_limit={accel:.1f} rev/s^2"
              f"   ({_limits_str(scale, vel, accel)})")
        return False

    idx = 0                 # which pose we are heading to / holding
    moving = True           # True: en route.  False: holding at pose.
    cycles_since_switch = 0
    hold_t0 = None
    t0 = time.time()
    fd = sys.stdin.fileno() # becomes None once stdin hits EOF (keys off)

    name, pose = POSES[idx]
    scale = start_move(pose)

    try:
        with _cbreak(fd):
            while True:
                t = time.time() - t0

                # Same target every cycle; the controller runs the trajectory.
                # vel/accel are re-read every cycle, so a key press mid-move
                # changes the speed at once.
                commands = [
                    ctrl[mid].make_position(position=pose[mid],
                                            velocity=0.0,
                                            velocity_limit=vel * scale[mid],
                                            accel_limit=accel * scale[mid],
                                            query=True)
                    for mid in MOTOR_IDS
                ]
                results = await transport.cycle(commands)
                by_id = {r.id: r for r in results}
                _update_actual(actual, results)
                cycles_since_switch += 1

                if telemetry:        # send before the checks, so a trip is on record
                    telemetry.send(results, pose,
                                   f"{'MOVE' if moving else 'HOLD'} {name}",
                                   vel_lim=vel, accel_lim=accel)

                for mid in MOTOR_IDS:
                    _check_fault(by_id.get(mid), mid, args.max_current, args.max_temp)

                p1 = _pos(by_id.get(1))
                p2 = _pos(by_id.get(2))
                state = "MOVE" if moving else "HOLD"
                print(f"t={t:6.2f}s  {state} {name}  "
                      f"cmd1={pose[1]:+.3f} cmd2={pose[2]:+.3f}  "
                      f"pos1={p1}  pos2={p2}")

                if moving:
                    arrived = (cycles_since_switch > SETTLE_CYCLES and
                               all(_complete(by_id.get(mid)) for mid in MOTOR_IDS))
                    if arrived:
                        moving = False
                        hold_t0 = time.time()
                        print(f"[*] arrived at {name}; holding {args.dwell}s")
                elif time.time() - hold_t0 >= args.dwell:
                    idx = (idx + 1) % len(POSES)
                    name, pose = POSES[idx]
                    moving = True
                    cycles_since_switch = 0
                    scale = start_move(pose)

                for ch in (_pending_keys(fd) if fd is not None else ()):
                    if ch is None:          # stdin closed (EOF): keys are off
                        fd = None
                        break
                    if handle_key(ch):      # 'q'
                        return              # finally: stops the motors

                await asyncio.sleep(period)
    finally:
        # Ctrl-C cancels this task (asyncio.run), so a plain `except
        # KeyboardInterrupt` here would never run -- `finally` always does.
        print("\n[*] Stopping motors ...")
        if not args.dry_run:
            await transport.cycle([c.make_stop() for c in ctrl.values()])
        print("[✓] Stopped.")


def _pending_keys(fd):
    """Every key currently waiting on stdin, without blocking.  Yields None
    once if stdin has hit EOF.  (Same as jog.py.)"""
    while select.select([fd], [], [], 0)[0]:
        data = os.read(fd, 64)
        if not data:
            yield None
            return
        yield from data.decode(errors="ignore")


@contextlib.contextmanager
def _cbreak(fd):
    """No line buffering, no echo -- but Ctrl-C still works.  No-op when stdin
    is not a terminal (e.g. an IDE console or piped input).  (Same as jog.py.)"""
    if not os.isatty(fd):
        yield
        return
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _pos(result):
    """Safely extract POSITION from a Result (or None)."""
    if result is None:
        return "  n/a "
    val = result.values.get(moteus.Register.POSITION)
    return f"{val:+.3f}" if val is not None else "  n/a "


def _complete(result):
    """True if this Result says the trajectory is finished."""
    if result is None:
        return False
    return bool(result.values.get(moteus.Register.TRAJECTORY_COMPLETE, 0))


def _check_fault(result, mid, max_current=None, max_temp=None):
    """Bail out loudly if a controller faulted, hit its watchdog, or is drawing
    more current / running hotter than we said we would allow.  The controller
    has its own servo.max_current_A and servo.fault_temperature; this trips
    *before* those so we stop the experiment instead of the hardware stopping
    it for us."""
    if result is None:
        return
    v = result.values
    mode = v.get(moteus.Register.MODE)
    if mode in (moteus.Mode.FAULT, moteus.Mode.TIMEOUT):
        raise RuntimeError(
            f"motor {mid}: mode={mode} fault={fault_str(v.get(moteus.Register.FAULT, 0))}"
            f"   [bus {v.get(moteus.Register.VOLTAGE)} V, "
            f"board {v.get(moteus.Register.TEMPERATURE)} C]")
    if max_current:
        q = v.get(moteus.Register.Q_CURRENT)
        d = v.get(moteus.Register.D_CURRENT)
        if q is not None and d is not None:
            amps = math.hypot(q, d)
            if amps > max_current:
                raise RuntimeError(f"motor {mid}: phase current {amps:.2f} A "
                                   f"exceeded --max-current {max_current} A")
    if max_temp:
        for reg, what in ((moteus.Register.TEMPERATURE, "board"),
                          (moteus.Register.MOTOR_TEMPERATURE, "motor")):
            t = v.get(reg)
            if t is not None and t > max_temp:
                raise RuntimeError(f"motor {mid}: {what} temperature {t:.1f} C "
                                   f"exceeded --max-temp {max_temp} C")


# ---------------------------------------------------------------------------
# Fake transport for --dry-run.  It decodes the outgoing CAN frame with
# moteus's own multiplex parser, so it sees the commanded position and the
# velocity/accel limits, and runs a pretend motor along a trapezoid that
# honours both -- close enough to moteus's own planner to show the move
# timing and the two motors arriving on the same cycle.
# ---------------------------------------------------------------------------
class _FakeResult:
    def __init__(self, id_, position, complete, velocity=0.0, accel=0.0):
        self.id = id_
        # A pretend torque/current proportional to the acceleration, so the
        # dashboard has something plausible to draw in --dry-run.  Kt = 8.27/Kv
        # with the configured motor.Kv = 306.5, and a made-up 4e-3 kg.m^2 of
        # reflected inertia -- the shape is right, the numbers are invented.
        torque = 4e-3 * accel
        q = torque / (8.27 / 306.5)
        self.values = {
            moteus.Register.MODE: moteus.Mode.POSITION,
            moteus.Register.POSITION: position,
            moteus.Register.VELOCITY: velocity,
            moteus.Register.TORQUE: torque,
            moteus.Register.Q_CURRENT: q,
            moteus.Register.D_CURRENT: 0.05 * q,
            moteus.Register.POWER: abs(torque * velocity * 2 * math.pi),
            moteus.Register.VOLTAGE: 22.0,
            moteus.Register.TEMPERATURE: 31.0,
            moteus.Register.MOTOR_TEMPERATURE: 0.0,
            moteus.Register.FAULT: 0,
            moteus.Register.HOME_STATE: 1,
            moteus.Register.TRAJECTORY_COMPLETE: 1 if complete else 0,
        }


class _FakeTransport:
    def __init__(self, rate):
        self.dt = 1.0 / rate
        self.pos = {1: 0.0, 2: 0.0}           # pretend start pose
        self.vel = {1: 0.0, 2: 0.0}

    async def cycle(self, commands, **kwargs):
        results = []
        for cmd in commands:
            mid = cmd.destination
            target, vlim, alim = None, 1.0, 10.0
            for sf in mp.parse_frame(cmd.data):
                if sf.type != mp.SubframeType.WRITE:
                    continue
                if sf.register == moteus.Register.COMMAND_POSITION:
                    target = sf.value
                elif sf.register == moteus.Register.COMMAND_VELOCITY_LIMIT:
                    vlim = sf.value
                elif sf.register == moteus.Register.COMMAND_ACCEL_LIMIT:
                    alim = sf.value
            complete = True
            accel = 0.0
            if target is not None and not math.isnan(target):
                err = target - self.pos[mid]
                v = self.vel[mid]
                dv = alim * self.dt
                # Fastest we may go here and still stop at the target.
                v_stop = math.sqrt(2.0 * alim * abs(err))
                v_des = math.copysign(min(vlim, v_stop), err)
                v_prev = v
                v += max(-dv, min(dv, v_des - v))       # ramp at accel limit
                accel = (v - v_prev) / self.dt
                step = v * self.dt
                if abs(step) >= abs(err):               # reaches target now
                    self.pos[mid] = target
                    self.vel[mid] = 0.0
                    complete = abs(v) <= dv + 1e-9      # ...and gently
                else:
                    self.pos[mid] += step
                    self.vel[mid] = v
                    complete = False
            results.append(_FakeResult(mid, self.pos[mid], complete,
                                       self.vel[mid], accel))
        return results


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vel", type=float, default=VEL,
                    help=f"velocity limit for the move, rev/s (default {VEL})")
    ap.add_argument("--accel", type=float, default=ACCEL,
                    help=f"acceleration limit for the move, rev/s^2 (default {ACCEL})")
    ap.add_argument("--dwell", type=float, default=DWELL,
                    help=f"seconds to hold at each pose (default {DWELL})")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="command update rate in Hz (default 100)")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware/serial; simulate the motors")
    ap.add_argument("--telemetry", metavar="HOST:PORT", default=None,
                    help="stream per-cycle motor state as UDP JSON to the "
                         "dashboard, e.g. 127.0.0.1:9871 (default: off)")
    ap.add_argument("--max-current", type=float, default=MAX_CURRENT_A,
                    help=f"stop if phase current hypot(q,d) exceeds this, A "
                         f"(default {MAX_CURRENT_A}; 0 disables). The controllers "
                         f"are configured for servo.max_current_A = 9.")
    ap.add_argument("--max-temp", type=float, default=MAX_TEMP_C,
                    help=f"stop if board or motor temperature exceeds this, degC "
                         f"(default {MAX_TEMP_C}; 0 disables; controller faults at 75)")
    args = ap.parse_args()

    try:
        asyncio.run(run_step(args))
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        # run_step's `finally` has already stopped the motors by the time we get
        # here, so this is just the report.  No traceback: a trip is a result.
        print(f"\n[!] STOPPED: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
