#!/usr/bin/env python3
"""
Drive two moteus motors (ids 1 and 2) with a sine-wave position command.

Position sweeps between -2 and +2 revolutions:  position = A * sin(2*pi*f*t).

Key ideas (see https://mjbots.github.io/moteus/reference/python/#moteuscontroller):

  * A sine wave is a *streaming* trajectory.  You do NOT wait for each setpoint
    to complete -- that would turn smooth motion into point-to-point stepping.
    Instead you send a fresh setpoint every loop (~100 Hz) and let the
    controller chase it.  `velocity`/`accel_limit` shape how it chases.

  * Both motors are commanded in ONE CAN cycle via `transport.cycle([...])`,
    which is more efficient and better synchronized than two separate awaits.

  * If you DID want the "wait until the move finishes" behaviour (for stepping
    to discrete targets, not a sine), moteus provides
    `set_position_wait_complete(...)` -- see wait_demo() below.

Run:
  DRY RUN (no hardware, no serial -- just prints the setpoints it would send):
      python3 sine_two_motors.py --dry-run

  LIVE (motors move -- you run this manually):
      python3 sine_two_motors.py
      python3 sine_two_motors.py --amplitude 1.0 --freq 0.25 --rate 100
"""

import argparse
import asyncio
import math
import time

import moteus


# ---------------------------------------------------------------------------
# The sine-wave streaming loop (the thing you actually asked for)
# ---------------------------------------------------------------------------
async def run_sine(args):
    # One shared transport for both controllers (auto-detects the fdcanusb).
    transport = moteus.Fdcanusb() if not args.dry_run else _FakeTransport()

    c1 = moteus.Controller(id=1, transport=transport)
    c2 = moteus.Controller(id=2, transport=transport)

    # Start from a clean, known state (clears faults, stops any prior motion).
    if not args.dry_run:
        await transport.cycle([c1.make_stop(), c2.make_stop()])

    A = args.amplitude          # revolutions; +/- A  => e.g. -2 .. 2
    f = args.freq               # Hz  (full sine cycles per second)
    period = 1.0 / args.rate    # seconds between setpoints
    two_pi_f = 2.0 * math.pi * f

    print(f"[*] Sine: amplitude={A} rev, freq={f} Hz, rate={args.rate} Hz "
          f"({'DRY RUN' if args.dry_run else 'LIVE'}). Ctrl-C to stop.")

    t0 = time.time()
    try:
        while True:
            t = time.time() - t0

            theta1 = A * math.sin(two_pi_f * t)
            # Second motor 90 degrees out of phase (feel free to change).
            theta2 = A * math.sin(two_pi_f * t + two_pi_f/4 )

            # Build both position commands and send them in a single CAN cycle.
            # query=True asks each controller to report back its telemetry.
            commands = [
                c1.make_position(position=theta1, velocity=2.0,
                                 accel_limit=20.0, query=True),
                c2.make_position(position=theta2, velocity=2.0,
                                 accel_limit=20.0, query=True),
            ]
            results = await transport.cycle(commands)

            # cycle() returns a Result per replying controller, in no guaranteed
            # order -- index them by controller id.
            by_id = {r.id: r for r in results}
            p1 = _pos(by_id.get(1))
            p2 = _pos(by_id.get(2))

            print(f"t={t:6.2f}s  cmd1={theta1:+.3f} cmd2={theta2:+.3f}  "
                  f"pos1={p1}  pos2={p2}")

            await asyncio.sleep(period)
    except KeyboardInterrupt:
        print("\n[*] Stopping motors ...")
        if not args.dry_run:
            await transport.cycle([c1.make_stop(), c2.make_stop()])
        print("[✓] Stopped.")


def _pos(result):
    """Safely extract POSITION from a Result (or None)."""
    if result is None:
        return "  n/a "
    val = result.values.get(moteus.Register.POSITION)
    return f"{val:+.3f}" if val is not None else "  n/a "


# ---------------------------------------------------------------------------
# Optional: the "wait until the trajectory completes" pattern, for reference.
# This is for STEPPING to discrete targets, not for a continuous sine.
# ---------------------------------------------------------------------------
async def wait_demo():
    transport = moteus.Fdcanusb()
    c1 = moteus.Controller(id=1, transport=transport)

    await c1.set_stop()
    for target in (2.0, -2.0, 0.0):
        print(f"[*] Moving to {target} and waiting for completion ...")
        # Repeatedly sends the position command until the controller reports
        # trajectory_complete, then returns the final Result.
        result = await c1.set_position_wait_complete(
            position=target, velocity=1.0, accel_limit=0.5,
            query=True, period_s=0.02)
        print(f"    done, position = {result.values.get(moteus.Register.POSITION)}")
    await c1.set_stop()


# ---------------------------------------------------------------------------
# A tiny fake transport so --dry-run exercises the loop without hardware.
# ---------------------------------------------------------------------------
class _FakeResult:
    def __init__(self, id_, position):
        self.id = id_
        self.values = {moteus.Register.POSITION: position}


class _FakeTransport:
    """Stand-in for moteus.Fdcanusb: echoes commanded positions back."""
    async def cycle(self, commands, **kwargs):
        results = []
        for cmd in commands:
            dest = cmd.destination if isinstance(cmd.destination, int) else 0
            # We don't decode the payload here; just report a plausible value.
            results.append(_FakeResult(dest, position=0.0))
        return results


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--amplitude", type=float, default=1.0,
                    help="sine amplitude in revolutions (default 1.0 => -2..2)")
    ap.add_argument("--freq", type=float, default=0.25,
                    help="sine frequency in Hz (default 0.25)")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="setpoint update rate in Hz (default 100)")
    ap.add_argument("--dry-run", action="store_true",
                    help="no hardware/serial; just print setpoints")
    ap.add_argument("--wait-demo", action="store_true",
                    help="run the stepping/wait-for-completion demo instead")
    args = ap.parse_args()

    if args.wait_demo:
        asyncio.run(wait_demo())
    else:
        asyncio.run(run_sine(args))


if __name__ == "__main__":
    main()
