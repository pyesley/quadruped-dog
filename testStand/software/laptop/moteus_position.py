#!/usr/bin/env python3
"""
Send moteus POSITION-mode commands over the fdcanusb / mjcanfd-usb-1x adapter.

Position mode is moteus's primary control mode.  A single command carries the
whole desired state; unspecified fields keep their configured defaults.
See: https://mjbots.github.io/moteus/guides/control-modes/#understanding-position-mode

Command fields (all optional; NaN / None means "use default / current"):
  position            desired position, in revolutions.
                      NaN  -> hold at whatever position we reach (velocity mode)
  velocity            rev/s.  Rate the *target* position moves.
  feedforward_torque  extra torque (N*m) added to the control-loop output.
  kp_scale            scale the configured position gain (kp) for this command.
  kd_scale            scale the configured damping gain (kd) for this command.
  maximum_torque      clamp on commanded torque (N*m).  A key safety limit.
  stop_position       if set, target ramps toward this and then holds (no drift).
  watchdog_timeout    if no new command within this many seconds -> fault/stop.
  velocity_limit      per-command cap on trajectory velocity (rev/s).
  accel_limit         per-command cap on trajectory acceleration (rev/s^2).
  ilimit_scale        scale the integral limit for this command.

Two ways to run this program:

  DRY RUN (default, safe, NO hardware, NO serial):
      python3 moteus_position.py
      python3 moteus_position.py --position 0.25 --velocity 0.5 --max-torque 1.0
    Builds the exact CAN-FD frame(s) and prints the bytes that WOULD be sent.
    Nothing is transmitted.  Use this to verify command encoding.

  LIVE (you run this manually, motor will move):
      python3 moteus_position.py --live --id 1 --position 0.25 --max-torque 1.0
    Actually opens /dev/ttyACM0 via moteus.Fdcanusb and sends the command.
"""

import argparse
import asyncio
import math
import sys


def add_position_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--id", type=int, default=1, help="target controller id")
    ap.add_argument("--position", type=float, default=None,
                    help="revolutions; omit or 'nan' for velocity-only control")
    ap.add_argument("--velocity", type=float, default=None, help="rev/s")
    ap.add_argument("--feedforward-torque", type=float, default=None, help="N*m")
    ap.add_argument("--kp-scale", type=float, default=None)
    ap.add_argument("--kd-scale", type=float, default=None)
    ap.add_argument("--max-torque", type=float, default=None,
                    help="N*m torque clamp (recommended safety limit)")
    ap.add_argument("--stop-position", type=float, default=None,
                    help="revolutions; ramp here then hold")
    ap.add_argument("--watchdog-timeout", type=float, default=None,
                    help="seconds; fault if no command arrives in time")
    ap.add_argument("--velocity-limit", type=float, default=None, help="rev/s")
    ap.add_argument("--accel-limit", type=float, default=None, help="rev/s^2")


def position_kwargs(args) -> dict:
    """Map CLI args to moteus make_position() kwargs (dropping None)."""
    mapping = {
        "position": args.position,
        "velocity": args.velocity,
        "feedforward_torque": args.feedforward_torque,
        "kp_scale": args.kp_scale,
        "kd_scale": args.kd_scale,
        "maximum_torque": args.max_torque,
        "stop_position": args.stop_position,
        "watchdog_timeout": args.watchdog_timeout,
        "velocity_limit": args.velocity_limit,
        "accel_limit": args.accel_limit,
    }
    return {k: v for k, v in mapping.items() if v is not None}


def _round_up_dlc(size: int) -> int:
    """CAN-FD only allows certain frame lengths; the driver pads up to these."""
    for limit in (8, 12, 16, 20, 24, 32, 48, 64):
        if size <= limit:
            return limit
    return size


def frame_arbitration_id(cmd) -> int:
    """Reproduce moteus's CAN arbitration-id assembly (transport.py).

    id = dest_id | (0x8000 if reply_required) | (source << 8) | (can_prefix << 16)
    """
    dest = cmd.destination
    if isinstance(dest, int):
        dest_id = dest
    else:  # DeviceAddress
        dest_id = dest.can_id if dest.can_id is not None else 0x7F
    return (dest_id
            | (0x8000 if cmd.reply_required else 0)
            | (cmd.source << 8)
            | (cmd.can_prefix << 16))


def describe_frame(cmd, title: str) -> None:
    """Pretty-print a moteus command frame as it would go over fdcanusb."""
    arb = frame_arbitration_id(cmd)
    payload = bytes(cmd.data)
    dest_id = arb & 0x7F
    reply = bool(arb & 0x8000)

    # On the wire the payload is padded up to a valid CAN-FD length with 0x50.
    on_wire = _round_up_dlc(len(payload))
    padded = payload + b"\x50" * (on_wire - len(payload))

    print(f"  {title}")
    print(f"    arbitration_id : 0x{arb:04X}  (dest id={dest_id}, reply_requested={reply})")
    print(f"    payload ({len(payload)} B) : {payload.hex().upper()}")
    # The literal line the fdcanusb text protocol carries (B=bitrate switch, F=FD):
    print(f"    fdcanusb line  : can send {arb:04x} {padded.hex().upper()} B F")


def dry_run(args) -> int:
    """Build frames offline and print them. No serial, no hardware."""
    import moteus

    kwargs = position_kwargs(args)
    if "position" not in kwargs:
        # Explicitly express "hold current position" via NaN, the moteus idiom
        # for velocity-only / no-target-change control.
        kwargs["position"] = math.nan

    controller = moteus.Controller(id=args.id)

    print("=== DRY RUN (no data transmitted) ===")
    print(f"target controller id : {args.id}")
    print(f"position kwargs      : {kwargs}")
    print()

    # A real session first stops (clears faults / integrators), then commands.
    stop = controller.make_stop(query=True)
    describe_frame(stop, "1) STOP frame (clear/hold):")
    print()

    pos = controller.make_position(query=True, **kwargs)
    describe_frame(pos, "2) POSITION frame:")
    print()

    query = controller.make_query()
    describe_frame(query, "3) QUERY frame (read-only status):")
    print()
    print("To actually send these, re-run with --live (motor WILL move).")
    return 0


async def live(args) -> int:
    import moteus

    kwargs = position_kwargs(args)
    if "position" not in kwargs:
        kwargs["position"] = math.nan

    print(f"[*] Opening fdcanusb transport, controller id={args.id}")
    transport = moteus.Fdcanusb()          # auto-detects /dev/ttyACM0
    controller = moteus.Controller(id=args.id, transport=transport)

    print("[*] Sending stop (clear faults / hold) ...")
    await controller.set_stop()

    print(f"[*] Sending position command: {kwargs}")
    state = await controller.set_position(query=True, **kwargs)
    print("[*] Reply / state:")
    if state is not None:
        for key, value in state.values.items():
            print(f"      {key}: {value}")
    print("[✓] Done.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--live", action="store_true",
                    help="actually transmit over fdcanusb (motor moves!). "
                         "Default is a safe dry run.")
    add_position_args(ap)
    args = ap.parse_args()

    if args.live:
        return asyncio.run(live(args))
    return dry_run(args)


if __name__ == "__main__":
    sys.exit(main())
