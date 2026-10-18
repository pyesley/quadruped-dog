#!/usr/bin/env python3
"""
Communication test for the fdcanusb / mjcanfd-usb-1x adapter.

This does NOT command any motor.  It only proves that the host can open the
USB serial device and exchange the fdcanusb text protocol with the adapter.

What it does:
  1. Opens the serial port (default /dev/ttyACM0).
  2. Sends `can status` and prints the reply (expects a line ending in OK).
  3. Optionally sends a moteus "query only" frame to a given controller ID and
     waits for a `rcv` reply -- a non-invasive read of the motor's registers.

Run:
  python3 comm_test.py                 # just status (no bus traffic to motors)
  python3 comm_test.py --port /dev/ttyACM0
  python3 comm_test.py --query-id 1    # also read (not move) controller id 1

Note: querying a controller requires the moteus package to build the frame.
      The query frame only READS registers; it does not change control mode.
"""

import argparse
import sys

from fdcanusb import Fdcanusb


def build_query_frame(dest_id: int) -> tuple[int, bytes]:
    """Build a moteus 'query only' CAN-FD frame (read registers, no motion).

    Uses the moteus package to construct the payload so we stay byte-accurate
    with the register protocol.  Returns (arbitration_id, data).
    """
    import moteus
    from moteus_position import frame_arbitration_id

    controller = moteus.Controller(id=dest_id)
    # make_query() builds a read-only frame for the default query registers
    # (mode, position, velocity, torque, voltage, temperature, fault).
    # NOTE: the library only assembles the CAN arbitration id inside its
    # transport at send time, so we compute it here to stay byte-accurate.
    cmd = controller.make_query()
    return frame_arbitration_id(cmd), bytes(cmd.data)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="serial device path")
    ap.add_argument("--query-id", type=int, default=None,
                    help="also send a read-only query to this controller id")
    args = ap.parse_args()

    print(f"[*] Opening fdcanusb at {args.port} ...")
    try:
        dev = Fdcanusb(port=args.port)
    except Exception as e:  # noqa: BLE001
        print(f"[!] Could not open {args.port}: {e}")
        print("    - Is the adapter plugged in?  ls -l /dev/ttyACM*")
        print("    - Are you in the 'dialout' group?  groups | grep dialout")
        return 1

    with dev:
        print("[*] Sending: can status")
        for line in dev.status():
            print(f"      < {line}")

        if args.query_id is not None:
            arb_id, data = build_query_frame(args.query_id)
            print(f"[*] Sending read-only query to controller id={args.query_id}")
            print(f"      > can send {arb_id:X} {data.hex().upper()} F B")
            reply = dev.send_frame(arb_id, data)
            for line in reply:
                print(f"      < {line}")
            frame = dev.poll_rcv(timeout=0.5)
            if frame:
                print(f"      < {frame}")
            else:
                print("      (no rcv frame -- is a controller powered at that id?)")

    print("[✓] Communication test complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
