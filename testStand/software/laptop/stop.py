#!/usr/bin/env python3
"""Make the motors safe.  Run this any time you are unsure what state they are in.

Why this exists
---------------
step.py stops the motors in a `finally:` block, so Ctrl-C and a graceful stop
both de-energise them.  A *hard* kill does not run that block -- e.g. PyCharm's
Stop button pressed twice, a crashed process, or the USB cable pulled and
replaced.  When the controllers stop hearing commands they wait
`servo.default_timeout_s` (0.1 s) and then fall into `servo.timeout_mode`, which
on these controllers is 12 = ZERO_VELOCITY with up to
`servo.timeout_max_torque_Nm` (5 N.m) available.

That means a hard-killed run leaves the motors POWERED, actively resisting
motion, indefinitely.  They are not limp and they are not off.  This script
sends a real stop and then reads the mode back to prove it took.

    python3 stop.py
"""

import argparse
import asyncio
import sys

import moteus

MOTOR_IDS = (1, 2)


async def stop_all(ids):
    transport = moteus.Fdcanusb()
    ctrl = {mid: moteus.Controller(id=mid, transport=transport) for mid in ids}

    await transport.cycle([c.make_stop() for c in ctrl.values()])
    await asyncio.sleep(0.1)

    results = await transport.cycle([c.make_query() for c in ctrl.values()])
    by_id = {r.id: r for r in results}

    ok = True
    for mid in ids:
        r = by_id.get(mid)
        if r is None:
            print(f"  motor {mid}: NO REPLY -- is it powered? is the CAN cable in?")
            ok = False
            continue
        v = r.values
        mode = int(v.get(moteus.Register.MODE, -1))
        name = next((n for n in dir(moteus.Mode)
                     if not n.startswith("_") and int(getattr(moteus.Mode, n)) == mode),
                    str(mode))
        fault = int(v.get(moteus.Register.FAULT, 0) or 0)
        print(f"  motor {mid}: mode={mode} ({name})  pos={v.get(moteus.Register.POSITION):+.4f} rev"
              f"  {v.get(moteus.Register.VOLTAGE)} V  {v.get(moteus.Register.TEMPERATURE)} C"
              + (f"  FAULT {fault}" if fault else ""))
        if mode != int(moteus.Mode.STOPPED):
            ok = False
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", default=",".join(str(i) for i in MOTOR_IDS),
                    help="comma separated controller ids (default 1,2)")
    a = ap.parse_args()
    ids = [int(x) for x in a.ids.split(",") if x.strip()]

    print("[*] Sending stop ...")
    try:
        ok = asyncio.run(stop_all(ids))
    except Exception as e:                                   # noqa: BLE001
        print(f"[!] could not talk to the controllers: {e}", file=sys.stderr)
        print("    If they are still energised, cut the bus power.", file=sys.stderr)
        sys.exit(2)

    if ok:
        print("[/] All motors STOPPED (de-energised).")
    else:
        print("[!] Not every motor reported STOPPED -- see above.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
