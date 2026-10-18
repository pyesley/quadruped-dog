#!/usr/bin/env python3
"""Watch the motors without energising them.

step.py *commands* positions, so the motors are live whenever it runs.  This
does the opposite: it only ever sends queries, so the motors stay in mode 0
(STOPPED) and you can move the leg by hand while the encoders keep tracking it
and the dashboard keeps plotting it.

Use it to:
  * calibrate the leg geometry (hold the leg straight, press "Leg straight = 0"
    in the dashboard) -- you cannot do that while step.py is holding a pose;
  * check the sign of a joint: move it one way and watch the angle;
  * see where the leg actually is before starting step.py;
  * check how repeatable your power-up pose is (the --wrap column).

It sends one stop at startup so the motors are definitely free, then never
commands anything again.  A query does not feed the moteus watchdog, but a
STOPPED controller has nothing to time out, so it simply stays stopped.

    python3 monitor.py --telemetry 127.0.0.1:9871
    python3 monitor.py                       # console only, no dashboard
"""

import argparse
import asyncio
import json
import math
import os
import sys
import time

import moteus
import moteus.multiplex as mp

from step import MOTOR_IDS, _Telemetry

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "leg_config.json")


def load_cfg():
    try:
        with open(CFG_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def joint_deg(cfg, mid, pos):
    for j in cfg.get("joints", []):
        if int(j["id"]) == mid:
            return ((pos - j.get("zero_rev", 0.0)) / (j.get("gear_ratio") or 1.0)
                    * 360.0 * (j.get("sign", 1) or 1) + j.get("offset_deg", 0.0))
    return pos * 360.0


def leg_len_mm(cfg, pos_by_id):
    """cfg["joints"][0] is the root joint, [1] the distal one -- the order is what
    matters, not the controller ids."""
    lg = cfg.get("leg") or {}
    js = cfg.get("joints") or []
    if not lg or len(js) < 2:
        return None
    p1, p2 = pos_by_id.get(int(js[0]["id"])), pos_by_id.get(int(js[1]["id"]))
    if p1 is None or p2 is None:
        return None
    q_knee = joint_deg(cfg, int(js[1]["id"]), p2)
    # folded-zero convention: knee is the included angle, 0 folded / 180 straight
    inc = abs((q_knee + lg.get("knee_offset_deg", 0.0) + 180.0) % 360.0 - 180.0)
    return (abs(2.0 * lg.get("segment_mm", 160.0) * math.sin(math.radians(inc) / 2.0))
            + lg.get("foot_offset_mm", 0.0))


async def run(args):
    cfg = load_cfg()
    transport = moteus.Fdcanusb()

    qr = moteus.QueryResolution()
    qr.trajectory_complete = mp.INT8
    qr.q_current = mp.F32
    qr.d_current = mp.F32
    qr.power = mp.F32
    qr.motor_temperature = mp.F32
    qr.home_state = mp.INT8
    ctrl = {mid: moteus.Controller(id=mid, transport=transport, query_resolution=qr)
            for mid in MOTOR_IDS}

    # Make sure nothing is holding a pose, then never command again.
    await transport.cycle([c.make_stop() for c in ctrl.values()])

    telemetry = _Telemetry(args.telemetry) if args.telemetry else None
    period = 1.0 / args.rate
    print(f"[*] Monitor: read-only, motors STOPPED and free to move by hand. "
          f"{args.rate:g} Hz." + (f"  telemetry -> {args.telemetry}" if telemetry else ""))
    print("[*] 'wrap' is how far the position is from the +-0.5 rev power-up wrap, "
          "in degrees at the joint.")
    print("    Ctrl-C to quit.  This never energises the motors.")

    t_print = 0.0
    while True:
        results = await transport.cycle([c.make_query() for c in ctrl.values()])
        by_id = {r.id: r for r in results}
        if telemetry:
            telemetry.send(results, {}, "MONITOR")

        now = time.time()
        if now - t_print >= args.print_every:
            t_print = now
            cells = []
            pos = {}
            for mid in MOTOR_IDS:
                r = by_id.get(mid)
                if r is None:
                    cells.append(f"m{mid}: no reply")
                    continue
                v = r.values
                p = v.get(moteus.Register.POSITION)
                pos[mid] = p
                ratio = 9.0
                for j in cfg.get("joints", []):
                    if int(j["id"]) == mid:
                        ratio = j.get("gear_ratio") or 1.0
                wrap = (0.5 - abs(p)) * 360.0 / ratio if abs(p) < 0.5 else float("inf")
                cells.append(f"m{mid}: {p:+8.4f} rev {joint_deg(cfg, mid, p):+7.1f}d"
                             + (f" wrap {wrap:4.1f}d" if math.isfinite(wrap) else "          "))
            ln = leg_len_mm(cfg, pos)
            mode = by_id.get(MOTOR_IDS[0])
            mode_s = int(mode.values.get(moteus.Register.MODE, -1)) if mode else -1
            print("  " + "   ".join(cells)
                  + (f"   leg {ln:6.1f} mm" if ln is not None else "")
                  + f"   mode {mode_s}", flush=True)
        await asyncio.sleep(period)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--telemetry", metavar="HOST:PORT", default=None,
                    help="stream to the dashboard, e.g. 127.0.0.1:9871")
    ap.add_argument("--rate", type=float, default=100.0, help="query rate, Hz (default 100)")
    ap.add_argument("--print-every", type=float, default=0.25,
                    help="console line interval in seconds (default 0.25)")
    args = ap.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[*] done (motors were never energised).")


if __name__ == "__main__":
    main()
