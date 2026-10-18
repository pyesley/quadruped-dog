#!/usr/bin/env python3
"""String pot -> height.  Reads the kernel rotary-encoder device, applies cal.json.

    python3 ~/stringpot/stringpot.py --y0 23      # cart at 23 mm on the ruler now
    python3 ~/stringpot/stringpot.py              # assume calibration's start point
    python3 ~/stringpot/stringpot.py --raw        # counts only (used by the laptop cal app)

Prints "t count y_mm dy_mm" on every change (heartbeat every 0.5 s); dy is
relative to the start.  Import StringPot for use in other programs.
"""
import argparse, glob, json, math, os, select, struct, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
FMT = "llHHi"; SZ = struct.calcsize(FMT)          # struct input_event, 64-bit


class StringPot:
    def __init__(self, cal=os.path.join(HERE, "cal.json"), y0=None):
        c = json.load(open(cal))
        self.alpha, self.k, self.N0 = c["alpha_mm2_per_count2"], c["k_mm2"], c["N0_counts"]
        dev = next(("/dev/input/" + os.path.basename(p) for p in glob.glob("/sys/class/input/event*")
                    if open(p + "/device/name").read().startswith("rotary@")), None)
        if dev is None:
            sys.exit("no rotary@ device: sudo dtoverlay rotary-encoder pin_a=5 pin_b=6 "
                     "relative_axis=1 steps-per-period=4")
        if "steps-per-period=4" not in subprocess.run(["dtoverlay", "-l"], capture_output=True,
                                                      text=True).stdout:
            sys.exit("rotary-encoder overlay not loaded with steps-per-period=4 -- cal.json is invalid")
        self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        self.count = 0
        if y0 is not None:                         # re-solve N0 so that y(0) = y0
            self.N0 = math.copysign(math.sqrt(max(y0 ** 2 - self.k, 0) / self.alpha), self.N0)
        self.y_start = self.y(0)

    def y(self, n):
        return math.sqrt(max(self.alpha * (n + self.N0) ** 2 + self.k, 0.0))

    def poll(self, timeout=0.0):
        """Drain encoder events. Returns True if the count changed."""
        changed = False
        if select.select([self.fd], [], [], timeout)[0]:
            buf = os.read(self.fd, SZ * 256)
            for i in range(0, len(buf) - SZ + 1, SZ):
                _, _, typ, _, val = struct.unpack_from(FMT, buf, i)
                if typ == 2:                       # EV_REL
                    self.count += val; changed = True
        return changed

    @property
    def dy(self):
        return self.y(self.count) - self.y_start


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--y0", type=float, help="ruler height (mm) of the cart right now")
    ap.add_argument("--raw", action="store_true", help="print 't count' only, no calibration")
    a = ap.parse_args()
    sp = StringPot(y0=a.y0) if not a.raw else None
    if a.raw:                                      # raw mode needs no cal.json
        sp = StringPot.__new__(StringPot); sp.count = 0
        dev = next("/dev/input/" + os.path.basename(p) for p in glob.glob("/sys/class/input/event*")
                   if open(p + "/device/name").read().startswith("rotary@"))
        sp.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    else:
        print(f"# y_start {sp.y_start:.2f} mm  N0 {sp.N0:.1f}", flush=True)
    last = 0.0
    while True:
        ch = sp.poll(0.05)
        now = time.time()
        if ch or now - last > 0.5:
            print(f"{now:.3f} {sp.count}" if a.raw else
                  f"{now:.3f} {sp.count} {sp.y(sp.count):.2f} {sp.dy:+.2f}", flush=True)
            last = now
