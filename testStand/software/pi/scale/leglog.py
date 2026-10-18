#!/usr/bin/env python3
"""Leg test-stand logger: force plate + carriage height, from the command line.

The acquisition itself is `platelog` (C++, same directory); this wraps it with
the calibration files and adds plotting.

    python3 leglog.py tare                 # plate EMPTY: store zeros
    python3 leglog.py check                # live Fz / z readout to sanity-check
    python3 leglog.py run --seconds 20 --tag drop1
    python3 leglog.py run --osr 1024       # 4 kSPS for impacts
    python3 leglog.py plot logs/leg-....csv
    python3 leglog.py flip-encoder         # if z goes DOWN when the carriage goes UP

Hardware bring-up (once per boot):   sudo ./plate_setup.sh
"""
import argparse
import io
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PLATELOG = os.path.join(HERE, "platelog")
CAL = os.path.join(HERE, "scale_cal.json")
TARE = os.path.join(HERE, "plate_tare.json")
CFG = os.path.join(HERE, "leglog_config.json")
ENC_CAL = os.path.expanduser("~/encoder_calib.yaml")
LOGS = os.path.join(HERE, "logs")
G0 = 9.80665


# ------------------------------------------------------------------ config
def load_cfg():
    if os.path.exists(CFG):
        return json.load(open(CFG))
    cfg = {"m_per_count": -1.735e-4, "source": "default"}
    if os.path.exists(ENC_CAL):
        try:
            import yaml
            ec = yaml.safe_load(open(ENC_CAL))
            # July cal was fitted in cm per count
            cfg = {"m_per_count": float(ec["slope_b"]) / 100.0,
                   "source": f"{ENC_CAL} slope_b/100"}
        except Exception as e:  # noqa: BLE001
            cfg["source"] = f"default ({e})"
    json.dump(cfg, open(CFG, "w"), indent=2)
    return cfg


def newton_per_count():
    cal = json.load(open(CAL))
    return [k * G0 / 1000.0 for k in cal["gains"]], cal


def platelog_args(a, zeros):
    npc, cal = newton_per_count()
    cmd = [PLATELOG, "--gain", str(cal.get("gain_pga", 64)), "--osr", str(a.osr),
           "--newton", ",".join(f"{v:.9g}" for v in npc),
           "--m-per-count", f"{load_cfg()['m_per_count']:.9g}"]
    if zeros is not None:
        cmd += ["--zeros", ",".join(f"{z:.2f}" for z in zeros)]
    return cmd


def check_not_running():
    r = subprocess.run(["pgrep", "-x", "platelog"], capture_output=True, text=True)
    if r.stdout.strip():
        sys.exit(f"platelog is already running (pid {r.stdout.split()[0]}) -- "
                 "probably the dashboard. Stop it first, or use the dashboard's controls.")


def load_tare(a):
    if getattr(a, "tare", False) or not os.path.exists(TARE):
        return None
    t = json.load(open(TARE))
    age = time.time() - t["time"]
    if t.get("osr") != a.osr:
        print(f"note: stored tare was at osr {t.get('osr')}, using it anyway")
    print(f"using tare from {time.strftime('%H:%M:%S', time.localtime(t['time']))} "
          f"({age / 60:.1f} min ago); pass --tare to re-tare at start")
    return t["zeros"]


# ---------------------------------------------------------------- commands
def cmd_tare(a):
    check_not_running()
    print(f"taring over {a.seconds:.0f} s -- plate must be EMPTY (leg off the plate)...", flush=True)
    r = subprocess.run(platelog_args(a, None) + ["--tare-only", "--tare-seconds", str(a.seconds)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(r.stderr.strip())
    t = json.loads(r.stdout)
    t["time"] = time.time()
    json.dump(t, open(TARE, "w"), indent=2)
    npc, _ = newton_per_count()
    sd_n = float(np.sqrt(sum((s * n) ** 2 for s, n in zip(t["sd"], npc))))
    print("  zeros:", "  ".join(f"{z:.0f}" for z in t["zeros"]))
    print(f"  per-sample noise {sd_n:.4f} N   saved -> {TARE}")


def cmd_check(a):
    check_not_running()
    zeros = load_tare(a)
    cmd = platelog_args(a, zeros) + ["--status", "0.2", "--status-newline", "--seconds", str(a.seconds)]
    if zeros is None:
        cmd += ["--tare-seconds", "2"]
    print("Lift the carriage: z should INCREASE. Press on the plate: Fz should INCREASE.")
    print("If z goes the wrong way: python3 leglog.py flip-encoder\n")
    subprocess.run(cmd)


def cmd_run(a):
    check_not_running()
    os.makedirs(LOGS, exist_ok=True)
    zeros = load_tare(a)
    name = time.strftime("leg-%Y%m%d-%H%M%S") + (f"-{a.tag}" if a.tag else "") + ".csv"
    path = os.path.join(LOGS, name)
    cmd = platelog_args(a, zeros) + ["--out", path, "--udp", "127.0.0.1:9870", "--status", "0.5"]
    if zeros is None:
        cmd += ["--tare-seconds", "2"]
    if a.seconds:
        cmd += ["--seconds", str(a.seconds)]
    if a.note:
        cmd += ["--note", a.note]
    print("ctrl-C to stop" if not a.seconds else f"running {a.seconds:g} s")
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass
    if os.path.exists(path):
        print(f"\nsaved {path}")
        if not a.no_plot:
            a.csv = path
            cmd_plot(a)


def read_log(path):
    meta, rows = {}, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                parts = line[1:].split()
                if parts:
                    meta.setdefault(parts[0], []).append(" ".join(parts[1:]))
            else:
                rows.append(line)
    d = np.genfromtxt(io.StringIO("".join(rows)), delimiter=",", names=True)
    return meta, d


def cmd_plot(a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    meta, d = read_log(a.csv)
    t, fz, z = d["t_s"], d["fz_N"], d["z_m"]
    comp = z.max() - z                      # compression below the highest point seen
    out = os.path.splitext(a.csv)[0] + ".png"

    fig, ax = plt.subplots(3, 1, figsize=(11, 9))
    ax[0].plot(t, fz, lw=0.6)
    ax[0].set_ylabel("Fz [N]")
    ax[0].set_title(os.path.basename(a.csv) + "   " + (meta.get("end", [""])[0][:60]))
    ax[1].plot(t, z * 1000, lw=0.6, color="tab:orange")
    ax[1].set_ylabel("z [mm] (rel. start)")
    ax[1].set_xlabel("t [s]")
    ax[2].plot(comp * 1000, fz, lw=0.5, color="tab:green")
    ax[2].set_xlabel("compression [mm]  (z_max - z)")
    ax[2].set_ylabel("Fz [N]")
    for x in ax:
        x.grid(alpha=0.3)
    # a straight-line stiffness estimate over the loaded part, as in the paper
    m = fz > max(1.0, 0.1 * fz.max())
    if m.sum() > 50 and np.ptp(comp[m]) > 0.002:
        k, b = np.polyfit(comp[m], fz[m], 1)
        xs = np.linspace(comp[m].min(), comp[m].max(), 2)
        ax[2].plot(xs * 1000, k * xs + b, "k--", lw=1, label=f"fit {k:.0f} N/m (Fz > {max(1.0, 0.1 * fz.max()):.1f} N)")
        ax[2].legend()
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"plot -> {out}")
    print(f"  {len(t)} samples, {t[-1]:.2f} s, Fz {fz.min():.2f}..{fz.max():.2f} N, "
          f"z range {np.ptp(z) * 1000:.1f} mm")


def cmd_flip(a):
    cfg = load_cfg()
    cfg["m_per_count"] = -cfg["m_per_count"]
    cfg["source"] = cfg.get("source", "") + " (flipped)"
    json.dump(cfg, open(CFG, "w"), indent=2)
    print(f"m_per_count now {cfg['m_per_count']:+.6g}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--osr", type=int, default=4096, help="4096 = 1 kSPS (default), 1024 = 4 kSPS")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("tare"); s.add_argument("--seconds", type=float, default=3); s.set_defaults(fn=cmd_tare)
    s = sub.add_parser("check"); s.add_argument("--seconds", type=float, default=30)
    s.add_argument("--tare", action="store_true"); s.set_defaults(fn=cmd_check)
    s = sub.add_parser("run"); s.add_argument("--seconds", type=float, default=0)
    s.add_argument("--tag", default=""); s.add_argument("--note", default="")
    s.add_argument("--tare", action="store_true", help="re-tare at start (plate must be empty)")
    s.add_argument("--no-plot", action="store_true"); s.set_defaults(fn=cmd_run)
    s = sub.add_parser("plot"); s.add_argument("csv"); s.set_defaults(fn=cmd_plot)
    sub.add_parser("flip-encoder").set_defaults(fn=cmd_flip)
    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
