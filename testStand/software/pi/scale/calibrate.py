#!/usr/bin/env python3
"""Multi-point calibration of the 4-cell plate with a set of known masses.

Every reading is referenced to the nearest zero (tare) readings, so the drift
this rig is known for cancels out instead of landing in the calibration:

    zero -> 100 g -> zero -> 200 g -> zero -> 500 g -> ...

Each point uses the average of the zero taken just before it and the one just
after it (if there is one), so a slow wander during the session is bracketed
rather than baked in. The fit is then mass vs summed delta-counts across all
four cells, which is what a rigid platform physically measures:

    W = K * sum_i (r_i - z_i)          (+ optional per-cell gains, if the
                                        points span enough load positions)

Step-by-step (drive it from any shell, or let a script drive it):

    python3 calibrate.py zero                 # plate EMPTY, take a tare
    python3 calibrate.py point --mass 500     # 500 g on, take a reading
    python3 calibrate.py zero                 # empty again ...
    python3 calibrate.py point --mass 1000 --note "corner ch2"
    python3 calibrate.py fit                  # fit + write scale_cal.json
    python3 calibrate.py live                 # watch it in grams

Guided (prompts you through each mass, zeros between them, fits at the end):

    python3 calibrate.py session 100 200 500 1000 2000

Housekeeping:

    python3 calibrate.py status               # what has been recorded so far
    python3 calibrate.py reset                # archive session, start over
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ads131 import ADS131M0x, REG_GAIN1, REG_CLOCK  # noqa: E402

SESSION = os.path.join(HERE, "cal_session.json")
CAL_PATH = os.path.join(HERE, "scale_cal.json")
CELLS = [0, 1, 2, 3]
FULL_SCALE = 1 << 23


# ----------------------------------------------------------------- plumbing
def check_clkin():
    """The chip is dead without CLKIN. Say so instead of timing out cryptically."""
    try:
        out = subprocess.run(["pinctrl", "get", "4"], capture_output=True,
                             text=True, timeout=2).stdout
    except Exception:
        return
    if "a0" not in out or "GPCLK0" not in out:
        sys.exit("GPIO4 is not driving CLKIN (GPCLK0). Run:\n"
                 f"    sudo python3 {os.path.join(HERE, 'clkin.py')}\n"
                 "then try again.")


def open_adc(gain, osr):
    check_clkin()
    adc = ADS131M0x(nch=8)
    adc.reset()
    adc.configure(gain=gain, osr=osr)
    for _ in range(50):             # let the sinc filter refill after reconfig
        adc.read()
    return adc


def acquire(adc, seconds):
    """seconds of independent conversions -> per-cell mean, sd, and the
    drift slope of the total (counts/s) across the window."""
    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        r = adc.read()
        rows.append([r[c] for c in CELLS])
    arr = np.asarray(rows, dtype=float)
    tot = arr.sum(axis=1)
    t = np.linspace(0.0, seconds, len(tot))
    slope = float(np.polyfit(t, tot, 1)[0]) if len(tot) > 10 else 0.0
    return {"mean": arr.mean(axis=0).tolist(),
            "sd": arr.std(axis=0).tolist(),
            "n": len(rows), "seconds": seconds,
            "total_sd": float(tot.std()),
            "drift_counts_per_s": slope,
            "peak_abs": int(np.abs(arr).max())}


def load_session():
    if os.path.exists(SESSION):
        with open(SESSION) as f:
            return json.load(f)
    return {"created": time.strftime("%Y-%m-%d %H:%M:%S"), "gain": None,
            "osr": None, "events": []}


def save_session(s):
    with open(SESSION, "w") as f:
        json.dump(s, f, indent=1)


def stamp():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def fmt_cells(vals, w=9, d=0):
    return "  ".join(f"{v:>{w}.{d}f}" for v in vals)


# ------------------------------------------------------------ zero pairing
def paired_points(events):
    """Each point with its bracketing zeros: the last zero before it, and the
    first zero after it if one exists. Returns list of
    (point_event, zero_vector, how) where how is 'pre+post' or 'pre'."""
    out = []
    for i, ev in enumerate(events):
        if ev["type"] != "point":
            continue
        pre = next((e for e in reversed(events[:i]) if e["type"] == "zero"), None)
        post = next((e for e in events[i + 1:] if e["type"] == "zero"), None)
        if pre is None:
            continue
        if post is not None:
            z = ((np.array(pre["mean"]) + np.array(post["mean"])) / 2).tolist()
            how = "pre+post"
        else:
            z = pre["mean"]
            how = "pre"
        out.append((ev, z, how))
    return out


def running_K(events):
    """Single through-origin gain from everything recorded so far, for the
    provisional readout printed after each point. None until 1 point exists."""
    pts = paired_points(events)
    if not pts:
        return None
    tot = np.array([sum(np.array(p["mean"]) - np.array(z)) for p, z, _ in pts])
    m = np.array([p["mass"] for p, _, _ in pts])
    if not np.any(tot):
        return None
    return float((m @ tot) / (tot @ tot))


# --------------------------------------------------------------- commands
def cmd_zero(a):
    s = load_session()
    if s["gain"] not in (None, a.gain) or s["osr"] not in (None, a.osr):
        sys.exit(f"session was recorded at gain {s['gain']} / osr {s['osr']}; "
                 "use the same settings or `reset` first")
    adc = open_adc(a.gain, a.osr)
    print(f"taring over {a.seconds:.0f} s -- plate must be EMPTY ...", flush=True)
    r = acquire(adc, a.seconds)
    adc.close()
    r.update({"type": "zero", "time": stamp()})
    s["gain"], s["osr"] = a.gain, a.osr
    s["events"].append(r)
    save_session(s)
    print("  zero counts   " + fmt_cells(r["mean"]))
    print("  noise (sd)    " + fmt_cells(r["sd"]) +
          f"     total sd {r['total_sd']:.0f}")
    print(f"  drift         {r['drift_counts_per_s']:+.0f} counts/s over the window")
    K = running_K(s["events"])
    if K:
        print(f"  (= noise {abs(K) * r['total_sd']:.2f} g/sample, "
              f"drift {K * r['drift_counts_per_s']:+.2f} g/s)")
    nz = sum(1 for e in s["events"] if e["type"] == "zero")
    print(f"  saved zero #{nz}")


def cmd_point(a):
    s = load_session()
    zeros = [e for e in s["events"] if e["type"] == "zero"]
    if not zeros:
        sys.exit("no zero on file -- run `calibrate.py zero` with the plate empty first")
    if s["gain"] != a.gain or s["osr"] != a.osr:
        sys.exit(f"session was recorded at gain {s['gain']} / osr {s['osr']}; "
                 "use the same settings or `reset` first")
    last_z = zeros[-1]
    age = time.time() - time.mktime(time.strptime(last_z["time"], "%Y-%m-%d %H:%M:%S"))
    if age > 120:
        print(f"  WARNING: last zero is {age / 60:.1f} min old -- drift will leak "
              "into this point. Consider a fresh `zero` first.")

    adc = open_adc(a.gain, a.osr)
    print(f"reading {a.mass:g} g over {a.seconds:.0f} s ...", flush=True)
    r = acquire(adc, a.seconds)
    adc.close()
    r.update({"type": "point", "time": stamp(), "mass": a.mass, "note": a.note})

    d = np.array(r["mean"]) - np.array(last_z["mean"])
    tot = float(d.sum())
    share = 100 * np.abs(d) / max(np.abs(d).sum(), 1)
    K_prev = running_K(s["events"])       # from points BEFORE this one

    s["events"].append(r)
    save_session(s)

    print("  counts        " + fmt_cells(r["mean"]))
    print("  delta         " + fmt_cells(d) + f"     total {tot:>+10.0f}")
    print("  load share    " + fmt_cells(share, d=0) + "  %")
    print("  noise (sd)    " + fmt_cells(r["sd"]) +
          f"     total sd {r['total_sd']:.0f}  (sd of mean {r['total_sd'] / r['n'] ** 0.5:.0f})")
    print(f"  drift         {r['drift_counts_per_s']:+.0f} counts/s over the window")
    print(f"  headroom      largest |count| {r['peak_abs']:,} = "
          f"{100 * r['peak_abs'] / FULL_SCALE:.1f}% of full scale")
    if abs(tot) < 20 * r["total_sd"] / r["n"] ** 0.5:
        print("  WARNING: barely above the noise -- is the mass on the plate?")
    if a.mass:
        print(f"  this point    {tot / a.mass:+.1f} counts/g")
    if K_prev:
        est = K_prev * tot
        print(f"  vs earlier pts  reads {est:.1f} g  (nominal {a.mass:g} g, "
              f"{est - a.mass:+.1f} g, {100 * (est - a.mass) / a.mass:+.2f}%)")
    npt = sum(1 for e in s["events"] if e["type"] == "point")
    print(f"  saved point #{npt}" + (f"  [{a.note}]" if a.note else ""))


def cmd_status(a):
    s = load_session()
    if not s["events"]:
        print("empty session")
        return
    print(f"session started {s['created']}   gain {s['gain']}  osr {s['osr']}")
    print(f"{'#':>3} {'time':8} {'type':5} {'mass':>7}   {'total delta':>11}  note")
    for i, ev in enumerate(s["events"]):
        t = ev["time"][11:]
        if ev["type"] == "zero":
            print(f"{i:>3} {t} zero  {'':>7}   {'':>11}  sd {ev['total_sd']:.0f}")
        else:
            pre = next((e for e in reversed(s["events"][:i]) if e["type"] == "zero"), None)
            tot = sum(np.array(ev["mean"]) - np.array(pre["mean"])) if pre else float("nan")
            print(f"{i:>3} {t} point {ev['mass']:>7g}   {tot:>+11.0f}  {ev.get('note') or ''}")


def cmd_reset(a):
    if os.path.exists(SESSION):
        dst = os.path.join(HERE, time.strftime("cal_session_%Y%m%d-%H%M%S.json"))
        shutil.move(SESSION, dst)
        print(f"archived -> {dst}")
    print("session cleared")


# --------------------------------------------------------------------- fit
def fit_global(D, m):
    """mass = K * total  (through origin) and mass = K * total + b."""
    tot = D.sum(axis=1)
    K0 = float((m @ tot) / (tot @ tot))
    A = np.column_stack([tot, np.ones_like(tot)])
    (K1, b1), *_ = np.linalg.lstsq(A, m, rcond=None)
    return K0, float(K1), float(b1)


def zero_rows(events, max_gap_s=600):
    """Consecutive-zero differences. The plate is empty both times, so the
    true force change is zero, and any change in the SUM is gain mismatch
    acting on the redistribution between cells -- free information about the
    relative gains, and the only kind this rig produces in quantity."""
    zs = [e for e in events if e["type"] == "zero"]
    rows = []
    for a, b in zip(zs, zs[1:]):
        ta = time.mktime(time.strptime(a["time"], "%Y-%m-%d %H:%M:%S"))
        tb = time.mktime(time.strptime(b["time"], "%Y-%m-%d %H:%M:%S"))
        if tb - ta <= max_gap_s:
            rows.append(np.array(b["mean"]) - np.array(a["mean"]))
    return np.array(rows) if rows else np.zeros((0, len(CELLS)))


def fit_per_cell(D, m, lam, Z=None):
    """min ||D k - m||^2 + ||Z k||^2 + lam * scale * ||k - mean(k)||^2
    Ridge toward a common gain: only deviates from 'one gain for all' by as
    much as the data actually distinguishes the cells. Z rows are
    redistributions with a known target of zero."""
    n = D.shape[1]
    A = D.T @ D
    b = D.T @ m
    scale = np.trace(A) / n
    if Z is not None and len(Z):
        A = A + Z.T @ Z
    P = np.eye(n) - np.ones((n, n)) / n          # projects out the common mode
    k = np.linalg.solve(A + lam * scale * P, b)
    return k


def cmd_fit(a):
    s = load_session()
    pts = paired_points(s["events"])
    if len(pts) < 2:
        sys.exit(f"need at least 2 points with a zero before each; have {len(pts)}")

    D = np.array([np.array(p["mean"]) - np.array(z) for p, z, _ in pts])
    m = np.array([p["mass"] for p, _, _ in pts], dtype=float)
    notes = [p.get("note") or "" for p, _, _ in pts]
    hows = [h for _, _, h in pts]
    tot = D.sum(axis=1)

    K0, K1, b1 = fit_global(D, m)
    pred0, pred1 = K0 * tot, K1 * tot + b1
    err0, err1 = pred0 - m, pred1 - m

    print(f"{len(pts)} points, {sum(1 for e in s['events'] if e['type'] == 'zero')} zeros, "
          f"gain {s['gain']}, osr {s['osr']}\n")
    print(f"{'mass':>7} {'total delta':>12} {'counts/g':>9}  {'K*tot':>8} {'err g':>7} "
          f"{'err %':>7}  {'zero':8} note")
    for i in range(len(pts)):
        print(f"{m[i]:>7g} {tot[i]:>+12.0f} {tot[i] / m[i]:>+9.1f}  {pred0[i]:>8.1f} "
              f"{err0[i]:>+7.1f} {100 * err0[i] / m[i]:>+6.2f}%  {hows[i]:8} {notes[i]}")

    print(f"\nGLOBAL fit (one gain, all cells)")
    print(f"  through origin   K = {K0:.7g} g/count   ({1 / K0:+.2f} counts/g)")
    print(f"     rms err {np.sqrt(np.mean(err0 ** 2)):.2f} g   max |err| {np.abs(err0).max():.2f} g "
          f"({100 * np.abs(err0 / m).max():.2f}% of reading)")
    print(f"  with offset      K = {K1:.7g} g/count,  b = {b1:+.2f} g")
    print(f"     rms err {np.sqrt(np.mean(err1 ** 2)):.2f} g   max |err| {np.abs(err1).max():.2f} g")
    fs = m.max()
    print(f"  nonlinearity     {100 * np.abs(err1).max() / fs:.3f}% of {fs:g} g span "
          f"(residual after best line)")
    if abs(b1) > 3 * np.sqrt(np.mean(err0 ** 2)) and len(pts) >= 3:
        print(f"  NOTE: offset b={b1:+.1f} g is significant -- points may not share "
              "a consistent zero (drift between zero and point?)")

    # Per-cell gains. The mass points alone barely distinguish the cells (the
    # load was always near the middle), but the zero-to-zero redistributions
    # do: the plate re-seats by tens of thousands of counts between cells with
    # nothing on it, and whatever leaks into the sum is gain mismatch.
    gains = [K0] * len(CELLS)
    mode = "global"
    Z = zero_rows(s["events"])
    rms_g = float(np.sqrt(np.mean(err0 ** 2)))
    zres_g = K0 * Z.sum(axis=1) if len(Z) else np.zeros(0)
    print(f"\nPER-CELL fit  ({len(pts)} mass points + {len(Z)} zero-to-zero rows)")
    if len(Z):
        print(f"  redistribution between zeros: up to {np.abs(Z).max():,.0f} counts on one cell;")
        print(f"  with the global gain the empty-plate sum moves rms {np.sqrt(np.mean(zres_g ** 2)):.2f} g, "
              f"max {np.abs(zres_g).max():.2f} g")
    best, cands = None, []
    for lam in (0.0, 0.003, 0.01, 0.03, 0.1, 0.3, 1.0):
        try:
            k = fit_per_cell(D, m, lam, Z)
        except np.linalg.LinAlgError:
            continue
        e = D @ k - m
        rms = float(np.sqrt(np.mean(e ** 2)))
        zr = Z @ k if len(Z) else np.zeros(0)
        zrms = float(np.sqrt(np.mean(zr ** 2))) if len(Z) else 0.0
        rel = k / k.mean()
        print(f"  lam={lam:<6} mass rms {rms:5.2f} g  max {np.abs(e).max():5.2f} g   "
              f"zero-return rms {zrms:5.2f} g   rel gains "
              + " ".join(f"{r:.3f}" for r in rel))
        score = rms ** 2 + zrms ** 2
        cands.append((score, lam, k, e, rms, zrms))
    # Prefer the most regularised lambda that is within 5% of the best score:
    # nearly the same fit with less extreme gains extrapolates better.
    if cands:
        lo = min(c[0] for c in cands)
        best = max((c for c in cands if c[0] <= 1.05 * lo), key=lambda c: c[1])
    if best and best[4] < 0.8 * rms_g and np.all(np.abs(best[2] / best[2].mean() - 1) < 0.5):
        gains, mode = best[2].tolist(), "per-cell"
        print(f"  -> using per-cell gains (lam={best[1]}): mass rms {best[4]:.2f} g "
              f"(global {rms_g:.2f} g), zero-return rms {best[5]:.2f} g")
        print(f"     g/count per cell: " + "  ".join(f"ch{c}:{g:.6g}" for c, g in zip(CELLS, gains)))
        print(f"\n  {'mass':>7} {'global':>8} {'per-cell':>9}   note")
        for i in range(len(pts)):
            print(f"  {m[i]:>7g} {pred0[i]:>8.1f} {m[i] + best[3][i]:>9.1f}   {notes[i]}")
    else:
        print("  -> per-cell does not beat the global gain by enough; keeping global")

    # write the calibration in the format scale.py / server.py already read
    last_zero = [e for e in s["events"] if e["type"] == "zero"][-1]
    cal = {
        "cells": CELLS,
        "zeros": last_zero["mean"],
        "gains": gains,
        "unit": "g",
        "mode": mode,
        "gain_pga": s["gain"],
        "osr": s["osr"],
        "idle_sd": last_zero["total_sd"],
        "ref_masses": m.tolist(),
        "fit": {"K_origin": K0, "K_line": K1, "b_line": b1,
                "rms_err_g": float(np.sqrt(np.mean(err0 ** 2))),
                "max_err_g": float(np.abs(err0).max()),
                "nonlinearity_pct_fs": float(100 * np.abs(err1).max() / fs),
                "points": [{"mass": float(m[i]), "delta": D[i].tolist(),
                            "zero_how": hows[i], "note": notes[i]}
                           for i in range(len(pts))]},
        "calibrated": stamp(),
    }
    if os.path.exists(CAL_PATH):
        shutil.copy(CAL_PATH, CAL_PATH + ".bak")
    with open(CAL_PATH, "w") as f:
        json.dump(cal, f, indent=2)
    noise_g = abs(K0) * last_zero["total_sd"]
    print(f"\nnoise: {noise_g:.2f} g per sample at {s['osr']} OSR, "
          f"~{noise_g / 20 ** 0.5:.2f} g per 20-sample reading")
    print(f"saved -> {CAL_PATH}  (previous copy in scale_cal.json.bak)")


# -------------------------------------------------------------------- live
def cmd_live(a):
    s = load_session()
    cal = json.load(open(CAL_PATH)) if os.path.exists(CAL_PATH) else None
    gains = None
    zeros = None
    if cal and cal.get("gain_pga") == a.gain and any(cal.get("gains", [])):
        gains = np.array(cal["gains"])
        zeros = np.array(cal["zeros"])
        src = "scale_cal.json"
    zs = [e for e in s["events"] if e["type"] == "zero"]
    if zs and s.get("gain") == a.gain:
        zeros = np.array(zs[-1]["mean"])            # freshest tare wins
        src = "session zero"
    if zeros is None:
        zeros = np.zeros(len(CELLS))
        src = "none (raw counts)"
    adc = open_adc(a.gain, a.osr)
    tty = sys.stdout.isatty()
    print(f"live  --  zero from {src}"
          + ("" if gains is None else f",  K = {gains.mean():.5g} g/count")
          + "   (ctrl-C to stop)")
    if a.tare:
        print("taring ...", flush=True)
        zeros = np.array(acquire(adc, 2.0)["mean"])
    hdr = "   " + "  ".join(f"{'ch' + str(c):>9}" for c in CELLS) + f"  {'total':>10}"
    print(hdr + (f"  {'grams':>9}" if gains is not None else ""))
    try:
        while True:
            r = acquire(adc, a.interval)
            d = np.array(r["mean"]) - zeros
            line = "   " + fmt_cells(d) + f"  {d.sum():>+10.0f}"
            if gains is not None:
                line += f"  {float(gains @ d):>9.2f}"
            end = "\r" if tty and not a.lines else "\n"
            print(line + "   ", end=end, flush=True)
    except KeyboardInterrupt:
        print()
    adc.close()


# ----------------------------------------------------------------- session
def cmd_session(a):
    """Guided run. Same commands as the step-by-step, just prompted."""
    if a.masses and os.path.exists(SESSION) and load_session()["events"]:
        ans = input("a session already exists -- (a)ppend, (r)eset, (q)uit? ").strip().lower()
        if ans == "r":
            cmd_reset(a)
        elif ans != "a":
            return
    for i, mass in enumerate(a.masses):
        input(f"\n[{i + 1}/{len(a.masses)}] remove everything from the plate, then Enter ")
        cmd_zero(a)
        where = a.where or "the CENTER"
        input(f"        place {mass:g} g on {where}, wait for it to settle, then Enter ")
        a.mass, a.note = mass, (a.where or "")
        cmd_point(a)
    input("\nremove everything, then Enter for the closing zero ")
    cmd_zero(a)
    print()
    cmd_fit(a)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gain", type=int, default=64, help="PGA gain (1..128)")
    p.add_argument("--osr", type=int, default=4096, help="oversampling ratio (4096 = 1 kSPS)")
    p.add_argument("--seconds", type=float, default=3.0, help="averaging window per reading")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("zero", help="tare with the plate empty").set_defaults(fn=cmd_zero)
    q = sub.add_parser("point", help="record a known mass")
    q.add_argument("--mass", type=float, required=True, help="grams")
    q.add_argument("--note", default="", help="e.g. 'center', 'corner ch2'")
    q.set_defaults(fn=cmd_point)
    sub.add_parser("fit", help="fit and write scale_cal.json").set_defaults(fn=cmd_fit)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("reset").set_defaults(fn=cmd_reset)
    lv = sub.add_parser("live", help="continuous readout")
    lv.add_argument("--interval", type=float, default=0.25, help="seconds per line")
    lv.add_argument("--lines", action="store_true", help="newline per update, not \\r")
    lv.add_argument("--tare", action="store_true", help="tare at start")
    lv.set_defaults(fn=cmd_live)
    se = sub.add_parser("session", help="guided calibration")
    se.add_argument("masses", type=float, nargs="+", help="grams, in the order you'll place them")
    se.add_argument("--where", default="", help="placement note, e.g. 'corner ch0'")
    se.set_defaults(fn=cmd_session)

    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
