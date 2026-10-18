#!/usr/bin/env python3
"""Force-plate calibration with precision weights at known spots -> Fz AND the two moments.

    python3 platecal/platecal.py        (dash.py must be running)   http://localhost:8091

Print template.pdf at 100 %, tape it on the plate, lift the leg off the plate.  The page
walks through a plan of (mass, point) placements with an EMPTY reading between each, so
every load is referenced to the average of the empties either side of it (cancels drift).

Model.  A mass m at (x, y) (plate frame: origin = centre, x along the 200 mm length, y
across, mm) applies, exactly,
    Fz = m g,   Tx = Fz * x,   Ty = Fz * y          (N, N*mm, N*mm)
and the four cells read df = f - f_empty (N, platelog's existing per-cell gains).  Fit
    [Fz, Tx, Ty]^T = A df,     A = 3x4
by least squares, one row at a time.  A soaks up per-cell gain errors, where the cells
REALLY act (vs the drawing), the plate being over-constrained, moments carried through
the bolted joints, and crosstalk.  Centre of pressure = (Tx/Fz, Ty/Fz).  Each channel's
effective position falls out as (A[1,k]/A[0,k], A[2,k]/A[0,k]) -- which also answers
"which channel is which corner" without a press test.

Data come from dash.py's /cells (mean of each cell over a window).  Nothing is sent to
the Pi; platelog's own calibration is untouched.  Output: plate_matrix.json.
"""
import json, math, os, threading, time, urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 8091
DASH = "http://127.0.0.1:8080"
G = 9.80665
SETTLE_S, AVG_S = 1.5, 2.0
PTS = {p["pt"]: (p["x"], p["y"]) for p in json.load(open(os.path.join(HERE, "points.json")))}

# The plan: linearity at the centre, 200 g everywhere, 400 g at the far corners and ends.
PLAN = ([(11, m) for m in (5, 10, 20, 50, 100, 200, 400, 600)]
        + [(p, 200) for p in (1, 2, 3, 4, 5, 6, 7, 14, 13, 12, 11, 10, 9, 8, 15, 16, 17, 18, 19, 20, 21)]
        + [(p, 400) for p in (1, 7, 21, 15, 8, 14)])

lock = threading.Lock()
caps = []            # captures in order: {kind, f[4], s[4], t, (pt, x, y, mass_g for loads)}
ts0 = time.strftime("%Y%m%d-%H%M%S")
session_path = os.path.join(HERE, f"session-{ts0}.json")


def cells(sec):
    with urllib.request.urlopen(f"{DASH}/cells?sec={sec}", timeout=3) as r:
        return json.load(r)


def plan_pos():
    """Index into PLAN of the next load, and whether an empty is due first."""
    loads = sum(1 for c in caps if c["kind"] == "load")
    need_empty = not caps or caps[-1]["kind"] == "load"
    return loads, need_empty


def loads_with_deltas():
    """Each load minus the mean of the empties immediately before and after it."""
    out = []
    for i, c in enumerate(caps):
        if c["kind"] != "load":
            continue
        pre = next((caps[j] for j in range(i - 1, -1, -1) if caps[j]["kind"] == "empty"), None)
        post = next((caps[j] for j in range(i + 1, len(caps)) if caps[j]["kind"] == "empty"), None)
        if pre is None:
            continue
        e = np.array(pre["f"]) if post is None else (np.array(pre["f"]) + np.array(post["f"])) / 2
        drift = None if post is None else float(np.sum(post["f"]) - np.sum(pre["f"]))
        df = np.array(c["f"]) - e
        if df.sum() / G * 1000 < 0.3 * c["mass_g"]:
            continue                       # nothing (or not that mass) was on the plate
        out.append(dict(c, df=df.tolist(), drift=drift, bracketed=post is not None))
    return out


# Cell centres from forcePlate.pdf (pads at x +-49, y +-27), labelled as on the template.
CORNERS = {"A": (-49.0, 27.0), "B": (49.0, 27.0), "C": (-49.0, -27.0), "D": (49.0, -27.0)}


TWIST = {"A": 1.0, "B": -1.0, "C": -1.0, "D": 1.0}     # self-balanced pattern: sum, Tx, Ty all 0


def solve_general(D, T, u):
    """Least squares [Fz,Tx,Ty] = A df with A u = 0, u = the readings a pure twist
    produces.  Four supports under one plate: no weight can make that self-balanced
    + - - + load, but re-seating does, and an unconstrained A would read it as force."""
    Pj = np.eye(4) - np.outer(u, u) / (u @ u)
    X, *_ = np.linalg.lstsq(D @ Pj, T, rcond=None)
    return (Pj @ X).T


def phys_A(w, corners):
    xy = np.array([CORNERS[c] for c in corners])
    return np.vstack([w, w * xy[:, 0], w * xy[:, 1]])


def phys_w(D, T, corners):
    xy = np.array([CORNERS[c] for c in corners])
    M = np.vstack([D, D * xy[:, 0] / 50, D * xy[:, 1] / 50])
    w, *_ = np.linalg.lstsq(M, np.r_[T[:, 0], T[:, 1] / 50, T[:, 2] / 50], rcond=None)
    return w


def fit_physical(D, T):
    """Cells assumed at the drawing's pad centres; fit only one gain per channel, for every
    channel->corner assignment.  The best assignment answers "which channel is which corner";
    its residual says how well 'four vertical point supports' describes the plate."""
    import itertools
    best = None
    for perm in itertools.permutations("ABCD"):
        xy = np.array([CORNERS[c] for c in perm])
        # T_j = sum_k w_k * D_jk * [1, x_k, y_k]  -> linear in w (stack all three rows)
        M = np.vstack([D, D * xy[:, 0] / 50, D * xy[:, 1] / 50])
        w, *_ = np.linalg.lstsq(M, np.r_[T[:, 0], T[:, 1] / 50, T[:, 2] / 50], rcond=None)
        P = np.c_[D @ w, D @ (w * xy[:, 0]), D @ (w * xy[:, 1])]
        rms = float(np.sqrt(np.mean(((P[:, 0] - T[:, 0]) / G * 1000) ** 2)))
        cx = (P[:, 1] / P[:, 0]) - (T[:, 1] / T[:, 0])
        cy = (P[:, 2] / P[:, 0]) - (T[:, 2] / T[:, 0])
        big = T[:, 0] >= 0.1 * G
        cop = float(np.sqrt(np.mean(cx[big] ** 2 + cy[big] ** 2))) if big.any() else float("inf")
        if best is None or cop < best["cop_rms_mm"]:
            best = {"corners": "".join(perm), "gain_w": w.tolist(), "fz_rms_g": rms,
                    "cop_rms_mm": cop}
    return best


def fit():
    L = loads_with_deltas()
    res = {"n": len(L)}
    if not L:
        return res
    D = np.array([l["df"] for l in L])
    Fz = np.array([l["mass_g"] for l in L]) * G / 1000.0
    x = np.array([l["x"] for l in L], float)
    y = np.array([l["y"] for l in L], float)
    T = np.c_[Fz, Fz * x, Fz * y]
    # how far off the CURRENT calibration (plain sum of the cells) is
    s = D.sum(1)
    res["sum_gain_err_pct"] = float(np.median((s - Fz) / Fz) * 100)
    res["points"] = [{"pt": l["pt"], "mass_g": l["mass_g"], "sum_g": float(s[i] / G * 1000),
                      "drift_g": None if l["drift"] is None else l["drift"] / G * 1000,
                      "noise_g": float(np.sqrt(np.sum(np.square(l["s"]))) / G * 1000)}
                     for i, l in enumerate(L)]
    if len(L) < 6:
        return res
    ph = fit_physical(D, T)                             # diagnostic: is it 4 point supports?
    # Corner map from an UNCONSTRAINED fit: the signs of each channel's Tx/Ty
    # coefficients are unambiguous.  The twist then is plain + - - + across corners
    # (the per-cell gains are already within a few % of each other).
    A0, *_ = np.linalg.lstsq(D, T, rcond=None)
    A0 = A0.T
    corners = "".join(("A" if A0[1, k] < 0 else "B") if A0[2, k] > 0 else ("C" if A0[1, k] < 0 else "D")
                      for k in range(4))
    if sorted(corners) != list("ABCD"):
        corners = ph["corners"]
    u = np.array([TWIST[c] for c in corners])
    n = len(L)
    # leave-one-out prediction errors for both models: the honest numbers
    loo_g, loo_p = np.zeros((n, 3)), np.zeros((n, 3))
    for i in range(n):
        k = np.arange(n) != i
        loo_g[i] = solve_general(D[k], T[k], u) @ D[i]
        loo_p[i] = phys_A(phys_w(D[k], T[k], ph["corners"]), ph["corners"]) @ D[i]
    big = Fz >= 0.1 * G                                 # CoP error only means much at >=100 g

    def score(lo):
        ex, ey = lo[:, 1] / lo[:, 0] - x, lo[:, 2] / lo[:, 0] - y
        return {"fz_rms_g": float(np.sqrt(np.mean(((lo[:, 0] - Fz) / G * 1000) ** 2))),
                "cop_rms_mm": float(np.sqrt(np.mean(ex[big] ** 2 + ey[big] ** 2))) if big.any() else None}
    sg, sp = score(loo_g), score(loo_p)
    use_general = sg["cop_rms_mm"] is not None and sg["cop_rms_mm"] < 0.8 * sp["cop_rms_mm"]
    A = solve_general(D, T, u) if use_general else phys_A(np.array(ph["gain_w"]), ph["corners"])
    loo = loo_g if use_general else loo_p
    P = D @ A.T
    res["phys"] = ph
    res["models"] = {"general": sg, "physical": sp, "used": "general" if use_general else "physical"}
    for i, p in enumerate(res["points"]):
        p["fit_g"] = float(P[i, 0] / G * 1000)
        p["loo_err_g"] = float((loo[i, 0] - Fz[i]) / G * 1000)
        p["loo_cop_x"] = float(loo[i, 1] / loo[i, 0]) if loo[i, 0] else None
        p["loo_cop_y"] = float(loo[i, 2] / loo[i, 0]) if loo[i, 0] else None
        p["loo_err_x"] = None if p["loo_cop_x"] is None else p["loo_cop_x"] - x[i]
        p["loo_err_y"] = None if p["loo_cop_y"] is None else p["loo_cop_y"] - y[i]
    ex = np.array([p["loo_err_x"] for p in res["points"]], float)[big]
    ey = np.array([p["loo_err_y"] for p in res["points"]], float)[big]
    rel = (loo[:, 0] - Fz) / Fz
    res.update({
        "A": A.tolist(),
        "chan_pos_mm": [[float(A[1, k] / A[0, k]), float(A[2, k] / A[0, k])] for k in range(4)],
        "chan_corner": "".join(("A" if A[1, k] < 0 else "B") if A[2, k] > 0 else ("C" if A[1, k] < 0 else "D")
                               for k in range(4)),
        "chan_gain": [float(A[0, k]) for k in range(4)],
        "twist_leak_g_per_N": float(A[0] @ (u / np.linalg.norm(u) * 2)) / G * 1000,
        "loo_fz_rms_g": float(np.sqrt(np.mean(((loo[:, 0] - Fz) / G * 1000) ** 2))),
        "loo_fz_rms_pct_ge100g": float(np.sqrt(np.mean(rel[big] ** 2)) * 100) if big.any() else None,
        "loo_cop_rms_x_mm": float(np.sqrt(np.mean(ex ** 2))) if ex.size else None,
        "loo_cop_rms_y_mm": float(np.sqrt(np.mean(ey ** 2))) if ey.size else None,
        "cond": float(np.linalg.cond(D)),
    })
    return res


def save_session():
    with open(session_path, "w") as f:
        json.dump({"plan": PLAN, "caps": caps}, f)


def finish():
    r = fit()
    if "A" not in r:
        return None, r
    doc = {"_comment": ("[Fz N, Tx N*mm, Ty N*mm] = A @ (f - f_tare), f = platelog per-cell N "
                        "(f0..f3 in the feed). Plate frame: origin centre, x along the 200 mm "
                        "length (template right), y across (template up). CoP = (Tx/Fz, Ty/Fz)."),
           "date": time.strftime("%Y-%m-%d %H:%M"), "g": G, "A": r["A"],
           "chan_corner": r["chan_corner"], "chan_pos_mm": r["chan_pos_mm"],
           "chan_gain": r["chan_gain"], "physical_fit": r["phys"], "models": r["models"],
           "loo_fz_rms_g": r["loo_fz_rms_g"], "loo_fz_rms_pct_ge100g": r["loo_fz_rms_pct_ge100g"],
           "loo_cop_rms_x_mm": r["loo_cop_rms_x_mm"], "loo_cop_rms_y_mm": r["loo_cop_rms_y_mm"],
           "sum_gain_err_pct": r["sum_gain_err_pct"], "n": r["n"], "session": session_path}
    with open(os.path.join(HERE, "plate_matrix.json"), "w") as f:
        json.dump(doc, f, indent=1)
    return os.path.join(HERE, "plate_matrix.json"), r


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, (bytes, str)):
            body = json.dumps(body)
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _state(self):
        with lock:
            k, need_empty = plan_pos()
            nxt = None if k >= len(PLAN) else {"pt": PLAN[k][0], "mass_g": PLAN[k][1],
                                                "x": PTS[PLAN[k][0]][0], "y": PTS[PLAN[k][0]][1]}
            return {"step": k, "of": len(PLAN), "need_empty": need_empty, "next": nxt,
                    "ncaps": len(caps), "fit": fit(), "session": session_path}

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "platecal.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if self.path == "/template.pdf":
            with open(os.path.join(HERE, "template.pdf"), "rb") as f:
                return self._send(200, f.read(), "application/pdf")
        if self.path == "/points.json":
            with open(os.path.join(HERE, "points.json"), "rb") as f:
                return self._send(200, f.read())
        if self.path == "/state":
            return self._send(200, self._state())
        if self.path == "/live":
            try:
                return self._send(200, cells(0.5))
            except Exception as e:                       # noqa: BLE001
                return self._send(200, {"n": 0, "err": f"dash.py not reachable: {e}"})
        self._send(404, {"err": "not found"})

    def do_POST(self):
        ln = int(self.headers.get("Content-Length") or 0)
        req = json.loads(self.rfile.read(ln) or b"{}")
        if self.path == "/capture":
            with lock:
                k, need_empty = plan_pos()
            kind = "empty" if need_empty else "load"
            if kind == "load" and k >= len(PLAN) and "pt" not in req:
                return self._send(409, {"err": "plan finished -- press Finish"})
            time.sleep(SETTLE_S + AVG_S)                 # hands off, settle, then average
            try:
                c = cells(AVG_S)
            except Exception as e:                       # noqa: BLE001
                return self._send(409, {"err": f"dash.py not reachable: {e}"})
            if c.get("n", 0) < 20:
                return self._send(409, {"err": "no plate data from dash.py"})
            rec = {"kind": kind, "f": [c[f"f{i}"] for i in range(4)],
                   "s": [c[f"s{i}"] for i in range(4)], "t": time.time(), "nsamp": c["n"]}
            if kind == "load":
                pt = int(req.get("pt") or PLAN[k][0])
                rec.update(pt=pt, x=PTS[pt][0], y=PTS[pt][1],
                           mass_g=float(req.get("mass_g") or PLAN[k][1]))
            if kind == "load":
                with lock:
                    pre = next((c for c in reversed(caps) if c["kind"] == "empty"), None)
                if pre is not None:
                    got = (sum(rec["f"]) - sum(pre["f"])) / G * 1000
                    if got < 0.3 * rec["mass_g"]:
                        return self._send(409, {"err": f"only {got:.1f} g arrived, expected "
                                                       f"{rec['mass_g']:g} g -- nothing on point "
                                                       f"{rec['pt']}? Not recorded."})
            with lock:
                caps.append(rec)
                save_session()
            return self._send(200, self._state())
        if self.path == "/undo":
            with lock:
                if caps:
                    caps.pop()
                save_session()
            return self._send(200, self._state())
        if self.path == "/finish":
            with lock:
                path, r = finish()
            return self._send(200, {"path": path, "fit": r})
        self._send(404, {"err": "not found"})


if __name__ == "__main__":
    print(f"plate calibration on http://localhost:{PORT}   session -> {session_path}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
