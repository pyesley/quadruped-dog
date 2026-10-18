#!/usr/bin/env python3
"""String-pot calibration: ruler height y (mm) vs encoder count, served on :8090.

    python3 stringcal/stringcal.py          then open http://localhost:8090

The Pi streams the raw count (ssh -> ~/scale/enccount.py).  You move the cart,
read the ruler, type y, press Enter; the app stores (count, y).  Repeat; the fit
updates live.  "Finish" writes stringcal/cal-<ts>.json.

The model.  The string leaves the pot at an angle, so with N the string payout
in counts, y^2 = alpha*N^2 + k.  The encoder is incremental, so we only see
n = N - N0 (count relative to when the reader started) and N0 is a third
unknown:

    y^2 = alpha*(n + N0)^2 + k  =  a*n^2 + b*n + e         (linear in a, b, e)
    alpha = a,  N0 = b/(2a),  k = e - b^2/(4a)

Fit: weighted linear least squares on y^2 (weight 1/y^2, because a ruler error
dy gives a y^2 error of 2*y*dy) as the start, then Gauss-Newton on the residual
in y itself, which is where the ruler error actually lives.  n is scaled by
1000 internally for conditioning.

Diagnostic model B adds a ruler offset c:  y = c + sqrt(a n^2 + b n + e).  The
user's equation assumes the ruler's zero is level with the pot's exit point; if
c comes out significant (|c| >> its std error), that assumption is wrong.
A plain straight line is also fitted, to show whether the curvature is even
resolvable over the travel.
"""
import csv, json, math, os, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PORT = 8090
PI = "mdyesley@myesley"
READER = "python3 -u ~/stringpot/stringpot.py --raw"
SCALE = 1000.0                                   # n is fitted as n/SCALE

lock = threading.Lock()
st = {"count": None, "t": 0.0, "hist": [], "reader_gen": 0, "reader_err": "",
      "session_gen": None}
points = []                                      # dicts: n, y, spread, t
ts0 = time.strftime("%Y%m%d-%H%M%S")
session_csv = os.path.join(HERE, f"session-{ts0}.csv")


# ------------------------------------------------------------------ Pi reader
def reader_loop():
    while True:
        try:
            p = subprocess.Popen(["ssh", "-o", "ServerAliveInterval=2", "-o", "ConnectTimeout=5",
                                  PI, READER], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, bufsize=1)
            with lock:
                st["reader_gen"] += 1
                st["count"], st["hist"] = None, []
            for line in p.stdout:
                if line.startswith("#"):
                    continue
                t_pi, c = line.split()
                now, c = time.time(), int(c)
                with lock:
                    st["count"], st["t"] = c, now
                    st["hist"].append((now, c))
                    st["hist"] = [h for h in st["hist"] if now - h[0] < 2.0]
                    st["reader_err"] = ""
            err = p.stderr.read().strip()
        except Exception as e:                   # noqa: BLE001
            err = str(e)
        with lock:
            st["count"] = None
            st["reader_err"] = err or "reader exited"
        time.sleep(1.0)


# ---------------------------------------------------------------------- fits
def _gn(f_jac, p, y, iters=50):
    """Gauss-Newton with step halving. Returns p, residuals, covariance."""
    r, J = f_jac(p)
    cost = r @ r
    for _ in range(iters):
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        lam = 1.0
        while lam > 1e-6:
            pn = p + lam * step
            rn, Jn = f_jac(pn)
            if np.all(np.isfinite(rn)) and rn @ rn <= cost:
                break
            lam /= 2
        else:
            break
        done = abs(cost - rn @ rn) < 1e-14 * (1 + cost)
        p, r, J, cost = pn, rn, Jn, rn @ rn
        if done:
            break
    dof = len(y) - len(p)
    s2 = cost / dof if dof > 0 else float("nan")
    try:
        cov = np.linalg.inv(J.T @ J) * s2
    except np.linalg.LinAlgError:
        cov = np.full((len(p), len(p)), np.nan)
    return p, r, cov


def fit(pts):
    n = np.array([p["n"] for p in pts], float)
    y = np.array([p["y"] for p in pts], float)
    out = {"npts": len(pts)}
    if len(pts) < 2:
        return out
    x = n / SCALE

    # straight line (reference)
    A = np.c_[x, np.ones_like(x)]
    q, *_ = np.linalg.lstsq(A, y, rcond=None)
    rl = y - A @ q
    out["line"] = {"mm_per_count": q[0] / SCALE, "y0_mm": q[1],
                   "rms_mm": float(np.sqrt(np.mean(rl ** 2))),
                   "resid": rl.tolist()}
    if len(pts) < 3:
        return out

    # model A: y = sqrt(a x^2 + b x + e)
    W = 1.0 / y                                       # sqrt of weight 1/y^2
    A = np.c_[x ** 2, x, np.ones_like(x)]
    p0, *_ = np.linalg.lstsq(A * W[:, None], y ** 2 * W, rcond=None)

    def fA(p):
        s = p[0] * x ** 2 + p[1] * x + p[2]
        m = np.sqrt(np.maximum(s, 1e-12))
        J = np.c_[x ** 2, x, np.ones_like(x)] / (2 * m)[:, None]
        return m - y, J

    pA, rA, cA = _gn(fA, p0, y)
    a, b, e = pA
    a_c, b_c = a / SCALE ** 2, b / SCALE            # back to raw counts
    resA = {"a": a_c, "b": b_c, "e": e, "rms_mm": float(np.sqrt(np.mean(rA ** 2))),
            "max_mm": float(np.max(np.abs(rA))), "resid": (-rA).tolist(),
            "dof": len(y) - 3}
    if a != 0:
        resA["alpha"] = a_c                           # mm^2 / count^2
        resA["mm_per_count"] = math.sqrt(abs(a_c))    # string payout per count
        resA["N0"] = b_c / (2 * a_c)                  # counts
        resA["k"] = e - b_c ** 2 / (4 * a_c)          # mm^2
        # std errors of alpha, N0, k via the Jacobian of (a,b,e) -> (alpha,N0,k)
        G = np.array([[1, 0, 0],
                      [-b / (2 * a * a), 1 / (2 * a), 0],
                      [b * b / (4 * a * a), -b / (2 * a), 1]])
        C = G @ cA @ G.T
        sd = np.sqrt(np.abs(np.diag(C)))
        resA["alpha_sd"] = sd[0] / SCALE ** 2
        resA["N0_sd"] = sd[1] * SCALE
        resA["k_sd"] = sd[2]
        # implied geometry when k < 0: y^2 = L^2 - d^2, d = horizontal offset
        if resA["k"] < 0:
            resA["offset_mm"] = math.sqrt(-resA["k"])
    out["A"] = resA

    # model B: y = c + sqrt(a x^2 + b x + e)   (ruler zero not at the pot)
    if len(pts) >= 5:
        def fB(p):
            s = p[0] * x ** 2 + p[1] * x + p[2]
            m = np.sqrt(np.maximum(s, 1e-12))
            J = np.c_[x ** 2 / (2 * m), x / (2 * m), 1 / (2 * m), np.ones_like(x)]
            return p[3] + m - y, J
        pB, rB, cB = _gn(fB, np.r_[pA, 0.0], y)
        out["B"] = {"c_mm": float(pB[3]), "c_sd": float(math.sqrt(abs(cB[3, 3]))),
                    "rms_mm": float(np.sqrt(np.mean(rB ** 2))), "dof": len(y) - 4}
    return out


def safe_fit(pts):
    try:
        return fit(pts)
    except Exception as e:                       # noqa: BLE001
        return {"npts": len(pts), "err": str(e)}


def predict(res, n):
    s = res["a"] * n ** 2 + res["b"] * n + res["e"]
    return math.sqrt(s) if s > 0 else float("nan")


# ------------------------------------------------------------------- storage
def save_csv():
    with open(session_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "count", "y_mm", "spread"])
        for p in points:
            w.writerow([f"{p['t']:.3f}", p["n"], p["y"], p["spread"]])


def finish():
    res = fit(points)
    path = os.path.join(HERE, f"cal-{time.strftime('%Y%m%d-%H%M%S')}.json")
    A = res.get("A", {})
    doc = {
        "_comment": ("y_mm = sqrt(alpha*(n + N0)^2 + k), n = string-pot count relative to the "
                     "reader start of THIS session. alpha and k are the calibration; N0 is "
                     "session-specific -- re-establish it with one ruler reading: "
                     "N0 = +/-sqrt((y^2 - k)/alpha) - n, sign matching this file."),
        "alpha_mm2_per_count2": A.get("alpha"), "k_mm2": A.get("k"), "N0_counts": A.get("N0"),
        "mm_per_count": A.get("mm_per_count"), "rms_mm": A.get("rms_mm"),
        "fit": res, "points": points, "session_csv": session_csv,
        "encoder": "kernel rotary-encoder, GPIO5/6, steps-per-period=4 (every quadrature edge)",
    }
    with open(path, "w") as f:
        json.dump(doc, f, indent=1)
    return path, res


# ---------------------------------------------------------------------- http
class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if not isinstance(body, (bytes, str)):
            body = json.dumps(body, default=lambda o: None if isinstance(o, float) and not math.isfinite(o) else o)
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
            hist = [c for _, c in st["hist"] if time.time() - _ < 0.5]
            live = st["count"] is not None and time.time() - st["t"] < 2.0
            stale = st["session_gen"] is not None and st["session_gen"] != st["reader_gen"]
            return {"count": st["count"] if live else None,
                    "moving": bool(hist) and max(hist) != min(hist),
                    "reader_err": st["reader_err"], "stale": stale,
                    "points": points, "fit": safe_fit(points)}

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "stringcal.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if self.path == "/state":
            return self._send(200, self._state())
        self._send(404, {"err": "not found"})

    def do_POST(self):
        ln = int(self.headers.get("Content-Length") or 0)
        req = json.loads(self.rfile.read(ln) or b"{}")
        if self.path == "/point":
            with lock:
                if st["count"] is None or time.time() - st["t"] > 2.0:
                    return self._send(409, {"err": "no live encoder count"})
                if st["session_gen"] is None:
                    st["session_gen"] = st["reader_gen"]
                if st["session_gen"] != st["reader_gen"]:
                    return self._send(409, {"err": "encoder reader restarted: count reference "
                                                   "lost. Click 'New session'."})
                cs = [c for t, c in st["hist"] if time.time() - t < 0.5] or [st["count"]]
                points.append({"n": int(np.median(cs)), "y": float(req["y"]),
                               "spread": max(cs) - min(cs), "t": time.time()})
                save_csv()
            return self._send(200, self._state())
        if self.path == "/delete":
            with lock:
                i = int(req["i"])
                if 0 <= i < len(points):
                    points.pop(i)
                save_csv()
            return self._send(200, self._state())
        if self.path == "/new":
            global session_csv
            with lock:
                points.clear()
                st["session_gen"] = None
                session_csv = os.path.join(HERE, f"session-{time.strftime('%Y%m%d-%H%M%S')}.csv")
            return self._send(200, self._state())
        if self.path == "/finish":
            with lock:
                path, res = finish()
            return self._send(200, {"path": path, "fit": res})
        self._send(404, {"err": "not found"})


if __name__ == "__main__":
    threading.Thread(target=reader_loop, daemon=True).start()
    print(f"string-pot calibration on http://localhost:{PORT}   session -> {session_csv}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()
