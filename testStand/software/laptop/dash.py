#!/usr/bin/env python3
"""Live test-stand scope on the laptop: force plate, string pot, leg encoders,
motor currents -- with a rolling mode and a triggered one-shot mode.

    python3 dash.py            # starts platelog on the Pi, opens http://localhost:8080/
    python3 step.py --telemetry 127.0.0.1:9871      (in another terminal)

Data flow
---------
  Pi  platelog  --udp <this laptop>:9870   force / string-pot feed (text datagrams)
  laptop step.py --telemetry :9871         per-cycle motor state (json)
  this server   :8080                      merges both onto the laptop clock and
                                           streams to the browser (SSE)

Two display modes, like a scope
-------------------------------
CONTINUOUS  platelog sends a ~100 Hz decimated feed (each row carries the min/max
            of its bin, so a spike between rows is still visible) and the browser
            scrolls it.  Cheap enough to leave running for hours.

ONE-SHOT    the server asks platelog for *every* sample ("DIV 1" -> 1 kSPS,
            packed ~15 to a datagram), keeps a ring buffer, and on a trigger
            freezes a pre/post window around it.  That is the only way to see a
            hop: the interesting part of the force trace is ~10 ms wide, which a
            100 Hz feed cannot resolve.  Each shot is also written to
            logs/shot-<ts>.csv.  Triggers: manual, Fz rising/falling through a
            level, or step.py starting a move.

Time base
---------
platelog stamps each sample with its own monotonic clock; motor telemetry is
stamped with the laptop clock.  Those are tied together by tracking the *minimum*
observed (arrival - plate clock) over the last few seconds -- the least delayed
datagram is the one with no WiFi queueing in it -- and adding that offset to
every plate sample.  Each sample then gets a smooth per-sample laptop time
instead of the arrival time it shares with the other 14 samples in its datagram.
The residual plate-vs-motor alignment error is the jitter of that floor, a few ms
at worst; `/status` reports it as plate_off.

Commands to the plate (record / tare / zero-z / DIV) go back to platelog over the
same UDP socket its feed arrives on, so nothing else has to run on the Pi.
"""
import argparse
import json
import math
import os
import socket
import subprocess
import threading
import time
from collections import deque

from step import FAULT_NAMES, LIMIT_CODES
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "logs")
CFG_PATH = os.path.join(HERE, "leg_config.json")
SP_CAL_PATH = os.path.join(HERE, "stringcal", "stringpot_cal.json")
PM_PATH = os.path.join(HERE, "platecal", "plate_matrix.json")
HOP_CTL = 9872                      # hop.py --control-port
PI = "mdyesley@myesley"
PI_HOST = "myesley"
PI_SCALE = "~/scale"
PORT, PLATE_UDP, MOTOR_UDP = 8080, 9870, 9871

# 45 s of plate at 1 kSPS is plenty of headroom for a 1 s shot, and the deques
# are the only thing here that grows with the sample rate.
PLATE_KEEP = 45000
MOTOR_KEEP = 20000
SHOTS_KEEP = 24                  # completed shots held in memory for review

lock = threading.Lock()
plate = deque(maxlen=PLATE_KEEP)     # (seq, dict)
motors = deque(maxlen=MOTOR_KEEP)
seq = {"plate": 0, "motor": 0}
st = {"plate_last": 0.0, "motor_last": 0.0, "plate_hz": 0.0, "motor_hz": 0.0,
      "plate_peer": None, "plate_rec": 0, "gaps": 0, "recording": None,
      "last_fetch": None, "motor_state": "", "remote": "",
      "mode": "cont", "want_div": 0, "plate_div": None, "plate_sps": None,
      "plate_off": None, "plate_off_spread": None,
      "last_i": None, "udp_lost": 0}
shot = {"state": "idle", "id": 0, "why": "", "ready_id": None,
        "arm": None, "t_trig": None, "t_trig_wall": None, "t_end_wall": None}
shots = deque(maxlen=SHOTS_KEEP)     # completed shot dicts, newest last
cfg = {}
merged_fh = None
plate_sock = None

# 14 plate columns, then 10 per motor, then the two commands and the state.
# The currents / bus / temperature / mode / fault columns are the whole point of
# a long run: without them a thermal or over-voltage trip leaves no evidence.
_MCOLS = ("pos", "vel", "tq", "q", "d", "pw", "volt", "temp", "mode", "flt")
MERGED_HDR = ("t_arrive,src,t_src,t_send,fz_N,z_m,enc,f0_N,f1_N,f2_N,f3_N,fz_lo,fz_hi,plate_i,"
              + ",".join(f"m{m}_{c}" for m in (1, 2) for c in _MCOLS)
              + ",m1_cmd,m2_cmd,state\n")
_N_TRAIL = 2 * len(_MCOLS) + 3          # motor + cmd + state columns

SHOT_HDR = ("t_ms,src,fz_N,z_m,f0_N,f1_N,f2_N,f3_N,"
            "m1_pos,m1_vel,m1_tq,m1_q_A,m1_d_A,m1_pw_W,"
            "m2_pos,m2_vel,m2_tq,m2_q_A,m2_d_A,m2_pw_W,m1_cmd,m2_cmd,state\n")

TRIGGERS = ("manual", "fz_rise", "fz_fall", "fz_abs", "motor_move", "vel")

# Below this resting load, "did it leave the plate?" is not a question worth
# answering -- the plate is empty and the answer is noise.
FLIGHT_MIN_BASE_N = 1.5          # the tared leg+cart rests at ~2-5 N


def wall():
    return time.time()


# ---------------------------------------------------------------- string pot
# y_mm = sqrt(alpha*(enc + N0)^2 + k) (stringcal/stringpot_cal.json, 2026-10-03).
# platelog's enc counts from its own start, so N0 is only right after "Cart is
# ... mm" pins it; a platelog restart (sample index going backwards) unpins it.
# Until then the calibration session's N0 is used and the tile says so.
with open(SP_CAL_PATH) as _f:
    _c = json.load(_f)
sp = {"alpha": _c["alpha_mm2_per_count2"], "k": _c["k_mm2"], "N0_cal": _c["N0_counts"],
      "N0": _c["N0_counts"], "ref": False, "enc": 0, "last_i": None}


# ------------------------------------------------------- centre of pressure
# platecal/plate_matrix.json (precision weights, 2026-10-03): [Fz, Tx, Ty] = A @ f, f = the
# four per-cell forces platelog sends (N, tared).  Plate frame: origin = plate centre,
# x along the 200 mm length, y across, mm.  A ignores the plate's self-balanced + - - +
# "twist", so re-seating does not move the CoP.  Below COP_MIN_N there is nothing to locate.
COP_MIN_N = 0.5
try:
    with open(PM_PATH) as _f:
        PM = json.load(_f)
    PM_A = PM["A"]
except (OSError, ValueError, KeyError):
    PM, PM_A = None, None


# The matrix maps CHANGES from an empty plate.  platelog's tare zeros (plate_tare.json)
# were older than the calibration and left the empty plate reading a phantom 250 N*mm
# torque -- 90 mm of CoP error under the foot's ~3 N.  So subtract an empty-plate
# reading: cfg plate.cop_zero, else the calibration's own last empty.  "Tare plate"
# re-zeros platelog and resets cop_zero to 0.
def cop_zero():
    z = (cfg.get("plate") or {}).get("cop_zero")
    if z is None and PM is not None:
        z = PM.get("f_empty_last")
    return z or [0.0, 0.0, 0.0, 0.0]


def plate_cop(f):
    """-> (Fz from the matrix, CoP x mm, CoP y mm); CoP None when unloaded."""
    if PM_A is None:
        return None, None, None
    z = cop_zero()
    df = [f[k] - z[k] for k in range(4)]
    fz, tx, ty = (sum(PM_A[r][k] * df[k] for k in range(4)) for r in range(3))
    if fz < COP_MIN_N:
        return fz, None, None
    return fz, tx / fz, ty / fz


def sp_y_mm(enc):
    return math.sqrt(max(sp["alpha"] * (enc + sp["N0"]) ** 2 + sp["k"], 0.0))


def sp_set_cart(mm):
    """The cart is at `mm` on the ruler right now: solve N0 for the current count."""
    n_abs = math.sqrt(max(mm * mm - sp["k"], 0.0) / sp["alpha"])
    sp["N0"] = math.copysign(n_abs, sp["N0_cal"]) - sp["enc"]
    sp["ref"] = True


# ------------------------------------------------------------------- config
DEFAULT_CFG = {
    "_comment": ("Dashboard config. Rewritten by the Zero-angles button and the "
                 "one-shot controls; hand-edit gear_ratio / label / sign here."),
    "joints": [{"id": 1, "label": "joint 1", "gear_ratio": 1.0, "zero_rev": 0.0, "sign": 1},
               {"id": 2, "label": "joint 2", "gear_ratio": 1.0, "zero_rev": 0.0, "sign": 1}],
    "limits": {"max_current_A": 9.0, "warn_current_A": 6.0, "max_voltage_V": 25.0,
               "flux_brake_margin_V": 3.0, "min_voltage_V": 18.0,
               "fault_temperature_C": 75.0, "warn_temperature_C": 60.0,
               "max_velocity_rps": 500.0},
    "plate": {"nominal_sps": 1000},
    "shot": {"pre_ms": 200, "post_ms": 1500, "src": "fz_rise", "level": 5.0},
}


def load_cfg():
    """leg_config.json, with every missing key filled from DEFAULT_CFG."""
    global cfg
    out = json.loads(json.dumps(DEFAULT_CFG))
    try:
        with open(CFG_PATH) as f:
            user = json.load(f)
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(out.get(k), dict):
                out[k].update(v)
            else:
                out[k] = v
    except (OSError, ValueError) as e:
        print(f"[cfg] {CFG_PATH}: {e} -- using defaults", flush=True)
    cfg = out
    return cfg


def save_cfg():
    tmp = CFG_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")
    os.replace(tmp, CFG_PATH)


def joint(mid):
    for j in cfg["joints"]:
        if int(j["id"]) == int(mid):
            return j
    return {"id": mid, "label": f"joint {mid}", "gear_ratio": 1.0, "zero_rev": 0.0, "sign": 1}


def wrap180(a):
    """Fold an angle into [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a):
    """Fold an angle into [-180, 180); the leg's included angle lives in 0..180
    and its sign is just which way the knee is labelled."""
    return (a + 180.0) % 360.0 - 180.0


def joint_deg(mid, pos):
    """Motor revolutions -> physical joint degrees, through the 9:1 belt."""
    if pos is None:
        return None
    j = joint(mid)
    return ((pos - j.get("zero_rev", 0.0)) / (j.get("gear_ratio") or 1.0)
            * 360.0 * (j.get("sign", 1) or 1) + j.get("offset_deg", 0.0))


def leg_geom(pos_by_id):
    """Leg length (hip to foot) and foot position from the two joint angles, for
    two equal segments.  `pos_by_id` maps controller id -> position in rev.

    Which controller drives which joint is NOT assumed: cfg["joints"][0] is the
    root joint (hip) and [1] the distal one (knee), whatever ids they carry.
    `knee_relative` picks how the lower link is referenced: with the ODRI belt
    the lower link angle is absolute, so the *relative* knee angle is
    q_distal - q_root; a plain serial knee would just be q_distal.  Either way
    the leg length is 2*L*cos(knee/2)."""
    lg = cfg.get("leg") or {}
    if not lg.get("enabled", True) or len(cfg.get("joints", [])) < 2:
        return None
    j0, j1 = cfg["joints"][0], cfg["joints"][1]
    q_hip = joint_deg(j0["id"], pos_by_id.get(int(j0["id"])))
    q_knee = joint_deg(j1["id"], pos_by_id.get(int(j1["id"])))
    if q_hip is None or q_knee is None:
        return None
    L = lg.get("segment_mm", 160.0)
    # Angle convention, set by where you press "Zero angles": both read 0 with
    # the leg FOLDED (the pose it collapses into at power-off), and grow as it
    # opens, reaching hip 90 / knee 180 fully extended.  The knee reading is
    # then the included angle between the two links, so
    #     leg = 2 L sin(knee/2)      knee 0 -> 0 mm folded, 180 -> 2L straight
    # knee_offset_deg absorbs the fact that the links cannot fold quite flat.
    # foot_offset_mm is the cylinder on the end of the foot: the leg pivots stop
    # that far above the ground, so this is what makes len_mm the HIP HEIGHT
    # above the floor rather than just the hip-to-pivot distance.
    inc = abs(wrap360(q_knee + lg.get("knee_offset_deg", 0.0)))
    return {"q1": q_hip, "q2": q_knee, "knee": inc,
            "len_mm": abs(2.0 * L * math.sin(math.radians(inc) / 2.0))
                      + lg.get("foot_offset_mm", 0.0)}


def merged_write(row):
    if merged_fh:
        merged_fh.write(row)


# ------------------------------------------------------------ remote (Pi)
def my_ip_toward(host):
    try:
        ip = socket.gethostbyname(host)
        out = subprocess.run(["ip", "route", "get", ip], capture_output=True, text=True).stdout
        return out.split("src")[1].split()[0]
    except Exception:  # noqa: BLE001
        return None


def remote_start_platelog():
    ip = my_ip_toward(PI_HOST)
    if not ip:
        st["remote"] = f"cannot resolve {PI_HOST}"
        return
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", PI,
                        f"cd {PI_SCALE} && ./plate_start.sh {ip}"], capture_output=True, text=True)
    both = (r.stdout + r.stderr).strip()
    st["remote"] = both.splitlines()[-1] if both else f"rc {r.returncode}"
    print("[pi]", both, flush=True)


def fetch_latest_plate_file():
    """After a recording, pull the Pi's full-rate file next to our merged one."""
    time.sleep(2.0)                                   # let platelog close it
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", PI, f"ls -t {PI_SCALE}/logs/plate-*.csv | head -1"],
                       capture_output=True, text=True)
    remote = r.stdout.strip()
    if not remote:
        st["last_fetch"] = "no plate file on Pi"
        return
    name = os.path.basename(remote)
    r = subprocess.run(["scp", "-q", f"{PI}:{remote}", os.path.join(LOGS, name)], capture_output=True, text=True)
    st["last_fetch"] = name if r.returncode == 0 else f"scp failed: {r.stderr.strip()}"
    print("[fetch]", st["last_fetch"], flush=True)


def plate_cmd(cmd):
    peer = st["plate_peer"]
    if not peer or not plate_sock:
        return False
    try:
        plate_sock.sendto(cmd.encode(), peer)
    except OSError:
        return False
    return True


# --------------------------------------------------------------- recording
def start_recording():
    global merged_fh
    with lock:
        if st["recording"]:
            return
        os.makedirs(LOGS, exist_ok=True)
        path = os.path.join(LOGS, time.strftime("merged-%Y%m%d-%H%M%S.csv"))
        merged_fh = open(path, "w", buffering=1 << 16)
        merged_fh.write(f"# dash merged log, start {time.strftime('%Y-%m-%d %H:%M:%S')}  (t_* are unix seconds)\n")
        merged_fh.write(MERGED_HDR)
        st["recording"] = {"path": path, "t0": wall()}
    plate_cmd("REC")


def stop_recording():
    global merged_fh
    plate_cmd("REC")
    with lock:
        st["recording"] = None
        if merged_fh:
            merged_fh.close()
            merged_fh = None
    threading.Thread(target=fetch_latest_plate_file, daemon=True).start()


# ----------------------------------------------------------- UDP listeners
class _ClockOffset:
    """Minimum of (arrival - plate clock) over a sliding ~6 s window, in O(1) per
    sample: one running minimum for the current 1 s bucket plus the last six
    bucket minima.  The minimum is the best estimate of the one-way delay with no
    queueing in it; the spread between buckets is how much jitter there is."""

    def __init__(self):
        self.buckets = deque(maxlen=6)
        self.cur = math.inf
        self.t0 = None

    def add(self, tw, t_src):
        d = tw - t_src
        if self.t0 is None:
            self.t0 = tw
        if d < self.cur:
            self.cur = d
        if tw - self.t0 >= 1.0:
            self.buckets.append(self.cur)
            self.cur = math.inf
            self.t0 = tw
        return self.value(d)

    def value(self, fallback):
        vals = [v for v in list(self.buckets) + [self.cur] if math.isfinite(v)]
        return min(vals) if vals else fallback

    def spread(self):
        vals = [v for v in self.buckets if math.isfinite(v)]
        return (max(vals) - min(vals)) if len(vals) > 1 else None


def plate_listener():
    global plate_sock
    plate_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    plate_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    plate_sock.bind(("0.0.0.0", PLATE_UDP))
    off = _ClockOffset()
    n, t_rate = 0, wall()
    while True:
        data, addr = plate_sock.recvfrom(65535)     # batched: up to ~15 rows
        tw = wall()
        st["plate_peer"] = addr
        rows = []
        for line in data.decode(errors="ignore").splitlines():
            if line.startswith("PONG"):
                p = line.split(",")
                if len(p) >= 6:
                    if int(p[1]) != st["plate_div"]:
                        # decimation changed: `i` now steps by a different amount,
                        # and the loss count starts fresh for this capture session
                        st["plate_div"] = int(p[1])
                        st["last_i"] = None
                        st["udp_lost"] = 0
                    st["plate_sps"] = float(p[5])
                continue
            p = line.split(",")
            if len(p) < 14 or p[0] != "P":
                continue
            try:
                d = {"tw": tw, "t": float(p[1]), "fz": float(p[2]), "z": float(p[3]),
                     "enc": int(p[4]),
                     "f": [float(p[5]), float(p[6]), float(p[7]), float(p[8])],
                     "lo": float(p[9]), "hi": float(p[10]), "rec": int(p[11]),
                     "i": int(p[12]), "gaps": int(p[13]),
                     # platelog >= 2026-10-03: times it found the ADC's GAIN/CLOCK
                     # registers changed (motor EMI) and rewrote them
                     "rf": int(p[14]) if len(p) > 14 else 0}
            except ValueError:
                continue
            d["tp"] = d["t"] + off.add(tw, d["t"])   # plate sample on the laptop clock
            if sp["last_i"] is not None and d["i"] < sp["last_i"]:   # platelog restarted
                sp["N0"], sp["ref"] = sp["N0_cal"], False
            sp["last_i"], sp["enc"] = d["i"], d["enc"]
            d["z"] = sp_y_mm(d["enc"]) / 1000.0     # calibrated cart height, metres
            d["fzA"], d["cx"], d["cy"] = plate_cop(d["f"])
            rows.append(d)
        if not rows:
            continue
        st["plate_off"] = round(off.value(0.0), 5)
        spr = off.spread()
        st["plate_off_spread"] = round(spr, 5) if spr is not None else None
        with lock:
            for d in rows:
                # platelog numbers every sample it takes, so a hole in `i` is a
                # datagram WiFi dropped -- distinct from platelog's own `gaps`,
                # which counts samples the ADC loop never got.  Only meaningful at
                # full rate: when the feed is decimated `i` legitimately steps by
                # `div` and every row would look like a loss.
                li = st["last_i"]
                if st["plate_div"] == 1 and li is not None and li < d["i"] - 1:
                    st["udp_lost"] += d["i"] - li - 1
                st["last_i"] = d["i"]
                seq["plate"] += 1
                plate.append((seq["plate"], d))
                merged_write(f"{tw:.4f},P,{d['t']:.4f},,{d['fz']:.4f},{d['z']:.5f},{d['enc']},"
                             f"{d['f'][0]:.3f},{d['f'][1]:.3f},{d['f'][2]:.3f},{d['f'][3]:.3f},"
                             f"{d['lo']:.3f},{d['hi']:.3f},{d['i']}"
                             + "," * _N_TRAIL + "\n")
                if shot["state"] == "armed":
                    check_plate_trigger(d)
            last = rows[-1]
            st["plate_last"], st["plate_rec"], st["gaps"] = tw, last["rec"], last["gaps"]
            if last["rf"] != st.get("reg_fixes", 0):
                if last["rf"] > st.get("reg_fixes", 0):
                    st["reg_fix_last"] = tw
                    print(f"[plate] ADC registers were lost and rewritten (#{last['rf']}) -- "
                          f"plate data in the second before {time.strftime('%H:%M:%S')} is suspect",
                          flush=True)
                st["reg_fixes"] = last["rf"]
        n += len(rows)
        if n >= 200:
            now = wall()
            st["plate_hz"] = n / (now - t_rate)
            n, t_rate = 0, now


def motor_listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    s.bind(("0.0.0.0", MOTOR_UDP))
    n, t_rate = 0, wall()
    prev_state = ""
    while True:
        data, _ = s.recvfrom(65535)
        tw = wall()
        try:
            m = json.loads(data.decode())
        except ValueError:
            continue
        m["tw"] = tw
        with lock:
            seq["motor"] += 1
            motors.append((seq["motor"], m))
            st["motor_last"], st["motor_state"] = tw, m.get("state", "")
            if "hop" in m:                      # hop.py's control-panel status
                st["hop"], st["hop_t"] = m["hop"], tw
            mm, cmd = m.get("m", {}), m.get("cmd", {})
            m1, m2 = mm.get("1", {}), mm.get("2", {})

            def g(dct, k):
                v = dct.get(k)
                return "" if v is None else (f"{v:g}" if isinstance(v, int) else f"{v:.5f}")
            cells = ",".join(g(mm_, c) for mm_ in (m1, m2) for c in _MCOLS)
            merged_write(f"{tw:.4f},M,{m.get('t', 0):.4f},{m.get('tw_send', 0):.4f}"
                         + "," * 11 + cells        # 10 plate columns -> 11 separators
                         + f",{g(cmd, '1')},{g(cmd, '2')},{m.get('state', '')}\n")
            if shot["state"] == "armed":
                check_motor_trigger(m, prev_state)
            prev_state = m.get("state", "")
        n += 1
        if n >= 50:
            now = wall()
            st["motor_hz"] = n / (now - t_rate)
            n, t_rate = 0, now


# ------------------------------------------------------------------ one-shot
def set_mode(mode):
    if mode not in ("cont", "shot"):
        return False
    with lock:
        st["mode"] = mode
        st["want_div"] = 1 if mode == "shot" else 0
        if mode == "cont":
            shot["state"] = "idle"
            shot["why"] = ""
    plate_cmd(f"DIV {st['want_div']}")
    return True


def arm(pre_ms, post_ms, src, level):
    if src not in TRIGGERS:
        return False, f"unknown trigger {src}"
    pre_ms = max(10.0, min(5000.0, float(pre_ms)))
    post_ms = max(20.0, min(10000.0, float(post_ms)))
    if st["mode"] != "shot":
        set_mode("shot")
    with lock:
        cfg["shot"].update({"pre_ms": pre_ms, "post_ms": post_ms, "src": src,
                            "level": float(level)})
        shot.update({"state": "armed", "why": f"armed on {src}",
                     "arm": {"pre_ms": pre_ms, "post_ms": post_ms, "src": src,
                             "level": float(level)},
                     "_prev_fz": None,
                     "t_trig": None, "t_trig_wall": None, "t_end_wall": None})
    save_cfg()
    return True, "armed"


def _fire(why, t_plate, t_wall):
    """Called with the lock held, from whichever listener saw the trigger."""
    shot["state"] = "capturing"
    shot["id"] += 1
    shot["why"] = why
    shot["t_trig"] = t_plate
    shot["t_trig_wall"] = t_wall
    shot["t_end_wall"] = t_wall + shot["arm"]["post_ms"] / 1000.0 + 0.15


def check_plate_trigger(d):
    """Lock held.  `d` is the plate sample just ingested."""
    a = shot["arm"]
    src, lvl = a["src"], a["level"]
    fz, prev = d["fz"], shot.get("_prev_fz")
    shot["_prev_fz"] = fz
    hit = False
    if src == "fz_rise":
        hit = prev is not None and prev < lvl <= fz
    elif src == "fz_fall":
        hit = prev is not None and prev > lvl >= fz
    elif src == "fz_abs":
        hit = abs(fz) >= abs(lvl)
    if hit:
        _fire(f"Fz {src} {lvl:g} N", d["t"], d["tp"])


def check_motor_trigger(m, prev_state):
    """Lock held.  `m` is the motor telemetry packet just ingested."""
    a = shot["arm"]
    src, lvl = a["src"], a["level"]
    hit = why = None
    if src == "motor_move":
        state = m.get("state", "")
        if state.startswith("MOVE") and not prev_state.startswith("MOVE"):
            hit, why = True, f"step.py started {state}"
    elif src == "vel":
        for mid, v in (m.get("m") or {}).items():
            vel = v.get("vel")
            if vel is not None and abs(vel) >= abs(lvl):
                hit, why = True, f"motor {mid} |vel| >= {lvl:g} rev/s"
                break
    if hit:
        off = st["plate_off"] or 0.0
        _fire(why, m["tw"] - off, m["tw"])


def manual_trigger():
    with lock:
        if shot["state"] != "armed":
            return False
        tw = wall()
        _fire("manual", tw - (st["plate_off"] or 0.0), tw)
    return True


def _num(v, d=4):
    return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{d}f}"


def shot_stats(prows, mrows, pre_ms):
    """Numbers we actually care about for a hop, from the captured window."""
    s = {}
    if prows:
        fz = [r[1] for r in prows]
        base = [r[1] for r in prows if r[0] <= -0.2 * pre_ms]     # first 80% of pre-trigger
        s["fz_base_N"] = round(sum(base) / len(base), 3) if base else None
        pk = max(range(len(fz)), key=lambda i: fz[i])
        s["fz_peak_N"] = round(fz[pk], 3)
        s["fz_peak_ms"] = round(prows[pk][0], 2)
        s["fz_min_N"] = round(min(fz), 3)
        b = s["fz_base_N"] or 0.0
        imp = 0.0
        for i in range(1, len(prows)):
            dt = (prows[i][0] - prows[i - 1][0]) / 1000.0
            if 0.0 < dt < 0.05:                    # skip across a dropout
                imp += 0.5 * (fz[i] + fz[i - 1] - 2 * b) * dt
        s["impulse_Ns"] = round(imp, 4)
        # Flight = the longest run with essentially no force on the plate, anywhere
        # in the window -- a hop's biggest force is usually the LANDING, after the
        # flight, so searching only after the peak misses it.  Only meaningful if
        # something was standing on the plate first: otherwise an empty plate idling
        # below the threshold reads as one long "flight".
        s["flight_ms"] = 0.0
        if b >= FLIGHT_MIN_BASE_N:
            thresh = max(0.5, 0.10 * b)
            best, best_i0, best_i1, run_i0 = 0.0, None, None, None
            for i in range(len(prows)):
                if prows[i][1] < thresh:
                    if run_i0 is None:
                        run_i0 = i
                    if prows[i][0] - prows[run_i0][0] > best:
                        best, best_i0, best_i1 = prows[i][0] - prows[run_i0][0], run_i0, i
                else:
                    run_i0 = None
            if best > 2:
                s["flight_ms"] = round(best, 2)
                s["flight_start_ms"] = round(prows[best_i0][0], 2)
                tf = best / 1000.0
                # free flight, take-off = landing height: h = g t^2 / 8.  For a DROP
                # (no push-off) this is not a hop height -- the fall is ~ g t^2 / 2.
                s["hop_from_flight_mm"] = round(1000.0 * 9.80665 * tf * tf / 8.0, 1)
                before, after = fz[:best_i0], fz[best_i1 + 1:]
                if before:
                    s["push_peak_N"] = round(max(before), 3)
                if after:
                    s["land_peak_N"] = round(max(after), 3)
        else:
            s["flight_note"] = ("nothing standing on the plate before the shot "
                                f"(resting load < {FLIGHT_MIN_BASE_N:g} N)")
        z = [r[2] for r in prows]
        z0 = next((r[2] for r in prows if r[0] >= 0), z[0])
        s["z_at_trigger_mm"] = round(z0 * 1000, 2)
        s["z_min_mm"] = round(min(z) * 1000, 2)
        s["z_max_mm"] = round(max(z) * 1000, 2)
        s["z_rise_mm"] = round((max(z) - z0) * 1000, 2)
        s["n_plate"] = len(prows)
        span = (prows[-1][0] - prows[0][0]) / 1000.0
        s["plate_sps"] = round((len(prows) - 1) / span) if span > 0 else None
    if mrows:
        s["n_motor"] = len(mrows)
        span = (mrows[-1][0] - mrows[0][0]) / 1000.0
        s["motor_hz"] = round((len(mrows) - 1) / span, 1) if span > 0 else None
        for mid in ("1", "2"):
            vals = [r[1].get(mid, {}) for r in mrows]
            def peak(k):
                xs = [abs(v.get(k)) for v in vals if v.get(k) is not None]
                return round(max(xs), 3) if xs else None
            s[f"m{mid}_peak_vel"] = peak("vel")
            s[f"m{mid}_peak_tq"] = peak("tq")
            s[f"m{mid}_peak_q_A"] = peak("q")
            amps = [math.hypot(v.get("q") or 0.0, v.get("d") or 0.0) for v in vals
                    if v.get("q") is not None]
            s[f"m{mid}_peak_amp_A"] = round(max(amps), 3) if amps else None
            s[f"m{mid}_peak_pw_W"] = peak("pw")
        # How close did regen get to servo.max_voltage?  On a supply that cannot
        # sink current this is the number that decides whether a hop faults.
        volts = [v.get("volt") for r in mrows for v in r[1].values() if v.get("volt")]
        if volts:
            s["bus_max_V"] = round(max(volts), 2)
            s["bus_min_V"] = round(min(volts), 2)
        # Leg extension from the encoders -- the same quantity the string pot
        # measures, so the two are a cross-check on each other.
        lens = []
        at_trig = None
        for r in mrows:
            g = leg_geom({int(k): (v or {}).get("pos") for k, v in (r[1] or {}).items()})
            if g:
                lens.append(g["len_mm"])
                if at_trig is None and r[0] >= 0:
                    at_trig = g["len_mm"]
        if lens:
            s["leg_min_mm"] = round(min(lens), 2)
            s["leg_max_mm"] = round(max(lens), 2)
            s["leg_travel_mm"] = round(max(lens) - min(lens), 2)
            if at_trig is not None:
                s["leg_at_trigger_mm"] = round(at_trig, 2)
    return s


def build_shot():
    """Slice the ring buffers around the trigger, save a CSV, publish the shot."""
    with lock:
        a = dict(shot["arm"])
        sid = shot["id"]
        why = shot["why"]
        t_trig, t_trig_wall = shot["t_trig"], shot["t_trig_wall"]
        pre, post = a["pre_ms"] / 1000.0, a["post_ms"] / 1000.0
        window = [d for _, d in plate if -pre <= d["t"] - t_trig <= post]
        prows = [[round((d["t"] - t_trig) * 1000, 3), d["fz"], d["z"], d["f"]]
                 for d in window]
        div = st["plate_div"] or 1
        lost = (((window[-1]["i"] - window[0]["i"]) // div + 1) - len(window)) if window else 0
        mrows = [[round((m["tw"] - t_trig_wall) * 1000, 3), m.get("m", {}),
                  m.get("cmd", {}), m.get("state", "")]
                 for _, m in motors if -pre <= m["tw"] - t_trig_wall <= post]
        off, spread = st["plate_off"], st["plate_off_spread"]

    stats = shot_stats(prows, mrows, a["pre_ms"])
    stats["plate_lost"] = lost
    # an ADC register repair inside the window (or up to the 1 s check interval after
    # it) means the plate data around it may be wrong
    rf = [d.get("rf", 0) for d in window]
    stats["adc_regfix"] = bool(rf) and (rf[-1] > rf[0] or
                                        (st.get("reg_fix_last") or 0) >= t_trig_wall - pre)
    out = {"id": sid, "why": why, "arm": a, "t_wall": t_trig_wall,
           "when": time.strftime("%H:%M:%S", time.localtime(t_trig_wall)),
           "plate_off": off, "plate_off_spread": spread,
           "plate": [[r[0], r[1], r[2], d.get("cx"), d.get("cy")] for r, d in zip(prows, window)],
           "motors": mrows, "stats": stats, "file": None}

    try:
        os.makedirs(LOGS, exist_ok=True)
        name = time.strftime("shot-%Y%m%d-%H%M%S", time.localtime(t_trig_wall)) + f"-{sid:03d}.csv"
        path = os.path.join(LOGS, name)
        with open(path, "w") as f:
            f.write(f"# shot {sid}  {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t_trig_wall))}\n")
            f.write(f"# trigger {why}  ({a['src']}, level {a['level']:g})  "
                    f"pre {a['pre_ms']:g} ms  post {a['post_ms']:g} ms\n")
            f.write(f"# plate_clock_offset {off}  offset_spread {spread}  "
                    f"(motor rows are aligned to the plate through this offset)\n")
            f.write("# " + json.dumps(stats) + "\n")
            f.write(SHOT_HDR)
            rows = [(r[0], "P", r) for r in prows] + [(r[0], "M", r) for r in mrows]
            rows.sort(key=lambda x: x[0])
            for t_ms, src, r in rows:
                if src == "P":
                    f.write(f"{t_ms:.3f},P,{r[1]:.4f},{r[2]:.6f},"
                            f"{r[3][0]:.3f},{r[3][1]:.3f},{r[3][2]:.3f},{r[3][3]:.3f}"
                            + "," * 15 + "\n")
                else:
                    mm, cmd, state = r[1], r[2], r[3]
                    cells = []
                    for mid in ("1", "2"):
                        v = mm.get(mid, {})
                        cells += [_num(v.get(k), 5) for k in ("pos", "vel", "tq", "q", "d", "pw")]
                    f.write(f"{t_ms:.3f},M" + "," * 7 + ",".join(cells)
                            + f",{_num(cmd.get('1'), 5)},{_num(cmd.get('2'), 5)},{state}\n")
        out["file"] = name
        print(f"[shot {sid}] {why}: {len(prows)} plate + {len(mrows)} motor rows -> logs/{name}"
              f"   peak Fz {stats.get('fz_peak_N')} N", flush=True)
    except OSError as e:
        out["file"] = f"save failed: {e}"
        print(f"[shot {sid}] save failed: {e}", flush=True)

    with lock:
        shots.append(out)
        shot["state"] = "ready"
        shot["ready_id"] = sid
    return out


def watcher():
    """Finishes captures once their post-trigger window has elapsed, and keeps
    re-asserting the feed rate we want in case platelog was restarted."""
    t_assert = 0.0
    while True:
        time.sleep(0.02)
        with lock:
            due = (shot["state"] == "capturing" and shot["t_end_wall"]
                   and wall() >= shot["t_end_wall"])
        if due:
            try:
                build_shot()
            except Exception as e:                        # noqa: BLE001
                print("[shot] build failed:", e, flush=True)
                with lock:
                    shot["state"] = "idle"
                    shot["why"] = f"build failed: {e}"
        now = wall()
        if now - t_assert > 2.0:
            t_assert = now
            if st["plate_peer"]:
                plate_cmd(f"DIV {st['want_div']}")
                plate_cmd("PING")


# -------------------------------------------------------------------- HTTP
def warnings_now():
    """(alarms, notes) from the newest motor packet: alarms are red, notes are
    things that are merely worth knowing, like flux braking being engaged."""
    out, notes = [], []
    L = cfg["limits"]
    if st.get("reg_fixes"):
        ago = wall() - (st.get("reg_fix_last") or 0)
        msg = (f"plate ADC settings were lost and restored {st['reg_fixes']}x "
               f"(last {ago:.0f} s ago) -- plate data in the second before each is suspect")
        (out if ago < 60 else notes).append(msg)
    with lock:
        m = motors[-1][1] if motors else None
    if not m:
        return out, notes
    for mid, v in (m.get("m") or {}).items():
        q, d = v.get("q"), v.get("d")
        if q is not None and d is not None:
            amps = math.hypot(q, d)
            if amps >= L["max_current_A"]:
                out.append(f"motor {mid}: {amps:.1f} A at the {L['max_current_A']:g} A limit")
            elif amps >= L["warn_current_A"]:
                out.append(f"motor {mid}: {amps:.1f} A")
        for k, what in (("temp", "board"), ("mtemp", "motor")):
            t = v.get(k)
            if t and t >= L["warn_temperature_C"]:
                out.append(f"motor {mid} {what} {t:.0f} C "
                           f"(faults at {L['fault_temperature_C']:g})")
        volt = v.get("volt")
        if volt:
            brake = L["max_voltage_V"] - L["flux_brake_margin_V"]
            if volt >= L["max_voltage_V"]:
                out.append(f"motor {mid}: bus {volt:.1f} V at servo.max_voltage")
            elif volt >= brake + 0.25:
                # Only once the bus has actually been pushed up by regen: a supply
                # sitting exactly on the flux-brake threshold would otherwise warn
                # forever and train you to ignore this line.
                out.append(f"motor {mid}: bus {volt:.1f} V -- above the {brake:g} V "
                           f"flux-brake threshold (regen)")
            elif volt <= L["min_voltage_V"]:
                out.append(f"motor {mid}: bus {volt:.1f} V is low")
        flt = int(v.get("flt") or 0)
        if flt:
            # 96..105 say what is limiting the output right now; the controller
            # is still running.  Only a real fault stops the motor.
            name = FAULT_NAMES.get(flt, "unknown")
            if flt in LIMIT_CODES:
                if int(v.get("mode") or 0) != 1:
                    notes.append(f"motor {mid}: {name}")
                else:
                    out.append(f"motor {mid}: fault {flt} ({name})")
            else:
                out.append(f"motor {mid}: fault {flt} ({name})")
        if v.get("home") == 0:
            out.append(f"motor {mid}: home_state=relative -- position is not referenced")
    # moteus re-reads position from the rotor at every power-up, wrapped into
    # +-0.5 rev.  If the reference pose sits near that boundary, one power-up in
    # two can land a whole motor revolution out -- 360/ratio degrees at the joint.
    for j in cfg["joints"]:
        z, ratio = j.get("zero_rev", 0.0), (j.get("gear_ratio") or 1.0)
        margin = 0.5 - abs((z + 0.5) % 1.0 - 0.5)
        if z and margin < 0.1:
            out.append(f"{j['label']}: zero_rev {z:+.3f} is only {margin:.3f} rev "
                       f"({margin * 360 / ratio:.1f} deg at the joint) from the encoder "
                       f"wrap -- a power-up here can reference {360 / ratio:.0f} deg out")
    return out, notes


def status_json():
    warn, notes = warnings_now()
    now = wall()
    with lock:
        rec = st["recording"]
        sh = {"state": shot["state"], "id": shot["id"], "why": shot["why"],
              "ready_id": shot["ready_id"], "arm": shot["arm"],
              "t_trig_wall": shot["t_trig_wall"]}
        recent = [{"id": s["id"], "when": s["when"], "why": s["why"],
                   "file": s["file"], "stats": s["stats"]} for s in shots]
    return {"now": now,
            "plate_age": round(now - st["plate_last"], 2) if st["plate_last"] else None,
            "motor_age": round(now - st["motor_last"], 2) if st["motor_last"] else None,
            "plate_hz": round(st["plate_hz"]), "motor_hz": round(st["motor_hz"]),
            "plate_rec": st["plate_rec"], "gaps": st["gaps"],
            "udp_lost": st["udp_lost"],
            "plate_div": st["plate_div"], "plate_sps": st["plate_sps"],
            "plate_off": st["plate_off"], "plate_off_spread": st["plate_off_spread"],
            "mode": st["mode"], "want_div": st["want_div"], "z_ref": sp["ref"],
            "hop": st.get("hop") if now - st.get("hop_t", 0) < 1.5 else None,
            "cop_cal": None if PM is None else {k: PM.get(k) for k in
                       ("date", "loo_fz_rms_g", "loo_cop_rms_x_mm", "loo_cop_rms_y_mm", "chan_corner")},
            "recording": ({"file": os.path.basename(rec["path"]),
                           "seconds": round(now - rec["t0"], 1)} if rec else None),
            "last_fetch": st["last_fetch"], "motor_state": st["motor_state"],
            "remote": st["remote"], "shot": sh, "shots": recent,
            "cfg": cfg, "warnings": warn, "notes": notes}


def decimate_plate(rows, target):
    """Keep <= target plate rows [t, fz, z, lo, hi], widening the lo/hi band to
    cover every row it swallowed, so a spike inside a dropped group still shows
    up on the chart."""
    if len(rows) <= target:
        return rows
    step = math.ceil(len(rows) / target)
    out = []
    for i in range(0, len(rows), step):
        g = rows[i:i + step]
        r = list(g[-1])
        r[3] = min(x[3] for x in g)
        r[4] = max(x[4] for x in g)
        out.append(r)
    return out


def subsample(rows, target):
    """Keep <= target rows, last of each group. For motor rows, whose payload is
    a dict we cannot meaningfully combine."""
    if len(rows) <= target:
        return rows
    step = math.ceil(len(rows) / target)
    return [rows[min(i + step, len(rows)) - 1] for i in range(0, len(rows), step)]


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            return json.loads(self.rfile.read(n).decode()) if n else {}
        except (ValueError, OSError):
            return {}

    def _query(self):
        q = {}
        if "?" in self.path:
            for part in self.path.split("?", 1)[1].split("&"):
                if "=" in part:
                    k, v = part.split("=", 1)
                    q[k] = v
        return q

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            with open(os.path.join(HERE, "dash.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if path == "/cells":
            # Mean / std of the four per-cell forces (N, platelog's per-cell gains and
            # tare) over the last `sec` seconds -- the plate calibrator's data source.
            q = self._query()
            sec = float(q.get("sec", 2.0))
            t0 = wall() - sec
            with lock:
                rows = [d for _, d in plate if d["tw"] >= t0]
            if not rows:
                return self._send(200, {"n": 0})
            out = {"n": len(rows), "sec": sec, "t_end": rows[-1]["tw"]}
            for k in range(4):
                v = [d["f"][k] for d in rows]
                mu = sum(v) / len(v)
                out[f"f{k}"] = mu
                out[f"s{k}"] = (sum((x - mu) ** 2 for x in v) / len(v)) ** 0.5
            v = [d["fz"] for d in rows]
            mu = sum(v) / len(v)
            out["fz"], out["sfz"] = mu, (sum((x - mu) ** 2 for x in v) / len(v)) ** 0.5
            return self._send(200, out)
        if path == "/status":
            return self._send(200, status_json())
        if path == "/config":
            return self._send(200, cfg)
        if path == "/shot":
            want = self._query().get("id")
            with lock:
                if want:
                    s = next((x for x in shots if x["id"] == int(want)), None)
                else:
                    s = shots[-1] if shots else None
            return self._send(200, s) if s else self._send(404, {"error": "no shot"})
        if path == "/logs":
            files = sorted(os.listdir(LOGS)) if os.path.isdir(LOGS) else []
            return self._send(200, [{"name": f, "kb": round(os.path.getsize(os.path.join(LOGS, f)) / 1024)}
                                    for f in files if f.endswith(".csv")][-40:])
        if path == "/stream":
            return self.stream()
        self._send(404, {"error": "not found"})

    def do_POST(self):
        path = self.path.split("?")[0]
        b = self._body()
        if path == "/record":
            (stop_recording if st["recording"] else start_recording)()
            return self._send(200, {"recording": status_json()["recording"]})
        if path == "/hopkey":
            # control panel -> hop.py: the same single-key commands as its terminal
            k = str(b.get("key", ""))
            if k not in ("h", "a", "+", "-", "]", "[", "q", " "):
                return self._send(200, {"ok": False, "why": "unknown key"})
            if not st.get("hop") or wall() - st.get("hop_t", 0) > 1.5:
                return self._send(200, {"ok": False, "why": "hop.py is not running"})
            try:
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(k.encode(), ("127.0.0.1", HOP_CTL))
            except OSError as e:
                return self._send(200, {"ok": False, "why": str(e)})
            return self._send(200, {"ok": True})
        if path == "/tare":
            ok = plate_cmd("TARE")
            if ok:                      # platelog now reads 0 empty: no CoP offset left
                cfg.setdefault("plate", {})["cop_zero"] = [0.0, 0.0, 0.0, 0.0]
                save_cfg()
            return self._send(200, {"ok": ok})
        if path == "/cart-mm":
            try:
                mm = float(b.get("mm"))
            except (TypeError, ValueError):
                return self._send(200, {"ok": False, "why": "need a ruler height in mm"})
            if not st["plate_last"]:
                return self._send(200, {"ok": False, "why": "no plate/encoder feed yet"})
            sp_set_cart(mm)
            return self._send(200, {"ok": True, "N0": round(sp["N0"], 1),
                                    "y_now_mm": round(sp_y_mm(sp["enc"]), 2)})
        if path == "/restart-plate":
            threading.Thread(target=remote_start_platelog, daemon=True).start()
            return self._send(200, {"ok": True})
        if path == "/mode":
            return self._send(200, {"ok": set_mode(b.get("mode")), "mode": st["mode"]})
        if path == "/arm":
            s = cfg["shot"]
            ok, why = arm(b.get("pre_ms", s["pre_ms"]), b.get("post_ms", s["post_ms"]),
                          b.get("src", s["src"]), b.get("level", s["level"]))
            return self._send(200, {"ok": ok, "why": why})
        if path == "/trigger":
            return self._send(200, {"ok": manual_trigger()})
        if path == "/disarm":
            with lock:
                shot["state"] = "idle"
                shot["why"] = "disarmed"
            return self._send(200, {"ok": True})
        if path == "/zero-angles":
            with lock:
                m = motors[-1][1] if motors else None
            if not m:
                return self._send(200, {"ok": False, "why": "no motor telemetry"})
            for j in cfg["joints"]:
                p = (m.get("m") or {}).get(str(j["id"]), {}).get("pos")
                if p is not None:
                    j["zero_rev"] = round(p, 6)
            save_cfg()
            return self._send(200, {"ok": True, "joints": cfg["joints"]})
        if path == "/calib-straight":
            # Called with the leg held STRAIGHT (the two links collinear), which is
            # knee = 0 by definition and the only leg pose you can set by eye to
            # better than a degree.  Folded is a poor reference: the links stop
            # against each other at whatever angle their own thickness allows.
            with lock:
                m = motors[-1][1] if motors else None
            if not m:
                return self._send(200, {"ok": False, "why": "no motor telemetry"})

            def raw(j):     # joint angle with the stored zero/ratio/sign, offset 0
                pos = (m.get("m") or {}).get(str(j["id"]), {}).get("pos")
                if pos is None:
                    return None
                return ((pos - j.get("zero_rev", 0.0)) / (j.get("gear_ratio") or 1.0)
                        * 360.0 * (j.get("sign", 1) or 1))
            j1, j2 = cfg["joints"][0], cfg["joints"][1]   # [0] = root, [1] = distal
            a, b = raw(j1), raw(j2)
            if a is None or b is None:
                return self._send(200, {"ok": False, "why": "a motor did not report position"})
            rel = bool((cfg.get("leg") or {}).get("knee_relative"))
            j1["offset_deg"] = 0.0
            j2["offset_deg"] = (-b) if rel else (a - b)
            save_cfg()
            mm = m.get("m") or {}
            g = leg_geom({int(k): (v or {}).get("pos") for k, v in mm.items()})
            return self._send(200, {"ok": True, "offset_deg": round(j2["offset_deg"], 3),
                                    "len_now_mm": round(g["len_mm"], 1) if g else None,
                                    "knee_now_deg": round(g["knee"], 2) if g else None})
        if path == "/calib-length":
            # Calibrate from a tape measure instead of from a pose you have to
            # hold by eye: "the hip-to-foot distance is X mm right now".  Works
            # at the folded pose, which is the one you can always reproduce.
            try:
                mm = float(b.get("mm"))
            except (TypeError, ValueError):
                return self._send(200, {"ok": False, "why": "need a length in mm"})
            lg = cfg.get("leg") or {}
            L = lg.get("segment_mm", 160.0)
            if not 0.0 <= mm <= 2 * L:
                return self._send(200, {"ok": False,
                                        "why": f"length must be 0..{2 * L:g} mm"})
            with lock:
                m = motors[-1][1] if motors else None
            if not m:
                return self._send(200, {"ok": False, "why": "no motor telemetry"})

            def raw(j):
                pos = (m.get("m") or {}).get(str(j["id"]), {}).get("pos")
                if pos is None:
                    return None
                return ((pos - j.get("zero_rev", 0.0)) / (j.get("gear_ratio") or 1.0)
                        * 360.0 * (j.get("sign", 1) or 1))
            j0, j1 = cfg["joints"][0], cfg["joints"][1]
            a, bb = raw(j0), raw(j1)
            if a is None or bb is None:
                return self._send(200, {"ok": False, "why": "a motor did not report position"})
            # In the folded-zero convention leg = 2 L sin(inc/2), so a measured
            # length pins the included angle.  Store the residual as the leg's
            # knee_offset_deg and leave the joint zeros alone -- they are yours.
            mm = max(0.0, mm - lg.get("foot_offset_mm", 0.0))   # measure to the GROUND
            target = 2.0 * math.degrees(math.asin(max(0.0, min(1.0, mm / (2.0 * L)))))
            cfg["leg"]["knee_offset_deg"] = round(target - abs(bb), 4)
            save_cfg()
            g = leg_geom({int(k): (v or {}).get("pos") for k, v in (m.get("m") or {}).items()})
            return self._send(200, {"ok": True, "offset_deg": round(j1["offset_deg"], 3),
                                    "knee_target_deg": round(target, 2),
                                    "len_now_mm": round(g["len_mm"], 1) if g else None})
        if path == "/config":
            for k, v in b.items():
                if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                    cfg[k].update(v)
                elif k in cfg:
                    cfg[k] = v
            save_cfg()
            return self._send(200, cfg)
        self._send(404, {"error": "not found"})

    def stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")   # no Content-Length: ends at close
        self.end_headers()
        self.close_connection = True
        with lock:                                   # replay the last 10 s
            cutoff = wall() - 10
            ps = next((s for s, d in plate if d["tw"] >= cutoff), seq["plate"] + 1) - 1
            ms = next((s for s, d in motors if d["tw"] >= cutoff), seq["motor"] + 1) - 1
        k = 0
        try:
            while True:
                with lock:
                    p_new = [[round(d["tp"], 4), d["fz"], d["z"], d["lo"], d["hi"],
                              None if d.get("cx") is None else round(d["cx"], 2),
                              None if d.get("cy") is None else round(d["cy"], 2)]
                             for s, d in plate if s > ps]
                    m_new = [[round(d["tw"], 4), d.get("m", {}), d.get("cmd", {}),
                              d.get("state", "")] for s, d in motors if s > ms]
                    ps, ms = seq["plate"], seq["motor"]
                # At 1 kSPS this is ~50 rows a tick; the browser only needs enough
                # to draw a scrolling trace, and the one-shot view gets the real
                # samples from /shot instead.
                msg = {"plate": decimate_plate(p_new, 12),
                       "motors": subsample(m_new, 12)}
                k += 1
                if k % 10 == 1:
                    msg["status"] = status_json()
                self.wfile.write(f"data: {json.dumps(msg)}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-remote", action="store_true", help="don't (re)start platelog on the Pi")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args()
    load_cfg()
    os.makedirs(LOGS, exist_ok=True)
    threading.Thread(target=plate_listener, daemon=True).start()
    threading.Thread(target=motor_listener, daemon=True).start()
    threading.Thread(target=watcher, daemon=True).start()
    if not a.no_remote:
        threading.Thread(target=remote_start_platelog, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), Handler)
    srv.daemon_threads = True
    print(f"dashboard: http://localhost:{a.port}/    plate feed udp:{PLATE_UDP}   "
          f"motor telemetry udp:{MOTOR_UDP}", flush=True)
    print(f"gear ratios / limits: {CFG_PATH}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if st["recording"]:
            stop_recording()


if __name__ == "__main__":
    main()
