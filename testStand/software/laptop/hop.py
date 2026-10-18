#!/usr/bin/env python3
"""
Hop the leg: CROUCH -> PUSH -> LAND -> back to CROUCH, one hop per key press.
Or, with --style drop, a landing test with no push-off: from standing, fold the
leg faster than gravity ("the chair breaks"), so the body free-falls a set
distance onto the soft LAND catch, then stand back up.

    python3 hop.py --style drop --telemetry 127.0.0.1:9871     # 10 mm drops, +/- 5 mm

    python3 hop.py --dry-run                                   # no hardware
    python3 hop.py --telemetry 127.0.0.1:9871                  # LIVE, gentle defaults
    python3 hop.py --telemetry 127.0.0.1:9871 --push-accel 150 # harder

Poses are HIP HEIGHTS (mm above the floor), turned into joint angles by the same
IK as the dashboard -- foot straight under the hip, knee = 2 asin((h - foot)/2L),
hip = knee/2 -- and then into motor revolutions through leg_config.json (zero_rev,
gear_ratio, sign, offsets).  Both motors run synchronised trapezoids, so the leg
stays on the hip = knee/2 line the whole way and the push is vertical.

Phases (the state names are what the dashboard shows; only PUSH says "MOVE", so
the one-shot's "step.py starts a move" trigger catches exactly the hops):

  GO crouch    slow move to the crouch pose (start, and after every landing)
  HOLD crouch  stiff hold; press h to hop (or --auto N)
  MOVE push    fast synchronised move to the push pose.  The body leaves the
               floor when the leg decelerates faster than gravity -- the preflight
               estimate says whether this push will.
  LAND         move to the (bent) landing pose and hold it SOFTLY: kp_scale
               --land-kp, kd_scale 1, torque capped, so the leg catches the body
               like a spring-damper instead of fighting it.

Gains change per COMMAND only (kp_scale / kd_scale / ilimit_scale / maximum_torque
ride on each position command, exactly like velocity_limit / accel_limit in
step.py).  Nothing is written to the controllers' configuration.  The integral
term is off (ilimit_scale 0) from PUSH until back in the crouch hold, so nothing
winds up across the hop.

Control panel: with --telemetry, the dashboard (dash.py, http://localhost:8080)
shows a hop.py panel -- status, the estimate, the last result, and buttons for
everything below.  It sends the same key presses to 127.0.0.1:9872 (UDP), so the
keys keep working too.  Starting hop.py (and its y/N) stays in the terminal.

Keys: h hop · a auto on/off · + / - push VELOCITY x1.25 (the hop-height knob)
      ] / [ push accel x1.25 · space status · q quit.

How high: the body leaves the floor when the leg DEcelerates faster than gravity,
so the accel limit decides WHETHER it hops (>= ~90 knee rev/s^2 with these poses)
and the velocity limit decides HOW HIGH.  Preflight, accel 200 rev/s^2 (upper
bounds -- kinematics + gravity only):
      vel 3 rev/s ~1.5 mm · 4 ~3 mm · 5 ~4.5 mm · 6 ~7 mm · 8 ~13 mm · 10 ~22 mm
Suggested ramp: defaults (accel 60, vel 3) = push test, no flight -> ] up to
accel 200 (barely leaves) -> + one step at a time, reading the dashboard's peak
Fz and current between hops.
Ctrl-C or q stops the motors (same finally-block as step.py).  If this program is
killed hard the controllers' 0.1 s watchdog takes over, as with step.py.

Before the first live run: the positions are only meaningful if the controllers
powered up with the leg FOLDED (or "Zero angles" was redone since).  The program
checks that the measured joint angles are physically possible and refuses to move
otherwise.
"""

import argparse
import asyncio
import json
import math
import os
import socket
import sys
import time

import moteus
import moteus.multiplex as mp

# Shared, tested pieces of step.py (its main() only runs when executed directly).
from step import (_Telemetry, _FakeTransport, _check_fault, _complete, _pending_keys,
                  _cbreak, _sync_scale, SETTLE_CYCLES)

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "leg_config.json")
POSE_PATH = os.path.join(HERE, "taught_pose.json")   # --teach writes it, --crouch taught reads it
MOTOR_IDS = (1, 2)
G = 9.81

# Which way each joint OPENS in the leg_config frame.  From step.py's pose 2,
# which the user confirmed is the extended pose (328 mm): hip motor +1.932 rev ->
# +74.5 deg, knee motor -3.99 rev -> -158.4 deg.  So the hip opens positive and
# the knee NEGATIVE (leg_config's knee "sign" is still +1; this is the fix-up).
HIP_OPEN = +1
KNEE_OPEN = -1

# Rough figures for the preflight estimate only (they change no command):
KT_MOTOR = 8.27 / 306.5          # N.m per A at the motor, from motor.Kv
I_MAX = 12.0                     # servo.max_current_A (raised from 9 by the user, 2026-10-04)
# Lowest believable hip angle: the thigh can tilt well ABOVE horizontal (knee over
# the hip axle) -- a crouch taught 2026-10-17 read -31 deg and was real.
HIP_MIN = -60.0
# Straightest knee a taught push end may ask for (180 = straight).  The knee CAN spin past
# straight without damage (user, 2026-10-17); this only refuses a pose taught at/over straight.
PUSH_KNEE_MAX = 175.0


# ----------------------------------------------------------------- kinematics
class Leg:
    def __init__(self, cfg):
        lg = cfg["leg"]
        self.L = lg.get("segment_mm", 160.0)
        self.foot = lg.get("foot_offset_mm", 15.4)
        self.koff = lg.get("knee_offset_deg", 0.0)
        self.hip, self.knee = cfg["joints"][0], cfg["joints"][1]   # [0] root, [1] distal
        self.h_min, self.h_max = self.foot, 2 * self.L + self.foot

    def ik(self, h, x=0.0):
        """hip height mm, foot x mm AHEAD of the hip -> (hip deg, knee included deg).
        x = 0 is foot under the hip (hip = knee/2).  Hip = thigh angle below horizontal:
        beta - 90 + knee/2, beta = angle of the hip->ball line below horizontal."""
        y = h - self.foot
        d = math.hypot(x, y)
        if not (0.0 <= d <= 2 * self.L and y > 0):
            raise ValueError(f"height {h} mm, offset {x} mm is out of reach")
        k = 2 * math.degrees(math.asin(d / (2 * self.L)))
        return math.degrees(math.atan2(y, x)) - 90 + k / 2, k

    def fk(self, hip, knee):
        """(hip deg, knee included deg) -> (foot x mm ahead of the hip, hip height mm).
        The inverse of ik()."""
        d = 2 * self.L * math.sin(math.radians(knee) / 2)
        beta = math.radians(hip + 90 - knee / 2)
        return d * math.cos(beta), d * math.sin(beta) + self.foot

    def knee_height(self, hip, h):
        """How high (mm) the knee sits above the floor in that pose."""
        return h - self.L * math.sin(math.radians(hip))

    def height(self, knee, x=0.0):
        d = 2 * self.L * math.sin(math.radians(knee) / 2)
        return math.sqrt(max(d * d - x * x, 0.0)) + self.foot

    def knee_x(self, h, x=0.0):
        """How far (mm) the knee sits ahead of the hip in that pose."""
        return self.L * math.cos(math.radians(self.ik(h, x)[0]))

    @staticmethod
    def _rev(j, q_deg):
        return (j.get("zero_rev", 0.0) + (q_deg - j.get("offset_deg", 0.0)) / 360.0
                * (j.get("gear_ratio") or 1.0) * (j.get("sign", 1) or 1))

    @staticmethod
    def _deg(j, rev):
        return ((rev - j.get("zero_rev", 0.0)) / (j.get("gear_ratio") or 1.0) * 360.0
                * (j.get("sign", 1) or 1) + j.get("offset_deg", 0.0))

    def pose(self, h, x=0.0):
        """hip height (+ foot offset ahead of the hip) -> {motor id: rev}."""
        hip, knee = self.ik(h, x)
        return {int(self.hip["id"]): self._rev(self.hip, HIP_OPEN * hip),
                int(self.knee["id"]): self._rev(self.knee, KNEE_OPEN * knee - self.koff)}

    def measured(self, pos):
        """{motor id: rev} -> (hip deg, knee deg) in the OPEN-positive sense."""
        hip = HIP_OPEN * self._deg(self.hip, pos[int(self.hip["id"])])
        knee = KNEE_OPEN * (self._deg(self.knee, pos[int(self.knee["id"])]) + self.koff)
        return hip, knee


def preflight(leg, h0, h1, vel, accel, mass, x0=0.0, x1=0.0):
    """Simulate the push (the longer motor move is a trapezoid in motor revs, the other
    synchronised to it, so the leg follows a straight line in motor space -- as
    _sync_scale commands it) and report takeoff / hop height / force needed.
    x0 / x1: foot ahead of the hip at the crouch / push end.  Kinematics + gravity
    only: belts, friction and the current limit all make the real hop smaller."""
    (q0, k0), (q1, k1) = leg.ik(h0, x0), leg.ik(h1, x1)
    ratio = leg.knee.get("gear_ratio") or 1.0
    D = max(abs(k1 - k0), abs(q1 - q0)) / 360.0 * ratio  # longer motor move, revs
    t_acc = vel / accel
    if accel * t_acc ** 2 > D:                          # triangular
        t_acc = math.sqrt(D / accel)
        vpk, t_cru = accel * t_acc, 0.0
    else:
        vpk, t_cru = vel, (D - accel * t_acc ** 2) / vel
    T = 2 * t_acc + t_cru
    dt = 1e-4
    hs, xs, n = [], [], int(T / dt) + 2
    for i in range(n):
        t = min(i * dt, T)
        if t < t_acc:
            s = 0.5 * accel * t * t
        elif t < t_acc + t_cru:
            s = 0.5 * accel * t_acc ** 2 + vpk * (t - t_acc)
        else:
            u = T - t
            s = D - 0.5 * accel * u * u
        f = s / D
        xs.append(leg.fk(q0 + f * (q1 - q0), k0 + f * (k1 - k0)))
        hs.append(xs[-1][1] / 1000.0)
    v = [(hs[i + 1] - hs[i - 1]) / (2 * dt) for i in range(1, n - 1)]
    a = [(v[i + 1] - v[i - 1]) / (2 * dt) for i in range(1, len(v) - 1)]
    a_max = max(a)
    takeoff = next((i for i, x in enumerate(a) if x < -G), None)
    out = {"T_ms": T * 1000, "v_peak": max(v), "a_max_g": a_max / G}
    if takeoff is not None:
        vt = v[takeoff + 1]
        out.update(takeoff=True, v_to=vt, hop_mm=vt * vt / (2 * G) * 1000,
                   h_to=hs[takeoff + 2] * 1000)
    else:
        out.update(takeoff=False, decel_g=-min(a) / G)
    # Torque at the hardest-pushing instant.  A vertical force F at the ball needs
    # F x (horizontal distance joint -> ball) at each joint: hip F*|x|, knee
    # F*|x - knee_x|.  Foot under the hip (x = 0): the KNEE carries it all and the
    # hip nothing -- which is what the 12 A runs showed (knee 12 A, hip 4-8 A).
    F = mass * (G + a_max)
    i_max = a.index(a_max) + 2
    x, h_at = xs[i_max]
    kx = leg.knee_x(min(h_at, leg.h_max - 1), x)
    tau_k, tau_h = F * abs(x - kx) / 1000, F * abs(x) / 1000
    out.update(F_N=F, tau_Nm=max(tau_k, tau_h), tau_knee=tau_k, tau_hip=tau_h,
               tau_lim=KT_MOTOR * I_MAX * ratio)
    return out


def preflight_release(leg, h0, h_rel, tau_motor, mass):
    """Release push: the target jumps straight to the overshoot pose and the motors push
    at their torque cap (--push-torque, N.m at the motor) the whole way, until hop.py
    lets go at h_rel.  With the foot under the hip, joint torques tau_k (knee) and
    tau_h (hip = knee/2) give a vertical force F = (tau_k + tau_h/2) / (L cos(knee/2)),
    which GROWS as the leg straightens.  Integrates m h'' = F - m g.  Upper bound:
    no rotor inertia, belt loss or slider friction."""
    ratio = leg.knee.get("gear_ratio") or 1.0
    tau_j = tau_motor * ratio                         # per joint, both at the cap
    h, v, t, dt = h0 / 1000.0, 0.0, 0.0, 1e-5
    F0 = None
    while h * 1000 < h_rel and t < 1.0:
        k = leg.ik(h * 1000)[1]
        lever = leg.L / 1000 * math.cos(math.radians(k) / 2)
        F = 1.5 * tau_j / lever
        if F0 is None:
            F0 = F
        a = F / mass - G
        if a <= 0 and v <= 0:
            return {"takeoff": False, "F0": F0, "why": f"{F:.1f} N cannot lift {mass} kg"}
        v += a * dt
        h += v * dt
        t += dt
    return {"takeoff": True, "v_to": v, "hop_mm": v * v / (2 * G) * 1000, "T_ms": t * 1000,
            "F0": F0, "F_rel": 1.5 * tau_j / (leg.L / 1000 * math.cos(math.radians(leg.ik(h_rel)[1]) / 2)),
            "tau_j": tau_j, "I_A": tau_motor / KT_MOTOR}


def preflight_drop(leg, h_stand, drop, vel, accel):
    """Simulate the snap fold (knee motor trapezoid, stand -> stand - drop) with the
    body resting on the leg: the foot leaves the plate as soon as the leg shortens
    faster than gravity can pull the body after it; the body then free-falls until
    it meets the foot again.  Reports lift, clearance, time off the plate, impact
    speed, and whether touchdown comes before the half-way point of the fold, where
    hop.py switches to the soft LAND gains (else the catch meets the stiff snap gains).
    The leg's own braking at the end of the fold brings the foot back down early, so
    touchdown typically lands just before the fold FINISHES -- hence half-way."""
    k0, k1 = leg.ik(h_stand)[1], leg.ik(h_stand - drop)[1]
    ratio = leg.knee.get("gear_ratio") or 1.0
    D = (k0 - k1) / 360.0 * ratio
    t_acc = vel / accel
    if accel * t_acc ** 2 > D:
        t_acc = math.sqrt(D / accel)
        vpk, t_cru = accel * t_acc, 0.0
    else:
        vpk, t_cru = vel, (D - accel * t_acc ** 2) / vel
    T = 2 * t_acc + t_cru
    dt = 1e-4

    def leglen(t):                                    # hip height if the foot were on the floor, m
        t = min(max(t, 0.0), T)
        if t < t_acc:
            s = 0.5 * accel * t * t
        elif t < t_acc + t_cru:
            s = 0.5 * accel * t_acc ** 2 + vpk * (t - t_acc)
        else:
            u = T - t
            s = D - 0.5 * accel * u * u
        return leg.height(k0 - s / ratio * 360.0) / 1000.0
    a_leg = max(-(leglen(t + dt) - 2 * leglen(t) + leglen(t - dt)) / dt ** 2
                for t in (i * dt for i in range(1, int(T / dt))))
    out = {"T_ms": T * 1000, "a_leg_g": a_leg / G, "lifts": False}
    t, zb, vb, air, t_lift, clear = 0.0, leglen(0), 0.0, False, None, 0.0
    while t < T + 1.0:
        t += dt
        L = leglen(t)
        if not air:
            v_leg = (L - leglen(t - dt)) / dt
            a_need = (L - 2 * leglen(t - dt) + leglen(t - 2 * dt)) / dt ** 2
            if a_need < -G:                           # leg pulls away faster than 1 g
                air, t_lift, vb, zb = True, t, v_leg, L
                continue                              # free fall starts here
            else:
                zb, vb = L, v_leg
                if t > T:
                    break
        if air:
            vb -= G * dt
            zb += vb * dt
            clear = max(clear, zb - L)
            if zb <= L:                               # touchdown
                v_imp = -(vb - (L - leglen(t - dt)) / dt)
                out.update(lifts=True, clear_mm=clear * 1000, flight_ms=(t - t_lift) * 1000,
                           v_imp=v_imp, equiv_mm=v_imp ** 2 / (2 * G) * 1000,
                           td_before_soft=t < T / 2)   # hop.py goes soft half-way through the fold
                break
    return out


# ---------------------------------------------------------------- teach
async def teach(args, leg):
    """Motors stopped (limp, back-drivable): show the live pose while the user sets the
    leg by hand, save the joint angles on Enter.  Angles, not motor revs: they stay valid
    across power-ups as long as the controllers power up with the leg folded."""
    transport = _FakeTransport(args.rate) if args.dry_run else moteus.Fdcanusb()
    telemetry = _Telemetry(args.telemetry) if args.telemetry else None
    qr = moteus.QueryResolution()
    qr.home_state = mp.INT8
    ctrl = {m: moteus.Controller(id=m, transport=transport, query_resolution=qr) for m in MOTOR_IDS}
    if args.dry_run:                     # pretend the hand set: crouch 170 / land 220 / push 300 mm
        transport.pos = {"crouch": leg.pose(170, 25), "land": leg.pose(220, 10),
                         "push": leg.pose(300, 20)}[args.teach]
    else:
        await transport.cycle([c.make_stop() for c in ctrl.values()])
    print(f"[teach] motors are OFF -- move the leg by hand to the {args.teach.upper()} pose you want.\n"
          "        Enter = save this pose · q + Enter = quit without saving")
    line = asyncio.create_task(asyncio.to_thread(input))
    pos, hip, knee = {}, None, None
    try:
        while True:
            results = await transport.cycle([c.make_query() for c in ctrl.values()])
            for r in results:
                p = r.values.get(moteus.Register.POSITION)
                if p is not None:
                    pos[r.id] = p
            if telemetry:
                telemetry.send(results, dict(pos), "TEACH (motors off)")
            if all(m in pos for m in MOTOR_IDS):
                hip, knee = leg.measured(pos)
                x, h = leg.fk(hip, knee)
                sys.stdout.write(f"\r  hip {hip:6.1f}  knee {knee:6.1f} deg  ->  hip height {h:5.0f} mm, "
                                 f"foot {x:+4.0f} mm ahead, knee {leg.knee_height(hip, h):4.0f} mm up   ")
                sys.stdout.flush()
            if line.done():
                break
            await asyncio.sleep(0.05)
    finally:
        if not args.dry_run:
            await transport.cycle([c.make_stop() for c in ctrl.values()])
    print()
    if line.result().strip().lower().startswith("q"):
        print("[teach] not saved")
        return
    if hip is None:
        sys.exit("[!] no position from the motors -- nothing saved")
    if not (HIP_MIN <= hip <= 105 and -15 <= knee <= 195):
        sys.exit("[!] those joint angles are not physically possible (powered up with the leg not "
                 "folded?) -- nothing saved")
    x, h = leg.fk(hip, knee)
    rec = {}
    if os.path.exists(POSE_PATH):        # keep the other pose; previous file -> .prev, in case
        try:
            with open(POSE_PATH) as f:
                rec = json.load(f)
        except ValueError:
            pass
        os.replace(POSE_PATH, POSE_PATH + ".prev")
    rec[args.teach] = {"hip_deg": round(hip, 2), "knee_deg": round(knee, 2),
                       "height_mm": round(h, 1), "foot_x_mm": round(x, 1),
                       "motor_rev": {str(m): round(pos[m], 4) for m in MOTOR_IDS},
                       "when": time.strftime("%Y-%m-%d %H:%M")}
    with open(POSE_PATH, "w") as f:
        json.dump(rec, f, indent=2)
        f.write("\n")
    print(f"[teach] {args.teach} saved to {POSE_PATH}: hip {hip:.1f} knee {knee:.1f} deg = hip height "
          f"{h:.0f} mm, foot {x:+.0f} mm ahead of the hip.\n        Use it with:  python3 hop.py "
          f"--{args.teach} taught ...")


# ---------------------------------------------------------------- controller
async def run(args):
    with open(CFG_PATH) as f:
        cfg = json.load(f)
    leg = Leg(cfg)
    lo, hi = leg.h_min + 40, leg.h_max - 20
    if not -60 <= args.offset <= 60:
        sys.exit("--offset must be within +-60 mm")
    if args.teach:
        return await teach(args, leg)
    drop_style = args.style == "drop"
    release = not drop_style and args.push_mode == "release"
    through = not drop_style and args.push_mode == "through"
    def taught(name):
        """A pose saved by --teach -> (hip height mm, foot x mm ahead of the hip)."""
        if drop_style or release:
            sys.exit(f"--{name} taught is for the trapezoid hop only")
        try:
            with open(POSE_PATH) as f:
                tp = json.load(f)[name]
        except (OSError, KeyError, ValueError) as e:
            sys.exit(f"no taught {name} pose in {POSE_PATH} ({e}) -- run: python3 hop.py --teach {name}")
        x, h = leg.fk(tp["hip_deg"], tp["knee_deg"])
        print(f"[*] taught {name} ({tp.get('when', '?')}): hip {tp['hip_deg']:.1f} knee "
              f"{tp['knee_deg']:.1f} deg -> hip height {h:.0f} mm, foot {x:+.0f} mm ahead of the hip, "
              f"knee {leg.knee_height(tp['hip_deg'], h):.0f} mm above the floor")
        if not -120 <= x <= 120:
            sys.exit(f"taught {name} has the foot more than 120 mm from under the hip -- reteach")
        if name == "push" and tp["knee_deg"] > PUSH_KNEE_MAX:
            sys.exit(f"taught push has the knee at {tp['knee_deg']:.0f} deg -- keep it under "
                     f"{PUSH_KNEE_MAX:.0f} (180 = straight) -- reteach a little bent")
        return round(h, 2), round(x, 2)

    crouch_x = args.offset               # foot ahead of the hip in the crouch
    land_x = 0.0                         # ... and in the landing pose
    if args.crouch == "taught":
        args.crouch, crouch_x = taught("crouch")
    elif args.crouch is not None:
        args.crouch = float(args.crouch)
    if args.land == "taught":
        args.land, land_x = taught("land")
    else:
        args.land = float(args.land)
    if args.push == "taught":            # the lift-off pose: height AND foot x (replaces --offset)
        args.push, args.offset = taught("push")
        hi = max(hi, args.push)          # taught near-straight is allowed; taught() checked the knee
    else:
        args.push = float(args.push)
    if args.crouch is None:
        args.crouch = 270.0 if release else 180.0
    if release and not (args.push < args.overshoot <= leg.h_max - 15):
        sys.exit(f"need push (release height {args.push}) < overshoot <= {leg.h_max - 15:.0f} mm")
    if drop_style:
        if not (lo <= args.stand - args.max_drop and args.stand <= hi):
            sys.exit(f"need {lo:.0f} <= stand - max-drop and stand <= {hi:.0f} mm")
        HOME, FAST = "stand", "snap"
    else:
        if release and not (lo <= args.crouch < args.push and lo <= args.land < args.push <= hi):
            sys.exit(f"release push needs {lo:.0f} <= crouch, land < push <= {hi:.0f} mm "
                     f"(got {args.crouch}, {args.land}, {args.push})")
        if not release and not (lo <= args.crouch < args.land < args.push <= hi):
            sys.exit(f"need {lo:.0f} <= crouch < land < push <= {hi:.0f} mm "
                     f"(got {args.crouch}, {args.land}, {args.push})")
        HOME, FAST = "crouch", "push"
    PH_GO, PH_HOLD, PH_FAST = f"GO {HOME}", f"HOLD {HOME}", f"MOVE {FAST}"

    # live knobs: fast-move velocity / accel, and (drop) the fold distance
    fast_vel = args.snap_vel if drop_style else args.push_vel
    fast_accel = args.snap_accel if drop_style else args.push_accel
    drop_mm = args.drop
    push_torque = args.push_torque
    crouch_mm = args.crouch
    rehome = False                       # release: crouch changed while holding -> move there

    def heights():
        if drop_style:
            return {"home": args.stand, "fast": args.stand - drop_mm, "land": args.stand - drop_mm}
        return {"home": crouch_mm, "fast": args.overshoot if release else args.push,
                "land": args.land}

    def offset_of(name):                 # foot ahead of the hip: crouch + push only
        if drop_style:
            return 0.0
        return {"home": crouch_x, "land": land_x}.get(name, args.offset)
    P = {}

    def make_poses():
        for k, h in heights().items():
            P[k] = leg.pose(h, offset_of(k))
    make_poses()

    def estimate():
        if drop_style:
            return preflight_drop(leg, args.stand, drop_mm, fast_vel, fast_accel)
        if release:
            return preflight_release(leg, crouch_mm, args.push, push_torque, args.mass)
        return preflight(leg, args.crouch, args.push, fast_vel, fast_accel, args.mass,
                         crouch_x, args.offset)

    def plan_text(pf):
        if drop_style:
            s = (f"    snap {args.stand:.0f} -> {args.stand - drop_mm:.0f} mm (fold {drop_mm:.0f} mm) "
                 f"in {pf['T_ms']:.0f} ms (knee motor: vel {fast_vel:.2f} rev/s, accel {fast_accel:.0f} "
                 f"rev/s^2), leg shortens at up to {pf['a_leg_g']:.1f} g\n")
            if pf["lifts"]:
                s += (f"    -> FOOT LIFTS {pf['clear_mm']:.1f} mm, off the plate {pf['flight_ms']:.0f} ms, "
                      f"body lands at {pf['v_imp']:.2f} m/s (= a {pf['equiv_mm']:.0f} mm drop)")
                if pf["td_before_soft"]:
                    s += "\n    ** touchdown before the switch to soft gains: lands stiff -- raise accel **"
            else:
                s += "    -> foot stays down (leg shortens slower than 1 g): body just follows -- raise accel"
            return s
        if release:
            s = (f"    RELEASE push {crouch_mm:.0f} -> let go at {args.push:.0f} mm "
                 f"(stroke {args.push - crouch_mm:.0f} mm, target {args.overshoot:.0f}), "
                 f"torque cap {push_torque:.3f} N.m at the motor = {pf['F0'] and pf.get('tau_j', push_torque * 9):.2f} "
                 f"N.m per joint (~{push_torque / KT_MOTOR:.1f} A)\n")
            if pf["takeoff"]:
                s += (f"    -> push force {pf['F0']:.1f} N growing to {pf['F_rel']:.1f} N, {pf['T_ms']:.0f} ms; "
                      f"take-off {pf['v_to']:.2f} m/s: hop ~{pf['hop_mm']:.0f} mm "
                      f"(upper bound for {args.mass} kg -- no rotor inertia / belts / friction)")
            else:
                s += f"    -> no take-off: {pf['why']}"
            return s
        s = (f"    push {args.crouch:.0f} -> {args.push:.0f} mm in {pf['T_ms']:.0f} ms "
             f"(knee motor: vel {fast_vel:.2f} rev/s, accel {fast_accel:.0f} rev/s^2), "
             f"peak body speed {pf['v_peak']:.2f} m/s\n")
        if pf["takeoff"]:
            s += (f"    -> LEAVES THE FLOOR at ~{pf['h_to']:.0f} mm, {pf['v_to']:.2f} m/s: "
                  f"hop ~{pf['hop_mm']:.0f} mm (upper bound)\n")
        else:
            s += (f"    -> stays on the floor (leg decelerates at {pf['decel_g']:.2f} g < 1 g): "
                  f"a push test, no flight\n")
        s += (f"    force ~{pf['F_N']:.1f} N for {args.mass} kg -> knee {pf['tau_knee']:.2f}, hip "
              f"{pf['tau_hip']:.2f} N.m (12 A gives ~{pf['tau_lim']:.1f}; foot {crouch_x:+.0f} -> {args.offset:+.0f} mm ahead of hip)"
              + ("   ** will current-limit: the real push is weaker **" if pf["tau_Nm"] > pf["tau_lim"] else ""))
        return s

    transport = _FakeTransport(args.rate) if args.dry_run else moteus.Fdcanusb()
    telemetry = _Telemetry(args.telemetry) if args.telemetry else None
    qr = moteus.QueryResolution()
    qr.trajectory_complete = mp.INT8
    qr.q_current = mp.F32
    qr.d_current = mp.F32
    qr.power = mp.F32
    qr.motor_temperature = mp.F32
    qr.home_state = mp.INT8
    ctrl = {m: moteus.Controller(id=m, transport=transport, query_resolution=qr) for m in MOTOR_IDS}

    if not args.dry_run:
        await transport.cycle([c.make_stop() for c in ctrl.values()])
    actual = {}
    for _ in range(5):
        for r in await transport.cycle([c.make_query() for c in ctrl.values()]):
            p = r.values.get(moteus.Register.POSITION)
            if p is not None:
                actual[r.id] = p
        await asyncio.sleep(0.01)
    if args.dry_run:                     # the fake motors start at the home pose
        transport.pos = dict(P["home"])
        actual = dict(P["home"])
    if any(m not in actual for m in MOTOR_IDS):
        sys.exit(f"no position from motor(s) {[m for m in MOTOR_IDS if m not in actual]}")

    # ---- is the encoder frame believable?  (power-up with the leg folded)
    hip_now, knee_now = leg.measured(actual)
    print(f"[*] now: hip {hip_now:+.1f} deg, knee {knee_now:+.1f} deg "
          f"-> hip height {leg.fk(hip_now, knee_now)[1]:.0f} mm")
    if not (HIP_MIN <= hip_now <= 105 and -15 <= knee_now <= 195):
        sys.exit("[!] those joint angles are not physically possible: the controllers' position "
                 "frame does not match leg_config.json (powered up with the leg not folded?). "
                 "Fold the leg and power-cycle the controllers, or redo 'Zero angles'. Not moving.")

    pf = estimate()
    print(f"[*] style {args.style}" + (" (release push)" if release else "")
          + "; poses (hip height -> motor rev):")
    for nm, h in heights().items():
        hip, knee = leg.ik(h, offset_of(nm))
        print(f"    {nm:5s} {h:5.0f} mm  foot {offset_of(nm):+3.0f} mm  hip {hip:5.1f} knee {knee:5.1f} deg   "
              f"m1 {P[nm][1]:+.3f}  m2 {P[nm][2]:+.3f}")
    print(plan_text(pf))
    print(f"[*] LAND: kp_scale {args.land_kp}, kd_scale 1, max torque {args.land_torque} N.m (motor), "
          f"held {args.land_time} s.  Rate {args.rate} Hz.  Trips: current {f'{args.max_current} A' if args.max_current else 'off (controllers protect themselves)'}, temp {args.max_temp} C.")
    if not args.dry_run and not args.yes:
        # While the question waits, tell the dashboard panel why nothing is happening yet.
        async def waiting_beacon():
            while True:
                if telemetry:
                    telemetry.send([], P["home"], "WAIT y/N", extra={"hop": {
                        "style": args.style, "phase": "waiting for y + Enter in the terminal",
                        "ready": False, "home": HOME, "count": 0, "auto": False,
                        "vel": round(fast_vel, 3), "accel": round(fast_accel, 1),
                        "drop_mm": drop_mm if drop_style else None, "plan": plan_text(pf),
                        "last": "", "control": False, "waiting": True}})
                await asyncio.sleep(0.5)
        beacon = asyncio.create_task(waiting_beacon())
        try:
            ans = await asyncio.to_thread(
                input, f"[?] The leg will now move SLOWLY to the {HOME} pose. Type y + Enter to go: ")
        except EOFError:
            ans = ""
        finally:
            beacon.cancel()
        # tolerate a terminal left in an odd state (stray control characters, \r, "yes")
        clean = "".join(ch for ch in ans if ch.isprintable()).strip().lower()
        if not clean.startswith("y"):
            print(f"not moving (received {ans!r}; type y then Enter -- if typing looks odd, run: stty sane)")
            return

    # dashboard control panel: the same key presses, as UDP datagrams from dash.py
    ctl = None
    if args.control_port:
        ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ctl.bind(("127.0.0.1", args.control_port))
        ctl.setblocking(False)
    last_result = ""

    def panel_keys():
        if ctl is None:
            return
        while True:
            try:
                data, _ = ctl.recvfrom(256)
            except (BlockingIOError, OSError):
                return
            for ch in data.decode(errors="ignore")[:8]:
                yield ch

    period = 1.0 / args.rate
    phase, target = PH_GO, P["home"]
    scale = _sync_scale({m: abs(P["home"][m] - actual[m]) for m in MOTOR_IDS})
    cyc = 0                        # cycles since the phase began
    t_phase = time.time()
    auto_left = args.auto
    hops = 0
    peaks = None                   # per-hop extremes, reported after LAND
    fd = sys.stdin.fileno()
    print("[keys] h " + ("drop" if drop_style else "hop") + " · a auto · + / - "
          + ("fold distance (5 mm)" if drop_style else "torque cap x1.25" if release
             else "push velocity (height)")
          + (" · ] / [ stroke 10 mm deeper / shallower" if release else " · ] / [ accel")
          + " · space · q quit")

    knee_id = int(leg.knee["id"])
    start = {}
    brake_peak = None                    # through: furthest knee position seen while braking

    def aim_pose():
        """through push: the push pose carried on along the crouch -> push line in motor space,
        far enough (+10 %) that the planned move could only START slowing down there -- so it is
        still at full speed when the knee reaches the push pose and the brake takes over."""
        span = {m: P["fast"][m] - P["home"][m] for m in MOTOR_IDS}
        s_k = abs(span[knee_id]) / (max(abs(d) for d in span.values()) or 1e-9)
        ext = 1.1 * (fast_vel * s_k) ** 2 / (2 * fast_accel * s_k)     # knee motor rev
        f = ext / (abs(span[knee_id]) or 1e-9)
        return {m: P["fast"][m] + span[m] * f for m in MOTOR_IDS}

    if through:
        aim = aim_pose()
        a_hip, a_knee = leg.measured(aim)
        print(f"[*] THROUGH push: the plan aims at knee {a_knee:.0f} / hip {a_hip:.0f} deg (m1 {aim[1]:+.3f} "
              f"m2 {aim[2]:+.3f}) and is braked at the push pose; the estimate above is for the old "
              f"stop-at-the-pose plan, so it UNDER-states the take-off speed")

    def enter(new, tgt):
        nonlocal phase, target, scale, cyc, t_phase
        phase, target, cyc, t_phase = new, tgt, 0, time.time()
        start.update(actual)
        scale = _sync_scale({m: abs(tgt[m] - actual[m]) for m in MOTOR_IDS})
        print(f"[*] {new}")

    def command(m):
        """The per-phase position command for motor m."""
        common = dict(position=target[m], velocity=0.0, query=True)
        if phase == PH_GO:
            ramp = min(1.0, (time.time() - t_phase) / 0.3)          # soft -> stiff after LAND
            kp = args.land_kp + (1.0 - args.land_kp) * ramp
            return ctrl[m].make_position(**common, velocity_limit=args.slow_vel * scale[m],
                                         accel_limit=args.slow_accel * scale[m],
                                         kp_scale=kp, kd_scale=1.0, ilimit_scale=0.0)
        if phase == PH_HOLD:
            return ctrl[m].make_position(**common, velocity_limit=args.slow_vel * scale[m],
                                         accel_limit=args.slow_accel * scale[m],
                                         kp_scale=1.0, kd_scale=1.0, ilimit_scale=1.0)
        if phase == PH_FAST and release:
            # target jumps to the overshoot pose (huge limits = a step); the torque cap sets the
            # effort; kd off -- damping would brake the push at speed.  The overshoot target
            # still bounds the extension if the release were ever missed.
            return ctrl[m].make_position(**common, velocity_limit=200.0, accel_limit=10000.0,
                                         kp_scale=1.0, kd_scale=0.0, ilimit_scale=0.0,
                                         maximum_torque=push_torque)
        if phase == PH_FAST:
            return ctrl[m].make_position(**common, velocity_limit=fast_vel * scale[m],
                                         accel_limit=fast_accel * scale[m],
                                         kp_scale=1.0, kd_scale=1.0, ilimit_scale=0.0)
        if phase == "BRAKE":
            # through push: pure DAMPING brake -- kp 0, target velocity 0, kd only.  At push
            # speeds kd*v is far over the current clamp, so it brakes as hard as max_current_A
            # allows, then fades out as the leg stops: no spring back.  (A full-gain position
            # step here rang on 2026-10-17: 16 deg past the brake point, then 38 deg back.)
            return ctrl[m].make_position(**common, velocity_limit=200.0, accel_limit=10000.0,
                                         kp_scale=0.0, kd_scale=1.0, ilimit_scale=0.0)
        if phase == "RETRACT":
            # release push: turn the still-extending leg round, STIFF -- soft gains and a
            # torque cap would let it coast on toward full extension
            return ctrl[m].make_position(**common, velocity_limit=max(args.land_vel, fast_vel) * scale[m],
                                         accel_limit=args.retract_accel * scale[m],
                                         kp_scale=1.0, kd_scale=1.0, ilimit_scale=0.0)
        # LAND: (hop) retract fast to the bent pose before touchdown; (drop) already there.
        # Either way: held SOFTLY, torque capped -- the catch.
        return ctrl[m].make_position(**common, velocity_limit=args.land_vel * scale[m],
                                     accel_limit=max(fast_accel, args.land_vel * 10) * scale[m],
                                     kp_scale=args.land_kp, kd_scale=1.0, ilimit_scale=0.0,
                                     maximum_torque=args.land_torque)

    def handle_key(ch):
        nonlocal fast_accel, fast_vel, drop_mm, push_torque, crouch_mm, rehome, auto_left, pf
        if ch == "q":
            return "quit"
        if ch == "h":
            return "hop"
        if ch == "a":
            auto_left = 0 if auto_left else max(args.auto, 1000)
            print(f"[*] auto {'ON' if auto_left else 'off'}")
        elif ch in "+=-][":
            if release and ch in "+=-":
                push_torque = (min(push_torque * 1.25, args.max_push_torque) if ch != "-"
                               else max(push_torque / 1.25, 0.01))
            elif ch in "+=":
                if drop_style:
                    drop_mm = min(drop_mm + 5, args.max_drop)
                else:
                    fast_vel = min(fast_vel * 1.25, args.max_push_vel)
            elif ch == "-":
                if drop_style:
                    drop_mm = max(drop_mm - 5, 5)
                else:
                    fast_vel = max(fast_vel / 1.25, 0.5)
            elif release:                # ] deeper crouch = longer stroke, [ shallower
                c = crouch_mm - 10 if ch == "]" else crouch_mm + 10
                crouch_mm = min(max(c, 140.0), args.push - 10)
                rehome = True
            elif ch == "]":
                fast_accel = min(fast_accel * 1.25, args.max_snap_accel if drop_style else args.max_push_accel)
            else:
                fast_accel = max(fast_accel / 1.25, 5.0)
            make_poses()                 # takes effect at the next hop / drop
            pf = estimate()
            print(plan_text(pf))
        elif ch == " ":
            print(f"[*] {phase}, {hops} done, "
                  + (f"torque {push_torque:.3f} N.m, crouch {crouch_mm:.0f} mm" if release else
                     f"vel {fast_vel:.2f} rev/s, accel {fast_accel:.0f} rev/s^2")
                  + (f", fold {drop_mm:.0f} mm" if drop_style else "") + f", auto {auto_left}")
        return None

    next_t = time.time()
    try:
        with _cbreak(fd):
            while True:
                results = await transport.cycle([command(m) for m in MOTOR_IDS])
                by_id = {r.id: r for r in results}
                for r in results:
                    p = r.values.get(moteus.Register.POSITION)
                    if p is not None:
                        actual[r.id] = p
                cyc += 1
                if telemetry:
                    extra = None
                    if cyc % 10 == 1 or phase != PH_HOLD:      # panel status, ~20 Hz when idle
                        extra = {"hop": {
                            "style": args.style if not release else "hop (release)",
                            "phase": phase, "ready": phase == PH_HOLD,
                            "home": HOME, "count": hops, "auto": bool(auto_left),
                            "vel": round(fast_vel, 3), "accel": round(fast_accel, 1),
                            "drop_mm": drop_mm if drop_style else None,
                            "torque": round(push_torque, 4) if release else None,
                            "crouch": crouch_mm, "stroke": args.push - crouch_mm if release else None,
                            "plan": plan_text(pf), "last": last_result,
                            "control": bool(ctl)}}
                    telemetry.send(results, target, phase, vel_lim=fast_vel, accel_lim=fast_accel,
                                   extra=extra)
                for m in MOTOR_IDS:
                    _check_fault(by_id.get(m), m, args.max_current, args.max_temp)

                if peaks is not None:                       # fast move + LAND extremes
                    for m in MOTOR_IDS:
                        v = by_id[m].values if m in by_id else {}
                        q, d = v.get(moteus.Register.Q_CURRENT), v.get(moteus.Register.D_CURRENT)
                        if q is not None:
                            peaks[f"I{m}"] = max(peaks[f"I{m}"], math.hypot(q, d or 0))
                        vb = v.get(moteus.Register.VOLTAGE)
                        if vb is not None:
                            peaks["Vmax"], peaks["Vmin"] = max(peaks["Vmax"], vb), min(peaks["Vmin"], vb)
                        pw = v.get(moteus.Register.POWER)
                        if pw is not None:
                            peaks["Pmin"] = min(peaks["Pmin"], pw)
                        vel = v.get(moteus.Register.VELOCITY)
                        if vel is not None:
                            peaks[f"v{m}"] = max(peaks[f"v{m}"], abs(vel))
                        # code 100 = the controller is cutting current because its BOARD is hot
                        # (from fault_temperature - temperature_margin = 50 C on these boards)
                        if int(v.get(moteus.Register.FAULT, 0) or 0) == 100:
                            peaks["hot"].add(m)
                        tb = v.get(moteus.Register.TEMPERATURE)
                        if tb is not None:
                            peaks["T"] = max(peaks["T"], tb)

                done = cyc > SETTLE_CYCLES and all(_complete(by_id.get(m)) for m in MOTOR_IDS)
                key = None
                for ch in (_pending_keys(fd) if fd is not None else ()):
                    if ch is None:
                        fd = None
                        break
                    key = handle_key(ch) or key
                for ch in panel_keys():
                    key = handle_key(ch) or key
                if key == "quit":
                    return

                if phase == PH_GO and done:
                    enter(PH_HOLD, P["home"])
                elif phase == PH_HOLD and rehome:
                    rehome = False
                    enter(PH_GO, P["home"])           # release: move to the new crouch now
                elif phase == PH_HOLD:
                    held = time.time() - t_phase
                    if (key == "hop" or (auto_left and held >= args.dwell)) and held >= 0.5:
                        if auto_left:
                            auto_left -= 1
                        hops += 1
                        peaks = {"I1": 0.0, "I2": 0.0, "v1": 0.0, "v2": 0.0,
                                 "Vmax": 0.0, "Vmin": 1e9, "Pmin": 0.0, "hot": set(), "T": 0.0}
                        enter(PH_FAST, aim_pose() if through else P["fast"])
                    elif key == "hop":
                        print(f"[*] still settling in the {HOME} pose -- press h again in a moment")
                elif phase == PH_FAST:
                    if release:
                        # let go the moment the hip passes the release height -- read from
                        # the encoders every cycle, never on a timer
                        h_now = leg.height(max(leg.measured(actual)[1], 0.0))
                        if h_now >= args.push:
                            print(f"[*] released at {h_now:.0f} mm")
                            enter("RETRACT", P["land"])
                        elif done or time.time() - t_phase > args.push_timeout:
                            # never reached the release height: the weight is still on the leg,
                            # so NO stiff fast retract (that drew 9.6 A) -- straight to the soft hold
                            print(f"[*] push stalled at {h_now:.0f} mm -- no release, soft hold")
                            enter("LAND", P["land"])
                    elif through:
                        # brake the moment the knee reaches the push (lift-off) pose -- read from
                        # the encoders every cycle, one cycle AHEAD (at ~22 rev/s the knee moves
                        # ~0.2 motor rev = 9 deg between reads).  done = the plan reached the aim
                        # pose without the crossing being seen: brake anyway.
                        k0, k1 = start[knee_id], P["fast"][knee_id]
                        rk = by_id.get(knee_id)
                        vk = (rk.values.get(moteus.Register.VELOCITY) if rk is not None else None) or 0.0
                        crossed = (actual[knee_id] + vk * period - k0) / ((k1 - k0) or 1e-9) >= 1.0
                        if crossed or done or time.time() - t_phase > args.push_timeout:
                            if not crossed:
                                print(f"[*] push did not reach the lift-off pose "
                                      f"({'plan finished' if done else 'timeout'}) -- braking")
                            brake_peak = actual[knee_id]
                            enter("BRAKE", dict(actual))
                    else:
                        # drop: soft from half-way through the fold -- the foot is already in
                        # the air and touchdown comes before the fold would finish
                        span = abs(target[knee_id] - start[knee_id]) or 1e-9
                        half = drop_style and abs(actual[knee_id] - start[knee_id]) / span >= 0.5
                        if done or half or time.time() - t_phase > args.push_timeout:
                            enter("LAND", P["land"])
                elif phase == "BRAKE":
                    dirn = 1.0 if target[knee_id] >= P["home"][knee_id] else -1.0
                    if (actual[knee_id] - brake_peak) * dirn > 0:
                        brake_peak = actual[knee_id]
                    rk = by_id.get(knee_id)
                    vk = rk.values.get(moteus.Register.VELOCITY) if rk is not None else None
                    if (cyc > SETTLE_CYCLES and vk is not None and abs(vk) < 2.0) \
                            or time.time() - t_phase > 0.15:
                        over = (brake_peak - P["fast"][knee_id]) * dirn
                        ratio = leg.knee.get("gear_ratio") or 1.0
                        knee_d = leg.measured({**actual, knee_id: brake_peak})[1]
                        print(f"[*] braked: knee overshot the lift-off pose by {over * 360 / ratio:.0f} deg "
                              f"(peak knee {knee_d:.0f} deg; 180 = straight)")
                        enter("LAND", P["land"])
                elif phase == "RETRACT":
                    # soft once the leg is clearly on its way back to the bent pose
                    span = abs(target[knee_id] - start[knee_id]) or 1e-9
                    back = (start[knee_id] - actual[knee_id]) / (start[knee_id] - target[knee_id] or 1e-9)
                    if back >= 0.3 or done or time.time() - t_phase > 0.15:
                        enter("LAND", P["land"])
                elif phase == "LAND" and time.time() - t_phase >= args.land_time:
                    p = peaks
                    last_result = (f"{args.style} {hops}: peak current m1 {p['I1']:.1f} A, m2 {p['I2']:.1f} A · "
                                   f"peak speed m1 {p['v1']:.1f}, m2 {p['v2']:.1f} rev/s · "
                                   f"bus {p['Vmin']:.1f}..{p['Vmax']:.1f} V · most negative power "
                                   f"{p['Pmin']:.1f} W (regen) · boards up to {p['T']:.0f} C"
                                   + (f" · ** motor {sorted(p['hot'])} THROTTLED BY BOARD TEMPERATURE "
                                      f"(>50 C): let it cool **" if p["hot"] else ""))
                    print(f"[{last_result}]")
                    peaks = None
                    enter(PH_GO, P["home"])

                next_t += period                           # fixed-rate loop
                await asyncio.sleep(max(0.0, next_t - time.time()))
                if time.time() - next_t > 0.05:            # fell behind: don't try to catch up
                    next_t = time.time()
    finally:
        print("\n[*] Stopping motors ...")
        if not args.dry_run:
            await transport.cycle([c.make_stop() for c in ctrl.values()])
        print("[✓] Stopped.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--style", choices=("hop", "drop"), default="hop",
                    help="hop: push off from the crouch.  drop: from standing, fold the leg faster "
                         "than gravity so the body free-falls a set distance onto a soft catch -- "
                         "a landing test with no push-off (default hop)")
    ap.add_argument("--offset", type=float, default=0.0,
                    help="hop: put the foot this many mm AHEAD of the hip (toward the knee) in the crouch "
                         "and push poses (push only with --crouch taught).  Foot under the hip (0) makes the knee carry the whole push and "
                         "the hip nothing; ~30-40 mm shares it, ~+24 %% force at 40 mm (quasi-static, "
                         "12 A).  The foot drifts ~6-9 mm sideways mid-push.  Landing stays under the "
                         "hip (default 0)")
    ap.add_argument("--stand", type=float, default=290.0, help="drop: standing hip height, mm (default 290)")
    ap.add_argument("--drop", type=float, default=10.0,
                    help="drop: how far the leg folds = roughly the fall height, mm (default 10); "
                         "+/- keys change it by 5")
    ap.add_argument("--max-drop", type=float, default=40.0, help="drop: + key cap, mm (default 40)")
    ap.add_argument("--snap-vel", type=float, default=15.0, help="drop: fold velocity limit, rev/s (default 15)")
    ap.add_argument("--snap-accel", type=float, default=300.0,
                    help="drop: fold accel limit, rev/s^2 (default 300: ~1.7 g of leg shortening)")
    ap.add_argument("--max-snap-accel", type=float, default=450.0, help="drop: ] key cap (default 450)")
    ap.add_argument("--crouch", default=None,
                    help="crouch hip height, mm (default 180; 270 with --push-mode release, where the "
                         "]/[ keys move it 10 mm deeper/shallower = the stroke knob), or 'taught' = the "
                         "pose saved by --teach (its height AND foot position; --offset then sets only "
                         "the push end)")
    ap.add_argument("--push", default=290.0,
                    help="end of push, mm (default 290), or 'taught' = the lift-off pose saved by "
                         "--teach push (its height AND foot position; replaces --offset)")
    ap.add_argument("--land", default=250.0,
                    help="landing pose: hip height, mm (default 250; foot under the hip), or 'taught' = "
                         "the pose saved by --teach land -- where the leg waits, soft, to catch the body")
    ap.add_argument("--push-mode", choices=("trapezoid", "through", "release"), default="trapezoid",
                    help="EXPERIMENTAL release: first live try (2026-10-04) trembled (kd 0 during the "
                         "push) and tripped at 9.6 A (stiff retract after a stalled push -- now fixed). "
                         "Use trapezoid.  trapezoid: one planned move crouch -> push pose; it slows down over its "
                         "second half and the body leaves when that exceeds 1 g.  release: the target "
                         "jumps to --overshoot and the motors push at --push-torque the whole stroke; "
                         "the moment the hip passes --push hop.py switches to the bent pose (stiff "
                         "RETRACT, then the soft LAND).  through: like trapezoid, but the planned move "
                         "aims PAST the push pose (far enough that it never starts slowing down "
                         "before it), so the leg is at full speed at lift-off; the moment the knee "
                         "reaches the push pose the target steps there and the motors brake the "
                         "airborne leg as hard as they can (it overshoots by the stopping distance, "
                         "printed after each hop), then the soft LAND (default trapezoid)")
    ap.add_argument("--overshoot", type=float, default=315.0,
                    help="release push: the step target, mm -- bounds the extension if the release "
                         "were missed (default 315 = knee 139 deg)")
    ap.add_argument("--push-torque", type=float, default=0.05,
                    help="release push: torque cap, N.m AT THE MOTOR (x9 at the joint; 0.24 ~ 9 A). "
                         "The hop-height knob in release mode; +/- keys change it (default 0.05)")
    ap.add_argument("--max-push-torque", type=float, default=0.2,
                    help="release push: + key cap, N.m at the motor (default 0.2 ~ 7.4 A)")
    ap.add_argument("--retract-accel", type=float, default=400.0,
                    help="release push: stiff turn-round after release, knee motor rev/s^2 (default 400)")
    ap.add_argument("--push-vel", type=float, default=3.0,
                    help="push velocity limit, knee motor rev/s -- the hop-HEIGHT knob (default 3); "
                         "+/- keys change it")
    ap.add_argument("--push-accel", type=float, default=60.0,
                    help="push accel limit, knee motor rev/s^2 -- decides WHETHER it leaves the "
                         "floor (default 60: it does not -- a push test); ]/[ keys change it")
    ap.add_argument("--max-push-vel", type=float, default=16.0,
                    help="the + key never goes above this (default 16 rev/s; raised from 10 after the "
                         "first hops -- at 9 A the knee already reached 14 rev/s on its own)")
    ap.add_argument("--max-push-accel", type=float, default=700.0,
                    help="the ] key never goes above this (default 700)")
    ap.add_argument("--land-vel", type=float, default=10.0,
                    help="retract speed into the landing pose, rev/s (default 10)")
    ap.add_argument("--slow-vel", type=float, default=3.0, help="crouch moves, rev/s (default 3)")
    ap.add_argument("--slow-accel", type=float, default=10.0, help="crouch moves, rev/s^2 (default 10)")
    ap.add_argument("--land-kp", type=float, default=0.25,
                    help="kp_scale while landing (default 0.25 = a quarter of the configured kp)")
    ap.add_argument("--land-torque", type=float, default=0.2,
                    help="maximum_torque while landing, N.m at the motor (default 0.2 ~ 1.8 N.m at "
                         "the joint)")
    ap.add_argument("--land-time", type=float, default=1.0, help="seconds in LAND (default 1)")
    ap.add_argument("--push-timeout", type=float, default=1.0, help="give up on PUSH after s (default 1)")
    ap.add_argument("--dwell", type=float, default=2.0, help="auto mode: seconds between hops")
    ap.add_argument("--auto", type=int, default=0, help="hop N times automatically (default 0: key h)")
    ap.add_argument("--mass", type=float, default=0.6,
                    help="moving mass for the preflight estimate only, kg (default 0.6, unmeasured)")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="computer <-> moteus command rate, Hz.  The moteus runs its own "
                         "position/current loops internally at tens of kHz; this only sets how quickly "
                         "hop.py switches phases (<= 1 period = 10 ms late) and how finely telemetry/recordings "
                         "sample (default 100).  The fdcanusb sustains up to ~500 Hz (CAN cycle ~1-1.4 ms)")
    ap.add_argument("--telemetry", metavar="HOST:PORT", default=None, help="e.g. 127.0.0.1:9871")
    ap.add_argument("--control-port", type=int, default=9872,
                    help="UDP port on 127.0.0.1 for the dashboard's control panel (default 9872, 0 = off)")
    ap.add_argument("--max-current", type=float, default=0.0,
                    help="trip (stop the motors) if phase current exceeds this, A (default 0 = OFF).  The "
                         "controllers regulate and protect themselves: they clamp at servo.max_current_A "
                         "(fault 99 = 'limiting', not a fault), and the measured current overshoots that "
                         "briefly on hard pushes and on the through-push BRAKE.  A software trip only ever "
                         "stopped normal moves -- 12.5 A mid-push on 2026-10-04, 14.3 A in BRAKE on "
                         "2026-10-17 -- and dropped the leg unpowered in mid-air, so it is off by default.")
    ap.add_argument("--max-temp", type=float, default=70.0, help="trip temperature, C (default 70)")
    ap.add_argument("--yes", action="store_true", help="skip the 'Go?' question")
    ap.add_argument("--dry-run", action="store_true", help="no hardware; simulated motors")
    ap.add_argument("--teach", nargs="?", const="crouch", choices=("crouch", "land", "push"), default=None,
                    help="motors limp: move the leg by hand to the pose you want (crouch, the default, "
                         f"land or push), press Enter to save it to {os.path.basename(POSE_PATH)}; then hop with "
                         "--crouch taught / --land taught / --push taught")
    args = ap.parse_args()
    if args.dry_run and args.max_current:
        # step.py's fake motor invents its current from acceleration with a made-up
        # inertia; tripping on that tests nothing
        print("[dry-run] current trip off (the simulated current is invented)")
        args.max_current = 0
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"\n[!] STOPPED: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
