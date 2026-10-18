# Leg test stand — session summary (updated 2026-10-17)

Two-DOF ODRI leg on a vertical slider over a 4-cell force plate. The near-term
goal is **hopping**: get the leg to hop, then sweep velocity/acceleration to find
how high it can go without over-currenting a motor or controller. §III-A of the
Solo/ODRI paper (F_z vs leg compression → stiffness) is still the reference
experiment underneath.

---

## ▶ RESUME HERE — 2026-10-17 evening (through push, record 0.774 s flight)

**Start next time with** (one line; vel 60 so the plan never caps the push):
```
python3 hop.py --crouch taught --land taught --push taught --push-mode through --push-vel 60 --push-accel 700 --max-push-vel 80 --land-torque 0.15 --land-kp 0.15 --telemetry 127.0.0.1:9871
```
Judge hops by **plate flight time** (foot-off -> touchdown), not the string pot.

**Results** (flight time): old trapezoid best 0.669 s (20:57) -> through push 0.725-0.749 s (21:47)
-> **0.774 / 0.765 s (21:54, record)**, landings 24-36 N. Logs: `logs/merged-20261017-205702.csv` (old best), `-213803` … `-215444.csv` (through push).

**hop.py changes tonight**
- Hip check `HIP_MIN = -60`: the taught crouch really has the thigh tilted UP (hip -31 deg, knee above the hip axle).
- `--teach push` / `--push taught` = the lift-off pose (height AND foot x). `PUSH_KNEE_MAX = 175` (knee may spin
  past straight -- user OK). Taught: crouch hip -31.2/knee 24.9 (65 mm), land 24.6/66.7 (189 mm),
  push 77.2/167.1 (331 mm, foot +35).
- `--max-push-accel` default 700 (was 300).
- **`--push-mode through`**: the planned move aims PAST the lift-off pose (far enough that it never starts
  slowing before it); every cycle the knee encoder is checked one cycle ahead and at the pose -> **BRAKE** =
  pure damping (kp_scale 0, kd_scale 1, target vel 0), then soft LAND once the knee is < 2 rev/s (max 0.15 s).
  Prints `[*] braked: knee overshot ... (peak knee ...)`. A full-gain position step as the brake rang (16 deg past,
  38 deg back) -> replaced by damping.
- **`--max-current` software trip OFF by default** (user: the controllers are designed for these spikes; the trip
  fired at 14.3 A in BRAKE and dropped the leg limp -> 277 N landing). Temp trip 70 C kept.
- Bug found in the old trapezoid push: it ends on TRAJECTORY_COMPLETE (the PLANNED setpoint), so with high
  accel the plan finished while the 12 A-limited leg lagged at knee ~100 deg and LAND cut the stroke -- why accel
  above 229 made hops lower. Through mode avoids it.

**Power:** supply CC knob was ~5 A -> bus collapsed to 6 V mid-push, both faulted 33 (undervoltage). Now
**10 A**; pushes draw ~215-225 W, bus 22-25.5 V. Consider 12-15 A. Regen reached 25.5-26.0 V on the fastest
pushes; the clamp (blue LED) brakes at ~26.4 V.

**What limits height now**
1. Last hops ran **vel 24.4** (user starts at 8 and presses +): the knee hits 40 rev/s, overtakes the plan, and
   the controller brakes it back to ~13 rev/s before lift-off. -> start at vel 60 (command above).
2. Next likely: **bus voltage / back-EMF** ~45-50 knee-motor rev/s (current already falling at 49 rev/s on 22 V).
   Fix = higher supply voltage + raise servo.max_voltage / flux brake (user applies config).

**Open problems**
- String pot `z_m` unreliable at hop speeds (147-160 mm while holding the 65 mm crouch; non-monotonic in pushes).
- Heights from joint angles read up to ~40 mm above the string pot after the folded `--zero-offset` (knee has no
  folded hard stop). Which is wrong is unresolved -- matters for the sim.
- Plate overload: user is not worried (cells look sturdy) -- don't keep warning.

**Next project: MuJoCo model of the stand** (plan agreed, not started): slider + hip + knee + foot sphere on a
force plate; actuator model with 12 A clamp, back-EMF limit, rotor inertia x81 as `armature`, friction; moteus
PD + planner + hop.py phases. System ID by replaying logged currents and comparing Fz / joints, then predict an
untried setting. User will bring SolidWorks mass properties (or SW2URDF export; weigh printed parts and override
mass) and the detailed motor data; measure carriage mass + slider friction; motors-off swing test for inertia
(I = m g d T^2 / 4 pi^2). Do it on this laptop next to the logs; fresh session on the big PC later for RL.

## ▶ 2026-10-17 — bench supply + regen clamp + taught crouch

- Power is now a **bench supply at 24 V** (no more LiPo) through an **ODrive regen clamp**
  with a **2 ohm** brake resistor. Clamp brakes when OUT rises ~2.35-2.5 V above IN (~26.4 V),
  releases within 0.5-0.75 V. moteus flux brake = max_voltage 31 - margin 4 = 27 V -> only
  ~0.5 V above the clamp; proposed margin 3 (flux brake 28 V) so the clamp acts first.
  Not applied by me (user applies controller config).
- **Taught crouch:** `python3 hop.py --teach --telemetry 127.0.0.1:9871` (motors limp, set the leg
  by hand, Enter saves joint angles to `taught_pose.json`, previous -> `.prev`), then
  `python3 hop.py --crouch taught ...`. The taught pose fixes crouch height AND foot x;
  `--offset` now sets only the push end. preflight now simulates the real straight-line-in-
  motor-space path (crouch x -> push x), so its numbers moved slightly.
- **Taught land too:** `--teach land` / `--land taught` (the pose the leg waits in, soft, before
  touchdown; foot x taken from the pose, was always 0). One file holds both poses. Still need
  crouch height < land height < push height. Flux brake margin set to 3 by the user (brake 28 V).
- **Re-zeroed 2026-10-17 ~20:00:** folded knee read -16 deg and sat only 0.13 motor rev from the
  +-0.5 rev power-up wrap (knee has no hard stop when folded; possibly also a belt skip from the
  10-04 knee strikes -- check belt). User ran `moteus_tool --zero-offset` on both with the leg folded
  (output.offset id1 -0.0406 -> -0.3853, id2 -0.0082 -> -0.0784); leg_config zero_rev now 0 / 0, so
  folding can be +-20 deg off at power-up. Old taught poses -> taught_pose.json.pre-zerooffset (invalid).
- SSH to the Pi by NAME (`mdyesley@myesley`); the raw-IP known_hosts entry is stale.

## ▶ 2026-10-04 (end of the first hopping night)

**State at wrap-up:** user stopping for the night. Battery 6S at **23.5 V resting (~3.92 V/cell,
~55-60 %)**, dipping to 22.5 V on pushes; started the night at 24.5 V. **Unplug the battery when not
in use** (controllers draw a few W idle). Stop at ~22.2 V resting; dashboard warns at 21.0 V.

**The leg hops.** `hop.py` (trapezoid push, the default) with the dashboard control panel.
Best settings so far (paste as ONE line -- the user's terminal has truncated long lines before):
```
python3 hop.py --telemetry 127.0.0.1:9871 --crouch 140 --push 305 --push-vel 16 --push-accel 229 \
    --land 210 --land-torque 0.15 --land-kp 0.15 --offset 20 --max-push-vel 20
```
Best result: cart apex **+256..270 mm above the crouch** (~120-128 mm rise after take-off, ~0.5 s in
the air), landings normally 14-18 N, compression ~60 mm.

**Controller config now (user-applied; verified by read-only dumps):** max_current_A **12** (was 9),
pid_position kp **0.5** kd **0.025** (both), max_voltage 31, flux_brake_margin 4, max_regen 50 W.
Gains per phase are per-COMMAND scales in hop.py -- nothing written to the controllers.

**What we learned (in order of importance):**
1. **Thermal derating is now the limit.** Motor 1 (KNEE) controller board reaches 52-55 C after a
   few hops and reports code **100** ("limiting: board temperature"); derating starts at
   fault_temperature 75 - temperature_margin 25 = **50 C**, cutting the knee to ~9-10.9 A. After
   cooling (48 C) it gave the full 12.4 A (code 99). Hip board stays ~45 C. => cool the knee
   controller (fan / better heat sink) and rest until < 45 C between sets. hop.py's per-hop line
   now flags "THROTTLED BY BOARD TEMPERATURE".
2. **Foot under the hip => the hip adds NO vertical force** (its lever to the ball is 0); the knee
   carries the whole push. --offset (ball ahead of the hip) shares it, but force x speed = power,
   so the gain was small (+3-5 % at 40 mm). Offset 20 + straighter push end (305 mm) was best.
3. **Straighter push end (305 vs 290 mm)** gave the best apex even while thermally derated.
4. 9 A -> 12 A: ~+40 % height. preflight() estimates are only a TREND at these levels (under-
   predicted 21 mm vs 85 mm measured once); judge by the recorded string-pot apex.
5. Soft landing = LAND torque cap (--land-torque) + bent landing pose (--land): energy model
   weight*(fall + d) = F_cap*d matched the measured compression (35 vs 40 mm).

**OPEN / next time:**
- **Knee strikes:** the 72-74 N (and 31-37 N) landing peaks in merged-20261004-013513 were the
  **KNEE hitting the plate**, not the foot (user observation). With the ball under the hip the knee
  sits L*sin(knee/2) above the ball, so it can only get that low if the thigh swings down -- most
  likely the hip yielding under the SOFT LAND gains (kp_scale 0.15 + torque cap apply to both
  joints) on a hard landing. Ideas for next time: separate, stiffer hip gains in LAND (only the knee
  needs to be the spring); a guard that flags/aborts when the knee height
  (hip height - L*sin(thigh angle)) gets near the floor; check the leg/knee for damage. The cells
  saw ~2x their ~39 N rating -> **re-check them with the precision weights** (platecal, 15 min).
- Velocity above 16 not yet tested at full (un-derated) current.
- Release push (--push-mode release) FAILED live (trembled, 9.6 A trip) -- leave it.
- Repeated/periodic hops: user declined (heat; hard floor).
- Platelog ADC register check works live (caught a GAIN loss ~00:10).
- Recordings before 2026-10-04 00:45 have motor columns shifted one left (fixed since).

**Programs:** dash.py (:8080, panel + IK + CoP + top view), hop.py (hop/drop, panel-controlled),
platecal/ (weights calibration :8091), stringcal/ (string pot :8090), step.py, monitor.py, stop.py.

## Session log 2026-10-03 (string pot, IK, CoP, plate cal)

**State when the user left for dinner:** only `dash.py` running (laptop :8080, it ssh-started
platelog on the Pi). step.py and monitor.py NOT running. Motors on **6S LiPo (24.5 V)**.
Pi needed `sudo ./plate_setup.sh` this boot (CLKIN was off) — done; redo after any Pi reboot.

**Done today**
- String pot: not broken (overlay unloaded after reboot). Re-calibrated with the sqrt model,
  encoder now 4x resolution. **Ruler readings were CENTIMETRES** (user confirmed after some
  back-and-forth; values stored x10). Pi `~/stringpot/` (cal.json, stringpot.py --y0, README).
  Laptop app `stringcal/stringcal.py` :8090. Details in "String pot" section at the bottom.
- dash.py: cart height from the calibration; "Zero z" -> **Cart is … mm** (needed once per
  platelog start, tile nags until done). Not yet set this run.
- Controller config READ (not written) and confirmed both ids: max_voltage 31,
  flux_brake_margin 4 (brake 27 V), max_regen_power_W 50, max_current_A 9. Fresh dumps in
  `config_backup/moteus-id{1,2}-20261003-*.cfg`. leg_config limits updated to match
  (min_voltage_V 21 = 3.5 V/cell).
- step.py ran fine at `--vel 5 --accel 5` (user ran it). Cart vs encoder leg length agree:
  leg = 0.975*cart + 36 mm, rms 1.5 mm.
- **Regen check:** at vel 5 / accel 5 electrical power (`pw`) min -0.10 W, bus flat 24.50 V ->
  no meaningful regen; back-driven mech power (<1 W) is all eaten by I^2R. Test = `pw` < 0.
- **IK panel** in dash.html (see "IK panel" section). Mirrored side view (foot to the LEFT),
  foot = **30.8 mm ball; 160 mm shin ends at its centre** -> foot_offset_mm = foot_radius_mm = 15.4
  (was 13.5). Ball drawn rolling with spokes + contact dot. Page has a red JS-error banner.

**OPEN thread (was mid-discussion): foot slip vs kinematics**
- Drawing shows the foot SLIDING but the real rubber foot only ROLLS. Pose1 -> pose2: FK ball
  centre x 48 -> 29 mm (-19 mm), shin angle +39.3 deg -> pure rolling allows only 15.4*0.686 =
  10.6 mm => ~8 mm unexplained per stroke. Leading suspect: **hip zero off by ~2.4 deg**; else
  knee offset, belt stretch/backlash (encoders are motor-side).
- Plan agreed in principle: (1) live "implied slip" readout = d(x_centre) + R*d(shin angle),
  should be ~0; fit hip/knee zero offsets over a sweep to null it. (2) **Centre of pressure**
  from the 4 cells (f0..f3 already in the feed): CoP x = contact point = under the ball centre
  -> independent foot-x; compare dCoP with R*d(shin angle) for roll vs slip. Plate gives Fz,
  Mx, My only (no Fx/Fy/Mz, so friction itself is not measurable).
- **Plate geometry from `~/Downloads/forcePlate.pdf`** (mm, origin = plate centre): plate
  200 x 70 x 10, four 30 x 16 mm bosses 4 mm proud underneath, one per cell, centred at
  **x = +-49, y = +-27** (bolt pairs at x = +-44 / +-54, 8 mm in from the long edges, 10 mm apart).
  => cell spacing **98 mm along the length, 54 mm across**.
  CoP x = 49*(F_right - F_left)/Fz, y = 27*(F_far - F_near)/Fz.
- STILL WAITING ON USER: is the leg's plane of motion along the 200 mm length? channel ->
  corner map (press test); where the hip line meets the plate.
- **Plate calibration tool BUILT (2026-10-03 night), not yet run:** `platecal/` -- template.pdf
  (print 100 %, 21 points: x -75,-49,-25,0,25,49,75 / y +18,0,-18; cells A B top, C D bottom),
  `python3 platecal/platecal.py` -> :8091 (needs dash.py; reads its new `/cells?sec=` endpoint).
  Plan = 35 loads (centre 5..600 g, 200 g at all 21, 400 g at 6 edge points), empty between each.
  Fits [Fz, Tx=Fz*x, Ty=Fz*y] = A @ df two ways: physical (drawing positions + 4 gains, tries all
  24 channel->corner maps) and general 3x4 constrained to ignore the + - - + twist (A u = 0);
  uses whichever has lower leave-one-out CoP error. Output `platecal/plate_matrix.json`.
  **RUN 2026-10-03 22:20, result `platecal/plate_matrix.json`:** leave-one-out Fz 1.1 g rms
  (0.42 % >=100 g), CoP 1.9 mm (x) / 2.7 mm (y) rms >=100 g. Channels ch0..3 = cells D,B,A,C.
  Old sum-of-cells cal reads 0.48 % high -> no overload damage from the 65.8 N peak.
  The plate is NOT 4 point supports: effective lever arms ~+-100 mm (x) / ~+-38 mm (y) vs pads at
  +-49 / +-27 -> moment goes partly through the bolted joints; drawing-based CoP would be ~2x off.
  One bogus capture (empty plate recorded as 400 g after the plan ended) is auto-excluded now,
  and the server rejects such captures.
- **CoP now in dash** (backups `dash.*.bak-pre-cop`): dash.py applies plate_matrix A to f0..f3 ->
  `cx, cy` (plate mm, None below 0.5 N) appended to stream rows [t,fz,z,lo,hi,cx,cy] and shot rows.
  Tile "CoP x, y", chart "CoP x/y" (+ dashed encoder contact once linked), IK row "contact, plate
  CoP" with CoP - encoders difference, cyan CoP diamond on the side-view floor. Buttons
  **Link CoP → leg** (stores leg.cop_hip_x_mm = plate x of the hip line) and **Flip CoP dir**
  (leg.cop_dir). Needs a fresh **Tare plate** with the plate empty first (read -0.29 N empty).
  Top-view plate graphic (outline, pads w/ channel, CoP dot ∝ load, 2 s trail, hip line + encoder
  contact ring once linked); preview with http://localhost:8080/?fakecop .
- **Gains (user-applied 2026-10-03 ~23:00):** knee kp 0.05 -> 0.5 confirmed by dump at 22:55;
  I then proposed kp 1.0 / kd 0.025 on BOTH (hip kd was 0.001 = undamped) and user reports
  "new PID settings": foot no longer pulls toward points 4/5, no wobble at end of push. Verify
  with a dump when the port is free. Per-phase hop softening via kp_scale/kd_scale/ilimit_scale
  per COMMAND (no config writes) -- table in conversation: CROUCH 1/1/1, PUSH 1/1/0,
  FLIGHT .5/1/0, LAND .25/1/0.
- **ADC lost its PGA gain (64 -> 1) mid-run, ~23:00, right after the stiffer gains** -> plate read a
  constant ~5.15 N and ignored all load (each cell = -zero*gain exactly). I first misread that as
  the leg's weight -- WRONG, the leg was off the plate. Restarting platelog (dash /restart-plate)
  restored it: empty cells 5.19 -4.11 3.62 -4.98 N (matches the calibration empties). Same failure
  family as the 4 kSPS (OSR) reset -> motor EMI resetting/corrupting ADS131M08 registers.
  **FIXED 23:10:** platelog now reads back GAIN1/GAIN2/CLOCK every 1 s (`--reg-check S`, 0 = off),
  rewrites any that changed, logs `[t] ADC registers lost (#n)` to logs/platelog.log and
  `# regfix ...` into recordings, and sends the running count as a 15th field of every P row (and
  in PONG). dash shows a red warning for 60 s then a note, prints to its console, and one-shots
  get `adc_regfix: true`. Costs ~2 samples per check (not counted as gaps). Tested on hardware
  with `PLATELOG_TEST_CORRUPT=3` (forces gain 1 once): caught + rewritten 2 ms later, no false
  positives. Backups `~/scale/platelog{,.cpp}.bak-pre-regcheck`, `dash.py.bak-pre-regfix`.
  Root-cause option (not done): enable the ADC's input CRC (MODE RX_CRC_EN) so EMI-garbled SPI
  commands are rejected instead of executed -- needs a CRC word on every command we send.
  Quick check meanwhile: empty-plate cells near 5.2/-4.1/3.6/-5.0 N; if they read 11.0/-2.4/-7.2/3.8
  the gain is gone. Moving mass still UNKNOWN (not 525 g).
  NEXT: user links it, steps the leg, and we read CoP - encoders over the stroke (the slip test). Cells currently carry ~+-4-5 N of self-balanced
  twist preload (sum ~0) -> the over-constraint is real; twist-blind A matters.
- Peak Fz 65.8 N was seen while stepping: above the 4 x 1 kg (39 N) rating -- check for damage.
- (old) cell bolt spacing front-back (plane of motion) and side-side in mm;
  which channel is which corner (do it live: user presses each corner, watch f0..f3);
  optionally where the hip's vertical line meets the plate. `~/scale/server.py` has only an
  unverified unit-square LAYOUT guess. Caveat: plate is over-constrained (cells re-seat) ->
  CoP may need shims.
- Also offered, not built: regen tile / power chart (negative W, energy returned per move).

**Restart after a crash**
```
ssh mdyesley@myesley 'cd ~/scale && sudo ./plate_setup.sh'   # only if the Pi rebooted
cd ~/testRobot && python3 dash.py                            # then "Cart is … mm" once
python3 monitor.py --telemetry 127.0.0.1:9871                # read-only, OR step.py (not both)
```

## ⚠ Ground rules (do not violate)

1. **Never write config to the moteus controllers.** No `conf set`, `conf write`,
   or `moteus_tool --write-config`. The user applies every controller change
   manually. Propose the exact commands and stop. Reading (`--dump-config`,
   queries, `stop.py`) is fine.
2. **Only one program can own `/dev/ttyACM0`.** Never query the CAN bus while
   `step.py` or `monitor.py` is running — doing so wedged a live run this session.
   Read controller state from the dashboard's telemetry instead.
3. The user runs the motors. Offer commands; don't start motion.

---

## Hardware, as established this session

| | |
|---|---|
| Controllers | 2× **moteus-c1**, hardware family 2 (confirmed by `servo.derate_current_A = -3`) |
| **id 1 = KNEE** (distal) | **id 2 = HIP** (root). The order matters for kinematics; ids do not. |
| Adapter | **mjbots fdcanusb** on `/dev/ttyACM0`. `lsusb` misnames it "bytewerk candleLight" (shared VID:PID 1209:2323) and it also exposes a gs_usb `can0` — ignore `can0`, moteus uses the serial port. Confirm with `udevadm info -q property -n /dev/ttyACM0 \| grep ID_MODEL`. |
| Actuator | ODRI actuator module v1.1: **dual-stage 3:1 AT3 belt = 9:1**, **160 mm** segments, 2.5 N·m at 12 A at the output |
| Motor | **T-Motor MN4004 KV300 Antigravity**, 18N24P, 452 mΩ line-to-line, idle 0.2 A, **9 A / 216 W for 180 s** (a 3-minute rating, *not* continuous) |
| Controller rating | 10–51 V, 20 A peak, 5 A continuous bare / **14 A with thermal management — the user HAS a heat sink**, so the controller is not the bottleneck |
| Bus | bench TP3030E (30 V/30 A switcher, **cannot sink**). Final robot: **2× 3S LiPo in series = 6S, 2200 mAh, 48.8 Wh** |

Config backups of both controllers: `config_backup/moteus-id{1,2}-20260926-220128.cfg`.

### Key controller config (custom vs firmware default)

```
servo.max_voltage               25     <-- CUSTOM. Family-2 default is 54.
servo.flux_brake_margin_voltage  3     <-- default
  => flux brake point = 25 - 3 = 22.0 V   (derived, not a parameter)
servo.max_current_A              9     <-- CUSTOM (default 20); matches the motor's 180 s peak
servo.fault_temperature         75     <-- default
servo.motor_fault_temperature  nan     <-- NO motor thermistor; winding heat is invisible
servo.timeout_mode              12     = ZERO_VELOCITY (see "watchdog" below)
motor.Kv 306.5  motor.poles 24  motor.resistance_ohm 0.286  rotor_to_output_ratio 1
```

---

## 2026-10-03: now on 6S LiPo (2x 3S), user set per controller
servo.max_voltage 31, flux_brake_margin_voltage 4 (brake 27 V), max_regen_power_W 50.
Full pack 25.2 V is 1.8 V under the brake point -- the 22 V collision below is resolved.
Dashboard limits updated to match (min_voltage_V 21 = 3.5 V/cell warning).

## (resolved) THE headline problem: the 22 V collision

`servo.max_voltage = 25` puts the flux-brake point at exactly **22.0 V**, which is
exactly **6S nominal (22.2 V)** — the robot's intended operating voltage. Whoever
set 25 probably reasoned "6S tops out at 25.2 V"; but `max_voltage` is the *fault*
threshold, so that's backwards.

```
6S LiPo:  full 25.20 V  <-- ABOVE servo.max_voltage: would fault on connect
          storage 22.80 V  }  both above the 22 V brake point:
          nominal 22.20 V  }  flux braking would be on essentially always
          "land now" 21.00 V
```

**Proposed (user to apply manually):**
```
conf set servo.max_voltage 27.0
conf set servo.flux_brake_margin_voltage 1.0      # brake point becomes 26.0
conf write
```
Full pack 25.2 V then sits 0.8 V below the brake point and 1.8 V below the fault,
and the same setting also covers the bench supply — no footgun when swapping.
**Check first:** what input voltage does the mjpower-ss tolerate? Its reference
doesn't say, and it's on the same bus.

Do this per controller: `python3 -m moteus.moteus_tool --target 1 --console`.

---

## What we diagnosed

**Fault 104 = `kLimitFluxBraking`** — and it is **not a hard fault**. Codes 96–105
are "something is limiting the output right now" while the controller keeps
running (mode stays 10 = POSITION). Confirmed by dropping the bench supply to
**18 V**: fault 104 disappeared. Diagnosis closed.

It was motor 1 (knee) because the knee swings 158° between poses vs the hip's 74°,
so it decelerates hardest and regenerates most.

**The ~10-minute failure** (unreproduced since). Two candidates, both driven by the
same root cause:
- **over-voltage (34)** — intermittent, one deceleration overshoots 25 V
- **over-temperature (38)** — reproducible timing; flux-braking D-axis current
  flows through the FET bridge, so it heats the *board* as well as the windings.
  Board went 31 → 48 °C during flux-braked running.

At 18 V both should be gone. **Not yet verified over 10 minutes.**

**Where regen energy goes** on a non-sinking supply: bus capacitance (temporarily,
and the voltage rises hard because C is small) then heat, via flux braking, in the
windings *and* the FETs. Nowhere else. Numbers for this leg: ~135 mJ mechanical per
deceleration; if ~30% reaches the bus, 100 µF → 36 V, 1000 µF → 23.8 V. **A single
1000 µF / 50 V low-ESR cap across the bus would change the behaviour completely**
(~580 µF needed to stay inside 22→25 V). On a battery this mostly vanishes — the
pack absorbs at near-constant voltage.

**Power / runtime**, measured from the recorded stepping run:
```
mechanical (|tau*w|)                    0.51 W
winding I2R                             4.06 W
controller quiescent (ESTIMATE 1.5W x2) 3.00 W   <- now measurable via m*_pw columns
TOTAL                                   7.57 W   -> ~5 h on 6S 2200 mAh at 80% DoD
```
Only 7% of the energy does mechanical work; the rest is the hip holding the leg up
against gravity (2.5 A continuous, 56% duty). A gravity-balanced rest pose or a
brake would matter far more than a bigger battery.

---

## Angle / kinematics convention (the user's, use it)

Press **Zero angles** with the leg **FOLDED** (the pose it collapses into at
power-off). Then:

- folded → **hip 0°, knee 0°**
- fully extended → **hip 90°, knee 180°**
- The knee reading *is the included angle between the links*, so
  **height above the floor = 2 · 160 · sin(knee/2) + 13.5 mm**
  (13.5 mm is the cylinder on the foot; the links *do* fold flat because hip and
  knee run in parallel planes, so `knee_offset_deg = 0` is correct)

```
folded    hip   0.0  knee   0.0  ->   13.5 mm
          hip  45.0  knee  90.0  ->  239.8 mm
extended  hip  90.0  knee 180.0  ->  333.5 mm
```

Earlier confusion (now fixed) came from my using `2L·cos(knee/2)`, which assumes
knee=0 means *straight* — the opposite of this zero. The "Leg straight = 0°" button
has been removed for the same reason.

**Still to verify:** extend the leg by hand and check the angles go **positive**
(hip → +90, knee → +180). If they go negative, flip `"sign": 1` → `-1` on that
joint in `leg_config.json`. Length uses the magnitude so it reads right either way,
but `step.py`'s poses only map into the physical 0–180° knee range with the knee
**sign = -1**, which suggests it needs flipping.

`step.py`'s poses in this frame: pose 1 = knee +46.6°/hip +1.0° (**140 mm**),
pose 2 = knee +158.4°/hip +74.5° (**328 mm**).

---

## OPEN: the force plate ADC resets itself to 4 kSPS

`platelog` was started, measured a clean **1000 SPS**, and later — **same pid, never
restarted** — was running at **4000 SPS**. It is right now (400 Hz feed at div=10).
The ADS131M08 is losing its CLOCK register and falling back to its power-on default
OSR of 1024. Timeline points at motor activity (1000 SPS idle, 4000 SPS after
running the motors), so suspect EMI into the SPI bus or the chip's reset.

Why it matters: `platelog`'s `period` is computed from the assumed OSR, so gap
detection and the `expected_from_clock` CSV footer are wrong by 4×, and real
dropouts stop being counted. **The whole 2026-09-26 recording was at 4 kSPS.**

Workaround: restart platelog and re-check.
```
ssh mdyesley@myesley 'pkill platelog; sleep 1; cd ~/scale && ./plate_start.sh 192.168.86.34'
```
A one-shot's `plate_sps` stat is the quick check (1000 good, 4000 bad).

**Proposed fix (not yet done):** have `platelog` read the CLOCK register back
periodically, re-assert it, and log when it changed — fixes it *and* measures how
often it happens.

---

## What runs where

```
 Pi "myesley" (192.168.86.54, mdyesley)          Laptop (192.168.86.34)
 ADS131M08 + 4x 1 kg cells ─┐                    testRobot/dash.py :8080  <- browser
 string pot (quadrature,    │ ~/scale/platelog ──UDP──> :9870
   GPIO5/6) ────────────────┤   (C++, 1 kSPS)    step.py / monitor.py --telemetry :9871
                            └─ full-rate CSV     logs/merged-*.csv, shot-*.csv
```

Once per Pi boot (needs sudo): `ssh mdyesley@myesley 'cd ~/scale && sudo ./plate_setup.sh'`

### Programs (laptop `~/testRobot/`)

| file | role |
|---|---|
| `dash.py` + `dash.html` | Two-mode scope on :8080. Backups `*.bak-pre-scope`. |
| `step.py` | The motion program. Backups `*.bak-pre-current`, `*.bak-pre-telemetry`. |
| `monitor.py` | **Read-only**: sends one stop, then only queries. Motors stay mode 0 and free to move by hand. Use for calibration and for checking where the leg is. |
| `stop.py` | Panic stop. Sends a real stop and reads the mode back to prove it took. |
| `leg_config.json` | gear ratios, zeros, signs, leg geometry, limits the UI colours against |
| `.idea/runConfigurations/` | PyCharm configs 0–4, all with `EMULATE_TERMINAL=true` |

### The dashboard

**Continuous** — ~100 Hz decimated feed with a per-bin min/max band.
**One-shot** — sends `DIV 1` to platelog (every sample, ~15 packed per datagram),
freezes a pre/post window on a trigger, saves `logs/shot-*.csv`. Triggers: manual,
Fz rise/fall/abs, `motor_move` (step.py leaving a hold), `vel`.

Charts: Fz · z + leg length · joint angles · motor velocity · torque · phase
current (warn + `max_current_A` lines) · **bus voltage (flux-brake and
`max_voltage` lines always in view)**.

Tiles include **motor mode** (STOPPED / POSITION / FAULT) — the reliable answer to
"are the motors live?". When mode is STOPPED the current tiles dim, because the
q/d readings are then just **current-sense offset** (0.1–0.3 A, drifts with
temperature — motor 1 went −0.12 A at 31 °C to +0.22 A at 42 °C). Torque and power
read exactly 0.0 when truly de-energised.

Buttons: Record · Tare plate · Zero z · Zero angles · **Leg is … mm** (tape-measure
calibration, sets `leg.knee_offset_deg`) · window · Pause.

Measured: **1001 Hz over WiFi, 0 samples lost in 25 s, ±0.5 ms plate↔laptop clock
jitter.** `plate_off` in `/status` is a raw ~1.79e9 unix offset; the useful figure
is `plate_off_spread` (the jitter), which is what the UI shows.

### Start-up

```bash
cd ~/testRobot
python3 dash.py                                  # ssh-starts platelog on the Pi
python3 monitor.py --telemetry 127.0.0.1:9871    # read-only, motors free
python3 step.py --telemetry 127.0.0.1:9871 --vel 5 --accel 5 --dwell 2   # LIVE
python3 stop.py                                  # make safe
```
A login shell already resolves `python3` to the conda base with `moteus`; no
activation needed. **Stop the monitor before running step.py** (one port).

### Watchdog behaviour

If `step.py` is hard-killed (PyCharm Stop pressed twice, crash, wedge), its
`finally` never runs, and after `servo.default_timeout_s = 0.1` the controllers
enter `timeout_mode 12` = **ZERO_VELOCITY**. They stay *energised*. At rest that
commands almost no torque so it **feels** limp — "feels free" is NOT a test for
de-energised. Use the mode tile, or run `stop.py`.

---

## Logging

`merged-<ts>.csv` (Record) is now **37 columns**: 14 plate, then per motor
`pos,vel,tq,q,d,pw,volt,temp,mode,flt`, then `m1_cmd,m2_cmd,state`. Earlier runs
(including 2026-09-26) only had pos/vel/tq — **no bus voltage, temperature,
current, mode or fault**, which is why the 10-minute question is still open.

`shot-<ts>-<nnn>.csv` — `t_ms` relative to trigger, P and M rows interleaved, four
`#` header lines carrying the trigger, clock offset and stats JSON.

Shot stats: peak Fz and when, baseline, impulse, flight time, hop height from
flight, z rise, leg travel, bus max/min, peak current/velocity/torque per motor,
`plate_lost`, measured `plate_sps`.

---

## Next steps

1. **Re-check the plate rate** (one-shot → `plate_sps`). Restart platelog if 4000.
2. **Apply the voltage config** (`max_voltage 27`, `margin 1.0`) after checking the
   mjpower-ss rating. Then the bench can run at 22 V instead of 18.
3. **Run 10+ minutes at 18 V with Record on** and settle the failure question from
   the `m*_temp` / `m*_volt` / `m*_flt` columns.
4. **Verify the knee sign** by extending the leg by hand (see kinematics above).
5. **Convert `step.py`'s poses to joint degrees** relative to `zero_rev`, so they
   survive a controller power cycle. Currently absolute motor revolutions, valid
   only within one controller power session (position re-reads from the AS5047P and
   wraps into ±0.5 rev at every controller boot = 40° at the joint).
6. **Raise `step.py --rate`** before hops: 100 Hz gives ~10 motor samples across a
   100 ms push-off. The fdcanusb should manage 300–500 Hz for two controllers.
7. First hop: arm one-shot on `fz_rise` above the standing load; check
   `flight_ms` / `hop_from_flight_mm` against `leg_travel_mm`.
8. **String pot fixed + recalibrated 2026-10-03** (it was never broken: the encoder
   overlay just wasn't loaded after a reboot). See "String pot" below. dash.py now computes
   cart height from `stringcal/stringpot_cal.json`; "Zero z" became **Cart is … mm** (once per
   platelog start). platelog's own on-Pi CSV `z_m` column is still the old linear value.
9. Open questions: does the **power_dist pass reverse current** (decides whether
   the battery can sink regen at all)? Is `servo.max_current_A` **amplitude or
   RMS**? If T-Motor's 9 A is RMS, 12.7 A amplitude is available — worth ~40% more
   push-off torque, but **do not raise it** until (a) you see fault **99**
   (`kLimitMaxCurrent`) proving the limit binds, and (b) the units are confirmed
   with mjbots or a clamp meter. There is no motor thermistor to catch a mistake.
10. Consider a **motor thermistor** + `servo.motor_fault_temperature` before
    pushing hard, and a **1000 µF bus cap** if staying on the bench supply.

## Force plate calibration (2026-09-20, unchanged)

−454.2 counts/g summed at gain 64; per-cell g/count ch0 −0.002177, ch1 −0.002210,
ch2 −0.002240, ch3 −0.002280. Linearity 0.35% over 5–510 g, noise 0.018 N per 1 ms
sample. Calibrated only to 510 g but used to ~30 N — one check point at 1–2 kg
would close the extrapolation. Trust the **sum**, not individual cells (the
4-point mount is over-constrained).

Note: during the 2026-09-26 run the plate read a steady **5.05–5.40 N** — that was
the leg's ~530 g resting on it, and it varied only 0.35 N across a 111° knee swing,
i.e. the foot was not really working against the plate. Empty it reads ≈ −0.09 N.

## hop.py (2026-10-03 night) -- written, dry-run tested, NOT yet run live
CROUCH -> PUSH -> LAND, one hop per key h. Poses are hip HEIGHTS via IK (foot under hip, hip =
knee/2) -> motor revs through leg_config; open signs hip +1 / knee -1 (from step.py pose 2).
Refuses to move if measured angles are impossible (unreferenced power-up); asks y/N first.
Per-COMMAND gains only (no config writes): GO crouch kp ramps .25->1 ilimit 0; HOLD 1/1/1;
MOVE push 1/1/0; LAND kp .25, maximum_torque 0.2 N.m motor, ilimit 0, fast retract (land-vel 10).
Only the push is reported as "MOVE ..." so dash one-shot "motor_move" catches each hop.
Preflight (kinematics+gravity): accel decides WHETHER it leaves (~>90 rev/s^2 knee motor),
velocity decides HOW HIGH (accel 200: vel 3 ~1.5 mm, 5 ~4.5, 8 ~13, 10 ~22 mm, upper bounds).
Keys + / - velocity, ] / [ accel (caps --max-push-vel 10, --max-push-accel 300), a auto, q quit.
Defaults crouch 180 / land 250 / push 290 mm, vel 3, accel 60 (= push test, no flight), 200 Hz,
current trip 9.5 A (controllers clamp at 9). PyCharm configs "5 hop DRY RUN", "6 hop LIVE".
**--style drop** ("the chair breaks", landing test, no push-off): HOLD stand (290) -> MOVE snap
(fold by --drop mm at accel 300 / vel 15, stiff) -> LAND soft at the folded pose, switching to soft
HALF-WAY through the fold (the leg's own braking brings the foot down before the fold ends) -> GO
stand. Preflight (free-fall sim): fold 5/10/20/30/40 mm -> foot lifts 1.4/2.8/5.9/9.3/12.8 mm,
off the plate 32/45/64/78/90 ms, impact 0.28/0.40/0.60/0.76/0.89 m/s; needs >~150 rev/s^2 to lift.
Keys +/- fold 5 mm (cap --max-drop 40). PyCharm "7 drop LIVE (10 mm)". Plan: drops 10, 20, 30 mm
first (watch peak Fz on landing), then real hops.
Ramp plan: push test -> ] to ~200 (barely leaves) -> + one step at a time watching peak Fz
(1 kg cells, ~39 N rated; 65.8 N already seen) and per-hop current/regen summary.

**Control panel (2026-10-04):** dashboard shows a "hop.py" panel above the tiles: when hop.py
is not running it lists the start commands; when running (with --telemetry) it shows style,
phase, count, the preflight text, last result, and buttons DROP/HOP (enabled only in HOLD),
fold or velocity -/+, accel -/+, auto, Stop & quit. Buttons POST /hopkey -> dash sends the same
single-key commands by UDP to hop.py on 127.0.0.1:9872 (--control-port, 0 = off). hop.py status
rides in the telemetry packet as "hop" (step._Telemetry.send got an `extra=` arg). Starting
hop.py + its y/N stays in the terminal on purpose. Backups *.bak-pre-hoppanel / -pre-panel /
step.py.bak-pre-hopctl.

**First drops (2026-10-04 00:03-00:08), up to 40 mm:** landing peak Fz 19.7-22.4 N (cells ~39 N
rated), flight 108-123 ms (preflight said 90 ms), currents unremarkable per user. First LIVE
platelog register repair seen ~00:10 (worked). dash one-shot flight finder fixed: searches the
whole window (landing is usually the biggest force), resting load >= 1.5 N (was 5), and reports
push_peak_N / land_peak_N / flight_start_ms. Backup dash.py.bak-pre-flight. NEXT: real hops.

**Release push (2026-10-04), `--push-mode release`:** first version (trapezoid aimed past the end,
let go at 290) gained NOTHING -- a trapezoid must slow before its target and the leg's range is too
short; with a velocity limit later release is even slightly worse (leverage shrinks as the leg
straightens). Replaced by a TORQUE-CAPPED push: target steps to --overshoot (315 mm = knee 139,
also the extension bound if the release were missed), kd_scale 0, maximum_torque = --push-torque
(N.m at the motor); released on the ENCODER height each cycle at --push (290) -> RETRACT (stiff,
--retract-accel 400) -> LAND soft once 30 % back / 150 ms. Force grows as the leg straightens, so
it is abrupt: 140 mm crouch needs >0.065 N.m to lift and gives ~82 mm at 0.08. Hence release
mode defaults to --crouch 270 (20 mm stroke): torque 0.05/0.06/0.07/0.08/0.10/0.12 -> ~6/11/16/21/
31/41 mm (upper bounds, 0.6 kg). Keys/panel: +/- torque x1.25 (cap --max-push-torque 0.2),
]/[ stroke = crouch 10 mm deeper/shallower (moves there at once). The lift threshold measures the
real moving mass: m = 1.5*9*tau / (L cos(knee/2) * g). Backup hop.py.bak-pre-release.
**LIVE RESULT: FAILED.** User: "it just trembles and errors out ... we were doing much better with
the last approach." Trembled (kd_scale 0 during the push -> undamped chatter against a load it
could not lift) and tripped at 9.6 A in RETRACT (push stalled, timeout went to the stiff fast
retract with the weight still on the leg). Fixed the stall path (-> soft LAND); kd-0 issue NOT
fixed. Release mode marked experimental; user is back on the trapezoid push (default).

**First real hops (2026-10-04 ~00:35, trapezoid, crouch 140, vel 10, accel 229), merged-20261004-003456.csv:**
18 pushes; best: ~370 ms in the air, cart +210 mm from crouch (~127 mm push + ~80 mm flight above
take-off), leg at touchdown 245 mm (already in the LAND pose), ~40 mm compression, landing peak
15-23 N (body ~5 N), LAND torque ~0.17 N.m = near the 0.2 cap -> catch is torque-capped:
weight*(fall+d) = F_cap*d predicts d ~35 mm, measured ~40. User: landing feels hard, wants more
fold in the air + springier. Proposed: --land 230 --land-torque 0.13 --land-kp 0.15 (predict ~15 N,
~60 mm compression). merged CSV bug FIXED (M rows were shifted one column left: ","*10 -> ","*11);
files before 2026-10-04 00:45 need positional parsing of M rows (state = col 35, motor 1 = 13..22).

**2026-10-04 ~01:00 -- 12 A + 500 Hz.** User set servo.max_current_A 12 on both (dump confirms;
also kp 0.5 / kd 0.025, NOT the kp 1.0 proposed -> LAND kp_scale 0.15 = ~85 N/m vertical).
Soft-landing run (merged-20261004-004526.csv, --land 230 --land-torque 0.13 --land-kp 0.15): 3 hops,
~85 mm above take-off, compression ~60 mm (as predicted), land peak 19-20 N, push current 9.2-9.4 A
= AT the old 9 A clamp (and 0.1 A from the old 9.5 A trip); knee motor reached 14 rev/s with the
limit at 10 (leg lags, then over-speeds). preflight under-predicts badly at this level (21 mm vs 85).
Catch torque read 0.165 N.m vs the 0.13 cap -- unexplained. Re-read arXiv 1910.00093: same actuator,
0.225 N.m/A joint, 12 A -> 2.7 N.m, 1 kHz PC loop, ~0.65 m jump with joint POSITION control
(periodic), landing 150 N/m, unsprung-mass impact spike first 50 ms. User REJECTED repeated hops
(heat at 12 A; hard floor, no spring). hop.py --rate default now 100 (user's choice: the PC rate only sets phase-switch latency and data resolution; moteus closes its loops internally) (fdcanusb measured: 500 Hz
sustained, CAN cycle 1.0-1.4 ms; 1000 Hz no margin), --max-current 12.5, I_MAX 12. dash limits
max 12 / warn 10. Backup hop.py.bak-pre-12A.

**12 A hops (merged-20261004-010511.csv):** knee at the 12 A clamp every push (12.2-12.5 A, fault
99 = limiting), hip only 4-8 A -> KNEE is the bottleneck; knee speed tops at ~14.3 rev/s whatever
the vel setting. Rise above take-off 78/104/119/119 mm (vs ~85 at 9 A, +40 %), cart +240 mm from
crouch, compression 60-68 mm, landing 17-22.5 N, boards 43/38 C, bus 23.0-23.5 V. 5th push: hop.py
TRIPPED at 12.6 A (trip was 12.5) mid-push at leg 204 mm -> motors stopped, leg fell unpowered.
Trip default now 14 A. Idea (not done): load the hip more (it has headroom) to add push force.

**--offset (2026-10-04):** foot under the hip => hip torque contributes NOTHING to vertical force
(hip->ball horizontal arm = 0); knee carries all of it (arm = knee_x = L cos(hip)). My old
preflight 1:1/2 split was WRONG (fixed: tau_hip = F|x|, tau_knee = F|x - knee_x|). --offset X puts
the ball X mm ahead of the hip (toward the knee) in crouch + push; landing stays under the hip.
Quasi-static max force at 12 A, offset 0/30/40/50: crouch 19.8/22.8/24.5/26.5 N, 290 mm 35.5/44.6/
49.5/56.1 N; foot drifts 0/6.5/8.5/10.2 mm mid-push (linear motor-space path). Peak-push torque
(vel 14, accel 229): offset 0 knee 2.95 hip 0 (> 2.9 limit -> matches the 12 A saturation),
offset 40 knee 2.45 hip 0.82. General IK: hip = atan2(y, x) - 90 + knee/2. Backup hop.py.bak-pre-offset.

**Straighter push + offsets (01:16-01:28):** apex above crouch: 12 A/offset 0 ~240; offset 40 248-251;
offset 60 256-257; **push 305 + offset 20 + land 210: 256-270 (best)**, landing 14-18 N, leg compresses to
~145 mm (near the 140 crouch -> raise --land-torque for higher hops). From 01:25 motor 1 (knee) reports
code 100 = THERMAL DERATING (board 52-54 C; derate starts at fault 75 - temperature_margin 25 = 50 C) ->
knee only 9.4-10.9 A. I first misread that as "off the current limit / speed-limited" -- WRONG. Hip
board 45-46 C. hop.py per-hop summary now reports board temp and flags code 100. Plan: cool to <45 C
(fan/heat sink on the knee controller), then try --max-push-vel 20 while < 50 C.

## IK panel (2026-10-03)
dash.html, between the tiles and the charts: side-view drawing (measured leg solid, IK target dashed),
target hip height slider -> hip/knee deg + motor revs (knee: knee = 2 asin((h-13.5)/320), hip = knee/2),
foot = 30.8 mm ball, 160 mm shin ends at its CENTRE (foot_offset_mm = foot_radius_mm = 15.4,
was 13.5); drawn rolling, contact straight below the ball centre. "from string pot" row once "Link cart -> leg" stores leg.cart_to_hip_mm. step.py pose 2 (hip 73.8 /
knee 158.7) puts the foot ~30 mm AHEAD of the hip; IK wants hip 79.3 there. Backup dash.html.bak-pre-ik.

## String pot (2026-10-03)

Pi `~/stringpot/`: `cal.json`, `stringpot.py` (`--y0 <ruler mm>` -> `t count y_mm dy_mm`;
`--raw` -> counts; or `from stringpot import StringPot`), `README.txt`.
Encoder now counts every quadrature edge (`steps-per-period=4`, set in `plate_setup.sh`).
Model `y = sqrt(alpha*(n+N0)^2 + k)`: alpha 2.041e-3 mm^2/count^2 (0.0452 mm/count),
k 4210 +- 860 mm^2, rms 1.1 mm over 130-570 mm, 25 points (4 dropped: cart still moving).
**The ruler was read in whole CENTIMETRES** (values x10 in the cal). Verified against leg FK
while stepping: leg = 0.975*cart + 36 mm, rms 1.5 mm. A finer re-cal in mm would sharpen k.
N0 depends on where counting started -> give `--y0` at start. Straight line would be 0.15 mm rms.
Laptop re-cal app: `python3 stringcal/stringcal.py` -> http://localhost:8090.
