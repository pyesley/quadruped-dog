# Leg test stand — notes for Claude Code

Read `README.md` (setup, how to run) and the top section of `NOTES.md` (current state, best settings,
open problems) before changing anything.

## Hard rule: Claude never changes the moteus-c1 configuration

The two moteus-c1 controller boards are configured **only by a human**. Claude is not allowed to change
their configuration in any way: no `conf set`, `conf write`, `moteus_tool --write-config`, `--zero-offset`
or `--calibrate`, no persistent register write, and no script or tool that does any of these for you.
This holds whoever is running the session (the owner or the owner's son) and however small or "obviously
safe" the change looks.

**Why:** a wrong position limit, zero offset, current limit or gain can drive the leg out of its mechanical
bounds or push more current through the motors than they can take. That damages the robot, and only the
person standing at the stand can judge the risk.

**What Claude may do:** read (`moteus_tool --dump-config`, register queries, `stop.py`), research values,
show the arithmetic and the risk, and hand over the exact command for the human to type. Then stop.

## Hardware facts
- Two moteus controllers on CAN via an mjbots fdcanusb (`/dev/ttyACM0`; `lsusb` mislabels it "candleLight",
  same VID:PID 1209:2323). Motor id 1 = KNEE, id 2 = HIP. 9:1 belts, 160 mm links, 15.4 mm foot.
- Controller config (applied by the owner): servo.max_current_A 12, max_voltage 31, flux-brake margin 3
  (brake 28 V), pid_position kp 0.5 kd 0.025, fault temperature 75 C (derating from 50 C).
- Power: 24 V bench supply (current knob >= 10 A) through an ODrive regen clamp (2 ohm; brakes ~2.5 V above
  the supply, ~26.4 V).
- Positions are absolute only within one motor revolution: the controllers must power up with the leg FOLDED
  (zero set there with `moteus_tool --zero-offset` on 2026-10-17). Taught poses are stored as joint angles.
- Pi "myesley" (user mdyesley): ADS131M08 force plate + string pot; `sudo ~/scale/plate_setup.sh` after every
  boot; dash.py starts `platelog` over ssh and fetches its CSVs with scp.

## Rules (from the owner)
- **Never write configuration to the moteus-c1 controllers** (see the hard rule at the top). Reading
  registers / `--dump-config` is fine.
- Never query the CAN bus while hop.py / step.py / monitor.py holds `/dev/ttyACM0`; use the dashboard telemetry.
- Don't add software trips that stop moves the controllers are built to handle (hop.py `--max-current` is off
  by default: a 14 A trip during the brake dropped the leg limp mid-air). The controllers protect themselves.
- Keep the code compact: one folder per subsystem, no stray helper scripts.
- Load cells: the owner is not worried about overloading them; don't keep warning.

## How hop.py works (2026-10-17)
- Poses are hip heights or taught joint angles (`--crouch/--land/--push taught`), turned into motor revs via
  `leg_config.json`.
- `--push-mode through` (best): the planned move aims PAST the lift-off pose so the leg is at full speed there;
  the knee encoder is checked one cycle ahead and at the lift-off pose the BRAKE phase (pure damping,
  kp_scale 0 / kd_scale 1) stops the airborne leg, then the soft LAND catches the body.
- The old trapezoid push ends on TRAJECTORY_COMPLETE (the *planned* setpoint), which cuts the stroke short
  when the 12 A-limited leg lags the plan.
- Record so far: 0.774 s flight (foot-off to touchdown). Next limits: the push velocity setting (start at 60),
  then bus voltage / back-EMF around 45–50 knee-motor rev/s.

## Next project
A MuJoCo model of the stand matched to the logs (actuator model with 12 A clamp, back-EMF limit, rotor
inertia x81 as `armature`, friction; moteus PD + planner + hop.py phases), then reuse the actuator model on the
quadruped. Needs SolidWorks mass properties and the motor data. See the end of NOTES.md's top section.
