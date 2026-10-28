# Leg test stand — software

Software for the 2-DOF ODRI-style leg on the vertical slider, hopping on a 4-cell force plate.

```
 Raspberry Pi "myesley"                      Laptop (Linux / macOS)
 ┌──────────────────────────────┐   WiFi    ┌───────────────────────────────────────────┐
 │ ADS131M08 ADC + 4 load cells │  UDP 9870 │ dash.py  -> http://localhost:8080 (scope)  │
 │ string pot (GPIO5/6 encoder) │ ────────► │ hop.py / step.py / monitor.py / stop.py    │
 │ pi/scale/platelog (C++)      │ ◄──────── │   └─ USB fdcanusb ─ CAN ─ moteus id1 knee │
 └──────────────────────────────┘  ssh/scp  │                           moteus id2 hip  │
                                            └───────────────────────────────────────────┘
```

| Folder | What |
|---|---|
| `laptop/` | Everything that runs on the laptop: dashboard (`dash.py` + `dash.html`), hop controller (`hop.py`), motion/telemetry library (`step.py`), `monitor.py` (read-only, motors limp), `stop.py` (panic stop), `jog.py`, CAN test tools, `leg_config.json` (gear ratios, zeros, leg geometry, limits), `taught_pose.json` (taught crouch / land / push poses), `stringcal/` + `platecal/` calibration apps, `config_backup/` (moteus config dumps) |
| `pi/scale/` | Pi side of the force plate: `platelog.cpp` (1 kSPS logger + UDP feed), `plate_setup.sh` (per-boot bring-up), `plate_start.sh`, `clkin.py`, `ads131.py`, `calibrate.py`, `scale_cal.json` (load-cell calibration) |
| `pi/stringpot/` | String pot (carriage height) reader + calibration |
| `NOTES.md` | The full lab notebook: every session, what was learned, current best settings, open problems. **Read the top section first.** |
| `CLAUDE.md` | Context and rules for Claude Code if you use it in this folder |

## Setting up a second laptop

The Pi stays as it is (`~/scale`, `~/stringpot` on the Pi are the deployed copies of `pi/`).
Only **one laptop at a time** can drive the stand: there is one USB-CAN adapter, and the Pi sends its
plate feed to whichever laptop started the dashboard.

1. **OS:** Linux or macOS (the code uses `termios` and `ip route`). Windows would need WSL plus USB
   pass-through for the CAN adapter — not tested.
2. **Code + Python packages**
   ```bash
   git clone https://github.com/pyesley/quadruped-dog.git
   cd quadruped-dog/testStand/software/laptop
   pip install -r requirements.txt
   ```
3. **CAN adapter access** (Linux): `sudo usermod -aG dialout $USER`, log out and back in.
   The mjbots fdcanusb appears as `/dev/ttyACM0`. `lsusb` calls it "bytewerk candleLight" (same USB
   id) — that is fine; check with `udevadm info -q property -n /dev/ttyACM0 | grep ID_MODEL` → `fdcanusb`.
4. **Same WiFi as the Pi**, and passwordless ssh to it (the dashboard starts the Pi logger and fetches
   recordings over ssh/scp):
   ```bash
   ssh-keygen -t ed25519          # if you have no key yet
   ssh-copy-id mdyesley@myesley   # asks for the Pi password once (ask Dad)
   ssh mdyesley@myesley true      # must work without a password now
   ```
5. **Each time the Pi has rebooted:** `ssh -t mdyesley@myesley 'sudo ~/scale/plate_setup.sh'`
   (starts the ADC clock and the string-pot decoder; neither survives a reboot).

## Running a session

1. **Power:** bench supply 24 V with the **current-limit knob at ≥ 10 A** (at ~5 A a big push collapses
   the bus and both controllers fault with undervoltage). The ODrive regen clamp sits between supply and
   controllers.
2. **Power the controllers up with the leg FOLDED.** Positions are only absolute within one motor turn;
   the zero was set folded. `hop.py` refuses to move if the angles look impossible.
3. Terminal 1: `python3 dash.py` → open <http://localhost:8080>. Tare the plate with nothing on it.
4. Terminal 2: hop (one line):
   ```bash
   python3 hop.py --crouch taught --land taught --push taught --push-mode through --push-vel 60 --push-accel 700 --max-push-vel 80 --land-torque 0.15 --land-kp 0.15 --telemetry 127.0.0.1:9871
   ```
   Answer `y`, then `h` = one hop, `+`/`-` push velocity, `]`/`[` push accel, `q` = stop and quit.
   The dashboard has the same buttons. Press **Record** on the dashboard to save a merged CSV in `laptop/logs/`.
5. Try `--dry-run` first on any new command: it runs the whole program against simulated motors.

Other tools: `python3 hop.py --teach crouch|land|push` (motors limp, set the pose by hand, Enter saves);
`python3 monitor.py` (watch the motors, they stay limp); `python3 stop.py` (stop both motors / clear a fault).

## Rules

- **Never write configuration to the moteus-c1 controllers** (`conf set`, `conf write`, `moteus_tool --write-config`,
  `--zero-offset`) without Dad. Only a human does this, never Claude Code or a script: a wrong limit, offset or
  gain can drive the leg out of bounds or over-current the motors. Reading (`--dump-config`, queries) is fine.
- Only one program may use `/dev/ttyACM0` at a time (don't run `moteus_tool` while `hop.py` is running).
- Judge hop height by **flight time on the plate**, not by the string pot (it loses counts at hop speeds).
- Board temperature: the controllers derate above 50 °C; let them cool below ~45 °C between sets.

See `NOTES.md` for the full history, measurements and the reasoning behind every setting.
