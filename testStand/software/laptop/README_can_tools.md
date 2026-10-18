# testRobot — moteus / fdcanusb CAN test programs

Talk to a CAN-FD bus through the **mjbots fdcanusb / mjcanfd-usb-1x** USB adapter
and send **moteus position-mode** commands.

- Adapter: <https://mjbots.com/products/mjcanfd-usb-1x> · protocol: <https://github.com/mjbots/fdcanusb>
- Control modes: <https://mjbots.github.io/moteus/guides/control-modes/#understanding-position-mode>

The adapter enumerates as a USB CDC-ACM serial port, usually **`/dev/ttyACM0`**.

## Files

| File | Purpose |
|------|---------|
| `fdcanusb.py` | Minimal raw driver for the fdcanusb text protocol (`can send`, `can status`, `rcv` parsing). No motor logic. |
| `comm_test.py` | Proves the host can talk to the adapter. Sends `can status`; optionally a **read-only** query to a controller. Does not move a motor. |
| `moteus_position.py` | Builds/sends **position-mode** commands via the official `moteus` library. Dry-run by default. |
| `requirements.txt` | `moteus`, `pyserial`. |

## Setup

```bash
pip install -r requirements.txt
# Serial access (log out/in afterwards):
sudo usermod -aG dialout $USER
ls -l /dev/ttyACM0        # confirm the adapter is present
```

## 1. Verify communication (safe — no motor motion)

```bash
python3 comm_test.py                 # open port + 'can status'
python3 comm_test.py --query-id 1    # also READ registers of controller id 1
```

`--query-id` sends a query-only frame (reads mode/position/velocity/torque/…);
it does **not** change the control mode or move anything.

## 2. Inspect position commands (dry run — nothing transmitted)

The dry run builds the **exact** CAN-FD frames with the `moteus` library and prints
the literal `can send …` lines that would go over the wire — no serial port opened.

```bash
python3 moteus_position.py                                             # velocity-only (position=NaN)
python3 moteus_position.py --id 2 --position 0.25 --velocity 0.5 --max-torque 1.0
```

Example output (id 2, position 0.25 rev):

```
2) POSITION frame:
   arbitration_id : 0x8002  (dest id=2, reply_requested=True)
   payload (25 B) : 01000A0E200000803E0000003F0D250000803F11001F01130D
   fdcanusb line  : can send 8002 01000A0E...50505050505050 B F
```

Arbitration id = `dest_id | 0x8000 (reply bit) | source<<8 | can_prefix<<16`.
Payload is padded up to a valid CAN-FD length with `0x50`. `B`=bitrate switch, `F`=CAN-FD.

## 3. Send for real (you run this manually — the motor WILL move)

```bash
python3 moteus_position.py --live --id 1 --position 0.25 --max-torque 1.0
```

`--live` opens `/dev/ttyACM0` via `moteus.Fdcanusb`, sends a **stop** (clears
faults / holds), then the position command, and prints the returned state.

## Position-mode fields (see moteus docs)

`--position` (rev; omit ⇒ NaN ⇒ velocity-only) · `--velocity` (rev/s) ·
`--feedforward-torque` (N·m) · `--kp-scale` · `--kd-scale` ·
`--max-torque` (N·m clamp — recommended safety limit) · `--stop-position` (rev) ·
`--watchdog-timeout` (s) · `--velocity-limit` (rev/s) · `--accel-limit` (rev/s²).

> Safety: always start with a small `--max-torque` and a modest `--velocity-limit`.
> A position command with no watchdog keeps holding until the next command.
