# OpenRB-150 Leader Arm

Newer 7-DOF leader arm built around a **Robotis OpenRB-150** board and **7×
Dynamixel XL330-M077-T** servos. Drop-in replacement for the original SO-101
leader — both expose the same controller interface, so any script that worked
with the SO-101 works here once a calibration JSON exists.

## Hardware

| Item | Quantity | Notes |
|---|---|---|
| OpenRB-150 (Robotis SAMD21G18A board, USB-C) | 1 | On-board Dynamixel TTL transceiver, native USB-CDC |
| Dynamixel XL330-M077-T | 7 | Protocol 2.0, 12-bit encoder, 1 Mbps bus |
| 3-pin TTL daisy chain cable | 7 | Per Robotis standard |
| 5 V supply for the OpenRB DXL rail | 1 | USB power is sufficient for the leader use case (no torque load) |

Joint mapping is identical to the SO-101 leader so calibration files and
downstream code are interchangeable:

| Servo ID | Servo Name      | AIZEE joint    |
|---|---|---|
| 1 | shoulder_pan    | swivel         |
| 2 | shoulder_lift   | gantry_base    |
| 3 | elbow_flex      | gantry_mid     |
| 4 | wrist_flex      | gantry_end     |
| 5 | wrist_yaw       | wrist_pitch    |
| 6 | wrist_roll      | wrist_roll     |
| 7 | gripper         | gripper        |

## Software layout

| Path | Purpose |
|---|---|
| [firmware/openrb_leader/](../../firmware/openrb_leader/) | OpenRB-150 firmware (PlatformIO + Arduino, vendored Robotis SAMD BSP) |
| [python/teleop/openrb_leader.py](../../python/teleop/openrb_leader.py) | `OpenRBLeader` controller class (mirrors `So101Leader`'s interface) |
| [python/teleop/leader.py](../../python/teleop/leader.py) | `find_any_leader()` autodetect across both leader kinds |
| [python/scripts/openrb_setup_arm.py](../../python/scripts/openrb_setup_arm.py) | First-time servo ID assignment wizard |
| [python/scripts/openrb_calibrate.py](../../python/scripts/openrb_calibrate.py) | Per-joint min/max calibration wizard |
| [python/scripts/leader_monitor.py](../../python/scripts/leader_monitor.py) | Live position + limits monitor (works for either leader kind) |
| [config/openrb_calibration.json](../../config/openrb_calibration.json) | Calibration output (same schema as `so101_calibration.json`) |

## First-time bring-up

1. **Flash the firmware.**
   ```bash
   pip install --user platformio
   pio run -e openrb150 -t upload --upload-port COM4 \
       -d firmware/openrb_leader
   ```
   The PIO project vendors the Robotis OpenRB-150 SAMD BSP under
   `firmware/openrb_leader/openrb150_bsp/` so it builds on any machine
   without needing the Arduino IDE installed.

2. **Assign servo IDs (one servo at a time).** Factory XL330 servos all
   default to ID=1 @ 57600 baud. The wizard scans every supported baud,
   re-IDs whatever single servo is on the bus, and bumps it to 1 Mbps:
   ```bash
   python python/scripts/openrb_setup_arm.py
   ```
   - Disconnect every servo from the OpenRB before starting.
   - Plug servos in **one at a time** in the order printed by the wizard.
   - The wizard verifies every assignment with a follow-up scan.
   - If a step fails, re-run with `--start-at N` to resume mid-arm.

3. **Calibrate joint endpoints.** Plug all 7 servos in, run:
   ```bash
   python python/scripts/openrb_calibrate.py
   ```
   This walks each joint through MIN and MAX captures and writes
   `config/openrb_calibration.json` (same schema as the SO-101 file).

4. **Sanity-check positions and limits.**
   ```bash
   python python/scripts/leader_monitor.py
   ```
   Press **C** at any time to drive all 7 servos sequentially to encoder
   centre (2048). Sequential is intentional — energising all 7 at once
   would brown out the OpenRB's USB rail.

## Wire protocol (host ↔ OpenRB-150 over USB-CDC)

All multi-byte values are little-endian. CRC-8 is Dallas/Maxim
(polynomial `0x31`, seed `0x00`) over `[hdr, ...payload]`.

| Cmd | Bytes (host → mcu) | Reply (mcu → host) | Purpose |
|---|---|---|---|
| `IDENT` (0x50) | `0x50` | ASCII `"AIZEE-OPENRB-LEADER\n"` | Probe handshake — used by autodetect |
| `POLL` (0xA5) | `0xA5` | `[0xA5][N=7][int32 LE × 7][crc8]` (31 bytes) | Sync-read of all 7 `Present_Position` registers — hot path |
| `SCAN` (0x53) | `0x53` | `[0x53][N][(id, baud_code) × N][crc8]` | Sweep every supported baud, list responders |
| `REID` (0x52) | `0x52 [target_id]` | `[0x52][status][found_id][baud_code][crc8]` | Re-assign single bus device's ID and bump it to 1 Mbps |
| `CENTER` (0xC0) | `0xC0 [id]` | `[0xC0][status][id][int32 LE pos][crc8]` | Drive one servo to position 2048, then disable torque |

Baud codes: `0=1Mbps`, `1=57600`, `2=115200`, `3=2Mbps`.

REID status: `0=ok`, `1=not_found`, `2=ambiguous`, `3=set_id_failed`,
`4=set_baudrate_failed`, `5=verify_failed`.

CENTER status: `0=ok`, `1=not_found`, `2=timeout`, `3=write_failed`.

Wire-protocol constants are mirrored on the host in
[python/teleop/openrb_leader.py](../../python/teleop/openrb_leader.py) — keep
the two in sync if either is edited.

## Latency

The hot path (`CMD_POLL`) uses Protocol-2.0 SYNC_READ to read all 7
`Present_Position` registers in **one** bus transaction (~1–2 ms at 1 Mbps),
not 7 sequential reads. This is what bounds leader→target lag during teleop.
Setup commands (`SCAN`, `REID`, `CENTER`) use single-servo reads/writes
because they operate on one servo at a time by design.

## Integration

`collect_demo.py`, `leader_monitor.py`, and any future leader-driven script
should use `find_any_leader()` from `python/teleop/leader.py` rather than
hardcoding either leader class:

```python
from leader import find_any_leader, get_leader_class, default_calib_path

port, kind = find_any_leader(verbose=True)
leader = get_leader_class(kind)(port, calib=default_calib_path(kind))
leader.connect()
```

`collect_demo.py` already does this and accepts `--leader {auto,so101,openrb}`
to force a specific kind. With `auto` (the default) it tries the SO-101 first
to preserve existing user setups, then falls through to the OpenRB-150.
