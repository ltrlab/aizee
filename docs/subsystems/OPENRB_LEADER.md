# OpenRB-150 Leader Arm

## Shared guarded input (2026-09-24)

New robot-independent input code lives in `src/robot_platform/teleop/openrb.py`.
It identifies boards by USB serial from a device profile, preserves signed raw
encoder ticks, and does not enable, center or configure leader servos. The
firmware disables responding servos on startup. Joint routes and follower
calibration belong outside the driver. The older Aizee-specific mapping and
setup scripts below describe the legacy path; they are not Minerva calibration.

The fresh poll `0xA6` returns 50 bytes, little-endian fields:

| Offset | Field |
|---|---|
| 0–1 | `0xA6`, seven channels |
| 2–29 | Seven signed int32 encoder positions |
| 30–35 | int16 joystick X/Y, uint8 button/status |
| 36–47 | uint32 complete-read sequence, bus-read start ms, reply ms |
| 48 | All seven servo replies valid |
| 49 | CRC over preceding bytes |

Only a complete error-free seven-servo read advances the sequence/sample time.

**Firmware 2 (2026-10-12)** reads the servos continuously, one Sync Read after
another, and answers a poll from the latest complete read instead of reading on
request, so a poll costs the USB round trip. A poll is never answered twice with
the same sample (the host faults on a repeated sequence): one that arrives before a
newer read completes waits for it, and only after three failed reads in a row
(3 ms each at most) is it answered with byte 48 = 0, which the host skips. USB keeps
the highest interrupt priority and the servo UART comes next (firmware 3). Firmware 2
put the UART above USB, against overruns of the seven back-to-back replies (Return
Delay Time 0, 0.02–0.07% of reads under polling); the left board then stopped
answering USB setup requests (hung opens after a close, failed enumeration) on
2026-10-12 and again on 2026-10-14 on a bench supply. Firmware 3 also resets the
board if its loop stops for about a second (watchdog) and answers `0x57` with why it
last reset (`info` prints it: power-on, brown-out, watchdog, ...). Bytes 40–47 come from one
microsecond clock, the read start rounded down and the reply rounded up, so their
difference never understates a cached sample's age; the reply time never steps
back. The frame layout is unchanged, so hosts need no change. Firmware 2 adds
`INFO` and `RETURN_DELAY` (below) and applies force feedback between reads with one
Sync Write. `python scripts/openrb_leader_setup.py info` reports each board.
Legacy `0xA5` retains its original 37-byte reply but cannot provide this source
freshness guarantee. The CRC uses MSB-first polynomial `0x31`, seed zero (not
the reflected Dallas/Maxim implementation). The bridge uses a latest-source
cache; it retains original timestamps and rejects stale cached data.

Both installed Minerva leaders were flashed and checksum-verified. Their USB
serials and side assignment are in `config/devices/minerva-leaders-01.json`.
Neither reports a joystick. See [guarded teleoperation](../GUARDED_TELEOP.md).

The [dual leader viewer](../GELLO_TELEOP.md) now requests 100 Hz paired reads and
renders a configurable arm schematic with software reference/direction controls.
Independent USB buses are polled concurrently through a persistent worker pool;
both complete source-valid samples are still required. A short 10-second run
returned 996 pairs (maximum source age 15.23 ms, skew 1.93 ms, four 10 ms deadline
misses). This does not change the firmware, servo settings or the follower's
initial 25 Hz control target.

A 7-DoF teleop leader built around a **Robotis OpenRB-150** board and **7×
Dynamixel XL330-M077-T** servos, exposing its own binary protocol over USB-CDC.

It is one of three interchangeable AIZEE leaders (SO-101, OpenRB-150, Quest VR);
all present the same duck-typed controller interface, so any script that uses one
works with the others once a calibration JSON exists. This page covers the
OpenRB-150 specifically: firmware, setup wizard, and the USB wire protocol.

The OpenRB-150 leader also optionally carries an **M5Stack Joystick2** on I2C
(addr `0x63`, pins D11=SDA / D12=SCL), used for operator drive and
record start/stop without a separate USB gamepad. Its state rides in the POLL
reply.

## Hardware

| Item | Qty | Notes |
|---|---|---|
| OpenRB-150 (Robotis SAMD21G18A board, USB-C) | 1 | On-board Dynamixel TTL transceiver, native USB-CDC |
| Dynamixel XL330-M077-T | 7 | Protocol 2.0, 12-bit encoder, 1 Mbps bus |
| 3-pin TTL daisy-chain cable | 7 | Robotis standard |
| M5Stack Joystick2 (U156) | 0–1 | Optional; I2C 0x63 on D11/D12 |
| 5 V supply for the DXL rail | 1 | Used on Minerva's leaders and needed for force feedback. The left board's USB failures (failed enumeration, hung port opens, "Access is denied") were first put down to USB power on 2026-10-12; they came back on the bench supply on 2026-10-14 and are now attributed to firmware 2's interrupt priority (see the protocol notes). With the supply on, unplugging USB does not reset a board: power it off too |

Joint mapping mirrors the SO-101 leader, so calibration files and downstream code
are interchangeable. The AIZEE follower arm is 7-DoF swivel-first.

| Servo ID | Servo name | AIZEE joint | Follower CAN id |
|---|---|---|---|
| 1 | shoulder_pan | swivel | 0x03 |
| 2 | shoulder_lift | gantry_base | 0x05 |
| 3 | elbow_flex | gantry_mid | 0x06 |
| 4 | wrist_flex | gantry_end | 0x07 |
| 5 | wrist_yaw | wrist_pitch | 0x08 |
| 6 | wrist_roll | wrist_roll | 0x09 |
| 7 | gripper | gripper | 0x0A |

## Software layout

| Path | Purpose |
|---|---|
| `firmware/openrb_leader/` | OpenRB-150 firmware (PlatformIO + Arduino, vendored Robotis SAMD BSP under `openrb150_bsp/`) |
| `python/teleop/openrb_leader.py` | `OpenRBLeader` controller class + wire-protocol constants + autodetect |
| `python/teleop/leader.py` | `find_any_leader()` / `get_leader_class()` discovery across leader kinds |
| `python/teleop/serial_safe.py` | `open_serial()` — timeout-bounded port open (won't hang on a Bluetooth/unresponsive port) |
| `python/scripts/openrb_setup_arm.py` | First-time servo ID assignment wizard |
| `python/scripts/openrb_calibrate.py` | Per-joint min/max calibration wizard |
| `python/scripts/leader_monitor.py` | Live position + limits monitor (any leader kind) |
| `config/openrb_calibration.json` | Calibration output (same schema as `so101_calibration.json`) |

## First-time bring-up

1. **Flash the firmware.**
   ```bash
   pip install --user platformio
   pio run -e openrb150 -t upload --upload-port COM4 -d firmware/openrb_leader
   ```
   The PIO project vendors the Robotis OpenRB-150 SAMD BSP under
   `firmware/openrb_leader/openrb150_bsp/`, so it builds without the Arduino IDE.

2. **Assign servo IDs (one servo at a time).** Factory XL330 servos default to
   ID=1. The wizard scans every supported baud, re-IDs the single servo on the
   bus, and bumps it to 1 Mbps:
   ```bash
   python python/scripts/openrb_setup_arm.py            # auto-detect port
   python python/scripts/openrb_setup_arm.py --port COM5
   python python/scripts/openrb_setup_arm.py --start-at 3   # resume mid-arm
   ```
   - Plug servos in one at a time, in the order the wizard prints, unplugging
     each before the next.
   - Each assignment is verified with a follow-up scan.

3. **Calibrate joint endpoints.** Plug in all 7 servos, then:
   ```bash
   python python/scripts/openrb_calibrate.py
   ```
   Writes `config/openrb_calibration.json`.

4. **Sanity-check positions and limits.**
   ```bash
   python python/scripts/leader_monitor.py
   ```
   The `CENTER` command drives servos to encoder centre (2048) one at a time —
   sequential by design, since energising all 7 at once browns out the USB rail.

## Wire protocol (host ↔ OpenRB-150 over USB-CDC)

Multi-byte values are little-endian. CRC-8 is MSB-first (polynomial `0x31`,
seed `0x00`) computed over `[HDR, ...payload]`. Constants are mirrored in
`python/teleop/openrb_leader.py` and `firmware/openrb_leader/src/main.cpp` — keep
the two in sync.

| Cmd | Host → MCU | Reply (MCU → host) | Purpose |
|---|---|---|---|
| `IDENT` (0x50) | `0x50` | ASCII `"AIZEE-OPENRB-LEADER\n"` | Probe handshake (autodetect) |
| `POLL` (0xA5) | `0xA5` | 37-byte frame (see below) | Sync-read of all 7 positions + joystick — hot path |
| `SCAN` (0x53) | `0x53` | `[0x53][N][(id, baud_code) × N][crc8]` | Sweep every baud, list responders |
| `REID` (0x52) | `0x52 [target_id]` | `[0x52][status][found_id][baud_code][crc8]` | Re-assign the single bus servo's ID, bump to 1 Mbps |
| `CENTER` (0xC0) | `0xC0 [id]` | `[0xC0][status][id][int32 LE pos][crc8]` (8 bytes) | Drive one servo to 2048, then torque off |
| `FF_CURRENT` (0xCC) | `0xCC [N=7][int16 LE × 7][crc8]` | *(none — fire-and-forget)* | Set per-servo goal currents (force feedback) |
| `INFO` (0x49), firmware 2 | `0x49` | `[0x49][2][N][(id, servo_fw, return_delay, torque) × N][uint32 reads, failed_reads, last_read_us, max_read_us][crc8]` (48 bytes; 0xFF where a servo did not answer) | Board and servo diagnostics |
| `RETURN_DELAY` (0x44), firmware 2 | `0x44 [value][crc8 over 0x44, value]` | `[0x44][N][(id, before, after) × N][crc8]` (24 bytes) | Set each torque-off servo's Return Delay Time (EEPROM, 2 µs/unit); refused (no reply) with a bad crc or force feedback on |

**POLL reply (37 bytes, N=7):**

```
[0xA5][N=7][int32 LE × 7]      positions (Present_Position, modulo-4096 on host)
[int16 LE joy_x][int16 LE joy_y]
[uint8 joy_btn][uint8 joy_status]
[crc8]
```

`joy_btn` is `0=pressed, 1=released`. `joy_status` is `0=ok, 1=not present,
2=read error`; the host ignores the joystick fields when `joy_status != 0`.

**Codes:**

- `baud_code`: `0=1Mbps`, `1=57600`, `2=115200`, `3=2Mbps`.
- `REID status`: `0=ok`, `1=not_found`, `2=ambiguous`, `3=set_id_failed`,
  `4=set_baudrate_failed`, `5=verify_failed`.
- `CENTER status`: `0=ok`, `1=not_found`, `2=timeout`, `3=write_failed`.

### Hot path and latency

The firmware reads all 7 `Present_Position` registers (addr 132) with one
Protocol-2.0 Sync Read. At 1 Mbps that is 21 bytes out and 7 × 15 bytes back
(~1.3 ms) plus each servo's Return Delay Time before it answers: 500 µs at the
XL330 factory value of 250, 3.5 ms for seven, so a read took 4–5 ms (measured
2026-10-12, `scripts/probe_leader_timing.py`). Return Delay Time 0 removes that
wait. `SCAN`, `REID`, and `CENTER` operate on one servo at a time and use single
reads/writes.

### Force feedback (`FF_CURRENT`)

By default every servo is torque-disabled so the operator can backdrive the arm.
The host opts in per-poll by sending `FF_CURRENT` with 7 signed goal currents (mA),
clamped to ±200 mA (firmware and host both clamp); `INT16_MIN` (-32768) disables a
slot. It is fire-and-forget (no reply) and must be resent at a steady cadence
(~30 Hz). Firmware 2 collects the command without waiting on USB, switches a slot
into current control only when it turns on (Operating Mode is EEPROM), and writes
every active slot's current in one Sync Write between reads (no status replies,
~0.2 ms), so force feedback, for example on the gripper alone, does not slow the
position reads. The firmware runs a two-stage watchdog: at 200 ms stale it zeroes
currents (still energised), at 1000 ms it disables FF entirely. Driving FF on more
than one or two servos requires an external 5 V supply on the OpenRB rail.

## Integration

Use `find_any_leader()` from `python/teleop/leader.py` rather than hardcoding a
class. Discovery probes every enumerated serial port via `serial_safe.open_serial`,
so an unresponsive port (e.g. a Bluetooth serial device) can never gate startup:

```python
from leader import find_any_leader, get_leader_class, default_calib_path

port, kind = find_any_leader(verbose=True)          # tries SO-101 first, then OpenRB
leader = get_leader_class(kind)(port, calib=default_calib_path(kind))
leader.connect()
```

`collect_demo.py` already does this and accepts `--leader {auto,so101,openrb}` to
force a specific kind. With `auto` (default) it tries SO-101 first, then the
OpenRB-150. The OpenRB autodetect (`find_openrb_port`) ranks ports by known USB
VIDs (ROBOTIS, Arduino, Adafruit) but gates on the `IDENT` handshake.
