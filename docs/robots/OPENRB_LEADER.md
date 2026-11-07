# OpenRB-150 leader arm

A 7-DoF teleop leader built around a **Robotis OpenRB-150** board and **7 Dynamixel XL330-M077-T** servos, exposing its
own binary protocol over USB-CDC. Minerva's two GELLO leaders are this design, and so is AIZEE's optional leader; it is
one of AIZEE's three interchangeable leaders (SO-101, OpenRB-150, Quest VR). Two host drivers talk to it:

| Driver | Used by | Notes |
| --- | --- | --- |
| `src/robot_platform/teleop/openrb.py` (the guarded shared input) | Minerva, through Helm and the GELLO bridge | Identifies boards by USB serial from a device profile, keeps signed raw encoder ticks, uses the 50-byte `0xA6` freshness reply, and does not enable, centre or configure the servos |
| `python/teleop/openrb_leader.py` | AIZEE's collectors | The legacy driver with the 37-byte `0xA5` reply, autodetect and its own calibration files |

Joint routes and follower calibration are never in the firmware or the guarded driver: for Minerva they are the teleop
profile's routes and the leader ranges saved in Helm's Leaders tab. Installing the leaders on the operator station is
in [INSTALL_OPERATOR_STATION.md](../getting-started/INSTALL_OPERATOR_STATION.md#3-the-leaders); the firmware build is in
[its README](../../firmware/openrb_leader/README.md). Minerva's leaders are pinned by USB serial in
`config/devices/gello-dual-viewer.json`; they enumerate as USB `2F5D:2202`.

## Hardware

| Item | Qty | Notes |
| --- | --- | --- |
| OpenRB-150 (Robotis SAMD21G18A board, USB-C) | 1 | On-board Dynamixel TTL transceiver, native USB-CDC |
| Dynamixel XL330-M077-T | 7 | Protocol 2.0, 12-bit encoder, 1 Mbps bus |
| 3-pin TTL daisy-chain cable | 7 | Robotis standard |
| M5Stack Joystick2 (U156) | 0-1 | Optional; I2C `0x63` on D11 (SDA) and D12 (SCL), used for drive and record start/stop on AIZEE. Its state rides in the `0xA5` POLL reply. Neither Minerva leader reports one |
| 5 V supply for the DXL rail | 1 | Used on Minerva's leaders and needed for force feedback on more than one or two servos. With the supply on, unplugging USB does not reset a board: power it off too |

## Firmware

The firmware reports its version in `INFO` (`LEADER_FIRMWARE_VERSION`, now 3). Both installed Minerva leaders run
firmware 3.

- **Firmware 2** reads the servos continuously, one Sync Read after another, and answers a poll from the latest complete
  read instead of reading on request, so a poll costs the USB round trip. A poll is never answered twice with the same
  sample (the host faults on a repeated sequence): one that arrives before a newer read completes waits for it, and
  only after three failed reads in a row (3 ms each at most) is it answered with byte 48 = 0, which the host skips.
  Bytes 40-47 come from one microsecond clock, the read start rounded down and the reply rounded up, so their
  difference never understates a cached sample's age; the reply time never steps back. It adds `INFO` and
  `RETURN_DELAY`, and applies force feedback between reads with one Sync Write.
- **Firmware 3** gives USB the highest interrupt priority and the servo UART the next. Firmware 2 had put the UART
  above USB, against overruns of the seven back-to-back replies at Return Delay Time 0; the left board then stopped
  answering USB setup requests (hung opens after a close, failed enumeration), twice. Firmware 3 also resets the board if
  its loop stops for about a second (watchdog) and answers `0x57` with why it last reset (power-on, brown-out,
  watchdog, ...). The frame layout is unchanged, so hosts need no change.

`python scripts/openrb_leader_setup.py info` reports each board and `return-delay` sets the servos' Return Delay Time
(see the station guide). Legacy `0xA5` keeps its 37-byte reply but cannot provide the freshness guarantee. The bridge
uses a latest-source cache that keeps the original timestamps and rejects stale data. The
[dual leader viewer](../architecture/GELLO_TELEOP.md) polls both boards at 100 Hz through a worker pool and still requires
both complete, source-valid samples.

## Wire protocol (host to OpenRB-150 over USB-CDC)

Multi-byte values are little-endian. CRC-8 is MSB-first (polynomial `0x31`, seed `0x00`, not the reflected
Dallas/Maxim form) computed over `[HDR, ...payload]`. The constants are mirrored in `python/teleop/openrb_leader.py`,
`src/robot_platform/teleop/openrb.py` and `firmware/openrb_leader/src/main.cpp`: keep them in sync. An unknown byte is
dropped, so older firmware ignores newer commands.

| Cmd | Host to MCU | Reply (MCU to host) | Purpose |
| --- | --- | --- | --- |
| `IDENT` (0x50) | `0x50` | ASCII `"AIZEE-OPENRB-LEADER\n"` | Probe handshake (autodetect) |
| `POLL` (0xA5) | `0xA5` | 37-byte frame | Sync-read of all 7 positions and the joystick. Legacy hot path |
| `FRESH_POLL` (0xA6) | `0xA6` | 50-byte frame | The guarded path: positions plus the read's sequence and times |
| `SCAN` (0x53) | `0x53` | `[0x53][N][(id, baud_code) x N][crc8]` | Sweep every baud, list responders |
| `REID` (0x52) | `0x52 [target_id]` | `[0x52][status][found_id][baud_code][crc8]` | Re-assign the single bus servo's ID, bump to 1 Mbps |
| `CENTER` (0xC0) | `0xC0 [id]` | `[0xC0][status][id][int32 pos][crc8]` (8 bytes) | Drive one servo to 2048, then torque off |
| `FF_CURRENT` (0xCC) | `0xCC [N=7][int16 x 7][crc8]` | none | Set per-servo goal currents (force feedback) |
| `INFO` (0x49) | `0x49` | `[0x49][firmware][N][(id, servo_fw, return_delay, torque) x N][uint32 reads, failed_reads, last_read_us, max_read_us][crc8]` (48 bytes; 0xFF where a servo did not answer) | Board and servo diagnostics |
| `RETURN_DELAY` (0x44) | `0x44 [value][crc8 over 0x44, value]` | `[0x44][N][(id, before, after) x N][crc8]` (24 bytes) | Set each torque-off servo's Return Delay Time (EEPROM, 2 us/unit); no reply on a bad crc or with force feedback on |
| `RESET_CAUSE` (0x57), firmware 3 | `0x57` | `[0x57][RCAUSE][crc8]` | Why the board last reset |

**The `0xA6` reply (50 bytes):**

| Offset | Field |
| --- | --- |
| 0-1 | `0xA6`, seven channels |
| 2-29 | Seven signed int32 encoder positions |
| 30-35 | int16 joystick X/Y, uint8 button/status |
| 36-47 | Three uint32: the complete-read sequence (36-39), the bus-read start in ms (40-43) and the reply time in ms (44-47) |
| 48 | All seven servo replies valid |
| 49 | CRC over the preceding bytes |

Only a complete, error-free seven-servo read advances the sequence and sample time.

**The `0xA5` reply (37 bytes):** `[0xA5][N=7][int32 x 7 positions][int16 joy_x][int16 joy_y][uint8 joy_btn][uint8
joy_status][crc8]`. `joy_btn` is 0 pressed, 1 released; `joy_status` is 0 ok, 1 not present, 2 read error, and the host
ignores the joystick fields unless it is 0.

**Codes:** `baud_code` 0 = 1 Mbps, 1 = 57600, 2 = 115200, 3 = 2 Mbps. `REID` status 0 ok, 1 not found, 2 ambiguous,
3 set-id failed, 4 set-baud failed, 5 verify failed. `CENTER` status 0 ok, 1 not found, 2 timeout, 3 write failed.

### Hot path and latency

The firmware reads all 7 `Present_Position` registers (address 132) with one Protocol 2.0 Sync Read. At 1 Mbps that is
21 bytes out and 7 x 15 bytes back (about 1.3 ms) plus each servo's Return Delay Time before it answers: 500 us at the
XL330 factory value of 250, 3.5 ms for seven, so a read took 4-5 ms (measured with `scripts/probe_leader_timing.py`).
Return Delay Time 0 removes that wait. `SCAN`, `REID` and `CENTER` operate on one servo at a time.

### Force feedback (`FF_CURRENT`)

By default every servo is torque-disabled so the operator can backdrive the arm. The host opts in per poll by sending
`FF_CURRENT` with 7 signed goal currents (mA), clamped to plus or minus 200 mA by firmware and host; `INT16_MIN`
(-32768) disables a slot. It is fire-and-forget and must be resent at a steady cadence (about 30 Hz or faster).
Firmware 2 and later collect the command without waiting on USB, switch a slot into current control only when it turns
on (Operating Mode is EEPROM), and write every active slot's current in one Sync Write between reads (no status
replies, about 0.2 ms), so force feedback on the gripper alone does not slow the position reads. A two-stage watchdog
zeroes the currents at 200 ms stale (still energised) and disables force feedback entirely at 1000 ms. Driving it on
more than one or two servos needs an external 5 V supply on the OpenRB rail. What Minerva sends is described in
[GRIPPER_FORCE_FEEDBACK.md](minerva/GRIPPER_FORCE_FEEDBACK.md).

## AIZEE's mapping and setup scripts (legacy)

These describe the older AIZEE path and are not Minerva's calibration.

AIZEE's follower arm is 7-DoF, swivel first; the joint mapping mirrors the SO-101 leader so calibration files are
interchangeable.

| Servo ID | Servo name | AIZEE joint | Follower CAN id |
| --- | --- | --- | --- |
| 1 | shoulder_pan | swivel | 0x03 |
| 2 | shoulder_lift | gantry_base | 0x05 |
| 3 | elbow_flex | gantry_mid | 0x06 |
| 4 | wrist_flex | gantry_end | 0x07 |
| 5 | wrist_yaw | wrist_pitch | 0x08 |
| 6 | wrist_roll | wrist_roll | 0x09 |
| 7 | gripper | gripper | 0x0A |

| Path | Purpose |
| --- | --- |
| `python/teleop/openrb_leader.py` | `OpenRBLeader` controller class, wire constants, autodetect |
| `python/teleop/leader.py` | `find_any_leader()` and `get_leader_class()` across leader kinds |
| `python/teleop/serial_safe.py` | `open_serial()`, a timeout-bounded port open that will not hang on a Bluetooth or unresponsive port |
| `python/scripts/openrb_setup_arm.py` | First-time servo ID assignment wizard (`--port`, `--start-at N`). Factory XL330 servos are ID 1: plug servos in one at a time, in the order it prints |
| `python/scripts/openrb_calibrate.py` | Per-joint min/max wizard, writes `config/openrb_calibration.json` |
| `python/scripts/leader_monitor.py` | Live position and limits monitor; its `CENTER` command drives servos to 2048 one at a time, since energising all seven at once browns out the USB rail |

`collect_demo.py` takes `--leader {auto,so101,openrb}`; with `auto` it tries the SO-101 first, then the OpenRB-150. The
OpenRB autodetect ranks ports by known USB vendor ids (Robotis, Arduino, Adafruit) but gates on the `IDENT` handshake.
