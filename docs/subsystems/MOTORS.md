# ROBSTRIDE Motor System

AIZEE drives **9 ROBSTRIDE actuators on a single CAN bus** (`can1` @ 1 Mbps).
The Rust service `rust/motor_control` (systemd unit `aizee-motor-control-rover`)
owns the bus, runs the control loops, accepts JSON commands on ZMQ `:5555`, and
publishes motor telemetry on `:5556`. Config: `config/hardware_jetson_rover.yaml`.

## Motor Roster

| Motor        | CAN ID | Model | Role                |
|--------------|--------|-------|---------------------|
| left_wheel   | 0x02   | RS04  | drive wheel         |
| right_wheel  | 0x04   | RS04  | drive wheel         |
| swivel       | 0x03   | RS03  | base swivel         |
| gantry_base  | 0x05   | RS04  | arm                 |
| gantry_mid   | 0x06   | RS03  | arm                 |
| gantry_end   | 0x07   | RS02  | arm                 |
| wrist_pitch  | 0x08   | RS02  | arm                 |
| wrist_roll   | 0x09   | RS00  | arm                 |
| gripper      | 0x0A   | RS00  | gripper             |

The arm is **7-DoF in software**: swivel (swivel-first) plus the 6-DoF gantry.
Wheels are present only in rover mode — both wheel motors must be physically on
the bus or `motor_control` wedges during CAN init. (RS = ROBSTRIDE model series;
MIT max torque per model: RS04 120 Nm, RS03 60 Nm, RS02 17 Nm, RS00 2 Nm.)

## Control Loops

| Loop      | Rate   | Notes                                                        |
|-----------|--------|-------------------------------------------------------------|
| arm       | 400 Hz | CAN-limited (6 arm motors × ~280µs round-trip ≈ 1680µs/cycle) |
| base      | 100 Hz | wheels + swivel                                             |
| telemetry | 50 Hz  | published motor states on `:5556`                          |
| watchdog  | 0.5 s  | holds position if no command arrives (tolerates WiFi jitter) |

## CAN Interface Setup

On the Jetson (production), motors are on `can1`:

```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip link show can1
```

## Command & Telemetry Interface

Commands are JSON on `tcp://localhost:5555` (`enable`, `drive`, `arm_joints`,
`emergency_stop`); telemetry is JSON on `tcp://localhost:5556` (per-motor
position, velocity, torque, temperature, error). See `rust/motor_control/README.md`
for the full message shapes and `rust/motor_control/src/robstride.rs` for the
protocol implementation.

---

## Parameter Read/Write Protocol

ROBSTRIDE motors expose internal parameters via CAN frames in any state
(including disabled). Parameters persist in flash after a `SaveConfig`.

**Stop the motor_control service first** — it holds the CAN socket and its
control frames will race with parameter responses.

```bash
sudo systemctl stop aizee-motor-control-rover
```

### CAN Frame Format

All frames use **extended CAN IDs** (29-bit).

**Arbitration ID construction (host → motor):**
```
bits 28–24  msg_type   (ReadParam=17/0x11, WriteParam=18/0x12, SaveConfig=22/0x16)
bits 15–8   HOST_CAN_ID = 0xAA
bits  7–0   motor_id   (CAN ID from the roster above)
```

**Read parameter (8-byte data):**
```
bytes 0–1   param_id (little-endian u16)
bytes 2–7   zeros
```

**Write parameter (8-byte data):**
```
bytes 0–1   param_id (little-endian u16)
bytes 2–3   zeros
bytes 4–7   value (little-endian f32)
```

**Response arb ID (motor → host):**
```
bits 28–24  0x11 (ReadParam)
bits 15–8   motor_id
bits  7–0   0xAA (HOST_CAN_ID)
```
Response data bytes 0–1 = param_id, bytes 4–7 = value (LE f32).

> Note: the **MIT control frame** (msg_type 1) is encoded differently —
> torque rides in the arb ID (bits 8–23) and the 8 data bytes carry
> position | velocity | kp | kd as big-endian u16. See `robstride.rs`.

### Key Parameter IDs

| Parameter      | ID     | Type | Description                            |
|----------------|--------|------|----------------------------------------|
| `RUN_MODE`     | 0x7005 | u8   | 0=MIT, 1=position, 2=speed, 3=current  |
| `LIMIT_TORQUE` | 0x700B | f32  | Peak torque limit (Nm)                 |
| `LIMIT_SPD`    | 0x7017 | f32  | Peak speed limit (rad/s)               |
| `LIMIT_CUR`    | 0x7018 | f32  | Peak current limit (A)                 |
| `VBUS`         | 0x701C | f32  | Bus voltage, read-only (V)             |

(Full set in `robstride.rs::params`.)

### Decoding a float from candump output

Response bytes 4–7 are a little-endian IEEE 754 float. To decode manually,
reverse the 4 bytes to big-endian then interpret as float32.

Example: `00 00 C0 40` → reversed → `40 C0 00 00` → 6.0

```bash
python3 -c "import struct; print(struct.unpack('<f', bytes.fromhex('0000C040'))[0])"
```

---

## Read/Write via cansend (manual terminal)

### Read a parameter

```bash
# Format: cansend can1 <ARBIT_ID>#<8-byte-data>
# Read LIMIT_TORQUE from gantry_end (motor 0x07)
timeout 2 candump can1 & sleep 0.15; cansend can1 1100AA07#0B70000000000000; wait
```

Watch candump output for a response with arb ID `110007AA`.

### Write a parameter

```bash
# Write LIMIT_TORQUE = 6.0 Nm to gantry_end (6.0f32 LE = 00 00 C0 40)
cansend can1 1200AA07#0B7000000000C040

# Save to flash (survives power cycle)
cansend can1 1600AA07#0000000000000000

# Verify
timeout 2 candump can1 & sleep 0.15; cansend can1 1100AA07#0B70000000000000; wait
```

`scripts/set_gantry_end_torque_limit.sh` automates the gantry_end case
(stops the service, writes `LIMIT_TORQUE=6.0`, reads back, restarts). Note it
intentionally **skips the flash save**, so the value reverts on power cycle.

### Common float values in little-endian hex

| Value (Nm or A) | LE hex     |
|-----------------|------------|
| 2.0             | `00000040` |
| 4.0             | `00008040` |
| 5.0             | `0000A040` |
| 6.0             | `0000C040` |
| 8.0             | `00000041` |
| 10.0            | `00002041` |
| 12.0            | `00004041` |
| 15.0            | `00007041` |
| 20.0            | `0000A041` |

---

## Per-Motor Limits

`config/hardware_jetson_rover.yaml` sets per-motor software limits
(`max_torque`, `max_velocity`, position bounds). Position bounds derive from
`config/robstride_calibration.json` plus a safety margin. Software `max_torque`
bounds the feedforward term; PD torque is bounded by the MIT encoding.

The table below records confirmed on-motor `LIMIT_TORQUE` / `LIMIT_CUR` flash
values. Update after any parameter change.

| Motor      | CAN ID | Model | LIMIT_TORQUE (Nm) | LIMIT_CUR (A) | Notes                                       |
|------------|--------|-------|-------------------|---------------|---------------------------------------------|
| gantry_end | 0x07   | RS02  | 6.0               | 10.0          | Was 2.0 Nm from factory — raised 2026-03-03 |

> **Check remaining motors.** Only gantry_end has been inspected so far. Run the
> read commands above against each motor ID to populate this table.

---

## Arb ID Quick Reference

Pre-computed arb IDs (ReadParam=`11`, WriteParam=`12`, SaveConfig=`16`):

| Motor       | CAN ID | Read arb ID | Write arb ID | Save arb ID |
|-------------|--------|-------------|--------------|-------------|
| left_wheel  | 0x02   | `1100AA02`  | `1200AA02`   | `1600AA02`  |
| swivel      | 0x03   | `1100AA03`  | `1200AA03`   | `1600AA03`  |
| right_wheel | 0x04   | `1100AA04`  | `1200AA04`   | `1600AA04`  |
| gantry_base | 0x05   | `1100AA05`  | `1200AA05`   | `1600AA05`  |
| gantry_mid  | 0x06   | `1100AA06`  | `1200AA06`   | `1600AA06`  |
| gantry_end  | 0x07   | `1100AA07`  | `1200AA07`   | `1600AA07`  |
| wrist_pitch | 0x08   | `1100AA08`  | `1200AA08`   | `1600AA08`  |
| wrist_roll  | 0x09   | `1100AA09`  | `1200AA09`   | `1600AA09`  |
| gripper     | 0x0A   | `1100AA0A`  | `1200AA0A`   | `1600AA0A`  |
