# ROBSTRIDE Motor Configuration

Reference for configuring ROBSTRIDE motor internal parameters over CAN.

## Parameter Read/Write Protocol

ROBSTRIDE motors expose internal parameters via CAN frames while the motor is
in any state (including disabled). Parameters persist in flash after a
`SaveConfig` command.

**The motor_control service must be stopped first** — it holds the CAN socket
and its 1 kHz control frames will race with parameter responses.

```bash
# On Jetson (or via SSH)
sudo systemctl stop aizee-motor-control-rover
```

### CAN Frame Format

All frames use **extended CAN IDs** (29-bit).

**Arbitration ID construction (host → motor):**
```
bits 28–24  msg_type   (ReadParam=17/0x11, WriteParam=18/0x12, SaveConfig=22/0x16)
bits 15–8   HOST_CAN_ID = 0xAA
bits  7–0   motor_id   (CAN ID from hardware_jetson_rover.yaml)
```

**Motor IDs:**
| Motor        | CAN ID |
|---|---|
| left_wheel   | 0x02   |
| swivel       | 0x03   |
| right_wheel  | 0x04   |
| gantry_base  | 0x05   |
| gantry_mid   | 0x06   |
| gantry_end   | 0x07   |
| wrist_pitch  | 0x08   |
| wrist_roll   | 0x09   |
| gripper      | 0x0A   |

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

### Key Parameter IDs

| Parameter     | ID     | Type  | Description                        |
|---|---|---|---|
| `LIMIT_TORQUE`| 0x700B | f32   | Peak torque limit (Nm)             |
| `LIMIT_CUR`   | 0x7018 | f32   | Peak current limit (A)             |
| `LIMIT_SPD`   | 0x7017 | f32   | Peak speed limit (rad/s)           |
| `RUN_MODE`    | 0x7005 | u8    | 0=MIT, 1=position, 2=speed, 3=current |
| `VBUS`        | 0x701C | f32   | Bus voltage read-only (V)          |

### Decoding a float from candump output

Response bytes 4–7 are a little-endian IEEE 754 float. To decode manually:
reverse the 4 bytes to get big-endian, then interpret as float32.

Example: `00 00 C0 40` → reversed → `40 C0 00 00` → 6.0

Quick Python one-liner on the Jetson:
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

### Common float values in little-endian hex

| Value (Nm or A) | LE hex     |
|---|---|
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

## Per-Motor Default Limits (as configured 2026-03-03)

The table below records confirmed `LIMIT_TORQUE` and `LIMIT_CUR` values.
Update after any parameter changes.

| Motor       | CAN ID | Model | LIMIT_TORQUE (Nm) | LIMIT_CUR (A) | Notes |
|---|---|---|---|---|---|
| gantry_end  | 0x07   | RS02  | 6.0               | 10.0          | Was 2.0 Nm from factory — raised 2026-03-03 |

> **Check remaining motors.** Only gantry_end has been inspected so far.
> Run the read commands above against each motor ID to populate this table.

---

## Arb ID Quick Reference

Pre-computed arb IDs for common operations (ReadParam=`11`, WriteParam=`12`, SaveConfig=`16`):

| Motor       | CAN ID | Read arb ID | Write arb ID | Save arb ID |
|---|---|---|---|---|
| left_wheel  | 0x02   | `1100AA02`  | `1200AA02`   | `1600AA02`  |
| swivel      | 0x03   | `1100AA03`  | `1200AA03`   | `1600AA03`  |
| right_wheel | 0x04   | `1100AA04`  | `1200AA04`   | `1600AA04`  |
| gantry_base | 0x05   | `1100AA05`  | `1200AA05`   | `1600AA05`  |
| gantry_mid  | 0x06   | `1100AA06`  | `1200AA06`   | `1600AA06`  |
| gantry_end  | 0x07   | `1100AA07`  | `1200AA07`   | `1600AA07`  |
| wrist_pitch | 0x08   | `1100AA08`  | `1200AA08`   | `1600AA08`  |
| wrist_roll  | 0x09   | `1100AA09`  | `1200AA09`   | `1600AA09`  |
| gripper     | 0x0A   | `1100AA0A`  | `1200AA0A`   | `1600AA0A`  |
