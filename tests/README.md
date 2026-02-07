# AIZEE Test Scripts

This directory contains test scripts for validating motor control and system integration.

## Directory Structure

```
tests/
├── direct_can/         # Low-level CAN protocol tests (Python only, no Rust binary)
├── integration/        # Full system tests (Rust motor_control + ZeroMQ)
└── utils/             # Utilities for debugging and development
```

## Current Test Setup

**Hardware:**
- ROBSTRIDE03 (CAN ID 0x02) - Mapped as "right_wheel"
- ROBSTRIDE04 (CAN ID 0x03) - Mapped as "left_wheel"
- Configuration: `config/hardware_two_motors.yaml`

**Prerequisites:**
- CAN interface configured: `sudo ./scripts/setup_can.sh`
- Rust motor_control built: `cd rust/motor_control && cargo build --release`
- Python dependencies: `pip install python-can pyzmq`

## Tests

### Direct CAN Tests (`direct_can/`)

These tests communicate directly with motors via CAN bus without using the Rust binary.

#### `motor_control_test.py`
Low-level motor control with tuned PD gains.

**Features:**
- Direct CAN protocol implementation
- Tuned gains: Kp=3.0, Kd=0.3 (smooth motion, no vibrations)
- Proper signal handling (SIGINT/SIGTERM)
- Auto-disable on exit (safe for testing)

**Usage:**
```bash
# On Jetson
cd ~/aizee/tests/direct_can
./motor_control_test.py
# Press Ctrl+C to stop
```

**Note:** Only use for testing. Production systems should keep motors enabled to prevent dropping loads.

---

### Integration Tests (`integration/`)

These tests run the full Rust motor_control binary with ZeroMQ command interface.

#### `test_both_motors.sh`
Tests both motors together with zero positioning.

**What it does:**
1. Starts motor_control binary with two-motor config
2. Enables both motors
3. Zeros positions
4. Drives forward at 0.3 rad/s for 4 seconds
5. Stops and disables motors
6. Cleans up processes

**Usage:**
```bash
# On Jetson
cd ~/aizee/tests/integration
./test_both_motors.sh
```

**Output:** Log written to `/tmp/zeroed_test.log`

#### `test_individual.sh`
Tests each motor separately to validate individual motor operation.

**What it does:**
1. Tests ROBSTRIDE03 (ID 2, "right_wheel") alone
2. Tests ROBSTRIDE04 (ID 3, "left_wheel") alone
3. Each motor: enable → zero → spin @ 0.5 rad/s → stop → disable

**Usage:**
```bash
# On Jetson
cd ~/aizee/tests/integration
./test_individual.sh
```

**Output:** Log written to `/tmp/individual_test.log`

---

### Utilities (`utils/`)

#### `scan_all_motors.py`
Scans CAN bus for all connected motors (IDs 1-127).

**Usage:**
```bash
# On Jetson
cd ~/aizee/tests/utils
python3 scan_all_motors.py
```

**Output:**
```
Scanning all IDs 1-127...
✓ Found motor at CAN ID 2 (0x02)
✓ Found motor at CAN ID 3 (0x03)

Found 2 motor(s): [2, 3]
```

#### `send_zmq_command.py`
Send individual ZeroMQ commands to running motor_control process.

**Usage:**
```bash
# Start motor_control first, then:
python3 send_zmq_command.py

# Modify the script to send different commands:
# - {"type": "enable", "motor_ids": ["left_wheel"]}
# - {"type": "drive", "linear": 0.5, "angular": 0.0}
# - {"type": "disable", "motor_ids": ["left_wheel"]}
```

#### `capture_can_frames.sh`
Wrapper for `candump` to capture CAN traffic for debugging.

**Usage:**
```bash
# On Jetson
cd ~/aizee/tests/utils
./capture_can_frames.sh
# Press Ctrl+C to stop
```

---

## Proven Control Parameters

### ROBSTRIDE03 (Position Control)
- **Kp**: 3.0 - Smooth motion without vibrations
- **Kd**: 0.3-0.8 - Good damping, minimal oscillation
- **Frequency**: 50-100 Hz
- ⚠️ **Note**: Higher gains (Kp=20, Kd=2) cause vibrations

### ROBSTRIDE04 (Velocity Control)
- **Linear velocity**: 0.15-0.5 rad/s (tested via "drive" commands)
- **Zero position**: Working correctly
- **Control frequency**: 100 Hz (base loop)

---

## Common Issues

### CAN interface not found
```bash
sudo ./scripts/setup_can.sh
```

### Permission denied on CAN socket
```bash
sudo usermod -aG dialout $USER
# OR run tests with sudo (not recommended)
```

### Motor not responding
1. Check CAN wiring
2. Verify motor is powered
3. Scan for motors: `python3 tests/utils/scan_all_motors.py`
4. Check CAN traffic: `./tests/utils/capture_can_frames.sh`

### Rust binary not found
```bash
cd ~/aizee/rust/motor_control
cargo build --release
```

---

## Development Workflow

### Testing a new feature:
1. Write unit tests in `rust/motor_control/tests/`
2. Build: `cargo build --release`
3. Test individual components with utilities
4. Run integration tests
5. Monitor CAN traffic if issues occur

### Before committing changes:
1. Run all integration tests
2. Verify both motors respond correctly
3. Check logs for errors
4. Update this README if adding new tests

---

## Related Documentation

- Main project: `../README.md`
- Development guide: `../CLAUDE.md`
- Implementation phases: `../docs/PHASES.md`
- Hardware config: `../config/hardware_two_motors.yaml`
