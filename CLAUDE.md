# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AIZEE is a modular mobile manipulation robotics platform featuring:
- **6 ROBSTRIDE motors** (CAN bus): 3 for the wheeled base (2 drive wheels + 1 swivel), 3 for the arm
- **NVIDIA Jetson Orin Nano**: Rover module controller (base motors)
- **Raspberry Pi 4 (Arm)**: Arm module controller (arm motors)
- **4× Raspberry Pi 4**: Camera nodes with Intel RealSense D455 RGB-D cameras
- **ZeroMQ**: Inter-process communication for commands and telemetry
- **Rerun**: Real-time visualization and MCAP data logging

### Multi-Device Architecture

AIZEE uses a modular architecture where each functional subsystem runs on a separate compute module:
- **Rover Module** (Jetson 192.168.0.27): Base motors, ZMQ :5555/:5556
- **Arm Module** (RPi4 192.168.0.28): Arm motors, ZMQ :5557/:5558
- **Torso Module** (RPi4, future): Servo-based torso, ZMQ :5559/:5560

Each module runs an independent motor_control instance with module-specific configuration.
Unified teleop on dev machine controls all modules simultaneously.

See `docs/MULTI_DEVICE_DEPLOYMENT.md` and `docs/QUICK_START_MULTIDEVICE.md` for deployment guides.

## Development Commands

### Building

```bash
# Build all Rust workspace crates
cd rust
cargo build --release

# Build specific crate
cd rust/motor_control
cargo build --release

# Install Python dependencies
pip install -r requirements.txt
```

### Testing

```bash
# Run Rust tests
cd rust
cargo test

# Run specific crate tests
cargo test -p motor_control
cargo test -p comms

# Run Python tests (when available)
pytest python/
```

### Running the System

```bash
# Setup CAN interface (run once, requires sudo)
sudo ./scripts/setup_can.sh

# Launch motor control on Jetson
./scripts/launch_motor_control.sh
# Or manually with custom config:
AIZEE_CONFIG=path/to/config.yaml ./rust/motor_control/target/release/motor_control

# Start teleop interface (from dev machine or Jetson)
python python/teleop/simple_test.py

# Start camera node on RPi (via systemd)
sudo systemctl start aizee-camera
```

### Configuration

The system is configured via `config/hardware.yaml`, which contains:
- Motor CAN IDs and physical parameters (limits, max torque/velocity)
- Control loop frequencies (1kHz arm, 100Hz base)
- Network topology (IP addresses, ZeroMQ endpoints)
- Camera configuration (resolution, frame rate, compression)

Use `AIZEE_CONFIG` environment variable to specify alternate config file.

### Logging

Set Rust log level: `RUST_LOG=debug ./scripts/launch_motor_control.sh`

Logs are stored in `logs/` directory as MCAP files when using Rerun integration.

## Deployment to Jetson Orin Nano

### Current Development Setup

**Jetson Connection Details:**
- IP Address: 192.168.0.26 (local network)
- Username: `ltr`
- Password: `changeme!123`
- SSH Key: `P:/Workspace/ssh-keys/aizee_rover_id` (no passphrase)

### Deploying Code Changes

To deploy the codebase to the Jetson from the development machine:

```bash
# Deploy entire aizee directory to Jetson
cd /p/Workspace
scp -i /p/Workspace/ssh-keys/aizee_rover_id -r aizee ltr@192.168.0.26:~/

# Or deploy specific files/directories
scp -i /p/Workspace/ssh-keys/aizee_rover_id -r rust/motor_control ltr@192.168.0.26:~/aizee/rust/
```

### Building on Jetson

After deploying code, SSH into the Jetson and build:

```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.26

# On Jetson:
cd ~/aizee
source ~/.cargo/env  # Load Rust environment

# Build motor control
cd rust/motor_control
cargo build --release
```

### TODO: Automated Deployment

**Setup GitHub SSH key for direct git pull on Jetson:**
1. Generate SSH key on Jetson: `ssh-keygen -t ed25519 -C "aizee-rover"`
2. Add public key to GitHub repository deploy keys
3. Configure git on Jetson: `git config --global user.name/email`
4. Clone with SSH: `git clone git@github.com:ltrlab/aizee.git`
5. For updates: `cd ~/aizee && git pull`

This will enable direct git operations on the Jetson instead of requiring SCP from development machine.

## Architecture

### Multi-Language System

**Rust** (`rust/` directory):
- `motor_control/`: Main motor controller binary with deterministic control loops
- `comms/`: ZeroMQ communication library (command subscriber, telemetry publisher)
- `bindings/`: PyO3 Python bindings (future performance-critical paths)

**Python** (`python/` directory):
- `aizee/`: Main Python package
- `teleop/`: Teleoperation interface for joystick/keyboard control
- `nodes/`: RPi camera streaming nodes

### Communication Flow

```
Teleop (Python) --[ZMQ tcp://*:5555]--> Motor Control (Rust)
                                            |
                                            | CAN bus
                                            v
                                        ROBSTRIDE Motors
                                            |
                                            | Feedback
Motor Control (Rust) --[ZMQ tcp://*:5556]--> Rerun Bridge (Python)
                                                  |
                                                  v
RPi Cameras (Python) --[ZMQ tcp://*:5557]---> Rerun Bridge
                                                  |
                                                  v
                                          Visualization + MCAP Logging
```

### ZeroMQ Message Formats

**Commands** (JSON on tcp://*:5555):
```json
{"type": "drive", "linear": 0.5, "angular": 0.2}
{"type": "arm_joints", "positions": [0.1, 0.5, -0.3], "velocities": [0.0, 0.0, 0.0]}
{"type": "enable", "motor_ids": ["left_wheel", "shoulder_pitch"]}
{"type": "emergency_stop"}
```

**Telemetry** (JSON on tcp://*:5556):
```json
{
  "timestamp": 1234567890.123,
  "motors": {
    "left_wheel": {"position": 1.5, "velocity": 0.5, "torque": 2.1, "temperature": 45.0, "error": null}
  }
}
```

See `rust/comms/src/messages.rs` for full message schemas.

### Control System Architecture

The Rust motor control (`rust/motor_control/src/main.rs`) implements:

1. **Dual control loops** via tokio async:
   - **Arm loop** (1kHz): High-frequency position control for 3 arm joints
   - **Base loop** (100Hz): Velocity control for 2 drive wheels + 1 swivel joint

2. **Safety features**:
   - Watchdog timeout: Stops motors if no command received for >100ms
   - Soft position limits enforced from config
   - Emergency stop command halts all motors immediately
   - Motor error detection (temperature, overcurrent, encoding faults)

3. **CAN protocol** (`rust/motor_control/src/robstride.rs`):
   - ROBSTRIDE protocol implementation
   - Extended CAN frames with motor model-specific scaling
   - Three motor models: Model02 (low torque), Model03 (medium), Model04 (high torque)

4. **Motor groups**: Base motors and arm motors are organized into groups with different control frequencies

## Key Design Patterns

### Motor Configuration
Motor parameters are loaded from YAML and converted to internal `MotorConfig` structs. Each motor has:
- `can_id`: CAN bus identifier (0x01-0x06)
- `model`: Motor type (affects torque/velocity scaling)
- `min_position`/`max_position`: Software safety limits
- `max_velocity`/`max_torque`: Physical limits

### State Machine
Motors have states: `Disabled` → `Enabling` → `Running` → `Error`
Transitions happen via CAN enable/disable frames.

### Async with Deterministic Control
Uses tokio for async I/O but maintains hard real-time control loop via `tokio::time::interval`.
The arm control loop must maintain <1ms jitter.

## Working with CAN Bus

**IMPORTANT**: Motors are connected to **`can1`** interface (not can0).

CAN interface must be configured before running motor control:
```bash
sudo ./scripts/setup_can.sh
```

Useful CAN debugging tools:
- `candump can1` - Monitor CAN traffic
- `cansend can1 001#1122334455667788` - Send test frame
- `ip link show can1` - Check interface status

The system uses SocketCAN on Linux with 1 Mbps bitrate.

### CAN Interface Configuration
- **Jetson Orin Nano**: Motors on `can1` (configured in systemd service)
- **Bitrate**: 1000000 (1 Mbps)
- **Service**: Auto-configures can1 on startup via ExecStartPre

### Tuned Control Parameters

**ROBSTRIDE03 Motors** (CAN ID 0x03):
- **Kp (position gain)**: 3.0 - Smooth motion without vibrations
- **Kd (damping gain)**: 0.3-0.8 - Good damping, minimal oscillation
- **Control frequency**: 50-100 Hz for smooth continuous motion
- **Note**: Higher gains (Kp=20, Kd=2) cause significant vibrations

**ROBSTRIDE04 Motors** (CAN ID 0x02):
- Successfully tested with sine wave and drive velocity commands
- Responds to gentle velocity commands (linear=0.15-0.5 rad/s)
- Zero position command working correctly

**CAN Protocol Format**:
- CAN ID format: `motor_id | (0xAA << 8) | (msg_type << 24)`
- Host CAN ID: `0xAA` (required for ROBSTRIDE protocol)
- Message types: Enable=3, Disable=4, Control=1, ZeroPos=6

**Testing Scripts on Jetson**:
- `~/motor_control_test.py`: Direct CAN test with tuned gains (Kp=3.0, Kd=0.3)
  - Proper signal handling (SIGINT/SIGTERM) for safe cleanup
  - Auto-disables motor on exit (testing mode only)
  - Production robots should keep motors enabled to prevent dropping loads
- `~/test_both_motors_zeroed.sh`: Tests both motors via Rust motor_control + ZeroMQ
- `~/test_individual.sh`: Tests each motor separately
- `~/scan_all_motors.py`: Scans CAN bus for all connected motors (IDs 1-127)

**Note**: See `JETSON_TEST_REVIEW.md` for complete list of test scripts and cleanup recommendations

## Development Phases

The project follows a phased implementation plan (see `docs/PHASES.md`):
- Phase 0: Foundation (✓ complete)
- Phase 1: Rust Motor Control Core (in progress)
- Phase 2: Python Teleop Interface
- Phase 3: RPi Camera Nodes
- Phase 4: Rerun Integration
- Phase 5: System Integration
- Phase 6: Milestone Demo

## Hardware Notes

### Motor Assignments

**Current Testing Setup** (2 motors on ROBSTRIDE chain):
- **CAN ID 0x02**: ROBSTRIDE04 (test motor, mapped as "left_wheel" in config)
- **CAN ID 0x03**: ROBSTRIDE03 (test motor, mapped as "right_wheel" in config)
- Test config: `config/hardware_two_motors.yaml`

**Full System Configuration** (planned, 6 motors):
- **CAN ID 0x01-0x02**: Drive wheels (ROBSTRIDE04, high torque)
- **CAN ID 0x03**: Base swivel (ROBSTRIDE03)
- **CAN ID 0x04**: Shoulder pitch (ROBSTRIDE04)
- **CAN ID 0x05**: Elbow (ROBSTRIDE03)
- **CAN ID 0x06**: Wrist/gripper (ROBSTRIDE02, compact)

### Network Topology
- **Jetson**: 192.168.0.26
- **RPi cameras**: 192.168.1.21-24 (not yet deployed)
- Gigabit Ethernet with PoE switch for camera power

### RealSense D455 Cameras
- RGB: 640×480 @ 30fps (JPEG compressed)
- Depth: 640×480 @ 30fps (16-bit)
- IMU: 200Hz (accel + gyro)

## Common Issues

**CAN interface not found**: Run `sudo ./scripts/setup_can.sh` to configure

**Permission denied on CAN socket**: Add user to `dialout` group or run with sudo

**Motor not responding**: Check CAN wiring, verify motor is powered, ensure CAN ID matches config

**High latency**: Check network bandwidth with `iperf3`, reduce camera compression quality

**Control loop jitter**: Ensure RUST_LOG=info or error (debug logging adds latency)

## Code Style

### Rust
- Use workspace dependencies defined in root `Cargo.toml`
- Error handling via `anyhow::Result` for applications, `thiserror` for libraries
- Logging with `tracing` crate
- Keep control loop code deterministic (avoid allocations in hot path)

### Python
- Format with `black`
- Type hints with `mypy`
- Use `pytest` for tests

## Future Extensions

Planned but not yet implemented:
- PyO3 bindings for Rust→Python performance-critical paths
- 2× SLAMTEC RPLiDAR integration
- Sensor fusion (LiDAR + depth cameras)
- Autonomous navigation behaviors
- Multi-robot coordination
