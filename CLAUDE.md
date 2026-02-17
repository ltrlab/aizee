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

## Quick Reference

### Most Common Commands (By Task)

**Daily Startup**:
```bash
# Start all system components
./scripts/start_all_modules.sh          # Start rover + arm
./scripts/start_all_cameras.sh          # Start all cameras
python python/teleop/teleop.py           # Launch teleop interface
python python/rerun_bridge.py            # View live data
```

**After Code Changes**:
```bash
# Deploy to specific modules
./scripts/deploy_jetson_rover.sh         # Deploy to Jetson rover
./scripts/deploy_rpi4_arm.sh             # Deploy to RPi4 arm
./scripts/deploy_all_cameras.sh          # Deploy to all cameras

# Restart services after deployment
sudo systemctl restart aizee-motor-control-rover
python python/teleop/teleop.py           # Test changes
```

**System Health Check**:
```bash
# Check all system components
./scripts/check_system_status.sh

# Check specific module status and logs
sudo systemctl status aizee-motor-control-rover
journalctl -u aizee-motor-control-rover -f
```

**Debugging Issues**:
```bash
# CAN bus diagnostics
python python/nodes/find_motors.py       # Scan for motors
candump can1                             # Monitor CAN traffic

# Camera diagnostics
./scripts/test_all_camera_streams.sh     # Test all cameras
lsusb | grep Intel                       # Check USB connections

# LiDAR diagnostics
python python/test_lidar_telemetry.py    # Test LiDAR data
```

## Development Workflow

### Typical Development Cycle

**Local Development**:
1. Make code changes in local workspace (P:/Workspace/aizee)
2. Test Rust builds locally: `cd rust && cargo build --release`
3. Run Python tests locally: `pytest python/` (when available)

**Deployment**:
1. Deploy to target module(s):
   - Rover (Jetson): `./scripts/deploy_jetson_rover.sh`
   - Arm (RPi4): `./scripts/deploy_rpi4_arm.sh`
   - Cameras: `./scripts/deploy_all_cameras.sh`
2. SSH into module to rebuild if needed
3. Restart services: `sudo systemctl restart aizee-motor-control-rover`

**Testing on Hardware**:
1. Check system status: `./scripts/check_system_status.sh`
2. Monitor logs: `journalctl -u aizee-motor-control-rover -f`
3. Run teleop: `python python/teleop/teleop.py`
4. View in Rerun: `python python/rerun_bridge.py`

**Debugging**:
1. Check CAN bus: `candump can1`
2. Scan for motors: `python python/nodes/find_motors.py`
3. Test individual components with component-specific test scripts
4. Review systemd logs: `journalctl -u <service-name> -f`

### When to Deploy to Which Module

**Deploy to Jetson Rover (192.168.0.27)** when changing:
- Motor control code (`rust/motor_control/`)
- LiDAR code (`rust/lidar_control/`)
- UPS monitoring (`python/nodes/ups_node.py`)
- Base motor configurations (`config/hardware_jetson_rover.yaml`)
- Rover-specific scripts

**Deploy to RPi4 Arm (192.168.0.28)** when changing:
- Arm motor control code
- Arm motor configurations (`config/hardware_rpi4_arm.yaml`)

**Deploy to Camera Nodes (192.168.0.22-25)** when changing:
- Camera node code (`python/nodes/camera_node.py`)
- Camera configurations (`config/hardware_rpi4_cam_*.yaml`)

**Update on Dev Machine Only** when changing:
- Teleop code (`python/teleop/`)
- Rerun bridge (`python/rerun_bridge.py`)
- Test scripts that run locally

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

## Testing Strategy

### Unit Tests

```bash
# Rust unit tests (all crates)
cd rust
cargo test --all

# Specific crate tests
cargo test -p motor_control
cargo test -p comms
cargo test -p lidar_control

# Python unit tests (when available)
pytest python/tests/
```

### Integration Tests

```bash
# Test motor control with minimal config
AIZEE_CONFIG=config/hardware_two_motors.yaml ./rust/motor_control/target/release/motor_control

# Test connectivity to all modules
python python/teleop/test_connectivity.py

# Test individual motors
python python/teleop/detailed_motor_test.py

# Test both motors with ZeroMQ control
./tests/integration/test_both_motors.sh
```

### System Tests

```bash
# Full system startup sequence
./scripts/start_all_modules.sh           # Start rover + arm modules
./scripts/start_all_cameras.sh           # Start all 4 cameras

# Verify all components running
./scripts/check_system_status.sh

# Run full teleop test
python python/teleop/teleop.py

# Verify Rerun data aggregation
python python/rerun_bridge.py
```

### Hardware Debugging Tests

```bash
# CAN bus diagnostics
candump can1                              # Monitor CAN traffic in real-time
python python/nodes/find_motors.py        # Scan for all connected motors (IDs 1-127)
cansend can1 001#1122334455667788         # Send test CAN frame

# Camera diagnostics
./scripts/test_all_camera_streams.sh      # Test streaming from all 4 cameras
lsusb | grep Intel                        # Verify RealSense USB connections
python python/test_camera_subscriber.py   # Test camera ZMQ subscriber

# LiDAR diagnostics
python python/test_lidar_telemetry.py     # Verify LiDAR ZMQ telemetry
ls -l /dev/rplidar_*                      # Check udev device symlinks
sudo journalctl -u aizee-lidar-control -f # Monitor LiDAR service logs

# Battery/UPS diagnostics
python python/teleop/test_battery_display.py  # Test UPS monitoring integration
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
sudo systemctl start aizee-camera-cam_front  # Or cam_rear, cam_left, cam_right
```

### Systemd Services

The system uses systemd for auto-starting components on boot:

**Motor Control** (`config/systemd/`):
- `aizee-motor-control-rover.service`: Rover module (Jetson, can1)
- `aizee-motor-control-arm.service`: Arm module (RPi4, can0)

**Sensors**:
- `aizee-camera-cam_front.service`: Front camera node
- `aizee-camera-cam_rear.service`: Rear camera node
- `aizee-camera-cam_left.service`: Left camera node
- `aizee-camera-cam_right.service`: Right camera node
- `aizee-lidar-control.service`: Dual RPLiDAR control
- `aizee-ups-monitor.service`: Battery monitoring (INA219)

**Managing services**:
```bash
# Enable auto-start on boot
sudo systemctl enable aizee-motor-control-rover

# Start/stop/restart service
sudo systemctl start aizee-motor-control-rover
sudo systemctl stop aizee-motor-control-rover
sudo systemctl restart aizee-motor-control-rover

# Check status and logs
sudo systemctl status aizee-motor-control-rover
sudo journalctl -u aizee-motor-control-rover -f
```

### Configuration

The system uses module-specific YAML configuration files:

**Primary configurations** (production use):
- `config/hardware.yaml`: Full 6-motor system reference (documentation only)
- `config/hardware_jetson_rover.yaml`: **Rover module** (3 base motors on can1) - Used by Jetson
- `config/hardware_rpi4_arm.yaml`: **Arm module** (3 arm motors on can0) - Used by Arm Pi
- `config/hardware_jetson_dual_can.yaml`: Dual CAN bus setup (if using both rover and gantry on Jetson)

**Camera configurations** (per-node, used by camera systemd services):
- `config/hardware_rpi4_cam_front.yaml` - Camera Pi 192.168.0.22
- `config/hardware_rpi4_cam_rear.yaml` - Camera Pi 192.168.0.23
- `config/hardware_rpi4_cam_left.yaml` - Camera Pi 192.168.0.24
- `config/hardware_rpi4_cam_right.yaml` - Camera Pi 192.168.0.25

**Testing configurations** (bench testing and debugging):
- `config/hardware_two_motors.yaml`: 2-motor bench testing (CAN IDs 0x02, 0x03)
- `config/hardware_three_motors.yaml`: 3-motor testing
- `config/hardware_test_gantry_on_can1.yaml`: Gantry motors on can1
- `config/hardware_test_rover_on_can2.yaml`: Rover motors on can2

**Teleop configurations**:
- `config/teleop.yaml`: **Full system** teleop (rover + arm + cameras) - Default
- `config/teleop_rover_only.yaml`: Rover-only control (for testing base without arm)

**Configuration contents**:
Each config file specifies:
- Motor CAN IDs and physical parameters (limits, max torque/velocity)
- Control loop frequencies (1kHz arm, 100Hz base)
- Network topology (IP addresses, ZeroMQ endpoints)
- Camera configuration (resolution, frame rate, compression)

**Using alternate configs**:
```bash
# Specify config via environment variable
AIZEE_CONFIG=config/hardware_two_motors.yaml ./rust/motor_control/target/release/motor_control

# Or via command-line argument (if supported by the binary)
./rust/motor_control/target/release/motor_control --config config/hardware_jetson_rover.yaml
```

**When to use which config**:
- **Daily operation**: Use `hardware_jetson_rover.yaml` on Jetson, `hardware_rpi4_arm.yaml` on Arm Pi
- **Bench testing**: Use `hardware_two_motors.yaml` or `hardware_three_motors.yaml`
- **Dual CAN setup**: Use `hardware_jetson_dual_can.yaml` with both can1 and can2
- **Debugging single motor**: Use `hardware_two_motors.yaml` and modify motor IDs as needed

### Logging

Set Rust log level: `RUST_LOG=debug ./scripts/launch_motor_control.sh`

Logs are stored in `logs/` directory as MCAP files when using Rerun integration.

## Deployment to Jetson Orin Nano

### Current Development Setup

**Jetson Connection Details:**
- IP Address: 192.168.0.27 (local network)
- Username: `ltr`
- Password: `changeme!123`
- SSH Key: `P:/Workspace/ssh-keys/aizee_rover_id` (no passphrase)

### Deploying Code Changes

To deploy the codebase to the Jetson from the development machine:

```bash
# Deploy entire aizee directory to Jetson
cd /p/Workspace
scp -i /p/Workspace/ssh-keys/aizee_rover_id -r aizee ltr@192.168.0.27:~/

# Or deploy specific files/directories
scp -i /p/Workspace/ssh-keys/aizee_rover_id -r rust/motor_control ltr@192.168.0.27:~/aizee/rust/
```

### Building on Jetson

After deploying code, SSH into the Jetson and build:

```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

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

**Rust** (`rust/` directory) - Cargo workspace with 4 crates:
- `motor_control/`: Main motor controller binary with deterministic control loops
  - CAN bus interface (SocketCAN)
  - ROBSTRIDE protocol implementation
  - Dual control loops (1kHz arm, 100Hz base)
  - ZMQ command/telemetry interface
  - **Built as**: `./rust/motor_control/target/release/motor_control`
  - **Dependencies**: `comms` crate for ZMQ abstractions
- `comms/`: ZeroMQ communication library (command subscriber, telemetry publisher)
  - Shared library used by `motor_control` and `lidar_control`
  - Message serialization/deserialization
  - **Not a binary**, library crate only
- `bindings/`: PyO3 Python bindings (future performance-critical paths)
  - Placeholder for Rust→Python bindings
  - **Not yet implemented**, reserved for performance optimization
- `lidar_control/`: RPLiDAR A1M8 driver and ZMQ publisher
  - dmweis/rplidar_driver SDK integration
  - Dual sensor support with auto-reconnect
  - Point cloud publishing at ~5Hz
  - **Built as**: `./rust/lidar_control/target/release/lidar_control`
  - **Dependencies**: `comms` crate

**Rust workspace structure**:
```
rust/
├── Cargo.toml              # Workspace definition
├── motor_control/
│   ├── Cargo.toml         # Binary crate
│   ├── src/
│   │   ├── main.rs        # Entry point
│   │   ├── robstride.rs   # CAN protocol
│   │   └── ...
│   └── target/release/motor_control  # Built binary
├── lidar_control/
│   ├── Cargo.toml         # Binary crate
│   └── src/lib.rs
├── comms/
│   ├── Cargo.toml         # Library crate
│   └── src/
│       ├── lib.rs         # Library entry point
│       └── messages.rs    # Message schemas
└── bindings/
    ├── Cargo.toml         # PyO3 library crate
    └── src/lib.rs
```

**Building the Rust workspace**:
```bash
# Build all crates
cd rust
cargo build --release

# Build specific crate
cargo build -p motor_control --release
cargo build -p lidar_control --release

# Run tests
cargo test --all
```

**Python** (`python/` directory):
- `aizee/`: Main Python package (placeholder for future shared utilities)
- `teleop/`: Teleoperation interface for joystick/keyboard control
  - `teleop.py`: Full system controller
  - `simple_test.py`: Basic motor testing
  - `arm_teleop.py`: Arm-only control
  - `test_connectivity.py`: Network diagnostics
  - `detailed_motor_test.py`: Per-motor testing
  - `test_battery_display.py`: UPS monitoring test
- `nodes/`: Sensor nodes for RPi
  - `camera_node.py`: RealSense D455 streaming (pyrealsense2)
  - `camera_node_opencv.py`: OpenCV-based camera fallback
  - `ups_node.py`: INA219 battery monitoring
  - `find_motors.py`: CAN bus motor scanner
  - `ina219.py`: I2C battery sensor library
- `rerun_bridge.py`: Aggregates ZMQ streams for Rerun visualization
- `test_camera_subscriber.py`: Camera stream testing
- `test_lidar_telemetry.py`: LiDAR data verification

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

### Rerun Visualization Bridge

The Rerun bridge (`python/rerun_bridge.py`) aggregates all ZMQ streams for real-time visualization:
- Subscribes to motor telemetry (rover + arm modules)
- Subscribes to camera streams (4 RPi nodes)
- Subscribes to LiDAR point clouds
- Subscribes to UPS battery data
- Logs everything to MCAP files in `logs/` directory
- Renders 3D visualization with robot pose, cameras, and LiDAR

**Usage**:
```bash
# Start Rerun bridge (connects to all ZMQ endpoints)
python python/rerun_bridge.py

# Replay MCAP log file
rerun logs/session_001.mcap
```

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
- Phase 1: Rust Motor Control Core (✓ complete)
- Phase 2: Python Teleop Interface (✓ complete)
- Phase 3: RPi Camera Nodes (✓ complete)
- Phase 4: Rerun Integration (✓ complete)
- Phase 5: System Integration (✓ complete)
- Phase 6: Extensions (LiDAR, UPS, dual CAN - ✓ complete)

## Key Documentation

**Start here**: See [docs/README.md](docs/README.md) for complete documentation index.

**Documentation Structure** (reorganized February 2026):
The documentation has been reorganized into clear categories. Always use current docs from:
- `docs/quickstart/` - Up-to-date setup guides
- `docs/deployment/` - Current deployment procedures
- `docs/subsystems/` - Component-specific documentation
- `docs/archive/` - Historical docs (may be outdated, preserved for reference only)

**Quick Start Guides** (`docs/quickstart/`):
- `QUICK_START_MULTIDEVICE.md`: Fast 10-minute multi-device setup
- `QUICK_START_AFTER_REBOOT.md`: Post-reboot startup procedures
- `JETSON_QUICK_START.md`: Jetson Orin Nano setup

**Deployment** (`docs/deployment/`):
- `MULTI_DEVICE_DEPLOYMENT.md`: Multi-module architecture deployment guide
- `IMPLEMENTATION_SUMMARY.md`: Architecture implementation details
- `DEPLOYMENT_LOG.md`: Deployment history and lessons learned
- `TROUBLESHOOTING_CAN.md`: Dual CAN interface troubleshooting

**Subsystems** (`docs/subsystems/`):
- `CAMERAS.md`: Intel RealSense D455 camera system (4 RPi nodes)
- `LIDAR.md`: RPLiDAR A1M8 dual sensor integration
- `UPS.md`: INA219 battery monitoring system

**Component READMEs**:
- `rust/motor_control/README.md`: Motor control implementation
- `rust/lidar_control/README.md`: LiDAR control details
- `scripts/README_CAMERA_SCRIPTS.md`: Camera deployment scripts

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

**Active Deployment**:
- **Jetson Orin Nano (Rover)**: 192.168.0.27
  - Motor control: ZMQ :5555 (cmd sub), :5556 (telemetry pub)
  - LiDAR: ZMQ :5561
  - UPS monitor: ZMQ :5562
- **RPi4 (Arm)**: 192.168.0.28
  - Motor control: ZMQ :5557 (cmd sub), :5558 (telemetry pub)
- **RPi4 Camera Nodes**:
  - Front: 192.168.0.22 (AIZEE-ROVER-PI-1)
  - Rear: 192.168.0.23 (AIZEE-ROVER-PI-2)
  - Left: 192.168.0.24 (AIZEE-ROVER-PI-3)
  - Right: 192.168.0.25 (AIZEE-ROVER-PI-4)

**Infrastructure**:
- Gigabit Ethernet with PoE switch for camera power
- SSH keys at `P:/Workspace/ssh-keys/` for passwordless deployment

### RealSense D455 Cameras
- RGB: 640×480 @ 30fps (JPEG compressed)
- Depth: 640×480 @ 30fps (16-bit)
- IMU: 200Hz (accel + gyro)

## Common Issues

**CAN interface not found**: Run `sudo ./scripts/setup_can.sh` (single CAN) or `sudo ./scripts/setup_dual_can_jetson.sh` (dual CAN)

**Permission denied on CAN socket**: Add user to `dialout` group (`sudo usermod -a -G dialout $USER`) or run with sudo

**Motor not responding**:
- Check CAN wiring and verify motor is powered
- Ensure CAN ID matches config
- Verify correct CAN interface (can0 vs can1)
- Run `candump can1` to see if frames are being transmitted
- Use `python/nodes/find_motors.py` to scan for connected motors

**Dual CAN issues on Jetson**:
- If CAN interfaces swap or become unavailable, run `sudo ./scripts/reset_dual_can_and_motors.sh`
- See `FIX_CAN1.md` for kernel module and device tree troubleshooting

**Camera node not starting**:
- Check USB connection: `lsusb | grep Intel`
- Verify pyrealsense2 installed: `python -c "import pyrealsense2"`
- Check systemd logs: `journalctl -u aizee-camera-cam_front -f`
- Test with: `./scripts/test_all_camera_streams.sh`

**LiDAR not detected**:
- Verify USB connection: `ls -l /dev/rplidar_*`
- Check udev rules: `sudo ./scripts/install_lidar_udev.sh`
- Confirm motor spinning (should hear audible rotation)
- Check logs: `journalctl -u aizee-lidar-control -f`

**High latency**:
- Check network bandwidth with `iperf3`
- Reduce camera compression quality in config
- Verify ZMQ endpoints are correct (no firewall blocking)

**Control loop jitter**: Ensure RUST_LOG=info or error (debug logging adds latency)

**SSH deployment fails**:
- Run `./scripts/setup_ssh_keys.sh` to configure passwordless SSH
- Verify SSH keys in `P:/Workspace/ssh-keys/`
- Test connection: `ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27`

### Recently Fixed Issues (Reference)

These issues have been resolved but are documented for awareness and to help diagnose similar problems:

**Differential drive steering direction** (Fixed: February 2026):
- **Symptom**: Robot turned opposite direction from joystick input
- **Cause**: Inverted steering calculation in drive command
- **Fix**: Corrected sign in differential drive kinematics
- **Commit**: `813bd0e fix: correct differential drive steering direction`

**CAN buffer overflow causing motor faults** (Fixed: February 2026):
- **Symptom**: Gantry motors entering fault state during operation
- **Cause**: CAN transmit buffer overflow from high-frequency commands
- **Fix**: Improved buffer management and flow control
- **Commit**: `0d89b3d Fix CAN buffer overflow causing gantry motor faults`

**Dual-CAN interface swapping on boot** (Fixed: February 2026):
- **Symptom**: can1 and can2 interfaces swap or become unavailable after reboot
- **Cause**: Inconsistent device tree enumeration
- **Fix**: Use `./scripts/reset_dual_can_and_motors.sh` to reset interfaces
- **Commit**: `0e3cdc6 fix(config): correct CAN bus assignments for dual-CAN setup`

**IP address migration** (Updated: February 2026):
- **Change**: Jetson rover IP changed from 192.168.0.26 to 192.168.0.27
- **Action**: Update any hardcoded IPs in local configs or scripts
- **Commit**: `8dd46fa fix: update Jetson IP address from 192.168.0.26 to 192.168.0.27`

**Keyboard control smoothing issues** (Fixed: February 2026):
- **Symptom**: Jerky motion when using keyboard teleop
- **Fix**: Improved command smoothing and rate limiting
- **Commit**: `dfd36fc Fix keyboard control smoothing in teleop`

### Emergency Recovery Procedures

**Complete system reset after errors**:
```bash
# 1. Stop all services on Jetson
ssh ltr@192.168.0.27
sudo systemctl stop aizee-motor-control-rover
sudo systemctl stop aizee-lidar-control
sudo systemctl stop aizee-ups-monitor

# 2. Reset CAN interfaces
sudo ./scripts/reset_dual_can_and_motors.sh

# 3. Power cycle motors (if needed)
# Physically disconnect/reconnect motor power supply

# 4. Restart services
sudo systemctl start aizee-motor-control-rover
sudo systemctl start aizee-lidar-control
sudo systemctl start aizee-ups-monitor

# 5. Verify system status
./scripts/check_system_status.sh
```

**Camera node recovery** (if a camera stops responding):
```bash
# On specific camera Pi (e.g., 192.168.0.22)
ssh ltr@192.168.0.22

# Check service status
sudo systemctl status aizee-camera-cam_front

# Restart camera service
sudo systemctl restart aizee-camera-cam_front

# If still failing, check USB connection
lsusb | grep Intel

# If camera not detected, physically reconnect USB
# Then restart service
sudo systemctl restart aizee-camera-cam_front
```

**Motor fault recovery**:
```bash
# If motors enter fault state:
# 1. Check CAN traffic
candump can1

# 2. If no traffic, reset CAN interface
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can1 up

# 3. Restart motor control
sudo systemctl restart aizee-motor-control-rover

# 4. If specific motor is faulty, identify it
python python/nodes/find_motors.py

# 5. Check motor temperature (may need cool-down)
# Motors auto-fault at ~70°C, wait 5-10 minutes
```

**Network connectivity issues**:
```bash
# Test connectivity to all modules
python python/teleop/test_connectivity.py

# Check specific module
ping 192.168.0.27  # Jetson
ping 192.168.0.28  # Arm Pi
ping 192.168.0.22  # Camera front

# If module unreachable:
# 1. Check physical network connections
# 2. Verify static IP configuration on module
# 3. Check router/switch status
# 4. Restart network on module
ssh ltr@192.168.0.27
sudo systemctl restart networking
```

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

## LiDAR Integration

**Status**: ✅ Implemented (February 2026)

### Hardware
- **2× RPLiDAR A1M8** sensors on Jetson via USB serial
- **Device paths**: `/dev/rplidar_front`, `/dev/rplidar_back` (configured via udev rules)
- **Performance**: 155-165 points/scan at ~7-8 Hz, 360° coverage, 0.15-12m range

### Software
- **Rust crate**: `rust/lidar_control/` using dmweis/rplidar_driver SDK
- **ZMQ publishing**: Port 5561 at ~5Hz with polar→Cartesian conversion
- **Systemd service**: `aizee-lidar-control.service` for auto-start
- **Rerun visualization**: 3D point clouds color-coded by sensor

### Commands
```bash
# Deploy LiDAR control to Jetson
./scripts/deploy_lidar_control.sh

# Install systemd service and udev rules
sudo ./scripts/install_lidar_service.sh
sudo ./scripts/install_lidar_udev.sh

# Test LiDAR telemetry
python python/test_lidar_telemetry.py
```

See `docs/LIDAR_INTEGRATION_COMPLETE.md` for full details.

## Camera System

**Status**: ✅ Deployed across 4 RPi4 nodes

### Hardware Topology
- **cam_front** (192.168.0.22): Front-facing RealSense D455
- **cam_rear** (192.168.0.23): Rear-facing RealSense D455
- **cam_left** (192.168.0.24): Left-side RealSense D455
- **cam_right** (192.168.0.25): Right-side RealSense D455

### Deployment
Each camera node runs as a systemd service (`aizee-camera-cam_*.service`) that:
- Captures RGB (640×480 @ 30fps), depth, and IMU data
- Publishes to Jetson via ZMQ
- Auto-starts on boot with reconnection logic

### Management Scripts
```bash
# Deploy to all 4 camera nodes
./scripts/deploy_all_cameras.sh

# Start/stop all cameras
./scripts/start_all_cameras.sh
./scripts/stop_all_cameras.sh

# Test camera streams
./scripts/test_all_camera_streams.sh

# Shutdown all Pis safely
./scripts/shutdown_all_pis.sh
```

See `scripts/README_CAMERA_SCRIPTS.md` and `docs/CAMERA_DEPLOYMENT_COMPLETED.md`.

## UPS Power Monitoring

**Status**: ✅ Integrated with INA219 current sensor

The Jetson monitors battery voltage and current via I2C-connected INA219 sensor:
- **Battery monitoring**: Voltage, current, power consumption tracking
- **Low battery alerts**: Automatic warnings when voltage drops below threshold
- **ZMQ telemetry**: Published on port 5562
- **Systemd service**: `aizee-ups-monitor.service`
- **Teleop integration**: Battery status displayed in teleop UI

```bash
# Test battery display in teleop
python python/teleop/test_battery_display.py
```

See `docs/UPS_DEPLOYMENT.md` and `UPS_DEPLOYMENT_COMPLETE.md`.

## Dual CAN Interface (Jetson)

The Jetson supports dual CAN buses for separating subsystems:
- **can1**: Rover motors (wheels + swivel)
- **can2**: Gantry/arm motors (separate module)

### Setup
```bash
# Configure both CAN interfaces
sudo ./scripts/setup_dual_can_jetson.sh

# Reset dual CAN and re-enable motors
sudo ./scripts/reset_dual_can_and_motors.sh
```

**Configuration files**:
- `config/hardware_jetson_dual_can.yaml`
- `config/hardware_test_gantry_on_can1.yaml`
- `config/hardware_test_rover_on_can2.yaml`

See `FIX_CAN1.md` for troubleshooting dual CAN setups.

## System Management Scripts

### Status Checking
```bash
# Check rover module (Jetson) status
./scripts/check_rover_status.sh

# Check all system components
./scripts/check_system_status.sh
```

### Multi-Module Control
```bash
# Start all modules (rover + arm)
./scripts/start_all_modules.sh

# Deploy to specific modules
./scripts/deploy_jetson_rover.sh    # Rover module
./scripts/deploy_rpi4_arm.sh        # Arm module
```

### Camera-Specific
```bash
# Deploy using rsync (faster for iterative updates)
./scripts/deploy_rpi4_camera.sh

# Deploy using SCP (initial deployment)
./scripts/deploy_rpi4_camera_scp.sh

# Setup SSH keys for passwordless deployment
./scripts/setup_ssh_keys.sh
```

## Teleop Interface Variants

Multiple teleop scripts for different testing scenarios:
- `python/teleop/teleop.py`: Full system teleop (rover + arm + cameras)
- `python/teleop/simple_test.py`: Basic motor testing
- `python/teleop/arm_teleop.py`: Arm-only control
- `python/teleop/test_connectivity.py`: Network connectivity verification
- `python/teleop/detailed_motor_test.py`: Per-motor diagnostic tests

## Git Workflow

### Committing Changes

When making commits to this repository:

**Commit message format**:
- Use conventional commit style: `type(scope): brief description`
- Types: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`
- Examples:
  - `feat(motor_control): add emergency stop functionality`
  - `fix(teleop): correct joystick deadzone calculation`
  - `docs(cameras): update RealSense setup instructions`

**Before committing**:
1. Test changes locally (run relevant tests, verify on hardware if applicable)
2. Check that configs are not accidentally committed with sensitive data
3. Review diffs to avoid committing debug/test code

**Common workflow**:
```bash
# Check status and changes
git status
git diff

# Stage specific files
git add rust/motor_control/src/main.rs
git add config/hardware_jetson_rover.yaml

# Commit with descriptive message
git commit -m "feat(motor_control): add dual-CAN bus support"

# Push to remote
git push origin main
```

**Branch workflow** (if using feature branches):
```bash
# Create feature branch
git checkout -b feature/new-sensor-integration

# Make changes, commit
git add .
git commit -m "feat(sensors): add new sensor support"

# Push branch
git push origin feature/new-sensor-integration

# Create pull request on GitHub
gh pr create --title "Add new sensor integration"
```

### Documentation Archive

The `docs/archive/` directory contains historical documentation that has been superseded by current docs:
- Implementation notes from earlier phases
- Deployment checklists that are now automated
- Status reports from component integration

These files are preserved for reference but may be outdated. Always refer to:
- `docs/README.md` for current documentation index
- `docs/quickstart/` for up-to-date setup guides
- `docs/deployment/` for current deployment procedures
- `docs/subsystems/` for component-specific documentation

## Future Extensions

Potential next steps (not yet implemented):
- PyO3 bindings for Rust→Python performance-critical paths (if Python bottlenecks identified)
- Advanced sensor fusion (LiDAR + depth cameras for 3D mapping)
- Autonomous navigation behaviors (obstacle avoidance, path planning)
- Vision-based manipulation (object detection, grasp planning)
- Multi-robot coordination (swarm behaviors, distributed task allocation)
