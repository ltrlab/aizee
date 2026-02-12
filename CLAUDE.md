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

### Most Common Commands

**Deploy code changes**:
```bash
# Deploy to Jetson rover
./scripts/deploy_jetson_rover.sh

# Deploy to RPi4 arm
./scripts/deploy_rpi4_arm.sh

# Deploy to all cameras
./scripts/deploy_all_cameras.sh
```

**Control the system**:
```bash
# Start motor control on Jetson
sudo systemctl start aizee-motor-control-rover

# Start all cameras
./scripts/start_all_cameras.sh

# Run teleop from dev machine
python python/teleop/teleop.py

# View live data in Rerun
python python/rerun_bridge.py
```

**Check system status**:
```bash
# Check all system components
./scripts/check_system_status.sh

# Check specific module
sudo systemctl status aizee-motor-control-rover
journalctl -u aizee-motor-control-rover -f
```

**Debugging**:
```bash
# Scan for motors on CAN bus
python python/nodes/find_motors.py

# Monitor CAN traffic
candump can1

# Test camera streams
./scripts/test_all_camera_streams.sh
```

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

**Primary configurations**:
- `config/hardware.yaml`: Full 6-motor system reference
- `config/hardware_jetson_rover.yaml`: Rover module (3 base motors)
- `config/hardware_rpi4_arm.yaml`: Arm module (3 arm motors)
- `config/hardware_jetson_dual_can.yaml`: Dual CAN bus setup

**Camera configurations** (per-node):
- `config/hardware_rpi4_cam_front.yaml`
- `config/hardware_rpi4_cam_rear.yaml`
- `config/hardware_rpi4_cam_left.yaml`
- `config/hardware_rpi4_cam_right.yaml`

**Testing configurations**:
- `config/hardware_two_motors.yaml`: 2-motor bench testing
- `config/hardware_three_motors.yaml`: 3-motor testing
- `config/hardware_test_gantry_on_can1.yaml`: Gantry testing
- `config/hardware_test_rover_on_can2.yaml`: Rover on can2

**Teleop configurations**:
- `config/teleop.yaml`: Full system teleop settings
- `config/teleop_rover_only.yaml`: Rover-only control

Each config contains:
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
- `comms/`: ZeroMQ communication library (command subscriber, telemetry publisher)
- `bindings/`: PyO3 Python bindings (future performance-critical paths)
- `lidar_control/`: RPLiDAR A1M8 driver and ZMQ publisher
  - dmweis/rplidar_driver SDK integration
  - Dual sensor support with auto-reconnect
  - Point cloud publishing at ~5Hz

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

## Future Extensions

Planned but not yet implemented:
- PyO3 bindings for Rust→Python performance-critical paths
- Sensor fusion (LiDAR + depth cameras)
- Autonomous navigation behaviors
- Multi-robot coordination
