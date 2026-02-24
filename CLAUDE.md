# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

AIZEE is a modular mobile manipulation robotics platform:
- **9 ROBSTRIDE motors** on CAN bus: 3 for the wheeled base (2 drive wheels + 1 swivel), 6 for the gantry arm (6DoF)
- **NVIDIA Jetson Orin Nano** (192.168.0.27): Main controller, all 6 motors on `can1`, ZMQ :5555/:5556
- **4× Raspberry Pi 4** (10.42.0.11–14, PoE Ethernet): Camera nodes with Intel RealSense D455 RGB-D
- **2× RPLiDAR A1M8**: 360° scanning on Jetson via USB, ZMQ :5561
- **ZeroMQ**: Inter-process communication for all commands and telemetry
- **Rerun**: Real-time visualization and MCAP data logging

## Build Commands

```bash
# Build all Rust crates
cd rust && cargo build --release

# Build specific crate
cargo build -p motor_control --release
cargo build -p lidar_control --release

# Run Rust tests
cargo test --all
cargo test -p motor_control

# Python dependencies
pip install -r requirements.txt

# Python tests (when available)
pytest python/
```

## Development Workflow

**Local → Deploy → Test cycle:**

```bash
# 1. Build and test locally
cd rust && cargo build --release

# 2. Deploy to target
./scripts/deploy_jetson_rover.sh        # Motor/LiDAR code changes
./scripts/deploy_rpi4_arm.sh            # Arm module changes
./scripts/deploy_all_cameras.sh         # Camera node changes

# 3. Rebuild on Jetson after deploy
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27
cd ~/aizee && source ~/.cargo/env && cd rust/motor_control && cargo build --release

# 4. Restart services
sudo systemctl restart aizee-motor-control-rover

# 5. Test
python python/teleop/teleop.py          # Full teleop
python python/rerun_bridge.py           # Live data visualization
```

**Where to deploy when changing:**
- `rust/motor_control/`, `rust/lidar_control/`, `python/nodes/ups_node.py`, `config/hardware_jetson_rover.yaml` → Jetson (192.168.0.27)
- `python/camera_relay.py`, `config/systemd/aizee-camera-relay.service` → Jetson (relay service)
- `python/nodes/camera_node.py`, `config/hardware_rpi4_cam_*.yaml` → Camera Pis (via Jetson hop)
- `python/teleop/`, `python/rerun_bridge.py` → Dev machine only

**SSH access:**
- Key: `P:/Workspace/ssh-keys/aizee_rover_id`
- User: `ltr` on all nodes
- Pis are on PoE subnet (10.42.0.0/24) — reach via Jetson hop (NOT `-J` ProxyJump — that doesn't forward the key to the jump host):
  ```bash
  # Interactive shell on a Pi
  ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
      "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11"
  # Run a command on a Pi
  ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
      "ssh -i ~/.ssh/aizee_rover_id -o StrictHostKeyChecking=no ltr@10.42.0.11 'cmd'"
  ```
- Jetson has the Pi SSH key at `~/.ssh/aizee_rover_id` (deployed by `./scripts/deploy_jetson_rover.sh`)
- New Pi: run `./scripts/setup_pi_ethernet.sh <1-4>` to bootstrap key auth + static IP

## Running the System

```bash
# One-time CAN setup (requires sudo)
sudo ./scripts/setup_can.sh

# Start all components
./scripts/start_all_modules.sh          # Rover + arm modules
./scripts/start_all_cameras.sh          # All 4 camera nodes

# Or launch motor control manually with custom config
AIZEE_CONFIG=config/hardware_jetson_rover.yaml ./rust/motor_control/target/release/motor_control
RUST_LOG=debug ./scripts/launch_motor_control.sh   # With debug logging

# System health check
./scripts/check_system_status.sh
sudo systemctl status aizee-motor-control-rover
journalctl -u aizee-motor-control-rover -f
```

## Architecture

### Multi-Language System

**Rust** (`rust/` — Cargo workspace with 4 crates):
- `motor_control/`: Binary — CAN bus driver, dual control loops (1kHz arm, 100Hz base), ZMQ interface. Main entry point: `src/main.rs`. Depends on `comms`.
- `comms/`: Library — ZMQ abstractions and shared message schemas (`src/messages.rs`)
- `lidar_control/`: Binary — RPLiDAR A1M8 driver, dual-sensor support, ZMQ publisher at ~5Hz
- `bindings/`: Reserved PyO3 library placeholder (not yet implemented)

**Python** (`python/`):
- `teleop/teleop.py`: Full system controller with curses UI, Xbox gamepad + keyboard input
- `rerun_bridge.py`: Aggregates all ZMQ streams; logs MCAP to `logs/`
- `nodes/camera_node.py`: RealSense D455 capture (RGB + depth + IMU), JPEG compressed, ZMQ publisher
- `nodes/ups_node.py`: INA219 I2C battery monitoring, ZMQ publisher

### Communication Flow

```
Teleop (Python)       --[ZMQ tcp://*:5555]-->  Motor Control (Rust)
                                                    | CAN bus (1 Mbps)
                                                    v
                                               ROBSTRIDE Motors
                                                    | Feedback
Motor Control (Rust)  --[ZMQ tcp://*:5556]-->  Rerun Bridge (Python)
RPi Cameras (Python)  --[ZMQ tcp://*:5557-5560]-->  Camera Relay (Jetson)  --[ZMQ tcp://*:5557-5560]-->  Rerun Bridge
LiDAR (Rust)          --[ZMQ tcp://*:5561]-->  Rerun Bridge
UPS (Python)          --[ZMQ tcp://*:5562]-->  Rerun Bridge
                                                    v
                                            Visualization + MCAP logs/
```

### ZMQ Message Formats

Defined in `rust/comms/src/messages.rs`:

**Commands** (JSON, port 5555):
```json
{"type": "drive", "linear": 0.5, "angular": 0.2}
{"type": "arm_joints", "positions": [0.1, 0.5, -0.3], "velocities": [0.0, 0.0, 0.0]}
{"type": "enable", "motor_ids": ["left_wheel", "gantry_base"]}
{"type": "emergency_stop"}
```

**Telemetry** (JSON, port 5556):
```json
{
  "timestamp": 1234567890.123,
  "motors": {
    "left_wheel": {"position": 1.5, "velocity": 0.5, "torque": 2.1, "temperature": 45.0, "error": null}
  }
}
```

### Control System

**Dual control loops** (in `rust/motor_control/src/main.rs` via tokio):
- **Arm loop (1kHz)**: MIT mode impedance control for gantry_base, gantry_mid, gantry_end
  - Kp = [5.0, 5.0, 2.0], Kd = [0.2, 0.2, 0.1] per joint
  - Position clamping at ±π rad to prevent motor faults
  - Velocity feedforward for smooth joystick tracking
  - Asymmetric ramping: 8.0 rad/s² accel, 4.0 rad/s² decel
- **Base loop (100Hz)**: Velocity control for left_wheel, right_wheel, swivel
  - Limits: 2.0 rad/s linear, 1.5 rad/s angular, 1.0 rad/s swivel

**Safety**:
- Watchdog: Stops motors if no command for >100ms
- Emergency stop halts all motors immediately
- Motor fault detection (temperature, overcurrent, encoding errors)
- `arm loop must maintain <1ms jitter — avoid allocations in hot path`

**Motor state machine** (`rust/motor_control/src/motor.rs`):
`Disabled → Enabling → Running → Error` — transitions via CAN enable/disable frames

**CAN protocol** (`rust/motor_control/src/robstride.rs`):
- ROBSTRIDE motors on `can1` at 1 Mbps (not can0)
- CAN ID format: `motor_id | (0xAA << 8) | (msg_type << 24)`, host ID = `0xAA`
- Four motor models: Model00 (micro, ~2 Nm), Model02 (low torque), Model03 (medium), Model04 (high torque)

### Hardware Assignments

| Motor | CAN ID | Model | Group |
|---|---|---|---|
| left_wheel | 0x02 | ROBSTRIDE04 | Base (100Hz) |
| swivel | 0x03 | ROBSTRIDE03 | Base (100Hz) |
| right_wheel | 0x04 | ROBSTRIDE04 | Base (100Hz) |
| gantry_base | 0x05 | ROBSTRIDE04 | Arm (1kHz) |
| gantry_mid | 0x06 | ROBSTRIDE03 | Arm (1kHz) |
| gantry_end | 0x07 | ROBSTRIDE02 | Arm (1kHz) |
| wrist_pitch | 0x08 | ROBSTRIDE02 | Arm (1kHz) |
| wrist_roll | 0x09 | ROBSTRIDE00 | Arm (1kHz) |
| gripper | 0x0A | ROBSTRIDE00 | Arm (1kHz) |

### Arm FK Geometry

The arm is mounted 0.200 m above the rover base frame (`world/rover/arm`). All links extend along the local +X axis; rotations are in radians.

| Segment | Length | Parent joint | Child joint | Rotation axis |
|---|---|---|---|---|
| link_0 | 0.5906 m | gantry_base | gantry_mid | Z (yaw) |
| link_1 | 0.5649 m | gantry_mid | gantry_end | Y (pitch) |
| link_2 | 0.100 m | gantry_end | wrist_pitch | Y (pitch) |
| link_3 | 0.1063 m | wrist_pitch | wrist_roll | Y (pitch) |
| link_5 | 0.132 m | wrist_roll | gripper tip | X (roll) → Z (gripper open/close) |

**Rerun entity hierarchy** (all under `world/rover/arm/`):
```
joint_base                          ← gantry_base pos, rot Z
  link_0  [0→0.5906, 0, 0]
  joint_mid                         ← gantry_mid pos, rot Y, at [0.5906,0,0]
    link_1  [0→0.5649, 0, 0]
    joint_end                       ← gantry_end pos, rot Y, at [0.5649,0,0]
      link_2  [0→0.100, 0, 0]
      joint_wrist_pitch             ← wrist_pitch pos, rot Y, at [0.100,0,0]
        link_3  [0→0.1063, 0, 0]
        joint_wrist_roll            ← wrist_roll pos, rot X, at [0.1063,0,0]
          link_5  [0→0.132, 0, 0]
          joint_gripper             ← gripper pos, rot Z, at [0.132,0,0]
```

> **Note**: wrist_pitch→wrist_roll link length (L4) is not yet measured; the two joints are currently treated as coincident. Update `L4` in `rerun_bridge.py` and add `link_4` once measured.

### Network Topology

Two subnets:
- **WiFi** (192.168.0.0/24): dev machine ↔ Jetson
- **PoE Ethernet** (10.42.0.0/24): Jetson ↔ Pis only (not directly reachable from dev)

| Node | WiFi IP | PoE IP | ZMQ Ports |
|---|---|---|---|
| Jetson (Rover) | 192.168.0.27 (`wlP1p1s0`) | 10.42.0.1 (`enP8p1s0`) | :5555 cmd, :5556 telemetry, :5557–5560 camera relay, :5561 lidar, :5562 ups |
| RPi4 Arm (alt) | 192.168.0.28 | — | :5557 cmd, :5558 telemetry |
| cam_front (PI-1) | — | 10.42.0.11 | :5557 (published to Jetson only) |
| cam_rear (PI-2) | — | 10.42.0.12 | :5558 |
| cam_left (PI-3) | — | 10.42.0.13 | :5559 |
| cam_right (PI-4) | — | 10.42.0.14 | :5560 |

**Camera relay** (`python/camera_relay.py`, service `aizee-camera-relay` on Jetson): subscribes to Pi ZMQ streams on PoE subnet, re-publishes on all Jetson interfaces so dev machine can connect to `tcp://192.168.0.27:5557-5560`.

**Jetson DHCP**: `dnsmasq` on Jetson assigns leases to Pis on 10.42.0.0/24; leases in `/var/lib/misc/dnsmasq.leases`. Config in `/etc/dnsmasq.d/aizee-poe.conf`.

**Pi internet access (NAT masquerade)**: Pis have no direct internet. When internet is needed on Pis (e.g., apt-get, git clone for builds), enable NAT on Jetson:
```bash
# On Jetson — enable NAT (survives until reboot):
sudo iptables -t nat -A POSTROUTING -s 10.42.0.0/24 -o wlP1p1s0 -j MASQUERADE
echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward

# On each Pi — add default route and DNS:
sudo ip route add default via 10.42.0.1
echo -e "nameserver 8.8.8.8\nnameserver 1.1.1.1" | sudo tee /etc/resolv.conf
```
Note: these settings are runtime-only and don't persist across reboots.

## Configuration

**Primary configs** (selected via `AIZEE_CONFIG` env var):
- `config/hardware_jetson_rover.yaml`: Production — all 6 motors on can1
- `config/hardware_rpi4_arm.yaml`: Alternate arm module on can0
- `config/hardware_jetson_dual_can.yaml`: Dual CAN (can1=base, can2=gantry)

**Test/bench configs**:
- `config/hardware_two_motors.yaml`: 2-motor bench testing
- `config/hardware_three_motors.yaml`: 3-motor testing

**Teleop configs**: `config/teleop.yaml` (full system), `config/teleop_rover_only.yaml`

**Camera configs**: `config/hardware_rpi4_cam_{front,rear,left,right}.yaml`

## Systemd Services

Services defined in `config/systemd/`, deployed on respective nodes:

```bash
# Manage services
sudo systemctl {start|stop|restart|status|enable} aizee-motor-control-rover
sudo journalctl -u aizee-motor-control-rover -f
```

Services: `aizee-motor-control-rover`, `aizee-motor-control-arm`, `aizee-camera-relay` (Jetson), `aizee-camera-cam_{front,rear,left,right}` (Pis), `aizee-lidar-control`, `aizee-ups-monitor`

## Teleop Interface

**Controls** (in `python/teleop/teleop.py`):
- A button / `e` key: Enable all motors
- B button / `q` key: Disable all motors (requires second press to confirm)
- Back button / `space`: Emergency stop
- Start button / `r` key: Clear estop + faults
- H key: Home all arm joints (sets current position as zero — do this after enabling)
- Right stick Y-axis: gantry_base continuous velocity
- Keys 1/2: gantry_base ±0.02 rad
- Keys 3/4: gantry_mid ±0.02 rad
- Keys 5/6: gantry_end ±0.02 rad
- Keys 7/8: wrist_pitch ±0.02 rad
- Keys [/]: wrist_roll ±0.02 rad
- Keys -/=: gripper ±0.02 rad
- Z/C keys: swivel ±0.01 rad
- X key: safe shutdown (move all joints to zero, then disable)

## Debugging

```bash
# CAN diagnostics (on Jetson)
candump can1                              # Monitor CAN frames
python python/nodes/find_motors.py        # Scan for connected motors (IDs 1-127)
cansend can1 001#1122334455667788         # Send test frame
ip link show can1                         # Check interface status

# Camera diagnostics
./scripts/test_all_camera_streams.sh      # Tests via Jetson relay (192.168.0.27:5557-5560)
journalctl -u aizee-camera-relay -f       # Camera relay on Jetson
# On Pi via Jetson hop:
# ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'lsusb | grep Intel'"
# ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 "ssh -i ~/.ssh/aizee_rover_id ltr@10.42.0.11 'journalctl -u aizee-camera-cam_front -n 30'"

# LiDAR diagnostics
python python/test_lidar_telemetry.py
ls -l /dev/rplidar_*                      # Check udev symlinks
sudo ./scripts/install_lidar_udev.sh      # Fix missing udev rules

# Network diagnostics
python python/teleop/test_connectivity.py
python python/teleop/detailed_motor_test.py
```

## Common Issues

**CAN interface not found**: `sudo ./scripts/setup_can.sh` (single) or `sudo ./scripts/setup_dual_can_jetson.sh` (dual)

**Dual CAN interfaces swapping/unavailable on reboot**: `sudo ./scripts/reset_dual_can_and_motors.sh`; see `FIX_CAN1.md` for kernel/device-tree details

**Motor not responding**:
1. `candump can1` — verify frames are transmitting
2. Check CAN ID matches config (easy to have can0 vs can1 mismatch)
3. `python python/nodes/find_motors.py` — scan for live motors
4. Motor temperature fault (>70°C): wait 5–10 min to cool

**Control loop jitter**: Set `RUST_LOG=info` or `error` — debug logging adds measurable latency

**SSH deployment fails (Jetson)**: `./scripts/setup_ssh_keys.sh`; test with `ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27`

**SSH deployment fails (Pi)**: Pis are on PoE subnet — use Jetson hop (not `-J` ProxyJump). For a new Pi, run `./scripts/setup_pi_ethernet.sh <1-4>` to install key and set static IP. Pis have passwordless sudo.

## Code Style

**Rust**:
- Workspace dependencies defined in root `Cargo.toml`
- `anyhow::Result` for binary error handling, `thiserror` for library crates
- `tracing` crate for logging
- No allocations in control loop hot paths

**Python**:
- Format with `black`, type hints verified with `mypy`
- Tests with `pytest`

## Key Documentation

- `docs/README.md`: Documentation index
- `docs/quickstart/`: Setup guides (multidevice, post-reboot, Jetson)
- `docs/deployment/`: Deployment procedures and troubleshooting
- `docs/subsystems/`: Camera, LiDAR, UPS component docs
- `docs/archive/`: Historical docs (may be outdated)
