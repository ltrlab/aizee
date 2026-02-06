# AIZEE - Advanced Intelligent Zero-g Exploration Environment

A modular robotics software stack for teleoperation, autonomous navigation, and multi-sensor data fusion.

## Project Vision

AIZEE is a mobile manipulation platform designed for real-time control, rich sensor integration, and reproducible data logging. The system enables both human teleoperation and autonomous behaviors while maintaining deterministic low-latency control loops for precise manipulation tasks.

## Hardware Specifications

### Compute Architecture
- **Main Controller**: NVIDIA Jetson Orin Nano
  - Runs motor control (Rust), orchestration, and data aggregation
  - CAN bus interface for motor communication
  - ZeroMQ hub for inter-process communication

- **Camera Nodes**: 4× Raspberry Pi 4
  - Each running Intel RealSense D455 camera
  - Streams RGB-D + IMU data over PoE network
  - Distributed processing architecture

### Actuation
- **6× ROBSTRIDE Motors** (CAN bus)
  - **Base (3 motors)**:
    - 2× ROBSTRIDE04: Left/right drive wheels (high torque)
    - 1× ROBSTRIDE03: Base swivel joint
  - **Arm Chain (3 motors, 3DoF)**:
    - 1× ROBSTRIDE04: Shoulder joint (high torque for supporting arm weight)
    - 1× ROBSTRIDE03: Elbow joint (medium torque)
    - 1× ROBSTRIDE02: Wrist/gripper (low torque, compact form factor)

- **Control Frequencies**:
  - Arm joints (3 motors): 1 kHz deterministic loop
  - Base (wheels + swivel, 3 motors): 100 Hz

### Sensors
- **4× Intel RealSense D455**
  - RGB: 1280×720 @ 30fps
  - Depth: 1280×720 @ 30fps
  - IMU: 200 Hz (accelerometer + gyroscope)

- **2× SLAMTEC RPLiDAR A1** (future integration)
  - 360° scanning
  - 8m range

### Networking
- Gigabit Ethernet with PoE switch
- Static IP allocation for deterministic routing
- ZeroMQ over TCP for pub/sub messaging

## Software Architecture

### Core Technologies
- **Rust**: Low-level motor control, CAN protocol, deterministic loops
- **Python**: Teleop interfaces, camera nodes, data bridges
- **ZeroMQ**: Inter-process communication (command/telemetry)
- **Rerun**: Real-time visualization and MCAP logging
- **PyO3**: Rust→Python bindings for performance-critical paths

### System Components

```
┌─────────────┐
│   Teleop    │ (Python, joystick/keyboard)
│   Station   │
└──────┬──────┘
       │ ZMQ commands (20 Hz)
       ▼
┌─────────────────────────────────────────┐
│        Jetson Orin Nano                 │
│  ┌──────────────────────────────────┐  │
│  │  Motor Control (Rust)            │  │
│  │  - CAN bus driver                │  │
│  │  - 1kHz arm control loop         │  │
│  │  - Safety watchdog               │  │
│  └───────┬──────────────────────────┘  │
│          │ ZMQ telemetry (50 Hz)       │
│  ┌───────▼──────────────────────────┐  │
│  │  Rerun Bridge (Python)           │  │
│  │  - Aggregates all data streams   │  │
│  │  - MCAP logging                  │  │
│  │  - Real-time visualization       │  │
│  └───────▲──────────────────────────┘  │
└──────────┼──────────────────────────────┘
           │ ZMQ camera streams (30 Hz)
     ┌─────┴─────┬─────────┬─────────┐
     ▼           ▼         ▼         ▼
┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐
│  RPi 4  │ │  RPi 4  │ │  RPi 4  │ │  RPi 4  │
│ Camera  │ │ Camera  │ │ Camera  │ │ Camera  │
│  Node   │ │  Node   │ │  Node   │ │  Node   │
└─────────┘ └─────────┘ └─────────┘ └─────────┘
```

## Repository Structure

```
aizee/
├── rust/               # Rust workspace
│   ├── motor_control/  # CAN driver + control loops
│   ├── comms/          # ZeroMQ abstractions
│   └── bindings/       # PyO3 Python bindings
├── python/
│   ├── aizee/          # Main Python package
│   ├── teleop/         # Teleoperation interface
│   └── nodes/          # RPi camera streaming nodes
├── urdf/               # Robot URDF from OnShape
├── config/             # Hardware parameters (YAML)
├── logs/               # MCAP recordings
└── docs/               # Documentation
```

## Development Status

**Current Phase**: Phase 0 - Project Foundation

See [Implementation Phases](docs/PHASES.md) for detailed roadmap.

## Quick Start

### Prerequisites
- Jetson Orin Nano with JetPack 6.x
- Rust toolchain (stable)
- Python 3.10+
- ZeroMQ libraries

### Installation
```bash
# Clone repository
git clone https://github.com/ltrlab/aizee.git
cd aizee

# Install Rust dependencies (on Jetson)
cd rust/motor_control
cargo build --release

# Install Python dependencies
pip install -r requirements.txt
```

### Hardware Configuration
Edit `config/hardware.yaml` to match your CAN IDs and network topology.

### Running the System
```bash
# On Jetson: Start motor controller
./rust/motor_control/target/release/motor_control

# On dev machine: Start teleop
python python/teleop/teleop.py

# On each RPi: Start camera node (auto-started via systemd)
sudo systemctl start aizee-camera
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Contributing

This is an active research project. See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for guidelines.

## Contact

LTRLAB - [GitHub](https://github.com/ltrlab)
