# AIZEE — open-source mobile manipulation platform

![AIZEE preview](image.png)

AIZEE is a wheeled mobile manipulator with a 6-DoF arm, built for real-time
teleoperation, demonstration collection, and imitation-learning policies. It
pairs a deterministic Rust motor-control core with a Python teleop/learning
stack, all glued together over ZeroMQ.

- **CAD (OnShape):** https://cad.onshape.com/documents/191b8a861f2900918f30776f/w/8fcf08900b23701d0eff1c6a/e/fcb77dc7adbafb5b753006f3

> The platform runs in two physical configurations that share the same code and
> CAN bus: **rover mode** (drive wheels + arm) and **static mode** (arm only on a
> fixed stand, typically with the external scene camera). Optional subsystems
> auto-detect at runtime, so the same launch command works in either mode.

---

## Hardware

### Compute
- **NVIDIA Jetson Orin Nano** (JetPack 6.x) — the on-robot brain. Runs the Rust
  motor-control binary, the camera/UPS/display nodes, and the status dashboard.
  USB-CAN (gs_usb) adapter on `can1`; ZeroMQ for all inter-process messaging.
- **Operator / dev machine** — runs teleop, demonstration collection, and
  training. Drives the arm through a *leader* device (see below) and connects to
  the Jetson over one of three network paths.

### Actuation — 9× ROBSTRIDE motors (single CAN bus, `can1`)

| Joint | CAN ID | Motor | Role |
|------------|:------:|:----------:|------|
| left_wheel | 0x02 | ROBSTRIDE04 | drive wheel |
| right_wheel| 0x04 | ROBSTRIDE04 | drive wheel |
| swivel | 0x03 | ROBSTRIDE03 | base rotation (arm joint 0) |
| gantry_base| 0x05 | ROBSTRIDE04 | shoulder |
| gantry_mid | 0x06 | ROBSTRIDE03 | upper arm |
| gantry_end | 0x07 | ROBSTRIDE02 | elbow |
| wrist_pitch| 0x08 | ROBSTRIDE02 | wrist pitch |
| wrist_roll | 0x09 | ROBSTRIDE00 | wrist roll |
| gripper | 0x0A | ROBSTRIDE00 | gripper |

The arm is **7-DoF in software** (`swivel` + 6-DoF gantry, swivel-first
ordering). In static mode the two wheels are simply absent from the bus.

**Control loops** (`config/hardware_jetson_rover.yaml`):
- Arm: **400 Hz** (CAN-bus limited — ~280 µs per motor round-trip)
- Base (wheels + swivel): **100 Hz**
- Telemetry publish: **50 Hz**
- Watchdog: holds position if no command for **500 ms**

### Cameras
- **Gripper camera** — ELP-USBFHD01M-L21 USB UVC, color **1024×768 @ 30 fps**
  (MJPG), mounted on the gripper. Primary close-up training observation.
- **Scene camera** *(optional)* — Intel RealSense D435/D435i/D455, color +
  depth **640×480 @ 15 fps**, externally mounted for a wider workspace view with
  depth. Auto-detected at runtime: present ⇒ static mode (recorded), absent ⇒
  rover mode (hidden, not required).

### Other sensors & peripherals
- **2× SLAMTEC RPLiDAR A1M8** — 360° scan, driver in `rust/lidar_control`
  (optional; not enabled in all configurations).
- **UPS power monitor** — Waveshare INA219 over I²C, publishes battery
  voltage/current/% telemetry.
- **Tufty2040 status display** — RP2040 over USB CDC, on-robot status readout.
- **Wireless e-stop** — ESP32 ESP-NOW receiver bridged to ZMQ; broadcasts stop
  to all motors.

### Networking — three ways to reach the robot
| Path | Jetson address | Use |
|------|----------------|-----|
| LAN / WiFi client | `192.168.0.27` | production network (configured default) |
| USB-C ethernet | `10.42.0.1` | direct tether (NetworkManager shared link) |
| WiFi AP `aizee` | `192.168.50.1` | the robot broadcasts its own network (AP mode) |

The operator tools probe these **in priority order** and use the first that
answers, so the same command works whether you're on the LAN, tethered over
USB-C, or joined to the robot's own WiFi.

---

## Software

### Stack
- **Rust** — motor control, CAN protocol, deterministic control loops, CAN
  fault recovery.
- **Python 3.10+** — teleop, leader drivers, camera/UPS/display nodes,
  demonstration collection, policy training & inference.
- **ZeroMQ** — command/telemetry/camera pub-sub between host and Jetson.
- **Rerun** — real-time visualization and MCAP logging.

### Architecture
```
        Operator / dev machine
  ┌───────────────────────────────────┐
  │ leader arm (SO-101 / OpenRB-150 /  │
  │ Quest VR) · gamepad · M5 joystick  │
  │ collect_demo.py / teleop.py        │
  └───────────────┬───────────────────┘
                  │ ZMQ commands (~100 Hz)
                  │ path: 192.168.0.27 → 10.42.0.1 → 192.168.50.1
                  ▼
  ┌───────────────────────────────────────────────┐
  │              Jetson Orin Nano                   │
  │  motor_control (Rust)      400 Hz arm /100 Hz  │
  │    CAN bus · watchdog · fault recovery         │
  │      ├─ telemetry  PUB :5556  (50 Hz)          │
  │  camera_node (gripper UVC) PUB :5563           │
  │  camera_node (scene RGB-D) PUB :5564           │
  │  ups_node                  PUB :5562           │
  │  lidar_control             PUB :5561           │
  │  heartbeat dashboard       HTTP :8088          │
  └───────────────────────────────────────────────┘
```

### ZeroMQ / service ports
| Port | Type | Service | Payload |
|:----:|------|---------|---------|
| 5555 | SUB | motor_control | incoming motor commands |
| 5556 | PUB | motor_control | motor telemetry (state/pos/vel/torque/temp) |
| 5561 | PUB | lidar_control | LiDAR scans |
| 5562 | PUB | ups_node | battery telemetry |
| 5563 | PUB | gripper camera | color JPEG |
| 5573 | REP | gripper camera | runtime V4L2 control (GUI sliders) |
| 5564 | PUB | scene camera | color JPEG + Z16 depth |
| 8088 | HTTP | heartbeat | status/telemetry/log dashboard |

On-robot subsystems run as **systemd services** (`aizee-motor-control-rover`,
`aizee-gripper-cam`, `aizee-scene-cam`, `aizee-ups-monitor`, `aizee-heartbeat`,
`aizee-display`, …) and start on boot.

---

## Repository structure
```
aizee/
├── rust/
│   ├── motor_control/   # CAN driver + control loops (400 Hz arm, 100 Hz base) + recovery
│   ├── comms/           # ZeroMQ abstractions and message schemas
│   ├── lidar_control/   # RPLiDAR A1M8 dual-sensor driver
│   └── bindings/        # PyO3 Python bindings (reserved)
├── python/
│   ├── teleop/          # Leader drivers: SO-101, OpenRB-150, Quest VR + shared discovery
│   ├── nodes/           # Camera, UPS, ACT-policy inference nodes
│   ├── scripts/         # collect_demo (GUI), teleop, leader monitor, episode replay/view
│   ├── training/        # ACT + ACT-JEPA policy training and dataset utilities
│   ├── tools/           # heartbeat_server.py (status dashboard)
│   └── web/             # WebXR client for Quest teleop
├── config/              # Hardware YAML, calibration, systemd/udev units
├── scripts/             # Deploy, setup, and diagnostic shell scripts
├── firmware/            # Tufty2040 display, wireless e-stop, OpenRB-150 leader
├── urdf/                # Robot URDF (from OnShape)
├── episodes/            # Recorded demonstrations (HDF5)
├── docs/                # Quick-start, subsystem, and learning-pipeline docs
└── tests/               # Test suite
```

---

## Getting started

### Prerequisites
- NVIDIA Jetson Orin Nano with JetPack 6.x
- Rust toolchain (stable) and Python 3.10+
- A leader device for teleop (SO-101, OpenRB-150, or a Meta Quest headset) —
  optional; the stack also runs leaderless and hot-plugs a leader later.

### Install & deploy
```bash
git clone https://github.com/ltrlab/aizee.git
cd aizee
pip install -r requirements.txt

# Deploy to the Jetson (motor control + services)
./scripts/deploy_jetson_rover.sh

# Deploy optional subsystems
./scripts/deploy_gripper_camera.sh
./scripts/deploy_scene_cam.sh      # static mode only
./scripts/deploy_heartbeat.sh
```

### Check the robot is up
Open the **heartbeat dashboard** — service status, recent logs, host metrics,
and live robot telemetry (motors, batteries, cameras) on one page:
```
http://10.42.0.1:8088     # over USB-C ethernet
http://192.168.50.1:8088  # over the robot's WiFi AP
```

### Teleoperate
```bash
# Full teleop (gamepad / keyboard)
python python/teleop/teleop.py

# SO-101 leader-arm teleop
python python/scripts/so101_teleop.py --port /dev/ttyACM0

# Live leader position/limits monitor (auto-detects SO-101 or OpenRB-150)
python python/scripts/leader_monitor.py
```

### Collect demonstrations & train
```bash
# Record demos — leader auto-detected on USB; rover-IP auto-resolved;
# scene cam auto-detected (omit it for rover mode)
python python/scripts/collect_demo.py --gui

# Force a specific leader kind
python python/scripts/collect_demo.py --gui --leader openrb

# Train an ACT policy on collected episodes
python python/training/train.py --data-dir episodes/ --output-dir checkpoints/

# (Experimental) ACT-JEPA — adds a self-supervised world-model objective
python python/training/train_jepa.py --data-dir episodes/ --output-dir checkpoints/

# Run a trained policy on the arm
python python/nodes/act_policy_node.py --checkpoint checkpoints/act_epoch_0100.pt

# Visualize / replay a recorded episode
python python/scripts/view_episode.py episodes/episode_0001.hdf5
python python/scripts/episode_replay_live.py episodes/episode_0001.hdf5
```

### Leader devices
All three present the same 7-DoF interface, so teleop/collection code is
leader-agnostic once connected:
- **SO-101** — Feetech STS3215 servo arm over USB serial (original leader).
- **OpenRB-150 + Dynamixel XL330** — drop-in alternative; also carries an
  M5Stack Joystick2 for drive + record-toggle.
- **Quest VR** — Meta Quest Pro driving the arm via a WebXR client (`python/web`).

---

## Data format
Demonstrations are written as HDF5:
- **v4** — gripper camera + joint states/commands/torques + timestamps.
- **v5** — v4 **plus** `observations/images/scene` and `timestamps/camera_scene`
  when the scene camera is present. v4 episodes remain fully loadable.

---

## Status
All major subsystems are deployed and operational:
- ✅ Rust motor control (400 Hz arm / 100 Hz base) with CAN fault recovery
- ✅ Drive wheels + 6-DoF arm on one CAN bus (rover & static modes)
- ✅ Teleop via SO-101, OpenRB-150, and Quest VR leaders (7-DoF)
- ✅ On-Jetson cameras: ELP UVC gripper + RealSense scene (udev auto-start)
- ✅ Demonstration collection with camera sync (HDF5 v4/v5) + GUI
- ✅ ACT policy training & inference; ACT-JEPA training (experimental)
- ✅ Episode replay with safety clamping
- ✅ UPS battery monitoring, Tufty status display, wireless e-stop
- ✅ WiFi AP mode + status/telemetry dashboard
- ◻️ 2× RPLiDAR A1M8 integration (driver present; not always enabled)

See [docs/PHASES.md](docs/PHASES.md) for the full roadmap.

---

## Documentation
- [docs/quickstart/](docs/quickstart/) — connect, run, and post-reboot guides
- [docs/subsystems/](docs/subsystems/) — motors, cameras, leaders, UPS, display, LiDAR
- [docs/LEARNING_PIPELINE.md](docs/LEARNING_PIPELINE.md) — calibration → collection → training → deployment
- [docs/PHASES.md](docs/PHASES.md) — implementation roadmap

## License
MIT — see [LICENSE](LICENSE).

## Contributing
Active research project — contributions welcome.

## Contact
LTRLABS — [GitHub](https://github.com/ltrlab)
