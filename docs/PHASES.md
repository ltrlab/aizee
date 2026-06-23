# AIZEE Software Stack - Implementation Phases

Implementation roadmap and status. The early phases are complete and deployed;
status markers reflect the current system. See the README for the live
subsystem snapshot.

Legend: ✅ done · ◐ partial / optional · ◻️ future

---

## Phase 0: Project Foundation ✅

**Goal:** Establish repository structure, tooling, and development environment

### Tasks:
1. **Repository setup** ✅
   - Git repo with directory structure
   - LICENSE file (MIT)
   - README with project vision, hardware specs

2. **Development environment** ✅
   - Jetson Orin Nano setup: JetPack 6.x, Rust toolchain, Python 3.10+
   - Dev / operator machine: Rerun viewer, ZeroMQ, leader drivers, training stack

3. **Core dependencies** ✅
   - Rust: `tokio` (async), `zmq`, `socketcan` / `gs_usb` (CAN interface), `pyo3` (reserved)
   - Python: `pyzmq`, `rerun-sdk`, `numpy`, `torch`, `h5py`, `pyrealsense2`

4. **Hardware parameter config** ✅
   - `config/hardware_jetson_rover.yaml` (rover) + camera configs
     (`hardware_jetson_gripper_cam.yaml`, `hardware_jetson_scene_cam.yaml`)

**Deliverable:** Clean repo, hardware talking to dev machine, config files defining the system.

---

## Phase 1: Rust Motor Control Core ✅

**Goal:** Low-level CAN interface with ROBSTRIDE motors, deterministic control loop

### Tasks:
1. **CAN communication library** ✅
   - ROBSTRIDE protocol in Rust: `enable_motor()`, `set_position()`, `set_velocity()`, `read_state()`
   - Multi-motor single CAN bus (`can1`, gs_usb adapter)
   - Error handling: CAN timeouts, motor faults, fault recovery

2. **Motor controller node** ✅
   - Control loops: **400 Hz** arm (CAN-bus limited), **100 Hz** base (wheels + swivel)
   - `tokio` for async I/O with a deterministic control loop
   - State machine: IDLE → ENABLED → RUNNING → ERROR

3. **ZeroMQ command interface** ✅
   - Subscribe to commands on `tcp://*:5555` (JSON/msgpack)
   - Message types: `drive`, `arm_joints` (7-DoF, swivel-first), `bundle`, `enable`/`disable`, `emergency_stop`

4. **Telemetry publishing** ✅
   - Motor states on `tcp://*:5556` at **50 Hz** (position/velocity/torque/temperature)

5. **Safety features** ✅
   - Watchdog: holds position if no command for ~500 ms
   - Soft limits for arm joints (from config)
   - Emergency-stop path (wireless ESP-NOW e-stop bridged to ZMQ)

**Deliverable:** `motor_control` Rust binary on the Jetson, controllable via ZeroMQ, publishing telemetry. ✅

---

## Phase 2: Python Teleop Interface ✅

**Goal:** Human-in-the-loop control

### Tasks:
1. **Input handlers** ✅
   - Leader arms: **SO-101** (Feetech STS3215), **OpenRB-150 + Dynamixel XL330**, **Quest VR** (WebXR)
   - Gamepad + keyboard (WASD drive); M5Stack Joystick2 on the OpenRB leader
   - Per-leader calibration (`config/so101_calibration.json`, `openrb_calibration.json`, `robstride_calibration.json`)

2. **Command publisher** ✅
   - Connects to the Jetson over ZeroMQ; rover IP auto-resolves across networks
   - Dedicated cmd-sender thread re-emits the latest bundle at ~100 Hz

3. **GUI control panel** ✅
   - PySide6 panel (`--gui`) embedding the Rerun web viewer
   - Live motor state, camera preview, V4L2 camera controls, runtime leader hot-plug

4. **Configuration loader** ✅
   - `config/teleop.yaml` for input mappings, scaling, gains, and network endpoints

**Deliverable:** `teleop.py` + leader drivers, controlling the robot interactively. ✅

---

## Phase 3: Camera Nodes ✅

**Goal:** Stream camera data to the policy/recorder pipeline

> **Note:** the original plan streamed RealSense data from separate Raspberry
> Pi 4 nodes over PoE. That split has been retired — **both cameras now run on
> the Jetson**.

### Tasks:
1. **Gripper camera** ✅
   - ELP-USBFHD01M-L21 USB UVC, color **1024×768 @ 30 fps** (MJPG)
   - Primary close-up training observation; PUB on `tcp://*:5563`, V4L2 control REP on `:5573`

2. **Scene camera (optional)** ◐
   - Intel RealSense D435/D435i/D455, color + depth **640×480 @ 15 fps**
   - Externally mounted wider workspace view with depth; PUB on `tcp://*:5564`
   - Auto-detected at runtime: present ⇒ recorded (static mode), absent ⇒ rover mode

3. **Resilience & services** ✅
   - Auto-start via udev + systemd (`aizee-gripper-cam`, `aizee-scene-cam`)
   - Reinit on disconnect

**Deliverable:** Camera nodes on the Jetson, streaming over ZeroMQ. ✅

---

## Phase 4: Rerun Integration ✅

**Goal:** Visualize all data streams in real time + log

### Tasks:
1. **Live visualization** ✅
   - Rerun used across teleop, collection, evaluation, and inference
   - Motor telemetry, camera images, joint/leader scalars

2. **URDF visualization** ✅
   - URDF exported from OnShape, rendered with joint angles from telemetry (GUI / preview)

3. **Logging & playback** ✅
   - `.rrd` export from `view_episode.py` / `evaluate_policy.py`; offline replay in the viewer

**Deliverable:** Live visualization + logging working. ✅

---

## Phase 5: System Integration & Monitoring ✅

**Goal:** Robust multi-node system with health monitoring

### Tasks:
1. **Launch / services** ✅
   - On-robot subsystems run as systemd services and start on boot
   - Deploy scripts under `scripts/` (`deploy_jetson_rover.sh`, `deploy_gripper_camera.sh`, `deploy_scene_cam.sh`, `deploy_heartbeat.sh`)

2. **Health monitoring** ✅
   - **Heartbeat dashboard** at `http://<jetson>:8088` — service status, recent logs, host metrics, live telemetry
   - Tufty2040 on-robot status display

3. **Networking** ✅
   - Three reach paths probed in priority order: `192.168.0.27` (LAN/WiFi) → `10.42.0.1` (USB-C ethernet) → `192.168.50.1` (robot's own **WiFi AP**, autoconnects on boot)

4. **Error recovery** ✅
   - CAN fault recovery in the Rust core; camera reinit on disconnect; safe-stop on stale commands

**Deliverable:** Reliable, monitored system. ✅

---

## Phase 6: Imitation Learning Pipeline ✅

**Goal:** Collect demonstrations and train/deploy policies

### Tasks:
1. **Demonstration collection** ✅
   - `collect_demo.py --gui` — leader, rover IP, and scene cam auto-detected
   - HDF5 episodes: **v4** (gripper cam + 7-DoF joint states/commands/torques + timestamps), **v5** (v4 + scene cam) when the scene cam is present
   - Camera/telemetry timestamp sync; 20 Hz recording

2. **ACT training** ✅
   - `train.py` — Action Chunking with Transformers (CVAE + DETR decoder, ResNet18 backbone)
   - Gripper-camera observation + 7-DoF state; relative/absolute action modes; train/val split; checkpoints with embedded dataset stats

3. **ACT-JEPA training** ◐ *(experimental)*
   - `train_jepa.py` — adds a self-supervised world-model objective (future-image prediction + SIGReg)
   - Inference-compatible with the ACT checkpoint loader

4. **Offline evaluation** ✅
   - `evaluate_policy.py` — open-loop replay through the model, per-joint L1, optional temporal ensemble

5. **Live inference** ✅
   - `act_policy_node.py` — 20 Hz, single gripper camera, safety clamps + closest-start ready pose
   - `episode_replay_live.py` — replay recorded commands on hardware (no model)

> **Multi-camera training/inference is a follow-up.** The scene cam is recorded
> into v5 episodes, but `train.py`, `train_jepa.py`, `evaluate_policy.py`, and
> `act_policy_node.py` currently consume the gripper camera only.

**Deliverable:** End-to-end collect → train → evaluate → deploy pipeline. ✅
See [LEARNING_PIPELINE.md](LEARNING_PIPELINE.md).

---

## Future Work

- ◻️ **Multi-camera policies** — consume the recorded scene camera (RGB-D) in training and inference.
- ◻️ **LiDAR integration** — 2× SLAMTEC RPLiDAR A1M8 driver exists (`rust/lidar_control`) but is not enabled in all configurations; sensor fusion / mapping is future.
- ◻️ **Autonomous behaviors** — navigation / obstacle avoidance on the rover.
- ◻️ **PyO3 bindings** — promote hot Python paths into the Rust core if profiling warrants.
- ◻️ **Task-conditioned / multi-task policies** — beyond single-task ACT checkpoints.

---

## Status Summary

| Phase | Status |
|-------|--------|
| 0: Foundation | ✅ |
| 1: Rust motor control (400 Hz arm / 100 Hz base, fault recovery) | ✅ |
| 2: Teleop (SO-101 / OpenRB-150 / Quest VR, GUI) | ✅ |
| 3: Cameras on the Jetson (gripper + optional scene) | ✅ |
| 4: Rerun visualization + logging | ✅ |
| 5: Integration, heartbeat dashboard, WiFi AP | ✅ |
| 6: Imitation learning (ACT done, ACT-JEPA experimental) | ✅ / ◐ |
| Future: multi-camera, LiDAR fusion, autonomy | ◻️ |
