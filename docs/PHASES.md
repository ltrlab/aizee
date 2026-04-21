# AIZEE Software Stack - Implementation Phases

## Phase 0: Project Foundation (Week 1)

**Goal:** Establish repository structure, tooling, and development environment

### Tasks:
1. **Repository setup** ✓
   - Git repo with directory structure
   - LICENSE file (MIT)
   - README with project vision, hardware specs

2. **Development environment**
   - Jetson Orin Nano setup: Jetpack 6.x, Rust toolchain, Python 3.10+
   - RPi4 images: Lightweight OS (Raspberry Pi OS Lite), Python, `pyrealsense2`
   - Dev machine: Rerun viewer, ZeroMQ, cross-compilation tools for Rust→ARM64

3. **Core dependencies**
   - Rust: `tokio` (async), `zmq`, `socketcan` (CAN interface), `pyo3`
   - Python: `pyzmq`, `rerun-sdk`, `numpy`, `pyrealsense2`
   - Install + verify on all hardware

4. **Hardware parameter config** ✓
   - `config/hardware.yaml` created with motor CAN IDs, network topology

**Deliverable:** Clean repo, all hardware talking to dev machines, config files defining your system

---

## Phase 1: Rust Motor Control Core (Week 2-3)

**Goal:** Low-level CAN interface with ROBSTRIDE motors, deterministic control loop

### Tasks:
1. **CAN communication library**
   - Implement ROBSTRIDE protocol in Rust
   - Functions: `enable_motor()`, `set_position()`, `set_velocity()`, `read_state()`
   - Handle multi-motor CAN bus (6 motors on one line)
   - Error handling: CAN timeouts, motor faults

2. **Motor controller node**
   - Main control loop:
     - 1kHz for arm joints (3 motors)
     - 100Hz for base (wheels + swivel, 3 motors)
   - Use `tokio` for async I/O but keep control loop deterministic
   - State machine: IDLE → ENABLED → RUNNING → ERROR

3. **ZeroMQ command interface**
   - Subscribe to commands on `tcp://*:5555`
   - Message format (JSON):
     ```json
     {
       "type": "drive",
       "linear": 0.5,
       "angular": 0.2
     }
     ```
     ```json
     {
       "type": "arm_joints",
       "positions": [0.1, 0.5, -0.3],
       "velocities": [0.0, 0.0, 0.0]
     }
     ```

4. **Telemetry publishing**
   - Publish motor states on `tcp://*:5556` at 50Hz:
     ```json
     {
       "timestamp": 1234567890.123,
       "motors": {
         "left_wheel": {"position": 1.5, "velocity": 0.5, "current": 2.1},
         ...
       }
     }
     ```

5. **Safety features**
   - Watchdog: If no command received for >100ms, stop motors
   - Soft limits for arm joints (read from config)
   - Emergency stop flag

**Testing:**
- Bench test: Send CAN commands to single motor, verify response
- Full system: Command all 7 motors, log state at 1kHz
- Measure control loop jitter (should be <1ms for arm)

**Deliverable:** `motor_control` Rust binary running on Jetson, controllable via ZeroMQ, publishes telemetry

---

## Phase 2: Python Teleop Interface (Week 3-4)

**Goal:** Human-in-the-loop control with joystick/keyboard

### Tasks:
1. **Input handler**
   - Use `pygame` or `inputs` library for joystick
   - Keyboard fallback (WASD for drive, IJKL for arm, etc.)
   - Map inputs to motor commands with deadbands, scaling

2. **Command publisher**
   - Connect to Jetson motor controller via ZeroMQ
   - Publish at ~20Hz (human reaction time limited)
   - Display connection status, motor health

3. **Simple GUI (optional but recommended)**
   - Use `tkinter` or `PyQt` for basic interface:
     - Connection indicators
     - Motor state display (positions, currents)
     - Emergency stop button
     - Mode switcher (drive only, arm only, both)

4. **Configuration loader**
   - Read `config/teleop.yaml` for:
     - Input mappings (which joystick axis → which command)
     - Scaling factors (max speeds, acceleration limits)
     - Network endpoints

**Testing:**
- Drive rover in open area, verify responsive control
- Move arm through full range of motion
- Test E-stop (motors should halt immediately)

**Deliverable:** `teleop.py` script, runs on dev machine or Jetson, controls robot interactively

---

## Phase 3: RPi Camera Node (Week 4-5)

**Goal:** Stream Realsense data from one RPi to Jetson

### Tasks:
1. **Realsense interface (Python)**
   - Initialize D455, configure streams:
     - Color: 640x480 @ 30fps (compressed JPEG)
     - Depth: 640x480 @ 30fps (16-bit, optionally downsampled)
     - IMU: 200Hz
   - Handle disconnects, reinitialize on error

2. **Data publisher**
   - ZeroMQ PUB socket to Jetson
   - Messages:
     ```json
     {
       "camera_id": "cam_front",
       "timestamp": ...,
       "color": "<base64_jpeg>",
       "depth": "<compressed_array>",
       "imu": {"accel": [...], "gyro": [...]}
     }
     ```
   - Use `msgpack` or similar for efficient serialization
   - Target: <50ms latency from capture to network

3. **Network optimization**
   - Leverage PoE switch bandwidth
   - Test with `iperf3` to verify >100Mbps link
   - Adjust compression quality vs. bandwidth

4. **Systemd service**
   - Auto-start camera node on RPi boot
   - Logging to `/var/log/aizee/`

**Testing:**
- Run camera node on RPi, subscribe from dev machine
- Verify 30fps color + depth streams
- Measure network bandwidth usage
- Test resilience: unplug/replug USB camera

**Deliverable:** `camera_node.py` running on one RPi, streaming to Jetson over network

---

## Phase 4: Rerun Integration (Week 5-6)

**Goal:** Visualize all data streams in real-time + log to MCAP

### Tasks:
1. **Rerun bridge node (Python)**
   - Subscribe to all ZeroMQ topics:
     - Motor telemetry (from Jetson)
     - Camera streams (from RPi)
     - Commands (from teleop)
   - Initialize Rerun recording:
     ```python
     import rerun as rr
     rr.init("aizee", spawn=True)
     rr.save("logs/session_001.mcap")
     ```

2. **Data logging**
   - Motor states → Rerun scalars/time series:
     ```python
     rr.log("motors/left_wheel/position", rr.Scalar(position))
     ```
   - Camera images:
     ```python
     rr.log("cameras/front/color", rr.Image(color_img))
     rr.log("cameras/front/depth", rr.DepthImage(depth_img))
     ```
   - Robot pose (for now, just wheel odometry):
     ```python
     rr.log("world/robot", rr.Transform3D(...))
     ```

3. **URDF visualization**
   - Export URDF from OnShape
   - Load in Rerun:
     ```python
     rr.log("world/robot", rr.Asset3D(path="urdf/aizee.urdf"))
     ```
   - Update joint angles from motor telemetry

4. **Playback capability**
   - Load MCAP file, replay in Rerun viewer
   - Verify all streams synchronized by timestamp

**Testing:**
- Run full stack: motor control + teleop + camera + Rerun bridge
- Drive robot, move arm, verify visualization tracks reality
- Stop recording, replay MCAP, confirm data integrity

**Deliverable:** `rerun_bridge.py`, live visualization + MCAP logging working

---

## Phase 5: System Integration & Testing (Week 6-7)

**Goal:** Robust multi-node system with health monitoring

### Tasks:
1. **Launch system**
   - Create orchestration script (Python or bash):
     ```bash
     # On Jetson
     ./launch.sh
       → starts motor_control (Rust)
       → starts rerun_bridge (Python)
       → starts teleop (if local) or waits for remote

     # On each RPi
     systemctl start aizee-camera
     ```

2. **Health monitoring**
   - Add heartbeat messages to all nodes
   - Rerun bridge monitors:
     - Motor controller alive?
     - Cameras connected?
     - Command input active?
   - Display warnings in Rerun or teleop UI

3. **Error recovery**
   - If camera node crashes → log error, attempt reconnect
   - If motor controller loses CAN → safe stop, alert user
   - If network drops → buffer commands, resync on reconnect

4. **Performance profiling**
   - Measure end-to-end latency: joystick input → motor response
   - Target: <20ms for arm control
   - Camera latency: capture → display in Rerun (<100ms)
   - Log to CSV for analysis

5. **Documentation**
   - Write setup guide: how to deploy to fresh hardware
   - API docs: how to add new sensors/actuators
   - Troubleshooting: common errors + solutions

**Testing:**
- Full mission profile:
   1. Boot all hardware
   2. Drive rover around obstacles
   3. Pick up object with arm (scripted waypoints)
   4. Log entire session
   5. Review in Rerun
- Failure modes:
   - Kill one camera node mid-session
   - Disconnect CAN cable
   - Network congestion (flood with traffic)

**Deliverable:** Reliable, documented system ready for milestone demo

---

## Phase 6: Milestone Demo & Iteration (Week 7-8)

**Goal:** Validate architecture, identify next priorities

### Demo scenario:
- Teleoperate rover to navigate simple course
- Use arm to manipulate object (pick + place)
- All data logged to MCAP
- Post-mission: review in Rerun, analyze performance

### Evaluation criteria:
- ✅ Arm control loop maintains <10ms latency
- ✅ Camera stream at 30fps with <100ms latency
- ✅ No dropped commands during 10-minute session
- ✅ MCAP logs cleanly replay
- ✅ All code in Git, documented

### Next phase planning:
Based on results, prioritize:
- Add remaining 3 cameras + 2nd LiDAR
- Sensor fusion (merge LiDAR scans)
- Autonomous behaviors (e.g., obstacle avoidance)
- PyO3 bindings (if Python bottlenecks found)
- Multi-robot coordination (if you add 2nd rover)

---

## Summary Timeline

| Phase | Duration | Key Deliverable |
|-------|----------|----------------|
| 0: Foundation | 1 week | Repo + dev environment |
| 1: Motor Control | 2 weeks | Rust CAN controller |
| 2: Teleop | 1-2 weeks | Python control interface |
| 3: Camera Node | 1-2 weeks | RPi streaming |
| 4: Rerun Integration | 1-2 weeks | Visualization + logging |
| 5: Integration | 1-2 weeks | Full system test |
| 6: Milestone | 1 week | Demo + retrospective |

**Total: 7-8 weeks to first milestone**

---

## Immediate Next Steps

### Already Complete:
- ✓ Git repository with directory structure
- ✓ MIT License
- ✓ README with project vision
- ✓ `config/hardware.yaml` with motor CAN IDs

### This Week:
1. Set up development environment on Jetson (Rust + dependencies)
2. Install Python dependencies on dev machine
3. Configure CAN interface on Jetson
4. Write simplest possible Rust test: single motor CAN ping
5. Verify motor responds to basic enable/disable commands

**Once single motor responds to CAN, proceed to Phase 1 implementation.**
