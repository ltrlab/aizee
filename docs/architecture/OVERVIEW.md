# Architecture overview

Start here. This page says what runs on Minerva and what talks to what, then walks the source tree, then explains how a
robot is selected. The repository is one Python distribution, `ltrlab-robot-platform` (import name `robot_platform`),
one Rust workspace (`Cargo.toml` at the root; crates under `rust/`, build output in `rust/target`), and firmware,
configuration and tests around them.

Two robots live here. **Minerva** is the current two-arm torso (14 RobStride arm and gripper motors, a two-servo Feetech
neck, three cameras) on a Jetson, operated through **Helm**. **AIZEE** is the earlier single-arm prototype, kept as a
robot option. `lift` is declared in Minerva's definition but inactive: there is no receiver for it and no powered base.

## The running system on Minerva

```text
 operator station (Windows PC)                    Minerva's Jetson (CPUs: 0-2 background, 3 head, 4-5 control)
 +----------------------------+  SSH, forced    +-----------------------------------------------------------+
 | GELLO bridge               |  command, 10.42 | Helm  (minerva-helm, user service, HTTPS 8760, HTTP 8762  |
 |  teleop.gello_bridge       |<===============>|        on loopback)  <-- browsers, pedals, Quest /xr      |
 |  leader USB sampler 200 Hz |                 |   |  spawns, per session                                  |
 |  signed UDP leader link ---+--- UDP ---------+-> |  loop process  helm.edge  (100 Hz, CPUs 4-5)          |
 +----------------------------+                 |   |    |  ZMQ 127.0.0.1:5590     |  ZMQ 127.0.0.1:5591    |
 browser / Quest / pedals --- HTTPS ------------>|   |    v                         v                        |
                                                |   |  guarded_controller      head_controller              |
                                                |   |  (Rust, both arms,       (Rust, Feetech neck;         |
                                                |   |   CAN canL/canR)          publishes pose on :5596)    |
                                                |  camera publishers (ZMQ PUB): left wrist :5563,           |
                                                |    right wrist :5565, head D435 :5564 (control 5573/5574) |
                                                |  minerva-net (system)   minerva-ups (system)              |
                                                +-----------------------------------------------------------+
```

| Process | Where | What it does |
|---|---|---|
| **Helm** (`helm --on-robot`, user service `minerva-helm`) | Jetson | The operator web UI and API: login, one device in control, Check, Limits, Leaders, Teleop, Tune, Replay, settings, the camera relay, the headset page. It starts a loop process per session and never runs a control loop inside itself. Entry: `src/robot_platform/helm/cli.py`; installed by `src/robot_platform/ops/helm_service.py`. Details in [HELM_ON_JETSON.md](HELM_ON_JETSON.md). |
| **Loop process** (`helm.edge`, with `helm/teleop.py`) | Jetson, CPUs 4-5 | One per session: teleop, a Tune sweep or a Replay/policy source. It reads the leader link, runs the control cycle (100 Hz in the production profile), drives the receivers, feeds recording, and sends views back to Helm. |
| **Rust receiver** (`guarded_controller`, from `rust/control_core`) | Jetson, child of the loop | Owns both arms' CAN buses. Checks every command (limits, speed, lease, watchdogs, optional collision envelope) and stops the motors itself. ZMQ at `tcp://127.0.0.1:5590`. See [GUARDED_CONTROLLER.md](GUARDED_CONTROLLER.md). |
| **Head controller** (`head_controller`, `rust/control_core/src/bin/head_controller.rs`) | Jetson | Drives the two Feetech neck servos; ZMQ at `tcp://127.0.0.1:5591`, head pose published at `tcp://127.0.0.1:5596`. Python side: `src/robot_platform/neck/`. |
| **Camera publishers** (`python/nodes/gripper_camera_node.py` for the wrists, `camera_node.py` for the head) | Jetson, CPUs 0-2 | Multipart ZMQ PUB of JPEG frames: left wrist 5563, right wrist 5565, head 5564; wrist control ports 5573/5574. Units: `minerva-left-wrist`, `minerva-right-wrist` and `aizee-scene-cam` (the head D435; the name predates Minerva). |
| **Leader link** (`teleop/leader_link.py`) | operator station to Jetson | The loop asks for the latest leader pair with a random challenge; the station answers from its newest USB sample over UDP, HMAC-signed under a per-session key. A lost reply costs one sample. |
| **GELLO bridge** (`teleop/gello_bridge.py`, `helm/bridge.py`) | operator station | Keeps one pinned SSH session whose key may run only Helm's relay; the PC opens every connection and accepts none. It runs the leader-link process and streams leader samples to the Leaders tab. |
| **`minerva-net`** (`ops/network.py`) | Jetson, system service | The Ethernet port finds its own mode (LAN, or the tether at 10.42.0.1 with DHCP); optional Wi-Fi access point `minerva`. See [MINERVA_NETWORK.md](MINERVA_NETWORK.md). |
| **`minerva-ups`** (`ops/ups.py`) | Jetson, system service | Reads the UPS once a second for Helm and shuts the Jetson down cleanly before the battery runs out. See [UPS_MONITOR](../robots/minerva/UPS_MONITOR.md). |
| **CPU partition** (`ops/cpu_partition.py`, `control/scheduling.py`) | Jetson | Control on CPUs 4-5, the head's receiver on CPU 3, cameras and Helm on 0-2, so camera load cannot stretch a control cycle. See [JETSON_CPU_SPLIT.md](JETSON_CPU_SPLIT.md). |

Transports: Helm to browsers is HTTPS with a cookie login. Helm to its loop and Helm to the robot's own tools use
pinned SSH to `127.0.0.1` or child processes (camera stills and map captures run as Helm's own children). Loop to
receivers and capture to cameras is MessagePack over ZMQ on loopback. Leader link is signed UDP; bridge control is SSH.

### Clocks and rates

- Production profile `config/devices/minerva-teleop-direct-tuning.json`: `loop_hz` 100, `interpolation_ms` 0, collision
  checking off, receiver lease 150 ms, direct command freshness 30 ms. `ops/helm_service.py` installs it.
- Leader USB sampler: 200 Hz on the operator station. The loop asks for the latest pair each cycle.
- Receiver service period 2.5 ms (`live.period_us`); the head receiver ticks every 10 ms.
- Clocks never meet across machines: the leader link bounds a sample's age from its source age plus the round trip, and
  Helm restamps bridge samples on arrival. Capture on the Jetson uses the Jetson's clock for all streams. The hop-by-hop
  budget is in [TELEOP_LATENCY.md](TELEOP_LATENCY.md).

### Data flow into episodes

Task capture (`helm/task_capture.py`, `helm/task_capture_process.py`, profile `config/devices/minerva-task-capture.json`)
runs beside the receiver. It is read-only: its worker subscribes to the three camera sockets, takes one immutable paired
receiver row per control cycle from the loop, and `data/task_capture.TaskEpisodeWriter` writes `task_episode_*.hdf5`
atomically on the Jetson (`~/minerva/evidence`). Replay (`helm/episodes.py`) reviews episodes in place; Keep/Discard
decisions are recorded per session. `data/lerobot_export.py` turns kept episodes into a LeRobot dataset, resampled on a
fixed grid. See [EPISODE_FORMAT](../data/EPISODE_FORMAT.md) and [DATASET_PIPELINE](../data/DATASET_PIPELINE.md). Replay and
policy mode (`teleop/episode_source.py`, `teleop/policy_source.py`) feed the same loop as a source instead of the leaders.

### Episode formats

| Format | Robot | Written by | Status |
|---|---|---|---|
| 4, 5 | AIZEE (5 adds the scene camera) | `data/formats/aizee.py`, AIZEE collector | Legacy; read by the AIZEE training code. |
| 6, 7 | Minerva, 17-slot arrays | `data/formats/minerva.py`, the older `collect_minerva` collector (7 adds segments) | Legacy collector output. |
| 8 | Minerva, 14 axes, no torque | Helm task capture | What the running release recorded. Still read by export, Replay and the episode source. |
| 9 | Minerva, 16 axes with effort, velocity and head pose | Helm task capture (`data/roster.FORMAT_VERSION`) | **Current in the code. Never run on the robot.** Recording refuses without a live head. |

Policy mode drives 14 axes and refuses a policy whose joints differ (`data/lerobot_policy.py`), so a 16-axis policy
cannot drive the robot yet.

## The source tree

### `src/robot_platform/` (the installed package)

| Package | What it owns |
|---|---|
| `contracts.py`, `cli.py`, `__main__.py` | Validated joint, camera and robot definitions; the `robot-platform` command (`helm`, `sim`, `view`, `leaders`, `calibrate`, `deploy`, `check`, `neck`, `describe`, `config`, `run`, `list`). |
| `robots/` | Packaged robot definitions (`definitions/minerva.json`, `aizee.json`): joint order, groups, camera roles, capabilities. |
| `settings/` | Packaged per-robot defaults, layered JSON overlays, endpoint validation. |
| `apps/` | `launcher.py`: the older launch adapters for the legacy collectors and trainers in `python/`. |
| `transport/` | MessagePack wire envelopes (`wire.py`) and network discovery. |
| `control/` | Robot session and freshness, the Python side of the guarded motion contract (approach, collision envelope, guarded config, live receiver start-up, scheduling), calibration, mechanical zero and servo analysis. Motion checks are enforced in Rust (`rust/control_core`); Python prepares and audits them. |
| `teleop/` | Leader input and mapping, the leader link, the GELLO bridge, direct and guarded lifecycles, supervisor, episode, policy and headset sources, VR arm control. |
| `helm/` | The operator application: aiohttp server and static UI (`static/`, including `xr*.js` for the headset), access control, loop edge, teleop session, Check, Limits, Tune, cameras and WebRTC relay, task capture, Replay, head setup and headset teleop (`xr.py`, `head_teleop.py`). |
| `neck/` | Feetech neck: bus access, head pose, clearance, setup and the CLI. |
| `ops/` | Everything installed or deployed on the Jetson from the build PC: release build and deploy, `helm_service`, `helm_robot` (dependencies, meshes, bridge package), `network`, `ups`, `cpu_partition`, Check and smoke tests, SSH pinning. |
| `data/` | Episode writers (`formats/`), task capture, the roster of axes, LeRobot export and policy loading, provenance. |
| `evaluation/` | N-trial closed-loop policy evaluation against simulation or the robot. |
| `simulation/` | Loopback simulator, MuJoCo plant, task scene and workcell, collision queries, limit sweeps, model audit. |
| `models/`, `viewer/` | Mechanical profiles (URDF, MJCF, physics audit) and the shared model viewer. |
| `vision/` | Camera geometry, fiducial board, wrist and head alignment solves. |
| `characterization/` | Actuator manual audit, response fits, sim-to-real comparison. |

### Outside the package

| Path | Contents |
|---|---|
| `python/` | Older collectors, training models, camera/UPS/display nodes, tools and the web shim. See [python/README.md](../../python/README.md). |
| `rust/` | `control_core` (Minerva's guarded receiver and head controller), `comms`, `collision`, and the older `motor_control` and `lidar_control`. See [rust/README.md](../../rust/README.md). |
| `firmware/` | OpenRB leader firmware, the Tufty2040 display, the lift actuator controller. |
| `config/` | Hardware, calibration, device profiles, systemd and udev. See [config/README.md](../../config/README.md). |
| `scripts/` | Build, deploy, development and bench scripts. See [scripts/README.md](../../scripts/README.md). |
| `urdf/` | `aizee/` and `minerva/` mechanical exports and collision models. |
| `settings/examples/` | Operator overlays independent of the source. |
| `tests/` | Offline tests; see [tests/README.md](../../tests/README.md). |
| `docs/` | Indexed by [docs/README.md](../README.md). |

### Who owns motion checks

One owner changes the Python/Rust motion contract together. The Rust receiver (`rust/control_core`) is authoritative at
run time: ownership and expiry, the watchdog start-up gate, limits, speed, and collision enforcement when a profile
enables it. `control/` and `teleop/` compile profiles and plan motion; `helm/` operates sessions but never bypasses the
receiver. Physics libraries (collision, MuJoCo) run behind that boundary. The production direct profile has collision
checking off; see [COLLISION_RUNTIME_IMPLEMENTATION.md](COLLISION_RUNTIME_IMPLEMENTATION.md).

## How a robot is selected

- `robots/definitions/<name>.json` defines joints (with `active` flags), groups, cameras and capabilities.
  `load_robot(name_or_path)` also accepts a custom schema-1 JSON file.
- `settings/defaults/<name>.json` holds endpoints, the `{host}` placeholder and the older launcher's application choices.
  Precedence: packaged defaults, each `--settings FILE`, each `--set key=value`, then `--host`. Examples in `settings/examples/`.
- `apps/launcher.py` maps `robot-platform run collect|train --robot aizee|minerva` onto the legacy scripts
  (`python/scripts/collect_demo.py`, `collect_minerva.py`, `python/training/train.py`, `train_minerva.py`). `--plan`
  prints the resolved command without starting anything. This is the older path; Minerva operation goes through Helm.
  `src/robot_platform/settings/defaults/minerva.json` still carries AIZEE's old default host `192.168.0.27`; override it with `--host` or an overlay.
- `data/formats/` holds the legacy per-robot HDF5 writers (AIZEE 4/5, Minerva 6/7).
- AIZEE is one option here. Its Rust service is `rust/motor_control` (ZMQ commands on 5555, telemetry on 5556, `can1`),
  the older path that Minerva's first bring-up also used (`aizee-minerva-left/right.service`, `canL`/`canR`, 5555/5575).
  Minerva's current path is `control_core` under Helm. See [AIZEE](../robots/aizee/README.md).

Install and development: `python scripts/dev.py setup` creates `.venv-dev`; `python scripts/dev.py check` is the gate
([DEVELOPMENT.md](../DEVELOPMENT.md)). Packaging extras are in `pyproject.toml`.
