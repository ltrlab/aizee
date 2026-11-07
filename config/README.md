# Configuration ownership

What each file in `config/` is, who reads it, and which robot it belongs to. The files are inputs to active applications
and deployed services; they are not interchangeable schemas, so do not delete one because the shared settings also
describe the robot. Robot definitions and defaults live in the package, not here:
`src/robot_platform/robots/definitions/*.json` (joint order, groups, cameras, capabilities),
`src/robot_platform/settings/defaults/*.json` (endpoints, freshness, application choices) and `settings/examples/*.json`
(operator overlays). See [the architecture overview](../docs/architecture/OVERVIEW.md).

`src/robot_platform/settings/defaults/minerva.json` still carries AIZEE's old default host `192.168.0.27`. Only the older launcher
(`robot-platform run ...`) reads it; override it with `--host` or an overlay. Helm and the Jetson-hosted loop take their
addresses from the device profile and the tether (`10.42.0.1`).

## Minerva

### Device profiles (`devices/`)

Which are used by Helm on the Jetson, which by tools, and which are evidence rather than settings.

| File | Owner / consumer |
|---|---|
| `minerva-teleop-direct-tuning.json` | **Production teleop profile**: 100 Hz, collision off, `interpolation_ms` 0, operator-tunable gains. `ops/helm_service.py` runs Helm with it. |
| `minerva-teleop-direct.json`, `minerva-teleop-direct-trial.json` | The collision-free direct profile it derives from, and the attended left-J1 baseline trial (0.05 rad/s, small travel). |
| `minerva-teleop.json` | Guarded profile with the collision gate; virtual receiver, rehearsal values only. Used by rehearsal and the viewer, not the production loop. |
| `minerva-joint-calibration.json` | Per-unit calibration profile: device, reference, model, connection, reference positions, steps. Helm's `--profile`. Carries the unit's SSH host key. |
| `minerva-reference-20260924.json`, `minerva-reference-20261009.json` | The mechanical reference (zero pose, motor identities, post-zero readings). The 2026-10-09 revision is current: left J4 and J5 were remounted and re-zeroed there; the saved limits and servo calibration are bound to it. Evidence, not an installed motor-to-CAD map. |
| `minerva-production-01.json`, `minerva-production-01-joint-limits.json`, `minerva-production-01-servo.json` | One unit's identity (host, adapters, motor models, legacy-service conflicts), saved joint limits and servo calibration. Per unit; see [a new unit's profile](../docs/getting-started/NEW_UNIT_PROFILE.md). |
| `minerva-head.json` | Head profile: two Feetech STS3250 servos on the head receiver (`head_controller`, `tcp://127.0.0.1:5591`) and the head camera. |
| `minerva-head-alignment.json`, `minerva-wrist-cameras.json`, `minerva-table-board.json` | Camera alignment inputs: head and wrist stills against the table's fiducial board, and the board's tags. |
| `minerva-task-capture.json` | Task capture: 15 Hz, camera and receiver age limits, camera sockets (5563, 5565, 5564) and the head pose socket (5596), all on Jetson loopback. |
| `minerva-ups.json` | The UPS reader: INA219 on I2C bus 7 at 0x41, pack cells, shutdown thresholds. Read by `minerva-ups`. |
| `minerva-leaders-01.json`, `gello-dual-viewer.json`, `gello-dual-leader-ranges.json` | The two GELLO/OpenRB leaders: pinned USB serials and age limits, viewer geometry and force-feedback trigger channel, hand-swept channel ranges. No motor calibration or enable permission. |
| `minerva-gello-shadow.json` | Simulation-only channel-to-CAD routes for the virtual teleop test; not a physical profile. |
| `simulation/` | Actuator values, randomization and workcell for the simulated Minerva (`helm --sim`). Fitted on real teleop by `characterization.sim2real`, bound to each motor's UID. Releases leave this folder out. |

### Hardware, camera and service files

| File | Owner / consumer |
|---|---|
| `hardware_minerva_left.yaml`, `hardware_minerva_right.yaml` | Per-arm motor rosters, buses, limits and endpoints. Read by the older `motor_control` services and by the direct tuning; releases carry them. |
| `hardware_minerva_left_wrist.yaml`, `hardware_minerva_right_wrist.yaml` | Wrist UVC camera publishers (`minerva-left-wrist`, `minerva-right-wrist`). |
| `hardware_jetson_scene_cam.yaml` | The RealSense publisher. Minerva's head D435 runs it as `aizee-scene-cam`. |
| `minerva.yaml` | Minerva inference controls, camera sizing and direct-launch camera endpoints (`minerva_policy_node.py`). |
| `minerva_calibration.json`, `openrb_left.json`, `openrb_right.json` | Older collector inputs: follower limits (collector, standalone teleop, hardware audit) and per-arm leader calibration. |
| `minerva_gravity.json` | Empirical gravity model for the older collector; not a validated simulator profile. |
| `systemd/minerva-*-wrist.service`, `udev/80-minerva-can.rules` | Wrist camera units and CAN adapter naming. `aizee-minerva-left/right.service` are the older motor-service units. |

The units `minerva-helm`, `minerva-net`, `minerva-ups` and `minerva-control-cpus` are generated by `ops/helm_service.py`,
`ops/network.py`, `ops/ups.py` and `ops/cpu_partition.py`; they are not in `systemd/`.

## AIZEE

| File | Owner / consumer |
|---|---|
| `hardware_jetson_rover.yaml` | Rust controller, LiDAR, UPS and display service configuration. Wheels removed; the swivel and gantry remain. |
| `hardware_jetson_gripper_cam.yaml`, `hardware_jetson_scene_cam.yaml` | The gripper UVC and RealSense publishers. |
| `teleop.yaml`, `quest_teleop.yaml` | Standalone teleop, tracking gains and Quest/IK options. |
| `joint_limits.yaml` | IK and workspace bounds; read by `python/teleop/quest_leader.py`, `python/ik/workspace.py` and the web preview. Raw sweep reports are separate. |
| `joint_align.json` | Collector coordinate alignment (`collect_demo_app/alignment.py`, hot-reloaded) and the WebXR preview. |
| `openrb_calibration.json`, `so101_calibration.json` | Default leader calibration for `python/teleop/openrb_leader.py` and `so101_leader.py`. |
| `robstride_calibration.json` | Follower limits for `python/common/arm_constants.py` and standalone teleop/replay. |
| `systemd/aizee-*.service`, `udev/99-*`, `udev/99-rplidar.rules` | Service units and device identities for the rover Jetson. |

Manual Rust launches of `motor_control` need `AIZEE_CONFIG` set to the chosen hardware YAML. These files do not establish
approved limits for new hardware.

## Calibration and generated output

Stopped motor-to-CAD direction work uses Helm's own flows, or `scripts/calibrate_minerva.ps1` with
`devices/minerva-joint-calibration.json`; see [joint calibration](../docs/operator/JOINT_CALIBRATION.md). Candidate
mappings land in the git-ignored `calibration_runs/`, as do raw captures such as `gravity_calibrate.py --record-poses`
and `python -m python.ik.collision_sweep` reports (review them, then promote accepted bounds into `joint_limits.yaml`
by hand). Large data, trained weights and plots belong in ignored output directories, not in `config/`.

The e-stop bridge (an ESP-NOW stop) was retired in 2026-09; its service, receiver rule and deployment helper are gone.
See [safety](../docs/operator/SAFETY.md) for what stops what.
