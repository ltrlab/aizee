# Configuration ownership

`devices/gello-dual-viewer.json` configures the generic read-only leader viewer:
pinned USB identities, illustrative link geometry and display axes. Saved display
offsets/signs live separately in `calibration_runs/gello-dual/display.json`.
`devices/minerva-gello-shadow.json` contains simulation-only channel-to-CAD routes
for the virtual teleop test; it is not a physical motor profile.
See [dual GELLO setup](../docs/GELLO_TELEOP.md).

`devices/minerva-leaders-01.json` pins the dual GELLO/OpenRB USB serials and input
age/skew limits. It contains no motor-to-CAD calibration or enable permission.
The guarded physical zero-torque service now requires ordered
`reference_positions` (name, raw_position_rad, tolerance_rad) and explicit
operator pose confirmation; old profiles without this gate are rejected. See
[mechanical reference](../docs/MECHANICAL_REFERENCE.md).

`devices/minerva-reference-20260924.json` records the manually aligned arm pose,
fully closed jaw reference, pinned motor identities and post-zero readings.
All 14 motors passed one power-cycle persistence check; runtime approval remains
false. It is evidence, not
an automatically installed motor-to-CAD map. Its reference change invalidates
the old gripper fit's absolute encoder offset; preserve that fit as historical.

Select the robot and operator endpoints through the shared package. The files
here remain inputs to active applications and deployed services; they are not
all interchangeable schemas.

| Source | Owner / consumer |
|---|---|
| `src/robot_platform/robots/definitions/*.json` | Canonical joint order, active groups, cameras and capabilities |
| `src/robot_platform/settings/defaults/*.json` | Shared-session endpoints, freshness/lease settings, application paths |
| `settings/examples/*.json` | Operator/machine overlays; bench template is a separate characterization schema |
| `hardware_jetson_rover.yaml` | Aizee Rust controller, LiDAR, UPS and display service configuration |
| `hardware_minerva_left.yaml`, `hardware_minerva_right.yaml` | Per-arm Rust motor rosters, buses, limits and endpoints |
| `hardware_jetson_gripper_cam.yaml`, `hardware_jetson_scene_cam.yaml` | Aizee UVC and shared RealSense publishers |
| `hardware_minerva_left_wrist.yaml`, `hardware_minerva_right_wrist.yaml` | Minerva wrist UVC publishers |
| `minerva.yaml` | Minerva inference controls, camera sizing and direct-launch camera endpoints |
| `teleop.yaml`, `quest_teleop.yaml` | Active standalone Aizee teleop, tracking gains and Quest/IK options |
| `joint_limits.yaml` | Configured Aizee IK/workspace bounds; raw sweep reports are separate |
| `*_calibration.json`, `openrb_left.json`, `openrb_right.json` | Active leader/follower calibration inputs; reader audit below |
| `joint_align.json` | Aizee collector coordinate alignment and WebXR preview editing |
| `minerva_gravity.json` | Existing empirical gravity model; not a validated simulator profile |
| `systemd/`, `udev/` | Deployment templates and device identities |
| `devices/minerva-production-01.json` | Verified host/adapter identity and model labels, legacy-service conflicts and unresolved motion gates; generates observation/diagnostic profiles |

The ESP-NOW/serial stop bridge was retired on 2026-09-24. Its service, receiver
rule and deployment helper are removed. See [guarded commissioning](../docs/POWERED_COMMISSIONING.md)
for physical diagnostic results and the remaining powered-motion gates.

## Launch selection

```sh
robot-platform run collect --robot minerva --settings settings/examples/usb-tether.json --plan
robot-platform run collect --robot aizee --settings settings/examples/usb-tether.json --plan
```

Manual Rust launches require `AIZEE_CONFIG` set to the chosen hardware YAML.
The obsolete generic three-joint profile and unreferenced rover-only teleop
profile have been removed. Existing service units already select explicit
hardware profiles.

Shared settings override compatibility endpoint values at application launch.
Model architecture comes from training arguments/checkpoints. GUI preferences
and measured calibration are not yet fully unified into the shared schema. Do not remove these files merely because the
shared settings also describe the robot.

## Calibration status

For stopped motor-to-CAD direction and alignment work, use
[`scripts/calibrate_minerva.ps1`](../scripts/calibrate_minerva.ps1) and
[`devices/minerva-joint-calibration.json`](devices/minerva-joint-calibration.json).
The shared [live calibration workflow](../docs/JOINT_CALIBRATION.md) stores
candidate mappings in ignored `calibration_runs/`, independently of legacy limits.

The following files have runtime readers, beyond tools that merely write them:

| File | Runtime reader |
|---|---|
| `openrb_calibration.json` | Default calibration in `python/teleop/openrb_leader.py` for the Aizee leader |
| `openrb_left.json`, `openrb_right.json` | Per-arm leader selection in `collect_minerva.py` and `minerva_teleop.py` |
| `so101_calibration.json` | Default calibration in `python/teleop/so101_leader.py` |
| `robstride_calibration.json` | Aizee follower limits in `python/common/arm_constants.py` and standalone teleop/replay |
| `minerva_calibration.json` | Follower limits in `collect_minerva.py`, standalone teleop and the hardware audit |
| `minerva_gravity.json` | Gravity feedforward model loaded by `collect_minerva.py` |
| `joint_align.json` | `collect_demo_app/alignment.py` loads and hot-reloads offsets/signs; `python/web/server.py` reads and edits them |
| `joint_limits.yaml` | `python/teleop/quest_leader.py`, `python/ik/workspace.py` and the web preview load configured bounds |

These inputs do not establish approved limits for new hardware. The simulation
audit identifies unresolved actuator scales and reversed directional bounds;
see [measurement requirements](../docs/architecture/SIMULATION_IMPLEMENTATION.md).
Head/lift slots are inactive in Minerva's shared definition.

Use `config/local_hardware.yaml` for a local hardware override (Git-ignored).

## Generated output

`gravity_calibrate.py --record-poses` writes raw captures to
`calibration_runs/aizee/calibration_poses.json`; pass that path to `--poses`
when using a capture. The checked-in old capture was removed.

`python -m python.ik.collision_sweep` writes its full report to
`calibration_runs/aizee/joint_limit_sweep.yaml`. Review the report and promote
accepted bounds into `joint_limits.yaml` explicitly. Sweep counts, collision
statistics, timings and workstation paths are not active configuration.
Existing effective/URDF bounds and manual-edit provenance remain unchanged.

The obsolete `ready_pose.json` and its save controls were removed: no runtime
loaded the file. Minerva's equivalent save controls also had no reader.
Policy ready/start poses still come from recorded episodes and checkpoint
statistics; those are active inputs.

`calibration_runs/` is ignored. Large data, trained weights and generated plots
also belong in ignored output directories, not in `config/`.
