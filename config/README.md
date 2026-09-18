# Configuration ownership

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
| `joint_limits.yaml` | Existing Aizee IK/workspace inputs and Minerva placeholder derivation |
| `*_calibration.json`, `openrb_left.json`, `openrb_right.json` | Existing leader/follower coordinate calibration snapshots |
| `joint_align.json`, `ready_pose.json`, `calibration_poses.json` | Collector alignment, ready pose and gravity-calibration inputs |
| `minerva_gravity.json` | Existing empirical gravity model; not a validated simulator profile |
| `systemd/`, `udev/` | Deployment templates and device identities |

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

Calibration snapshots are retained because active collectors and tools load
them. They do not establish approved limits for new hardware. The simulation
audit identifies unresolved actuator scales and reversed directional bounds;
see [measurement requirements](../docs/architecture/SIMULATION_IMPLEMENTATION.md).
Head/lift slots are inactive in Minerva's shared definition.

Use `config/local_hardware.yaml` for a local hardware override (Git-ignored).
Large data, trained weights and generated plots belong in ignored output
directories, not in `config/`.
