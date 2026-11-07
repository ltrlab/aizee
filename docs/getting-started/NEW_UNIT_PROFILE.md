# A new unit's profile

Everything in `config/devices/` for Minerva describes **one robot**, the unit the repository was developed on
(`minerva-production-01`). The identities in it (addresses, serial numbers, host key, motor UIDs) and the measurements
(zeros, limits, tuning, camera mounts) are that unit's and are wrong for any other. A second robot shares the code but
needs its own copy of the files below. This page says which file holds which identity or measurement and how each one
is made. [CALIBRATION.md](../operator/CALIBRATION.md) gives the order to make the measurements in.

The repository's values are an example of the format and a starting point for the same build, not defaults to keep. The
release carries every `config/devices/*.json`, so a changed file reaches the robot at the next deploy.

## How the files fit together

`minerva-joint-calibration.json` is the root. It names the device file, the mechanical reference and the model, and it
holds the SSH connection block. The tools take it as `--profile`; Helm, Minerva Check and the deploy start from it.
The measured files are named after the asset id (`minerva-production-01-...`). Each records its `asset_id` and the
reference it was made against (`reference`, `reference_sha256` or `reference_id`). A measurement made against another
reference is stale, and the limits file keeps the reference's hash so the mismatch shows.

On the robot Helm does not edit the release's copies. It keeps the files the operator saves (limits, servo
calibration, leader ranges, the arms' reference, the head's files) in `~/minerva/helm/operator/`, seeds missing ones
from the release at each start, and sets an operator copy aside in `operator/replaced/` when a deploy changes the
release's version. So a measurement made in Helm lives on the robot first; copy it back into the repository to keep it
with the build.

## Identity files: written by hand

| File | What it holds | How to make it |
| --- | --- | --- |
| `minerva-production-01.json` | The robot computer: `hostname`, `address` (its address on the home network, used for the certificate and the bridge's last-resort route), `module_serial`, `machine_id`, the CAN helper path, each bus (`interface`, the adapter's `usb_serial`, the hardware profile), the services that must not run, and the motor models with the evidence for them | From the Jetson: `cat /etc/machine-id`, `hostname`, and the adapters' USB serials (`udevadm info`). The adapter serials also go into `config/udev/80-minerva-can.rules` and `scripts/setup_minerva_system.sh` ([INSTALL_JETSON.md](INSTALL_JETSON.md#3-can-adapters-helpers-and-sudoers)). Motor models are read off the labels; Minerva Check's motor scan reads each motor's identity and firmware |
| `minerva-joint-calibration.json`, block `connection` | The SSH user, the key file (relative to the profile; override with `AIZEE_SSH_KEY_FILE`), the staging folder, and `host_key_sha256`, the pinned fingerprint of the Jetson's SSH host key | `ssh-keyscan -t ed25519 <jetson-ip> \| ssh-keygen -lf -`. Reflashing the Jetson changes it |
| `minerva-joint-calibration.json`, other blocks | The wizard's steps and wording, `reference_positions`, `preview_transmissions` for the grippers | Generic for the same build; edit only if the mechanism differs |
| `minerva-leaders-01.json` and `gello-dual-viewer.json` | The leaders: USB serial of each board, the leader arm's geometry for the viewer, and each trigger's channel and open direction | `gello-dual-viewer.json` is the leader profile every teleop profile names; `minerva-leaders-01.json` is the bench identity file. Serials from Device Manager; left and right by moving one leader ([INSTALL_OPERATOR_STATION.md](INSTALL_OPERATOR_STATION.md#3-the-leaders)) |
| `minerva-head.json` | The head: the Waveshare adapter's `/dev/serial/by-id` path and USB serial, the two servos' ids and model, the head receiver's limits, motion defaults, the camera link and origin in the model | `robot-platform neck ports` and `neck probe` on the Jetson give the path, serial and ids ([NECK_FEETECH.md](../robots/minerva/NECK_FEETECH.md)). Motion values are starting points for the head's jog |
| `minerva-ups.json` | The UPS: I2C bus and address, shunt, cell count, the shutdown thresholds | Same module and pack: keep. `capacity_wh` and `pack_ohm` are assumed for three 18650 cells until a logged discharge measures them ([UPS_MONITOR.md](../robots/minerva/UPS_MONITOR.md)). Install again on the Jetson after editing |
| `minerva-task-capture.json` | Recording: rate, age and skew limits, the camera sockets, the wrist cameras' USB ids, the head pose socket | Keep for the same camera ports. Change the USB ids in `src/robot_platform/helm/cameras.py` and the wrist YAML files if the cameras differ |

## Measured files: made on the robot

Make these in this order. Each depends on the one before it, and each names it, so redoing an earlier one invalidates
the later ones.

| File | What it holds | How it is made |
| --- | --- | --- |
| `minerva-reference-<date>.json` | The **mechanical reference**: every motor's bus, id, UID, firmware and reference reading with the arms in their zero pose and the grippers closed, plus whether a power cycle kept it | The stopped procedure in [MECHANICAL_REFERENCE.md](../operator/MECHANICAL_REFERENCE.md), or Helm's Joint zero. The profile's `reference` names the current file. `minerva-reference-20260924.json` is the earlier one it supersedes |
| Directions (in the Helm runs folder, `<asset>-directions/`) | Each motor's sign against the CAD, marked verified | Helm's calibration wizard (Flip, Looks right): [JOINT_CALIBRATION.md](../operator/JOINT_CALIBRATION.md). `docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json` is one unit's, used to seed a fresh runs folder |
| `<asset>-joint-limits.json` | Each joint's hand-swept range, from stopped reads, bound to the reference's hash | Helm's **Limits** tab |
| `<asset>-servo.json` | Per-joint servo calibration: gains found by the Tune sweep, gravity fits, inertia and friction | Helm's **Tune** tab, with the arms enabled and attended. The simulator's fitted actuator file, `config/devices/simulation/<asset>-actuators.json`, is built from real teleop for a given set of motors (UID and firmware) and applies only to them |
| `gello-dual-leader-ranges.json` | Each leader channel's centre and span, and the leader zero | Helm's **Leaders** tab |
| `<asset>-head-reference.json`, `-head-limits.json`, `-head-servo.json`, `-head-camera.json` | The head's centred reference, swept limits (with the shoulder-clearance corners), servo settings and the head camera's mount | Helm's **Limits** tab, head section, and the head camera alignment; the CLI's `neck` commands exist for the bench only |
| Wrist camera calibration (`~/minerva/helm/calibration/<run>/result.json`) | Each wrist camera's intrinsics, distortion and mount on the gripper | Helm's wrist-camera stills and solve: [WRIST_CAMERA_CALIBRATION.md](../operator/WRIST_CAMERA_CALIBRATION.md). The solve writes into its run folder, not into a config file |

## Supporting files, usually reused

| File | What it holds |
| --- | --- |
| `minerva-wrist-cameras.json` | The wrist-camera rig: which model link each camera sits on, the image size, how many frames to stack for a still. These are choices with their sources written in, not measurements of the unit |
| `minerva-head-alignment.json` | Settings for the head camera alignment procedure (frames per still, tolerances). The result is saved as the unit's `-head-camera.json` |
| `minerva-table-board.json` | The table's fiducial tags: positions on the taped rectangle, the tag family and size. Re-measure if you lay out your own board ([TABLE_FIDUCIALS.md](../operator/TABLE_FIDUCIALS.md)) |
| `minerva-teleop-direct-tuning.json` | The production teleop profile Helm runs: 100 Hz, collision checking off, the leader profile and ranges it uses. It names the calibration profile; it holds no per-unit identity itself |
| `minerva-teleop.json`, `minerva-teleop-direct.json`, `minerva-teleop-direct-trial.json` | The guarded (collision-checked) teleop profile and two direct variants; Helm's `--teleop` takes any of them |
| `minerva-gello-shadow.json` | Simulation-only leader routes |

## Checklist for a second unit

1. Copy the files in the first table to new names with the new asset id and edit the identities.
2. Point `minerva-joint-calibration.json` at the new device file and a new, empty reference; the old reference, limits,
   servo and range files describe other motors and must not be reused.
3. Update the constants that still name the first unit: the account and path in `ops/deploy.py` and `ops/network.py`,
   and `CALIBRATION` and `robot` in `ops/release.py`. They are code, not profile entries.
4. Deploy ([INSTALL_JETSON.md](INSTALL_JETSON.md)), then work through [CALIBRATION.md](../operator/CALIBRATION.md).

Step 3 has not been done: only one unit exists, so nothing here has been run for a second.
