# Calibrating a new unit, in order

The order matters: each step is bound to the one before it. A new mechanical zero invalidates the limits, directions and servo
sweep; a new reference or limit invalidates what hangs on it. Do the steps in Helm unless a step says otherwise, with the
motors disabled until the Tune step. Safety: [Safety](SAFETY.md). The profile a new unit starts from is in
[A new unit's profile](../getting-started/NEW_UNIT_PROFILE.md).

On the robot, Helm keeps its saved copies of the measured files in `~/minerva/helm/operator/`; the repository holds the
committed starting copies in `config/devices/`. `<asset>` is the unit's asset id (`minerva-production-01` for the one unit
in this repository).

| # | Step | Writes | Per unit? | Procedure |
|---|---|---|---|---|
| 1 | **Mechanical zero**: each motor's zero at the documented reference pose, saved on the drive, then a power-cycle check | The drives' flash (zero); a new reference revision: `config/devices/minerva-reference-<date>.json`, or `<asset>-arms-reference.json` in the operator files when done through Helm's Joint zero | Yes | [Mechanical reference](MECHANICAL_REFERENCE.md); in Helm: [Joint zero](HELM_MANUAL.md#limits) |
| 2 | **Directions**: move each disabled joint by hand, **Flip** one that turns the model the wrong way, **Looks right** | The direction session (`calibration_runs/<asset>-directions/session.json` under Helm's runs folder) | Yes | [Joint calibration](JOINT_CALIBRATION.md); [Helm Limits](HELM_MANUAL.md#limits) |
| 3 | **Limits sweep**: each joint to both ends, **Confirm**, **Save limits** | `<asset>-joint-limits.json` | Yes | [Helm Limits](HELM_MANUAL.md#limits) |
| 4 | **Leader ranges and zero**: sweep each leader channel, **Save**, **Set leader zero** | `gello-dual-leader-ranges.json` (bound to each leader board's USB serial) | Yes, per pair of leaders | [Helm Leaders](HELM_MANUAL.md#leaders) |
| 5 | **Tune servo sweep**: gains, speeds, braking, gravity slice, tracking bound per joint, **Save selected** | `<asset>-servo.json` (bound to each motor's UID and firmware) | Yes, and again after replacing a motor | [Helm Tune](HELM_MANUAL.md#tune); the powered gates in [powered commissioning](POWERED_COMMISSIONING.md) |
| 6 | **Check**, including both live sweeps | A report under `calibration_runs/minerva-check/` | Each session | [Helm Check](HELM_MANUAL.md#check) |
| 7 | **Head**: reference, directions, limits and shoulder clearance (Limits > Head), then a Tune jog | `<asset>-head-reference.json`, `<asset>-head-limits.json` (with the clearance), `<asset>-head-servo.json` | Yes | [Helm Head](HELM_MANUAL.md#limits); [neck](../robots/minerva/NECK_FEETECH.md) |
| 8 | **Wrist cameras** on the table's tags (Teleop > Calibrate > 1 Wrist cameras), then **Solve** | A run folder under `~/minerva/helm/calibration/<run>/` (`stills.jsonl`, `images/`, `result.json`) | Yes | [Wrist camera calibration](WRIST_CAMERA_CALIBRATION.md) |
| 9 | **Head camera alignment** (Calibrate > 2 Head camera), **Save alignment** | `<asset>-head-camera.json`; the camera's orientation is set on the camera service | Yes | [Head camera](HEAD_CAMERA_ORIENTATION.md) |
| 10 | **Digital twin** correction (Calibrate > 3), drawing only | `<asset>-twin.json` | Yes | [Wrist camera calibration](WRIST_CAMERA_CALIBRATION.md) |
| 11 | **Table fiducials**: stick, measure and record the tags the cameras calibrate against | `config/devices/minerva-table-board.json` (tag positions); the workcell file for simulation | Per table | [Table fiducials](TABLE_FIDUCIALS.md) |

Steps 8 to 10 need the table's tags in place, so do step 11 before them. The camera steps need a live head and the arms in a
session, so they come after Check.

## What is per unit

Everything measured on a robot is specific to that robot and must be redone for a new one: the reference and its zeros, the
directions, the limits, the leader ranges, the servo sweep, the head's files, the camera mounts, and the table. The device
profile `config/devices/minerva-production-01.json` and the joint calibration profile
`config/devices/minerva-joint-calibration.json` pin one unit's host, adapters, motor identities and reference; a second unit
needs its own (see [A new unit's profile](../getting-started/NEW_UNIT_PROFILE.md)). What is shared, because it is the design
rather than a measurement: the robot definition, the model, the production teleop profile's structure, and the RobStride
manuals.

## Rules that hold throughout

- A value saved in one step names the revision it depends on (the reference's id and hash, each motor's UID and firmware).
  Helm refuses a file whose binding no longer matches and says why. Redo the later steps; do not edit the files.
- Calibrations and reference saves run through Helm's flows, so each can be repeated at any time without a new release.
- Nothing here is `runtime_approved`; a calibration does not qualify motion. Check, then an attended session.
