# Status

What has run on a real robot, what has only passed offline tests, and what is open. Updated 2026-11-06. The dated
records behind this page are in [`history/`](history/) and [`devices/`](devices/).

## Minerva

**Running on the robot** (release `20261105T040341Z-fc68d9d`; this branch is ahead of it, see below):

- Both arms teleoperated live from the GELLO leaders, on the 100 Hz direct profile with collision checking off. First
  powered teleop 2026-09-27, both arms 2026-10-09, a 12-minute two-arm session with the head on 2026-10-31.
- Helm as a service on the Jetson, over HTTPS with a login, one device in control, with the GELLO bridge on the operator
  station.
- The neck on its own receiver, steered by a Quest Pro, including from another network over WebRTC (2026-11-03).
- Recording in episode format 8: 14 axes with no torque. One real demonstration at the last count (2026-10-22).

**Merged and passing the offline gate, not yet on a robot:**

- [Episode format 9](data/EPISODE_FORMAT.md): 16 axes, effort and velocity, the head camera's pose per frame. Recording
  refuses without a live head.
- The arms holding after a joint-speed trip instead of dropping torque.
- The lift over USB serial (`firmware/lift_actuator`), tested only in its simulator. `lift` stays inactive in the
  robot's definition and has no receiver.
- The simulation line: scripted demonstrator, generation, randomized layouts, table fiducials.

**Open:**

- Format 9's live test: the checks are in [its reference](data/EPISODE_FORMAT.md). Everything after it depends on data
  from it, so learning is unproven on the robot.
- A policy trained on format 9's 16 axes cannot drive the robot yet: policy mode runs the 14 arm axes and refuses a
  policy whose joints differ. It fails closed.
- No collision checking in the production teleop profile, and no independent hardware emergency stop that covers every
  actuator: [Safety](operator/SAFETY.md) says exactly what stops what.
- `lift` and the rover base are not under control. There is no powered base.
- One unit only. Everything in `config/devices/` was measured on it: see [a new unit's profile](getting-started/NEW_UNIT_PROFILE.md).

## AIZEE

Kept as a robot option and not under active work. [Its page](robots/aizee/README.md) says what it runs and which
`aizee-*` names are really Minerva's.

## The offline gate

`python scripts/dev.py check` passes with no skips. It does not cover what needs torch or OpenCV (`tests/ml`, the
learning and policy-session tests, the collector simulation process), Linux services or hardware.
One timing test, `test_a_live_head_cycles_in_a_process_of_its_own_so_the_arms_never_wait_for_it`, fails about one gate
in four on a Windows laptop because a scheduler stall exceeds its bound. It has not been relaxed; rerun the gate.
