# Minerva episode format 9

Task capture episodes are HDF5 files named `task_episode_<uuid>.hdf5`, written by
`robot_platform.data.task_capture.TaskEpisodeWriter` for both the live robot (Helm's task capture) and the simulation
(`simulation.task_scene`). This page is the reference for the file layout. Capture behaviour is in
[HELM_TASK_WORKFLOW.md](../operator/HELM_TASK_WORKFLOW.md) and
[MINERVA_TASK_SCENE.md](../simulation/MINERVA_TASK_SCENE.md). LeRobot export is in
[DATASET_PIPELINE.md](DATASET_PIPELINE.md).

Format 9 records the **whole robot**: 16 axes, the arms' measured effort and velocity, and the head's pose. Format 8
recorded the two arms and grippers alone (14 axes) and is still read by the exporter, Replay and the episode source.

## The 16 axes

Every per-axis array uses this order, stored in the `minerva_joints` attribute (`robot_platform.data.roster.AXES`):

| Columns | Axes | Receiver | Units |
| --- | --- | --- | --- |
| 0–5 | `left_arm_j1` … `left_arm_j6` | RobStride, CAN | receiver motor radians |
| 6–11 | `right_arm_j1` … `right_arm_j6` | RobStride, CAN | receiver motor radians |
| 12–13 | `left_gripper`, `right_gripper` | RobStride, CAN | receiver motor radians |
| 14–15 | `head_pan`, `head_tilt` | Feetech STS3250, own process | raw servo radians (steps × 2π / 4096) |

These are motor coordinates, not CAD joint angles. The episode's `metadata_json` carries the joint map, with each
motor's sign, scale and offset onto its URDF joint. `head_pan` drives `neck_z` and `head_tilt` drives `neck_y`.

## Datasets

`T` is the number of frames. Every dataset is chunked one row per frame and compressed with gzip.

| Dataset | Shape, type | Contents |
| --- | --- | --- |
| `observations/qpos` | `[T, 16]` float32 | Measured positions. |
| `observations/qcmd` | `[T, 16]` float32 | The commanded positions, from the same receiver row as `qpos`. |
| `observations/torques` | `[T, 16]` float32 | Measured effort, N m (see below). NaN where unavailable. |
| `observations/torque_valid` | `[T, 16]` bool | True where `torques` holds a reading. |
| `observations/qvel` | `[T, 16]` float32 | Measured velocity, rad/s in each axis's motor coordinates. NaN where unavailable. |
| `observations/qvel_valid` | `[T, 16]` bool | True where `qvel` holds a reading. |
| `observations/neck_angles` | `[T, 2]` float32 | The neck's CAD angles `neck_z`, `neck_y` (rad). NaN without a head sample. |
| `observations/head_camera_pose` | `[T, 7]` float64 | The head camera's `[x, y, z, qx, qy, qz, qw]` in the robot frame. NaN without a head sample. |
| `observations/images/<role>` | `[T, H, W, 3]` uint8 | RGB per camera role, as stored (the live profile reduces 640×480 to 320×240). |
| `actions` | `[T, 16]` float32 | The next frame's `qcmd`; the last row repeats its own. |
| `action_valid` | `[T]` bool | False on the last row and before any change of authority. |
| `authority` | `[T]` string | `human`, `policy`, `correction` or `scripted_fixture`. |
| `task_state` | `[T]` string | JSON per frame: camera, receiver and head provenance, task events. |
| `timestamps/telem` | `[T]` float64 | The paired receiver sample's source time, s. Strictly increasing. |
| `timestamps/head` | `[T]` float64 | The head sample's time, in the same clock. Never later than `timestamps/telem`. |
| `timestamps/head_valid` | `[T]` bool | True where the frame has a head sample. |
| `timestamps/camera_<role>` | `[T]` float64 | Each camera's publisher timestamp, in the same clock. |
| `timestamps/scene` | `[T]` float64 | Simulation time; NaN for live capture. |
| `timestamps/scene_valid` | `[T]` bool | True in simulation. |

### Effort

- **Arms and grippers.** The effort each RobStride reports, as the receiver's feedback row carries it.
- **Neck.** The STS3250's `Present_Load` register (‰ of full load) × `full_load_nm` (4.9 N m) from
  `config/devices/minerva-head.json`. That is a rated figure, not a calibration, so the neck's effort is an estimate and
  coarser than the arms'.
- **Simulation.** There is no effort, velocity or head sample. All are NaN with their masks false. The neck's
  `qpos`/`qcmd` columns are 0 (`metadata.head` says so).

### The head sample

The head runs on its own receiver and publishes its rows locally (`neck.head_pose.HeadPoseFeed`,
`tcp://127.0.0.1:5596`, one row per 40 ms cycle). Each live frame takes the newest head row that capture had
**received** by the frame's receiver cutoff and that is at most `head.max_age_s` (0.1 s) old at that cutoff. Without
one, the frame is not recorded and capture reports it is waiting for the head. A row may serve two consecutive frames,
because its own time is recorded. `task_state.head` records the row's publisher time, receive time and age at the
cutoff.

The camera pose is computed from the neck angles: from Helm's saved head alignment when there is one, else from the CAD
mount. The arms are not moved by this; recording the head adds no work to the arms' control loop.

## Attributes

| Attribute | Value |
| --- | --- |
| `format_version` | `9` |
| `task_capture_schema` | `2` |
| `minerva_joints` | The 16 axis names, comma-separated, in column order. |
| `head_joints` | `neck_z,neck_y`: the columns of `neck_angles`. |
| `head_camera_pose_layout` | `x,y,z,qx,qy,qz,qw; robot frame; optical axes x right, y down, z ahead` |
| `authority_scope` | `whole_robot`: one authority per frame, covering both arms and the head. |
| `torque_units`, `action_units` | Text, as above. |
| `action_space`, `action_semantics` | `absolute`; next paired receiver `qcmd`. |
| `hz` | The requested recording rate. The achieved rate is in the timestamps. |
| `camera_roles` | JSON list of the recorded cameras. |
| `data_source` | `real` or `simulation`. |
| `metadata_json` | Capture profile, joint map, calibration and camera provenance. |
| `language_instruction` | The task text, or empty. |
| `collected_at` | ISO 8601 UTC. |
| `complete`, `frame_count` | Set when the episode is finished. A `.partial.hdf5` was never finished. |
| `capture_eligible` | Capture's own checks passed. A scripted fixture never is. |
| `training_promoted`, `sim2real_validated` | Always false at capture; review is a separate step. |

## Reading it

- Apply `action_valid` over a whole action chunk: a chunk must not cross an authority change or the episode's end.
- Use a reading only where its mask is true. NaN is never replaced by zero.
- LeRobot export (`python -m robot_platform.data.lerobot_export`) writes `observation.state` = `qpos` and `action` =
  `actions` over all 16 axes. It adds `observation.effort`, `observation.velocity` and `observation.head_camera_pose`
  only when every exported row has them. Episodes that differ in axes, cameras, image size or those extras are never
  mixed in one dataset.
- A policy trained on 16 axes cannot yet run on the robot: Helm's policy mode drives the 14 arm axes and refuses a policy
  whose joints differ. Driving the head from a policy is future work.
