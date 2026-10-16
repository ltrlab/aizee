# Helm task walkthrough: both arms, cube handoff and bucket

This workflow adds task setup and episode recording to Helm's existing guarded
teleoperation session. Simulation and robot mode are selected at process launch.
The task controls never create another motor publisher or enable the robot.

## Simulation on the operator PC

From the repository root, with the project's development environment prepared:

```powershell
.\.venv-dev\Scripts\python.exe -m robot_platform helm --profile config/devices/minerva-joint-calibration.json --teleop config/devices/minerva-teleop-direct.json --sim
```

Open `http://127.0.0.1:8760/`. The header must say **Simulation only**.
This simulation command uses the 100 Hz direct profile from the visible-scene
probe: collision enforcement is disabled, but the guarded receiver and task
contact validity checks remain. It is not a hardware operating prescription. The task
scene defaults to `--sim-visual-mode collision_proxy`: its camera images use the
source collision geometry to reduce rendering cost. `--sim-visual-mode cad`
selects full CAD visuals. The worker reports the actual mode in Helm and episode
provenance. This changes image appearance, not contact geometry or the control
watchdogs; full CAD capture still has to meet the same timing checks.

This mode does not construct an SSH connection or stopped live reader and refuses
physical connection, calibration, tuning, receiver and Jetson-target API actions.
It reads the saved robot calibration and actuator parameters locally. Both leader
USB devices are opened only by an explicit leader connection or session start.
Close other programs that own their serial ports before that operator action.

1. Open **Teleop**, then select **Set up task** before starting the session.
   Setup selects the cube/table/bucket contact scene; it does not open USB.
2. Check the saved leader identities, ranges and robot mappings in **Leaders**.
   Use the existing explicit connection controls when ready to operate the leaders.
3. **Start session**. Helm unloads its duplicate CAD preview while the task
   session runs, preserving the controls and telemetry. A separate MuJoCo scene
   window shows both arms, the cube,
   table and bucket. **Reset simulated scene** is available only in disabled or
   idle lifecycle states and never during a recording. Reset does not reposition
   the arms or recapture leader zero.
4. Use **Idle (I)**, **Enable (E)** to hold, then **Engage (T)** to align and follow.
   Both arms and both grippers participate. T again holds; H disables immediately;
   X requests the existing controlled return to the reference pose.
5. Choose **Record episode** while holding or following. Use the leaders to pinch
   and lift the cube, hand it to the other gripper, and release it into the bucket.
   The task status reports contact-derived progress; a failed attempt is still a
   recorded attempt, never automatically labeled as successful.
6. Choose **Stop recording**, and wait for `complete` and the final HDF5 path.
   Stop recording does not stop robot motion. Use the existing hold/shutdown
   controls and **Stop session** to end the session. Closing a session or scene
   during recording aborts the episode; it does not publish a completed file.

The simulated arms retain the calibrated independent-joint ArmPlant response.
Their CAD geometry is posed into a MuJoCo contact scene; the cube is dynamically
stepped against it, the table and the bucket. There is no synthetic grasp
attachment. Contact reaction does not feed back into the arm dynamics, so this is
not a validated coupled robot/object dynamics model or hardware qualification.
The episode records this provenance, simulated camera stamps and task validity.
See [the scene implementation guide](MINERVA_TASK_SCENE.md) for contact assumptions
and automatic quarantine conditions.

## Attended real capture with both arms

Prepare the existing production Helm connection and calibrated device profile.
An owned subprocess on the Jetson receives bounded scalar snapshots from the
existing teleop receiver. It owns the camera subscriptions, JPEG decoding and
HDF5 writer; RGB images stay on that host and do not traverse the control loop's
IPC or Helm's control/SSH status RPC. Imports initialize in the child, and camera
readiness requires actual decoded frames. Worker startup, status stalls and exit
are reported as failures. This isolation still requires a loaded receiver and
recorder timing check on the Jetson before hardware collection is qualified.

```powershell
.\.venv-dev\Scripts\python.exe -m robot_platform helm --profile config/devices/minerva-joint-calibration.json --teleop config/devices/minerva-teleop.json --capture-profile config/devices/minerva-task-capture.json
```

The robot example deliberately retains `minerva-teleop.json`, whose bimanual
live session enforces the existing motion envelope. It is a different workload
from the 100 Hz collision-disabled `minerva-teleop-direct-tuning.json` used for
isolated Jetson capture timing. A direct-profile timing pass does not qualify
this profile or overlapping physical handoff. The intended live profile must
pass its own timing and attended readiness checks before collection.

Robot mode retains the existing stopped reader and live connection behavior.
Select **Tether**, **Minerva's Jetson**, and **Live · both**, then **Set up task**.
A one-arm receiver cannot configure this capture workflow. Do not launch the
legacy collector alongside Helm; Helm owns the current receiver session.

Before enabling movement, complete the existing attended robot checks, verify the
inter-arm/workspace collision strategy for the intended handoff, and keep the
existing physical stop accessible. This addition records data; it does not change
motion gains, watchdogs or collision enforcement, and does not qualify a
collision-disabled profile for overlapping bimanual motion.

The initial profile explicitly requires right_wrist and head, per the operator choice.
Both arms and both grippers still participate. A later three-camera profile may add
left_wrist; the file records its exact camera roster and never fills an omitted role:

| Capture role | Publisher identity | Jetson loopback endpoint | Physical identity |
| --- | --- | --- | --- |
| left_wrist | `left_wrist` | `tcp://127.0.0.1:5563` | USB `05a3:9230` |
| right_wrist | `right_wrist` | `tcp://127.0.0.1:5565` | USB `32e4:9230` |
| head | `scene_cam` | `tcp://127.0.0.1:5564` | Intel RealSense D435, static neck mount; extrinsics unknown |

The wrist configurations request JPEG 640 × 480 at 30 Hz. Service status alone
does not prove image availability. On the read-only device check of 2026-10-16,
the initial check found no cameras. After the operator connected the right wrist
camera and static-neck Intel RealSense D435, the subsequent check verified fresh
640 × 480 right_wrist and scene_cam RGB streams (and D435 depth at its publisher).
The left wrist camera was still absent. This capture boundary stores RGB; it does
not silently add a depth modality. The D435 mount is recorded as static neck and
its extrinsic calibration remains unknown: no camera-to-robot transform is invented.
These are dated observations, not permanent device state. The GUI counts actual
fresh streams once the bimanual session starts: 2/2 for the initial profile. A future
three-camera profile will require the left wrist too.

1. Connect/verify both configured cameras and their existing publishers. Confirm that
   each publisher's read-complete timestamp uses the same Jetson Unix clock; the
   supplied loopback profile is invalid for a relay from another host clock.
2. Start the existing bimanual session. The task panel reports each camera as
   fresh, waiting or invalid. It refuses missing, duplicate, backwards, stale,
   future, wrong-identity and malformed camera data. It never fills missing images.
3. Verify the saved limits and both arms/grippers' fresh receiver feedback before
   Idle/Enable/Engage. **Record episode** becomes available only with a paired
   receiver sample, active command authority in Holding/Executing, and all
   configured cameras at its conservative observation cutoff.
4. Perform the attended task and finish the episode explicitly as in simulation.
   Inspect the final path and completed frame count. Evaluate task success by
   operator review; the physical collector does not infer object contact/success.
5. Confirm file readability, all 14 named axes, both configured RGB streams, timestamps,
   action validity and provenance before promoting the episode into a dataset.

The profile's current capture admission bounds are 20 Hz requested recording,
receiver age ≤ 30 ms, camera age ≤ 200 ms and inter-camera publisher timestamp skew ≤ 20 ms.
The receiver age is limited by the existing receiver feedback timeout. Camera
bounds are explicit capture configuration, not measured synchronization guarantees.
The current publisher timestamps are host read-complete times, not known sensor
exposure times; this limitation is preserved in metadata. Frames are selected at/before a conservative receiver cutoff and only new publisher-stamped camera
frames are recorded. Actual achieved rate and recorded timing must be checked;
recording does not silently relax these bounds. Publisher frame counters can reset,
so capture preserves the raw counter and adds its own monotonic receive identity.

Receiver status exposes elapsed whole milliseconds while recorder rows retain
fractional milliseconds. Capture admits a row only below the next millisecond
boundary and evaluates the 30 ms age limit and camera cutoff conservatively from
that upper boundary. Feedback stamps must remain at/before the status value.
Raw receiver stamps, the status interval, the 1 ms quantization uncertainty and
RPC round-trip duration remain separate provenance fields; total clock uncertainty
includes both the quantization and RPC duration. A row from a later millisecond
bucket is rejected rather than assumed to share the status time.

## Files, failures and downstream use

Existing `samples.f32`, column metadata and event logs remain diagnostic outputs.
The shared task writer produces `episodes/task_episode_<id>.hdf5` under the
receiver session directory, on the PC in simulation and on the Jetson for live
capture. The GUI shows the actual path. Files remain `.partial.hdf5` until explicit
successful finish drains and validates the writer, then they are atomically
published. Stale receiver/camera data, queue overflow, decode errors, disk errors
or interrupted closure retain incomplete data and report a failure. Loss of the
receiver's enabled Holding/Executing authority also aborts the active episode;
stopped observations cannot become emitted-command training rows. Stop recording
and wait for completion before disabling or ending the session. Parent loss or
session closure cancels pending publication, including a finish waiting on disk.
Each process keeps a bounded `capture-worker-*.stderr.log` under its episode
directory for startup/native diagnostics.

The raw v8 capture declares camera_roles and joint names; paired measured and emitted commanded
motor coordinates; RGB images; source, scene and camera clocks; per-frame task
state; authority; terminal/authority-boundary action-validity masks; and source
provenance. Physical scene time is absent, and unavailable torque is explicitly
masked. Simulation is labeled synthetic. This format is deliberately excluded
from legacy `episode_*.hdf5` discovery: a validator/loader that honors its validity
masks must promote it before training. A completed recording is not automatically
a validated training dataset, a successful task, or a qualified control policy.
