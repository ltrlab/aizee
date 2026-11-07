# Task setup and recording: both arms, cube handoff and bucket

Helm's task card sets up a task and records episodes in a teleop session. It never creates another motor publisher and never
enables the robot. The same card runs against the simulator (`helm --sim`, for rehearsal; see
[Simulate](../getting-started/SIMULATE.md)) and against the robot.

**Status.** The running release records episode format 8 (14 axes, no torque); one real demonstration had been recorded at the
last count. The code in this repository records [format 9](../data/EPISODE_FORMAT.md) (16 axes with the head, effort and
velocity, the head camera's pose) and refuses to record without a live head. **Format 9 has never run on the robot.**

## In the simulator

Run Helm with `--sim` on a development PC ([Simulate](../getting-started/SIMULATE.md) has the command). The header says
**Simulation only**. The scene defaults to `--sim-visual-mode collision_proxy` (camera images from the collision geometry, to
reduce rendering cost); `cad` selects full CAD visuals. This changes appearance, not contact geometry or the watchdogs. The
process refuses physical connection, calibration, tuning, receiver and Jetson-target actions. The leaders' USB is opened only by
an explicit leader connection or session start; close other programs that own their serial ports first.

1. Open **Teleop**, then **Set up task** before starting the session. It selects the cube, table and bucket contact scene; it
   opens no USB.
2. Check the saved leader identities, ranges and robot mappings in **Leaders**.
3. **Start session**. A MuJoCo window shows both arms, the cube, the table and the bucket. **Reset simulated scene** is
   available only in disabled or Idle states and never during a recording; it does not move the arms.
4. **Idle (I)**, **Enable (E)** to hold, **Engage (T)** to align and follow. T holds; H disables; X returns to the reference.
5. **Record episode** while holding or following. Pinch and lift the cube, hand it to the other gripper and release it into the
   bucket. The status reports contact-derived progress; a failed attempt is still a recorded attempt, never auto-labelled
   successful.
6. **Stop recording** and wait for `complete` and the final HDF5 path. Stop recording does not stop the arms. Closing a session
   or scene during a recording aborts the episode; it does not publish a completed file.

The simulated arms keep the calibrated independent-joint response; contact reaction does not feed back into the arms, so this is
not a validated coupled dynamics model. See [the scene guide](../simulation/MINERVA_TASK_SCENE.md).

## On the robot

Helm on the Jetson starts with the capture profile `config/devices/minerva-task-capture.json`. An owned subprocess on the Jetson
receives bounded scalar snapshots from the teleop receiver and owns the camera subscriptions, JPEG decoding and the HDF5 writer.
RGB images stay on the Jetson and do not pass through the control loop's IPC or Helm's status RPC.

Prepare as for any live session ([Safety](SAFETY.md), [Check](HELM_MANUAL.md#check)): attended, foot switch in reach, a clear
path. Choose **Live** and **Both** (a one-arm receiver cannot configure this capture), then **Set up task**. Do not run the
legacy collector beside Helm; Helm owns the receiver.

The capture requires **left_wrist, right_wrist and head**, the 16 named axes and the head pose feed. All must be fresh before
recording, and Helm never fills a missing view:

| Capture role | Publisher | Jetson loopback endpoint | Camera |
| --- | --- | --- | --- |
| left_wrist | `left_wrist` | `tcp://127.0.0.1:5563` | USB UVC |
| right_wrist | `right_wrist` | `tcp://127.0.0.1:5565` | USB UVC |
| head | `scene_cam` | `tcp://127.0.0.1:5564` | Intel RealSense D435 on the neck; pose recorded per frame |

Check the **3/3 cameras fresh** indication each session; service status says nothing about current freshness. Capture stores RGB
only, even though the D435 also publishes depth. The head camera moves with the neck: each frame records the neck's angles and
the camera's pose from the head pose feed (`tcp://127.0.0.1:5596`, at most 0.1 s old at the frame's cutoff). That pose is Helm's
saved head alignment when there is one, else the CAD mount. Stored images' intrinsics are not recorded.

`recording_image_size: [320, 240]` downsamples each 640 x 480 RGB frame by an integer factor with `Image.reduce(2)` (recorded as
`operation: integer_reduce` with the Pillow version, shapes and RGB order); live previews keep 640 x 480. The profile requests
**15 Hz** (the head publishes about 15 fps and every recorded frame needs a new acquisition from every camera). Admission bounds:
receiver age at most 30 ms, camera age at most 200 ms, inter-camera publisher timestamp skew at most 20 ms. These are capture
admission limits, not measured synchronization guarantees; publisher stamps are host read-complete times, not exposure times.
Check the recorded rate and timing.

1. Confirm all three cameras are fresh.
2. Start the bimanual session. The task panel reports each camera as fresh, waiting or invalid and refuses missing, duplicate,
   backwards, stale, future, wrong-identity and malformed data.
3. Verify the saved limits and both arms' feedback before Idle, Enable, Engage. **Record episode** (or pedal **A**) becomes
   available only with a paired receiver sample, active command authority in Holding or Executing, and all cameras fresh at the
   conservative cutoff.
4. Perform the task and finish the episode explicitly (A again, or **Stop recording**). Judge success by review; the collector
   does not infer contact or success.
5. Confirm the file reads, all 16 axes, the effort and head-pose masks, the three RGB streams, timestamps, action validity and
   provenance before promoting the episode into a dataset ([Replay](HELM_MANUAL.md#replay-tab) to review).

After preprocessing, a fresh monotonic clock checks the same paired sample against 30 ms and the same camera acquisitions against
200 ms; exhausting either budget aborts the capture rather than restamping a frame. Raw receiver stamps, the status interval, the
1 ms quantization uncertainty and the RPC round trip stay separate provenance fields.

## Files, failures and downstream use

`samples.f32`, `columns.json` and `events.jsonl` remain diagnostic outputs. The task writer produces
`episodes/task_episode_<id>.hdf5` under the session folder (`~/minerva/evidence/live-<time>/`). A file stays `.partial.hdf5`
until a successful finish drains and validates the writer, then it is atomically published. Stale data, queue overflow, decode or
disk errors, loss of the receiver's Holding/Executing authority, or an interrupted closure keep the incomplete data and report a
failure; parent loss cancels pending publication. Stop recording and wait for completion before disabling or ending the session.
During teardown the owner uses one bounded 5-second budget; if it times out the child exits with failure and the partial file is
never published as complete. Each process keeps a `capture-worker-*.stderr.log` beside the episodes.

A recording or diagnostics failure latches **RECORDING FAILED** for the session, requests the episode abort and stops following
the leaders: a healthy receiver moves to its guarded HOLD with torque on. It does not return or disable the arms for a camera or
disk failure. Enable, engage, route and zero edits, tuning and new recordings stay locked until the session is stopped and
restarted. H still disables immediately; X and Stop session request the controlled return, so make sure that path is clear of
the table, the object and obstacles. Entering this HOLD captures the measured positions of every axis, including both grippers,
and may remove a grip squeeze: **object retention is not qualified**.

The raw capture ([EPISODE_FORMAT.md](../data/EPISODE_FORMAT.md)) declares the camera roles and joint names; paired measured and
emitted commanded motor coordinates; measured effort and velocity; the head's neck angles and camera pose; RGB images; source,
head, scene and camera clocks; per-frame task state; authority; terminal and authority-boundary action-validity masks; and source
provenance. An unavailable effort, velocity or head reading is NaN with its mask false. Simulation is labelled synthetic. The
format is deliberately excluded from legacy `episode_*.hdf5` discovery: a validator that honours the masks must promote it before
training. A completed recording is not automatically a validated dataset, a successful task, or a qualified policy. Export and
training: [Dataset pipeline](../data/DATASET_PIPELINE.md), [Training](../data/TRAINING.md).
