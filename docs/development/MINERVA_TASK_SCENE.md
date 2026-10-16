# Minerva contact task scene and raw episode capture

The task is to pick up the cube with one gripper, pass it to the other, and release it into the bucket. Helm supplies both arm and gripper coordinates through the existing ArmPlant receiver adapter. The scene does not open USB, SSH, CAN, or physical motor services. The UI integration owns leader connections and explicit recording commands.

## Implemented mechanics and limits

`robot_platform.simulation.task_scene.MinervaTaskScene` uses the generated Minerva model for forward kinematics and displays the original CAD visuals. A separate MuJoCo model has one dynamically integrated body: a 36 mm, 35 g cube. The robot consists of FK-posed mocap bodies. A table and five-piece open bucket have ordinary rigid contact geometry. ArmPlant motor dynamics are outside this object scene; contact reaction does not load the arms or change motor response.

The source finger mesh is concave. MuJoCo's convex hull fills the open jaw, which prevented a valid pinch in an initial contact probe. The scene therefore fits one box to the straight terminal 30 mm of each finger's actual CAD vertex bounds. Those four boxes are real frictional contact surfaces. The remaining finger stem geometry is visual only. Other robot collision meshes retain their source convex geometry. The capability manifest lists every active collision geometry and records the scene XML hash, MuJoCo version, task dimensions and limitations.

Moving mocap bodies have no generalized velocity. Native MuJoCo `geom_surfacevel`, calculated from finite differences of FK geometry poses, supplies tangential contact velocity. Normal contact still comes from geometry penetration. No attachment, grasp equality, adhesion, cube teleport during manipulation, or synthetic grasp force is used. **MuJoCo 3.15 or newer is required**; older versions fail explicitly rather than falling back to immobile friction surfaces.

Contact friction and cube properties are synthetic assumptions, not measurements. The robot has no coupled dynamics, calibrated finger force, or self-collision response. Robot/table/bucket contact cannot push a kinematic arm back: explicit signed-distance checks record penetration deeper than 2 mm, preserve the offending geometry names, and make the episode ineligible. These runs are never marked `sim2real_validated`.

The table is 0.28 m above the source model base. The bucket rim is 0.36 m high. These heights were selected after checking reach within the current source joint limits; an initially considered 0.50 m table was not reachable with downward fingers. Positions are synthetic task settings, not measurements of a real workcell.

## Outcomes

A pickup requires force-bearing contacts on both opposing fingers of one gripper while the cube is above the table for at least 50 ms. A handoff requires simultaneous opposing contacts on both grippers followed by receiving-gripper support and giver release. Bucket placement requires the cube to be inside the bucket, contacting its bottom, released by both grippers and settled for 250 ms. A cube simply initialized or dropped into the bucket does not count as task success. A return to the table or floor after pickup records a dropped-cube failure. Forbidden environment penetration also prevents success.

`reset()` restores cube state, counters and events, generates a new run ID, and keeps the current robot pose. Position the arms clear of the table before resetting and recording. Reset during an active recording is refused so one episode cannot silently span multiple scenes.

## Thread and capture interfaces

`TaskSceneView(scene, joint_map, names, directory=..., metadata=..., viewer=True, camera_roles=("right_wrist", "head"))` owns physics, the visible MuJoCo window, rendering and capture commands on a separate thread. Its interface is:

- `show(q, qcmd=None, source_time=None, authority="human")`: nonblocking latest paired receiver-snapshot mailbox. Coordinates use the supplied 14-axis motor order and joint map. The adapter must pass q, qcmd and time from the same receiver row.
- `command("reset" | "start_record" | "stop_record", payload=None)`: bounded asynchronous request with an ID. `snapshot()["last_command"]` reports pending, complete or failed.
- `health()`: small callback status containing readiness, availability and recording errors; no geometry/events are copied.
- `snapshot()`: cached task events/outcome, capability manifest, capture readiness, camera timestamps, recording state and sticky failure information.
- `close()`: ends the scene. An unfinished recording is aborted, never published as complete.

The scene uses 0.5 ms contact steps and interpolates the latest robot pose over its measured source-time interval, preserving substep rounding remainder. Source gaps over 250 ms abort the scene/capture; readiness expires after 250 ms without source progress. These are provisional capture-quality bounds, not validated robot timing limits. Every interpolated moving substep is checked for forbidden environment penetration. This is a sampled visual/object simulation, not a replay of every 400 Hz receiver tick. Capture stores both source receiver time and scene time. Camera frames are rendered from one scene state, with explicit synthetic camera timestamps. Head and wrist camera views are synthetic viewpoints rather than calibrated physical camera extrinsics.

`robot_platform.data.task_capture.TaskEpisodeWriter` is shared by simulation and real collectors. `append` is keyword-only: `q`, `qcmd`, `source_time`, `scene_time`, `images`, `camera_times`, `task_state`, optional `authority` and `torques`. A real collector passes `scene_time=None`, declared publisher timestamps and explicit camera/clock provenance. A publisher timestamp may describe exposure or host read-complete; that stage and exposure-time uncertainty must be explicit. A collector receive timestamp must not silently replace it. This is not exposure synchronization certification. The initial operator profile explicitly records `right_wrist` and `head`, while both arms and both grippers still provide all 14 named axes. The writer validates the exact declared `camera_roles` roster and never fabricates a left-wrist image. An explicitly selected three-camera roster is also supported. Missing torque is NaN plus an explicit validity mask. Missing required coordinates, stale timestamps, dropped queue entries, or capture errors abort the episode.

HDF5 writes occur incrementally in a bounded worker queue. A unique `.partial.hdf5` remains unpublished until explicit `finish()` drains the queue, verifies at least two frames, closes and fsyncs the file, and atomically renames it. `abort(reason)` only signals failure and never publishes. Both methods expose sticky errors via `status()`.

## Raw schema and training boundary

Schema 8 stores the familiar `observations/qpos`, `observations/qcmd`, `observations/images/<camera>` and `actions` arrays, plus `observations/torques`, `observations/torque_valid`, `timestamps/telem`, `timestamps/scene`, `timestamps/scene_valid`, declared camera publisher timestamps, per-frame authority and JSON task state. File metadata declares real/simulation source, clock domains, joint ordering, capabilities, calibration provenance supplied by the adapter, completion and promotion state.

Actions are next-sample qcmd, preserving the old collector's convention. `action_valid` is false on the terminal row and immediately before an authority change so training must not construct a chunk crossing an intervention. Both arms share authority; this interface does not implement mixed human/policy control of separate arms. Training readers must apply that mask across the entire chunk and validate clocks, active units, quality and split provenance.

Completed raw files use `task_episode_*.hdf5`, intentionally outside the legacy `episode_*.hdf5` discovery glob. The old trainer does not understand the new masks or promotion contract. **Raw capture is not automatic training ingestion.** Dataset validation/promotion remains a separate deliverable. `capture_eligible` indicates local capture checks only; `training_promoted` and `sim2real_validated` remain false. Explicitly labelled scripted fixtures are never capture-eligible demonstrations.

## Evidence and reproduction

Run the focused regressions in the worktree's pinned developer environment:

```powershell
.venv-dev/Scripts/python.exe -m pytest tests/unit/test_task_scene.py tests/unit/test_task_capture.py -q
```

`tests/fixtures/minerva_task_contact_path.json` is a frozen offline IK trajectory, with a prepositioned start and an explicit fixture-only gripper mapping. It exercises genuine pickup, overlapping bimanual contact, release to the other hand, transport, free drop and bucket settling. It does not exercise leader USB, the Rust receiver, ArmPlant tracking, or a safe approach from the hanging pose. The regression checks real MuJoCo forces/events and no forbidden environment penetration, with no cube pose modifications during the trajectory. Negative controls prove bucket contact alone is not success and penetrating the table is ineligible.

Additional capture tests cover atomic publication, both real and simulation clocks, raw paired commands, missing torque, intervention boundaries, stale/missing cameras, malformed coordinates, queue overflow, abort and terminal samples. Rendered three-camera fixture evidence is stored under the ignored `build/task_evidence/` directory when generated locally; it is explicitly marked scripted fixture and excluded from training promotion.
