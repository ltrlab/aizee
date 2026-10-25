# Feature task: Minerva's head and neck in the robot model

- **Branch and worktree:** `claude/head-model` in `C:\Users\ltr\Workspace\aizee-head-model`.
- **Base commit:** `e7dd618b7c0668cb0d9c276c5ca2dce0c0d0def2` (`claude/helm-on-jetson`, the deployed Helm line).
- **Owner:** Claude (2026-10-25 session) for the CAD export. The collision model and the active model belong to the
  motion-contract owner (AGENTS.md: approach, collision envelope, guarded lifecycle, receiver checks).
- **Request:** the user added the head to the Onshape assembly as
  [Head V1+urdf](https://cad.onshape.com/documents/359ad50fa475851ee9487588/v/349129ac06492b67378d6724/e/f4466d56fce7b6028aae0a06)
  and asked for the model to be re-exported. Asked how the neck should enter the active model, they chose
  **"Movable neck now"** (not a fixed head) and to drop the loose servo horn. They added:
  - "The center of AWG_RGB_CAMERA is the origin point for where the head cam view is showing."
  - The neck runs on a Waveshare bus servo board (as on the SO-101 arm) with two Feetech STS3250s.
  - "Should have the same limits, teleop and all pipeline that we use for the robstrides", and "tuning parameters
    extracted too".
- **Onshape budget:** the user allowed **two export runs**. This used one: 631 requests.

## Done on this branch

The new export is in [urdf/minerva/onshape-to-robot-head-v1](../../urdf/minerva/onshape-to-robot-head-v1/README.md),
beside the untouched 1.1 export that the active model references.

- **The model:** 19 links, 18 joints.
  - `neck_z` (pan) is about +z at (0.1, −1.5, 1131.2) mm, from the base to the pan bracket.
  - `neck_y` (tilt) is about −x at (−18.0, −1.5, 1187.8) mm, from the bracket to the head.
  - The structural audit passes against expectations that include the two neck joints.
- **Nothing else moved:** the 16 arm and jaw joint frames match 1.1 within 0.005 mm and 0.001°, and the geometry
  of the existing parts is unchanged apart from re-tessellation.
- **Preview:** a profile, `Minerva (Head V1)`, shows the head with neck sliders in the 3D viewer. It has no
  collision model of its own.
- **Helm's 3D view draws it** (`helm.display_model`, branch `claude/head-in-3d-view`, at the user's request on
  2026-10-25). This is for display only:
  - Helm checks at start that this model is the active one plus the neck: every active joint has the same links,
    axis, frame, bounds and coupling. Otherwise the view draws the active model, as before.
  - Every guide (the stopped reader's, a session's, a sweep's) and the joint telemetry are drawn on it by joint name.
  - The neck shows the head's measured angles (`HeadTeleop.status()['joints']`, each servo through its axis) while
    a teleop session drives it. Otherwise nothing reads the neck, and the view draws it at its zero.
  - Its 73 meshes that the active model lacks go to Minerva's mesh store with the active model's
    (`ops.helm_robot meshes`).
  - The guard, the receivers, the guides and the recordings keep the active model.
- **Unchanged:** the active model (`urdf/minerva/model.json` and `minerva.urdf`), the collision models, the limit
  reviews and the MuJoCo export.

## Hazard now: the arms can reach the head

At zero, the head and neck occupy x −90…90, y −117…60 and z 1128…1428 mm. That box is 177 mm from the left arm's j2
origin and 244 mm from the right's, and each arm reaches about 945 mm from j2. **The active collision model has no
head.** If the head is mounted on Minerva, the guard does not know it is there, and nothing stops an arm from
hitting it in teleop, replay or a policy run. Helm's 3D view showing the head does not change this: it only draws
it.

**The user's decision (2026-10-25), after seeing the head in Helm's 3D view:** the head does not need to be added to
the collision checks. Step 1 below, and the collision half of step 2, are therefore not planned; the operator keeps
the arms clear of the head.

## Remaining work, in order

1. **Collision model (motion-contract owner). Not planned:** the user decided the head needs no collision checks.
   Kept for reference:
   - Add `neck_z` and `neck_y`, with the frames above, to a new collision URDF, plus simplified collision geometry
     for both neck links: the pan bracket and the head.
   - Extend the broad phase (`urdf/minerva/collision/broadphase-*.json`) and the pair lists for arm against head,
     the limit review and the keep-out policy.
   - Rerun the collision sweeps and timing. The neck's position must reach the guard as joint state.
2. **Compose the active model** from this export and that collision URDF with `scripts/compose_collision_model.py`.
   - It refuses mismatched joint sets, and visible links without collision geometry.
   - Then update `urdf/minerva/model.json`: a Head group, provisional neck limits and notes.
   - Regenerate the MuJoCo export (`urdf/minerva/mujoco`); its test pins the URDF hash and 17 placeholder links.
3. **Everything that assumes 16 joints.**
   - Helm's mappings, live view, telemetry and Tune.
   - Recording schemas and the LeRobot exporter.
   - Policy state and action vectors: add the neck there only on purpose.
4. **The neck controller (separate task).** Two Feetech STS3250 servos on a Waveshare bus servo board, the same
   serial bus servos and adapter as the SO-101 arm. The user wants the RobStride pipeline for them:
   - swept limits (Limits);
   - teleop;
   - tuning parameters extracted by sweeps (Tune), with Minerva Check covering them;
   - live telemetry and recording.

   Still undecided: the neck's teleop input, its signs and zero against the CAD, and the servos' mechanical travel.
5. **Head camera frame.** The view origin is the centre of `AWG_RGB_CAMERA`. At zero it sits at
   (32.8, −110.5, 1269.9) mm in the robot frame, or (−109.0, −82.1, −50.8) mm in the head link's frame. The
   optical axes still need defining, from a `frame_head_camera` mate connector in CAD or a fixed frame added at
   composition, before calibrating the head camera (see [HEAD_CAMERA_ORIENTATION](HEAD_CAMERA_ORIENTATION.md)).

## Acceptance and identity

- **Audit:** `python -m robot_platform sim audit-model --model urdf/minerva/onshape-to-robot-head-v1/model.json --expectations urdf/minerva/onshape-to-robot-head-v1/source_manifest.json`.
  It reports 0 errors and 0 warnings, with static checks passed.
- **Viewer:** `python -m robot_platform view --model urdf/minerva/onshape-to-robot-head-v1/model.json --port 8771`.
  Checked: the head renders, `neck_z` pans and `neck_y` tilts it.
- **Identity:**
  - URDF sha256: `e78833725fce41fd344ee67bd8c54661b54e1cf9ee21bd1f4860c1159390eca2`.
  - Exporter output before normalization: `cf74e84e5d9e3dbf892c41c529c3bc70266e7f18adb924ed5af18845e9fec00e`.
  - Every mesh is hashed in the manifest.

## Remaining uncertainty

- **Neck limits:** the exporter's ±π default for both joints. Neither the servos' travel nor the head's clearance
  has been measured.
- **Masses:** only 12 of 157 parts carry mass, eight arm links. There is no physics use yet.
- **Link names:** `part_6` and `awg_rgb_camera_5` come from part names. `link_` mate connectors in CAD would rename
  them, but that costs a re-export (cheap if only the assembly changes).
