# Minerva MJCF, kinematics first

**What it is.** A MuJoCo model of Minerva's 17-link arms-and-torso URDF (the neck has its own export,
[HEAD_NECK_MODEL.md](../robots/minerva/HEAD_NECK_MODEL.md)), generated from the URDF and committed at
`urdf/minerva/mujoco/`. **Status: done for kinematics; not usable for dynamics.** All 17 URDF masses are exporter
placeholders (`1e-9 kg`), so `MujocoPlant` still refuses the model. The contact task scene
([MINERVA_TASK_SCENE.md](MINERVA_TASK_SCENE.md)) uses the model's forward kinematics only.

Code: `src/robot_platform/models/mjcf.py`, `scripts/view_mujoco.py`, `tests/unit/test_mjcf_export.py`,
`tests/fixtures/physics/coupled_jaws.urdf`. The model was built to these bounds, which the tests hold:

- Link frames equal the URDF's within 1e-9 m and 1e-9 in rotation, at 200 seeded poses plus every joint at both ends of
  its range.
- Every mesh keeps its triangle count. Its world bounds and centroid sit within 2 um of the source STL placed by the URDF
  (MuJoCo stores vertices as float32).
- A mimic jaw follows `offset + multiplier x source` within 0.1 mm while driven across its stroke.
- The export never overwrites a file. Regenerating the committed export reproduces it. The export's sha256 and every mesh
  hash are in `minerva.manifest.json`.

## Change

`python -m robot_platform.models.mjcf urdf/minerva/model.json --output urdf/minerva/mujoco/minerva.xml
--limits config/devices/minerva-production-01-joint-limits.json --directions <Helm direction session>`
writes the MJCF and its manifest. It refuses to overwrite either; delete the old export to regenerate.

- Each link becomes a named body at its URDF link frame, and each joint becomes a hinge or slide at the
  body origin with the URDF axis and range. Compiler settings are explicit: radians, no inertia from geoms,
  no balancing, no mass or inertia floor, no static fusion. The timestep is 0.5 ms, as in `MujocoPlant`.
- Inertials are copied as declared, with the tensor rotated into the link frame. All 17 are placeholders,
  so the manifest says `dynamics_usable: false` and `MujocoPlant` still refuses the model.
- Each mimic becomes a joint equality with time constant 2 × timestep, the stiffest MuJoCo keeps stable.
  With MuJoCo's default (0.02 s) the synthetic jaw sagged 0.54 mm under its own weight; the stiff equality
  gives 2 µm.
- Meshes are referenced in place, named by their source path. The two RB04 motor-body visuals have about
  269,000 triangles, over MuJoCo's 200,000-face STL limit. They are stored as lossless OBJ (same triangles,
  exact duplicate vertices merged) under `urdf/minerva/mujoco/meshes/`, in LFS.
- Arm ranges come from Helm. Each arm joint's hand-swept motor range
  (`minerva-production-01-joint-limits.json`, reference `minerva-mechanical-reference-20261009`) goes
  through Helm's confirmed direction mapping, `model = sign × motor + offset`, as the receiver's `cad_map`
  applies it. Right J1, J3 and J4 have sign −1 and all offsets are 0. The swept ends are used exactly,
  without the receiver's 0.02 rad overtravel, which is a fault margin. The wrist swivels span about
  6.5 rad, past the URDF's ±π.
  - **Mapping source.** A complete Helm direction session (all 12 joints confirmed). It is local Helm state, not in
    Git; the manifest records what was used.
  - **Manifest.** It records the session's bindings and each joint's motor, sign, offset and swept values,
    so the export can be checked without the session.
  - **Jaws.** These keep the CAD stroke. Their swept motor travel through the unverified preview scale is
    48.0 mm, against 48.2 mm in CAD.
- Contacts are off (`contype`/`conaffinity` 0). Visuals are geom group 2 and source collision meshes group 3.
  No damping, friction, armature or actuator is added.
- `scene.xml` adds lights and a contact-free floor at the base. `scripts/view_mujoco.py` poses the model from
  URDF joint coordinates and renders a PNG, or opens MuJoCo's viewer. It never steps physics.

## Results

- 15 new tests, including the swept-range mapping (both signs), its refusals (unconfirmed direction, another
  mechanical zero or URDF revision) and the committed ranges against the committed limits file.
- MuJoCo's link frames match `CollisionScene.link_transforms` at all 202 Minerva poses. All 220
  meshes (129 visual, 91 collision) keep their triangle counts and placement. The synthetic fixture matches
  the URDF inertia tensor `R I Rᵀ` within 1e-12, and its coupling error is 2 µm (bound 0.1 mm).
- Mutation check: a rotating-axis rpy convention, a conjugated quaternion and a 1 mm mesh shift each fail
  the tests.
- The export loads in MuJoCo in about 2.5 s; the kinematic-only tree loads in 0.02 s.
- Rendered at the reference pose and a test pose, inspected by eye (not compared with Helm's view). The
  collision set lines up with the visuals. The interactive viewer opened and stayed up for an 8 s smoke test.

## Not known

- **Dynamics.** All 17 links need measured or corrected-CAD mass, COM and inertia, per
  [the checklist](../robots/minerva/MINERVA_PHYSICS_MEASUREMENTS.md). Until then the model is kinematic only.
- **Independent kinematics.** This proves the MJCF equals the URDF; it does not prove the URDF equals the
  robot. Measured end-effector positions at separated poses are still required.
- **Jaws.** The coupling is modeled rigid, with no backlash or compliance. The motor-to-jaw transmission is
  unverified: the 2026-09-24 right-gripper fit was rejected (1.2 mm residual, 1.1 mm direction separation),
  and the left scale is borrowed. Motor bindings for the grippers wait on it.
- **Joint ranges.** The arm ranges are hand-swept (2026-09-26; left J4/J5 shifted for the 2026-10-09 re-zero).
  A sweep can only fall short of a hard stop. MuJoCo will treat them as stops once dynamics are enabled.
  The direction session is local Helm state, so a new sweep or direction change needs a regenerated export.
- **Not run.** Contacts, gravity direction against the real mounting, and Linux CI.
- **Next.** Import a measured link record (the checklist's schema) into a new MJCF revision, with validation; this can be
  built and tested on synthetic records before measurements exist. The motor-to-model mapping is not identity (Helm's
  directions reverse right J1, J3 and J4, and the grippers need their transmission); the task scene applies it through its
  own `joint_map`.
