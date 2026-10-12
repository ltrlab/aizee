# Feature task: Minerva MJCF, kinematics first

- Branch and worktree: `claude/minerva-physical-model`, `C:/Users/ltr/Workspace/aizee-minerva-model`
- Base commit (full SHA): `f846b65a945f612a94f60081d1905caa4b32029e` (`codex/overhaul-integration`)
- Owner: Claude Code, at the operator's request ("continue work on the MuJoCo simulation")
- Problem / saved reproducer: `MujocoPlant` runs fixtures only. Minerva's URDF cannot drive it: its 17 masses
  are exporter placeholders (`1e-9 kg`), its two jaw mimics have no MJCF coupling, and no Minerva MJCF
  exists. Measurements are still unavailable (confirmed 2026-10-10), so dynamics stay blocked. The
  [lane brief](NEXT_TASKS.md#minerva-physical-model) allows kinematic frame comparisons before them.
- Hypothesis: the active URDF converts to an MJCF that keeps every link frame, joint axis and range, mesh
  placement, inertial and jaw coupling. That MJCF is then the base the measured inertials replace.
- Owned files: `src/robot_platform/models/mjcf.py`, `scripts/view_mujoco.py`, `urdf/minerva/mujoco/`,
  `tests/unit/test_mjcf_export.py`, `tests/fixtures/physics/coupled_jaws.urdf`, this brief.
- Shared interfaces held fixed: `MujocoPlant` and its placeholder-mass refusal, `PlantReceiver`, the URDF,
  its meshes and model profile, `CollisionScene` kinematics. No receiver, teleop or collision code changed.
- Dependencies / integration order: none; touches no file another open branch owns. `scripts/dev.py` gains
  one gate entry, and `.gitattributes` gains LFS/line-ending rules for the export.
- Required behavior and numerical/time bounds (declared before running):
  - Link frames equal the URDF's within 1e-9 m and 1e-9 in rotation, at 200 seeded poses plus every joint
    at both ends of its range.
  - Every mesh keeps its triangle count. Its world bounds and centroid sit within 2 µm of the source STL
    placed by the URDF. MuJoCo stores vertices as float32.
  - A mimic jaw follows `offset + multiplier × source` within 0.1 mm while driven across its stroke.
  - The export never overwrites a file. Regenerating the committed export reproduces it.
- Acceptance command and expected result: `python scripts/dev.py check`, zero failures/skips/xfails.
- Source/model identity and seed: URDF sha256 `792f32da…ddcf8`; MuJoCo 3.15.0; pose seeds 20261012
  (frames) and 7 (meshes). The export's own sha256 and every mesh hash are in `minerva.manifest.json`.

## Change

`python -m robot_platform.models.mjcf urdf/minerva/model.json --output urdf/minerva/mujoco/minerva.xml`
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
- Contacts are off (`contype`/`conaffinity` 0). Visuals are geom group 2 and source collision meshes group 3.
  No damping, friction, armature or actuator is added.
- `scene.xml` adds lights and a contact-free floor at the base. `scripts/view_mujoco.py` poses the model from
  URDF joint coordinates and renders a PNG, or opens MuJoCo's viewer. It never steps physics.

## Results

- 9 new tests. MuJoCo's link frames match `CollisionScene.link_transforms` at all 202 Minerva poses. All 220
  meshes (129 visual, 91 collision) keep their triangle counts and placement. The synthetic fixture matches
  the URDF inertia tensor `R I Rᵀ` within 1e-12, and its coupling error is 2 µm (bound 0.1 mm).
- Mutation check: a rotating-axis rpy convention, a conjugated quaternion and a 1 mm mesh shift each fail
  the tests.
- The export loads in MuJoCo in about 2.5 s; the kinematic-only tree loads in 0.02 s.
- Rendered at the reference pose and a test pose, inspected by eye (not compared with Helm's view). The
  collision set lines up with the visuals. The interactive viewer opened and stayed up for an 8 s smoke test.
- Gate: 656 Python tests, zero failures/skips/xfails, and 122 Rust tests, on `a6939f2`
  (`build/dev-check/20261012T161039Z-538a042c/report.json`). The first run skipped the release test, because
  this new worktree had no ARM64 binaries. `scripts/build_jetson.ps1 -Offline -Package robot_control,robot_collision`
  built them locally; nothing was deployed.

## Remaining uncertainty / physical inputs needed

- **Dynamics.** All 17 links need measured or corrected-CAD mass, COM and inertia, per
  [the checklist](../MINERVA_PHYSICS_MEASUREMENTS.md). Until then the model is kinematic only.
- **Independent kinematics.** This proves the MJCF equals the URDF; it does not prove the URDF equals the
  robot. Measured end-effector positions at separated poses are still required.
- **Jaws.** The coupling is modeled rigid, with no backlash or compliance. The motor-to-jaw transmission is
  unverified: the 2026-09-24 right-gripper fit was rejected (1.2 mm residual, 1.1 mm direction separation),
  and the left scale is borrowed. Motor bindings for the grippers wait on it.
- **Joint ranges.** These are the URDF's limits (±π, provisional for all 12 arm joints), not measured hard
  stops. MuJoCo would treat them as stops once dynamics are enabled.
- **Not run.** Contacts, gravity direction against the real mounting, and Linux CI.
- **Next.** Map the 14 receiver motors to model joints (arms are identity, as the viewer and joint
  calibration assume). Also import a measured link record (the checklist's schema) into a new MJCF revision,
  with validation. Both can be built and tested on synthetic records before measurements exist.
