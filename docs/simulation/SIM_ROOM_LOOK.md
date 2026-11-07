# A dark grey robot in a house room

**What it is.** How the simulated scene looks to its cameras: a dark grey robot, a blurred house room as the backdrop, a
wood floor and warm lights. It changes pixels only, never physics. **Status: done and merged.** The look is described
under "Room and look" in [MINERVA_TASK_SCENE.md](MINERVA_TASK_SCENE.md) and varies per layout in
[SIM_GENERATION.md](SIM_GENERATION.md); this page keeps the reasons and the bounds.

## Why

Drawn pitch black (the real robot is black), the grippers were darker than the black table: in renders of the nominal
trial (`scripts/sim/render_look.py`) their mean sRGB luma was 7-12 against 10-21 for the table top, in the head and wrist
views. Beyond the table the scene was black, and the floor was a slate-blue plane. A dark grey robot with a little
specular, a blurred room, a wood floor and warm lights of the same brightness make the grippers stand out and keep the
workcell's exposure.

## Code

- `src/robot_platform/simulation/room.py`: the look, the painting and `python -m robot_platform.simulation.room`. The six
  512 px backdrop faces and the 1024 px floor wood are in `src/robot_platform/simulation/room/*.png` (about 0.4 MB).
- `simulation/task_scene.py`: the robot finish (`ROBOT_*`, `robot_finish`, `scene_look`), the room's assets, the floor, the
  warm lights, the haze, `RENDER_FLAGS`, and the capabilities' `look` and `room_backdrop`.
- `simulation/workcell.py` (`look`, `LOOK_KEYS`) and `simulation/layouts.py` (`look`, `LOOK_RANGES`): a workcell may give a
  look, and a randomization file may draw one per layout; the episode records it.
- Tests: `tests/unit/test_room.py`, `test_robot_finish.py`, and additions to `test_workcell.py`, `test_layouts.py` and
  `tests/contract/test_generate_layouts.py`.

## Bounds the tests hold

- Every robot geom in both visual modes is the look's grey (0.22; the tests allow 0.15-0.3), through a material with
  specular 0.3, shininess 0.25 and reflectance 0.
- Rendered where each arm closes on the block, its gripper is brighter than the table top by more than 25 levels of luma,
  in the head view and in its wrist view.
- The backdrop files are exactly what the seeded painting makes (seed 20261018, 11 px blur); their sha256 is in the
  capabilities' `room_backdrop`. MuJoCo shows each face where the painting meant it.
- The lights have luma 1, so a warmer light is no brighter. A look changes no physics array and no contact geometry hash.
- The nominal trial is bit for bit the same: every step's time, qpos, qvel and surface-velocity digest, and the events.

## Not known

- The look is a design choice, not measured. The real room behind the table is dimmer and cluttered, and the real robot
  is black.
- The backdrop is a skybox with no parallax, so a moving wrist camera sees it as it would a sky. The floor plane is 4 x
  4 m, and its far edge is faded into the backdrop by MuJoCo's horizon haze, not fogged.
- The episode appearance variation (lights' strength, workcell colours, wrist cameras) still runs on top, and its colour
  tint now also falls on the wood floor.
- The table keeps MuJoCo's default specular, so at grazing angles a wrist camera sees its glare.
