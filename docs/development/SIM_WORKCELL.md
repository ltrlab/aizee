# Simulated workcell

The task scene ([MINERVA_TASK_SCENE.md](MINERVA_TASK_SCENE.md)) was built on a synthetic layout: a 0.28 m table chosen so a top grasp reaches it, and a square blue bucket. A workcell file now describes the real bench instead, and `helm --sim` and the evaluation harness use it.

Code: `src/robot_platform/simulation/workcell.py`. The file for this robot is `config/devices/simulation/minerva-production-01-workcell.json`. It sits beside the actuator file in `simulation/`, which robot releases leave out.

## What the file holds

Every value is `{"value": ..., "source": "..."}`. A value nobody measured has a source starting with `placeholder`. The scene still uses it, and it is listed in the scene's capabilities (`workcell_placeholders`), its limitations, and every recorded episode's metadata.

Measured by the user on 2026-10-16:

| Value | Measurement |
|---|---|
| Table size | about 5 ft by 2.5 ft (1.524 by 0.762 m) |
| Table top height | about 6 in below the hanging arms' J3. The model's J3 anchor is at z 0.664 m with the arms hanging, so the top is at z 0.512 m |
| Bucket | a cardboard box about 6 in by 12 in (0.152 by 0.305 m), taken as the inside |
| Cube | wood, about 1.5 in (38.1 mm) on a side |
| Appearance | black table with silver trim, cardboard box, wood cubes |

Not measured yet. These are placeholders and should be measured:
- **Table position.** Where its near edge is relative to the hanging arms, and whether the robot is centred on the long side.
- **Bucket.** Height, position and orientation, plus wall and bottom thickness.
- **Table details.** Thickness, the floor height and the trim width.
- **Cube.** Mass (weigh one) and friction against the finger pads.
- **Cube positions.** The region trials place the cube in.
- **Head camera.** Pose and field of view.

A real head frame (the recording proof of 2026-10-17, `build/recording-readiness-20261016/`) shows two wood cubes on the robot's left and the box on its right, with the box's long side pointing away from the robot. The placeholders follow that arrangement, and the head camera's synthetic view is turned 180 degrees so the far side of the table is at the top, as in the real images. The sim's head camera still looks almost straight down from in front of the robot, while the real one looks forward and sees no robot parts. The wrist cameras also differ from the real views. Camera calibration is still needed.

## Use

- `helm --sim` takes `--sim-workcell <file|synthetic>`. Without it, Helm uses the asset's workcell file when one exists, else the synthetic layout.
- `python -m robot_platform.evaluation run|grid --workcell <file>` builds the scene from the file. The cube grid then covers the file's cube region, with trial 0 at its cube start.
- `MinervaTaskScene(..., workcell=...)` takes a `Workcell`, its description, `'synthetic'` or a path. The default is the synthetic layout, which keeps the frozen contact fixture and the harness's reference results valid. With the synthetic layout, the scene compiles to exactly the same geometry as before (all 103 geoms, cameras and the cube were checked array by array).

The bucket may be rectangular and rotated. The scene's in-bucket check uses the bucket's own axes. A cube that touches the table's trim counts as a table contact.

## Reach on the real table

The real table is 0.23 m higher than the synthetic one. This changes how the cube can be grasped.

`scripts/sim/workcell_reach.py` probes three points per arm in the cube region (outside the gate):

- **Straight down: refused.** A grasp with the pads on the cube and the fingers pointing straight down was out of reach at every point for both arms. Every best attempt ended against joint limits:
  - right J2, J3 and J4;
  - left J2, J3, J4 and J6.

  An earlier run with 40 random starting poses per point agreed.
- **Tilted 65-80 degrees: mostly reached.** With the fingers 65-80 degrees from vertical, leaning away from the shoulder, the grasp was reached at 5 of 6 points with 8 starting poses each.
  - The sixth point, right (-0.10, -0.25), was reached in an earlier run with 12 starting poses.
  - Tilts of 20-50 degrees were not reached at any point.
- **Consequence.** On the real bench the cube is taken from the side with the gripper nearly level. The synthetic scene's 0.28 m table was chosen for the same reason (MINERVA_TASK_SCENE.md: a 0.50 m table was not reachable with downward fingers).

These are kinematic checks within the receiver's joint limits. They do not check penetration, approach paths or the handoff.

## Tests

`tests/unit/test_workcell.py` (in the gate):
- the synthetic constants are unchanged;
- every measured value has a source, and the placeholders are listed;
- a value without a source is refused;
- the grid and the scene follow the workcell, and the cube rests on the measured table;
- a cube dropped into the far end of the 6 x 12 in box (unrotated and turned 30 degrees) settles in it;
- the same drop across the short side lands outside.
