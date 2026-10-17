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
- **Fiducial tags.** Each tag's place and a printed label's scale (see "Fiducial tags").

A real head frame (the recording proof of 2026-10-17, `build/recording-readiness-20261016/`) shows two wood cubes on the robot's left and the box on its right, with the box's long side pointing away from the robot. The placeholders follow that arrangement, and the head camera's synthetic view is turned 180 degrees so the far side of the table is at the top, as in the real images. The sim's head camera still looks almost straight down from in front of the robot, while the real one looks forward and sees no robot parts. The wrist cameras also differ from the real views. Camera calibration is still needed.

## Use

- `helm --sim` takes `--sim-workcell <file|synthetic>`. Without it, Helm uses the asset's workcell file when one exists, else the synthetic layout.
- `python -m robot_platform.evaluation run|grid --workcell <file>` builds the scene from the file. The cube grid then covers the file's cube region, with trial 0 at its cube start.
- `MinervaTaskScene(..., workcell=...)` takes a `Workcell`, its description, `'synthetic'` or a path. The default is the synthetic layout, which keeps the frozen contact fixture and the harness's reference results valid. With the synthetic layout, the scene compiles to exactly the same geometry as before (all 103 geoms, cameras and the cube were checked array by array).

The bucket may be rectangular and rotated. The scene's in-bucket check uses the bucket's own axes. A cube that touches the table's trim counts as a table contact.

## Fiducial tags

The printed ArUco labels on the table (`claude/table-fiducial-labels`, TABLE_FIDUCIALS.md there) are in the workcell file's `fiducials` section, and the scene draws them on its table.

- **Frames.** The file gives tag centres in the table frame: origin at the near-left outer corner of the top (outer edge of the trim), seen from behind the robot facing the table; x right along the near edge, y away from the robot. With the table square to the robot, table (x, y) is base (c - x, -d - y) on the top, where (c, -d) is that corner, from `table.center` and `table.size` (c = 0.762 m, d = 0.080 m with today's placeholders). Every label lies flat with its text end toward the robot, so each tag's marker frame (OpenCV's: x right, y toward the top of the code, z out of the label) is the table frame: the base frame turned half a turn about z.
- **Values, all placeholders.** The family is what was printed, DICT_4X4_50. The black square (43.18 mm), the label (58.93 x 71.71 mm, the printed area) and the tag's place on it (centre 6.39 mm from the label's centre toward its far end) are the generator's design until a printed label is measured. The tape's unprinted white margins (a cut label is about 62 x 77 mm) are not drawn. Tags 0-5 sit at the layout given to the user on 2026-10-17. Tags 6 and 7 follow the bucket: centred across on its near-left and near-right outer corners, their centres 42 mm in front of its near outer face (the label's far end 1 cm from the face, the tag centre about 32 mm from that end).

| ID | Table frame (mm) | Base frame (m) |
|---|---|---|
| 0 | 100, 100 | 0.662, -0.180 |
| 1 | 1424, 100 | -0.662, -0.180 |
| 2 | 1143, 662 | -0.381, -0.742 |
| 3 | 381, 662 | 0.381, -0.742 |
| 4 | 762, 662 | 0.000, -0.742 |
| 5 | 762, 100 | 0.000, -0.180 |
| 6 | 932, 121.5 (from the bucket) | -0.170, -0.2015 |
| 7 | 1092, 121.5 (from the bucket) | -0.330, -0.2015 |

- **Drawing.** `simulation/fiducials.py` holds each ID's cells, copied from OpenCV 4.13. `scripts/sim/check_fiducial_codes.py` checks them against OpenCV, which the development environment does not include. Each tag is a body, `task/fiducial/<id>`, in its marker frame on the table top. It holds a white label box, the black square on it, and one box per run of white cells along a row, stacked 0.1 mm apart so the renderer draws each over the one below; that is a drawing order, not the paper's thickness. The label's text and scale are not drawn.
- **Visual only.** The tags have no contacts and no mass, and stay out of the penetration audit and the appearance variation. They are added after every other body, so no other body or geom changes its index.
- **Physics unchanged.** The nominal trial (handoff demonstrator through the receiver, driven grippers, this workcell, trial 0) is bit for bit the same with the tags as on `0a4d8ef` without them: the same qpos, qvel, time and surface velocities of all 187 other geoms after each of its 19,688 scene steps, and the same events (pickup 6.278 s, handoff 23.6625 s, bucket 49.216 s). Interleaved timing on the fixture path gave 975 us per physics step with the tags and 961 us without, within this laptop's load noise.
- **Real cameras.** `scripts/sim/fiducial_camera_check.py <frames dir>` finds the tags in frames as the cameras publish them and reads each code back. For the head frame it compares them with the scene's head camera and solves the real camera's pose from the tags; see its docstring. It needs OpenCV, which the development environment does not include.

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
- the same drop across the short side lands outside;
- the fiducial codes are OpenCV's (its DICT_4X4_50 bytes for IDs 0-7);
- each tag sits at its table-frame place mapped onto the base, and tags 6 and 7 follow the bucket;
- the compiled scene has every tag flat on the table in the table frame's orientation, with its code cell by cell and a white margin round the black square;
- the tags change no other body or geom, and a cube dropped onto tag 6 lands on the table exactly as without the tags;
- a tag off the table, with an unknown code or ID, or without a source is refused.
