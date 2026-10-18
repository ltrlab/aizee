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
- **Tag rectangle.** Where it is relative to the robot (see "Fiducial tags and the head camera"); the head camera follows it.

A real head frame (the recording proof of 2026-10-17, `build/recording-readiness-20261016/`) shows two wood cubes on the robot's left and the box on its right, with the box's long side pointing away from the robot. The placeholders follow that arrangement. The head camera is now calibrated against the fiducial tags (below); the wrist cameras are not, and still differ from the real views.

## Use

- `helm --sim` takes `--sim-workcell <file|synthetic>`. Without it, Helm uses the asset's workcell file when one exists, else the synthetic layout.
- `python -m robot_platform.evaluation run|grid --workcell <file>` builds the scene from the file. The cube grid then covers the file's cube region, with trial 0 at its cube start.
- `MinervaTaskScene(..., workcell=...)` takes a `Workcell`, its description, `'synthetic'` or a path. The default is the synthetic layout, which keeps the frozen contact fixture and the harness's reference results valid. With the synthetic layout, the scene compiles to exactly the same geometry as before (all 103 geoms, cameras and the cube were checked array by array).

The bucket may be rectangular and rotated. The scene's in-bucket check uses the bucket's own axes. A cube that touches the table's trim counts as a table contact.

## Fiducial tags and the head camera

The printed ArUco labels (`claude/table-fiducial-labels`, TABLE_FIDUCIALS.md there) are the real-to-sim calibration reference (the user, 2026-10-18: "now use that as the calibration reference for the real2sim"). They are in the workcell file's `fiducials` section, the scene draws them, and the scene's head camera is calibrated against them.

- **The tag rectangle.** The user taped tags 0-3 as a 650 x 441 mm rectangle over the outer corners of their black squares (0 near-left, 1 near-right, 2 far-right, 3 far-left). The head camera settles that the corners are the squares', not the labels': with the D435's intrinsics the squares' corners fit at 0.75 px RMS, the labels' at 3.59 px. Tags 4 and 5 lie at the middles of the near and far sides, where the head camera puts them (rays through their corners onto the table plane). The user confirmed the near-middle tag is ID 4.
- **Frames.** Tags 0-5 and the head camera are given in the rectangle's frame: origin at the outer corner of tag 0's square, x along the near side toward tag 1, y away from the robot, z up from the table top. One entry places the rectangle relative to the robot: `fiducials.rectangle.robot_offsets` (c, d) puts its origin at base (c, -d), with `yaw_deg` its turn from square. Both are placeholders (centred on the robot, the near tags 18 cm in front of the hanging arms). Measuring them moves the tags and the camera together; the head camera's mount relative to the robot then follows, and that becomes the lasting calibration. A tag may also be given in the table frame (origin at the table top's near-left outer corner; table (x, y) is base (c - x, -d - y)).
- **Labels.** Family DICT_4X4_50, black square 43.18 mm (the user's calipers, 2026-10-18; the 10 mm scale arms measure 10 mm). The label drawn is the printed area, 58.93 x 71.71 mm, the tag centre 6.39 mm from its centre toward its far end. The tape's unprinted white margins are not drawn. Every flat label lies with its text end toward the robot, so its marker frame (OpenCV's: x right, y toward the top of the code, z out of the label) is the rectangle's.
- **Tags on the box.** Tag 6 is upright on the front wall's outer face and tag 7 on the back wall's inner face, near the rim, both facing the robot. That is how the head camera decodes them (0 cells wrong); the user's description had them the other way round. They follow the box's pose (`wall`, `face`, `along`, `below_rim`); their height below the rim is a placeholder.

| ID | Rectangle frame (mm) | Base frame with the placeholder offsets (m) | From |
|---|---|---|---|
| 0 | 21.6, 21.6 | 0.303, -0.180 | tape |
| 1 | 628.4, 21.6 | -0.303, -0.180 | tape |
| 2 | 628.4, 419.4 | -0.303, -0.578 | tape |
| 3 | 21.6, 419.4 | 0.303, -0.578 | tape |
| 4 | 323.7, 22.4 | 0.001, -0.181 | head camera |
| 5 | 323.6, 416.8 | 0.001, -0.575 | head camera |
| 6 | front wall, outer face, middle | on the box placeholder | head camera decode |
| 7 | back wall, inner face, middle | on the box placeholder | head camera decode |

- **Drawing.** `simulation/fiducials.py` holds each ID's cells, copied from OpenCV 4.13; `scripts/sim/check_fiducial_codes.py` checks them against OpenCV, which the development environment does not include. Each tag is a body, `task/fiducial/<id>`, in its marker frame: flat on the table, or tipped upright onto its wall. It holds a white label box, the black square on it, and one box per run of white cells along a row, stacked 0.1 mm apart so the renderer draws each over the one below (a drawing order, not the paper's thickness). The label's text and scale are not drawn.
- **Visual only.** The tags have no contacts and no mass, and stay out of the penetration audit and the appearance variation. They are added after every other body, so no other body or geom changes its index. The nominal trial (handoff demonstrator through the receiver, driven grippers, this workcell, trial 0) is bit for bit the same as on `0a4d8ef` without the tags: the same qpos, qvel, time and surface velocities of all 187 other geoms after each of its 19,688 scene steps, and the same events (pickup 6.278 s, handoff 23.6625 s, bucket 49.216 s). Interleaved timing on the fixture path gave 975 us per physics step with the tags and 961 us without, within this laptop's load noise.

### The head camera

- **Intrinsics.** The D435 (serial 919122072056) colour stream at 640 x 480, read with `rs-enumerate-devices -c` on 2026-10-18: fx 614.336, fy 614.734, PPX 313.263, PPY 236.985, no distortion (Inverse Brown-Conrady, all coefficients 0). The head node publishes the image turned half a turn, so for the published image the principal point is (639 - PPX, 479 - PPY) = (325.737, 242.015), as the node's own published depth intrinsics are. The workcell keeps `resolution`, `focalpixel` and `principalpixel` and the fovy they give, 42.65 degrees. MuJoCo takes `principalpixel` as the image centre's offset from the principal point, ((W - 1) / 2 - cx, (H - 1) / 2 - cy), here (-6.237, -2.515) px; `workcell.head_pinhole` turns the camera back into (fx, fy, cx, cy).
- **The 180-degree turn.** The pose is solved on the published image, so the scene's camera is the sensor turned half a turn about its optical axis and renders the published orientation directly; nothing is rotated after rendering.
- **Pose.** Fitted to tags 0, 2, 3, 4 and 5 at their places in the rectangle on the head frame of 2026-10-18 03:25Z, with the intrinsics held fixed: 0.72 px RMS, at most 1.13 px. The camera sits at (275.2, -269.4, 544.3) mm in the rectangle frame: 27 cm in front of the near side, 54 cm above the table, looking 45.8 degrees down, turned 1.5 degrees to the robot's left, rolled -1.6 degrees.
- **In the scene.** The scene gives MuJoCo the camera's `resolution`, `focalpixel` and `principalpixel` with a 1 um sensor pixel (only the ratio to the focal length matters). MuJoCo renders the principal point at ((W - 1) / 2 - px, (H - 1) / 2 - py). Rendering spheres at known places (MuJoCo 3.15, EGL) showed it: each landed 2 px and 2 py from where the opposite sign puts it. Until this was fixed the file held the opposite sign, so the scene rendered with the unrotated sensor's principal point, (313.26, 236.99), 12.5 px left of and 5.0 px above the published image's. Projected as OpenGL's glFrustum projects the frustum `mjv_updateScene` gives the renderer (no GL needed), every tag corner lands within 2e-5 px of the D435's pinhole model. `tests/unit/test_workcell.py` holds it within 1e-3 px (MuJoCo keeps the sensor and focal lengths as float32) and checks the principal point against the D435's (639 - PPX, 479 - PPY).
- **Proof on the real frames.** `scripts/sim/fiducial_camera_check.py` projects the workcell's tags through the scene's head camera onto a head frame and draws them over the tags found (`head.sim-vs-real.png`). Per tag, RMS over its corners:
  - 03:25Z, the frame the camera was fitted on: tags 0, 2, 3, 4, 5 at 0.77, 0.75, 0.73, 0.57, 0.79 px.
  - 03:45Z, with the box placed: tags 0, 3, 4, 5 at 3.6, 3.8, 1.4, 3.3 px. Refitted on that frame the camera differs from the workcell's by 2 mm and 0.48 degrees (0.81 px RMS on tags 0, 3, 4, 5): the table, or the robot, probably turned about half a degree when the box went down. Tags 6 and 7 miss by 168 and 39 px, because the box keeps its placeholder pose (below).

### Follow-up task: the real box and the robot-to-rectangle placement

Not done on this branch. Once the user's robot-to-rectangle numbers arrive (each gripper to the near side, the left gripper's sideways offset from tag 0's corner, and the fingertip-to-table height), a separate task should:
- set `fiducials.rectangle.robot_offsets` and `yaw_deg` from them, which moves the tags and the head camera onto the robot;
- apply the box where tags 6 and 7 put it (head camera, D435 intrinsics, frame 20261018T034533Z): turned -18.6 degrees from square to the rectangle, its front toward the robot's left; its middle at (471.8, 226.9) mm in the rectangle frame; along its axis the front wall's outer face 235.8 mm and the back wall's inner face 491.3 mm from the rectangle's origin, so about 251 mm inside front to back with a 4 mm wall, against the placeholder 305 mm; tag centres 147-148 mm up, so a rim near 180 mm (placeholder 150 mm). The user's tape numbers for the box replace these;
- re-plan the demonstrator's reach map, cube region and nominal cube start for that layout. With the box at the camera's pose (and the placeholder offsets) the demonstrator finds no plan for the cube at today's placeholder start, so this branch keeps the box's placeholder pose;
- refit the head camera on a fresh frame if the table stays where the box put it (the 0.48-degree shift above).

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
- tags 0-3 sit half a square in from the taped rectangle's corners, 4 and 5 at its sides' middles, all in the rectangle's orientation, and the head camera moves with the rectangle;
- tags 6 and 7 stand upright on the box's front-outer and back-inner faces facing the robot, and follow the box when it moves or turns;
- the compiled scene has every tag where its pose puts it, with its code cell by cell and a white margin round the black square;
- the scene's head camera, as MuJoCo renders it, projects every tag corner within 1e-3 px of the D435's pinhole model;
- the tags change no other body or geom, and a cube dropped onto tag 4 lands on the table exactly as without the tags;
- a tag off the table or off its wall, in an unknown frame or face, with an unknown code or ID, or without a source is refused.
