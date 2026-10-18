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
- **Bucket.** Wall and bottom thickness. Its size, place, turn and rim height are estimated from the head camera's depth and the tags ("The head camera's depth"), not taped.
- **Table details.** Thickness, the floor height and the trim width.
- **Cube.** Mass (weigh one), friction against the finger pads, and its size: the head camera suggests the two blocks are not 38.1 mm cubes ("The head camera's depth").
- **Cube positions.** The region trials place the cube in.
- **Tag rectangle.** Where it is relative to the robot (see "Fiducial tags and the head camera"); the head camera follows it.

A real head frame (the recording proof of 2026-10-17, `build/recording-readiness-20261016/`) shows two wood cubes on the robot's left and the box on its right, with the box's long side pointing away from the robot. The placeholders follow that arrangement. The head camera is now placed from its own depth and the fiducial tags (below); the wrist cameras are not, and still differ from the real views.

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
- **Tags on the box.** Tag 6 is upright on the front wall's outer face and tag 7 on the back wall's inner face, near the rim, both facing the robot. That is how the head camera decodes them (0 cells wrong); the user's description had them the other way round. They follow the box's pose (`wall`, `face`, `along`, `below_rim`); their centres are 42 mm below the rim (the head camera's depth and the tags' own squares).

| ID | Rectangle frame (mm) | Base frame with the placeholder offsets (m) | From |
|---|---|---|---|
| 0 | 21.6, 21.6 | 0.303, -0.180 | tape |
| 1 | 628.4, 21.6 | -0.303, -0.180 | tape |
| 2 | 628.4, 419.4 | -0.303, -0.578 | tape |
| 3 | 21.6, 419.4 | 0.303, -0.578 | tape |
| 4 | 323.7, 22.4 | 0.001, -0.181 | head camera |
| 5 | 323.6, 416.8 | 0.001, -0.575 | head camera |
| 6 | front wall, outer face, middle | on the box | head camera decode |
| 7 | back wall, inner face, middle | on the box | head camera decode |

- **Drawing.** `simulation/fiducials.py` holds each ID's cells, copied from OpenCV 4.13; `scripts/sim/check_fiducial_codes.py` checks them against OpenCV, which the development environment does not include. Each tag is a body, `task/fiducial/<id>`, in its marker frame: flat on the table, or tipped upright onto its wall. It holds a white label box, the black square on it, and one box per run of white cells along a row, stacked 0.1 mm apart so the renderer draws each over the one below (a drawing order, not the paper's thickness). The label's text and scale are not drawn.
- **Visual only.** The tags have no contacts and no mass, and stay out of the penetration audit and the appearance variation. They are added after every other body, so no other body or geom changes its index. The nominal trial (handoff demonstrator through the receiver, driven grippers, this workcell, trial 0) is bit for bit the same as on `0a4d8ef` without the tags: the same qpos, qvel, time and surface velocities of all 187 other geoms after each of its 19,688 scene steps, and the same events (pickup 6.278 s, handoff 23.6625 s, bucket 49.216 s). Interleaved timing on the fixture path gave 975 us per physics step with the tags and 961 us without, within this laptop's load noise.

### The head camera

- **Intrinsics.** The D435 (serial 919122072056) colour stream at 640 x 480, read with `rs-enumerate-devices -c` on 2026-10-18: fx 614.336, fy 614.734, PPX 313.263, PPY 236.985, no distortion (Inverse Brown-Conrady, all coefficients 0). The head node publishes the image turned half a turn, so for the published image the principal point is (639 - PPX, 479 - PPY) = (325.737, 242.015), as the node's own published depth intrinsics are. The workcell keeps `resolution`, `focalpixel` and `principalpixel` and the fovy they give, 42.65 degrees. MuJoCo takes `principalpixel` as the image centre's offset from the principal point, ((W - 1) / 2 - cx, (H - 1) / 2 - cy), here (-6.237, -2.515) px; `workcell.head_pinhole` turns the camera back into (fx, fy, cx, cy).
- **The 180-degree turn.** The pose is solved on the published image, so the scene's camera is the sensor turned half a turn about its optical axis and renders the published orientation directly; nothing is rotated after rendering.
- **Pose.** The camera is on a temporary mount (the user, 2026-10-18: not calibrated there), so its pose comes from one frame, depth-20261018T044530Z (04:45Z). The table plane fitted to that frame's depth sets the camera's height, 554.9 mm, and its tilt, 45.59 degrees down. The flat tags 0, 3, 4 and 5 at their taped places set only the three unknowns left: where it stands over the plane and which way it faces. It sits at (275.2, -276.7, 554.9) mm in the rectangle frame, turned 1.0 degree to the robot's left and rolled -1.6 degrees. The tags miss by 3.4 px RMS, at most 4.2 px. Leaving one tag out at a time moves it at most 4 mm over the table and 0.25 degrees in turn.
- **The tags alone** (all six unknowns, 0.78 px RMS) put the camera 10.2 mm lower, at 544.7 mm, and 6.9 mm nearer the table. The two disagree because the taped rectangle reads about 1.9% short of the depth. The tags' calipered squares alone (no tape) give 559.6 mm, 0.9% from the depth, so the depth is the one to trust. From `6174681` to `f850ee2` the workcell held the tags-alone pose of frame 03:25Z.
- **In the scene.** The scene gives MuJoCo the camera's `resolution`, `focalpixel` and `principalpixel` with a 1 um sensor pixel (only the ratio to the focal length matters). MuJoCo renders the principal point at ((W - 1) / 2 - px, (H - 1) / 2 - py). Rendering spheres at known places (MuJoCo 3.15, EGL) showed it: each landed 2 px and 2 py from where the opposite sign puts it. Until this was fixed the file held the opposite sign, so the scene rendered with the unrotated sensor's principal point, (313.26, 236.99), 12.5 px left of and 5.0 px above the published image's. Projected as OpenGL's glFrustum projects the frustum `mjv_updateScene` gives the renderer (no GL needed), every tag corner lands within 2e-5 px of the D435's pinhole model. `tests/unit/test_workcell.py` holds it within 1e-3 px (MuJoCo keeps the sensor and focal lengths as float32) and checks the principal point against the D435's (639 - PPX, 479 - PPY).
- **Proof on the real frames.** `scripts/sim/fiducial_camera_check.py` projects the workcell's tags through the scene's head camera onto a head frame and draws them over the tags found (`head.sim-vs-real.png`). For the tags-alone pose of 03:25Z, per tag, RMS over its corners:
  - 03:25Z, the frame it was fitted on: tags 0, 2, 3, 4, 5 at 0.77, 0.75, 0.73, 0.57, 0.79 px.
  - 03:45Z, with the box placed: tags 0, 3, 4, 5 at 3.6, 3.8, 1.4, 3.3 px. Refitted on that frame the camera differs by 2 mm and 0.48 degrees (0.81 px RMS on tags 0, 3, 4, 5): the table, or the robot, probably turned about half a degree when the box went down.
- **Proof in the render.** The scene rendered through its head camera (MuJoCo 3.15, EGL, robot hidden; `scripts/sim/render_head_depth.py`), OpenCV finds tags 0, 3, 4, 5, 6 and 7 in the render within 0.55 px RMS of where `head_pinhole` projects them, so the render is the pinhole model. Against the real frame of 04:45Z, the rendered flat tags miss by 3.5 px RMS (the tape's 1.9%), tag 6 by 3.8 px and tag 7 by 7.9 px.
- **The robot in the view.** With the placeholder `robot_offsets`, the camera sits at base (0.050, 0.118, 1.067) m, and the robot model's own links fill most of its view (`render_fiducial_views.py`'s head.png). The real camera sees no part of the robot. Until the robot is registered to the camera (below), render the head view with the robot hidden, as `render_head_depth.py` does.

## The head camera's depth

The head D435's depth, read on 2026-10-18 at 04:45Z (frame `depth-20261018T044530Z`), measures the table, the box and the blocks. It also places the head camera (above).

- **Grab.** `scripts/sim/grab_head_depth.py` runs on the Jetson through the Minerva terminal tools' `robot_remote.py`, read-only, beside the camera service. It takes the per-pixel median of 20 depth frames as the head node publishes them, with one colour frame. 88% of the pixels have a value; the median frame-to-frame spread is 2.0 mm. `scripts/sim/unpack_head_depth.py` unpacks the result.
- **Into the colour camera.** `scripts/sim/head_depth_check.py` takes each depth pixel into the depth sensor's frame (the node's published depth intrinsics), then into the colour sensor's by the D435's own Depth-to-Color extrinsics (`rs-enumerate-devices -c`: 14.85 mm along x), then turns it half a turn into the published colour frame. It writes `registered_depth.npy` on the colour pixel grid.
- **Table.** The plane fits 67,351 points at 1.63 mm RMS. It sets the camera's height and tilt. It lies 0.10 degrees from the tags' plane and 10-11 mm below it at the four taped corners: the tape's 1.9% (above). The table top is seen out to y = 488 mm in the rectangle frame. Its near and side edges are out of view. A shelf about 150 mm up overhangs it from about 30 mm inside the rectangle's far side, so measurements stop 35 mm short of that side.
- **Rectangle frame.** From the table on, every measurement is in the rectangle frame seated on the depth's plane (the depth-anchored frame): the plane is its z = 0, and the tags set only its place and turn on the plane.
- **Box.** Its walls, seen from above, are straight runs of steady depth. The front wall is seen from outside, the back wall from inside. A plane through each face gives its lean and where it stands at each height:
  - turned -19.4 degrees from square to the rectangle (tags 6 and 7 alone: -18.6);
  - middle at (479.8, 227.2) mm, midway between the walls at mid-height. Leaving one tag out moves it up to 2.5 mm;
  - outer width 162.9 mm: the front face's extent, all of it in view;
  - the front wall's outer face leans out 6.8 degrees and the back wall's inner face 0.7 degrees. They stand 236.6 mm apart at the table, 246.2 mm at mid-height and 252.3 mm at the tags' height (tags 6 and 7: 255.5 mm);
  - rim 187 mm (95% of its points) to 191 mm (99%) above the table;
  - wall thickness not resolved. The back face's inner extent, 157.9 mm, against the front's outer 162.9 mm suggests about 2.5 mm, but the faces are only good to about 2 mm.

  The workcell takes the inside as 155 x 242 mm (4 mm placeholder walls), the rim as 189 mm and the place and turn above. Its walls are upright, so the sim's front wall sits about 11 mm out at its foot and 12 mm in at its rim.
- **Blocks.** Two blocks stand in the rectangle. Their tops are 39.0 and 37.0 mm above the table (the densest 2 mm of their heights). Each is fitted as the workcell's 38.1 mm cube to the steady depth within 60 mm of it: every point's depth against where its own ray meets the cube or the table, errors capped at 15 mm so the outline's mixed pixels do not pull.
  - The near block's centre is at (152.8, 124.3) mm, turned -37.0 degrees, 4.6 mm RMS.
  - The far block's is at (154.0, 234.2) mm, turned -22.0 degrees, 5.2 mm RMS.
  - The top face's principal axes do not settle a square's turn: they gave +38 degrees for the near block.
- **The blocks' size is not settled.** In the colour image, at those places, the real blocks are wider than a 38.1 mm cube: 64 x 66 px against 55 x 61 px for the near block. A cuboid fitted to each block's colour outline (the hull of its colour mask, eroded a pixel for JPEG colour bleed) fits with an outline IoU of 0.92-0.93, against 0.70-0.78 for a 38.1 mm cube:
  - the near block about 37 x 52 mm with its height held at the depth's 39 mm;
  - the far block about 47 x 48 mm at 37 mm.

  These fits put the centres 7-9 mm farther from the camera than the cube fits do. A free size cannot be fitted to the depth: from one view a block's far side is hidden, and the size drifts into it. The workcell keeps the user's 38.1 mm until someone measures the blocks.

### The sim against the real depth

`scripts/sim/render_head_depth.py` renders the scene's head camera with the robot hidden and the cube at each block's fitted place. `scripts/sim/compare_head_depth.py` classes the real registered depth into table, box and blocks and reports sim minus real depth along the optical axis. It reports each class over all its pixels and over its interior, more than 3 px from a depth step of 15 mm in either image. A pixel or two of misplaced outline swings a difference by the object's whole height, so the interior is the surface measure. For the box and each block it also compares the outlines (IoU and centroid shift of the sim's pixels of it against the real ones).

Rendered on coolserver GPU 0 (EGL), sim minus real, mm:

| Class | Pixels (interior) | Median | RMS | 90% of abs | Interior median | Interior RMS |
|---|---|---|---|---|---|---|
| Table | 35,869 (35,114) | -0.1 | 2.7 | 4.3 | -0.1 | 2.6 |
| Box | 17,118 (12,677) | 1.1 | 62.9 | 20.8 | 0.3 | 12.1 |
| Blocks | 1,606 (999) | 2.2 | 20.4 | 42.8 | 1.0 | 13.3 |

- **Outlines.** The box's outline IoU is 0.92, its centroid within 0.6 and 3.2 px. The blocks' IoUs are 0.72 each, their centroids within 2.6, 3.8 px and 1.9, 1.7 px. The sim's blocks cover about 72% of the real ones' pixels, as their size above suggests.
- **What is left.** The box's interior differences follow the front wall's lean: sim nearer at its foot, farther near its rim. The table's 2.6 mm RMS is the D435's own noise and the plane fit.
- **The tags-alone camera.** With the tags-alone pose of 03:25Z (and its box), the table is 14.4 mm too near (15.6 mm RMS), the box's interior 20.9 mm RMS.
- **Before the principal point fix.** The scene rendered the principal point 12.5 px left and 5.0 px up, and the table was 18.8 mm too near (26.5 mm RMS).

The images are in `build/fiducials-check/depth-compare/e73710c/`: `out-depth/depth_compare_depth-anchored.png` (real, sim, difference), `out-tags/depth_compare_tags-alone.png`, `blocks_zoom.png` (the sim's block outlines over the real frame) and `out-views/` (the scene from above and through the head camera with the robot).

### Repeating it after the camera moves

The camera's mount is temporary, so the alignment is one frame's. After the camera moves:
1. Grab: `grab_head_depth.py` through the terminal tools' `robot_remote.py` (read-only), then `unpack_head_depth.py <result.json> <dir>`.
2. Measure: `python scripts/sim/head_depth_check.py <dir>`. Its `proposed_workcell_head_camera` is the new pose; the box and blocks are in its `objects`.
3. Put the pose and the box into the workcell file with the frame ID in their sources.
4. Check: render with `render_head_depth.py <out> --blocks-from <dir>/depth_check.json` on a GPU (EGL), then `compare_head_depth.py <dir> <out>`. The table's median should be within 1 mm.

### Follow-up: the robot's place, the demonstrator and the blocks

Not done on this branch:
- **The robot's place.** Set `fiducials.rectangle.robot_offsets` and `yaw_deg` (still placeholders), which moves the tags, the box and the head camera onto the robot. Either from the user's tape numbers (each gripper to the near side, the left gripper's sideways offset from tag 0's corner, the fingertip-to-table height) or by registering the robot to the head camera (below).
- **The demonstrator.** The box is now where the depth puts it, and the handoff demonstrator finds no plan for the cube at the placeholder start ("no plan reaches the cube at [0.15, -0.35, 0.531]"). `tests/unit/test_demonstrator.py` and `tests/contract/test_demonstrator.py` fail until its reach map, cube region and nominal start are re-planned for this layout, best after the robot's place is measured. Generation (`scripts/sim/generate_parallel.py`), the evaluation harness and `helm --sim` default to this workcell file too.
- **The blocks.** Measure them (above). The scene has one cube size.
- **The box's lean.** The scene's walls are upright; the real front wall leans out 6.8 degrees.

### Plan: registering the robot to the head camera from depth

Not done. The aim is `robot_offsets` and `yaw_deg` without a tape: the head camera's pose in the robot's base frame, from the arms seen in its depth.
1. **Poses.** The user holds 4-6 still poses, under the usual idle-first procedure, that bring the forearms, wrists and grippers into the head view over the table. Spread them left and right, near and far, clear of the box. For each, record the joint angles from the guarded loop's state and a head depth grab (median of 20 frames). Both are read-only.
2. **Model.** Pose the URDF's link meshes by forward kinematics at those angles. Render the scene's head depth with the robot shown at the current estimate of its place; the render takes care of what hides what.
3. **Data.** Take the depth points more than 15 mm above the depth-fitted table, outside the box's and blocks' footprints and inside the view: the arms.
4. **Solve.** Find one rigid transform for all poses together: the rectangle frame in the base frame (c, d and yaw, plus the table's height and tilt if they are freed). Use point-to-plane ICP between the real points and the rendered surfaces, from the placeholder, with a robust loss; after convergence drop points more than 2 cm off.
5. **Check.** Per-pose residuals: expect 2-4 mm RMS at 0.6-0.9 m. The leave-one-pose-out spread of c, d and yaw. Each gripper's projected outline against the colour image. The table's height against the user's 6 in below J3.
6. **Write.** Put `robot_offsets` and `yaw_deg` into the workcell file with the frame IDs and residuals as their sources. The camera then follows, and its mount relative to the head comes with it.

Risks:
- Joint zero errors bias the forward kinematics. Mitigate with several poses, or by solving a per-joint offset as an extra unknown.
- The D435 fattens thin links' outlines. Weight broad link faces.
- The black robot's dark parts may return little depth. Check the fill rate on the first grab.

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
- the scene's head camera, as MuJoCo renders it (OpenGL's projection of `mjv_updateScene`'s frustum), projects every tag corner within 1e-3 px of the D435's pinhole model, its principal point at the D435's (639 - PPX, 479 - PPY);
- the tags change no other body or geom, and a cube dropped onto tag 4 lands on the table exactly as without the tags;
- a tag off the table or off its wall, in an unknown frame or face, with an unknown code or ID, or without a source is refused.
