# The headset's 3D view: plan

The operator teleoperates Minerva's arms through the GELLO leaders, with a Meta Quest Pro showing what the robot
sees. The 2D picture gave little sense of depth (the user, 2026-10-31), so the headset now shows the head camera's
depth in 3D (phase 1, deployed `adc2484`). This plan takes it further, in the order the pieces depend on each other.
The user asked for all of it on 2026-10-31: world-locked depth with smarter hole filling, the wrist cameras floating
above the tracked hands (as GHOST floats Spot's gripper camera above the controllers), Minerva's 3D model aligned
with the camera views, and Gaussian splatting, first on the operator station soliloquy (RTX 3080), later on the Quest
Pro alone.

Reference: GHOST (Brown/MIT, [arXiv 2608.29080](https://arxiv.org/html/2608.29080v1), Spot VR teleoperation):
completes each RGB-D frame with PromptDA on the VR client's GPU (131 ms), renders splats, floats the gripper
camera's picture above the controllers, and draws the robots' models in the same space.

## Frames

| Frame | What | Where it comes from |
|---|---|---|
| Robot | Minerva's base: front −y, left +x, up +z | the display model (`urdf/minerva/onshape-to-robot-head-v1`, `front_direction`) |
| Head link | `awg_rgb_camera_5`, after `neck_z` (pan) and `neck_y` (tilt) | the URDF joints |
| Colour camera | optical (x right, y down, z ahead) at the `AWG_RGB_CAMERA` centre, (−109.0, −82.1, −50.8) mm in the head link, looking along the robot's front at zero | the head model README, `front_direction` |
| Depth sensor | optical, where the camera's registration puts it (`p_color = R p_depth + t`) | the camera service (`6aeeb56`) |
| VR world | WebXR `local` space | the headset |

**The zero mapping** `M` (robot → VR world): when the operator zeroes the head (the trigger, or starting head
tracking), the headset records its eye pose `E0` (position, yaw and pitch, no roll). The head's zero is the camera
at the reference pose, `T_robot_color0`. Then `M = E0 · F · T_robot_color0⁻¹`, with `F` the flip between the optical
axes and three.js's (x right, y up, −z ahead). At the zero, the robot's camera sees exactly what the operator's eyes
would; every other camera pose and every robot link follows through `M`.

## Phase 2: world-locked depth, its holes filled (built)

- **The head's measured pose at each depth frame** (`neck/head_pose.py`). A live head's teleop publishes every reply's
  rows of the head service's 100 Hz record on a local ZMQ PUB (`tcp://127.0.0.1:5596`): the Jetson's wall time (the
  service's clock plus the smallest gap seen, let rise slowly), `neck_z` and `neck_y` (CAD angles through the
  head's axes), and the colour camera's pose in the robot frame (forward kinematics on the URDF; the camera at the
  head profile's `camera`), with its pose at zero. Helm's camera relay subscribes and stamps each head depth frame
  with the sample nearest its capture (its timestamp less 20 ms; none within 50 ms, none). Both clocks are the
  Jetson's: nothing compares the PC's. A feed that fails ends; the head goes on.
- **World-locked in the headset.** Helm's zero (its status replies carry it, wherever it was set) and the eyes'
  position at the next frame after it give `E0`; each frame's rig is `M · T_robot_color(capture) · T_color_depth · F`.
  The 3D stays where it is in the room as the operator's head turns, a lagging robot head only delays the newly seen
  parts, and leaning shows parallax. Without a head sample (no session) it falls back to phase 1 (it follows the
  eyes). World-locked, the 3D needs no following: Follow head only moves the flat picture then. A zero taken with
  the head pitched tilts the room by that pitch, as the head follows the pitch from there.
- **Holes, filled in the headset** (`static/xr_fill.js`, in a worker: ~3 ms a frame on the laptop; each pixel keeps
  the pixel its colour comes from, and the shader colours it from that one's own reading):
  - small holes (the sensor's dropouts, up to 2 pixels from a reading) take the farthest reading near them;
  - pixels the colour camera cannot see (it sits 15 mm beside the depth sensor and sees past a near object's edge
    from the side: a z-buffer of the depth in its view, each reading over its footprint) take their colour from the
    nearest seen pixel along the row on their own surface. Phase 1 painted the table beside an object's edge with
    the object's colour; spread by the fill, that would have coloured whole regions;
  - behind each near object's edge, a background layer drawn behind it: the far surface carried up to 40 pixels
    under the object as the plane it lies in (straight in inverse depth), coloured from its edge;
  - filled parts are drawn a little darker (`FILL_SHADE`);
  - larger holes (out of range) keep the 3 m colour backdrop.
- **Checked offline** (a synthetic table, box and wall through the real scene and worker in Chromium): leaning 35 cm,
  the region behind the box shows the table, darker, where phase 1 showed the box's colours projected 3 m away.
  Not yet seen in the headset.
- **Next for the holes:** learned completion (PromptDA-style, as GHOST) needs a GPU: soliloquy's first (phase 4),
  the Quest's own once WebGPU models are small enough.

## Phase 3: Minerva's model and the wrist cameras (built)

- **The model in the same space** (`static/xr_model.js`). The headset loads the display model Helm's 3D view draws
  (`/api/models/<id>`), posed by the positions the 3D view follows (`/api/calibration-guide`, arms and neck, at
  10 Hz while shown), and places it by `M`, as the 3D picture is. The arms in the depth then lie on the arms of the
  model: the operator sees the arms even where the camera does not, and any misalignment shows the camera's mount
  needs a calibration (through a Helm flow, the user's rule). It shows with the world-locked 3D only (M comes from
  the depth's head pose).
  - Its meshes come coarse: the full model is 2.66 M triangles and 128 MB, too much for a Quest in stereo. Helm
    serves each mesh with `?detail=coarse` (`viewer/server.py`: vertices merged within 5 mm cubes, made once from the
    mesh store's copy on Minerva, cached): 89 k triangles, 0.67 MB over the wire.
  - The head's own links (from `neck_z` on) are left out: they would sit around the operator's eyes.
  - It is drawn see-through (55 % of the picture's opacity), on top where it and the depth meet; a real surface in
    front of it still hides it.
- **The wrist cameras above the hands** (`static/xr_hands.js`). The session asks for hand tracking
  (`hand-tracking`, optional); each wrist camera's picture (Helm's `left_wrist`, `right_wrist`) floats 14 cm above
  the matching hand's wrist, facing the eyes, as GHOST floats the gripper camera above the controller. A hand lost to
  tracking leaves its picture where it was, dimmed; without hand tracking, the controllers carry them.
- **The bar** has Model and Hands switches, both on by default. The two wrist streams add about 1 MB/s to the Quest's
  Wi-Fi; if the head camera's rate drops, Hands off frees it.
- **Checked offline:** the coarse model, loaded through the real scene from the model viewer, stands upright where
  the zero puts it, its head left out. Hand tracking needs the headset.

## Alignment: the cameras with each other, then the model with the cameras (head camera built)

The user asked (2026-10-31) for one way to align everything, using the table's fiducials, which are placed to within
5 mm. Today each view lines up only by CAD: the head camera's mount is the `AWG_RGB_CAMERA` part's centre, looking
along the model's front, and the wrist cameras are only calibrated for their own lenses. Everything after (the model
over the depth, the wrist pictures, the map's frame) depends on these being measured.

- **The table is the reference.** The board frame (`config/devices/minerva-table-board.json`: origin at tag 0's
  outer corner, x toward tag 1, y away from Minerva, z up; tags 0-5, 43.18 mm squares) is the world frame of the
  workcell, and later the map's.
- **Head camera first.** The head is held at a few poses where it sees the board, steered by the headset or jogged
  (never swept: the user's rule). At each, a noise-stacked still (as the wrist calibration takes them) and the neck's
  measured angles (the head pose feed). Each still's tag corners give the camera's pose against the board (the
  camera's own published intrinsics). Over the poses, one solve finds both the camera's mount in the head link and
  where the board lies in Minerva's frame (a hand-eye problem, refined on every corner's reprojection). It reports
  the residuals in pixels and millimetres.
- **Wrist cameras next, against the same board.** The arms are posed (teleop) so each wrist camera sees the board;
  with the board's place in Minerva's frame now known and the arms' measured joints, each wrist camera's mount in its
  wrist link follows. The cameras are then aligned with each other through the board.
- **Then the model with the cameras.** With every mount measured, the model's arms and grippers are drawn over each
  camera's picture: where they miss, the residuals say whether a joint's zero is off (a separate, deliberate step),
  or a mount.
- **A Helm flow, redone at will** (the user's rule: calibrations only through Helm flows): steps on screen, a capture
  key or pedal per still, the solve, its residuals and the overlay, then save into the operator files with the
  previous ones kept. The head pose feed, the headset's model placement and the map all read the saved mounts.
- **Accuracy to expect:** the board's tags within 5 mm, each still's pose to 1-2 mm at the table (the head camera fit
  the tags to 0.75 px on 2026-10-18), so the views should agree to about 5 mm.

**The head camera's part, built (2026-10-31, deployed `2552fe2`; not yet run on the real head):**
- **The flow:** the Teleop tab's calibration card (Stills of the table's tags) has the head as a third camera. In a
  session with the live head: turn it until its camera sees three or more tags, hold (T), capture (P); 8-12 stills
  turned and tilted at least 10 degrees apart; Solve; Save alignment. Runs live beside the wrists' under
  `~/minerva/helm/calibration/head/<run>/` (stills.jsonl, the averaged images, result.json).
- **A still** (`helm.calibration_still`, on the robot): the stack of fresh frames averaged and its tags found, as a
  wrist still; and the head's pose feed read from before the first frame until past the last. The still keeps the
  neck's mean CAD angles over the stack, refused when either joint spread more than two of the servos' 1.53 mrad steps
  (`config/devices/minerva-head-alignment.json`) or the samples do not cover the frames. Every still carries the
  intrinsics the camera published with its frames and names the head reference its angles were read through.
- **The solve** (`vision.head_alignment`): the wrist calibration's (each still's board pose, the robot-world hand-eye
  closed form, then every corner's reprojection with soft-L1), with the intrinsics held at the D435's published ones.
  It reports the mount against the CAD guess, the error in pixels and millimetres at the board, where the neck's pan
  axis lies on the board from tag 0's corner (a tape measure can check it), and whether the head turned and tilted
  enough. A two-joint neck cannot tell its pan zero from where the board lies (the placement takes it up) nor its tilt
  zero from the mount: the result compares the board with the arms' own placement from the latest solved wrist
  calibration, and the part of the difference about the pan axis is the pan zero error.
- **Saved** as the head's camera file beside its reference and limits (`<asset>-head-camera.json`, the one it
  replaces kept in the run's folder). A live head's pose feed then places the camera through the measured mount
  when the file was made through the head reference in use (`neck.head_pose.aligned_mount`), else through the CAD
  one; the head's status says which (`camera_mount`). So the headset's world-locked 3D and model, and the relay's
  stamps on the head's depth, use it from the next session. Display only: nothing that moves reads it.
- **Checked offline** (`tests/unit/test_head_alignment.py`, synthetic stills with 0.2 px noise from a mount 10.8 mm
  and 2.7 degrees off the CAD guess): ten stills find the mount within 1.5 mm and 0.15 degrees and the board within
  1 mm; with a 2 degree pan zero error the mount stays as good and the comparison with the arms reads 1.97 degrees.
  Five stills already find the mount to about 1 mm. On Minerva, a head still from the real camera (a stand-in head
  feed): 8 frames in 0.47 s, 54 feed samples covering them, the D435's intrinsics, 1.7 s in all.
- **Still to come:** the wrist cameras' mounts against the same board (their calibration solves the board's place
  too; held against the head's), the model drawn over each camera's picture, and the pan zero, if the arms' board
  says it is off, set deliberately (the head's reference flow).

## Phase 4: Gaussian splatting on soliloquy (a static map built end to end; a moving Minerva needs a live map)

- **Frames:** the map in the world frame (the table's board, from the alignment), Minerva's pose in it
  `T_world_robot`: fixed today (the alignment's board pose), from odometry and the map's tracking once the base moves.
  Every capture and every live frame carries the camera's pose in the world: `T_world_robot · T_robot_camera(q)`.
- **Static first:** with the base still, the head camera's posed RGB-D builds the map on soliloquy (the trainer here,
  then the live mapping), in the world frame; the headset draws it placed through Minerva's pose.
- **Rover-ready from the start:** `T_world_robot` is an input of its own everywhere (a constant now, a stream later),
  so nothing changes when the base starts reporting odometry but where that pose comes from.
- **Training on soliloquy** (set up 2026-10-31, the user: "yes, set up WSL2 training on soliloquy"). Its Ubuntu (WSL2)
  has a venv of its own in `~/minerva-splat` (5.1 GB: CUDA PyTorch 2.4.1 cu124, gsplat 1.5.3's prebuilt wheel,
  nothing system-wide). WSL runs bridged there (the user's `.wslconfig`) and its Ubuntu has no network, so
  `scripts/splat/download_wheels.ps1` fetches the Linux wheels on the Windows side (2.7 GB, including torch's
  Linux-only CUDA libraries, which pip skips on Windows) and `scripts/splat/install_wsl.sh` installs them offline.
  Switching WSL to mirrored networking would let it download directly; that is the user's setting.
- **The trainer** (`scripts/splat/train_head_scene.py`) takes a head scene capture: `scene.json` (the colour and
  depth intrinsics, the registration, each frame's colour camera pose in the robot frame), the camera's JPEGs and its
  depth (zlib uint16). Gaussians start from the depth, registered into the colour camera and back-projected into the
  robot frame (one per centimetre cube); colour (L1 + SSIM) and depth are fitted with gsplat's rasterizer and its
  default densification; every eighth frame is held out and scored. It writes a standard 3DGS PLY in the robot
  frame, which the headset places by `M` as it places the model.
- **Checked on a synthetic capture** (`scripts/splat/synthetic_scene.py`: a table, a box and a wall seen as the neck
  pans and tilts, 64 frames): 3000 steps in 59 s on the 3080 (shared), 105 k to 158 k Gaussians, 34.3 dB PSNR on the
  held-out frames; the PLY is 39 MB (spherical harmonics of degree 3).
- **Not a static scene: Minerva will move** (the user, 2026-10-31: "Minerva will not stay static, I am working on a
  mobile base for it"). A scene trained once in the robot frame stops lining up the moment the base drives. The map
  has to live in a world frame, with Minerva's pose in it tracked all the time:
  - **Tracking:** Minerva's pose in the world from the head camera's RGB-D (visual odometry against the map), with
    the neck's measured pose (the camera's motion relative to the body, known exactly) and, once the base exists,
    its wheel odometry as priors.
  - **Mapping, live, on soliloquy:** an incremental RGB-D Gaussian-splatting SLAM fed by the head camera's frames as
    they come (candidates to try on the 3080: GS-ICP SLAM and RTG-SLAM for speed; MonoGS and SplaTAM as references).
    It adds what the camera sees, refines what it saw before, and keeps out what moves: Minerva's own arms (the
    model, posed, projected into each frame) and anything the live depth disagrees with.
  - **In the headset:** the map drawn in the world frame. The zero mapping becomes world to room, anchored at the
    zero. Minerva's model and the live depth are placed by her tracked pose, so as the base drives, the room stays
    put around the operator and Minerva moves through it. The live depth keeps drawing over the map, for what moves.
  - The trainer built here stays useful: for first maps of a room, for tests, and as the quality reference for what
    the live mapping reaches.
- **The static map, built (2026-10-31, deployed `2552fe2`; not yet captured live):**
  - **Capture:** the Teleop tab's map card (Head map), in a session with the live head engaged: Capture 15, 30 or
    60 s, then look slowly around the workspace in the headset. `helm.scene_capture` runs on the robot, reading the
    head camera's frames and the head's pose feed; a frame is kept when the head turned 3 degrees or moved 3 cm since
    the last kept one (at most 3 a second), steady around its exposure. It goes to
    `~/minerva/helm/maps/captures/<stamp>/` in the trainer's format: the picture as published, the depth, the
    camera's pose in Minerva's frame and Minerva's pose in the world beside it (the board's once the head camera's
    alignment holds, else her own frame). Rehearsed on Minerva (a stand-in head feed stepping 5 degrees): 61 frames
    seen in 4 s, 10 kept, 1.8 MB.
  - **Train:** `scripts/splat/train_on_station.py` on the build PC (it holds both keys) zips the newest capture
    without a map on Minerva, trains it in soliloquy's WSL venv (its GPU's load printed first; nothing trains with
    less than 3 GB free) and writes `map.splat` and `map.json` into `~/minerva/helm/maps/maps/<capture>/`. The
    trainer builds the scene in the capture's world and writes the compact `.splat` (32 bytes a Gaussian, most
    visible first: 3.5 MB for 109 k Gaussians, against the PLY's 27 MB). Run end to end on the real machines.
  - **Serve:** `/api/helm/maps` lists the maps (newest first) with Minerva's pose in each world;
    `/api/helm/maps/<name>/map.splat` serves the Gaussians.
  - **Draw:** the headset's bar has a Map switch. With the world-locked 3D, `xr_splat.js` draws the newest map it can
    place, by Minerva's pose in its world and `M`: each Gaussian a quad facing the eye, sized by its covariance
    projected for that eye, blended back to front, the order sorted in a worker as the view turns. Drawn from a
    synthetic capture's own frames it matches what the trainer fitted: 34.4 dB on a trained frame, 32.0 dB on a
    held-out one (the trainer scored 32.7 dB held out).
  - **Rover-ready:** Minerva's pose in the world is an input of its own everywhere (each capture frame records it,
    the listing gives the headset the current one); when the base drives, only where that pose comes from changes.
- **Next:**
  1. Stream the head camera's posed RGB-D to soliloquy: the relay's frames with their head pose (phase 2), at a rate
     the tether carries.
  2. A live splatting SLAM on soliloquy from that stream, the neck's pose as its prior; first with the base still,
     then with odometry once the base exists.
  3. The map's changes to the headset, and a splat renderer there (instanced quads, the splats sorted by depth in a
     worker), placed by Minerva's tracked pose.
  4. **Then on the Quest alone:** rendering splats is light enough for the Quest Pro's GPU; building them is not
     yet. When it is (WebGPU, smaller models), soliloquy drops out.

## What each phase must show

| Phase | Proof |
|---|---|
| 2 | Turning the head, the table stays put in the headset; leaning shows what is behind an object, filled. |
| 3 | The model's arms lie on the arms in the depth; each wrist camera follows its hand. |
| Alignment | From the table's tags, each camera's mount and the board's place in Minerva's frame, saved by a Helm flow; every view agrees to about 5 mm. The head camera's flow is built; its first real run is pending. |
| 4 | A live splat map of the room renders in the headset around the live depth, aligned with it, and stays put as Minerva moves. |
