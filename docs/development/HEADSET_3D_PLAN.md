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

## Phase 3: Minerva's model and the wrist cameras (next)

- **The model in the same space.** The headset loads the display model as Helm's 3D view does (`/api/models/<id>`,
  its meshes) and poses it with the live positions (`/api/calibration-guide`, arms and neck), placed by `M`. The
  arms in the depth then lie on the arms of the model: the operator sees the arms even where the camera does not,
  and any misalignment shows the camera's mount needs a calibration (through a Helm flow, the user's rule).
- **The wrist cameras above the hands.** The session asks for hand tracking (`hand-tracking`, optional); each wrist
  camera's picture (Helm's `left_wrist`, `right_wrist`) floats just above the matching hand, facing the eyes, as
  GHOST floats the gripper camera above the controller. The operator's hands hold the GELLO leaders, so the hands,
  not controllers, carry them; without hand tracking, the controllers do. A switch on the bar turns them off.

## Phase 4: Gaussian splatting on soliloquy

- **Offline first.** A recorder on the Jetson keeps posed RGB-D (the head camera with its phase 2 pose) for a few
  minutes of a session. soliloquy trains a 3D Gaussian Splatting scene from it (depth-initialised, gsplat or
  nerfstudio on the RTX 3080, minutes). The headset renders the splats (a three.js splat renderer over WebXR) in
  the robot frame through `M`, with the live depth over it: a full, photo-real workcell around the live view.
- **Then live.** An incremental splatting SLAM (SplaTAM, MonoGS and kin) on soliloquy keeps the scene up to date
  from the live stream and sends the changed Gaussians to the headset.
- **Then on the Quest alone.** Rendering splats is light enough for the Quest Pro's GPU; building them is not yet.
  When it is (WebGPU, smaller models), soliloquy drops out.

## What each phase must show

| Phase | Proof |
|---|---|
| 2 | Turning the head, the table stays put in the headset; leaning shows what is behind an object, filled. |
| 3 | The model's arms lie on the arms in the depth; each wrist camera follows its hand. |
| 4 | A recorded splat scene of the workcell renders in the headset around the live depth, aligned with it. |
