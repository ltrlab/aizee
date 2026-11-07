# Wrist camera calibration

Both wrist cameras' intrinsics, distortion and mounts, and the table board's placement in the robot frame, found from still
poses in a live teleop session on the table's tags. Nothing here moves a motor or changes the motion contract. The camera
noise tool is the only thing that writes camera settings, and it restores them.

**Status.** The solver, the stills and the Helm flow are built and tested offline, and stills have been taken live (the first on
2026-11-02). **No document records a solve on the robot**, so no solved wrist mount exists yet: the headset's wrist pictures use
the nominal mount from the simulator's wrist camera. The board is a rough one (about +-5 mm), and the light-panel session
below has not been done.

## In Helm

Teleop > **Mode: Calibrate** > step **1 Wrist cameras** ([Helm manual](HELM_MANUAL.md#calibrate-mode)). Calibration exists only
with Helm on Minerva, and only with a live session (Hold or Teleop) on the real arm and no recording running (a still's decoding
would share the recorder's CPUs).

1. **Place the robot** where you can measure it against the table's taped rectangle ([table fiducials](TABLE_FIDUCIALS.md)).
   Measure each shoulder's J1 axis (the top motor's centre) projected straight down: its distance along the near side from tag 0's
   outer corner (x) and from the near side (y, negative on the robot's side), and the table top's height below the J1 axes. The
   solve reports the same numbers (the "placement" line) so you can compare.
2. **Start a live session** (Left, Right or Both). For each camera take 15 to 25 stills: bring the camera over the table until it
   sees one or more tags (more is better; one is fine), pause (**T** while following, or pedal B), wait for the still, then
   resume. **Capture still (P)** takes one by hand. Vary distance (about 15 to 50 cm), tilt (up to about 45 degrees), roll (turn
   J6) and where the tags fall in the image, corners and edges included (distortion is largest there). Check each still's line:
   tags seen, movement under 1.5 mrad, white well above black (aim for white 150 to 230). **Undo last** drops the last still;
   **New run** starts a new run.
3. **Solve.** Read each camera's error (about 0.3 to 1 px with good kinematics, more if not) and its held-out error, the mounts
   and their sigmas, the placement against your measurement, and whether the arms agree. The verdict is good to 1.5 px, not
   usable past 3 px or with the two arms' boards more than 25 mm apart.

A solve's acceptance for a first test: both cameras solve; held-out error close to the fit's; mounts within a few mm and degrees
of the simulator's guess; the placement within about 10 mm of the tape measure; the two arms agreeing within a few mm.

Runs live under `~/minerva/helm/calibration/<run>/` (`stills.jsonl`, `images/`, `result.json`) and can be re-solved offline from
the run folder. The rig and board are `config/devices/minerva-wrist-cameras.json` and `config/devices/minerva-table-board.json`.
The intrinsics are for 640 x 480 MJPEG as the nodes publish it; another resolution needs its own calibration.

## How it works

**Unknowns, per camera:** intrinsics (fx, fy, cx, cy), Brown distortion (k1, k2, p1, p2; k3 optional, OpenCV's order) and the mount
X = T_link_camera on the gripper body after J6 (`1_1_02_eb571_960_stp[_2]`). **Shared:** the board's placement Y = T_base_board and
each tag's offset (dx, dy, dyaw) from the taped layout with priors of 5 mm and 2 degrees. **Optional:** J1 to J5 zero offsets per
arm (prior 1 degree); J6's is the mount's. A board corner p is seen at `project(camera, X^-1 F(q)^-1 Y p)`, with F(q) the URDF chain
at the still's joints.

- **Solver** (`vision/wrist_calibration.py`, numpy and scipy only): planar calibration per camera (Zhang), then robot-world-hand-eye
  in closed form per camera (Shah 2013), then everything refined together on reprojection error (soft-L1, so a bad corner pulls
  with bounded force) with MINPACK's Levenberg-Marquardt.
- **Report:** per camera the error per still and over the image (a 4 x 3 coverage grid, error by radius), parameter sigmas, a
  5-fold cross-validated error, each camera's own board placement (the two arms' kinematic agreement), and where the robot sits on
  the board.
- **Solve** (`vision/solve.py`): fits the URDF as is and with joint offsets, recommends the lower cross-validated error, and fits
  the cameras on J6's parent link too: if that fits better the camera does not turn with J6.
- **A still** (`helm/calibration.py`, `helm/calibration_still.py`): Helm reads the arm's joints twice 0.25 s apart, runs the helper
  on the robot (8 fresh frames averaged, tags found, light measured), and reads again; the still is kept only if no joint moved
  more than 1.5 mrad. Corners are refined with a window inside each tag's border cell. The helper runs as a child of Helm, not
  over SSH, because an SSH logout once stalled the arms' receiver.
- **The light on each tag:** its black border and the white label half a cell outside, as levels, temporal noise and the white's
  R/G and B/G, so exposure and white balance are checked in the working scene.

**Offline checks** (synthetic stills from the real URDF chains): mounts within 0.25 mm and 0.03 degrees, intrinsics within 0.1 px,
placement within 2 mm, cross-validated error at the noise floor. Joint zero errors matter: 0.3 degrees on J1 to J5 costs about
2 px and biases a mount by up to 7 mm; 1 degree costs 7.5 px and 20 mm, which the optional joint offsets recover. The wrong mount
link fits at 210 px against 0.26 px. Detection on the Jetson's OpenCV 4.5.4 found every tag with the right ID, corners 0.55 px RMS
on 20 to 30 px tags.

## Camera noise

`scripts/camera_noise_stack.py` on the Jetson, with no session running and the arms at rest, stacks 120 frames as the camera nodes
deliver them (640 x 480 JPEG q85). Measured with the wrist cameras looking at the robot's frame under room light:

| | left wrist | right wrist | head (D435 colour) |
|---|---|---|---|
| frame rate | 62.5 fps | 50.1 fps | 15.0 fps |
| mean luma | 32 of 255 | 33 | 70 |
| temporal noise (luma, median) | 2.10 DN | 2.38 DN | 1.87 DN |
| noise of the 120-frame average | 0.18 DN | 0.18 DN | 0.19 DN |

- The noise is temporally white, so N frames cut it by sqrt(N): a still of 8 frames brings about 2.2 DN to 0.8 DN. Calibration
  stills use 8.
- The wrist images are dark (mean luma about 30 at 15 ms exposure and gain 30); noise grows with level. Flicker bands sit 6 to 13
  times above the noise floor, worst on the right wrist (power-line setting 50 Hz): light flicker under a rolling shutter, not noise.
- A gain sweep (0, 15, 30, 60, 100) gives luma 23, 27, 32, 40, 50 and noise 1.6 to 3.2 DN on the left wrist: gain buys brightness
  at proportional noise; light buys it free. "Manual" exposure behaves oddly (16.7 ms and 33.3 ms gave the same or lower luma than
  15 ms), so the processor seems to adjust gain on its own.
- Both wrists run auto white balance; a fixed `white_balance_temperature` (2800 to 6500 K) works and the tool restores auto after.

**Not yet done: the light-panel session.** Light the work area with panels at one colour temperature (no session), hold a panel's
diffuser in front of a wrist camera and sweep its white balance against the known temperature:
`python3 camera_noise_stack.py --label panel5600 --roles left_wrist right_wrist --frames 60 --set white_balance_automatic=0
--sweep white_balance_temperature=4000,4800,5600,6400`. R/G and B/G of the centre are 1 for a neutral camera: the setting
nearest 1 is that camera's white balance under that light. The same stack gives vignetting and photo-response non-uniformity.

## Uncertainty

The board is known to +-5 mm, so the placement is too; the cameras' relative geometry is not limited by it. Unmodelled kinematics
(link lengths, compliance under gravity, backlash) set a floor the joint offsets can only partly absorb: the cross-validated
error is the honest measure. The head camera uses the same solver with the mount in the base frame ([head camera](HEAD_CAMERA_ORIENTATION.md)).

## Files

`config/devices/minerva-table-board.json`, `config/devices/minerva-wrist-cameras.json`; `src/robot_platform/vision/`
(`geometry`, `board`, `chain`, `rig`, `wrist_calibration`, `solve`, `synthetic`); `src/robot_platform/helm/calibration.py`,
`calibration_still.py`, `static/calibration.js`, `static/calibration_guide.js`; `scripts/camera_noise_stack.py`; tests
`tests/unit/test_vision_geometry.py`, `test_wrist_calibration.py`, `test_helm_calibration.py`, `tests/ui/test_helm_calibration.mjs`.
