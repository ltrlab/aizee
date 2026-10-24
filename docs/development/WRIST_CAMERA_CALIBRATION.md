# Feature task: wrist camera calibration and camera noise

- **Branch and worktree:** `claude/wrist-camera-calibration` in `C:\Users\ltr\Workspace\aizee-camera-calib`.
- **Base commit:** `67673e4` (`claude/helm-on-jetson`, the deployed release `a94921a` plus its record).
- **Owner:** Claude (the 2026-10-24 session), at the user's request: "measure the camera noise with frame stacks [...]
  Can we also use the board I already setup for calibration for now? It will be rough since it's not precise,
  probably within +-5mm [...] build out the calibration structure until you can test live [...] I want both the wrist
  cameras calibrated for this first test since the head is not ready yet."
- **Scope:** both wrist cameras' intrinsics, distortion and mounts, and the table board's placement in the robot
  frame, from still poses in a live teleop session. The head D435 is out of scope for now. Nothing here moves a
  motor or changes the motion contract; the only camera writes are the noise tool's, which restores them.

## Camera noise, measured 2026-10-24 (frame stacks)

`scripts/camera_noise_stack.py` on the Jetson, no session running, the arms at rest (the wrist cameras look at the
robot's wooden frame under room light). Stacks of 120 frames as the camera nodes deliver them (640x480 JPEG q85).

| | left wrist | right wrist | head (D435 colour) |
|---|---|---|---|
| frame rate | 62.5 fps | 50.1 fps | 15.0 fps |
| mean luma | 32 of 255 | 33 | 70 |
| temporal noise (luma, median) | 2.10 DN | 2.38 DN | 1.87 DN (blue 5.2) |
| noise of the 120-frame average | 0.18 DN | 0.18 DN | 0.19 DN |
| white-noise prediction, sigma/sqrt(120) | 0.19 | 0.22 | 0.17 |
| rolling-shutter row bands (std / noise floor) | 0.51 / 0.08 | 1.26 / 0.09 | 0.23 / 0.07 |

- **Stacking works as theory says:** the noise is temporally white, so N frames cut it by sqrt(N). A still of 8
  frames brings ~2.2 DN to ~0.8 DN. The calibration stills use 8.
- **The wrist images are dark:** mean luma ~30 with exposure 15 ms and gain 30. Noise grows with level
  (shot noise): 1.6 DN in the darkest bin to 3.9 DN at 64-80 on the right wrist.
- **Flicker bands** sit 6-13x above the noise floor, worst on the right wrist (its power-line setting is 50 Hz).
  Bands are smooth across rows (lag-1 correlation 0.95-0.99): light flicker under a rolling shutter, not noise.
- **Gain sweep** (exposure 15 ms; each value restored): gain 0 / 15 / 30 / 60 / 100 gives luma 23 / 27 / 32 / 40 /
  50 and noise 1.6 / 1.8 / 2.1 / 2.5 / 3.2 DN on the left wrist. Gain buys brightness at proportional noise; light buys
  it free.
- **Exposure behaves oddly:** 16.7 ms and 33.3 ms at gain 0 gave the same or lower luma than 15 ms (the frame rate
  did drop to 28 fps at 33 ms). The camera's processor seems to adjust gain on its own in "manual" exposure. Measure
  this under the light panels, where the scene is controlled.
- **White balance:** both wrists run auto white balance. A fixed `white_balance_temperature` (2800-6500 K) works and
  the tool restores auto afterwards (tested 2026-10-24).

### Next: the light panels (the user's tuned panels at set colour temperatures)

1. Light the work area with the panels at one colour temperature, no session running.
2. Flat field and colour: hold a panel's diffuser in front of a wrist camera, filling its view (out of focus is
   fine), and sweep the camera's own white balance against the panel's known temperature:
   `python3 camera_noise_stack.py --label panel5600 --roles left_wrist right_wrist --frames 60
   --set white_balance_automatic=0 --sweep white_balance_temperature=4000,4800,5600,6400`.
   R/G and B/G of the centre are 1 for a neutral camera: the setting nearest 1 is that camera's white balance under
   that light. Repeat per panel temperature for a table of panel temperature against camera setting. The same stack
   gives vignetting (corner luma against centre) and photo-response non-uniformity.
3. Noise against light: repeat the baseline and a gain sweep under the panels.
4. During calibration, every still also measures the light on its tags (below), so the working scene gets checked
   too.

## Wrist calibration: the design

**Unknowns, per camera:** intrinsics (fx, fy, cx, cy), Brown distortion (k1, k2, p1, p2; k3 optional, in OpenCV's
order so results plug into OpenCV), and the mount X = T_link_camera on the gripper body after J6
(`1_1_02_eb571_960_stp[_2]`, from the sim's wrist cameras). **Shared:** the board's placement Y = T_base_board, and
each tag's offset (dx, dy, dyaw) from the taped layout with priors of 5 mm and 2 degrees. **Optional:** J1-J5 zero
offsets per arm (prior 1 degree); J6's is the mount's.

A board corner p is seen at `project(camera, X^-1 F(q)^-1 Y p)`, with F(q) the URDF chain at the still's joints.

- **Solver** (`vision/wrist_calibration.py`, numpy and scipy only, so the gate tests every path without OpenCV):
  1. Planar calibration per camera (Zhang): focal length from the stills' homographies, then intrinsics and board
     poses refined, from starting focal lengths until one fits.
  2. Robot-world-hand-eye in closed form per camera (Shah 2013), translation by least squares.
  3. Everything refined together on reprojection error (soft-L1, so a bad corner pulls with bounded force).
  - MINPACK's Levenberg-Marquardt with a grouped finite-difference Jacobian: scipy's trust-region solver spent 90 % of
    its time in multithreaded BLAS on these small matrices (46 s against 2.8 s).
- **Report:** per camera the error per still and over the image (a 4x3 coverage grid, error by radius), parameter
  sigmas from the Jacobian, a 5-fold cross-validated error (each fold's stills predicted from mounts fitted to the
  others), each camera's own board placement (the two arms' kinematic agreement), and where the robot sits on the
  board: each J1 axis from tag 0's outer corner (x along the near side, y toward the far side, z above the table) and
  the robot's turn.
- **Solve** (`vision/solve.py`): fits both models (URDF as is; with joint offsets), recommends the lower
  cross-validated error, and fits the cameras on J6's parent link too: if that fits better, the camera does not turn
  with J6. Writes `result.json`.
- **A still** (`helm/calibration.py`, `helm/calibration_still.py`): needs a live session (HOLD or TELEOP) on the real
  arm, and no recording (the still's decoding would share the recorder's CPUs). Helm reads the arm's joints twice 0.25 s apart, runs the helper on the robot (8 fresh frames averaged, tags
  found, light measured, the average saved as PNG), reads again; the still is kept only if no joint spread more than
  1.5 mrad over the whole of it. Corners are refined with a window inside each tag's border cell: OpenCV's fixed
  11 px window put corners 3 px off on small tags (rehearsed).
- **The light on each tag:** its black border and the white label half a cell outside, as levels, temporal noise and
  the white's R/G and B/G: exposure and white balance checked in the working scene.
- **Helm:** the Teleop tab's "Wrist camera calibration" card: camera choice, Capture (C), Undo, Solve, New run, the
  last still with its tags outlined, and the solve's lines. Runs live under `~/minerva/helm/calibration/<run>/`
  (`stills.jsonl`, `images/`, `result.json`). Calibration exists only with Helm on Minerva.

## Rehearsals (before any live test)

- **Synthetic stills from the real URDF chains** (`vision/synthetic.py`, `tests/unit/test_wrist_calibration.py`):
  14-20 stills a camera, 0.2-0.3 px noise, the layout off by ~4 mm, 1 degree:
  - mounts within 0.25 mm and 0.03 degrees, intrinsics within 0.1 px, placement within 2 mm (the tape's mean), the
    tags' relative layout within 2 mm; cross-validated error at the noise floor;
  - **joint zero errors matter:** 0.3 degrees on J1-J5 costs ~2 px and biases a mount by up to 7 mm; 1 degree costs
    7.5 px and 20 mm. Fitted, 0.5-degree errors are recovered within 0.04 degrees and the error falls back to the
    noise floor (0.40 px), mounts within 0.3 mm;
  - a corner moved 25 px is flagged as an outlier and moves the mount under 1.5 mm;
  - the wrong mount link (J6's parent) fits at 210 px against 0.26 px.
- **Detection on the Jetson** (OpenCV 4.5.4): tags rendered into synthetic distorted views (white labels on a black
  table, noise, JPEG q85), stacked: all tags found with the right IDs and order; corners 0.55 px RMS on 20-30 px
  tags; white 234 / black 17.5 against 235 / 18 rendered.
- **Live stills on both wrist cameras** through the helper: 8 frames in 0.11-0.14 s, 0.4 s per still including
  decoding, saving and the thumbnail; no tags at the rest pose, as expected.

## The live test (with the user present)

**Before:** deploy this branch (the user's approval), light the table (the panels), no recording running.

1. **Place the robot** where you can measure it against the taped rectangle. Measure, as well as you can:
   - each shoulder's J1 axis (the top motor's centre) projected straight down: its distance along the near side from
     tag 0's outer corner (x), and its distance from the near side (y, negative on the robot's side);
   - the table top's height below the J1 axes.
   The solve reports the same numbers (the "placement" line), so the two can be compared.
2. **Start a live session** (Live · left, then Live · right, or Live · both). For each wrist camera, 15-25 stills:
   - bring the camera over the table until it sees one or more tags; more tags a still is better, but one is fine;
   - hold (T), wait a moment, Capture (C), then T to move on;
   - vary distance (about 15-50 cm), tilt (up to ~45 degrees), roll (turn J6), and where the tags fall in the image,
     corners and edges included (distortion is largest there);
   - check each still's line: tags seen, movement under 1.5 mrad, white well above black (aim for white 150-230).
3. **Solve.** Read: each camera's error (expect about 0.3-1 px with good kinematics, more if not) and held-out error,
   the mounts and their sigmas, the placement against your measurement, and whether the arms agree.
4. Everything is kept on Minerva; I can re-solve offline from the run folder.

**Acceptance for this first test:** both cameras solve; held-out error close to the fit's; mounts within a few mm and
degrees of the sim's guess (it is a guess); the placement within about 10 mm of the tape measure (the board is
+-5 mm and the tape similar); the two arms agreeing within a few mm.

## Uncertainty and next steps

- The board is known to +-5 mm, so the placement is too (its mean); the cameras' relative geometry is not limited by it.
- Unmodelled kinematics (link lengths, compliance under gravity, backlash) set a floor the joint offsets can only
  partly absorb. The cross-validated error is the honest measure.
- The intrinsics are for 640x480 MJPEG as the nodes publish it; another resolution needs its own calibration.
- Later: the head D435 (fixed camera: the same solver with X in the base frame and no chain), and the sim's wrist
  cameras updated from the result.

## Files

- `config/devices/minerva-table-board.json`, `config/devices/minerva-wrist-cameras.json`
- `src/robot_platform/vision/` (`geometry`, `board`, `chain`, `rig`, `wrist_calibration`, `solve`, `synthetic`)
- `src/robot_platform/helm/calibration.py`, `calibration_still.py`, `static/calibration.js`; the card in
  `index.html`, `helm.js`, `helm.css`, `classic-helm.css`
- `scripts/camera_noise_stack.py`
- Tests: `tests/unit/test_vision_geometry.py`, `test_wrist_calibration.py`, `test_helm_calibration.py`,
  `tests/ui/test_helm_calibration.mjs`
