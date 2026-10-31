import assert from 'node:assert/strict';
import {captureBlock, countsText, headCaptureBlock, headCountsText, headLastText, headResultLines, lastText,
  resultLines} from '../../src/robot_platform/helm/static/calibration.js';

const calibration = (extra = {}) => ({run: '20261025T010203Z', cameras: ['left_wrist', 'right_wrist'], busy: false,
  solving: false, counts: {left_wrist: {stills: 3, tagged: 2}, right_wrist: {stills: 1, tagged: 1}}, ...extra});

// A still needs a live session holding or following, and nothing else in progress.
assert.match(captureBlock(null, {phase: 'running', state: 'HOLD'}), /Helm on Minerva/);
assert.match(captureBlock(calibration(), {phase: 'stopped'}), /Start a live session/);
assert.match(captureBlock(calibration(), {phase: 'running', state: 'IDLE'}), /Enable the arms \(E\), then hold \(T\)/);
assert.match(captureBlock(calibration({busy: true}), {phase: 'running', state: 'HOLD'}), /Capturing/);
assert.match(captureBlock(calibration({solving: true}), {phase: 'running', state: 'HOLD'}), /Solving/);
assert.equal(captureBlock(calibration(), {phase: 'running', state: 'HOLD'}), null);
assert.equal(captureBlock(calibration(), {phase: 'running', state: 'TELEOP'}), null);
assert.match(captureBlock(calibration(), {phase: 'running', state: 'TELEOP', task: {recording: {state: 'recording'}}}),
  /Stop the recording/);
assert.equal(captureBlock(calibration(), {phase: 'running', state: 'HOLD', task: {recording: {state: 'idle'}}}), null);

assert.equal(countsText(calibration()),
  'Run 20261025T010203Z · Left wrist 3 stills (2 see tags) · Right wrist 1 still');
assert.match(countsText(calibration({run: null, counts: {}})), /^No run yet · Left wrist 0 stills/);

// The last still: its tags, stillness and light, or that it saw no board tags.
assert.equal(lastText({name: '004-left_wrist', tags: [0, 4], moved_mrad: .31, white: 201.4, black: 22.6, r_over_g: 1.021,
  b_over_g: .968, noise: .82}),
  '004-left_wrist: tags 0, 4 · moved 0.31 mrad · white 201, black 23 · R/G 1.02, B/G 0.97 · noise 0.8');
assert.match(lastText({name: '005-right_wrist', tags: [], others: [7]}), /no board tags seen \(only 7\)\. Undo it/);
assert.equal(lastText(null), '');

// A solve: the recommended model first, each camera's errors and mount, the board, and any check.
const camera = {rms_px: .41, holdout_px: .47, stills: 18, focal: [331.2, 332.0], mount_mm: [.4, 35.2, 34.9],
  mount_sigma_mm: [.2, .3, .2], outliers: ['007-left_wrist']};
const lines = resultLines({recommended: 'joint_offsets', notes: ['left_wrist fits better on part_42 (before J6)'],
  models: {plain: {error: 'left_wrist: 4 stills see the board'},
           joint_offsets: {cameras: {left_wrist: camera}, board_mm: [325.1, -158.4, 512.0],
                           left_vs_right: {translation_mm: 1.84, rotation_deg: .123},
                           robot_in_board: {j1_mm: {left_arm_j1: [200.2, -158.4, 593.2],
                                                    right_arm_j1: [450.1, -158.9, 583.4]}, yaw_deg: .31}}}});
assert.equal(lines[0].title, 'with joint offsets (recommended) · Left wrist');
assert.equal(lines[0].text, '0.41 px (held out 0.47) over 18 stills · f 331.2 · mount 0.4, 35.2, 34.9 mm ± 0.2/0.3/0.2'
  + ' · outliers 007-left_wrist');
assert.equal(lines[1].title, 'with joint offsets (recommended) · placement');
assert.equal(lines[1].text, "from tag 0's outer corner (mm): left J1 x 200, y -158, 593 above; right J1 x 450, y -159,"
  + ' 583 above · turned 0.31° · the arms agree within 1.8 mm, 0.12°');
assert.deepEqual(lines[2], {title: 'URDF as is', text: 'left_wrist: 4 stills see the board', bad: true});
assert.deepEqual(lines[3], {title: 'Check', text: 'left_wrist fits better on part_42 (before J6)', bad: true});
assert.deepEqual(resultLines(null), []);
// The head camera: a still needs the live head held or following; the card says which alignment the head uses.
const alignment = (extra = {}) => ({run: '20261031T220000Z', stills: 4, tagged: 3, busy: false, solving: false,
  saved: null, ...extra});
const live = (extra = {}) => ({phase: 'running', receiver_choice: {mode: 'live', arm: 'head'}, head: {state: 'HOLD'},
  ...extra});
assert.match(headCaptureBlock(null, live()), /Helm on Minerva/);
assert.match(headCaptureBlock(alignment(), {phase: 'stopped'}), /live session with the head/);
assert.match(headCaptureBlock(alignment(), live({receiver_choice: {mode: 'virtual'}})), /real head/);
assert.match(headCaptureBlock(alignment(), live({head: {state: 'IDLE'}})), /Enable the head \(E\), then hold \(T\)/);
assert.match(headCaptureBlock(alignment({busy: true}), live()), /Capturing/);
assert.match(headCaptureBlock(alignment(), live({task: {recording: {state: 'finishing'}}})), /Stop the recording/);
assert.equal(headCaptureBlock(alignment(), live()), null);
assert.equal(headCaptureBlock(alignment(), live({head: {state: 'TELEOP'}})), null);
assert.equal(headCountsText(alignment()),
  'Run 20261031T220000Z · Head 4 stills (3 see tags) · no alignment saved: the head uses its CAD mount');
assert.match(headCountsText(alignment({saved: {run: '20261030T010000Z', current: true, fit: {rms_px: .312}}})),
  /in use: run 20261030T010000Z \(0\.31 px\)$/);
assert.match(headCountsText(alignment({saved: {run: '20261030T010000Z', current: false}})),
  /saved for another head reference: the head uses its CAD mount$/);
assert.equal(headLastText({name: '003-head', tags: [0, 3, 4], moved_mrad: 1.53, noise: .8,
  joints_deg: {neck_z: 12.34, neck_y: -45.06}}),
  '003-head: tags 0, 3, 4 · moved 1.53 mrad · noise 0.8 · pan 12.3°, tilt -45.1°');
const head = headResultLines({stills: 10, rms_px: .262, holdout_px: .281, rms_mm: .29, outliers: [],
  distance_m: {median: .667}, mount_mm: [-116.9, -85.3, -44.9], mount_sigma_mm: [.23, .46, .43],
  cad: {translation_mm: 10.38, rotation_deg: 2.731}, pan_axis_in_board_mm: [325.5, -157.1, 618.5],
  spread_deg: {neck_z: 53.8, neck_y: 20.7},
  arms: {run: '20261031T000000Z', board_mm: 13.51, pan_deg: 1.97, other_mm: .99, other_deg: .091},
  notes: ['neck_y moved only 6.2° over the stills: tilt the head more between them (at least 10°)']});
assert.deepEqual(head.map(line => line.title), ['Fit', 'Mount', 'Board', 'Against the arms', 'Check']);
assert.equal(head[0].text, '0.26 px (held out 0.28), 0.3 mm at the board 0.67 m away, over 10 stills');
assert.equal(head[1].text, '-116.9, -85.3, -44.9 mm ± 0.2/0.5/0.4 in the head link · 10.4 mm, 2.73° from the CAD guess');
assert.equal(head[2].text, "the neck's pan axis at x 326, y -157 mm from tag 0's outer corner, 619 mm above the table"
  + ' · turned 54°, tilted 21° over the stills');
assert.equal(head[3].text, "the board's middle 13.5 mm apart (run 20261031T000000Z); a pan zero of 1.97° explains all"
  + ' but 1.0 mm, 0.09°');
assert.equal(head[4].bad, true);
assert.deepEqual(headResultLines({error: 'head: 3 stills see the board'}),
  [{title: 'Solve', text: 'head: 3 stills see the board', bad: true}]);
assert.deepEqual(headResultLines(null), []);
console.log('calibration card ok');
