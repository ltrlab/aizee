import assert from 'node:assert/strict';
import {
  GAP_MINUTES,
  HEAD_STILLS,
  WRIST_STILLS,
  cellsSeen,
  coverageGap,
  duration,
  firstStep,
  gapText,
  headHint,
  headNow,
  headProgress,
  headSpread,
  headTarget,
  headVerdict,
  nextStep,
  runGaps,
  sessionChecks,
  stepStatus,
  stepWarning,
  wristProgress,
  wristVerdict,
} from '../../src/robot_platform/helm/static/calibration_guide.js';

// -- a wrist solve's verdict --------------------------------------------------------------------------------------
const camera = (extra = {}) => ({
  rms_px: 0.41,
  holdout_px: 0.47,
  stills: 18,
  mount_sigma_mm: [0.2, 0.3, 0.2],
  outliers: [],
  cells_seen: 11,
  ...extra,
});
const wrists = (cameras, extra = {}) => ({
  recommended: 'joint_offsets',
  solved_at: '2026-11-03T03:00:00+00:00',
  notes: [],
  models: {
    plain: { error: 'left_wrist: 4 stills see the board' },
    joint_offsets: { cameras, left_vs_right: { translation_mm: 1.8, rotation_deg: 0.12 }, ...extra },
  },
});
let verdict = wristVerdict(wrists({ left_wrist: camera(), right_wrist: camera({ rms_px: 0.62 }) }));
assert.equal(verdict.level, 'good');
assert.deepEqual(verdict.reasons, []);
assert.equal(verdict.cameras.right_wrist.px, 0.62);
verdict = wristVerdict(wrists({ left_wrist: camera({ rms_px: 1.9, holdout_px: 2.1 }), right_wrist: camera() }));
assert.equal(verdict.level, 'check');
assert.deepEqual(verdict.reasons, ["Left wrist: 1.90 px, more than a good fit's 1.5."]);
verdict = wristVerdict(
  wrists({
    left_wrist: camera({ rms_px: 3.4, holdout_px: 9, stills: 10, cells_seen: 5, mount_sigma_mm: [7, 1, 1] }),
    right_wrist: camera(),
  }),
);
assert.equal(verdict.level, 'redo');
assert.match(verdict.reasons[0], /^Left wrist: its stills disagree \(3\.40 px; about 1 px is good\)/); // Worst first.
assert.equal(verdict.reasons.length, 5); // Held out, the mount, the stills and the image's parts: checks after it.
verdict = wristVerdict(wrists({ left_wrist: camera() }, { left_vs_right: { translation_mm: 31.2 } }));
assert.equal(verdict.level, 'redo');
assert.match(verdict.reasons[0], /board 31\.2 mm apart: the table or Minerva moved between their stills/);
assert.match(verdict.reasons[1], /^Right wrist: not in this solve/);
verdict = wristVerdict({
  ...wrists({ left_wrist: camera(), right_wrist: camera() }),
  notes: ['left_wrist fits better on part_42 (before J6): check the mount'],
});
assert.equal(verdict.level, 'check');
assert.deepEqual(
  wristVerdict({ recommended: null, models: { plain: { error: 'left_wrist: 4 stills see the board' } } }),
  { level: 'redo', reasons: ['left_wrist: 4 stills see the board'], cameras: {} },
);
assert.equal(wristVerdict(null), null);

// -- the head solve's verdict: the first real head run (2026-11-02), then its last five stills alone ------------------
const mixed = {
  stills: 7,
  rms_px: 21.164,
  holdout_px: null,
  rms_mm: 32.98,
  outliers: [],
  mount_mm: [159.1, 528.1, 468.8],
  mount_sigma_mm: [59.65, 164.57, 62.15],
  cad: { translation_mm: 845.11, rotation_deg: 13.531 },
  spread_deg: { neck_z: 41.0, neck_y: 12.7 },
  distance_m: { median: 0.958 },
  arms: null,
};
verdict = headVerdict(mixed);
assert.equal(verdict.level, 'redo');
assert.deepEqual(verdict.reasons, [
  'The stills disagree (21.16 px; about 1 px is good): most likely the table or Minerva moved between them. Start a new run and take them in one go.',
  'The camera came out 845 mm from where it is fixed on the head: the stills do not pin it down.',
  'The head turned 41° and tilted 13°: about 20° each places its camera better.',
  'Its place is uncertain (±165 mm): add stills turned and tilted further.',
  'Only 7 stills: aim for 10.',
]);
assert.deepEqual(verdict.notes, [
  "It looks 14° off its CAD direction: the camera's bracket or the head's tilt zero (the alignment cannot tell which).",
  'Solve the wrist cameras too: the head alone cannot tell its pan zero from where the board lies.',
]);
verdict = headVerdict({
  ...mixed,
  stills: 5,
  rms_px: 1.136,
  mount_sigma_mm: [10.9, 13.59, 9.78],
  cad: { translation_mm: 125.09, rotation_deg: 18.722 },
  spread_deg: { neck_z: 7.6, neck_y: 7.2 },
});
assert.equal(verdict.level, 'redo');
assert.match(verdict.reasons[0], /^The head turned 8° and tilted 7° across the stills: too little to place its camera/);
assert.match(verdict.reasons[1], /125 mm from where it is fixed/);
const good = {
  stills: 10,
  rms_px: 0.262,
  holdout_px: 0.281,
  outliers: [],
  mount_sigma_mm: [0.23, 0.46, 0.43],
  cad: { translation_mm: 10.38, rotation_deg: 2.731 },
  spread_deg: { neck_z: 53.8, neck_y: 20.7 },
  arms: { run: '20261031T000000Z', board_mm: 13.51, pan_deg: 1.97, other_mm: 0.99, other_deg: 0.091 },
};
verdict = headVerdict(good);
assert.equal(verdict.level, 'good');
assert.deepEqual(verdict.notes, ["Against the arms' board: a pan zero of +1.97°, and the rest within 1.0 mm."]);
assert.equal(headVerdict({ ...good, arms: { ...good.arms, other_mm: 42 } }).level, 'check'); // The board moved since.
assert.deepEqual(headVerdict({ error: 'head: 3 stills see the board' }), {
  level: 'redo',
  reasons: ['head: 3 stills see the board'],
  notes: [],
});

// -- stills far apart in time: the first head run's three stills of two days -----------------------------------
const headTaken = [
  { name: '001-head', tags: 4, at: '2026-11-01T21:28:51+00:00', pan_deg: -18.0, tilt_deg: -26.5 },
  { name: '002-head', tags: 2, at: '2026-11-03T00:25:10+00:00', pan_deg: -26.3, tilt_deg: -23.2 },
  { name: '003-head', tags: 4, at: '2026-11-03T01:54:14+00:00', pan_deg: 7.9, tilt_deg: -31.1 },
  { name: '004-head', tags: 3, at: '2026-11-03T01:55:02+00:00', pan_deg: 11.6, tilt_deg: -28.7 },
];
const gaps = runGaps(headTaken);
assert.deepEqual(
  gaps.map((gap) => [gap.after, gap.before, Math.round(gap.minutes)]),
  [
    ['001-head', '002-head', 1616],
    ['002-head', '003-head', 89],
  ],
);
assert.equal(
  gapText(gaps),
  '27 h passed between 001-head and 002-head (and 1 more gap): if the table or Minerva moved' +
    ' in between, these stills do not fit together. Start a new run (New run) and take them in one go.',
);
assert.deepEqual(runGaps(headTaken.slice(2)), []);
assert.equal(gapText([]), null);
assert.equal(GAP_MINUTES, 30);
assert.deepEqual([duration(45), duration(89.6), duration(130), duration(4000)], ['45 min', '90 min', '2 h', '3 days']);

// -- where a wrist camera's tags have been seen, and how far the head turned -------------------------------------
assert.equal(
  coverageGap([
    [4, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
  ]),
  'top-right corner',
);
assert.equal(
  coverageGap([
    [1, 0, 0, 1],
    [0, 0, 0, 0],
    [1, 0, 0, 1],
  ]),
  'top edge',
);
assert.equal(
  coverageGap([
    [1, 1, 1, 1],
    [1, 1, 1, 1],
    [1, 1, 1, 1],
  ]),
  null,
);
assert.equal(
  cellsSeen([
    [1, 0, 2, 0],
    [0, 0, 0, 0],
    [0, 3, 0, 0],
  ]),
  3,
);
const tonight = [
  { name: '003-head', tags: 4, pan_deg: 7.9, tilt_deg: -31.1 },
  { name: '005-head', tags: 2, pan_deg: 8.7, tilt_deg: -34.0 },
  { name: '006-head', tags: 3, pan_deg: 7.2, tilt_deg: -35.9 },
  { name: '007-head', tags: 3, pan_deg: 14.8, tilt_deg: -31.3 },
  { name: '008-head', tags: 0, pan_deg: 40, tilt_deg: 0 },
]; // Without tags: not part of the spread.
const spread = headSpread(tonight);
assert.deepEqual([spread.count, spread.pan, spread.tilt], [4, [7.2, 14.8], [-35.9, -31.1]]);
// The axis furthest from 20° first (tilted 4.8° here), toward the side the head is on now.
assert.equal(
  headHint(spread, { pan: 14.8, tilt: -31.3 }),
  'tilt the head about 16° further up, farther on the table, still seeing tags',
);
assert.equal(
  headHint({ ...spread, tilted: 25 }, { pan: 14.8, tilt: -31.3 }),
  'turn the head about 13° further left, still seeing tags',
);
assert.equal(
  headHint({ ...spread, tilted: 25 }, { pan: 7.0, tilt: -31.3 }),
  'turn the head about 13° further right, still seeing tags',
);
assert.equal(
  headHint({ ...spread, turned: 25, tilted: 5 }, { pan: 0, tilt: -35 }),
  'tilt the head about 15° further down, nearer on the table, still seeing tags',
);
assert.equal(headHint({ ...spread, turned: 25, tilted: 22 }), null);
assert.match(headHint(null), /three or more tags/);
// The box the plot draws: 20° from the farthest still toward where the hint sends the head, or the stills' own range.
const round = (range) => range.map((value) => Math.round(value * 10) / 10);
let box = headTarget(spread, { pan: 14.8, tilt: -31.3 }); // Left of the middle and above it: further left and up.
assert.deepEqual(
  [round(box.pan), round(box.tilt)],
  [
    [7.2, 27.2],
    [-35.9, -15.9],
  ],
);
box = headTarget(spread, { pan: 7.0, tilt: -35 }); // Right and below: further right and down.
assert.deepEqual(
  [round(box.pan), round(box.tilt)],
  [
    [-5.2, 14.8],
    [-51.1, -31.1],
  ],
);
box = headTarget({ ...spread, pan: [-20, 14], turned: 34 });
assert.deepEqual(box.pan, [-20, 14]);
assert.equal(headTarget(null), null);
const now = headNow({ head: { joints: { neck_z: Math.PI / 6, neck_y: -Math.PI / 4 } } }); // The neck's CAD angles, radians.
assert.deepEqual([Math.round(now.pan * 1e6) / 1e6, Math.round(now.tilt * 1e6) / 1e6], [30, -45]);
assert.equal(headNow({ head: { joints: {} } }), null);

// -- the steps and the next thing to do ---------------------------------------------------------------------------
const live = (extra = {}) => ({
  phase: 'running',
  state: 'TELEOP',
  receiver_choice: { mode: 'live', arm: 'both' },
  head: { state: 'TELEOP', joints: { neck_z: 0.2, neck_y: -0.55 } },
  task: { recording: { state: 'idle' } },
  ...extra,
});
const grid = (filled = 0) =>
  [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [8, 9, 10, 11],
  ].map((row) => row.map((cell) => (cell < filled ? 2 : 0)));
const wristState = (left, right, extra = {}) => ({
  run: '20261103T030000Z',
  cameras: ['left_wrist', 'right_wrist'],
  busy: false,
  solving: false,
  counts: { left_wrist: { stills: left, tagged: left }, right_wrist: { stills: right, tagged: right } },
  coverage: { left_wrist: grid(left), right_wrist: grid(right) },
  result: null,
  taken: [],
  ...extra,
});
const headState = (extra = {}) => ({
  run: '20261103T031000Z',
  stills: 0,
  tagged: 0,
  busy: false,
  solving: false,
  result: null,
  saved: null,
  taken: [],
  ...extra,
});
const twin = (extra = {}) => ({
  busy: false,
  proposal: { notes: [] },
  applied: null,
  drawing: { notes: [] },
  ...extra,
});
const state = (extra = {}) => ({
  teleop: live(),
  calibration: wristState(0, 0),
  head_alignment: headState(),
  twin: twin(),
  ...extra,
});

let statuses = stepStatus(state());
assert.deepEqual(statuses, {
  wrists: { status: 'todo', summary: 'not started' },
  head: { status: 'todo', summary: 'not started' },
  twin: { status: 'todo', summary: 'after the cameras' },
});
assert.equal(firstStep(statuses), 'wrists');
assert.deepEqual(stepStatus({}).wrists, { status: 'todo', summary: 'not on this Helm' });

// Before a session: what to start; the checks say what is missing.
let next = nextStep(state({ teleop: { phase: 'stopped' } }), 'wrists', 'left_wrist');
assert.deepEqual(next, { text: 'Start a live session to capture stills.', tone: 'warn', action: null, label: null });
assert.deepEqual(
  sessionChecks({ phase: 'stopped' }, 'wrists').map((check) => check.ok),
  [false, false, true],
);
assert.deepEqual(sessionChecks(live(), 'head'), [
  { text: 'A live session with the head', ok: true },
  { text: 'The head enabled (E)', ok: true },
  { text: 'Not recording', ok: true },
]);
assert.deepEqual(sessionChecks(live(), 'twin'), []);
// The first still, then the parts of the image left to fill.
next = nextStep(state(), 'wrists', 'left_wrist');
assert.match(next.text, /^Left wrist: bring its camera over the table until it sees tags, then pause \(pedal B or T\)/);
assert.equal(next.action, 'capture');
next = nextStep(state({ calibration: wristState(7, 0) }), 'wrists', 'left_wrist');
assert.equal(next.text, "Left wrist: 7 of 15 stills. Next: tags near the image's bottom-left corner.");
assert.equal(
  nextStep(state({ calibration: wristState(14, 0) }), 'wrists', 'left_wrist').text,
  'Left wrist: 14 of 15 stills. Next: another distance, tilt or roll.',
);
// One camera done: the other; both: Solve; solving.
next = nextStep(state({ calibration: wristState(WRIST_STILLS, 2) }), 'wrists', 'left_wrist');
assert.deepEqual(
  [next.text, next.action, next.label],
  ['Left wrist has its 15 stills. Next: the right wrist.', { role: 'right_wrist' }, 'Right wrist →'],
);
assert.equal(stepStatus(state({ calibration: wristState(WRIST_STILLS, 2) })).wrists.summary, 'left 15, right 2 of 15');
next = nextStep(state({ calibration: wristState(15, 16) }), 'wrists', 'right_wrist');
assert.deepEqual(
  [next.text, next.action],
  ['Both wrist cameras have their stills: Solve (it takes a minute or two).', 'solve'],
);
assert.match(
  nextStep(state({ calibration: wristState(15, 16, { solving: true }) }), 'wrists', 'left_wrist').text,
  /^Solving/,
);
// Solved: good goes on to the head; a still since the solve asks for another.
const solvedAt = '2026-11-03T03:00:00+00:00',
  solved = wristState(15, 16, {
    result: wrists({ left_wrist: camera(), right_wrist: camera({ rms_px: 0.62 }) }),
    taken: [{ name: '031-right_wrist', camera: 'right_wrist', tags: 2, at: '2026-11-03T02:58:00+00:00' }],
  });
next = nextStep(state({ calibration: solved }), 'wrists', 'left_wrist');
assert.deepEqual(
  [next.text, next.tone, next.action],
  ['Wrist cameras calibrated (0.41 and 0.62 px). Next: the head camera.', 'good', { step: 'head' }],
);
assert.deepEqual(stepStatus(state({ calibration: solved })).wrists, { status: 'done', summary: '0.4 · 0.6 px' });
const later = {
  ...solved,
  taken: [...solved.taken, { name: '032-left_wrist', camera: 'left_wrist', tags: 1, at: '2026-11-03T03:05:00+00:00' }],
};
assert.deepEqual(
  nextStep(state({ calibration: later }), 'wrists', 'left_wrist').text,
  '1 stills since the last solve: Solve again.',
);
assert.deepEqual(stepStatus(state({ calibration: later })).wrists, { status: 'active', summary: 'Solve again' });
assert.equal(solvedAt, solved.result.solved_at);
const bad = wristState(15, 16, { result: wrists({ left_wrist: camera({ rms_px: 4.2 }), right_wrist: camera() }) });
assert.deepEqual(
  [
    nextStep(state({ calibration: bad }), 'wrists', 'left_wrist').tone,
    stepStatus(state({ calibration: bad })).wrists.status,
  ],
  ['bad', 'redo'],
);

// The head: blocked without the live head, its spread, the solve, Save, then the twin.
next = nextStep(state({ teleop: live({ receiver_choice: { mode: 'virtual' } }) }), 'head');
assert.deepEqual([next.text, next.tone], ['Head stills need the real head: start a live session.', 'warn']);
next = nextStep(state({ head_alignment: headState({ stills: 4, tagged: 4, taken: tonight.slice(0, 4) }) }), 'head');
assert.equal(
  next.text,
  'Head: 4 of 10 stills, turned 8° and tilted 5° of 20°. Next: tilt the head about 16° further up,' +
    ' farther on the table, still seeing tags.',
);
assert.equal(next.action, 'capture');
const spreadOut = Array.from({ length: HEAD_STILLS }, (_, k) => ({
  name: `0${k}-head`,
  tags: 3,
  at: '2026-11-03T03:20:00+00:00',
  pan_deg: -12 + 3 * k,
  tilt_deg: -45 + 2.5 * k,
}));
next = nextStep(state({ head_alignment: headState({ stills: 10, tagged: 10, taken: spreadOut }) }), 'head');
assert.deepEqual([next.text, next.action], ['Head stills done: Solve.', 'solve']);
const headSolved = headState({
  stills: 10,
  tagged: 10,
  taken: spreadOut,
  result: { ...good, solved_at: '2026-11-03T03:30:00+00:00' },
  run: '20261103T031000Z',
});
next = nextStep(state({ head_alignment: headSolved }), 'head');
assert.deepEqual(
  [next.text, next.action],
  ["Head camera aligned (0.26 px): save it for the headset's 3D view and model.", 'save'],
);
assert.deepEqual(stepStatus(state({ head_alignment: headSolved })).head, {
  status: 'active',
  summary: 'solved: save it',
});
const headSaved = { ...headSolved, saved: { run: '20261103T031000Z', current: true, fit: { rms_px: 0.262 } } };
next = nextStep(state({ head_alignment: headSaved }), 'head');
assert.deepEqual([next.action, next.label], [{ step: 'twin' }, 'Digital twin →']);
assert.deepEqual(stepStatus(state({ head_alignment: headSaved })).head, { status: 'done', summary: 'saved · 0.3 px' });
const headBad = headState({
  stills: 7,
  tagged: 7,
  taken: headTaken,
  result: { ...mixed, solved_at: '2026-11-03T03:30:00+00:00' },
});
assert.equal(nextStep(state({ head_alignment: headBad }), 'head').tone, 'bad');
assert.match(stepWarning(state({ head_alignment: headBad }), 'head'), /^27 h passed between 001-head and 002-head/);
assert.equal(stepWarning(state(), 'wrists'), null);

// The twin: nothing yet, ready, applied.
next = nextStep(state(), 'twin');
assert.deepEqual([next.action, next.label], [{ step: 'wrists' }, 'Wrist cameras →']);
const ready = twin({ proposal: { arms: { run: '20261103T030000Z', offsets_deg: { left_arm_j2: 0.4 } }, notes: [] } });
next = nextStep(state({ twin: ready }), 'twin');
assert.equal(next.action, 'apply');
assert.match(
  next.text,
  /draw the arms where the cameras put them \(drawing only\)\. The head's part comes once its alignment is saved/,
);
assert.deepEqual(stepStatus(state({ twin: ready })).twin, { status: 'active', summary: 'arms ready' });
const both = twin({
  proposal: { ...ready.proposal, head: { alignment_run: '20261103T031000Z', offsets_deg: { neck_z: 1.97 } } },
});
assert.match(
  nextStep(state({ twin: both }), 'twin').text,
  /draw the arms and head where the cameras put them \(drawing only\)\.$/,
);
assert.deepEqual(stepStatus(state({ twin: both })).twin, { status: 'active', summary: 'ready to apply' });
const applied = { ...ready, applied: { current: true, saved_at: '2026-11-03T03:40:00Z' } };
assert.equal(nextStep(state({ twin: applied }), 'twin').tone, 'good');
assert.deepEqual(stepStatus(state({ calibration: solved, head_alignment: headSaved, twin: applied })), {
  wrists: { status: 'done', summary: '0.4 · 0.6 px' },
  head: { status: 'done', summary: 'saved · 0.3 px' },
  twin: { status: 'done', summary: 'applied' },
});
assert.equal(firstStep(stepStatus(state({ calibration: solved, head_alignment: headSaved, twin: applied }))), 'twin');
assert.equal(firstStep(stepStatus(state({ calibration: solved }))), 'head');

// What each camera's progress shows.
const rows = wristProgress(wristState(7, 0));
assert.deepEqual(
  rows.map((row) => [row.role, row.count, row.target, row.seen]),
  [
    ['left_wrist', 7, 15, 7],
    ['right_wrist', 0, 15, 0],
  ],
);
const head = headProgress(headState({ stills: 5, tagged: 4, taken: tonight }), live());
assert.deepEqual(
  [head.count, head.stills, head.target, Math.round(head.turned * 10) / 10, head.points.length],
  [4, 5, 10, 7.6, 5],
);
assert.ok(Math.abs(head.now.pan - 11.459) < 0.01 && Math.abs(head.now.tilt + 31.513) < 0.01);
console.log('calibration guide ok');
