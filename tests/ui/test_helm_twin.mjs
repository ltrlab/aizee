import assert from 'node:assert/strict';
import { jointLabel, twinCanApply, twinLines, twinStatusText } from '../../src/robot_platform/helm/static/twin.js';

// The Teleop tab's twin card: the proposal from the newest wrist calibration and the head camera's alignment, what is
// applied and whether the 3D views draw it, and when Apply has something new to save.
const arms = {
  run: '20261101T000000Z',
  offsets_deg: { left_arm_j1: 0.4, right_arm_j2: -0.31 },
  sigma_deg: { left_arm_j1: 0.05, right_arm_j2: null },
};
const head = {
  alignment_run: '20261101T010000Z',
  offsets_deg: { neck_z: 1.5 },
  board_mm: 14.2,
  other_mm: 1.84,
  other_deg: 0.21,
};
const twin = (extra = {}) => ({
  busy: false,
  error: null,
  proposal: { wrists_run: '20261101T000000Z', arms, head, notes: [] },
  applied: null,
  drawing: { arms: 0, head: 0, notes: [] },
  ...extra,
});
const applied = (extra = {}) => ({
  saved_at: '2026-11-01T02:03:04+00:00',
  wrists_run: '20261101T000000Z',
  arms,
  head,
  current: true,
  ...extra,
});

assert.equal(jointLabel('left_arm_j1'), 'left J1');
assert.equal(jointLabel('right_arm_j5'), 'right J5');
assert.equal(jointLabel('neck_z'), 'pan');
assert.equal(jointLabel('other'), 'other');

assert.equal(twinStatusText(null), '');
assert.equal(
  twinStatusText(twin()),
  'Not applied: the 3D views draw the measured angles · Apply to draw the ' + 'proposal below',
);
assert.equal(
  twinStatusText(twin({ proposal: { arms: null, head: null, notes: [] } })),
  'Not applied: the 3D views draw the measured angles',
);
assert.equal(
  twinStatusText(twin({ applied: applied(), drawing: { arms: 10, head: 1, notes: [] } })),
  'Applied 2026-11-01 02:03 UTC from wrist run 20261101T000000Z · the 3D views draw 10 arm joints and the pan corrected',
);
assert.equal(
  twinStatusText(twin({ applied: applied({ current: false }), drawing: { arms: 0, head: 0, notes: ['x'] } })),
  'Applied 2026-11-01 02:03 UTC from wrist run 20261101T000000Z · the 3D views draw none of it · the calibrations ' +
    'changed since: Apply again for the proposal below',
);

assert.equal(twinCanApply(twin()), true);
assert.equal(twinCanApply(twin({ busy: true })), false);
assert.equal(
  twinCanApply(twin({ proposal: { arms: null, head: null, notes: ['Head: no head camera alignment saved'] } })),
  false,
);
assert.equal(twinCanApply(twin({ applied: applied() })), false); // Already applied and drawn.
assert.equal(twinCanApply(twin({ applied: applied({ current: false }) })), true);
assert.equal(
  twinCanApply(twin({ applied: applied(), drawing: { arms: 0, head: 0, notes: ['Arms not corrected'] } })),
  true,
);
assert.equal(twinCanApply(null), false);

const lines = twinLines(
  twin({
    proposal: { wrists_run: '20261101T000000Z', arms, head, notes: ['Head: the head has no reference'] },
    drawing: { arms: 0, head: 0, notes: ['Arms not corrected: the arms were zeroed again since it was applied'] },
  }),
);
assert.deepEqual(
  lines.map((line) => [line.title, line.bad]),
  [
    ['Arms (wrist run 20261101T000000Z)', false],
    ['Head (alignment 20261101T010000Z)', false],
    ['', false],
    ['Applied, not drawn:', true],
  ],
);
assert.equal(lines[0].text, 'left J1 +0.40° ±0.05, right J2 −0.31°');
assert.equal(
  lines[1].text,
  "pan +1.50° · the two boards 14.2 mm apart; beside the pan 1.8 mm and 0.21°, the neck's " +
    'place against CAD (not drawn)',
);
assert.equal(lines[2].text, 'Head: the head has no reference');
assert.deepEqual(twinLines(null), []);
console.log('twin card ok');
