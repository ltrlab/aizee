import assert from 'node:assert/strict';
import {
  INPUTS,
  MODES,
  NO_RECORDING,
  PAUSE_WAIT_MS,
  PauseStill,
  SETTLE_MS,
  STILL_PENDING,
  headsetArm,
  inputChoice,
  knownInput,
  knownMode,
  modeCards,
  modeChoice,
  pauseStillText,
  sessionInput,
  sessionMode,
  stillOutcome,
  stillTaking,
} from '../../src/robot_platform/helm/static/teleop_mode.js';

// The modes, and an unknown or missing choice (a cleared browser) is Direct.
assert.deepEqual(MODES, ['direct', 'calibrate', 'policy']);
assert.equal(knownMode('calibrate'), 'calibrate');
assert.equal(knownMode(null), 'direct');
assert.equal(knownMode('replay'), 'direct');
assert.equal(sessionMode('calibrate', { policyMode: true }), 'policy'); // A running policy session is Policy.
assert.equal(sessionMode('calibrate'), 'calibrate');

// Each mode's cards: Direct records, Calibrate takes stills, Policy chooses a policy; the leaders' trigger feel
// shows whenever leaders drive the arms.
const cards = (task, calibration, policyStart, grip, headset = false) => ({
  task,
  calibration,
  policyStart,
  grip,
  headset,
});
assert.deepEqual(modeCards('direct'), cards(true, false, false, true));
assert.deepEqual(modeCards('calibrate'), cards(false, true, false, true));
// Policy records too: the policy's stretches and the operator's corrections (policy corrections).
assert.deepEqual(modeCards('policy'), cards(true, false, true, true));
assert.deepEqual(modeCards('policy', { policyMode: true }), cards(true, false, true, false));
// A replay shows its own card; the head alone takes head stills but neither records nor runs a policy.
assert.deepEqual(modeCards('calibrate', { replaying: true }), cards(false, false, false, false));
assert.deepEqual(modeCards('calibrate', { headOnly: true }), cards(false, true, false, false));
assert.deepEqual(modeCards('policy', { headOnly: true }), cards(false, false, false, false));
// The headset in the leaders' place: its own card, no trigger feel, and Direct records nothing yet; Calibrate still
// takes stills. Policy, a replay and the head alone take no input.
assert.deepEqual(modeCards('direct', { input: 'headset' }), cards(false, false, false, false, true));
assert.deepEqual(modeCards('calibrate', { input: 'headset' }), cards(false, true, false, false, true));
assert.deepEqual(modeCards('policy', { input: 'headset' }), cards(true, false, true, true));
assert.deepEqual(modeCards('direct', { input: 'headset', replaying: true }), cards(false, false, false, false));
assert.deepEqual(modeCards('direct', { input: 'headset', headOnly: true }), cards(false, false, false, false));

// The inputs; an unknown or missing choice is the leaders. A running session shows its own input.
assert.deepEqual(INPUTS, ['gello', 'headset']);
assert.equal(knownInput('headset'), 'headset');
assert.equal(knownInput(undefined), 'gello');
assert.equal(sessionInput('headset', { phase: 'stopped', headset: { arms: {} } }), 'headset');
assert.equal(sessionInput('gello', { phase: 'running', headset: { arms: {} } }), 'headset');
assert.equal(sessionInput('headset', { phase: 'running', headset: null }), 'gello');
// Choosing: either between sessions for Direct and Calibrate; a session keeps its own; Policy, a replay and the head
// alone take none.
for (const input of INPUTS) assert.equal(inputChoice(input, { phase: 'stopped' }, 'calibrate').allowed, true);
const headsetSession = { phase: 'running', state: 'TELEOP', headset: { arms: {} } };
assert.equal(inputChoice('headset', headsetSession, 'direct').allowed, true);
assert.match(inputChoice('gello', headsetSession, 'direct').reason, /Stop this session/);
assert.match(
  inputChoice('headset', { phase: 'running', state: 'TELEOP', headset: null }, 'direct').reason,
  /Stop this session/,
);
assert.match(inputChoice('headset', { phase: 'stopped' }, 'policy').reason, /policy drives/);
assert.match(inputChoice('headset', { phase: 'stopped' }, 'direct', { headOnly: true }).reason, /head alone/);
assert.match(inputChoice('gello', { phase: 'running', replay: { episode: 'e' } }, 'direct').reason, /replay/);

// Choosing: anything between sessions; Direct and Calibrate in a leader session, not while it records; a policy or
// replay session keeps its own.
const stopped = { phase: 'stopped' },
  leaders = { phase: 'running', state: 'TELEOP', task: { recording: { state: 'idle' } } };
for (const mode of MODES) assert.equal(modeChoice(mode, stopped, 'direct').allowed, true);
assert.equal(modeChoice('calibrate', leaders, 'direct').allowed, true);
assert.equal(modeChoice('direct', leaders, 'calibrate').allowed, true);
assert.match(modeChoice('policy', leaders, 'direct').reason, /policy session/);
const recording = { ...leaders, task: { recording: { active: true, state: 'recording' } } };
assert.match(modeChoice('calibrate', recording, 'direct').reason, /Finish the recording/);
assert.equal(modeChoice('direct', recording, 'direct').allowed, true); // The mode it is in stays pressed and usable.
assert.equal(modeChoice('policy', { phase: 'running', policy: { spec: 'hold_still' } }, 'direct').allowed, true);
assert.match(
  modeChoice('direct', { phase: 'running', policy: { spec: 'hold_still' } }, 'direct').reason,
  /policy session/,
);
assert.match(modeChoice('direct', { phase: 'starting', replay: { episode: 'e' } }, 'direct').reason, /replay/);

// Calibrate's still: a session, a pedal B pause the loop confirmed, Hold reported, SETTLE_MS of Hold, once.
const session = (state, result, extra = {}) => ({
  phase: 'running',
  state,
  fault: null,
  recording_fault: null,
  pedals: { session_id: 's1', last_result: result },
  ...extra,
});
const pause = (id) => ({ press_id: id, pedal: 'b', action: 'pause', accepted: true, error: null });
let still = new PauseStill();
assert.equal(still.due(session('TELEOP', null), 0, 'calibrate'), false); // The page meets the session.
assert.equal(still.remaining(0), null);
assert.equal(still.due(session('HOLD', pause(1)), 100, 'calibrate'), false);
assert.equal(still.remaining(100), SETTLE_MS / 1000);
assert.equal(still.due(session('HOLD', pause(1)), 100 + SETTLE_MS - 1, 'calibrate'), false);
assert.equal(still.due(session('HOLD', pause(1)), 100 + SETTLE_MS, 'calibrate'), true);
assert.equal(still.due(session('HOLD', pause(1)), 100 + 2 * SETTLE_MS, 'calibrate'), false); // Once a pause.
assert.equal(still.remaining(5000), null);
// The next pause in the same session takes the next still.
assert.equal(still.due(session('HOLD', pause(2)), 5000, 'calibrate'), false);
assert.equal(still.due(session('HOLD', pause(2)), 5000 + SETTLE_MS, 'calibrate'), true);

// Resuming before the still takes none; so does a refused pause, a fault, another mode or a new session.
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
still.due(session('HOLD', pause(1)), 10, 'calibrate');
assert.equal(still.due(session('ENGAGING', { ...pause(1), press_id: 2, action: 'resume' }), 500, 'calibrate'), false);
assert.equal(still.due(session('TELEOP', { ...pause(1), press_id: 2, action: 'resume' }), 2000, 'calibrate'), false);
assert.equal(still.due(session('HOLD', { ...pause(3), accepted: false }), 3000, 'calibrate'), false);
assert.equal(still.due(session('HOLD', { ...pause(3), accepted: false }), 3000 + SETTLE_MS, 'calibrate'), false);
for (const [state, extra, mode] of [
  ['HOLD', { fault: 'Receiver faulted' }, 'calibrate'],
  ['HOLD', {}, 'direct'],
  ['HOLD', { recording_fault: 'disk' }, 'calibrate'],
  ['DISABLED', {}, 'calibrate'],
]) {
  still = new PauseStill();
  still.due(session('TELEOP', null), 0, 'calibrate');
  still.due(session('HOLD', pause(1), extra), 10, mode);
  assert.equal(still.due(session(state, pause(1), extra), 10 + SETTLE_MS, mode), false, `${state} ${mode}`);
}
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
still.due(session('HOLD', pause(1)), 10, 'calibrate');
assert.equal(
  still.due(
    { ...session('HOLD', pause(1)), pedals: { session_id: 's2', last_result: pause(1) } },
    10 + SETTLE_MS,
    'calibrate',
  ),
  false,
);

// A page opened on a session that was paused before takes no still for that old pause.
still = new PauseStill();
assert.equal(still.due(session('HOLD', pause(7)), 0, 'calibrate'), false);
assert.equal(still.due(session('HOLD', pause(7)), 5000, 'calibrate'), false);

// A Hold the operator did not ask for (a camera dropping off USB, a head fault) takes no still.
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
assert.equal(
  still.due(session('HOLD', null, { message: 'Paused: the camera at USB 1-2.3.2 dropped off USB' }), 10, 'calibrate'),
  false,
);
assert.equal(still.due(session('HOLD', null), 10 + SETTLE_MS, 'calibrate'), false);

// T while the arms follow is a pause too: the still waits for Hold, then SETTLE_MS; T from Hold engages, no still.
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
still.keyPause(session('TELEOP', null), 100);
assert.equal(still.due(session('TELEOP', null), 150, 'calibrate'), false); // The key's answer is on its way.
assert.equal(still.due(session('HOLD', null), 300, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 300 + SETTLE_MS, 'calibrate'), true);
still.keyPause(session('HOLD', null), 5000);
assert.equal(still.due(session('HOLD', null), 5000 + 2 * SETTLE_MS, 'calibrate'), false);
// A T that never reaches Hold (refused) is forgotten after PAUSE_WAIT_MS, so a later unasked Hold takes no still.
still.keyPause(session('TELEOP', null), 10000);
assert.equal(still.due(session('TELEOP', null), 10000 + PAUSE_WAIT_MS + 1, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 10000 + PAUSE_WAIT_MS + 10, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 10000 + PAUSE_WAIT_MS + 10 + SETTLE_MS, 'calibrate'), false);

// The headset's B/Y holding the arms is a pause too (the session counts the presses); presses counted before the page
// met the session take none.
still = new PauseStill();
const held = (state, holds) => session(state, null, { headset: { hold_requests: holds, arms: {} } });
assert.equal(still.due(held('TELEOP', 2), 0, 'calibrate'), false);
assert.equal(still.due(held('HOLD', 2), 100, 'calibrate'), false);
assert.equal(still.due(held('HOLD', 2), 100 + 2 * SETTLE_MS, 'calibrate'), false);
assert.equal(still.due(held('TELEOP', 2), 3000, 'calibrate'), false);
assert.equal(still.due(held('HOLD', 3), 3100, 'calibrate'), false);
assert.equal(still.due(held('HOLD', 3), 3100 + SETTLE_MS, 'calibrate'), true);

// The card's line: what a pause does, the countdown, that the still is being taken, then what it found.
assert.match(pauseStillText(null, 'head'), /each pause \(pedal B or T\) takes a still of the head camera 1 s later/);
assert.match(pauseStillText(null, 'left_wrist', { input: 'headset' }), /each pause \(B\/Y, pedal B or T\)/);
assert.match(pauseStillText(null, 'head'), /the arms hold until it is saved/);
assert.equal(pauseStillText(0.42, 'right_wrist'), 'Still of the right wrist camera in 0.4 s: hold still.');
assert.equal(
  pauseStillText(null, 'left_wrist', { taking: true }),
  'Taking a still of the left wrist camera: hold still. Resume once it is saved.',
);
assert.equal(
  pauseStillText(0.2, 'left_wrist', { taking: true }),
  'Still of the left wrist camera in 0.2 s: hold still.',
);
const found = stillOutcome({ name: '004-left_wrist', tags: [1, 4] }, null, 'left_wrist');
assert.equal(found, '004-left_wrist saved: tags 1, 4. Resume for the next pose.');
assert.equal(pauseStillText(null, 'left_wrist', { note: found }), found);
assert.equal(
  pauseStillText(null, 'left_wrist', { taking: true, note: found }), // The next still is on its way.
  'Taking a still of the left wrist camera: hold still. Resume once it is saved.',
);
// 2026-11-02: the first live still saw no tags and nothing said so.
assert.equal(
  stillOutcome({ name: '003-left_wrist', tags: [] }, null, 'left_wrist'),
  '003-left_wrist saved, but it saw no board tags: aim the left wrist camera at the board, then pause again (Undo drops it).',
);
assert.equal(
  stillOutcome({ name: '005-right_wrist', tags: [6] }, null, 'right_wrist'),
  '005-right_wrist saved: tag 6. Resume for the next pose.',
);
assert.equal(
  stillOutcome(null, 'The arm moved 2.0 mrad while the frames were taken', 'left_wrist'),
  'No still: The arm moved 2.0 mrad while the frames were taken',
);
assert.equal(stillOutcome(null, null, 'head'), null);
// A still is being taken while this page's request is out, or Helm reports one (another page's).
assert.equal(stillTaking(), false);
assert.equal(stillTaking({ pending: true }), true);
assert.equal(stillTaking({ calibration: { busy: false }, headAlignment: { busy: true } }), true);
assert.equal(stillTaking({ calibration: { busy: false }, headAlignment: null }), false);
assert.equal(STILL_PENDING, 'Taking a still: resume once it is saved'); // helm.teleop STILL_PENDING says the same.
assert.match(NO_RECORDING, /Calibrate records nothing/);

// The headset card's arm lines: waiting, lost, free, dragging, and what holds a dragged arm back.
assert.deepEqual(headsetArm(undefined), { state: 'Waiting for the headset', detail: '', tone: '' });
// Before following nothing tracks: a fresh controller (its trigger reported) is ready, else there is none; while
// Engage aligns, a hanging arm says it rises and any other that it stays.
assert.equal(headsetArm({ tracking: false, trigger: 0 }, { stage: 'held' }).state, 'Ready');
assert.equal(headsetArm({ tracking: false, trigger: null }, { stage: 'held' }).state, 'No controller');
assert.deepEqual(headsetArm({ tracking: false, trigger: 0 }, { stage: 'aligning', raising: true }), {
  state: 'Raising',
  detail: 'To its ready pose at the alignment speed: forearm forward, gripper level',
  tone: 'near',
});
assert.equal(headsetArm({ tracking: false, trigger: 0 }, { stage: 'aligning' }).state, 'Aligning');
assert.equal(headsetArm({ tracking: false, gripped: false }).tone, 'near');
assert.deepEqual(headsetArm({ tracking: true, gripped: false, trigger: 0.25 }), {
  state: 'Free',
  detail: 'Hold the grip to drag this arm · gripper 25% closed',
  tone: '',
});
assert.deepEqual(headsetArm({ tracking: true, gripped: true, converged: true, at_limit: [], trigger: 0 }), {
  state: 'Dragging',
  detail: 'Following the hand · gripper 0% closed',
  tone: '',
});
const pinned = headsetArm({
  tracking: true,
  gripped: true,
  converged: true,
  near_singular: true,
  clamped: true,
  at_limit: ['left_arm_j5'],
  trigger: null,
});
assert.equal(pinned.tone, 'near');
assert.match(pinned.detail, /nearly straight: lift the hand.*at its limit: J5.*held back/);
assert.equal(headsetArm({ tracking: true, gripped: true, converged: false, at_limit: [] }).tone, 'bad');
console.log('teleop mode checks passed');
