import assert from 'node:assert/strict';
import {MODES, NO_RECORDING, PAUSE_WAIT_MS, PauseStill, SETTLE_MS, knownMode, modeCards, modeChoice, pauseStillText,
  sessionMode} from '../../src/robot_platform/helm/static/teleop_mode.js';

// The modes, and an unknown or missing choice (a cleared browser) is Direct.
assert.deepEqual(MODES, ['direct', 'calibrate', 'policy']);
assert.equal(knownMode('calibrate'), 'calibrate');
assert.equal(knownMode(null), 'direct');
assert.equal(knownMode('replay'), 'direct');
assert.equal(sessionMode('calibrate', {policyMode: true}), 'policy');  // A running policy session is Policy.
assert.equal(sessionMode('calibrate'), 'calibrate');

// Each mode's cards: Direct records, Calibrate takes stills, Policy chooses a policy; the leaders' trigger feel
// shows whenever leaders drive the arms.
assert.deepEqual(modeCards('direct'), {task: true, calibration: false, policyStart: false, grip: true});
assert.deepEqual(modeCards('calibrate'), {task: false, calibration: true, policyStart: false, grip: true});
assert.deepEqual(modeCards('policy'), {task: false, calibration: false, policyStart: true, grip: true});
assert.deepEqual(modeCards('policy', {policyMode: true}), {task: false, calibration: false, policyStart: true, grip: false});
// A replay shows its own card; the head alone takes head stills but neither records nor runs a policy.
assert.deepEqual(modeCards('calibrate', {replaying: true}), {task: false, calibration: false, policyStart: false, grip: false});
assert.deepEqual(modeCards('calibrate', {headOnly: true}), {task: false, calibration: true, policyStart: false, grip: false});
assert.deepEqual(modeCards('policy', {headOnly: true}), {task: false, calibration: false, policyStart: false, grip: false});

// Choosing: anything between sessions; Direct and Calibrate in a leader session, not while it records; a policy or
// replay session keeps its own.
const stopped = {phase: 'stopped'}, leaders = {phase: 'running', state: 'TELEOP', task: {recording: {state: 'idle'}}};
for (const mode of MODES) assert.equal(modeChoice(mode, stopped, 'direct').allowed, true);
assert.equal(modeChoice('calibrate', leaders, 'direct').allowed, true);
assert.equal(modeChoice('direct', leaders, 'calibrate').allowed, true);
assert.match(modeChoice('policy', leaders, 'direct').reason, /policy session/);
const recording = {...leaders, task: {recording: {active: true, state: 'recording'}}};
assert.match(modeChoice('calibrate', recording, 'direct').reason, /Finish the recording/);
assert.equal(modeChoice('direct', recording, 'direct').allowed, true);  // The mode it is in stays pressed and usable.
assert.equal(modeChoice('policy', {phase: 'running', policy: {spec: 'hold_still'}}, 'direct').allowed, true);
assert.match(modeChoice('direct', {phase: 'running', policy: {spec: 'hold_still'}}, 'direct').reason, /policy session/);
assert.match(modeChoice('direct', {phase: 'starting', replay: {episode: 'e'}}, 'direct').reason, /replay/);

// Calibrate's still: a session, a pedal B pause the loop confirmed, Hold reported, SETTLE_MS of Hold, once.
const session = (state, result, extra = {}) => ({phase: 'running', state, fault: null, recording_fault: null,
  pedals: {session_id: 's1', last_result: result}, ...extra});
const pause = id => ({press_id: id, pedal: 'b', action: 'pause', accepted: true, error: null});
let still = new PauseStill();
assert.equal(still.due(session('TELEOP', null), 0, 'calibrate'), false);  // The page meets the session.
assert.equal(still.remaining(0), null);
assert.equal(still.due(session('HOLD', pause(1)), 100, 'calibrate'), false);
assert.equal(still.remaining(100), SETTLE_MS / 1000);
assert.equal(still.due(session('HOLD', pause(1)), 100 + SETTLE_MS - 1, 'calibrate'), false);
assert.equal(still.due(session('HOLD', pause(1)), 100 + SETTLE_MS, 'calibrate'), true);
assert.equal(still.due(session('HOLD', pause(1)), 100 + 2 * SETTLE_MS, 'calibrate'), false);  // Once a pause.
assert.equal(still.remaining(5000), null);
// The next pause in the same session takes the next still.
assert.equal(still.due(session('HOLD', pause(2)), 5000, 'calibrate'), false);
assert.equal(still.due(session('HOLD', pause(2)), 5000 + SETTLE_MS, 'calibrate'), true);

// Resuming before the still takes none; so does a refused pause, a fault, another mode or a new session.
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
still.due(session('HOLD', pause(1)), 10, 'calibrate');
assert.equal(still.due(session('ENGAGING', {...pause(1), press_id: 2, action: 'resume'}), 500, 'calibrate'), false);
assert.equal(still.due(session('TELEOP', {...pause(1), press_id: 2, action: 'resume'}), 2000, 'calibrate'), false);
assert.equal(still.due(session('HOLD', {...pause(3), accepted: false}), 3000, 'calibrate'), false);
assert.equal(still.due(session('HOLD', {...pause(3), accepted: false}), 3000 + SETTLE_MS, 'calibrate'), false);
for (const [state, extra, mode] of [['HOLD', {fault: 'Receiver faulted'}, 'calibrate'],
  ['HOLD', {}, 'direct'], ['HOLD', {recording_fault: 'disk'}, 'calibrate'], ['DISABLED', {}, 'calibrate']]) {
  still = new PauseStill();
  still.due(session('TELEOP', null), 0, 'calibrate');
  still.due(session('HOLD', pause(1), extra), 10, mode);
  assert.equal(still.due(session(state, pause(1), extra), 10 + SETTLE_MS, mode), false, `${state} ${mode}`);
}
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
still.due(session('HOLD', pause(1)), 10, 'calibrate');
assert.equal(still.due({...session('HOLD', pause(1)), pedals: {session_id: 's2', last_result: pause(1)}},
  10 + SETTLE_MS, 'calibrate'), false);

// A page opened on a session that was paused before takes no still for that old pause.
still = new PauseStill();
assert.equal(still.due(session('HOLD', pause(7)), 0, 'calibrate'), false);
assert.equal(still.due(session('HOLD', pause(7)), 5000, 'calibrate'), false);

// A Hold the operator did not ask for (a camera dropping off USB, a head fault) takes no still.
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
assert.equal(still.due(session('HOLD', null, {message: 'Paused: the camera at USB 1-2.3.2 dropped off USB'}),
  10, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 10 + SETTLE_MS, 'calibrate'), false);

// T while the arms follow is a pause too: the still waits for Hold, then SETTLE_MS; T from Hold engages, no still.
still = new PauseStill();
still.due(session('TELEOP', null), 0, 'calibrate');
still.keyPause(session('TELEOP', null), 100);
assert.equal(still.due(session('TELEOP', null), 150, 'calibrate'), false);  // The key's answer is on its way.
assert.equal(still.due(session('HOLD', null), 300, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 300 + SETTLE_MS, 'calibrate'), true);
still.keyPause(session('HOLD', null), 5000);
assert.equal(still.due(session('HOLD', null), 5000 + 2 * SETTLE_MS, 'calibrate'), false);
// A T that never reaches Hold (refused) is forgotten after PAUSE_WAIT_MS, so a later unasked Hold takes no still.
still.keyPause(session('TELEOP', null), 10000);
assert.equal(still.due(session('TELEOP', null), 10000 + PAUSE_WAIT_MS + 1, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 10000 + PAUSE_WAIT_MS + 10, 'calibrate'), false);
assert.equal(still.due(session('HOLD', null), 10000 + PAUSE_WAIT_MS + 10 + SETTLE_MS, 'calibrate'), false);

// The card's line: what a pause does, then the countdown.
assert.match(pauseStillText(null, 'head'), /each pause \(pedal B or T\) takes a still of the head camera 1 s later/);
assert.equal(pauseStillText(.42, 'right_wrist'), 'Still of the right wrist camera in 0.4 s: hold still.');
assert.match(NO_RECORDING, /Calibrate records nothing/);
console.log('teleop mode checks passed');
