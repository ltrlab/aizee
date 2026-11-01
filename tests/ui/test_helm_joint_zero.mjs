import assert from 'node:assert/strict';
import {ARM_JOINTS, jointZeroBlocked, jointZeroResult, jointZeroStatus, pendingLines, zeroChoices}
  from '../../src/robot_platform/helm/static/joint_zero.js';

// The Limits tab's Joint zero card: the arm joints to choose with their readings, when the zero and Verify may run,
// what waits for the power cycle, what Verify recorded and which joints to sweep again.
assert.equal(ARM_JOINTS.length, 12);
assert.deepEqual(ARM_JOINTS.slice(0, 2), ['left_arm_j1', 'left_arm_j2']);

const rows = [{name: 'right_arm_j5', label: 'Right wrist J5', current_rad: .2}, {name: 'left_gripper', label: 'Left gripper'},
  {name: 'left_arm_j5', label: 'Left wrist J5', current_rad: -1.6543}, {name: 'left_arm_j6', label: 'Left wrist J6'}];
assert.deepEqual(zeroChoices(rows).map(choice => [choice.motor, choice.arm, choice.text]),
  [['left_arm_j5', 'left', '−1.654 rad'], ['left_arm_j6', 'left', '—'], ['right_arm_j5', 'right', '+0.200 rad']]);
assert.deepEqual(zeroChoices(null), []);

const zero = (extra = {}) => ({available: true, reason: null, running: null, applied: null, pending: null, resweep: [],
  last: null, error: null, reference: 'minerva-mechanical-reference-20261009', restarts: true, ...extra});
const idle = {joint_zero: zero(), robot: {connection: 'robot'}};
assert.equal(jointZeroBlocked(idle), '');
assert.equal(jointZeroBlocked({}), 'Joint zero runs with Helm on Minerva.');
assert.equal(jointZeroBlocked({joint_zero: zero({available: false, reason: 'Only Helm on Minerva has them'})}),
  'Only Helm on Minerva has them.');
assert.equal(jointZeroBlocked({...idle, teleop: {phase: 'running'}}), 'Stop the teleop session first.');
assert.equal(jointZeroBlocked({...idle, tune: {phase: 'starting'}}), 'Finish or stop the servo sweep first.');
assert.equal(jointZeroBlocked({...idle, check: {running: true}}), 'Wait for Minerva Check to finish.');
assert.equal(jointZeroBlocked({...idle, gripper_zero: {running: 'left_gripper'}}), 'Wait for the gripper zero to finish.');
assert.equal(jointZeroBlocked({joint_zero: zero(), robot: {connection: 'wifi'}}), 'Choose Tether first.');
assert.equal(jointZeroBlocked({joint_zero: zero({running: 'verify'})}), 'Verifying…');
assert.equal(jointZeroBlocked({joint_zero: zero({applied: {reference: 'r2'}})}),
  'Helm is restarting to load the new reference.');
assert.equal(jointZeroBlocked({joint_zero: zero({applied: {reference: 'r2'}, restarts: false})}),
  'Restart Helm to load the new reference.');

const pending = [{motor: 'left_arm_j5', label: 'Left wrist J5', before_rad: -1.654, after_save_rad: 0,
  save_response_missing: true}];
assert.equal(jointZeroStatus(null), '');
assert.equal(jointZeroStatus(zero()), 'Every joint keeps its zero (reference minerva-mechanical-reference-20261009).');
assert.equal(jointZeroStatus(zero({pending})), "Zeroed, waiting for the power cycle: Left wrist J5. Switch the motors' "
  + 'power off and on, leave the arm where it is, then Verify. Nothing that moves the arms starts until then.');
assert.equal(jointZeroStatus(zero({resweep: [{motor: 'left_arm_j5', label: 'Left wrist J5'}]})),
  'Sweep Left wrist J5 again below and save: its limits were cleared when it was zeroed.');
assert.equal(jointZeroStatus(zero({resweep: [{label: 'Left wrist J5'}, {label: 'Left wrist J6'}]})),
  'Sweep Left wrist J5, Left wrist J6 again below and save: their limits were cleared when they were zeroed.');
assert.match(jointZeroStatus(zero({applied: {reference: 'r2'}, pending})), /^Recorded the arms' reference r2\. Helm restarts/);
assert.match(jointZeroStatus(zero({applied: {reference: 'r2'}, restarts: false})), /Restart Helm to load it/);

assert.deepEqual(pendingLines(zero({pending})),
  ['Left wrist J5: read −1.654 rad at the zero pose, now +0.000 rad (its save gave no reply)']);
assert.deepEqual(pendingLines(null), []);

assert.equal(jointZeroResult(zero()), null);
assert.deepEqual(jointZeroResult(zero({last: {action: 'zero', success: true, pending: ['left_arm_j5']}})),
  {text: 'Zeroed and saved 1 joint. Now the power cycle.', bad: false});
assert.deepEqual(jointZeroResult(zero({last: {action: 'zero', success: false, error: 'pose changed',
  pending: ['right_arm_j5']}})), {text: 'The zero failed: pose changed (1 joint was zeroed before it stopped: they '
  + 'wait for the power cycle too)', bad: true});
assert.deepEqual(jointZeroResult(zero({last: {action: 'verify', success: true, reference: 'r2'}})),
  {text: 'Verified: every zeroed joint kept its zero. Recorded r2.', bad: false});
assert.deepEqual(jointZeroResult(zero({last: {action: 'verify', success: false, error: 'Left wrist J5 reads +0.2'}})),
  {text: 'Verify did not pass: Left wrist J5 reads +0.2', bad: true});
console.log('joint zero card ok');
