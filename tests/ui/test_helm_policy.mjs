import assert from 'node:assert/strict';
import {HOLD_STILL, policyLabel, policyNote, policyOptions, policyStats} from '../../src/robot_platform/helm/static/policy.js';

// The choices: hold_still always, then each exported policy; one Minerva could not read is listed but not offered.
const options = policyOptions({policies: [
  {name: 'act-002', path: '/home/ltr/minerva/policies/act-002', policy_type: 'act', fps: 15, camera_roles: ['left_wrist', 'right_wrist', 'head']},
  {name: 'broken', path: '/home/ltr/minerva/policies/broken', error: 'policy.pt is missing'},
]});
assert.deepEqual(options.map(option => [option.value, option.disabled]), [
  [HOLD_STILL, false], ['lerobot:/home/ltr/minerva/policies/act-002', false], ['lerobot:/home/ltr/minerva/policies/broken', true]]);
assert.match(options[1].label, /^act-002 · act 15 fps · 3 cameras$/);
assert.match(options[2].label, /policy\.pt is missing/);
assert.deepEqual(policyOptions(null).map(option => option.value), [HOLD_STILL]);  // Minerva unreachable: hold_still only.

assert.equal(policyLabel('lerobot:/home/ltr/minerva/policies/act-002'), 'act-002');
assert.equal(policyLabel('lerobot:C:\\policies\\act-003\\'), 'act-003');
assert.equal(policyLabel(HOLD_STILL), HOLD_STILL);

assert.equal(policyStats({loading: true}), 'loading the policy…');
assert.equal(policyStats({chunks: 41, chunk_age_s: .083, inference_ms: 37.4, stale_cycles: 0}), '41 chunks · 0.08 s old · 37 ms');
assert.equal(policyStats({chunks: 3, chunk_age_s: null, inference_ms: null, stale_cycles: 6}), '3 chunks · 6 stale cycles');

assert.match(policyNote('HOLD', {aligned: false}), /^Computing the policy's first action/);
assert.match(policyNote('HOLD', {aligned: true}), /^Engage \(T\) moves the arms to the policy's first action/);
assert.match(policyNote('ENGAGING', {aligned: true}), /^Aligning/);
assert.match(policyNote('TELEOP', {aligned: true}), /^The policy drives the arms/);
assert.match(policyNote('DISABLED', {}), /^Idle \(I\)/);
assert.match(policyNote('HOLD', {loading: true}), /^Loading the policy/);
assert.equal(policyNote('TELEOP', null), '');
console.log('policy mode page checks passed');
