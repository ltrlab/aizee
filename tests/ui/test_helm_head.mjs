import assert from 'node:assert/strict';
import {headView, signedDegrees} from '../../src/robot_platform/helm/static/head.js';
import {degrees, poseMessage} from '../../src/robot_platform/helm/static/xr_status.js';

// No session and no headset: no card.
assert.equal(headView(null, null, 'minerva:8760').shown, false);
assert.equal(headView({phase: 'stopped'}, {connected: false}, 'minerva:8760').shown, false);

// A session with a head and no headset: it holds at its zero, and the card says where the headset page is.
const idle = headView({head: {state: 'HOLD', source: 'zero'}}, {connected: false, tracking: false}, 'minerva:8760');
assert.deepEqual([idle.shown, idle.fault, idle.zero], [true, false, false]);
assert.equal(idle.readout, 'Head hold');
assert.equal(idle.headset, 'No headset: the head holds at its zero. On the Quest, open https://minerva:8760/xr');

// Following: where the head's targets come from, and the headset's rate and angles.
const following = headView({head: {state: 'TELEOP', source: 'headset'}},
  {connected: true, tracking: true, device: 'Quest headset', rate_hz: 59.6, yaw: .5, pitch: -.1}, 'minerva:8760');
assert.equal(following.readout, 'Head teleop · following the headset');
assert.equal(following.headset, 'Quest headset tracking · 60 Hz · yaw +29° · pitch -6°');
assert.equal(following.zero, true);
assert.equal(headView({head: {state: 'TELEOP', source: 'holding'}}, {}, 'h').readout,
  'Head teleop · holding: the headset went quiet');
assert.equal(headView({head: {state: 'TELEOP', source: 'zero'}}, {}, 'h').readout, 'Head teleop · at its zero');

// Connected but not tracking (the page is open, the XR session is not).
const open = headView({head: {state: 'IDLE'}}, {connected: true, tracking: false, device: 'Quest headset'}, 'h');
assert.equal(open.headset, 'Quest headset connected: start head tracking in it.');
assert.equal(open.zero, false);

// A head that failed: its fault, as a fault; a headset connected without a session still shows the card.
const failed = headView({head: {state: null, fault: 'Head receiver did not start: no bus'}}, {}, 'h');
assert.deepEqual([failed.readout, failed.fault], ['Head receiver did not start: no bus', true]);
assert.equal(headView(null, {connected: true, tracking: false}, 'h').readout,
  'No session: the head moves only while one follows (T).');
assert.equal(headView({phase: 'running', head: null}, {connected: true}, 'h').readout, 'No head in this session.');
assert.equal(headView(null, {connected: true, tracking: false}, 'h').shown, true);

assert.equal(signedDegrees(0), '+0°');
assert.equal(signedDegrees(-Math.PI / 2), '-90°');

// The headset page's pose message and its angles.
assert.deepEqual(poseMessage({x: .1, y: .2, z: .3, w: .9}, 7), {type: 'pose', seq: 7, q: [.1, .2, .3, .9]});
assert.equal(degrees(null), '—');
assert.equal(degrees(Math.PI / 4), '+45.0°');
