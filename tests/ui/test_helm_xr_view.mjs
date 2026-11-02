import assert from 'node:assert/strict';
import {HINTS, handMessage, otherLinks, panelRows, poseMessage, stripText} from '../../src/robot_platform/helm/static/xr_status.js';
import {HeadCamera} from '../../src/robot_platform/helm/static/xr_camera.js';

// The headset's screens: the line under the head camera's picture, and the status panel.
const following = {tracking: true, rate_hz: 59.6, yaw: .21, pitch: -.09, head: 'TELEOP', source: 'headset'};
const fresh = {fresh: true, fps: 9.5, age_s: .08, message: null};
assert.deepEqual(stripText(following, fresh),
  [['Head camera · 10 fps · 0.1 s old', ''], ['Following · yaw +12° · pitch −5°', 'ok']]);
assert.deepEqual(panelRows({connected: true}, following, fresh), [
  ['Helm', 'connected', 'ok'], ['Tracking', 'yes · 60 Hz', 'ok'], ['Yaw / pitch', '+12.0° / −5.2°', ''],
  ['Head', 'teleop · following the headset', 'ok'], ['Camera', '10 fps · 0.1 s old', 'ok'], ['3D', 'off', ''],
  ['Map', 'off', '']]);
// The 3D picture: waiting for its first depth, fixed in the room from the head's pose, or around the eyes without.
const depthRow = depth => panelRows({connected: true}, following, fresh, depth).at(-2);
assert.deepEqual(depthRow({on: true, shown: false}), ['3D', 'waiting for depth', 'warn']);
assert.deepEqual(depthRow({on: true, shown: true, locked: true}), ['3D', 'fixed in the room', 'ok']);
assert.deepEqual(depthRow({on: true, shown: true, locked: false}),
  ['3D', 'around the eyes: no head pose (no session)', 'warn']);
// The splat map: loading, why none shows, waiting for the world-locked 3D, or which map shows.
const mapRow = map => panelRows({connected: true}, following, fresh, null, map).at(-1);
assert.deepEqual(mapRow({on: true, loading: true}), ['Map', 'loading…', 'warn']);
assert.deepEqual(mapRow({on: true, error: 'no map yet'}), ['Map', 'no map yet', 'warn']);
assert.deepEqual(mapRow({on: true, name: '20261101T010000Z', count: 109367, shown: false}),
  ['Map', 'with the 3D, fixed in the room', 'warn']);
assert.deepEqual(mapRow({on: true, name: '20261101T010000Z', count: 109367, shown: true}),
  ['Map', '20261101T010000Z · 109 k splats', 'ok']);

// A session not yet engaged, none at all, a quiet headset, a head at its zero, a head that failed.
const stale = {fresh: false, fps: 0, age_s: 3.2, message: 'last frame 3 s ago'};
assert.deepEqual(stripText({tracking: true, head: 'HOLD', source: 'zero'}, stale),
  [['Head camera: last frame 3 s ago', 'warn'], ['Head hold: it follows once engaged (T)', 'warn']]);
assert.deepEqual(stripText({}, {fresh: false, message: null})[0], ['Head camera: waiting', 'warn']);
assert.deepEqual(stripText({tracking: false}, stale)[1], ['No teleop session: the head holds still', 'warn']);
assert.deepEqual(stripText({head: 'TELEOP', source: 'holding'}, fresh)[1], ['Holding: the headset went quiet', 'warn']);
assert.deepEqual(stripText({head: 'TELEOP', source: 'zero'}, fresh)[1], ['At its zero', 'warn']);
assert.deepEqual(stripText({head: null, head_fault: 'Head receiver did not start: no bus'}, fresh)[1],
  ['Head fault: Head receiver did not start: no bus', 'bad']);
const lost = panelRows({connected: false, message: 'reconnecting…'}, {tracking: false}, stale);
assert.deepEqual(lost.map(row => row[1]),
  ['reconnecting…', 'no', '— / —', 'no teleop session', 'last frame 3 s ago', 'off', 'off']);
assert.deepEqual(lost.map(row => row[2]), ['warn', 'warn', '', 'warn', 'warn', '', '']);
assert.deepEqual(panelRows({connected: true, error: 'Quest headset is steering the head'}, {}, stale)[0],
  ['Helm', 'Quest headset is steering the head', 'bad']);
assert.equal(HINTS.length, 3);

// The head camera's stream: Helm's camera socket for the head's frames only, heartbeats, the newest frame decoded.
const timers = [];
globalThis.setInterval = (callback, ms) => timers.push({callback, ms, repeat: true}) - 1;
globalThis.setTimeout = (callback, ms) => timers.push({callback, ms, repeat: false}) - 1;
globalThis.clearInterval = globalThis.clearTimeout = id => {
  if (timers[id]) timers[id].cleared = true;
};
const tick = ms => {  // Each live timer of this period fires once.
  for (const timer of [...timers]) if (!timer.cleared && timer.ms === ms) {
    if (!timer.repeat) timer.cleared = true;
    timer.callback();
  }
};

class Socket {
  constructor(address) {
    Object.assign(this, {address, sent: [], readyState: 0});
    sockets.push(this);
  }
  send(text) {
    this.sent.push(JSON.parse(text));
  }
  open() {
    this.readyState = 1;
    this.onopen();
  }
  close() {
    if (this.readyState === 3) return;
    this.readyState = 3;
    this.onclose?.();
  }
}
const sockets = [];
let clock = 1000;
const decodes = [];  // Each decode waits until the test resolves it.
const frames = [];
function packet(sequence, role = 'head', age = .03) {
  const header = new TextEncoder().encode(JSON.stringify({role, sequence, age_s: age, source_timestamp: 1, viewer_heartbeat: 1}));
  const jpeg = new Uint8Array([255, 216, 255, 192, 0, 8, 8, 1, 224, 2, 128, 3]);
  const result = new Uint8Array(4 + header.length + jpeg.length);
  new DataView(result.buffer).setUint32(0, header.length);
  result.set(header, 4);
  result.set(jpeg, 4 + header.length);
  return result.buffer;
}
const settle = () => new Promise(resolve => setImmediate(resolve));

let token = 'secret';
const camera = new HeadCamera({token: () => token, url: 'https://minerva:8760/xr', now: () => clock,
  open: address => new Socket(address),
  decode: blob => new Promise((resolve, reject) => decodes.push({blob, resolve, reject})),
  onFrame: (bitmap, metadata) => frames.push([bitmap, metadata.sequence])});
camera.start();
const [first] = sockets;
const address = new URL(first.address);
assert.equal(`${address.protocol}//${address.host}${address.pathname}`, 'wss://minerva:8760/api/helm/cameras/live');
assert.deepEqual([address.searchParams.get('token'), address.searchParams.get('roles')], ['secret', 'head']);
assert.equal(first.binaryType, 'arraybuffer');
assert.equal(camera.stats.message, 'connecting…');
first.open();
assert.deepEqual(first.sent, [{type: 'heartbeat', sequence: 1}]);  // Frames come only after a heartbeat.
assert.equal(camera.stats.message, 'waiting for a frame');
tick(250);
assert.deepEqual(first.sent.at(-1), {type: 'heartbeat', sequence: 2});

// Helm's status says why there is no picture yet.
first.onmessage({data: JSON.stringify({type: 'status', cameras: {head: {fresh: false, error: 'Head camera is not streaming'}}})});
assert.deepEqual([camera.stats.fresh, camera.stats.message], [false, 'Head camera is not streaming']);

// The first frame decodes and shows; frames arriving meanwhile wait as one, the newest; old and other cameras' drop.
first.onmessage({data: packet(1)});
assert.equal(decodes.length, 1);
assert.equal(decodes[0].blob.type, 'image/jpeg');
for (const sequence of [2, 3, 4]) first.onmessage({data: packet(sequence)});
first.onmessage({data: packet(3)});
first.onmessage({data: packet(5, 'left_wrist')});
clock += 50;
decodes[0].resolve('bitmap 1');
await settle();
assert.deepEqual(frames, [['bitmap 1', 1]]);
assert.equal(decodes.length, 2, 'the newest pending frame decodes next, the ones before it never');
decodes[1].resolve('bitmap 4');
await settle();
assert.deepEqual(frames.at(-1), ['bitmap 4', 4]);
assert.equal(decodes.length, 2);
const shown = camera.stats;
assert.equal(shown.fresh, true);
assert.equal(shown.fps, 2, 'four frames in the two-second window');
assert.ok(Math.abs(shown.age_s - .03) < 1e-9);
assert.equal(shown.message, null);

// A broken packet or an undecodable frame says so and keeps the stream.
first.onmessage({data: new ArrayBuffer(3)});
assert.equal(camera.message, 'Invalid camera packet size');
first.onmessage({data: packet(6)});
decodes[2].reject(new Error('bad JPEG'));
await settle();
assert.equal(camera.message, 'undecodable frame (bad JPEG)');

// No new frame for over a second: the picture is stale and says how old.
clock += 2500;
assert.deepEqual([camera.stats.fresh, camera.stats.message], [false, 'last frame 3 s ago']);

// Five seconds without a word from Helm: the socket closes; two seconds later it connects again, then stops for good.
clock += 5001;
tick(250);
assert.equal(first.readyState, 3);
assert.equal(camera.stats.message, 'last frame 8 s ago');
assert.equal(sockets.length, 1);
token = 'restarted';  // Helm restarted meanwhile: the next socket carries its new token.
tick(2000);
assert.equal(sockets.length, 2);
const second = sockets[1];
assert.equal(new URL(second.address).searchParams.get('token'), 'restarted');
second.open();
assert.deepEqual(second.sent, [{type: 'heartbeat', sequence: 3}], 'heartbeats keep counting up across sockets');
camera.stop();
assert.equal(second.readyState, 3);
tick(2000);
tick(250);
assert.equal(sockets.length, 2, 'a stopped camera never reconnects');
assert.deepEqual(second.sent, [{type: 'heartbeat', sequence: 3}]);

// The pose message: the headset's orientation and, for the arms' grab and drag, each controller (helm/xr.py).
assert.deepEqual(poseMessage({x: 0, y: .1, z: 0, w: .995}, 7), {type: 'pose', seq: 7, q: [0, .1, 0, .995]});
assert.equal('hands' in poseMessage({x: 0, y: 0, z: 0, w: 1}, 8, {}), false);
const pad = {buttons: [{value: .456789, pressed: true}, {value: 1, pressed: true}, {}, {pressed: true}, {},
  {pressed: true}]};
const right = handMessage({x: .123456, y: 1.234567, z: -.4}, {x: 0, y: .7071068, z: 0, w: .7071068}, pad, false);
assert.deepEqual(right, {p: [.1235, 1.2346, -.4], q: [0, .70711, 0, .70711], grip: 1, trigger: .457, b: true,
  ui: false});
// Out-of-range or missing buttons read as 0 to 1; a controller holding a screen says so.
assert.deepEqual(handMessage({x: 0, y: 0, z: 0}, {x: 0, y: 0, z: 0, w: 1}, {buttons: [{value: 1.4}]}, true),
  {p: [0, 0, 0], q: [0, 0, 0, 1], grip: 0, trigger: 1, b: false, ui: true});
// Both hands at their longest stay well inside Helm's 1024-byte socket messages.
const long = handMessage({x: -1.23456, y: 1.23456, z: -1.23456}, {x: -.7071068, y: .7071068, z: -.0000123, w: .0000234},
  {buttons: [{value: .123456}, {value: .987654}, {}, {}, {}, {pressed: true}]}, false);
assert.ok(JSON.stringify(poseMessage({x: -.123456789, y: .987654321, z: -.123456789, w: .0123456789}, 123456,
  {left: long, right: long})).length < 600);
// Three hints fit the panel: the thumbstick zeroes the head now; the trigger is the gripper's.
assert.match(HINTS[0], /Thumbstick/);
// Opened on the network, the page knows the access point's link, and the other way round, for when this one drops.
const links = [{mode: 'network', url: 'https://198.51.100.43:8760/xr'},
  {mode: 'access_point', url: 'https://192.0.2.1:8760/xr'}];
assert.deepEqual(otherLinks(links, '198.51.100.43:8760'), [{where: "Minerva's access point", url: links[1].url}]);
assert.deepEqual(otherLinks(links, '192.0.2.1:8760'), [{where: 'the network', url: links[0].url}]);
assert.deepEqual(otherLinks(links, 'aizee-rover.local:8760').length, 2);
assert.deepEqual(otherLinks(undefined, 'x'), []);
assert.deepEqual(otherLinks([{mode: 'network', url: 'not a url'}], 'x'), []);

console.log('Headset screen text, and the head camera stream: role filter, heartbeats, newest-frame decode, reconnect.');
