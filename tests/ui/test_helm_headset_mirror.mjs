import assert from 'node:assert/strict';
import { HeadsetMirror, headsetLink, headsetWord } from '../../src/robot_platform/helm/static/headset_mirror.js';
import { HINTS, panelRows, stripText } from '../../src/robot_platform/helm/static/xr_status.js';

// Settings → Headset draws what the headset shows from Helm's state: the headset's tracking and angles (state.xr) and
// the session's head (state.teleop.head), as the headset's own replies carry them.
const state = {
  mode: 'robot',
  action_token: 'secret',
  xr: { connected: true, tracking: true, rate_hz: 59.6, yaw: 0.21, pitch: -0.09, preview: false },
  teleop: { phase: 'running', head: { state: 'TELEOP', source: 'headset', fault: null } },
};
assert.deepEqual(headsetWord(state), {
  tracking: true,
  rate_hz: 59.6,
  yaw: 0.21,
  pitch: -0.09,
  head: 'TELEOP',
  source: 'headset',
  head_fault: null,
});
assert.deepEqual(headsetWord({}), {
  tracking: false,
  rate_hz: undefined,
  yaw: undefined,
  pitch: undefined,
  head: undefined,
  source: undefined,
  head_fault: undefined,
});
assert.deepEqual(headsetLink(state), { connected: true });
assert.deepEqual(headsetLink({ xr: { connected: false } }), { connected: false, message: 'no headset connected' });

// Stub canvases: what is drawn on them, as text.
function canvas(width, height) {
  const drawn = { texts: [], images: 0, classes: new Set() };
  const element = {
    width,
    height,
    drawn,
    classList: { toggle: (name, on) => (on ? drawn.classes.add(name) : drawn.classes.delete(name)) },
  };
  const context = {
    canvas: element,
    fillRect() {},
    clearRect() {},
    drawImage: () => drawn.images++,
    fillText: (text) => drawn.texts.push(text),
  };
  element.getContext = () => context;
  return element;
}
const picture = canvas(1024, 768),
  strip = canvas(1536, 104),
  panel = canvas(1024, 768);
const cameras = [];
function camera(options) {
  const made = {
    options,
    started: false,
    stopped: false,
    stats: { fresh: true, fps: 9.5, age_s: 0.08, message: null },
    start() {
      this.started = true;
    },
    stop() {
      this.stopped = true;
    },
  };
  cameras.push(made);
  return made;
}
const mirror = new HeadsetMirror({ picture, strip, panel, camera });
assert.deepEqual(picture.drawn.texts, ['Head camera']);

// Off screen: no stream, nothing drawn.
mirror.update(state, false);
assert.equal(cameras.length, 0);
assert.deepEqual(strip.drawn.texts, []);

// On screen: the head camera's stream starts (with Helm's current token), and the screens read as the headset's do.
mirror.update(state, true);
assert.equal(cameras.length, 1);
assert.equal(cameras[0].started, true);
assert.equal(cameras[0].options.token(), 'secret');
assert.equal(picture.drawn.texts.at(-1), 'Waiting for the head camera…');
const stats = cameras[0].stats;
assert.deepEqual(
  strip.drawn.texts,
  stripText(headsetWord(state), stats).map(([text]) => text),
);
const rows = panelRows({ connected: true }, headsetWord(state), stats);
assert.deepEqual(panel.drawn.texts, [
  'Minerva head tracking',
  ...rows.flatMap(([label, value]) => [label, value]),
  ...HINTS,
]);
assert.deepEqual(strip.drawn.texts, ['Head camera · 10 fps · 0.1 s old', 'Following · yaw +12° · pitch −5°']);

// A frame: drawn at its own size, its bitmap released; a stale picture dims; the same stream keeps running.
let closed = 0;
cameras[0].options.onFrame({ width: 640, height: 480, close: () => closed++ });
assert.deepEqual([picture.width, picture.height, picture.drawn.images, closed], [640, 480, 1, 1]);
mirror.update(state, true);
assert.equal(picture.drawn.classes.has('stale'), false);
cameras[0].stats = { fresh: false, fps: 0, age_s: 3.2, message: 'last frame 3 s ago' };
strip.drawn.texts.length = 0;
mirror.update({ ...state, action_token: 'restarted' }, true);
assert.equal(cameras.length, 1);
assert.equal(cameras[0].options.token(), 'restarted'); // A restarted Helm's token, for the next reconnect.
assert.equal(picture.drawn.classes.has('stale'), true);
assert.deepEqual(strip.drawn.texts, ['Head camera: last frame 3 s ago', 'Following · yaw +12° · pitch −5°']);

// Leaving the section stops the stream; a frame still decoding then is released, not drawn.
mirror.update(state, false);
assert.equal(cameras[0].stopped, true);
cameras[0].options.onFrame({ width: 640, height: 480, close: () => closed++ });
assert.deepEqual([picture.drawn.images, closed], [1, 2]);

// Simulation has no physical camera: no stream, and the screens say so. No headset: the panel says that too.
mirror.update({ ...state, mode: 'simulation', xr: { connected: false } }, true);
assert.equal(cameras.length, 1);
assert.equal(strip.drawn.texts.at(-2), 'Head camera: no physical camera in simulation');
assert.ok(panel.drawn.texts.includes('no headset connected'));

console.log("Settings → Headset: the headset's screens from Helm's state, its camera only while on screen.");
