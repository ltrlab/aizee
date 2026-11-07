import assert from 'node:assert/strict';
import { mapCaptureBlock, mapLines, mapStatusText } from '../../src/robot_platform/helm/static/maps.js';

// The Teleop tab's map card: a capture needs the live head engaged; the card says which world captures are placed
// in, how the last went, and lists the maps (the headset draws the newest it can place) and the captures.
const maps = (extra = {}) => ({
  busy: false,
  error: null,
  last: null,
  world: 'board',
  maps: [],
  captures: [],
  ...extra,
});
const live = (extra = {}) => ({
  phase: 'running',
  receiver_choice: { mode: 'live' },
  head: { state: 'TELEOP' },
  ...extra,
});
assert.match(mapCaptureBlock(null, live()), /Helm on Minerva/);
assert.match(mapCaptureBlock(maps({ busy: true }), live()), /look around slowly/);
assert.match(mapCaptureBlock(maps(), { phase: 'stopped' }), /live session with the head/);
assert.match(mapCaptureBlock(maps(), live({ receiver_choice: { mode: 'virtual' } })), /real head/);
assert.match(mapCaptureBlock(maps(), live({ head: { state: 'IDLE' } })), /Enable the head \(E\), then engage it \(T\)/);
assert.match(mapCaptureBlock(maps(), live({ task: { recording: { active: true } } })), /Stop the recording/);
assert.equal(mapCaptureBlock(maps(), live()), null);
assert.equal(mapCaptureBlock(maps(), live({ head: { state: 'HOLD' } })), null);

assert.match(mapStatusText(maps()), /^Captures are placed in the table's frame/);
assert.match(mapStatusText(maps({ world: 'robot' })), /align the head camera to place them on the table$/);
assert.equal(
  mapStatusText(
    maps({
      last: {
        name: '20261101T000000Z',
        frames: 12,
        bytes: 4.2e6,
        counts: { seen: 90, unposed: 0, turning: 9, same_view: 40, rate: 29, dropped: 0 },
      },
    }),
  ),
  "Captures are placed in the table's frame (the head camera is aligned) · last: 20261101T000000Z, 12 frames kept of 90 " +
    '(4.2 MB); skipped 9 turning, 40 same view, 29 rate',
);
assert.equal(mapStatusText(null), '');

const lines = mapLines(
  maps({
    maps: [{ name: '20261101T020000Z', world: 'board', gaussians: 109367, bytes: 3.5e6, psnr: 32.72 }],
    captures: [
      { name: '20261101T020000Z', frames: 40, world: 'board', trained: true },
      { name: '20261101T030000Z', frames: 25, world: 'robot', trained: false },
    ],
  }),
);
assert.deepEqual(
  lines.map((line) => line.title),
  ['Map 20261101T020000Z (newest)', 'Capture 20261101T020000Z', 'Capture 20261101T030000Z'],
);
assert.equal(lines[0].text, "109 k splats, 3.5 MB · 32.7 dB held out · in the table's frame");
assert.equal(lines[1].text, "40 frames in the table's frame · trained");
assert.equal(
  lines[2].text,
  "25 frames in Minerva's frame · train it on the operator station (scripts/splat/train_on_station.py)",
);
assert.deepEqual(mapLines(null), []);
console.log('map card ok');
