import assert from 'node:assert/strict';
import { deflateSync } from 'node:zlib';
import {
  BEHIND_PX,
  DepthFiller,
  EDGE_SHARE,
  SMALL_HOLE_PX,
  fillDepth,
  registrationOf,
} from '../../src/robot_platform/helm/static/xr_fill.js';

// The head camera's depth, filled for the headset's 3D picture (xr_fill.js): the sensor's small dropouts take the
// farthest reading near them, pixels the colour camera cannot see take a seen one's colour, a background layer
// reaches under each near object from its edge as the far surface's plane, and every pixel keeps where its colour
// comes from.
const W = 160,
  H = 20;
const at = (u, v) => v * W + u;
const reading = (layer, u, v) => layer[2 * at(u, v)];
const source = (layer, u, v) => layer[2 * at(u, v) + 1];

// A wall 2 m away, a box 1 m away in front of it (columns 60-89), and a dropout beside the box's left edge.
const values = new Uint16Array(W * H).fill(2000);
for (let v = 0; v < H; v++) for (let u = 60; u < 90; u++) values[at(u, v)] = 1000;
values[at(58, 5)] = values[at(59, 5)] = 0; // The projector's shadow: two pixels of wall without a reading.
for (let v = 0; v < H; v++) for (let u = 150; u < W; u++) values[at(u, v)] = 0; // Out of range: a wide hole.
const { foreground, background, filled, recoloured, behind } = fillDepth(values, W, H);

// Readings are their own source.
assert.equal(reading(foreground, 5, 5), 2000);
assert.equal(source(foreground, 5, 5), at(5, 5));
assert.equal(reading(foreground, 70, 5), 1000);
// The dropout takes the farthest reading within SMALL_HOLE_PX: the wall, not the box, and keeps where it came from.
assert.equal(SMALL_HOLE_PX, 2);
assert.equal(reading(foreground, 59, 5), 2000);
assert.ok(values[source(foreground, 59, 5)] === 2000 && source(foreground, 59, 5) !== at(59, 5));
// The wide hole keeps its middle empty (the colour backdrop shows there); only its edge, SMALL_HOLE_PX deep, fills.
assert.equal(reading(foreground, 151, 5), 2000);
assert.equal(reading(foreground, 152, 5), 0);
assert.equal(filled, 2 + 2 * H);
assert.equal(recoloured, 0); // No registration: every pixel keeps its own colour.

// The background layer: the wall carried under the box from both its edges, its source a wall pixel; none where the
// wall is seen. The wall is flat: so is the layer.
assert.ok(BEHIND_PX >= 30);
for (const u of [60, 70, 80, 89]) assert.equal(Math.round(reading(background, u, 10)), 2000);
assert.equal(values[source(background, 74, 10)], 2000);
assert.ok(source(background, 65, 10) % W < 60); // From the left edge,
assert.ok(source(background, 85, 10) % W >= 90); // and the right one.
assert.equal(reading(background, 10, 5), 0);
// The edge's far side seeds it: there the background is the wall itself.
assert.equal(reading(background, 90, 5), 2000);
assert.equal(source(background, 90, 5), at(90, 5));
assert.equal(behind, 30 * H);

// A box wider than twice BEHIND_PX: its middle has no background.
const wide = new Uint16Array(W * H).fill(2000);
for (let v = 0; v < H; v++) for (let u = 4; u < 156; u++) wide[at(u, v)] = 1000;
const under = fillDepth(wide, W, H).background;
assert.ok(reading(under, 4 + BEHIND_PX - 1, 10) > 0);
assert.equal(reading(under, 4 + BEHIND_PX + 1, 10), 0);
assert.equal(reading(under, 80, 10), 0);

// A slanted wall behind the box: the layer continues its plane (straight in inverse depth), not a flat curtain.
const slanted = new Uint16Array(W * H),
  wallAt = (u) => 1 / (1 / 1500 + u * 2e-6);
for (let v = 0; v < H; v++)
  for (let u = 0; u < W; u++) slanted[at(u, v)] = Math.round(u >= 60 && u < 90 ? 800 : wallAt(u));
const planar = fillDepth(slanted, W, H).background;
for (const u of [62, 70, 80, 88])
  assert.ok(Math.abs(reading(planar, u, 10) - wallAt(u)) < 3, `${u}: ${reading(planar, u, 10)}`);

// A gentle slope is no edge: nothing to carry.
const slope = Uint16Array.from({ length: W * H }, (_, i) => 1000 + (i % W) * 5);
assert.equal(fillDepth(slope, W, H).behind, 0);
assert.ok(5 / 1800 < EDGE_SHARE);

// The colour camera 15 mm beside the depth sensor sees past the box's edge from a little to the side: the wall pixels
// it cannot see, beside that edge, take their colour from the nearest wall pixel along the row it can; the rest keep
// their own. Its view here: 640 x 480 colour pixels for the depth's 160 x 20 (fx 4x the depth's).
const DEPTH_K = { fx: 100, fy: 100, cx: 79.5, cy: 9.5 };
const COLOR_K = { fx: 400, fy: 400, cx: 319.5, cy: 39.5, width: 640, height: 80 };
for (const side of [1, -1]) {
  const registration = {
    depthK: DEPTH_K,
    colorK: COLOR_K,
    scale: 0.001,
    toColor: {
      rotation: [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
      ],
      translation: [side * 0.015, 0, 0],
    },
  };
  const seen = fillDepth(values, W, H, { registration });
  // The box at 1 m, the wall at 2 m: the colour camera's view of the wall shifts by 100 x 0.015 x (1/1 - 1/2), about
  // 0.75 depth pixels, against the box's: at most a pixel or two of wall beside one edge is hidden.
  assert.ok(seen.recoloured > 0 && seen.recoloured <= 2 * 2 * H, `${seen.recoloured}`);
  const hiddenSide = side > 0 ? 90 : 59; // The edge the colour camera looks past.
  const recolouredHere = source(seen.foreground, hiddenSide, 10);
  assert.notEqual(recolouredHere, at(hiddenSide, 10));
  assert.equal(values[recolouredHere], 2000); // Its colour from the wall, never the box.
  assert.equal(source(seen.foreground, side > 0 ? 59 : 90, 10), at(side > 0 ? 59 : 90, 10)); // The other edge: seen.
  assert.equal(source(seen.foreground, 30, 10), at(30, 10));
  assert.equal(values[source(seen.background, 75, 10)], 2000); // The layer carries the wall's colour too.
}
assert.deepEqual(registrationOf({ intrinsics: DEPTH_K, color_intrinsics: COLOR_K, to_color: {}, scale: 0.001 }), {
  depthK: DEPTH_K,
  colorK: COLOR_K,
  toColor: {},
  scale: 0.001,
});
assert.equal(registrationOf({ intrinsics: DEPTH_K, scale: 0.001 }), null); // An older camera: no registration.

// The filler: in a worker where the browser has one (here a stand-in), else in this thread.
const packed = new Uint8Array(deflateSync(Buffer.from(values.buffer)));
const META = { width: W, height: H, encoding: 'zlib-u16le', intrinsics: DEPTH_K, scale: 0.001 };
const here = await new DepthFiller({ createWorker: () => null }).fill(packed, META);
assert.deepEqual(here, fillDepth(values, W, H));
const posted = [];
const worker = {
  postMessage(message) {
    posted.push(message);
    setTimeout(() => this.onmessage({ data: { id: message.id, layers: { filled: 7 } } }));
  },
};
globalThis.Worker ??= class {}; // As in a browser: the filler starts its worker.
const viaWorker = new DepthFiller({ createWorker: () => worker });
assert.deepEqual(await viaWorker.fill(packed, META), { filled: 7 });
assert.equal(posted[0].meta, META);
// A worker that fails: the frame fails, and the next is filled here.
viaWorker.worker.onerror({ preventDefault() {} });
assert.equal(viaWorker.worker, null);
assert.deepEqual(await viaWorker.fill(packed, META), fillDepth(values, W, H));
console.log('headset depth fill: ok');
