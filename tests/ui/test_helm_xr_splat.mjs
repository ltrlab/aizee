import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';

// Minerva's splat map in the headset (xr_splat_sort.js, xr_splat.js, on the real three.js): a .splat read into the
// drawing's texture (each Gaussian's covariance from its scales and rotation), ordered back to front for a view, the
// newest map Helm offers that the headset can place, and the map placed by M and Minerva's pose in its world.
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register('data:text/javascript,' + encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`), pathToFileURL('./'));
const THREE = await import(three);
const {ROW, SPLAT_BYTES, parseSplat, sortOrder} = await import(
  new URL('src/robot_platform/helm/static/xr_splat_sort.js', root).href);
const {MAPS_MS, SORT_MS, SplatMap, chooseMap} = await import(
  new URL('src/robot_platform/helm/static/xr_splat.js', root).href);
const close = (actual, expected, tolerance = 1e-6) => assert.ok(Math.abs(actual - expected) <= tolerance,
  `${actual} is not ${expected}`);
const near = (actual, expected, tolerance = 1e-6) => actual.toArray().forEach((value, i) =>
  close(value, expected.toArray()[i], tolerance));

// Two Gaussians as the trainer writes them (splat_bytes): centre, scales, RGBA, rotation (w x y z, q * 128 + 128).
function splats(rows) {
  const buffer = new ArrayBuffer(rows.length * SPLAT_BYTES);
  const floats = new Float32Array(buffer), bytes = new Uint8Array(buffer);
  rows.forEach(({centre, scale, rgba, rotation}, i) => {
    floats.set([...centre, ...scale], i * 8);
    bytes.set([...rgba, ...rotation], i * SPLAT_BYTES + 24);
  });
  return buffer;
}
const BUFFER = splats([
  {centre: [0, 0, -2], scale: [.1, .2, .3], rgba: [255, 0, 0, 255], rotation: [255, 128, 128, 128]},
  // A quarter turn about z (w = z = 0.71): its long x axis lies along y.
  {centre: [1, 0, -4], scale: [.3, .1, .1], rgba: [0, 255, 0, 128], rotation: [219, 128, 128, 219]}]);
const parsed = parseSplat(BUFFER.slice(0));
assert.equal(parsed.count, 2);
assert.equal(parsed.rows, 1);
assert.equal(parsed.data.length, ROW * 4 * 4);
const texel = (i, k) => Array.from(parsed.data.slice(i * 16 + k * 4, i * 16 + k * 4 + 4));
const expect = (actual, expected) => expected.forEach((value, k) => close(actual[k], value, 1e-5));
expect(texel(0, 0), [0, 0, -2, 1]);  // Centre and alpha.
expect(texel(0, 1), [.01, 0, 0, 1]);  // Covariance xx xy xz, red.
expect(texel(0, 2), [.04, 0, .09, 0]);  // yy yz zz, green.
expect(texel(0, 3), [0, 0, 0, 0]);  // Blue.
expect(texel(1, 1), [.01, 0, 0, 0]);
expect(texel(1, 2), [.09, 0, .01, 1]);
close(texel(1, 0)[3], 128 / 255);
expect(Array.from(parsed.centres), [0, 0, -2, 1, 0, -4]);
assert.throws(() => parseSplat(new ArrayBuffer(33)), /whole number of Gaussians/);

// Back to front: a camera at the origin looking down -z (the model-view's third row 0 0 1 0) sees depth -z.
assert.deepEqual(Array.from(sortOrder(new Float32Array([0, 0, -2, 1, 0, -4, 0, 0, -1]), 3, [0, 0, 1, 0])), [1, 0, 2]);
// Turned to look down +x (depth x), the order turns with it.
assert.deepEqual(Array.from(sortOrder(new Float32Array([0, 0, -2, 1, 0, -4, 3, 0, -1]), 3, [-1, 0, 0, 0])), [2, 1, 0]);

// The map to draw: the newest Helm offers whose world the headset can place.
const IDENTITY = new THREE.Matrix4().toArray();
const ON_BOARD = new THREE.Matrix4().makeTranslation(.5, -.2, 0);
const listing = (board = null) => ({maps: [{name: '20261101T020000Z', world: 'board'},
  {name: '20261101T010000Z', world: 'robot'}], robot_in_world: {robot: IDENTITY, board}});
assert.equal(chooseMap(listing()).map.name, '20261101T010000Z');  // The board's world unknown: the robot's map.
assert.equal(chooseMap(listing(ON_BOARD.toArray())).map.name, '20261101T020000Z');
assert.equal(chooseMap({maps: [{name: 'a', world: 'board'}], robot_in_world: {board: null}}), null);
assert.equal(chooseMap(null), null);

// The map from Helm: listed when the switch turns on, loaded, sorted by a worker, placed by M after Minerva's pose
// in its world.
let now = 0, offer = listing(ON_BOARD.toArray());
const requests = [], posts = [];
const fetch = async url => {
  requests.push(url);
  if (url === '/api/helm/maps') return {ok: true, json: async () => offer};
  if (url.endsWith('/map.splat')) return {ok: true, arrayBuffer: async () => BUFFER.slice(0)};
  return {ok: false, status: 404, json: async () => ({error: 'missing'})};
};
const worker = {postMessage: message => posts.push(message), onmessage: null};
const map = new SplatMap({fetch, now: () => now, createWorker: () => worker});
assert.deepEqual(map.state, {on: false, shown: false, name: null, count: 0, error: null, loading: false});
map.show(true);
await map.listing;
assert.deepEqual(requests, ['/api/helm/maps', '/api/helm/maps/20261101T020000Z/map.splat']);
assert.equal(map.name, '20261101T020000Z');
assert.equal(map.count, 2);
assert.equal(posts[0].count, 2);  // The centres, once.
assert.equal(map.mesh.geometry.instanceCount, 2);
assert.equal(map.mesh.frustumCulled, false);
assert.equal(map.group.visible, false);  // No M yet.
const room = new THREE.Matrix4().makeRotationY(.3).setPosition(.1, 1.2, -.4);
map.place(room);
assert.equal(map.group.visible, true);
near(map.group.matrix, room.clone().multiply(ON_BOARD.clone().invert()));
// A sort for the eyes' view, then none until the view changes and SORT_MS has passed.
const eyes = new THREE.Matrix4().setPosition(0, 1.6, 0);
now += SORT_MS + 1;
map.update(eyes);
const request = posts.at(-1);
const viewRow = new THREE.Matrix4().copy(eyes).invert().multiply(map.group.matrix).elements;
assert.deepEqual(request, {id: map.generation, view: [viewRow[2], viewRow[6], viewRow[10], viewRow[14]]});
map.update(eyes);
assert.equal(posts.at(-1), request);  // One in flight.
worker.onmessage({data: {id: map.generation, order: new Uint32Array([1, 0])}});
assert.deepEqual(Array.from(map.mesh.geometry.getAttribute('splatIndex').array), [1, 0]);
now += SORT_MS + 1;
map.update(eyes);
assert.equal(posts.at(-1), request);  // The same view: no sort.
worker.onmessage({data: {id: map.generation - 1, order: new Uint32Array([0, 1])}});  // An earlier map's: ignored.
assert.deepEqual(Array.from(map.mesh.geometry.getAttribute('splatIndex').array), [1, 0]);
// Each eye's focal length and viewport, in pixels.
const eye = new THREE.PerspectiveCamera();
eye.viewport = new THREE.Vector4(0, 0, 1000, 800);
eye.projectionMatrix.elements[0] = 1.2;
eye.projectionMatrix.elements[5] = 1.5;
map.mesh.onBeforeRender(null, null, eye);
const uniforms = map.mesh.material.uniforms;
assert.deepEqual([uniforms.focal.value.x, uniforms.focal.value.y], [600, 600]);
assert.deepEqual([uniforms.viewport.value.x, uniforms.viewport.value.y], [1000, 800]);
map.setOpacity(.4);
close(uniforms.opacity.value, .4);
// Hidden with the switch off or without M; asked again after MAPS_MS, a newer map replaces it.
map.show(false);
assert.equal(map.group.visible, false);
map.show(true);
assert.equal(map.group.visible, true);
map.place(null);
assert.equal(map.group.visible, false);
offer = {...offer, maps: [{name: '20261101T030000Z', world: 'robot'}, ...offer.maps]};
now += MAPS_MS + 1;
map.place(room);
map.update(eyes);
await map.listing;
assert.equal(map.name, '20261101T030000Z');
near(map.group.matrix, room);  // Minerva's own frame: M alone.
assert.equal(posts.at(-1).count, 2);  // The new map's centres.
// No map Helm can place: the panel says why, and nothing is drawn.
const empty = new SplatMap({fetch: async () => ({ok: true, json: async () => ({maps: [], robot_in_world: {}})}),
  now: () => 0, createWorker: () => null});
empty.show(true);
await empty.listing;
assert.equal(empty.state.error, 'no map yet');
assert.equal(empty.group.visible, false);
console.log('headset splat map: ok');
