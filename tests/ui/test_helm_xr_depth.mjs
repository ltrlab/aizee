import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';
import {deflateSync} from 'node:zlib';

// The headset's 3D picture: the head camera's frames with their depth (Helm's head_depth role), inflated and filled
// in the headset (xr_camera.js, xr_fill.js), placed on the GPU from the camera's intrinsics and coloured through its
// registration (xr_depth.js), and the bar's 3D switch (xr_scene.js, on the real three.js). A reading outside the
// colour picture is left out, not drawn grey (the user, 2026-11-03: the near ones came out white).
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register('data:text/javascript,' + encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`), pathToFileURL('./'));
const noop = () => {};
globalThis.document = {createElement: () => {
  const canvas = {width: 0, height: 0};
  const drawing = {canvas, texts: [], clearRect: noop, fillRect: noop, beginPath: noop, roundRect: noop, fill: noop,
    arc: noop, fillText(text) { this.texts.push(text); }};
  canvas.getContext = () => drawing;
  return canvas;
}};
const THREE = await import(three);
const {parseCameraPacket} = await import(new URL('src/robot_platform/helm/static/camera_live.js', root).href);
const {HeadCamera, inflateDepth} = await import(new URL('src/robot_platform/helm/static/xr_camera.js', root).href);
const {fillDepth, registrationOf} = await import(new URL('src/robot_platform/helm/static/xr_fill.js', root).href);
const {BACKDROP_M, backdropCorners, colorUv, depthPoint} = await import(
  new URL('src/robot_platform/helm/static/xr_depth.js', root).href);
const {HeadsetScene} = await import(new URL('src/robot_platform/helm/static/xr_scene.js', root).href);
const {CONTROLS, SIZES} = await import(new URL('src/robot_platform/helm/static/xr_panels.js', root).href);
const close = (actual, expected, tolerance = 1e-6) => assert.ok(Math.abs(actual - expected) <= tolerance,
  `${actual} is not ${expected}`);

const JPEG = new Uint8Array([255, 216, 255, 192, 0, 8, 8, 1, 224, 2, 128, 3, 255, 217]);
const K = {fx: 191, fy: 191, cx: 161.2, cy: 118.8, width: 4, height: 3};
const COLOR_K = {fx: 615, fy: 615, cx: 320, cy: 240, width: 640, height: 480};
const TO_COLOR = {rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], translation: [.015, 0, 0]};
const readings = new Uint16Array([0, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 0]);
const packed = new Uint8Array(deflateSync(Buffer.from(readings.buffer)));
const META = {width: 4, height: 3, scale: .001, encoding: 'zlib-u16le', range_m: [.15, 2.5], intrinsics: K,
  color_intrinsics: COLOR_K, to_color: TO_COLOR, bytes: packed.length};

function depthPacket(sequence, meta = META, depth = packed) {
  const header = new TextEncoder().encode(JSON.stringify({role: 'head_depth', sequence, age_s: .05,
    source_timestamp: 1, viewer_heartbeat: 1, jpeg_bytes: JPEG.length, depth: meta}));
  const result = new Uint8Array(4 + header.length + JPEG.length + depth.length);
  new DataView(result.buffer).setUint32(0, header.length);
  result.set(header, 4);
  result.set(JPEG, 4 + header.length);
  result.set(depth, 4 + header.length + JPEG.length);
  return result.buffer;
}

// The packet: the frame's JPEG, then its compressed depth.
const parsed = parseCameraPacket(depthPacket(3));
assert.deepEqual([...parsed.jpeg], [...JPEG]);
assert.deepEqual([...parsed.depth], [...packed]);
assert.equal(parsed.metadata.depth.width, 4);
assert.throws(() => parseCameraPacket(depthPacket(3, {...META, bytes: packed.length + 1})), /Invalid camera depth/);
assert.equal(parseCameraPacket(depthPacket(3)).metadata.role, 'head_depth');

// Inflated in the headset by the browser's own DecompressionStream.
assert.deepEqual([...await inflateDepth(packed, META)], [...readings]);
await assert.rejects(inflateDepth(packed, {...META, width: 5}), /size/);
await assert.rejects(inflateDepth(packed, {...META, encoding: 'png'}), /encoding/);

// The camera asks Helm for the head's frames with their depth, and switching back asks for the frames alone.
const sockets = [];
class Socket {
  constructor(address) {
    Object.assign(this, {address, sent: [], readyState: 0});
    sockets.push(this);
  }
  send(text) { this.sent.push(JSON.parse(text)); }
  open() { this.readyState = 1; this.onopen(); }
  close() {
    if (this.readyState === 3) return;
    this.readyState = 3;
    this.onclose?.();
  }
}
globalThis.setInterval = () => 1;
globalThis.clearInterval = globalThis.clearTimeout = noop;
globalThis.setTimeout = () => 2;
const frames = [];
const camera = new HeadCamera({token: 'secret', role: 'head_depth', url: 'https://minerva:8760/xr',
  open: address => new Socket(address), decode: async () => 'bitmap', now: () => 1000,
  onFrame: (bitmap, metadata, depth) => frames.push({bitmap, sequence: metadata.sequence, depth})});
camera.start();
assert.equal(new URL(sockets[0].address).searchParams.get('roles'), 'head_depth');
sockets[0].open();
sockets[0].onmessage({data: depthPacket(7)});
for (let turn = 0; turn < 200 && !frames.length; turn++) await new Promise(resolve => setImmediate(resolve));
assert.equal(frames.length, 1, camera.message);
assert.equal(frames[0].bitmap, 'bitmap');
// Filled here, as where the browser has no worker: the readings, each its own source, and the one hole filled.
const layers = frames[0].depth.layers;
assert.deepEqual([...layers.foreground.filter((_, i) => i % 2 === 0)].slice(1, 11), [...readings].slice(1, 11));
assert.deepEqual(layers, fillDepth(readings, 4, 3, {registration: registrationOf(META)}));
assert.equal(frames[0].depth.meta.encoding, 'zlib-u16le');
camera.setRole('head');
assert.equal(sockets[0].readyState, 3);
assert.equal(new URL(sockets[1].address).searchParams.get('roles'), 'head');
camera.stop();

// The shader's placement and colouring: a reading's point ahead of the eyes, and its pixel in the color picture.
const [x, y, z] = depthPoint(2, 1, 1.5, K);
close(x, (2 - K.cx) / K.fx * 1.5);
close(y, -(1 - K.cy) / K.fy * 1.5);
close(z, -1.5);
const [u, v] = colorUv([.1, -.05, 1], COLOR_K, TO_COLOR);
close(u, (615 * (.1 + .015) + 320 + .5) / 640);
close(v, (615 * -.05 + 240 + .5) / 480);

// The backdrop: the colour picture's edges at 3 m ahead of the colour camera, 15 mm to the depth sensor's right.
const corners = backdropCorners(COLOR_K, TO_COLOR);
assert.equal(BACKDROP_M, 3);
close(corners[0].position[0], (-.5 - 320) / 615 * 3 - .015);  // Top left: x, then y up and z ahead (three.js).
close(corners[0].position[1], -((-.5 - 240) / 615 * 3));
close(corners[0].position[2], -3);
assert.deepEqual(corners.map(corner => corner.uv), [[0, 0], [1, 0], [1, 1], [0, 1]]);
// Each corner seen from the colour camera lands on the picture's own corner.
for (const corner of corners) {
  const optical = [corner.position[0], -corner.position[1], -corner.position[2]];
  const [cu, cv] = colorUv(optical, COLOR_K, TO_COLOR);
  close(cu, corner.uv[0], 1e-9);
  close(cv, corner.uv[1], 1e-9);
}

// The bar's 3D switch: the flat picture gives way to the depth once its first frame comes, around the eyes.
const eyes = new THREE.PerspectiveCamera();
const renderer = {xr: {getController: () => new THREE.Group(), getHand: () => new THREE.Group(),
  getControllerGrip: () => new THREE.Group(), getCamera: () => eyes}};
const switched = [];
const view = new HeadsetScene(renderer, {onDepth: depth => switched.push(depth)});
assert.equal(view.view.depth, false);
assert.ok(view.picture.visible && !view.depth.rig.visible);
const look = new THREE.Quaternion().setFromEuler(new THREE.Euler(-.2, .5, 0, 'YXZ'));
const pose = {transform: {position: {x: .1, y: 1.6, z: .2}, orientation: {x: look.x, y: look.y, z: look.z, w: look.w}}};
view.update(pose);
const controller = view.controllers[0];
const source = {targetRayMode: 'tracked-pointer'};
controller.dispatchEvent({type: 'connected', data: source});
function aimAtBar([px, py]) {
  view.bar.updateWorldMatrix(true, false);
  const target = view.bar.localToWorld(new THREE.Vector3((px / SIZES.controls[0] - .5) * 1.12,
    (.5 - py / SIZES.controls[1]) * view.bar.geometry.parameters.height, 0));
  controller.position.set(.1, 1.6, .2);
  controller.quaternion.setFromRotationMatrix(new THREE.Matrix4().lookAt(controller.position, target,
    new THREE.Vector3(0, 1, 0)));
  controller.updateMatrixWorld(true);
}
aimAtBar([CONTROLS.depth.x + 100, 64]);
controller.dispatchEvent({type: 'selectstart'});
assert.equal(view.select(source), true);  // The bar's, not the head's zero.
assert.deepEqual(switched, [true]);
assert.ok(view.view.depth && view.picture.visible && !view.depth.rig.visible);  // No depth yet: the picture stays.
assert.ok(view.controls.context.texts.includes('3D · on'));
// Registered, a reading outside the colour picture is discarded; unregistered, it is a dim grey, never near white.
const fragment = view.depth.material.fragmentShader;
assert.match(fragment, /else if \(registered > 0\.5\) \{[^}]*discard;/);
assert.match(fragment, /mix\(0\.45, 0\.2, far\)/);
assert.doesNotMatch(fragment, /0\.85/);
view.showDepth({meta: META, layers});
assert.ok(!view.picture.visible && view.depth.rig.visible);
const uniforms = view.depth.material.uniforms;
assert.deepEqual([uniforms.depthK.value.x, uniforms.depthK.value.z, uniforms.depthScale.value], [191, 161.2, .001]);
assert.equal(uniforms.registered.value, 1);
assert.equal(uniforms.colorMap.value, view.video);  // Coloured from the very picture it came with.
assert.deepEqual(uniforms.toColorT.value.toArray(), [.015, 0, 0]);
assert.deepEqual([...view.depth.textures.foreground.image.data], [...layers.foreground]);
assert.deepEqual([...view.depth.textures.background.image.data], [...layers.background]);
assert.equal(view.depth.behindMaterial.uniforms.depthMap.value, view.depth.textures.background);
assert.equal(view.depth.behindMaterial.uniforms.registered.value, 1);  // Both layers coloured alike.
assert.ok(view.depth.backdrop.visible);  // Where the depth has no reading, the picture fills in behind it.
assert.equal(view.depth.backdropMaterial.map, view.video);
const backdrop = view.depth.backdrop.geometry.attributes.position;
close(backdrop.getZ(0), -3);
close(backdrop.getX(0), corners[0].position[0]);
view.update(pose);  // Following: the rig is the head camera, at the eyes, turned with their look.
assert.deepEqual(view.depth.rig.position.toArray(), [.1, 1.6, .2]);
assert.ok(view.depth.rig.quaternion.angleTo(look) < 1e-6);
view.setOpacity(.4);
assert.equal(uniforms.opacity.value, .4);
assert.ok(view.depth.material.transparent && view.depth.behindMaterial.transparent
  && view.depth.backdropMaterial.transparent);
assert.equal(view.depth.backdropMaterial.opacity, .4);
view.showDepth({meta: {...META, color_intrinsics: undefined, to_color: undefined}, layers});
assert.equal(uniforms.registered.value, 0);  // An older camera, no registration: shaded by depth instead,
assert.ok(!view.depth.backdrop.visible);  // with no picture to place behind it.
aimAtBar([CONTROLS.depth.x + 100, 64]);
controller.dispatchEvent({type: 'selectstart'});
view.select(source);
assert.deepEqual(switched, [true, false]);
assert.ok(view.picture.visible && !view.depth.rig.visible);
console.log('headset 3D picture: ok');
