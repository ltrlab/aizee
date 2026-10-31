import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';

// The headset's picture and its bar (xr_scene.js, on the real three.js): Follow head keeps the picture in front of
// the eyes as the head turns, level, or leaves it where it is; the slider sets its opacity; a trigger on the bar is
// the bar's, anywhere else it zeroes the head; the headset remembers the view (xr_view.js).
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register('data:text/javascript,' + encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`), pathToFileURL('./'));

// Stub canvases: the texts drawn on them.
function context(canvas) {
  const noop = () => {};
  return {canvas, texts: [], clearRect: noop, fillRect: noop, beginPath: noop, roundRect: noop, fill: noop, arc: noop,
    fillText(text) {
      this.texts.push(text);
    }};
}
globalThis.document = {createElement: () => {
  const canvas = {width: 0, height: 0};
  const drawing = context(canvas);
  canvas.getContext = () => drawing;
  return canvas;
}};

const THREE = await import(three);
const {HeadsetScene} = await import(new URL('src/robot_platform/helm/static/xr_scene.js', root).href);
const {CONTROLS, SIZES, controlAt, opacityAt} = await import(new URL('src/robot_platform/helm/static/xr_panels.js', root).href);
const {DEFAULT_VIEW, FOLLOW_OFFSET, STORAGE_KEY, loadView, lookAngles, opacityStep, saveView} =
  await import(new URL('src/robot_platform/helm/static/xr_view.js', root).href);

const close = (actual, expected, tolerance = 1e-6) => assert.ok(Math.abs(actual - expected) <= tolerance,
  `${actual} is not ${expected}`);
const degrees = value => value * Math.PI / 180;

// The view, remembered: defaults for anything missing or unreadable, opacity in 5 % steps.
const store = new Map();
const storage = {getItem: key => store.get(key) ?? null, setItem: (key, value) => store.set(key, value)};
assert.deepEqual(loadView(storage), DEFAULT_VIEW);
assert.deepEqual(loadView(null), DEFAULT_VIEW);
store.set(STORAGE_KEY, '{not json');
assert.deepEqual(loadView(storage), DEFAULT_VIEW);
store.set(STORAGE_KEY, JSON.stringify({follow: false, opacity: .437, offset: [0, 0, .5]}));
assert.deepEqual(loadView(storage), {follow: false, opacity: .45, offset: FOLLOW_OFFSET});  // Behind the eyes: no.
saveView(storage, {follow: true, opacity: 2, offset: [.2, -.1, -1]});
assert.deepEqual(JSON.parse(store.get(STORAGE_KEY)), {follow: true, opacity: 1, offset: [.2, -.1, -1]});
saveView({setItem() { throw new Error('full'); }}, DEFAULT_VIEW);  // Unremembered, not broken.
assert.equal(opacityStep(-1), 0);
assert.equal(opacityStep('x'), 1);
store.clear();

// Where the headset looks: yaw left and pitch up, its roll left out.
const orientation = (yaw, pitch, roll) => new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, yaw, roll, 'YXZ'));
for (const [yaw, pitch, roll] of [[0, 0, 0], [30, -10, 20], [-120, 35, -15], [179, 5, 0]]) {
  const q = orientation(degrees(yaw), degrees(pitch), degrees(roll));
  const [y, p] = lookAngles([q.x, q.y, q.z, q.w]);
  close(y, degrees(yaw));
  close(p, degrees(pitch));
}

// The bar's controls, by where a controller points on it.
assert.deepEqual(controlAt([280, 64]), {control: 'follow'});
assert.deepEqual(controlAt([CONTROLS.track.x + CONTROLS.track.width / 2, 64]), {control: 'opacity', value: .5});
assert.deepEqual(controlAt([CONTROLS.slider.x + 2, 20]), {control: 'opacity', value: 0});  // Just before the track.
assert.equal(controlAt([650, 64]), null);  // The slider's label.
assert.equal(opacityAt(5000), 1);

// The scene, on a stand-in renderer: its controllers are three.js groups, as WebXR's are.
const eyes = new THREE.PerspectiveCamera();
const renderer = {xr: {getController: () => new THREE.Group(), getCamera: () => eyes}};
const view = new HeadsetScene(renderer, {storage});
const picture = view.screen.children[0];
const barTexts = () => view.controls.context.texts.slice(-4);
assert.equal(view.view.follow, true);
assert.ok(barTexts().includes('Follow head · on') && barTexts().includes('100 %'));

function pose(yaw, pitch, roll = 0, position = [.1, 1.6, .2]) {
  const q = orientation(degrees(yaw), degrees(pitch), degrees(roll));
  eyes.position.fromArray(position);
  eyes.quaternion.copy(q);
  eyes.updateMatrixWorld(true);
  return {transform: {position: {x: position[0], y: position[1], z: position[2]},
    orientation: {x: q.x, y: q.y, z: q.z, w: q.w}}};
}

// Following: the picture at its place in front of the eyes, turned with their yaw and pitch, level, facing them.
function inFront(yaw, pitch, offset = FOLLOW_OFFSET) {
  const look = orientation(degrees(yaw), degrees(pitch), 0);
  const relative = view.screen.position.clone().sub(eyes.position).applyQuaternion(look.clone().invert());
  relative.toArray().forEach((value, i) => close(value, offset[i]));
  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(view.screen.quaternion);
  const lookRight = new THREE.Vector3(1, 0, 0).applyQuaternion(look);
  close(right.dot(lookRight), 1);  // No roll of its own: level in the look's frame.
  const facing = new THREE.Vector3(0, 0, 1).applyQuaternion(view.screen.quaternion);
  const toEyes = eyes.position.clone().sub(view.screen.position).normalize();
  assert.ok(facing.dot(toEyes) > .999);
}
view.update(pose(30, -10, 20));
inFront(30, -10);
view.update(pose(-75, 25));
inFront(-75, 25);

// A controller pointed at a point of the bar (canvas pixels), from the eyes.
const controller = view.controllers[0];
const source = {targetRayMode: 'tracked-pointer'};
controller.dispatchEvent({type: 'connected', data: source});
function aimAt(object, local) {
  object.updateWorldMatrix(true, false);
  const target = object.localToWorld(local.clone());
  controller.position.copy(eyes.position);
  controller.quaternion.setFromRotationMatrix(new THREE.Matrix4().lookAt(eyes.position, target, new THREE.Vector3(0, 1, 0)));
  controller.updateMatrixWorld(true);
}
const [barWidth, barHeight] = SIZES.controls;
const onBar = ([x, y]) => aimAt(view.bar, new THREE.Vector3((x / barWidth - .5) * 1.12, (.5 - y / barHeight) * view.bar.geometry.parameters.height, 0));
const away = () => aimAt(view.bar, new THREE.Vector3(0, 3, 0));

// A trigger anywhere but the bar is the head's zero (xr.js sends it).
away();
controller.dispatchEvent({type: 'selectstart'});
assert.equal(view.select(source), false);
controller.dispatchEvent({type: 'selectend'});

// The switch, off: the picture stays where it is as the head turns, and the headset remembers it.
onBar([280, 64]);
view.update(pose(-75, 25));
assert.equal(view.hover, 'follow');
controller.dispatchEvent({type: 'selectstart'});
assert.equal(view.select(source), true);
controller.dispatchEvent({type: 'selectend'});
assert.equal(view.view.follow, false);
assert.ok(barTexts().includes('Follow head · off'));
assert.equal(JSON.parse(store.get(STORAGE_KEY)).follow, false);
const kept = view.screen.position.clone();
view.update(pose(40, 0));
assert.ok(view.screen.position.distanceTo(kept) < 1e-9);

// A press on the bar that slips off before the release is still the bar's: no zero, no switch.
onBar([280, 64]);
controller.dispatchEvent({type: 'selectstart'});
away();
assert.equal(view.select(source), true);
assert.equal(view.view.follow, false);

// The slider: pressed at its middle, dragged to a quarter (off the thin bar, still along it), let go.
onBar([CONTROLS.track.x + CONTROLS.track.width / 2, 64]);
controller.dispatchEvent({type: 'selectstart'});
assert.equal(view.view.opacity, .5);
assert.ok(picture.material.transparent);
close(picture.material.opacity, .5);
onBar([CONTROLS.track.x + CONTROLS.track.width / 4, 64 + 400]);
view.update(pose(40, 0));
assert.equal(view.view.opacity, .25);
assert.equal(view.select(source), true);
controller.dispatchEvent({type: 'selectend'});
assert.equal(JSON.parse(store.get(STORAGE_KEY)).opacity, .25);
assert.ok(barTexts().includes('25 %'));

// Following again: it goes back in front of the eyes. Carried with the grip and let go, it keeps that place there.
onBar([280, 64]);
controller.dispatchEvent({type: 'selectstart'});
view.select(source);
controller.dispatchEvent({type: 'selectend'});
assert.equal(view.view.follow, true);
view.update(pose(40, 0));
inFront(40, 0);
aimAt(picture, new THREE.Vector3(0, 0, 0));
controller.dispatchEvent({type: 'squeezestart'});
assert.equal(controller.userData.held, view.screen);
controller.position.add(new THREE.Vector3(0, .25, 0).applyQuaternion(orientation(degrees(40), 0, 0)));
controller.updateMatrixWorld(true);
view.update(pose(40, 0));  // Carried: it does not follow.
controller.dispatchEvent({type: 'squeezeend'});
const offset = JSON.parse(store.get(STORAGE_KEY)).offset;
close(offset[1], FOLLOW_OFFSET[1] + .25, 1e-3);
close(Math.hypot(offset[0], offset[2]), Math.hypot(FOLLOW_OFFSET[0], FOLLOW_OFFSET[2]), 1e-3);
view.update(pose(-20, 10));
inFront(-20, 10, offset);

// Remembered by the next session's scene.
const again = new HeadsetScene(renderer, {storage});
assert.deepEqual([again.view.follow, again.view.opacity], [true, .25]);
again.view.offset.forEach((value, i) => close(value, offset[i]));
console.log('headset follow and opacity: ok');
