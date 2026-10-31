import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';

// Minerva's model in the headset (xr_model.js), and the wrist cameras above the hands (xr_hands.js), on the real
// three.js: the display model Helm serves, coarse, its head left out, posed by Helm's guide and placed by M; each
// wrist camera's picture above its hand, facing the eyes, dimmed where the hand was lost.
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register('data:text/javascript,' + encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`), pathToFileURL('./'));
const THREE = await import(three);
const {MODEL_OPACITY, RETRY_MS, RobotModel, headLinks, meshGeometry} = await import(
  new URL('src/robot_platform/helm/static/xr_model.js', root).href);
const {ABOVE_M, LOST_S, WristScreens} = await import(new URL('src/robot_platform/helm/static/xr_hands.js', root).href);
const close = (actual, expected, tolerance = 1e-6) => assert.ok(Math.abs(actual - expected) <= tolerance,
  `${actual} is not ${expected}`);
const at = (vector, expected) => expected.forEach((value, i) => close(vector.toArray()[i], value));

// A model as Helm's viewer API serves it: a base, an arm joint, a coupled finger, and a head on its pan joint.
const MODEL = {id: 'minerva-display', root: 'base', links: ['base', 'arm', 'finger', 'neck', 'camera'],
  joints: [
    {name: 'arm_j1', type: 'revolute', parent: 'base', child: 'arm', origin: {xyz: [0, .1, .5], rpy: [0, 0, 0]},
      axis: [0, 0, 1], mimic: null},
    {name: 'finger', type: 'prismatic', parent: 'arm', child: 'finger', origin: {xyz: [.2, 0, 0], rpy: [0, 0, 0]},
      axis: [1, 0, 0], mimic: {joint: 'arm_j1', multiplier: .01, offset: 0}},
    {name: 'neck_z', type: 'revolute', parent: 'base', child: 'neck', origin: {xyz: [0, 0, 1.1], rpy: [0, 0, 0]},
      axis: [0, 0, 1], mimic: null},
    {name: 'neck_y', type: 'revolute', parent: 'neck', child: 'camera', origin: {xyz: [0, 0, .05], rpy: [-Math.PI / 2, 0, Math.PI / 2]},
      axis: [0, 0, 1], mimic: null}],
  independent: ['arm_j1', 'neck_z', 'neck_y'], mimic_order: ['finger'],
  geometry: ['base', 'arm', 'finger', 'neck', 'camera'].flatMap((link, asset) => [
    {shape: 'mesh', asset, link, role: 'visual', color: [.5, .5, .5, 1], origin: {xyz: [0, 0, 0], rpy: [0, 0, 0]}},
    {shape: 'mesh', asset, link, role: 'collision', color: [1, 1, 1, 1], origin: {xyz: [0, 0, 0], rpy: [0, 0, 0]}}])};
function triangle() {
  const buffer = new ArrayBuffer(8 + 3 * 12 + 12), view = new DataView(buffer);
  view.setUint32(0, 3, true);
  view.setUint32(4, 1, true);
  new Float32Array(buffer, 8, 9).set([0, 0, 0, .1, 0, 0, 0, .1, 0]);
  new Uint32Array(buffer, 44, 3).set([0, 1, 2]);
  return buffer;
}
const asked = [];
let guide = {model: MODEL.id, urdf_sha256: 'abc', positions: {arm_j1: -.3, neck_z: 0, neck_y: 0}, sequence: 1,
  live: true, stale: false};
MODEL.urdf_sha256 = 'abc';
const fetch = async url => {
  asked.push(url);
  const body = url === '/api/models' ? {models: [{id: MODEL.id}], selected: MODEL.id}
    : url === `/api/models/${MODEL.id}` ? MODEL : url === '/api/calibration-guide' ? guide : null;
  return body ? {ok: true, json: async () => body} : {ok: true, arrayBuffer: async () => triangle()};
};
const timers = [];

assert.deepEqual([...headLinks(MODEL)], ['neck', 'camera']);
assert.throws(() => meshGeometry(new ArrayBuffer(12)), /Incomplete/);
const robot = new RobotModel({fetch, setTimer: fn => timers.push(fn), clearTimer: () => {}});
await robot.load();
// Coarse visual meshes only, and none of the head's.
const meshes = asked.filter(url => url.includes('/meshes/'));
assert.deepEqual(meshes, [0, 1, 2].map(i => `/api/models/${MODEL.id}/meshes/${i}?detail=coarse`));
assert.equal(robot.materials.length, 3);
assert.ok(robot.materials.every(material => material.opacity === MODEL_OPACITY));
// Posed by Helm's guide: the arm turns, its coupled finger slides with it.
robot.pose({arm_j1: .5, neck_z: .2, neck_y: 0});
assert.ok(robot.motions.get('arm_j1').quaternion.angleTo(
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), .5)) < 1e-6);
close(robot.motions.get('finger').position.x, .005);
// Hidden until placed; placed by M, it moves as a whole.
assert.equal(robot.group.visible, false);
const M = new THREE.Matrix4().makeTranslation(1, 2, 3);
robot.place(M);
assert.ok(robot.group.visible);
robot.group.updateMatrixWorld(true);
const base = new THREE.Vector3().setFromMatrixPosition(robot.group.children[0].matrixWorld);
assert.deepEqual(base.toArray(), [1, 2, 3]);
robot.place(null);
assert.equal(robot.group.visible, false);
robot.setOpacity(.5);
assert.ok(robot.materials.every(material => material.opacity === MODEL_OPACITY * .5));
// Shown, it follows Helm's guide: each fresh pose once, never a stale one or another model's.
robot.track(true);
await new Promise(resolve => setImmediate(resolve));
assert.ok(robot.motions.get('arm_j1').quaternion.angleTo(
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -.3)) < 1e-6);
assert.equal(timers.length, 1);  // Asks again in GUIDE_MS.
guide = {...guide, positions: {arm_j1: .9, neck_z: 0, neck_y: 0}, sequence: 2, stale: true};
await timers.shift()();
assert.ok(robot.motions.get('arm_j1').quaternion.angleTo(
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -.3)) < 1e-6);  // Stale: kept.
guide = {...guide, stale: false, urdf_sha256: 'other'};
await timers.shift()();
assert.ok(robot.motions.get('arm_j1').quaternion.angleTo(
  new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -.3)) < 1e-6);  // Another CAD revision: kept.
guide = {...guide, urdf_sha256: 'abc'};
robot.track(false);
await timers.shift()();
assert.equal(timers.length, 0);  // Hidden: it stops asking.

// A model Helm cannot serve: asked again only after RETRY_MS, not at each depth frame.
let clock = 0, tries = 0;
const failing = new RobotModel({now: () => clock, fetch: async () => {
  tries++;
  return {ok: false, status: 503, json: async () => ({error: 'Helm is starting'})};
}});
await assert.rejects(failing.load(), /Helm is starting/);
clock += RETRY_MS / 2;
await assert.rejects(failing.load(), /Helm is starting/);
assert.equal(tries, 1);
clock += RETRY_MS;
await assert.rejects(failing.load());
assert.equal(tries, 2);

// The wrist cameras above the hands.
const hands = [new THREE.Group(), new THREE.Group()], grips = [new THREE.Group(), new THREE.Group()];
const renderer = {xr: {getHand: i => hands[i], getControllerGrip: i => grips[i]}};
const screens = new WristScreens(renderer);
hands[0].dispatchEvent({type: 'connected', data: {handedness: 'left', hand: {}}});
hands[1].dispatchEvent({type: 'connected', data: {handedness: 'right', hand: {}}});
for (const hand of hands) {
  hand.joints = {wrist: new THREE.Group()};
  hand.add(hand.joints.wrist);
}
hands[0].joints.wrist.position.set(-.2, 1.0, -.4);
hands[1].joints.wrist.position.set(.2, 1.0, -.4);
const bitmap = {closed: 0, close() { this.closed++; }};
screens.showFrame('left_wrist', bitmap);
screens.showFrame('right_wrist', {close() {}});
const eye = new THREE.Vector3(0, 1.6, 0);
for (const object of [...hands, ...grips]) object.updateMatrixWorld(true);
screens.update(eye, 10);
const left = screens.screens.left.mesh;
assert.ok(left.visible);
at(left.position, [-.2, 1 + ABOVE_M, -.4]);
// Facing the eyes.
const facing = new THREE.Vector3(0, 0, 1).applyQuaternion(left.quaternion);
const toEye = eye.clone().sub(left.position).normalize();
assert.ok(facing.angleTo(toEye) < 1e-6);
assert.equal(screens.screens.left.material.opacity, 1);
// A newer frame releases the old one.
screens.showFrame('left_wrist', {close() {}});
assert.equal(bitmap.closed, 1);
// The left hand lost: its picture stays where it was, dimmed once LOST_S has passed.
hands[0].joints.wrist.visible = false;
screens.update(eye, 10 + LOST_S / 2);
assert.equal(screens.screens.left.material.opacity, 1);
screens.update(eye, 10 + LOST_S * 2);
assert.ok(screens.screens.left.material.opacity < 1 && left.visible);
at(left.position, [-.2, 1 + ABOVE_M, -.4]);
// Without the hand, its controller carries it.
grips[0].dispatchEvent({type: 'connected', data: {handedness: 'left'}});
grips[0].position.set(-.3, .9, -.3);
grips[0].updateMatrixWorld(true);
screens.update(eye, 20);
at(left.position, [-.3, .9 + ABOVE_M, -.3]);
console.log('headset model and wrist cameras: ok');
