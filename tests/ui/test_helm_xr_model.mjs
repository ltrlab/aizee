import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';

// Minerva's model in the headset (xr_model.js), and the wrist cameras' pictures on its wrists (xr_wrists.js), on the
// real three.js: the display model Helm serves, coarse, its head left out, posed by Helm's guide and placed by M; each
// wrist camera's picture where its camera is on the model's link, along its view, so it moves and turns with the
// wrist (the user, 2026-11-03: not floating above the hands, facing the eyes).
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register('data:text/javascript,' + encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`), pathToFileURL('./'));
const THREE = await import(three);
const {MODEL_OPACITY, RETRY_MS, RobotModel, headLinks, meshGeometry} = await import(
  new URL('src/robot_platform/helm/static/xr_model.js', root).href);
const {CAMERAS, WRIST_W, WristScreens, mountMatrix, squared, windowAt} = await import(
  new URL('src/robot_platform/helm/static/xr_wrists.js', root).href);
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
// Hidden until placed; placed by M, it moves as a whole. Placed undrawn (for the wrists' pictures), it is hidden
// but its links are where M puts them.
assert.equal(robot.group.visible, false);
const M = new THREE.Matrix4().makeTranslation(1, 2, 3);
const linked = new THREE.Matrix4();
assert.equal(robot.linkMatrix('arm', linked), null);  // Not placed: no link in the room.
robot.place(M);
assert.ok(robot.group.visible && robot.placed);
robot.group.updateMatrixWorld(true);
const base = new THREE.Vector3().setFromMatrixPosition(robot.group.children[0].matrixWorld);
assert.deepEqual(base.toArray(), [1, 2, 3]);
robot.place(M, false);
assert.ok(!robot.group.visible && robot.placed);
// The arm's link: M, then its joint's origin, turned by arm_j1 (.5 rad about z).
at(new THREE.Vector3().setFromMatrixPosition(robot.linkMatrix('arm', linked)), [1, 2.1, 3.5]);
assert.equal(robot.linkMatrix('nothing', linked), null);
robot.place(null);
assert.ok(!robot.group.visible && !robot.placed);
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

// With the browser's own timers: they throw "Illegal invocation" when called as another object's method (they want
// the window), and the headset's model never followed Helm's guide (2026-10-31). Here as strict as there.
{
  const [plainSet, plainClear] = [globalThis.setTimeout, globalThis.clearTimeout];
  const scheduled = [];
  const strict = (name, act) => function (...args) {
    if (this !== undefined && this !== globalThis) throw new TypeError(`Illegal invocation: ${name}`);
    return act(...args);
  };
  globalThis.setTimeout = strict('setTimeout', (fn, ms) => scheduled.push(ms));
  globalThis.clearTimeout = strict('clearTimeout', () => {});
  try {
    const browser = new RobotModel({fetch});
    await browser.load();
    browser.track(true);
    await new Promise(resolve => setImmediate(resolve));
    assert.equal(scheduled.length, 1);  // Asked again, as the browser lets it.
    browser.track(false);
  } finally {
    [globalThis.setTimeout, globalThis.clearTimeout] = [plainSet, plainClear];
  }
}

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

// The wrist cameras' pictures on the model's wrists: Helm's mounts (T_link_camera, optical axes x right, y down, z
// ahead) on the model's links, each picture WRIST_W wide as far ahead of its lens as fills the camera's view.
const [distance, height] = windowAt([90, 2 * Math.atan(.75) * 180 / Math.PI]);
close(distance, WRIST_W / 2);  // 90 degrees across: half its width ahead.
close(height, WRIST_W * .75);  // 4:3.
assert.deepEqual(CAMERAS, ['left_wrist', 'right_wrist']);
// A camera 35 mm along its link's y, looking along the link's x (a quarter turn about y, then about z).
const MOUNT = [[0, 0, 1, 0], [-1, 0, 0, .035], [0, -1, 0, 0], [0, 0, 0, 1]];
assert.deepEqual(mountMatrix(MOUNT).elements, [0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, .035, 0, 1]);  // Row-major in.
const screens = new WristScreens();
screens.setMounts({left_wrist: {link: 'arm', matrix: MOUNT, fov_deg: [90, 2 * Math.atan(.75) * 180 / Math.PI]},
  right_wrist: {link: 'arm', matrix: 'not a matrix', fov_deg: [90, 70]}, other: {link: 'arm'}});
const left = screens.screens.left_wrist.mesh;
assert.equal(screens.screens.right_wrist.mount, null);  // An unreadable mount: no picture.
close(left.geometry.parameters.width, WRIST_W);
close(left.geometry.parameters.height, WRIST_W * .75);
// The picture upright as the camera takes it: the screen's top left shows the picture's top left (an ImageBitmap's
// rows run top down, uploaded unflipped).
const {position, uv} = left.geometry.attributes;
const corner = [...Array(uv.count).keys()].find(i => position.getX(i) < 0 && position.getY(i) > 0);
assert.deepEqual([uv.getX(corner), uv.getY(corner)], [0, 0]);
// Not placed, or no frame yet: hidden.
robot.pose({arm_j1: 0, neck_z: 0, neck_y: 0});
robot.place(M, false);
screens.update(robot, true);
assert.equal(left.visible, false);
const bitmap = {closed: 0, close() { this.closed++; }};
screens.showFrame('left_wrist', bitmap);
screens.update(robot, false);
assert.equal(left.visible, false);
// Placed and framed: in front of the lens along its view, facing back at it, its right and up the camera's.
screens.update(robot, true);
assert.ok(left.visible);
const lens = new THREE.Vector3(1, 2.135, 3.5);  // M, the arm's origin, the mount's 35 mm along y.
const centre = new THREE.Vector3().setFromMatrixPosition(left.matrix);
at(centre, lens.clone().add(new THREE.Vector3(WRIST_W / 2, 0, 0)).toArray());
const axes = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
left.matrix.extractBasis(...axes);
at(axes[0], [0, -1, 0]);  // Its right: the camera's x (the link's -y).
at(axes[1], [0, 0, 1]);  // Its up: the camera's -y (the link's z).
at(axes[2], [-1, 0, 0]);  // Its face: back at the lens.
// The wrist turns: the picture turns with it.
robot.pose({arm_j1: Math.PI / 2, neck_z: 0, neck_y: 0});
screens.update(robot, true);
at(new THREE.Vector3().setFromMatrixPosition(left.matrix), [1 - .035, 2.1 + WRIST_W / 2, 3.5]);
// The model gone from the room: hidden; the wrists' own opacity.
screens.setOpacity(.4);
screens.update(robot, false);
assert.equal(left.visible, false);
assert.equal(screens.screens.left_wrist.material.opacity, .4);
// A newer frame releases the old one; a frame for no camera is released at once.
screens.showFrame('left_wrist', {close() {}});
assert.equal(bitmap.closed, 1);
const stray = {closed: 0, close() { this.closed++; }};
screens.showFrame('head', stray);
assert.equal(stray.closed, 1);
// Squared to the gripper's axis (the user, 2026-11-03: perpendicular to the wrist, not tilted as the camera): the
// simulation's camera, 35 mm along the link's y and z, looks 37 degrees off the link's y (the gripper's axis); its
// picture looks along y from the lens, keeping the camera's x, so only the tilt goes.
const NOMINAL = [[1, 0, 0, 0], [0, -.606, .795, .035], [0, -.795, -.606, .035], [0, 0, 0, 1]];
const square = squared(mountMatrix(NOMINAL), [0, 1, 0]);
const basis = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
square.extractBasis(...basis);
at(basis[0], [1, 0, 0]);  // The camera's x, already across the gripper.
at(basis[1], [0, 0, -1]);  // The picture's down: the link's -z, where the camera's leaned 37 degrees off it.
at(basis[2], [0, 1, 0]);  // Ahead: along the gripper.
at(new THREE.Vector3().setFromMatrixPosition(square), [0, .035, .035]);  // At the lens.
squared(mountMatrix(NOMINAL), [0, -1, 0]).extractBasis(...basis);
at(basis[2], [0, 1, 0]);  // An axis given the other way: still ahead of the lens.
assert.deepEqual(squared(mountMatrix(NOMINAL), [1, 0, 0]).elements, mountMatrix(NOMINAL).elements);  // Along x.
// On the model: the right wrist's picture in front of its lens along the gripper, facing back up it.
const square2 = new WristScreens();
square2.setMounts({right_wrist: {link: 'arm', matrix: NOMINAL, fov_deg: [90, 2 * Math.atan(.75) * 180 / Math.PI],
  axis: [0, 1, 0]}});
square2.showFrame('right_wrist', {close() {}});
robot.pose({arm_j1: 0, neck_z: 0, neck_y: 0});
robot.place(M, false);
square2.update(robot, true);
const right = square2.screens.right_wrist.mesh;
assert.ok(right.visible);
at(new THREE.Vector3().setFromMatrixPosition(right.matrix), [1, 2.1 + .035 + WRIST_W / 2, 3.5 + .035]);
right.matrix.extractBasis(...basis);
at(basis[0], [1, 0, 0]);  // Its right: the camera's.
at(basis[1], [0, 0, 1]);  // Its up: the link's z, square to the gripper.
at(basis[2], [0, -1, 0]);  // Its face: back up the gripper at the lens.
console.log('headset model and wrist cameras: ok');
