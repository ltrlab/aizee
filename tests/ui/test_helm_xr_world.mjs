import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';

// The headset's 3D picture, world-locked (xr_depth.js, xr_scene.js, on the real three.js): from Helm's zero and the
// eyes as the operator zeroed the head, each depth frame goes where the head camera saw it from in the room, so it
// stays put as the eyes turn; without the head's pose (no session) it follows the eyes, as before.
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
const {depthRig, poseMatrix, robotToWorld, zeroAnchor} = await import(
  new URL('src/robot_platform/helm/static/xr_depth.js', root).href);
const {fillDepth} = await import(new URL('src/robot_platform/helm/static/xr_fill.js', root).href);
const {HeadsetScene} = await import(new URL('src/robot_platform/helm/static/xr_scene.js', root).href);
const near = (actual, expected, tolerance = 1e-6) => actual.toArray().forEach((value, i) =>
  assert.ok(Math.abs(value - expected.toArray()[i]) <= tolerance, `${actual.toArray()} is not ${expected.toArray()}`));

// The head camera at its zero (neck.head_pose.pose7: 32.8, -110.5, 1269.9 mm, looking along the robot's front, -y),
// and panned left about the neck's axis (the robot's z, near enough through its origin).
const ZERO = [.0328, -.1105, 1.2699, 0, Math.SQRT1_2, -Math.SQRT1_2, 0];
function panned(angle) {
  const [position, turn, scale] = [new THREE.Vector3(), new THREE.Quaternion(), new THREE.Vector3()];
  new THREE.Matrix4().makeRotationZ(angle).multiply(poseMatrix(ZERO)).decompose(position, turn, scale);
  return [...position.toArray(), ...turn.toArray()];
}
// The camera's own axes at zero: x right (the robot's -x), y down, z ahead (the robot's front, -y).
near(new THREE.Vector3(0, 0, 1).transformDirection(poseMatrix(ZERO)), new THREE.Vector3(0, -1, 0));
near(new THREE.Vector3(1, 0, 0).transformDirection(poseMatrix(ZERO)), new THREE.Vector3(-1, 0, 0));

const eye = new THREE.Vector3(.1, 1.6, .2);
const zero = {yaw: .3, pitch: -.1};
const anchor = zeroAnchor(eye, zero);
const look = new THREE.Quaternion().setFromEuler(new THREE.Euler(zero.pitch, zero.yaw, 0, 'YXZ'));
const world = robotToWorld(anchor, ZERO);

// At the zero, the depth rig is the eyes as they zeroed (no registration: the depth sensor at the colour camera).
const atZero = depthRig(world, ZERO, null);
near(new THREE.Vector3().setFromMatrixPosition(atZero), eye);
near(new THREE.Vector3(0, 0, -1).transformDirection(atZero), new THREE.Vector3(0, 0, -1).applyQuaternion(look));
near(new THREE.Vector3(0, 1, 0).transformDirection(atZero), new THREE.Vector3(0, 1, 0).applyQuaternion(look));
// Panned left half a radian, it looks half a radian left of where the eyes looked, about the robot's vertical: the
// zero's up (the zero pitched 0.1 down, the robot appears pitched with it, as the head follows the pitch from there).
const turned = depthRig(world, panned(.5), null);
near(new THREE.Vector3(0, 0, -1).transformDirection(turned), new THREE.Vector3(0, 0, -1).applyQuaternion(
  look.clone().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), .5))));
// With the registration, the depth sensor sits where it is: 15 mm to the colour camera's right.
const TO_COLOR = {rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], translation: [.015, 0, 0]};
near(new THREE.Vector3().setFromMatrixPosition(depthRig(world, ZERO, TO_COLOR)),
  eye.clone().add(new THREE.Vector3(.015, 0, 0).applyQuaternion(look)));

// The scene: Helm's zero, the eyes at the next frame anchoring the room, then frames with the head's pose.
const eyes = new THREE.PerspectiveCamera();
const view = new HeadsetScene({xr: {getController: () => new THREE.Group(), getCamera: () => eyes}});
view.setDepth(true);
const pose = (position, yaw, pitch) => {
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, yaw, 0, 'YXZ'));
  return {transform: {position: {x: position.x, y: position.y, z: position.z}, orientation: {x: q.x, y: q.y, z: q.z, w: q.w}}};
};
view.setZero(zero);
view.update(pose(eye, zero.yaw, zero.pitch));
const META = {width: 4, height: 3, scale: .001, intrinsics: {fx: 191, fy: 191, cx: 1.5, cy: 1}, to_color: TO_COLOR};
const layers = fillDepth(new Uint16Array(12).fill(1000), 4, 3);
view.showDepth({meta: {...META, head: {pose: panned(.5), zero: ZERO, joints: {neck_z: .5, neck_y: 0}, skew_ms: 2}},
  layers});
assert.ok(view.worldLocked && view.depthState.locked && view.depth.rig.visible);
const placed = depthRig(world, panned(.5), TO_COLOR);
const rigMatrix = () => view.depth.rig.updateMatrix() || view.depth.rig.matrix;
near(rigMatrix(), placed);
// The eyes turn and move: the 3D stays where the head camera saw it.
view.update(pose(new THREE.Vector3(.3, 1.5, 0), -.6, .2));
near(rigMatrix(), placed);
// Zeroed again elsewhere: the room's anchor moves with the eyes of the next frame.
view.setZero({yaw: -.6, pitch: .2});
const there = new THREE.Vector3(.3, 1.5, 0);
view.update(pose(there, -.6, .2));
near(rigMatrix(), depthRig(robotToWorld(zeroAnchor(there, {yaw: -.6, pitch: .2}), ZERO), panned(.5), TO_COLOR));
// No head pose (the session ended): the 3D is around the eyes again, following them.
view.showDepth({meta: META, layers});
assert.ok(!view.worldLocked);
view.update(pose(eye, .1, 0));
near(view.depth.rig.position, eye);
assert.ok(view.depth.rig.quaternion.angleTo(new THREE.Quaternion().setFromEuler(new THREE.Euler(0, .1, 0, 'YXZ')))
  < 1e-6);
console.log('headset world-locked 3D: ok');
