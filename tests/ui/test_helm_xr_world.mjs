import assert from 'node:assert/strict';
import { register } from 'node:module';
import { pathToFileURL } from 'node:url';

// The headset's 3D picture, world-locked (xr_depth.js, xr_scene.js, on the real three.js): from Helm's zero and the
// eyes as the operator zeroed the head, each depth frame goes where the head camera saw it from in the room, so it
// stays put as the eyes turn; without the head's pose (no session) it follows the eyes, as before.
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register(
  'data:text/javascript,' +
    encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`),
  pathToFileURL('./'),
);
const noop = () => {};
globalThis.document = {
  createElement: () => {
    const canvas = { width: 0, height: 0 };
    const drawing = {
      canvas,
      texts: [],
      clearRect: noop,
      fillRect: noop,
      beginPath: noop,
      roundRect: noop,
      fill: noop,
      arc: noop,
      fillText(text) {
        this.texts.push(text);
      },
    };
    canvas.getContext = () => drawing;
    return canvas;
  },
};
const THREE = await import(three);
const { depthRig, poseMatrix, robotToWorld, zeroAnchor } = await import(
  new URL('src/robot_platform/helm/static/xr_depth.js', root).href
);
const { fillDepth } = await import(new URL('src/robot_platform/helm/static/xr_fill.js', root).href);
const { HeadsetScene } = await import(new URL('src/robot_platform/helm/static/xr_scene.js', root).href);
const near = (actual, expected, tolerance = 1e-6) =>
  actual
    .toArray()
    .forEach((value, i) =>
      assert.ok(
        Math.abs(value - expected.toArray()[i]) <= tolerance,
        `${actual.toArray()} is not ${expected.toArray()}`,
      ),
    );

// The head camera at its zero (neck.head_pose.pose7: 32.8, -110.5, 1269.9 mm, looking along the robot's front, -y),
// and panned left about the neck's axis (the robot's z, near enough through its origin).
const ZERO = [0.0328, -0.1105, 1.2699, 0, Math.SQRT1_2, -Math.SQRT1_2, 0];
function panned(angle) {
  const [position, turn, scale] = [new THREE.Vector3(), new THREE.Quaternion(), new THREE.Vector3()];
  new THREE.Matrix4().makeRotationZ(angle).multiply(poseMatrix(ZERO)).decompose(position, turn, scale);
  return [...position.toArray(), ...turn.toArray()];
}
// The camera's own axes at zero: x right (the robot's -x), y down, z ahead (the robot's front, -y).
near(new THREE.Vector3(0, 0, 1).transformDirection(poseMatrix(ZERO)), new THREE.Vector3(0, -1, 0));
near(new THREE.Vector3(1, 0, 0).transformDirection(poseMatrix(ZERO)), new THREE.Vector3(-1, 0, 0));

const eye = new THREE.Vector3(0.1, 1.6, 0.2);
const zero = { yaw: 0.3, pitch: -0.1 };
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
const turned = depthRig(world, panned(0.5), null);
near(
  new THREE.Vector3(0, 0, -1).transformDirection(turned),
  new THREE.Vector3(0, 0, -1).applyQuaternion(
    look.clone().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), 0.5)),
  ),
);
// With the registration, the depth sensor sits where it is: 15 mm to the colour camera's right.
const TO_COLOR = {
  rotation: [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ],
  translation: [0.015, 0, 0],
};
near(
  new THREE.Vector3().setFromMatrixPosition(depthRig(world, ZERO, TO_COLOR)),
  eye.clone().add(new THREE.Vector3(0.015, 0, 0).applyQuaternion(look)),
);

// The scene: Helm's zero, the eyes at the next frame anchoring the room, then frames with the head's pose.
const eyes = new THREE.PerspectiveCamera();
const view = new HeadsetScene({
  xr: {
    getController: () => new THREE.Group(),
    getHand: () => new THREE.Group(),
    getControllerGrip: () => new THREE.Group(),
    getCamera: () => eyes,
  },
});
view.setDepth(true);
const pose = (position, yaw, pitch) => {
  const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, yaw, 0, 'YXZ'));
  return {
    transform: {
      position: { x: position.x, y: position.y, z: position.z },
      orientation: { x: q.x, y: q.y, z: q.z, w: q.w },
    },
  };
};
view.setZero(zero);
view.update(pose(eye, zero.yaw, zero.pitch));
const META = {
  width: 4,
  height: 3,
  scale: 0.001,
  intrinsics: { fx: 191, fy: 191, cx: 1.5, cy: 1 },
  to_color: TO_COLOR,
};
const layers = fillDepth(new Uint16Array(12).fill(1000), 4, 3);
view.showDepth({
  meta: { ...META, head: { pose: panned(0.5), zero: ZERO, joints: { neck_z: 0.5, neck_y: 0 }, skew_ms: 2 } },
  layers,
});
assert.ok(view.worldLocked && view.depthState.locked && view.depth.rig.visible);
const placed = depthRig(world, panned(0.5), TO_COLOR);
const rigMatrix = () => view.depth.rig.updateMatrix() || view.depth.rig.matrix;
near(rigMatrix(), placed);
// The eyes turn and move: the 3D stays where the head camera saw it.
view.update(pose(new THREE.Vector3(0.3, 1.5, 0), -0.6, 0.2));
near(rigMatrix(), placed);
// Zeroed again elsewhere: the room's anchor moves with the eyes of the next frame.
view.setZero({ yaw: -0.6, pitch: 0.2 });
const there = new THREE.Vector3(0.3, 1.5, 0);
view.update(pose(there, -0.6, 0.2));
near(rigMatrix(), depthRig(robotToWorld(zeroAnchor(there, { yaw: -0.6, pitch: 0.2 }), ZERO), panned(0.5), TO_COLOR));
// No head pose (the session ended): the 3D is around the eyes again, following them.
view.showDepth({ meta: META, layers });
assert.ok(!view.worldLocked);
view.update(pose(eye, 0.1, 0));
near(view.depth.rig.position, eye);
assert.ok(
  view.depth.rig.quaternion.angleTo(new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0.1, 0, 'YXZ'))) < 1e-6,
);
// Minerva's model: placed by M while the world-locked 3D shows and its switch or the Wrists switch is on (the wrists'
// pictures ride on its links), following Helm's guide then, drawn only with its own switch on; not placed around the
// eyes (no head pose) or with both switches off. The Wrists switch asks xr.js for the wrist cameras.
const placements = [],
  drawn = [],
  tracking = [],
  hands = [];
const robot = {
  group: new THREE.Group(),
  model: { id: 'minerva' },
  placed: false,
  load: async () => {},
  setOpacity() {},
  linkMatrix: () => null,
  track: (on) => tracking.push(on),
  place(matrix, draw = true) {
    this.placed = !!matrix;
    placements.push(matrix && matrix.clone());
    drawn.push(!!matrix && draw);
  },
};
const scene = new HeadsetScene({}, { robot, onHands: (on) => hands.push(on) });
assert.deepEqual(scene.wristState, [false, 'Wrists show on the model with the 3D on']);
scene.setDepth(true);
scene.setZero(zero);
scene.update(pose(eye, zero.yaw, zero.pitch));
assert.deepEqual(scene.wristState, [false, 'Wrists show on the model once a session places it']);
scene.showDepth({ meta: { ...META, head: { pose: panned(0.5), zero: ZERO, joints: {}, skew_ms: 0 } }, layers });
near(placements.at(-1), world); // The robot's frame where M puts it,
assert.equal(drawn.at(-1), true);
assert.equal(tracking.at(-1), true); // posed by the guide.
assert.deepEqual(scene.wristState, [false, 'Wrists: no camera mounts']);
scene.setWristMounts({
  left_wrist: {
    link: 'arm',
    matrix: [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1],
    ],
    fov_deg: [90, 70],
  },
});
assert.deepEqual(scene.wristState, [true, null]);
scene.setModel(false); // The wrists still want it: placed, posed, not drawn.
near(placements.at(-1), world);
assert.equal(drawn.at(-1), false);
assert.equal(tracking.at(-1), true);
scene.setHands(false);
assert.equal(placements.at(-1), null);
assert.equal(tracking.at(-1), false);
assert.equal(scene.wrists.group.visible, false);
scene.setModel(true);
near(placements.at(-1), world);
assert.equal(drawn.at(-1), true);
scene.showDepth({ meta: META, layers }); // No head pose: no M, no model.
assert.equal(placements.at(-1), null);
scene.setHands(true);
assert.deepEqual(hands, [false, true]);
assert.ok(scene.wrists.group.visible);
// The splat map: placed by M (it then puts itself by Minerva's pose in its world) while the world-locked 3D shows
// and its switch is on (off at first); hidden around the eyes. It sorts for each frame's eyes.
const mapPlacements = [],
  mapShown = [],
  mapEyes = [];
const map = {
  group: new THREE.Group(),
  state: {},
  setOpacity() {},
  show: (on) => mapShown.push(on),
  place: (matrix) => mapPlacements.push(matrix && matrix.clone()),
  update: (eyes) => mapEyes.push(eyes.clone()),
};
const mapped = new HeadsetScene({}, { robot, map });
mapped.setDepth(true);
mapped.setZero(zero);
mapped.update(pose(eye, zero.yaw, zero.pitch));
mapped.showDepth({ meta: { ...META, head: { pose: panned(0.5), zero: ZERO, joints: {}, skew_ms: 0 } }, layers });
assert.equal(mapPlacements.at(-1), null);
assert.equal(mapShown.at(-1), false);
mapped.setMap(true);
near(mapPlacements.at(-1), world);
assert.equal(mapShown.at(-1), true);
const eyesAt = new THREE.Matrix4().makeTranslation(0.1, 1.6, 0.2);
mapped.update({
  ...pose(eye, zero.yaw, zero.pitch),
  transform: { ...pose(eye, zero.yaw, zero.pitch).transform, matrix: eyesAt.toArray() },
});
near(mapEyes.at(-1), eyesAt);
mapped.showDepth({ meta: META, layers }); // No head pose: no M, no map.
assert.equal(mapPlacements.at(-1), null);
console.log('headset world-locked 3D: ok');
