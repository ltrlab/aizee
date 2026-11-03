import assert from 'node:assert/strict';
import {register} from 'node:module';
import {pathToFileURL} from 'node:url';

// The headset's picture, its ring and its alert (xr_scene.js, on the real three.js): Follow head keeps the picture in
// front of the eyes as the head turns, level, or leaves it where it is; the ring's View items switch it and set its
// opacities; Recenter puts it back in front; the headset remembers the view (xr_view.js). The ring opens in front of
// the eyes, a little low, and stays there while open; the alert sits low in view with the ring closed.
const root = new URL('../../', import.meta.url);
const three = new URL('src/robot_platform/viewer/static/vendor/three.module.js', root).href;
register('data:text/javascript,' + encodeURIComponent(`export async function resolve(specifier, context, next) {
  return specifier === 'three' ? {url: ${JSON.stringify(three)}, shortCircuit: true} : next(specifier, context);
}`), pathToFileURL('./'));

// Stub canvases: the texts drawn on them.
function context(canvas) {
  const noop = () => {};
  return {canvas, texts: [], clearRect: noop, fillRect: noop, beginPath: noop, roundRect: noop, fill: noop, arc: noop,
    closePath: noop, stroke: noop, measureText: text => ({width: String(text).length * 18}),
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
const {ALERT_AHEAD_M, ALERT_BELOW, ALERT_W, CARD_GAP_M, CARD_H, HeadsetScene, RING_AHEAD_M, RING_BELOW, RING_W} =
  await import(new URL('src/robot_platform/helm/static/xr_scene.js', root).href);
const {ALERT_LINES, SIZES, TEXT, drawAlert, drawCard} = await import(
  new URL('src/robot_platform/helm/static/xr_panels.js', root).href);
const {Ring} = await import(new URL('src/robot_platform/helm/static/xr_ring.js', root).href);
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
assert.deepEqual(loadView(storage), {follow: false, depth: false, model: true, hands: true, map: false,
  opacity: .45, wristOpacity: 1, offset: FOLLOW_OFFSET});  // Behind: no.
store.set(STORAGE_KEY, JSON.stringify({depth: true}));
assert.equal(loadView(storage).depth, true);  // The 3D picture, remembered.
store.set(STORAGE_KEY, JSON.stringify({model: false, hands: false}));
assert.deepEqual([loadView(storage).model, loadView(storage).hands], [false, false]);  // The model and wrists, off.
store.set(STORAGE_KEY, JSON.stringify({map: true}));
assert.equal(loadView(storage).map, true);  // The splat map, remembered.
saveView(storage, {follow: true, depth: true, model: false, hands: true, map: true, opacity: 2, offset: [.2, -.1, -1]});
assert.deepEqual(JSON.parse(store.get(STORAGE_KEY)), {follow: true, depth: true, model: false, hands: true, map: true,
  opacity: 1, wristOpacity: 1, offset: [.2, -.1, -1]});
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

// The scene: its renderer is WebXR's; it needs nothing of it here.
const eyes = new THREE.PerspectiveCamera();
const view = new HeadsetScene({}, {storage});
const picture = view.screen.children[0];
assert.equal(view.view.follow, true);
assert.equal(view.ring.visible, false);  // No ring, no alert, until there is something to show.
assert.equal(view.alert.visible, false);

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

// Where something is from the eyes, in the frame of their look (yaw and pitch).
const fromEyes = (object, yaw, pitch) => object.position.clone().sub(eyes.position)
  .applyQuaternion(orientation(degrees(yaw), degrees(pitch), 0).invert());

// Follow head, off (the ring's View item): the picture stays where it is as the head turns; remembered.
assert.equal(view.choose({id: 'view:follow'}), true);
assert.equal(view.view.follow, false);
assert.equal(JSON.parse(store.get(STORAGE_KEY)).follow, false);
const kept = view.screen.position.clone();
view.update(pose(40, 0));
assert.ok(view.screen.position.distanceTo(kept) < 1e-9);
// Anything not the view's is not the scene's to choose.
assert.equal(view.choose({id: 'key:I'}), false);
assert.equal(view.choose({id: 'view:nothing'}), false);

// The picture's slider: it changes as the stick moves, remembered once let go.
view.choose({id: 'slider:opacity', value: .5});
assert.equal(view.view.opacity, .5);
assert.ok(picture.material.transparent);
close(picture.material.opacity, .5);
assert.equal(JSON.parse(store.get(STORAGE_KEY)).opacity, 1);
view.choose({id: 'slider:opacity', value: .25, done: true});
assert.equal(view.view.opacity, .25);
assert.equal(JSON.parse(store.get(STORAGE_KEY)).opacity, .25);
// The wrist cameras' slider sets theirs alone (the user, 2026-10-31), remembered too.
view.choose({id: 'slider:wristOpacity', value: .6, done: true});
assert.equal(view.view.wristOpacity, .6);
assert.equal(view.wrists.opacity, .6);
assert.equal(view.view.opacity, .25);
assert.equal(JSON.parse(store.get(STORAGE_KEY)).wristOpacity, .6);

// Following again: it goes back in front of the eyes.
view.choose({id: 'view:follow'});
assert.equal(view.view.follow, true);
view.update(pose(40, 0));
inFront(40, 0);

// A picture remembered elsewhere in front of the eyes follows there; Recenter puts it back, remembered.
store.set(STORAGE_KEY, JSON.stringify({...JSON.parse(store.get(STORAGE_KEY)), offset: [.3, .1, -1.2]}));
const moved = new HeadsetScene({}, {storage});
moved.update(pose(-20, 10));
[.3, .1, -1.2].forEach((value, i) => close(fromEyes(moved.screen, -20, 10).getComponent(i), value));
assert.equal(moved.choose({id: 'recenter'}), true);
moved.update(pose(-20, 10));
FOLLOW_OFFSET.forEach((value, i) => close(fromEyes(moved.screen, -20, 10).getComponent(i), value));
assert.deepEqual(JSON.parse(store.get(STORAGE_KEY)).offset, FOLLOW_OFFSET);

// The ring: opened, it is drawn RING_AHEAD_M in front of the eyes, RING_BELOW under their look, facing them, and
// stays there as the head turns; closed and opened again, it comes where the eyes look then. As wide across the view
// as at first (36 degrees), its text large enough to read in the headset (the user, 2026-11-03: 21-38 pixel text was
// too small): 1.5 degrees or more on the ring and its card, as on the alert.
close(2 * Math.atan(RING_W / 2 / RING_AHEAD_M) * 180 / Math.PI, 36.4, .5);
const perDegree = 1024 / (2 * Math.atan(RING_W / 2 / RING_AHEAD_M) * 180 / Math.PI);
assert.ok(Object.values(TEXT).every(size => size / perDegree >= 1.5), JSON.stringify(TEXT));
const alertDegrees = 2 * Math.atan(ALERT_W / 2 / ALERT_AHEAD_M) * 180 / Math.PI;
assert.ok(TEXT.alert / (SIZES.alert[0] / alertDegrees) >= 1.5);
assert.ok(TEXT.alertLeast / (SIZES.alert[0] / alertDegrees) >= 1.5);  // A long alert's smaller text too.
const ring = new Ring(() => ({title: 'Minerva', items: [{id: 'a', label: 'Start', submenu: true}, {id: 'b',
  label: 'View', submenu: true}]}));
ring.step([{hand: 'right', menu: true, press: false, stick: [0, 0]}], 0);
view.update(pose(30, 5));
view.showRing(ring.drawing(0), [['No session', '']]);
assert.equal(view.ring.visible, true);
const ringAt = fromEyes(view.ring, 30, 5);
close(ringAt.x, 0);
close(ringAt.y, -Math.sin(RING_BELOW) * RING_AHEAD_M);
close(ringAt.z, -Math.cos(RING_BELOW) * RING_AHEAD_M);
const ringFacing = new THREE.Vector3(0, 0, 1).applyQuaternion(view.ring.quaternion);
assert.ok(ringFacing.dot(eyes.position.clone().sub(view.ring.position).normalize()) > .999);
const texts = view.ringCanvas.context.texts;
assert.ok(texts.includes('Start ›') && texts.includes('View ›') && texts.includes('Minerva'));
// Its card just over it, facing the same way, with the status and how the ring is worked.
const cardTexts = view.cardCanvas.context.texts;
assert.ok(cardTexts.includes('No session'), cardTexts);
assert.ok(cardTexts.some(text => /press the stick to choose/.test(text)), cardTexts);
view.ring.updateMatrixWorld(true);
const cardAt = view.card.getWorldPosition(new THREE.Vector3()).sub(view.ring.position);
const ringUp = new THREE.Vector3(0, 1, 0).applyQuaternion(view.ring.quaternion);
close(cardAt.length(), RING_W / 2 + CARD_GAP_M + CARD_H / 2);
assert.ok(cardAt.normalize().dot(ringUp) > .999);
assert.ok(view.card.visible);
const placed = view.ring.position.clone();
view.update(pose(-60, 0));
view.showRing(ring.drawing(0), [['No session', '']]);
assert.ok(view.ring.position.distanceTo(placed) < 1e-9);
const drawn = texts.length, cardDrawn = cardTexts.length;
view.showRing(ring.drawing(0), [['No session', '']]);
assert.equal(texts.length, drawn);  // Unchanged: not drawn again.
view.showRing(ring.drawing(0), [['Virtual · IDLE', 'warn']]);
assert.equal(texts.length, drawn);  // The status changed: the card alone is drawn again.
assert.ok(cardTexts.length > cardDrawn && cardTexts.includes('Virtual · IDLE'));
view.showRing(null);
assert.equal(view.ring.visible, false);
view.showRing(ring.drawing(0));
assert.ok(view.ring.position.distanceTo(placed) > .1);  // Where the eyes look now.

// The alert: hidden while the ring is open; with it closed, low in view, following the eyes.
view.showAlert({text: 'Fault: limit', tone: 'bad'});
assert.equal(view.alert.visible, false);
view.showRing(null);
view.showAlert({text: 'Fault: limit', tone: 'bad'});
assert.equal(view.alert.visible, true);
assert.ok(view.alertCanvas.context.texts.includes('Fault: limit'));
view.update(pose(10, -20));
const alertAt = fromEyes(view.alert, 10, -20);
close(alertAt.y, -Math.sin(ALERT_BELOW) * ALERT_AHEAD_M);
close(alertAt.z, -Math.cos(ALERT_BELOW) * ALERT_AHEAD_M);
view.showAlert(null);
assert.equal(view.alert.visible, false);

// A fault's whole reason: a session's start fault was cut short at two lines (the user, 2026-11-03). The alert takes
// up to ALERT_LINES lines, in smaller text past two, and only then ends with '…'; the card says all of it.
const said = drawing => drawing.texts.join(' ').replace(/\s+/g, ' ').trim();
const reason = 'Fault: ' + Array.from({length: 36}, (_, i) => `word${i}`).join(' ');  // About 240 characters.
const alertDrawing = context({width: SIZES.alert[0], height: SIZES.alert[1]});
drawAlert(alertDrawing, [reason, 'bad']);
assert.equal(said(alertDrawing), reason);
assert.ok(alertDrawing.texts.length > 2 && alertDrawing.texts.length <= ALERT_LINES, alertDrawing.texts);
const endless = 'Fault: ' + Array.from({length: 120}, (_, i) => `word${i}`).join(' ');
const cutDrawing = context({width: SIZES.alert[0], height: SIZES.alert[1]});
drawAlert(cutDrawing, [endless, 'bad']);
assert.equal(cutDrawing.texts.length, ALERT_LINES);
assert.ok(cutDrawing.texts.at(-1).endsWith(' …'));
const cardDrawing = context({width: SIZES.card[0], height: SIZES.card[1]});
drawCard(cardDrawing, {title: 'Minerva', items: [], aim: null, notice: null, depth: 0, confirm: null, progress: 0,
  adjust: null}, [[reason, 'bad'], ['Head still: no session', 'warn']]);
assert.ok(said(cardDrawing).includes(reason), cardDrawing.texts);

// Remembered by the next session's scene.
const again = new HeadsetScene({}, {storage});
assert.deepEqual([again.view.follow, again.view.opacity, again.view.wristOpacity], [true, .25, .6]);
console.log('headset follow, ring and alert: ok');
