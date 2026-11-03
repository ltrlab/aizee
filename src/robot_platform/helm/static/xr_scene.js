// What the headset shows while it steers Minerva's head (xr.js): the head camera's picture, a ring of controls when
// the operator opens it (xr_ring.js: A or X opens it, the thumbstick tilts to pick and presses to choose), and a
// one-line alert low in view with the ring closed. The floating status panel, the picture's status line and bar of
// controls, and the controllers' pointing rays are gone (the user, 2026-11-03: they got in the way); the status
// shows on the ring's card.
// The ring's View level sets how the picture shows (xr_view.js, remembered by the headset's browser): Follow head
// keeps it in front of the eyes as the head turns, level (as the robot's head, pan and tilt, keeps its camera), or
// leaves it where it is; 3D shows the head camera's depth instead (xr_depth.js), where the head camera saw it in the
// room while a session runs (world-locked: it stays put as the eyes turn), else around the eyes; Model shows
// Minerva's model with the world-locked 3D (xr_model.js); Wrists puts the wrist cameras' pictures on the model's
// wrists, along each camera's view (xr_wrists.js); Map shows her splat map around the world-locked 3D (xr_splat.js);
// two sliders set the picture's opacity (flat or 3D, the model and the map) and the wrists'; Recenter puts the
// picture back in front. It all starts in front of where the session began: WebXR's 'local' space has the head at its
// origin, looking down -Z. The ring and the alert draw over the 3D picture, which surrounds them.
import * as THREE from 'three';
import {DepthView, depthRig, robotToWorld, zeroAnchor} from './xr_depth.js';
import {WristScreens} from './xr_wrists.js';
import {RobotModel} from './xr_model.js';
import {SplatMap} from './xr_splat.js';
import {SIZES, drawAlert, drawCard, drawPlaceholder, drawRing} from './xr_panels.js';
import {FOLLOW_OFFSET, SLIDERS, SWITCHES, loadView, lookAngles, offsetOf, opacityStep, saveView} from './xr_view.js';

const SCREEN_W = 1.12, SCREEN_H = 0.84;
// The ring: this wide, this far ahead and this far below where the eyes look as it opens; it stays there while open.
// About 36 degrees across, as at first (0.36 m at 0.55 m), a little farther off to focus on. Its card, as wide, sits
// just over it.
export const RING_W = .46, RING_AHEAD_M = .7, RING_BELOW = 20 * Math.PI / 180;
export const CARD_H = RING_W * SIZES.card[1] / SIZES.card[0], CARD_GAP_M = .015;
// The alert: low in view, following the eyes; about 38 degrees across.
export const ALERT_W = .62, ALERT_AHEAD_M = .9, ALERT_BELOW = 24 * Math.PI / 180;

function drawable([width, height]) {
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return {canvas, context: canvas.getContext('2d'), texture};
}

// A screen of text: drawn over the 3D picture around it (never hidden inside it).
function panelMesh(width, height, texture) {
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({map: texture, transparent: true, side: THREE.DoubleSide, depthTest: false}));
  mesh.renderOrder = 3;
  mesh.visible = false;
  return mesh;
}

export class HeadsetScene {
  // `storage`: where the picture's view is remembered (the headset browser's localStorage), or null. `onDepth` and
  // `onHands`: called with the 3D and Wrists switches' new states (xr.js asks Helm for the depth, or the wrist
  // cameras, then). `robot`: Minerva's model (xr_model.js), `map`: her splat map (xr_splat.js); tests pass their own.
  constructor(renderer, {storage = null, onDepth = () => {}, onHands = () => {}, robot = new RobotModel(),
    map = new SplatMap()} = {}) {
    this.renderer = renderer;
    this.storage = storage;
    this.onDepth = onDepth;
    this.onHands = onHands;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(70, 1, .05, 50);  // WebXR's own cameras replace it while presenting.
    this.eye = new THREE.Vector3();  // This frame's eye position and look (yaw, pitch; no roll).
    this.look = new THREE.Quaternion();
    this.looked = false;
    this.euler = new THREE.Euler();
    this.facing = new THREE.Object3D();
    this.followOffset = new THREE.Vector3();
    this.followFacing = new THREE.Quaternion();
    this.point = new THREE.Vector3();

    // The camera screen: the picture, a placeholder until the first frame.
    this.placeholder = drawable(SIZES.picture);
    this.video = new THREE.Texture();
    this.video.colorSpace = THREE.SRGBColorSpace;
    this.video.flipY = false;  // ImageBitmaps upload top row first: the screen's UVs are flipped instead.
    const picture = new THREE.PlaneGeometry(SCREEN_W, SCREEN_H);
    const uv = picture.attributes.uv;
    for (let i = 0; i < uv.count; i++) uv.setY(i, 1 - uv.getY(i));
    this.screenMaterial = new THREE.MeshBasicMaterial({map: this.placeholder.texture, side: THREE.DoubleSide});
    this.placeholder.texture.flipY = false;
    this.screen = new THREE.Group();
    this.picture = new THREE.Mesh(picture, this.screenMaterial);
    this.screen.add(this.picture);
    this.writePlaceholder('Waiting for the head camera…');
    this.scene.add(this.screen);

    // The ring, its card (a child: it moves with the ring) and the alert: hidden until there is something to show.
    this.ringCanvas = drawable(SIZES.ring);
    this.ring = panelMesh(RING_W, RING_W, this.ringCanvas.texture);
    this.ringDrawn = null;
    this.cardCanvas = drawable(SIZES.card);
    this.card = panelMesh(RING_W, CARD_H, this.cardCanvas.texture);
    this.card.visible = true;
    this.card.position.set(0, RING_W / 2 + CARD_GAP_M + CARD_H / 2, 0);
    this.ring.add(this.card);
    this.cardDrawn = null;
    this.alertCanvas = drawable(SIZES.alert);
    this.alert = panelMesh(ALERT_W, ALERT_W * SIZES.alert[1] / SIZES.alert[0], this.alertCanvas.texture);
    this.alertDrawn = null;
    this.scene.add(this.ring, this.alert);

    // The 3D picture: the head camera's depth, once a frame with depth has come.
    this.depth = new DepthView();
    this.depthFramed = false;
    this.scene.add(this.depth.rig);
    // World-locked, from Helm's zero for the head (the yaw and pitch the operator looked at as they zeroed it) and
    // where the eyes were then (`anchor`, taken at the next frame after a zero), and each depth frame's head pose.
    this.zero = {yaw: 0, pitch: 0};
    this.anchor = null;
    this.anchorPending = true;
    this.depthHead = null;  // The last depth frame's head pose (camera_video_worker), null without a session.
    this.depthToColor = null;
    this.worldLocked = false;
    this.world = new THREE.Matrix4();
    this.rigPose = new THREE.Matrix4();
    // Minerva's model, lit, in the room by M with the world-locked 3D; the wrist cameras' pictures on its wrists.
    this.robot = robot;
    this.scene.add(this.robot.group, new THREE.HemisphereLight(0xffffff, 0x3a3f4b, 2.5));
    this.wrists = new WristScreens();
    this.scene.add(this.wrists.group);
    // The splat map, placed by M and Minerva's pose in its world, with the world-locked 3D.
    this.map = map;
    this.eyes = new THREE.Matrix4();
    this.scene.add(this.map.group);
    this.view = loadView(storage);
    this.setOffset(this.point.fromArray(this.view.offset));
    this.applyView();
    this.place();
  }

  place() {
    this.screen.position.set(0, -.08, -1.45);
    this.screen.lookAt(0, 0, 0);
  }

  // A ring item for the view (xr_session.ringMenu's View level): a switch toggled, a slider's value (saved once it
  // is let go: `done`), or the picture recentred. False for an item that is not the view's.
  choose({id, value, done}) {
    const [kind, name] = id.split(':');
    if (kind === 'view' && SWITCHES.includes(name)) {
      const set = {follow: 'setFollow', depth: 'setDepth', model: 'setModel', hands: 'setHands', map: 'setMap'}[name];
      this[set](!this.view[name]);
    } else if (kind === 'slider' && SLIDERS.includes(name)) {
      this.setSlider(name, value);
      if (done) this.save();
    } else if (id === 'recenter') {
      this.recenter();
    } else {
      return false;
    }
    return true;
  }

  setFollow(follow) {
    this.view.follow = follow;  // Off, it stays where it is now; on, it goes in front of the eyes at the next frame.
    this.applyView();
    this.save();
  }

  // The 3D picture on or off: the flat picture gives way to the head camera's depth around the eyes (once its first
  // frame has come), and xr.js asks Helm for the depth, or stops.
  setDepth(depth) {
    this.view.depth = depth;
    this.applyView();
    this.save();
    this.onDepth(depth);
  }

  // Minerva's model with the world-locked 3D, or not.
  setModel(model) {
    this.view.model = model;
    this.applyView();
    this.save();
  }

  // The wrist cameras' pictures on the model's wrists, or not: xr.js streams them, or stops.
  setHands(hands) {
    this.view.hands = hands;
    this.applyView();
    this.save();
    this.onHands(hands);
  }

  // Minerva's splat map around the world-locked 3D, or not.
  setMap(map) {
    this.view.map = map;
    this.applyView();
    this.save();
  }

  setOpacity(value) {
    this.setSlider('opacity', value);
  }

  setWristOpacity(value) {
    this.setSlider('wristOpacity', value);
  }

  // A slider's value (SLIDERS: the picture's opacity, the wrists'), in its steps.
  setSlider(name, value) {
    const stepped = opacityStep(value);
    if (stepped === this.view[name]) return;
    this.view[name] = stepped;
    this.applyView();
  }

  save() {
    saveView(this.storage, {...this.view, offset: this.followOffset.toArray()});
  }

  // Following: where the picture sits in front of the eyes (in the frame of their yaw and pitch), turned to face them.
  setOffset(offset) {
    this.followOffset.copy(offset);
    this.facing.position.copy(offset);
    this.facing.lookAt(0, 0, 0);
    this.followFacing.copy(this.facing.quaternion);
  }

  // The picture back at its first place in front of the eyes: following, from now on; left where it is, now.
  recenter() {
    this.setOffset(this.point.fromArray(FOLLOW_OFFSET));
    if (this.looked) this.follow();
    this.save();
  }

  // The picture's opacity and which of the flat and 3D pictures shows, as the view is now.
  applyView() {
    const transparent = this.view.opacity < 1;
    if (this.screenMaterial.transparent !== transparent) {
      this.screenMaterial.transparent = transparent;
      this.screenMaterial.needsUpdate = true;
    }
    this.screenMaterial.opacity = this.view.opacity;
    this.depth.setOpacity(this.view.opacity);
    this.robot.setOpacity(this.view.opacity);
    this.map.setOpacity(this.view.opacity);
    this.wrists.setOpacity(this.view.wristOpacity);
    const depth = this.view.depth && this.depthFramed;  // Until the first depth comes, the flat picture stays.
    this.picture.visible = !depth;
    this.depth.rig.visible = depth;
    this.wrists.group.visible = this.view.hands;
    this.placeRobot();
  }

  // A frame's depth ({meta, layers}, xr_camera.js) with the picture it came with (showFrame, just before).
  showDepth({meta, layers}) {
    this.depth.show(meta, layers, this.video);
    this.depthHead = meta.head || null;
    this.depthToColor = meta.to_color || null;
    this.placeDepth();
    if (!this.depthFramed) {
      this.depthFramed = true;
      this.applyView();
    }
  }

  // The 3D picture for the status (xr_status.panelRows): whether it shows, whether it is world-locked, and how near
  // its frame's head pose came to the frame's capture.
  get depthState() {
    return {on: this.view.depth, shown: this.view.depth && this.depthFramed, locked: this.worldLocked,
      skew_ms: this.depthHead?.skew_ms ?? null};
  }

  // Helm's zero for the head (its status replies): a new one anchors the 3D at the eyes of the next frame.
  setZero(zero) {
    if (!zero || !Number.isFinite(zero.yaw) || !Number.isFinite(zero.pitch)) return;
    if (zero.yaw === this.zero.yaw && zero.pitch === this.zero.pitch) return;
    this.zero = {yaw: zero.yaw, pitch: zero.pitch};
    this.anchorPending = true;
  }

  // The head zeroed or its tracking started again (xr.js): the eyes of the next frame anchor the 3D.
  rezero() {
    this.anchorPending = true;
  }

  // World-locked where the last depth frame's head pose puts it, once anchored; without a head pose (no session),
  // the rig is where the eyes are (update).
  placeDepth() {
    this.worldLocked = !!(this.depthHead && this.anchor);
    if (this.worldLocked) {
      robotToWorld(this.anchor, this.depthHead.zero, this.world);
      depthRig(this.world, this.depthHead.pose, this.depthToColor, this.rigPose)
        .decompose(this.depth.rig.position, this.depth.rig.quaternion, this.depth.rig.scale);
    }
    this.placeRobot();
  }

  // Minerva's model where M puts the robot while the world-locked 3D shows: loaded the first time and following
  // Helm's guide while the Model or the Wrists switch wants it, drawn only with the Model switch on. The map, with the
  // Map switch on, by M and her pose in its world.
  placeRobot() {
    const locked = this.view.depth && this.depthFramed && this.worldLocked;
    const wanted = (this.view.model || this.view.hands) && locked;
    if (wanted && !this.robot.model) this.robot.load().then(() => this.placeRobot(), () => {});
    this.robot.place(wanted ? this.world : null, this.view.model);
    this.robot.track(wanted && !!this.robot.model);
    this.map.show(this.view.map);
    this.map.place(this.view.map && locked ? this.world : null);
  }

  // Whether the wrists' pictures can be on the model now, and why not: [ok, reason] for the ring's status.
  get wristState() {
    if (!this.view.hands) return [false, null];
    if (!this.view.depth) return [false, 'Wrists show on the model with the 3D on'];
    if (!this.worldLocked) return [false, 'Wrists show on the model once a session places it'];
    if (!this.robot.placed) return [false, 'Wrists: the model is loading'];
    if (!Object.values(this.wrists.screens).some(screen => screen.mount)) return [false, 'Wrists: no camera mounts'];
    return [true, null];
  }

  // The map for the status (xr_status.panelRows).
  get mapState() {
    return this.map.state;
  }

  // The wrist cameras' mounts from Helm (/api/helm/xr/wrists).
  setWristMounts(mounts) {
    this.wrists.setMounts(mounts);
  }

  // A wrist camera's frame (xr.js streams them while the Wrists switch is on).
  showWrist(role, bitmap) {
    this.wrists.showFrame(role, bitmap);
  }

  // This frame's eyes, from its viewer pose: where they are, and their look (yaw and pitch; no roll).
  see(pose) {
    const {position, orientation} = pose.transform;
    const [yaw, pitch] = lookAngles([orientation.x, orientation.y, orientation.z, orientation.w]);
    this.look.setFromEuler(this.euler.set(pitch, yaw, 0, 'YXZ'));
    this.eye.set(position.x, position.y, position.z);
    this.looked = true;
  }

  // Following: the screen at its offset from the eyes, turned with their yaw and pitch.
  follow() {
    this.screen.position.copy(this.followOffset).applyQuaternion(this.look).add(this.eye);
    this.screen.quaternion.copy(this.look).multiply(this.followFacing);
    this.screen.updateMatrixWorld(true);
  }

  // `mesh` this far ahead of the eyes and this far below their look, facing them.
  inView(mesh, ahead, below) {
    mesh.position.set(0, -Math.sin(below) * ahead, -Math.cos(below) * ahead).applyQuaternion(this.look).add(this.eye);
    mesh.lookAt(this.eye);
    mesh.updateMatrixWorld(true);
  }

  // The ring as xr_ring.Ring.drawing gives it (null: closed), with the status for its card: placed in front of the
  // eyes as it opens, then left there; each drawn again only when it changes.
  showRing(drawing, status = []) {
    if (!drawing) {
      this.ring.visible = false;
      this.ringDrawn = this.cardDrawn = null;
      return;
    }
    if (!this.ring.visible) {
      this.inView(this.ring, RING_AHEAD_M, RING_BELOW);
      this.ring.visible = true;
    }
    const ring = JSON.stringify(drawing), card = JSON.stringify([drawing, status]);
    if (ring !== this.ringDrawn) {
      this.ringDrawn = ring;
      drawRing(this.ringCanvas.context, drawing);
      this.ringCanvas.texture.needsUpdate = true;
    }
    if (card !== this.cardDrawn) {
      this.cardDrawn = card;
      drawCard(this.cardCanvas.context, drawing, status);
      this.cardCanvas.texture.needsUpdate = true;
    }
  }

  // The alert ({text, tone}; null: none), low in view while the ring is closed: placed as it shows, then following.
  showAlert(alert) {
    const shown = this.alert.visible;
    this.alert.visible = !!alert && !this.ring.visible;
    if (this.alert.visible && !shown && this.looked) this.inView(this.alert, ALERT_AHEAD_M, ALERT_BELOW);
    const key = alert ? `${alert.tone}:${alert.text}` : null;
    if (!alert || key === this.alertDrawn) return;
    this.alertDrawn = key;
    drawAlert(this.alertCanvas.context, [alert.text, alert.tone]);
    this.alertCanvas.texture.needsUpdate = true;
  }

  // Each frame, with its viewer pose (null when the headset lost tracking): the picture following the head, the
  // world-locked 3D's anchor, the wrists' pictures on the model, and the alert low in view.
  update(pose = null) {
    if (pose) this.see(pose);
    if (pose && this.view.map) this.map.update(this.eyes.fromArray(pose.transform.matrix));
    if (pose && this.anchorPending) {
      this.anchor = zeroAnchor(this.eye, this.zero, this.anchor || new THREE.Matrix4());
      this.anchorPending = false;
      this.placeDepth();
    }
    if (pose && this.view.follow) {
      this.follow();
      if (!this.worldLocked) {
        // The 3D picture from the operator's eyes, as the head camera sees from its own: its rig is the camera.
        this.depth.rig.position.copy(this.eye);
        this.depth.rig.quaternion.copy(this.look);
        this.depth.rig.scale.set(1, 1, 1);
      }
    }
    if (this.view.hands) this.wrists.update(this.robot, this.robot.placed);
    if (pose && this.alert.visible) this.inView(this.alert, ALERT_AHEAD_M, ALERT_BELOW);
  }

  showFrame(bitmap) {
    const previous = this.video.image;
    this.video.image = bitmap;
    this.video.needsUpdate = true;
    if (this.screenMaterial.map !== this.video) {
      this.screenMaterial.map = this.video;
      this.screenMaterial.needsUpdate = true;
    }
    previous?.close?.();  // An ImageBitmap's pixels go now, not at collection.
  }

  // Whether the picture is current: a stale one dims.
  setFresh(fresh) {
    this.screenMaterial.color.setHex(fresh || this.screenMaterial.map !== this.video ? 0xffffff : 0x5a5a5a);
  }

  writePlaceholder(text) {
    drawPlaceholder(this.placeholder.context, text);
    this.placeholder.texture.needsUpdate = true;
  }
}
