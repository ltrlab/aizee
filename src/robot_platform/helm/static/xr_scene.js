// What the headset shows while it steers Minerva's head (xr.js): the head camera's picture on a screen with a status
// line and a bar of controls under it, and a status panel. Either screen can be grabbed with a controller's grip
// button and moved; let go, it turns upright to face the viewer. The bar's Follow head switch keeps the picture in
// front of the eyes as the head turns, level (as the robot's head, pan and tilt, keeps its camera), or leaves it where
// it is; its 3D switch shows the head camera's depth instead of the flat picture (xr_depth.js), where the head camera
// saw it in the room while a session runs (world-locked: it stays put as the eyes turn, and needs no following), else
// around the eyes; its Model switch shows Minerva's model with the world-locked 3D (xr_model.js), and its Hands switch
// the wrist cameras above the hands (xr_hands.js); its sliders set the picture's opacity (flat or 3D, and the model)
// and the wrist cameras'. The headset's browser remembers them (xr_view.js). Each controller
// shows its pointing ray (cyan over a screen, green holding one), and its trigger works the bar's control under it.
// They start in front of where the session began: WebXR's 'local' space has the head at its origin, looking down -Z.
// The screens' canvases are drawn by xr_panels.js, as Settings → Headset draws its flat copy; they and the rays draw
// over the 3D picture, which surrounds them.
import * as THREE from 'three';
import {DepthView, depthRig, robotToWorld, zeroAnchor} from './xr_depth.js';
import {WristScreens} from './xr_hands.js';
import {RobotModel} from './xr_model.js';
import {SIZES, SLIDERS, SWITCHES, controlAt, drawControls, drawPanel, drawPlaceholder, drawStrip, sliderAt} from
  './xr_panels.js';
import {loadView, lookAngles, offsetOf, opacityStep, saveView} from './xr_view.js';

const SCREEN_W = 1.12, SCREEN_H = 0.84, STRIP_H = 0.075, PANEL_W = 0.56, PANEL_H = 0.42;
const CONTROLS_H = SCREEN_W * SIZES.controls[1] / SIZES.controls[0];
const RAY = {idle: 0x9aa3b2, aim: 0x4fd3ff, hold: 0x3fbf7f};

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
  mesh.renderOrder = 2;
  return mesh;
}

export class HeadsetScene {
  // `storage`: where the picture's view is remembered (the headset browser's localStorage), or null. `onDepth` and
  // `onHands`: called with the 3D and Hands switches' new states (xr.js asks Helm for the depth, or the wrist cameras,
  // then). `robot`: Minerva's model (xr_model.js; tests pass their own).
  constructor(renderer, {storage = null, onDepth = () => {}, onHands = () => {}, robot = new RobotModel()} = {}) {
    this.renderer = renderer;
    this.storage = storage;
    this.onDepth = onDepth;
    this.onHands = onHands;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(70, 1, .05, 50);  // WebXR's own cameras replace it while presenting.
    this.raycaster = new THREE.Raycaster();
    this.rotation = new THREE.Matrix4();
    this.head = new THREE.Vector3();
    this.eye = new THREE.Vector3();  // Following: this frame's eye position and look (yaw, pitch; no roll).
    this.look = new THREE.Quaternion();
    this.looked = false;
    this.euler = new THREE.Euler();
    this.facing = new THREE.Object3D();
    this.followOffset = new THREE.Vector3();
    this.followFacing = new THREE.Quaternion();
    this.barPlane = new THREE.Plane();
    this.point = new THREE.Vector3();
    this.normal = new THREE.Vector3();

    // The camera screen: the picture (a placeholder until the first frame), the status line and the controls under it.
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
    this.strip = drawable(SIZES.strip);
    const strip = panelMesh(SCREEN_W, STRIP_H, this.strip.texture);
    strip.position.y = -(SCREEN_H + STRIP_H) / 2 - .01;
    this.screen.add(strip);
    this.controls = drawable(SIZES.controls);
    this.bar = panelMesh(SCREEN_W, CONTROLS_H, this.controls.texture);
    this.bar.position.y = strip.position.y - (STRIP_H + CONTROLS_H) / 2 - .01;
    this.screen.add(this.bar);
    this.writePlaceholder('Waiting for the head camera…');

    this.panel = new THREE.Group();
    this.panelCanvas = drawable(SIZES.panel);
    this.panel.add(panelMesh(PANEL_W, PANEL_H, this.panelCanvas.texture));

    this.panels = [this.screen, this.panel];
    for (const group of this.panels) this.scene.add(group);
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
    // Minerva's model, lit, in the room by M with the world-locked 3D; the wrist cameras above the hands.
    this.robot = robot;
    this.scene.add(this.robot.group, new THREE.HemisphereLight(0xffffff, 0x3a3f4b, 2.5));
    this.wrists = new WristScreens(renderer);
    this.scene.add(this.wrists.group);
    this.view = loadView(storage);
    this.hover = null;
    this.setOffset(this.point.fromArray(this.view.offset));
    this.applyView();
    this.place();
    this.controllers = [0, 1].map(index => this.controller(index));
  }

  place() {
    this.screen.position.set(0, -.08, -1.45);
    this.screen.lookAt(0, 0, 0);
    this.panel.position.set(-.92, -.42, -1.05);
    this.panel.lookAt(0, 0, 0);
  }

  controller(index) {
    const controller = this.renderer.xr.getController(index);
    const ray = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, -1)]),
      new THREE.LineBasicMaterial({color: RAY.idle, transparent: true, opacity: .75, depthTest: false}));
    ray.renderOrder = 3;
    ray.scale.z = 2.5;
    ray.visible = false;
    controller.add(ray);
    controller.userData = {ray, held: null, pointer: false, source: null, pressed: null};
    controller.addEventListener('connected', event => {
      controller.userData.source = event.data;
      // Controllers, not hands: tracked for the wrist cameras, hands holding the leaders point anywhere.
      controller.userData.pointer = event.data?.targetRayMode === 'tracked-pointer' && !event.data?.hand;
      ray.visible = controller.userData.pointer;
    });
    controller.addEventListener('disconnected', () => {
      this.release(controller);
      this.unpress(controller);
      controller.userData.source = null;
      controller.userData.pointer = ray.visible = false;
    });
    controller.addEventListener('squeezestart', () => this.grab(controller));
    controller.addEventListener('squeezeend', () => this.release(controller));
    // A trigger pressed on the bar is the bar's (select, below, then takes it); on the slider, it drags.
    controller.addEventListener('selectstart', () => {
      const hit = this.barControl(controller);
      controller.userData.pressed = hit ? hit.control || 'bar' : null;
      if (SLIDERS.includes(hit?.control)) this.setSlider(hit.control, hit.value);
    });
    controller.addEventListener('selectend', () => this.unpress(controller));  // A press the session cancelled.
    this.scene.add(controller);
    return controller;
  }

  // A controller's pointing ray, in the raycaster.
  pointRay(controller) {
    controller.updateWorldMatrix(true, false);  // This frame's pose: the scene's own update comes at render.
    this.rotation.identity().extractRotation(controller.matrixWorld);
    this.raycaster.ray.origin.setFromMatrixPosition(controller.matrixWorld);
    this.raycaster.ray.direction.set(0, 0, -1).applyMatrix4(this.rotation);
  }

  // The screen or panel a controller points at, and how far: its ray against the free ones.
  aim(controller) {
    this.pointRay(controller);
    const held = new Set(this.controllers.map(other => other.userData.held).filter(Boolean));
    const hits = this.raycaster.intersectObjects(this.panels.filter(group => !held.has(group)), true);
    if (!hits.length) return null;
    let group = hits[0].object;
    while (group && !this.panels.includes(group)) group = group.parent;
    return group ? {group, distance: hits[0].distance} : null;
  }

  held(group) {
    return this.controllers.some(controller => controller.userData.held === group);
  }

  // Where a controller points on the bar, in its canvas pixels, or null off it (or while the screen is carried).
  // `anywhere`: on the bar's plane, past its edges too (a slider dragged off the thin bar keeps going).
  barPoint(controller, anywhere = false) {
    if (this.held(this.screen)) return null;
    this.pointRay(controller);
    const [width, height] = SIZES.controls;
    if (!anywhere) {
      const hit = this.raycaster.intersectObject(this.bar, false)[0];
      return hit?.uv ? [hit.uv.x * width, (1 - hit.uv.y) * height] : null;
    }
    this.bar.updateWorldMatrix(true, false);
    this.barPlane.setFromNormalAndCoplanarPoint(this.normal.set(0, 0, 1).transformDirection(this.bar.matrixWorld),
      this.point.setFromMatrixPosition(this.bar.matrixWorld));
    if (!this.raycaster.ray.intersectPlane(this.barPlane, this.point)) return null;
    this.bar.worldToLocal(this.point);
    return [(this.point.x / SCREEN_W + .5) * width, (.5 - this.point.y / CONTROLS_H) * height];
  }

  // The control a controller points at on the bar ({control: null} between its controls), or null off the bar.
  barControl(controller) {
    const point = this.barPoint(controller);
    return point ? controlAt(point) || {control: null} : null;
  }

  // A trigger pull (select) from `source`: one pressed on the bar works the control under it and is the bar's
  // (true); anywhere else it is not (false: xr.js zeroes the head).
  select(source) {
    const controller = this.controllers.find(other => other.userData.source === source);
    const pressed = controller?.userData.pressed;
    if (!pressed) return false;
    const control = this.barControl(controller)?.control;
    if (pressed === 'follow' && control === 'follow') this.setFollow(!this.view.follow);
    if (pressed === 'depth' && control === 'depth') this.setDepth(!this.view.depth);
    if (pressed === 'model' && control === 'model') this.setModel(!this.view.model);
    if (pressed === 'hands' && control === 'hands') this.setHands(!this.view.hands);
    this.unpress(controller);
    return true;
  }

  unpress(controller) {
    if (SLIDERS.includes(controller.userData.pressed)) this.save();
    controller.userData.pressed = null;
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

  // The wrist cameras above the hands, or not: xr.js streams them, or stops.
  setHands(hands) {
    this.view.hands = hands;
    this.applyView();
    this.save();
    this.onHands(hands);
  }

  setOpacity(value) {
    this.setSlider('opacity', value);
  }

  setWristOpacity(value) {
    this.setSlider('wristOpacity', value);
  }

  // A slider's value (SLIDERS: the picture's opacity, the wrist cameras'), in its steps.
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

  // The picture's opacity, which of the flat and 3D pictures shows, and the bar's drawing, as the view and the
  // pointed-at control are now.
  applyView() {
    const transparent = this.view.opacity < 1;
    if (this.screenMaterial.transparent !== transparent) {
      this.screenMaterial.transparent = transparent;
      this.screenMaterial.needsUpdate = true;
    }
    this.screenMaterial.opacity = this.view.opacity;
    this.depth.setOpacity(this.view.opacity);
    this.robot.setOpacity(this.view.opacity);
    this.wrists.setOpacity(this.view.wristOpacity);
    const depth = this.view.depth && this.depthFramed;  // Until the first depth comes, the flat picture stays.
    this.picture.visible = !depth;
    this.depth.rig.visible = depth;
    this.wrists.group.visible = this.view.hands;
    this.placeRobot();
    drawControls(this.controls.context, {...Object.fromEntries(SWITCHES.map(name => [name, this.view[name]])),
      opacity: this.view.opacity, wristOpacity: this.view.wristOpacity, hover: this.hover});
    this.controls.texture.needsUpdate = true;
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

  // The 3D picture for the status panel (xr_status.panelRows): whether it shows, whether it is world-locked, and how
  // near its frame's head pose came to the frame's capture.
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

  // Minerva's model where M puts the robot, while the world-locked 3D shows and the Model switch is on: loaded the
  // first time, following Helm's guide while shown.
  placeRobot() {
    const shown = this.view.model && this.view.depth && this.depthFramed && this.worldLocked;
    if (shown && !this.robot.model) this.robot.load().then(() => this.placeRobot(), () => {});
    this.robot.place(shown ? this.world : null);
    this.robot.track(shown && !!this.robot.model);
  }

  // A wrist camera's frame (xr.js streams them while the Hands switch is on).
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
    this.screen.updateMatrixWorld(true);  // This frame's rays meet it where it is drawn.
  }

  grab(controller) {
    if (!controller.userData.pointer) return;
    const hit = this.aim(controller);
    if (!hit) return;
    controller.attach(hit.group);  // Keeps where it is: it moves with the hand from here.
    controller.userData.held = hit.group;
  }

  release(controller) {
    const group = controller.userData.held;
    if (!group) return;
    controller.userData.held = null;
    this.scene.attach(group);
    this.renderer.xr.getCamera().getWorldPosition(this.head);
    group.lookAt(this.head);  // Upright, facing the viewer.
    if (group === this.screen && this.view.follow && this.looked) {
      // Following, the picture stays where it was let go of in front of the eyes: its offset from them, unturned.
      const offset = this.point.copy(group.position).sub(this.eye).applyQuaternion(this.look.clone().invert());
      this.setOffset(this.point.fromArray(offsetOf(offset.toArray())));
      this.save();
    }
  }

  // Each frame, with its viewer pose (null when the headset lost tracking) and the time (seconds): the picture
  // following the head, the wrist cameras above the hands, the rays' colours and lengths, a dragged slider, and the
  // bar's pointed-at control.
  update(pose = null, now = performance.now() / 1000) {
    if (pose) this.see(pose);
    if (pose && this.view.hands) this.wrists.update(this.eye, now);
    if (pose && this.anchorPending) {
      this.anchor = zeroAnchor(this.eye, this.zero, this.anchor || new THREE.Matrix4());
      this.anchorPending = false;
      this.placeDepth();
    }
    if (pose && this.view.follow) {
      if (!this.held(this.screen)) this.follow();
      if (!this.worldLocked) {
        // The 3D picture from the operator's eyes, as the head camera sees from its own: its rig is the camera.
        this.depth.rig.position.copy(this.eye);
        this.depth.rig.quaternion.copy(this.look);
        this.depth.rig.scale.set(1, 1, 1);
      }
    }
    let hover = null;
    for (const controller of this.controllers) {
      const {ray, held, pointer, pressed} = controller.userData;
      if (!pointer) continue;
      if (SLIDERS.includes(pressed)) {
        const point = this.barPoint(controller, true);
        if (point) this.setSlider(pressed, sliderAt(pressed, point[0]));
      }
      const hit = held ? null : this.aim(controller);
      ray.material.color.setHex(held ? RAY.hold : hit ? RAY.aim : RAY.idle);
      ray.scale.z = hit ? hit.distance : held ? .3 : 2.5;
      const control = SLIDERS.includes(pressed) ? pressed : held ? null : this.barControl(controller)?.control;
      hover = hover || control || null;
    }
    if (hover !== this.hover) {
      this.hover = hover;
      this.applyView();
    }
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

  writePlaceholder(text) {
    drawPlaceholder(this.placeholder.context, text);
    this.placeholder.texture.needsUpdate = true;
  }

  // The status line ([[text, tone], [text, tone]]: left, right), the panel's rows ([label, value, tone]) and hints;
  // `fresh`: whether the picture is current.
  setText(strip, rows, hints, fresh) {
    this.screenMaterial.color.setHex(fresh || this.screenMaterial.map !== this.video ? 0xffffff : 0x5a5a5a);
    drawStrip(this.strip.context, strip);
    this.strip.texture.needsUpdate = true;
    drawPanel(this.panelCanvas.context, rows, hints);
    this.panelCanvas.texture.needsUpdate = true;
  }
}
