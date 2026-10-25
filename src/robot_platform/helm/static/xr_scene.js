// What the headset shows while it steers Minerva's head (xr.js): the head camera's picture on a screen with a status
// line under it, and a status panel. Either can be grabbed with a controller's grip button and moved; let go, it
// turns upright to face the viewer. Each controller shows its pointing ray (cyan over a screen, green holding one).
// They start in front of where the session began: WebXR's 'local' space has the head at its origin, looking down -Z.
// The screens' canvases are drawn by xr_panels.js, as Settings → Headset draws its flat copy.
import * as THREE from 'three';
import {SIZES, drawPanel, drawPlaceholder, drawStrip} from './xr_panels.js';

const SCREEN_W = 1.12, SCREEN_H = 0.84, STRIP_H = 0.075, PANEL_W = 0.56, PANEL_H = 0.42;
const RAY = {idle: 0x9aa3b2, aim: 0x4fd3ff, hold: 0x3fbf7f};

function drawable([width, height]) {
  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return {canvas, context: canvas.getContext('2d'), texture};
}

function panelMesh(width, height, texture) {
  return new THREE.Mesh(new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({map: texture, transparent: true, side: THREE.DoubleSide}));
}

export class HeadsetScene {
  constructor(renderer) {
    this.renderer = renderer;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(70, 1, .05, 50);  // WebXR's own cameras replace it while presenting.
    this.raycaster = new THREE.Raycaster();
    this.rotation = new THREE.Matrix4();
    this.head = new THREE.Vector3();

    // The camera screen: the picture (a placeholder until the first frame), and the status line under it.
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
    this.screen.add(new THREE.Mesh(picture, this.screenMaterial));
    this.strip = drawable(SIZES.strip);
    const strip = panelMesh(SCREEN_W, STRIP_H, this.strip.texture);
    strip.position.y = -(SCREEN_H + STRIP_H) / 2 - .01;
    this.screen.add(strip);
    this.writePlaceholder('Waiting for the head camera…');

    this.panel = new THREE.Group();
    this.panelCanvas = drawable(SIZES.panel);
    this.panel.add(panelMesh(PANEL_W, PANEL_H, this.panelCanvas.texture));

    this.panels = [this.screen, this.panel];
    for (const group of this.panels) this.scene.add(group);
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
      new THREE.LineBasicMaterial({color: RAY.idle, transparent: true, opacity: .75}));
    ray.scale.z = 2.5;
    ray.visible = false;
    controller.add(ray);
    controller.userData = {ray, held: null, pointer: false};
    controller.addEventListener('connected', event => {
      controller.userData.pointer = event.data?.targetRayMode === 'tracked-pointer';  // Controllers, not hands.
      ray.visible = controller.userData.pointer;
    });
    controller.addEventListener('disconnected', () => {
      this.release(controller);
      controller.userData.pointer = ray.visible = false;
    });
    controller.addEventListener('squeezestart', () => this.grab(controller));
    controller.addEventListener('squeezeend', () => this.release(controller));
    this.scene.add(controller);
    return controller;
  }

  // The screen or panel a controller points at, and how far: its ray against the free ones.
  aim(controller) {
    controller.updateWorldMatrix(true, false);  // This frame's pose: the scene's own update comes at render.
    this.rotation.identity().extractRotation(controller.matrixWorld);
    this.raycaster.ray.origin.setFromMatrixPosition(controller.matrixWorld);
    this.raycaster.ray.direction.set(0, 0, -1).applyMatrix4(this.rotation);
    const held = new Set(this.controllers.map(other => other.userData.held).filter(Boolean));
    const hits = this.raycaster.intersectObjects(this.panels.filter(group => !held.has(group)), true);
    if (!hits.length) return null;
    let group = hits[0].object;
    while (group && !this.panels.includes(group)) group = group.parent;
    return group ? {group, distance: hits[0].distance} : null;
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
  }

  // Each frame: the rays' colours and lengths.
  update() {
    for (const controller of this.controllers) {
      const {ray, held, pointer} = controller.userData;
      if (!pointer) continue;
      const hit = held ? null : this.aim(controller);
      ray.material.color.setHex(held ? RAY.hold : hit ? RAY.aim : RAY.idle);
      ray.scale.z = hit ? hit.distance : held ? .3 : 2.5;
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
