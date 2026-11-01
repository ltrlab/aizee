// The wrist cameras above the operator's hands (xr_scene.js), as GHOST floats Spot's gripper camera above the
// controllers: the operator's hands hold the GELLO leaders, so the headset tracks the hands (WebXR hand tracking) and
// floats each wrist camera's picture (Helm's left_wrist, right_wrist) a little above the matching hand, facing the
// eyes. A hand lost to tracking leaves its picture where it was, dimmed; without hand tracking, the controllers carry
// them. No socket here: xr.js streams the pictures (xr_camera.js).
import * as THREE from 'three';

export const WRIST_W = .26, WRIST_H = .195;  // Metres: the wrist cameras' 4:3 pictures.
export const ABOVE_M = .14;  // Above the wrist.
export const LOST_S = .5;  // A hand this long untracked: its picture dims where it was.
export const SIDES = {left: 'left_wrist', right: 'right_wrist'};
// The wrist cameras' pictures come as the cameras take them, the jaws at the bottom (Helm's camera cards, recordings
// and calibrations keep them so), and above the operator's hands they read upside down (user, 2026-11-01): each
// screen turns its picture a half turn.
export const TURNED = true;

export class WristScreens {
  constructor(renderer) {
    this.group = new THREE.Group();
    this.screens = {};
    for (const [side, role] of Object.entries(SIDES)) {
      const texture = new THREE.Texture();
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.flipY = false;  // ImageBitmaps upload top row first: the UVs are flipped instead.
      const geometry = new THREE.PlaneGeometry(WRIST_W, WRIST_H);
      const uv = geometry.attributes.uv;
      for (let i = 0; i < uv.count; i++) uv.setY(i, 1 - uv.getY(i));
      if (TURNED) for (let i = 0; i < uv.count; i++) uv.setXY(i, 1 - uv.getX(i), 1 - uv.getY(i));
      const material = new THREE.MeshBasicMaterial({map: texture, transparent: true, side: THREE.DoubleSide,
        depthTest: false, toneMapped: false});
      const mesh = new THREE.Mesh(geometry, material);
      mesh.renderOrder = 2;  // Over the 3D picture, as the screens are.
      mesh.visible = false;
      this.group.add(mesh);
      this.screens[side] = {role, mesh, material, texture, framed: false, seenAt: -Infinity};
    }
    // Hands and controllers, by handedness: a tracked hand's wrist first, else its controller.
    this.sources = [0, 1].flatMap(index => [[renderer.xr.getHand(index), true],
      [renderer.xr.getControllerGrip(index), false]].map(([object, hand]) => {
      const source = {object, hand, side: null};
      object.addEventListener('connected', event => { source.side = event.data?.handedness || null; });
      object.addEventListener('disconnected', () => { source.side = null; });
      return source;
    }));
    this.opacity = 1;
    this.at = new THREE.Vector3();
    this.above = new THREE.Vector3(0, ABOVE_M, 0);
  }

  // A wrist camera's newest frame (role left_wrist or right_wrist).
  showFrame(role, bitmap) {
    const screen = Object.values(this.screens).find(item => item.role === role);
    if (!screen) {
      bitmap?.close?.();
      return;
    }
    const previous = screen.texture.image;
    screen.texture.image = bitmap;
    screen.texture.needsUpdate = true;
    screen.framed = true;
    previous?.close?.();
  }

  // Where a side's hand is this frame (a Vector3, shared), or null: a tracked hand's wrist first, else its controller.
  anchor(side) {
    for (const hand of [true, false]) {
      const source = this.sources.find(item => item.side === side && item.hand === hand);
      const target = source && (hand ? source.object.joints?.wrist : source.object);
      if (target?.visible) return target.getWorldPosition(this.at);
    }
    return null;
  }

  // Each frame: each picture above its hand, facing the eyes (`eye`, a Vector3), dimmed once its hand is lost
  // (`now`: seconds).
  update(eye, now) {
    for (const [side, screen] of Object.entries(this.screens)) {
      const at = this.anchor(side);
      if (at) {
        screen.mesh.position.copy(at).add(this.above);
        screen.mesh.lookAt(eye);
        screen.seenAt = now;
      }
      screen.mesh.visible = screen.framed && Number.isFinite(screen.seenAt);
      screen.material.opacity = this.opacity * (now - screen.seenAt > LOST_S ? .35 : 1);
    }
  }

  setOpacity(opacity) {
    this.opacity = opacity;
  }
}
