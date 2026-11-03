// The wrist cameras' pictures on Minerva's model in the headset (xr_scene.js): each where its camera is on the
// model's wrist and along its view, a window a little in front of the lens as wide as the camera sees, so it moves
// and turns with the real wrist (the user, 2026-11-03: not floating above the hands, facing the eyes). Each camera's
// mount on its link (T_link_camera, optical axes x right, y down, z ahead) and field of view come from Helm
// (/api/helm/xr/wrists: the newest wrist calibration's, else the nominal mount). Shown only while the model's wrists
// are in the room (the world-locked 3D); the pictures upright as each camera takes them, since the screen turns with
// its camera. No socket here: xr.js streams the pictures (xr_camera.js).
import * as THREE from 'three';

export const WRIST_W = .2;  // Metres: each picture's width; it sits as far ahead of the lens as fills the view.
export const CAMERAS = ['left_wrist', 'right_wrist'];
const FLIP = new THREE.Matrix4().makeScale(1, -1, -1);  // Optical axes to the plane's (x right, y up, facing back).

// A mount as Helm sends it (four rows of four) as a matrix.
export function mountMatrix(rows, target = new THREE.Matrix4()) {
  return target.set(...rows.flat());
}

// How far ahead of the lens a picture WRIST_W wide fills the view, and its height: [distance, height] in metres,
// from the field of view in degrees ([horizontal, vertical]).
export function windowAt([horizontal, vertical], width = WRIST_W) {
  const distance = width / 2 / Math.tan(horizontal * Math.PI / 360);
  return [distance, 2 * distance * Math.tan(vertical * Math.PI / 360)];
}

export class WristScreens {
  constructor() {
    this.group = new THREE.Group();
    this.screens = {};
    for (const role of CAMERAS) {
      const texture = new THREE.Texture();
      texture.colorSpace = THREE.SRGBColorSpace;
      texture.flipY = false;  // ImageBitmaps upload top row first: the UVs are flipped instead.
      const material = new THREE.MeshBasicMaterial({map: texture, transparent: true, side: THREE.DoubleSide,
        depthTest: false, toneMapped: false});
      const mesh = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), material);
      mesh.matrixAutoUpdate = false;
      mesh.renderOrder = 2;  // Over the 3D picture and the model, as a screen is.
      mesh.visible = false;
      this.group.add(mesh);
      this.screens[role] = {mesh, material, texture, framed: false, mount: null, link: null,
        local: new THREE.Matrix4()};
    }
    this.opacity = 1;
    this.placed = false;
    this.link = new THREE.Matrix4();
  }

  // Helm's mounts ({role: {link, matrix, fov_deg}}): each screen's place on its link, sized to its camera's view.
  setMounts(mounts) {
    for (const [role, screen] of Object.entries(this.screens)) {
      const mount = mounts?.[role];
      if (!mount?.link || !Array.isArray(mount.matrix) || !Array.isArray(mount.fov_deg)) {
        screen.mount = null;
        continue;
      }
      const [distance, height] = windowAt(mount.fov_deg);
      const geometry = new THREE.PlaneGeometry(WRIST_W, height);
      const uv = geometry.attributes.uv;
      for (let i = 0; i < uv.count; i++) uv.setY(i, 1 - uv.getY(i));
      screen.mesh.geometry.dispose();
      screen.mesh.geometry = geometry;
      screen.link = mount.link;
      screen.mount = mountMatrix(mount.matrix);
      screen.local.copy(screen.mount).multiply(new THREE.Matrix4().makeTranslation(0, 0, distance)).multiply(FLIP);
    }
  }

  // A wrist camera's newest frame.
  showFrame(role, bitmap) {
    const screen = this.screens[role];
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

  // Each frame: each picture on its link where the model is (`robot`: xr_model.RobotModel, placed in the room), or
  // none while the model is not in the room (`placed` false).
  update(robot, placed) {
    this.placed = placed;
    for (const screen of Object.values(this.screens)) {
      const on = placed && screen.framed && !!screen.mount && !!robot.linkMatrix(screen.link, this.link);
      screen.mesh.visible = on;
      if (on) {
        screen.mesh.matrix.copy(this.link).multiply(screen.local);
        screen.mesh.matrixWorldNeedsUpdate = true;
      }
      screen.material.opacity = this.opacity;
    }
  }

  setOpacity(opacity) {
    this.opacity = opacity;
  }
}
