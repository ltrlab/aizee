// The wrist cameras' pictures on Minerva's model in the headset (xr_scene.js): each where its camera is on the
// model's wrist, a window a little in front of the lens as wide as the camera sees, so it moves and turns with the
// real wrist (the user, 2026-11-03: not floating above the hands, facing the eyes). Each is squared to its gripper's
// axis (the rig's picture axis): perpendicular to the wrist, not tilted as the camera looks (the user, 2026-11-03),
// keeping the camera's side to side. Each camera's mount on its link (T_link_camera, optical axes x right, y down,
// z ahead), field of view and axis come from Helm (/api/helm/xr/wrists: the newest wrist calibration's mount, else
// the nominal one). Shown only while the model's wrists are in the room (the world-locked 3D); the pictures upright
// as each camera takes them. No socket here: xr.js streams the pictures (xr_camera.js).
import * as THREE from 'three';

export const WRIST_W = .2;  // Metres: each picture's width; it sits as far ahead of the lens as fills the view.
export const CAMERAS = ['left_wrist', 'right_wrist'];
const FLIP = new THREE.Matrix4().makeScale(1, -1, -1);  // Optical axes to the plane's (x right, y up, facing back).

// A mount as Helm sends it (four rows of four) as a matrix.
export function mountMatrix(rows, target = new THREE.Matrix4()) {
  return target.set(...rows.flat());
}

// A mount (T_link_camera) squared to `axis` (the gripper's, in the link's frame): at the lens, looking along the axis
// (ahead of the lens, not behind it), its x the camera's own made perpendicular to the axis, so the picture loses only
// the camera's tilt. The mount as it is when the camera's x runs along the axis.
export function squared(mount, axis, target = new THREE.Matrix4()) {
  const [x, y, z, lens] = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
  mount.extractBasis(x, y, z);
  lens.setFromMatrixPosition(mount);
  const ahead = new THREE.Vector3().fromArray(axis).normalize();
  if (ahead.dot(z) < 0) ahead.negate();
  x.addScaledVector(ahead, -x.dot(ahead));
  if (x.lengthSq() < 1e-9) return target.copy(mount);
  x.normalize();
  y.crossVectors(ahead, x);
  return target.makeBasis(x, y, ahead).setPosition(lens);
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

  // Helm's mounts ({role: {link, matrix, fov_deg, axis?}}): each screen's place on its link, sized to its camera's
  // view, squared to the axis where one is given.
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
      const axis = Array.isArray(mount.axis) && mount.axis.length === 3 && mount.axis.every(Number.isFinite)
        ? mount.axis : null;
      screen.local.copy(axis ? squared(screen.mount, axis) : screen.mount)
        .multiply(new THREE.Matrix4().makeTranslation(0, 0, distance)).multiply(FLIP);
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
