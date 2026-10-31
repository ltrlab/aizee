// Minerva's model in the headset (xr_scene.js), as GHOST draws Spot's: the display model Helm's 3D view draws
// (/api/models), its meshes coarse for the headset (?detail=coarse: vertices merged within 5 mm, viewer/server.py),
// posed by the positions Helm's 3D view follows (/api/calibration-guide), and placed in the room as the 3D picture is
// (M, xr_depth.js): the model's arms lie on the arms in the 3D picture, and show where the camera does not see. Where
// a part of it and the depth meet, the model is drawn on top; a real surface in front of it still hides it. The
// head's own links (from neck_z on) are left out: they would sit around the operator's eyes. Read-only: nothing here
// moves the robot.
import * as THREE from 'three';

export const MODEL_OPACITY = .55;  // Of the picture's own opacity: the depth shows through it.
export const GUIDE_MS = 100;  // As Helm's 3D view asks.
export const RETRY_MS = 5000;  // A model that failed to load is asked for again this long after.
const LOADERS = 4;

// The links the model leaves out: the head's, from its pan joint on.
export function headLinks(model, joint = 'neck_z') {
  const children = new Map();
  for (const item of model.joints) children.set(item.parent, [...(children.get(item.parent) || []), item.child]);
  const pan = model.joints.find(item => item.name === joint);
  const links = new Set();
  for (const pending = pan ? [pan.child] : []; pending.length;) {
    const link = pending.pop();
    links.add(link);
    pending.push(...(children.get(link) || []));
  }
  return links;
}

// A mesh as Helm sends it (viewer/server.mesh_payload): vertex and face counts, float32 vertices, uint32 faces.
export function meshGeometry(buffer) {
  const header = new DataView(buffer), nv = header.getUint32(0, true), nf = header.getUint32(4, true);
  if (buffer.byteLength !== 8 + nv * 12 + nf * 12) throw new Error('Incomplete mesh');
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buffer, 8, nv * 3), 3));
  geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer, 8 + nv * 12, nf * 3), 1));
  geometry.computeVertexNormals();
  return geometry;
}

export class RobotModel {
  // `fetch`, the timers and the clock (ms): the browser's (tests pass their own). Each is wrapped: called as this
  // object's method, the browser's own setTimeout throws "Illegal invocation" (it wants the window), and the model
  // never followed Helm's guide in the headset (2026-10-31).
  constructor({fetch: get = (...args) => fetch(...args), setTimer = (fn, ms) => setTimeout(fn, ms),
    clearTimer = id => clearTimeout(id), now = () => performance.now()} = {}) {
    Object.assign(this, {get, setTimer, clearTimer, now});
    this.group = new THREE.Group();
    this.group.visible = false;
    this.group.matrixAutoUpdate = false;
    this.motions = new Map();
    this.materials = [];
    this.model = null;
    this.loading = null;
    this.error = null;
    this.failedAt = null;
    this.opacity = 1;
    this.tracking = false;
    this.timer = null;
    this.posed = null;  // The guide's last pose applied (its stream and sequence).
  }

  async json(url) {
    const response = await this.get(url, {cache: 'no-store'});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
    return data;
  }

  // The model, built once: its links and joints as the 3D view builds them (rpy about fixed axes), its visual meshes.
  // After a failure, asked again only once RETRY_MS has passed.
  load() {
    if (!this.loading && this.failedAt !== null && this.now() - this.failedAt < RETRY_MS) {
      return Promise.reject(new Error(this.error));
    }
    this.loading ||= (async () => {
      const catalog = await this.json('/api/models');
      const model = await this.json(`/api/models/${encodeURIComponent(catalog.selected)}`);
      const links = new Map(model.links.map(name => [name, new THREE.Group()]));
      const root = new THREE.Group();
      root.add(links.get(model.root));
      for (const joint of model.joints) {
        const frame = new THREE.Group(), motion = new THREE.Group();
        frame.position.fromArray(joint.origin.xyz);
        frame.rotation.set(...joint.origin.rpy, 'ZYX');
        links.get(joint.parent).add(frame);
        frame.add(motion);
        motion.add(links.get(joint.child));
        this.motions.set(joint.name, motion);
      }
      const hidden = headLinks(model);
      const visuals = model.geometry.filter(item => item.role === 'visual' && item.shape === 'mesh'
        && !hidden.has(item.link));
      let next = 0;
      const loader = async () => {
        while (next < visuals.length) {
          const item = visuals[next++];
          const response = await this.get(`/api/models/${encodeURIComponent(model.id)}/meshes/${item.asset}?detail=coarse`);
          if (!response.ok) throw new Error(`Mesh ${item.asset}: HTTP ${response.status}`);
          const material = new THREE.MeshLambertMaterial({
            color: new THREE.Color().setRGB(...item.color.slice(0, 3), THREE.SRGBColorSpace), transparent: true,
            opacity: MODEL_OPACITY * this.opacity, depthWrite: false, side: THREE.DoubleSide,
            polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2});
          const mesh = new THREE.Mesh(meshGeometry(await response.arrayBuffer()), material);
          mesh.position.fromArray(item.origin.xyz);
          mesh.rotation.set(...item.origin.rpy, 'ZYX');
          if (item.scale) mesh.scale.fromArray(item.scale);
          links.get(item.link).add(mesh);
          this.materials.push(material);
        }
      };
      await Promise.all(Array.from({length: LOADERS}, loader));
      this.group.add(root);
      this.model = model;
      this.error = null;
      return model;
    })().catch(error => {
      this.error = error.message;
      this.failedAt = this.now();
      this.loading = null;  // Asked again later, it tries again.
      throw error;
    });
    return this.loading;
  }

  // The joints' positions ({joint: value}: Helm's guide, its independent joints), the coupled ones from them.
  pose(positions) {
    if (!this.model) return;
    const values = {...positions};
    for (const name of this.model.mimic_order) {
      const mimic = this.model.joints.find(joint => joint.name === name).mimic;
      values[name] = values[mimic.joint] * mimic.multiplier + mimic.offset;
    }
    for (const joint of this.model.joints) {
      const motion = this.motions.get(joint.name), value = values[joint.name];
      if (joint.type === 'fixed' || !Number.isFinite(value)) continue;
      if (joint.type === 'prismatic') motion.position.fromArray(joint.axis).multiplyScalar(value);
      else motion.quaternion.setFromAxisAngle(new THREE.Vector3(...joint.axis), value);
    }
  }

  // Following Helm's guide while shown: its newest fresh pose, as the 3D view takes it.
  track(on) {
    if (on === this.tracking) return;
    this.tracking = on;
    this.clearTimer(this.timer);
    if (on) this.poll();
  }

  async poll() {
    try {
      const guide = await this.json('/api/calibration-guide');
      const identity = `${guide.stream_id || ''}:${guide.sequence}`;
      if (this.model && guide.model === this.model.id && guide.urdf_sha256 === this.model.urdf_sha256
          && !guide.stale && identity !== this.posed) {
        this.pose(guide.positions);
        this.posed = identity;
      }
    } catch {
      // No guide (Helm without one, or a moment unreachable): the model keeps its last pose.
    }
    if (this.tracking) this.timer = this.setTimer(() => this.poll(), GUIDE_MS);
  }

  // Where the robot's frame is in the room (M), or null to hide it.
  place(matrix) {
    this.group.visible = !!matrix && !!this.model;
    if (matrix) this.group.matrix.copy(matrix);
    this.group.matrixWorldNeedsUpdate = true;
  }

  setOpacity(opacity) {
    this.opacity = opacity;
    for (const material of this.materials) material.opacity = MODEL_OPACITY * opacity;
  }
}
