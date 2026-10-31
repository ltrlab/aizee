// Minerva's splat map in the headset (docs/development/HEADSET_3D_PLAN.md, phase 4): the 3D Gaussians the trainer on
// soliloquy builds from the head camera's posed captures (scripts/splat/train_head_scene.py), drawn around the live
// 3D picture, as GHOST draws its scene. A map lies in its world: the table's board once the head camera is aligned
// on its tags (helm.head_alignment), else Minerva's own frame. So it is placed by Minerva's pose in that world (Helm's
// maps listing says it: fixed while the base stands, from odometry once it drives) and the zero mapping M that puts
// the robot in the room: room = M T_world_robot^-1, and a driving Minerva leaves it where it is.
// Each Gaussian is a quad facing the eye, sized by its covariance projected for that eye (the 3DGS rasterizer's
// approximation: the perspective's Jacobian at its centre), reaching three standard deviations, its colour and
// opacity fading as the Gaussian does, blended back to front. The order comes from a worker (xr_splat_worker.js),
// sorted again whenever the view has turned or moved. The live 3D and the model draw over it. Read-only: nothing
// here moves the robot.
import * as THREE from 'three';
import {ROW, TEXELS, parseSplat} from './xr_splat_sort.js';

export const SORT_MS = 50;  // At most one sort this often.
export const MAPS_MS = 10000;  // Helm's maps asked again this often while the map shows: a newer one replaces it.

const VERTEX = `
precision highp float;
precision highp int;
precision highp sampler2D;
uniform sampler2D splats;
uniform vec2 focal;
uniform vec2 viewport;
uniform float opacity;
attribute float splatIndex;
varying vec4 vColor;
varying vec2 vPosition;
const float EXTENT = 3.0;  // Each quad reaches three standard deviations.

void main() {
  int i = int(splatIndex + 0.5);
  ivec2 base = ivec2((i % ${ROW}) * ${TEXELS}, i / ${ROW});
  vec4 t0 = texelFetch(splats, base, 0);
  vec4 t1 = texelFetch(splats, base + ivec2(1, 0), 0);
  vec4 t2 = texelFetch(splats, base + ivec2(2, 0), 0);
  vec4 t3 = texelFetch(splats, base + ivec2(3, 0), 0);
  vec4 view = modelViewMatrix * vec4(t0.xyz, 1.0);
  vec4 clip = projectionMatrix * view;
  float bound = 1.2 * clip.w;
  if (view.z > -0.05 || clip.x < -bound || clip.x > bound || clip.y < -bound || clip.y > bound) {
    gl_Position = vec4(0.0, 0.0, 2.0, 1.0);  // Behind the eye or well off its view: not drawn.
    return;
  }
  // The covariance seen by the eye: pixels (right, up) from its centre, by the perspective's Jacobian there.
  float depth = -view.z;
  mat3 J = mat3(focal.x / depth, 0.0, 0.0, 0.0, focal.y / depth, 0.0,
                focal.x * view.x / (depth * depth), focal.y * view.y / (depth * depth), 0.0);
  mat3 T = J * mat3(modelViewMatrix);
  mat3 sigma = mat3(t1.x, t1.y, t1.z, t1.y, t2.x, t2.y, t1.z, t2.y, t2.z);
  mat3 cov = T * sigma * transpose(T);
  float a = cov[0][0] + 0.3, b = cov[0][1], d = cov[1][1] + 0.3;  // A pixel's worth of blur, as the rasterizer.
  float mid = 0.5 * (a + d), radius = length(vec2(0.5 * (a - d), b));
  float major = mid + radius, minor = max(mid - radius, 0.1);
  vec2 axis = abs(b) > 1e-9 ? normalize(vec2(b, major - a)) : (a >= d ? vec2(1.0, 0.0) : vec2(0.0, 1.0));
  vec2 majorAxis = min(sqrt(major), 1024.0) * axis;
  vec2 minorAxis = min(sqrt(minor), 1024.0) * vec2(-axis.y, axis.x);  // A quarter turn on: the quad faces the eye.
  vPosition = position.xy * EXTENT;
  vColor = vec4(t1.w, t2.w, t3.x, t0.w * opacity);
  vec2 offset = (position.x * majorAxis + position.y * minorAxis) * EXTENT * 2.0 / viewport;
  gl_Position = vec4(clip.xy / clip.w + offset, clip.z / clip.w, 1.0);
}`;

// GLSL 3: three.js declares no output for it (r160), so the shader does.
const FRAGMENT = `
precision highp float;
varying vec4 vColor;
varying vec2 vPosition;
out highp vec4 fragColor;

void main() {
  float distance = dot(vPosition, vPosition);  // In standard deviations, squared.
  if (distance > 9.0) discard;
  float alpha = exp(-0.5 * distance) * vColor.a;
  if (alpha < 1.0 / 255.0) discard;
  fragColor = vec4(vColor.rgb * alpha, alpha);  // Premultiplied, blended back to front.
}`;

// The map to draw from Helm's listing ({maps, robot_in_world}: /api/helm/maps, the newest first): the newest whose
// world the headset can place (Minerva's own frame always; the board's once the head camera is aligned), with
// Minerva's pose in that world (16 numbers, column-major as three.js reads them). Null when none can be placed.
export function chooseMap(listing) {
  for (const map of listing?.maps || []) {
    const pose = listing.robot_in_world?.[map.world];
    if (Array.isArray(pose) && pose.length === 16 && pose.every(Number.isFinite)) return {map, robotInWorld: pose};
  }
  return null;
}

export class SplatMap {
  // `fetch`, the clock (ms) and the sorting worker: the browser's (tests pass their own). Each is wrapped, as the
  // headset's model learned (xr_model.js): the browser's own, called as a method, throws "Illegal invocation".
  constructor({fetch: get = (...args) => fetch(...args), now = () => performance.now(),
    createWorker = () => new Worker(new URL('./xr_splat_worker.js', import.meta.url), {type: 'module'})} = {}) {
    Object.assign(this, {get, now, createWorker});
    this.group = new THREE.Group();
    this.group.matrixAutoUpdate = false;
    this.group.visible = false;
    this.mesh = null;
    this.name = null;
    this.count = 0;
    this.error = null;
    this.loading = false;
    this.wanted = false;
    this.room = null;  // M: the robot in the room, while the world-locked 3D shows.
    this.robotInWorld = new THREE.Matrix4();
    this.opacity = 1;
    this.worker = null;
    this.generation = 0;
    this.sorting = false;
    this.sortedAt = -Infinity;
    this.lastView = null;
    this.listedAt = -Infinity;
    this.listing = null;
    this.inverse = new THREE.Matrix4();
    this.modelView = new THREE.Matrix4();
    this.size = new THREE.Vector2();
  }

  // The map's state for the status panel (xr_status.panelRows).
  get state() {
    return {on: this.wanted, shown: this.group.visible, name: this.name, count: this.count, error: this.error,
      loading: this.loading};
  }

  // Helm's maps: the one to draw (chooseMap), loaded when it is not the one drawn.
  async refresh() {
    this.listedAt = this.now();
    try {
      const response = await this.get('/api/helm/maps', {cache: 'no-store'});
      const listing = await response.json();
      if (!response.ok) throw new Error(listing.error || `HTTP ${response.status}`);
      const chosen = chooseMap(listing);
      if (!chosen) {
        throw new Error(listing.maps?.length ? 'no map in a world the headset can place: align the head camera'
          : 'no map yet');
      }
      this.robotInWorld.fromArray(chosen.robotInWorld);
      if (chosen.map.name !== this.name) await this.load(chosen.map);
      this.error = null;
    } catch (error) {
      this.error = error.message;
    }
    this.apply();
  }

  async load(map) {
    this.loading = true;
    try {
      const response = await this.get(`/api/helm/maps/${encodeURIComponent(map.name)}/map.splat`);
      if (!response.ok) throw new Error(`map ${map.name}: HTTP ${response.status}`);
      this.build(parseSplat(await response.arrayBuffer()));
      this.name = map.name;
    } finally {
      this.loading = false;
    }
  }

  // The drawing of a map's Gaussians (parseSplat): one instanced quad each, read from a float texture by index.
  build({count, centres, data, rows}) {
    this.dispose();
    const texture = new THREE.DataTexture(data, ROW * TEXELS, rows, THREE.RGBAFormat, THREE.FloatType);
    texture.minFilter = texture.magFilter = THREE.NearestFilter;
    texture.generateMipmaps = false;
    texture.needsUpdate = true;
    const geometry = new THREE.InstancedBufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1,
      0]), 3));
    geometry.setIndex([0, 1, 2, 0, 2, 3]);
    const order = new THREE.InstancedBufferAttribute(new Float32Array(count).map((_, i) => i), 1);
    order.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('splatIndex', order);
    geometry.instanceCount = count;
    const material = new THREE.ShaderMaterial({
      glslVersion: THREE.GLSL3, vertexShader: VERTEX, fragmentShader: FRAGMENT, transparent: true, depthWrite: false,
      side: THREE.DoubleSide,  // Either way round, a Gaussian is seen.
      uniforms: {splats: {value: texture}, focal: {value: new THREE.Vector2(1, 1)},
        viewport: {value: new THREE.Vector2(1, 1)}, opacity: {value: this.opacity}},
      blending: THREE.CustomBlending, blendEquation: THREE.AddEquation, blendSrc: THREE.OneFactor,
      blendDst: THREE.OneMinusSrcAlphaFactor, blendSrcAlpha: THREE.OneFactor,
      blendDstAlpha: THREE.OneMinusSrcAlphaFactor});
    this.mesh = new THREE.Mesh(geometry, material);
    this.mesh.frustumCulled = false;  // Its bounds are a quad's; each Gaussian leaves the view on its own.
    this.mesh.renderOrder = -1;  // Before the live 3D and the model, which draw over it.
    this.mesh.onBeforeRender = (renderer, _scene, camera) => this.eye(renderer, camera, material);
    this.group.add(this.mesh);
    this.count = count;
    this.generation += 1;
    this.lastView = null;
    this.sorting = false;
    if (!this.worker) {
      this.worker = this.createWorker();
      if (this.worker) this.worker.onmessage = ({data: reply}) => this.sorted(reply);
    }
    this.worker?.postMessage({centres, count}, [centres.buffer]);
  }

  // Each eye's focal length and viewport, in pixels (a WebXR eye's own viewport; a plain camera's, the drawing
  // buffer): a Gaussian's footprint is its covariance projected there.
  eye(renderer, camera, material) {
    const viewport = camera.viewport;
    if (viewport) this.size.set(viewport.z, viewport.w);
    else renderer.getDrawingBufferSize(this.size);
    const p = camera.projectionMatrix.elements;
    material.uniforms.viewport.value.copy(this.size);
    material.uniforms.focal.value.set(p[0] * this.size.x / 2, p[5] * this.size.y / 2);
    material.uniformsNeedUpdate = true;
  }

  // The map wanted (the bar's Map switch), or not: Helm's maps are asked for when it turns on.
  show(on) {
    this.wanted = on;
    if (on && !this.listing && this.now() - this.listedAt > MAPS_MS) this.list();
    this.apply();
  }

  list() {
    this.listing = this.refresh().finally(() => {
      this.listing = null;
    });
    return this.listing;
  }

  // Where the robot is in the room (M, from the world-locked 3D), or null to hide the map.
  place(room) {
    this.room = room ? (this.room || new THREE.Matrix4()).copy(room) : null;
    this.apply();
  }

  apply() {
    const shown = this.wanted && !!this.room && !!this.mesh;
    this.group.visible = shown;
    if (shown) this.group.matrix.multiplyMatrices(this.room, this.inverse.copy(this.robotInWorld).invert());
    this.group.matrixWorldNeedsUpdate = true;
  }

  setOpacity(opacity) {
    this.opacity = opacity;
    if (this.mesh) this.mesh.material.uniforms.opacity.value = opacity;
  }

  // Each frame, with the eyes' pose in the room (a Matrix4): Helm's maps now and then while it shows, and a new
  // order once the view has turned or moved since the last.
  update(eyes) {
    if (!this.wanted) return;
    if (!this.listing && this.now() - this.listedAt > MAPS_MS) this.list();
    if (!this.group.visible || !this.worker || this.sorting || this.now() - this.sortedAt < SORT_MS) return;
    const e = this.modelView.copy(eyes).invert().multiply(this.group.matrix).elements;
    const view = [e[2], e[6], e[10], e[14]];
    if (this.lastView && view.every((value, k) => Math.abs(value - this.lastView[k]) < 1e-3)) return;
    this.sorting = true;
    this.lastView = view;
    this.worker.postMessage({id: this.generation, view});
  }

  sorted({id, order}) {
    this.sorting = false;
    this.sortedAt = this.now();
    if (id !== this.generation || !this.mesh || order.length !== this.count) return;  // An earlier map's.
    const attribute = this.mesh.geometry.getAttribute('splatIndex');
    attribute.array.set(order);
    attribute.needsUpdate = true;
  }

  dispose() {
    if (!this.mesh) return;
    this.group.remove(this.mesh);
    this.mesh.geometry.dispose();
    this.mesh.material.uniforms.splats.value.dispose();
    this.mesh.material.dispose();
    this.mesh = null;
    this.count = 0;
    this.name = null;
  }
}
