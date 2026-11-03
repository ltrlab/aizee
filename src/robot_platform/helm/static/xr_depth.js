// The head camera's depth as the headset's 3D picture (xr_scene.js). A grid with a vertex per depth pixel, each put
// on the GPU where its reading says (the camera's intrinsics); coloured from the head camera's picture through the
// registration the camera publishes (camera_node: its color intrinsics and the depth sensor's pose in the color
// camera); and cut where neighbouring readings jump (an object's edge), so near and far never join. The depth comes
// filled (xr_fill.js): its small dropouts filled, and a background layer behind each near object's edge, drawn
// behind it. Each pixel is coloured from where its reading came from, filled ones a little darker. Behind it all, the
// colour picture itself at BACKDROP_M fills where the depth has no reading. The grid sits in the depth sensor's frame
// at its rig's origin, looking down -Z; the rig goes where the head camera saw from (world-locked, below) or where
// the eyes are (xr_scene.js).
import * as THREE from 'three';
import {EDGE_SHARE} from './xr_fill.js';

export {EDGE_SHARE};
// The colour picture behind the depth, just past its 2.5 m range, in the colour camera's own view: where the depth
// has no reading (too far, or behind something), the picture fills in (the user, 2026-10-31).
export const BACKDROP_M = 3;
export const FILL_SHADE = .75;  // A filled reading's colour, dimmed: guessed, not seen.

// The backdrop's corners in the rig's frame, with their texture coordinates: the colour picture's edges (pixel
// centres at whole numbers) at `distance` ahead of the colour camera, which sits where the registration puts it.
export function backdropCorners(colorK, toColor, distance = BACKDROP_M) {
  const [r, t] = [toColor.rotation, toColor.translation];
  const [w, h] = [colorK.width, colorK.height];
  return [[-.5, -.5], [w - .5, -.5], [w - .5, h - .5], [-.5, h - .5]].map(([u, v]) => {
    const c = [(u - colorK.cx) / colorK.fx * distance, (v - colorK.cy) / colorK.fy * distance, distance];
    const d = [0, 1, 2].map(j => r[0][j] * (c[0] - t[0]) + r[1][j] * (c[1] - t[1]) + r[2][j] * (c[2] - t[2]));
    return {position: [d[0], -d[1], -d[2]], uv: [(u + .5) / w, (v + .5) / h]};  // Depth optical: p = R^T (c - t).
  });
}

// A reading's point in the rig's frame (x right, y up, -z ahead): the vertex shader's placement, for the tests.
export function depthPoint(u, v, z, k) {
  return [(u - k.cx) / k.fx * z, -(v - k.cy) / k.fy * z, -z];
}

// Where a point of the depth sensor's optical frame (x right, y down, z ahead) lands in the color picture, as a
// texture coordinate (0 to 1, the top row at 0): the vertex shader's colouring, for the tests.
export function colorUv([x, y, z], colorK, toColor) {
  const [r, t] = [toColor.rotation, toColor.translation];
  const c = [0, 1, 2].map(i => r[i][0] * x + r[i][1] * y + r[i][2] * z + t[i]);
  return [(colorK.fx * c[0] / c[2] + colorK.cx + .5) / colorK.width,
    (colorK.fy * c[1] / c[2] + colorK.cy + .5) / colorK.height];
}

// World-locked: the 3D picture where the head camera saw it in the room, from the head's measured pose at each
// frame's capture (camera_video_worker: the colour camera's pose in the robot frame, and its pose at the head's
// zero). When the operator zeroes the head, the camera at its zero is where their eyes were, looking where they
// looked (`anchor`): M = anchor · F · T_zero⁻¹ takes the robot frame into the room's, F the turn from the optical
// axes (x right, y down, z ahead) to three.js's (x right, y up, -z ahead).
const FLIP = new THREE.Matrix4().makeScale(1, -1, -1);
const UNIT = new THREE.Vector3(1, 1, 1);

// The eyes' pose at the zero: where they were, turned to Helm's zero (its yaw and pitch, no roll).
export function zeroAnchor(eye, {yaw, pitch}, target = new THREE.Matrix4()) {
  return target.compose(new THREE.Vector3().copy(eye),
    new THREE.Quaternion().setFromEuler(new THREE.Euler(pitch, yaw, 0, 'YXZ')), UNIT);
}

// A pose [x, y, z, qx, qy, qz, qw] (neck.head_pose.pose7) as a matrix.
export function poseMatrix([x, y, z, qx, qy, qz, qw], target = new THREE.Matrix4()) {
  return target.compose(new THREE.Vector3(x, y, z), new THREE.Quaternion(qx, qy, qz, qw).normalize(), UNIT);
}

// M: the robot frame into the room's, from the zero's anchor and the camera's pose at the head's zero.
export function robotToWorld(anchor, zero, target = new THREE.Matrix4()) {
  return target.copy(anchor).multiply(FLIP).multiply(poseMatrix(zero).invert());
}

// The depth rig in the room for a frame's camera pose: M · T_robot_color · T_color_depth · F (the depth sensor where
// the registration puts it; with none, at the colour camera).
export function depthRig(world, pose, toColor, target = new THREE.Matrix4()) {
  target.copy(world).multiply(poseMatrix(pose));
  if (toColor) {
    const [r, t] = [toColor.rotation, toColor.translation];
    target.multiply(new THREE.Matrix4().set(r[0][0], r[0][1], r[0][2], t[0], r[1][0], r[1][1], r[1][2], t[1],
      r[2][0], r[2][1], r[2][2], t[2], 0, 0, 0, 1));
  }
  return target.multiply(FLIP);
}

const VERTEX = `
uniform highp sampler2D depthMap;  // Per pixel: its reading (raw units, 0 for none) and its source pixel's index.
uniform highp sampler2D sourceMap;  // The depth's own readings (the foreground layer): where a source pixel lies.
uniform vec4 depthK;
uniform float depthScale;
uniform vec4 colorK;
uniform vec2 colorSize;
uniform mat3 toColorR;
uniform vec3 toColorT;
uniform float edgeShare;
varying vec2 vColorUv;
varying float vKeep;
varying float vDepth;
varying float vFilled;

vec2 texel(ivec2 px, ivec2 size) {
  return texelFetch(depthMap, clamp(px, ivec2(0), size - 1), 0).rg;
}

float readingAt(ivec2 px, ivec2 size) {
  return texel(px, size).r * depthScale;
}

vec3 opticalAt(ivec2 px, float z) {
  return vec3((float(px.x) - depthK.z) / depthK.x * z, (float(px.y) - depthK.w) / depthK.y * z, z);
}

void main() {
  ivec2 size = textureSize(depthMap, 0);
  ivec2 px = ivec2(position.xy);
  vec2 here = texel(px, size);
  float z = here.r * depthScale;
  int index = int(here.g + 0.5);
  ivec2 source = ivec2(index % size.x, index / size.x);
  float keep = z > 0.0 ? 1.0 : 0.0;
  // Behind an edge (a neighbour much nearer), the far reading is cut: the triangles across the edge go, the near
  // surface keeps its whole outline.
  float limit = edgeShare * z;
  float zx = readingAt(px + ivec2(1, 0), size), zy = readingAt(px + ivec2(0, 1), size);
  float zl = readingAt(px - ivec2(1, 0), size), zu = readingAt(px - ivec2(0, 1), size);
  if ((zx > 0.0 && z - zx > limit) || (zy > 0.0 && z - zy > limit)
      || (zl > 0.0 && z - zl > limit) || (zu > 0.0 && z - zu > limit)) keep = 0.0;
  vec3 optical = opticalAt(px, z);
  // Coloured from its source pixel, where that one's own reading puts it: a filled pixel never takes the colour of a
  // near object in front (at the filled depth, the source's ray would land elsewhere in the colour picture).
  float zs = texelFetch(sourceMap, source, 0).r * depthScale;
  vec3 c = toColorR * opticalAt(source, zs > 0.0 ? zs : z) + toColorT;
  vColorUv = vec2((colorK.x * c.x / c.z + colorK.z + 0.5) / colorSize.x,
                  (colorK.y * c.y / c.z + colorK.w + 0.5) / colorSize.y);
  vKeep = keep;
  vDepth = z;
  vFilled = source == px ? 0.0 : 1.0;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(optical.x, -optical.y, -optical.z, 1.0);
}`;

const FRAGMENT = `
uniform sampler2D colorMap;
uniform float registered;
uniform float opacity;
uniform float fillShade;
uniform vec2 range;
varying vec2 vColorUv;
varying float vKeep;
varying float vDepth;
varying float vFilled;

void main() {
  if (vKeep < 0.999) discard;  // A triangle touching no reading or an edge.
  vec3 color;
  bool inside = all(greaterThanEqual(vColorUv, vec2(0.0))) && all(lessThanEqual(vColorUv, vec2(1.0)));
  if (registered > 0.5 && inside) {
    color = texture2D(colorMap, vColorUv).rgb;
  } else if (registered > 0.5) {
    // Outside the colour picture (the depth sees wider): left out. Drawn grey, the nearest came out nearly white and
    // speckled the view (the user, 2026-11-03).
    discard;
  } else {
    // No registration: a dim grey, a little lighter nearer, never mistaken for the scene's own colours.
    float far = clamp((vDepth - range.x) / (range.y - range.x), 0.0, 1.0);
    color = vec3(mix(0.45, 0.2, far));
  }
  gl_FragColor = vec4(color * mix(1.0, fillShade, vFilled), opacity);
  #include <colorspace_fragment>
}`;

function layerMaterial() {
  const material = new THREE.ShaderMaterial({
    uniforms: {
      depthMap: {value: null}, sourceMap: {value: null}, depthK: {value: new THREE.Vector4(1, 1, 0, 0)},
      depthScale: {value: .001},
      colorMap: {value: null}, colorK: {value: new THREE.Vector4(1, 1, 0, 0)}, colorSize: {value: new THREE.Vector2(1, 1)},
      toColorR: {value: new THREE.Matrix3()}, toColorT: {value: new THREE.Vector3()}, registered: {value: 0},
      edgeShare: {value: EDGE_SHARE}, opacity: {value: 1}, fillShade: {value: FILL_SHADE},
      range: {value: new THREE.Vector2(.15, 2.5)}},
    vertexShader: VERTEX, fragmentShader: FRAGMENT, side: THREE.DoubleSide});
  material.toneMapped = false;
  return material;
}

export class DepthView {
  constructor() {
    this.rig = new THREE.Group();
    this.rig.visible = false;
    this.size = null;
    this.mesh = this.behind = null;
    this.textures = null;
    // The depth with its small holes filled, and the background layer behind near objects' edges, drawn first.
    this.material = layerMaterial();
    this.behindMaterial = layerMaterial();
    this.materials = [this.material, this.behindMaterial];
    this.backdropMaterial = new THREE.MeshBasicMaterial({side: THREE.DoubleSide, toneMapped: false});
    const backdrop = new THREE.BufferGeometry();
    backdrop.setAttribute('position', new THREE.BufferAttribute(new Float32Array(12), 3));
    backdrop.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(8), 2));
    backdrop.setIndex([0, 3, 1, 1, 3, 2]);
    this.backdrop = new THREE.Mesh(backdrop, this.backdropMaterial);
    this.backdrop.frustumCulled = false;
    this.backdrop.visible = false;
    this.backdrop.renderOrder = -2;  // Farthest first, as a see-through picture needs.
    this.backdropKey = null;
    this.rig.add(this.backdrop);
    this.filled = this.behindCount = 0;  // The pixels each fill gave a reading, in the last frame.
  }

  // The colour picture behind the depth, where the registration puts the colour camera (only with one).
  placeBackdrop(colorK, toColor, colorTexture) {
    const key = JSON.stringify([colorK, toColor]);
    if (key !== this.backdropKey) {
      const {position, uv} = this.backdrop.geometry.attributes;
      backdropCorners(colorK, toColor).forEach((corner, i) => {
        position.setXYZ(i, ...corner.position);
        uv.setXY(i, ...corner.uv);
      });
      position.needsUpdate = uv.needsUpdate = true;
      this.backdropKey = key;
    }
    if (this.backdropMaterial.map !== colorTexture) {
      this.backdropMaterial.map = colorTexture;
      this.backdropMaterial.needsUpdate = true;
    }
  }

  // A grid of width x height vertices, each one's position its depth pixel (the shader places it), drawn once for
  // each layer.
  build(width, height) {
    const positions = new Float32Array(width * height * 3);
    for (let v = 0, i = 0; v < height; v++) {
      for (let u = 0; u < width; u++, i += 3) {
        positions[i] = u;
        positions[i + 1] = v;
      }
    }
    const indices = new Uint32Array((width - 1) * (height - 1) * 6);
    for (let v = 0, i = 0; v < height - 1; v++) {
      for (let u = 0; u < width - 1; u++, i += 6) {
        const a = v * width + u, b = a + 1, c = a + width, d = c + 1;
        indices.set([a, c, b, b, c, d], i);
      }
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    if (this.mesh) {
      this.mesh.geometry.dispose();
      this.rig.remove(this.mesh, this.behind);
    }
    for (const texture of Object.values(this.textures || {})) texture.dispose();
    const layer = () => {
      const texture = new THREE.DataTexture(new Float32Array(width * height * 2), width, height, THREE.RGFormat,
        THREE.FloatType);
      texture.minFilter = texture.magFilter = THREE.NearestFilter;
      return texture;
    };
    this.textures = {foreground: layer(), background: layer()};
    this.material.uniforms.depthMap.value = this.textures.foreground;
    this.behindMaterial.uniforms.depthMap.value = this.textures.background;
    for (const material of this.materials) material.uniforms.sourceMap.value = this.textures.foreground;
    this.mesh = new THREE.Mesh(geometry, this.material);
    this.behind = new THREE.Mesh(geometry, this.behindMaterial);
    this.behind.renderOrder = -1;
    for (const mesh of [this.mesh, this.behind]) {
      mesh.frustumCulled = false;  // Its positions are pixels until the shader places them.
      this.rig.add(mesh);
    }
    this.size = [width, height];
  }

  // A frame's depth (its metadata and filled layers, xr_fill.fillDepth) with the picture texture it was taken with.
  show(meta, layers, colorTexture) {
    if (!this.size || this.size[0] !== meta.width || this.size[1] !== meta.height) this.build(meta.width, meta.height);
    for (const [name, texture] of Object.entries(this.textures)) {
      texture.image.data.set(layers[name]);
      texture.needsUpdate = true;
    }
    this.filled = layers.filled || 0;
    this.behindCount = layers.behind || 0;
    const k = meta.intrinsics, ck = meta.color_intrinsics, toColor = meta.to_color;
    const registered = ck && toColor && colorTexture ? 1 : 0;
    for (const material of this.materials) {
      const u = material.uniforms;
      u.depthK.value.set(k.fx, k.fy, k.cx, k.cy);
      u.depthScale.value = meta.scale;
      u.range.value.set(...(meta.range_m || [.15, 2.5]));
      u.colorMap.value = colorTexture;
      u.registered.value = registered;
      if (registered) {
        u.colorK.value.set(ck.fx, ck.fy, ck.cx, ck.cy);
        u.colorSize.value.set(ck.width, ck.height);
        u.toColorR.value.set(...toColor.rotation.flat());  // Row by row; three keeps it column-major for the shader.
        u.toColorT.value.set(...toColor.translation);
      }
    }
    if (registered) this.placeBackdrop(ck, toColor, colorTexture);
    this.backdrop.visible = !!registered;
  }

  setOpacity(opacity) {
    for (const material of this.materials) material.uniforms.opacity.value = opacity;
    const transparent = opacity < 1;
    for (const material of [...this.materials, this.backdropMaterial]) {
      material.opacity = opacity;
      if (material.transparent !== transparent) {
        material.transparent = transparent;
        material.needsUpdate = true;
      }
    }
  }
}
