// The head camera's depth around the eyes: the headset's 3D picture (xr_scene.js). A grid with a vertex per depth
// pixel, each put on the GPU where its reading says (the camera's intrinsics); coloured from the head camera's
// picture through the registration the camera publishes (camera_node: its color intrinsics and the depth sensor's
// pose in the color camera); and cut where neighbouring readings jump (an object's edge), so near and far never
// join. Behind it, the colour picture itself at BACKDROP_M fills where the depth has no reading. Helm sends the
// depth after the picture's JPEG, compressed (camera_video_worker.head_depth); xr_camera.js
// inflates it. The grid sits in the camera's frame at its rig's origin, looking down -Z, as the head camera looks;
// the rig goes where the eyes are (xr_scene.js).
import * as THREE from 'three';

export const EDGE_SHARE = .04;  // Neighbouring readings further apart than this share of the depth: an edge.
// The colour picture behind the depth, just past its 2.5 m range, in the colour camera's own view: where the depth
// has no reading (too far, or behind something), the picture fills in (the user, 2026-10-31).
export const BACKDROP_M = 3;

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

const VERTEX = `
uniform highp sampler2D depthMap;
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

float readingAt(ivec2 px, ivec2 size) {
  return texelFetch(depthMap, clamp(px, ivec2(0), size - 1), 0).r * depthScale;
}

void main() {
  ivec2 size = textureSize(depthMap, 0);
  ivec2 px = ivec2(position.xy);
  float z = readingAt(px, size);
  float keep = z > 0.0 ? 1.0 : 0.0;
  // Behind an edge (a neighbour much nearer), the far reading is cut: the triangles across the edge go, the near
  // surface keeps its whole outline.
  float limit = edgeShare * z;
  float zx = readingAt(px + ivec2(1, 0), size), zy = readingAt(px + ivec2(0, 1), size);
  float zl = readingAt(px - ivec2(1, 0), size), zu = readingAt(px - ivec2(0, 1), size);
  if ((zx > 0.0 && z - zx > limit) || (zy > 0.0 && z - zy > limit)
      || (zl > 0.0 && z - zl > limit) || (zu > 0.0 && z - zu > limit)) keep = 0.0;
  vec3 optical = vec3((float(px.x) - depthK.z) / depthK.x * z, (float(px.y) - depthK.w) / depthK.y * z, z);
  vec3 c = toColorR * optical + toColorT;
  vColorUv = vec2((colorK.x * c.x / c.z + colorK.z + 0.5) / colorSize.x,
                  (colorK.y * c.y / c.z + colorK.w + 0.5) / colorSize.y);
  vKeep = keep;
  vDepth = z;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(optical.x, -optical.y, -optical.z, 1.0);
}`;

const FRAGMENT = `
uniform sampler2D colorMap;
uniform float registered;
uniform float opacity;
uniform vec2 range;
varying vec2 vColorUv;
varying float vKeep;
varying float vDepth;

void main() {
  if (vKeep < 0.999) discard;  // A triangle touching no reading or an edge.
  vec3 color;
  if (registered > 0.5 && all(greaterThanEqual(vColorUv, vec2(0.0))) && all(lessThanEqual(vColorUv, vec2(1.0)))) {
    color = texture2D(colorMap, vColorUv).rgb;
  } else {
    // Outside the color picture (the depth sees wider) or no registration: grey, lighter nearer, never mistaken for
    // the scene's own colours.
    float far = clamp((vDepth - range.x) / (range.y - range.x), 0.0, 1.0);
    color = vec3(mix(0.85, 0.25, far));
  }
  gl_FragColor = vec4(color, opacity);
  #include <colorspace_fragment>
}`;

export class DepthView {
  constructor() {
    this.rig = new THREE.Group();
    this.rig.visible = false;
    this.size = null;
    this.mesh = null;
    this.depthTexture = null;
    this.material = new THREE.ShaderMaterial({
      uniforms: {
        depthMap: {value: null}, depthK: {value: new THREE.Vector4(1, 1, 0, 0)}, depthScale: {value: .001},
        colorMap: {value: null}, colorK: {value: new THREE.Vector4(1, 1, 0, 0)}, colorSize: {value: new THREE.Vector2(1, 1)},
        toColorR: {value: new THREE.Matrix3()}, toColorT: {value: new THREE.Vector3()}, registered: {value: 0},
        edgeShare: {value: EDGE_SHARE}, opacity: {value: 1}, range: {value: new THREE.Vector2(.15, 2.5)}},
      vertexShader: VERTEX, fragmentShader: FRAGMENT, side: THREE.DoubleSide});
    this.material.toneMapped = false;
    this.backdropMaterial = new THREE.MeshBasicMaterial({side: THREE.DoubleSide, toneMapped: false});
    const backdrop = new THREE.BufferGeometry();
    backdrop.setAttribute('position', new THREE.BufferAttribute(new Float32Array(12), 3));
    backdrop.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(8), 2));
    backdrop.setIndex([0, 3, 1, 1, 3, 2]);
    this.backdrop = new THREE.Mesh(backdrop, this.backdropMaterial);
    this.backdrop.frustumCulled = false;
    this.backdrop.visible = false;
    this.backdropKey = null;
    this.rig.add(this.backdrop);
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

  // A grid of width x height vertices, each one's position its depth pixel (the shader places it).
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
      this.rig.remove(this.mesh);
    }
    this.depthTexture?.dispose();
    this.depthTexture = new THREE.DataTexture(new Float32Array(width * height), width, height, THREE.RedFormat,
      THREE.FloatType);
    this.depthTexture.minFilter = this.depthTexture.magFilter = THREE.NearestFilter;
    this.material.uniforms.depthMap.value = this.depthTexture;
    this.mesh = new THREE.Mesh(geometry, this.material);
    this.mesh.frustumCulled = false;  // Its positions are pixels until the shader places them.
    this.rig.add(this.mesh);
    this.size = [width, height];
  }

  // A frame's depth (its metadata and readings) with the picture texture it was taken with.
  show(meta, values, colorTexture) {
    if (!this.size || this.size[0] !== meta.width || this.size[1] !== meta.height) this.build(meta.width, meta.height);
    this.depthTexture.image.data.set(values);
    this.depthTexture.needsUpdate = true;
    const u = this.material.uniforms, k = meta.intrinsics;
    u.depthK.value.set(k.fx, k.fy, k.cx, k.cy);
    u.depthScale.value = meta.scale;
    u.range.value.set(...(meta.range_m || [.15, 2.5]));
    u.colorMap.value = colorTexture;
    const ck = meta.color_intrinsics, toColor = meta.to_color;
    u.registered.value = ck && toColor && colorTexture ? 1 : 0;
    if (u.registered.value) {
      u.colorK.value.set(ck.fx, ck.fy, ck.cx, ck.cy);
      u.colorSize.value.set(ck.width, ck.height);
      u.toColorR.value.set(...toColor.rotation.flat());  // Row by row; three keeps it column-major for the shader.
      u.toColorT.value.set(...toColor.translation);
      this.placeBackdrop(ck, toColor, colorTexture);
    }
    this.backdrop.visible = !!u.registered.value;
  }

  setOpacity(opacity) {
    this.material.opacity = opacity;
    this.material.uniforms.opacity.value = opacity;
    this.backdropMaterial.opacity = opacity;
    const transparent = opacity < 1;
    for (const material of [this.material, this.backdropMaterial]) {
      if (material.transparent !== transparent) {
        material.transparent = transparent;
        material.needsUpdate = true;
      }
    }
  }
}
