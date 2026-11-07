// A black outline around the model and its parts, for Helm's RetroUI look (joint-telemetry.sceneOutline). Each frame
// the model's visual meshes (on OUTLINE_LAYER) are drawn once more into a target, as view-space normals with their
// depth; a full-screen pass then draws the outline over the picture wherever the depth jumps (the silhouettes: the
// model against the background, a part in front of another) or the surface turns sharply (the parts' edges). Only
// the visual meshes: never the grid, collision shapes, axes or the joint overlay. The model is millions of triangles,
// so its depth and normals are drawn again only when the camera, the canvas or the model changed (invalidate(): a
// new pose, a mesh loaded); otherwise only the full-screen pass runs.
import * as THREE from 'three';

export const OUTLINE_LAYER = 3;

// Whether two camera matrices are the same view: the orbit controls' damping keeps nudging the camera by amounts
// far below a pixel, which must not redraw the model.
function same(a, b) {
  for (let i = 0; i < 16; i++) if (Math.abs(a.elements[i] - b.elements[i]) > 1e-6) return false;
  return true;
}

const VERTEX = `varying vec2 vUv;
void main() { vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }`;

const FRAGMENT = `#include <packing>
uniform sampler2D tNormal;
uniform sampler2D tDepth;
uniform vec2 stride;  // The outline's half width, in texture coordinates.
uniform float near, far, depthEdge, normalEdge, opacity;
uniform vec3 color;
varying vec2 vUv;

float distanceAt(vec2 uv) {  // Positive view distance; the far plane where nothing was drawn.
  return -perspectiveDepthToViewZ(texture2D(tDepth, uv).x, near, far);
}
vec3 normalAt(vec2 uv) { return unpackRGBToNormal(texture2D(tNormal, uv).rgb); }

void main() {
  float d = distanceAt(vUv);
  bool model = texture2D(tDepth, vUv).x < 1.0;
  vec3 n = normalAt(vUv);
  float edge = 0.0;
  vec2 offsets[8];
  offsets[0] = vec2(1.0, 0.0); offsets[1] = vec2(-1.0, 0.0); offsets[2] = vec2(0.0, 1.0); offsets[3] = vec2(0.0, -1.0);
  offsets[4] = vec2(0.7, 0.7); offsets[5] = vec2(-0.7, 0.7); offsets[6] = vec2(0.7, -0.7); offsets[7] = vec2(-0.7, -0.7);
  for (int i = 0; i < 8; i++) {
    vec2 uv = vUv + offsets[i] * stride;
    bool other = texture2D(tDepth, uv).x < 1.0;
    if (model != other) { edge = 1.0; break; }  // The silhouette against the background.
    if (!model) continue;
    float dn = distanceAt(uv);
    edge = max(edge, smoothstep(depthEdge, depthEdge * 2.0, abs(dn - d) / min(d, dn)));
    edge = max(edge, smoothstep(normalEdge, normalEdge + 0.25, 1.0 - dot(n, normalAt(uv))));
  }
  if (edge <= 0.0) discard;
  gl_FragColor = vec4(color, opacity * edge);
}`;

export class Outline {
  constructor(renderer) {
    this.renderer = renderer;
    this.target = new THREE.WebGLRenderTarget(1, 1, { depthTexture: new THREE.DepthTexture(1, 1) });
    this.normals = new THREE.MeshNormalMaterial({ side: THREE.DoubleSide });
    this.material = new THREE.ShaderMaterial({
      uniforms: {
        tNormal: { value: this.target.texture },
        tDepth: { value: this.target.depthTexture },
        stride: { value: new THREE.Vector2() },
        near: { value: 0.1 },
        far: { value: 100 },
        depthEdge: { value: 0.02 },
        normalEdge: { value: 0.5 },
        opacity: { value: 1 },
        color: { value: new THREE.Color() },
      },
      vertexShader: VERTEX,
      fragmentShader: FRAGMENT,
      transparent: true,
      depthTest: false,
      depthWrite: false,
    });
    const quad = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), this.material);
    quad.frustumCulled = false;
    this.overlay = new THREE.Scene();
    this.overlay.add(quad);
    this.flat = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this.size = new THREE.Vector2();
    this.clear = new THREE.Color();
    this.view = new THREE.Matrix4(); // The camera the target was drawn from.
    this.projection = new THREE.Matrix4();
    this.stale = true;
  }

  // The model moved or changed: its depth and normals are drawn again at the next frame.
  invalidate() {
    this.stale = true;
  }

  // After the picture is drawn: the outline over it. `settings`: {color, width (CSS px), depth, normal, opacity}.
  render(scene, camera, settings) {
    const { renderer, target, material } = this;
    renderer.getDrawingBufferSize(this.size);
    const resized = target.width !== this.size.x || target.height !== this.size.y;
    if (resized) target.setSize(this.size.x, this.size.y);
    if (
      this.stale ||
      resized ||
      !same(this.view, camera.matrixWorld) ||
      !same(this.projection, camera.projectionMatrix)
    ) {
      this.draw(scene, camera);
      this.view.copy(camera.matrixWorld);
      this.projection.copy(camera.projectionMatrix);
      this.stale = false;
    }
    const width = settings.width * renderer.getPixelRatio();
    const u = material.uniforms;
    u.stride.value.set(width / this.size.x, width / this.size.y);
    u.near.value = camera.near;
    u.far.value = camera.far;
    u.depthEdge.value = settings.depth;
    u.normalEdge.value = settings.normal;
    u.opacity.value = settings.opacity ?? 1;
    u.color.value.setHex(settings.color);
    const autoClear = renderer.autoClear;
    renderer.autoClear = false;
    renderer.render(this.overlay, this.flat);
    renderer.autoClear = autoClear;
  }

  // The visual meshes' view-space normals and depth, into the target.
  draw(scene, camera) {
    const { renderer, target } = this;
    const mask = camera.layers.mask,
      override = scene.overrideMaterial,
      background = scene.background;
    const clearAlpha = renderer.getClearAlpha();
    renderer.getClearColor(this.clear);
    camera.layers.set(OUTLINE_LAYER);
    scene.overrideMaterial = this.normals;
    scene.background = null;
    renderer.setRenderTarget(target);
    renderer.setClearColor(0x000000, 0);
    renderer.clear();
    renderer.render(scene, camera);
    renderer.setRenderTarget(null);
    renderer.setClearColor(this.clear, clearAlpha);
    camera.layers.mask = mask;
    scene.overrideMaterial = override;
    scene.background = background;
  }
}
