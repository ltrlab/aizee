import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import {OUTLINE, sceneOutline} from '../../src/robot_platform/viewer/static/joint-telemetry.js';

// Helm's RetroUI look outlines the model; its original look and the standalone viewer do not.
const page = look => ({documentElement: {dataset: look ? {look} : {}}});
assert.equal(sceneOutline(page('retro')), OUTLINE);
assert.equal(sceneOutline(page('classic')), null);
assert.equal(sceneOutline(page(null)), null);
assert.equal(OUTLINE.color, 0x000000);

// The outline pass (outline.js), on the vendored three.js, with a renderer that records what it draws.
const root = new URL('../../src/robot_platform/viewer/static/', import.meta.url);
const threeURL = `data:text/javascript;base64,${Buffer.from(await fs.readFile(new URL('vendor/three.module.js', root),
  'utf8')).toString('base64')}`;
const source = (await fs.readFile(new URL('outline.js', root), 'utf8')).replace("from 'three'", `from '${threeURL}'`);
const THREE = await import(threeURL);
const {Outline, OUTLINE_LAYER} = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);

const drawn = [];
let size = [1200, 800];
const renderer = {
  autoClear: true, target: null, clears: [],
  getDrawingBufferSize: vector => vector.set(...size),
  getPixelRatio: () => 1.5,
  getClearColor: color => color.setHex(0x223344),
  getClearAlpha: () => .5,
  setClearColor(color, alpha) {
    this.clears.push([new THREE.Color(color).getHex(), alpha]);
  },
  clear() {},
  setRenderTarget(target) {
    this.target = target;
  },
  render(scene, camera) {
    drawn.push({target: this.target, layers: camera.layers.mask, override: scene.overrideMaterial,
      autoClear: this.autoClear});
  },
};
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1.5, .001, 1000);
camera.position.set(0, -2, 1);
camera.lookAt(0, 0, 1);
camera.updateMatrixWorld(true);
const outline = new Outline(renderer);
const frame = () => {
  drawn.length = 0;
  camera.updateMatrixWorld(true);
  outline.render(scene, camera, OUTLINE);
  return drawn.map(pass => pass.target ? 'model' : 'outline');
};

// The first frame: the model's normals and depth (its layer only, the normal material), then the outline over the
// picture without clearing it; the camera, scene and renderer as they were.
assert.deepEqual(frame(), ['model', 'outline']);
assert.equal(drawn[0].target, outline.target);
assert.equal(drawn[0].layers, 1 << OUTLINE_LAYER);
assert.equal(drawn[0].override, outline.normals);
assert.equal(drawn[1].autoClear, false);
assert.deepEqual([camera.layers.mask, scene.overrideMaterial, renderer.autoClear, renderer.target], [1, null, true, null]);
assert.deepEqual(renderer.clears.at(-1), [0x223344, .5]);
assert.deepEqual([outline.target.width, outline.target.height], [1200, 800]);
const u = outline.material.uniforms;
assert.deepEqual([u.stride.value.x, u.stride.value.y], [1.5 / 1200, 1.5 / 800]);  // 1 CSS px at a 1.5 pixel ratio.
assert.equal(u.color.value.getHex(), 0x000000);

// Nothing moved: only the outline pass. The orbit controls' damping nudging the camera far below a pixel: the same.
assert.deepEqual(frame(), ['outline']);
camera.position.x += 1e-9;
assert.deepEqual(frame(), ['outline']);
// The camera moved a millimetre, the model moved (a pose), the canvas resized: the model is drawn again.
camera.position.x += .001;
assert.deepEqual(frame(), ['model', 'outline']);
assert.deepEqual(frame(), ['outline']);
outline.invalidate();
assert.deepEqual(frame(), ['model', 'outline']);
size = [1600, 900];
assert.deepEqual(frame(), ['model', 'outline']);
assert.deepEqual([outline.target.width, outline.target.height], [1600, 900]);
camera.zoom = 1.2;
camera.updateProjectionMatrix();
assert.deepEqual(frame(), ['model', 'outline']);

console.log('RetroUI outline: drawn over the picture, the model redrawn only when the view or the model changes.');
