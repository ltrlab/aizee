import * as THREE from 'three';
import {gaugeSpec, heatColor, rotaryBasis} from './joint-telemetry.js';

// Readings drawn on the model, display only. Around each joint's axis a dial shows its range (grey track), its
// reference (white tick), its travel from the reference (blue arc) and its position (white needle). A violet ring
// beside the needle grows with torque as a share of the receiver's limit; the hub at the joint is tinted by
// temperature as a share of its limit. One short label carries the position and the temperature, the latter in
// the hub's colour. Dials need a verified direction mapping, as motion arrows do; stale readings turn grey. A dial
// seen nearly edge-on hides; its hub and label stay.
const COLOR = {track:0x6b737c, reference:0xe9ebee, value:0x7cc4ff, needle:0xf4f6f8, stale:0x8a8f96, torque:0xc9b6ff};
const LABEL_PX = 22;
const EDGE_ON = .25;  // |axis · view| below this, a dial collapses to a line: hide it.

function material(color, opacity=1) {
  return new THREE.MeshBasicMaterial({color, transparent:true, opacity, depthTest:false, depthWrite:false,
    side:THREE.DoubleSide});
}

function dispose(object) {
  object.traverse(child => {
    child.geometry?.dispose();
    child.material?.map?.dispose();
    child.material?.dispose();
  });
  object.parent?.remove(object);
}

function arc(inner, outer, start, length, color, opacity) {
  const segments = Math.max(4, Math.ceil(Math.abs(length) / (Math.PI / 48)));
  const mesh = new THREE.Mesh(new THREE.RingGeometry(inner, outer, segments, 1, start, length), material(color, opacity));
  mesh.renderOrder = 9;
  return mesh;
}

class Label {
  constructor(doc) {
    this.canvas = doc.createElement('canvas');
    this.canvas.width = 320;
    this.canvas.height = 48;
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.sprite = new THREE.Sprite(new THREE.SpriteMaterial({map:this.texture, transparent:true, depthTest:false,
      depthWrite:false, sizeAttenuation:false}));
    this.sprite.center.set(0, .5);
    this.sprite.renderOrder = 12;
    this.drawn = null;
  }
  // `parts`: [{text, color}], drawn in a row with a separator between them.
  draw(parts, selected) {
    const key = JSON.stringify([parts, selected]);
    if (key === this.drawn) return;
    this.drawn = key;
    this.text = parts.map(part => part.text).join(' · ');
    const ctx = this.canvas.getContext?.('2d');
    if (!ctx) return;  // No canvas (tests): the sprite stays blank.
    const {width, height} = this.canvas;
    ctx.clearRect(0, 0, width, height);
    ctx.font = '600 26px system-ui, sans-serif';
    const textWidth = Math.min(width - 16, ctx.measureText(this.text).width);
    ctx.fillStyle = 'rgba(16,17,19,.84)';
    ctx.strokeStyle = selected ? '#7cc4ff' : 'rgba(255,255,255,.18)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect?.(2, 4, textWidth + 14, height - 8, 8);
    ctx.fill();
    ctx.stroke();
    ctx.textBaseline = 'middle';
    const scale = textWidth / Math.max(1, ctx.measureText(this.text).width);  // Squeezed to fit, as fillText would.
    let x = 9;
    parts.forEach((part, i) => {
      const text = i ? ` · ${part.text}` : part.text, w = ctx.measureText(text).width * scale;
      ctx.fillStyle = part.color;
      ctx.fillText(text, x, height / 2 + 1, w);
      x += w;
    });
    this.texture.needsUpdate = true;
  }
  size(camera, viewportHeight) {
    // With size attenuation off, a sprite's height on screen is scale × P11 × viewport height / 2.
    const height = LABEL_PX * 2 / (camera.projectionMatrix.elements[5] * Math.max(1, viewportHeight));
    this.sprite.scale.set(height * this.canvas.width / this.canvas.height, height, 1);
  }
}

class Gauge {
  constructor({frame, joint, radius, doc}) {
    Object.assign(this, {frame, joint, prismatic:joint.type === 'prismatic'});
    this.R = radius * .085;
    this.basis = rotaryBasis(joint.axis);
    this.root = new THREE.Group();
    this.root.renderOrder = 9;
    if (this.basis && !this.prismatic) {
      const {u, v, n} = this.basis;
      this.root.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(
        new THREE.Vector3(...u), new THREE.Vector3(...v), new THREE.Vector3(...n)));
    }
    frame.add(this.root);
    this.parts = new THREE.Group();
    this.root.add(this.parts);
    this.label = new Label(doc);
    this.root.add(this.label.sprite);
    this.signature = null;
  }
  // Rotary dial coordinates are joint angles: the dial's x axis is the basis u, and a joint angle q turns u
  // toward v about the joint axis, so a point at angle q moves with the child link.
  update(row, {selected, fields}) {
    const spec = gaugeSpec(row);
    const signature = JSON.stringify([spec.verified, spec.stale, spec.track, spec.reference,
      spec.value == null ? null : Math.round(spec.value / .002), spec.effort == null ? null : Math.round(spec.effort * 50),
      spec.heat == null ? null : Math.round(spec.heat * 100), selected, fields]);
    if (signature !== this.signature) {
      this.signature = signature;
      for (const child of [...this.parts.children]) dispose(child);
      if (this.prismatic) this.linear(spec, fields, selected);
      else this.rotary(spec, fields, selected);
    }
    const color = spec.stale ? '#a59b90' : spec.verified ? '#e9ebee' : '#c9cdd2';
    const parts = [];
    if (fields.position && spec.value != null) parts.push({text:spec.label, color});
    if (fields.temperature && spec.temperature != null) parts.push({text:spec.temperature, color:spec.stale || spec.heat == null
      ? color : `#${heatColor(spec.heat).toString(16).padStart(6, '0')}`});
    this.label.sprite.visible = parts.length > 0;
    this.label.draw(parts, selected);
  }
  rotary(spec, fields, selected) {
    const R = this.R, w = R * .07, live = spec.verified && !spec.stale, angle = spec.value;
    if (fields.temperature && spec.heat != null) {
      // A sphere, so the temperature shows from any side.
      const hub = new THREE.Mesh(new THREE.SphereGeometry(R * .3, 16, 12),
        material(spec.stale ? COLOR.stale : heatColor(spec.heat), .95));
      hub.userData.hub = true;
      hub.renderOrder = 10;
      this.parts.add(hub);
    }
    const labelAt = angle != null && spec.verified ? angle : 0;
    this.label.sprite.position.set(Math.cos(labelAt) * R * 1.45, Math.sin(labelAt) * R * 1.45, 0);
    if (!fields.position || !spec.verified) return;  // Unverified direction: the angle could be mirrored.
    const track = spec.track ? arc(R - w / 2, R + w / 2, spec.track[0], spec.track[1] - spec.track[0], COLOR.track, selected ? .8 : .5)
      : arc(R - w / 2, R + w / 2, 0, 2 * Math.PI, COLOR.track, selected ? .6 : .35);
    this.parts.add(track);
    if (spec.reference != null) {
      const tick = new THREE.Mesh(new THREE.PlaneGeometry(R * .32, w * .9), material(COLOR.reference, .9));
      tick.position.set(Math.cos(spec.reference) * R, Math.sin(spec.reference) * R, 0);
      tick.rotation.z = spec.reference;
      tick.renderOrder = 11;
      this.parts.add(tick);
    }
    if (angle == null) return;
    if (spec.reference != null && Math.abs(angle - spec.reference) > 1e-4) {
      const start = Math.min(spec.reference, angle);
      this.parts.add(arc(R - w * 1.1, R + w * 1.1, start, Math.abs(angle - spec.reference),
        live ? COLOR.value : COLOR.stale, .9));
    }
    const needle = new THREE.Mesh(new THREE.CircleGeometry(w * 1.6, 20), material(live ? COLOR.needle : COLOR.stale));
    needle.position.set(Math.cos(angle) * R, Math.sin(angle) * R, 0);
    needle.renderOrder = 11;
    this.parts.add(needle);
    if (fields.effort && live && spec.effort != null && spec.effort > .005) {
      // Torque share as length, centred on the needle: magnitude only, as the drive reports it.
      const span = spec.effort * Math.PI / 2;
      this.parts.add(arc(R + w * 1.8, R + w * 3.2, angle - span / 2, span, COLOR.torque, .85));
    }
  }
  linear(spec, fields, selected) {
    // A slide's gauge runs beside its axis: the track is its range, the needle its position along it.
    const {u} = this.basis || {u:[0, 0, 1]};
    const axis = new THREE.Vector3(...this.joint.axis).normalize(), side = new THREE.Vector3(...u).multiplyScalar(this.R);
    const at = value => side.clone().addScaledVector(axis, value);
    const live = spec.verified && !spec.stale, line = (a, b, color, opacity) => {
      const object = new THREE.Line(new THREE.BufferGeometry().setFromPoints([at(a), at(b)]),
        new THREE.LineBasicMaterial({color, transparent:true, opacity, depthTest:false, depthWrite:false}));
      object.renderOrder = 10;
      return object;
    };
    const dot = (value, size, color) => {
      const object = new THREE.Mesh(new THREE.SphereGeometry(size, 12, 8), material(color));
      object.position.copy(at(value));
      object.renderOrder = 11;
      return object;
    };
    if (fields.temperature && spec.heat != null) this.parts.add(dot(spec.track?.[0] ?? 0, this.R * .18,
      spec.stale ? COLOR.stale : heatColor(spec.heat)));
    this.label.sprite.position.copy(at(spec.value != null && spec.verified ? spec.value : 0)).addScaledVector(
      side.clone().normalize(), this.R * .4);
    if (!fields.position || !spec.verified) return;
    if (spec.track) this.parts.add(line(spec.track[0], spec.track[1], COLOR.track, selected ? .9 : .6));
    if (spec.reference != null) this.parts.add(dot(spec.reference, this.R * .07, COLOR.reference));
    if (spec.value == null) return;
    if (spec.reference != null) this.parts.add(line(spec.reference, spec.value, live ? COLOR.value : COLOR.stale, 1));
    this.parts.add(dot(spec.value, this.R * .11, live ? COLOR.needle : COLOR.stale));
  }
  face(camera) {
    if (this.prismatic) return;
    const axis = new THREE.Vector3(0, 0, 1).applyQuaternion(this.root.getWorldQuaternion(new THREE.Quaternion()));
    const view = camera.getWorldPosition(new THREE.Vector3()).sub(this.root.getWorldPosition(new THREE.Vector3())).normalize();
    const visible = Math.abs(axis.dot(view)) >= EDGE_ON;
    for (const part of this.parts.children) if (!part.userData.hub) part.visible = visible;
  }
  dispose() {
    dispose(this.root);
  }
}

export class JointGauges {
  constructor({frames, model, radius, doc=globalThis.document}) {
    Object.assign(this, {frames, radius, doc});
    this.joints = new Map(model.joints.map(joint => [joint.name, joint]));
    this.gauges = new Map();
  }
  update(rows, {selected, fields, camera, viewportHeight}) {
    const shown = new Set();
    for (const row of rows) {
      const joint = this.joints.get(row.joint), frame = this.frames.get(row.joint);
      if (!joint || !frame || !['revolute', 'continuous', 'prismatic'].includes(joint.type)) continue;
      let gauge = this.gauges.get(row.motor);
      if (gauge && gauge.joint !== joint) { gauge.dispose(); gauge = null; }
      if (!gauge) {
        gauge = new Gauge({frame, joint, radius:this.radius, doc:this.doc});
        this.gauges.set(row.motor, gauge);
      }
      gauge.root.visible = true;
      gauge.update(row, {selected:row.motor === selected, fields});
      if (camera) { gauge.label.size(camera, viewportHeight); gauge.face(camera); }
      shown.add(row.motor);
    }
    for (const [motor, gauge] of this.gauges) if (!shown.has(motor)) gauge.root.visible = false;
  }
  dispose() {
    for (const gauge of this.gauges.values()) gauge.dispose();
    this.gauges.clear();
  }
}
