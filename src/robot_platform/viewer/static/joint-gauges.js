import * as THREE from 'three';
import { gaugeSpec, heatColor, rotaryBasis, sceneTheme } from './joint-telemetry.js';

// Readings drawn on the model, display only. Around each joint's axis a dial shows its range (grey track), its
// reference (tick), its travel from the reference (blue arc) and its position (needle); tick and needle are white on
// a dark background and ink on a light one (the scene theme, joint-telemetry.js). A violet ring beside the needle
// grows with torque as a share of the receiver's limit; the hub at the joint is tinted by temperature as a share of
// its limit. One short label carries the position and the temperature, the latter in the hub's colour. Dials need a
// verified direction mapping, as motion arrows do; stale readings turn grey. A dial seen nearly edge-on hides; its
// hub and label stay.
const LABEL_PX = 22; // Label type comes with the scene theme (joint-telemetry.js): Helm's look sets it.
const EDGE_ON = 0.25; // |axis · view| below this, a dial collapses to a line: hide it.

function material(color, opacity = 1) {
  return new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    depthTest: false,
    depthWrite: false,
    toneMapped: false,
    side: THREE.DoubleSide,
  });
}

function dispose(object) {
  object.traverse((child) => {
    child.geometry?.dispose();
    child.material?.map?.dispose();
    child.material?.dispose();
  });
  object.parent?.remove(object);
}

function arc(inner, outer, start, length, color, opacity) {
  const segments = Math.max(4, Math.ceil(Math.abs(length) / (Math.PI / 48)));
  const mesh = new THREE.Mesh(
    new THREE.RingGeometry(inner, outer, segments, 1, start, length),
    material(color, opacity),
  );
  mesh.renderOrder = 9;
  return mesh;
}

class Label {
  constructor(doc) {
    // Drawn at the size it is shown, in device pixels (the renderer's ratio, at most 2), and sampled pixel for pixel:
    // pixel type scaled down blurs (5 reads as S). The chip's layout is in units of a 48 px tall label (k).
    this.ratio = Math.min(globalThis.devicePixelRatio || 1, 2);
    this.k = (LABEL_PX * this.ratio) / 48;
    this.canvas = doc.createElement('canvas');
    this.canvas.width = Math.round(480 * this.k); // Room for the widest reading; the rest stays transparent.
    this.canvas.height = Math.round(48 * this.k);
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.texture.generateMipmaps = false;
    this.texture.minFilter = this.texture.magFilter = THREE.NearestFilter;
    this.sprite = new THREE.Sprite(
      new THREE.SpriteMaterial({
        map: this.texture,
        transparent: true,
        depthTest: false,
        toneMapped: false,
        depthWrite: false,
        sizeAttenuation: false,
      }),
    );
    this.sprite.center.set(0, 0.5);
    this.sprite.renderOrder = 12;
    this.drawn = null;
    this.fonts = doc.fonts;
    this.awaiting = new Set();
  }
  // `parts`: [{text, color}], drawn in a row with a separator between them, on the scene theme's chip.
  draw(parts, selected, theme) {
    const key = JSON.stringify([parts, selected, theme.name]);
    if (key === this.drawn) return;
    this.drawn = key;
    this.text = parts.map((part) => part.text).join(' · ');
    const ctx = this.canvas.getContext?.('2d');
    if (!ctx) return; // No canvas (tests): the sprite stays blank.
    const fontPx = Math.round(theme.type.px * this.ratio),
      font = `${theme.type.weight} ${fontPx}px ${theme.type.family}`;
    if (this.fonts && !this.fonts.check(font) && !this.awaiting.has(font)) {
      // A canvas draws with what has loaded: ask for the font once, and draw this label again when it arrives.
      this.awaiting.add(font);
      this.fonts.load(font).then(
        () => {
          this.drawn = null;
        },
        () => {},
      );
    }
    const { width, height } = this.canvas,
      k = this.k;
    ctx.clearRect(0, 0, width, height);
    ctx.font = font;
    const textWidth = Math.min(width - 16 * k, ctx.measureText(this.text).width);
    ctx.fillStyle = theme.label.fill;
    ctx.strokeStyle = selected ? theme.label.selected : theme.label.edge;
    ctx.lineWidth = Math.max(1, Math.round(2 * k));
    ctx.beginPath();
    ctx.roundRect?.(
      Math.round(2 * k),
      Math.round(4 * k),
      Math.round(textWidth + 14 * k),
      height - Math.round(8 * k),
      8 * k,
    );
    ctx.fill();
    ctx.stroke();
    ctx.textBaseline = 'alphabetic'; // Digits on a whole-pixel baseline, centred in the chip.
    const scale = textWidth / Math.max(1, ctx.measureText(this.text).width); // Squeezed to fit, as fillText would.
    let x = 9 * k;
    parts.forEach((part, i) => {
      const text = i ? ` · ${part.text}` : part.text,
        w = ctx.measureText(text).width * scale;
      ctx.fillStyle = part.color;
      ctx.fillText(text, Math.round(x), Math.round(height / 2 + theme.type.baseline * fontPx), w);
      x += w;
    });
    this.texture.needsUpdate = true;
  }
  size(camera, viewportHeight) {
    // With size attenuation off, a sprite's height on screen is scale × P11 × viewport height / 2.
    const height = (LABEL_PX * 2) / (camera.projectionMatrix.elements[5] * Math.max(1, viewportHeight));
    this.sprite.scale.set((height * this.canvas.width) / this.canvas.height, height, 1);
  }
}

class Gauge {
  constructor({ frame, joint, radius, doc }) {
    Object.assign(this, { frame, joint, prismatic: joint.type === 'prismatic' });
    this.R = radius * 0.085;
    this.basis = rotaryBasis(joint.axis);
    this.root = new THREE.Group();
    this.root.renderOrder = 9;
    if (this.basis && !this.prismatic) {
      const { u, v, n } = this.basis;
      this.root.quaternion.setFromRotationMatrix(
        new THREE.Matrix4().makeBasis(new THREE.Vector3(...u), new THREE.Vector3(...v), new THREE.Vector3(...n)),
      );
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
  update(row, { selected, fields, theme }) {
    const spec = gaugeSpec(row);
    const signature = JSON.stringify([
      theme.name,
      spec.verified,
      spec.stale,
      spec.track,
      spec.reference,
      spec.value == null ? null : Math.round(spec.value / 0.002),
      spec.effort == null ? null : Math.round(spec.effort * 50),
      spec.heat == null ? null : Math.round(spec.heat * 100),
      selected,
      fields,
    ]);
    if (signature !== this.signature) {
      this.signature = signature;
      for (const child of [...this.parts.children]) dispose(child);
      if (this.prismatic) this.linear(spec, fields, selected, theme);
      else this.rotary(spec, fields, selected, theme);
    }
    const color = spec.stale ? theme.label.stale : spec.verified ? theme.label.text : theme.label.unverified;
    const parts = [];
    if (fields.position && spec.value != null) parts.push({ text: spec.label, color });
    if (fields.temperature && spec.temperature != null)
      parts.push({
        text: spec.temperature,
        color:
          spec.stale || spec.heat == null
            ? color
            : `#${heatColor(spec.heat, theme.heat).toString(16).padStart(6, '0')}`,
      });
    this.label.sprite.visible = parts.length > 0;
    this.label.draw(parts, selected, theme);
  }
  rotary(spec, fields, selected, theme) {
    const R = this.R,
      w = R * 0.07,
      live = spec.verified && !spec.stale,
      angle = spec.value,
      COLOR = theme.gauge;
    if (fields.temperature && spec.heat != null) {
      // A sphere, so the temperature shows from any side.
      const hub = new THREE.Mesh(
        new THREE.SphereGeometry(R * 0.3, 16, 12),
        material(spec.stale ? COLOR.stale : heatColor(spec.heat, theme.heat), 0.95),
      );
      hub.userData.hub = true;
      hub.renderOrder = 10;
      this.parts.add(hub);
    }
    const labelAt = angle != null && spec.verified ? angle : 0;
    this.label.sprite.position.set(Math.cos(labelAt) * R * 1.45, Math.sin(labelAt) * R * 1.45, 0);
    if (!fields.position || !spec.verified) return; // Unverified direction: the angle could be mirrored.
    const track = spec.track
      ? arc(R - w / 2, R + w / 2, spec.track[0], spec.track[1] - spec.track[0], COLOR.track, selected ? 0.8 : 0.5)
      : arc(R - w / 2, R + w / 2, 0, 2 * Math.PI, COLOR.track, selected ? 0.6 : 0.35);
    this.parts.add(track);
    if (spec.reference != null) {
      const tick = new THREE.Mesh(new THREE.PlaneGeometry(R * 0.32, w * 0.9), material(COLOR.reference, 0.9));
      tick.position.set(Math.cos(spec.reference) * R, Math.sin(spec.reference) * R, 0);
      tick.rotation.z = spec.reference;
      tick.renderOrder = 11;
      this.parts.add(tick);
    }
    if (angle == null) return;
    if (spec.reference != null && Math.abs(angle - spec.reference) > 1e-4) {
      const start = Math.min(spec.reference, angle);
      this.parts.add(
        arc(R - w * 1.1, R + w * 1.1, start, Math.abs(angle - spec.reference), live ? COLOR.value : COLOR.stale, 0.9),
      );
    }
    const needle = new THREE.Mesh(new THREE.CircleGeometry(w * 1.6, 20), material(live ? COLOR.needle : COLOR.stale));
    needle.position.set(Math.cos(angle) * R, Math.sin(angle) * R, 0);
    needle.renderOrder = 11;
    this.parts.add(needle);
    if (fields.effort && live && spec.effort != null && spec.effort > 0.005) {
      // Torque share as length, centred on the needle: magnitude only, as the drive reports it.
      const span = (spec.effort * Math.PI) / 2;
      this.parts.add(arc(R + w * 1.8, R + w * 3.2, angle - span / 2, span, COLOR.torque, 0.85));
    }
  }
  linear(spec, fields, selected, theme) {
    // A slide's gauge runs beside its axis: the track is its range, the needle its position along it.
    const { u } = this.basis || { u: [0, 0, 1] },
      COLOR = theme.gauge;
    const axis = new THREE.Vector3(...this.joint.axis).normalize(),
      side = new THREE.Vector3(...u).multiplyScalar(this.R);
    const at = (value) => side.clone().addScaledVector(axis, value);
    const live = spec.verified && !spec.stale,
      line = (a, b, color, opacity) => {
        const object = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([at(a), at(b)]),
          new THREE.LineBasicMaterial({
            color,
            transparent: true,
            opacity,
            depthTest: false,
            depthWrite: false,
            toneMapped: false,
          }),
        );
        object.renderOrder = 10;
        return object;
      };
    const dot = (value, size, color) => {
      const object = new THREE.Mesh(new THREE.SphereGeometry(size, 12, 8), material(color));
      object.position.copy(at(value));
      object.renderOrder = 11;
      return object;
    };
    if (fields.temperature && spec.heat != null)
      this.parts.add(
        dot(spec.track?.[0] ?? 0, this.R * 0.18, spec.stale ? COLOR.stale : heatColor(spec.heat, theme.heat)),
      );
    this.label.sprite.position
      .copy(at(spec.value != null && spec.verified ? spec.value : 0))
      .addScaledVector(side.clone().normalize(), this.R * 0.4);
    if (!fields.position || !spec.verified) return;
    if (spec.track) this.parts.add(line(spec.track[0], spec.track[1], COLOR.track, selected ? 0.9 : 0.6));
    if (spec.reference != null) this.parts.add(dot(spec.reference, this.R * 0.07, COLOR.reference));
    if (spec.value == null) return;
    if (spec.reference != null) this.parts.add(line(spec.reference, spec.value, live ? COLOR.value : COLOR.stale, 1));
    this.parts.add(dot(spec.value, this.R * 0.11, live ? COLOR.needle : COLOR.stale));
  }
  face(camera) {
    if (this.prismatic) return;
    const axis = new THREE.Vector3(0, 0, 1).applyQuaternion(this.root.getWorldQuaternion(new THREE.Quaternion()));
    const view = camera
      .getWorldPosition(new THREE.Vector3())
      .sub(this.root.getWorldPosition(new THREE.Vector3()))
      .normalize();
    const visible = Math.abs(axis.dot(view)) >= EDGE_ON;
    for (const part of this.parts.children) if (!part.userData.hub) part.visible = visible;
  }
  dispose() {
    dispose(this.root);
  }
}

export class JointGauges {
  constructor({ frames, model, radius, doc = globalThis.document }) {
    Object.assign(this, { frames, radius, doc });
    this.joints = new Map(model.joints.map((joint) => [joint.name, joint]));
    this.gauges = new Map();
  }
  update(rows, { selected, fields, camera, viewportHeight }) {
    const shown = new Set(),
      theme = sceneTheme(this.doc);
    for (const row of rows) {
      const joint = this.joints.get(row.joint),
        frame = this.frames.get(row.joint);
      if (!joint || !frame || !['revolute', 'continuous', 'prismatic'].includes(joint.type)) continue;
      let gauge = this.gauges.get(row.motor);
      if (gauge && gauge.joint !== joint) {
        gauge.dispose();
        gauge = null;
      }
      if (!gauge) {
        gauge = new Gauge({ frame, joint, radius: this.radius, doc: this.doc });
        this.gauges.set(row.motor, gauge);
      }
      gauge.root.visible = true;
      gauge.update(row, { selected: row.motor === selected, fields, theme });
      if (camera) {
        gauge.label.size(camera, viewportHeight);
        gauge.face(camera);
      }
      shown.add(row.motor);
    }
    for (const [motor, gauge] of this.gauges) if (!shown.has(motor)) gauge.root.visible = false;
  }
  dispose() {
    for (const gauge of this.gauges.values()) gauge.dispose();
    this.gauges.clear();
  }
}
