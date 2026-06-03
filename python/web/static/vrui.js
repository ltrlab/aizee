// VR UI — pokable buttons for the in-headset operator.
//
// Each button is a flat rounded panel with a label that the operator points
// at with their LEFT index fingertip (right hand is the IK driver; keeping
// the two separate avoids the pinch-clutch / pinch-click overload).
//
// Activation modes:
//   * Dwell-to-click: hold fingertip inside the button for 0.5 s — visual
//     fill bar shows progress.
//   * Pinch-to-click: pinch left index+thumb while hovering — fires
//     immediately (skip the dwell).
//
// Buttons issue side effects via POST /api/quest_command; this module is
// otherwise self-contained.

import * as THREE from 'three';

const POST_URL = '/api/quest_command';

const _BTN_W = 0.20;
const _BTN_H = 0.08;
const _BTN_GAP = 0.015;
// Hit detection: spherical zone around each button's center.  Made
// generous (10 cm) since mid-air pointing with hand tracking is jittery
// and the operator can't feel the button edge.
const _HIT_RADIUS = 0.10;
const _DWELL_MS = 500;

const _state = {
  group: null,         // THREE.Group containing the whole UI
  buttons: [],         // list of Button records
  fingerCursor: null,  // small sphere at the left index-finger-tip — visual feedback
};

// main.js registers handlers here for buttons whose cmd starts with `_local_`.
const _localHandlers = {};
export function registerLocalCommand(name, handler) { _localHandlers[name] = handler; }


class Button {
  constructor(label, cmd, args = {}) {
    this.label = label;
    this.cmd = cmd;
    this.args = args;
    // Canvas + texture for the text label.  Redrawn whenever hover/dwell
    // state changes, keeping the GPU upload off the hot path.
    this.canvas = document.createElement('canvas');
    this.canvas.width = 512; this.canvas.height = 160;
    this.ctx = this.canvas.getContext('2d');
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(_BTN_W, _BTN_H),
      new THREE.MeshBasicMaterial({ map: this.texture, transparent: true }),
    );
    this.dwellMs = 0;
    this.lastRenderedKey = '';
    this.lastPinch = false;
    this.lastT = performance.now();
    this.flashUntil = 0;  // performance.now() time when the post-click flash ends
    this._render(0, false, false);
  }

  /** Returns true if this button just fired its click (rising edge). */
  update(fingerWorldPos, leftPinch, scene) {
    const now = performance.now();
    const dt = now - this.lastT;
    this.lastT = now;
    let inside = false;
    if (fingerWorldPos) {
      // Simple spherical hit-test around the button's world position.
      // Works regardless of the button's orientation, doesn't depend on
      // worldToLocal matrix freshness, and is forgiving of imperfect aim.
      const buttonWorld = new THREE.Vector3();
      this.mesh.getWorldPosition(buttonWorld);
      inside = buttonWorld.distanceTo(fingerWorldPos) < _HIT_RADIUS;
    }
    // Dwell accumulates only while finger is inside; decays fast when not.
    let fired = false;
    if (inside) {
      this.dwellMs += dt;
      // Pinch path: fire on rising edge of left index pinch.
      if (leftPinch && !this.lastPinch) fired = true;
      // Dwell path: fire when threshold reached, then cool down.
      if (this.dwellMs >= _DWELL_MS) {
        fired = true;
        this.dwellMs = -300;   // 300 ms cooldown after click — avoids retrigger
      }
    } else {
      this.dwellMs = Math.max(0, this.dwellMs - dt * 4);  // 4x decay
    }
    this.lastPinch = leftPinch;
    if (fired) this.flashUntil = now + 350;  // 350 ms yellow confirmation flash
    const flashing = now < this.flashUntil;
    // Re-render label only when something visually changed (keeps the
    // CanvasTexture upload off the per-frame hot path for static buttons).
    const progress = Math.max(0, Math.min(1, this.dwellMs / _DWELL_MS));
    const key = `${inside ? 1 : 0}|${progress.toFixed(1)}|${flashing ? 1 : 0}`;
    if (key !== this.lastRenderedKey) {
      this._render(progress, inside, flashing);
      this.lastRenderedKey = key;
    }
    return fired;
  }

  _render(progress, hover, flashing) {
    const ctx = this.ctx;
    const w = this.canvas.width, h = this.canvas.height;
    ctx.clearRect(0, 0, w, h);
    // Background plate.  Flash > hover > idle in the color priority.
    let bg;
    if (flashing) bg = 'rgba(255, 211, 61, 0.95)';
    else if (hover) bg = 'rgba(46, 160, 67, 0.65)';
    else bg = 'rgba(33, 38, 45, 0.85)';
    ctx.fillStyle = bg;
    _roundRect(ctx, 6, 6, w - 12, h - 12, 18);
    ctx.fill();
    // Dwell-progress fill (left-to-right) — hidden during flash to keep the
    // confirmation visually clean.
    if (progress > 0 && !flashing) {
      ctx.save();
      ctx.beginPath();
      _roundRect(ctx, 6, 6, (w - 12) * progress, h - 12, 18);
      ctx.clip();
      ctx.fillStyle = 'rgba(46, 160, 67, 0.95)';
      ctx.fillRect(0, 0, w, h);
      ctx.restore();
    }
    // Label.  Darker text on the flash background so it stays readable.
    ctx.fillStyle = flashing ? '#0c0f14' : '#e6edf3';
    ctx.font = '700 48px system-ui, -apple-system, "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(this.label, w / 2, h / 2);
    this.texture.needsUpdate = true;
  }

  relabel(text) {
    // Force a re-render of the label on the next update() tick.
    this.label = text;
    this.lastRenderedKey = '';
  }

  async fire() {
    // Local-only commands (browser-side side effects, no server call).
    if (this.cmd.startsWith('_local_')) {
      const handler = _localHandlers[this.cmd];
      if (handler) {
        // Pass the button so toggle handlers can relabel it to show state.
        try { await handler(this.args, this); }
        catch (e) { console.warn('[vrui] local handler failed:', e); }
      }
      return;
    }
    const body = { cmd: this.cmd, ...this.args };
    try {
      const r = await fetch(POST_URL, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      });
      if (!r.ok) console.warn('[vrui]', this.cmd, 'failed HTTP', r.status);
    } catch (e) {
      console.warn('[vrui]', this.cmd, 'fetch failed:', e);
    }
  }
}


function _roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}


/**
 * Build the in-VR UI panel and add it to the scene.  Called once on VR entry.
 * Returns a `tick(leftFingerPos, leftPinch)` function that should be called
 * every XR frame with the left index-finger-tip world position + pinch state.
 */
export function buildVRUI(scene) {
  if (_state.group) return _state.group;
  const group = new THREE.Group();
  // Panel position: floating to the operator's LEFT at chest height,
  // closer to them so it's clearly within arm's reach.  Tilted to face them.
  group.position.set(-0.35, 1.20, -0.40);
  group.rotation.y = Math.PI / 5;
  scene.add(group);

  // Visible fingertip cursor — small bright sphere at the left index tip
  // so the operator can SEE where their click point is.  Tracks via the
  // tick() argument; not parented to anything, just placed in world frame.
  _state.fingerCursor = new THREE.Mesh(
    new THREE.SphereGeometry(0.012, 12, 10),
    new THREE.MeshBasicMaterial({ color: 0x5599ff, transparent: true, opacity: 0.85 }),
  );
  _state.fingerCursor.renderOrder = 998;
  _state.fingerCursor.visible = false;
  scene.add(_state.fingerCursor);

  // Card background behind the buttons.
  const cardCanvas = document.createElement('canvas');
  cardCanvas.width = 256; cardCanvas.height = 32;
  const cardCtx = cardCanvas.getContext('2d');
  cardCtx.fillStyle = 'rgba(22, 27, 34, 0.55)';
  _roundRect(cardCtx, 0, 0, 256, 32, 14);
  cardCtx.fill();
  cardCtx.fillStyle = '#e6edf3';
  cardCtx.font = '600 18px system-ui, sans-serif';
  cardCtx.textAlign = 'center';
  cardCtx.textBaseline = 'middle';
  cardCtx.fillText('VR controls — left fingertip to poke', 128, 16);
  const titleTex = new THREE.CanvasTexture(cardCanvas);
  titleTex.colorSpace = THREE.SRGBColorSpace;
  const title = new THREE.Mesh(
    new THREE.PlaneGeometry(_BTN_W, _BTN_W * 32 / 256),
    new THREE.MeshBasicMaterial({ map: titleTex, transparent: true }),
  );
  // Place the title above the first button.
  title.position.y = 0.5 * (_BTN_H + _BTN_GAP) + 0.04;
  group.add(title);

  // The actual button set.  Re-arrange as needed.  The local handler
  // (passed in via args._localHandler — main.js wires this) is invoked
  // before the server POST for buttons that need a side effect in the
  // browser too (e.g. passthrough toggle).
  const defs = [
    { label: 'Re-align',          cmd: 'realign' },
    { label: 'Reset sim',         cmd: 'reset_sim' },
    { label: 'Align to arm',      cmd: 'align_to_actual' },
    { label: 'Workspace +10%',    cmd: 'grow_workspace',   args: { factor: 1.10 } },
    { label: 'Workspace -10%',    cmd: 'shrink_workspace', args: { factor: 0.90 } },
    { label: 'Center on EE',      cmd: 'center_workspace_on_ee' },
    { label: 'Reset workspace',   cmd: 'reset_workspace' },
    { label: 'Preview: ON',       cmd: '_local_toggle_preview' },
    { label: 'Ghost: ON',         cmd: '_local_toggle_ghost' },
    { label: 'Toggle passthrough', cmd: '_local_toggle_passthrough' },
  ];
  const totalH = defs.length * _BTN_H + (defs.length - 1) * _BTN_GAP;
  for (let i = 0; i < defs.length; i++) {
    const d = defs[i];
    const btn = new Button(d.label, d.cmd, d.args);
    btn.mesh.position.set(0, totalH / 2 - i * (_BTN_H + _BTN_GAP) - _BTN_H / 2, 0);
    group.add(btn.mesh);
    _state.buttons.push(btn);
  }
  _state.group = group;
  return group;
}


/**
 * Call once per XR frame with the LEFT index-finger-tip world position
 * (or null if hand tracking lost / no left hand).  leftPinch is the
 * left-index pinch state (true while pinched).  Fires button onClicks
 * via /api/quest_command.
 */
/** True if the fingertip is within any button's hit sphere — used by
 *  main.js to decide whether a left pinch should press a button or start
 *  a robot grab. */
export function isPointerOverButton(fingerWorldPos) {
  if (!_state.group || !fingerWorldPos) return false;
  const bw = new THREE.Vector3();
  for (const btn of _state.buttons) {
    btn.mesh.getWorldPosition(bw);
    if (bw.distanceTo(fingerWorldPos) < _HIT_RADIUS) return true;
  }
  return false;
}

export function tickVRUI(leftFingerWorldPos, leftPinch) {
  if (!_state.group) return;
  // Update fingertip cursor visibility/position so the operator can see
  // where their click point is regardless of whether buttons are nearby.
  if (_state.fingerCursor) {
    if (leftFingerWorldPos) {
      _state.fingerCursor.position.copy(leftFingerWorldPos);
      _state.fingerCursor.visible = true;
    } else {
      _state.fingerCursor.visible = false;
    }
  }
  for (const btn of _state.buttons) {
    if (btn.update(leftFingerWorldPos, !!leftPinch, _state.group)) {
      btn.fire();
    }
  }
}
