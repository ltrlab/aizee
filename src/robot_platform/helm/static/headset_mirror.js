// Settings → Headset: what the headset shows (xr.js), flat. The head camera's picture with its status line, and the
// status panel, drawn on the headset's own canvases (xr_panels.js) from Helm's state and the head camera's stream
// (xr_camera.js), so they read as the Quest's screens do. The stream runs only while the section is on screen.
import {HeadCamera} from './xr_camera.js';
import {drawPanel, drawPlaceholder, drawStrip} from './xr_panels.js';
import {HINTS, panelRows, stripText} from './xr_status.js';

// What the headset's screens read (xr.js's `headset`: Helm's replies on its pose socket), from Helm's state.
export function headsetWord(state) {
  const xr = state?.xr || {}, head = state?.teleop?.head || {};
  return {tracking: !!xr.tracking, rate_hz: xr.rate_hz, yaw: xr.yaw, pitch: xr.pitch, head: head.state,
    source: head.source, head_fault: head.fault};
}

// The panel's Helm row: in the headset, its own link to Helm; here, whether a headset is connected at all.
export function headsetLink(state) {
  return state?.xr?.connected ? {connected: true} : {connected: false, message: 'no headset connected'};
}

export class HeadsetMirror {
  // `picture`, `strip`, `panel`: canvases. `camera`: makes the head camera's stream (xr_camera.HeadCamera's options).
  constructor({picture, strip, panel, camera = options => new HeadCamera(options)}) {
    Object.assign(this, {picture, strip, panel, makeCamera: camera});
    this.camera = null;
    this.token = null;
    this.framed = false;  // Whether the picture holds a frame.
    drawPlaceholder(picture.getContext('2d'), 'Head camera');
  }

  // Each state: the stream on while `visible` (and there is a physical camera), the screens drawn while on screen.
  update(state, visible) {
    this.token = state?.action_token || null;
    const simulation = state?.mode === 'simulation';
    const streaming = visible && !simulation && !!this.token;
    if (streaming && !this.camera) {
      this.camera = this.makeCamera({token: () => this.token, onFrame: bitmap => this.frame(bitmap)});
      this.camera.start();
      if (!this.framed) drawPlaceholder(this.picture.getContext('2d'), 'Waiting for the head camera…');
    } else if (!streaming && this.camera) {
      this.camera.stop();
      this.camera = null;
    }
    if (!visible) return;
    const picture = this.camera ? this.camera.stats : {fresh: false, fps: 0, age_s: Infinity,
      message: simulation ? 'no physical camera in simulation' : 'not streaming'};
    const word = headsetWord(state);
    drawStrip(this.strip.getContext('2d'), stripText(word, picture));
    drawPanel(this.panel.getContext('2d'), panelRows(headsetLink(state), word, picture), HINTS);
    this.picture.classList.toggle('stale', this.framed && !picture.fresh);  // Dimmed, as the headset dims it.
  }

  frame(bitmap) {
    if (!this.camera) {
      bitmap.close?.();
      return;
    }
    const {picture} = this;
    if (picture.width !== bitmap.width || picture.height !== bitmap.height) {
      picture.width = bitmap.width;
      picture.height = bitmap.height;
    }
    picture.getContext('2d').drawImage(bitmap, 0, 0);
    bitmap.close?.();
    this.framed = true;
  }
}
