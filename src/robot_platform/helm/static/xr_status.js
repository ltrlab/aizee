// What the headset's screens say (xr_scene.js), from Helm's replies on the pose socket (helm/xr.py) and the head
// camera's stream (xr_camera.js). No DOM and no three.js, so node tests it.

export function poseMessage(orientation, seq) {
  return {type: 'pose', seq, q: [orientation.x, orientation.y, orientation.z, orientation.w]};
}

export function degrees(radians) {
  return radians == null ? '—' : `${radians >= 0 ? '+' : ''}${(radians * 180 / Math.PI).toFixed(1)}°`;
}

const signed = (radians, digits = 0) => (radians == null || !Number.isFinite(radians)) ? '—'
  : `${radians >= 0 ? '+' : '−'}${Math.abs(radians * 180 / Math.PI).toFixed(digits)}°`;

function cameraText(camera) {
  return camera.fresh ? `${Math.round(camera.fps)} fps · ${camera.age_s.toFixed(1)} s old` : camera.message || 'waiting';
}

// What the head does with the poses, from Helm's word on the session's head (its lifecycle state, where its target
// comes from, its fault): [the screen line's text, the panel's text, tone].
function headWord(headset) {
  if (headset.head_fault) return [`Head fault: ${headset.head_fault}`, headset.head_fault, 'bad'];
  const state = headset.head;
  if (!state) return ['No teleop session: the head holds still', 'no teleop session', 'warn'];
  if (state !== 'TELEOP') {
    const word = String(state).toLowerCase();
    return [`Head ${word}: it follows once engaged (T)`, `${word} · follows once engaged (T)`, 'warn'];
  }
  if (headset.source === 'headset') {
    return [`Following · yaw ${signed(headset.yaw)} · pitch ${signed(headset.pitch)}`, 'teleop · following the headset',
      'ok'];
  }
  if (headset.source === 'holding') return ['Holding: the headset went quiet', 'teleop · holding: the headset went quiet', 'warn'];
  return ['At its zero', 'teleop · at its zero', 'warn'];
}

// The line under the camera screen, as [[text, tone], [text, tone]]: the picture's freshness on the left, what the
// head does on the right.
export function stripText(headset, camera) {
  const [head, , tone] = headWord(headset);
  return [[camera.fresh ? `Head camera · ${cameraText(camera)}` : `Head camera: ${cameraText(camera)}`,
    camera.fresh ? '' : 'warn'], [head, tone]];
}

// How the 3D picture is placed (xr_scene.depthState): [the panel's text, tone].
function depthWord(depth) {
  if (!depth?.on) return ['off', ''];
  if (!depth.shown) return ['waiting for depth', 'warn'];
  return depth.locked ? ['fixed in the room', 'ok'] : ['around the eyes: no head pose (no session)', 'warn'];
}

// The status panel's rows: [label, value, tone] with tone 'ok', 'warn', 'bad' or ''.
export function panelRows(link, headset, camera, depth = null) {
  const [, head, tone] = headWord(headset);
  return [
    ['Helm', link.error || (link.connected ? 'connected' : link.message || 'reconnecting…'),
      link.error ? 'bad' : link.connected ? 'ok' : 'warn'],
    ['Tracking', headset.tracking ? `yes · ${Math.round(headset.rate_hz || 0)} Hz` : 'no', headset.tracking ? 'ok' : 'warn'],
    ['Yaw / pitch', `${signed(headset.yaw, 1)} / ${signed(headset.pitch, 1)}`, ''],
    ['Head', head, tone],
    ['Camera', cameraText(camera), camera.fresh ? 'ok' : 'warn'],
    ['3D', ...depthWord(depth)],
  ];
}

export const HINTS = ['Trigger: zero the head where you look, or use the bar', 'Grip: grab a screen and move it',
  'Meta button: stop head tracking'];
