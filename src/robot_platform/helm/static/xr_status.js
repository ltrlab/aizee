// What the headset says (xr_scene.js: in its ring's middle) and Settings → Headset's flat copy of it on the operator PC
// (headset_mirror.js: the status line and panel), from Helm's replies on the pose socket (helm/xr.py) and the head
// camera's stream (xr_camera.js). No DOM and no three.js, so node tests it.

const rounded = digits => value => Number(value.toFixed(digits));

// A pose message: the headset's orientation and, when given ({left, right}: handMessage), its controllers.
export function poseMessage(orientation, seq, hands = null) {
  const message = {type: 'pose', seq, q: [orientation.x, orientation.y, orientation.z, orientation.w]};
  if (hands && Object.keys(hands).length) message.hands = hands;
  return message;
}

// A controller for Helm (helm/xr.py; the arms' grab and drag, teleop/vr_arms.py): its grip's position (metres, to a
// tenth of a millimetre) and orientation in the 'local' space, its grip and trigger (0 to 1) and B or Y (an
// xr-standard gamepad: buttons 1, 0 and 5), and `ui`: it is working the headset's ring (xr_ring.js), so its arm
// takes no part.
export function handMessage(position, orientation, gamepad, ui) {
  const value = index => Math.min(1, Math.max(0, Number(gamepad.buttons[index]?.value) || 0));
  return {p: [position.x, position.y, position.z].map(rounded(4)),
    q: [orientation.x, orientation.y, orientation.z, orientation.w].map(rounded(5)), grip: rounded(3)(value(1)),
    trigger: rounded(3)(value(0)), b: Boolean(gamepad.buttons[5]?.pressed), ui: Boolean(ui)};
}

// Helm's /xr links other than this page's (helm/network.py: the network Minerva is on, and its access point), for the
// page to say where to go when this one stops answering: [{where, url}].
export function otherLinks(links, host) {
  return (links || []).filter(link => {
    try {
      return new URL(link.url).host !== host;
    } catch {
      return false;
    }
  }).map(link => ({where: link.mode === 'network' ? 'the network' : "Minerva's access point", url: link.url}));
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

// What the head does, as the ring's card says it: [text, tone], short; its yaw and pitch left out (they change at each
// look, and the card would be drawn again each time).
export function headLine(headset) {
  if (headset.head_fault) return [`Head fault: ${headset.head_fault}`, 'bad'];
  const state = headset.head;
  if (!state) return ['Head still: no session', 'warn'];
  if (state !== 'TELEOP') return [`Head ${String(state).toLowerCase()}: follows once engaged`, 'warn'];
  if (headset.source === 'headset') return ['Head follows you', 'ok'];
  if (headset.source === 'holding') return ['Head holding: the headset went quiet', 'warn'];
  return ['Head at its zero', 'warn'];
}

// The head camera's picture while it is not current, as the ring's card says it: [text, tone], or null.
export function cameraLine(camera) {
  return camera.fresh ? null : [`Head camera: ${cameraText(camera)}`, 'warn'];
}

// The mirror's line under the camera screen, as [[text, tone], [text, tone]]: the picture's freshness on the left,
// what the head does on the right.
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

// The splat map (xr_scene.mapState): [the panel's text, tone].
function mapWord(map) {
  if (!map?.on) return ['off', ''];
  if (map.loading) return ['loading…', 'warn'];
  if (map.error && !map.shown) return [map.error, 'warn'];
  if (!map.shown) return [map.name ? 'with the 3D, fixed in the room' : 'waiting for a map', 'warn'];
  return [`${map.name} · ${Math.round(map.count / 1000)} k splats`, 'ok'];
}

// The status panel's rows: [label, value, tone] with tone 'ok', 'warn', 'bad' or ''.
export function panelRows(link, headset, camera, depth = null, map = null) {
  const [, head, tone] = headWord(headset);
  return [
    ['Helm', link.error || (link.connected ? 'connected' : link.message || 'reconnecting…'),
      link.error ? 'bad' : link.connected ? 'ok' : 'warn'],
    ['Tracking', headset.tracking ? `yes · ${Math.round(headset.rate_hz || 0)} Hz` : 'no', headset.tracking ? 'ok' : 'warn'],
    ['Yaw / pitch', `${signed(headset.yaw, 1)} / ${signed(headset.pitch, 1)}`, ''],
    ['Head', head, tone],
    ['Camera', cameraText(camera), camera.fresh ? 'ok' : 'warn'],
    ['3D', ...depthWord(depth)],
    ['Map', ...mapWord(map)],
  ];
}

// This page's link and the way to the other. Inside a session the two look alike, and a URL is hard to type on a headset,
// so the page says which one it is before it starts.
export function pageLink(remote) {
  return remote
    ? {page: '/xr?remote', name: 'Remote: H.264 over WebRTC, head camera only, flat', other: '/xr',
      switchTo: 'Use the ordinary link'}
    : {page: '/xr', name: 'Ordinary: JPEG over WebSockets, for a good network', other: '/xr?remote',
      switchTo: 'Use the remote link'};
}

export const HINTS = ['A or X: the ring of controls (tilt the stick to pick, press it to choose)',
  'Thumbstick press: zero the head where you look', 'Meta button: stop head tracking'];
