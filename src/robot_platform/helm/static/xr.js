// The headset's pose to Helm (helm/xr.py), which steers Minerva's head while a teleop session follows. A WebXR session
// (passthrough where the headset has it) reads the viewer's orientation in the level 'local' space each frame and sends
// it, at most 60 times a second, over Helm's socket. A trigger pull or a pinch (select) zeroes the head where the
// headset looks; starting the session or recentering the headset restarts the zero at the heading, level.
const $ = id => document.getElementById(id);
const SEND_INTERVAL_MS = 1000 / 60;
const RETRY_MS = 1000;
const BUSY_RETRY_MS = 3000;  // Another device's headset steers: ask again now and then.

let socket = null;
let token = null;
let sequence = 0;
let session = null;
let lastSent = -Infinity;
let mode = null;
let retryMs = RETRY_MS;
let immersive = null;  // Whether this browser can start an immersive session; null until it says.

export function poseMessage(orientation, seq) {
  return {type: 'pose', seq, q: [orientation.x, orientation.y, orientation.z, orientation.w]};
}

export function degrees(radians) {
  return radians == null ? '—' : `${radians >= 0 ? '+' : ''}${(radians * 180 / Math.PI).toFixed(1)}°`;
}

function show(id, text, tone) {
  const element = $(id);
  element.textContent = text;
  element.className = tone || '';
}

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

// What the page says while no XR session runs.
function idle() {
  if (session) return;
  if (!navigator.xr || immersive === false) {
    show('status', `This browser ${navigator.xr ? 'cannot start an immersive session' : 'has no WebXR'}: open this `
      + 'page in the headset\'s browser.', 'bad');
  } else if (socket && socket.readyState === WebSocket.OPEN && immersive) {
    show('status', 'Ready: put the headset on and start head tracking.');
  }
}

function onStatus(status) {
  if (status.type === 'error') {
    show('status', status.error, 'bad');
    if (status.busy) retryMs = BUSY_RETRY_MS;
    return;
  }
  show('tracking', status.tracking ? `yes · ${status.rate_hz ? Math.round(status.rate_hz) + ' Hz' : ''}` : 'no',
    status.tracking ? 'ok' : 'warn');
  show('angles', `${degrees(status.yaw)} / ${degrees(status.pitch)}`);
  const head = status.head ? `${status.head}${status.source ? ' · ' + status.source : ''}` : 'no teleop session';
  show('head', status.steering ? `steering · ${head}` : head, status.steering ? 'ok' : 'warn');
  if (status.zeroed) show('status', 'Zeroed: this is the head\'s straight ahead.', 'ok');
  if (status.restarted) show('status', 'Tracking: where you looked as it started, level, is the head\'s zero.', 'ok');
}

async function connect() {
  try {
    const response = await fetch('/api/helm/state', {cache: 'no-store'});
    if (response.status === 401) {
      location.href = '/login?next=/xr';
      return;
    }
    token = (await response.json()).action_token;
  } catch (error) {
    show('link', `unreachable (${error.message})`, 'bad');
    setTimeout(connect, RETRY_MS);
    return;
  }
  const url = new URL('/api/helm/xr', location.href);
  url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  url.searchParams.set('token', token);
  socket = new WebSocket(url);
  socket.onopen = () => {
    sequence = 0;  // A new connection starts its own sequence.
    retryMs = RETRY_MS;
    show('link', 'connected', 'ok');
    idle();
  };
  socket.onmessage = event => {
    try {
      onStatus(JSON.parse(event.data));
    } catch (error) {
      show('status', `Unreadable message from Helm: ${error.message}`, 'bad');
    }
  };
  socket.onclose = () => {
    show('link', 'reconnecting…', 'warn');
    setTimeout(connect, retryMs);
  };
}

async function start() {
  if (session) {
    await session.end();
    return;
  }
  mode = await navigator.xr.isSessionSupported('immersive-ar') ? 'immersive-ar' : 'immersive-vr';
  session = await navigator.xr.requestSession(mode, {requiredFeatures: ['local']});
  const canvas = document.createElement('canvas');
  const gl = canvas.getContext('webgl', {xrCompatible: true, alpha: true});
  await session.updateRenderState({baseLayer: new XRWebGLLayer(session, gl)});
  const space = await session.requestReferenceSpace('local');
  send({type: 'start'});
  space.addEventListener('reset', () => send({type: 'start'}));  // A recenter: the space starts again here.
  session.addEventListener('select', () => send({type: 'zero'}));
  session.addEventListener('end', () => {
    session = null;
    $('start').textContent = 'Start head tracking';
    show('status', 'Head tracking stopped: the head holds, then returns to its zero.', 'warn');
  });
  $('start').textContent = 'Stop head tracking';
  show('status', `Tracking (${mode === 'immersive-ar' ? 'passthrough' : 'VR'}).`, 'ok');
  const frame = (time, xrFrame) => {
    if (!session) return;
    session.requestAnimationFrame(frame);
    const pose = xrFrame.getViewerPose(space);
    if (pose && time - lastSent >= SEND_INTERVAL_MS) {
      sequence += 1;
      send(poseMessage(pose.transform.orientation, sequence));
      lastSent = time;
    }
    const layer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
    gl.clearColor(0.06, 0.07, 0.09, mode === 'immersive-ar' ? 0 : 1);  // Passthrough shows through a clear frame.
    gl.clear(gl.COLOR_BUFFER_BIT);
  };
  session.requestAnimationFrame(frame);
}

if (typeof document !== 'undefined' && $('start')) {
  $('start').onclick = () => start().catch(error => show('status', `Could not start: ${error.message}`, 'bad'));
  if (navigator.xr) {
    navigator.xr.isSessionSupported('immersive-vr').then(supported => {
      immersive = supported;
      $('start').disabled = !supported;
      idle();
    });
  }
  idle();
  connect();
}
