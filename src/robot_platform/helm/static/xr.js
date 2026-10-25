// The headset's pose to Helm (helm/xr.py), which steers Minerva's head while a teleop session follows. A WebXR session
// (passthrough where the headset has it) reads the viewer's orientation in the level 'local' space each frame and sends
// it, at most 60 times a second, over Helm's socket. A trigger pull or a pinch (select) zeroes the head where the
// headset looks; starting the session or recentering the headset restarts the zero at the heading, level. Inside, the
// headset shows the head camera's picture with a status line, and a status panel; the grip buttons grab and move them
// (xr_scene.js).
import * as THREE from 'three';
import {HeadCamera} from './xr_camera.js';
import {HeadsetScene} from './xr_scene.js';
import {HINTS, panelRows, poseMessage, degrees, stripText} from './xr_status.js';

const $ = id => document.getElementById(id);
const SEND_INTERVAL_MS = 1000 / 60;
const TEXT_INTERVAL_MS = 250;
const STATUS_STALE_MS = 1000;  // Helm answers every 15th pose: this long without a word, the headset is not tracking.
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
let renderer = null;
let view = null;
let camera = null;
let headset = {};  // Helm's latest word on the headset and the head (its status replies),
let statusAt = -Infinity;  // and when it came.
const link = {connected: false, message: 'connecting…', error: null};  // `error`: Helm's last refusal, until a status.

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
    link.error = status.error;
    if (status.busy) retryMs = BUSY_RETRY_MS;
    return;
  }
  link.error = null;
  headset = {...headset, ...status};  // A zero or start reply says nothing of the head: its last word stays.
  statusAt = performance.now();
  show('tracking', status.tracking ? `yes · ${status.rate_hz ? Math.round(status.rate_hz) + ' Hz' : ''}` : 'no',
    status.tracking ? 'ok' : 'warn');
  show('angles', `${degrees(status.yaw)} / ${degrees(status.pitch)}`);
  const head = headset.head ? `${headset.head}${headset.source ? ' · ' + headset.source : ''}` : 'no teleop session';
  show('head', headset.steering ? `steering · ${head}` : head, headset.steering ? 'ok' : 'warn');
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
    link.connected = true;
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
    link.connected = false;
    link.message = 'reconnecting…';
    headset = {};
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
  if (!renderer) {
    renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
    renderer.xr.enabled = true;
    renderer.xr.setReferenceSpaceType('local');
    view = new HeadsetScene(renderer);
  }
  renderer.setClearColor(0x0f1218, mode === 'immersive-ar' ? 0 : 1);  // Passthrough shows through a clear frame.
  await renderer.xr.setSession(session);
  const space = renderer.xr.getReferenceSpace();
  send({type: 'start'});
  space.addEventListener('reset', () => send({type: 'start'}));  // A recenter: the space starts again here.
  session.addEventListener('select', () => send({type: 'zero'}));
  camera = new HeadCamera({token: () => token, onFrame: bitmap => view.showFrame(bitmap)});
  camera.start();
  session.addEventListener('end', () => {
    renderer.setAnimationLoop(null);
    camera?.stop();
    camera = null;
    session = null;
    $('start').textContent = 'Start head tracking';
    show('status', 'Head tracking stopped: the head holds, then returns to its zero.', 'warn');
  });
  $('start').textContent = 'Stop head tracking';
  show('status', `Tracking (${mode === 'immersive-ar' ? 'passthrough' : 'VR'}).`, 'ok');
  let textAt = -Infinity;
  renderer.setAnimationLoop((time, frame) => {
    const pose = frame?.getViewerPose(renderer.xr.getReferenceSpace());
    if (pose && time - lastSent >= SEND_INTERVAL_MS) {
      sequence += 1;
      send(poseMessage(pose.transform.orientation, sequence));
      lastSent = time;
    }
    if (camera && time - textAt >= TEXT_INTERVAL_MS) {
      textAt = time;
      const picture = camera.stats;
      const word = performance.now() - statusAt > STATUS_STALE_MS ? {...headset, tracking: false} : headset;
      view.setText(stripText(word, picture), panelRows(link, word, picture), HINTS, picture.fresh);
    }
    view.update();
    renderer.render(view.scene, view.camera);
  });
}

if (typeof document !== 'undefined' && $('start')) {
  $('start').onclick = () => start().catch(error => {
    session = null;
    show('status', `Could not start: ${error.message}`, 'bad');
  });
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
