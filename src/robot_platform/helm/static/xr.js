// The headset's pose to Helm (helm/xr.py), which steers Minerva's head while a teleop session follows. A WebXR session
// (passthrough where the headset has it) reads the viewer's orientation in the level 'local' space each frame and sends
// it, at most 60 times a second, over Helm's socket, with each controller's grip pose, grip, trigger and B/Y for the
// arms' grab and drag (teleop/vr_arms.py). A thumbstick press zeroes the head where the headset looks; starting the
// session or recentering the headset restarts the zero at the heading, level. Inside, the headset shows the head
// camera's picture and, when the operator presses A or X, a ring of controls in front of them (xr_ring.js): that
// controller's thumbstick tilts to pick an item and presses down to choose it (its press is then the ring's, not the
// head's zero), and A or X goes back or closes: start, step and end Helm's teleop session from the headset (taking
// Helm's control from another device, which keeps showing the session, view-only: xr_session.js), the picture's view,
// the head's zero. While the ring is open on a controller, its arm takes no part. With the ring closed, a one-line
// alert shows a fault, a hold, lost control or a lost link. The wrist cameras' pictures sit on the model's wrists.
// Opened with ?remote, for a slow or lossy link (over Tailscale, say): the head camera and the poses go over WebRTC
// (xr_remote.js), with no depth or wrist pictures, and the page asks Helm for less, less often.
import * as THREE from 'three';
import {HeadCamera} from './xr_camera.js';
import {RobotModel} from './xr_model.js';
import {RemoteCamera} from './xr_remote.js';
import {Ring} from './xr_ring.js';
import {HeadsetScene} from './xr_scene.js';
import {ALERT_S, POLL_MS, SessionControl, alertOf, ringMenu, statusLines} from './xr_session.js';
import {cameraLine, degrees, handMessage, headLine, otherLinks, pageLink, poseMessage} from './xr_status.js';
import {DEFAULT_VIEW} from './xr_view.js';
import {CAMERAS} from './xr_wrists.js';

const $ = id => document.getElementById(id);
const SEND_INTERVAL_MS = 1000 / 60;
const TEXT_INTERVAL_MS = 250;
const STATUS_STALE_MS = 1000;  // Helm answers every 15th pose: this long without a word, the headset is not tracking.
const RETRY_MS = 1000;
const BUSY_RETRY_MS = 3000;  // Another device's headset steers: ask again now and then.
const HANDS = ['left', 'right'];
const REMOTE = typeof location !== 'undefined' && new URLSearchParams(location.search).has('remote');
const PAGE = pageLink(REMOTE);
const REMOTE_POLL_MS = 2000, REMOTE_GUIDE_MS = 500;  // Helm's state and the model's pose: a link's worth, not Helm's.

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
let wrists = [];  // The wrist cameras' streams, while the Wrists switch is on.
let mounts = null;  // The wrist cameras' mounts on the model (Helm's /api/helm/xr/wrists), once fetched.
let headset = {};  // Helm's latest word on the headset and the head (its status replies),
let statusAt = -Infinity;  // and when it came.
let sticks = {};  // Each controller's thumbstick press at the last frame, by hand.
let others = [];  // Helm's /xr links elsewhere (the network, Minerva's access point), from its last state.
let alert = null;  // The alert with the ring closed: {text, tone, until} (until Infinity: it stays until the ring opens).
let polling = null;
let pollRun = 0;  // Each XR session's polling: one ended mid-request does not poll on beside the next one's.
let status = [];  // The ring's middle, every TEXT_INTERVAL_MS.
const link = {connected: false, message: 'connecting…', error: null};  // `error`: Helm's last refusal, until a status.

// Helm's teleop session from here: its state polled while the headset runs, each new one checked for an alert.
const control = new SessionControl({token: () => token, onState: (state, previous) => {
  token = state.action_token || token;
  const next = alertOf(state, previous, link);
  if (next) raise(next);
}});
const ring = new Ring(path => ringMenu(control.state, path, view?.view || DEFAULT_VIEW));

function raise({text, tone, sticky}) {
  alert = {text, tone, until: sticky ? Infinity : performance.now() + ALERT_S * 1000};
}

function show(id, text, tone) {
  const element = $(id);
  element.textContent = text;
  element.className = tone || '';
}

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

// A pose: on the remote link's channel while it is open, else the socket.
function sendPose(message) {
  const text = JSON.stringify(message);
  if (camera?.sendPose?.(text)) return;
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(text);
}

// The page's WebRTC offer to Helm, its answer back.
async function remoteOffer(offer) {
  const response = await fetch('/api/helm/xr/remote', {method: 'POST', body: JSON.stringify(offer),
    headers: {'Content-Type': 'application/json', 'X-Helm-Token': token || ''}});
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || `Helm answered ${response.status}`);
  return data;
}

function linkLine(camera) {
  const {bitrate_bps: rate, rtt_ms: rtt} = camera.link || {};
  return REMOTE ? [`Remote link${rate == null ? '' : ` · ${(rate / 1e6).toFixed(2)} Mbit/s`}${rtt == null ? ''
    : ` · ${Math.round(rtt)} ms`}`, ''] : null;
}

// What the page says while no XR session runs.
function idle() {
  if (session) return;
  if (!navigator.xr || immersive === false) {
    show('status', `This browser ${navigator.xr ? 'cannot start an immersive session' : 'has no WebXR'}: open this `
      + 'page in the headset\'s browser.', 'bad');
  } else if (socket && socket.readyState === WebSocket.OPEN && immersive) {
    show('status', `Ready${REMOTE ? ' on the remote link' : ''}: put the headset on and start head tracking.`);
  }
}

function onStatus(reply) {
  if (reply.type === 'error') {
    show('status', reply.error, 'bad');
    link.error = reply.error;
    if (reply.busy) retryMs = BUSY_RETRY_MS;
    return;
  }
  link.error = null;
  headset = {...headset, ...reply};  // A zero or start reply says nothing of the head: its last word stays.
  view?.setZero(reply.zero);  // Helm's zero, wherever it was set (here, or Helm's teleop tab): the 3D's anchor.
  statusAt = performance.now();
  show('tracking', reply.tracking ? `yes · ${reply.rate_hz ? Math.round(reply.rate_hz) + ' Hz' : ''}` : 'no',
    reply.tracking ? 'ok' : 'warn');
  show('angles', `${degrees(reply.yaw)} / ${degrees(reply.pitch)}`);
  const head = headset.head ? `${headset.head}${headset.source ? ' · ' + headset.source : ''}` : 'no teleop session';
  show('head', headset.steering ? `steering · ${head}` : head, headset.steering ? 'ok' : 'warn');
  if (reply.zeroed) show('status', 'Zeroed: this is the head\'s straight ahead.', 'ok');
  if (reply.restarted) show('status', 'Tracking: where you looked as it started, level, is the head\'s zero.', 'ok');
}

async function connect() {
  try {
    const response = await fetch('/api/helm/state', {cache: 'no-store'});
    if (response.status === 401) {
      location.href = `/login?next=${PAGE.page}`;  // Back to this page as it was opened: logins end with Helm.
      return;
    }
    const state = await response.json();
    token = state.action_token;
    control.take(state);
    others = otherLinks(state.network?.links, location.host);
    show('elsewhere', others.length ? others.map(other => `${other.where}: ${other.url}`).join(' · ') : '—');
  } catch (error) {
    show('link', `unreachable (${error.message})`, 'bad');
    if (others.length && !session) {  // Minerva may have switched networks: say where Helm is then.
      show('status', `Helm does not answer here. If Minerva moved to ${others.map(other => `${other.where}, open `
        + other.url).join('; if to ')} (join its Wi-Fi first).`, 'warn');
    }
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

// Helm's state while the headset runs: the ring's items and middle follow it, and a change may raise an alert.
async function poll(run) {
  try {
    await control.refresh();
  } catch (error) {
    raise({text: `Helm: ${error.message}`, tone: 'warn', sticky: false});
  }
  if (session && run === pollRun) polling = setTimeout(() => poll(run), REMOTE ? REMOTE_POLL_MS : POLL_MS);
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
    // The 3D switch asks Helm for the head camera's frames with their depth (or just the frames) from here on.
    view = new HeadsetScene(renderer, {storage: browserStorage(), ...(REMOTE && {robot: new RobotModel({guideMs: REMOTE_GUIDE_MS})}),
      onDepth: depth => camera?.setRole(depth ? 'head_depth' : 'head'), onHands: streamWrists});
  }
  renderer.setClearColor(0x0f1218, mode === 'immersive-ar' ? 0 : 1);  // Passthrough shows through a clear frame.
  await renderer.xr.setSession(session);
  const space = renderer.xr.getReferenceSpace();
  send({type: 'start'});
  view.rezero();
  space.addEventListener('reset', () => {  // A recenter: the space starts again here.
    send({type: 'start'});
    view.rezero();
  });
  const onFrame = (bitmap, _metadata, depth) => {
    view.showFrame(bitmap);
    if (depth) view.showDepth(depth);
  };
  camera = REMOTE ? new RemoteCamera({offer: remoteOffer, onFrame, onStatus: onStatus})
    : new HeadCamera({token: () => token, role: view.view.depth ? 'head_depth' : 'head', onFrame});
  camera.start();
  loadMounts();
  streamWrists(view.view.hands);
  poll(++pollRun);
  session.addEventListener('end', () => {
    renderer.setAnimationLoop(null);
    camera?.stop();
    camera = null;
    session = null;
    sticks = {};
    clearTimeout(polling);
    ring.close();
    view.showRing(null);
    alert = null;
    streamWrists(false);
    $('start').textContent = 'Start head tracking';
    show('status', 'Head tracking stopped: the head holds, then returns to its zero.', 'warn');
  });
  $('start').textContent = 'Stop head tracking';
  show('status', `Tracking (${mode === 'immersive-ar' ? 'passthrough' : 'VR'}${REMOTE ? ', remote link' : ''}).`, 'ok');
  let textAt = -Infinity, frameAt = null;
  renderer.setAnimationLoop((time, frame) => {
    const pose = frame?.getViewerPose(renderer.xr.getReferenceSpace());
    const working = ring.hand;  // The controller on the ring as this frame began: its stick press is the ring's.
    for (const action of ring.step(ringInputs(), time / 1000, frameAt === null ? 0 : (time - frameAt) / 1000)) {
      choose(action);
    }
    frameAt = time;
    if (pose && time - lastSent >= SEND_INTERVAL_MS) {
      sequence += 1;
      sendPose(poseMessage(pose.transform.orientation, sequence, controllers(frame)));
      lastSent = time;
    }
    zeroOnThumbstick(working);
    if (camera && time - textAt >= TEXT_INTERVAL_MS) {
      textAt = time;
      const picture = camera.stats;
      const word = performance.now() - statusAt > STATUS_STALE_MS ? {...headset, tracking: false} : headset;
      const [, wristWhy] = view.wristState;
      status = statusLines(control.state, link, [headLine(word), cameraLine(picture), linkLine(camera),
        !REMOTE && wristWhy && [wristWhy, 'warn']]);
      view.setFresh(picture.fresh);
    }
    view.update(pose);  // This frame's pose: a following picture is drawn where the eyes are now.
    view.showRing(ring.drawing(time / 1000), status);
    if (alert && performance.now() > alert.until) alert = null;
    view.showAlert(alert);
    renderer.render(view.scene, view.camera);
  });
}

// Each controller's A or X, thumbstick press and tilt (xr-standard: buttons 4 and 3, axes 2 and 3), for the ring.
function ringInputs() {
  const inputs = [];
  for (const source of session.inputSources) {
    if (source.hand || !source.gamepad || !HANDS.includes(source.handedness)) continue;
    const {buttons, axes} = source.gamepad;
    inputs.push({hand: source.handedness, menu: Boolean(buttons[4]?.pressed), press: Boolean(buttons[3]?.pressed),
      stick: [Number(axes[2]) || 0, Number(axes[3]) || 0]});
  }
  return inputs;
}

// A ring item chosen: the view's own, the head's zero, or one of the session's (Helm's answer, or its refusal, in the
// alert once the ring has closed, else in the ring).
function choose(action) {
  if (action.id === 'opened') {
    alert = null;  // Opening the ring answers the alert: its middle says what is wrong.
    return;
  }
  if (view.choose(action)) return;
  if (action.id === 'zero') {
    send({type: 'zero'});
    view.rezero();
    return;
  }
  control.act(action.id).then(state => {
    const [text, tone] = statusLines(state, {connected: true})[0];
    if (!ring.open) raise({text, tone: tone === 'bad' ? 'bad' : 'ok', sticky: false});
  }, error => {
    if (ring.open) ring.notice = error.message;
    else raise({text: error.message, tone: 'bad', sticky: false});
  });
}

// The controllers in this frame, for Helm: {left, right} (handMessage), each with its grip's pose in the 'local'
// space; a controller working the ring is marked so its arm takes no part.
function controllers(frame) {
  const hands = {};
  for (const source of session.inputSources) {
    if (source.hand || !source.gripSpace || !source.gamepad || !HANDS.includes(source.handedness)) continue;
    const grip = frame.getPose(source.gripSpace, renderer.xr.getReferenceSpace());
    if (grip) {
      hands[source.handedness] = handMessage(grip.transform.position, grip.transform.orientation, source.gamepad,
        ring.hand === source.handedness);
    }
  }
  return hands;
}

// A thumbstick pressed (xr-standard button 3) zeroes the head where the headset looks, once a press; not on the
// controller working the ring (`working`: as the frame began), whose press chooses, even one that closed the ring.
function zeroOnThumbstick(working) {
  for (const source of session.inputSources) {
    if (source.hand || !source.gamepad) continue;
    const pressed = Boolean(source.gamepad.buttons[3]?.pressed);
    if (pressed && !sticks[source.handedness] && working !== source.handedness && ring.hand !== source.handedness) {
      send({type: 'zero'});
      view.rezero();
    }
    sticks[source.handedness] = pressed;
  }
}

// The wrist cameras' mounts on the model, once from Helm.
function loadMounts() {
  if (mounts) {
    view.setWristMounts(mounts);
    return;
  }
  fetch('/api/helm/xr/wrists', {cache: 'no-store'}).then(response => response.ok ? response.json() : null)
    .then(data => {
      if (!data?.mounts) return;
      mounts = data.mounts;
      view?.setWristMounts(mounts);
    }, () => {});
}

// The wrist cameras' pictures on the model's wrists: their streams while the Wrists switch is on (and a session
// runs), else none.
function streamWrists(on) {
  for (const stream of wrists) stream.stop();
  wrists = on && session && !REMOTE ? CAMERAS.map(role => {
    const stream = new HeadCamera({token: () => token, role, onFrame: bitmap => view.showWrist(role, bitmap)});
    stream.start();
    return stream;
  }) : [];
}

// The headset browser's localStorage, where the picture's view is remembered, or null where it is refused.
function browserStorage() {
  try {
    return window.localStorage;
  } catch {
    return null;
  }
}

if (typeof document !== 'undefined' && $('start')) {
  show('video', PAGE.name, REMOTE ? 'ok' : '');
  $('other-link').textContent = PAGE.switchTo;
  $('other-link').onclick = () => {
    location.href = PAGE.other;
  };
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
