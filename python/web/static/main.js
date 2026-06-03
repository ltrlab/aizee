// AIZEE Quest WebXR client — Phase 1 entry point.
//
// Boots a WebXR immersive-vr session and streams controller poses + buttons
// over /ws/control to the host at the headset's frame rate (~90 Hz on Quest
// Pro).  No scene is rendered yet beyond a tiny reticle; Phase 3+ adds the
// camera plane and Phase 4 adds the URDF mirror.
//
// Architecture note: we keep ALL state in module-scope variables here and
// avoid a heavyweight framework.  The whole client is ~2 files because the
// host does all the IK + safety logic.

import * as THREE from 'three';
import { ControlSocket, TelemSocket } from '/static/ws.js';
import { buildScene, updateReticles, attachCameraVideo, updateURDF, animateURDF, setPassthroughMode, renderPreviewPanel,
         setPreviewEnabled, setGhostEnabled, getRobotGroupPosition, getRobotGroupYaw, nudgeRobotGroup } from '/static/scene.js';
import { startCameraWebRTC, getCameraVideo, onCamState } from '/static/cam.js';
import { buildVRUI, tickVRUI, registerLocalCommand, isPointerOverButton } from '/static/vrui.js';

let _passthrough = false;   // current session mode
// Direct grab: left-pinch (NOT over a UI button) grabs the robot group
// and moves it with the hand; release to drop.  Offset captured on grab
// so the robot doesn't snap to the fingertip.
let _grabbing = false;
let _grabOffset = [0, 0, 0];
let _grabYawRobot0 = 0;     // robot yaw at grab start
let _grabYawHand0 = 0;      // left-wrist yaw at grab start
let _leftPinchPrev = false;

// Extract the yaw component (rotation about world +Y) from a wrist
// quaternion using YXZ Euler decomposition.  YXZ puts yaw first so the
// returned angle is decoupled from wrist pitch/roll — twisting the wrist
// rotates the robot, tilting the wrist does not.
function _quatYaw(quatArray) {
  const q = new THREE.Quaternion(quatArray[0], quatArray[1], quatArray[2], quatArray[3]);
  const e = new THREE.Euler().setFromQuaternion(q, 'YXZ');
  return e.y;
}
let _previewOn = true;
let _ghostOn = true;

const $ = (sel) => document.querySelector(sel);
const setStatus = (msg, cls = '') => {
  const el = $('#status');
  el.textContent = msg;
  el.style.borderLeftColor = cls === 'err' ? 'var(--err)'
    : cls === 'warn' ? 'var(--warn)' : 'var(--accent)';
};

// ---- WebXR feature detection -----------------------------------------------
//
// We DON'T gate the button on detection — it's always clickable so the
// operator can tap it the moment the page loads.  detectXR() updates the
// status message in the background; if the click fires before detection
// completes, enterVR() handles the error path itself.

async function detectXR() {
  if (!('xr' in navigator)) {
    setStatus('WebXR API not available in this browser. Use the Meta Quest browser.', 'err');
    return false;
  }
  try {
    const ok = await navigator.xr.isSessionSupported('immersive-vr');
    if (!ok) {
      setStatus('immersive-vr session not supported. Quest browser required.', 'err');
      return false;
    }
  } catch (e) {
    setStatus(`WebXR probe failed: ${e.message}`, 'err');
    return false;
  }
  setStatus('Ready. Tap "Enter VR" to start.');
  return true;
}

// ---- WebSocket bring-up ----------------------------------------------------

const ctrlSock = new ControlSocket();
const telemSock = new TelemSocket();

function bindSocketIndicators() {
  ctrlSock.onState = (s) => { $('#ws-control').textContent = s; };
  telemSock.onState = (s) => { $('#ws-telem').textContent = s; };
  onCamState((s) => { $('#ws-cam').textContent = `webrtc: ${s}`; });
}

bindSocketIndicators();

// ---- Pose extraction helpers ----------------------------------------------

function poseToObj(pose) {
  // XRPose -> { pos:[x,y,z], quat:[x,y,z,w] } in the session's reference space.
  if (!pose) return null;
  const t = pose.transform;
  const p = t.position;
  const o = t.orientation;
  return { pos: [p.x, p.y, p.z], quat: [o.x, o.y, o.z, o.w] };
}

function readController(inputSource, frame, refSpace) {
  if (!inputSource) return null;
  // Two input modalities:
  //   * Tracked controllers — have gripSpace + gamepad (Touch / Touch Pro)
  //   * Hand tracking — have an XRHand (joint poses) but no gripSpace
  // We pick the right path per inputSource.
  if (inputSource.hand) return readHand(inputSource, frame, refSpace);
  if (inputSource.gripSpace) return readGamepadController(inputSource, frame, refSpace);
  return null;
}

function readGamepadController(inputSource, frame, refSpace) {
  const pose = frame.getPose(inputSource.gripSpace, refSpace);
  const out = poseToObj(pose);
  if (!out) return null;
  // Buttons: gamepad mapping for Quest controllers
  //   gp.buttons: [0]trigger [1]grip [2](unused) [3]thumbstick [4]A/X [5]B/Y
  //   gp.axes:    [0]touchpad_x [1]touchpad_y [2]stick_x [3]stick_y
  const gp = inputSource.gamepad;
  if (gp) {
    out.trigger = gp.buttons[0]?.value ?? 0;
    out.grip    = (gp.buttons[1]?.value ?? 0) > 0.5;
    out.thumb   = (gp.buttons[3]?.pressed ?? false);
    out.a       = (gp.buttons[4]?.pressed ?? false);
    out.b       = (gp.buttons[5]?.pressed ?? false);
    out.stick   = [gp.axes[2] ?? 0, gp.axes[3] ?? 0];
  }
  out.hand = inputSource.handedness;  // "left" | "right" | "none"
  out.kind = 'controller';
  return out;
}

// Pinch thresholds: index→thumb tip distance in METERS.  Hysteresis avoids
// jitter at the edge — once pinched, we hold until fingers spread wider.
const _PINCH_ON  = 0.025;   // 2.5 cm
const _PINCH_OFF = 0.040;   // 4.0 cm
const _pinchHeld = { left: { idx: false, mid: false }, right: { idx: false, mid: false } };

function _jointDist(frame, refSpace, hand, jointA, jointB) {
  const a = hand.get(jointA);
  const b = hand.get(jointB);
  if (!a || !b) return Infinity;
  const ap = frame.getJointPose ? frame.getJointPose(a, refSpace) : null;
  const bp = frame.getJointPose ? frame.getJointPose(b, refSpace) : null;
  if (!ap || !bp) return Infinity;
  const dx = ap.transform.position.x - bp.transform.position.x;
  const dy = ap.transform.position.y - bp.transform.position.y;
  const dz = ap.transform.position.z - bp.transform.position.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function readHand(inputSource, frame, refSpace) {
  const handedness = inputSource.handedness;  // "left" | "right"
  const hand = inputSource.hand;
  if (!hand || !frame.getJointPose) return null;
  // Wrist pose is the "controller-equivalent" position.  It's the most
  // stable joint and roughly where the controller's gripSpace sits, so
  // the IK delta math feels similar to controller mode.
  const wristJoint = hand.get('wrist');
  if (!wristJoint) return null;
  const wristPose = frame.getJointPose(wristJoint, refSpace);
  if (!wristPose) return null;
  const out = poseToObj({ transform: wristPose.transform });
  if (!out) return null;

  // Pinch detection with hysteresis.  Index pinch = clutch (grip).
  // Middle pinch = gripper close (mapped to trigger axis 0/1).
  const heldState = _pinchHeld[handedness] || _pinchHeld.right;
  const dIdx = _jointDist(frame, refSpace, hand, 'thumb-tip', 'index-finger-tip');
  const dMid = _jointDist(frame, refSpace, hand, 'thumb-tip', 'middle-finger-tip');
  heldState.idx = heldState.idx ? (dIdx < _PINCH_OFF) : (dIdx < _PINCH_ON);
  heldState.mid = heldState.mid ? (dMid < _PINCH_OFF) : (dMid < _PINCH_ON);
  out.grip    = heldState.idx;        // index pinch = clutch
  out.trigger = heldState.mid ? 1.0 : 0.0;  // middle pinch = gripper closed
  // No A/B/stick on hands — keep the fields so the server-side schema is
  // identical, just inert.  E-stop has to come from the controller path
  // for now (or the user can do dual-pinch — left of the brain to add).
  out.a = false;
  out.b = false;
  out.thumb = false;
  out.stick = [0, 0];
  out.hand = handedness;
  out.kind = 'hand';
  out.pinch_dist = dIdx;   // surfaced so the HUD can show pinch progress
  return out;
}

// ---- WebXR session ---------------------------------------------------------

let renderer = null;
let scene = null;
let camera = null;
let xrSession = null;
let xrRefSpace = null;

let lastSendTs = 0;
let lastFrameLog = 0;

function startRenderer() {
  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.xr.enabled = true;
  document.body.appendChild(renderer.domElement);
  renderer.domElement.style.display = 'none';  // hidden in lobby
  ({ scene, camera } = buildScene(renderer));
}

async function enterVR(passthrough = false) {
  if (!('xr' in navigator)) {
    setStatus('WebXR API not available in this browser. Open this page in the Meta Quest Browser.', 'err');
    return;
  }
  const mode = passthrough ? 'immersive-ar' : 'immersive-vr';
  try {
    const ok = await navigator.xr.isSessionSupported(mode);
    if (!ok) {
      setStatus(`${mode} not supported by this browser.`, 'err');
      return;
    }
  } catch (e) {
    setStatus(`WebXR probe failed: ${e.message}`, 'err');
    return;
  }
  if (!renderer) startRenderer();
  _passthrough = passthrough;
  setPassthroughMode(scene, renderer, passthrough);
  // AR/passthrough composites the camera feed on top of our render, which
  // is much heavier than VR.  Shed GPU load so the framerate holds:
  //   * Drop the framebuffer resolution (set BEFORE the session starts).
  //   * Max foveation (low-res periphery).
  //   * Turn off the render-to-texture preview panel (a full 2nd scene
  //     render) — in AR you can see the real arm through passthrough anyway.
  try {
    renderer.xr.setFramebufferScaleFactor(passthrough ? 0.7 : 1.0);
  } catch {}
  if (passthrough) setPreviewEnabled(false);
  else setPreviewEnabled(_previewOn);
  const sessionInit = {
    requiredFeatures: ['local-floor'],
    optionalFeatures: ['hand-tracking', 'high-fixed-foveation-level'],
  };
  try {
    xrSession = await navigator.xr.requestSession(mode, sessionInit);
  } catch (e) {
    setStatus(`requestSession failed: ${e.message}`, 'err');
    return;
  }
  renderer.xr.setSession(xrSession);
  // Foveated rendering — render the periphery at lower resolution.  Max
  // (1.0) in AR where we're GPU-bound; moderate (0.5) in VR.
  try { renderer.xr.setFoveation(passthrough ? 1.0 : 0.5); } catch {}
  xrRefSpace = await xrSession.requestReferenceSpace('local-floor');
  xrSession.addEventListener('end', () => {
    xrSession = null;
    xrRefSpace = null;
    setStatus('VR session ended. Tap "Enter VR" to resume.');
    renderer.domElement.style.display = 'none';
    $('#lobby').style.display = '';
  });
  $('#lobby').style.display = 'none';
  renderer.domElement.style.display = '';
  // Spawn the in-VR control panel — pokable buttons for realign / workspace
  // resize / etc.  Sits floating to the operator's left at chest height.
  buildVRUI(scene);
  // Local commands handled in the browser (no server roundtrip).  Passthrough
  // toggle ends the current XR session and re-requests the other mode.
  registerLocalCommand('_local_toggle_passthrough', async () => {
    const next = !_passthrough;
    if (xrSession) {
      try { await xrSession.end(); } catch {}
    }
    // The 'end' event handler will reset state; kick off a new session
    // a tick later so the renderer fully tears down first.
    setTimeout(() => enterVR(next), 200);
  });
  // Preview / ghost render toggles — these are the expensive extra mesh
  // renders, so the operator can turn them off if framerate suffers.
  registerLocalCommand('_local_toggle_preview', (_args, btn) => {
    _previewOn = !_previewOn;
    setPreviewEnabled(_previewOn);
    if (btn) btn.relabel(`Preview: ${_previewOn ? 'ON' : 'OFF'}`);
  });
  registerLocalCommand('_local_toggle_ghost', (_args, btn) => {
    _ghostOn = !_ghostOn;
    setGhostEnabled(_ghostOn);
    if (btn) btn.relabel(`Ghost: ${_ghostOn ? 'ON' : 'OFF'}`);
  });
  // Bring up camera (WebRTC) + telem socket now that we're in-headset.
  startCameraWebRTC().then(() => {
    const videoEl = getCameraVideo();
    if (videoEl) {
      const onCanPlay = () => {
        attachCameraVideo(scene, videoEl);
        videoEl.removeEventListener('canplay', onCanPlay);
      };
      // 'loadeddata' fires before 'canplay' on Quest; either is fine since
      // THREE.VideoTexture handles late-arriving frames.
      if (videoEl.readyState >= 2) attachCameraVideo(scene, videoEl);
      else videoEl.addEventListener('canplay', onCanPlay);
    }
  }).catch((e) => console.warn('[main] camera WebRTC failed:', e));
  telemSock.connect((telem) => { updateURDF(scene, telem); });

  renderer.setAnimationLoop(onXRFrame);
}

function onXRFrame(t, frame) {
  if (!frame || !xrSession || !xrRefSpace) return;
  // Headset pose
  const viewerPose = frame.getViewerPose(xrRefSpace);
  const headPose = viewerPose ? poseToObj({ transform: viewerPose.transform }) : null;
  // Controllers (or hands, transparently)
  let right = null, left = null;
  let leftFingerWorld = null;
  for (const src of xrSession.inputSources) {
    const c = readController(src, frame, xrRefSpace);
    if (c) {
      if (c.hand === 'right') right = c;
      else if (c.hand === 'left') left = c;
    }
    // Separately, capture the LEFT index-finger-tip world position for VR
    // UI hit-testing.  Wrist (used as the controller-equivalent pose) is
    // too far back to feel like "poking".
    if (src.handedness === 'left' && src.hand && frame.getJointPose) {
      const tipJoint = src.hand.get('index-finger-tip');
      if (tipJoint) {
        const tipPose = frame.getJointPose(tipJoint, xrRefSpace);
        if (tipPose) {
          const p = tipPose.transform.position;
          leftFingerWorld = new THREE.Vector3(p.x, p.y, p.z);
        }
      }
    }
  }
  updateReticles(scene, right, left);
  // Per-render-frame eased URDF tween so missed telem messages don't
  // show as position jumps in the mirror.
  animateURDF();

  const leftPinch = left && left.kind === 'hand' ? left.grip : false;

  // Direct grab-to-move: a left pinch that STARTS away from the UI panel
  // grabs the robot group; while held, the robot follows the hand; release
  // to drop.  A pinch that starts over a button is left for the UI.
  if (leftPinch && !_leftPinchPrev && leftFingerWorld && !isPointerOverButton(leftFingerWorld)) {
    const rp = getRobotGroupPosition();
    if (rp) {
      _grabOffset = [rp.x - leftFingerWorld.x, rp.y - leftFingerWorld.y, rp.z - leftFingerWorld.z];
      _grabYawRobot0 = getRobotGroupYaw() || 0;
      _grabYawHand0  = left && left.quat ? _quatYaw(left.quat) : 0;
      _grabbing = true;
    }
  }
  if (!leftPinch) _grabbing = false;
  _leftPinchPrev = leftPinch;

  if (_grabbing && leftFingerWorld) {
    // Apply wrist twist as world-Y yaw on the robot.  Wrap the delta to
    // [-π, π] so the robot doesn't spin the long way when the wrist
    // crosses ±π in YXZ space.
    let newYaw = _grabYawRobot0;
    if (left && left.quat) {
      const dy = _quatYaw(left.quat) - _grabYawHand0;
      const dyaw = Math.atan2(Math.sin(dy), Math.cos(dy));
      newYaw = _grabYawRobot0 + dyaw;
    }
    nudgeRobotGroup([leftFingerWorld.x, leftFingerWorld.y, leftFingerWorld.z], _grabOffset, newYaw);
    tickVRUI(null, false);   // UI inert while grabbing
  } else {
    // VR UI: poke detection.  Left-index pinch is the optional shortcut
    // to skip the dwell timer.
    tickVRUI(leftFingerWorld, leftPinch);
  }

  // Render the kinematic-preview panel to its off-screen target BEFORE
  // the main XR render, so the texture is fresh when the main camera
  // composites it.
  renderPreviewPanel(renderer, scene);

  // Stream to host at ~90 Hz, throttled to >= 1 ms gap so a 120 Hz headset
  // doesn't blow past the bandwidth budget.
  const now = performance.now();
  if ((now - lastSendTs) >= 1) {
    ctrlSock.send({
      ts: now / 1000,
      head: headPose,
      right, left,
    });
    lastSendTs = now;
  }
  if ((now - lastFrameLog) >= 1000) {
    $('#ctrl-ts').textContent = (now / 1000).toFixed(2) + ' s';
    lastFrameLog = now;
  }

  renderer.render(scene, camera);
}

// ---- Bootstrap -------------------------------------------------------------

window.addEventListener('load', async () => {
  ctrlSock.connect();  // pre-connect so it's hot when VR starts
  await detectXR();
  $('#enter-vr').addEventListener('click', () => enterVR(false));
  $('#enter-ar').addEventListener('click', () => enterVR(true));
});
