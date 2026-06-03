// WebRTC camera client.
//
// One peer connection per session.  We're recv-only (the server has the
// camera); aiortc gives us back a video track that we attach to a hidden
// <video> element.  scene.js then wraps the element in a THREE.VideoTexture
// so the rest of the scene code doesn't have to know about MediaStreams.

const SIGNAL_URL = '/api/webrtc/offer';

let _pc = null;
let _video = null;
let _state = 'disconnected';
let _onState = () => {};

export function onCamState(cb) { _onState = cb; cb(_state); }
function _set(s) { _state = s; _onState(s); }

function _waitIce(pc) {
  // aiortc waits for ICE gathering before returning the local description,
  // but the browser is trickle-by-default — we explicitly wait so the
  // offer we send already contains its host candidates (matches what the
  // server expects in non-trickle mode).
  return new Promise((resolve) => {
    if (pc.iceGatheringState === 'complete') return resolve();
    const check = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', check);
        resolve();
      }
    };
    pc.addEventListener('icegatheringstatechange', check);
  });
}

export async function startCameraWebRTC() {
  if (_pc) return _pc;  // idempotent — caller may invoke once per session
  _set('connecting');
  const pc = new RTCPeerConnection({ iceServers: [] });  // LAN host candidates
  _pc = pc;

  pc.addEventListener('connectionstatechange', () => {
    if (pc.connectionState === 'connected') _set('connected');
    else if (pc.connectionState === 'failed') _set('error');
    else if (pc.connectionState === 'disconnected') _set('disconnected');
    else if (pc.connectionState === 'closed') _set('closed');
  });

  // Receive-only: we want the server's track and don't send our own.
  pc.addTransceiver('video', { direction: 'recvonly' });

  // Build a hidden <video> element to back the texture; the WebRTC layer
  // hands us a MediaStream we attach to it.  Quest browser requires
  // playsInline + muted to autoplay a non-user-initiated video.
  const video = document.createElement('video');
  video.autoplay = true;
  video.muted = true;
  video.playsInline = true;
  video.style.position = 'absolute';
  video.style.left = '-9999px';  // off-screen but in DOM (some browsers need this)
  video.style.width = '2px'; video.style.height = '2px';
  document.body.appendChild(video);
  _video = video;

  pc.addEventListener('track', (evt) => {
    if (evt.track.kind !== 'video') return;
    const stream = evt.streams[0] ?? new MediaStream([evt.track]);
    video.srcObject = stream;
    // play() returns a promise; ignore failures (autoplay policy already permits)
    video.play().catch((e) => console.warn('[cam] video.play() failed:', e));
  });

  // Offer/answer
  const offer = await pc.createOffer({ offerToReceiveVideo: true });
  await pc.setLocalDescription(offer);
  await _waitIce(pc);

  let answer;
  try {
    const resp = await fetch(SIGNAL_URL, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        sdp: pc.localDescription.sdp,
        type: pc.localDescription.type,
      }),
    });
    if (!resp.ok) throw new Error(`signaling HTTP ${resp.status}`);
    answer = await resp.json();
  } catch (e) {
    _set('error');
    console.error('[cam] signaling failed:', e);
    await stopCameraWebRTC();
    throw e;
  }
  await pc.setRemoteDescription(answer);
  return pc;
}

export function getCameraVideo() {
  return _video;
}

export async function stopCameraWebRTC() {
  if (_pc) {
    try { _pc.close(); } catch {}
    _pc = null;
  }
  if (_video) {
    _video.srcObject = null;
    _video.remove();
    _video = null;
  }
  _set('disconnected');
}
