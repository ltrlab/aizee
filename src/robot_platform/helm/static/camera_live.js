// Read-only physical preview. This socket never starts a session or changes a camera.
const $ = id => document.getElementById(id);
const ROLES = {left_wrist:'Left wrist', right_wrist:'Right wrist', head:'Head'};
const STALE_S = 1, MAX_JPEG = 2 * 1024 * 1024, MAX_HEADER = 4096;
const RETRY_MS = [1000, 2000, 4000];

function boundedJpeg(jpeg) {
  // Bound decoded allocation before handing compressed bytes to the browser.
  for (let offset = 2; offset + 4 < jpeg.length;) {
    if (jpeg[offset++] !== 0xff) break;
    while (jpeg[offset] === 0xff) offset++;
    const marker = jpeg[offset++];
    if (marker === 0xda || marker === 0xd9) break;
    const length = (jpeg[offset] << 8) | jpeg[offset + 1];
    if (length < 2 || offset + length > jpeg.length) break;
    if ([0xc0, 0xc1, 0xc2].includes(marker) && length >= 8) {
      const height = (jpeg[offset + 3] << 8) | jpeg[offset + 4];
      const width = (jpeg[offset + 5] << 8) | jpeg[offset + 6];
      if (!width || !height || width > 4096 || height > 4096 || width * height > 4194304)
        throw new Error('Camera image dimensions exceed preview memory bound');
      return;
    }
    offset += length;
  }
  throw new Error('Missing supported JPEG dimensions');
}

export function parseCameraPacket(packet) {
  if (!(packet instanceof ArrayBuffer) || packet.byteLength < 8 || packet.byteLength > MAX_JPEG + MAX_HEADER + 4)
    throw new Error('Invalid camera packet size');
  const length = new DataView(packet).getUint32(0);
  if (!length || length > MAX_HEADER || length + 4 >= packet.byteLength || packet.byteLength - length - 4 > MAX_JPEG)
    throw new Error('Invalid camera header size');
  const metadata = JSON.parse(new TextDecoder('utf-8', {fatal:true}).decode(new Uint8Array(packet, 4, length)));
  if (!Object.hasOwn(ROLES, metadata.role) || !Number.isSafeInteger(metadata.sequence) || metadata.sequence < 0
      || !Number.isFinite(metadata.age_s) || metadata.age_s < 0 || metadata.age_s >= STALE_S
      || !Number.isFinite(metadata.source_timestamp) || !Number.isSafeInteger(metadata.viewer_heartbeat)
      || metadata.viewer_heartbeat < 0) throw new Error('Invalid camera metadata');
  const jpeg = new Uint8Array(packet, length + 4);
  if (jpeg[0] !== 0xff || jpeg[1] !== 0xd8) throw new Error('Invalid camera JPEG');
  boundedJpeg(jpeg);
  return {metadata, jpeg};
}

export class LiveCameraView {
  constructor({onView = () => {}} = {}) {
    this.onView = onView; this.scope = ''; this.mode = 'cameras'; this.selected = 'right_wrist';
    this.paused = false; this.attempt = 0; this.generation = 0; this.roles = new Map();
    this.heartbeats = new Map(); this.heartbeatSequence = 0;
    for (const [role, title] of Object.entries(ROLES)) {
      const button = document.createElement('button'); button.type = 'button'; button.dataset.cameraRole = role;
      const image = document.createElement('img'); image.alt = `${title} physical camera`; image.hidden = true;
      const label = document.createElement('b'); label.textContent = title;
      const status = document.createElement('span'); status.textContent = 'Disconnected';
      button.append(image, label, status); button.onclick = () => {this.selected = role; this.paint();};
      $('camera-thumbnails').append(button);
      this.roles.set(role, {button, image, status, title, sequence:-1, message:'Disconnected', url:null, decode:null});
    }
    for (const button of document.querySelectorAll('[data-preview-view]')) button.onclick = () => {
      this.mode = button.dataset.previewView; this.sync();
    };
    $('camera-pause').onclick = () => {this.paused = !this.paused; this.attempt = 0; this.sync();};
    $('camera-retry').onclick = () => {this.attempt = 0; this.stop('Reconnecting'); this.sync();};
    this.visibility = () => this.sync();
    this.pagehide = () => {this.suspended = true; this.wasWanted = false; this.stop('Disconnected');};
    this.pageshow = () => {this.suspended = false; this.sync();};
    document.addEventListener('visibilitychange', this.visibility);
    window.addEventListener('pagehide', this.pagehide); window.addEventListener('pageshow', this.pageshow);
    this.paint();
  }
  updateContext(state, scope) {
    if (scope !== this.scope) {this.mode = 'cameras'; this.paused = false; this.attempt = 0;}
    const changed = this.token !== state?.action_token || this.robotMode !== state?.mode;
    this.scope = scope; this.token = state?.action_token; this.robotMode = state?.mode;
    if (changed) this.stop('Disconnected');
    this.sync();
  }
  wanted() {
    return !!this.scope && this.mode === 'cameras' && this.robotMode === 'robot' && !!this.token
      && !this.paused && !document.hidden && !this.suspended;
  }
  sync() {
    const physical = this.robotMode === 'robot';
    this.showing = !!this.scope && physical && this.mode === 'cameras';
    $('preview-view-choice').hidden = !this.scope;
    $('camera-live-view').hidden = !this.showing;
    document.body.dataset.cameraView = this.showing ? 'cameras' : 'model';
    for (const button of document.querySelectorAll('[data-preview-view]')) {
      button.disabled = button.dataset.previewView === 'cameras' && !physical;
      button.setAttribute('aria-pressed', String(button.dataset.previewView === (this.showing ? 'cameras' : 'model')));
    }
    $('camera-live-note').textContent = this.robotMode === 'simulation'
      ? 'Physical camera feeds are unavailable in simulation. Use the simulation scene window.'
      : 'Live physical cameras appear in the camera pane. Preview freshness is separate from recording readiness.';
    $('camera-pause').textContent = this.paused ? 'Resume preview' : 'Pause preview';
    $('camera-pause').hidden = !this.showing;
    this.onView(this.showing);
    const wanted = this.wanted();
    if (wanted && !this.wasWanted) this.attempt = 0;
    this.wasWanted = wanted;
    if (!wanted) {this.stop(this.paused ? 'Paused' : 'Disconnected'); return;}
    if (!this.socket && !this.retryTimer && this.attempt <= RETRY_MS.length) this.connect();
  }
  stop(message) {
    clearTimeout(this.retryTimer); this.retryTimer = null; clearInterval(this.watchdog); this.watchdog = null;
    this.generation++;
    this.heartbeats.clear();
    if (this.socket) {this.socket.onclose = this.socket.onerror = this.socket.onmessage = null; this.socket.close(); this.socket = null;}
    for (const role of this.roles.values()) this.clear(role, message);
    $('camera-stream-status').textContent = message; $('camera-retry').hidden = true;
    this.paint();
  }
  connect() {
    const generation = ++this.generation;
    for (const role of this.roles.values()) {this.clear(role, 'Waiting for frame'); role.sequence = -1;}
    $('camera-stream-status').textContent = this.attempt ? `Reconnecting (${this.attempt}/3)…` : 'Connecting…';
    $('camera-retry').hidden = true;
    const url = new URL('/api/helm/cameras/live', location.href);
    url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:'; url.searchParams.set('token', this.token);
    let socket;
    try {socket = new WebSocket(url); socket.binaryType = 'arraybuffer';}
    catch {this.lost(generation); return;}
    this.socket = socket; this.lastMessage = performance.now();
    socket.onopen = () => {
      if (generation === this.generation) {this.heartbeat(); $('camera-stream-status').textContent = 'Connected · waiting for fresh cameras';}
    };
    socket.onclose = () => this.lost(generation);
    socket.onerror = () => socket.close();
    socket.onmessage = event => {
      if (generation !== this.generation || !this.wanted()) return;
      this.lastMessage = performance.now();
      try {
        if (typeof event.data === 'string') this.status(event.data);
        else this.frame(parseCameraPacket(event.data), generation);
      } catch {this.lost(generation, 'Invalid camera stream');}
    };
    this.watchdog = setInterval(() => {
      if (performance.now() - this.lastMessage > 5000) this.lost(generation, 'Camera stream timed out');
      else {this.heartbeat(); this.paint();}
    }, 250);
    this.paint();
  }
  lost(generation, message = 'Camera stream disconnected') {
    if (generation !== this.generation) return;
    this.stop(message);
    if (!this.wanted()) return;
    if (this.attempt >= RETRY_MS.length) {
      this.attempt = RETRY_MS.length + 1;
      $('camera-stream-status').textContent = `${message}. Automatic retries exhausted.`;
      $('camera-retry').hidden = false; return;
    }
    const delay = RETRY_MS[this.attempt++];
    $('camera-stream-status').textContent = `${message}. Retrying in ${delay / 1000}s…`;
    this.retryTimer = setTimeout(() => {this.retryTimer = null; if (this.wanted()) this.connect();}, delay);
  }
  heartbeat() {
    if (this.socket?.readyState !== WebSocket.OPEN) return;
    const sequence = ++this.heartbeatSequence;
    this.heartbeats.set(sequence, performance.now());
    while (this.heartbeats.size > 16) this.heartbeats.delete(this.heartbeats.keys().next().value);
    this.socket.send(JSON.stringify({type:'heartbeat', sequence}));
  }
  transitBound(sequence) {
    const sent = this.heartbeats.get(sequence);
    return sent === undefined ? Infinity : (performance.now() - sent) / 1000;
  }
  status(text) {
    if (text.length > 65536) throw new Error('Oversized status');
    const state = JSON.parse(text);
    if (state.type !== 'status' || state.stale_after_s !== STALE_S || !state.cameras) throw new Error('Invalid status');
    $('camera-stream-status').textContent = state.error || (state.link === 'slow'
      ? 'Slow link · fewer frames per camera' : 'Physical preview · up to 10 fps per camera');
    const transit = this.transitBound(state.viewer_heartbeat);
    for (const [name, role] of this.roles) {
      const camera = state.cameras[name];
      if (camera?.fresh !== true || !Number.isFinite(camera.age_s) || camera.age_s < 0 || camera.age_s + transit >= STALE_S)
        this.clear(role, camera?.error || 'Stale · waiting for fresh frame');
    }
    this.paint();
  }
  age(role) {return role.age + (performance.now() - role.received) / 1000;}
  clear(role, message) {
    clearTimeout(role.expiry);
    if (role.decode) {clearTimeout(role.decode.timer); role.decode.image.src = ''; URL.revokeObjectURL(role.decode.url); role.decode = null;}
    if (role.url) URL.revokeObjectURL(role.url);
    role.url = null; role.message = message;
  }
  frame({metadata, jpeg}, generation) {
    const role = this.roles.get(metadata.role);
    const boundedAge = metadata.age_s + this.transitBound(metadata.viewer_heartbeat);
    if (boundedAge >= STALE_S) {this.clear(role, 'Stale · waiting for fresh frame'); this.paint(); return;}
    if (metadata.sequence <= role.sequence) return;
    role.sequence = metadata.sequence;
    if (role.decode) return; // Drop incoming work while one decode is in flight; never queue frames.
    const image = new Image(), url = URL.createObjectURL(new Blob([jpeg], {type:'image/jpeg'}));
    const decode = {image, url, received:performance.now(), age:boundedAge};
    role.decode = decode; image.src = url;
    decode.timer = setTimeout(() => {
      if (role.decode === decode) {this.clear(role, 'Frame decode timed out'); this.paint();}
    }, 1000);
    image.decode().then(() => {
      if (role.decode !== decode || generation !== this.generation || !this.wanted()) return;
      clearTimeout(decode.timer); role.decode = null;
      const age = decode.age + (performance.now() - decode.received) / 1000;
      if (age >= STALE_S || image.naturalWidth > 4096 || image.naturalHeight > 4096) {
        URL.revokeObjectURL(url); this.clear(role, 'Stale · waiting for fresh frame'); this.paint(); return;
      }
      if (role.url) URL.revokeObjectURL(role.url);
      role.url = url; role.received = decode.received; role.age = decode.age;
      clearTimeout(role.expiry);
      role.expiry = setTimeout(() => {this.clear(role, 'Stale · waiting for fresh frame'); this.paint();}, (STALE_S - age) * 1000);
      this.paint();
    }).catch(() => {
      if (role.decode === decode) {this.clear(role, 'Camera image could not be decoded'); this.paint();}
    });
  }
  paintImage(image, url) {
    image.hidden = !url;
    if (url && image.getAttribute('src') !== url) image.src = url;
    else if (!url) image.removeAttribute('src');
  }
  paint() {
    for (const [name, role] of this.roles) {
      if (role.url && this.age(role) >= STALE_S) this.clear(role, 'Stale · waiting for fresh frame');
      role.button.setAttribute('aria-pressed', String(name === this.selected));
      role.button.dataset.fresh = String(!!role.url);
      role.status.textContent = role.url ? `Live · ${Math.ceil(this.age(role) * 1000)} ms` : role.message;
      this.paintImage(role.image, role.url);
    }
    const selected = this.roles.get(this.selected);
    $('camera-focus-title').textContent = selected.title;
    $('camera-focus-status').textContent = selected.status.textContent;
    $('camera-focus-placeholder').textContent = selected.url ? '' : selected.message;
    $('camera-focus-placeholder').hidden = !!selected.url;
    this.paintImage($('camera-focus-image'), selected.url);
  }
}
