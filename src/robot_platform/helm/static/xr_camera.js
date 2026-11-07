// The head camera in the headset: Helm's camera stream (the same packets camera_live.js reads), asking for the head's
// frames only, each newest one decoded to an ImageBitmap for the screen (xr_scene.js). For the 3D picture it asks for
// the head's frames with their depth (role head_depth), inflated and filled in a worker (xr_fill.js) for xr_depth.js.
// Read-only: this socket never starts a session or changes a camera. Its heartbeats tell Helm the headset still
// watches, as the page's viewer does. No three.js here.
import { parseCameraPacket } from './camera_live.js';
import { DepthFiller, inflateDepth } from './xr_fill.js';

export { inflateDepth };

const HEARTBEAT_MS = 250,
  SILENT_MS = 5000,
  RETRY_MS = 2000,
  STALE_S = 1,
  FPS_WINDOW_MS = 2000;

// `token`: Helm's action token, or a function giving the current one (a restarted Helm has a new one). `role`:
// 'head', or 'head_depth' with the depth (onFrame then gets {meta, layers} after the bitmap and its metadata: the
// depth's two filled layers, xr_fill.fillDepth). `fill`: (packed depth, its metadata) to those layers; by default a
// worker, started with the first depth.
export class HeadCamera {
  constructor({
    token,
    onFrame,
    role = 'head',
    url = location.href,
    open = (address) => new WebSocket(address),
    decode = (blob) => createImageBitmap(blob),
    fill = null,
    now = () => performance.now(),
  }) {
    Object.assign(this, { token, onFrame, role, url, open, decode, now });
    this.fill = fill || ((packed, meta) => (this.filler ||= new DepthFiller()).fill(packed, meta));
    this.socket = null;
    this.stopped = true;
    this.sequence = -1;
    this.heartbeatSequence = 0;
    this.arrivals = [];
    this.shownAt = null; // When the screen last got a frame, and that frame's age then.
    this.shownAge = 0;
    this.message = 'connecting…';
    this.decoding = false;
    this.pending = null;
  }

  start() {
    this.stopped = false;
    this.connect();
  }

  stop() {
    this.stopped = true;
    clearInterval(this.timer);
    clearTimeout(this.retry);
    this.socket?.close();
    this.socket = null;
  }

  // The 3D picture switched on or off: Helm streams the role a viewer asks for as it connects, so connect again.
  setRole(role) {
    if (role === this.role) return;
    this.role = role;
    this.sequence = -1;
    this.pending = null;
    if (this.stopped) return;
    clearTimeout(this.retry);
    const socket = this.socket;
    this.socket = null;
    socket?.close();
    this.connect();
  }

  connect() {
    const address = new URL('/api/helm/cameras/live', this.url);
    address.protocol = address.protocol === 'https:' ? 'wss:' : 'ws:';
    address.searchParams.set('token', typeof this.token === 'function' ? this.token() : this.token);
    address.searchParams.set('roles', this.role);
    const socket = (this.socket = this.open(address.toString()));
    socket.binaryType = 'arraybuffer';
    this.lastMessage = this.now();
    socket.onopen = () => {
      this.message = 'waiting for a frame';
      this.heartbeat();
    };
    socket.onmessage = (event) => this.receive(event.data);
    socket.onerror = () => socket.close();
    socket.onclose = () => {
      if (this.socket !== socket) return;
      clearInterval(this.timer);
      this.socket = null;
      this.message = 'reconnecting…';
      if (!this.stopped) this.retry = setTimeout(() => this.connect(), RETRY_MS);
    };
    clearInterval(this.timer);
    this.timer = setInterval(() => {
      if (this.now() - this.lastMessage > SILENT_MS) socket.close();
      else this.heartbeat();
    }, HEARTBEAT_MS);
  }

  heartbeat() {
    if (this.socket?.readyState === 1)
      this.socket.send(JSON.stringify({ type: 'heartbeat', sequence: ++this.heartbeatSequence }));
  }

  receive(data) {
    this.lastMessage = this.now();
    if (typeof data === 'string') {
      try {
        const head = JSON.parse(data).cameras?.head;
        if (head && head.fresh !== true) this.message = head.error || 'no fresh frame';
      } catch {
        this.message = 'unreadable status';
      }
      return;
    }
    let packet;
    try {
      packet = parseCameraPacket(data);
    } catch (error) {
      this.message = error.message;
      return;
    }
    if (packet.metadata.role !== this.role || packet.metadata.sequence <= this.sequence) return;
    this.sequence = packet.metadata.sequence;
    const now = this.now();
    this.arrivals = [...this.arrivals.filter((at) => now - at < FPS_WINDOW_MS), now];
    if (this.decoding)
      this.pending = packet; // Only the newest waits: a slow decode never queues frames.
    else this.show(packet);
  }

  async show(packet) {
    this.decoding = true;
    try {
      const [bitmap, layers] = await Promise.all([
        this.decode(new Blob([packet.jpeg], { type: 'image/jpeg' })),
        packet.depth && this.fill(packet.depth, packet.metadata.depth),
      ]);
      this.shownAt = this.now();
      this.shownAge = packet.metadata.age_s;
      this.onFrame(bitmap, packet.metadata, layers ? { meta: packet.metadata.depth, layers } : null);
    } catch (error) {
      this.message = `undecodable frame (${error.message})`;
    } finally {
      this.decoding = false;
      const next = this.pending;
      this.pending = null;
      if (next) this.show(next);
    }
  }

  // {fresh, fps, age_s, message}: a picture older than STALE_S is not fresh.
  get stats() {
    const now = this.now();
    const age = this.shownAt == null ? Infinity : this.shownAge + (now - this.shownAt) / 1000;
    const recent = this.arrivals.filter((at) => now - at < FPS_WINDOW_MS);
    return {
      fresh: age < STALE_S,
      fps: (recent.length * 1000) / FPS_WINDOW_MS,
      age_s: age,
      message: age < STALE_S ? null : this.shownAt == null ? this.message : `last frame ${age.toFixed(0)} s ago`,
    };
  }
}
