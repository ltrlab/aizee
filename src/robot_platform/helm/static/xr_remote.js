// The head camera over WebRTC, for a headset on a slow or lossy link (the page opened with ?remote): H.264 that drops what
// is late and lowers its rate, instead of JPEGs on a socket that resends them, and an unordered channel for the poses,
// where the newest wins and a lost one holds nothing back. The same face as xr_camera.HeadCamera for xr.js, so it hands
// over ImageBitmaps for the same screen. No depth: the 3D picture needs each frame's depth and head pose, and video here
// carries neither, so the screen stays flat. Read-only for the picture; poses go to Helm, which checks them as it does
// the socket's. No three.js here.
const FRAME_CHECK_MS = 33, STALE_S = 1, FPS_WINDOW_MS = 2000, RETRY_MS = 3000, GATHER_MS = 2000, STATS_MS = 1000;
const POSE_CHANNEL = {ordered: false, maxRetransmits: 0};  // Late is useless: the next pose is 17 ms away.

export class RemoteCamera {
  // `offer`: (the page's offer) to Helm's answer. `onFrame`, as HeadCamera's. `onStatus`: Helm's word over the channel.
  // The rest are the browser's, replaceable to test.
  constructor({offer, onFrame, onStatus = () => {}, peer = () => new RTCPeerConnection({iceServers: []}),
    video = () => makeVideo(), grab = element => createImageBitmap(element), now = () => performance.now(),
    every = (fn, ms) => setInterval(fn, ms), stopEvery = id => clearInterval(id), after = (fn, ms) => setTimeout(fn, ms),
    stopAfter = id => clearTimeout(id)}) {
    Object.assign(this, {offer, onFrame, onStatus, peer, video, grab, now, every, stopEvery, after, stopAfter});
    this.stopped = true;
    this.pc = this.channel = this.element = null;
    this.arrivals = [];
    this.shownAt = null;
    this.frames = 0;  // Frames the browser had decoded at the last look.
    this.grabbing = false;
    this.message = 'connecting…';
    this.link = {};  // The last look at the link: {bitrate_bps, lost, rtt_ms, decoder}.
    this.bytes = null;
    this.bytesAt = null;
  }

  start() {
    this.stopped = false;
    this.element ||= this.video();
    this.checking = this.every(() => this.check(), FRAME_CHECK_MS);
    this.reading = this.every(() => this.read(), STATS_MS);
    this.connect();
  }

  stop() {
    this.stopped = true;
    this.stopEvery(this.checking);
    this.stopEvery(this.reading);
    this.stopAfter(this.retry);
    this.close();
    if (this.element) {
      this.element.srcObject = null;
      this.element.remove?.();
      this.element = null;
    }
  }

  close() {
    this.channel = null;
    this.pc?.close();
    this.pc = null;
  }

  setRole() {}  // The 3D switch: the picture stays flat here.

  async connect() {
    const pc = this.pc = this.peer();
    pc.addTransceiver('video', {direction: 'recvonly'});
    const channel = pc.createDataChannel('poses', POSE_CHANNEL);
    channel.onopen = () => {
      if (this.pc === pc) this.channel = channel;
    };
    channel.onclose = () => {
      if (this.channel === channel) this.channel = null;
    };
    channel.onmessage = event => {
      try {
        this.onStatus(JSON.parse(event.data));
      } catch {
        this.message = 'unreadable status';
      }
    };
    pc.ontrack = event => {
      this.element.srcObject = event.streams[0] || new MediaStream([event.track]);
      this.element.play?.().catch?.(() => {});
    };
    pc.onconnectionstatechange = () => {
      if (this.pc !== pc) return;
      if (pc.connectionState === 'failed' || pc.connectionState === 'closed') this.again(pc, 'reconnecting…');
      else if (pc.connectionState === 'connected') this.message = 'waiting for a frame';
    };
    try {
      await pc.setLocalDescription(await pc.createOffer());
      await gathered(pc, this.after, this.stopAfter);
      if (this.pc !== pc) return;
      const answer = await this.offer({sdp: pc.localDescription.sdp, type: pc.localDescription.type});
      if (this.pc === pc) await pc.setRemoteDescription(answer);
    } catch (error) {
      this.again(pc, `no link: ${error.message}`);
    }
  }

  again(pc, message) {
    if (this.pc !== pc) return;
    this.message = message;
    this.close();
    if (!this.stopped) this.retry = this.after(() => this.connect(), RETRY_MS);
  }

  // A pose for Helm, on the channel: false if it is not open, and the page's socket carries it.
  sendPose(text) {
    if (this.channel?.readyState !== 'open') return false;
    this.channel.send(text);
    return true;
  }

  // Whether the browser has decoded a frame since the last look, then that frame as a bitmap for the screen. A timer,
  // since a video element's frame callback may wait on the window's animation frames, which pause in an immersive session.
  async check() {
    const element = this.element;
    if (!element || this.grabbing || element.readyState < 2) return;
    const decoded = element.getVideoPlaybackQuality?.().totalVideoFrames ?? this.frames + 1;
    if (decoded === this.frames) return;
    this.frames = decoded;
    this.grabbing = true;
    try {
      const bitmap = await this.grab(element);
      const now = this.now();
      this.shownAt = now;
      this.arrivals = [...this.arrivals.filter(at => now - at < FPS_WINDOW_MS), now];
      this.onFrame(bitmap, {role: 'head'}, null);
    } catch (error) {
      this.message = `undecodable frame (${error.message})`;
    } finally {
      this.grabbing = false;
    }
  }

  // What the link does, from the browser's own counters.
  async read() {
    const report = await this.pc?.getStats?.().catch(() => null);
    if (!report) return;
    const now = this.now();
    for (const item of report.values()) {
      if (item.type === 'inbound-rtp' && item.kind === 'video') {
        if (this.bytes !== null && now > this.bytesAt) {
          this.link.bitrate_bps = (item.bytesReceived - this.bytes) * 8000 / (now - this.bytesAt);
        }
        this.bytes = item.bytesReceived;
        this.bytesAt = now;
        this.link.lost = item.packetsLost;
        this.link.decoder = item.decoderImplementation;
      } else if (item.type === 'candidate-pair' && item.nominated && item.currentRoundTripTime !== undefined) {
        this.link.rtt_ms = item.currentRoundTripTime * 1000;
      }
    }
  }

  // {fresh, fps, age_s, message}, as HeadCamera's: how old a frame was when it left the camera is unknown here, so the
  // age counts from when it was decoded.
  get stats() {
    const now = this.now();
    const age = this.shownAt == null ? Infinity : (now - this.shownAt) / 1000;
    const recent = this.arrivals.filter(at => now - at < FPS_WINDOW_MS);
    return {fresh: age < STALE_S, fps: recent.length * 1000 / FPS_WINDOW_MS, age_s: age,
      message: age < STALE_S ? null : this.shownAt == null ? this.message : `last frame ${age.toFixed(0)} s ago`};
  }
}

// ICE gathering done, or as long as a page should wait: the answer needs the page's candidates in its offer.
function gathered(pc, after, stopAfter) {
  if (pc.iceGatheringState === 'complete') return Promise.resolve();
  return new Promise(done => {
    const timer = after(done, GATHER_MS);
    pc.onicegatheringstatechange = () => {
      if (pc.iceGatheringState === 'complete') {
        stopAfter(timer);
        done();
      }
    };
  });
}

// The video element the stream plays in: on the page, since a browser may not decode for one that is not, but not seen.
function makeVideo() {
  const element = document.createElement('video');
  Object.assign(element, {autoplay: true, muted: true, playsInline: true});
  element.setAttribute('aria-hidden', 'true');
  element.style.cssText = 'position:fixed;width:2px;height:2px;opacity:0.01;pointer-events:none';
  document.body.append(element);
  return element;
}
