// WebSocket wrappers for the three host endpoints.
//
// All three reconnect with exponential backoff up to 5 s.  The state
// callback (onState) gets one of: 'disconnected' | 'connecting' |
// 'connected' | 'error' — used by the lobby diagnostics + in-VR HUD.

const WS_BASE = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}`;

class _BaseSocket {
  constructor(path) {
    this.path = path;
    this.ws = null;
    this.backoff = 250;
    this.onState = () => {};
  }
  _setState(s) { this.onState(s); }
  _scheduleReconnect(open) {
    setTimeout(() => open(), this.backoff);
    this.backoff = Math.min(this.backoff * 2, 5000);
  }
}

export class ControlSocket extends _BaseSocket {
  constructor() { super('/ws/control'); }
  connect() {
    this._setState('connecting');
    const ws = new WebSocket(WS_BASE + this.path);
    ws.onopen = () => { this.backoff = 250; this._setState('connected'); };
    ws.onclose = () => { this._setState('disconnected'); this._scheduleReconnect(() => this.connect()); };
    ws.onerror = () => { this._setState('error'); };
    this.ws = ws;
  }
  send(frame) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    this.ws.send(JSON.stringify(frame));
  }
}

export class TelemSocket extends _BaseSocket {
  constructor() { super('/ws/telem'); this.onTelem = null; }
  connect(onTelem) {
    this.onTelem = onTelem ?? this.onTelem;
    this._setState('connecting');
    const ws = new WebSocket(WS_BASE + this.path);
    ws.onopen = () => { this.backoff = 250; this._setState('connected'); };
    ws.onmessage = (e) => {
      try {
        const telem = JSON.parse(e.data);
        if (this.onTelem) this.onTelem(telem);
      } catch {}
    };
    ws.onclose = () => { this._setState('disconnected'); this._scheduleReconnect(() => this.connect()); };
    ws.onerror = () => { this._setState('error'); };
    this.ws = ws;
  }
}
