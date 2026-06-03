"""aiohttp-based HTTPS server for the Quest WebXR teleop client.

Routes:
  GET  /                        -> static/index.html
  GET  /static/*                -> static/*
  GET  /meshes/*                -> URDF meshes for the in-headset URDF mirror
  WS   /ws/control              -> controller pose + buttons (browser -> host)
  WS   /ws/cam                  -> gripper-camera JPEG frames (host -> browser)
  WS   /ws/telem                -> qpos + state telemetry (host -> browser)

`SharedState` is the in-process bridge that the QuestLeader and the
camera/telemetry pumps write to and that the WebSocket handlers read
from / write to.  Single source of truth — no locking needed for the
single-writer/single-reader assignments because Python attribute writes
on built-ins are atomic, and we never mutate buffers in place.
"""

from __future__ import annotations

import asyncio
import json
import ssl
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

try:
    from aiohttp import WSMsgType, web
    _AIOHTTP_AVAILABLE = True
except ImportError:
    _AIOHTTP_AVAILABLE = False

from .cert import ensure_self_signed, _local_ipv4s
from .webrtc import WebRTCBridge


def _dt_now_iso() -> str:
    import datetime as _dt
    return _dt.datetime.now(_dt.timezone.utc).isoformat()


# -----------------------------------------------------------------------------
# Shared state — the seam between QuestLeader / pumps and the server
# -----------------------------------------------------------------------------

from collections import deque


@dataclass
class SharedState:
    """Latest-only snapshots; readers always see the newest frame.

    `latest_control`: dict with {ts, right:{pos,quat,trigger,grip,a,b},
                                  left:{pos,quat,grip,a,stick}, head:{pos,quat}}
        Written by the /ws/control handler on every incoming frame.
        Read by the QuestLeader background thread.

    `latest_cam_jpeg`: bytes — most recent JPEG frame from gripper camera.
        Written by the camera bridge.
        Read by the WebRTC JpegCameraTrack, decoded once per frame and
        re-encoded into the browser's negotiated video codec.

    `latest_telem`: dict with {ts, qpos:[...], state:str, recording:bool, ...}
        Written by the telemetry bridge.
        Read & broadcast by the /ws/telem handler.
    """
    latest_control: Optional[dict] = None
    latest_cam_jpeg: Optional[bytes] = None
    latest_cam_seq: int = 0
    latest_telem: Optional[dict] = None
    latest_telem_seq: int = 0
    # Discrete operator commands issued from the in-VR UI (or fake_quest).
    # QuestLeader.poll() drains this on every tick.  deque.append /
    # deque.popleft are atomic in CPython, so no extra lock is needed.
    pending_commands: deque = field(default_factory=deque)
    # Stats — useful for the HUD and for debugging dropped frames
    stats: dict = field(default_factory=lambda: {
        "control_rx_hz": 0.0,
        "telem_tx_hz": 0.0,
        "webrtc_peers": 0,
    })


# -----------------------------------------------------------------------------
# Server
# -----------------------------------------------------------------------------

_STATIC_DIR = Path(__file__).resolve().parent / "static"
_URDF_DIR = Path(__file__).resolve().parents[2] / "urdf" / "aizee"
_MESH_DIR = _URDF_DIR / "meshes"
_URDF_FILE = _URDF_DIR / "aizee.urdf"
_JOINT_LIMITS_YAML = Path(__file__).resolve().parents[2] / "config" / "joint_limits.yaml"
_JOINT_ALIGN_JSON = Path(__file__).resolve().parents[2] / "config" / "joint_align.json"

# Lazy: only built when /api/collision_check is first hit (1.5 s mesh load
# would otherwise slow down operator-mode startup that doesn't need it).
_mesh_world_singleton = None
_mesh_world_lock = None


def _get_mesh_world():
    """Lazy construct + cache a MeshWorld for the collision-check API."""
    global _mesh_world_singleton, _mesh_world_lock
    if _mesh_world_lock is None:
        import threading as _th
        _mesh_world_lock = _th.Lock()
    with _mesh_world_lock:
        if _mesh_world_singleton is None:
            import sys as _sys
            _sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
            from ik.mesh_world import MeshWorld  # noqa: WPS433
            _mesh_world_singleton = MeshWorld(_URDF_FILE, use_convex=True)
    return _mesh_world_singleton


class QuestWebServer:
    def __init__(
        self,
        state: SharedState,
        *,
        bind: str = "0.0.0.0",
        port: int = 8443,
        cert_dir: Optional[Path] = None,
        telem_hz: float = 30.0,
    ) -> None:
        if not _AIOHTTP_AVAILABLE:
            raise RuntimeError(
                "aiohttp is required — install via: pip install aiohttp"
            )
        self.state = state
        self.bind = bind
        self.port = port
        self.cert_dir = cert_dir
        self.telem_hz = telem_hz
        self._runner: Optional[web.AppRunner] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        # Built inside _serve so it lives on the server's event loop —
        # aiortc binds its async primitives to the loop at construction time.
        self._rtc: Optional[WebRTCBridge] = None

    # ---- routes --------------------------------------------------------

    def _build_app(self) -> web.Application:
        app = web.Application(client_max_size=4 * 1024 * 1024)
        app.router.add_get("/", self._index)
        app.router.add_get("/preview", self._preview)
        app.router.add_static("/static/", path=str(_STATIC_DIR), show_index=False)
        app.router.add_get("/aizee.urdf", self._urdf)
        app.router.add_get("/api/joint_limits", self._joint_limits)
        app.router.add_post("/api/joint_limits", self._save_joint_limits)
        app.router.add_get("/api/joint_align", self._joint_align)
        app.router.add_post("/api/joint_align", self._save_joint_align)
        app.router.add_post("/api/collision_check", self._collision_check)
        app.router.add_post("/api/quest_command", self._quest_command)
        if _MESH_DIR.exists():
            app.router.add_static("/meshes/", path=str(_MESH_DIR), show_index=False)
        app.router.add_get("/ws/control", self._ws_control)
        app.router.add_get("/ws/telem", self._ws_telem)
        app.router.add_post("/api/webrtc/offer", self._webrtc_offer)
        return app

    async def _index(self, _req: web.Request) -> web.Response:
        return web.FileResponse(_STATIC_DIR / "index.html")

    async def _preview(self, _req: web.Request) -> web.Response:
        return web.FileResponse(_STATIC_DIR / "preview.html")

    async def _urdf(self, _req: web.Request) -> web.Response:
        if not _URDF_FILE.exists():
            return web.Response(status=404, text="aizee.urdf not found")
        return web.FileResponse(_URDF_FILE, headers={"Content-Type": "application/xml"})

    async def _joint_limits(self, _req: web.Request) -> web.Response:
        if not _JOINT_LIMITS_YAML.exists():
            return web.json_response(
                {"error": "joint_limits.yaml not found — run python -m ik.collision_sweep"},
                status=404,
            )
        import yaml as _yaml
        data = _yaml.safe_load(_JOINT_LIMITS_YAML.read_text()) or {}
        return web.json_response(data)

    async def _save_joint_limits(self, req: web.Request) -> web.Response:
        """Patch the effective_lower/effective_upper of selected joints in
        joint_limits.yaml.  Preserves all other sweep metadata (urdf bounds,
        colliding pairs, etc.) so subsequent sweeps + manual edits coexist.
        """
        try:
            payload = await req.json()
        except Exception:
            return web.json_response({"error": "invalid JSON"}, status=400)
        updates = payload.get("joints") or {}
        if not isinstance(updates, dict):
            return web.json_response({"error": "expected {joints: {...}}"}, status=400)
        import yaml as _yaml
        if _JOINT_LIMITS_YAML.exists():
            data = _yaml.safe_load(_JOINT_LIMITS_YAML.read_text()) or {}
        else:
            data = {"joints": {}}
        joints = data.setdefault("joints", {})
        changed: list[str] = []
        for name, patch in updates.items():
            if not isinstance(patch, dict):
                continue
            entry = joints.setdefault(name, {"joint": name})
            for k in ("effective_lower", "effective_upper"):
                if k in patch:
                    entry[k] = float(patch[k])
                    changed.append(name)
            # Recompute reduction_pct if we have both bounds + URDF bounds.
            if (entry.get("urdf_lower") is not None
                    and entry.get("urdf_upper") is not None
                    and entry.get("effective_lower") is not None
                    and entry.get("effective_upper") is not None):
                urdf_span = entry["urdf_upper"] - entry["urdf_lower"]
                eff_span  = entry["effective_upper"] - entry["effective_lower"]
                if urdf_span > 0:
                    entry["reduction_pct"] = float(100.0 * (1.0 - eff_span / urdf_span))
            # Tag the entry as hand-edited so a future sweep can decide
            # whether to overwrite or preserve.
            entry["manually_edited"] = True
        data["last_manual_edit"] = _dt_now_iso()
        _JOINT_LIMITS_YAML.parent.mkdir(parents=True, exist_ok=True)
        _JOINT_LIMITS_YAML.write_text(_yaml.safe_dump(data, sort_keys=False))
        return web.json_response({"ok": True, "updated": sorted(set(changed))})

    async def _joint_align(self, _req: web.Request) -> web.Response:
        """Return per-joint visual alignment (7 offsets + 7 signs).

        Applied by collect_demo at the motor↔URDF boundary:
            q_urdf  = q_motor * sign + offset
            q_motor = (q_urdf - offset) * sign   (sign is ±1)
        Velocities and torques get the sign flip too; the offset is a
        position-only constant.  Defaults: offsets=0, signs=+1."""
        import json as _json
        offsets = [0.0] * 7
        signs   = [1.0] * 7
        if _JOINT_ALIGN_JSON.exists():
            try:
                data = _json.loads(_JOINT_ALIGN_JSON.read_text())
                o = data.get("offsets")
                if isinstance(o, list) and len(o) >= 7:
                    offsets = [float(x) for x in o[:7]]
                s = data.get("signs")
                if isinstance(s, list) and len(s) >= 7:
                    signs = [1.0 if float(x) >= 0 else -1.0 for x in s[:7]]
            except Exception:
                pass
        return web.json_response({"offsets": offsets, "signs": signs})

    async def _save_joint_align(self, req: web.Request) -> web.Response:
        """Persist per-joint alignment (offsets + signs) to joint_align.json."""
        import json as _json
        try:
            payload = await req.json()
        except Exception:
            return web.json_response({"error": "invalid JSON"}, status=400)
        offsets = payload.get("offsets")
        signs   = payload.get("signs")
        if not isinstance(offsets, list) or len(offsets) < 7:
            return web.json_response({"error": "expected offsets[7]"}, status=400)
        # signs is optional for backwards compat; default to all +1
        if signs is None:
            signs = [1.0] * 7
        if not isinstance(signs, list) or len(signs) < 7:
            return web.json_response({"error": "expected signs[7]"}, status=400)
        _JOINT_ALIGN_JSON.parent.mkdir(parents=True, exist_ok=True)
        _JOINT_ALIGN_JSON.write_text(_json.dumps({
            "offsets": [float(x) for x in offsets[:7]],
            "signs":   [1 if float(x) >= 0 else -1 for x in signs[:7]],
        }, indent=2))
        return web.json_response({"ok": True})

    async def _quest_command(self, req: web.Request) -> web.Response:
        """Queue a discrete operator command for QuestLeader to process on
        its next poll() tick.  Used by the in-VR UI buttons (realign,
        grow_workspace, shrink_workspace, center_workspace_on_ee, ...)."""
        try:
            payload = await req.json()
        except Exception:
            return web.json_response({"error": "invalid JSON"}, status=400)
        cmd = payload.get("cmd")
        if not isinstance(cmd, str):
            return web.json_response({"error": "missing 'cmd' string"}, status=400)
        self.state.pending_commands.append(payload)
        return web.json_response({"ok": True, "queued": cmd})

    async def _collision_check(self, req: web.Request) -> web.Response:
        try:
            payload = await req.json()
        except Exception:
            return web.json_response({"error": "invalid JSON"}, status=400)
        qpos = payload.get("qpos") or {}
        # Run the FCL check on the event loop's default executor so the
        # ~150 us call doesn't block other handlers if many requests stack.
        import asyncio as _aio
        loop = _aio.get_running_loop()
        def _do_check():
            world = _get_mesh_world()
            world.set_qpos({k: float(v) for k, v in qpos.items()})
            pairs = world.colliding_pairs()
            return [sorted(list(p)) for p in pairs]
        try:
            pairs = await loop.run_in_executor(None, _do_check)
        except Exception as e:
            return web.json_response({"error": str(e)}, status=500)
        return web.json_response({"collision": bool(pairs), "pairs": pairs})

    # ---- WS: control (browser -> host) ---------------------------------

    async def _ws_control(self, req: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=2.0, max_msg_size=64 * 1024)
        await ws.prepare(req)
        # Per-connection RX rate counter (decays toward 0 if the client
        # stops sending so the HUD can show "stale").
        rx_count = 0
        rx_window_start = time.time()
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        frame = json.loads(msg.data)
                    except Exception:
                        continue
                    frame["_rx_ts"] = time.time()
                    self.state.latest_control = frame
                    rx_count += 1
                    if rx_count >= 30:
                        now = time.time()
                        dt = now - rx_window_start
                        if dt > 0:
                            self.state.stats["control_rx_hz"] = rx_count / dt
                        rx_count = 0
                        rx_window_start = now
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            await ws.close()
        return ws

    # ---- WebRTC: SDP offer/answer for the camera video track ------------

    async def _webrtc_offer(self, req: web.Request) -> web.Response:
        if self._rtc is None:
            return web.json_response(
                {"error": "WebRTC bridge not initialized"}, status=503,
            )
        try:
            payload = await req.json()
        except Exception:
            return web.json_response({"error": "invalid JSON"}, status=400)
        if "sdp" not in payload or "type" not in payload:
            return web.json_response(
                {"error": "expected {sdp, type}"}, status=400,
            )
        answer = await self._rtc.handle_offer(payload)
        self.state.stats["webrtc_peers"] = self._rtc.active_peer_count
        return web.json_response(answer)

    # ---- WS: telemetry (host -> browser, JSON) -------------------------

    async def _ws_telem(self, req: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(req)
        sent_seq = -1
        period = 1.0 / max(self.telem_hz, 1.0)
        tx_count = 0
        tx_window = time.time()
        try:
            while not ws.closed:
                telem = self.state.latest_telem
                seq = self.state.latest_telem_seq
                if telem is not None and seq != sent_seq:
                    await ws.send_str(json.dumps(telem))
                    sent_seq = seq
                    tx_count += 1
                    if tx_count >= 30:
                        now = time.time()
                        self.state.stats["telem_tx_hz"] = tx_count / (now - tx_window)
                        tx_count = 0
                        tx_window = now
                await asyncio.sleep(period)
        finally:
            await ws.close()
        return ws

    # ---- lifecycle -----------------------------------------------------

    async def _serve(self) -> None:
        # WebRTC bridge has to be constructed on the loop that will service
        # its peer connections (aiortc binds async primitives at construction).
        self._rtc = WebRTCBridge(self.state)
        app = self._build_app()
        app.on_shutdown.append(self._on_shutdown)
        cert_path, key_path, fingerprint = ensure_self_signed(self.cert_dir)
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(certfile=str(cert_path), keyfile=str(key_path))
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        site = web.TCPSite(self._runner, host=self.bind, port=self.port, ssl_context=ssl_ctx)
        await site.start()
        print(f"[quest-web] HTTPS listening on https://{self.bind}:{self.port}", flush=True)
        # Helpful: explicit per-IP URLs operator can type on the Quest browser.
        # When binding 0.0.0.0 the LAN IP is what actually matters.
        for ip in _local_ipv4s():
            if ip == "127.0.0.1" and self.bind != "127.0.0.1":
                continue
            print(f"[quest-web]   on Quest, open: https://{ip}:{self.port}", flush=True)
        print(f"[quest-web] cert fingerprint (SHA-256): {fingerprint}", flush=True)
        print(f"[quest-web] cert files: {cert_path}  /  {key_path}", flush=True)
        # Idle forever until the run loop is cancelled by stop().
        while True:
            await asyncio.sleep(3600)

    async def _on_shutdown(self, _app: web.Application) -> None:
        if self._rtc is not None:
            await self._rtc.close_all()

    def run_blocking(self) -> None:
        """Run the server on the current thread, blocking until KeyboardInterrupt."""
        try:
            asyncio.run(self._serve())
        except KeyboardInterrupt:
            pass


def start_server_in_thread(
    state: SharedState,
    *,
    bind: str = "0.0.0.0",
    port: int = 8443,
    cert_dir: Optional[Path] = None,
) -> threading.Thread:
    """Start the server in a daemon thread; returns the thread handle.

    Used by collect_demo.py to bring the WebXR endpoint up alongside the
    main teleop loop without managing a second process.
    """
    srv = QuestWebServer(state, bind=bind, port=port, cert_dir=cert_dir)

    def _entry() -> None:
        srv.run_blocking()

    t = threading.Thread(target=_entry, name="quest-web", daemon=True)
    t.start()
    return t


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def _cli() -> None:
    import argparse
    ap = argparse.ArgumentParser(description="AIZEE Quest WebXR server (standalone)")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--cert-dir", type=Path, default=None)
    args = ap.parse_args()
    state = SharedState()
    # Phase-1 standalone: log every Nth control frame so the dev sees motion.
    def _logger() -> None:
        last = -1.0
        while True:
            frame = state.latest_control
            if frame is not None:
                ts = frame.get("ts", 0.0)
                if ts != last:
                    last = ts
                    right = frame.get("right", {})
                    p = right.get("pos")
                    print(f"[ctrl] t={ts:.3f}  right.pos={p}  "
                          f"trig={right.get('trigger')}  grip={right.get('grip')}",
                          flush=True)
            time.sleep(0.25)
    threading.Thread(target=_logger, daemon=True).start()
    QuestWebServer(state, bind=args.bind, port=args.port, cert_dir=args.cert_dir).run_blocking()


if __name__ == "__main__":
    _cli()
