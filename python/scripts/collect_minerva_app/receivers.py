"""receivers.py — background telemetry + multi-camera receiver threads.

Generalizes AIZEE's collect_demo_app/receivers.py from a fixed
gripper(+scene) pair to an arbitrary dict of named camera streams (Minerva:
left_wrist / right_wrist / head). Same discipline: each SUB socket is drained
to its newest multipart frame inside a poller loop; the latest message per
source is cached under a lock; the main loop reads the cache.

Each starter returns (stop_event, thread, lock, cache).
"""

from __future__ import annotations

import threading
import time
from typing import Dict, Tuple

import zmq

from common.wire import unpack_camera, unpack_msg


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------

def start_telem_receiver(
    ctx: zmq.Context, endpoint: str,
) -> Tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    lock = threading.Lock()
    cache: dict = {"msg": None, "time": 0.0}
    stop = threading.Event()

    def _run() -> None:
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.connect(endpoint)
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        try:
            while not stop.is_set():
                try:
                    events = dict(poller.poll(timeout=30))
                except zmq.ZMQError:
                    break
                if sock in events:
                    try:
                        msg = unpack_msg(sock.recv(zmq.NOBLOCK))
                    except Exception:
                        continue
                    with lock:
                        cache["msg"] = msg
                        cache["time"] = time.time()
        finally:
            sock.close()

    thread = threading.Thread(target=_run, daemon=True, name="TelemRx")
    thread.start()
    return stop, thread, lock, cache


# ---------------------------------------------------------------------------
# Cameras (N named streams)
# ---------------------------------------------------------------------------

def start_cam_receiver(
    ctx: zmq.Context, cam_endpoints: Dict[str, str],
) -> Tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    """One thread owning a SUB socket per camera, all in one poller.

    Cache keys per camera <name>: <name> (latest unpacked msg),
    <name>_time (host recv time), <name>_ts (publisher timestamp).

    NOTE: multipart (header + raw JPEG) is incompatible with zmq.CONFLATE — it
    triggers libzmq's `!_more` assertion — so we use RCVHWM=2 and drain to the
    newest frame explicitly (mirrors AIZEE's receivers.py:93-99).
    """
    lock = threading.Lock()
    cache: dict = {}
    for name in cam_endpoints:
        cache[name] = None
        cache[f"{name}_time"] = 0.0
        cache[f"{name}_ts"] = None
    stop = threading.Event()

    def _run() -> None:
        socks: Dict[str, zmq.Socket] = {}
        poller = zmq.Poller()
        for name, ep in cam_endpoints.items():
            s = ctx.socket(zmq.SUB)
            s.setsockopt(zmq.LINGER, 0)
            s.setsockopt(zmq.RCVHWM, 2)
            s.setsockopt_string(zmq.SUBSCRIBE, "")
            s.connect(ep)
            socks[name] = s
            poller.register(s, zmq.POLLIN)
        try:
            while not stop.is_set():
                try:
                    events = dict(poller.poll(timeout=30))
                except zmq.ZMQError:
                    break
                now = time.time()
                for name, s in socks.items():
                    if s not in events:
                        continue
                    latest = None
                    while True:
                        try:
                            latest = unpack_camera(s.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                        except Exception:
                            latest = None
                            break
                    if latest is not None:
                        with lock:
                            cache[name] = latest
                            cache[f"{name}_time"] = now
                            ts = latest.get("timestamp")
                            if ts is not None:
                                cache[f"{name}_ts"] = float(ts)
        finally:
            for s in socks.values():
                s.close()

    thread = threading.Thread(target=_run, daemon=True, name="CamRx")
    thread.start()
    return stop, thread, lock, cache


__all__ = ["start_telem_receiver", "start_cam_receiver"]
