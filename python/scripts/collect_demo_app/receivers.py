"""Background telemetry / camera / e-stop receiver threads (from collect_demo.py)."""
from __future__ import annotations

import json
import threading
import time
from typing import Optional

import zmq

try:
    import serial as _serial
    _pyserial_available = True
except ImportError:
    _pyserial_available = False

# Timeout-bounded serial open (never blocks forever on an unresponsive device
# such as a Bluetooth serial port).  Shared with the leader-arm probes.
try:
    from serial_safe import open_serial as _open_serial
except ImportError:
    _open_serial = None

from common.wire import unpack_camera, unpack_msg

# ---------------------------------------------------------------------------
# Background telemetry receiver
# ---------------------------------------------------------------------------
# Mirrors the camera-receiver pattern: keeps json.loads of fat telem packets
# (multi-motor state) off the main loop.  Caches the latest message; main
# loop reads it under a lock.

def _start_telem_receiver(
    ctx: zmq.Context,
    endpoint: str,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    lock  = threading.Lock()
    cache: dict = {"msg": None, "time": 0.0}
    stop  = threading.Event()

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
                        cache["msg"]  = msg
                        cache["time"] = time.time()
        finally:
            sock.close()

    thread = threading.Thread(target=_run, daemon=True, name="TelemRx")
    thread.start()
    return stop, thread, lock, cache


# ---------------------------------------------------------------------------
# Background camera / UPS receiver
# ---------------------------------------------------------------------------
# Runs in its own thread so that JSON-parsing large camera frames never
# blocks the motor-command loop.  Caches the latest message per source;
# the main loop reads cached values under a lock.

def _start_cam_receiver(
    ctx: zmq.Context,
    gripper_ep: str,
    ups_ep: Optional[str],
    scene_ep: Optional[str] = None,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    lock = threading.Lock()
    cache: dict = {
        "gripper": None, "gripper_time": 0.0, "gripper_ts": None,
        "scene":   None, "scene_time":   0.0, "scene_ts":   None,
        "ups": None,
    }
    stop = threading.Event()

    def _run() -> None:
        # NOTE: zmq.CONFLATE is incompatible with multi-frame messages.  After
        # the camera channel switched to multipart (header msgpack + raw JPEG),
        # CONFLATE on the SUB triggered libzmq's `!_more` assertion in fq.cpp
        # the first time a new frame arrived mid-receive.  We drop CONFLATE
        # and instead set a small RCVHWM and drain to the latest message
        # explicitly inside the recv block.
        gripper_sock = ctx.socket(zmq.SUB)
        gripper_sock.setsockopt(zmq.LINGER, 0)
        gripper_sock.setsockopt(zmq.RCVHWM, 2)
        gripper_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        gripper_sock.connect(gripper_ep)

        scene_sock: Optional[zmq.Socket] = None
        if scene_ep:
            # Same multipart drain pattern as the gripper cam — the scene cam
            # publisher (camera_node.py) also uses pack_camera() multipart.
            scene_sock = ctx.socket(zmq.SUB)
            scene_sock.setsockopt(zmq.LINGER, 0)
            scene_sock.setsockopt(zmq.RCVHWM, 2)
            scene_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            scene_sock.connect(scene_ep)

        ups_sock: Optional[zmq.Socket] = None
        if ups_ep:
            ups_sock = ctx.socket(zmq.SUB)
            ups_sock.setsockopt(zmq.LINGER, 0)
            ups_sock.setsockopt(zmq.CONFLATE, 1)
            ups_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            ups_sock.connect(ups_ep)

        poller = zmq.Poller()
        poller.register(gripper_sock, zmq.POLLIN)
        if scene_sock:
            poller.register(scene_sock, zmq.POLLIN)
        if ups_sock:
            poller.register(ups_sock, zmq.POLLIN)

        try:
            while not stop.is_set():
                try:
                    # 30 ms ≈ one main-loop tick; matches LOOP_HZ so a stop
                    # request or backlog is noticed within ~one frame.
                    events = dict(poller.poll(timeout=30))
                except zmq.ZMQError:
                    break
                now = time.time()

                if gripper_sock in events:
                    # Drain to the newest available frame (CONFLATE no longer
                    # applies; old frames would otherwise queue up to RCVHWM=2).
                    latest = None
                    while True:
                        try:
                            latest = unpack_camera(gripper_sock.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                        except Exception:
                            latest = None
                            break
                    if latest is not None:
                        with lock:
                            cache["gripper"] = latest
                            cache["gripper_time"] = now
                            ts = latest.get("timestamp")
                            if ts is not None:
                                cache["gripper_ts"] = float(ts)

                if scene_sock is not None and scene_sock in events:
                    latest = None
                    while True:
                        try:
                            latest = unpack_camera(scene_sock.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                        except Exception:
                            latest = None
                            break
                    if latest is not None:
                        with lock:
                            cache["scene"] = latest
                            cache["scene_time"] = now
                            ts = latest.get("timestamp")
                            if ts is not None:
                                cache["scene_ts"] = float(ts)

                if ups_sock and ups_sock in events:
                    try:
                        msg = unpack_msg(ups_sock.recv(zmq.NOBLOCK))
                        with lock:
                            cache["ups"] = msg
                    except Exception:
                        pass
        finally:
            gripper_sock.close()
            if scene_sock is not None:
                scene_sock.close()
            if ups_sock:
                ups_sock.close()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread, lock, cache


# ---------------------------------------------------------------------------
# Background e-stop serial reader
# ---------------------------------------------------------------------------
# Reads JSON lines from the ESP32 e-stop receiver over serial.
# Sets/clears a threading.Event so the main loop can gate motor commands.

def _start_estop_reader(
    port: str,
    stop: threading.Event,
    flag: threading.Event,
) -> Optional[threading.Thread]:
    if not _pyserial_available:
        print("WARNING: pyserial not installed — hardware e-stop disabled")
        return None

    def _run() -> None:
        ser = None
        while not stop.is_set():
            if ser is None:
                try:
                    if _open_serial is not None:
                        ser = _open_serial(port, 115200, read_timeout=1)
                    else:
                        ser = _serial.Serial(port, 115200, timeout=1)
                    print(f"E-stop receiver connected on {port}")
                except (_serial.SerialException, OSError, TimeoutError):
                    stop.wait(2)
                    continue
            try:
                raw = ser.readline()
            except _serial.SerialException:
                print(f"E-stop serial error, reconnecting {port}...")
                ser = None
                stop.wait(1)
                continue
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            estop = data.get("estop")
            if estop is not None:
                if estop:
                    flag.set()
                else:
                    flag.clear()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t
