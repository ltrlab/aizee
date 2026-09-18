"""Aizee compatibility boundary around the shared session worker.

The collector retains URDF calibration and its tracking/state machine. Messages
arrive here already in motor coordinates; the shared backend owns routing,
freshness checks and every command/telemetry socket.
"""
from __future__ import annotations

import json
import os
import threading
import time

from robot_platform.control import RobotSession, ThreadedRobotClient
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings


class AizeeSessionTransport:
    def __init__(self, context, *, command: str, telemetry: str) -> None:
        spec = load_robot("aizee")
        overrides = {"endpoints": {"arms": {"main": {
            "command": command, "telemetry": telemetry,
        }}}}
        # Endpoint arguments already reflect launcher settings or legacy host
        # discovery. The launcher's freshness policy must travel with them.
        resolved = os.environ.get("ROBOT_PLATFORM_SETTINGS_JSON")
        if resolved:
            settings = json.loads(resolved)
            overrides["control"] = settings.get("control", {})
        self.session = RobotSession(spec, load_settings(spec, overrides=overrides))
        self.client = ThreadedRobotClient(self.session, context=context)
        self.client.start()

    def send_message(self, message: dict) -> None:
        self.client.submit({"main": message})

    def close(self) -> None:
        self.client.close()


def _refresh_telem_cache(client, lock, cache) -> None:
    """Mirror a sample once; repeated reads never refresh its receive time."""
    message, received_at = client.telemetry_sample("main")
    if received_at is None:
        return
    with lock:
        if received_at == cache.get("received_at"):
            return
        cache["msg"] = message
        cache["received_at"] = received_at
        # Old display/tracking code uses wall time. Shared command gating uses
        # the original monotonic time, even if the machine's wall clock changes.
        cache["time"] = time.time() - max(0.0, time.monotonic() - received_at)


def start_telem_cache(client):
    """Expose the historical lock/cache interface without another SUB socket."""
    lock = threading.Lock()
    cache = {"msg": None, "time": 0.0, "received_at": None}
    stop = threading.Event()

    def run():
        while not stop.is_set():
            _refresh_telem_cache(client, lock, cache)
            stop.wait(0.005)

    thread = threading.Thread(target=run, daemon=True, name="AizeeTelemView")
    thread.start()
    return stop, thread, lock, cache
