"""WebXR / Quest teleop HTTP(S) bridge.

Serves the in-headset client to the Quest Pro browser and exposes
WebSocket endpoints for:
  * /ws/control  — controller pose + buttons (browser -> host)
  * /ws/cam      — gripper-camera JPEG frames (host -> browser, binary)
  * /ws/telem    — qpos / state telemetry (host -> browser, JSON)

Runnable standalone for development:
    python -m web.server --bind 0.0.0.0 --port 8443

Or embedded by collect_demo.py via `start_server_in_thread(state)`.
"""

from .server import (
    QuestWebServer,
    SharedState,
    start_server_in_thread,
)

__all__ = ["QuestWebServer", "SharedState", "start_server_in_thread"]
