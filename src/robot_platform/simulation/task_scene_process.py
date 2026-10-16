"""Owned scene subprocess and scene-specific adapter over the shared capture mailbox."""
from __future__ import annotations
from pathlib import Path
import sys
import threading
from robot_platform.data.capture_process import ProcessCapture, _read, _write

class TaskSceneView(ProcessCapture):
    """Scene-specific constructor over the shared bounded capture process mailbox."""
    def __init__(self, scene, joint_map, names, *, directory, metadata=None, seed=1,
                 viewer=True, rate_hz=30., record_hz=20., width=320, height=240,
                 camera_roles=("right_wrist", "head"), visual_mode="collision_proxy"):
        if rate_hz <= 0 or record_hz <= 0 or record_hz > rate_hz:
            raise ValueError("rates must satisfy 0 < record_hz <= rate_hz")
        if visual_mode not in {"collision_proxy", "cad"}:
            raise ValueError("visual_mode must be collision_proxy or cad")
        roles = ("left_wrist", "right_wrist", "head")
        if isinstance(camera_roles, str) or not camera_roles or len(set(camera_roles)) != len(camera_roles) or not set(camera_roles).issubset(roles):
            raise ValueError("capture camera roles must be a distinct supported nonempty subset")
        self.directory, self.camera_roles = Path(directory), tuple(camera_roles)
        config = dict(scene=str(scene), joint_map=joint_map, names=list(names), directory=str(directory),
                      metadata=metadata or {}, seed=seed, viewer=viewer, rate_hz=rate_hz,
                      record_hz=record_hz, width=width, height=height,
                      camera_roles=list(camera_roles), visual_mode=visual_mode)
        self.camera_roles = tuple(camera_roles)
        self._record_intent = False
        super().__init__(config, worker_module=__name__)

    def command(self, action, payload=None):
        if action == "start_record":
            self._record_intent = True
        try:
            return super().command(action, payload)
        except Exception:
            if action == "start_record": self._record_intent = False
            raise

    def show(self, q, qcmd=None, source_time=None, authority="human", command_valid=True):
        with self._lock:
            command = self._state.get("last_command") or {}
            if ((command.get("action") == "stop_record" and command.get("state") == "complete") or
                    (command.get("action") == "start_record" and command.get("state") == "failed")):
                self._record_intent = False
            active = self._record_intent or (self._state.get("recording") or {}).get("active", False)
        if command_valid is not True and active:
            self.abort("receiver lost valid armed commands during recording")
            return
        self.submit((tuple(q), None if qcmd is None else tuple(qcmd), source_time, str(authority), command_valid is True),
                    source_time=source_time)

def _worker():
    from .task_scene import _ThreadTaskSceneView
    view = None
    stopped = threading.Event()
    ids = {}
    def publish():
        known_capabilities = None
        try:
            while not stopped.is_set():
                state = view.snapshot()
                capabilities = state.get("capabilities")
                if capabilities == known_capabilities:
                    state.pop("capabilities", None)
                else:
                    known_capabilities = capabilities
                # Full event history stays in the episode; UI gets a bounded tail.
                task = state.get("task") or {}
                if "events" in task:
                    task["event_count"] = len(task["events"])
                    task["events"] = task["events"][-16:]
                command = state.get("last_command")
                if command and command.get("id") in ids:
                    command["id"] = ids[command["id"]]
                _write(sys.stdout.buffer, state)
                stopped.wait(.1)
        except (BrokenPipeError, OSError, ValueError):
            view.abort("parent status pipe closed")
            stopped.set()
    publisher = None
    try:
        first = _read(sys.stdin.buffer)
        config = first["config"]
        view = _ThreadTaskSceneView(**config)
        publisher = threading.Thread(target=publish, daemon=True, name="task-scene-publish")
        publisher.start()
        while not stopped.is_set():
            packet = _read(sys.stdin.buffer)
            if packet.get("close"):
                break
            if packet.get("abort"):
                view.abort(packet["abort"])
                break
            sample = packet.get("sample")
            if sample is not None:
                view.show(*sample)
            for identity, action, payload in packet.get("commands", []):
                accepted = view.command(action, payload)
                ids[accepted["id"]] = identity
                if len(ids) > 32:
                    del ids[next(iter(ids))]
    except (EOFError, OSError, ValueError, KeyError, RuntimeError) as exc:
        if view is not None:
            view.abort(f"parent connection ended: {exc}")
    finally:
        if view is not None:
            view.close()
            stopped.set()
            if publisher is not None:
                publisher.join(timeout=1)
            try:
                _write(sys.stdout.buffer, view.snapshot())
            except (OSError, ValueError):
                pass


if __name__ == "__main__":
    _worker()
