"""Existing collector UI backed only by the guarded virtual receiver.

The leader workbench remains the single USB owner. This adapter neither opens
serial ports nor constructs the legacy physical follower/camera transports.
"""
from __future__ import annotations

import queue
import sys
import threading
import time
from pathlib import Path


LIFECYCLE_KEYS = frozenset({"I", "E", "T", "H", "X"})
PERIOD_S = .04
MODE_MESSAGE = "VIRTUAL RECEIVER · live GELLO input · no physical motor connection"
UNSUPPORTED = "Unavailable in guarded shadow: recording, calibration, jog, routing and motor tuning. Use I / E / T / H / X."


def empty_snapshot():
    return {
        "state": "DISABLED", "qpos": [float("nan")] * 17,
        "target": [float("nan")] * 17, "leader": [float("nan")] * 17,
        "temp": None, "torque": None, "recording": False,
        "rec_steps": 0, "dropped": 0, "present": {"left": False, "right": False},
        "leaders": {"left": False, "right": False}, "both_ok": False,
        "robot_ok": False, "telem_age": None, "cam_ages": {},
        "jetson": "VIRTUAL", "host": None, "message": MODE_MESSAGE,
        "guarded_shadow": True,
    }


class ShadowGuiAdapter:
    """Translate existing UI keys without adding another control state machine."""
    swapped = False

    def __init__(self, runtime=None, *, fault=None):
        self.runtime = runtime
        self.snapshot = empty_snapshot()
        self.fault = None
        if fault:
            self._fail(str(fault))

    def _fail(self, message):
        self.fault = message
        self.snapshot.update(state="FAULT", message=f"FAULT · {message} · last positions retained; quit and restart explicitly.",
                             present={"left": False, "right": False}, both_ok=False,
                             robot_ok=False, telem_age=None)
        self.close()

    def tick(self, key=None):
        if self.fault or self.runtime is None:
            return dict(self.snapshot)
        unsupported = key is not None and key not in LIFECYCLE_KEYS
        try:
            result = self.runtime.tick(None if unsupported else key)
            if not isinstance(result, dict):
                raise ValueError("Guarded runtime did not return a GUI snapshot")
            self.snapshot.update(result)
            self.snapshot["guarded_shadow"] = True
            if getattr(self.runtime, "fault", None):
                self._fail(str(self.runtime.fault))
            elif str(self.snapshot.get("state", "")).upper() == "FAULT":
                self._fail(str(self.snapshot.get("message") or "Guarded runtime faulted"))
            elif unsupported:
                self.snapshot["message"] = UNSUPPORTED
        except Exception as exc:
            self._fail(str(exc) or type(exc).__name__)
        return dict(self.snapshot)

    def toggle_swap(self):
        return None

    def jog(self, *_):
        return None

    def close(self):
        runtime, self.runtime = self.runtime, None
        if runtime is not None:
            try:
                runtime.close()
            except Exception as exc:
                message = f"Virtual receiver close failed: {exc}"
                if self.fault:
                    message = f"{self.fault}; {message}"
                self.fault = self.fault or message
                self.snapshot.update(state="FAULT", message=message)


def _terminal_commands(commands, stream):
    """Line-based terminal entry uses the same keys as the existing Qt buttons."""
    for line in stream:
        key = line.strip().upper()
        if key:
            try:
                commands.put_nowait(key)
            except queue.Full:
                pass
        if key == "Q":
            return
    # Closing the controlling input should close the virtual session as well.
    try:
        commands.put_nowait("Q")
    except queue.Full:
        pass


def run_guarded(args):
    """Run the original GUI/keyboard lifecycle against an isolated virtual robot."""
    from robot_platform.teleop.shadow_runtime import ShadowTeleopRuntime

    commands = queue.Queue(maxsize=32)
    renderer = None
    adapter = None
    print(MODE_MESSAGE, flush=True)
    print("Idle (I) → Enable (E) → Engage / Align (T); Shutdown (X) returns slowly to zero. H disables; Q quits.", flush=True)
    try:
        try:
            runtime = ShadowTeleopRuntime(
                Path(args.model), Path(args.leader_display_calibration), leader_url=args.leader_url)
            adapter = ShadowGuiAdapter(runtime)
        except Exception as exc:
            adapter = ShadowGuiAdapter(fault=str(exc) or type(exc).__name__)

        if args.gui:
            from collect_minerva_gui import QtRenderer
            renderer = QtRenderer(
                cmd_queue=commands,
                meta={"guarded_shadow": True, "language_instruction": "", "mode_message": MODE_MESSAGE},
                teleop=adapter, cameras=[], output_dir=Path(args.output_dir),
                params={"guarded_shadow": True, "grav_ok": False, "grip_ff": False})
            renderer.start()
        else:
            print("Enter one key followed by Enter. Ctrl+C also quits.", flush=True)
            threading.Thread(target=_terminal_commands, args=(commands, sys.stdin),
                             daemon=True, name="GuardedShadowKeys").start()

        previous_status = None
        while True:
            started = time.perf_counter()
            if renderer is not None and renderer.should_quit():
                break
            try:
                key = commands.get_nowait()
            except queue.Empty:
                key = None
            if key == "Q":
                break
            snapshot = adapter.tick(key)
            if renderer is not None:
                with renderer.lock:
                    renderer.holder["args"] = snapshot
            status = (snapshot.get("state"), snapshot.get("message"))
            if status != previous_status:
                print(f"[{status[0]}] {status[1] or ''}", flush=True)
                previous_status = status
            time.sleep(max(0., PERIOD_S - (time.perf_counter() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if adapter is not None:
            adapter.close()
        if renderer is not None:
            renderer.request_quit()
            renderer.join(timeout=3.0)
    return 1 if adapter is not None and adapter.fault else 0
