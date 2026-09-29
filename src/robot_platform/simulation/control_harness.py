"""MessagePack transport to the deterministic native controller harness.

This executable has no hardware backend, sockets or independent wall-clock
scheduler. Each step advances receiver time explicitly, including silent-client
ticks. It is a protocol/fault harness, not a physical dynamics simulation.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
from queue import Empty, Queue
import struct
import subprocess
from threading import Thread

from robot_platform.transport.wire import pack_msg, unpack_msg


def default_binary() -> Path:
    if os.environ.get("ROBOT_CONTROL_HARNESS"):
        return Path(os.environ["ROBOT_CONTROL_HARNESS"]).resolve()
    root = Path(__file__).resolve().parents[3]
    suffix = ".exe" if os.name == "nt" else ""
    return root / "rust" / "target" / "debug" / f"control_harness{suffix}"


class NativeControlHarness:
    def __init__(self, config: dict, initial_positions: list[float], *, binary=None, timeout_s=3.0,
                 setup_json=None):
        """`setup_json`: a path to write the setup to as JSON and have the harness read it from, as the
        live receiver reads its configuration (it parses floats from text), instead of MessagePack."""
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        self.timeout_s = timeout_s
        self.binary = Path(binary) if binary is not None else default_binary()
        if not self.binary.is_file():
            raise FileNotFoundError(f"Build the native harness first: cargo build -p robot_control --bin control_harness ({self.binary})")
        setup = {"config": config, "initial_positions": initial_positions}
        if setup_json is not None:
            import json
            Path(setup_json).write_text(json.dumps(setup, indent=1, allow_nan=False), encoding="utf-8")
            argument, setup = str(setup_json), None
        else:
            argument, setup = "-", pack_msg(setup)
            if len(setup) > 256 * 1024:
                raise ValueError("Controller setup exceeds 256 KiB")
        self.process = subprocess.Popen([str(self.binary), argument], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._responses = Queue()
        self._reader = Thread(target=self._read_replies, daemon=True, name="controller-harness-reader")
        self._reader.start()
        try:
            if setup is not None:
                self.process.stdin.write(struct.pack("<I", len(setup)) + setup)
                self.process.stdin.flush()
        except BaseException:
            self.close()
            raise
        self.now_ms = 0
        self.last = None

    def _read_replies(self):
        try:
            while True:
                header = self.process.stdout.read(4)
                if len(header) != 4:
                    raise RuntimeError("Native controller harness ended before replying")
                length, = struct.unpack("<I", header)
                if not 0 < length <= 256 * 1024:
                    raise RuntimeError("Invalid native harness response size")
                body = self.process.stdout.read(length)
                if len(body) != length:
                    raise RuntimeError("Truncated native harness reply")
                self._responses.put(unpack_msg(body))
        except Exception as exc:
            self._responses.put(exc)

    def step(self, now_ms: int, request: dict | None = None, *, faults=None, feedback=None) -> dict:
        """Advance the receiver to `now_ms`, handling `request` if any. `feedback` ([{position, velocity,
        effort}] per joint) is what the next read reports, as a simulated plant measured it; without it the
        joints follow their commands exactly."""
        if type(now_ms) is not int or now_ms < 0:
            raise ValueError("now_ms must be a nonnegative integer in the receiver clock")
        self.now_ms = now_ms
        frame = {"protocol_version": 1, "now_ms": now_ms, "request": request, "faults": faults}
        if feedback is not None:
            frame["feedback"] = feedback
        frame = pack_msg(frame)
        if len(frame) > 256 * 1024:
            raise ValueError("Controller harness frame exceeds 256 KiB")
        self.process.stdin.write(struct.pack("<I", len(frame)) + frame)
        self.process.stdin.flush()
        try:
            result = self._responses.get(timeout=self.timeout_s)
        except Empty as exc:
            self.process.kill()
            raise TimeoutError("Native controller harness response timed out") from exc
        if isinstance(result, Exception):
            raise result
        self.last = result
        if result["error"]:
            raise ValueError(result["error"])
        return result

    def exchange(self, request: dict) -> dict:
        return self.step(self.now_ms, request)["reply"]

    def telemetry(self) -> dict:
        """Map actual harness samples to the shared session telemetry contract."""
        if self.last is None:
            self.step(self.now_ms)
        status = self.last["status"]
        return {"telemetry_schema_version": 2, "producer_id": status["epoch"],
                "timestamp": status["now_ms"] / 1000.0,
                "emergency_stop": status["reason"] == "emergency_stop",
                "motors": {name: {"position": feedback["position"], "velocity": feedback["velocity"],
                                  "torque": feedback["effort"], "temperature": feedback["temperature"],
                                  "state": status["state"], "mode": "simulation", "error": None,
                                  "feedback": {"sequence": feedback["sequence"],
                                               "age_s": (status["now_ms"] - feedback["sampled_at_ms"]) / 1000.0,
                                               "valid": feedback["sequence"] > 0 and not feedback["fault"],
                                               "provenance": "simulation"}}
                           for name, feedback in zip(status["joint_names"], self.last["feedback"])}}

    def close(self):
        try:
            if self.process.stdin and not self.process.stdin.closed:
                self.process.stdin.close()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        finally:
            self._reader.join(timeout=2)
            for pipe in (self.process.stdout, self.process.stderr):
                pipe.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
