"""Bounded latest-sample process mailbox shared by simulation and live capture.

Only I/O threads serialize or write pipes. Camera images stay in the worker.
"""
from __future__ import annotations
import copy
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
import uuid

MAX_PACKET = 1024 * 1024
MAX_COMMANDS = 16
STATUS_TIMEOUT_S = .75
HEARTBEAT_S = .1
STDERR_LIMIT = 64 * 1024


def _write(stream, packet):
    body = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()
    if len(body) > MAX_PACKET:
        raise ValueError("capture IPC packet exceeds bound")
    pending = memoryview(struct.pack("<I", len(body)) + body)
    while pending:
        count = stream.write(pending)
        if count is None or count <= 0:
            raise OSError("capture IPC write failed")
        pending = pending[count:]
    stream.flush()


def _exact(stream, count):
    result = bytearray()
    while len(result) < count:
        chunk = stream.read(count-len(result))
        if not chunk:
            break
        result.extend(chunk)
    return bytes(result)


def _read(stream):
    header = _exact(stream, 4)
    if not header:
        raise EOFError("capture IPC closed")
    if len(header) != 4:
        raise ValueError("truncated capture IPC header")
    count, = struct.unpack("<I", header)
    if not 0 < count <= MAX_PACKET:
        raise ValueError("invalid capture IPC packet size")
    data = _exact(stream, count)
    if len(data) != count:
        raise ValueError("truncated capture IPC packet")
    def invalid_constant(value):
        raise ValueError(f"invalid JSON constant: {value}")
    packet = json.loads(data, parse_constant=invalid_constant)
    if not isinstance(packet, dict):
        raise ValueError("capture IPC packet must be an object")
    return packet


def _status(packet):
    """Validate small status fields before using child data in the control callback."""
    for key in ("available", "capture_ready", "failed"):
        if key in packet and type(packet[key]) is not bool:
            raise ValueError(f"capture status {key} must be boolean")
    for key in ("error",):
        if packet.get(key) is not None and not isinstance(packet[key], str):
            raise ValueError(f"capture status {key} must be text")
    deadline = packet.get('camera_valid_until')
    if deadline is not None and (type(deadline) not in (int, float) or not math.isfinite(deadline)):
        raise ValueError('capture camera freshness deadline must be finite')
    for key in ("recording", "capabilities", "task", "cameras"):
        if key in packet and not isinstance(packet[key], dict):
            raise ValueError(f"capture status {key} must be an object")
    recording = packet.get("recording") or {}
    for key in ("active", "complete"):
        if recording.get(key) is not None and type(recording[key]) is not bool:
            raise ValueError(f"capture recording {key} must be boolean")
    if packet.get("last_command") is not None and not isinstance(packet["last_command"], dict):
        raise ValueError("capture command status must be an object")
    return packet


class ProcessCapture:
    """Nonblocking latest-sample/command mailbox over one owned scene process."""
    def __init__(self, config, *, worker_module, source_timeout_s=.25, source_clock=None,
                 startup_timeout_s=30.):
        if not 0 < source_timeout_s <= .25 or not 0 < startup_timeout_s <= 60:
            raise ValueError("capture source/startup timeout is outside its bound")
        self._source_timeout_s, self._source_clock = source_timeout_s, source_clock
        self._startup_timeout_s = startup_timeout_s
        self.directory = Path(config["directory"])
        self.directory.mkdir(parents=True, exist_ok=True)
        self.stderr_path = self.directory / f"capture-worker-{uuid.uuid4().hex}.stderr.log"
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = threading.Event()
        self._closing = False
        self._latest, self._sequence, self._commands = None, 0, []
        self._source_changed_wall = None
        self._latest_stamp = None
        self._received = time.monotonic()
        self._started = self._received
        self._failure = None
        self._have_status = False
        self._was_available = False
        self._pending_command = None
        self._state = dict(available=False, capture_ready=False, error=None, failed=False,
                           capabilities={}, task={}, recording={"active": False},
                           cameras={"ready": False, "timestamps": {}}, last_command=None,
                           worker_stderr=str(self.stderr_path))
        flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        self._process = subprocess.Popen([sys.executable, "-m", worker_module, "--worker"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            creationflags=flags, bufsize=0)
        self._sender = threading.Thread(target=self._send, args=(config,), daemon=True, name="capture-send")
        self._reader = threading.Thread(target=self._receive, daemon=True, name="capture-status")
        self._stderr_reader = threading.Thread(target=self._stderr, daemon=True, name="capture-stderr")
        self._stderr_reader.start()
        self._reader.start(); self._sender.start()

    def _stderr(self):
        """Keep a bounded diagnostic prefix while draining excess output so the child never blocks."""
        written = 0
        try:
            with self.stderr_path.open('wb') as log:
                while True:
                    data = self._process.stderr.read(4096)
                    if not data:
                        return
                    if written < STDERR_LIMIT:
                        chunk = data[:STDERR_LIMIT-written]
                        log.write(chunk)
                        written += len(chunk)
                        if written == STDERR_LIMIT:
                            log.write(b'\n[stderr log truncated; remaining output drained]\n')
                        log.flush()
        except (OSError, ValueError):
            pass

    def submit(self, sample, *, source_time=None):
        """Replace the latest immutable/plain-data sample; caller owns its snapshot."""
        with self._lock:
            if self._closing or self._failure:
                return
            if self._latest is None or source_time != self._latest_stamp:
                self._source_changed_wall = time.monotonic()
            self._latest_stamp = source_time
            self._latest = sample
            self._sequence += 1
        self._wake.set()

    def command(self, action, payload=None):
        if action not in {"reset", "start_record", "stop_record"}:
            raise ValueError("unknown capture command")
        identity = uuid.uuid4().hex
        with self._lock:
            if self._closing or self._failure:
                raise RuntimeError(self._failure or "capture is closed")
            if len(self._commands) >= MAX_COMMANDS:
                raise RuntimeError("capture command queue is full")
            self._commands.append((identity, action, dict(payload or {})))
            self._pending_command = {"id": identity, "action": action, "state": "pending"}
            self._state = dict(self._state, last_command=self._pending_command)
        self._wake.set()
        return {"accepted": True, "id": identity}

    def _fail(self, reason):
        with self._lock:
            if self._failure is None:
                self._failure = str(reason)
            recording = dict(self._state.get("recording") or {})
            if recording.get("active"):
                recording.update(active=False, complete=False, state="failed", error=self._failure)
            self._state = dict(self._state, available=False, capture_ready=False, failed=True,
                               error=self._failure, recording=recording)
        self._wake.set()

    def abort(self, reason):
        """Latch failure and notify the child without waiting for any pipe or disk I/O."""
        self._fail(reason)

    def _send(self, config):
        sent, last_sent = -1, 0.
        try:
            _write(self._process.stdin, {"config": config})
            while not self._closed.is_set():
                self._wake.wait(.02); self._wake.clear()
                with self._lock:
                    closing = self._closing
                    commands, self._commands = self._commands, []
                    sequence, sample = self._sequence, self._latest
                    now = time.monotonic()
                    stale = self._have_status and now-self._received > STATUS_TIMEOUT_S
                    starting = not self._was_available and now-self._started > self._startup_timeout_s
                if not closing and (stale or starting):
                    self._fail("capture status stalled" if stale else "capture worker startup timed out")
                if closing:
                    _write(self._process.stdin, {"close": True})
                    return
                if self._failure:
                    _write(self._process.stdin, {"abort": self._failure})
                    return
                if commands or (sample is not None and sequence != sent):
                    _write(self._process.stdin, {"sample": sample, "commands": commands})
                    sent, last_sent = sequence, time.monotonic()
                elif time.monotonic()-last_sent >= HEARTBEAT_S:
                    _write(self._process.stdin, {"heartbeat": True})
                    last_sent = time.monotonic()
        except (OSError, EOFError, ValueError, TypeError) as exc:
            if not self._closing:
                self._fail(f"capture transport failed: {exc}")

    def _receive(self):
        try:
            while not self._closed.is_set():
                state = _status(_read(self._process.stdout))
                with self._lock:
                    # Capabilities arrive only when changed; small status updates are otherwise sufficient.
                    pending = self._pending_command
                    incoming = state.get("last_command") or {}
                    if pending and incoming.get("id") != pending["id"]:
                        state["last_command"] = pending
                    elif pending and incoming.get("state") != "pending":
                        self._pending_command = None
                    if self._failure is None:
                        self._state = dict(self._state, **state)
                    self._have_status = True
                    self._was_available = self._was_available or state.get('available') is True
                    self._received = time.monotonic()
        except (OSError, EOFError, ValueError) as exc:
            if not self._closing:
                self._fail(f"capture worker ended: {exc}")

    def health(self):
        now = time.monotonic()
        with self._lock:
            stale = self._have_status and now-self._received > STATUS_TIMEOUT_S
            starting = not self._was_available and now-self._started > self._startup_timeout_s
            age = None if self._source_changed_wall is None else now-self._source_changed_wall
            if self._source_clock is not None:
                stamp = self._latest_stamp
                age = (self._source_clock()-stamp if type(stamp) in (int, float)
                       and math.isfinite(stamp) else None)
            state, failure = self._state, self._failure
            recording = state.get("recording") or {}
            failed = bool(failure or state.get("failed") or stale or starting)
            return {"available": bool(state["available"] and not failed and not self._closing),
                    "error": failure or ("capture status stalled" if stale else
                                         "capture worker startup timed out" if starting else state.get("error")),
                    "failed": failed, "source_age_s": age,
                    "capture_ready": bool(state["capture_ready"] and age is not None and 0 <= age <= self._source_timeout_s and not failed and not self._closing),
                    "recording": {key: recording.get(key) for key in ("active", "error", "state", "complete")}}

    def snapshot(self):
        with self._lock:
            state = self._state
        result = copy.deepcopy(state)
        health = self.health()
        result.update({k:v for k,v in health.items() if k != "recording"})
        return result

    def close(self):
        if self._closed.is_set():
            return
        with self._lock:
            self._closing = True
        self._wake.set()
        forced = False
        try:
            self._process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            forced = True
            self._process.terminate()
            try:
                self._process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._process.kill(); self._process.wait(timeout=2)
        self._closed.set()
        self._sender.join(timeout=1); self._reader.join(timeout=1)
        self._stderr_reader.join(timeout=1)
        for stream in (self._process.stdin, self._process.stdout, self._process.stderr):
            stream.close()
        if forced:
            self._fail("capture worker shutdown timed out; unfinished episode remains partial")
        with self._lock:
            self._state = dict(self._state, available=False, capture_ready=False)

