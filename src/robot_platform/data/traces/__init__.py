"""Versioned event streams with original timestamps and bounded disk I/O."""
from __future__ import annotations

import json
from pathlib import Path
import queue
import threading
import time


class TraceWriter:
    """One writer thread owns HDF5. Producers never wait for disk.

    JSON event payloads preserve asynchronous streams and raw bytes as hex.
    Overflow is counted and makes the trace unsuitable for latency validation.
    """
    def __init__(self, path, *, clock_domain, max_pending=8192, clock=time.monotonic):
        if type(max_pending) is not int or max_pending < 1:
            raise ValueError("max_pending must be a positive integer")
        self.path = Path(path)
        self.clock_domain = clock_domain
        self._clock = clock
        self.queue = queue.Queue(max_pending)
        self.dropped = 0
        self.error = None
        self.closed = False
        self.sequence = 0
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._write, name="SimulationTrace", daemon=True)
        self._thread.start()
        if not self._ready.wait(5):
            self._stop.set()
            raise RuntimeError("trace writer startup timed out")
        if self.error:
            raise RuntimeError("cannot create trace") from self.error

    def record(self, kind, payload, *, time_s=None):
        with self._lock:
            if self.closed:
                raise RuntimeError("trace writer is closed")
            if self.error:
                raise RuntimeError("trace writer failed") from self.error
            self.sequence += 1
            event = json.dumps({"schema_version": 1, "sequence": self.sequence,
                                "clock_domain": self.clock_domain, "time_s": self._clock() if time_s is None else time_s,
                                "kind": kind, "payload": payload}, allow_nan=False)
            try:
                self.queue.put_nowait(event)
            except queue.Full:
                self.dropped += 1

    def _write(self):
        try:
            import h5py
            with h5py.File(self.path, "x") as file:
                file.attrs["schema_version"] = 1
                file.attrs["clock_domain"] = self.clock_domain
                events = file.create_dataset("events", (0,), maxshape=(None,), dtype=h5py.string_dtype("utf-8"), chunks=True)
                self._ready.set()
                while not self._stop.is_set() or not self.queue.empty():
                    try:
                        first = self.queue.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    batch = [first]
                    while len(batch) < 256:
                        try:
                            batch.append(self.queue.get_nowait())
                        except queue.Empty:
                            break
                    offset = len(events)
                    events.resize((offset + len(batch),))
                    events[offset:] = batch
                file.attrs["dropped_events"] = self.dropped
                file.attrs["complete"] = True
        except Exception as exc:
            self.error = exc
        finally:
            self._ready.set()

    def close(self):
        with self._lock:
            self.closed = True
            self._stop.set()
        self._thread.join(5)
        if self._thread.is_alive():
            raise RuntimeError("trace writer shutdown timed out")
        if self.error:
            raise RuntimeError("trace write failed") from self.error


def read_events(path):
    import h5py
    with h5py.File(path, "r") as file:
        if file.attrs.get("schema_version") != 1:
            raise ValueError("unsupported trace schema")
        if not file.attrs.get("complete") or file.attrs.get("dropped_events", 0):
            raise ValueError("trace is incomplete or has dropped events")
        for event in file["events"].asstr():
            yield json.loads(event)
