"""Bounded recording I/O outside the motor-control thread.

Writes and flushes only enqueue immutable operations. One worker owns every file
and preserves their shared FIFO order. Overflow or worker failure is sticky and
must fault the caller; no operation is silently dropped. ``close`` is the only
blocking API and must be called only after the motors have been disabled.
"""
from collections import deque
import math
from pathlib import Path
import threading


class RecordingError(RuntimeError):
    """Recording cannot be confirmed complete; control must stop safely."""


def _open_file(path, binary):
    path.parent.mkdir(parents=True, exist_ok=True)
    return path.open('ab') if binary else path.open('a', encoding='utf-8')


class RecordingStream:
    """A producer handle; it never owns or calls the underlying file."""

    def __init__(self, writer, name, binary):
        self._writer, self.name, self.binary = writer, name, binary
        self._closed = False

    def write(self, data):
        if self.binary:
            if not isinstance(data, (bytes, bytearray, memoryview)):
                raise TypeError('Binary recording writes require bytes-like data')
            size = data.nbytes if isinstance(data, memoryview) else len(data)
            self._writer._check_chunk(size)
            chunk = bytes(data)  # Snapshot mutable buffers before handing them off.
            count = len(chunk)
        else:
            if not isinstance(data, str):
                raise TypeError('Text recording writes require str')
            # A Python string uses at most four bytes per character. Reserving
            # that upper bound avoids encoding on the control thread and also
            # bounds non-ASCII strings, independently of the eventual file bytes.
            size = 4 * len(data)
            self._writer._check_chunk(size)
            chunk, count = data, len(data)
        self._writer._enqueue('write', self, chunk, size)
        return count

    def flush(self):
        self._writer._enqueue('flush', self)

    def close(self):
        """Enqueue this stream's close; only writer.close waits for completion."""
        self._writer._enqueue('close', self)

    @property
    def closed(self):
        with self._writer._ready:
            return self._closed or self._writer._closing


class RecordingWriter:
    """One bounded FIFO for all recording streams, including in-flight I/O.

    ``opener(path, binary)`` is injectable for deterministic I/O fault tests.
    It, and every operation on its returned handles, run only on the worker.
    Byte reservations count binary bytes or a conservative four bytes per text
    character; per-operation overhead is bounded separately by max_chunks.
    """

    def __init__(self, directory, *, max_bytes=8 * 1024 * 1024, max_chunks=2048, opener=None):
        if (type(max_bytes) is not int or max_bytes <= 0
                or type(max_chunks) is not int or max_chunks <= 0):
            raise ValueError('Positive integer recording byte and chunk bounds required')
        self.directory = Path(directory)
        self._max_bytes, self._max_chunks = max_bytes, max_chunks
        self._opener = opener or _open_file
        self._ready = threading.Condition(threading.Lock())
        self._queue, self._streams = deque(), {}
        self._bytes = self._chunks = self._peak_bytes = self._peak_chunks = 0
        self._error = None
        self._closing = self._closed = False
        self._done = threading.Event()
        self._worker = threading.Thread(target=self._run, name='helm-recording', daemon=True)
        self._worker.start()

    def stream(self, name, *, binary=False):
        if (not isinstance(name, str) or not name or Path(name).name != name
                or '/' in name or '\\' in name or name in ('.', '..')):
            raise ValueError('Recording stream must use a file basename')
        if type(binary) is not bool:
            raise ValueError('Recording binary mode must be boolean')
        with self._ready:
            self._check_locked()
            if name in self._streams:
                raise ValueError(f'Recording stream already registered: {name}')
            stream = RecordingStream(self, name, binary)
            self._enqueue_locked('open', stream, None, 0)
            self._streams[name] = stream
            return stream

    def check(self):
        """Surface worker failure without waiting for any I/O."""
        with self._ready:
            if self._error is not None:
                raise RecordingError(self._error)

    def snapshot(self):
        with self._ready:
            return dict(pending_bytes=self._bytes, pending_chunks=self._chunks,
                        peak_bytes=self._peak_bytes, peak_chunks=self._peak_chunks,
                        closing=self._closing, closed=self._closed, error=self._error)

    def _check_locked(self):
        if self._error is not None:
            raise RecordingError(self._error)
        if self._closing:
            raise RecordingError('Recording is closing; operation was not accepted')

    def _fail_locked(self, message):
        # Accepted data still drains after overflow. A failed file operation,
        # however, cannot safely be retried (it may have written a prefix).
        if self._error is None:
            self._error = message
        self._closing = True
        self._ready.notify_all()

    def _check_chunk(self, size):
        with self._ready:
            self._check_locked()
            if size > self._max_bytes:
                self._fail_locked(f'Recording queue overflow: chunk needs {size} bytes, limit '
                                  f'{self._max_bytes}; operation was not accepted')
                raise RecordingError(self._error)

    def _enqueue(self, operation, stream, data=None, size=0):
        with self._ready:
            if operation == 'close' and stream._closed:
                return
            self._check_locked()
            if stream._closed:
                raise RecordingError(f'Recording stream is closed: {stream.name}')
            self._enqueue_locked(operation, stream, data, size)
            if operation == 'close':
                stream._closed = True

    def _enqueue_locked(self, operation, stream, data, size):
        if self._chunks >= self._max_chunks or self._bytes + size > self._max_bytes:
            self._fail_locked(f'Recording queue overflow: {self._chunks} chunks / {self._bytes} bytes '
                              f'pending; bounds {self._max_chunks} chunks / {self._max_bytes} bytes; '
                              'operation was not accepted')
            raise RecordingError(self._error)
        self._queue.append((operation, stream, data, size))
        self._chunks += 1
        self._bytes += size
        self._peak_chunks = max(self._peak_chunks, self._chunks)
        self._peak_bytes = max(self._peak_bytes, self._bytes)
        self._ready.notify()

    def _run(self):
        handles = {}
        try:
            while True:
                with self._ready:
                    self._ready.wait_for(lambda: self._queue or self._closing)
                    if not self._queue:
                        break
                    operation, stream, data, size = self._queue.popleft()
                try:
                    if operation == 'open':
                        handles[stream.name] = self._opener(self.directory / stream.name, stream.binary)
                    elif operation == 'write':
                        written = handles[stream.name].write(data)
                        if written != len(data):
                            raise OSError(f'Short recording write: {written!r} of {len(data)}')
                    elif operation == 'flush':
                        handles[stream.name].flush()
                    elif operation == 'close':
                        handle = handles[stream.name]
                        handle.close()
                        del handles[stream.name]
                except Exception as exc:
                    with self._ready:
                        pending = len(self._queue)
                        self._fail_locked(f'Recording {operation} failed for {stream.name}: '
                                          f'{type(exc).__name__}: {exc}; {pending} subsequent operations '
                                          'were not written')
                        # The failure is retained and raised by check/write/close.
                        # Never retry a possibly partial write or retain a hidden backlog.
                        self._queue.clear()
                        self._bytes, self._chunks = size, 1
                    break
                finally:
                    with self._ready:
                        self._bytes -= size
                        self._chunks -= 1
        finally:
            for name, handle in handles.items():
                try:
                    handle.close()  # File.close flushes; cleanup remains on this worker.
                except Exception as exc:
                    with self._ready:
                        self._fail_locked(f'Recording close failed for {name}: {type(exc).__name__}: {exc}')
            with self._ready:
                self._closing = self._closed = True
            self._done.set()

    def close(self, *, timeout=5.):
        """Drain/close after motor disable; never call this from a powered cycle.

        A blocked OS write cannot be safely interrupted. On timeout the daemon
        remains responsible for its handles, and this call reports incomplete
        recording instead of blocking shutdown indefinitely.
        """
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout < 0:
            raise ValueError('Finite nonnegative recording drain timeout required')
        with self._ready:
            self._closing = True
            self._ready.notify_all()
        self._worker.join(timeout)
        if self._worker.is_alive():
            with self._ready:
                self._fail_locked(f'Recording drain timed out after {timeout:g}s; '
                                  f'{self._chunks} operations / {self._bytes} bytes still pending')
            raise RecordingError(self._error)
        self.check()
