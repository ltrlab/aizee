"""Shared live-camera relay, independent of settings/motion and browser speed."""
from collections import deque
import json
import math
from pathlib import Path
import shlex
import struct
import threading
import time

from .camera_video_worker import MAX_HEADER, MAX_JPEG, ROLES

STALE_AFTER = 1.


class PacketDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets = []
        while len(self.buffer) >= 8:
            header_size, jpeg_size = struct.unpack('!II', self.buffer[:8])
            if not 1 <= header_size <= MAX_HEADER or jpeg_size > MAX_JPEG:
                raise ValueError('Camera relay packet exceeds bound')
            size = 8 + header_size + jpeg_size
            if len(self.buffer) < size:
                break
            header = json.loads(self.buffer[8:8+header_size])
            jpeg = bytes(self.buffer[8+header_size:size])
            del self.buffer[:size]
            packets.append((header, jpeg))
        return packets


def browser_packet(header, jpeg):
    encoded = json.dumps(header, allow_nan=False, separators=(',', ':')).encode()
    return struct.pack('!I', len(encoded)) + encoded + jpeg


class CameraVideo:
    """One worker per Helm process, latest-only JPEG references, no image decode."""
    def __init__(self, profile, *, address, open_ssh=None, clock=time.monotonic):
        self.profile, self.address, self.open_ssh, self.clock = Path(profile), address, open_ssh, clock
        self._lock = threading.Lock()
        self._viewers, self._frames, self._errors = set(), {}, {}
        self._sequences = {role:0 for role in ROLES}
        self._heartbeats = deque(maxlen=32)
        self._heartbeat_sequence = 0
        self._thread = self._client = self._channel = None
        self._stop = threading.Event()
        self._closed = False
        self._error = 'Waiting for a viewer'

    def subscribe(self):
        with self._lock:
            if self._closed:
                raise RuntimeError('Camera video is closed')
            token = object()
            self._viewers.add(token)
            if self._thread is None:
                self._start_locked()
            return token

    def _start_locked(self):
        self._stop = threading.Event()
        self._error = 'Connecting to camera video'
        self._thread = threading.Thread(target=self._run, args=(self._stop,), name='helm-camera-video', daemon=True)
        self._thread.start()

    def unsubscribe(self, token):
        with self._lock:
            self._viewers.discard(token)
            if self._viewers:
                return
            self._stop.set()
            self._frames.clear()
            self._error = 'No camera viewers'
            client, channel, thread = self._client, self._channel, self._thread
        if channel is not None:
            channel.close()
        if client is not None:
            client.close()
        if thread is not None:
            thread.join(5.)

    def close(self):
        with self._lock:
            self._closed = True
            self._viewers.clear()
        self.unsubscribe(None)

    def snapshot(self):
        now = self.clock()
        with self._lock:
            cameras, frames = {}, {}
            for role in ROLES:
                frame = self._frames.get(role)
                age = frame['age_at_receive'] + now-frame['received_at'] if frame else None
                fresh = frame is not None and age < STALE_AFTER and self._error is None and not self._errors.get(role)
                cameras[role] = {'fresh':fresh, 'age_s':age, 'sequence':self._sequences[role],
                    'error':self._error or self._errors.get(role) or (None if fresh else 'Waiting for a fresh camera frame')}
                if fresh:
                    header = {'role':role, 'sequence':frame['sequence'], 'age_s':age,
                              'source_timestamp':frame['source_timestamp']}
                    frames[role] = (header, frame['jpeg'])
            return {'type':'status', 'cameras':cameras, 'target_fps':10, 'stale_after_s':STALE_AFTER}, frames

    def _accept(self, header, jpeg):
        now = self.clock()
        if not isinstance(header, dict):
            raise ValueError('Malformed relay header')
        token = header.get('heartbeat')
        with self._lock:
            sent = dict(self._heartbeats).get(token) if type(token) is int else None
            if sent is None:
                raise ValueError('Relay heartbeat is unknown or too old')
            transit_bound = now-sent
            if transit_bound < 0 or transit_bound >= STALE_AFTER:
                self._frames.clear()
                self._error = 'Camera transport is stale'
                return
            kind = header.get('kind')
            if kind == 'status' and not jpeg:
                errors = header.get('errors')
                if not isinstance(errors, dict) or set(errors)-set(ROLES):
                    raise ValueError('Malformed camera status')
                self._errors = {role: str(value)[:200] for role, value in errors.items() if value}
                self._error = None
                return
            if kind != 'frame' or header.get('role') not in ROLES:
                raise ValueError('Unexpected relay frame role')
            role = header['role']
            age, stamp = header.get('source_age_s'), header.get('source_timestamp')
            if any(type(value) not in (int,float) or not math.isfinite(value) for value in (age, stamp)) or age < 0:
                raise ValueError('Invalid camera freshness metadata')
            if not 4 <= len(jpeg) <= MAX_JPEG or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                raise ValueError('Invalid relay JPEG envelope')
            bound = age + transit_bound
            if bound >= STALE_AFTER:
                self._frames.pop(role, None)
                self._errors[role] = 'Camera frame is stale'
                return
            self._sequences[role] += 1
            self._frames[role] = {'jpeg':jpeg, 'source_timestamp':stamp, 'age_at_receive':bound,
                                 'received_at':now, 'sequence':self._sequences[role]}
            self._errors.pop(role, None)
            self._error = None

    def _heartbeat(self, stdin, channel, stop):
        try:
            while not stop.is_set():
                with self._lock:
                    self._heartbeat_sequence += 1
                    token = self._heartbeat_sequence
                    self._heartbeats.append((token, self.clock()))
                stdin.write(json.dumps({'heartbeat':token}) + '\n')
                stdin.flush()
                stop.wait(.2)
        except Exception:
            channel.close()

    def _pump(self, channel, stop):
        decoder, progress, stderr = PacketDecoder(), self.clock(), bytearray()
        while not stop.is_set():
            if channel.recv_stderr_ready():
                chunk = channel.recv_stderr(4096)
                stderr.extend(chunk[:max(0,4096-len(stderr))])
            if channel.recv_ready():
                chunk = channel.recv(65536)
                if not chunk:
                    raise EOFError('Camera relay closed')
                for header, jpeg in decoder.feed(chunk):
                    self._accept(header, jpeg)
                    progress = self.clock()
            elif channel.exit_status_ready():
                raise EOFError('Camera relay exited: ' + stderr.decode(errors='replace'))
            elif self.clock()-progress > 2.:
                raise TimeoutError('Camera relay stopped responding')
            else:
                stop.wait(.01)

    def _run(self, stop):
        from robot_platform.ops.ssh import connect, open_command
        delay = .5
        try:
            while not stop.is_set():
                client = channel = sender = None
                connection_stop = threading.Event()
                try:
                    client = (self.open_ssh or connect)(self.profile, address=self.address(), timeout=3.)
                    with self._lock:
                        self._client = client
                    if stop.is_set():
                        break
                    source = (Path(__file__).with_name('camera_video_worker.py')).read_text(encoding='utf-8')
                    stdin, out, _err = open_command(client, 'python3 -u -c ' + shlex.quote(source), timeout=2.)
                    channel = out.channel
                    channel.settimeout(2.)
                    with self._lock:
                        self._channel = channel
                        self._frames.clear()
                        self._heartbeats.clear()
                    sender = threading.Thread(target=self._heartbeat, args=(stdin, channel, connection_stop), daemon=True)
                    sender.start()
                    self._pump(channel, stop)
                    delay = .5
                except Exception as exc:
                    with self._lock:
                        self._frames.clear()
                        self._error = str(exc)[:500]
                finally:
                    connection_stop.set()
                    if channel is not None:
                        channel.close()
                    if client is not None:
                        client.close()
                    if sender is not None:
                        sender.join(2.5)
                    with self._lock:
                        self._client = self._channel = None
                if not stop.is_set():
                    stop.wait(delay)
                    delay = min(4., delay*2)
        finally:
            with self._lock:
                self._frames.clear()
                self._thread = None
                if self._viewers and not self._closed:
                    self._start_locked()
