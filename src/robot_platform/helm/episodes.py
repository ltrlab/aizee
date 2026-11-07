"""Helm's recorded-episode library, the server side of Replay.

One worker (helm.episode_worker) reads the episodes where they were recorded: on Minerva's Jetson over the pinned
SSH connection (its source is sent, as the camera relay's is), or in a local directory. This side keeps one worker
connection, answers the API from caches (episode summaries and series, and recent frames, fetched for every camera
at once) and asks the worker nothing while `busy()` names a reason, such as a running recording: an episode already
loaded keeps playing from the cache.
"""
from collections import OrderedDict
import queue
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

WORKER = Path(__file__).with_name('episode_worker.py')
MAX_REPLY = 64 * 1024 * 1024
FRAME_CACHE = 1500  # Every camera of a frame is one entry: ~70 KB at 320x240, about 100 MB at most.
EPISODE_CACHE = 8
IDLE_CLOSE_S = 120.


class EpisodeError(RuntimeError):
    """The worker could not answer: unreachable robot, missing file, malformed episode."""


class EpisodeBusy(EpisodeError):
    """Replay is waiting: `busy()` named a reason, and the answer was not cached."""


class _Connection:
    """One running worker: a request goes out, its reply comes back through the reader thread's queue."""

    def __init__(self, stdin, stdout, closer):
        self.stdin, self.stdout, self.closer = stdin, stdout, closer
        self.replies = queue.Queue()
        self.ids = 0
        self.reader = threading.Thread(target=self._read, daemon=True, name='helm-episode-reader')
        self.reader.start()

    def _read(self):
        import msgpack
        try:
            while True:
                header = self._exactly(4)
                size = int.from_bytes(header, 'big')
                if not 0 < size <= MAX_REPLY:
                    raise EpisodeError(f'A {size}-byte reply exceeds the limit')
                self.replies.put(msgpack.unpackb(self._exactly(size), raw=False))
        except Exception as exc:  # noqa: BLE001 - the end of the stream is delivered as the next reply.
            self.replies.put(EpisodeError(f'The episode reader stopped: {exc}' if str(exc) else
                                          'The episode reader stopped'))

    def _exactly(self, size):
        data = b''
        while len(data) < size:
            chunk = self.stdout.read(size - len(data))
            if not chunk:
                raise EOFError('end of stream')
            data += chunk
        return data

    def reply(self, timeout):
        try:
            message = self.replies.get(timeout=timeout)
        except queue.Empty:
            raise EpisodeError('The episode reader did not answer in time') from None
        if isinstance(message, Exception):
            self.replies.put(message)
            raise message
        return message

    def call(self, op, timeout, **fields):
        import msgpack
        self.ids += 1
        body = msgpack.packb(dict(fields, id=self.ids, op=op), use_bin_type=True)
        try:
            self.stdin.write(len(body).to_bytes(4, 'big') + body)
            self.stdin.flush()
        except (OSError, ValueError) as exc:
            raise EpisodeError(f'The episode reader is not running: {exc}') from exc
        message = self.reply(timeout)
        if message.get('id') != self.ids:
            raise EpisodeError('The episode reader answered out of order')
        if not message.get('ok'):
            raise EpisodeError(message.get('error') or 'The episode reader refused the request')
        return message['result']

    def close(self):
        try:
            self.closer()
        except Exception:  # noqa: BLE001 - closing a dead transport is not an error.
            pass


def local_transport(root, *, python=sys.executable):
    """A worker on this computer, reading `root`; it keeps this computer's CPU affinity and priority."""
    def open_connection():
        process = subprocess.Popen([python, '-u', str(WORKER), '--root', str(root), '--no-background'],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

        def close():
            for stream in (process.stdin, process.stdout):
                stream.close()
            process.kill()
            process.wait(5)
        return _Connection(process.stdin, process.stdout, close)
    return open_connection


def robot_transport(profile, address, *, open_ssh=None, timeout=5.):
    """A worker on Minerva's Jetson, over the pinned SSH connection to `address()`; it reads ~/minerva/evidence."""
    def open_connection():
        from robot_platform.ops.ssh import connect, open_command
        client = (open_ssh or connect)(profile, address=address(), timeout=timeout)
        try:
            source = WORKER.read_text(encoding='utf-8')
            stdin, out, _err = open_command(client, 'python3 -u -c ' + shlex.quote(source), timeout=timeout)
            # The reader thread waits for replies between requests; each request bounds its own wait.
            out.channel.settimeout(None)
        except BaseException:
            client.close()
            raise

        def close():
            out.channel.close()
            client.close()
        return _Connection(stdin, out, close)
    return open_connection


class EpisodeLibrary:
    def __init__(self, transport, *, busy=None, clock=time.monotonic, timeout=15., frame_timeout=6.):
        self.transport, self.busy, self.clock = transport, busy or (lambda: None), clock
        self.timeout, self.frame_timeout = timeout, frame_timeout
        self._lock = threading.Lock()
        self._connection, self._used = None, clock()
        self._episodes, self._frames = OrderedDict(), OrderedDict()
        self._listing, self._error = [], None
        self._closed = False
        self._idle = threading.Thread(target=self._close_when_idle, daemon=True, name='helm-episode-idle')
        self._idle.start()

    def _call(self, op, timeout=None, **fields):
        """Under the lock: one worker request, starting the worker first if needed; a failure drops the worker."""
        if self._closed:
            raise EpisodeError('Replay is closed')
        reason = self.busy()
        if reason:
            raise EpisodeBusy(reason)
        self._used = self.clock()
        try:
            if self._connection is None:
                self._connection = self.transport()
                hello = self._connection.reply(self.timeout)
                if hello.get('hello') != 1:
                    raise EpisodeError(f'Unexpected episode reader {hello!r}')
            return self._connection.call(op, timeout or self.timeout, **fields)
        except EpisodeError:
            self._drop()
            raise
        except Exception as exc:  # noqa: BLE001 - an unreachable robot is an answer for the page, not a crash.
            self._drop()
            raise EpisodeError(str(exc) or type(exc).__name__) from exc

    def _drop(self):
        if self._connection is not None:
            self._connection.close()
            self._connection = None

    def listing(self):
        """{'episodes': newest first, 'busy': reason or None, 'error': text or None}; the last list while busy."""
        with self._lock:
            try:
                self._listing, self._error = self._call('list'), None
            except EpisodeBusy as exc:
                return {'episodes': self._listing, 'busy': str(exc), 'error': None}
            except EpisodeError as exc:
                self._error = str(exc)
            return {'episodes': self._listing, 'busy': None, 'error': self._error}

    def episode(self, episode):
        """{'summary': ..., 'series': ...}: per-frame times (s from the first frame) and joint arrays as lists."""
        with self._lock:
            known = self._episodes.get(episode)
            if known is None:
                summary = self._call('summary', episode=episode)
                known = {'summary': summary, 'series': _series(self._call('series', episode=episode))}
                self._episodes[episode] = known
                while len(self._episodes) > EPISODE_CACHE:
                    self._episodes.popitem(last=False)
            self._episodes.move_to_end(episode)
            return known

    def frame(self, episode, index, role):
        """One camera's JPEG of frame `index`; the worker is asked once for every camera of a frame."""
        key = (episode, index)
        with self._lock:
            images = self._frames.get(key)
            if images is None:
                images = self._call('frame', self.frame_timeout, episode=episode, index=index)['images']
                self._frames[key] = images
                while len(self._frames) > FRAME_CACHE:
                    self._frames.popitem(last=False)
            self._frames.move_to_end(key)
        if role not in images:
            raise KeyError(f'Episode {episode} has no {role} camera')
        return images[role]

    def review(self, episode, verdict, note=''):
        """Keep or Discard `episode` for training (None clears it); the robot appends it to the session's
        reviews.jsonl. The listing already read shows it at once."""
        with self._lock:
            record = self._call('review', episode=episode, verdict=verdict, note=note)
            for entry in self._listing:
                if entry.get('id') == episode:
                    entry['review'] = record
            return record

    def policies(self):
        """The exported policies on the robot (or beside a local --episodes copy): {'policies', 'busy', 'error'}."""
        with self._lock:
            try:
                return {'policies': self._call('policies'), 'busy': None, 'error': None}
            except EpisodeBusy as exc:
                return {'policies': [], 'busy': str(exc), 'error': None}
            except EpisodeError as exc:
                return {'policies': [], 'busy': None, 'error': str(exc)}

    def complete(self, episode):
        """True once the episode's file is known to be complete: its frames will not change."""
        known = self._episodes.get(episode)
        return bool(known and known['summary'].get('complete'))

    def _close_when_idle(self):
        while not self._closed:
            time.sleep(5.)
            with self._lock:
                if self._connection is not None and self.clock() - self._used > IDLE_CLOSE_S:
                    self._drop()

    def close(self):
        with self._lock:
            self._closed = True
            self._drop()


def _series(raw):
    """The worker's float32 bytes as rounded lists, the shape the page plots and scrubs over."""
    import numpy as np
    series = {'t': np.frombuffer(raw['t'], dtype='<f8').round(4).tolist(), 'frames': raw['frames']}
    for key in ('qpos', 'qcmd', 'actions', 'torques', 'qvel'):
        if key in raw:
            data = np.frombuffer(raw[key]['data'], dtype='<f4').reshape(raw[key]['shape']).round(4)
            # A reading the robot did not report is NaN in the file and null here: JSON has no NaN.
            finite = np.isfinite(data)
            series[key] = data.tolist() if finite.all() else np.where(finite, data, None).tolist()
    if 'action_valid' in raw:
        series['action_valid'] = [bool(v) for v in np.frombuffer(raw['action_valid'], dtype=np.uint8)]
    return series
