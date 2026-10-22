"""Episode reader for Helm's Replay. Source is delivered over pinned SSH, not installed.

Run as `python3 -u -c <this source> [--root DIR]` on the robot (Helm's EpisodeLibrary sends it, as it sends
the camera relay), or locally with --root for episodes on this computer. Depends only on h5py, numpy, Pillow
and msgpack. It never writes an episode file and never opens a file outside its root. Its one write is a review:
a line appended to the session's reviews.jsonl (Keep or Discard for training; the latest line per episode wins).
It serves framed requests on stdin:

    request  {'id': n, 'op': 'list'}
             {'id': n, 'op': 'summary' | 'series', 'episode': '<session>/<name>'}
             {'id': n, 'op': 'frame', 'episode': ..., 'index': i, 'quality': 85}
             {'id': n, 'op': 'review', 'episode': ..., 'verdict': 'keep' | 'discard' | None, 'note': text}
    reply    {'id': n, 'ok': True, 'result': ...} or {'id': n, 'ok': False, 'error': text}

Each frame on the wire is a 4-byte big-endian length and a msgpack body. The first reply is a hello
({'hello': VERSION, 'root': ...}). On the robot it moves to the background CPUs and lowers its priority, so
reading never competes with the control loop; it exits at the end of stdin or after IDLE_EXIT_S without a request.
"""
from collections import OrderedDict
import io
import json
import os
from pathlib import Path
import re
import sys
import threading
import time

VERSION = 1
MAX_REQUEST = 64 * 1024
IDLE_EXIT_S = 600.
OPEN_FILES = 2
SESSION_RE = re.compile(r'^live-\d{8}T\d{6}Z$')
NAME_RE = re.compile(r'^task_episode_[0-9a-f]{32}(\.partial)?$')
REVIEWS = 'reviews.jsonl'
VERDICTS = ('keep', 'discard')
MAX_NOTE = 300
PARTITION = '/etc/minerva/cpu-partition.json'


def resolve_episode(root, episode):
    """The file of `episode` ('<session>/<name>') below `root`; a ValueError for anything else."""
    if not isinstance(episode, str) or episode.count('/') != 1:
        raise ValueError('An episode is named <session>/<name>')
    session, name = episode.split('/')
    if not SESSION_RE.match(session) or not NAME_RE.match(name):
        raise ValueError(f'Not an episode name: {episode!r}')
    path = (root / session / 'episodes' / (name + '.hdf5')).resolve()
    if root not in path.parents or not path.is_file():
        raise ValueError(f'No such episode: {episode}')
    return path


def _text(value):
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    return value.item() if hasattr(value, 'item') else value


def _roles(attrs):
    roles = _text(attrs.get('camera_roles', '[]'))
    try:
        return [str(role) for role in json.loads(roles)]
    except (TypeError, ValueError):
        return []


class Episodes:
    """The episodes below one root, with a few files kept open and per-file facts cached by size and mtime."""

    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self._open = OrderedDict()
        self._facts = {}
        self._reviews = {}  # A session's reviews by file size and mtime.

    def reviews(self, session_dir):
        """{episode name: its latest review} from the session's reviews.jsonl; a cleared verdict is None."""
        path = Path(session_dir) / REVIEWS
        try:
            stat = path.stat()
        except FileNotFoundError:
            return {}
        key = (stat.st_size, stat.st_mtime_ns)
        cached = self._reviews.get(path)
        if cached is not None and cached[0] == key:
            return cached[1]
        latest = {}
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
            try:
                record = json.loads(line)
            except ValueError:
                continue  # A line cut short by a power loss: the earlier ones stand.
            if isinstance(record, dict) and isinstance(record.get('episode'), str) and record.get('verdict') in (*VERDICTS, None):
                latest[record['episode']] = record
        self._reviews[path] = (key, latest)
        return latest

    def review(self, episode, verdict, note=''):
        """Mark `episode` Keep or Discard for training (None clears it): one line appended to its session's
        reviews.jsonl, flushed to disk. The episode file is never touched."""
        from datetime import datetime, timezone
        path = resolve_episode(self.root, episode)
        if verdict not in (*VERDICTS, None):
            raise ValueError('A review is keep, discard, or none to clear it')
        if not isinstance(note, str) or len(note) > MAX_NOTE:
            raise ValueError(f'A review note is text of at most {MAX_NOTE} characters')
        name = path.name[:-len('.hdf5')]
        if verdict == 'keep':
            facts = self._quick_facts(path)
            if name.endswith('.partial') or not facts['complete'] or facts['error']:
                raise ValueError('Only a complete episode can be kept for training')
        record = {'episode': name, 'verdict': verdict, 'note': note.strip(),
                  'at': datetime.now(timezone.utc).isoformat(timespec='seconds'), 'by': 'helm-replay'}
        target = path.parent.parent / REVIEWS
        with open(target, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, separators=(',', ':')) + '\n')
            handle.flush()
            os.fsync(handle.fileno())
        return record

    def file(self, episode):
        import h5py
        path = resolve_episode(self.root, episode)
        stat = path.stat()
        key = (stat.st_size, stat.st_mtime_ns)
        held = self._open.pop(path, None)
        if held is not None and held[0] != key:
            held[1].close()
            held = None
        if held is None:
            held = (key, h5py.File(path, 'r'))
        self._open[path] = held
        while len(self._open) > OPEN_FILES:
            self._open.popitem(last=False)[1][1].close()
        return held[1]

    def close(self):
        while self._open:
            self._open.popitem()[1][1].close()

    def list(self):
        episodes = []
        if not self.root.is_dir():
            return episodes
        for session in self.root.iterdir():
            if not SESSION_RE.match(session.name) or not (session / 'episodes').is_dir():
                continue
            reviews = self.reviews(session)
            for path in (session / 'episodes').glob('task_episode_*.hdf5'):
                name = path.name[:-len('.hdf5')]
                if not NAME_RE.match(name):
                    continue
                stat = path.stat()
                key = (path, stat.st_size, stat.st_mtime_ns)
                if key not in self._facts:
                    self._facts[key] = self._quick_facts(path)
                episodes.append(dict(self._facts[key], id=f'{session.name}/{name}', session=session.name, name=name,
                                     partial=name.endswith('.partial'), size=stat.st_size, modified=stat.st_mtime,
                                     review=reviews.get(name)))
        episodes.sort(key=lambda entry: entry['modified'], reverse=True)
        return episodes

    @staticmethod
    def _quick_facts(path):
        import h5py
        try:
            with h5py.File(path, 'r') as f:
                attrs = f.attrs
                frames, duration = 0, None
                if 'timestamps/telem' in f:
                    telem = f['timestamps/telem']
                    frames = int(telem.shape[0])
                    if frames > 1:
                        duration = float(telem[frames - 1] - telem[0])
                return dict(complete=bool(_text(attrs.get('complete', False))), frames=frames, duration=duration,
                            collected_at=_text(attrs.get('collected_at')), roles=_roles(attrs), error=None)
        except Exception as exc:  # noqa: BLE001 - a damaged or locked file is listed, with the reason.
            return dict(complete=False, frames=0, duration=None, collected_at=None, roles=[], error=str(exc)[:200])

    def summary(self, episode):
        f = self.file(episode)
        attrs = f.attrs
        telem = f['timestamps/telem'][:]
        frames = int(telem.shape[0])
        gaps = [float(b - a) for a, b in zip(telem[:-1], telem[1:])]
        roles = _roles(attrs) or sorted(f['observations/images'].keys())
        metadata = {}
        try:
            metadata = json.loads(_text(attrs.get('metadata_json', '{}')))
        except (TypeError, ValueError):
            pass
        profile = metadata.get('capture_profile') or {}
        joints = str(_text(attrs.get('minerva_joints', ''))).split(',') if 'minerva_joints' in attrs else []
        duration = float(telem[-1] - telem[0]) if frames > 1 else 0.
        return dict(
            id=episode, frames=frames, duration=duration,
            rate_hz=(frames - 1) / duration if duration > 0 else None,
            gap_s=dict(median=sorted(gaps)[len(gaps) // 2], max=max(gaps)) if gaps else None,
            hz=_text(attrs.get('hz')), roles=roles,
            image_shapes={role: list(f['observations/images/' + role].shape[1:]) for role in roles
                          if 'observations/images/' + role in f},
            joints=joints, complete=bool(_text(attrs.get('complete', False))),
            collected_at=_text(attrs.get('collected_at')), data_source=_text(attrs.get('data_source')),
            language_instruction=_text(attrs.get('language_instruction', '')) or None,
            capture_eligible=_text(attrs.get('capture_eligible')), training_promoted=_text(attrs.get('training_promoted')),
            format_version=_text(attrs.get('format_version')), recording_hz=profile.get('hz'))

    def series(self, episode):
        """Per-frame arrays as little-endian float32 bytes with their shapes: compact over the tether."""
        import numpy as np
        f = self.file(episode)
        telem = f['timestamps/telem'][:]
        result = {'t': (telem - telem[0]).astype('<f8').tobytes(), 'frames': int(telem.shape[0])}
        for key, name in (('qpos', 'observations/qpos'), ('qcmd', 'observations/qcmd'), ('actions', 'actions')):
            if name in f:
                data = np.asarray(f[name][:], dtype='<f4')
                result[key] = {'shape': list(data.shape), 'data': data.tobytes()}
        if 'action_valid' in f:
            result['action_valid'] = np.asarray(f['action_valid'][:], dtype=np.uint8).tobytes()
        return result

    def frame(self, episode, index, quality=85):
        from PIL import Image
        f = self.file(episode)
        telem = f['timestamps/telem']
        frames = int(telem.shape[0])
        if type(index) is not int or not 0 <= index < frames:
            raise ValueError(f'Frame {index!r} is outside 0..{frames - 1}')
        if type(quality) is not int or not 30 <= quality <= 95:
            raise ValueError('JPEG quality must be an integer in 30..95')
        images = {}
        for role in _roles(f.attrs) or sorted(f['observations/images'].keys()):
            dataset = 'observations/images/' + role
            if dataset not in f:
                continue
            buffer = io.BytesIO()
            Image.fromarray(f[dataset][index]).save(buffer, format='JPEG', quality=quality)
            images[role] = buffer.getvalue()
        return {'index': index, 't': float(telem[index] - telem[0]), 'images': images}


def background():
    """Move onto the robot's background CPUs (if it has a partition) and lower this process's priority."""
    try:
        split = json.loads(Path(PARTITION).read_text(encoding='utf-8'))
        cpus = {int(cpu) for cpu in split['background']}
        if cpus:
            os.sched_setaffinity(0, cpus)
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass
    try:
        os.nice(10)
    except (OSError, AttributeError):
        pass


def _read_exactly(stream, size):
    data = b''
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise EOFError
        data += chunk
    return data


def serve(root, stdin, stdout, *, idle_exit_s=IDLE_EXIT_S):
    import msgpack
    episodes = Episodes(root)
    last = [time.monotonic()]
    done = threading.Event()

    def watchdog():
        while not done.wait(5.):
            if time.monotonic() - last[0] > idle_exit_s:
                os._exit(0)

    threading.Thread(target=watchdog, daemon=True).start()

    def send(message):
        body = msgpack.packb(message, use_bin_type=True)
        stdout.write(len(body).to_bytes(4, 'big') + body)
        stdout.flush()

    send({'hello': VERSION, 'root': str(episodes.root), 'pid': os.getpid()})
    try:
        while True:
            try:
                size = int.from_bytes(_read_exactly(stdin, 4), 'big')
            except EOFError:
                return
            if not 0 < size <= MAX_REQUEST:
                raise ValueError(f'A {size}-byte request exceeds the limit')
            request = msgpack.unpackb(_read_exactly(stdin, size), raw=False)
            last[0] = time.monotonic()
            ident = request.get('id') if isinstance(request, dict) else None
            try:
                op = request['op']
                if op == 'list':
                    result = episodes.list()
                elif op in ('summary', 'series'):
                    result = getattr(episodes, op)(request['episode'])
                elif op == 'frame':
                    result = episodes.frame(request['episode'], request['index'], request.get('quality', 85))
                elif op == 'review':
                    result = episodes.review(request['episode'], request.get('verdict'), request.get('note', ''))
                else:
                    raise ValueError(f'Unknown request {op!r}')
                send({'id': ident, 'ok': True, 'result': result})
            except Exception as exc:  # noqa: BLE001 - every failure is answered, never fatal.
                send({'id': ident, 'ok': False, 'error': str(exc)[:500] or type(exc).__name__})
    finally:
        done.set()
        episodes.close()


def main(argv):
    root = Path.home() / 'minerva' / 'evidence'
    if '--root' in argv:
        root = Path(argv[argv.index('--root') + 1])
    if '--no-background' not in argv:
        background()
    serve(root, sys.stdin.buffer, sys.stdout.buffer)


if __name__ == '__main__':
    main(sys.argv[1:])
