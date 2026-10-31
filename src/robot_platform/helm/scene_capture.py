"""A head scene capture for a splat map (docs/development/HEADSET_3D_PLAN.md, phase 4), run on the robot: the head
camera's colour pictures and depth with the head camera's pose in Minerva's frame, as the splat trainer reads them
(scripts/splat/train_head_scene.py).

For `seconds`, the head camera's published frames are read with the head's pose feed (neck.head_pose: each of the
head service's rows with the camera's pose, at the Jetson's wall time, the clock the frames are stamped with). A frame
is kept when the head turned or moved enough since the last kept one, at most `fps` a second, and only if the head was
steady around its exposure (a turning head blurs the picture and smears its pose). Each kept frame's picture is saved
as published (JPEG), its depth zlib-compressed as the trainer reads it, its pose the feed's sample nearest its exposure,
with Minerva's pose in the world beside it (`robot_in_world`, from Helm: fixed while the base stands).

Helm sends this file as source over its pinned SSH (helm.maps), so it imports only what the robot's python3 has:
pyzmq and msgpack. Nothing here moves a motor or changes a camera setting. It prints one JSON reply.
"""
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import zlib

CAPTURE_ROOT = '~/minerva/helm/maps/captures'
FRAMES = 'tcp://127.0.0.1:5564'  # The head camera's stream (helm.calibration_still ROLES['head']).
CAMERA_ID = 'scene_cam'
HEAD_POSE = 'tcp://127.0.0.1:5596'  # neck.head_pose.ENDPOINT.
CAPTURE_LAG_S = .02  # A frame's stamp is this long after its exposure (helm.camera_video_worker).
MARGIN_S = .05  # The head's samples this far either side of an exposure tell how steady it was.
FIRST_S = 1.  # The feed's first sample is awaited this long: none means no live head.
MAX_SECONDS = 120
MAX_FPS = 6.
QUEUE = 8  # Kept frames waiting for the disk; more are dropped (the microSD stalls for seconds at times).


def _path(folder):
    root = Path(os.path.expanduser(CAPTURE_ROOT)).resolve()
    target = Path(os.path.expanduser(str(folder))).resolve()
    if root not in target.parents:
        raise ValueError(f'Captures are saved under {CAPTURE_ROOT}')
    if target.exists():
        raise ValueError(f'{target.name} exists already')
    return target


def _rotation_angle(a, b):
    """The angle between two orientations (quaternions x y z w), radians."""
    dot = abs(sum(p * q for p, q in zip(a, b)))
    return 2 * math.acos(min(1., dot))


class HeadFeed:
    """The head pose feed's samples ([wall time, *angles, *pose]), read on a thread of its own."""

    def __init__(self, endpoint=HEAD_POSE):
        self.endpoint, self.samples, self.joints = endpoint, [], None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        import zmq
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        try:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SUBSCRIBE, b'')
            socket.connect(self.endpoint)
            while not self._stop.is_set():
                if socket.poll(20):
                    data = json.loads(socket.recv_string())
                    self.joints = list(data['joints'])
                    self.samples.extend([float(v) for v in row[:1 + len(self.joints) + 7]] for row in data['samples'])
                    del self.samples[:-1000]  # Ten seconds of rows is plenty.
        finally:
            socket.close(0)
            context.term()

    def around(self, moment, margin):
        """(nearest sample, largest spread of any joint) within `margin` of `moment`; None unless the samples reach
        both sides of it."""
        rows = [row for row in list(self.samples) if abs(row[0] - moment) <= margin]
        if not rows or rows[0][0] > moment or rows[-1][0] < moment or not self.joints:
            return None
        width = len(self.joints)
        spread = max(max(row[1 + k] for row in rows) - min(row[1 + k] for row in rows) for k in range(width))
        return min(rows, key=lambda row: abs(row[0] - moment)), spread

    def close(self):
        self._stop.set()
        self._thread.join(1.)


def scene(request, *, clock=time.time, sleep=time.sleep, frames_endpoint=FRAMES, feed_endpoint=HEAD_POSE):
    import msgpack
    import zmq
    seconds, fps = float(request.get('seconds', 30)), float(request.get('fps', 3))
    if not (0 < seconds <= MAX_SECONDS and 0 < fps <= MAX_FPS):
        raise ValueError(f'seconds must be 0..{MAX_SECONDS}, fps 0..{MAX_FPS}')
    min_turn = math.radians(float(request.get('min_turn_deg', 3.)))
    min_move = float(request.get('min_move_m', .03))
    steady = float(request.get('steady_rad', .02))
    world_pose = request.get('robot_in_world')
    if world_pose is not None and (len(world_pose) != 7 or not all(math.isfinite(float(v)) for v in world_pose)):
        raise ValueError('robot_in_world is seven numbers: x y z qx qy qz qw')
    folder = _path(request.get('folder', ''))
    feed = HeadFeed(feed_endpoint)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    pending, written, failures = queue.Queue(QUEUE), [], []

    def write():
        while True:
            item = pending.get()
            if item is None:
                return
            index, jpeg, depth = item
            try:
                (folder / f'color/{index:06d}.jpg').write_bytes(jpeg)
                (folder / f'depth/{index:06d}.u16.z').write_bytes(zlib.compress(depth, 1))
                written.append(index)
            except OSError as exc:
                failures.append(str(exc))

    writer = threading.Thread(target=write, daemon=True)
    try:
        deadline = clock() + FIRST_S
        while not feed.samples and clock() < deadline:
            sleep(.01)
        if not feed.samples:
            raise ValueError('The head pose feed is silent: a capture needs a session running the live head')
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVHWM, 4)
        socket.setsockopt(zmq.MAXMSGSIZE, 8 * 1024 * 1024)
        socket.setsockopt(zmq.SUBSCRIBE, b'')
        socket.connect(frames_endpoint)
        (folder / 'color').mkdir(parents=True)
        (folder / 'depth').mkdir()
        writer.start()
        frames, meta, last = [], None, None
        counts = {'seen': 0, 'unposed': 0, 'turning': 0, 'same_view': 0, 'rate': 0, 'dropped': 0}
        end = clock() + seconds
        while clock() < end:
            if not socket.poll(100):
                continue
            parts = socket.recv_multipart()
            header = msgpack.unpackb(parts[0], raw=False)
            names = header.get('parts', [])
            if header.get('camera_id') != CAMERA_ID or len(names) != len(parts) - 1:
                raise ValueError(f'Not the head camera\'s stream: {header.get("camera_id")!r}')
            if 'depth' not in names or 'color' not in names:
                raise ValueError('The head camera sends no depth: a capture needs its depth stream')
            counts['seen'] += 1
            depth_meta, color = header['depth'], header['color']
            this = dict(color_intrinsics=color.get('intrinsics'), depth_intrinsics=depth_meta.get('intrinsics'),
                        to_color=depth_meta.get('to_color'), depth_scale=depth_meta.get('scale'))
            if meta is None:
                if any(value is None for value in this.values()) or color.get('format', '').lower() != 'jpeg':
                    raise ValueError('The head camera publishes no colour registration with its depth')
                meta = this
            elif this != meta:
                raise ValueError('The head camera\'s intrinsics changed during the capture')
            exposure = float(header['timestamp']) - CAPTURE_LAG_S
            if last is not None and exposure - last['exposure'] < 1 / fps:
                counts['rate'] += 1
                continue
            found = None
            for _ in range(20):  # The feed comes in a reply's rows: the ones after this exposure may be on their way.
                found = feed.around(exposure, MARGIN_S)
                if found is not None:
                    break
                sleep(.005)
            if found is None:
                counts['unposed'] += 1
                continue
            sample, spread = found
            if spread > steady:
                counts['turning'] += 1
                continue
            pose = sample[1 + len(feed.joints):]
            if last is not None and _rotation_angle(pose[3:], last['pose'][3:]) < min_turn \
                    and math.dist(pose[:3], last['pose'][:3]) < min_move:
                counts['same_view'] += 1
                continue
            index = len(frames)
            try:
                pending.put_nowait((index, bytes(parts[1 + names.index('color')]),
                                    bytes(parts[1 + names.index('depth')])))
            except queue.Full:
                counts['dropped'] += 1
                continue
            record = {'color': f'color/{index:06d}.jpg', 'depth': f'depth/{index:06d}.u16.z', 'pose': pose,
                      'joints': dict(zip(feed.joints, sample[1:1 + len(feed.joints)])), 'stamp': exposure}
            if world_pose is not None:
                record['robot_in_world'] = [float(v) for v in world_pose]
            frames.append(record)
            last = {'exposure': exposure, 'pose': pose}
    finally:
        feed.close()
        socket.close(0)
        context.term()
        if writer.is_alive():
            pending.put(None)
            writer.join(30.)
    if failures:
        raise RuntimeError(f'The capture did not save: {failures[0]}')
    saved = set(written)
    frames = [frame for index, frame in enumerate(frames) if index in saved]
    if meta is None:
        raise ValueError('No head camera frame came during the capture')
    scene_file = dict(schema=1, kind='head_scene', source='minerva', world=request.get('world', 'robot'),
                      captured_at=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), seconds=seconds, frames=frames,
                      **meta)
    (folder / 'scene.json').write_text(json.dumps(scene_file, indent=1), encoding='utf-8')
    size = sum(path.stat().st_size for path in folder.rglob('*') if path.is_file())
    return dict(folder=str(folder), name=folder.name, frames=len(frames), counts=counts, bytes=size,
                world=scene_file['world'])


def remote_main(request):
    import signal

    def expired(*_):
        raise TimeoutError('The capture exceeded its robot budget')
    timed = hasattr(signal, 'SIGALRM')  # The robot's Linux; not a Windows test run.
    if timed:
        signal.signal(signal.SIGALRM, expired)
        signal.alarm(int(min(float(request.get('seconds', 30)), MAX_SECONDS)) + 40)
    try:
        if request.get('operation') != 'scene':
            raise ValueError('Unknown capture operation')
        print(json.dumps({'ok': True, 'result': scene(request)}), flush=True)
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'}), flush=True)
    finally:
        if timed:
            signal.alarm(0)
