"""Read-only Jetson PUB relay. Source is delivered over pinned SSH, not installed.

No image decoding, camera writes, motor interfaces, or disk artifacts. numpy only
for the head camera's depth, and only while Helm asks for it (a headset's 3D
picture); the relay keeps to the robot's background CPUs.
The stdin watchdog terminates even when a pipe write is blocked.
"""
import json
import math
import os
import struct
import sys
import threading
import time

MAX_HEADER = 4096
MAX_JPEG = 2 * 1024 * 1024
ROLES = {'left_wrist': (5563, 'left_wrist'), 'right_wrist': (5565, 'right_wrist'), 'head': (5564, 'scene_cam')}
# The head camera's depth for a headset: nearer is under the D435's reach, farther is clutter at the table.
DEPTH_RANGE_M = (.15, 2.5)
DEPTH_ENCODING = 'zlib-u16le'
MAX_DEPTH_SIDE = 1280
PARTITION = '/etc/minerva/cpu-partition.json'


def keep_off_control_cpus(path=PARTITION):
    """This relay's threads onto the background CPUs where the robot has a CPU split (control.scheduling's file):
    it compresses the head camera's depth for a headset and must not take the arms' loop's CPUs. Threads started
    later inherit it."""
    try:
        with open(path, encoding='utf-8') as source:
            background = json.load(source)['background']
        os.sched_setaffinity(0, set(background))
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass


def head_depth(parts):
    """The head camera's depth for a headset: each 2x2 block's nearest reading (a near edge stays near), readings
    outside DEPTH_RANGE_M dropped (0), as zlib-compressed little-endian uint16; with its metadata: the half-size
    raster's intrinsics and the color registration the camera publishes. None where the camera sent no depth."""
    import msgpack
    import numpy as np
    import zlib
    header = msgpack.unpackb(parts[0], raw=False)
    names, meta = header.get('parts', []), header.get('depth')
    if 'depth' not in names or not isinstance(meta, dict):
        return None
    width, height, k, scale = meta.get('width'), meta.get('height'), meta.get('intrinsics'), meta.get('scale')
    raw = parts[1 + names.index('depth')]
    if (type(width) is not int or type(height) is not int or not 2 <= width <= MAX_DEPTH_SIDE
            or not 2 <= height <= MAX_DEPTH_SIDE or width % 2 or height % 2 or len(raw) != width * height * 2
            or not isinstance(k, dict) or type(scale) not in (int, float) or not 0 < scale < 1):
        raise ValueError('Malformed camera depth')
    depth = np.frombuffer(raw, dtype='<u2').reshape(height, width)
    low, high = (int(round(m / scale)) for m in DEPTH_RANGE_M)
    near = None
    for block in (depth[0::2, 0::2], depth[0::2, 1::2], depth[1::2, 0::2], depth[1::2, 1::2]):
        readings = block.copy()
        readings[(readings < low) | (readings > high)] = 65535  # No reading (0), or out of range: never the nearest.
        near = readings if near is None else np.minimum(near, readings, out=near)
    near[near == 65535] = 0
    packed = zlib.compress(near.astype('<u2', copy=False).tobytes(), 1)
    # A half-size pixel u' covers u = 2u' and 2u'+1, centred on 2u' + 0.5: (u - cx)/fx = (u' - (cx - .5)/2)/(fx/2).
    half = dict(fx=k['fx'] / 2, fy=k['fy'] / 2, cx=(k['cx'] - .5) / 2, cy=(k['cy'] - .5) / 2,
                width=width // 2, height=height // 2)
    out = {'width': width // 2, 'height': height // 2, 'scale': scale, 'encoding': DEPTH_ENCODING,
           'range_m': list(DEPTH_RANGE_M), 'intrinsics': half, 'bytes': len(packed)}
    color_k, to_color = (header.get('color') or {}).get('intrinsics'), meta.get('to_color')
    if isinstance(color_k, dict) and isinstance(to_color, dict):
        out.update(color_intrinsics=color_k, to_color=to_color)
    return out, packed


def camera_packet(parts, camera_id, wall_now):
    import msgpack
    if not 2 <= len(parts) <= 3 or len(parts[0]) > MAX_HEADER:
        raise ValueError('Malformed camera multipart header')
    header = msgpack.unpackb(parts[0], raw=False)
    if not isinstance(header, dict) or header.get('camera_id') != camera_id:
        raise ValueError('Camera identity mismatch')
    stamp = header.get('timestamp')
    if type(stamp) not in (int, float) or not math.isfinite(stamp):
        raise ValueError('Camera publisher timestamp missing')
    age = wall_now - stamp
    if not -.1 <= age < 1.:
        raise ValueError('Camera publisher frame is stale or future-dated')
    names = header.get('parts', [])
    if not isinstance(names, list) or len(names) != len(parts)-1 or names.count('color') != 1:
        raise ValueError('Malformed camera parts')
    if header.get('color', {}).get('format', '').lower() != 'jpeg':
        raise ValueError('Camera did not publish JPEG')
    jpeg = parts[1 + names.index('color')]
    if not 4 <= len(jpeg) <= MAX_JPEG or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
        raise ValueError('Invalid camera JPEG envelope')
    return {'source_timestamp': stamp, 'source_age_s': max(0., age)}, jpeg


def write_packet(output, header, jpeg=b''):
    encoded = json.dumps(header, allow_nan=False, separators=(',', ':')).encode()
    if len(encoded) > MAX_HEADER or len(jpeg) > MAX_JPEG:
        raise ValueError('Relay packet exceeds bound')
    output.write(struct.pack('!II', len(encoded), len(jpeg)) + encoded + jpeg)
    output.flush()


def run(roles=None, *, heartbeat_timeout=3.):
    import zmq
    keep_off_control_cpus()
    roles = roles or ROLES
    lock = threading.Lock()
    heartbeat = {'at': time.monotonic(), 'token': None, 'depth': False}
    def read_parent():
        while True:
            line = sys.stdin.buffer.readline(256)
            if not line or len(line) >= 256:
                os._exit(0)
            try:
                message = json.loads(line)
                token, depth = message['heartbeat'], message.get('depth', False)  # Whether a headset wants depth.
                if type(token) is not int or token < 0 or type(depth) is not bool:
                    raise ValueError('Invalid heartbeat')
            except Exception:
                os._exit(2)
            with lock:
                heartbeat.update(at=time.monotonic(), token=token, depth=depth)
    def watchdog():
        while True:
            time.sleep(.1)
            with lock:
                expired = time.monotonic() - heartbeat['at'] > heartbeat_timeout
            if expired:
                os._exit(0)
    threading.Thread(target=read_parent, daemon=True).start()
    threading.Thread(target=watchdog, daemon=True).start()
    context, poller, sockets = zmq.Context(), zmq.Poller(), {}
    errors, sent, stamps = {}, {}, {}
    try:
        for role, (port, identity) in roles.items():
            socket = context.socket(zmq.SUB)
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.RCVHWM, 1)
            socket.setsockopt(zmq.MAXMSGSIZE, MAX_JPEG)
            socket.setsockopt(zmq.SUBSCRIBE, b'')
            socket.connect('tcp://127.0.0.1:' + str(port))
            sockets[socket] = role, identity
            poller.register(socket, zmq.POLLIN)
        status_at = 0.
        while True:
            ready = dict(poller.poll(50))
            with lock:
                token, depth_wanted = heartbeat['token'], heartbeat['depth']
            if token is None:
                continue
            for socket, (role, identity) in sockets.items():
                if socket not in ready:
                    continue
                parts = socket.recv_multipart()
                now = time.monotonic()
                if now - sent.get(role, -1.) < .1:
                    continue
                try:
                    header, jpeg = camera_packet(parts, identity, time.time())
                    if header['source_timestamp'] <= stamps.get(role, -1.):
                        raise ValueError('Camera publisher timestamp did not advance')
                    header, payload = dict(header, kind='frame', role=role, heartbeat=token), jpeg
                    depth = head_depth(parts) if role == 'head' and depth_wanted else None
                    if depth is not None:  # The depth rides after its own frame's JPEG.
                        header.update(jpeg_bytes=len(jpeg), depth=depth[0])
                        payload = jpeg + depth[1]
                    write_packet(sys.stdout.buffer, header, payload)
                    sent[role], stamps[role], errors[role] = now, header['source_timestamp'], None
                except (ValueError, KeyError, TypeError, ImportError) as exc:  # ImportError: no numpy for depth.
                    errors[role] = str(exc)[:200]
            now = time.monotonic()
            if now - status_at >= .2:
                write_packet(sys.stdout.buffer, {'kind':'status', 'heartbeat':token, 'errors':errors})
                status_at = now
    finally:
        for socket in sockets:
            socket.close(0)
        context.term()


if __name__ == '__main__':
    run()
