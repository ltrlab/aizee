"""Read-only Jetson PUB relay. Source is delivered over pinned SSH, not installed.

No numpy, image decoding, camera writes, motor interfaces, or disk artifacts.
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
    roles = roles or ROLES
    lock = threading.Lock()
    heartbeat = {'at': time.monotonic(), 'token': None}
    def read_parent():
        while True:
            line = sys.stdin.buffer.readline(256)
            if not line or len(line) >= 256:
                os._exit(0)
            try:
                token = json.loads(line)['heartbeat']
                if type(token) is not int or token < 0:
                    raise ValueError('Invalid heartbeat')
            except Exception:
                os._exit(2)
            with lock:
                heartbeat.update(at=time.monotonic(), token=token)
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
                token = heartbeat['token']
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
                    write_packet(sys.stdout.buffer, dict(header, kind='frame', role=role, heartbeat=token), jpeg)
                    sent[role], stamps[role], errors[role] = now, header['source_timestamp'], None
                except (ValueError, KeyError, TypeError) as exc:
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
