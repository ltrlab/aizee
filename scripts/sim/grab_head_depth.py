"""Run on the Jetson, read-only: the head camera's depth, as a median over up to 20 frames.

It subscribes to the head camera node's publisher (port 5564) for up to 6 s and prints JSON: the per-pixel median of
the valid (non-zero) depth over the frames, the count of valid frames and the median absolute deviation (each uint16,
zlib, base64), the last colour JPEG and its header. It opens no camera, commands nothing and writes nothing. Run it
with `python3 -c "$(cat grab_head_depth.py)"` on the Jetson (the Minerva terminal tools' robot_remote.py does that),
and unpack the JSON into what scripts/sim/head_depth_check.py reads: depth_median.npy, depth_count.npy, depth_mad.npy,
head.jpg and header.json.
"""

import base64, json, time, zlib
import msgpack, numpy as np, zmq

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.setsockopt(zmq.SUBSCRIBE, b'')
sock.setsockopt(zmq.RCVHWM, 4)
sock.setsockopt(zmq.LINGER, 0)
sock.connect('tcp://127.0.0.1:5564')
frames, header, jpeg, stamps = [], None, None, []
started = time.monotonic()
try:
    while time.monotonic() - started < 6 and len(frames) < 20:
        if not sock.poll(200):
            continue
        parts = sock.recv_multipart()
        head = msgpack.unpackb(parts[0], raw=False)
        names = head.get('parts', [])
        if 'depth' not in names or 'color' not in names:
            continue
        d = head['depth']
        raw = parts[names.index('depth') + 1]
        depth = np.frombuffer(raw, dtype='<u2').reshape(d['height'], d['width'])
        if time.monotonic() - started < 0.3:
            continue  # Skip anything queued before we joined.
        frames.append(depth.copy())
        header, jpeg = head, parts[names.index('color') + 1]
        stamps.append([head.get('frame_number'), head.get('timestamp')])
finally:
    sock.close()
    ctx.term()
result = {'motors_commanded': False, 'frames': len(frames), 'stamps': stamps, 'header': header}
if frames:
    stack = np.stack(frames).astype(np.float32)
    stack[stack == 0] = np.nan
    median = np.nanmedian(stack, axis=0)
    count = np.sum(np.isfinite(stack), axis=0).astype(np.uint8)
    mad = np.nanmedian(np.abs(stack - median[None]), axis=0)
    median = np.nan_to_num(median, nan=0.0).round().astype('<u2')
    mad = np.nan_to_num(mad, nan=0.0).clip(0, 65535).round().astype('<u2')
    for name, array in (('median', median), ('count', count), ('mad', mad)):
        result[name] = {
            'dtype': str(array.dtype),
            'shape': list(array.shape),
            'zlib_base64': base64.b64encode(zlib.compress(array.tobytes(), 6)).decode(),
        }
    result['color_jpeg_base64'] = base64.b64encode(jpeg).decode()
print(json.dumps(result))
