"""One calibration still from a camera, run on the robot: a stack of fresh frames averaged, the table's tags found in
the average, and the light on the tags (their black border and the white label around it).

Helm sends this file as source over its pinned SSH (as helm.cameras does), so it imports only what the robot's
python3 has: numpy, OpenCV with cv2.aruco (4.5's API or 4.7's), pyzmq and msgpack. It reads the camera's published
stream; nothing here moves a motor or changes a camera setting. It saves the averaged image as a PNG under the
calibration folder and prints one JSON reply.
"""
import base64
import json
import math
import os
from pathlib import Path
import time

ROLES = {
    'left_wrist': {'frame_port': 5563, 'camera_id': 'left_wrist'},
    'right_wrist': {'frame_port': 5565, 'camera_id': 'right_wrist'},
    'head': {'frame_port': 5564, 'camera_id': 'scene_cam'},
}
CALIBRATION_ROOT = '~/minerva/helm/calibration'
MAX_FRAMES = 30
# Frames stamped before the request (plus this) may have been exposed before the arm was known to be still.
FRESH_AFTER_S = .1
# The marker's cells across its black square: 4 code bits and a one-cell border each side (DICT_4X4).
CELLS = 6
THUMBNAIL_WIDTH = 320


def collect(role, count, *, timeout_s=3., fresh_after_s=FRESH_AFTER_S, clock=time.time):
    """`count` consecutive frames from the role's stream, each stamped after the request: [(timestamp, jpeg)]."""
    import msgpack
    import zmq
    config = ROLES[role]
    start = clock()
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVHWM, 4 * MAX_FRAMES)
        socket.setsockopt(zmq.MAXMSGSIZE, 4 * 1024 * 1024)
        socket.setsockopt(zmq.SUBSCRIBE, b'')
        socket.connect('tcp://127.0.0.1:' + str(config['frame_port']))
        frames, deadline = [], start + timeout_s
        while len(frames) < count:
            remaining = deadline - clock()
            if remaining <= 0 or not socket.poll(int(remaining * 1000)):
                raise TimeoutError(f'{role}: {len(frames)} of {count} frames within {timeout_s:.1f} s')
            parts = socket.recv_multipart()
            header = msgpack.unpackb(parts[0], raw=False)
            if header.get('camera_id') != config['camera_id']:
                raise ValueError(f'{role}: stream identity is {header.get("camera_id")!r}')
            names = header.get('parts', [])
            if names.count('color') != 1 or len(names) != len(parts) - 1:
                raise ValueError(f'{role}: not a color frame packet')
            if header.get('color', {}).get('format', '').lower() != 'jpeg':
                raise ValueError(f'{role}: frames are not JPEG')
            stamp = header.get('timestamp')
            if type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise ValueError(f'{role}: frame without a timestamp')
            if stamp < start + fresh_after_s:
                continue
            frames.append((float(stamp), bytes(parts[1 + names.index('color')])))
        return frames
    finally:
        socket.close(0)
        context.term()


def decode(jpeg):
    import cv2
    import numpy as np
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('A frame did not decode')
    return image


def stack(images):
    """(mean BGR float32, temporal standard deviation of luma float32) of same-sized BGR uint8 images."""
    import numpy as np
    total = square = luma_total = None
    for image in images:
        x = np.asarray(image, np.float64)
        luma = .114 * x[..., 0] + .587 * x[..., 1] + .299 * x[..., 2]
        if total is None:
            total, square, luma_total = np.zeros_like(x), np.zeros_like(luma), np.zeros_like(luma)
        elif x.shape != total.shape:
            raise ValueError('Frame size changed within the stack')
        total += x
        luma_total += luma
        square += luma * luma
    n = len(images)
    if not n:
        raise ValueError('No frames to stack')
    mean = total / n
    variance = np.maximum(square / n - (luma_total / n) ** 2, 0.) * (n / (n - 1) if n > 1 else 0.)
    return mean.astype(np.float32), np.sqrt(variance).astype(np.float32)


def detector(family):
    """A function gray -> (corners, ids) with pixel-level corners, for OpenCV's old (4.5) or new (4.7+) aruco API.
    Sub-pixel refinement is left to refine(): OpenCV's fixed window (11 px) is wider than a cell of a small tag and
    pulls corners onto the code's inner edges (3 px off on 30 px tags, rehearsed)."""
    import cv2
    aruco = cv2.aruco
    code = getattr(aruco, family)
    dictionary = aruco.getPredefinedDictionary(code) if hasattr(aruco, 'getPredefinedDictionary') else \
        aruco.Dictionary_get(code)
    params = aruco.DetectorParameters() if hasattr(aruco, 'ArucoDetector') else aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
    if hasattr(aruco, 'ArucoDetector'):
        found = aruco.ArucoDetector(dictionary, params)
        return lambda gray: found.detectMarkers(gray)[:2]
    return lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


def refine(gray, quad):
    """Sub-pixel corners of one tag, with a search window inside the corner's own border cell: half-width 0.4 of a
    cell (2 to 8 px)."""
    import cv2
    import numpy as np
    quad = np.asarray(quad, np.float32).reshape(4, 2)
    side = float(np.mean(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)))
    half = int(max(2, min(8, round(.4 * side / CELLS))))
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, .005)
    refined = cv2.cornerSubPix(gray, quad.reshape(-1, 1, 2).copy(), (half, half), (-1, -1), criteria)
    return refined.reshape(4, 2).astype(float)


def detect(gray, family, wanted):
    """({id: [[u, v] x 4]} in OpenCV's corner order for wanted IDs seen once, other IDs, duplicated IDs)."""
    corners, ids = detector(family)(gray)
    tags, others, seen, duplicates = {}, set(), set(), set()
    for quad, tag in zip(corners, [] if ids is None else [int(i) for i in ids.ravel()]):
        if tag in seen:
            duplicates.add(tag)
        seen.add(tag)
        if tag not in wanted:
            others.add(tag)
        else:
            tags[tag] = refine(gray, quad)
    for tag in duplicates:
        tags.pop(tag, None)
    return tags, sorted(others), sorted(duplicates)


def _sample(image, points):
    """Bilinear samples of an (H, W[, C]) image at (N, 2) pixel points; points outside give NaN."""
    import numpy as np
    h, w = image.shape[:2]
    u, v = points[:, 0], points[:, 1]
    inside = (u >= 0) & (v >= 0) & (u <= w - 1) & (v <= h - 1)
    u0 = np.clip(np.floor(u).astype(int), 0, w - 2)
    v0 = np.clip(np.floor(v).astype(int), 0, h - 2)
    a, b = np.clip(u - u0, 0, 1), np.clip(v - v0, 0, 1)
    if image.ndim == 3:
        a, b = a[:, None], b[:, None]
    value = (image[v0, u0] * (1 - a) * (1 - b) + image[v0, u0 + 1] * a * (1 - b) + image[v0 + 1, u0] * (1 - a) * b
             + image[v0 + 1, u0 + 1] * a * b).astype(float)
    value[~inside] = np.nan
    return value


def _cell_points(quad, cells):
    """Pixels of marker cell coordinates (column, row from the code's top-left corner, 0..CELLS across), through
    the homography of the square's four corners (solved directly: numpy only)."""
    import numpy as np
    square = [(0, 0), (CELLS, 0), (CELLS, CELLS), (0, CELLS)]
    rows = []
    for (x, y), (u, v) in zip(square, np.asarray(quad, float).reshape(4, 2)):
        rows.append([x, y, 1, 0, 0, 0, -u * x, -u * y, u])
        rows.append([0, 0, 0, x, y, 1, -v * x, -v * y, v])
    a = np.array(rows, float)
    h = np.append(np.linalg.solve(a[:, :8], a[:, 8]), 1.).reshape(3, 3)
    points = np.c_[np.asarray(cells, float), np.ones(len(cells))] @ h.T
    return points[:, :2] / points[:, 2:]


def photometry(mean, sigma, tags):
    """The light on each tag: its black border cells and the white label half a cell outside its square, as mean BGR,
    luma and temporal sigma (the noise left in one frame), plus the white's red and blue against green."""
    import numpy as np
    edge = (0, CELLS - 1)
    border = [(c + .5, r + .5) for r in range(CELLS) for c in range(CELLS) if r in edge or c in edge]
    margin = [p for k in range(CELLS) for p in ((k + .5, -.5), (k + .5, CELLS + .5), (-.5, k + .5),
                                               (CELLS + .5, k + .5))]
    result = {}
    for tag, quad in tags.items():
        rows = {}
        for name, cells in (('black', border), ('white', margin)):
            points = _cell_points(quad, cells)
            bgr, noise = _sample(mean, points), _sample(sigma, points)
            keep = ~np.isnan(bgr).any(axis=1)
            if keep.sum() < 4:
                continue
            b, g, r = (float(v) for v in bgr[keep].mean(axis=0))
            rows[name] = dict(bgr=[round(b, 1), round(g, 1), round(r, 1)],
                              luma=round(.114 * b + .587 * g + .299 * r, 1),
                              sigma=round(float(np.nanmedian(noise[keep])), 2))
        if 'white' in rows:
            b, g, r = rows['white']['bgr']
            rows['white'].update(r_over_g=round(r / g, 3) if g else None, b_over_g=round(b / g, 3) if g else None)
        if 'white' in rows and 'black' in rows:
            rows['contrast'] = round(rows['white']['luma'] - rows['black']['luma'], 1)
        result[str(tag)] = rows
    return result


def thumbnail(mean, tags, others):
    """A small JPEG of the averaged image with the found tags outlined (green, their first corner dotted)."""
    import cv2
    import numpy as np
    image = np.clip(mean + .5, 0, 255).astype(np.uint8)
    for tag, quad in tags.items():
        points = np.round(np.asarray(quad)).astype(np.int32)
        cv2.polylines(image, [points.reshape(-1, 1, 2)], True, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.circle(image, tuple(int(v) for v in points[0]), 4, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(image, str(tag), tuple(int(v) for v in points.mean(axis=0)), cv2.FONT_HERSHEY_SIMPLEX, .8,
                    (0, 255, 255), 2, cv2.LINE_AA)
    scale = THUMBNAIL_WIDTH / image.shape[1]
    small = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode('.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        raise RuntimeError('Thumbnail did not encode')
    return base64.b64encode(jpeg.tobytes()).decode('ascii')


def _image_path(path):
    root = Path(os.path.expanduser(CALIBRATION_ROOT)).resolve()
    target = Path(os.path.expanduser(str(path))).resolve()
    if root not in target.parents or target.suffix != '.png':
        raise ValueError(f'Stills are saved as .png under {CALIBRATION_ROOT}')
    return target


def still(request):
    import cv2
    import numpy as np
    role, count = request.get('role'), request.get('frames', 8)
    if role not in ROLES:
        raise ValueError('Expected camera role left_wrist, right_wrist or head')
    if type(count) is not int or not 2 <= count <= MAX_FRAMES:
        raise ValueError(f'frames must be 2..{MAX_FRAMES}')
    wanted = {int(t) for t in request.get('tags', [])}
    target = _image_path(request.get('path', ''))
    frames = collect(role, count)
    mean, sigma = stack([decode(jpeg) for _, jpeg in frames])
    gray = cv2.cvtColor(np.clip(mean + .5, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY)
    tags, others, duplicates = detect(gray, request.get('family', 'DICT_4X4_50'), wanted)
    target.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(target), np.clip(mean + .5, 0, 255).astype(np.uint8)):
        raise RuntimeError('The still did not save')
    stamps = [t for t, _ in frames]
    return dict(role=role, frames=len(frames), first_stamp=stamps[0], last_stamp=stamps[-1],
                size=[int(mean.shape[1]), int(mean.shape[0])], image=str(target),
                tags={str(t): np.round(q, 3).tolist() for t, q in sorted(tags.items())}, others=others,
                duplicates=duplicates, photometry=photometry(mean, sigma, tags),
                noise=dict(luma_sigma_median=round(float(np.median(sigma)), 3), frames=len(frames)),
                thumbnail=thumbnail(mean, tags, others))


def remote_main(request):
    import signal

    def expired(*_):
        raise TimeoutError('Calibration still exceeded its 10 second robot budget')
    timed = hasattr(signal, 'SIGALRM')  # The robot's Linux; not a Windows test run.
    if timed:
        signal.signal(signal.SIGALRM, expired)
        signal.alarm(10)
    try:
        if request.get('operation') != 'still':
            raise ValueError('Unknown calibration operation')
        print(json.dumps({'ok': True, 'result': still(request)}), flush=True)
    except Exception as exc:
        print(json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'}), flush=True)
    finally:
        if timed:
            signal.alarm(0)
