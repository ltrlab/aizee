"""A synthetic head scene capture for checking the splat trainer (train_head_scene.py) without the robot: a textured
table with a box on it, a wall and a floor, seen by the head camera as the neck pans and tilts. Same format as a real
capture: scene.json, the colour pictures as JPEGs and the depth (from the depth sensor, 15 mm beside the colour
camera) as zlib-compressed uint16. numpy and Pillow only.

    python synthetic_scene.py <out folder> [--frames 64]
"""
import argparse
import io
import json
import math
from pathlib import Path
import zlib

import numpy as np
from PIL import Image

COLOR_K = {'fx': 615.3, 'fy': 615.6, 'cx': 322.4, 'cy': 238.9, 'width': 640, 'height': 480}
DEPTH_K = {'fx': 383.0, 'fy': 383.0, 'cx': 320.0, 'cy': 240.0, 'width': 640, 'height': 480}
TO_COLOR = {'rotation': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], 'translation': [.015, 0., 0.]}
NECK = np.array([0., 0., 1.13])  # The pan axis through the robot's base, the tilt joint above it.
HEAD_CAMERA = np.array([.033, -.11, 1.27])


def rotation(axis, angle):
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def camera_pose(pan, tilt):
    """The colour camera's optical pose in the robot frame: looking along the front (-y), level, then tilted down
    about the neck's tilt axis and panned about its pan axis (both through the neck, as the head turns)."""
    base = np.column_stack([[-1., 0, 0], [0, 0, -1.], [0, -1., 0]])  # x right, y down, z ahead (front -y).
    tilt_r = rotation([-1., 0, 0], tilt)  # About the robot's -x: positive raises a view along the front (-y).
    pan_r = rotation([0, 0, 1.], pan)  # About the robot's z: positive turns it left.
    turn = pan_r @ tilt_r
    pose = np.eye(4)
    pose[:3, :3] = turn @ base
    pose[:3, 3] = NECK + turn @ (HEAD_CAMERA - NECK)
    return pose


def pose7(pose):
    r = pose[:3, :3]
    w = math.sqrt(max(0., 1 + np.trace(r))) / 2
    if w > 1e-6:
        q = [(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w]
    else:  # A half turn: from the largest diagonal.
        k = int(np.argmax(np.diag(r)))
        v = np.zeros(3)
        v[k] = math.sqrt(max(0., 1 + 2 * r[k, k] - np.trace(r))) / 2
        for j in range(3):
            if j != k:
                v[j] = (r[k, j] + r[j, k]) / (4 * v[k])
        q = [*v, 0.]
    return [float(c) for c in (*pose[:3, 3], *q)]


def render(pose, k):
    """Colour (H x W x 3, uint8) and depth along the optical axis (metres, 0 for none) from a camera pose."""
    h, w = k['height'], k['width']
    v, u = np.mgrid[0:h, 0:w].astype(np.float64)
    rays = np.stack([(u - k['cx']) / k['fx'], (v - k['cy']) / k['fy'], np.ones_like(u)], -1)
    world = rays @ pose[:3, :3].T
    origin = pose[:3, 3]
    best = np.full((h, w), np.inf)
    colour = np.zeros((h, w, 3))

    def plane(axis, value, bounds, paint):
        d = world[..., axis]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (value - origin[axis]) / d
        hit = origin + world * t[..., None]
        inside = (t > 1e-6) & (t < best)
        for other, (low, high) in bounds.items():
            inside &= (hit[..., other] >= low) & (hit[..., other] <= high)
        best[inside] = t[inside]
        colour[inside] = paint(hit[inside])

    checker = lambda p, size, a, b: np.where(((np.floor(p[:, 0] / size) + np.floor(p[:, 1] / size)) % 2)[:, None] > 0,  # noqa: E731
                                             a, b)
    plane(2, 0., {0: (-3, 3), 1: (-3, 1)}, lambda p: checker(p, .25, [90, 90, 100], [60, 60, 70]))  # Floor.
    plane(1, -1.6, {0: (-3, 3), 2: (0, 3)}, lambda p: np.where(((np.floor(p[:, 2] / .1) % 2) > 0)[:, None],
                                                                [120, 140, 170], [100, 115, 150]))  # Wall.
    plane(2, .75, {0: (-.6, .6), 1: (-1.0, -.3)}, lambda p: checker(p, .1, [215, 200, 160], [150, 120, 80]))  # Table.
    box = {0: (-.1, .1), 1: (-.7, -.5), 2: (.75, .9)}
    for axis, value, shade in ((2, .9, 1.), (1, -.5, .85), (0, -.1, .7), (0, .1, .7), (1, -.7, .6)):
        others = {a: b for a, b in box.items() if a != axis}
        plane(axis, value, others, lambda p, shade=shade: (np.array([200, 50, 50]) * shade
                                                           + 40 * ((np.floor(p.sum(1) / .03) % 2)[:, None])))
    depth = np.where(np.isfinite(best), best, 0.)  # Along the ray with z = 1: the optical depth.
    return np.clip(colour, 0, 255).astype(np.uint8), depth


def write(out, frames=64):
    out = Path(out)
    (out / 'color').mkdir(parents=True, exist_ok=True)
    (out / 'depth').mkdir(exist_ok=True)
    to_color = np.eye(4)
    to_color[:3, :3], to_color[:3, 3] = TO_COLOR['rotation'], TO_COLOR['translation']
    rng = np.random.default_rng(1)
    records = []
    for i in range(frames):
        pan = .55 * math.sin(2 * math.pi * i / frames) + rng.normal(0, .02)
        tilt = -.55 + .2 * math.cos(4 * math.pi * i / frames) + rng.normal(0, .02)
        pose = camera_pose(pan, tilt)
        image, _ = render(pose, COLOR_K)
        _, depth = render(pose @ to_color, DEPTH_K)  # The depth sensor where the registration puts it.
        buffer = io.BytesIO()
        Image.fromarray(image).save(buffer, format='JPEG', quality=92)
        (out / f'color/{i:06d}.jpg').write_bytes(buffer.getvalue())
        raw = np.where((depth > .15) & (depth < 3.), np.round(depth * 1000), 0).astype('<u2')
        (out / f'depth/{i:06d}.u16.z').write_bytes(zlib.compress(raw.tobytes(), 1))
        records.append({'color': f'color/{i:06d}.jpg', 'depth': f'depth/{i:06d}.u16.z', 'pose': pose7(pose)})
    (out / 'scene.json').write_text(json.dumps({
        'schema': 1, 'kind': 'head_scene', 'source': 'synthetic', 'color_intrinsics': COLOR_K,
        'depth_intrinsics': DEPTH_K, 'to_color': TO_COLOR, 'depth_scale': .001, 'frames': records}, indent=1))
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--frames', type=int, default=64)
    args = parser.parse_args()
    print(write(args.out, args.frames))
