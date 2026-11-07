"""The room around the task scene, and how the scene is lit and finished: the user's look of 2026-10-18.

The user (2026-10-18): "The whole robot shouldn't be pitch black, make it a rather dark gray, and let's make the
background something like those game scenes with the blurred infinite distance looking background, and this one perhaps
of a normal house room with warm lighting and grayish walls and a wooden floor."

So the scene draws:
- the robot dark grey (LOOK['robot_grey']), with a little specular so that its edges read against the black table;
- a house room as its skybox: grey walls with a warm tint, a wooden floor of planks, a baseboard, a window's light with
  curtains, a framed picture, a doorway, a couch and a bookshelf, lit warm by a ceiling light and a lamp, and blurred
  so that it reads as out of focus and infinitely far (`backdrop` paints the six faces; they are committed beside
  this module, in room/, and `python -m robot_platform.simulation.room` writes them again, the same for a seed);
- its floor plane in the same wood (room/floor.png);
- warm lights (LOOK['light_kelvin']), at the scene's earlier strength.

A workcell file may give its own look (`look`), and a training randomization file may vary it per layout (wall tone,
floor tone, light warmth and the robot's grey; simulation.layouts). Nothing here is measured.
"""

from __future__ import annotations

import functools
import hashlib
import math
from pathlib import Path

import numpy as np

# The user's look. robot_grey: the robot's colour, each channel; room_tone and floor_tone: factors on the backdrop and
# on the floor plane's wood (1: as painted); light_kelvin: the colour of the scene's lights.
LOOK = {'robot_grey': 0.22, 'room_tone': (1.0, 1.0, 1.0), 'floor_tone': (1.0, 1.0, 1.0), 'light_kelvin': 4500.0}
LOOK_SOURCE = (
    'user 2026-10-18: "make it a rather dark gray", "a normal house room with warm lighting and grayish '
    'walls and a wooden floor"; the values are a design choice, not measured'
)
# The robot's finish beside its grey: a little specular, so that its edges read; no reflection.
ROBOT_SPECULAR, ROBOT_SHININESS = 0.3, 0.25
FACES = ('right', 'left', 'up', 'down', 'front', 'back')  # MuJoCo's skybox files: +x, -x, +z, -z, -y, +y
ASSETS = Path(__file__).with_name('room')
SEED, SIZE, BLUR_PX = 20261018, 512, 11.0
FLOOR_SIZE = 1024  # the floor plane's wood, over its 4 x 4 m
# The painted room, in the base frame (x to the robot's left, y backward, z up; metres): the eye is the head camera's
# place on the production workcell, the floor that workcell's.
EYE = np.array([0.05, -0.08, 1.07])
FLOOR_Z, CEILING = -0.238, 2.7
ROOM = {'x': (-2.9, 3.1), 'y': (-4.3, 2.2), 'z': (FLOOR_Z, FLOOR_Z + CEILING)}
WALL, BASEBOARD, CEILING_PAINT = (
    np.array([0.60, 0.575, 0.54]),
    np.array([0.80, 0.78, 0.74]),
    np.array([0.68, 0.665, 0.64]),
)
WOOD = np.array([0.41, 0.295, 0.205])
# MuJoCo's horizon haze on the floor plane (visual/rgba haze, visual/map haze): the backdrop's colour where its floor meets
# its walls, so that the plane's far edge fades into it.
HAZE, HAZE_SPAN = (0.31, 0.255, 0.19), 0.3


def kelvin_rgb(kelvin):
    """A light of colour temperature `kelvin` as RGB of luma 1 (Tanner Helland's fit of the black-body colours), so that
    a warmer light is no brighter."""
    t = kelvin / 100.0
    red = 255.0 if t <= 66 else 329.698727446 * (t - 60) ** -0.1332047592
    green = 99.4708025861 * math.log(t) - 161.1195681661 if t <= 66 else 288.1221695283 * (t - 60) ** -0.0755148492
    blue = 255.0 if t >= 66 else 0.0 if t <= 19 else 138.5177312231 * math.log(t - 10) - 305.0447927307
    rgb = np.clip(np.array([red, green, blue]) / 255.0, 0, 1)
    return rgb / float(rgb @ (0.2126, 0.7152, 0.0722))


def face_directions(face, size, margin=0):
    """Each pixel's unit direction (rows, cols, 3) of one skybox face as MuJoCo shows it, with `margin` extra pixels
    around it. Read off renders (MuJoCo 3.15, 2026-10-18): looking along +x the face 'right' fills the view mirrored
    left to right, and so on for the other sides; 'up' and 'down' have their top row toward +y and -y."""
    index = (np.arange(-margin, size + margin) + 0.5) / size
    r, c = np.meshgrid(index, index, indexing='ij')
    one = np.ones_like(r)
    u, v = 2 * c - 1, 1 - 2 * r
    d = {
        'right': (one, u, v),
        'left': (-one, -u, v),
        'up': (u, v, one),
        'down': (u, -v, -one),
        'front': (u, -one, v),
        'back': (-u, one, v),
    }[face]
    d = np.stack(d, axis=-1)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def _box_blur(image, radius, axis):
    if radius < 1:
        return image
    pad = [(0, 0)] * image.ndim
    pad[axis] = (radius + 1, radius)
    total = np.cumsum(np.pad(image, pad, mode='edge'), axis=axis)
    upper = np.take(total, np.arange(2 * radius + 1, total.shape[axis]), axis=axis)
    lower = np.take(total, np.arange(0, total.shape[axis] - 2 * radius - 1), axis=axis)
    return (upper - lower) / (2 * radius + 1)


def blur(image, sigma):
    """A Gaussian blur of `sigma` pixels, as three box blurs each way."""
    radius = int(round((math.sqrt(12 * sigma * sigma / 3 + 1) - 1) / 2))
    for axis in (0, 1):
        for _ in range(3):
            image = _box_blur(image, radius, axis)
    return image


def _noise(rng, shape, scale):
    """Smooth value noise in [0, 1] on a grid `scale` cells across."""
    coarse = rng.random((scale + 3, scale + 3))
    y = np.linspace(1, scale + 1, shape[0])
    x = np.linspace(1, scale + 1, shape[1])
    y0, x0 = np.floor(y).astype(int), np.floor(x).astype(int)
    fy, fx = (y - y0)[:, None], (x - x0)[None, :]
    fy, fx = fy * fy * (3 - 2 * fy), fx * fx * (3 - 2 * fx)
    a, b = coarse[y0][:, x0], coarse[y0][:, x0 + 1]
    c, d = coarse[y0 + 1][:, x0], coarse[y0 + 1][:, x0 + 1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


class _Planks:
    """The floor's boards: 14 cm wide along y, 0.9-1.6 m long, each its own shade and grain."""

    def __init__(self, rng):
        self.width = 0.14
        self.shift = rng.uniform(0, 1.6, 400)
        self.length = rng.uniform(0.9, 1.6, 400)
        self.shade = rng.uniform(0.82, 1.14, (400, 40))
        self.hue = rng.uniform(-0.04, 0.04, (400, 40))
        self.phase = rng.uniform(0, 2 * math.pi, (400, 40))

    def albedo(self, x, y):
        column = np.floor((x + 10.0) / self.width).astype(int) % 400
        across = (x + 10.0) / self.width - np.floor((x + 10.0) / self.width)
        along = y + 10.0 + self.shift[column]
        board = np.floor(along / self.length[column]).astype(int) % 40
        within = along / self.length[column] - np.floor(along / self.length[column])
        shade = self.shade[column, board]
        grain = 0.5 + 0.5 * np.sin(
            2 * math.pi * (across * 5.5 + 0.35 * np.sin(along * 3.1 + self.phase[column, board]))
        )
        streak = 0.5 + 0.5 * np.sin(
            2 * math.pi * (across * 23.0 + 0.2 * np.sin(along * 7.3 + 2 * self.phase[column, board]))
        )
        tone = shade * (0.86 + 0.1 * grain + 0.06 * streak)
        colour = WOOD[None] * tone[..., None]
        colour[..., 0] *= 1 + self.hue[column, board]
        colour[..., 2] *= 1 - self.hue[column, board]
        seam = (across < 0.025) | (across > 0.975) | (within < 0.006 * 1.2 / self.length[column])
        return np.where(seam[..., None], colour * 0.55, colour)


def _paint(directions, planks, rng_values):
    """The room's colour along each direction from EYE: the hit surface's colour, lit."""
    d = directions.reshape(-1, 3)
    n = len(d)
    big = np.full(n, np.inf)
    # The room's walls, floor and ceiling.
    t = big.copy()
    surface = np.zeros(n, int)  # 0 x wall, 1 y wall, 2 floor/ceiling
    for axis in range(3):
        bounds = ROOM['xyz'[axis]]
        with np.errstate(divide='ignore', invalid='ignore'):
            far = np.where(d[:, axis] > 0, (bounds[1] - EYE[axis]) / d[:, axis], (bounds[0] - EYE[axis]) / d[:, axis])
        far = np.where(np.isfinite(far) & (far > 0), far, np.inf)
        closer = far < t
        t = np.where(closer, far, t)
        surface = np.where(closer, axis, surface)
    hit = EYE + d * t[:, None]
    normal = np.zeros((n, 3))
    normal[np.arange(n), surface] = -np.sign(d[np.arange(n), surface])
    kind = np.where(surface == 2, np.where(d[:, 2] < 0, 'floor', 'ceiling'), 'wall').astype(object)
    # Furniture: axis-aligned boxes in the room.
    x0, x1 = ROOM['x']
    y0, y1 = ROOM['y']
    f = FLOOR_Z
    boxes = [
        ((-1.95, y0, f), (0.35, y0 + 0.92, f + 0.44), 'couch'),  # seat
        ((-1.95, y0, f), (0.35, y0 + 0.26, f + 0.86), 'couch'),  # back
        ((-2.15, y0, f), (-1.95, y0 + 0.92, f + 0.62), 'couch'),  # arms
        ((0.35, y0, f), (0.55, y0 + 0.92, f + 0.62), 'couch'),
        ((x0, -3.6, f), (x0 + 0.34, -2.35, f + 1.95), 'shelf'),
        ((0.85, y0 + 0.2, f), (1.25, y0 + 0.6, f + 0.55), 'side table'),
        ((0.95, y0 + 0.3, f + 1.25), (1.15, y0 + 0.5, f + 1.55), 'lamp'),
    ]
    for low, high, name in boxes:
        low, high = np.array(low), np.array(high)
        with np.errstate(divide='ignore', invalid='ignore'):
            ta = (low - EYE) / d
            tb = (high - EYE) / d
        enter = np.nanmax(np.minimum(ta, tb), axis=1)
        leave = np.nanmin(np.maximum(ta, tb), axis=1)
        inside = (enter < leave) & (enter > 0) & (enter < t)
        t = np.where(inside, enter, t)
        axis = np.argmax(np.minimum(ta, tb), axis=1)
        hit = np.where(inside[:, None], EYE + d * t[:, None], hit)
        fresh = np.zeros((n, 3))
        fresh[np.arange(n), axis] = -np.sign(d[np.arange(n), axis])
        normal = np.where(inside[:, None], fresh, normal)
        kind = np.where(inside, name, kind)
    colour = np.zeros((n, 3))
    emit = np.zeros((n, 3))
    x, y, z = hit[:, 0], hit[:, 1], hit[:, 2]
    height = z - FLOOR_Z
    wall = kind == 'wall'
    colour[wall] = WALL * (0.97 + 0.06 * rng_values['wall'](x[wall] + y[wall], z[wall]))[:, None]
    low = wall & (height < 0.09)
    colour[low] = BASEBOARD
    floor = kind == 'floor'
    colour[floor] = planks.albedo(x[floor], y[floor])
    # A rug under the couch's end of the room.
    rug = floor & (x > -2.3) & (x < 0.6) & (y > y0 + 1.05) & (y < y0 + 2.75)
    border = rug & ((x < -2.15) | (x > 0.45) | (y < y0 + 1.2) | (y > y0 + 2.6))
    colour[rug] = np.array([0.50, 0.42, 0.36]) * (0.92 + 0.1 * rng_values['rug'](x[rug], y[rug]))[:, None]
    colour[border] = np.array([0.36, 0.27, 0.22])
    colour[kind == 'ceiling'] = CEILING_PAINT
    couch = kind == 'couch'
    colour[couch] = (
        np.array([0.37, 0.39, 0.36]) * (0.95 + 0.08 * rng_values['wall'](x[couch] * 3, z[couch] * 3))[:, None]
    )
    shelf = kind == 'shelf'
    colour[shelf] = np.array([0.30, 0.21, 0.14])
    face = shelf & (normal[:, 0] > 0.5) & (height > 0.05) & (height < 1.9)
    row = np.floor(height[face] / 0.38)
    slot = np.floor((y[face] + 10) / 0.045).astype(int)
    books = rng_values['books'][(slot + 37 * row.astype(int)) % len(rng_values['books'])]
    gap = (height[face] - row * 0.38) > 0.3
    colour[face] = np.where(gap[:, None], np.array([0.18, 0.13, 0.09]), books)
    colour[kind == 'side table'] = np.array([0.33, 0.23, 0.15])
    lamp = kind == 'lamp'
    colour[lamp] = np.array([0.95, 0.88, 0.74])
    emit[lamp] = np.array([1.1, 0.85, 0.55])
    # The front wall (y0): a window with curtains, a framed picture. The left wall (x1): a doorway.
    front = wall & (np.abs(y - y0) < 1e-6)
    window = front & (x > 0.75) & (x < 1.95) & (height > 0.95) & (height < 2.15)
    pane = window & (np.abs(x - 1.35) > 0.025) & (np.abs(height - 1.55) > 0.025)
    colour[window] = np.array([0.85, 0.83, 0.80])
    emit[pane] = np.array([0.30, 0.285, 0.26]) * (0.85 + 0.3 * (height[pane] - 0.95))[:, None]
    curtain = front & (height > 0.1) & (height < 2.3) & (((x > 0.45) & (x < 0.78)) | ((x > 1.92) & (x < 2.25)))
    colour[curtain] = np.array([0.62, 0.52, 0.43]) * (0.88 + 0.12 * np.sin(x[curtain] * 60.0))[:, None]
    frame = front & (x > -1.75) & (x < -0.75) & (height > 1.25) & (height < 1.95)
    canvas = frame & (x > -1.70) & (x < -0.80) & (height > 1.30) & (height < 1.90)
    colour[frame] = np.array([0.24, 0.17, 0.11])
    mix = ((x[canvas] + 1.7) / 0.9)[:, None]
    colour[canvas] = (np.array([0.42, 0.52, 0.56]) * (1 - mix) + np.array([0.70, 0.58, 0.40]) * mix) * (
        0.9 + 0.15 * rng_values['rug'](x[canvas] * 4, height[canvas] * 4)
    )[:, None]
    side = wall & (np.abs(x - x1) < 1e-6)
    door = side & (y > -2.55) & (y < -1.65) & (height < 2.1)
    opening = door & (y > -2.48) & (y < -1.72) & (height < 2.03)
    colour[door] = BASEBOARD
    colour[opening] = np.array([0.16, 0.14, 0.12])
    # Light: warm ambient, a ceiling light, the lamp and the window; corners darker.
    light = np.array([0.40, 0.36, 0.31]) * np.ones((n, 1))
    for place, power, reach in (
        (np.array([0.3, -1.4, FLOOR_Z + CEILING - 0.05]), np.array([1.0, 0.86, 0.66]), 2.4),
        (np.array([1.05, y0 + 0.4, FLOOR_Z + 1.4]), np.array([0.9, 0.68, 0.44]), 1.3),
        (np.array([1.35, y0 + 0.3, FLOOR_Z + 1.55]), np.array([0.25, 0.24, 0.22]), 2.0),
    ):
        towards = place - hit
        distance = np.linalg.norm(towards, axis=1)
        cosine = np.clip(np.sum(normal * towards, axis=1) / np.maximum(distance, 1e-9), 0, 1)
        light += power[None] * (cosine / (1 + (distance / reach) ** 2))[:, None]
    edges = []
    for axis in range(3):
        bounds = ROOM['xyz'[axis]]
        edges.append(np.minimum(np.abs(hit[:, axis] - bounds[0]), np.abs(hit[:, axis] - bounds[1])))
    edges = np.sort(np.stack(edges, axis=1), axis=1)[:, 1]  # the distance to the nearest other surface
    shade = 1 - 0.3 * np.exp(-edges / 0.35)
    glow = (kind == 'ceiling') & (np.hypot(x - 0.3, y + 1.4) < 0.15)
    emit[glow] = np.array([0.85, 0.76, 0.6])
    result = colour * light * shade[:, None] + emit
    return result.reshape(directions.shape)


def backdrop(seed=SEED, size=SIZE, sigma=BLUR_PX):
    """The six skybox faces {name: uint8 (size, size, 3)}: the room painted from EYE, then blurred by `sigma` pixels.
    Each face is painted with a margin of the neighbouring faces, so the blur runs across their seams."""
    rng = np.random.default_rng(seed)
    planks = _Planks(rng)
    fields = {name: _noise(rng, (64, 64), 6) for name in ('wall', 'rug')}

    def sampler(field):
        def value(a, b):
            i = (np.floor(np.abs(a) * 9).astype(int)) % 64
            j = (np.floor(np.abs(b) * 9).astype(int)) % 64
            return field[i, j]

        return value

    values = {name: sampler(field) for name, field in fields.items()}
    values['books'] = rng.uniform(0.15, 0.55, (97, 3)) * np.array([1.0, 0.85, 0.75])
    margin = int(math.ceil(3 * sigma))
    faces = {}
    for face in FACES:
        painted = _paint(face_directions(face, size, margin), planks, values)
        soft = blur(painted, sigma)[margin : margin + size, margin : margin + size]
        faces[face] = np.clip(np.round(soft * 255), 0, 255).astype(np.uint8)
    return faces


def floor_texture(seed=SEED, size=FLOOR_SIZE, half=2.0):
    """The floor plane's wood (uint8 (size, size, 3)) over its +/-`half` m, unlit (the scene's lights light it): the
    backdrop's planks, a little blurred so the plane does not alias."""
    rng = np.random.default_rng(seed)
    planks = _Planks(rng)
    index = (np.arange(size) + 0.5) / size * 2 * half - half
    y, x = np.meshgrid(-index, index, indexing='ij')  # rows from +y down to -y, columns from -x to +x
    albedo = blur(planks.albedo(x, y), 1.2)
    return np.clip(np.round(albedo * 255), 0, 255).astype(np.uint8)


@functools.lru_cache(maxsize=1)
def provenance():
    """Where the committed backdrop and floor come from: this module's painting, its seed and size, and each file's
    sha256."""
    files = [f'{face}.png' for face in FACES] + ['floor.png']
    return {
        'painted_by': 'robot_platform.simulation.room',
        'seed': SEED,
        'face_px': SIZE,
        'blur_px': BLUR_PX,
        'floor_px': FLOOR_SIZE,
        'source': LOOK_SOURCE,
        'sha256': {name: hashlib.sha256((ASSETS / name).read_bytes()).hexdigest() for name in files},
    }


def write(directory=ASSETS, seed=SEED):
    """Write the backdrop's faces (<face>.png) and the floor's wood (floor.png) into `directory`."""
    from PIL import Image

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = []
    for face, pixels in backdrop(seed).items():
        paths.append(directory / f'{face}.png')
        Image.fromarray(pixels).save(paths[-1], optimize=True)
    paths.append(directory / 'floor.png')
    Image.fromarray(floor_texture(seed)).save(paths[-1], optimize=True)
    return paths


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Paint the task scene's room backdrop and floor (room/).")
    parser.add_argument('--out', default=str(ASSETS))
    parser.add_argument('--seed', type=int, default=SEED)
    args = parser.parse_args()
    for path in write(args.out, args.seed):
        print(path)


if __name__ == '__main__':
    main()
