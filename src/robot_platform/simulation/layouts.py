"""Randomized task layouts for generating training episodes: each episode draws its own cube, bucket, table placement
and head camera pose from the ranges in a randomization file.

The file (config/devices/simulation/<asset>-randomization.json, kind `sim_layout_randomization`) gives every range with
its `source`: a training-randomization design choice and what it is centred on. Ranges are not measurements. They
spread the layouts around the workcell file's values, so that a policy trained on them does not depend on one table.
The workcell file stays the fixed layout: without a randomization file nothing here is used.

A draw starts from the workcell file's document (simulation.workcell) and, in the frame of the table group as the file
places it (the robot's base frame before the group moves):
- the block: its three edges, each drawn on its own (a cuboid; length along its yaw, width across, height up), a mass
  scaled with its volume from the workcell's mass per volume (a placeholder), its yaw, and its position anywhere in
  the cube region;
- the bucket: its inside length (along its own y, front to back before its yaw) and width (across), its wall height
  above the table, its yaw, and its position anywhere in the bucket region, with its outer walls on the table top and
  clear of the cube by `cube_clearance`. A position that is not is drawn again, up to `position_draws` times.
Then the whole table group (the table, the tags with the head camera calibrated against them, the bucket and the cube)
turns by the drawn table yaw about the middle of the tag rectangle (the table's centre without one) and shifts by the
drawn (x, y) in the base frame: the robot-to-table placement, which nobody has measured. Last the head camera moves
by its own shift (per axis of the frame it is given in) and turns about its own axes, about its calibrated pose.

Whether the arms can do the task in a drawn layout is the planner's question, not this module's: the generator asks
the demonstrator and draws again when it finds no plan (evaluation.generate).
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .workcell import from_document, on_table

KIND = 'sim_layout_randomization'
RANDOMIZED = 'randomized'  # The source prefix of a value a layout drew.
# Every range the file gives: (group, key). Each is {"range": [low, high], "source": "..."}.
RANGES = (('cube', 'length'), ('cube', 'width'), ('cube', 'height'), ('cube', 'yaw_deg'),
          ('bucket', 'inner_length'), ('bucket', 'inner_width'),
          ('bucket', 'height'), ('bucket', 'yaw_deg'), ('table_offset', 'x'), ('table_offset', 'y'),
          ('table_offset', 'yaw_deg'), ('head_camera', 'shift'), ('head_camera', 'turn_deg'))
# Each region is {"value": [x low, x high, y low, y high], "source": "..."} or {"polygon": [[x, y], ...], "source": ...}.
REGIONS = (('cube', 'region'), ('bucket', 'region'))
NUMBERS = (('bucket', 'cube_clearance'), ('draws', 'position_draws'), ('draws', 'plan_draws'))


@dataclass(frozen=True)
class Randomization:
    """A randomization file's ranges (low, high) and regions (polygons, ((x, y), ...)) by `group.key`."""
    name: str
    ranges: dict
    regions: dict
    cube_clearance: float
    position_draws: int
    plan_draws: int
    sources: dict = field(default_factory=dict)
    asset_id: str | None = None
    sha256: str | None = None
    path: str | None = None

    def describe(self):
        return {'name': self.name, 'asset_id': self.asset_id, 'path': self.path, 'sha256': self.sha256,
                'ranges': {key: list(value) for key, value in self.ranges.items()},
                'regions': {key: [list(point) for point in value] for key, value in self.regions.items()},
                'cube_clearance_m': self.cube_clearance, 'position_draws': self.position_draws,
                'plan_draws': self.plan_draws, 'sources': dict(self.sources)}


def _entry(document, group, key):
    entry = (document.get(group) or {}).get(key)
    if not isinstance(entry, dict) or not isinstance(entry.get('source'), str) or not entry['source'].strip():
        raise ValueError(f'randomization {group}.{key}: give it with a named source')
    return entry


def _numbers(values, count):
    if not isinstance(values, list) or len(values) != count or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        raise ValueError(f'expected {count} finite numbers')
    return tuple(float(v) for v in values)


def inside(polygon, x, y):
    """Whether (x, y) lies inside `polygon` ((x, y), ...; even-odd rule)."""
    result, count = False, len(polygon)
    for k in range(count):
        (x0, y0), (x1, y1) = polygon[k], polygon[(k + 1) % count]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            result = not result
    return result


def _region(entry, name):
    """A region's polygon from its rectangle (`value`) or its vertices (`polygon`)."""
    if 'polygon' in entry:
        points = entry['polygon']
        if not isinstance(points, list) or len(points) < 3:
            raise ValueError(f'randomization {name}.polygon: at least three [x, y] vertices')
        try:
            polygon = tuple(_numbers(point, 2) for point in points)
        except ValueError as exc:
            raise ValueError(f'randomization {name}.polygon: {exc}') from None
        area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1])) / 2
        if abs(area) < 1e-6:
            raise ValueError(f'randomization {name}.polygon: no area')
        return polygon
    try:
        x0, x1, y0, y1 = _numbers(entry.get('value'), 4)
    except ValueError as exc:
        raise ValueError(f'randomization {name}.value: {exc}') from None
    if not (x0 < x1 and y0 < y1):
        raise ValueError(f'randomization {name}: [x low, x high, y low, y high]')
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def load_randomization(path):
    """A Randomization from a randomization file (schema 1). Refuses a range without a source, a range whose low end is
    above its high end, sizes that are not positive, a cube that cannot fit its smallest bucket, and empty regions."""
    path = Path(path)
    text = path.read_text(encoding='utf-8')
    document = json.loads(text)
    if not isinstance(document, dict) or document.get('schema') != 1 or document.get('kind') != KIND:
        raise ValueError(f'{path} is not a randomization file (schema 1, kind {KIND})')
    ranges, regions, sources, numbers = {}, {}, {}, {}
    for group, key in RANGES:
        entry = _entry(document, group, key)
        try:
            low, high = _numbers(entry.get('range'), 2)
        except ValueError as exc:
            raise ValueError(f'randomization {group}.{key}.range: {exc}') from None
        if low > high:
            raise ValueError(f'randomization {group}.{key}.range: low above high')
        ranges[f'{group}.{key}'], sources[f'{group}.{key}'] = (low, high), entry['source']
    for group, key in REGIONS:
        entry = _entry(document, group, key)
        regions[f'{group}.{key}'], sources[f'{group}.{key}'] = _region(entry, f'{group}.{key}'), entry['source']
    for group, key in NUMBERS:
        entry = _entry(document, group, key)
        try:
            (value,) = _numbers([entry.get('value')], 1)
        except ValueError as exc:
            raise ValueError(f'randomization {group}.{key}.value: {exc}') from None
        numbers[f'{group}.{key}'], sources[f'{group}.{key}'] = value, entry['source']
    mass = _entry(document, 'cube', 'mass')
    if mass.get('rule') != 'volume':
        raise ValueError('randomization cube.mass: {"rule": "volume", "source": "..."} (the only rule)')
    sources['cube.mass'] = mass['source']
    if min(ranges['cube.length'][0], ranges['cube.width'][0], ranges['cube.height'][0],
           ranges['bucket.inner_length'][0], ranges['bucket.inner_width'][0],
           ranges['bucket.height'][0]) <= 0 or ranges['head_camera.shift'][0] > 0 or \
            ranges['head_camera.turn_deg'][0] > 0:
        raise ValueError('randomization: sizes must be positive, and the camera ranges must hold its calibrated pose')
    if max(ranges[f'cube.{key}'][1] for key in ('length', 'width', 'height')) >= \
            min(ranges['bucket.inner_length'][0], ranges['bucket.inner_width'][0]):
        raise ValueError('randomization: the largest cube must fit inside the smallest bucket')
    if numbers['bucket.cube_clearance'] < 0 or numbers['draws.position_draws'] < 1 or numbers['draws.plan_draws'] < 1 \
            or numbers['draws.position_draws'] % 1 or numbers['draws.plan_draws'] % 1:
        raise ValueError('randomization: the clearance is not negative and the draw caps are whole and positive')
    return Randomization(name=str(document.get('name') or path.stem), asset_id=document.get('asset_id'),
                         ranges=ranges, regions=regions, cube_clearance=numbers['bucket.cube_clearance'],
                         position_draws=int(numbers['draws.position_draws']),
                         plan_draws=int(numbers['draws.plan_draws']), sources=sources,
                         sha256=hashlib.sha256(text.encode('utf-8')).hexdigest(), path=str(path.resolve()))


@dataclass(frozen=True)
class Layout:
    """One drawn layout: its workcell, the cube's start in the base frame ({'x', 'y', 'yaw'}, as the trial grid gives
    one) and what was drawn (`values`, for the episode's metadata)."""
    workcell: object
    cube_start: dict
    values: dict

    def describe(self):
        return json.loads(json.dumps(self.values))


def _rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    if axis == 0:
        return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])
    if axis == 1:
        return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class LayoutSampler:
    """Draws layouts from a workcell file's document and a Randomization (see the module's description)."""

    def __init__(self, workcell_path, randomization):
        self.workcell_path = Path(workcell_path)
        self.document = json.loads(self.workcell_path.read_text(encoding='utf-8'))
        self.base = from_document(self.document, name=self.workcell_path.stem, origin=self.workcell_path)
        self.randomization = randomization if isinstance(randomization, Randomization) else \
            load_randomization(randomization)
        r = self.randomization
        if r.asset_id and self.base.asset_id and r.asset_id != self.base.asset_id:
            raise ValueError(f'randomization {r.name} is for {r.asset_id}, the workcell for {self.base.asset_id}')
        # The table group turns about the middle of the tag rectangle (the table's centre without one).
        rectangle = self.base.fiducials.get('rectangle') if self.base.fiducials else None
        if rectangle:
            (ox, oy), turn = rectangle['origin'], math.pi + rectangle['yaw']
            u, v = rectangle['size'][0] / 2, rectangle['size'][1] / 2
            self.pivot = (ox + math.cos(turn) * u - math.sin(turn) * v, oy + math.sin(turn) * u + math.cos(turn) * v)
        else:
            self.pivot = tuple(self.base.table_center)
        # The workcell's own cube mass per volume, which a drawn cube keeps (a placeholder until one is weighed).
        self.density = self.base.cube_mass / float(np.prod(self.base.block))

    def _uniform(self, rng, key):
        low, high = self.randomization.ranges[key]
        return float(rng.uniform(low, high))

    def _in_region(self, rng, key):
        """A point drawn uniformly over the region: drawn over its bounds until one lies inside it."""
        polygon = self.randomization.regions[key]
        xs, ys = [x for x, _ in polygon], [y for _, y in polygon]
        for _ in range(10_000):
            x, y = float(rng.uniform(min(xs), max(xs))), float(rng.uniform(min(ys), max(ys)))
            if inside(polygon, x, y):
                return x, y
        raise ValueError(f'no point drawn inside the {key} in 10000 tries')

    def draw(self, rng, *, seed=None):
        """One layout from `rng` (a numpy Generator). ValueError if no bucket position is clear of the cube and on the
        table within `position_draws`, which the caller counts as a rejected draw. `seed` is recorded as given."""
        r, base = self.randomization, self.base
        tag = f'{RANDOMIZED} (training layout, {r.name})'
        # In the table group's frame, before it moves.
        block = tuple(self._uniform(rng, f'cube.{key}') for key in ('length', 'width', 'height'))
        cube_yaw = math.radians(self._uniform(rng, 'cube.yaw_deg'))
        cube = self._in_region(rng, 'cube.region')
        footprint = math.hypot(block[0], block[1]) / 2  # The block's footprint's circumscribed circle.
        if not on_table(base, *cube, margin=footprint):
            raise ValueError(f'the cube region reaches off the table: {cube}')
        length, width = self._uniform(rng, 'bucket.inner_length'), self._uniform(rng, 'bucket.inner_width')
        height = self._uniform(rng, 'bucket.height')
        bucket_yaw = math.radians(self._uniform(rng, 'bucket.yaw_deg'))
        wall = base.bucket_wall
        half = (width / 2 + wall, length / 2 + wall)  # The outer walls, across and along.
        c, s = math.cos(bucket_yaw), math.sin(bucket_yaw)
        corners = [(c * u - s * v, s * u + c * v) for u in (-half[0], half[0]) for v in (-half[1], half[1])]
        reach = footprint + r.cube_clearance
        tries = 0
        for tries in range(1, r.position_draws + 1):
            middle = self._in_region(rng, 'bucket.region')
            # The cube against the outer walls, in the bucket's own axes.
            dx, dy = cube[0] - middle[0], cube[1] - middle[1]
            along = (c * dx + s * dy, -s * dx + c * dy)
            gap = math.hypot(max(0., abs(along[0]) - half[0]), max(0., abs(along[1]) - half[1]))
            if gap >= reach and all(on_table(base, middle[0] + x, middle[1] + y) for x, y in corners):
                break
        else:
            raise ValueError(f'no bucket position clear of the cube and on the table in {r.position_draws} draws')
        # The table group's placement relative to the robot.
        shift = (self._uniform(rng, 'table_offset.x'), self._uniform(rng, 'table_offset.y'))
        turn = math.radians(self._uniform(rng, 'table_offset.yaw_deg'))
        # The head camera about its calibrated pose: a shift per axis, then turns about its own x, y and z.
        camera_shift = [self._uniform(rng, 'head_camera.shift') for _ in range(3)]
        camera_turn = [self._uniform(rng, 'head_camera.turn_deg') for _ in range(3)]
        tc, ts = math.cos(turn), math.sin(turn)
        px, py = self.pivot

        def moved(x, y):
            return (px + tc * (x - px) - ts * (y - py) + shift[0], py + ts * (x - px) + tc * (y - py) + shift[1])

        document = copy.deepcopy(self.document)
        document['name'] = f"{base.name} | {r.name} draw {seed if seed is not None else ''}".rstrip()

        def put(group, key, value, source):
            document.setdefault(group, {})[key] = {'value': value, 'source': f'{tag}: {source}'}
        put('table', 'center', list(moved(*base.table_center)), 'the table group moved by the drawn table offset')
        put('table', 'yaw_deg', math.degrees(base.table_yaw + turn), 'the drawn table offset\'s yaw')
        rectangle = document.get('fiducials', {}).get('rectangle')
        if rectangle:
            ox, oy = moved(*base.fiducials['rectangle']['origin'])
            rectangle['robot_offsets'] = {'value': [ox, -oy], 'source': f'{tag}: the tag rectangle moved with the '
                                          f"table group (the file's {rectangle['robot_offsets']['value']}: "
                                          f"{rectangle['robot_offsets']['source']})"}
            rectangle['yaw_deg'] = {'value': rectangle['yaw_deg']['value'] + math.degrees(turn),
                                    'source': f"{tag}: the file's {rectangle['yaw_deg']['value']} degrees turned with "
                                              f'the table group'}
        bucket = document['bucket']
        bucket['frame'] = 'base'
        put('bucket', 'center', list(moved(*middle)), 'drawn in the bucket region, moved with the table group')
        put('bucket', 'yaw_deg', math.degrees(bucket_yaw + turn), 'drawn, turned with the table group')
        put('bucket', 'inner_size', [width, length], 'drawn inside width (across) and length (front to back)')
        put('bucket', 'height', height, 'drawn wall height above the table')
        start = moved(*cube)
        put('cube', 'size', list(block), 'drawn edges: length along its yaw, width across, height up')
        mass = self.density * float(np.prod(block))
        # Still a placeholder (its source says so first): no cube has been weighed.
        document['cube']['mass'] = {'value': mass, 'source': f"placeholder ({tag}): the workcell's cube mass per volume "
                                    f"({base.cube_mass * 1000:.1f} g for {' x '.join(f'{e * 1000:.1f}' for e in base.block)} mm, itself "
                                    f"{base.sources.get('cube.mass', '')}) times the drawn volume; not weighed"}
        put('cube', 'start', [start[0], start[1], cube_yaw + turn], 'drawn in the cube region, moved with the table group')
        region = [moved(x, y) for x, y in r.regions['cube.region']]
        put('cube', 'region', [min(p[0] for p in region), max(p[0] for p in region), min(p[1] for p in region),
                               max(p[1] for p in region)], 'the cube region moved with the table group (its bounds)')
        head = document['cameras']['head']
        pos, xyaxes = head['pos']['value'], head['xyaxes']['value']
        if head.get('frame', 'base') == 'base':
            # Given in the base frame: it moves with the table group here; in a frame on the table it moves with it.
            pos = [*moved(pos[0], pos[1]), pos[2]]
            xyaxes = [v for axis in (xyaxes[:3], xyaxes[3:])
                      for v in (tc * axis[0] - ts * axis[1], ts * axis[0] + tc * axis[1], axis[2])]
        x_axis = np.array(xyaxes[:3], dtype=float)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.array(xyaxes[3:], dtype=float)
        y_axis -= (y_axis @ x_axis) * x_axis
        y_axis /= np.linalg.norm(y_axis)
        frame = np.column_stack([x_axis, y_axis, np.cross(x_axis, y_axis)]) @ (
            _rotation(0, math.radians(camera_turn[0])) @ _rotation(1, math.radians(camera_turn[1]))
            @ _rotation(2, math.radians(camera_turn[2])))
        head['pos'] = {'value': [p + d for p, d in zip(pos, camera_shift)],
                       'source': f"{tag}: the calibrated pose ({head['pos']['source']}) shifted by the drawn offset"}
        head['xyaxes'] = {'value': [*frame[:, 0].tolist(), *frame[:, 1].tolist()],
                          'source': f"{tag}: the calibrated axes ({head['xyaxes']['source']}) turned by the drawn angles"}
        workcell = from_document(document, origin=f'layout draw {seed}')
        values = {
            'seed': seed, 'randomization': {'name': r.name, 'path': r.path, 'sha256': r.sha256},
            'bucket_position_draws': tries,
            'cube': {'size_m': list(block), 'mass_kg': mass, 'yaw_deg': math.degrees(cube_yaw + turn),
                     'start_m': list(start), 'drawn_in_group_frame_m': list(cube)},
            'bucket': {'inner_length_m': length, 'inner_width_m': width, 'height_m': height,
                       'yaw_deg': math.degrees(bucket_yaw + turn), 'center_m': list(workcell.bucket_center),
                       'drawn_in_group_frame': {'center_m': list(middle), 'yaw_deg': math.degrees(bucket_yaw)}},
            'table_offset': {'x_m': shift[0], 'y_m': shift[1], 'yaw_deg': math.degrees(turn), 'pivot_m': list(self.pivot),
                             'table_center_m': list(workcell.table_center)},
            'head_camera': {'shift_m': camera_shift, 'turn_deg': {'x': camera_turn[0], 'y': camera_turn[1],
                                                                  'z': camera_turn[2]},
                            'given_in': head.get('frame', 'base'), 'pos_m': list(workcell.head_camera['pos']),
                            'xyaxes': list(workcell.head_camera['xyaxes'])}}
        return Layout(workcell=workcell, cube_start={'x': start[0], 'y': start[1], 'yaw': cube_yaw + turn},
                      values=values)
