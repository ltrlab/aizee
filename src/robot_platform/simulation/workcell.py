"""The task's table, box and cube: where they sit, how big they are, and what each number rests on.

`SYNTHETIC` is the layout the contact scene was built with (2026-10-16): a 0.28 m table chosen so that a
top grasp reaches it, and a square blue bucket. The frozen contact fixture and the evaluation harness's
reference results belong to it.

A workcell file describes one asset's real workcell. It sits beside the servo calibration in
`config/devices/simulation/`, which robot releases leave out. Every value in it names its source. A value
nobody measured is a placeholder: the scene still uses it, but lists it among its limitations until someone
measures it.

Coordinates are the Minerva model's base frame (urdf/minerva, metres): x toward the robot's left, y
backward (the robot faces -y), z up.

The file places the fiducial tags in the table's own frame (`fiducials.table_frame`): origin at the table top's
near-left outer corner, seen from behind the robot facing the table, x to the right along the near edge, y away from
the robot. With the table square to the robot, table (x, y) is base (c - x, -d - y) on the top, where (c, -d) is that
corner, from the table's centre and size. A tag may instead be given in a measured rectangle on the table top
(`fiducials.rectangle`, itself placed in the table frame), or on one of the bucket's front and back walls, where it
follows the bucket.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path

from .fiducials import CODES, grid, marker_axes

KIND = 'sim_workcell'
PLACEHOLDER = 'placeholder'
COLORS = {'floor': (.17, .21, .24, 1.), 'table': (.46, .34, .23, 1.), 'trim': (.72, .73, .75, 1.),
          'leg': (.3, .3, .32, 1.), 'bucket_bottom': (.10, .35, .64, 1.), 'bucket_wall': (.12, .42, .74, 1.),
          'cube': (.95, .30, .06, 1.), 'fiducial_label': (.90, .90, .88, 1.), 'fiducial_print': (.05, .05, .05, 1.)}
# A wall tag's wall of the bucket, as its side along the bucket's own y axis (+: toward the robot), and the wall faces.
BUCKET_WALLS = {'front': 1, 'back': -1}
WALL_FACES = ('inner', 'outer')
# Named appearances a workcell file may give instead of RGBA values.
APPEARANCE = {'black': (.035, .035, .04, 1.), 'silver': (.72, .73, .75, 1.), 'wood': (.80, .64, .43, 1.),
              'cardboard': (.62, .47, .31, 1.), 'cardboard_inside': (.55, .41, .27, 1.)}
LEG_HALF = .022
LEG_INSET = (.06, .04)  # From the table's x and y edges to a leg's centre.


@dataclass(frozen=True)
class Workcell:
    """Resolved task geometry. Sizes are full sizes unless named half; angles are radians."""
    name: str
    table_top: float
    table_center: tuple
    table_size: tuple
    table_thickness: float
    table_legs: tuple  # ((x, y), ...) leg centres.
    floor_z: float
    trim_width: float  # A border strip around the table top; 0: none.
    bucket_center: tuple
    bucket_yaw: float
    bucket_inner: tuple  # Inner (x, y) before yaw.
    bucket_height: float  # Wall height above the table top.
    bucket_wall: float
    bucket_bottom: float
    cube_edge: float
    cube_mass: float
    cube_friction: tuple
    cube_start: tuple  # (x, y, yaw)
    cube_region: tuple  # ((x low, x high), (y low, y high)) where trials place the cube.
    head_camera: dict  # pos, xyaxes, fovy
    colors: dict = field(default_factory=dict)
    sources: dict = field(default_factory=dict)
    asset_id: str | None = None
    # The fiducial tags, or {} for none: family; square, the black square's edge; label, the label's size (across,
    # along the tape); tag_offset, the tag centre's distance from the label centre toward the label's far end (the top
    # of the code); tags, ((id, x, y, z, yaw, tilt), ...): each tag's centre and its marker axes
    # (fiducials.marker_axes(yaw, tilt)) in the base frame, flat on the table with tilt 0 or upright on a bucket wall;
    # rectangle, {} or the measured rectangle some tags are given in: its size, its origin in the base frame and its
    # turn from square to the robot.
    fiducials: dict = field(default_factory=dict)

    @property
    def cube_half(self):
        return self.cube_edge / 2

    def fiducial_label(self, tag):
        """A tag's label: its centre (x, y, z), its half size (across, along) and its marker axes; `tag` is one of
        fiducials['tags']."""
        f, (_, x, y, z, yaw, tilt) = self.fiducials, tag
        axes = marker_axes(yaw, tilt)
        centre = tuple(p - f['tag_offset'] * a for p, a in zip((x, y, z), axes[1]))
        return centre, (f['label'][0] / 2, f['label'][1] / 2), axes

    def placeholders(self):
        """The values nobody has measured yet, by name."""
        return sorted(key for key, source in self.sources.items() if str(source).startswith(PLACEHOLDER))

    def color(self, key):
        return tuple(self.colors.get(key, COLORS[key]))

    def describe(self):
        """A JSON-safe copy, for capabilities, episode metadata and the scene's worker processes."""
        values = asdict(self)
        return json.loads(json.dumps(values))

    @classmethod
    def from_description(cls, values):
        values = dict(values)
        for key in ('table_center', 'table_size', 'bucket_center', 'bucket_inner', 'cube_friction', 'cube_start'):
            values[key] = tuple(float(v) for v in values[key])
        values['table_legs'] = tuple(tuple(float(v) for v in leg) for leg in values['table_legs'])
        values['cube_region'] = tuple(tuple(float(v) for v in span) for span in values['cube_region'])
        values['colors'] = {key: tuple(float(v) for v in rgba) for key, rgba in values.get('colors', {}).items()}
        values['head_camera'] = {'pos': tuple(float(v) for v in values['head_camera']['pos']),
                                 'xyaxes': tuple(float(v) for v in values['head_camera']['xyaxes']),
                                 'fovy': float(values['head_camera']['fovy'])}
        fiducials = values.get('fiducials') or {}
        rectangle = fiducials.get('rectangle') or {}
        values['fiducials'] = {
            'family': str(fiducials['family']), 'square': float(fiducials['square']),
            'label': tuple(float(v) for v in fiducials['label']), 'tag_offset': float(fiducials['tag_offset']),
            'tags': tuple((int(tag[0]), *(float(v) for v in tag[1:])) for tag in fiducials['tags']),
            'rectangle': {'size': tuple(float(v) for v in rectangle['size']),
                          'origin': tuple(float(v) for v in rectangle['origin']), 'yaw': float(rectangle['yaw'])}
            if rectangle else {}} if fiducials else {}
        workcell = cls(**values)
        validate(workcell)
        return workcell


SYNTHETIC = Workcell(
    name='synthetic_v1', table_top=.28, table_center=(0., -.38), table_size=(.96, .52), table_thickness=.05,
    table_legs=((-.42, -.60), (-.42, -.16), (.42, -.60), (.42, -.16)), floor_z=0., trim_width=0.,
    bucket_center=(-.20, -.40), bucket_yaw=0., bucket_inner=(.17, .17), bucket_height=.08, bucket_wall=.012,
    bucket_bottom=.014, cube_edge=.036, cube_mass=.035, cube_friction=(1., .005, .0001),
    cube_start=(.18, -.34, 0.), cube_region=((.10, .26), (-.42, -.26)),
    head_camera={'pos': (0., -.18, 1.10), 'xyaxes': (1., 0., 0., 0., .975, -.222), 'fovy': 75.},
    sources={'layout': 'synthetic task settings chosen for top-grasp reach; not a measured workcell'})


def validate(workcell):
    """Refuse a workcell the scene cannot build or judge."""
    w = workcell
    numbers = [w.table_top, w.table_thickness, w.floor_z, w.trim_width, w.bucket_yaw, w.bucket_height, w.bucket_wall,
               w.bucket_bottom, w.cube_edge, w.cube_mass, *w.table_center, *w.table_size, *w.bucket_center,
               *w.bucket_inner, *w.cube_friction, *w.cube_start, *(v for span in w.cube_region for v in span),
               *(v for leg in w.table_legs for v in leg), *w.head_camera['pos'], *w.head_camera['xyaxes'],
               w.head_camera['fovy']]
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers):
        raise ValueError(f'workcell {w.name}: every value must be a finite number')
    if min(*w.table_size, w.table_thickness, *w.bucket_inner, w.bucket_height, w.bucket_wall, w.bucket_bottom,
           w.cube_edge, w.cube_mass) <= 0:
        raise ValueError(f'workcell {w.name}: sizes, thicknesses and the cube mass must be positive')
    if w.floor_z >= w.table_top - w.table_thickness:
        raise ValueError(f'workcell {w.name}: the floor must lie below the table')
    if w.trim_width < 0 or 2 * w.trim_width >= min(w.table_size):
        raise ValueError(f'workcell {w.name}: the trim must be a narrow border')
    if min(w.bucket_inner) <= w.cube_edge:
        raise ValueError(f'workcell {w.name}: the cube must fit inside the bucket')
    if w.bucket_bottom >= w.bucket_height:
        raise ValueError(f'workcell {w.name}: the bucket bottom must be thinner than its walls are tall')
    (x0, x1), (y0, y1) = w.cube_region
    if not (x0 < x1 and y0 < y1):
        raise ValueError(f'workcell {w.name}: the cube region is (x low, x high), (y low, y high)')
    if not 0 < w.head_camera['fovy'] < 180 or len(w.head_camera['pos']) != 3 or len(w.head_camera['xyaxes']) != 6:
        raise ValueError(f'workcell {w.name}: the head camera needs pos (3), xyaxes (6) and fovy in (0, 180)')
    if not 0 < w.cube_friction[0]:
        raise ValueError(f'workcell {w.name}: the cube needs sliding friction')
    if w.fiducials:
        _validate_fiducials(w)
    return workcell


def _validate_fiducials(w):
    f = w.fiducials
    if f.get('family') not in CODES:
        raise ValueError(f"workcell {w.name}: no codes for fiducial family {f.get('family')!r}; known: {sorted(CODES)}")
    tags = f['tags']
    if not tags or any(len(tag) != 6 for tag in tags):
        raise ValueError(f'workcell {w.name}: each fiducial tag is (id, x, y, z, yaw, tilt)')
    numbers = [f['square'], *f['label'], f['tag_offset'], *(v for tag in tags for v in tag[1:])]
    if len(f['label']) != 2 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                                       for v in numbers):
        raise ValueError(f'workcell {w.name}: every fiducial value must be a finite number')
    ids = [tag[0] for tag in tags]
    if len(set(ids)) != len(ids) or not set(ids) <= set(CODES[f['family']]):
        raise ValueError(f"workcell {w.name}: fiducial IDs must be distinct, from {sorted(CODES[f['family']])}")
    cell = f['square'] / grid(f['family'])
    if f['square'] <= 0 or f['label'][0] < f['square'] + 2 * cell or \
            f['label'][1] / 2 < abs(f['tag_offset']) + f['square'] / 2 + cell:
        raise ValueError(f'workcell {w.name}: a fiducial label must hold its black square and a one-cell white margin')
    (cx, cy), (sx, sy) = w.table_center, w.table_size
    for tag in tags:
        centre, (across, along), (x_axis, y_axis, _) = w.fiducial_label(tag)
        # The label's corners, each half its size along its own axes from its centre.
        points = [[p + u * a + v * b for p, a, b in zip(centre, x_axis, y_axis)]
                  for u in (-across, across) for v in (-along, along)]
        if tag[5] == 0:
            if abs(tag[3] - w.table_top) > 1e-9 or any(abs(x - cx) > sx / 2 + 1e-9 or abs(y - cy) > sy / 2 + 1e-9
                                                        for x, y, _ in points):
                raise ValueError(f'workcell {w.name}: fiducial {tag[0]} reaches off the table top')
            continue
        # An upright tag sits on a face of the bucket's front or back wall, facing out of it, within the wall.
        along_x, along_y = bucket_frame(w, tag[1], tag[2])
        half_x, half_y = w.bucket_inner[0] / 2, w.bucket_inner[1] / 2
        outer = abs(abs(along_y) - (half_y + w.bucket_wall)) < 1e-9
        normal = bucket_frame(w, w.bucket_center[0] + marker_axes(tag[4], tag[5])[2][0],
                              w.bucket_center[1] + marker_axes(tag[4], tag[5])[2][1])
        facing = (1 if outer else -1) * math.copysign(1, along_y)  # Out of the face, along the bucket's y.
        wall_points = [bucket_frame(w, x, y) + (z,) for x, y, z in points]
        if not (outer or abs(abs(along_y) - half_y) < 1e-9) or abs(normal[0]) > 1e-9 or \
                abs(normal[1] - facing) > 1e-9 or abs(marker_axes(tag[4], tag[5])[2][2]) > 1e-9 or \
                any(abs(x) > half_x + (w.bucket_wall if outer else 0.) + 1e-9 or
                    not w.table_top - 1e-9 <= z <= w.table_top + w.bucket_height + 1e-9 for x, _, z in wall_points):
            raise ValueError(f'workcell {w.name}: fiducial {tag[0]} is not wholly on a face of the bucket wall')


def workcell_path(servo_path, asset_id):
    """An asset's workcell file: beside the servo calibration, in `simulation/`, which robot releases leave out."""
    return Path(servo_path).resolve().parent / 'simulation' / f'{asset_id}-workcell.json'


def _value(group, key, sources, *, length=None):
    """The value at `key` (dotted; its last part names the entry in `group`), with its source recorded."""
    entry = group.get(key.rsplit('.', 1)[-1]) if isinstance(group, dict) else None
    if not isinstance(entry, dict) or 'value' not in entry or not isinstance(entry.get('source'), str) or \
            not entry['source'].strip():
        raise ValueError(f'workcell {key}: each value is {{"value": ..., "source": "..."}} with a named source')
    value = entry['value']
    if length is not None and (not isinstance(value, list) or len(value) != length):
        raise ValueError(f'workcell {key}: expected {length} numbers')
    sources[key] = entry['source']
    return tuple(float(v) for v in value) if isinstance(value, list) else float(value)


def _rgba(name, key):
    if isinstance(name, str):
        if name not in APPEARANCE:
            raise ValueError(f'workcell {key}: unknown appearance {name!r}; use one of {sorted(APPEARANCE)} or RGBA')
        return APPEARANCE[name]
    rgba = tuple(float(v) for v in name)
    if len(rgba) != 4 or not all(0 <= v <= 1 for v in rgba):
        raise ValueError(f'workcell {key}: an RGBA colour is four numbers in [0, 1]')
    return rgba


def _word(group, key, sources, choices):
    """A named choice at `key` (dotted, as _value), {"value": "<one of choices>", "source": "..."}."""
    entry = group.get(key.rsplit('.', 1)[-1]) if isinstance(group, dict) else None
    if not isinstance(entry, dict) or entry.get('value') not in choices or not isinstance(entry.get('source'), str) or \
            not entry['source'].strip():
        raise ValueError(f'workcell {key}: {{"value": one of {sorted(choices)}, "source": "..."}}')
    sources[key] = entry['source']
    return entry['value']


def _frames(group, sources, table_center, table_size):
    """The frames on the table top that tags may be given in, as (x, y, yaw) of each origin and x
    axis in the base frame (z up, the origin on the table top): the table frame, and the measured rectangle when the
    fiducials have one (`fiducials.rectangle`). The rectangle's place relative to the robot is one entry,
    `robot_offsets` (c, d): its origin is base (c, -d), its x axis along base -x turned by `yaw_deg`. Returns the
    frames and the rectangle's description ({} without one)."""
    frames = {'table': (table_center[0] + table_size[0] / 2, table_center[1] + table_size[1] / 2, math.pi)}
    if not isinstance(group, dict) or 'rectangle' not in group:
        return frames, {}
    r = group['rectangle']
    c, d = _value(r, 'fiducials.rectangle.robot_offsets', sources, length=2)
    rectangle = {'size': _value(r, 'fiducials.rectangle.size', sources, length=2), 'origin': (c, -d),
                 'yaw': math.radians(_value(r, 'fiducials.rectangle.yaw_deg', sources))}
    frames['rectangle'] = (c, -d, math.pi + rectangle['yaw'])
    return frames, rectangle


def _in_base(frame, point):
    """The base-frame (x, y) of a point (x, y, ...) in a frame on the table."""
    ox, oy, turn = frame
    c, s = math.cos(turn), math.sin(turn)
    return ox + c * point[0] - s * point[1], oy + s * point[0] + c * point[1]


def _fiducials(group, sources, top, frames, rectangle, bucket):
    """The file's fiducial tags, resolved into the base frame (Workcell.fiducials), or {} for none.

    A tag on the table top has a `center` in the table frame, or in another frame of _frames (`"frame": "rectangle"`).
    Its label lies flat with its text end toward the robot and its edges square to its frame, so its marker frame is
    that frame's orientation. A tag on the bucket gives its `wall` (front, toward the robot, or back), the wall's `face`
    (inner or outer), its centre `along` the wall from the wall's middle toward the robot's left, and `below_rim`: the
    code upright, its top toward the rim, facing out of that face. It follows the bucket's pose."""
    if group is None:
        return {}
    family = group.get('family')
    if not isinstance(family, dict) or not isinstance(family.get('value'), str) or \
            not isinstance(family.get('source'), str) or not family['source'].strip():
        raise ValueError('workcell fiducials.family: {"value": "<OpenCV dictionary name>", "source": "..."}')
    sources['fiducials.family'] = family['source']
    square = _value(group, 'fiducials.square', sources)
    label = _value(group, 'fiducials.label_size', sources, length=2)
    offset = _value(group, 'fiducials.tag_offset', sources)
    tags = []
    for key, entry in sorted(group.get('tags', {}).items(), key=lambda item: int(item[0])):
        prefix = f'fiducials.tags.{key}'
        if not isinstance(entry, dict) or ('center' in entry) == ('wall' in entry):
            raise ValueError(f'workcell {prefix}: give either a center on the table or a bucket wall')
        if 'center' in entry:
            if entry.get('frame', 'table') not in frames:
                raise ValueError(f"workcell {prefix}.frame: one of {sorted(frames)}")
            frame = frames[entry.get('frame', 'table')]
            x, y = _in_base(frame, _value(entry, f'{prefix}.center', sources, length=2))
            tags.append((int(key), x, y, top, frame[2], 0.))
            continue
        side = BUCKET_WALLS[_word(entry, f'{prefix}.wall', sources, BUCKET_WALLS)]
        outer = _word(entry, f'{prefix}.face', sources, WALL_FACES) == 'outer'
        along = _value(entry, f'{prefix}.along', sources)
        below = _value(entry, f'{prefix}.below_rim', sources)
        # In the bucket's own axes: the face's y, and the way out of it.
        face_y = side * (bucket['inner'][1] / 2 + (bucket['wall'] if outer else 0.))
        out = side if outer else -side
        (bx, by), yaw = bucket['center'], bucket['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        normal = (-s * out, c * out)
        # Upright, the top of the code up: marker z is the face's normal (fiducials.marker_axes with a quarter tilt).
        tags.append((int(key), bx + c * along - s * face_y, by + s * along + c * face_y,
                     top + bucket['height'] - below, math.atan2(normal[0], -normal[1]), math.pi / 2))
    return {'family': family['value'], 'square': square, 'label': label, 'tag_offset': offset, 'tags': tuple(tags),
            'rectangle': rectangle}


def load_workcell(path):
    """A Workcell from a workcell file (schema 1)."""
    path = Path(path)
    document = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema') != 1 or document.get('kind') != KIND:
        raise ValueError(f'{path} is not a workcell file (schema 1, kind {KIND})')
    sources = {}
    table, bucket, cube = document['table'], document['bucket'], document['cube']
    top = _value(table, 'table.top_z', sources)
    center = _value(table, 'table.center', sources, length=2)
    size = _value(table, 'table.size', sources, length=2)
    thickness = _value(table, 'table.thickness', sources)
    floor = _value(table, 'table.floor_z', sources)
    trim = _value(table, 'table.trim_width', sources)
    inset = LEG_INSET
    legs = tuple((center[0] + sx * (size[0] / 2 - inset[0]), center[1] + sy * (size[1] / 2 - inset[1]))
                 for sx in (-1, 1) for sy in (-1, 1))
    head = document['cameras']['head']
    camera = {'pos': _value(head, 'cameras.head.pos', sources, length=3),
              'xyaxes': _value(head, 'cameras.head.xyaxes', sources, length=6),
              'fovy': _value(head, 'cameras.head.fovy', sources)}
    frames, rectangle = _frames(document.get('fiducials'), sources, center, size)
    region = _value(cube, 'cube.region', sources, length=4)
    appearance = document.get('appearance', {})
    colors = {key: _rgba(value, f'appearance.{key}') for key, value in appearance.items()}
    unknown = set(colors) - set(COLORS)
    if unknown:
        raise ValueError(f'workcell appearance: unknown surfaces {sorted(unknown)}')
    # The bucket's centre is in the base frame, or (`"frame": "rectangle"`) in a frame on the table with its yaw from
    # that frame's square, so that it moves with that frame.
    frame = bucket.get('frame', 'base')
    if frame != 'base' and frame not in frames:
        raise ValueError(f"workcell bucket.frame: base or one of {sorted(frames)}")
    middle = _value(bucket, 'bucket.center', sources, length=2)
    turn = math.radians(_value(bucket, 'bucket.yaw_deg', sources))
    if frame != 'base':
        middle, turn = _in_base(frames[frame], middle), turn + frames[frame][2] - math.pi
    box = dict(center=middle, yaw=turn,
               inner=_value(bucket, 'bucket.inner_size', sources, length=2),
               height=_value(bucket, 'bucket.height', sources), wall=_value(bucket, 'bucket.wall', sources),
               bottom=_value(bucket, 'bucket.bottom', sources))
    workcell = Workcell(
        name=str(document.get('name') or path.stem), asset_id=document.get('asset_id'),
        table_top=top, table_center=center, table_size=size, table_thickness=thickness, table_legs=legs,
        floor_z=floor, trim_width=trim,
        bucket_center=box['center'], bucket_yaw=box['yaw'], bucket_inner=box['inner'], bucket_height=box['height'],
        bucket_wall=box['wall'], bucket_bottom=box['bottom'],
        cube_edge=_value(cube, 'cube.edge', sources), cube_mass=_value(cube, 'cube.mass', sources),
        cube_friction=_value(cube, 'cube.friction', sources, length=3),
        cube_start=_value(cube, 'cube.start', sources, length=3),
        cube_region=(region[0:2], region[2:4]),
        head_camera=camera, colors=colors, sources=sources,
        fiducials=_fiducials(document.get('fiducials'), sources, top, frames, rectangle, box))
    return validate(workcell)


def resolve(workcell):
    """A Workcell from None (SYNTHETIC), a Workcell, its description, 'synthetic' or a workcell file's path."""
    if workcell is None or workcell == 'synthetic':
        return SYNTHETIC
    if isinstance(workcell, Workcell):
        return validate(workcell)
    if isinstance(workcell, dict):
        return Workcell.from_description(workcell)
    return load_workcell(workcell)


def bucket_frame(workcell, x, y):
    """(along x, along y) of a world (x, y) in the bucket's own axes."""
    dx, dy = x - workcell.bucket_center[0], y - workcell.bucket_center[1]
    c, s = math.cos(workcell.bucket_yaw), math.sin(workcell.bucket_yaw)
    return c * dx + s * dy, -s * dx + c * dy
