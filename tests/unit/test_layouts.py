"""Randomized training layouts: each draw is reproducible from its seed, stays inside the file's ranges and regions,
keeps the bucket clear of the cube and on the table, and moves the table group rigidly."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from robot_platform.simulation.layouts import RANDOMIZED, LayoutSampler, _rotation, inside, load_randomization
from robot_platform.simulation.workcell import bucket_frame, load_workcell, on_table, table_frame

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
RANDOMIZATION = ROOT / 'config/devices/simulation/minerva-production-01-randomization.json'


@pytest.fixture(scope='module')
def sampler():
    return LayoutSampler(WORKCELL, RANDOMIZATION)


@pytest.fixture(scope='module')
def draws(sampler):
    layouts = []
    for k in range(200):
        try:
            layouts.append(sampler.draw(np.random.default_rng([7, k]), seed=[7, k]))
        except ValueError:
            pass  # No bucket place clear of the cube: the generator counts it as a rejected draw.
    return layouts


def _turn(point, angle, pivot):
    c, s = math.cos(angle), math.sin(angle)
    x, y = point[0] - pivot[0], point[1] - pivot[1]
    return pivot[0] + c * x - s * y, pivot[1] + s * x + c * y


def test_every_range_names_its_source_as_a_training_design_choice():
    randomization = load_randomization(RANDOMIZATION)
    assert set(randomization.ranges) >= {'cube.length', 'cube.width', 'cube.height', 'cube.yaw_deg',
                                         'bucket.inner_length', 'bucket.inner_width',
                                         'bucket.height', 'bucket.yaw_deg', 'table_offset.x', 'table_offset.y',
                                         'table_offset.yaw_deg', 'head_camera.shift', 'head_camera.turn_deg'}
    for key in [*randomization.ranges, *randomization.regions]:
        source = randomization.sources[key]
        assert 'training randomization' in source and 'design choice' in source and '2026-10-18' in source, key
        assert 'centred on' in source, key
    # The sizes and the box's turn cite the head camera's depth check they are centred on.
    for key in ('cube.length', 'cube.width', 'cube.height', 'bucket.inner_width', 'bucket.inner_length', 'bucket.height',
                'bucket.yaw_deg'):
        assert "The head camera's depth" in randomization.sources[key], key
    assert randomization.sources['cube.mass'].startswith('placeholder')
    # The ranges re-centred on the depth check (the coordinator, 2026-10-18), the table offset widened to 4 cm around
    # the interim placement.
    assert randomization.ranges['cube.length'] == randomization.ranges['cube.width'] == (.030, .055)
    # Blocks under 35 mm tall are left out (the coordinator, accepted by the user, 2026-10-18): below about 35 mm the
    # left's retreat from the handoff passes within 1 mm of the right's fingers, a planner follow-up.
    assert randomization.ranges['cube.height'] == (.035, .055)
    assert randomization.ranges['bucket.inner_length'] == (.200, .300)
    assert randomization.ranges['bucket.inner_width'] == (.130, .200)
    assert randomization.ranges['bucket.height'] == (.150, .220)
    assert randomization.ranges['bucket.yaw_deg'] == pytest.approx((-19.4 - 30., -19.4 + 30.))
    assert randomization.ranges['table_offset.x'] == randomization.ranges['table_offset.y'] == (-.04, .04)
    assert randomization.ranges['table_offset.yaw_deg'] == (-3., 3.)
    assert randomization.ranges['head_camera.shift'] == (-.02, .02)
    assert randomization.ranges['head_camera.turn_deg'] == (-2., 2.)


@pytest.mark.parametrize('change, refusal', [
    (lambda d: d['cube']['length'].pop('source'), 'named source'),
    (lambda d: d['bucket']['height'].update(range=[.22, .12]), 'low above high'),
    (lambda d: d['cube']['height'].update(range=[.03, .14]), 'fit inside'),
    (lambda d: d['bucket'].update(region={'value': [.1, .1, -.5, -.3], 'source': 'test'}), 'x low, x high'),
    (lambda d: d['cube'].update(region={'polygon': [[0, -.4], [.1, -.4]], 'source': 'test'}), 'three'),
    (lambda d: d['cube'].update(region={'polygon': [[0, -.4], [.1, -.4], [.2, -.4]], 'source': 'test'}), 'no area'),
    (lambda d: d['cube']['mass'].update(rule='fixed'), 'rule'),
    (lambda d: d['head_camera']['shift'].update(range=[.01, .02]), 'calibrated pose'),
])
def test_a_randomization_file_that_cannot_be_drawn_from_is_refused(tmp_path, change, refusal):
    document = json.loads(RANDOMIZATION.read_text(encoding='utf-8'))
    change(document)
    path = tmp_path / 'randomization.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError, match=refusal):
        load_randomization(path)


def test_a_seed_draws_the_same_layout_and_another_seed_another(sampler):
    first = sampler.draw(np.random.default_rng([3, 1]), seed=[3, 1])
    again = sampler.draw(np.random.default_rng([3, 1]), seed=[3, 1])
    other = sampler.draw(np.random.default_rng([3, 2]), seed=[3, 2])
    assert first.workcell == again.workcell and first.values == again.values and first.cube_start == again.cube_start
    assert first.workcell.describe() == again.workcell.describe()
    assert other.values['cube'] != first.values['cube'] and other.values['bucket'] != first.values['bucket']


def test_every_draw_stays_in_its_ranges_and_regions(sampler, draws):
    r, base = sampler.randomization, sampler.base
    assert len(draws) >= 190  # Hardly any draw runs out of clear bucket places.

    def within(value, key, slack=1e-9):
        low, high = r.ranges[key]
        return low - slack <= value <= high + slack
    for layout in draws:
        v, w = layout.values, layout.workcell
        offset = v['table_offset']
        assert within(offset['x_m'], 'table_offset.x') and within(offset['y_m'], 'table_offset.y')
        assert within(offset['yaw_deg'], 'table_offset.yaw_deg')
        # Each edge drawn on its own: a cuboid.
        assert all(within(edge, f'cube.{key}') for edge, key in zip(v['cube']['size_m'], ('length', 'width', 'height')))
        assert w.block == tuple(v['cube']['size_m']) and w.cube_half == w.block[2] / 2
        # The mass keeps the workcell's mass per volume, and stays a placeholder.
        assert w.cube_mass == pytest.approx(base.cube_mass * np.prod(w.block) / np.prod(base.block), rel=1e-12)
        assert 'cube.mass' in w.placeholders()
        assert within(v['cube']['yaw_deg'] - offset['yaw_deg'], 'cube.yaw_deg', 1e-6)
        assert within(v['bucket']['inner_length_m'], 'bucket.inner_length')
        assert within(v['bucket']['inner_width_m'], 'bucket.inner_width')
        assert w.bucket_inner == (v['bucket']['inner_width_m'], v['bucket']['inner_length_m'])
        assert within(w.bucket_height, 'bucket.height')
        assert within(math.degrees(w.bucket_yaw) - offset['yaw_deg'], 'bucket.yaw_deg', 1e-6)
        camera = v['head_camera']
        assert all(within(value, 'head_camera.shift') for value in camera['shift_m'])
        assert all(within(value, 'head_camera.turn_deg') for value in camera['turn_deg'].values())
        # Drawn in the regions, in the table group's frame before it moved.
        assert inside(r.regions['cube.region'], *v['cube']['drawn_in_group_frame_m'])
        assert inside(r.regions['bucket.region'], *v['bucket']['drawn_in_group_frame']['center_m'])
        # Every drawn value says it was drawn; nothing else changed its source.
        drawn = {key for key, source in w.sources.items() if source.startswith(RANDOMIZED)}
        assert {'table.center', 'table.yaw_deg', 'bucket.center', 'bucket.inner_size', 'bucket.height',
                'cube.size', 'cube.start', 'cameras.head.pos', 'cameras.head.xyaxes'} <= drawn
        # (A drawn block gives its three edges, so the workcell's one edge is not read.)
        assert all(w.sources[key] == source for key, source in base.sources.items() if key not in drawn and
                   key not in ('cube.mass', 'cube.edge') and not key.startswith('fiducials.rectangle.'))


def test_the_bucket_stays_on_the_table_and_clear_of_the_cube(sampler, draws):
    clearance = sampler.randomization.cube_clearance
    for layout in draws:
        w = layout.workcell
        half = (w.bucket_inner[0] / 2 + w.bucket_wall, w.bucket_inner[1] / 2 + w.bucket_wall)
        c, s = math.cos(w.bucket_yaw), math.sin(w.bucket_yaw)
        for u in (-half[0], half[0]):
            for v in (-half[1], half[1]):
                assert on_table(w, w.bucket_center[0] + c * u - s * v, w.bucket_center[1] + s * u + c * v)
        x, y = layout.cube_start['x'], layout.cube_start['y']
        footprint = math.hypot(w.block[0], w.block[1]) / 2
        assert on_table(w, x, y, margin=footprint)
        # The block's corners never come within the clearance of the bucket's outer walls.
        along = bucket_frame(w, x, y)
        gap = math.hypot(max(0., abs(along[0]) - half[0]), max(0., abs(along[1]) - half[1]))
        assert gap >= footprint + clearance - 1e-9


def test_the_table_group_moves_as_one_and_the_camera_about_its_calibrated_pose(sampler, draws):
    base = sampler.base
    rectangle = base.fiducials['rectangle']
    for layout in draws[:40]:
        w, v = layout.workcell, layout.values
        turn, shift = math.radians(v['table_offset']['yaw_deg']), (v['table_offset']['x_m'], v['table_offset']['y_m'])

        def moved(point):
            x, y = _turn(point, turn, sampler.pivot)
            return x + shift[0], y + shift[1]
        assert w.table_center == pytest.approx(moved(base.table_center), abs=1e-12)
        assert w.table_yaw == pytest.approx(base.table_yaw + turn, abs=1e-12)
        assert w.fiducials['rectangle']['origin'] == pytest.approx(moved(rectangle['origin']), abs=1e-12)
        assert w.fiducials['rectangle']['yaw'] == pytest.approx(rectangle['yaw'] + turn, abs=1e-12)
        # The table tags keep their places on the table: the same table-frame coordinates as in the workcell.
        for tag, before in zip(w.fiducials['tags'][:6], base.fiducials['tags'][:6]):
            assert table_frame(w, tag[1], tag[2]) == pytest.approx(table_frame(base, before[1], before[2]), abs=1e-9)
        assert w.bucket_center == pytest.approx(moved(v['bucket']['drawn_in_group_frame']['center_m']), abs=1e-12)
        assert (layout.cube_start['x'], layout.cube_start['y']) == \
            pytest.approx(moved(v['cube']['drawn_in_group_frame_m']), abs=1e-12)
        # In the rectangle's frame, the camera is its calibrated pose moved by the drawn shift and turns.
        ox, oy = w.fiducials['rectangle']['origin']
        angle = math.pi + w.fiducials['rectangle']['yaw']
        c, s = math.cos(angle), math.sin(angle)
        x, y, z = w.head_camera['pos']
        in_rectangle = (c * (x - ox) + s * (y - oy), -s * (x - ox) + c * (y - oy), z - w.table_top)
        calibrated = json.loads(WORKCELL.read_text(encoding='utf-8'))['cameras']['head']
        assert np.subtract(in_rectangle, calibrated['pos']['value']) == pytest.approx(v['head_camera']['shift_m'],
                                                                                         abs=1e-9)
        local = np.array([[c * a[0] + s * a[1], -s * a[0] + c * a[1], a[2]]
                          for a in np.array(w.head_camera['xyaxes']).reshape(2, 3)])
        x_axis, y_axis = (np.array(calibrated['xyaxes']['value'][:3]), np.array(calibrated['xyaxes']['value'][3:]))
        x_axis /= np.linalg.norm(x_axis)
        y_axis -= (y_axis @ x_axis) * x_axis
        y_axis /= np.linalg.norm(y_axis)
        frame = np.column_stack([x_axis, y_axis, np.cross(x_axis, y_axis)])
        angles = [math.radians(v['head_camera']['turn_deg'][axis]) for axis in 'xyz']
        expected = frame @ _rotation(0, angles[0]) @ _rotation(1, angles[1]) @ _rotation(2, angles[2])
        assert local[0] == pytest.approx(expected[:, 0], abs=1e-12) and local[1] == pytest.approx(expected[:, 1],
                                                                                                abs=1e-12)
        # Its optical axis is within the drawn turns of the calibrated one.
        assert math.degrees(math.acos(min(1., float(np.cross(*local) @ frame[:, 2])))) <= \
            math.degrees(math.hypot(angles[0], angles[1])) + 1e-6


def test_a_polygon_region_is_drawn_from_uniformly_and_only_inside(tmp_path):
    document = json.loads(RANDOMIZATION.read_text(encoding='utf-8'))
    # An L: the square (0, -.6)-(.2, -.4) less its far-left quarter.
    l_shape = [[0., -.6], [.1, -.6], [.1, -.5], [.2, -.5], [.2, -.4], [0., -.4]]
    document['cube']['region'] = {'polygon': l_shape, 'source': 'test'}
    path = tmp_path / 'randomization.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    sampler = LayoutSampler(WORKCELL, path)
    rng = np.random.default_rng(5)
    points = np.array([sampler._in_region(rng, 'cube.region') for _ in range(3000)])
    assert all(inside(sampler.randomization.regions['cube.region'], x, y) for x, y in points)
    assert not np.any((points[:, 0] > .1) & (points[:, 1] < -.5))
    # Uniform: each of its three quarters gets about a third.
    quarters = [np.mean((points[:, 0] < .1) & (points[:, 1] < -.5)), np.mean((points[:, 0] < .1) & (points[:, 1] > -.5)),
                np.mean((points[:, 0] > .1) & (points[:, 1] > -.5))]
    assert all(abs(q - 1 / 3) < .04 for q in quarters)


def test_without_a_randomization_file_the_workcell_is_the_fixed_layout():
    assert load_workcell(WORKCELL).table_yaw == 0.
