"""The task scene's workcell: measured values carry their sources, and the scene judges the bucket it was given."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from robot_platform.evaluation.grid import GridSpec, cube_poses
from robot_platform.simulation.fiducials import CODES, cells, corners, grid, marker_axes
from robot_platform.simulation.task_scene import CUBE_START, TABLE_TOP, MinervaTaskScene
from robot_platform.simulation.workcell import (SYNTHETIC, Workcell, bucket_frame, head_pinhole, load_workcell, resolve,
                                               validate, workcell_path)

ROOT = Path(__file__).resolve().parents[2]
MEASURED = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1., offset=0.) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=.01, offset=0.)
    return mapping


def test_synthetic_layout_keeps_the_constants_the_fixture_was_built_on():
    assert SYNTHETIC.table_top == TABLE_TOP == .28
    assert (SYNTHETIC.cube_start[0], SYNTHETIC.cube_start[1], SYNTHETIC.table_top + SYNTHETIC.cube_half + .002) \
        == CUBE_START
    assert resolve(None) is SYNTHETIC and resolve('synthetic') is SYNTHETIC
    assert SYNTHETIC.placeholders() == []
    assert cube_poses(GridSpec())[0]['x'] == CUBE_START[0]


def test_measured_file_names_a_source_for_every_value_and_lists_what_is_unmeasured():
    workcell = load_workcell(MEASURED)
    assert workcell.asset_id == 'minerva-production-01'
    assert workcell_path(ROOT / 'config/devices/minerva-joint-calibration.json', workcell.asset_id) == MEASURED
    # The user's measurements are values, not placeholders.
    assert workcell.table_top == pytest.approx(.664 - .152, abs=1e-3)
    # The box as the head camera's depth measured it (the user's 2026-10-16 "about 6 by 12 in" before that).
    assert workcell.cube_edge == pytest.approx(.0381) and workcell.bucket_inner == pytest.approx((.155, .242))
    measured = {'table.size', 'table.top_z', 'bucket.inner_size', 'cube.edge'}
    assert measured.isdisjoint(workcell.placeholders())
    # The head camera is measured against the tags; where the tags are relative to the robot is not yet.
    assert {'table.center', 'bucket.wall', 'cube.mass', 'fiducials.rectangle.robot_offsets'} <= set(workcell.placeholders())
    assert {'cameras.head.pos', 'cameras.head.xyaxes', 'cameras.head.fovy'}.isdisjoint(workcell.placeholders())
    assert all(workcell.sources[key] for key in workcell.sources)
    # The description crosses process boundaries unchanged.
    assert Workcell.from_description(json.loads(json.dumps(workcell.describe()))) == workcell


def test_a_value_without_a_source_is_refused(tmp_path):
    document = json.loads(MEASURED.read_text(encoding='utf-8'))
    document['table']['top_z'] = {'value': .5}
    path = tmp_path / 'bad-workcell.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError, match='named source'):
        load_workcell(path)
    with pytest.raises(ValueError, match='fit inside'):
        validate(Workcell.from_description(dict(SYNTHETIC.describe(), bucket_inner=[.03, .03])))


def test_grid_and_scene_follow_the_workcell():
    workcell = load_workcell(MEASURED)
    poses = cube_poses(GridSpec.for_workcell(workcell))
    assert (poses[0]['x'], poses[0]['y']) == workcell.cube_start[:2]
    (x0, x1), (y0, y1) = workcell.cube_region
    assert all(x0 <= p['x'] <= x1 and y0 <= p['y'] <= y1 for p in poses)
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=workcell)
    try:
        scene._q = np.zeros(14)
        state = scene.reset()
        for _ in range(6):
            state = scene.update(np.zeros(14), .05)
        # The cube rests on the measured table; the hanging arms do not reach it.
        assert state['cube_position'][2] == pytest.approx(workcell.table_top + workcell.cube_half, abs=2e-4)
        assert state['cube_contacts'] == ['task/table'] and not state['forbidden_contacts']
        assert scene.capabilities['workcell']['name'] == workcell.name
        assert scene.capabilities['workcell_placeholders'] == workcell.placeholders()
        assert any('not yet measured' in item for item in scene.capabilities['limitations'])
    finally:
        scene.close()


def _bucket_drop(workcell, offset):
    """Drop the cube from above the bucket at `offset` (bucket axes); the scene's verdict on where it settles."""
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=workcell)
    try:
        scene._q = np.zeros(14)
        scene.reset()
        c, s = np.cos(workcell.bucket_yaw), np.sin(workcell.bucket_yaw)
        x = workcell.bucket_center[0] + c * offset[0] - s * offset[1]
        y = workcell.bucket_center[1] + s * offset[0] + c * offset[1]
        scene.data.qpos[scene.cube_qadr:scene.cube_qadr + 3] = [x, y, workcell.table_top + workcell.bucket_height + .05]
        scene.mj.mj_forward(scene.model, scene.data)
        inside = touched = False
        for _ in range(40):
            state = scene.update(np.zeros(14), .05)
            touched = touched or 'task/bucket/bottom' in state['cube_contacts']
        along = np.array(state['cube_position'][:2]) - np.array(workcell.bucket_center)
        inside = abs(c * along[0] + s * along[1]) < workcell.bucket_inner[0] / 2 and \
            abs(-s * along[0] + c * along[1]) < workcell.bucket_inner[1] / 2
        return inside, touched, state
    finally:
        scene.close()


@pytest.mark.parametrize('yaw_deg', [0., 30.])
def test_a_rectangular_yawed_bucket_holds_the_cube_dropped_into_its_long_end(yaw_deg):
    measured = load_workcell(MEASURED)
    # Without the fiducial tags: the file placed its wall tags on the unturned bucket.
    workcell = Workcell.from_description(dict(measured.describe(), bucket_yaw=np.radians(yaw_deg), fiducials={}))
    # Near the far end of the long side: inside a 0.305 m box, outside a square one of the short side.
    inside, touched, state = _bucket_drop(workcell, (0., .10))
    assert inside and touched
    # Settled on the bottom with no gripper: the scene sees it in the bucket, out of sequence, so no success.
    assert state['bucket_settled'] and not state['success'] and not state['forbidden_contacts']
    # The same distance across the short side is outside the 0.152 m box.
    inside, touched, state = _bucket_drop(workcell, (.10, 0.))
    assert not inside and not touched and not state['bucket_settled']


# OpenCV's DICT_4X4_50 bytes for IDs 0-7 (cv2.aruco dictionary bytesList[id][0][:2], the unrotated marker): their
# bits, most significant first, are the code's cells row by row from its top, 1 white. An independent copy of what
# fiducials.CODES must say; scripts/sim/check_fiducial_codes.py checks the table against OpenCV itself.
OPENCV_DICT_4X4_50 = {0: (181, 50), 1: (15, 154), 2: (51, 45), 3: (153, 70), 4: (84, 158), 5: (121, 205),
                      6: (158, 46), 7: (196, 242)}
# The user's taped rectangle (2026-10-18): 650 x 441 mm over the outer corners of tags 0-3's black squares.
RECTANGLE = (.650, .441)
SQUARE = .04318
CORNERS = {0: (SQUARE / 2, SQUARE / 2), 1: (RECTANGLE[0] - SQUARE / 2, SQUARE / 2),
           2: (RECTANGLE[0] - SQUARE / 2, RECTANGLE[1] - SQUARE / 2), 3: (SQUARE / 2, RECTANGLE[1] - SQUARE / 2)}
HALF_TURN = np.diag([-1., -1., 1.])  # A marker frame on the base: x along the table's x, y away from the robot.


def _opencv_cells(tag_id):
    bits = ''.join(f'{byte:08b}' for byte in OPENCV_DICT_4X4_50[tag_id])
    return [[0] * 6] + [[0, *(int(b) for b in bits[4 * r:4 * r + 4]), 0] for r in range(4)] + [[0] * 6]


def _table_corner(workcell):
    """(c, d): the table top's near-left outer corner is base (c, -d)."""
    return (workcell.table_center[0] + workcell.table_size[0] / 2,
            -(workcell.table_center[1] + workcell.table_size[1] / 2))


def test_the_fiducial_codes_are_opencvs():
    assert sorted(CODES['DICT_4X4_50']) == sorted(OPENCV_DICT_4X4_50)
    for tag_id in OPENCV_DICT_4X4_50:
        assert cells('DICT_4X4_50', tag_id) == _opencv_cells(tag_id)


def test_the_tags_sit_where_the_rectangle_and_the_bucket_put_them():
    workcell = load_workcell(MEASURED)
    f = workcell.fiducials
    tags = {tag[0]: tag for tag in f['tags']}
    assert sorted(tags) == list(range(8)) and f['family'] == 'DICT_4X4_50' and f['square'] == SQUARE
    # Tags 0-3 sit half a square in from the taped rectangle's corners. Its robot offsets (c, d) put its origin at
    # base (c, -d), its near side along base -x and its depth along base -y.
    assert f['rectangle']['size'] == RECTANGLE and f['rectangle']['yaw'] == 0.
    (ox, oy) = f['rectangle']['origin']
    assert (ox, -oy) == pytest.approx((.325, .1584))  # The placeholder: centred on the robot, 16 cm out.
    for tag_id, (x, y) in CORNERS.items():
        assert tags[tag_id][1:4] == pytest.approx((ox - x, oy - y, workcell.table_top), abs=1e-12)
    # 4 and 5 sit at the middles of the near and far sides, where the head camera found them.
    for tag_id, side in ((4, CORNERS[0][1]), (5, CORNERS[3][1])):
        x, y = ox - tags[tag_id][1], oy - tags[tag_id][2]
        assert x == pytest.approx(RECTANGLE[0] / 2, abs=.003) and y == pytest.approx(side, abs=.003)
    # On the table every marker frame is the rectangle's, turned onto the base: x along its near side, y away.
    for tag_id in range(6):
        assert np.array(marker_axes(*tags[tag_id][4:])).T == pytest.approx(HALF_TURN, abs=1e-12)
    # The head camera is given in the rectangle too, so it moves with the tags.
    head = workcell.head_camera
    assert head['pos'] == pytest.approx((ox - .2752, oy + .2694, workcell.table_top + .5443), abs=1e-12)
    # 6 is upright on the front wall's outer face and 7 on the back wall's inner face (as the head camera decodes
    # them), both facing the robot along the box's axis, their centres 42 mm below the rim in the middle of their walls.
    # The box itself is placed in the rectangle, as the head camera's depth measured it.
    assert workcell.bucket_inner == pytest.approx((.155, .242)) and math.degrees(workcell.bucket_yaw) == pytest.approx(-19.2)
    assert bucket_frame(workcell, ox - .4779, oy - .2337) == pytest.approx((0., 0.), abs=1e-12)
    toward_robot = (-math.sin(workcell.bucket_yaw), math.cos(workcell.bucket_yaw), 0.)
    half = (workcell.bucket_inner[0] / 2, workcell.bucket_inner[1] / 2)
    for tag_id, wall_y in ((6, half[1] + workcell.bucket_wall), (7, -half[1])):
        _, x, y, z, yaw, tilt = tags[tag_id]
        assert bucket_frame(workcell, x, y) == pytest.approx((0., wall_y), abs=1e-12)
        assert z == pytest.approx(workcell.table_top + workcell.bucket_height - .042)
        x_axis, y_axis, z_axis = marker_axes(yaw, tilt)
        assert z_axis == pytest.approx(toward_robot, abs=1e-12) and y_axis == pytest.approx((0., 0., 1.), abs=1e-12)
    # Placeholders until measured: the rectangle's place relative to the robot and the wall tags' unknowns.
    placeholders = set(workcell.placeholders())
    assert {'fiducials.rectangle.robot_offsets', 'fiducials.rectangle.yaw_deg', 'bucket.wall', 'bucket.bottom'} <= \
        placeholders
    assert {'fiducials.family', 'fiducials.square', 'fiducials.rectangle.size', 'fiducials.tags.6.wall',
            'fiducials.tags.7.face', 'fiducials.tags.6.below_rim', 'bucket.center', 'bucket.yaw_deg', 'bucket.height',
            'bucket.inner_size'} | {f'fiducials.tags.{i}.center' for i in range(6)} <= set(workcell.sources) - placeholders
    assert SYNTHETIC.fiducials == {}


def test_the_wall_tags_follow_the_bucket(tmp_path):
    before = {tag[0]: tag for tag in load_workcell(MEASURED).fiducials['tags']}
    document = json.loads(MEASURED.read_text(encoding='utf-8'))
    document['bucket']['center']['value'] = [.40, .25]  # In the rectangle frame.
    document['bucket']['yaw_deg']['value'] = 10.
    moved = load_workcell(_write(document, tmp_path))
    assert math.degrees(moved.bucket_yaw) == pytest.approx(10.)
    tags = {tag[0]: tag for tag in moved.fiducials['tags']}
    assert all(tags[tag_id] == before[tag_id] for tag_id in range(6))
    for tag_id, wall_y in ((6, moved.bucket_inner[1] / 2 + moved.bucket_wall), (7, -moved.bucket_inner[1] / 2)):
        _, x, y, z, yaw, tilt = tags[tag_id]
        assert bucket_frame(moved, x, y) == pytest.approx((0., wall_y), abs=1e-12)
        # Still facing the robot's way out of its face, turned with the bucket.
        normal = marker_axes(yaw, tilt)[2]
        assert normal == pytest.approx((-math.sin(math.radians(10.)), math.cos(math.radians(10.)), 0.), abs=1e-12)


def test_the_scene_draws_each_tag_with_its_code():
    workcell = load_workcell(MEASURED)
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=workcell)
    try:
        m, d, f = scene.model, scene.data, workcell.fiducials
        # MuJoCo keeps colours as float32.
        paper, ink = (tuple(np.float32(workcell.color(key))) for key in ('fiducial_label', 'fiducial_print'))
        n = grid(f['family'])
        size = f['square'] / n
        assert any('Fiducial tags' in item for item in scene.capabilities['limitations'])
        for tag in f['tags']:
            tag_id, x, y, z, yaw, tilt = tag
            body = scene._id('body', f'task/fiducial/{tag_id}')
            # On the table or on its wall, its marker axes where marker_axes puts them.
            assert d.xpos[body] == pytest.approx((x, y, z), abs=1e-12)
            assert d.xmat[body].reshape(3, 3) == pytest.approx(np.array(marker_axes(yaw, tilt)).T, abs=1e-12)
            geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body]
            # Visual only: no contacts, no mass, drawn in both visual modes, outside the penetration audit.
            assert m.body_mass[body] == 0 and not m.geom_contype[geoms].any() and not m.geom_conaffinity[geoms].any()
            assert not m.geom_group[geoms].any() and not set(geoms) & set(scene._environment_ids.tolist()) and \
                not set(geoms) & set(scene._collision_ids.tolist())
            assert {tuple(m.geom_rgba[g]) for g in geoms} == {paper, ink}

            def white(u, v):
                """Whether the top geom at marker-frame point (u, v) is white."""
                over = [g for g in geoms if abs(u - m.geom_pos[g][0]) < m.geom_size[g][0] and
                        abs(v - m.geom_pos[g][1]) < m.geom_size[g][1]]
                top = max(over, key=lambda g: m.geom_pos[g][2] + m.geom_size[g][2])
                return tuple(m.geom_rgba[top]) == paper
            code = _opencv_cells(tag_id)
            for row in range(n):
                for column in range(n):
                    assert white((column - (n - 1) / 2) * size, ((n - 1) / 2 - row) * size) == bool(code[row][column])
            # The black square's corners are where fiducials.corners puts them, inside a one-cell white margin.
            axes = np.array(marker_axes(yaw, tilt))
            for corner in corners(f['square'], x, y, z, yaw, tilt):
                u, v, w = axes @ np.subtract(corner, (x, y, z))
                assert w == pytest.approx(0., abs=1e-12)
                assert not white(u * .99, v * .99) and white(u + np.sign(u) * size * .99, v + np.sign(v) * size * .99)
    finally:
        scene.close()


# The head D435's colour intrinsics at 640 x 480 (serial 919122072056, rs-enumerate-devices -c, 2026-10-18), for its
# own unrotated image; the head node publishes the image turned half a turn.
D435_COLOUR = {'fx': 614.336486816406, 'fy': 614.734069824219, 'ppx': 313.262512207031, 'ppy': 236.9853515625}


def test_the_scene_head_camera_projects_as_the_real_one():
    """The scene's head camera, as MuJoCo renders it (the projection OpenGL builds from mjv_updateScene's frustum, no
    GL needed), puts every tag corner where the D435's pinhole model does for the published image: its focal lengths,
    the principal point turned half a turn to (639 - PPX, 479 - PPY), and the workcell's pose."""
    workcell = load_workcell(MEASURED)
    head = workcell.head_camera
    width, height = head['intrinsics']['resolution']
    fx, fy, cx, cy = head_pinhole(head)
    assert (fx, fy) == (D435_COLOUR['fx'], D435_COLOUR['fy'])
    assert (cx, cy) == pytest.approx((width - 1 - D435_COLOUR['ppx'], height - 1 - D435_COLOUR['ppy']), abs=1e-9)
    assert head_pinhole(head, width // 2, height // 2) == pytest.approx((fx / 2, fy / 2, (cx - .5) / 2, (cy - .5) / 2))
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=workcell)
    try:
        mj, model = scene.mj, scene.model
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        camera_id = scene._id('camera', 'head')
        assert model.cam_fovy[camera_id] == pytest.approx(head['fovy'], abs=1e-3)
        view = mj.MjvCamera()
        view.type, view.fixedcamid = mj.mjtCamera.mjCAMERA_FIXED, camera_id
        abstract = mj.MjvScene(model, maxgeom=10000)
        mj.mjv_updateScene(model, data, mj.MjvOption(), None, view, mj.mjtCatBit.mjCAT_ALL, abstract)
        eye = abstract.camera[0]
        left, right = eye.frustum_center - eye.frustum_width, eye.frustum_center + eye.frustum_width
        bottom, top, near = eye.frustum_bottom, eye.frustum_top, eye.frustum_near
        frame = data.cam_xmat[camera_id].reshape(3, 3)  # Columns: MuJoCo's camera x (right), y (up), z (backward).
        # The camera is where the workcell puts it, its axes the published image's.
        assert data.cam_xpos[camera_id] == pytest.approx(head['pos'], abs=1e-12)
        assert frame[:, 0] == pytest.approx(np.array(head['xyaxes'][:3]) / np.linalg.norm(head['xyaxes'][:3]), abs=1e-5)
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        worst = 0.
        for tag in workcell.fiducials['tags']:
            for corner in corners(workcell.fiducials['square'], *tag[1:]):
                x, y, z = (np.asarray(corner) - data.cam_xpos[camera_id]) @ frame
                # glFrustum(left, right, bottom, top, near, far): the frustum's left edge at -1, its right at +1.
                ndc = ((2 * near * x / -z - (right + left)) / (right - left),
                       (2 * near * y / -z - (top + bottom)) / (top - bottom))
                # Pixel centres on integers, rows down (the renderer turns OpenGL's bottom-up rows over).
                rendered = ((ndc[0] + 1) / 2 * width - .5, (1 - ndc[1]) / 2 * height - .5)
                pinhole = K @ np.array([x, -y, -z])
                worst = max(worst, float(np.abs(np.subtract(rendered, pinhole[:2] / pinhole[2])).max()))
        # MuJoCo keeps the camera's sensor size and focal length as float32: a few ten-thousandths of a pixel.
        assert worst < 1e-3, worst
    finally:
        scene.close()


def test_the_tags_are_visual_only_and_change_no_physics():
    workcell = load_workcell(MEASURED)
    bare = Workcell.from_description(dict(workcell.describe(), fiducials={}))
    scenes = [MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, workcell=w) for w in (workcell, bare)]
    try:
        tagged, plain = scenes
        nbody, ngeom = plain.model.nbody, plain.model.ngeom
        assert tagged.model.nbody == nbody + 8 and tagged.model.ncam == plain.model.ncam
        # The tags come after every other body and geom, which keep their indices and values.
        for key in ('geom_type', 'geom_bodyid', 'geom_pos', 'geom_quat', 'geom_size', 'geom_contype', 'geom_conaffinity',
                    'geom_friction', 'geom_solref', 'geom_solimp', 'geom_margin', 'geom_rgba'):
            assert np.array_equal(getattr(tagged.model, key)[:ngeom], getattr(plain.model, key)), key
        for key in ('body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_parentid', 'body_mocapid'):
            assert np.array_equal(getattr(tagged.model, key)[:nbody], getattr(plain.model, key)), key
        assert tagged.model.stat.meaninertia == plain.model.stat.meaninertia
        assert tagged.capabilities['contact_geometry_sha256'] == plain.capabilities['contact_geometry_sha256']
        # A cube dropped onto tag 4 lands on the table exactly as it does with no tag there.
        _, x, y, _, _, _ = next(tag for tag in workcell.fiducials['tags'] if tag[0] == 4)
        for scene in scenes:
            scene._q = np.zeros(14)
            scene.reset()
            scene.data.qpos[scene.cube_qadr:scene.cube_qadr + 3] = [x, y, workcell.table_top + .05]
            scene.mj.mj_forward(scene.model, scene.data)
            for _ in range(8):
                state = scene.update(np.zeros(14), .05)
            assert state['cube_contacts'] == ['task/table'] and not state['forbidden_contacts']
        assert np.array_equal(tagged.data.qpos, plain.data.qpos) and np.array_equal(tagged.data.qvel, plain.data.qvel)
    finally:
        for scene in scenes:
            scene.close()


def _write(document, directory):
    path = Path(directory) / 'workcell.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    return path


@pytest.mark.parametrize('change, refusal', [
    (lambda f: f['rectangle']['robot_offsets'].update(value=[.325, .50]), 'off the table'),
    (lambda f: f['tags']['0']['center'].update(value=[.02, .10]) or f['tags']['0'].pop('frame'), 'off the table'),
    (lambda f: f['tags']['7']['along'].update(value=.06), 'not wholly on a face'),
    (lambda f: f['tags']['6']['below_rim'].update(value=.17), 'not wholly on a face'),
    (lambda f: f['tags'].update({'8': f['tags']['0']}), 'distinct'),
    (lambda f: f['family'].update(value='DICT_6X6_50'), 'no codes'),
    (lambda f: f['tags']['1'].update(wall={'value': 'front', 'source': 'test'}), 'either'),
    (lambda f: f['tags']['1'].update(frame='bucket'), 'frame'),
    (lambda f: f['tags']['6']['face'].update(value='top'), 'face'),
    (lambda f: f['square'].update(value=.05), 'white margin'),
    (lambda f: f['tags']['2']['center'].pop('source'), 'named source'),
])
def test_a_tag_the_scene_cannot_draw_is_refused(tmp_path, change, refusal):
    document = json.loads(MEASURED.read_text(encoding='utf-8'))
    change(document['fiducials'])
    with pytest.raises(ValueError, match=refusal):
        load_workcell(_write(document, tmp_path))
