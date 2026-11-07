"""The task scene draws the whole robot dark grey, so that it reads against the black table as the real Minerva does to
the eye, and leaves the rest of the scene and its physics alone. The user (2026-10-18): "The whole robot shouldn't be
pitch black, make it a rather dark gray"."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.simulation import room
from robot_platform.simulation.task_scene import (
    JAW_BODIES,
    ROBOT_MATERIAL,
    ROOM_FLOOR,
    WRISTS,
    MinervaTaskScene,
    RENDER_FLAGS,
)
from robot_platform.simulation.workcell import load_workcell

ROOT = Path(__file__).resolve().parents[2]
MEASURED = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']
# Dark grey: every RGB channel of a robot geom's colour, and of its material's, the look's grey, between 0.15 and 0.3.
DARK_GREY = (0.15, 0.30)
# Against the black table, a gripper's pixels are this much brighter on average (sRGB luma, of 255), in the head view
# and in the wrist view of the arm that grasps.
STANDS_OUT = 25


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1.0, offset=0.0) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=0.01, offset=0.0)
    return mapping


def _name(scene, kind, index):
    return scene.mj.mj_id2name(scene.model, getattr(scene.mj.mjtObj, f'mjOBJ_{kind}'), int(index)) or ''


def _robot_geoms(scene):
    """Every geom of the robot: all but the workcell's, the cube's and the tags' (named task/...)."""
    return [g for g in range(scene.model.ngeom) if not _name(scene, 'GEOM', g).startswith('task/')]


def _luma(pixels):
    return np.asarray(pixels, dtype=float) @ np.array([0.2126, 0.7152, 0.0722])


@pytest.mark.parametrize('visual_mode', ['cad', 'collision_proxy'])
def test_every_robot_geom_is_drawn_dark_grey(visual_mode):
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode=visual_mode, workcell=MEASURED)
    try:
        m = scene.model
        shown = np.flatnonzero(scene.visual_groups)
        robot = _robot_geoms(scene)
        drawn = [g for g in robot if m.geom_group[g] in shown]
        # The grasp-frame marker boxes are never drawn (group 5, transparent); every other robot geom is dark grey.
        markers = [g for g in robot if _name(scene, 'GEOM', g).endswith('/terminal_pad')]
        assert len(markers) == 4 and all(m.geom_group[g] == 5 and m.geom_rgba[g][3] == 0 for g in markers)
        material = scene._id('material', ROBOT_MATERIAL)
        grey = room.LOOK['robot_grey']
        assert DARK_GREY[0] <= grey <= DARK_GREY[1]
        for g in sorted(set(robot) - set(markers)):
            assert np.allclose(m.geom_rgba[g], (grey, grey, grey, 1.0)), _name(scene, 'GEOM', g)
            assert m.geom_matid[g] == material, _name(scene, 'GEOM', g)
        assert np.allclose(m.mat_rgba[material], (grey, grey, grey, 1.0))
        # A little specular, so that edges read; no reflection.
        assert m.mat_specular[material] == pytest.approx(room.ROBOT_SPECULAR) and 0 < room.ROBOT_SPECULAR <= 0.4
        assert m.mat_shininess[material] == pytest.approx(room.ROBOT_SHININESS) and m.mat_reflectance[material] == 0
        # Every robot body with geometry draws some of it in this mode: the torso and shoulders, every link and
        # motor of both arms, both wrists and all four jaws.
        bodies = {_name(scene, 'BODY', m.geom_bodyid[g]) for g in robot}
        assert {_name(scene, 'BODY', m.geom_bodyid[g]) for g in drawn} == bodies
        assert {
            'ikea_desk_leg',
            '1_1_02_eb571_960_stp',
            '1_1_02_eb571_960_stp_2',
            'carriage_lwl7b',
            'carriage_lwl7b_2',
            'carriage_lwl7b_3',
            'carriage_lwl7b_4',
        } <= bodies and len(bodies) == 17
        assert np.allclose(scene.capabilities['robot_finish']['rgba'], m.mat_rgba[material])
    finally:
        scene.close()


def test_the_workcell_the_cube_and_the_tags_keep_their_own_colours_and_the_floor_is_wood():
    workcell = load_workcell(MEASURED)
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode='cad', workcell=workcell)
    try:
        m = scene.model
        expected = {}
        for g in range(m.ngeom):
            name = _name(scene, 'GEOM', g)
            if not name.startswith('task/'):
                continue
            if name == 'task/floor':
                # The room's wood, in the look's floor tone (1: as painted), in place of the workcell's floor colour.
                assert m.geom_matid[g] == scene._id('material', ROOM_FLOOR)
                assert np.allclose(m.geom_rgba[g], (*room.LOOK['floor_tone'], 1.0))
                continue
            if name.startswith('task/fiducial/'):
                key = 'fiducial_print' if name.endswith('/square') else 'fiducial_label'
            elif name.startswith('task/table/trim/'):
                key = 'trim'
            else:
                key = {'task/table': 'table', 'task/cube/geom': 'cube', 'task/bucket/bottom': 'bucket_bottom'}.get(
                    name
                ) or (
                    'leg'
                    if name.startswith('task/leg/')
                    else 'bucket_wall'
                    if name.startswith('task/bucket/wall/')
                    else None
                )
            assert key, name
            expected[g] = key
            assert np.array_equal(m.geom_rgba[g], np.float32(workcell.color(key))), name
            assert m.geom_matid[g] == -1, name
        assert set(expected.values()) == {
            'table',
            'trim',
            'leg',
            'bucket_bottom',
            'bucket_wall',
            'cube',
            'fiducial_label',
            'fiducial_print',
        }
    finally:
        scene.close()


class _Brightest:
    """A generator that draws every bound at its top: the appearance variation's brightest look."""

    def uniform(self, low, high, size=None):
        return high if size is None else np.full(size, high, dtype=float)

    def normal(self, size=None):
        return np.ones(size)


def _render(scene, camera, height=240, width=320, segments=False):
    mj = scene.mj
    renderer = mj.Renderer(scene.model, height=height, width=width)
    try:
        options = mj.MjvOption()
        options.geomgroup[:] = scene.visual_groups
        if segments:
            renderer.enable_segmentation_rendering()
        renderer.update_scene(scene.data, camera=camera, scene_option=options)
        for flag, value in RENDER_FLAGS.items():
            renderer.scene.flags[getattr(mj.mjtRndFlag, flag)] = value
        return renderer.render().copy()
    finally:
        renderer.close()


def test_the_appearance_variation_leaves_the_robot_dark_grey():
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode='cad', workcell=MEASURED)
    try:
        m, mj = scene.model, scene.mj
        robot = _robot_geoms(scene)
        rgba, matid = m.geom_rgba[robot].copy(), m.geom_matid[robot].copy()
        look = scene.randomize_appearance(_Brightest(), light=0.3, color=0.1, camera_shift=0.01, camera_turn_deg=2.0)
        assert look['light_scale'] == pytest.approx(1.3)
        assert not {_name(scene, 'GEOM', g) for g in robot} & set(look['color_factors'])
        assert np.array_equal(m.geom_rgba[robot], rgba) and np.array_equal(m.geom_matid[robot], matid)
        # Rendered at that look, from in front of the robot: the robot stays a dark grey, far from white, and the box
        # and the cube are far brighter.
        scene._q = np.zeros(len(NAMES))
        scene.reset()
        camera = mj.MjvCamera()
        camera.type = mj.mjtCamera.mjCAMERA_FREE
        camera.azimuth, camera.elevation, camera.distance = 90.0, -25.0, 1.6
        camera.lookat[:] = (0.0, -0.25, 0.85)
        image, segments = _render(scene, camera), _render(scene, camera, segments=True)
        on_robot = (segments[..., 1] == int(mj.mjtObj.mjOBJ_GEOM)) & np.isin(segments[..., 0], robot)
        assert on_robot.mean() > 0.05  # The robot fills part of the view.
        robot_luma = _luma(image[on_robot])
        assert 30 < np.median(robot_luma) < 140 and np.percentile(robot_luma, 99) < 230
        boxed = (segments[..., 1] == int(mj.mjtObj.mjOBJ_GEOM)) & np.isin(
            segments[..., 0], [scene._id('geom', 'task/cube/geom'), scene._id('geom', 'task/bucket/bottom')]
        )
        assert boxed.any() and _luma(image[boxed]).mean() > 1.3 * np.median(robot_luma)
    finally:
        scene.restore_appearance()
        scene.close()


@pytest.fixture(scope='module')
def grasps():
    """The demonstrator's plan on the measured workcell: the joints where the left and then the right close on the
    block."""
    from robot_platform.evaluation.demonstrator import HandoffDemonstrator
    from robot_platform.evaluation.plants import _Stack, derive_stack

    workcell = load_workcell(MEASURED)
    stack = derive_stack()
    limits = _Stack(stack)
    context = SimpleNamespace(
        joint_names=stack['names'],
        joint_map=stack['derived']['joint_map'],
        lower=limits.lower,
        upper=limits.upper,
        home=limits.home,
        workcell=workcell,
        period_s=limits.period_s,
        lead=limits.lead,
        cube_start={'x': workcell.cube_start[0], 'y': workcell.cube_start[1], 'yaw': workcell.cube_start[2]},
    )
    demo = HandoffDemonstrator()
    try:
        demo.reset(context)
        frames = {frame.label: frame for frame in demo.frames}
        yield stack, {'left': frames['left on the cube'].target, 'right': frames['right on the cube'].target}
    finally:
        demo.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_the_grippers_stand_out_against_the_black_table(grasps, side):
    """Rendered as the episodes are (CAD), where each arm closes on the block: its gripper's pixels are clearly
    brighter than the table top's in the head view and in that arm's wrist view."""
    stack, targets = grasps
    scene = MinervaTaskScene(
        stack['derived']['joint_map'], stack['names'], width=320, height=240, visual_mode='cad', workcell=MEASURED
    )
    try:
        mj, m = scene.mj, scene.model
        scene._pose(np.asarray(targets[side], dtype=float))
        mj.mj_forward(m, scene.data)
        gripper = [
            g for g in range(m.ngeom) if _name(scene, 'BODY', m.geom_bodyid[g]) in {WRISTS[side], *JAW_BODIES[side]}
        ]
        table = [scene._id('geom', 'task/table')]
        for camera in ('head', f'{side}_wrist'):
            image, segments = _render(scene, camera), _render(scene, camera, segments=True)
            geoms = np.where(segments[..., 1] == int(mj.mjtObj.mjOBJ_GEOM), segments[..., 0], -1)
            on_gripper, on_table = np.isin(geoms, gripper), np.isin(geoms, table)
            assert on_gripper.sum() > 200 and on_table.sum() > 200, camera
            assert _luma(image[on_gripper]).mean() > _luma(image[on_table]).mean() + STANDS_OUT, camera
    finally:
        scene.close()
