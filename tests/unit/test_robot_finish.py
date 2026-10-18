"""The task scene draws the whole robot black and matte, as the real Minerva is, and leaves the rest of the scene and
its physics alone."""
import json
from pathlib import Path

import numpy as np
import pytest

from robot_platform.simulation.task_scene import ROBOT_MATERIAL, MinervaTaskScene
from robot_platform.simulation.workcell import load_workcell

ROOT = Path(__file__).resolve().parents[2]
MEASURED = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']
# Near-black: every RGB channel of a robot geom's colour, and of its material's, at most 0.06 (15 of 255).
NEAR_BLACK = .06
# Rendered, at the brightest look the appearance variation draws, no robot pixel brighter than 32 of 255 in any channel.
RENDERED_BLACK = 32


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1., offset=0.) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=.01, offset=0.)
    return mapping


def _name(scene, kind, index):
    return scene.mj.mj_id2name(scene.model, getattr(scene.mj.mjtObj, f'mjOBJ_{kind}'), int(index)) or ''


def _robot_geoms(scene):
    """Every geom of the robot: all but the workcell's, the cube's and the tags' (named task/...)."""
    return [g for g in range(scene.model.ngeom) if not _name(scene, 'GEOM', g).startswith('task/')]


@pytest.mark.parametrize('visual_mode', ['cad', 'collision_proxy'])
def test_every_robot_geom_is_drawn_matte_black(visual_mode):
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode=visual_mode, workcell=MEASURED)
    try:
        m = scene.model
        shown = np.flatnonzero(scene.visual_groups)
        robot = _robot_geoms(scene)
        drawn = [g for g in robot if m.geom_group[g] in shown]
        # The grasp-frame marker boxes are never drawn (group 5, transparent); every other robot geom is black.
        markers = [g for g in robot if _name(scene, 'GEOM', g).endswith('/terminal_pad')]
        assert len(markers) == 4 and all(m.geom_group[g] == 5 and m.geom_rgba[g][3] == 0 for g in markers)
        material = scene._id('material', ROBOT_MATERIAL)
        for g in sorted(set(robot) - set(markers)):
            assert np.all(m.geom_rgba[g][:3] <= NEAR_BLACK) and m.geom_rgba[g][3] == 1, _name(scene, 'GEOM', g)
            assert m.geom_matid[g] == material, _name(scene, 'GEOM', g)
        assert np.all(m.mat_rgba[material][:3] <= NEAR_BLACK)
        assert m.mat_specular[material] == 0 and m.mat_shininess[material] == 0 and m.mat_reflectance[material] == 0
        # Every robot body with geometry draws some of it in this mode: the torso and shoulders, every link and
        # motor of both arms, both wrists and all four jaws.
        bodies = {_name(scene, 'BODY', m.geom_bodyid[g]) for g in robot}
        assert {_name(scene, 'BODY', m.geom_bodyid[g]) for g in drawn} == bodies
        assert {'ikea_desk_leg', '1_1_02_eb571_960_stp', '1_1_02_eb571_960_stp_2', 'carriage_lwl7b',
                'carriage_lwl7b_2', 'carriage_lwl7b_3', 'carriage_lwl7b_4'} <= bodies and len(bodies) == 17
        assert np.allclose(scene.capabilities['robot_finish']['rgba'], m.mat_rgba[material])
    finally:
        scene.close()


def test_the_workcell_the_cube_and_the_tags_keep_their_own_colours():
    workcell = load_workcell(MEASURED)
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode='cad', workcell=workcell)
    try:
        m = scene.model
        expected = {}
        for g in range(m.ngeom):
            name = _name(scene, 'GEOM', g)
            if not name.startswith('task/'):
                continue
            if name.startswith('task/fiducial/'):
                key = 'fiducial_print' if name.endswith('/square') else 'fiducial_label'
            elif name.startswith('task/table/trim/'):
                key = 'trim'
            else:
                key = {'task/floor': 'floor', 'task/table': 'table', 'task/cube/geom': 'cube',
                       'task/bucket/bottom': 'bucket_bottom'}.get(name) or \
                    ('leg' if name.startswith('task/leg/') else 'bucket_wall' if name.startswith('task/bucket/wall/')
                     else None)
            assert key, name
            expected[g] = key
            assert np.array_equal(m.geom_rgba[g], np.float32(workcell.color(key))), name
            assert m.geom_matid[g] == -1, name
        assert set(expected.values()) == {'floor', 'table', 'trim', 'leg', 'bucket_bottom', 'bucket_wall', 'cube',
                                          'fiducial_label', 'fiducial_print'}
    finally:
        scene.close()


class _Brightest:
    """A generator that draws every bound at its top: the appearance variation's brightest look."""

    def uniform(self, low, high, size=None):
        return high if size is None else np.full(size, high, dtype=float)

    def normal(self, size=None):
        return np.ones(size)


def test_the_appearance_variation_leaves_the_robot_black():
    scene = MinervaTaskScene(fixture_map(), NAMES, width=64, height=48, visual_mode='cad', workcell=MEASURED)
    try:
        m, mj = scene.model, scene.mj
        robot = _robot_geoms(scene)
        rgba, matid = m.geom_rgba[robot].copy(), m.geom_matid[robot].copy()
        look = scene.randomize_appearance(_Brightest(), light=.3, color=.1, camera_shift=.01, camera_turn_deg=2.)
        assert look['light_scale'] == pytest.approx(1.3)
        assert not {_name(scene, 'GEOM', g) for g in robot} & set(look['color_factors'])
        assert np.array_equal(m.geom_rgba[robot], rgba) and np.array_equal(m.geom_matid[robot], matid)
        # Rendered at that look, from in front of the robot: every robot pixel stays dark.
        scene._q = np.zeros(len(NAMES))
        scene.reset()
        renderer = mj.Renderer(m, height=240, width=320)
        try:
            options = mj.MjvOption()
            options.geomgroup[:] = scene.visual_groups
            camera = mj.MjvCamera()
            camera.type = mj.mjtCamera.mjCAMERA_FREE
            camera.azimuth, camera.elevation, camera.distance = 90., -25., 1.6
            camera.lookat[:] = (0., -.25, .85)
            renderer.update_scene(scene.data, camera=camera, scene_option=options)
            for flag in (mj.mjtRndFlag.mjRND_SHADOW, mj.mjtRndFlag.mjRND_REFLECTION, mj.mjtRndFlag.mjRND_FOG):
                renderer.scene.flags[flag] = False
            image = renderer.render().copy()
            renderer.enable_segmentation_rendering()
            renderer.update_scene(scene.data, camera=camera, scene_option=options)
            segments = renderer.render().copy()
        finally:
            renderer.close()
        on_robot = (segments[..., 1] == int(mj.mjtObj.mjOBJ_GEOM)) & np.isin(segments[..., 0], robot)
        assert on_robot.mean() > .05  # The robot fills part of the view.
        assert int(image[on_robot].max()) <= RENDERED_BLACK
        # The workcell is still drawn in its own colours: the box and the cube are far brighter.
        boxed = (segments[..., 1] == int(mj.mjtObj.mjOBJ_GEOM)) & np.isin(
            segments[..., 0], [scene._id('geom', 'task/cube/geom'), scene._id('geom', 'task/bucket/bottom')])
        assert boxed.any() and image[boxed].mean() > 3 * RENDERED_BLACK
    finally:
        scene.restore_appearance()
        scene.close()
