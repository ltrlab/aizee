"""Varying the task scene's look changes what the cameras see and nothing the physics uses."""
import json
from pathlib import Path

import numpy as np
import pytest

from robot_platform.simulation.task_scene import MinervaTaskScene

FIXTURE = json.loads((Path(__file__).parents[1] / 'fixtures/minerva_task_contact_path.json').read_text())
NAMES = FIXTURE['joint_names']


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1., offset=0.) for n in NAMES if 'arm' in n}
    for side in ('left', 'right'):
        mapping[f'{side}_gripper_finger_a_slide'] = dict(actuator=f'{side}_gripper', scale=.01, offset=0.)
    return mapping


@pytest.fixture(scope='module')
def scene():
    scene = MinervaTaskScene(fixture_map(), NAMES, width=96, height=72)
    yield scene
    scene.close()


def test_a_varied_look_is_reproducible_bounded_and_undone(scene):
    physics = {key: getattr(scene.model, key).copy() for key in
               ('geom_size', 'geom_pos', 'geom_friction', 'geom_solref', 'body_mass', 'geom_contype')}
    before = scene.render(('head',))[0]['head']
    first = scene.randomize_appearance(np.random.default_rng(7), light=.3, color=.1, camera_shift=.01,
                                       camera_turn_deg=2.)
    varied = scene.render(('head',))[0]['head']
    second = scene.randomize_appearance(np.random.default_rng(7), light=.3, color=.1, camera_shift=.01,
                                        camera_turn_deg=2.)
    assert first == second  # Each call starts from the scene's own look: the same seed, the same look.
    assert .7 <= first['light_scale'] <= 1.3
    assert all(.9 <= f <= 1.1 for factors in first['color_factors'].values() for f in factors)
    for camera in first['cameras'].values():
        assert max(abs(v) for v in camera['shift_m']) <= .01 and camera['turn_deg'] <= 2.
    assert set(first['cameras']) == {'left_wrist', 'right_wrist', 'head'}
    assert np.abs(varied.astype(int) - before).mean() > 1.
    for key, value in physics.items():
        assert np.array_equal(getattr(scene.model, key), value), key
    scene.restore_appearance()
    assert np.array_equal(scene.render(('head',))[0]['head'], before)
