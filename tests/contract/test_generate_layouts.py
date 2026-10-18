"""Generating on drawn training layouts through the actual receiver: a draw the planner refuses is drawn again and
counted, each attempt runs on its own layout, and a recorded episode carries the layout it was drawn with."""
import json
from pathlib import Path

import h5py
import numpy as np

from robot_platform.evaluation.generate import APPEARANCE, EpisodeRecorder, draw_layout, generate
from robot_platform.evaluation.grid import GridSpec
from robot_platform.evaluation.plants import derive_stack
from robot_platform.evaluation.policies import HoldStillPolicy
from robot_platform.evaluation.runner import RunConfig, TrialContext, TrialRunner
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.layouts import LayoutSampler

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = ROOT / 'config/devices/simulation/minerva-production-01-workcell.json'
RANDOMIZATION = ROOT / 'config/devices/simulation/minerva-production-01-randomization.json'


class _Refuses(HoldStillPolicy):
    """Holds still; its planner refuses the first `refusals` layouts it is asked to plan."""

    def __init__(self, refusals):
        super().__init__()
        self.refusals, self.planned = refusals, []

    def reset(self, context):
        self.planned.append(context.workcell)
        if len(self.planned) <= self.refusals:
            raise ValueError('no plan clears the table, the bucket and the other arm (test)')
        super().reset(context)


def test_a_refused_draw_is_drawn_again_and_the_episode_records_its_layout(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)
    runner = TrialRunner(RunConfig(grippers='driven', timeout_s=1., camera_size=(64, 48), workcell=str(WORKCELL)),
                         stack=derive_stack(), directory=tmp_path / 'receiver')
    policy = _Refuses(2)
    looks = dict(APPEARANCE, cameras=('left_wrist', 'right_wrist'))
    recorder = EpisodeRecorder(tmp_path / 'episodes', hz=15., appearance=looks, seed=11)
    try:
        layout, rejections = draw_layout(runner, policy, sampler, [11, 1])
        # Two draws refused, with their reasons and layouts; the third planned. Each draw is its own layout.
        assert [r['draw'] for r in rejections] == [0, 1] and layout.values['seed'] == [11, 1, 2]
        assert all('(test)' in r['reason'] and r['layout']['seed'] == [11, 1, r['draw']] for r in rejections)
        assert len({json.dumps(w.describe(), sort_keys=True) for w in policy.planned}) == 3
        assert policy.planned[-1] == layout.workcell
        # The trial runs on the layout's own scene: its table, bucket, cube and head camera.
        runner.use_workcell(layout.workcell)
        record = runner.run_trial(policy, dict(layout.cube_start, cell='layout', index=1), 1)
        assert record['outcome'] == 'timeout'
        scene, plant = runner.scene, runner.plant
        assert scene.workcell == layout.workcell
        head = scene._id('camera', 'head')
        assert np.allclose(scene.model.cam_pos[head], layout.workcell.head_camera['pos'], atol=1e-12)
        cube = scene.workcell.cube_start
        assert (cube[0], cube[1]) == (layout.cube_start['x'], layout.cube_start['y'])
        # A recording on it carries the drawn layout, and the look's variation leaves the head camera where the
        # layout put it.
        context = TrialContext(index=1, cube_start=dict(layout.cube_start), joint_names=tuple(plant.names),
                               joint_map=plant.joint_map, lower=plant.lower, upper=plant.upper, home=plant.home,
                               period_s=plant.period_s, plant='armplant', workcell=scene.workcell)
        recorder.annotate(layout=dict(layout.describe(), rejected_draws=rejections))
        recorder.start(scene, plant, context, policy)
        assert np.array_equal(scene.model.cam_pos[head], scene._appearance['cam_pos'][head])
        for _ in range(20):
            plant.cycle()
            scene.update(plant.scene_pose(), plant.period_s)
            recorder.sample(scene, plant)
        path = recorder.finish({'outcome': 'success', 'eligible': True})
    finally:
        runner.close()
    with h5py.File(path, 'r') as episode:
        metadata = json.loads(episode.attrs['metadata_json'])
    drawn = metadata['layout']
    assert drawn['seed'] == [11, 1, 2] and len(drawn['rejected_draws']) == 2
    assert set(drawn) >= {'cube', 'bucket', 'table_offset', 'head_camera', 'randomization', 'look'}
    # The drawn look (the robot's grey, the room's and the floor's tones, the lights' warmth) is the scene's.
    assert metadata['scene']['look'] == drawn['look'] == layout.values['look']
    assert drawn['cube']['size_m'] == list(layout.workcell.block) and drawn['bucket']['height_m'] == \
        layout.workcell.bucket_height
    assert metadata['scene']['workcell'] == layout.workcell.describe()
    assert set(metadata['appearance']['cameras']) == {'left_wrist', 'right_wrist'}
    # The next trial's metadata does not inherit this layout.
    assert recorder.trial_metadata == {}


def test_generate_draws_a_layout_for_every_attempt_and_lists_them(tmp_path):
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)
    # Seed 24's first draws pass the head-camera checks (layout_checks), so only the policy refuses one.
    config = RunConfig(plant='armplant', timeout_s=.5, grid=GridSpec(seed=24, nominal=False), camera_size=(64, 48),
                       workcell=str(WORKCELL), grippers='driven')
    made, attempts = generate(_Refuses(1), config, tmp_path, episodes=1, stack=derive_stack(), layouts=sampler,
                              attempts=2)
    assert made == [] and attempts == 2
    entries = [json.loads(line) for line in (tmp_path / 'manifest.jsonl').read_text(encoding='utf-8').splitlines()]
    assert [e['attempt'] for e in entries] == [1, 2] and [e['layout_seed'] for e in entries] == [[24, 1], [24, 2]]
    # The first attempt's first draw was refused; every attempt then ran its own layout, which held still.
    assert [e['rejected_draws'] for e in entries] == [1, 0] and entries[0]['rejections'][0]['draw'] == 0
    assert '(test)' in entries[0]['rejections'][0]['reason']
    assert all(e['outcome'] == 'timeout' and e['episode'] is None for e in entries)
    assert entries[0]['layout']['seed'] == [24, 1, 1] and entries[1]['layout']['seed'] == [24, 2, 0]
    assert entries[0]['cube_start'] != entries[1]['cube_start']


def test_generate_draws_again_when_the_bucket_leaves_the_head_image(tmp_path):
    """generate checks each draw with layout_checks before planning it: seed 42's first draw puts a corner of the
    bucket's opening outside the head image, so the attempt counts it and runs its next draw."""
    assert default_binary().is_file(), 'Required contract: build the actual native control_harness first'
    sampler = LayoutSampler(WORKCELL, RANDOMIZATION)
    config = RunConfig(plant='armplant', timeout_s=.5, grid=GridSpec(seed=42, nominal=False), camera_size=(64, 48),
                       workcell=str(WORKCELL), grippers='driven')
    generate(_Refuses(0), config, tmp_path, episodes=1, stack=derive_stack(), layouts=sampler, attempts=1)
    (entry,) = [json.loads(line) for line in (tmp_path / 'manifest.jsonl').read_text(encoding='utf-8').splitlines()]
    assert entry['rejected_draws'] == 1 and entry['rejections'][0]['draw'] == 0
    assert entry['rejections'][0]['reason'].startswith("head camera: the bucket's opening is not whole")
    assert entry['layout']['seed'] == [42, 1, 1] and entry['outcome'] == 'timeout'


def test_a_head_camera_against_the_robot_is_refused():
    """A draw moves the head camera with the table group and about its calibrated pose; near the robot that can put it
    against or inside the torso, which the real camera cannot be. The workcell's own camera stands clear; moved 6 cm
    toward the robot it is refused."""
    from robot_platform.evaluation.generate import HeadCameraClearance
    from robot_platform.simulation.workcell import Workcell, load_workcell
    workcell = load_workcell(WORKCELL)
    check = HeadCameraClearance(derive_stack())
    try:
        assert check(workcell) is None
        description = workcell.describe()
        x, y, z = description['head_camera']['pos']
        description['head_camera']['pos'] = [x, y + .06, z]  # Base +y: toward the robot.
        reason = check(Workcell.from_description(description))
        assert reason is not None and reason.startswith('head camera')
    finally:
        check.close()


def test_a_bucket_whose_opening_leaves_the_head_image_is_refused():
    """The randomization file asks that each layout's head image hold the bucket's whole opening (BucketInView): the
    workcell's own box passes, about 9 px from the image's right edge. With the camera moved 10 cm to its own left the
    box moves right, out of the image, and is refused; the margin is the opening's least distance from an edge. The
    generator checks the camera's clearance first, then the opening (layout_checks)."""
    from robot_platform.evaluation.generate import BucketInView, HeadCameraClearance, layout_checks
    from robot_platform.simulation.layouts import load_randomization
    from robot_platform.simulation.workcell import Workcell, load_workcell
    workcell, randomization = load_workcell(WORKCELL), load_randomization(RANDOMIZATION)
    view = BucketInView(**randomization.bucket_view)
    slack = view.slack(workcell)
    assert view(workcell) is None and slack >= view.margin
    assert BucketInView((320, 240), slack - .5)(workcell) is None
    assert BucketInView((320, 240), slack + .5)(workcell).startswith("head camera: the bucket's opening")
    description = workcell.describe()
    x_axis = np.array(description['head_camera']['xyaxes'][:3])
    description['head_camera']['pos'] = list(np.array(description['head_camera']['pos']) - .10 * x_axis / np.linalg.norm(
        x_axis))
    moved = Workcell.from_description(description)
    assert view.slack(moved) < 0
    reason = view(moved)
    assert reason.startswith("head camera: the bucket's opening") and 'outside' in reason
    checks = layout_checks(derive_stack(), randomization)
    try:
        assert [type(check) for check in checks.checks] == [HeadCameraClearance, BucketInView]
        assert checks(workcell) is None
    finally:
        checks.close()
