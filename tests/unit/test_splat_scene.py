"""The splat trainer's data (scripts/splat): a head scene capture's format, as the synthetic one writes it, read back
with its depth registered into the colour camera, the starting Gaussians on the scene's surfaces in the world the
capture names, and the compact .splat the headset draws. Training itself (torch, gsplat) runs on the operator
station, not here."""
import json
import sys

import numpy as np
import pytest

from robot_platform.ops.release import ROOT

sys.path.insert(0, str(ROOT / 'scripts/splat'))
import synthetic_scene  # noqa: E402
import train_head_scene  # noqa: E402


@pytest.fixture(scope='module')
def capture(tmp_path_factory):
    return synthetic_scene.write(tmp_path_factory.mktemp('scene') / 'capture', frames=3)


def test_a_capture_is_its_scene_file_pictures_and_depth(capture):
    scene = json.loads((capture / 'scene.json').read_text())
    assert (scene['schema'], scene['kind'], len(scene['frames'])) == (1, 'head_scene', 3)
    for frame in scene['frames']:
        assert (capture / frame['color']).is_file() and (capture / frame['depth']).is_file()
        pose = train_head_scene.pose_matrix(frame['pose'])
        np.testing.assert_allclose(pose[:3, :3] @ pose[:3, :3].T, np.eye(3), atol=1e-9)  # A rotation, from the pose.


def test_the_depth_is_registered_into_the_colour_camera(capture):
    images, poses, color_k, depths = train_head_scene.load(capture)
    assert images.shape == (3, 480, 640, 3) and poses.shape == (3, 4, 4) and depths.shape == (3, 480, 640)
    # The depth sensor sits 15 mm beside the colour camera: moved into the colour camera's view, each reading lands
    # where the colour camera sees that surface at that depth.
    # The colour camera resolves about 1.6x finer than the depth (fx 615 against 383), so the readings cover some
    # 40 % of its pixels: the depth loss fits those alone.
    _image, truth = synthetic_scene.render(poses[0], synthetic_scene.COLOR_K)
    seen = (depths[0] > 0) & (truth > 0)
    assert seen.mean() > .3
    assert np.median(np.abs(depths[0][seen] - truth[seen])) < .002


def test_the_starting_gaussians_lie_on_the_scenes_surfaces(capture):
    images, poses, color_k, depths = train_head_scene.load(capture)
    points, colours = train_head_scene.starting_points(images, poses, color_k, depths)
    assert len(points) > 1000 and colours.min() >= 0 and colours.max() <= 1
    # The table's top (z 0.75 m, the robot frame) carries its share of them, flat to within a voxel.
    on_table = (np.abs(points[:, 0]) < .55) & (points[:, 1] > -.95) & (points[:, 1] < -.35) & (points[:, 2] < .8) \
        & ~((np.abs(points[:, 0]) < .12) & (points[:, 1] > -.72) & (points[:, 1] < -.48))
    assert on_table.sum() > 100
    assert np.all(np.abs(points[on_table, 2] - .75) < .015)


def test_a_capture_taken_elsewhere_in_the_world_builds_the_scene_in_the_world(tmp_path):
    # Minerva stands turned and moved in the world (the board's frame): the frames' poses stay in her frame, each
    # with her pose in the world, and the scene comes out where it is in the world, the table's top at 0.75 m.
    robot_in_world = np.eye(4)
    turn = np.radians(20.)
    robot_in_world[:2, :2] = [[np.cos(turn), -np.sin(turn)], [np.sin(turn), np.cos(turn)]]
    robot_in_world[:3, 3] = (.15, -.05, 0.)
    capture = synthetic_scene.write(tmp_path / 'capture', frames=3, robot_in_world=robot_in_world)
    scene = json.loads((capture / 'scene.json').read_text())
    assert scene['world'] == 'board' and len(scene['frames'][0]['robot_in_world']) == 7
    images, poses, color_k, depths = train_head_scene.load(capture)
    robot_pose = train_head_scene.pose_matrix(scene['frames'][0]['pose'])
    np.testing.assert_allclose(poses[0], robot_in_world @ robot_pose, atol=1e-9)
    points, _colours = train_head_scene.starting_points(images, poses, color_k, depths)
    table = (np.abs(points[:, 0]) < .6) & (points[:, 1] > -1.) & (points[:, 1] < -.3) & (np.abs(points[:, 2] - .75) < .015)
    assert table.sum() > 100


def test_the_compact_splat_keeps_what_shows_most_visible_first():
    means = np.array([[0., 0., 1.], [1., 2., 3.], [4., 5., 6.]])
    log_scales = np.log([[.01, .01, .01], [.05, .05, .02], [.1, .1, .1]])
    quats = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [2., 0., 0., 0.]])
    opacity = np.array([3., 3., -8.])  # The last is all but transparent: left out.
    sh0 = np.array([[0., 0., 0.], [1., -1., 0.], [0., 0., 0.]])
    data = train_head_scene.splat_bytes(means, log_scales, quats, opacity, sh0)
    assert len(data) == 2 * 32
    record = np.frombuffer(data, dtype=[('mean', '<f4', 3), ('scale', '<f4', 3), ('rgba', 'u1', 4), ('rot', 'u1', 4)])
    np.testing.assert_allclose(record['mean'][0], [1., 2., 3.])  # The larger of the two first.
    np.testing.assert_allclose(record['scale'][0], [.05, .05, .02], rtol=1e-6)
    assert list(record['rgba'][1]) == [128, 128, 128, 243]  # Grey from a zero harmonic; sigmoid(3) of 255.
    assert list(record["rgba"][0][:3]) == [199, 56, 128]  # 0.5 +- C0, of 255.
    assert list(record['rot'][0]) == [128, 255, 128, 128] and list(record['rot'][1]) == [255, 128, 128, 128]
