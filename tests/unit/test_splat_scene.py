"""The splat trainer's data (scripts/splat): a head scene capture's format, as the synthetic one writes it, read back
with its depth registered into the colour camera, and the starting Gaussians on the scene's surfaces. Training
itself (torch, gsplat) runs on the operator station, not here."""
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
