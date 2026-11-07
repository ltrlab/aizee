"""Camera geometry without OpenCV: transforms, the distortion model and its inverse, homographies, the table board and
the URDF chains the wrist calibration uses (checked against the project's own forward kinematics)."""

import math
from pathlib import Path

import numpy as np
import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.collision import CollisionScene
from robot_platform.vision.board import DEFAULT_BOARD, board_from_dict, load_board
from robot_platform.vision.chain import Chain
from robot_platform.vision.geometry import (
    apply,
    homography,
    invert,
    invert_each,
    matrix_rpy,
    normalize,
    plane_pose,
    project,
    rotation,
    rotations,
    rotvec,
    rpy_matrix,
    transform,
)

ROOT = Path(__file__).resolve().parents[2]
MINERVA = ROOT / 'urdf/minerva/model.json'
WRISTS = {'left_wrist': '1_1_02_eb571_960_stp_2', 'right_wrist': '1_1_02_eb571_960_stp'}
CAMERA = np.array([330.0, 331.0, 322.0, 236.0, -0.32, 0.11, 0.0008, -0.0005, 0.01])


def test_rotation_vectors_round_trip_from_zero_to_a_half_turn():
    rng = np.random.default_rng(0)
    for angle in (0.0, 1e-9, 1e-5, 0.3, 2.0, math.pi - 1e-4, math.pi):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        r = rotation(axis * angle)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12) and np.isclose(np.linalg.det(r), 1.0)
        assert np.allclose(rotation(rotvec(r)), r, atol=1e-9)
    v = rng.normal(size=(6, 3))
    assert np.allclose(rotations(v), [rotation(x) for x in v], atol=1e-12)


def test_urdf_roll_pitch_yaw_round_trips():
    for rpy in ((0.1, -0.4, 2.5), (-3.0, 1.2, -0.7), (0.0, 0.0, math.pi / 2)):
        assert np.allclose(matrix_rpy(rpy_matrix(*rpy)), rpy, atol=1e-12)
    # URDF: about fixed x, then y, then z, so R = Rz(yaw) Ry(pitch) Rx(roll).
    assert np.allclose(rpy_matrix(0.0, 0.0, math.pi / 2) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_transforms_invert_singly_and_in_batches():
    rng = np.random.default_rng(1)
    ts = np.array([transform(rng.normal(size=3), rng.normal(size=3)) for _ in range(4)])
    for t, inverse in zip(ts, invert_each(ts)):
        assert np.allclose(invert(t), inverse) and np.allclose(t @ inverse, np.eye(4), atol=1e-12)
    points = rng.normal(size=(5, 3))
    assert np.allclose(apply(invert(ts[0]), apply(ts[0], points)), points)


def test_normalize_undoes_the_distortion_across_a_wide_lens():
    grid = np.stack(np.meshgrid(np.linspace(-0.9, 0.9, 9), np.linspace(-0.7, 0.7, 7)), axis=-1).reshape(-1, 2)
    points = np.c_[grid, np.ones(len(grid))]
    pixels = project(CAMERA, points)
    assert np.abs(normalize(CAMERA, pixels) - grid).max() < 1e-6


def test_project_matches_opencvs_distortion_formula():
    # cv2.projectPoints with zero rotation and translation, worked by hand for one point.
    x, y = 0.3, -0.2
    r2 = x * x + y * y
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = CAMERA
    radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
    u = fx * (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)) + cx
    v = fy * (y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y) + cy
    assert np.allclose(project(CAMERA, [[x * 2, y * 2, 2.0]]), [[u, v]])


def test_plane_pose_from_a_homography_keeps_the_seen_points_in_front():
    """A steep view: the board's origin is behind the camera while the tags it sees are in front of it."""
    t = transform(rotation([0.2, 0.0, 0.0]) @ rotation([0.0, -math.pi / 3, 0.0]), (-0.25, -0.25, -0.2))
    plane = np.array([[0.5, 0.1], [0.62, 0.1], [0.62, 0.2], [0.5, 0.2], [0.55, 0.32], [0.6, 0.4]])
    points = apply(t, np.c_[plane, np.zeros(len(plane))])
    assert (points[:, 2] > 0.1).all() and t[2, 3] < 0
    h = homography(plane, points[:, :2] / points[:, 2:])
    assert np.allclose(plane_pose(h, plane), t, atol=1e-9)


def test_the_table_board_matches_the_measured_rectangle():
    board = load_board()
    assert board.family == 'DICT_4X4_50' and math.isclose(board.square, 0.04318)
    assert sorted(board.tags) == [0, 1, 2, 3, 4, 5]  # 6 and 7 ride on the box.
    # OpenCV's order: the code's top-left, top-right, bottom-right, bottom-left, its top away from the robot (+y).
    corners = board.corners(0)
    assert np.allclose(corners[3], [0.0, 0.0, 0.0]) and np.allclose(corners[1], [board.square, board.square, 0.0])
    assert np.allclose(board.corners(1)[2], [0.65, 0.0, 0.0], atol=1e-9)  # Outer near-right corner.
    assert np.allclose(board.corners(2)[1], [0.65, 0.441, 0.0], atol=1e-9)
    assert np.allclose(board.corners(3)[0], [0.0, 0.441, 0.0], atol=1e-9)
    assert board.sigma_m == 0.005
    moved = board.corners(0, (0.001, -0.002, math.pi / 2))
    assert np.allclose(moved.mean(axis=0), [board.tags[0][0] + 0.001, board.tags[0][1] - 0.002, 0.0])
    assert np.allclose(moved[0], moved.mean(axis=0) + [-board.square / 2, -board.square / 2, 0.0])


def test_board_files_are_checked():
    import json

    data = json.loads(DEFAULT_BOARD.read_text(encoding='utf-8'))
    with pytest.raises(ValueError, match='minerva_table_board'):
        board_from_dict(dict(data, kind='other'))
    with pytest.raises(ValueError, match='finite'):
        board_from_dict(dict(data, tags=dict(data['tags'], **{'0': {'center_m': [float('nan'), 0.0]}})))


def test_wrist_chains_match_the_projects_forward_kinematics():
    profile = ModelProfile.load(MINERVA)
    model = UrdfModel(profile)
    scene = CollisionScene.from_urdf(
        profile.urdf,
        profile.package_root,
        mesh_resolver=profile.resolve_mesh,
        mesh_bounds=lambda _: [[-0.01] * 3, [0.01] * 3],
    )
    rng = np.random.default_rng(2)
    bounds = {n: model.by_name[n]['control_bounds'] for n in model.independent}
    chains = {name: Chain(profile.urdf, link) for name, link in WRISTS.items()}
    assert chains['left_wrist'].joints == tuple(f'left_arm_j{k}' for k in range(1, 7))
    for _ in range(5):
        q = {n: float(rng.uniform(*bounds[n])) for n in model.independent}
        expected = scene.link_transforms(q)
        for name, chain in chains.items():
            assert np.allclose(chain.pose(q), expected[WRISTS[name]], atol=1e-12)
    states = [{n: float(rng.uniform(*bounds[n])) for n in model.independent} for _ in range(4)]
    chain = chains['right_wrist']
    offsets = rng.normal(0, 0.01, len(chain.joints))
    batch = chain.poses(chain.positions(states), offsets)
    for state, pose in zip(states, batch):
        assert np.allclose(pose, chain.pose(state, dict(zip(chain.joints, offsets))), atol=1e-12)


def test_chains_refuse_unknown_links_and_missing_joints():
    profile = ModelProfile.load(MINERVA)
    with pytest.raises(ValueError, match='No link'):
        Chain(profile.urdf, 'no_such_link')
    chain = Chain(profile.urdf, WRISTS['left_wrist'])
    with pytest.raises(ValueError, match='left_arm_j6'):
        chain.pose({f'left_arm_j{k}': 0.0 for k in range(1, 6)})
