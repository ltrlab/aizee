"""The wrist calibration solver on synthetic stills of the table board from the real arm chains: it must recover a
known camera, mount, board placement and tag layout, the arms' joint zero offsets when asked, and shrug off a bad
corner; and it must refuse what it cannot solve."""
import math
from pathlib import Path

import numpy as np
import pytest

from robot_platform.vision.board import load_board
from robot_platform.vision.chain import Chain
from robot_platform.vision.geometry import invert, rotation, rotvec, transform
from robot_platform.vision.synthetic import SIM_CAMERA, SIM_MOUNT, SIM_PLACEMENT, sample_stills
from robot_platform.vision.wrist_calibration import (CalibrationError, CameraSpec, Still, calibrate,
                                                     robot_world_hand_eye)

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / 'urdf/minerva/minerva.urdf'
WRISTS = {'left_wrist': '1_1_02_eb571_960_stp_2', 'right_wrist': '1_1_02_eb571_960_stp'}
RIGHT_CAMERA = SIM_CAMERA * np.r_[1.01, 1.01, 1, 1, 1.1, .9, 1, 1, 1] + np.r_[0, 0, 4, -3, 0, 0, 0, 0, 0]
RIGHT_MOUNT = SIM_MOUNT @ transform(rotation([.02, -.01, .03]), (.004, -.003, .002))


def scene(seed, count, *, noise_px=.2, joint_errors_deg=0.):
    board = load_board()
    rng = np.random.default_rng(seed)
    chains = {name: Chain(URDF, link) for name, link in WRISTS.items()}
    offsets = {t: (rng.normal(0, .004), rng.normal(0, .004), rng.normal(0, math.radians(1))) for t in board.tags}
    truth = dict(cameras={'left_wrist': SIM_CAMERA, 'right_wrist': RIGHT_CAMERA},
                 mounts={'left_wrist': SIM_MOUNT, 'right_wrist': RIGHT_MOUNT}, offsets=offsets, joints={})
    stills = []
    for name, chain in chains.items():
        errors = {j: rng.normal(0, math.radians(joint_errors_deg)) for j in chain.joints[:-1]}
        truth['joints'][name] = errors
        stills += sample_stills(board, chain, name, rng, count, camera=truth['cameras'][name],
                                mount=truth['mounts'][name], placement=SIM_PLACEMENT, offsets=offsets,
                                noise_px=noise_px, joint_errors=errors)
    specs = [CameraSpec(name, chain, (640, 480)) for name, chain in chains.items()]
    return board, specs, stills, truth


def pose_error(found, expected):
    d = invert(np.asarray(expected)) @ np.asarray(found)
    return float(np.linalg.norm(d[:3, 3])) * 1e3, math.degrees(float(np.linalg.norm(rotvec(d[:3, :3]))))


def test_hand_eye_closed_form_is_exact_without_noise():
    rng = np.random.default_rng(3)
    x = transform(rng.normal(size=3), rng.normal(size=3) * .05)
    y = transform(rng.normal(size=3), rng.normal(size=3))
    links = [transform(rng.normal(size=3), rng.normal(size=3)) for _ in range(5)]
    views = [invert(f @ x) @ y for f in links]
    found_x, found_y, condition = robot_world_hand_eye(links, views)
    assert np.allclose(found_x, x, atol=1e-9) and np.allclose(found_y, y, atol=1e-9) and condition < .5


def test_recovers_cameras_mounts_and_the_board_from_synthetic_stills():
    board, specs, stills, truth = scene(1, 14)
    report = calibrate(board, specs, stills)
    for name in WRISTS:
        camera = report['cameras'][name]
        mm, deg = pose_error(camera['mount']['matrix'], truth['mounts'][name])
        assert mm < 1. and deg < .1, (name, mm, deg)
        found = [camera['intrinsics'][k] for k in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2')]
        assert np.allclose(found[:4], truth['cameras'][name][:4], atol=1.5)
        assert np.allclose(found[4:], truth['cameras'][name][4:6], atol=.02)
        assert camera['rms_px'] < .4 and camera['link'] == WRISTS[name]
        assert report['cross_validation']['cameras'][name]['rms_px'] < .5
        assert all(0 < s < 2 for s in camera['mount']['sigma_xyz_mm'])
    mm, deg = pose_error(report['board']['placement']['matrix'], SIM_PLACEMENT)
    # The placement is known only as well as the taped layout's mean: about 5 mm / sqrt(6) a side.
    assert mm < 6. and deg < .3
    # The tags' layout relative to each other comes out of the stills, not from the tape.
    found = np.array([[o['dx_mm'], o['dy_mm']] for o in report['board']['tag_offsets'].values()])
    true = np.array([truth['offsets'][t][:2] for t in sorted(board.tags)]) * 1e3
    assert np.abs((found - found.mean(axis=0)) - (true - true.mean(axis=0))).max() < 2.
    assert report['per_camera_placement']['difference']['translation_mm'] < 2.


def test_fits_joint_zero_offsets_that_would_otherwise_bias_the_mounts():
    board, specs, stills, truth = scene(3, 16, noise_px=.3, joint_errors_deg=.5)
    plain = calibrate(board, specs, stills, folds=0)
    fitted = calibrate(board, specs, stills, joint_offsets=True, folds=0)
    assert plain['fit']['rms_px'] > 2. and fitted['fit']['rms_px'] < .5
    for name in WRISTS:
        mm, deg = pose_error(fitted['cameras'][name]['mount']['matrix'], truth['mounts'][name])
        assert mm < 1. and deg < .1
        offsets = fitted['cameras'][name]['joint_offsets']
        assert list(offsets) == list(truth['joints'][name])  # J1-J5: the mount absorbs J6.
        for joint, value in offsets.items():
            assert abs(value['deg'] - math.degrees(truth['joints'][name][joint])) < .1, joint
    assert fitted['options']['joint_offsets']['left_wrist'][-1] == 'left_arm_j5'


def test_a_misplaced_corner_is_flagged_and_does_not_move_the_answer():
    board, specs, stills, truth = scene(5, 14)
    bad = stills[4]
    tag = sorted(bad.corners)[0]
    bad.corners[tag] = bad.corners[tag] + np.array([[25., -18.], [0., 0.], [0., 0.], [0., 0.]])
    report = calibrate(board, specs, stills, folds=0)
    rows = {row['name']: row for row in report['cameras'][bad.camera]['stills']}
    assert rows[bad.name]['outlier'] and rows[bad.name]['max_px'] > 15
    assert sum(row['outlier'] for row in rows.values()) == 1
    mm, deg = pose_error(report['cameras'][bad.camera]['mount']['matrix'], truth['mounts'][bad.camera])
    assert mm < 1.5 and deg < .15


def test_refuses_too_few_stills_and_unknown_cameras():
    board, specs, stills, _ = scene(7, 6)
    with pytest.raises(CalibrationError, match='left_wrist: 4 stills see the board'):
        calibrate(board, specs, stills[2:])
    with pytest.raises(CalibrationError, match='unknown camera'):
        calibrate(board, specs, stills + [Still('head', {}, {0: np.zeros((4, 2))}, name='h')])
    with pytest.raises(CalibrationError, match="last joint"):
        calibrate(board, specs, stills, joint_offsets={'left_wrist': ('left_arm_j6',)})
    # Stills that see none of the board's tags (the box's 6 and 7 only) do not count.
    off_board = [Still(s.camera, s.joints, {6: c}, name=s.name) for s, c in zip(stills, (s.corners[min(s.corners)]
                                                                                       for s in stills))]
    with pytest.raises(CalibrationError, match='0 stills see the board'):
        calibrate(board, specs, off_board)
