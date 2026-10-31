"""The head camera's alignment on the table's tags without hardware: the solve on synthetic stills (the camera's
intrinsics held), what it can and cannot tell from the head alone, and the saved file the head's pose feed reads.
Helm's flow is in test_helm_head_alignment.py."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.neck import head
from robot_platform.neck.head_pose import CameraPose, aligned_mount, cad_mount, reference_digest
from robot_platform.vision import head_alignment as ha
from robot_platform.vision.head_alignment import alignment_document
from robot_platform.vision.board import load_board
from robot_platform.vision.chain import Chain
from robot_platform.vision.geometry import invert, rotvec, transform
from robot_platform.vision.synthetic import SIM_PLACEMENT, sample_stills
from robot_platform.vision.wrist_calibration import CalibrationError

ROOT = Path(__file__).resolve().parents[2]
# The head camera's colour intrinsics as the D435 publishes them (2026-10-31).
INTRINSICS = {'fx': 614.3364868164062, 'fy': 614.7340698242188, 'cx': 313.26251220703125, 'cy': 236.9853515625,
              'width': 640, 'height': 480}
CAMERA = np.array([INTRINSICS['fx'], INTRINSICS['fy'], INTRINSICS['cx'], INTRINSICS['cy'], 0., 0., 0., 0., 0.])
REFERENCE = {'motors': [{'name': 'head_pan', 'raw_reference_rad': 3.1}, {'name': 'head_tilt',
                                                                          'raw_reference_rad': 3.2}]}
RANGES = {'neck_z': (-.6, .6), 'neck_y': (-1.3, -.3)}


@pytest.fixture(scope='module')
def rig():
    config = ha.load_rig()
    chain = Chain(config['urdf'], config['link'])
    cad = cad_mount(chain.pose({name: 0. for name in chain.joints}), config['front'], config['origin_m'])
    # Where the camera really sits: a few millimetres and degrees off the CAD guess.
    truth = cad @ transform(np.radians([1.5, -2., 1.]), (.006, -.004, .008))
    return dict(config=config, chain=chain, cad=cad, truth=truth, board=load_board(config['board']))


def synthetic(rig, count=10, seed=3, pan_error=0.):
    return sample_stills(rig['board'], rig['chain'], 'head', np.random.default_rng(seed), count, camera=CAMERA,
                         mount=rig['truth'], placement=SIM_PLACEMENT, ranges=RANGES, min_tags=3, noise_px=.2,
                         joint_errors={'neck_z': pan_error})


def write_run(folder, stills, intrinsics=INTRINSICS, references=None):
    folder.mkdir(parents=True, exist_ok=True)
    with open(folder / 'stills.jsonl', 'w', encoding='utf-8') as file:
        for k, still in enumerate(stills):
            file.write(json.dumps(dict(name=still.name, camera='head', joints=still.joints, intrinsics=intrinsics,
                                       reference_sha256=(references or {}).get(k, 'r'),
                                       tags={str(t): q.tolist() for t, q in still.corners.items()})) + '\n')


def write_wrists(folder, placement):
    run = folder / '20261031T000000Z'
    run.mkdir(parents=True)
    (run / 'result.json').write_text(json.dumps({'recommended': 'plain', 'models': {'plain': {'board': {
        'placement': {'matrix': placement.tolist()}}}}}), encoding='utf-8')
    return folder


def apart(a, b):
    """(millimetres, degrees) between two rigid transforms."""
    return (float(np.linalg.norm(a[:3, 3] - b[:3, 3])) * 1e3,
            math.degrees(float(np.linalg.norm(rotvec((invert(a) @ b)[:3, :3])))))


# -- the solve ------------------------------------------------------------------------------------------------------

def test_the_solve_finds_the_mount_and_the_board_with_the_cameras_own_intrinsics(rig, tmp_path):
    write_run(tmp_path / 'run', synthetic(rig))
    result = ha.solve_run(tmp_path / 'run', wrists=write_wrists(tmp_path / 'wrists', SIM_PLACEMENT))
    assert 'error' not in result, result.get('error')
    camera = result['report']['cameras']['head']
    assert camera['intrinsics']['fx'] == pytest.approx(INTRINSICS['fx'])  # Held, not fitted.
    assert result['report']['options']['intrinsics'] == 'held' and camera['rms_px'] < .35
    mm, deg = apart(rig['truth'], np.array(camera['mount']['matrix']))
    assert mm < 1.5 and deg < .15
    mm, deg = apart(SIM_PLACEMENT, np.array(result['report']['board']['placement']['matrix']))
    assert mm < 1. and deg < .1
    assert result['cad']['translation_mm'] == pytest.approx(10.8, abs=1.5)  # It reports how far CAD was off.
    assert result['rms_mm'] < .5 and .55 < result['distance_m']['median'] < .8
    assert abs(result['arms']['pan_deg']) < .05 and result['notes'] == []
    summary = ha.summary(json.loads((tmp_path / 'run' / 'result.json').read_text(encoding='utf-8')))
    assert summary['stills'] == 10 and summary['holdout_px'] < .4 and summary['arms']['other_mm'] < 2.


def test_a_pan_zero_error_moves_the_board_and_only_the_arms_can_tell(rig, tmp_path):
    # The head alone fits as well, its mount still right, but the board turns with the pan error about the pan
    # axis; the arms' own placement of the board reads that turn back.
    write_run(tmp_path / 'run', synthetic(rig, pan_error=math.radians(2.)))
    result = ha.solve_run(tmp_path / 'run', wrists=write_wrists(tmp_path / 'wrists', SIM_PLACEMENT))
    camera = result['report']['cameras']['head']
    assert camera['rms_px'] < .35
    mm, deg = apart(rig['truth'], np.array(camera['mount']['matrix']))
    assert mm < 1.5 and deg < .15
    assert apart(SIM_PLACEMENT, np.array(result['report']['board']['placement']['matrix']))[1] == pytest.approx(
        2., abs=.1)
    arms = result['arms']
    assert arms['pan_deg'] == pytest.approx(2., abs=.1) and arms['board_mm'] > 10. and arms['other_mm'] < 2.


def test_without_the_arms_the_solve_says_what_it_cannot_tell(rig, tmp_path):
    write_run(tmp_path / 'run', synthetic(rig))
    result = ha.solve_run(tmp_path / 'run', wrists=tmp_path / 'nothing')
    assert 'arms' not in result and any('pan zero' in note for note in result['notes'])


def test_too_little_turn_or_tilt_is_named(rig, tmp_path):
    stills = sample_stills(rig['board'], rig['chain'], 'head', np.random.default_rng(5), 8, camera=CAMERA,
                           mount=rig['truth'], placement=SIM_PLACEMENT, min_tags=3, noise_px=.2,
                           ranges={'neck_z': (-.04, .04), 'neck_y': (-1.1, -.5)})
    write_run(tmp_path / 'run', stills)
    notes = ha.solve_run(tmp_path / 'run')['notes']
    assert any(note.startswith('neck_z moved only') and 'turn the head more' in note for note in notes)


def test_the_solve_refuses_mixed_or_missing_intrinsics_and_references(rig, tmp_path):
    stills = synthetic(rig, count=6)
    write_run(tmp_path / 'a', stills, intrinsics=None)
    assert 'no intrinsics' in ha.solve_run(tmp_path / 'a')['error']
    write_run(tmp_path / 'b', stills, references={2: 'another'})
    assert "reference changed" in ha.solve_run(tmp_path / 'b')['error']
    with pytest.raises(CalibrationError, match='changed between stills'):
        ha.held_camera([INTRINSICS, dict(INTRINSICS, fx=600.)])
    with pytest.raises(CalibrationError, match='distortion'):
        ha.held_camera([dict(INTRINSICS, coeffs=[.1, 0., 0., 0., 0.])])
    camera, size = ha.held_camera([dict(INTRINSICS, coeffs=[0.] * 5)] * 2)
    assert size == (640, 480) and camera[0] == pytest.approx(614.336, abs=1e-3) and not camera[4:].any()
    write_run(tmp_path / 'c', stills[:3])
    assert 'capture at least 5' in ha.solve_run(tmp_path / 'c')['error']


# -- the saved file, as the head's pose feed reads it ---------------------------------------------------------------

def test_the_saved_mount_applies_only_to_its_link_and_head_reference(rig, tmp_path):
    write_run(tmp_path / 'run', synthetic(rig), references={k: reference_digest(REFERENCE) for k in range(10)})
    result = ha.solve_run(tmp_path / 'run')
    document = alignment_document(result, saved_at='2026-10-31T00:00:00+00:00')
    link = rig['config']['link']
    assert np.allclose(aligned_mount(document, link, REFERENCE), result['report']['cameras']['head']['mount']['matrix'])
    with pytest.raises(ValueError, match='another head reference'):
        aligned_mount(document, link, dict(REFERENCE, motors=[]))
    with pytest.raises(ValueError, match='camera on awg_rgb_camera_5, not part_6'):
        aligned_mount(document, 'part_6', REFERENCE)
    bent = json.loads(json.dumps(document))
    bent['mount']['matrix'][0][0] = 2.
    with pytest.raises(ValueError, match='rigid transform'):
        aligned_mount(bent, link, REFERENCE)
    with pytest.raises(ValueError, match='not a head camera alignment'):
        aligned_mount({'kind': 'servo_calibration'}, link, REFERENCE)


def test_the_head_camera_pose_rides_the_measured_mount(rig):
    profile = head.load_profile(ROOT / head.PROFILE)
    model = UrdfModel(ModelProfile.load(ROOT / profile['model_profile']))
    pose = CameraPose(model, profile['camera'], rig['truth'])
    assert np.allclose(pose.cad, rig['cad'], atol=1e-9)  # One CAD mount, from the model or the URDF alone.
    q = {'neck_z': .3, 'neck_y': -.7}
    assert np.allclose(pose.pose(q), rig['chain'].pose(q) @ rig['truth'], atol=1e-9)
    assert np.allclose(CameraPose(model, profile['camera']).mount, rig['cad'])
