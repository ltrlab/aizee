"""The digital twin's correction without hardware (vision.twin): the arms' joint zeros from the wrist calibration's
joint-offset model, the head's pan zero from its camera's alignment against the same board, each only for what it was
made through, and the head camera's pose and a map's board turned with it. Helm's flow is in test_helm_twin.py."""

import json
import math

import numpy as np
import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.neck import head
from robot_platform.neck.head_pose import ALIGNMENT_KIND, CameraPose, aligned_mount, reference_digest
from robot_platform.ops.release import ROOT
from robot_platform.vision import head_alignment as ha
from robot_platform.vision import twin
from robot_platform.vision.geometry import invert, transform
from robot_platform.vision.synthetic import SIM_PLACEMENT
from tests.unit.test_head_alignment import REFERENCE, apart, rig, synthetic, write_run  # noqa: F401

RUN = '20261101T000000Z'
# A joint-offset solve's zeros, degrees: where each joint really is beyond what it reports.
ARMS = {
    'left_wrist': {
        'left_arm_j1': 0.4,
        'left_arm_j2': -0.3,
        'left_arm_j3': 0.2,
        'left_arm_j4': -0.6,
        'left_arm_j5': 0.5,
    },
    'right_wrist': {
        'right_arm_j1': -0.2,
        'right_arm_j2': 0.1,
        'right_arm_j3': 0.3,
        'right_arm_j4': 0.25,
        'right_arm_j5': -0.45,
    },
}


def wrist_run(
    folder, *, recommended='joint_offsets', offsets=ARMS, references=('arms-a', 'arms-a'), placement=SIM_PLACEMENT
):
    """A solved wrist run as vision.solve writes it, its stills each with the arms' reference Helm recorded."""
    run = folder / RUN
    run.mkdir(parents=True)
    board = {'placement': {'matrix': placement.tolist()}}
    cameras = {
        camera: {'joint_offsets': {j: {'deg': d, 'sigma_deg': 0.05} for j, d in joints.items()}}
        for camera, joints in offsets.items()
    }
    (run / 'result.json').write_text(
        json.dumps(
            {
                'recommended': recommended,
                'models': {
                    'plain': {'board': board, 'cameras': {}},
                    'joint_offsets': {'board': board, 'cameras': cameras},
                },
            }
        ),
        encoding='utf-8',
    )
    (run / 'stills.jsonl').write_text(
        ''.join(json.dumps({'name': f'{k:03d}', 'reference_sha256': r}) + '\n' for k, r in enumerate(references)),
        encoding='utf-8',
    )
    return folder


def camera_file(rig, pan_deg, *, reference=REFERENCE, moved=np.eye(4)):
    """A saved head camera alignment whose board is the arms' (SIM_PLACEMENT) as a head `pan_deg` off its zero puts
    it, `moved` further in Minerva's frame."""
    turn = twin.correction(rig['chain'], {'neck_z': math.radians(pan_deg)})
    return {
        'schema': 1,
        'kind': ALIGNMENT_KIND,
        'link': rig['config']['link'],
        'run': '20261101T010000Z',
        'reference_sha256': reference_digest(reference),
        'mount': {'matrix': rig['truth'].tolist()},
        'board_in_robot': {'matrix': (moved @ invert(turn) @ SIM_PLACEMENT).tolist()},
    }


def propose(rig, wrists, camera=None, reference=None, arms='arms-a'):
    return twin.propose(wrists, arms, camera, reference, rig['chain'], ha.board_middle(rig['board']))


def head_model():
    profile = head.load_profile(ROOT / head.PROFILE)
    return UrdfModel(ModelProfile.load(ROOT / profile['model_profile'])), profile


def test_a_pan_zero_error_is_drawn_back_onto_the_true_camera(rig, tmp_path):
    # The head solve fits as well with its pan zero 2 degrees off, its board turned instead; against the arms' board
    # the twin finds the turn, and the camera's pose through it is where the camera really is.
    pan_error = math.radians(2.0)
    write_run(
        tmp_path / 'head',
        synthetic(rig, pan_error=pan_error),
        references={k: reference_digest(REFERENCE) for k in range(10)},
    )
    camera = ha.alignment_document(ha.solve_run(tmp_path / 'head'), saved_at='now')
    proposal = propose(rig, wrist_run(tmp_path / 'wrists', recommended='plain'), camera, REFERENCE)
    assert proposal['arms'] is None and 'nothing to correct' in proposal['notes'][0]
    part = proposal['head']
    assert part['offsets_deg']['neck_z'] == pytest.approx(2.0, abs=0.1) and part['other_mm'] < 2.0
    offsets = twin.head_offsets(twin.document(proposal, saved_at='now'), REFERENCE, camera)
    model, profile = head_model()
    mount = aligned_mount(camera, rig['config']['link'], REFERENCE)
    corrected, measured = (
        CameraPose(model, profile['camera'], mount, offsets),
        CameraPose(model, profile['camera'], mount),
    )
    for q in ({'neck_z': 0.3, 'neck_y': -0.7}, {'neck_z': -0.4, 'neck_y': -1.1}):
        truth = rig['chain'].pose(q, {'neck_z': pan_error}) @ rig['truth']
        mm, deg = apart(corrected.pose(q), truth)
        assert mm < 1.5 and deg < 0.15
        assert apart(measured.pose(q), truth)[1] == pytest.approx(2.0, abs=0.15)
    # A map's board turns with the camera's poses: the head's placement of it, turned, is the arms'.
    mm, deg = apart(
        twin.correction(rig['chain'], offsets) @ np.array(camera['board_in_robot']['matrix']), SIM_PLACEMENT
    )
    assert mm < 1.5 and deg < 0.1


def test_the_arms_part_is_the_joint_offset_models_through_the_arms_reference(rig, tmp_path):
    proposal = propose(rig, wrist_run(tmp_path / 'a'))
    assert proposal['wrists_run'] == RUN and proposal['head'] is None
    assert proposal['arms']['offsets_deg'] == {**ARMS['left_wrist'], **ARMS['right_wrist']}
    assert proposal['arms']['sigma_deg']['left_arm_j1'] == 0.05 and proposal['arms']['run'] == RUN
    assert proposal['notes'] == ['Head: no head camera alignment saved']
    document = twin.document(proposal, saved_at='now')
    assert twin.arm_offsets(document, 'arms-a') == pytest.approx(
        {j: math.radians(d) for j, d in proposal['arms']['offsets_deg'].items()}
    )
    with pytest.raises(ValueError, match='zeroed again since it was applied'):
        twin.arm_offsets(document, 'arms-b')
    for folder, message in (
        (wrist_run(tmp_path / 'b', references=('arms-a', 'arms-b')), 'zeroed again since run'),
        (wrist_run(tmp_path / 'c', references=(None,)), 'before Helm recorded it'),
        (
            wrist_run(tmp_path / 'd', offsets={'left_wrist': {'left_arm_j2': 12.0}}),
            'left_arm_j2 more than 10°, beyond a zero error',
        ),
    ):
        found = propose(rig, folder)
        assert found['arms'] is None and message in found['notes'][0], found['notes']
    plain = propose(rig, wrist_run(tmp_path / 'e', recommended='plain'))
    assert plain['arms'] is None and "the URDF's joint zeros fit run" in plain['notes'][0]
    with pytest.raises(ValueError, match='Nothing to correct'):
        twin.document(plain, saved_at='now')
    nothing = propose(rig, tmp_path / 'none')
    assert nothing['wrists_run'] is None and 'No solved wrist calibration' in nothing['notes'][0]


def test_the_head_part_holds_only_for_its_alignment_reference_and_one_board(rig, tmp_path):
    wrists = wrist_run(tmp_path, recommended='plain')
    camera = camera_file(rig, 1.5)
    proposal = propose(rig, wrists, camera, REFERENCE)
    part = proposal['head']
    assert part['offsets_deg'] == pytest.approx({'neck_z': 1.5}, abs=1e-3) and part['other_mm'] < 0.1
    assert part['wrists_run'] == RUN and part['alignment_run'] == '20261101T010000Z' and part['board_mm'] > 5.0
    document = twin.document(proposal, saved_at='now')
    assert twin.head_offsets(document, REFERENCE, camera) == pytest.approx({'neck_z': math.radians(1.5)}, abs=1e-5)
    with pytest.raises(ValueError, match='another reference since it was applied'):
        twin.head_offsets(document, dict(REFERENCE, motors=[]), camera)
    with pytest.raises(ValueError, match='aligned again since it was applied'):
        twin.head_offsets(document, REFERENCE, camera_file(rig, 1.4))
    with pytest.raises(ValueError, match='no saved alignment'):
        twin.head_offsets(document, REFERENCE, None)
    # The board moved 5 cm between the wrist and head stills: no turn about the pan explains that.
    moved = propose(rig, wrists, camera_file(rig, 1.5, moved=transform(np.zeros(3), (0.05, 0.0, 0.0))), REFERENCE)
    assert moved['head'] is None and 'was the board moved' in moved['notes'][-1]
    other = propose(rig, wrists, camera_file(rig, 1.5, reference={'motors': []}), REFERENCE)
    assert other['head'] is None and 'another head reference' in other['notes'][-1]
    far = propose(rig, wrists, camera_file(rig, 12.0), REFERENCE)
    assert far['head'] is None and 'beyond a zero error' in far['notes'][-1]
    assert propose(rig, wrists, camera, None)['notes'][-1] == 'Head: the head has no reference'


def test_the_correction_is_a_fixed_turn_for_the_first_joint_only(rig):
    chain = rig['chain']
    turn = twin.correction(chain, {'neck_z': 0.05})
    for q in ({'neck_z': 0.3, 'neck_y': -0.7}, {'neck_z': -1.0, 'neck_y': 0.2}):
        np.testing.assert_allclose(turn @ chain.pose(q), chain.pose(q, {'neck_z': 0.05}), atol=1e-12)
    np.testing.assert_allclose(twin.correction(chain, {}), np.eye(4), atol=1e-12)
    with pytest.raises(ValueError, match="Only neck_z's offset"):
        twin.correction(chain, {'neck_y': 0.05})


def test_a_saved_correction_is_read_back_and_refused_when_it_is_not_one(tmp_path):
    assert twin.load(tmp_path / 'none.json') is None
    path = twin.path(tmp_path, 'minerva-production-01')
    assert path.name == 'minerva-production-01-twin.json'
    path.write_text(json.dumps({'schema': 1, 'kind': 'servo_calibration'}), encoding='utf-8')
    with pytest.raises(ValueError, match='not a twin correction'):
        twin.load(path)
    path.write_text(json.dumps({'schema': 1, 'kind': twin.KIND, 'arms': None, 'head': None}), encoding='utf-8')
    with pytest.raises(ValueError, match="no arms' part"):
        twin.arm_offsets(twin.load(path), 'arms-a')
    with pytest.raises(ValueError, match='no head part'):
        twin.head_offsets(twin.load(path), REFERENCE, None)


def test_a_live_heads_camera_takes_the_pan_correction_only_when_it_holds(rig, tmp_path):
    from robot_platform.helm.head_teleop import HeadTeleop

    camera = camera_file(rig, 1.5)
    document = twin.document(propose(rig, wrist_run(tmp_path, recommended='plain'), camera, REFERENCE), saved_at='now')
    teleop = HeadTeleop.__new__(HeadTeleop)  # Only what choosing the mount reads.
    teleop.model, teleop.profile = head_model()
    teleop.bundle = {'camera': camera, 'twin': document}
    pose = teleop._camera_pose(REFERENCE)
    assert teleop.camera_twin == 'corrected'
    assert teleop.camera_offsets == pytest.approx({'neck_z': math.radians(1.5)}, abs=1e-5)
    q = {'neck_z': 0.2, 'neck_y': -0.6}
    np.testing.assert_allclose(pose.pose(q), rig['chain'].pose(q, teleop.camera_offsets) @ rig['truth'], atol=1e-9)
    teleop.bundle = {'camera': camera_file(rig, 1.4), 'twin': document}  # Aligned again since it was applied.
    pose = teleop._camera_pose(REFERENCE)
    assert pose.offsets == {} and teleop.camera_twin.startswith('none: the head camera was aligned again')
    teleop.bundle = {'camera': camera, 'twin': document}
    teleop._camera_pose(dict(REFERENCE, motors=[]))  # The CAD mount: the pan was never compared through it.
    assert teleop.camera_offsets == {} and teleop.camera_twin.startswith('none: the camera is on its CAD mount')
    teleop.bundle = {'camera': camera, 'twin': None}
    teleop._camera_pose(REFERENCE)
    assert teleop.camera_offsets == {} and teleop.camera_twin == 'none: no correction of the head applied'
