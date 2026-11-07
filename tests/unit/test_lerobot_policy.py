"""An exported LeRobot policy on the robot: its description is checked, it warms up before it reports ready, it takes
the cameras in its own order at the recorded size, and it refuses a receiver with other joints. The TorchScript call
itself needs torch (checked on the Jetson and coolserver, docs/operator/POLICY_MODE.md); a stand-in runs here."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.data import lerobot_policy
from robot_platform.data.lerobot_policy import KIND, ExportedPolicy, validate_meta
from robot_platform.evaluation.policies import load_policy

JOINTS = [f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)]
JOINTS = JOINTS[:6] + ['left_gripper'] + JOINTS[6:] + ['right_gripper']
META = {
    'kind': KIND,
    'schema': 1,
    'policy_type': 'act',
    'fps': 15.0,
    'joints': JOINTS,
    'camera_roles': ['left_wrist', 'right_wrist', 'head'],
    'image_shape_chw': [3, 240, 320],
    'chunk_size': 100,
}


class Runner:
    """Stands in for the TorchScript module: records its calls, answers a chunk that starts at the state."""

    def __init__(self, path, device):
        self.path, self.device, self.calls = path, device, []

    def __call__(self, state, images):
        self.calls.append((np.asarray(state).copy(), [image.copy() for image in images]))
        return np.tile(np.asarray(state, dtype=np.float32) + 0.1, (100, 1))


def exported(tmp_path, **changes):
    (tmp_path / 'minerva_policy.json').write_text(json.dumps(dict(META, **changes)), encoding='utf-8')
    (tmp_path / 'policy.pt').write_bytes(b'stand-in')
    runners = []

    def runner(path, device):
        runners.append(Runner(path, device))
        return runners[-1]

    return ExportedPolicy(tmp_path, runner=runner), runners


def test_it_warms_up_before_it_is_ready_and_answers_in_its_camera_order(tmp_path):
    policy, (runner,) = exported(tmp_path)
    assert runner.path == tmp_path / 'policy.pt' and len(runner.calls) == 3  # TorchScript's slow first calls.
    assert all(
        state.shape == (14,) and [i.shape for i in images] == [(240, 320, 3)] * 3 for state, images in runner.calls
    )
    assert policy.camera_roles == ('left_wrist', 'right_wrist', 'head') and policy.action_dt == pytest.approx(1 / 15)
    assert policy.name == f'lerobot:act:{tmp_path.name}'
    policy.reset(SimpleNamespace(joint_names=tuple(JOINTS)))
    images = {
        role: np.full((240, 320, 3), value, np.uint8)
        for value, role in enumerate(('head', 'left_wrist', 'right_wrist'))
    }
    chunk = policy({'q': np.arange(14) / 10, 'images': images})
    assert chunk.shape == (100, 14) and chunk.dtype == np.float64 and np.allclose(chunk[0], np.arange(14) / 10 + 0.1)
    state, frames = runner.calls[-1]
    assert [int(frame[0, 0, 0]) for frame in frames] == [1, 2, 0]  # Left wrist, right wrist, head: its order.


def test_it_refuses_other_joints_cameras_and_image_sizes(tmp_path):
    policy, _ = exported(tmp_path)
    with pytest.raises(ValueError, match='trained on the joints'):
        policy.reset(SimpleNamespace(joint_names=tuple(JOINTS[7:] + JOINTS[:7])))
    good = {role: np.zeros((240, 320, 3), np.uint8) for role in policy.camera_roles}
    with pytest.raises(ValueError, match='needs the head camera'):
        policy({'q': np.zeros(14), 'images': {k: v for k, v in good.items() if k != 'head'}})
    with pytest.raises(ValueError, match=r'gave \(480, 640, 3\)'):
        policy({'q': np.zeros(14), 'images': dict(good, head=np.zeros((480, 640, 3), np.uint8))})
    with pytest.raises(ValueError, match='every joint'):
        policy({'q': np.zeros(13), 'images': good})


def test_a_malformed_export_or_answer_is_refused(tmp_path):
    for change, message in (
        ({'kind': 'other'}, 'Not an exported'),
        ({'joints': []}, 'names the joints'),
        ({'camera_roles': ['head', 'head']}, 'once each'),
        ({'image_shape_chw': [1, 240, 320]}, 'RGB'),
        ({'fps': 0}, 'chunk rate'),
    ):
        with pytest.raises(ValueError, match=message):
            validate_meta(dict(META, **change))
    (tmp_path / 'minerva_policy.json').write_text(json.dumps(META), encoding='utf-8')
    with pytest.raises(ValueError, match='not a finite'):
        ExportedPolicy(tmp_path, runner=lambda path, device: lambda state, images: np.zeros((100, 13)))


def test_load_policy_takes_an_exported_directory(tmp_path, monkeypatch):
    (tmp_path / 'minerva_policy.json').write_text(json.dumps(META), encoding='utf-8')
    monkeypatch.setattr(lerobot_policy, 'torchscript_runner', Runner)
    policy = load_policy(f'lerobot:{tmp_path}')
    assert isinstance(policy, ExportedPolicy) and policy.describe()['camera_roles'] == META['camera_roles']
