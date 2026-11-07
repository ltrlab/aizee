"""Atomic raw capture contract; no hardware, model weights or camera devices."""

import json
import threading

import h5py
import numpy as np
import pytest

from robot_platform.data.task_capture import AXES, CAMERAS, TaskEpisodeWriter

NAMES = list(AXES)


def packet(i, *, real=False, authority='human'):
    return dict(
        q=np.full(16, i, dtype=float),
        qcmd=np.full(16, i + 0.1),
        source_time=100.0 + i * 0.05,
        scene_time=None if real else i * 0.05,
        images={n: np.full((8, 12, 3), i, dtype=np.uint8) for n in CAMERAS},
        camera_times={n: (200.0 if real else 0.0) + i * 0.05 for n in CAMERAS},
        task_state={
            'training_eligible': True,
            'receiver_stamp': i * 50,
            'camera_provenance': {n: {'kind': 'fixture'} for n in CAMERAS},
        },
        authority=authority,
    )


def metadata(real=False):
    return (
        {'data_source': 'real'} if real else {'data_source': 'simulation', 'evidence': 'synthetic', 'run_id': 'fixture'}
    )


@pytest.mark.parametrize('real', [False, True])
def test_atomic_publication_raw_pairing_clocks_and_torque_missingness(tmp_path, real):
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata(real))
    for i in range(4):
        writer.append(**packet(i, real=real, authority='human' if i < 2 else 'correction'))
    assert not writer.path.exists()
    path = writer.finish()
    assert path.exists() and not writer.partial_path.exists()
    assert not list(tmp_path.glob('episode_*.hdf5'))  # No unsafe legacy training discovery.
    with h5py.File(path) as f:
        assert f.attrs['complete'] and f.attrs['format_version'] == 9
        assert f.attrs['data_source'] == ('real' if real else 'simulation')
        assert not f.attrs['training_promoted']
        assert f['observations/qpos'].shape == (4, 16)
        assert np.allclose(f['observations/qcmd'][:, 0], [0.1, 1.1, 2.1, 3.1])
        assert np.allclose(f['actions'][:, 0], [1.1, 2.1, 3.1, 3.1])
        assert f['action_valid'][:].tolist() == [True, False, True, False]
        assert np.isnan(f['observations/torques'][:]).all()
        assert not f['observations/torque_valid'][:].any()
        assert np.isnan(f['observations/qvel'][:]).all() and not f['observations/qvel_valid'][:].any()
        assert np.isnan(f['observations/head_camera_pose'][:]).all() and not f['timestamps/head_valid'][:].any()
        assert np.allclose(f['timestamps/telem'][:], [100.0, 100.05, 100.1, 100.15])
        assert f['timestamps/scene_valid'][:].tolist() == [not real] * 4
        assert all(f[f'observations/images/{n}'].shape == (4, 8, 12, 3) for n in CAMERAS)
        assert json.loads(f['task_state'][2])['receiver_stamp'] == 100
    assert writer.status()['state'] == 'complete'
    assert writer.finish() == path


@pytest.mark.parametrize(
    'problem', ['q_nan', 'stale_camera', 'stale_receiver', 'missing_camera', 'wrong_scene_clock', 'source_gap']
)
def test_rejected_frame_quarantines_entire_episode(tmp_path, problem):
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata())
    writer.append(**packet(0))
    bad = packet(1)
    if problem == 'q_nan':
        bad['q'][3] = np.nan
    if problem == 'stale_camera':
        bad['camera_times']['head'] = 0.0
    if problem == 'stale_receiver':
        bad['source_time'] = 100.0
    if problem == 'missing_camera':
        del bad['images']['left_wrist']
    if problem == 'wrong_scene_clock':
        bad['scene_time'] = None
    if problem == 'source_gap':
        bad['source_time'] = 100.3
    with pytest.raises(ValueError):
        writer.append(**bad)
    with pytest.raises(RuntimeError):
        writer.finish()
    assert not writer.path.exists()
    assert writer.status()['error']


def test_abort_is_nonblocking_and_never_publishes(tmp_path):
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata())
    writer.append(**packet(0))
    writer.append(**packet(1))
    writer.abort('camera disconnected')
    with pytest.raises(RuntimeError, match='camera disconnected'):
        writer.finish()
    assert not writer.path.exists()
    with h5py.File(writer.partial_path) as f:
        assert not f.attrs['complete']


def test_abort_returns_before_disk_cleanup_but_owner_wait_is_bounded(tmp_path, monkeypatch):
    entered, release, aborted = threading.Event(), threading.Event(), threading.Event()
    original = TaskEpisodeWriter._run

    def blocked(self):
        entered.set()
        assert release.wait(3), 'fixture writer was not released'
        original(self)

    monkeypatch.setattr(TaskEpisodeWriter, '_run', blocked)
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata())
    assert entered.wait(1)

    def abort():
        writer.abort('original camera failure')
        aborted.set()

    caller = threading.Thread(target=abort)
    caller.start()
    try:
        assert aborted.wait(0.2), 'abort waited for the blocked writer'
        with pytest.raises(TimeoutError, match='integrity is unconfirmed'):
            writer.wait_closed(timeout=0.01)
        writer.abort('later generic session closure')
        assert writer.status()['error'] == 'original camera failure'
    finally:
        release.set()
        caller.join(1)
        writer.wait_closed(timeout=2)
    assert not writer.path.exists() and not writer._thread.is_alive()
    with h5py.File(writer.partial_path, 'r') as file:
        assert not file.attrs['complete']
        assert 'original camera failure' in file.attrs['error']


def test_queue_overflow_never_drops_silently(tmp_path, monkeypatch):
    started, release = threading.Event(), threading.Event()
    original = TaskEpisodeWriter._run

    def delayed(self):
        started.set()
        release.wait(5)
        original(self)

    monkeypatch.setattr(TaskEpisodeWriter, '_run', delayed)
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata(), max_pending=1)
    assert started.wait(2)
    try:
        writer.append(**packet(0))
        with pytest.raises(RuntimeError, match='overflow'):
            writer.append(**packet(1))
    finally:
        release.set()
    with pytest.raises(RuntimeError, match='overflow'):
        writer.finish()
    assert not writer.path.exists()


def test_one_frame_and_scripted_fixture_are_not_training_demos(tmp_path):
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata())
    writer.append(**packet(0))
    with pytest.raises(RuntimeError, match='at least two'):
        writer.finish()
    fixture = TaskEpisodeWriter(tmp_path, NAMES, dict(metadata(), collection_mode='scripted_fixture'))
    fixture.append(**packet(0, authority='scripted_fixture'))
    fixture.append(**packet(1, authority='scripted_fixture'))
    with h5py.File(fixture.finish()) as f:
        assert not f.attrs['capture_eligible']


def test_abort_during_final_fsync_cannot_publish(tmp_path, monkeypatch):
    from robot_platform.data import task_capture

    syncing, release = threading.Event(), threading.Event()
    original = task_capture.os.fsync

    def blocked_sync(fd):
        syncing.set()
        assert release.wait(5)
        return original(fd)

    monkeypatch.setattr(task_capture.os, 'fsync', blocked_sync)
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata())
    writer.append(**packet(0))
    writer.append(**packet(1))
    errors = []

    def finish():
        try:
            writer.finish()
        except RuntimeError as exc:
            errors.append(str(exc))

    thread = threading.Thread(target=finish)
    thread.start()
    try:
        assert syncing.wait(3)
        writer.abort('session closed during final sync')
    finally:
        release.set()
        thread.join(5)
    assert errors and 'session closed' in errors[0]
    assert not writer.path.exists() and not writer.status()['complete']
    with h5py.File(writer.partial_path) as f:
        assert not f.attrs['complete']


@pytest.mark.parametrize('roles', [('right_wrist', 'head'), ('left_wrist', 'right_wrist', 'head')])
def test_explicit_camera_roster_without_fabricated_images(tmp_path, roles):
    writer = TaskEpisodeWriter(tmp_path, NAMES, dict(metadata(), camera_roles=list(roles)))
    for i in range(2):
        row = packet(i)
        row['images'] = {name: row['images'][name] for name in roles}
        row['camera_times'] = {name: row['camera_times'][name] for name in roles}
        writer.append(**row)
    with h5py.File(writer.finish()) as f:
        assert set(f['observations/images']) == set(roles)
        assert json.loads(f.attrs['camera_roles']) == list(roles)


@pytest.mark.parametrize('roles', [[], ['head', 'head'], ['unknown'], 'head'])
def test_invalid_camera_roster_is_rejected(tmp_path, roles):
    with pytest.raises(ValueError, match='camera_roles'):
        TaskEpisodeWriter(tmp_path, NAMES, dict(metadata(), camera_roles=roles))


def test_missing_declared_camera_or_undeclared_camera_aborts(tmp_path):
    for roles in [('right_wrist', 'head'), ('left_wrist', 'right_wrist', 'head')]:
        writer = TaskEpisodeWriter(tmp_path, NAMES, metadata(), camera_roles=roles)
        row = packet(0)
        if len(roles) == 3:
            del row['images']['left_wrist']
        with pytest.raises(ValueError, match='declared camera_roles'):
            writer.append(**row)
        with pytest.raises(RuntimeError):
            writer.finish()
        assert not writer.path.exists()


def test_whole_robot_frame_records_torques_velocities_and_the_head(tmp_path):
    """Format 9: the 16 axes' measured effort and velocity, and the head's sample (its source time, the neck's CAD
    angles and the head camera's pose), each with its validity."""
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata(True))
    for i in range(3):
        frame = packet(i, real=True)
        frame.update(
            torques=np.arange(16) * 0.1 + i,
            velocities=np.full(16, -0.5),
            head=dict(
                time=frame['source_time'] - 0.02,
                angles=[0.1 * i, -0.2],
                camera_pose=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
            ),
        )
        writer.append(**frame)
    with h5py.File(writer.finish()) as f:
        assert f.attrs['minerva_joints'].split(',') == NAMES and f.attrs['authority_scope'] == 'whole_robot'
        assert f.attrs['head_joints'] == 'neck_z,neck_y'
        assert (
            np.allclose(f['observations/torques'][1], np.arange(16) * 0.1 + 1)
            and f['observations/torque_valid'][:].all()
        )
        assert np.allclose(f['observations/qvel'][:], -0.5) and f['observations/qvel_valid'][:].all()
        assert np.allclose(f['observations/neck_angles'][:, 0], [0.0, 0.1, 0.2])
        assert np.allclose(f['observations/head_camera_pose'][2], [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        assert (
            np.allclose(f['timestamps/head'][:], f['timestamps/telem'][:] - 0.02)
            and f['timestamps/head_valid'][:].all()
        )


def test_the_arms_alone_are_not_a_whole_robot_episode(tmp_path):
    with pytest.raises(ValueError, match='16 named axes'):
        TaskEpisodeWriter(tmp_path, NAMES[:14], metadata())


@pytest.mark.parametrize('problem', ['future_head', 'head_nan', 'short_pose', 'torque_inf', 'short_torques'])
def test_a_bad_head_or_torque_reading_quarantines_the_episode(tmp_path, problem):
    writer = TaskEpisodeWriter(tmp_path, NAMES, metadata(True))
    writer.append(**packet(0, real=True))
    bad = packet(1, real=True)
    head = dict(time=bad['source_time'] - 0.01, angles=[0.0, 0.0], camera_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    if problem == 'future_head':
        head['time'] = bad['source_time'] + 0.001
    if problem == 'head_nan':
        head['angles'] = [np.nan, 0.0]
    if problem == 'short_pose':
        head['camera_pose'] = [0.0, 0.0, 0.0]
    bad['head'] = head
    if problem == 'torque_inf':
        bad['torques'] = np.full(16, np.inf)
    if problem == 'short_torques':
        bad['torques'] = np.zeros(14)
    with pytest.raises(ValueError):
        writer.append(**bad)
    with pytest.raises(RuntimeError):
        writer.finish()
    assert not writer.path.exists()
