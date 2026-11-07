"""An episode's recorded commands as the loop's target: the playback clock, interpolation, the end and rewinding."""

from pathlib import Path

import h5py
import numpy as np
import pytest

from robot_platform.data.task_capture import TaskEpisodeWriter
from robot_platform.teleop.episode_source import SPEEDS, EpisodeSource, validate_request

NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')
ROSTER = NAMES + ('head_pan', 'head_tilt')  # An episode records the whole robot.


def source(speed=1.0):
    # Two joints moving 0 -> 1 rad over 2 s, recorded at 0.5 s.
    times = [10.0, 10.5, 11.0, 11.5, 12.0]
    return EpisodeSource(('a', 'b'), times, [[t - 10, -(t - 10)] for t in times], speed=speed, episode='s/e')


def positions(rows):
    return [round(row['position'], 6) for row in rows]


def test_the_clock_runs_only_while_following_and_engage_aligns_to_the_current_frame():
    episode = source()
    assert positions(episode.target('HOLD', 100.0)) == [0.0, 0.0]  # Enabled: the first frame, to align to.
    assert positions(episode.target('ENGAGING', 100.4)) == [0.0, 0.0]  # Aligning: the clock waits.
    assert positions(episode.target('TELEOP', 101.0)) == [0.0, 0.0]  # First following cycle: no jump.
    assert positions(episode.target('TELEOP', 101.25)) == [0.25, -0.25]  # Interpolated between recorded frames.
    assert positions(episode.target('HOLD', 102.0)) == [0.25, -0.25]  # Paused: the clock stops.
    assert positions(episode.target('ENGAGING', 103.0)) == [0.25, -0.25]  # Engage aligns to where it stopped.
    episode.target('TELEOP', 104.0)
    rows = episode.target('TELEOP', 104.5)
    assert positions(rows) == [0.75, -0.75] and {row['name'] for row in rows} == {'a', 'b'}
    assert all(row['sampled_at'] == 104.5 for row in rows)


def test_every_target_is_a_new_sample_and_the_end_holds_the_last_frame():
    episode = source(speed=0.5)
    sequences = []
    for i in range(30):
        rows = episode.target('TELEOP', 200.0 + i * 0.25)
        sequences.append(rows[0]['sequence'])
    assert sequences == sorted(set(sequences))  # Never a repeated sample: the lifecycle counts those as misses.
    assert episode.finished and episode.finished_at == pytest.approx(204.0)  # 2 s of episode at 0.5x from 200 s.
    assert positions(episode.target('TELEOP', 300.0)) == [2.0, -2.0]  # Held, never extrapolated.
    snap = episode.snapshot()
    assert (snap['time_s'], snap['duration_s'], snap['frame'], snap['frames'], snap['finished']) == (
        2.0,
        2.0,
        4,
        5,
        True,
    )
    episode.rewind()
    assert not episode.finished and positions(episode.target('HOLD', 301.0)) == [0.0, 0.0]


def test_speeds_and_episodes_are_checked():
    for speed in (0, 2.0, 0.3, float('nan')):
        with pytest.raises(ValueError, match='Replay runs at'):
            EpisodeSource(('a',), [0.0, 1.0], [[0.0], [1.0]], speed=speed)
    with pytest.raises(ValueError, match='at least two frames'):
        EpisodeSource(('a', 'b'), [0.0, 1.0], [[0.0], [1.0]])
    with pytest.raises(ValueError, match='must increase'):
        EpisodeSource(('a',), [0.0, 1.0, 1.0], [[0.0], [1.0], [2.0]])
    with pytest.raises(ValueError, match='finite'):
        EpisodeSource(('a',), [0.0, 1.0], [[0.0], [float('inf')]])
    assert validate_request({'episode': 'live-20261022T100000Z/task_episode_' + 'a' * 32}) == {
        'kind': 'episode',
        'episode': 'live-20261022T100000Z/task_episode_' + 'a' * 32,
        'speed': 1.0,
    }
    assert validate_request({'episode': 's/e', 'speed': 0.5})['speed'] == 0.5
    for bad in (
        {'speed': 1},
        {'episode': 's/e', 'speed': 4},
        {'episode': 'nested/s/e'},
        {'episode': 's/e', 'x': 1},
        [],
    ):
        with pytest.raises(ValueError):
            validate_request(bad)
    assert SPEEDS == (0.25, 0.5, 1.0)


def record(directory, frames, *, finish=True):
    writer = TaskEpisodeWriter(
        directory, ROSTER, {'data_source': 'real', 'camera_roles': ['head']}, hz=15.0, max_pending=64
    )
    for i in range(frames):
        writer.append(
            q=[j + i / 100 for j in range(16)],
            qcmd=[j + i / 100 + 0.001 for j in range(16)],
            source_time=10.0 + i / 15,
            scene_time=None,
            images={'head': np.zeros((4, 6, 3), np.uint8)},
            camera_times={'head': 10.0 + i / 15 - 0.01},
            task_state={'i': i},
            authority='human',
        )
    if finish:
        return Path(writer.finish())
    import time

    deadline = time.monotonic() + 10.0
    while writer.status()['written_frames'] < frames and time.monotonic() < deadline:
        time.sleep(0.01)
    writer.abort('test')
    writer.wait_closed(timeout=5.0)
    return Path(writer.partial_path)


def test_a_recorded_episode_drives_the_receivers_joints_in_their_order(tmp_path):
    path = record(tmp_path / 'episodes', 10)
    right = tuple(name for name in NAMES if name.startswith('right'))  # A one-arm receiver.
    episode = EpisodeSource.load(path, right, episode='x/y')
    rows = episode.target('HOLD', 1.0)
    assert [row['name'] for row in rows] == list(right)
    assert positions(rows) == [round(NAMES.index(name) + 0.001, 6) for name in right]  # Its qcmd, by name.
    assert episode.duration == pytest.approx(9 / 15)
    head = EpisodeSource.load(path, ('head_tilt', 'head_pan'))  # The neck's columns, by name.
    assert positions(head.target('HOLD', 1.0)) == [15.001, 14.001]
    with pytest.raises(ValueError, match='has no waist'):
        EpisodeSource.load(path, NAMES + ('waist',))
    partial = record(tmp_path / 'partial', 5, finish=False)
    with pytest.raises(ValueError, match='Only a complete episode'):
        EpisodeSource.load(partial, NAMES)
    with h5py.File(path, 'r') as f:  # The recorded commands, not the measured positions.
        assert np.allclose(f['observations/qcmd'][0], [j + 0.001 for j in range(16)])
