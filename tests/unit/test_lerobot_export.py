"""Exporting kept Minerva episodes to a LeRobot dataset: selection, span, resampling and frames, without LeRobot."""
import json
from pathlib import Path
import subprocess
import sys

import h5py
import numpy as np
import pytest

from robot_platform.data import lerobot_export as export
from robot_platform.data.task_capture import TaskEpisodeWriter
from robot_platform.helm.episode_worker import Episodes

ROOT = Path(__file__).resolve().parents[2]
NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')
ROSTER = NAMES + ('head_pan', 'head_tilt')  # Format 9 records the whole robot.
POSE = [.05, -.1, .6, 0., 0., 0., 1.]
ROLES = ('left_wrist', 'right_wrist', 'head')
SESSION = 'live-20261022T120000Z'


def record(root, times, *, authority=None, lifecycle=None, finish=True, session=SESSION, missing_torque=()):
    """An episode whose row i has qpos[i] = i + j/100 and images at level i (mod 250) on every camera; effort
    i - j/100, velocity j/10 and the head's sample 10 ms before the receiver's, except the rows in missing_torque.
    `lifecycle` gives each row's lifecycle state in its task state, as a policy session's capture does."""
    writer = TaskEpisodeWriter(root / session / 'episodes', ROSTER, {'data_source': 'real', 'camera_roles': list(ROLES)},
                               hz=15., max_pending=256)
    for i, t in enumerate(times):
        torques = [np.nan if i in missing_torque else i - j / 100 for j in range(16)]
        writer.append(q=[i + j / 100 for j in range(16)], qcmd=[i + j / 100 + .5 for j in range(16)], source_time=t,
                      torques=torques, velocities=[j / 10 for j in range(16)],
                      head=dict(time=t - .01, angles=[.1, -.2], camera_pose=POSE),
                      scene_time=None, images={role: np.full((6, 8, 3), i % 250, dtype=np.uint8) for role in ROLES},
                      camera_times={role: t - .02 for role in ROLES},
                      task_state={'row': i, **({'lifecycle': lifecycle[i]} if lifecycle else {})},
                      authority=(authority or {}).get(i, 'human'))
    if finish:
        path = Path(writer.finish())
    else:
        import time
        deadline = time.monotonic() + 10.
        while writer.status()['written_frames'] < len(times) and time.monotonic() < deadline:
            time.sleep(.01)
        writer.abort('test: interrupted')
        writer.wait_closed(timeout=5.)
        path = Path(writer.partial_path)
    return f'{session}/{path.name[:-len(".hdf5")]}', path


def steady(n, start=100., period=1 / 15):
    return [start + i * period for i in range(n)]


def test_the_span_is_the_human_rows_with_a_valid_action():
    assert export.usable_span([True, True, True, False], ['human'] * 4) == (0, 2)  # The terminal row has no action.
    assert export.usable_span([True] * 5 + [False], ['policy', 'policy', 'human', 'human', 'human', 'human']) == (2, 4)
    with pytest.raises(export.ExportError, match='inside the span'):
        export.usable_span([True] * 6, ['human', 'human', 'correction', 'human', 'human', 'human'])
    with pytest.raises(export.ExportError, match='no row'):
        export.usable_span([False, False], ['human', 'human'])


def test_jittery_rows_resample_onto_a_fixed_grid_without_looking_ahead():
    # Demo 1's 20 Hz capture: gaps of ~50 and ~100-120 ms, 14.97 Hz on average.
    gaps = [.05, .05, .1, .05, .11, .05, .05, .1, .06, .05, .12, .05] * 5
    telem = np.cumsum([100.] + gaps)
    rows = export.resample(telem, 0, len(telem) - 1, 15)
    grid = telem[0] + np.arange(len(rows)) / 15
    assert len(rows) == int(np.floor((telem[-1] - telem[0]) * 15 + 1e-9)) + 1  # The last frame ends the grid.
    assert np.all(telem[rows] <= grid + 1e-9) and np.all(np.diff(rows) >= 0)  # The latest frame at or before.
    assert np.all(grid - telem[rows] <= 2.5 / 15)
    steady_rows = export.resample(np.array(steady(30)), 0, 29, 15)
    assert steady_rows.tolist() == list(range(30))  # A steady 15 Hz recording maps one to one.
    dropped = export.resample(np.array(steady(10)[:4] + steady(10)[5:]), 0, 8, 15)  # One head frame dropped: kept.
    assert dropped.tolist() == [0, 1, 2, 3, 3, 4, 5, 6, 7, 8]
    with pytest.raises(export.ExportError, match='a gap of 267 ms'):
        export.resample(np.array([0., .066, .133, .433, .5]), 0, 4, 15)


def test_only_complete_episodes_reviewed_keep_are_selected(tmp_path):
    kept, kept_path = record(tmp_path, steady(10))
    discarded, _ = record(tmp_path, steady(10))
    unreviewed, unreviewed_path = record(tmp_path, steady(10))
    partial, _ = record(tmp_path, steady(10), finish=False)
    episodes = Episodes(tmp_path)
    episodes.review(kept, 'keep', 'clean handoff')
    episodes.review(discarded, 'discard', 'dropped the cube')
    episodes.review(partial, 'discard')
    chosen = export.select(tmp_path)
    assert [path for path, _ in chosen] == [kept_path.resolve()]
    assert chosen[0][1]['note'] == 'clean handoff'
    with_unreviewed = [path for path, _ in export.select(tmp_path, include_unreviewed=True)]
    assert with_unreviewed == [kept_path.resolve(), unreviewed_path.resolve()]  # Never the discarded or partial one.


def test_a_planned_episode_keeps_its_human_span_and_a_different_roster_is_refused(tmp_path):
    first, _ = record(tmp_path, steady(20), authority={0: 'policy', 1: 'policy'})
    other, other_path = record(tmp_path, steady(20))
    episodes = Episodes(tmp_path)
    episodes.review(first, 'keep')
    episodes.review(other, 'keep')
    with h5py.File(other_path, 'r+') as f:  # The same episode as the arms alone named their joints (format 8).
        f.attrs['minerva_joints'] = ','.join(NAMES)
    plans, refused = export.plan(tmp_path, 15)
    assert len(plans) == 1 and plans[0]['first'] == 2 and plans[0]['last'] == 18  # Policy rows out; terminal row out.
    assert plans[0]['rows'].tolist() == list(range(2, 19))
    assert len(refused) == 1 and 'different joints' in refused[0][1]


# A policy session at 15 Hz: the policy drives rows 0-44 (3 s); the operator takes over (hold 45-49, aligning to the
# leaders 50-54) and corrects 55-84; hands back (hold 85-89, aligning to the policy 90-94); the policy drives 95-119.
TAKEOVER = ['policy'] * 45 + ['correction'] * 40 + ['policy'] * 35
STATES = ['TELEOP'] * 45 + ['HOLD'] * 5 + ['ENGAGING'] * 5 + ['TELEOP'] * 30 + ['HOLD'] * 5 + ['ENGAGING'] * 5 + ['TELEOP'] * 25


def valid(authority):
    return [a == b for a, b in zip(authority, authority[1:])] + [False]  # As the episode writer marks actions.


def test_a_policy_sessions_stretches_leave_out_holds_alignments_and_the_policy_just_before_a_take_over():
    telem = np.array(steady(120))
    kept = export.corrected_stretches(telem, valid(TAKEOVER), TAKEOVER, STATES, pre_takeover_s=.9)
    assert [(s['first'], s['last'], s['authority']) for s in kept] == [
        (0, 29, 'policy'), (55, 83, 'correction'), (95, 118, 'policy')]  # Each row's action is a Teleop command.
    assert kept[0]['trimmed_s'] == pytest.approx(14 / 15, abs=1e-3) and kept[1]['trimmed_s'] == kept[2]['trimmed_s'] == 0
    # A camera drop paused the policy (hold, then the policy again): no take over, nothing trimmed.
    paused = ['TELEOP'] * 40 + ['HOLD'] * 10 + ['ENGAGING'] * 5 + ['TELEOP'] * 45
    kept = export.corrected_stretches(telem, valid(['policy'] * 100), ['policy'] * 100, paused)
    assert [(s['first'], s['last'], s['trimmed_s']) for s in kept] == [(0, 38, 0.), (55, 98, 0.)]
    # Taken over, then finished in the hold without correcting: still taken over.
    ended = ['policy'] * 50 + ['correction'] * 10
    kept = export.corrected_stretches(telem, valid(ended), ended, ['TELEOP'] * 50 + ['HOLD'] * 10, pre_takeover_s=.9)
    assert [(s['first'], s['last']) for s in kept] == [(0, 34)]
    # A stretch shorter than the minimum is left out.
    short = ['TELEOP'] * 45 + ['HOLD'] * 5 + ['ENGAGING'] * 5 + ['TELEOP'] * 6 + ['HOLD'] * 59
    kept = export.corrected_stretches(telem, valid(TAKEOVER), TAKEOVER, short, pre_takeover_s=.9)
    assert [s['authority'] for s in kept] == ['policy']


def test_a_policy_episode_exports_each_stretch_as_its_own_episode_saying_who_drove_it(tmp_path):
    kept, _ = record(tmp_path, steady(120), authority=dict(enumerate(TAKEOVER)), lifecycle=STATES)
    unlabelled, _ = record(tmp_path, steady(30), authority={i: 'policy' for i in range(30)})
    episodes = Episodes(tmp_path)
    episodes.review(kept, 'keep', 'took over at the handoff')
    episodes.review(unlabelled, 'keep')
    plans, refused = export.plan(tmp_path, 15, pre_takeover_s=.9)
    assert len(refused) == 1 and 'do not all say their lifecycle state' in refused[0][1]
    assert [(s['first'], s['last'], s['authority']) for s in plans[0]['stretches']] == [
        (0, 29, 'policy'), (55, 83, 'correction'), (95, 118, 'policy')]

    class Dataset:
        frames, saved = [], []

        def add_frame(self, frame):
            Dataset.frames.append(frame)

        def save_episode(self):
            Dataset.saved.append(len(Dataset.frames))
    out = tmp_path / 'dataset'
    export.write(plans, out, 'minerva/handoff', 15, export.TASK, create=lambda **options: Dataset())
    assert Dataset.saved == [30, 59, 83]  # One LeRobot episode a stretch: none spans a hold or an alignment.
    assert np.allclose(Dataset.frames[30]['observation.state'], [55 + j / 100 for j in range(16)])
    sidecar = json.loads((out / 'meta' / 'minerva_provenance.json').read_text(encoding='utf-8'))
    assert [(e['episode_index'], e['source'], e['source_span'], e['authority']) for e in sidecar['episodes']] == [
        (0, kept, [0, 29], 'policy'), (1, kept, [55, 83], 'correction'), (2, kept, [95, 118], 'policy')]
    assert sidecar['episodes'][0]['trimmed_s'] == pytest.approx(14 / 15, abs=1e-3)


def test_frames_carry_state_action_cameras_and_task_and_the_sidecar_records_provenance(tmp_path):
    kept, path = record(tmp_path, steady(12))
    Episodes(tmp_path).review(kept, 'keep', 'good')
    plans, refused = export.plan(tmp_path, 15)
    assert not refused

    class Dataset:
        created, frames, saved = None, [], 0

        def add_frame(self, frame):
            Dataset.frames.append(frame)

        def save_episode(self):
            Dataset.saved += 1

    def create(**options):
        Dataset.created = options
        return Dataset()
    out = tmp_path / 'dataset'
    export.write(plans, out, 'minerva/handoff', 15, export.TASK, create=create)
    options = Dataset.created
    assert (options['fps'], options['robot_type'], options['use_videos']) == (15, 'minerva', True)
    feature = options['features']
    assert feature['observation.state'] == {'dtype': 'float32', 'shape': (16,), 'names': list(ROSTER)}
    assert feature['action']['names'] == list(ROSTER)
    assert feature['observation.effort'] == feature['observation.velocity'] == feature['observation.state']
    assert feature['observation.head_camera_pose'] == {'dtype': 'float32', 'shape': (7,),
                                                       'names': ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']}
    assert feature['observation.images.head'] == {'dtype': 'video', 'shape': (6, 8, 3),
                                                  'names': ['height', 'width', 'channels']}
    assert Dataset.saved == 1 and len(Dataset.frames) == 11  # Rows 0-10: the terminal row has no action.
    frame = Dataset.frames[3]
    assert frame['task'] == export.TASK
    assert np.allclose(frame['observation.state'], [3 + j / 100 for j in range(16)])
    assert np.allclose(frame['action'], [4 + j / 100 + .5 for j in range(16)])  # The next commanded position.
    assert np.allclose(frame['observation.effort'], [3 - j / 100 for j in range(16)])
    assert np.allclose(frame['observation.velocity'], [j / 10 for j in range(16)])
    assert np.allclose(frame['observation.head_camera_pose'], POSE)
    assert frame['observation.images.left_wrist'].shape == (6, 8, 3) and frame['observation.images.left_wrist'][0, 0, 0] == 3
    sidecar = json.loads((out / 'meta' / 'minerva_provenance.json').read_text(encoding='utf-8'))
    assert sidecar['fps'] == 15 and sidecar['state'] == 'qpos' and sidecar['action'] == 'actions'
    assert sidecar['format'] == 'minerva task capture 9 -> LeRobot v2.1'
    assert sidecar['extras'] == ['observation.effort', 'observation.velocity', 'observation.head_camera_pose']
    entry = sidecar['episodes'][0]
    assert entry['source'] == kept and entry['source_span'] == [0, 10] and entry['frames'] == 11
    assert entry['review']['note'] == 'good' and len(entry['sha256']) == 64
    with pytest.raises(export.ExportError, match='nothing to export'):
        export.write([], tmp_path / 'empty', 'x/y', 15, export.TASK, create=create)


def test_the_dry_run_lists_the_plan_without_lerobot(tmp_path):
    kept, _ = record(tmp_path, steady(30))
    Episodes(tmp_path).review(kept, 'keep')
    result = subprocess.run([sys.executable, '-m', 'robot_platform.data.lerobot_export', '--evidence', str(tmp_path),
                             '--dry-run'], capture_output=True, text=True, timeout=120, cwd=ROOT)
    assert result.returncode == 0, result.stderr
    assert f'export {kept}: rows 0-28 -> 29 frames at 15 fps' in result.stdout


def test_an_extra_is_exported_only_where_every_row_has_it_and_never_mixed(tmp_path):
    """A missing effort reading inside the span drops observation.effort from that episode; such an episode is not
    mixed with ones that have it. Nothing is filled in."""
    full, _ = record(tmp_path, steady(12))
    gap, _ = record(tmp_path, steady(12, start=200.), missing_torque=(4,))
    episodes = Episodes(tmp_path)
    episodes.review(full, 'keep')
    episodes.review(gap, 'keep')
    plans, refused = export.plan(tmp_path, 15)
    assert [p['exported_extras'] for p in plans] == [('observation.effort', 'observation.velocity',
                                                      'observation.head_camera_pose')]
    assert len(refused) == 1 and 'extra features' in refused[0][1]
    alone = export.plan_episode(refused[0][0], 15)
    assert alone['exported_extras'] == ('observation.velocity', 'observation.head_camera_pose')
