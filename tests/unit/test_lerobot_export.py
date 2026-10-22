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
ROLES = ('left_wrist', 'right_wrist', 'head')
SESSION = 'live-20261022T120000Z'


def record(root, times, *, authority=None, finish=True, session=SESSION):
    """An episode whose row i has qpos[i] = i + j/100 and images at level i (mod 250) on every camera."""
    writer = TaskEpisodeWriter(root / session / 'episodes', NAMES, {'data_source': 'real', 'camera_roles': list(ROLES)},
                               hz=15., max_pending=256)
    for i, t in enumerate(times):
        writer.append(q=[i + j / 100 for j in range(14)], qcmd=[i + j / 100 + .5 for j in range(14)], source_time=t,
                      scene_time=None, images={role: np.full((6, 8, 3), i % 250, dtype=np.uint8) for role in ROLES},
                      camera_times={role: t - .02 for role in ROLES}, task_state={'row': i},
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
    with h5py.File(other_path, 'r+') as f:  # The same episode as recorded with the 2-DoF head's two joints.
        f.attrs['minerva_joints'] = ','.join(NAMES + ('head_pan', 'head_tilt'))
    plans, refused = export.plan(tmp_path, 15)
    assert len(plans) == 1 and plans[0]['first'] == 2 and plans[0]['last'] == 18  # Policy rows out; terminal row out.
    assert plans[0]['rows'].tolist() == list(range(2, 19))
    assert len(refused) == 1 and 'different joints' in refused[0][1]


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
    assert feature['observation.state'] == {'dtype': 'float32', 'shape': (14,), 'names': list(NAMES)}
    assert feature['action']['names'] == list(NAMES)
    assert feature['observation.images.head'] == {'dtype': 'video', 'shape': (6, 8, 3),
                                                  'names': ['height', 'width', 'channels']}
    assert Dataset.saved == 1 and len(Dataset.frames) == 11  # Rows 0-10: the terminal row has no action.
    frame = Dataset.frames[3]
    assert frame['task'] == export.TASK
    assert np.allclose(frame['observation.state'], [3 + j / 100 for j in range(14)])
    assert np.allclose(frame['action'], [4 + j / 100 + .5 for j in range(14)])  # The next commanded position.
    assert frame['observation.images.left_wrist'].shape == (6, 8, 3) and frame['observation.images.left_wrist'][0, 0, 0] == 3
    sidecar = json.loads((out / 'meta' / 'minerva_provenance.json').read_text(encoding='utf-8'))
    assert sidecar['fps'] == 15 and sidecar['state'] == 'qpos' and sidecar['action'] == 'actions'
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
