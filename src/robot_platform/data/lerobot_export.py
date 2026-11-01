"""Export Minerva's kept demonstrations (task capture formats 8 and 9) to a LeRobot dataset for ACT and pi0/pi0.5 fine-tuning.

LeRobot dataset format v2.1, the version openpi pins (lerobot 0cf864870cf29f4738d3ade893e6fd13fbd7cdb5): run it where
that LeRobot is installed, beside the episodes copied off the robot (docs/development/DATASET_PIPELINE.md):

    python -m robot_platform.data.lerobot_export --evidence ~/minerva-episodes --out datasets/minerva-handoff \\
        --repo-id minerva/handoff [--fps 15] [--task TEXT] [--include-unreviewed] [--dry-run]

Each episode, as recorded:
  - Only complete episodes reviewed Keep in Replay (the session's reviews.jsonl), unless --include-unreviewed.
  - Its span runs from the first to the last row a human drove with a valid action (`authority` 'human' and
    `action_valid`): the terminal row has no next command. A row inside the span that is not both refuses the episode.
  - Resampled onto a fixed grid of `fps` by the robot-clock timestamps (`timestamps/telem`): each grid time takes the
    latest recorded frame at or before it, so nothing comes from the future. A grid time more than 2.5 periods after
    its frame (two dropped frames in a row) is a gap in the recording and refuses the episode.
  - observation.state = qpos and action = actions (the next commanded position), both in receiver motor radians in
    the episode's own joint order; observation.images.<role> per camera, as stored (HxWx3 uint8).
  - Format 9 (16 axes: both arms, both grippers and the neck) adds, when every exported row has it:
    observation.effort (the motors' measured effort, N m), observation.velocity (rad/s) and
    observation.head_camera_pose (the head camera's [x, y, z, qx, qy, qz, qw] in the robot frame). docs/EPISODE_FORMAT.md
Episodes with different joints, cameras, image sizes or extra features are never mixed in one dataset: format 8
recorded the arms alone (14 axes), format 9 the whole robot (16).
A sidecar meta/minerva_provenance.json records each exported episode's source, hash, review and span.

The selection and resampling need only numpy and h5py; LeRobot is imported only to write.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

TASK = 'Pick up the cube with the left hand, hand it to the right hand, and put it in the box.'
FORMATS = (8, 9)
# Format 9's extra observations: (LeRobot feature, episode dataset, validity mask, width, names or None for the joints).
EXTRAS = (('observation.effort', 'observations/torques', 'observations/torque_valid', None, None),
          ('observation.velocity', 'observations/qvel', 'observations/qvel_valid', None, None),
          ('observation.head_camera_pose', 'observations/head_camera_pose', 'timestamps/head_valid', 7,
           ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']))
MAX_GAP_PERIODS = 2.5  # One dropped camera frame (a two-period gap) passes; two in a row refuse the episode.


class ExportError(ValueError):
    pass


def _text(value):
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    return value.item() if hasattr(value, 'item') else value


def read_episode(path):
    """An episode's arrays and identity, images left in the file: {'joints', 'roles', 'image_shape', 'telem',
    'qpos', 'actions', 'action_valid', 'authority', 'complete', 'collected_at', 'language_instruction'}."""
    import h5py
    import numpy as np
    with h5py.File(path, 'r') as f:
        attrs = f.attrs
        version = int(_text(attrs.get('format_version', 0)) or 0)
        if version not in FORMATS:
            raise ExportError(f'{path.name}: format {version}, not a task capture format {FORMATS}')
        joints = [name for name in str(_text(attrs.get('minerva_joints', ''))).split(',') if name]
        roles = json.loads(_text(attrs.get('camera_roles', '[]')))
        shapes = {role: tuple(f[f'observations/images/{role}'].shape[1:]) for role in roles}
        if len(set(shapes.values())) != 1:
            raise ExportError(f'{path.name}: cameras of different sizes {shapes}')
        authority = [_text(value) for value in f['authority'][:]]
        extras = {}
        for name, dataset, mask, _width, _names in EXTRAS:
            if dataset in f and mask in f:
                values, valid = np.asarray(f[dataset][:], dtype=np.float32), np.asarray(f[mask][:], dtype=bool)
                extras[name] = (values, valid.all(axis=-1) if valid.ndim > 1 else valid)
        return dict(joints=joints, roles=roles, image_shape=next(iter(shapes.values())), version=version, extras=extras,
                    telem=np.asarray(f['timestamps/telem'][:], dtype=np.float64),
                    qpos=np.asarray(f['observations/qpos'][:], dtype=np.float32),
                    actions=np.asarray(f['actions'][:], dtype=np.float32),
                    action_valid=np.asarray(f['action_valid'][:], dtype=bool), authority=authority,
                    complete=bool(_text(attrs.get('complete', False))),
                    collected_at=_text(attrs.get('collected_at')),
                    language_instruction=_text(attrs.get('language_instruction', '')) or '')


def usable_span(action_valid, authority):
    """(first, last) row a human drove with a valid action; every row between must be both."""
    good = [bool(valid) and who == 'human' for valid, who in zip(action_valid, authority)]
    if not any(good):
        raise ExportError('no row with a human-driven valid action')
    first, last = good.index(True), len(good) - 1 - good[::-1].index(True)
    bad = [i for i in range(first, last + 1) if not good[i]]
    if bad:
        raise ExportError(f'rows {bad[:5]} inside the span are not human-driven with a valid action')
    return first, last


def resample(telem, first, last, fps):
    """The source row for each time on a fixed grid from telem[first] to telem[last]: the latest at or before it."""
    import numpy as np
    span = float(telem[last] - telem[first])
    grid = telem[first] + np.arange(int(math.floor(span * fps + 1e-9)) + 1) / fps
    rows = np.searchsorted(telem[first:last + 1], grid, side='right') - 1 + first
    lag = grid - telem[rows]
    if lag.max() > MAX_GAP_PERIODS / fps:
        at = int(lag.argmax())
        raise ExportError(f'a gap of {lag.max() * 1000:.0f} ms at {grid[at] - telem[first]:.2f} s: '
                          f'over {MAX_GAP_PERIODS} periods of {fps} fps')
    return rows


def features(joints, roles, image_shape, extras=()):
    """The LeRobot features of one dataset: state and action in the episodes' joint order, a video per camera, and
    the format 9 extras every episode has (EXTRAS)."""
    vector = {'dtype': 'float32', 'shape': (len(joints),), 'names': list(joints)}
    result = {'observation.state': dict(vector), 'action': dict(vector)}
    for name, _dataset, _mask, width, names in EXTRAS:
        if name in extras:
            result[name] = dict(vector) if width is None else {'dtype': 'float32', 'shape': (width,), 'names': names}
    for role in roles:
        result[f'observation.images.{role}'] = {'dtype': 'video', 'shape': tuple(image_shape),
                                                'names': ['height', 'width', 'channels']}
    return result


def plan_episode(path, fps):
    """What one episode exports as: its identity, span and grid rows, or an ExportError."""
    episode = read_episode(path)
    if not episode['complete']:
        raise ExportError(f'{path.name}: the recording did not finish')
    first, last = usable_span(episode['action_valid'], episode['authority'])
    rows = resample(episode['telem'], first, last, fps)
    # An extra is exported only when every row the grid takes has it: no NaN, nothing filled in.
    extras = tuple(name for name, _dataset, _mask, _width, _names in EXTRAS
                   if name in episode['extras'] and bool(episode['extras'][name][1][rows].all()))
    return dict(episode, path=path, first=first, last=last, rows=rows, exported_extras=extras)


def frames(plan, task):
    """LeRobot frames for one planned episode, images read one source row at a time."""
    import h5py
    with h5py.File(plan['path'], 'r') as f:
        images = {role: f[f'observations/images/{role}'] for role in plan['roles']}
        cached = (None, None)
        for row in plan['rows']:
            row = int(row)
            if cached[0] != row:  # A repeated row (a grid finer than the recording) reads its images once.
                cached = (row, {role: images[role][row] for role in plan['roles']})
            frame = {'observation.state': plan['qpos'][row], 'action': plan['actions'][row], 'task': task}
            frame.update({name: plan['extras'][name][0][row] for name in plan['exported_extras']})
            frame.update({f'observation.images.{role}': image for role, image in cached[1].items()})
            yield frame


def select(evidence, *, include_unreviewed=False):
    """The episodes to export, oldest first: (path, review) for each complete one reviewed Keep (or not reviewed at
    all, with include_unreviewed), never one reviewed Discard. Reviews come from Replay's reviews.jsonl."""
    from robot_platform.helm.episode_worker import Episodes
    root = Path(evidence).expanduser().resolve()
    chosen = []
    for entry in Episodes(root).list():
        verdict = (entry.get('review') or {}).get('verdict')
        if entry['partial'] or not entry['complete'] or entry.get('error'):
            continue
        if verdict == 'keep' or (include_unreviewed and verdict is None):
            chosen.append((root / entry['session'] / 'episodes' / (entry['name'] + '.hdf5'), entry.get('review')))
    return sorted(chosen, key=lambda item: item[0].stat().st_mtime)


def plan(evidence, fps, *, include_unreviewed=False):
    """(plans, refused): each selected episode planned, or refused with its reason. Mixed rosters are refused."""
    plans, refused = [], []
    for path, review in select(evidence, include_unreviewed=include_unreviewed):
        try:
            planned = plan_episode(path, fps)
        except ExportError as exc:
            refused.append((path, str(exc)))
            continue
        planned['review'] = review
        key = ('joints', 'roles', 'image_shape', 'exported_extras')
        if plans and tuple(planned[k] for k in key) != tuple(plans[0][k] for k in key):
            refused.append((path, 'different joints, cameras, image size or extra features from the first episode: '
                                  'export them as their own dataset'))
            continue
        plans.append(planned)
    return plans, refused


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def write(plans, out, repo_id, fps, task, *, create=None):
    """Write the planned episodes as one LeRobot dataset under `out`, with the provenance sidecar."""
    if not plans:
        raise ExportError('nothing to export: no episode is kept for training')
    if create is None:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset  # openpi's pinned revision: format v2.1
        create = LeRobotDataset.create
    out = Path(out)
    first = plans[0]
    dataset = create(repo_id=repo_id, fps=fps, root=out, robot_type='minerva',
                     features=features(first['joints'], first['roles'], first['image_shape'], first['exported_extras']),
                     use_videos=True, image_writer_threads=4)
    provenance = []
    for index, planned in enumerate(plans):
        for frame in frames(planned, task):
            dataset.add_frame(frame)
        dataset.save_episode()
        rows = planned['rows']
        provenance.append(dict(
            episode_index=index, source=f"{planned['path'].parent.parent.name}/{planned['path'].stem}",
            sha256=_sha256(planned['path']), collected_at=planned['collected_at'], review=planned['review'],
            source_span=[int(planned['first']), int(planned['last'])], source_rows=int(planned['last'] - planned['first'] + 1),
            frames=len(rows), repeated_rows=int(len(rows) - len(set(int(r) for r in rows)))))
    meta = out / 'meta'
    meta.mkdir(parents=True, exist_ok=True)
    (meta / 'minerva_provenance.json').write_text(json.dumps(dict(
        fps=fps, task=task, format=f"minerva task capture {first['version']} -> LeRobot v2.1", state='qpos',
        action='actions', extras=list(first['exported_extras']), joints=first['joints'], cameras=first['roles'],
        episodes=provenance), indent=1) + '\n', encoding='utf-8')
    return dataset


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m robot_platform.data.lerobot_export', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--evidence', required=True, help='a copy of ~/minerva/evidence (live-*/episodes, reviews)')
    parser.add_argument('--out', help='the dataset directory to create (must not exist)')
    parser.add_argument('--repo-id', default='minerva/handoff')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--task', default=TASK)
    parser.add_argument('--include-unreviewed', action='store_true', help='also export episodes not yet reviewed')
    parser.add_argument('--dry-run', action='store_true', help='show what would be exported; needs no LeRobot')
    args = parser.parse_args(argv)
    if not 1 <= args.fps <= 60:
        parser.error('--fps is 1 to 60')
    plans, refused = plan(args.evidence, args.fps, include_unreviewed=args.include_unreviewed)
    for planned in plans:
        rows = planned['rows']
        print(f"export {planned['path'].parent.parent.name}/{planned['path'].stem}: rows {planned['first']}-"
              f"{planned['last']} -> {len(rows)} frames at {args.fps} fps "
              f"({len(rows) / args.fps:.1f} s, {len(rows) - len(set(rows.tolist()))} repeated)")
    for path, reason in refused:
        print(f'refused {path.parent.parent.name}/{path.stem}: {reason}', file=sys.stderr)
    if args.dry_run:
        return 0
    if not args.out:
        parser.error('--out is required unless --dry-run')
    if Path(args.out).exists():
        parser.error(f'{args.out} exists: LeRobot creates the dataset directory itself')
    try:
        write(plans, args.out, args.repo_id, args.fps, args.task)
    except ExportError as exc:
        print(exc, file=sys.stderr)
        return 1
    print(f'wrote {len(plans)} episodes to {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
