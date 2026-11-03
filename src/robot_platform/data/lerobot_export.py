"""Export Minerva's kept demonstrations (task capture format 8) to a LeRobot dataset for ACT and pi0/pi0.5 fine-tuning.

LeRobot dataset format v2.1, the version openpi pins (lerobot 0cf864870cf29f4738d3ade893e6fd13fbd7cdb5): run it where
that LeRobot is installed, beside the episodes copied off the robot (docs/development/DATASET_PIPELINE.md):

    python -m robot_platform.data.lerobot_export --evidence ~/minerva-episodes --out datasets/minerva-handoff \\
        --repo-id minerva/handoff [--fps 15] [--task TEXT] [--include-unreviewed] [--pre-takeover-s 1]
        [--min-stretch-s 0.5] [--dry-run]

Each episode, as recorded:
  - Only complete episodes reviewed Keep in Replay (the session's reviews.jsonl), unless --include-unreviewed.
  - A teleop episode's span runs from the first to the last row a human drove with a valid action (`authority`
    'human' and `action_valid`): the terminal row has no next command. A row inside the span that is not both refuses
    the episode.
  - A policy session's episode (policy corrections, helm.teleop: every row the policy's or a correction) keeps the
    stretches the policy drove and the ones the operator corrected (corrected_stretches): each a run of rows one of
    them drove while the arms followed, and each its own LeRobot episode, in recorded order. The holds and the
    alignments to the next driver between them are left out, so no exported episode jumps across them. A policy
    stretch the operator took over from loses its last --pre-takeover-s, where the policy went wrong.
  - Resampled onto a fixed grid of `fps` by the robot-clock timestamps (`timestamps/telem`): each grid time takes the
    latest recorded frame at or before it, so nothing comes from the future. A grid time more than 2.5 periods after
    its frame (two dropped frames in a row) is a gap in the recording and refuses the episode.
  - observation.state = qpos and action = actions (the next commanded position), both in receiver motor radians in
    the episode's own joint order; observation.images.<role> per camera, as stored (HxWx3 uint8).
Episodes with different joints, cameras or image sizes are never mixed in one dataset: the 2-DoF head adds two joints.
A sidecar meta/minerva_provenance.json records each exported episode's source, hash, review, span and who drove it.

The selection and resampling need only numpy and h5py; LeRobot is imported only to write.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

TASK = 'Pick up the cube with the left hand, hand it to the right hand, and put it in the box.'
FORMAT = 8
MAX_GAP_PERIODS = 2.5  # One dropped camera frame (a two-period gap) passes; two in a row refuse the episode.
CORRECTED = ('policy', 'correction')  # Who drives a policy session's frames (helm.teleop._capture_label).
PRE_TAKEOVER_S = 1.  # A policy stretch the operator took over from loses its last second: where it went wrong.
MIN_STRETCH_S = .5  # A kept stretch shorter than this (after that) is left out.


class ExportError(ValueError):
    pass


def _text(value):
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    return value.item() if hasattr(value, 'item') else value


def read_episode(path):
    """An episode's arrays and identity, images left in the file: {'joints', 'roles', 'image_shape', 'telem',
    'qpos', 'actions', 'action_valid', 'authority', 'lifecycle', 'complete', 'collected_at', 'language_instruction'}.
    `lifecycle`: a policy session's lifecycle state for each row, from its task state; None for any other episode."""
    import h5py
    import numpy as np
    with h5py.File(path, 'r') as f:
        attrs = f.attrs
        version = int(_text(attrs.get('format_version', 0)) or 0)
        if version != FORMAT:
            raise ExportError(f'{path.name}: format {version}, not the task capture format {FORMAT}')
        joints = [name for name in str(_text(attrs.get('minerva_joints', ''))).split(',') if name]
        roles = json.loads(_text(attrs.get('camera_roles', '[]')))
        shapes = {role: tuple(f[f'observations/images/{role}'].shape[1:]) for role in roles}
        if len(set(shapes.values())) != 1:
            raise ExportError(f'{path.name}: cameras of different sizes {shapes}')
        authority = [_text(value) for value in f['authority'][:]]
        lifecycle = None
        if authority and set(authority) <= set(CORRECTED):
            lifecycle = [json.loads(_text(value)).get('lifecycle') for value in f['task_state'][:]]
        return dict(joints=joints, roles=roles, image_shape=next(iter(shapes.values())),
                    telem=np.asarray(f['timestamps/telem'][:], dtype=np.float64),
                    qpos=np.asarray(f['observations/qpos'][:], dtype=np.float32),
                    actions=np.asarray(f['actions'][:], dtype=np.float32),
                    action_valid=np.asarray(f['action_valid'][:], dtype=bool), authority=authority,
                    lifecycle=lifecycle, complete=bool(_text(attrs.get('complete', False))),
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


def corrected_stretches(telem, action_valid, authority, lifecycle, *, pre_takeover_s=PRE_TAKEOVER_S,
                        min_stretch_s=MIN_STRETCH_S):
    """A policy session's kept stretches, in recorded order: [{'first', 'last', 'authority', 'trimmed_s'}].
    - A stretch is a run of rows one driver drove (the policy, or the operator's correction) while the arms followed:
      the row and the next one, whose command is the row's action, both in Teleop, and a valid action. The holds, and
      the alignments to the next driver after them, are left out.
    - The operator took over from a policy stretch when a correction comes after it, before the policy drives again:
      its last pre_takeover_s is where the policy went wrong, and is left out (trimmed_s says how much).
    - A stretch shorter than min_stretch_s after that is left out."""
    n = len(authority)

    def drives(i, who):
        return (i + 1 < n and bool(action_valid[i]) and authority[i] == who and who in CORRECTED
                and lifecycle[i] == 'TELEOP' and lifecycle[i + 1] == 'TELEOP')
    runs, i = [], 0
    while i < n:
        who = authority[i]
        if not drives(i, who):
            i += 1
            continue
        last = i
        while drives(last + 1, who):
            last += 1
        runs.append((i, last, who))
        i = last + 1
    stretches = []
    for index, (first, last, who) in enumerate(runs):
        end, trimmed = last, 0.
        policy_again = next((run[0] for run in runs[index + 1:] if run[2] == 'policy'), n)
        if who == 'policy' and 'correction' in authority[last + 1:policy_again]:
            cut = telem[last] - pre_takeover_s
            while end >= first and telem[end] > cut:
                end -= 1
            trimmed = float(telem[last] - telem[max(end, first)])
        if end < first or telem[end] - telem[first] < min_stretch_s:
            continue
        stretches.append(dict(first=first, last=end, authority=who, trimmed_s=round(trimmed, 3)))
    return stretches


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


def features(joints, roles, image_shape):
    """The LeRobot features of one dataset: state and action in the episodes' joint order, a video per camera."""
    vector = {'dtype': 'float32', 'shape': (len(joints),), 'names': list(joints)}
    result = {'observation.state': dict(vector), 'action': dict(vector)}
    for role in roles:
        result[f'observation.images.{role}'] = {'dtype': 'video', 'shape': tuple(image_shape),
                                                'names': ['height', 'width', 'channels']}
    return result


def plan_episode(path, fps, *, pre_takeover_s=PRE_TAKEOVER_S, min_stretch_s=MIN_STRETCH_S):
    """What one episode exports as: its identity and its stretches, each a span and its grid rows (a teleop
    episode's one human span, a policy session's corrected_stretches), or an ExportError."""
    episode = read_episode(path)
    if not episode['complete']:
        raise ExportError(f'{path.name}: the recording did not finish')
    if episode['lifecycle'] is not None:
        if None in episode['lifecycle']:
            raise ExportError(f'{path.name}: a policy episode whose rows do not all say their lifecycle state')
        stretches = corrected_stretches(episode['telem'], episode['action_valid'], episode['authority'],
                                        episode['lifecycle'], pre_takeover_s=pre_takeover_s,
                                        min_stretch_s=min_stretch_s)
        if not stretches:
            raise ExportError(f'{path.name}: no stretch the policy or a correction drove for {min_stretch_s:g} s')
        for stretch in stretches:
            stretch['rows'] = resample(episode['telem'], stretch['first'], stretch['last'], fps)
        return dict(episode, path=path, stretches=stretches)
    first, last = usable_span(episode['action_valid'], episode['authority'])
    rows = resample(episode['telem'], first, last, fps)
    return dict(episode, path=path, first=first, last=last, rows=rows,
                stretches=[dict(first=first, last=last, authority='human', trimmed_s=0., rows=rows)])


def frames(plan, task, rows=None):
    """LeRobot frames for one planned episode (or the grid `rows` of one of its stretches), images read one source
    row at a time."""
    import h5py
    with h5py.File(plan['path'], 'r') as f:
        images = {role: f[f'observations/images/{role}'] for role in plan['roles']}
        cached = (None, None)
        for row in (plan['rows'] if rows is None else rows):
            row = int(row)
            if cached[0] != row:  # A repeated row (a grid finer than the recording) reads its images once.
                cached = (row, {role: images[role][row] for role in plan['roles']})
            frame = {'observation.state': plan['qpos'][row], 'action': plan['actions'][row], 'task': task}
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


def plan(evidence, fps, *, include_unreviewed=False, pre_takeover_s=PRE_TAKEOVER_S, min_stretch_s=MIN_STRETCH_S):
    """(plans, refused): each selected episode planned, or refused with its reason. Mixed rosters are refused."""
    plans, refused = [], []
    for path, review in select(evidence, include_unreviewed=include_unreviewed):
        try:
            planned = plan_episode(path, fps, pre_takeover_s=pre_takeover_s, min_stretch_s=min_stretch_s)
        except ExportError as exc:
            refused.append((path, str(exc)))
            continue
        planned['review'] = review
        if plans and (planned['joints'], planned['roles'], planned['image_shape']) != (
                plans[0]['joints'], plans[0]['roles'], plans[0]['image_shape']):
            refused.append((path, 'different joints, cameras or image size from the first episode: '
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
    """Write the planned episodes as one LeRobot dataset under `out`, with the provenance sidecar: a LeRobot episode
    for each stretch (a teleop episode has one; a policy session's has one per stretch the policy or a correction
    drove), its provenance saying which recorded rows it came from and who drove them."""
    if not plans:
        raise ExportError('nothing to export: no episode is kept for training')
    if create is None:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset  # openpi's pinned revision: format v2.1
        create = LeRobotDataset.create
    out = Path(out)
    first = plans[0]
    dataset = create(repo_id=repo_id, fps=fps, root=out, robot_type='minerva',
                     features=features(first['joints'], first['roles'], first['image_shape']),
                     use_videos=True, image_writer_threads=4)
    provenance = []
    for planned in plans:
        digest = _sha256(planned['path'])
        for stretch in planned['stretches']:
            rows = stretch['rows']
            for frame in frames(planned, task, rows):
                dataset.add_frame(frame)
            dataset.save_episode()
            provenance.append(dict(
                episode_index=len(provenance), source=f"{planned['path'].parent.parent.name}/{planned['path'].stem}",
                sha256=digest, collected_at=planned['collected_at'], review=planned['review'],
                source_span=[int(stretch['first']), int(stretch['last'])],
                source_rows=int(stretch['last'] - stretch['first'] + 1), frames=len(rows),
                repeated_rows=int(len(rows) - len(set(int(r) for r in rows))), authority=stretch['authority'],
                trimmed_s=stretch['trimmed_s']))
    meta = out / 'meta'
    meta.mkdir(parents=True, exist_ok=True)
    (meta / 'minerva_provenance.json').write_text(json.dumps(dict(
        fps=fps, task=task, format='minerva task capture 8 -> LeRobot v2.1', state='qpos', action='actions',
        joints=first['joints'], cameras=first['roles'], episodes=provenance), indent=1) + '\n', encoding='utf-8')
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
    parser.add_argument('--pre-takeover-s', type=float, default=PRE_TAKEOVER_S,
                        help='seconds left out before each take over from the policy (policy sessions)')
    parser.add_argument('--min-stretch-s', type=float, default=MIN_STRETCH_S,
                        help='the shortest policy or correction stretch kept (policy sessions)')
    parser.add_argument('--dry-run', action='store_true', help='show what would be exported; needs no LeRobot')
    args = parser.parse_args(argv)
    if not 1 <= args.fps <= 60:
        parser.error('--fps is 1 to 60')
    if not (0 <= args.pre_takeover_s <= 10 and 0 <= args.min_stretch_s <= 60):
        parser.error('--pre-takeover-s is 0 to 10 s and --min-stretch-s 0 to 60 s')
    plans, refused = plan(args.evidence, args.fps, include_unreviewed=args.include_unreviewed,
                          pre_takeover_s=args.pre_takeover_s, min_stretch_s=args.min_stretch_s)
    for planned in plans:
        for stretch in planned['stretches']:
            rows = stretch['rows']
            who = '' if stretch['authority'] == 'human' else f" {stretch['authority']}"
            trimmed = f", {stretch['trimmed_s']:g} s before the take over left out" if stretch['trimmed_s'] else ''
            print(f"export {planned['path'].parent.parent.name}/{planned['path'].stem}:{who} rows {stretch['first']}-"
                  f"{stretch['last']} -> {len(rows)} frames at {args.fps} fps "
                  f"({len(rows) / args.fps:.1f} s, {len(rows) - len(set(rows.tolist()))} repeated{trimmed})")
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
    print(f"wrote {sum(len(planned['stretches']) for planned in plans)} episodes from {len(plans)} recordings "
          f'to {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
