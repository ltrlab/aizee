"""Helm on Minerva: the operator's saved files kept beside the release, never in it.

Helm saves three files that sit beside the profiles in this repository: the joint limits (Limits tab), the servo
calibration (Tune tab) and the leader ranges (Leaders tab). On Minerva, Helm runs from a verified release, which must
not change (ops.release.verify_tree: a changed file stops every session from starting), so `helm --operator-files DIR`
reads and saves them in DIR instead. The arms' mechanical reference is kept there too, as
`<asset>-arms-reference.json` (control.joint_calibration.operator_reference): a copy of the profile's, until Joint zero
records a new revision (helm.joint_zero).

Each start reconciles DIR with the release it runs (prepare):
- a file DIR lacks is copied from the release;
- a release whose copy differs from the one DIR's file started from replaces it, as a deploy did on the operator PC:
  a committed change takes effect. If the operator had saved over the old copy, that save is moved to DIR/replaced/
  first, never lost, and a notice says where;
- otherwise DIR's copy, the operator's last save, stands.
DIR/operator-files.json records which release copy each file started from.
"""

import hashlib
import json
import os
from pathlib import Path
import time

REGISTRY = 'operator-files.json'


def operator_files(profile_path, asset_id, teleop_path=None):
    """{file name: the copy beside the profiles} of the joint limits, servo calibration, arms' reference and (with a
    teleop profile) leader ranges."""
    from robot_platform.control.joint_calibration import arms_reference_name

    profile_path = Path(profile_path).resolve()
    files = {name: profile_path.parent / name for name in (f'{asset_id}-joint-limits.json', f'{asset_id}-servo.json')}
    reference = json.loads(profile_path.read_text(encoding='utf-8'))['reference']
    files[arms_reference_name(asset_id)] = (profile_path.parent / reference).resolve()
    if teleop_path is not None:
        teleop_path = Path(teleop_path).resolve()
        ranges = json.loads(teleop_path.read_text(encoding='utf-8')).get('leader_ranges')
        if ranges:
            files[Path(ranges).name] = (teleop_path.parent / ranges).resolve()
    return files


def _sha(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except FileNotFoundError:
        return None


def _write(path, data):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_bytes(data)
    os.replace(temporary, path)


def prepare(files, directory, *, clock=time.time):
    """Reconcile `directory` with the release's copies (`files`, as operator_files gives them); returns a notice
    ({'file', 'kind': 'updated' | 'replaced', 'message', 'kept'}) for each file it changed."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    try:
        registry = json.loads((directory / REGISTRY).read_text(encoding='utf-8'))
    except FileNotFoundError:
        registry = {}
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(clock()))
    notices = []
    for name, source in files.items():
        state = directory / name
        release, mine = _sha(source), _sha(state)
        entry = registry.get(name)
        if mine is None:
            if release is not None:
                _write(state, Path(source).read_bytes())
        elif entry is None or release is None or release == entry.get('release_sha256'):
            # Placed here by hand (a migration), dropped from the release, or the release did not change it: the
            # operator's copy stands.
            if entry is not None:
                continue
        elif mine == entry.get('release_sha256'):
            _write(state, Path(source).read_bytes())  # Never saved over: the release's new copy takes its place.
            notices.append(
                {'file': name, 'kind': 'updated', 'kept': None, 'message': f'{name}: updated from the release'}
            )
        elif mine != release:
            kept = directory / 'replaced' / f'{stamp}-{name}'
            kept.parent.mkdir(exist_ok=True)
            os.replace(state, kept)
            _write(state, Path(source).read_bytes())
            notices.append(
                {
                    'file': name,
                    'kind': 'replaced',
                    'kept': str(kept),
                    'message': f'{name}: the release brought a new copy; the one saved on Minerva before is '
                    f'kept at {kept}',
                }
            )
        registry[name] = {'release_sha256': release}
    _write(directory / REGISTRY, (json.dumps(registry, indent=1, sort_keys=True) + '\n').encode())
    return notices
