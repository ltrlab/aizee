"""Deploy a robot release: upload, verify on the robot, switch atomically, prune.

Layout on the robot (user space only; no sudo, services or compilers):
    ~/minerva/releases/<id>/   immutable, verified against its own manifest
    ~/minerva/current          symlink to the active release, switched atomically
    ~/minerva/deploy.log       one line per deployment
Only the newest `keep` releases (always including the current and previous
ones) are kept, so repeated deploys do not grow the disk.
"""
import json
import re
import shlex

from .ssh import run

BASE = '/home/ltr/minerva'
RELEASE_ID = re.compile(r'^\d{8}T\d{6}Z-[0-9a-f]{7}(-dirty[0-9a-f]{6})?$')

VERIFY = '''
import json, sys
sys.path.insert(0, sys.argv[1] + "/src")
from robot_platform.ops.release import verify_tree
problems = verify_tree(sys.argv[1])
print(json.dumps(problems))
sys.exit(1 if problems else 0)
'''


def remote_state(client, base=BASE):
    """{'current': id or None, 'releases': [{id, content_sha256}]} on the robot."""
    script = f'''
import json, os, pathlib
base = pathlib.Path({base!r})
current = os.path.realpath(base / "current") if (base / "current").is_symlink() else None
releases = []
for path in sorted((base / "releases").glob("*")):
    try:
        manifest = json.loads((path / "release.json").read_text())
    except (OSError, ValueError):
        continue
    releases.append({{"id": path.name, "content_sha256": manifest.get("content_sha256"),
                      "commit": manifest.get("source", {{}}).get("commit")}})
print(json.dumps({{"current": os.path.basename(current) if current else None, "releases": releases}}))
'''
    status, out, err = run(client, 'python3 -', stdin=script)
    if status != 0:
        raise RuntimeError(f'Could not read the robot releases: {err.strip()}')
    return json.loads(out)


def stale_releases(names, current, previous, keep):
    """Releases to prune: all but the newest `keep`, never the current or previous one."""
    ordered = sorted(name for name in set(names) | {current} if RELEASE_ID.match(name))
    protect = set(ordered[-keep:]) | {current, previous}
    return [name for name in ordered if name not in protect]


def smoke(client, root):
    """The robot-side smoke test of a release (robot_platform.ops.smoke), as a dict."""
    q = shlex.quote
    status, out, err = run(client, f'cd {q(root)} && PYTHONPATH=src python3 -m robot_platform.ops.smoke {q(root)}',
                           timeout=180)
    try:
        return json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {'ok': False, 'error': err.strip()[-500:] or f'smoke test exited with {status}'}


def deploy(client, manifest, archive, *, base=BASE, keep=3, log=print):
    """Install `archive` (built by release.build) and make it current; returns the release id."""
    release_id = manifest['id']
    if not RELEASE_ID.match(release_id) or keep < 2:
        raise ValueError('Invalid release id or keep count')
    q = shlex.quote
    releases, incoming = f'{base}/releases', f'{base}/incoming'
    state = remote_state(client, base) if run(client, f'test -d {q(releases)}')[0] == 0 else {'releases': []}
    same = [r['id'] for r in state['releases'] if r['content_sha256'] == manifest['content_sha256']]
    if same:
        release_id = same[-1]
        log(f'Release content already on the robot as {release_id}; switching to it.')
    else:
        status, _out, err = run(client, f'mkdir -p {q(releases)} {q(incoming)}')
        if status:
            raise RuntimeError(f'Could not prepare {base}: {err.strip()}')
        upload = f'{incoming}/{release_id}.tar.gz'
        with client.open_sftp() as sftp:
            sftp.put(str(archive), upload)
        partial = f'{releases}/{release_id}.partial'
        status, out, err = run(client, f'rm -rf {q(partial)} && mkdir {q(partial)} && '
                                       f'tar -xzf {q(upload)} -C {q(partial)} && rm -f {q(upload)}')
        if status:
            raise RuntimeError(f'Unpacking failed: {err.strip()}')
        status, out, err = run(client, f'python3 - {q(partial)}', stdin=VERIFY)
        if status:
            run(client, f'rm -rf {q(partial)}')
            raise RuntimeError(f'The release failed verification on the robot: {out.strip() or err.strip()}')
        status, _out, err = run(client, f'mv {q(partial)} {q(releases + "/" + release_id)}')
        if status:
            raise RuntimeError(f'Could not install the release: {err.strip()}')
        log(f'Uploaded and verified {release_id} ({len(manifest["files"])} files).')
    previous = state.get('current')
    status, _out, err = run(client, f'cd {q(base)} && ln -sfn releases/{q(release_id)} current.new && '
                                    f'mv -Tf current.new current')
    if status:
        raise RuntimeError(f'Could not switch the current release: {err.strip()}')
    report = smoke(client, f'{base}/current')
    if not report.get('ok'):
        if previous:
            run(client, f'cd {q(base)} && ln -sfn releases/{q(previous)} current.new && mv -Tf current.new current')
        raise RuntimeError(f"The release failed its smoke test on the robot"
                           f"{f'; rolled back to {previous}' if previous else ''}: {report.get('error')}")
    log(f"Smoke test passed: collision model {report['load_ms']} ms to load, keep-out query p50 "
        f"{report['keepout_query_ms']['p50']} ms, receiver harness {report['receiver_harness']}.")
    stale = stale_releases({r['id'] for r in remote_state(client, base)['releases']}, release_id, previous, keep)
    if stale:
        run(client, 'rm -rf ' + ' '.join(q(f'{releases}/{name}') for name in stale))
        log(f'Pruned {len(stale)} old release(s): {", ".join(stale)}')
    entry = json.dumps({'release': release_id, 'commit': manifest['source']['commit'],
                        'dirty': manifest['source']['dirty'], 'previous': previous})
    run(client, f'printf "%s %s\\n" "$(date -u +%FT%TZ)" {q(entry)} >> {q(base + "/deploy.log")}')
    return release_id
