"""Helm on Minerva, from the build PC: what the Jetson cannot make itself (ops.helm_service).

    python -m robot_platform.ops.helm_robot [--address 192.168.8.180] ACTION

    meshes   convert this checkout's model meshes for the 3D view and upload the ones Minerva lacks (~/minerva/meshes).
             The CAD stays here, as for every release; the robot serves the converted meshes by their digests.
    deps     install requirements/helm-robot.lock on Minerva (~/minerva/helm/deps/<lock>): the wheels are downloaded
             here with their hashes checked, then installed there from those files only
    update   deps and meshes for the release Minerva runs, then restart Helm there when no session runs (after a deploy)
    bridge   package the GELLO bridge for the operator PC (build/gello-bridge/gello-bridge-<commit>.zip)
    remote   run ops.helm_service on Minerva: remote prepare | status | install | restart | authorize-local | ...

Over SSH with the deploy key (AIZEE_SSH_KEY_FILE), pinned to the robot's host key and machine id (ops.ssh.connect).
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import zipfile

from .release import HELM_LOCK, MODEL, ROOT
from .ssh import TETHER, run

SERVICE = 'cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service'
# The Jetson: CPython 3.10 on aarch64 with glibc 2.35, so any of these manylinux wheels.
PLATFORMS = ('manylinux_2_17_aarch64', 'manylinux2014_aarch64', 'manylinux_2_28_aarch64', 'manylinux_2_34_aarch64')
BRIDGE_FILES = ('src/robot_platform', 'config/devices', 'requirements/gello-bridge.lock',
                'scripts/install_gello_bridge.ps1', 'pyproject.toml')

VERIFY_MESHES = '''
import hashlib, json, os, pathlib, sys
store = pathlib.Path.home() / "minerva" / "meshes"
expected, bad = json.loads(sys.stdin.read()), []
for name, sha in expected.items():
    incoming = store / ".incoming" / name
    if hashlib.sha256(incoming.read_bytes()).hexdigest() != sha:
        bad.append(name)
        incoming.unlink()
    else:
        os.replace(incoming, store / name)
print(json.dumps({"stored": len(expected) - len(bad), "bad": bad}))
'''


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    partial.write_bytes(data)
    os.replace(partial, path)


def wanted_meshes(root=ROOT):
    """{stored mesh name: its CAD file} for the model this checkout builds releases of."""
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.models.urdf import asset_digest
    from robot_platform.viewer.server import stored_mesh_name
    model = UrdfModel(ModelProfile.load(Path(root) / MODEL))
    wanted = {}
    for path in model.assets:
        wanted.setdefault(stored_mesh_name(asset_digest(path)), Path(path))
    return wanted


def converted(name, source, cache):
    """A mesh's stored form, kept in `cache` so a second upload converts nothing."""
    from robot_platform.viewer.server import mesh_payload
    path = Path(cache) / name
    if not path.is_file():
        _atomic(path, gzip.compress(mesh_payload(source), compresslevel=6, mtime=0))
    return path.read_bytes()


def meshes(client, *, root=ROOT, cache=None, log=print):
    """Upload the converted meshes Minerva lacks; returns {'wanted', 'uploaded', 'bytes'}."""
    cache = Path(cache or Path(root) / 'build' / 'helm-robot-meshes')
    wanted = wanted_meshes(root)
    status, out, err = run(client, 'mkdir -p ~/minerva/meshes/.incoming && ls ~/minerva/meshes')
    if status:
        raise RuntimeError(f'Could not read the mesh store: {err.strip()}')
    missing = sorted(set(wanted) - set(out.split()))
    log(f'{len(wanted)} meshes; {len(missing)} to upload.')
    sent, expected = 0, {}
    with client.open_sftp() as sftp:
        for count, name in enumerate(missing, 1):
            data = converted(name, wanted[name], cache)
            with sftp.open(f'minerva/meshes/.incoming/{name}', 'wb') as remote:
                remote.set_pipelined(True)
                remote.write(data)
            expected[name], sent = _sha(data), sent + len(data)
            if count % 20 == 0 or count == len(missing):
                log(f'  {count}/{len(missing)} sent, {sent / 1e6:.0f} MB')
    if expected:
        status, out, err = run(client, 'python3 -c ' + shlex.quote(VERIFY_MESHES), stdin=json.dumps(expected),
                               timeout=300)
        report = json.loads(out.strip().splitlines()[-1]) if status == 0 else None
        if report is None or report['bad']:
            raise RuntimeError(f"Mesh upload failed verification: {report['bad'] if report else err.strip()}")
    return {'wanted': len(wanted), 'uploaded': len(expected), 'bytes': sent}


def lock_id(root=ROOT):
    return _sha((Path(root) / HELM_LOCK).read_bytes())[:12]


def download_wheels(root=ROOT, cache=None, python=sys.executable):
    """The lock's aarch64 wheels, downloaded with their hashes checked; returns their folder."""
    folder = Path(cache or Path(root) / 'build' / 'helm-robot-deps') / lock_id(root)
    platforms = [option for platform in PLATFORMS for option in ('--platform', platform)]
    subprocess.run([python, '-m', 'pip', 'download', '--disable-pip-version-check', '--quiet', '--only-binary=:all:',
                    '--no-deps', '--require-hashes', *platforms, '--python-version', '3.10', '--implementation', 'cp',
                    '--abi', 'cp310', '--abi', 'abi3', '--abi', 'none', '-r', str(Path(root) / HELM_LOCK),
                    '-d', str(folder)], check=True)
    return folder


def deps(client, *, root=ROOT, cache=None, log=print):
    """Install this checkout's Helm lock on Minerva; returns the lock id. Nothing is fetched from there."""
    lock = lock_id(root)
    target = f'minerva/helm/deps/{lock}'
    if run(client, f'test -f ~/{target}/.complete')[0] == 0:
        log(f'Dependencies {lock} already installed.')
        return lock
    folder = download_wheels(root, cache)
    wheels = sorted(folder.glob('*.whl'))
    incoming = f'minerva/helm/wheels/{lock}'
    status, _out, err = run(client, f'rm -rf ~/{incoming} && mkdir -p ~/{incoming} ~/minerva/helm/deps')
    if status:
        raise RuntimeError(f'Could not prepare the upload: {err.strip()}')
    with client.open_sftp() as sftp:
        for path in wheels + [Path(root) / HELM_LOCK]:
            sftp.put(str(path), f'{incoming}/{path.name}')
    q = shlex.quote
    partial = f'~/{target}.partial'
    command = (f'rm -rf {partial} && python3 -m pip install --disable-pip-version-check --no-index '
               f'--find-links ~/{incoming} --require-hashes --only-binary=:all: --no-deps --target {partial} '
               f'-r ~/{incoming}/{q(Path(HELM_LOCK).name)} && '
               f'PYTHONPATH={partial} python3 -c "import aiohttp, yarl, multidict; print(aiohttp.__version__)" && '
               f'echo {q(lock)} > {partial}/.complete && mv {partial} ~/{target} && rm -rf ~/{incoming}')
    status, out, err = run(client, command, timeout=600)
    if status:
        raise RuntimeError(f'Installing the dependencies failed: {(err or out).strip()[-800:]}')
    log(f'Dependencies {lock} installed: aiohttp {out.strip().splitlines()[-1]} ({len(wheels)} wheels).')
    return lock


def remote(client, action, *args, timeout=120.):
    """ops.helm_service on Minerva: (exit status, its JSON report or None, stderr)."""
    command = ' '.join([SERVICE, shlex.quote(action), *map(shlex.quote, args)])
    status, out, err = run(client, command, timeout=timeout)
    try:
        report = json.loads(out)
    except ValueError:
        report = None
    return status, report, err.strip()


def update(client, *, root=ROOT, log=print):
    """After a deploy: the running release's dependencies and meshes, then Helm restarted on it (when installed)."""
    status, report, err = remote(client, 'status')
    if status or report is None:
        raise RuntimeError(f'Could not read Helm on Minerva: {err}')
    if report['service'].get('LoadState') != 'loaded':
        log('Helm is not installed as a service on Minerva: nothing to update.')
        return report
    deps(client, root=root, log=log)
    meshes(client, root=root, log=log)
    status, report, err = remote(client, 'restart', timeout=120.)
    if status:
        raise RuntimeError(err)
    log(f"Helm restarted on Minerva: {'listening' if report['listening'] else 'NOT listening'}"
        + (f"; missing {report['missing']}" if report['missing'] else ''))
    return report


def bridge_package(root=ROOT, out=None):
    """The GELLO bridge for the operator PC, from committed files only: (zip path, manifest)."""
    root = Path(root)
    git = lambda *args: subprocess.check_output(['git', *args], cwd=root, text=True).strip()  # noqa: E731
    if git('status', '--porcelain'):
        raise RuntimeError('Commit first: the bridge package holds committed files only')
    commit = git('rev-parse', 'HEAD')
    paths = [p for p in git('ls-files', *BRIDGE_FILES).splitlines()
             if '/static/' not in p and '__pycache__' not in p]
    folder = Path(out or root / 'build' / 'gello-bridge')
    folder.mkdir(parents=True, exist_ok=True)
    archive = folder / f'gello-bridge-{commit[:10]}.zip'
    entries = {p: _sha((root / p).read_bytes()) for p in paths}
    manifest = {'kind': 'gello_bridge_package', 'commit': commit, 'files': entries}
    partial = archive.with_name(archive.name + '.partial')
    with zipfile.ZipFile(partial, 'w', compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            info = zipfile.ZipInfo(path, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            bundle.writestr(info, (root / path).read_bytes())
        bundle.writestr(zipfile.ZipInfo('gello-bridge.json', date_time=(2026, 1, 1, 0, 0, 0)),
                        json.dumps(manifest, indent=1))
    os.replace(partial, archive)
    return archive, manifest


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m robot_platform.ops.helm_robot', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('meshes', 'deps', 'update', 'bridge', 'remote'))
    parser.add_argument('rest', nargs=argparse.REMAINDER, help='remote: the helm_service action and its options')
    parser.add_argument('--address', default=TETHER, help=f'robot address (default: the tether, {TETHER})')
    args = parser.parse_args(argv)
    if args.action == 'bridge':
        archive, manifest = bridge_package()
        print(f"{archive} ({len(manifest['files'])} files, {archive.stat().st_size / 1e6:.1f} MB)")
        return 0
    from .ssh import connect
    client = connect(address=args.address)
    try:
        if args.action == 'meshes':
            print(json.dumps(meshes(client)))
        elif args.action == 'deps':
            deps(client)
        elif args.action == 'update':
            update(client)
        else:
            if not args.rest:
                parser.error('remote needs a helm_service action, e.g. remote status')
            status, report, err = remote(client, *args.rest)
            print(json.dumps(report, indent=2) if report is not None else '', err, sep='\n')
            return status
    except (RuntimeError, ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        client.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
