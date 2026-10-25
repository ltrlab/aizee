"""Helm on Minerva's install: the robot-side service (ops.helm_service) and the build PC's part (ops.helm_robot)."""
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

import pytest

from robot_platform.ops import helm_robot, helm_service
from robot_platform.ops.release import HELM_LOCK

ROOT = Path(__file__).resolve().parents[2]
BRIDGE_KEY = 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIEnykGnVsgM3uUhoxQHByeKsZ1VBPh2wsL094geYmsMN claude@3080'
LOCAL_KEY = 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIK0zH6tHk6bE5mR8m5H2Z1q7JcVb9jK5S8u4v2W1x3y0 minerva-helm-local'


class Robot(helm_service.Host):
    """A Jetson in a directory: the commands it would run, recorded, with their files made."""

    def __init__(self, home, *, linger=False, sessions=()):
        super().__init__(home, run=self.command, proc=Path(home) / 'proc')
        self.calls, self.units, self.lingering, self.up = [], {}, linger, False
        for pid, argv in enumerate(sessions, 100):
            (self.proc / str(pid)).mkdir(parents=True)
            (self.proc / str(pid) / 'cmdline').write_bytes(b'\0'.join(part.encode() for part in argv))

    def command(self, *args):
        self.calls.append(args)
        if args[0] == 'ssh-keygen':
            key = Path(args[args.index('-f') + 1])
            key.write_text('PRIVATE')
            key.with_suffix('.pub').write_text(LOCAL_KEY + '\n')
        elif args[:2] == ('openssl', 'req'):
            Path(args[args.index('-keyout') + 1]).write_text('KEY')
            Path(args[args.index('-out') + 1]).write_text('CERT')
            self.config = Path(args[args.index('-config') + 1]).read_text()
        elif args[:2] == ('openssl', 'x509'):
            return 'notAfter=Jan 25 00:00:00 2029 GMT\nsha256 Fingerprint=AB:CD:EF\n'
        elif args[0] == 'loginctl':
            return 'yes\n' if self.lingering else 'no\n'
        elif args[:2] == ('systemctl', '--user'):
            if args[2] == 'show':
                return ''.join(f'{key}={value}\n' for key, value in self.units.get(args[-1], {}).items())
            if args[2] in ('enable', 'restart'):
                self.units[helm_service.UNIT] = {'LoadState': 'loaded', 'ActiveState': 'active',
                                                 'UnitFileState': 'enabled'}
                self.up = args[2] == 'restart'
            if args[2] == 'disable':
                self.units.pop(helm_service.UNIT, None)
        return ''

    def listening(self, port=helm_service.PORT, timeout=.5):
        return self.up


def test_prepare_makes_the_folders_key_and_a_certificate_phones_accept_once(tmp_path):
    robot = Robot(tmp_path)
    report = helm_service.prepare(robot)
    p = robot.paths
    assert all(folder.is_dir() for folder in (p.deps, p.meshes, p.operator, p.runs, p.logs, p.keys, p.tls))
    if os.name != 'nt':
        assert (p.keys.stat().st_mode & 0o777) == 0o700 and (p.tls_key.stat().st_mode & 0o777) == 0o600
    assert p.local_key.is_file() and p.cert.read_text() == 'CERT' and p.tls_key.read_text() == 'KEY'
    for line in ('IP:10.42.0.1', 'IP:127.0.0.1', 'IP:192.168.8.180', 'basicConstraints = critical, CA:FALSE',
                 'extendedKeyUsage = serverAuth'):
        assert line in robot.config
    assert ('-days', '825') == robot.calls[1][robot.calls[1].index('-days'):][:2]
    assert report['certificate'] == {'present': True, 'expires': 'Jan 25 00:00:00 2029 GMT', 'sha256': 'AB:CD:EF'}
    assert any('set-password' in item for item in report['missing'])  # The operator chooses the password.
    calls = len(robot.calls)
    helm_service.prepare(robot)  # Once only: the key and certificate stay.
    assert not [call for call in robot.calls[calls:] if call[0] in ('ssh-keygen',) or call[:2] == ('openssl', 'req')]


def test_keys_are_authorized_narrowly_once_and_removed_cleanly(tmp_path):
    robot = Robot(tmp_path)
    p = robot.paths
    p.authorized.parent.mkdir(parents=True)
    p.authorized.write_text('ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOTHER orin-nano-access\n')
    with pytest.raises(RuntimeError, match='prepare first'):
        helm_service.authorize_local(robot)
    helm_service.prepare(robot)
    report = helm_service.authorize_local(robot, clock=lambda: 0.)
    lines = p.authorized.read_text().splitlines()
    assert lines[0].endswith('orin-nano-access')  # What was there stays, first.
    assert lines[1] == ('from="127.0.0.1,::1",no-agent-forwarding,no-port-forwarding,no-X11-forwarding,no-pty '
                        + ' '.join(LOCAL_KEY.split()[:2]) + ' minerva-helm-local')
    assert report['authorized'] == {'local': True, 'bridges': []}
    assert list(p.authorized.parent.glob('authorized_keys.before-helm-*'))  # A copy of what was there.
    # An earlier release's line for this bridge (the tether only) ...
    narrower = helm_service.bridge_line(BRIDGE_KEY, 'operator-3080').replace(
        'from="10.42.0.0/16,192.168.8.0/24"', 'from="10.42.0.0/24"')
    p.authorized.write_text(p.authorized.read_text() + narrower + '\n')
    helm_service.authorize_bridge(BRIDGE_KEY, 'operator-3080', robot)
    bridge = p.authorized.read_text().splitlines()[2]
    # ... becomes the tether, the access point and Minerva's network, in place.
    assert bridge.startswith('command="cd ~/minerva/current && PYTHONPATH=src exec python3 -u -m '
                             'robot_platform.helm.bridge relay",from="10.42.0.0/16,192.168.8.0/24",')
    assert bridge.endswith(' minerva-gello-bridge:operator-3080') and 'no-port-forwarding' in bridge
    helm_service.authorize_bridge(BRIDGE_KEY, 'operator-3080', robot)  # Again: no second line.
    assert len(p.authorized.read_text().splitlines()) == 3
    with pytest.raises(ValueError, match='other options'):
        helm_service.authorize_bridge(BRIDGE_KEY, 'other-name', robot)
    for key, name in ((BRIDGE_KEY, 'bad name'), ('command="sh" ' + BRIDGE_KEY, 'pc'), (BRIDGE_KEY + '\nssh-rsa AAAA x', 'pc')):
        with pytest.raises(ValueError):
            helm_service.bridge_line(key, name)
    assert helm_service.status(robot)['authorized'] == {'local': True, 'bridges': ['operator-3080']}
    helm_service.remove(robot, clock=lambda: 1.)
    assert p.authorized.read_text().splitlines() == ['ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOTHER orin-nano-access']


def test_install_writes_the_unit_and_restart_waits_for_an_idle_robot(tmp_path):
    robot = Robot(tmp_path)
    with pytest.raises(RuntimeError, match='prepare first'):
        helm_service.install(robot)
    helm_service.prepare(robot)
    report = helm_service.install(robot, wait_s=1., clock=lambda: 0., sleep=lambda s: None)
    unit = robot.paths.unit.read_text()
    assert ('ExecStart=/usr/bin/python3 -m robot_platform.ops.helm_service run '
            '--teleop minerva-teleop-direct-tuning.json --capture-profile minerva-task-capture.json') in unit
    for line in ('CPUAffinity=0 1 2 3', 'Nice=5', f'RestartPreventExitStatus={helm_service.CONFIG_EXIT}',
                 'WorkingDirectory=%h/minerva/current', 'WantedBy=default.target'):
        assert line in unit
    assert [call[2:] for call in robot.calls if call[:2] == ('systemctl', '--user') and call[2] != 'show'] == [
        ('daemon-reload',), ('enable', helm_service.UNIT), ('restart', helm_service.UNIT)]
    assert report['listening'] and report['linger'] is False
    with pytest.raises(ValueError, match='not in this release'):
        helm_service.install(robot, teleop='nonexistent.json')
    busy = Robot(tmp_path / 'busy', sessions=[('python3', '-u', '-m', 'robot_platform.helm.edge')])
    busy.units[helm_service.UNIT] = {'LoadState': 'loaded', 'ActiveState': 'active'}
    with pytest.raises(RuntimeError, match='Not restarting Helm'):
        helm_service.restart(busy)
    assert not [call for call in busy.calls if 'restart' in call]
    idle = Robot(tmp_path / 'idle', sessions=[('python3', '-m', 'robot_platform.ops.cpu_partition')])
    idle.units[helm_service.UNIT] = {'LoadState': 'loaded', 'ActiveState': 'active'}
    helm_service.restart(idle, wait_s=1., clock=lambda: 0., sleep=lambda s: None)
    assert ('systemctl', '--user', 'restart', helm_service.UNIT) in idle.calls


def test_run_starts_helm_with_minervas_options_or_says_what_is_missing(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(sys, 'path', list(sys.path))
    monkeypatch.setenv('AIZEE_SSH_KEY_FILE', 'unset')
    robot = Robot(tmp_path)
    started = []
    assert helm_service.run([], robot, main=started.append) == helm_service.CONFIG_EXIT
    assert not started and 'set-password' in capsys.readouterr().err
    helm_service.prepare(robot)
    p = robot.paths
    p.access.write_text('{}')
    lock = helm_service.lock_id(ROOT)
    (p.deps / lock).mkdir()
    (p.deps / lock / helm_service.COMPLETE).write_text(lock)
    assert helm_service.run(['--teleop', 'minerva-teleop.json'], robot, main=lambda options: started.append(options) or 0) == 0
    options = started[0]
    flags = dict(zip(options[1::2], options[2::2])) if options[0] == '--on-robot' else {}
    assert options[0] == '--on-robot' and flags['--host'] == '0.0.0.0' and flags['--port'] == '8760'
    assert flags['--loopback-port'] == '8762'  # Plain HTTP for this computer's tools and SSH tunnels.
    assert Path(flags['--teleop']) == ROOT / 'config/devices/minerva-teleop.json'
    assert Path(flags['--operator-files']) == p.operator and Path(flags['--mesh-store']) == p.meshes
    assert Path(flags['--access']) == p.access and '--no-live' in options and '--tls-cert' in options
    assert os.environ['AIZEE_SSH_KEY_FILE'] == str(p.local_key) and sys.path[0] == str(p.deps / lock)


def test_the_mesh_store_gets_the_head_models_meshes_too():
    """Helm's 3D view draws the head model (helm.display_model): its meshes go to Minerva with the active model's."""
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.models.urdf import asset_digest
    from robot_platform.ops.release import HEAD_MODEL, MODEL
    from robot_platform.viewer.server import stored_mesh_name
    wanted = helm_robot.wanted_meshes(ROOT)
    for model in (MODEL, HEAD_MODEL):
        assets = UrdfModel(ModelProfile.load(ROOT / model)).assets
        assert {stored_mesh_name(asset_digest(asset)) for asset in assets} <= set(wanted), model


def test_meshes_upload_only_what_minerva_lacks_and_verify_it_there(tmp_path, monkeypatch):
    import gzip
    trimesh = pytest.importorskip('trimesh')
    from robot_platform.viewer.server import mesh_payload
    trimesh.creation.box().export(tmp_path / 'box.stl')
    trimesh.creation.icosphere().export(tmp_path / 'ball.stl')
    wanted = {'aaaa.mesh1.gz': tmp_path / 'box.stl', 'bbbb.mesh1.gz': tmp_path / 'ball.stl'}
    monkeypatch.setattr(helm_robot, 'wanted_meshes', lambda root=None: wanted)
    home = tmp_path / 'jetson'
    store = home / 'minerva' / 'meshes'
    store.mkdir(parents=True)
    (store / 'aaaa.mesh1.gz').write_bytes(b'already there')
    commands = []

    def run(client, command, *, timeout=60., stdin=None):
        commands.append(command)
        if command.startswith('mkdir'):
            (store / '.incoming').mkdir(exist_ok=True)
            return 0, '\n'.join(path.name for path in store.iterdir() if not path.name.startswith('.')), ''
        env = dict(os.environ, HOME=str(home), USERPROFILE=str(home))
        result = subprocess.run([sys.executable, '-c', helm_robot.VERIFY_MESHES], input=stdin, capture_output=True,
                                text=True, env=env)
        return result.returncode, result.stdout, result.stderr

    class Sftp:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def open(self, path, mode):
            handle = (home / path).open(mode)
            handle.set_pipelined = lambda flag: None
            return handle

    monkeypatch.setattr(helm_robot, 'run', run)
    client = type('Client', (), {'open_sftp': lambda self: Sftp()})()
    report = helm_robot.meshes(client, cache=tmp_path / 'cache', log=lambda message: None)
    assert report['wanted'] == 2 and report['uploaded'] == 1
    assert gzip.decompress((store / 'bbbb.mesh1.gz').read_bytes()) == mesh_payload(tmp_path / 'ball.stl')
    assert (store / 'aaaa.mesh1.gz').read_bytes() == b'already there' and not list((store / '.incoming').iterdir())
    assert helm_robot.meshes(client, cache=tmp_path / 'cache', log=lambda message: None)['uploaded'] == 0


def test_deps_install_from_the_uploaded_wheels_only_once(tmp_path, monkeypatch):
    commands, puts = [], []
    installed = {'done': False}

    def run(client, command, *, timeout=60., stdin=None):
        commands.append(command)
        if command.startswith('test -f'):
            return (0 if installed['done'] else 1), '', ''
        if 'pip install' in command:
            installed['done'] = True
            return 0, '3.14.4\n', ''
        return 0, '', ''

    class Sftp:
        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def put(self, local, remote):
            puts.append((Path(local).name, remote))

    wheels = tmp_path / 'wheels'
    wheels.mkdir()
    (wheels / 'aiohttp-3.14.4-cp310-cp310-manylinux2014_aarch64.whl').write_bytes(b'wheel')
    monkeypatch.setattr(helm_robot, 'run', run)
    monkeypatch.setattr(helm_robot, 'download_wheels', lambda root, cache: wheels)
    client = type('Client', (), {'open_sftp': lambda self: Sftp()})()
    lock = helm_robot.deps(client, log=lambda message: None)
    assert lock == helm_robot.lock_id() == helm_service.lock_id(ROOT)
    install = next(command for command in commands if 'pip install' in command)
    for part in ('--no-index', '--require-hashes', '--only-binary=:all:', '--no-deps',
                 f'--target ~/minerva/helm/deps/{lock}.partial', f'> ~/minerva/helm/deps/{lock}.partial/.complete',
                 f'mv ~/minerva/helm/deps/{lock}.partial ~/minerva/helm/deps/{lock}', 'import aiohttp'):
        assert part in install
    assert {name for name, _ in puts} == {'aiohttp-3.14.4-cp310-cp310-manylinux2014_aarch64.whl', 'helm-robot.lock'}
    commands.clear()
    helm_robot.deps(client, log=lambda message: None)  # Installed: nothing sent again.
    assert not [command for command in commands if 'pip' in command]


def test_the_bridge_package_holds_committed_files_without_the_page(tmp_path):
    def git(*args):
        subprocess.run(['git', *args], cwd=tmp_path, check=True, capture_output=True)
    for path, text in (('src/robot_platform/teleop/gello_bridge.py', 'BRIDGE = 1\n'),
                       ('src/robot_platform/helm/static/helm.js', 'page\n'), ('config/devices/robot.json', '{}\n'),
                       ('requirements/gello-bridge.lock', 'pyserial==3.5\n'),
                       ('scripts/install_gello_bridge.ps1', 'param()\n'), ('notes.md', 'not shipped\n')):
        (tmp_path / path).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / path).write_text(text)
    git('init', '-q')
    git('add', '.')
    git('-c', 'user.name=t', '-c', 'user.email=t@example.com', 'commit', '-q', '-m', 'x')
    archive, manifest = helm_robot.bridge_package(tmp_path, tmp_path / 'out')
    with zipfile.ZipFile(archive) as bundle:
        names = set(bundle.namelist())
        assert json.loads(bundle.read('gello-bridge.json')) == manifest
    assert names == {'src/robot_platform/teleop/gello_bridge.py', 'config/devices/robot.json',
                     'requirements/gello-bridge.lock', 'scripts/install_gello_bridge.ps1', 'gello-bridge.json'}
    assert archive.name == f"gello-bridge-{manifest['commit'][:10]}.zip"
    (tmp_path / 'config/devices/robot.json').write_text('{"changed": true}\n')
    with pytest.raises(RuntimeError, match='Commit first'):
        helm_robot.bridge_package(tmp_path, tmp_path / 'out')


def test_the_robot_lock_and_the_bridge_lock_pin_the_development_versions():
    dev = {line.split(' ')[0] for line in (ROOT / 'requirements/dev.lock').read_text().splitlines() if '==' in line}
    for lock in (HELM_LOCK, 'requirements/gello-bridge.lock'):
        pins = {line.split(' ')[0] for line in (ROOT / lock).read_text().splitlines() if '==' in line
                and not line.startswith('#')}
        extra = {pin for pin in pins - dev if not pin.startswith('async-timeout==')}  # Only Python 3.10 needs it.
        assert pins and not extra, (lock, extra)
