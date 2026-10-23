"""Helm on Minerva: the user service that serves Helm from the Jetson to the operator's devices.

Run on the Jetson as the robot's user, from the active release:

    cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service ACTION

    prepare           folders, Helm's SSH key to this computer and a self-signed HTTPS certificate
    authorize-local   let Helm's key log in to this computer from this computer only (adds a line to authorized_keys)
    authorize-bridge  --key "ssh-ed25519 AAAA..." --name NAME: let the GELLO bridge's key run Helm's relay and nothing
                      else, over the tether only (adds a line to authorized_keys)
    install           write, enable and start the user unit minerva-helm.service
    restart           restart it, only while no session or check runs on the robot (after a deploy)
    status            what is in place, as JSON
    remove            stop and remove the unit and the keys authorized here; the folders stay
    run               what the unit runs: Helm with Minerva's options

Once, by the user (root): the unit keeps running when nobody is logged in only with lingering,
    sudo loginctl enable-linger ltr
and Helm's password is the operator's to choose (it asks for it):
    cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.helm.access set-password ~/minerva/helm/access.json

Layout:
    ~/minerva/helm/deps/<lock>/   Helm's dependencies for a release's requirements/helm-robot.lock, installed from the
                                  build PC (python -m robot_platform.ops.helm_robot deps)
    ~/minerva/meshes/             the 3D view's meshes, converted on the build PC (ops.helm_robot meshes)
    ~/minerva/helm/keys/          Helm's SSH key to this computer: every robot path Helm takes is the same pinned SSH as
                                  from the operator PC, to 127.0.0.1
    ~/minerva/helm/tls/           the HTTPS certificate and key
    ~/minerva/helm/access.json    the login password's scrypt hash (helm.access)
    ~/minerva/helm/operator/      the joint limits, servo calibration and leader ranges the operator saves
                                  (helm.operator_files)
    ~/minerva/helm/runs/          Helm's sessions and evidence
    ~/minerva/helm/logs/helm.log  what Helm prints
Standard library only, besides the release's own modules.
"""
import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]  # The release (or checkout) this module runs from.
UNIT = 'minerva-helm.service'
PORT = 8760
PROFILE = 'config/devices/minerva-joint-calibration.json'
TELEOP = 'minerva-teleop-direct-tuning.json'  # What the operator PC's Helm ran (config/devices).
CAPTURE = 'minerva-task-capture.json'
CONFIG_EXIT = 78  # A missing password, key or dependency: the unit does not restart until it is fixed.
LOCAL_NAME = 'minerva-helm-local'
BRIDGE_NAME = 'minerva-gello-bridge'
COMPLETE = '.complete'
SESSIONS = ('robot_platform.helm.edge', 'robot_platform.ops.probe')  # A session's loop, or Minerva Check.
PUBLIC_KEY = re.compile(r'^(ssh-ed25519|ecdsa-sha2-nistp(?:256|384|521)|ssh-rsa) ([A-Za-z0-9+/]+={0,3})(?: [^\n]*)?$')
NAME = re.compile(r'^[A-Za-z0-9._-]{1,40}$')


class Paths:
    def __init__(self, home):
        self.home = Path(home)
        self.base = self.home / 'minerva'
        self.helm = self.base / 'helm'
        self.deps = self.helm / 'deps'
        self.meshes = self.base / 'meshes'
        self.keys = self.helm / 'keys'
        self.local_key = self.keys / 'helm_local_ed25519'
        self.tls = self.helm / 'tls'
        self.cert, self.tls_key = self.tls / 'helm.crt', self.tls / 'helm.key'
        self.access = self.helm / 'access.json'
        self.operator = self.helm / 'operator'
        self.runs = self.helm / 'runs'
        self.logs = self.helm / 'logs'
        self.authorized = self.home / '.ssh' / 'authorized_keys'
        self.unit = self.home / '.config' / 'systemd' / 'user' / UNIT


class Host:
    """The robot's commands and processes. Tests give it a home directory and record what it would run."""

    def __init__(self, home=None, run=None, proc='/proc'):
        self.paths = Paths(home or Path.home())
        self._run, self.proc = run, Path(proc)

    def run(self, *args, check=True):
        if self._run is not None:
            return self._run(*args)
        env = dict(os.environ)
        if hasattr(os, 'getuid'):
            env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')  # systemctl --user over plain SSH.
        result = subprocess.run(list(args), capture_output=True, text=True, env=env)
        if check and result.returncode != 0:
            raise RuntimeError(f"{' '.join(args)}: {(result.stderr or result.stdout).strip()}")
        return result.stdout

    def systemctl(self, *args, check=True):
        return self.run('systemctl', '--user', *args, check=check)

    def unit_state(self):
        text = self.systemctl('show', '-p', 'LoadState', '-p', 'ActiveState', '-p', 'UnitFileState', UNIT, check=False)
        return dict(line.split('=', 1) for line in text.splitlines() if '=' in line)

    def linger(self):
        user = os.environ.get('USER') or self.paths.home.name
        return self.run('loginctl', 'show-user', user, '-p', 'Linger', '--value', check=False).strip() == 'yes'

    def sessions(self):
        """Processes running a session's loop or a check on this computer."""
        found = []
        try:
            entries = list(self.proc.iterdir())
        except OSError:
            return found
        for entry in entries:
            if entry.name.isdigit():
                try:
                    argv = (entry / 'cmdline').read_bytes().split(b'\0')
                except OSError:
                    continue
                if any(marker.encode() in argv for marker in SESSIONS):
                    found.append(int(entry.name))
        return sorted(found)

    def listening(self, port=PORT, timeout=.5):
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=timeout):
                return True
        except OSError:
            return False


def lock_id(release=ROOT):
    """The dependency folder name of a release: its Helm lock's digest."""
    from .release import HELM_LOCK
    return hashlib.sha256((Path(release) / HELM_LOCK).read_bytes()).hexdigest()[:12]


def _write(path, data, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    descriptor = os.open(partial, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    with os.fdopen(descriptor, 'wb') as handle:
        handle.write(data)
    os.chmod(partial, mode)
    os.replace(partial, path)


# -- prepare: folders, Helm's SSH key, the HTTPS certificate --------------------------------------------------------

def addresses(profile=None):
    """(IP addresses, DNS names) the certificate names: the tether, loopback, the Wi-Fi address of the device profile,
    and this computer's name."""
    from .ssh import TETHER, robot_identity
    device = robot_identity(profile or ROOT / PROFILE)[0]
    ips = [TETHER, '127.0.0.1'] + ([device['address']] if device.get('address') else [])
    name = socket.gethostname()
    return list(dict.fromkeys(ips)), [name, f'{name}.local']


def certificate_config(ips, names):
    """An OpenSSL request config for a self-signed server certificate that browsers on phones and tablets accept once
    trusted: names in subjectAltName, not a CA, serverAuth (Apple also limits it to 825 days)."""
    alt = ', '.join([f'IP:{ip}' for ip in ips] + [f'DNS:{name}' for name in names])
    return (f'[req]\ndistinguished_name = dn\nx509_extensions = ext\nprompt = no\n[dn]\nCN = {names[0]}\n'
            f'O = Minerva Helm\n[ext]\nsubjectAltName = {alt}\nbasicConstraints = critical, CA:FALSE\n'
            'keyUsage = critical, digitalSignature\nextendedKeyUsage = serverAuth\nsubjectKeyIdentifier = hash\n')


def prepare(host=None, *, profile=None):
    """Make the folders, Helm's SSH key to this computer and the HTTPS certificate, each only once."""
    host = host or Host()
    p = host.paths
    for folder in (p.helm, p.deps, p.meshes, p.operator, p.runs, p.logs):
        folder.mkdir(parents=True, exist_ok=True)
    for folder in (p.keys, p.tls):
        folder.mkdir(parents=True, exist_ok=True)
        os.chmod(folder, 0o700)
    if not p.local_key.is_file():
        host.run('ssh-keygen', '-q', '-t', 'ed25519', '-N', '', '-C', LOCAL_NAME, '-f', str(p.local_key))
    if not (p.cert.is_file() and p.tls_key.is_file()):
        ips, names = addresses(profile)
        with tempfile.TemporaryDirectory() as scratch:
            config = Path(scratch) / 'helm-certificate.cnf'
            config.write_text(certificate_config(ips, names), encoding='utf-8')
            key, cert = p.tls / 'helm.key.partial', p.tls / 'helm.crt.partial'
            host.run('openssl', 'req', '-x509', '-config', str(config), '-newkey', 'ec', '-pkeyopt',
                     'ec_paramgen_curve:prime256v1', '-nodes', '-days', '825', '-keyout', str(key), '-out', str(cert))
            os.chmod(key, 0o600)
            os.replace(key, p.tls_key)
            os.replace(cert, p.cert)
    return status(host)


# -- authorized keys -------------------------------------------------------------------------------------------------

def tether_network():
    from .ssh import TETHER
    return str(ipaddress.ip_network(f'{TETHER}/24', strict=False))


def local_line(public_key):
    """Helm's own key: from this computer only, no forwarding, no terminal."""
    kind, blob = _parse(public_key)
    return f'from="127.0.0.1,::1",no-agent-forwarding,no-port-forwarding,no-X11-forwarding,no-pty {kind} {blob} {LOCAL_NAME}'


def bridge_line(public_key, name):
    """The GELLO bridge's key: Helm's relay only (a forced command), from the tether only."""
    from robot_platform.helm.bridge import RELAY_COMMAND
    if not NAME.match(name or ''):
        raise ValueError('A bridge name is 1-40 letters, digits, dots, dashes or underscores')
    kind, blob = _parse(public_key)
    return (f'command="{RELAY_COMMAND}",from="{tether_network()}",no-agent-forwarding,no-port-forwarding,'
            f'no-X11-forwarding,no-pty {kind} {blob} {BRIDGE_NAME}:{name}')


def _parse(public_key):
    match = PUBLIC_KEY.match(str(public_key).strip())
    if not match:
        raise ValueError('Expected one public key line, as ssh-keygen writes it: "ssh-ed25519 AAAA... comment"')
    return match.group(1), match.group(2)


def _ours(line):
    fields = line.split()
    return bool(fields) and (fields[-1] == LOCAL_NAME or fields[-1].startswith(BRIDGE_NAME + ':'))


def _rewrite(path, lines, clock):
    """Replace authorized_keys with `lines`, keeping a dated copy of what was there."""
    if path.is_file():
        stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(clock()))
        shutil.copy2(path, path.with_name(f'{path.name}.before-helm-{stamp}'))
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    _write(path, ''.join(line + '\n' for line in lines).encode(), mode=0o600)


def authorize(host, line, *, clock=time.time):
    """Add one key line to authorized_keys; False when it is already there. A key already present with other options
    is refused: adding ours would not narrow what that line allows."""
    path = host.paths.authorized
    lines = path.read_text(encoding='utf-8').splitlines() if path.is_file() else []
    blob = line.split()[-2]
    for current in lines:
        if blob in current.split():
            if current.strip() == line:
                return False
            raise ValueError('That key is already in authorized_keys with other options: remove that line first')
    _rewrite(path, lines + [line], clock)
    return True


def authorize_local(host=None, *, clock=time.time):
    host = host or Host()
    public = host.paths.local_key.with_suffix('.pub')
    if not public.is_file():
        raise RuntimeError('No Helm key yet: run prepare first')
    lines = host.paths.authorized.read_text(encoding='utf-8').splitlines() if host.paths.authorized.is_file() else []
    line = local_line(public.read_text(encoding='utf-8'))
    stale = [current for current in lines if current.split()[-1:] == [LOCAL_NAME] and current.strip() != line]
    if stale:  # A key made again by prepare: the old one goes.
        _rewrite(host.paths.authorized, [current for current in lines if current not in stale], clock)
    authorize(host, line, clock=clock)
    return status(host)


def authorize_bridge(public_key, name, host=None, *, clock=time.time):
    host = host or Host()
    authorize(host, bridge_line(public_key, name), clock=clock)
    return status(host)


def authorized(host):
    """{'local': bool (Helm's current key), 'bridges': [names]} among the lines this module added."""
    lines = host.paths.authorized.read_text(encoding='utf-8').splitlines() if host.paths.authorized.is_file() else []
    public = host.paths.local_key.with_suffix('.pub')
    local = local_line(public.read_text(encoding='utf-8')) if public.is_file() else None
    return {'local': local is not None and local in (line.strip() for line in lines),
            'bridges': [line.split()[-1].split(':', 1)[1] for line in lines
                        if line.split()[-1:] and line.split()[-1].startswith(BRIDGE_NAME + ':')]}


# -- the unit --------------------------------------------------------------------------------------------------------

def unit_text(*, teleop=TELEOP, capture=CAPTURE, live=False):
    options = f'--teleop {teleop} --capture-profile {capture}' + (' --live' if live else '')
    return f"""[Unit]
Description=Helm on Minerva: the operator console, served to the operator's devices
After=network-online.target

[Service]
Type=simple
WorkingDirectory=%h/minerva/current
Environment=PYTHONPATH=%h/minerva/current/src
Environment=PYTHONUNBUFFERED=1
ExecStart=/usr/bin/python3 -m robot_platform.ops.helm_service run {options}
Restart=on-failure
RestartSec=5
RestartPreventExitStatus={CONFIG_EXIT}
# Off the control CPUs (4-5) the teleop loop and receiver keep to (robot_platform.ops.cpu_partition), and below the
# cameras that share 0-3.
CPUAffinity=0 1 2 3
Nice=5
StandardOutput=append:%h/minerva/helm/logs/helm.log
StandardError=inherit

[Install]
WantedBy=default.target
"""


def install(host=None, *, teleop=TELEOP, capture=CAPTURE, live=False, wait_s=60., clock=time.monotonic,
            sleep=time.sleep):
    host = host or Host()
    p = host.paths
    if not (p.local_key.is_file() and p.cert.is_file()):
        raise RuntimeError('Run prepare first')
    for name in (teleop, capture):
        if not (ROOT / 'config/devices' / name).is_file():
            raise ValueError(f'config/devices/{name} is not in this release')
    _write(p.unit, unit_text(teleop=teleop, capture=capture, live=live).encode())
    host.systemctl('daemon-reload')
    host.systemctl('enable', UNIT)
    host.systemctl('restart', UNIT)  # Picks up a reinstall's new options too.
    return _await(host, wait_s, clock, sleep)


def _await(host, wait_s, clock, sleep):
    """The status once Helm listens, or once it is plain it will not (a missing password, key or dependency)."""
    deadline = clock() + wait_s
    while True:
        report = status(host)
        if report['listening'] or report['missing'] or clock() > deadline:
            return report
        if report['service'].get('ActiveState') == 'failed':
            return report
        sleep(.5)


def restart(host=None, *, wait_s=60., clock=time.monotonic, sleep=time.sleep):
    """Restart Helm (a deploy switched the release), never under a running session or check."""
    host = host or Host()
    if host.unit_state().get('LoadState') != 'loaded':
        return status(host)
    busy = host.sessions()
    if busy:
        raise RuntimeError(f"Not restarting Helm: a session or Minerva Check runs on the robot (process "
                           f"{', '.join(map(str, busy))}). Restart it when that ends: python3 -m "
                           'robot_platform.ops.helm_service restart')
    host.systemctl('restart', UNIT)
    return _await(host, wait_s, clock, sleep)


def remove(host=None, *, clock=time.time):
    host = host or Host()
    if host.unit_state().get('LoadState') == 'loaded':
        host.systemctl('disable', '--now', UNIT, check=False)
    if host.paths.unit.exists():
        host.paths.unit.unlink()
    host.systemctl('daemon-reload', check=False)
    path = host.paths.authorized
    if path.is_file():
        lines = path.read_text(encoding='utf-8').splitlines()
        if any(_ours(line) for line in lines):
            _rewrite(path, [line for line in lines if not _ours(line)], clock)
    return status(host)


def _certificate(host):
    if not host.paths.cert.is_file():
        return None
    try:
        text = host.run('openssl', 'x509', '-in', str(host.paths.cert), '-noout', '-enddate', '-fingerprint',
                        '-sha256', check=False)
    except OSError:
        return {'present': True}
    facts = dict(line.split('=', 1) for line in text.splitlines() if '=' in line)
    return {'present': True, 'expires': facts.get('notAfter'),
            'sha256': next((value for key, value in facts.items() if 'Fingerprint' in key), None)}


def status(host=None, *, release=ROOT):
    """What is in place, and `missing`: what keeps Helm from starting."""
    host = host or Host()
    p = host.paths
    report = {'service': host.unit_state(), 'linger': host.linger(), 'listening': host.listening(),
              'key': p.local_key.is_file(), 'certificate': _certificate(host), 'password': p.access.is_file(),
              'authorized': authorized(host), 'dependencies': None, 'meshes': None, 'missing': []}
    try:
        lock = lock_id(release)
        report['dependencies'] = {'lock': lock, 'installed': (p.deps / lock / COMPLETE).is_file()}
    except OSError:
        report['dependencies'] = {'lock': None, 'installed': False}
    try:
        from .release import LOCK
        from robot_platform.viewer.server import stored_mesh_name
        wanted = set(json.loads((Path(release) / LOCK).read_text(encoding='utf-8')).values())
        have = {path.name for path in p.meshes.glob('*.gz')} if p.meshes.is_dir() else set()
        report['meshes'] = {'wanted': len(wanted), 'missing': sum(stored_mesh_name(sha) not in have for sha in wanted)}
    except (OSError, ValueError):
        pass
    missing = report['missing']
    if not report['password']:
        missing.append('password: python3 -m robot_platform.helm.access set-password ~/minerva/helm/access.json')
    if not report['key']:
        missing.append("Helm's SSH key: prepare")
    if not report['dependencies']['installed']:
        missing.append("this release's dependencies: python -m robot_platform.ops.helm_robot deps (from the build PC)")
    return report


# -- run: what the unit runs -----------------------------------------------------------------------------------------

def run(argv=None, host=None, *, release=ROOT, main=None):
    """Helm with Minerva's options: on the robot, beyond loopback with a login, its state outside the release."""
    parser = argparse.ArgumentParser(prog='python3 -m robot_platform.ops.helm_service run')
    parser.add_argument('--teleop', default=TELEOP, help='Teleop profile in config/devices')
    parser.add_argument('--capture-profile', default=CAPTURE, help='Camera capture profile in config/devices')
    parser.add_argument('--live', action='store_true', help='Start with the stopped-motor reader on')
    args = parser.parse_args(argv)
    host = host or Host()
    p = host.paths
    report = status(host, release=release)
    if report['missing']:
        print('Helm on Minerva cannot start without ' + '; '.join(report['missing']), file=sys.stderr, flush=True)
        return CONFIG_EXIT
    sys.path.insert(0, str(p.deps / report['dependencies']['lock']))  # Before anything imports aiohttp.
    os.environ['AIZEE_SSH_KEY_FILE'] = str(p.local_key)  # Every robot path: pinned SSH to 127.0.0.1 (ops.ssh).
    devices = Path(release) / 'config/devices'
    options = ['--on-robot', '--profile', str(Path(release) / PROFILE), '--host', '0.0.0.0', '--port', str(PORT),
               '--access', str(p.access), '--runs', str(p.runs), '--operator-files', str(p.operator),
               '--mesh-store', str(p.meshes), '--teleop', str(devices / args.teleop),
               '--capture-profile', str(devices / args.capture_profile)]
    if not args.live:
        options.append('--no-live')
    if p.cert.is_file() and p.tls_key.is_file():
        options += ['--tls-cert', str(p.cert), '--tls-key', str(p.tls_key)]
    if main is None:
        from robot_platform.helm.cli import main
    return main(options)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ['run']:
        return run(argv[1:])  # Its own options (run --help).
    parser = argparse.ArgumentParser(prog='python3 -m robot_platform.ops.helm_service', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('prepare', 'authorize-local', 'authorize-bridge', 'install', 'restart',
                                           'status', 'remove', 'run'))
    parser.add_argument('--key', help='authorize-bridge: the bridge PC\'s public key line')
    parser.add_argument('--name', help='authorize-bridge: a name for that PC, e.g. operator-3080')
    parser.add_argument('--teleop', default=TELEOP, help=f'install: teleop profile in config/devices ({TELEOP})')
    parser.add_argument('--capture-profile', default=CAPTURE, help=f'install: capture profile ({CAPTURE})')
    parser.add_argument('--live', action='store_true', help='install: start Helm with the stopped-motor reader on')
    args = parser.parse_args(argv)
    try:
        if args.action == 'prepare':
            report = prepare()
        elif args.action == 'authorize-local':
            report = authorize_local()
        elif args.action == 'authorize-bridge':
            report = authorize_bridge(args.key, args.name)
        elif args.action == 'install':
            report = install(teleop=args.teleop, capture=args.capture_profile, live=args.live)
        elif args.action == 'restart':
            report = restart()
        elif args.action == 'remove':
            report = remove()
        else:
            report = status()
    except (RuntimeError, ValueError, OSError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    if args.action in ('install', 'restart') and not report['linger']:
        print('Lingering is off: Helm stops when the last login session ends. Once, as root: '
              'sudo loginctl enable-linger ' + (os.environ.get('USER') or 'ltr'), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
