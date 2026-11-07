"""Train a head scene capture on the operator station and put the map where Helm on Minerva serves it
(docs/architecture/HEADSET_3D_PLAN.md, phase 4). Run on the build PC, which holds both keys: the robot's (the device
profile's, pinned by ops.ssh) and the station's.

    python scripts/splat/train_on_station.py --station <user>@<station> --station-key <key file> \
        --station-host-key SHA256:<its host key> [--robot <address>] \
        [--capture NAME] [--steps 7000] [--min-free-mb 3000]

1. Minerva: the capture under ~/minerva/helm/maps/captures (Helm's map card records them; the newest without a map
   when none is named), zipped there and fetched.
2. The station: the capture and this folder's trainer copied into %USERPROFILE%\\minerva-splat\\scripts, then trained
   in its Ubuntu (WSL2) venv (~/minerva-splat/venv, set up by install_wsl.sh). Its GPU is shared with the user's own
   work: its load is printed first, and nothing trains with less than --min-free-mb free.
3. Minerva: map.splat and map.json into ~/minerva/helm/maps/maps/<capture>/, written beside and then renamed into
   place, where Helm serves them; the headset's Map draws the newest it can place.
Nothing here moves the robot or changes the station beyond its minerva-splat folders.
"""

import argparse
import base64
import hashlib
import json
from pathlib import Path
import re
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
TRAINER = Path(__file__).with_name('train_head_scene.py')
NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')
MAPS = 'minerva/helm/maps'
WSL_SCRIPT = """#!/usr/bin/env bash
# Written by scripts/splat/train_on_station.py: one capture trained in this station's WSL venv.
set -euo pipefail
NAME="$1"; STEPS="$2"; UP="$3"
ROOT="$HOME/minerva-splat"
if [ "$NAME" = "--gpu" ]; then
  nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits
  exit 0
fi
mkdir -p "$ROOT/captures" "$ROOT/scenes"
rm -rf "${ROOT:?}/captures/$NAME"
python3 -m zipfile -e "$UP/$NAME.zip" "$ROOT/captures/"
cp "$UP/train_head_scene.py" "$ROOT/"
. "$ROOT/venv/bin/activate"
python "$ROOT/train_head_scene.py" "$ROOT/captures/$NAME" "$ROOT/scenes/$NAME.ply" --steps "$STEPS"
cp "$ROOT/scenes/$NAME.splat" "$ROOT/scenes/$NAME.json" "$UP/"
rm -f "$UP/$NAME.zip"
"""
# Lists the captures and maps on Minerva, as JSON (python3 there).
LIST = (
    """import json, pathlib
root = pathlib.Path.home() / %r
captures = sorted(p.name for p in (root / 'captures').glob('*') if (p / 'scene.json').is_file())
maps = sorted(p.name for p in (root / 'maps').glob('*') if (p / 'map.splat').is_file())
print(json.dumps({'captures': captures, 'maps': maps}))
"""
    % MAPS
)


def pinned_client(host, user, key, fingerprint, timeout=15.0):
    """An SSH client to the station, refusing any host key but `fingerprint` (SHA256:...)."""
    import paramiko

    class Pinned(paramiko.MissingHostKeyPolicy):
        def missing_host_key(self, client, hostname, host_key):
            actual = 'SHA256:' + base64.b64encode(hashlib.sha256(host_key.asbytes()).digest()).decode().rstrip('=')
            if actual != fingerprint:
                raise paramiko.SSHException(f'Host key mismatch for {hostname}: {actual}')
            client.get_host_keys().add(hostname, host_key.get_name(), host_key)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(Pinned())
    client.connect(
        host,
        username=user,
        key_filename=str(key),
        look_for_keys=False,
        allow_agent=False,
        timeout=timeout,
        auth_timeout=timeout,
        banner_timeout=timeout,
    )
    client.get_transport().set_keepalive(15)
    return client


def run(client, command, timeout):
    from robot_platform.ops.ssh import run as remote

    status, out, err = remote(client, command, timeout=timeout)
    if status:
        raise RuntimeError(f'{command.split()[0]} failed ({status}): {(err or out).strip()[-1500:]}')
    return out


def choose(listing, name):
    """The capture to train: the one named, else the newest without a map."""
    if name is not None:
        if name not in listing['captures']:
            raise SystemExit(f'No capture {name} on Minerva; there are: {", ".join(listing["captures"]) or "none"}')
        return name
    untrained = [capture for capture in listing['captures'] if capture not in listing['maps']]
    if not untrained:
        raise SystemExit('Every capture on Minerva has its map; name one with --capture to train it again')
    return untrained[-1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--station', required=True, help='user@host of the operator station')
    parser.add_argument('--station-key', required=True, type=Path, help='private key for the station')
    parser.add_argument('--station-host-key', required=True, help="the station's host key fingerprint, SHA256:...")
    parser.add_argument('--robot', default=None, help="Minerva's address (default: the tether's, ops.ssh)")
    parser.add_argument('--profile', type=Path, default=None, help='calibration profile naming the robot key')
    parser.add_argument('--capture', default=None)
    parser.add_argument('--steps', type=int, default=7000)
    parser.add_argument('--min-free-mb', type=int, default=3000)
    args = parser.parse_args(argv)
    from robot_platform.ops.ssh import DEFAULT_PROFILE, TETHER, connect

    if args.capture is not None and not NAME.match(args.capture):
        raise SystemExit('A capture name is letters, digits, dots, dashes and underscores')
    user, _, host = args.station.rpartition('@')
    robot = connect(args.profile or DEFAULT_PROFILE, address=args.robot or TETHER, timeout=10.0)
    station = pinned_client(host, user, args.station_key, args.station_host_key)
    try:
        listing = json.loads(run(robot, 'python3 -c ' + _quote(LIST), 30))
        name = choose(listing, args.capture)
        robot_sftp, station_sftp = robot.open_sftp(), station.open_sftp()
        home = robot_sftp.normalize('.')
        # PowerShell, encoded: through the station's shell, cmd's echo came back with a stray quote.
        query = base64.b64encode('[Environment]::GetFolderPath("UserProfile")'.encode('utf-16-le')).decode()
        profile = run(station, f'powershell.exe -NoProfile -NonInteractive -EncodedCommand {query}', 30)
        profile = profile.strip().replace('\\', '/')
        windows = f'{profile}/minerva-splat/scripts'
        linux = '/mnt/' + profile[0].lower() + profile[2:] + '/minerva-splat/scripts'
        _mkdirs(station_sftp, windows)
        station_sftp.putfo(_bytes(WSL_SCRIPT.encode()), f'{windows}/train_capture.sh')
        wsl = f'wsl.exe -d Ubuntu -- bash {linux}/train_capture.sh'
        used, total, load = (float(v) for v in run(station, f'{wsl} --gpu 0 {linux}', 60).split(','))
        print(f'Station GPU: {used:.0f} of {total:.0f} MB in use, {load:.0f} % busy', flush=True)
        if total - used < args.min_free_mb:
            raise SystemExit(
                f'Only {total - used:.0f} MB of the station GPU is free (--min-free-mb '
                f'{args.min_free_mb}): try again when its own work is done'
            )
        with tempfile.TemporaryDirectory() as scratch:
            archive = Path(scratch) / f'{name}.zip'
            print(f'Fetching capture {name} from Minerva...', flush=True)
            run(robot, f'cd {home}/{MAPS}/captures && python3 -m zipfile -c /tmp/{name}.zip {name}', 300)
            robot_sftp.get(f'/tmp/{name}.zip', str(archive))
            run(robot, f'rm -f /tmp/{name}.zip', 30)
            station_sftp.put(str(archive), f'{windows}/{name}.zip')
            station_sftp.put(str(TRAINER), f'{windows}/train_head_scene.py')
            print(f'Training on the station ({args.steps} steps)...', flush=True)
            print(run(station, f'{wsl} {name} {args.steps} {linux}', 3600).strip()[-2000:], flush=True)
            target = f'{home}/{MAPS}/maps/{name}'
            _mkdirs(robot_sftp, target)
            for suffix, final in (('.splat', 'map.splat'), ('.json', 'map.json')):
                local = Path(scratch) / f'{name}{suffix}'
                station_sftp.get(f'{windows}/{name}{suffix}', str(local))
                station_sftp.remove(f'{windows}/{name}{suffix}')
                robot_sftp.put(str(local), f'{target}/{final}.part')
                robot_sftp.posix_rename(f'{target}/{final}.part', f'{target}/{final}')
        report = json.loads((robot_sftp.open(f'{target}/map.json').read()).decode())
        print(
            f'Map {name} on Minerva: {report.get("gaussians")} Gaussians, {report.get("psnr_held_out")} dB held out, '
            f"in {report.get('world')}'s frame",
            flush=True,
        )
    finally:
        station.close()
        robot.close()
    return 0


def _quote(text):
    import shlex

    return shlex.quote(text)


def _bytes(data):
    import io

    return io.BytesIO(data)


def _mkdirs(sftp, path):
    """Each missing folder down to `path` (/home/... on Minerva, C:/Users/... on the station)."""
    parts = path.split('/')
    for k in range(1, len(parts) + 1):
        current = '/'.join(parts[:k])
        if not parts[k - 1] or current.endswith(':'):
            continue
        try:
            sftp.stat(current)
        except OSError:
            sftp.mkdir(current)


if __name__ == '__main__':
    raise SystemExit(main())
