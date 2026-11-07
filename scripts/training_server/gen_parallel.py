"""Run several `evaluation generate` workers at once on coolserver, with disjoint grid seeds and their own output
folders, then gather the published episodes into one folder (hard links) with one combined manifest.

Linux port of the laptop's gen_parallel.py. Each worker runs its own receiver (the Linux control_harness) and scene
subprocesses over pipes; nothing is shared but the CPU and GPU 0, where MuJoCo renders through EGL. Workers run under
`nice`, with one BLAS/OpenMP thread each: the box is shared. Every worker log line is stamped with its wall time, so
the summary can give the wall time per attempt.

    $AIZEE_SIM/.venv/bin/python $AIZEE_SIM/bin/gen_parallel.py --out DIR --episodes N [--workers 6] \
        [--tree $AIZEE_SIM/tree-01e2198] [--seed 1000] -- [more generate options]
"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import statistics
import subprocess
import sys
import threading
import time

HOME = Path(os.environ.get('AIZEE_SIM', Path.home() / 'aizee-sim'))
WORKCELL = 'config/devices/simulation/minerva-production-01-workcell.json'
ATTEMPT = re.compile(r'^attempt (\d+): (\S+)')


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def environment(tree, egl_device):
    env = dict(os.environ)
    env.update(
        {
            'PYTHONPATH': str(tree / 'src'),
            'ROBOT_CONTROL_HARNESS': str(HOME / 'bin/control_harness'),
            'ROBOT_COLLISION_LIBRARY': str(HOME / 'bin/librobot_collision.so'),
            # EGL device 0 is nvidia-smi GPU 0 on this box (checked 2026-10-18).
            'MUJOCO_GL': 'egl',
            'MUJOCO_EGL_DEVICE_ID': str(egl_device),
            'PYOPENGL_PLATFORM': 'egl',
            'OMP_NUM_THREADS': '1',
            'OPENBLAS_NUM_THREADS': '1',
            'MKL_NUM_THREADS': '1',
            'NUMEXPR_NUM_THREADS': '1',
            'TMPDIR': str(HOME / 'tmp'),
            'PYTHONUNBUFFERED': '1',
        }
    )
    return env


def pump(process, log, started, stamps):
    """Copy a worker's output into its log, each line stamped; keep the stamp of each 'attempt N:' line."""
    for raw in process.stdout:
        line = raw.decode('utf-8', 'replace').rstrip('\n')
        now = time.time()
        log.write(f'[{datetime.datetime.fromtimestamp(now):%H:%M:%S} +{now - started:7.1f}s] {line}\n')
        log.flush()
        match = ATTEMPT.match(line)
        if match:
            stamps.append((int(match.group(1)), match.group(2), now))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', required=True)
    p.add_argument('--episodes', type=int, required=True, help='total successful episodes wanted')
    p.add_argument('--workers', type=int, default=6)
    p.add_argument('--seed', type=int, default=1000, help='worker k uses grid seeds from seed + 100 k')
    p.add_argument('--tree', default=str(HOME / 'tree-01e2198'), help='the source tree to run (its src, config, urdf)')
    p.add_argument('--python', default=str(HOME / '.venv/bin/python'))
    p.add_argument('--workcell', default=WORKCELL)
    p.add_argument('--nice', type=int, default=10)
    p.add_argument('--egl-device', type=int, default=0)
    p.add_argument('extra', nargs=argparse.REMAINDER, help='more generate options, after --')
    a = p.parse_args()
    extra = a.extra[1:] if a.extra[:1] == ['--'] else a.extra
    tree, out = Path(a.tree).resolve(), Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    (HOME / 'tmp').mkdir(exist_ok=True)
    env = environment(tree, a.egl_device)
    counts = [a.episodes // a.workers + (k < a.episodes % a.workers) for k in range(a.workers)]
    started = time.time()
    workers = []
    for k, count in enumerate(counts):
        if not count:
            continue
        log = open(out / f'w{k}.log', 'w', encoding='utf-8')
        command = [
            a.python,
            '-m',
            'robot_platform.evaluation',
            'generate',
            '--episodes',
            str(count),
            '--out',
            str(out / f'w{k}'),
            '--workcell',
            a.workcell,
            '--seed',
            str(a.seed + 100 * k),
            '--plant-seed',
            str(k + 1),
            *extra,
        ]
        # Its own session, so a stop reaches the worker's receiver and scene subprocesses too.
        process = subprocess.Popen(
            ['nice', '-n', str(a.nice), *command],
            cwd=tree,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        stamps = []
        thread = threading.Thread(target=pump, args=(process, log, started, stamps), daemon=True)
        thread.start()
        workers.append(
            {
                'k': k,
                'process': process,
                'log': log,
                'command': command,
                'thread': thread,
                'stamps': stamps,
                'count': count,
            }
        )

    def stop(signum, frame):
        for w in workers:
            if w['process'].poll() is None:
                try:
                    os.killpg(w['process'].pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGHUP, stop)
    codes, ended = {}, {}
    for w in workers:
        codes[w['k']] = w['process'].wait()
        ended[w['k']] = time.time()
        w['thread'].join(timeout=10)
        w['log'].close()

    episodes = out / 'episodes'
    episodes.mkdir(exist_ok=True)
    manifest = []
    for w in workers:
        path = out / f'w{w["k"]}' / 'manifest.jsonl'
        if not path.exists():
            continue
        # Wall time of each attempt: from the previous attempt's line (or the worker's start) to its own.
        stamped, previous = {}, started
        for number, _, at in w['stamps']:
            stamped[number] = round(at - previous, 1)
            previous = at
        for line in path.read_text(encoding='utf-8').splitlines():
            entry = dict(json.loads(line), worker=w['k'])
            entry['wall_s'] = stamped.get(entry.get('attempt'))
            if entry.get('episode'):
                source = Path(entry['episode'])
                target = episodes / source.name
                if not target.exists():
                    os.link(source, target)
                entry['episode'] = str(target)
                entry['bytes'] = target.stat().st_size
            manifest.append(entry)
    (out / 'manifest.jsonl').write_text(''.join(json.dumps(entry) + '\n' for entry in manifest), encoding='utf-8')
    walls = [e['wall_s'] for e in manifest if e.get('wall_s') is not None]
    made = [e for e in manifest if e.get('episode')]
    wall = time.time() - started
    source = tree / 'SOURCE.json'
    summary = {
        'exit_codes': codes,
        'episodes': len(made),
        'attempts': len(manifest),
        'outcomes': {},
        'wall_s': round(wall, 1),
        'workers': len(workers),
        'episodes_per_hour': round(len(made) / wall * 3600, 2) if wall > 0 else None,
        'attempt_wall_s': (
            {'median': statistics.median(walls), 'min': min(walls), 'max': max(walls)} if walls else None
        ),
        'episode_mb': (
            {
                'median': round(statistics.median(e['bytes'] for e in made) / 1e6, 1),
                'total': round(sum(e['bytes'] for e in made) / 1e6, 1),
            }
            if made
            else None
        ),
        'worker_wall_s': {w['k']: round(ended[w['k']] - started, 1) for w in workers},
        'host': socket.gethostname(),
        'tree': str(tree),
        'source': json.loads(source.read_text(encoding='utf-8')) if source.exists() else None,
        'binaries': {
            name: sha256(HOME / 'bin' / name)
            for name in ('control_harness', 'librobot_collision.so')
            if (HOME / 'bin' / name).exists()
        },
        'egl_device': a.egl_device,
        'nice': a.nice,
        'commands': [w['command'] for w in workers],
    }
    for entry in manifest:
        summary['outcomes'][entry['outcome']] = summary['outcomes'].get(entry['outcome'], 0) + 1
    (out / 'summary.json').write_text(json.dumps(summary, indent=1) + '\n', encoding='utf-8')
    print(
        json.dumps(
            {
                key: summary[key]
                for key in (
                    'exit_codes',
                    'episodes',
                    'attempts',
                    'outcomes',
                    'wall_s',
                    'episodes_per_hour',
                    'attempt_wall_s',
                    'episode_mb',
                )
            }
        )
    )
    return 0 if all(code == 0 for code in codes.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
