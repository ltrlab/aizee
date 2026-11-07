"""Generate demonstrations with several `evaluation generate` workers at once, then gather them in one folder.

    .venv-dev/Scripts/python.exe scripts/sim/generate_parallel.py --out <dir> --episodes 120 --workers 4 -- \\
        --grid 6x5 --jitter 1 --yaw 0.785398

Worker k runs `python -m robot_platform.evaluation generate` with its share of the episodes in <out>/w<k>, grid seeds
from SEED + 100 k and plant seed k + 1; each runs its own receiver and scene. Options after `--` go to every worker
(the workcell defaults to the measured bench). The published episodes are then hard-linked into <out>/episodes,
beside one manifest.jsonl and a summary.json.

The poses go row by row through the grid, so give each worker a whole pass: episodes per worker = grid cells (120
episodes over 4 workers: a 6x5 grid). Four workers made 120 episodes from 121 attempts in about 2.5 h on the 16 GB
development laptop and used most of its memory; a fifth process (training, say) beside them ran it out.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
WORKCELL = 'config/devices/simulation/minerva-production-01-workcell.json'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True, help='output folder')
    parser.add_argument('--episodes', type=int, required=True, help='successful episodes wanted in all')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seed', type=int, default=1000, help='worker k draws grid seeds from SEED + 100 k')
    parser.add_argument('generate', nargs=argparse.REMAINDER, help='after --: options for every generate worker')
    args = parser.parse_args(argv)
    extra = args.generate[1:] if args.generate[:1] == ['--'] else args.generate
    if '--workcell' not in extra:
        extra = ['--workcell', WORKCELL, *extra]
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    shares = [args.episodes // args.workers + (k < args.episodes % args.workers) for k in range(args.workers)]
    started = time.time()
    workers = []
    for k, share in enumerate(shares):
        if not share:
            continue
        command = [
            sys.executable,
            '-m',
            'robot_platform.evaluation',
            'generate',
            '--episodes',
            str(share),
            '--out',
            str(out / f'w{k}'),
            '--seed',
            str(args.seed + 100 * k),
            '--plant-seed',
            str(k + 1),
            *extra,
        ]
        log = open(out / f'w{k}.log', 'w', encoding='utf-8')
        workers.append((k, command, log, subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)))
    codes = {}
    for k, _, log, process in workers:
        codes[k] = process.wait()
        log.close()
    episodes = out / 'episodes'
    episodes.mkdir(exist_ok=True)
    manifest = []
    for k, *_ in workers:
        path = out / f'w{k}' / 'manifest.jsonl'
        for line in path.read_text(encoding='utf-8').splitlines() if path.exists() else []:
            entry = dict(json.loads(line), worker=k)
            if entry.get('episode'):
                source = Path(entry['episode'])
                target = episodes / source.name
                if not target.exists():
                    os.link(source, target)
                entry['episode'] = str(target)
            manifest.append(entry)
    (out / 'manifest.jsonl').write_text(''.join(json.dumps(entry) + '\n' for entry in manifest), encoding='utf-8')
    outcomes = {}
    for entry in manifest:
        outcomes[entry['outcome']] = outcomes.get(entry['outcome'], 0) + 1
    summary = {
        'exit_codes': codes,
        'episodes': sum(1 for entry in manifest if entry.get('episode')),
        'attempts': len(manifest),
        'outcomes': outcomes,
        'wall_s': round(time.time() - started, 1),
        'commands': [command for _, command, *_ in workers],
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=1) + '\n', encoding='utf-8')
    print(json.dumps({key: summary[key] for key in ('exit_codes', 'episodes', 'attempts', 'outcomes', 'wall_s')}))
    return 0 if all(code == 0 for code in codes.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
