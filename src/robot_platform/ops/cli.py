"""Deploy to and check the robot from the operator PC."""
import argparse
from pathlib import Path
import sys
import tempfile

from .ssh import TETHER


def deploy_main(argv=None):
    parser = argparse.ArgumentParser(
        prog='robot-platform deploy',
        description='Build a verified release on this PC and make it current on the robot. User space only: '
                    'no sudo, services or compiling on the robot; the last releases are kept, older ones pruned.')
    parser.add_argument('--address', default=TETHER, help=f'robot address (default: the tether, {TETHER})')
    parser.add_argument('--keep', type=int, default=3, help='releases to keep on the robot (default 3)')
    parser.add_argument('--build-only', type=Path, metavar='DIR', help='write the release archive to DIR and stop')
    args = parser.parse_args(argv)
    from .release import build
    with tempfile.TemporaryDirectory() as scratch:
        manifest, archive = build(args.build_only or scratch)
        size = archive.stat().st_size / 1e6
        print(f"Built {manifest['id']}: {len(manifest['files'])} files, {size:.1f} MB compressed"
              + (' (uncommitted changes included)' if manifest['source']['dirty'] else ''), flush=True)
        if args.build_only:
            print(f'Archive: {archive}')
            return 0
        from .deploy import deploy
        from .ssh import connect
        client = connect(address=args.address)
        try:
            release_id = deploy(client, manifest, archive, keep=args.keep)
        finally:
            client.close()
    print(f'Minerva now runs release {release_id}.')
    return 0


def check_main(argv=None):
    parser = argparse.ArgumentParser(
        prog='robot-platform check',
        description='Minerva Check: the go/no-go for live operation. Checks this PC\'s calibration, the tether, '
                    'the Jetson\'s release and health, and reads every motor while it stays disabled.')
    parser.add_argument('--address', default=TETHER, help=f'robot address (default: the tether, {TETHER})')
    parser.add_argument('--runs', type=Path, help='evidence directory (default: calibration_runs)')
    parser.add_argument('--no-motors', action='store_true', help='skip the stopped motor scan (verdict: incomplete)')
    parser.add_argument('--link-seconds', type=float, default=5., help='leader-link test length (default 5 s)')
    args = parser.parse_args(argv)
    if not 1 <= args.link_seconds <= 60:
        parser.error('--link-seconds must be within 1..60')
    from .check import DEFAULT_RUNS, run_check
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')  # A redirected Windows console cannot encode every symbol.
    shown = set()

    def publish(report):
        for stage in report['stages']:
            for entry in stage['checks']:
                if entry['status'] in ('pending', 'running') or entry['id'] in shown:
                    continue
                if not shown & {c['id'] for c in stage['checks']}:
                    print(f"\n{stage['title']}", flush=True)
                shown.add(entry['id'])
                print(f"  {entry['status'].upper():<7} {entry['title']}: {entry['detail']}", flush=True)

    print('Minerva Check: motors stay disabled.', flush=True)
    report = run_check(address=args.address, runs=args.runs or DEFAULT_RUNS, motors=not args.no_motors,
                       link_seconds=args.link_seconds, publish=publish)
    print(f"\n{report['summary']}\nEvidence: {report['evidence']}", flush=True)
    return 0 if report['verdict'] == 'go' else 1
