"""Deploy to and check the robot from the operator PC."""
import argparse
from pathlib import Path
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
