"""Start Helm on this computer. The robot stays untouched until Connect is pressed."""
import argparse
from pathlib import Path
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(prog='robot-platform helm', description=__doc__)
    parser.add_argument('--profile', required=True, type=Path, help='Device calibration profile (device, reference, model, SSH)')
    parser.add_argument('--runs', type=Path, default=Path('calibration_runs'), help='Local session and evidence directory')
    parser.add_argument('--limits', type=Path, help='Persistent joint-limit file (default: next to the profile)')
    parser.add_argument('--seed', type=Path, help='Import compatible earlier direction evidence into a new session')
    parser.add_argument('--teleop', type=Path,
                        help='Guarded teleop profile (virtual receiver) for the Leaders and Teleop workspaces')
    parser.add_argument('--port', type=int, default=8760)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('port must be in 1..65535')
    from aiohttp import web
    from robot_platform.control.joint_calibration import SshCapture
    from .link import LiveLink
    from .server import create_app
    from .session import HelmSession
    try:
        session = HelmSession(args.profile, runs=args.runs, limits=args.limits, seed=args.seed)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f'Helm could not start: {exc}', file=sys.stderr)
        return 1
    try:
        link = LiveLink(session, lambda: SshCapture(session.plan['connection'], session.device,
                                                    session.reference, session.profile_path.parent))
        teleop = leaders = None
        if args.teleop:
            from .leaders import LeaderWorkspace
            from .teleop import TeleopSession
            leaders = LeaderWorkspace(args.teleop, labels=session.labels, evidence=session.evidence)
            teleop = TeleopSession(session, args.teleop, runs=args.runs, leaders=leaders)
        app = create_app(session, link, teleop=teleop, leaders=leaders)
        if session.stale_lock_pid:
            print(f'Took over a calibration lock left by exited process {session.stale_lock_pid}.', flush=True)
        print(f'Helm: http://127.0.0.1:{args.port}/  ({session.asset_id}; not connected until you press Connect)',
              flush=True)
        web.run_app(app, host='127.0.0.1', port=args.port, print=None)
        return 0
    finally:
        session.close()


if __name__ == '__main__':
    raise SystemExit(main())
