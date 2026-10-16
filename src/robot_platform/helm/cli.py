"""Start Helm on this computer. It reads the arms (motors disabled) whenever Minerva is online over the tether."""
import argparse
from pathlib import Path
import sys


class SimulationLink:
    """No physical connection can be created by a simulation-only Helm process."""
    running = False

    def connect(self):
        raise ValueError('Simulation mode cannot connect to the physical robot')

    def disconnect(self):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(prog='robot-platform helm', description=__doc__)
    parser.add_argument('--profile', required=True, type=Path, help='Device calibration profile (device, reference, model, SSH)')
    parser.add_argument('--runs', type=Path, default=Path('calibration_runs'), help='Local session and evidence directory')
    parser.add_argument('--limits', type=Path, help='Persistent joint-limit file (default: next to the profile)')
    parser.add_argument('--seed', type=Path, help='Import compatible earlier direction evidence into a new session')
    parser.add_argument('--teleop', type=Path,
                        help='Guarded teleop profile (virtual receiver) for the Leaders and Teleop workspaces')
    parser.add_argument('--loop', choices=('jetson', 'pc'), default='jetson',
                        help="where the teleop loop runs at first: Minerva's Jetson (default) or this PC")
    parser.add_argument('--sim', action='store_true',
                        help="Teleop's virtual receiver drives a simulated Minerva in a MuJoCo window (loop on this PC)")
    parser.add_argument('--sim-visual-mode', choices=('cad', 'collision_proxy'),
                        help='Task camera appearance: collision_proxy (default) or full CAD; contact physics is unchanged')
    parser.add_argument('--capture-profile', type=Path,
                        help='JSON camera identity/clock configuration for bimanual live task recording')
    parser.add_argument('--port', type=int, default=8760)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('port must be in 1..65535')
    if args.sim and not args.teleop:
        parser.error('--sim needs --teleop')
    if args.sim_visual_mode is not None and not args.sim:
        parser.error('--sim-visual-mode needs --sim')
    from aiohttp import web
    from .server import create_app
    from .session import HelmSession
    try:
        session = HelmSession(args.profile, runs=args.runs, limits=args.limits, seed=args.seed)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f'Helm could not start: {exc}', file=sys.stderr)
        return 1
    try:
        # Simulation has no SSH factory, stopped reader, check, sweep or drive-zero workspace.
        # Leader USB remains explicit: Connect leaders / Start session opens it, never startup.
        if args.sim:
            link = SimulationLink()
        else:
            from robot_platform.control.joint_calibration import SshCapture
            from .link import LiveLink
            link = LiveLink(session, lambda: SshCapture(session.plan['connection'], dict(session.device,
                                                                                         address=session.address),
                                                        session.reference, session.profile_path.parent))
        teleop = leaders = tune = None
        if args.teleop:
            from .leaders import LeaderWorkspace
            from .teleop import TeleopSession
            leaders = LeaderWorkspace(args.teleop, labels=session.labels, evidence=session.evidence)
            simulation = None
            if args.sim:
                # The simulator's actuator file, beside the servo calibration: the motors it matches take the values
                # sim2real fitted on real teleop.
                from robot_platform.simulation.minerva_sim import actuators_path
                simulation = {'actuators': str(actuators_path(session.profile_path, session.asset_id)),
                              'visual_mode': args.sim_visual_mode or 'collision_proxy'}
            teleop = TeleopSession(session, args.teleop, runs=args.runs, leaders=leaders,
                                   target='pc' if args.sim else args.loop, simulation=simulation,
                                   capture_profile=args.capture_profile)
            if not args.sim:
                from .tune import TuneSession
                tune = TuneSession(session, args.teleop, target=args.loop)
        check = None
        if not args.sim:
            from robot_platform.ops.check import DEFAULT_TELEOP
            from .check import CheckWorkspace
            check = CheckWorkspace(profile=session.profile_path, teleop=args.teleop or DEFAULT_TELEOP,
                                   runs=args.runs, limits=session.limits_path, address=lambda: session.address)
        if tune is not None:
            # Minerva Check's live joint sweep runs through the Tune loop; its outcome goes into the report.
            tune.on_finished = lambda mode, arm, results, fault: (
                check.record_live(arm, results, fault=fault) if mode == 'verify' else None)
        app = create_app(session, link, teleop=teleop, leaders=leaders, check=check, tune=tune,
                         live=not args.sim, simulation_only=args.sim)
        if session.stale_lock_pid:
            print(f'Took over a calibration lock left by exited process {session.stale_lock_pid}.', flush=True)
        mode = ('SIMULATION ONLY; no physical robot connection' if args.sim else
                'robot mode; reads the arms, motors disabled, whenever Minerva is online')
        print(f'Helm: http://127.0.0.1:{args.port}/  ({session.asset_id}; {mode})', flush=True)
        web.run_app(app, host='127.0.0.1', port=args.port, print=None)
        return 0
    finally:
        session.close()


if __name__ == '__main__':
    raise SystemExit(main())
