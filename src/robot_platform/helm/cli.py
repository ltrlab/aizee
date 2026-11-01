"""Start Helm on this computer. It reads the arms (motors disabled) whenever Minerva is online over the tether."""
import argparse
from pathlib import Path
import sys


def serve(app, listeners, *, handle_signals=True, stop=None):
    """web.run_app for several listeners, each (host, port, TLS context or None): until SIGINT or SIGTERM (or `stop`,
    a threading.Event), then Helm's shutdown and cleanup as run_app runs them."""
    import asyncio
    from aiohttp import web

    async def main():
        runner = web.AppRunner(app, handle_signals=handle_signals)
        await runner.setup()
        try:
            for host, port, tls in listeners:
                await web.TCPSite(runner, host, port, ssl_context=tls).start()
            while not (stop is not None and stop.is_set()):
                await asyncio.sleep(.2)
        finally:
            await runner.cleanup()
    try:
        asyncio.run(main())
    except (web.GracefulExit, KeyboardInterrupt):
        pass


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
    parser.add_argument('--no-live', action='store_true',
                        help='Start without the automatic stopped-motor reader; camera setup remains available')
    parser.add_argument('--sim', action='store_true',
                        help="Teleop's virtual receiver drives a simulated Minerva in a MuJoCo window (loop on this PC)")
    parser.add_argument('--sim-visual-mode', choices=('cad', 'collision_proxy'),
                        help='Task camera appearance: collision_proxy (default) or full CAD; contact physics is unchanged')
    parser.add_argument('--capture-profile', type=Path,
                        help='JSON camera identity/clock configuration for bimanual live task recording')
    parser.add_argument('--episodes', type=Path,
                        help="Replay episodes from this local directory (live-*/episodes/*.hdf5) instead of Minerva's "
                             'Jetson; a live replay with the loop on this PC reads them here too')
    parser.add_argument('--on-robot', action='store_true',
                        help="Helm runs on Minerva's Jetson: it reaches the robot at its own loopback address, and the "
                             'GELLO bridge on the operator PC serves the leader arms (helm.bridge)')
    parser.add_argument('--host', default='127.0.0.1',
                        help='Address to serve the page on (default: this computer only); beyond it needs --access')
    parser.add_argument('--access', type=Path,
                        help='Access file (python -m robot_platform.helm.access set-password FILE): a login, and one '
                             'device in control')
    parser.add_argument('--operator-files', type=Path,
                        help='Read and save the joint limits, servo calibration and leader ranges here instead of '
                             'beside the profiles (Helm on Minerva, whose release must not change: helm.operator_files)')
    parser.add_argument('--mesh-store', type=Path,
                        help="Serve the 3D view's meshes, converted on the build PC, from here instead of converting "
                             'the CAD (Helm on Minerva: python -m robot_platform.ops.helm_robot meshes)')
    parser.add_argument('--tls-cert', type=Path, help='HTTPS certificate (PEM); with --tls-key')
    parser.add_argument('--tls-key', type=Path, help='HTTPS private key (PEM)')
    parser.add_argument('--port', type=int, default=8760)
    parser.add_argument('--loopback-port', type=int,
                        help='Also serve plain HTTP on 127.0.0.1 at this port, for tools on this computer and SSH '
                             'tunnels (Helm on Minerva serves its devices over HTTPS); the login still applies')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('port must be in 1..65535')
    if args.loopback_port is not None and (not 1 <= args.loopback_port <= 65535 or args.loopback_port == args.port):
        parser.error('--loopback-port must be in 1..65535 and differ from --port')
    if args.host not in ('127.0.0.1', 'localhost', '::1') and args.access is None:
        parser.error('Serving beyond this computer needs --access: a login '
                     '(python -m robot_platform.helm.access set-password FILE)')
    if (args.tls_cert is None) != (args.tls_key is None):
        parser.error('--tls-cert and --tls-key go together')
    if args.on_robot and args.sim:
        parser.error('--on-robot drives the robot it runs on; --sim is a simulation on a PC')
    if args.on_robot and args.loop != 'jetson':
        parser.error("--on-robot runs each loop in its own process on Minerva's control CPUs (--loop jetson)")
    if args.sim and not args.teleop:
        parser.error('--sim needs --teleop')
    if args.sim_visual_mode is not None and not args.sim:
        parser.error('--sim-visual-mode needs --sim')
    from aiohttp import web
    from .server import create_app
    from .session import HelmSession
    operator = None
    try:
        if args.on_robot:
            # Like the loop it starts: the release's verified mesh digests and native paths, so the model loads
            # without the CAD files, which stay on the build PC. A source checkout has the CAD.
            from robot_platform.ops.release import MANIFEST, ROOT, activate
            if (ROOT / MANIFEST).is_file():
                release = activate(ROOT)
                print(f"Helm on Minerva: release {release['id']}", flush=True)
        if args.operator_files is not None:
            from robot_platform.ops.ssh import robot_identity
            from .operator_files import operator_files, prepare
            files = operator_files(args.profile, robot_identity(args.profile)[0]['asset_id'], args.teleop)
            operator = {'directory': str(args.operator_files.resolve()), 'notices': prepare(files, args.operator_files)}
            for notice in operator['notices']:
                print(f"Operator files: {notice['message']}", flush=True)
        session = HelmSession(args.profile, runs=args.runs, limits=args.limits, seed=args.seed, on_robot=args.on_robot,
                              operator_files=args.operator_files)
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
        teleop = leaders = tune = bridge = None
        if args.on_robot:
            # The leader arms stay on the operator PC: its GELLO bridge joins Helm here (helm.bridge).
            from .bridge import BridgeHub
            bridge = BridgeHub()
        if args.teleop:
            from .leaders import LeaderWorkspace
            from .teleop import TeleopSession
            leaders = LeaderWorkspace(args.teleop, labels=session.labels, evidence=session.evidence,
                                      open_leaders=bridge.open_leaders if bridge is not None else None,
                                      operator_files=args.operator_files)
            simulation = None
            if args.sim:
                # The simulator's actuator file, beside the servo calibration: the motors it matches take the values
                # sim2real fitted on real teleop.
                from robot_platform.simulation.minerva_sim import actuators_path
                simulation = {'actuators': str(actuators_path(session.profile_path, session.asset_id)),
                              'visual_mode': args.sim_visual_mode or 'collision_proxy'}
            teleop = TeleopSession(session, args.teleop, runs=args.runs, leaders=leaders,
                                   target='pc' if args.sim else 'jetson' if args.on_robot else args.loop,
                                   simulation=simulation, capture_profile=args.capture_profile, episodes=args.episodes,
                                   bridge=bridge)
            if not args.sim:
                from .tune import TuneSession, tune_profile
                tune = TuneSession(session, tune_profile(args.teleop), target=args.loop)
        check = cameras = camera_video = None
        if not args.sim:
            from robot_platform.ops.check import DEFAULT_TELEOP
            from .check import CheckWorkspace
            from .camera_video import CameraVideo
            from .cameras import CameraSettings
            cameras = CameraSettings(session.profile_path, address=lambda: session.address)
            camera_video = CameraVideo(session.profile_path, address=lambda: session.address)
            check = CheckWorkspace(profile=session.profile_path, teleop=args.teleop or DEFAULT_TELEOP,
                                   runs=args.runs, limits=session.limits_path, address=lambda: session.address,
                                   operator_files=args.operator_files)
        calibration = head_alignment = maps = twin = None
        if args.on_robot and teleop is not None:
            # Wrist camera stills during a live session; the robot saves their images under the same folder.
            from .calibration import CameraCalibration
            from .calibration_still import CALIBRATION_ROOT
            calibration = CameraCalibration(session.profile_path, address=lambda: session.address,
                                            teleop=teleop.snapshot, root=CALIBRATION_ROOT,
                                            reference=lambda: session.bindings['reference'])
            # The head camera's alignment on the same tags; its stills go beside the wrists' runs.
            from robot_platform.control.servo_calibration import servo_path
            from robot_platform.neck import head
            from .head_alignment import HeadAlignment
            operator_directory = lambda: servo_path(session.profile_path, session.asset_id,  # noqa: E731
                                                    session.operator_files).parent
            head_files = lambda: head.files(operator_directory(), session.asset_id)  # noqa: E731
            head_alignment = HeadAlignment(session.profile_path, address=lambda: session.address,
                                           teleop=teleop.snapshot, root=f'{CALIBRATION_ROOT}/head',
                                           wrists=CALIBRATION_ROOT, files=head_files)
            # The digital twin's correction from both, for drawing only; its file beside the head's.
            from robot_platform.vision.twin import path as twin_path
            from .twin import TwinCorrection
            twin = TwinCorrection(wrists=CALIBRATION_ROOT, files=head_files, teleop=teleop.snapshot,
                                  path=lambda: twin_path(operator_directory(), session.asset_id),
                                  arms_reference=lambda: session.bindings['reference'],
                                  history=f'{CALIBRATION_ROOT}/twin')
            # Head scene captures for splat maps, and the trained maps the headset draws.
            from .maps import MapLibrary
            maps = MapLibrary(session.profile_path, address=lambda: session.address, teleop=teleop.snapshot,
                              root='~/minerva/helm/maps', files=head_files, correction=twin.camera_turn)
        if tune is not None:
            # Minerva Check's live joint sweep runs through the Tune loop; its outcome goes into the report.
            tune.on_finished = lambda mode, arm, results, fault: (
                check.record_live(arm, results, fault=fault) if mode == 'verify' else None)
        # Replay reads the episodes where they were recorded: on Minerva's Jetson, or a local copy given here.
        from .episodes import EpisodeLibrary, local_transport, robot_transport
        episodes = (EpisodeLibrary(local_transport(args.episodes)) if args.episodes else None if args.sim else
                    EpisodeLibrary(robot_transport(session.profile_path, lambda: session.address)))
        # Minerva's UPS: the battery pill and banner, and Helm's part in a low-battery shutdown (helm.ups).
        from .ups import UpsWatch, robot_ups
        ups = None if args.sim else UpsWatch(robot_ups(session.profile_path), lambda: session.address)
        # Minerva's network service (ops.network): its state, and the access point switch.
        network = None
        if args.on_robot:
            from .network import NetworkView
            network = NetworkView()
        access = None
        if args.access is not None:
            from .access import Access
            access = Access(args.access)
        # The 3D view draws the head model (Minerva with her neck); the guard and the receivers keep the active one.
        from robot_platform.ops.release import HEAD_MODEL, ROOT as RELEASE_ROOT
        from .display_model import DisplayModel
        display = DisplayModel.load(session.model, RELEASE_ROOT / HEAD_MODEL)
        # The head's setup (Limits tab, Head) reads the neck's servos on their board, which only Minerva has.
        head_setup = None
        if args.on_robot and not args.sim:
            from robot_platform.control.servo_calibration import servo_path
            from .head_setup import HeadSetup
            head_setup = HeadSetup(servo_path(session.profile_path, session.asset_id, session.operator_files).parent,
                                   session.asset_id, session.evidence)
        app = create_app(session, link, teleop=teleop, leaders=leaders, check=check, tune=tune,
                         live=not (args.sim or args.no_live), simulation_only=args.sim, cameras=cameras,
                         camera_video=camera_video, episodes=episodes, ups=ups, access=access, bridge=bridge,
                         mesh_store=args.mesh_store, operator_files=operator, calibration=calibration,
                         network=network, display=display, head_setup=head_setup, head_alignment=head_alignment,
                         maps=maps, twin=twin)
        if session.stale_lock_pid:
            print(f'Took over a calibration lock left by exited process {session.stale_lock_pid}.', flush=True)
        mode = ('SIMULATION ONLY; no physical robot connection' if args.sim else
                'robot mode; automatic stopped-motor reader off' if args.no_live else
                'robot mode; reads the arms, motors disabled, whenever Minerva is online')
        tls = None
        if args.tls_cert is not None:
            import ssl
            tls = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            tls.load_cert_chain(str(args.tls_cert), str(args.tls_key))
        where = 'on Minerva' if args.on_robot else 'on this PC'
        print(f"Helm {where}: {'https' if tls else 'http'}://{args.host}:{args.port}/  ({session.asset_id}; {mode})"
              + (f'; http://127.0.0.1:{args.loopback_port}/ on this computer' if args.loopback_port else ''), flush=True)
        if args.loopback_port is None:
            web.run_app(app, host=args.host, port=args.port, print=None, ssl_context=tls)
        else:
            serve(app, [(args.host, args.port, tls), ('127.0.0.1', args.loopback_port, None)])
        return 0
    finally:
        session.close()


if __name__ == '__main__':
    raise SystemExit(main())
