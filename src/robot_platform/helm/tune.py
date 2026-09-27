"""Helm's servo sweep: the attended live sweep that steps each joint's settings and measures them.

TuneCore is the loop body. It runs on Minerva's Jetson through helm.edge (or in a
local process on the virtual receiver, for previews and tests), like
TeleopCore. It starts the live receiver for one arm with the same gates as live
teleop, with its stiffness, damping and speed ceilings raised to what the sweep
may try (control.servo_sweep.sweep_profile). Then it hands every cycle to the
sweep engine: the receiver's rows since the last cycle in, the next command out.
Keys: X finishes (the joint under test returns to the reference, then the arm
is disabled); H disables at once.

TuneSession is Helm's handle: it starts and stops the loop, caches its views and
results, and saves the joints the operator confirms as the servo calibration,
from which the receiver then takes its gains, speeds and tracking bounds.
"""
from copy import deepcopy
import json
import multiprocessing
from pathlib import Path
import threading
import time

from robot_platform.control.guarded_config import arm_subset, derive_receiver
from robot_platform.control.servo_calibration import load_servo, save_servo, servo_path  # noqa: F401 (re-exported)
from robot_platform.control.scheduling import prefer_responsive
from .limits import utc_now
from .teleop import AdvisoryCompiler, load_profile

PERIOD_S = .04
RESULTS_EVERY_S = 1.


class TuneCore:
    """The sweep's loop body. Openers are injectable for tests."""
    period_s = PERIOD_S

    def __init__(self, bundle, *, open_backend=None, open_harness=None, open_receiver=None, clock=time.perf_counter,
                 evidence_root=None):
        self.bundle, self.clock = deepcopy(bundle), clock
        self.evidence_root = Path(evidence_root) if evidence_root else Path.home() / 'minerva' / 'evidence'
        self.profile = load_profile(bundle['teleop_profile'])
        receiver = bundle.get('receiver') or {'mode': 'virtual'}
        self.live_arm = receiver.get('arm') if receiver.get('mode') == 'live' else None
        self.arm = receiver.get('arm') or bundle.get('arm')
        if self.arm not in ('left', 'right'):
            raise ValueError('A servo sweep runs on the left or right arm')
        if receiver.get('mode') == 'live' and 'live' not in self.profile:
            raise ValueError("A live servo sweep needs the teleop profile's live block")
        self.open_backend = open_backend or self._open_backend
        self.open_harness = open_harness or self._open_harness
        self.open_receiver = open_receiver or self._open_receiver
        self.phase, self.stage = 'starting', 'starting'
        self.message = ('Checking the arm, then starting the live receiver…' if self.live_arm
                        else 'Loading the collision model and virtual receiver…')
        self.fault = None
        self.engine = self.controller = self.receiver = self.compiler = self.derived = None
        self.pending = []  # Receiver results since the engine last saw them.
        self.cycles = []
        self._closers = []
        self._stage_since = None
        self._results_at = 0.
        self.scheduling = {}
        self.evidence = None
        self.sink = None

    # -- resources -------------------------------------------------------------------------

    def _open_backend(self, model):
        from robot_platform.simulation.native_collision import NativeCollisionBackend
        from robot_platform.simulation.runtime_model import RuntimeCollisionModel
        return NativeCollisionBackend(RuntimeCollisionModel(model.profile), geometry_mode='keepout')

    def _open_harness(self, config, initial):
        from robot_platform.simulation.control_harness import NativeControlHarness
        harness = NativeControlHarness(config, initial)
        self.scheduling['receiver'] = prefer_responsive(harness.process.pid)
        harness.step(0)
        harness.timeout_s = .035
        return harness

    def _open_receiver(self, config, device, reference):
        from robot_platform.control.live_receiver import LiveReceiver, service_config
        from robot_platform.ops.release import ROOT
        live = self.profile['live']
        buses = {motor['bus'] for motor in reference['motors']}
        service = service_config(config, device, reference, self.bundle['receiver']['motor_models'],
                                 bus=buses.pop(), tolerance_rad=live['reference_tolerance_rad'],
                                 period_us=live['period_us'], maximum_lateness_us=live['maximum_lateness_us'])
        return LiveReceiver(service, device=device, reference=reference, binary=ROOT / 'bin' / 'guarded_controller',
                            directory=self.evidence, labels=self.labels).start()

    # -- build -------------------------------------------------------------------------------

    def build(self):
        from robot_platform.control.guarded import GuardedControllerClient
        from robot_platform.control.joint_calibration import load_plan, validate_preview_transmissions
        from robot_platform.control.servo_sweep import PathChecker, SweepEngine, sweep_profile, sweep_settings
        plan, _paths, device, reference, model, pose, bindings = load_plan(self.bundle['calibration_profile'])
        if bindings != self.bundle['bindings']:
            raise ValueError('Calibration files changed since Helm started; restart Helm')
        transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), plan['steps'],
                                                       reference, model)
        self.labels = {s['motor']: s['label'] for s in plan['steps']}
        self.labels.update({t['motor']: t['motor'].replace('_', ' ').capitalize() for t in transmissions})
        # One arm: the other is not in this receiver and is never enabled.
        reference, steps, transmissions = arm_subset(reference, plan['steps'], transmissions, self.arm)
        motors = [motor['name'] for motor in reference['motors']]
        sweep = [motor for motor in (self.bundle.get('joints') or motors) if motor in motors]
        if not sweep:
            raise ValueError('Choose at least one joint of the arm to sweep')
        backend = self.open_backend(model)
        self._closers.append(backend.close)
        # The profile's gains as derived (and any saved servo calibration), with the sweep's ceilings raised.
        base = derive_receiver(reference=reference, steps=steps, directions=self.bundle['directions'],
                               transmissions=transmissions, model=model, limits=self.bundle['limits'],
                               profile=self.profile, backend=backend, servo=self.bundle.get('servo'))
        gains = {joint['name']: (joint['hold_kp'], joint['hold_kd']) for joint in base['config']['joints']}
        derived = derive_receiver(reference=reference, steps=steps, directions=self.bundle['directions'],
                                  transmissions=transmissions, model=model, limits=self.bundle['limits'],
                                  profile=sweep_profile(self.profile, motors, gains), backend=backend)
        self.derived = derived
        stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        self.evidence = (self.evidence_root / f'sweep-{stamp}') if self.live_arm else None
        if self.evidence is not None:
            self.evidence.mkdir(parents=True, exist_ok=True)
        names = tuple(joint['name'] for joint in derived['config']['joints'])
        if self.live_arm:
            self.receiver = self.open_receiver(derived['config'], device, reference)
            self._closers.append(self.receiver.close)
            self.receiver.transport.on_result = self.pending.append
            exchange = self.receiver.exchange
            latest = lambda: self.receiver.last  # noqa: E731
        else:
            harness = self.open_harness(derived['config'], derived['initial'])
            self._closers.append(harness.close)
            epoch = self.clock()

            def exchange(request):
                now_ms = max(harness.now_ms, int((self.clock() - epoch) * 1000))
                result = harness.step(now_ms, request)
                self.pending.append(result)
                return result['reply']
            self.receiver = harness
            latest = lambda: harness.last  # noqa: E731
        self.latest = latest
        self.controller = GuardedControllerClient(exchange, group=derived['config']['group'],
                                                  identity=derived['config']['identity'], joint_names=names)
        fixed = {name: pose[name] for name in pose if name not in derived['joint_map']}
        self.compiler = AdvisoryCompiler(backend, derived['config'], derived['joint_map'], fixed=fixed,
                                         cad_limits=derived['cad_limits'],
                                         compute_budget_s=self.profile['receiver']['compute_budget_s'])
        tolerance = {row['name']: row['tolerance'] for row in derived['lifecycle']}
        settings = {name: dict(sweep_settings(self.profile, name), tolerance=tolerance[name]) for name in names}
        home = {motor['name']: motor['raw_reference_rad'] for motor in reference['motors']}
        checker = PathChecker(backend, derived['joint_map'], home=home, fixed=fixed, cad_limits=derived['cad_limits'],
                              clearance_m=settings[sweep[0]]['clearance_m'],
                              step_rad=settings[sweep[0]]['path_step_rad'])
        policy = derived['config']['collision']
        window_s = (policy['max_verdict_ms'] + derived['config']['feedback_timeout_ms'] + policy['reaction_ms']) / 1000
        self.engine = SweepEngine(derived['config']['joints'], home=home, gains=gains, settings=settings, sweep=sweep,
                                  checker=checker, window_s=window_s, verdict_s=policy['max_verdict_ms'] / 1000,
                                  labels=self.labels)
        if self.evidence is not None:
            self.sink = (self.evidence / 'samples.f32').open('ab')
            self.engine.recording.sink = self.sink
            (self.evidence / 'columns.json').write_text(json.dumps(
                {'dtype': 'float32', 'columns': ['t_ms'] + [f'{name}.{field}' for name in names
                                                           for field in ('position', 'velocity', 'effort', 'command')]},
                indent=1) + '\n', encoding='utf-8')
        self.phase, self.stage = 'running', 'arming'
        self._stage_since = self.clock()
        self.message = (f'SERVO SWEEP · LIVE {self.arm.upper()} ARM · enabling, then the sweep moves the arm. '
                        'Keep the foot switch in reach.' if self.live_arm else
                        'SERVO SWEEP · VIRTUAL RECEIVER · no motors move.')

    # -- the cycle -----------------------------------------------------------------------------

    def _feed(self):
        """Hand the engine every receiver row since the last cycle."""
        from robot_platform.control.servo_sweep import result_rows
        names = self.engine.names
        for result in self.pending:
            rows, dropped = result_rows(result, names)
            self.engine.observe(rows, dropped=dropped)
        self.pending.clear()
        last = self.latest() or {}
        temperatures = {name: sample.get('temperature') for name, sample in zip(names, last.get('feedback') or [])}
        verdict = getattr(self.compiler, 'last_verdict', None) or {}
        self.engine.observe([], temperatures=temperatures, collision_clear=verdict.get('clear', True))

    def _move(self, command):
        self.controller.renew()
        offer_started = self.clock()
        offered = self.controller.collision_offer()
        feedback = (self.latest() or {}).get('feedback') or []
        joints = [{'position': target, 'velocity': 0., 'effort': 0., 'kp': kp, 'kd': kd}
                  for target, (kp, kd) in zip(command['targets'], command['gains'])]
        envelope = self.compiler.compile(offered, feedback, joints, started_at=offer_started)
        self.controller.move(joints, collision=envelope)

    def tick(self, key, started):
        if self.fault or self.stage in ('finished', 'disabled'):
            return
        record = {'started_at': started}
        try:
            if key == 'H':
                self._disable('Disabled at your request: the arm is limp where it is.')
                return
            if key == 'X' and self.engine is not None:
                self.engine.stop()
                self.message = 'Finishing: the joint under test returns to the reference, then the arm is disabled.'
            self._feed()
            if self.stage == 'arming':
                self._arm()
            elif self.stage == 'sweeping':
                command = self.engine.command(started)
                if command is None:
                    self._disable('Sweep finished: the arm is back at the reference and disabled. Review the results, '
                                  'then save the joints you accept.' if self.engine.task == 'finished' else
                                  'Sweep stopped: the arm is back at the reference and disabled.')
                    self.stage = 'finished'
                else:
                    self._move(command)
                    progress = self.engine.progress()
                    self.message = f"{progress['task']} ({progress['joints_done']}/{progress['joints']} joints done)"
        except Exception as exc:
            status = getattr(exc, 'status', None) or (self.controller.status if self.controller else None) or {}
            reason = status.get('reason')
            self.fail(f"Receiver {status.get('state')}: {reason} · {exc}" if reason not in (None, 'ok') else str(exc))
        finally:
            record['completed_at'] = self.clock()
            self.cycles.append(record)
            if len(self.cycles) > 500:
                del self.cycles[:100]

    def _arm(self):
        """Acquire, enable at zero torque, then hold at the measured pose; the sweep starts there."""
        elapsed = self.clock() - self._stage_since
        if not self.controller.owned:
            status = self.controller.acquire()
            if status['state'] != 'disarmed':
                raise RuntimeError('The receiver must be disarmed before the sweep enables it')
            self.controller.idle()
            self._stage_since = self.clock()
            self.message = 'Motors on at zero torque; holding in a moment…'
            return
        self.controller.renew()
        if elapsed < .5:
            self.controller.idle()  # A command is only good for its lease: repeat it every cycle.
            return
        offer_started = self.clock()
        offered = self.controller.collision_offer()
        feedback = (self.latest() or {}).get('feedback') or []
        self.controller.arm(collision=self.compiler.compile(offered, feedback, None, started_at=offer_started))
        self.engine.begin({name: sample['position'] for name, sample in zip(self.engine.names, feedback)})
        self.stage = 'sweeping'
        self.message = 'Holding. The sweep moves the arm joint by joint.'

    def _disable(self, message):
        if self.controller is not None and self.controller.owned:
            try:
                self.controller.disable()
            except Exception as exc:
                message += f' (disable reported: {exc})'
        self.stage = 'disabled' if self.stage != 'finished' else self.stage
        self.message = message

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason
        self.message = f'Sweep stopped: {reason}. The receiver disables the arm.'

    def close(self):
        if self.controller is not None and self.controller.owned:
            try:
                self.controller.disable()
            except Exception:
                pass
        failures = []
        for close in reversed(self._closers):
            try:
                close()
            except Exception as exc:
                failures.append(str(exc) or type(exc).__name__)
        self._closers = []
        if self.sink is not None:
            self.sink.close()
            self.sink = None
        if self.engine is not None and self.evidence is not None:
            (self.evidence / 'results.json').write_text(json.dumps(
                {'arm': self.arm, 'finished_at': utc_now(), 'task': self.engine.task, 'events': self.engine.events,
                 'joints': self.engine.summary()}, indent=1, default=float) + '\n', encoding='utf-8')
        if self.phase != 'fault':
            self.phase = 'stopped'
            if self.stage not in ('finished',):
                self.message = 'Sweep stopped. The receiver disabled the arm and its CAN link is down.'
        if failures:
            self.message += ' Closing reported: ' + '; '.join(failures)

    # -- views ---------------------------------------------------------------------------------

    def view(self, *, results=None):
        now = self.clock()
        if results is None:
            results = self.stage == 'finished' or self.phase != 'running' or now - self._results_at >= RESULTS_EVERY_S
        last = self.latest() if self.engine is not None else None
        durations = sorted((c['completed_at'] - c['started_at']) * 1000 for c in self.cycles[-100:]
                           if c.get('completed_at') is not None)
        view = {'phase': self.phase, 'stage': self.stage, 'message': self.message, 'fault': self.fault,
                'arm': self.arm, 'live': self.live_arm is not None,
                'receiver': ((last or {}).get('status') or {}).get('state'),
                'motors_enabled': (last or {}).get('hardware_enabled'),
                'progress': self.engine.progress() if self.engine else None,
                'cycle_ms': {'p50': durations[len(durations) // 2], 'p99': durations[int(.99 * (len(durations) - 1))]}
                if durations else None,
                'evidence': str(self.evidence) if self.evidence else None}
        if results and self.engine is not None:
            view['results'] = self.engine.summary()
            self._results_at = now
        return view


def run_tune_loop(commands, views, core):
    """Like helm.teleop.run_loop: keys in, views out. The loop ends when the sweep finishes, is
    disabled or faults; the last view carries the results."""
    clock = core.clock
    try:
        views.send(('view', core.view(results=False), None))
        core.build()
        views.send(('view', core.view(), None))
        while True:
            started = clock()
            key = None
            while True:
                try:
                    if not commands.poll():
                        break
                    message = commands.recv()
                except (EOFError, OSError):
                    return
                if message[0] == 'stop':
                    return
                if message[0] == 'key':
                    key = message[1] if key is None or message[1] == 'H' else key
            core.tick(key, started)
            if core.fault or core.stage in ('finished', 'disabled'):
                return  # Nothing more to do: close the receiver and CAN link now; the last view has the results.
            try:
                views.send(('view', core.view(), None))
            except (OSError, EOFError, ValueError):
                return
            remaining = core.period_s - (clock() - started)
            if remaining > 0:
                time.sleep(remaining)
    except Exception as exc:
        core.fail(str(exc) or type(exc).__name__)
    finally:
        core.close()
        try:
            views.send(('view', core.view(results=True), None))
        except (OSError, EOFError, ValueError):
            pass
        views.close()
        commands.close()


def _process_main(commands, views, bundle):
    core = TuneCore(bundle)
    core.scheduling['loop'] = prefer_responsive()
    run_tune_loop(commands, views, core)


class TuneSession:
    """Helm's handle on a sweep: start, keys, stop, views, results and saving."""

    TARGETS = ('jetson', 'pc')

    def __init__(self, helm, profile_path, *, target='jetson', core_factory=None, edge_factory=None):
        self.helm = helm
        self.profile_path = Path(profile_path).resolve()
        self.profile = load_profile(self.profile_path)
        self.target, self.core_factory, self.edge_factory = target, core_factory, edge_factory
        self.receiver = {'mode': 'live', 'arm': 'right'}
        self.joints = None
        self.path = servo_path(helm.profile_path, helm.asset_id)
        self._lock = threading.Lock()
        self._conn = self._views = self._worker = self._reader = None
        self._view = {'phase': 'stopped', 'stage': None, 'message': 'The servo sweep is not running.',
                      'fault': None, 'progress': None, 'results': None}
        self.results = None
        self.saved = self._load()
        self.saved_error = None

    def _load(self):
        try:
            return load_servo(self.path)
        except (OSError, ValueError) as exc:
            self.saved_error = str(exc)
            return None

    @property
    def active(self):
        worker = self._worker
        return worker is not None and worker.is_alive()

    def configure(self, *, arm, joints=None, mode='live', target=None):
        if arm not in ('left', 'right') or mode not in ('live', 'virtual'):
            raise ValueError('Sweep the left or right arm, live or on the virtual receiver')
        with self._lock:
            if self.active:
                raise ValueError('Stop the sweep before changing it')
            if target is not None:
                if target not in self.TARGETS:
                    raise ValueError('The loop runs on jetson or pc')
                self.target = target
            if mode == 'live' and self.target != 'jetson':
                raise ValueError("A live sweep runs its loop on Minerva's Jetson")
            self.receiver = {'mode': mode, 'arm': arm}
            self.joints = list(joints) if joints else None

    def _bundle(self):
        helm = self.helm
        return {'kind': 'tune', 'calibration_profile': str(helm.profile_path), 'bindings': deepcopy(helm.bindings),
                'directions': deepcopy(helm.mapping), 'limits': deepcopy(helm.saved_limits),
                'teleop_profile': str(self.profile_path), 'asset_id': helm.asset_id,
                'receiver': dict(self.receiver), 'arm': self.receiver['arm'], 'joints': self.joints,
                'servo': deepcopy(self.saved)}

    def start(self, *, confirm=False):
        with self._lock:
            if self.active:
                raise ValueError('The sweep is already running')
            live = self.receiver['mode'] == 'live'
            if live and confirm is not True:
                raise ValueError(f"Confirm the sweep: the {self.receiver['arm']} arm's motors move on their own, "
                                 'the foot switch is in reach and the arm hangs in its reference pose')
            if live and getattr(self.helm, 'connection', 'tether') != 'tether':
                raise ValueError('A live sweep runs over the wired tether: choose Tether at the top first')
            if self.helm.saved_limits is None:
                raise ValueError('Save joint limits for every motor in the Limits workspace first')
            bundle = self._bundle()
            local = self.target == 'pc' or self.core_factory is not None
            if local:
                context = multiprocessing.get_context('spawn')
                command_reader, command_writer = context.Pipe(duplex=False)
                view_reader, view_writer = context.Pipe(duplex=False)
                if self.core_factory is None:
                    worker = context.Process(target=_process_main, args=(command_reader, view_writer, bundle),
                                             daemon=True, name='helm-tune')
                else:
                    worker = threading.Thread(target=run_tune_loop, daemon=True, name='helm-tune',
                                              args=(command_reader, view_writer, self.core_factory(bundle)))
            else:
                from robot_platform.control.commissioning import motor_models
                from robot_platform.ops.release import ROOT
                from .edge import EdgeWorker
                for key in ('calibration_profile', 'teleop_profile'):
                    path = Path(bundle[key]).resolve()
                    if not path.is_relative_to(ROOT):
                        raise ValueError(f'{path.name} is not part of the robot release')
                    bundle[key] = path.relative_to(ROOT).as_posix()
                if live:
                    bundle['receiver'] = dict(bundle['receiver'], motor_models=motor_models(self.helm.device, ROOT))
                worker = (self.edge_factory or EdgeWorker)(bundle, None, base_view=dict(self._view),
                                                           real_leaders=False, name='The servo sweep',
                                                           address=getattr(self.helm, 'address', None))
                command_writer, view_reader = worker.commands, worker.views
            self._view = dict(self._view, phase='starting', fault=None, results=None,
                              message=(f"Starting the sweep on the {self.receiver['arm']} arm: checking it is still and "
                                       'in its reference pose, then starting the receiver…'))
            self._conn, self._views, self._worker = command_writer, view_reader, worker
            worker.start()
            if local and self.core_factory is None:
                command_reader.close()
                view_writer.close()
            self._reader = threading.Thread(target=self._read, args=(view_reader,), daemon=True,
                                            name='helm-tune-reader')
            self._reader.start()

    def _read(self, conn):
        try:
            while True:
                message = conn.recv()
                if message[0] != 'view':
                    continue
                with self._lock:
                    view = message[1]
                    if view.get('results') is not None:
                        self.results = view['results']
                    self._view = view
        except (EOFError, OSError):
            with self._lock:
                if self._view.get('phase') in ('starting', 'running'):
                    self._view = dict(self._view, phase='fault', fault='The sweep loop ended unexpectedly',
                                      message='Sweep stopped: the loop process ended unexpectedly.')

    def key(self, key):
        if key not in ('X', 'H'):
            raise ValueError('Use X to finish or H to disable now')
        with self._lock:
            conn = self._conn
        if conn is None or not self.active:
            raise ValueError('Start the sweep first')
        try:
            conn.send(('key', key))
        except (OSError, ValueError) as exc:
            raise ValueError(f'The sweep is not reachable: {exc}') from exc

    def stop(self, timeout=15.):
        with self._lock:
            conn, views, worker, reader = self._conn, self._views, self._worker, self._reader
        if worker is None:
            return
        try:
            conn.send(('stop',))
        except (OSError, ValueError):
            pass
        worker.join(timeout)
        if worker.is_alive() and hasattr(worker, 'terminate'):
            worker.terminate()
            worker.join(5)
        if reader is not None:
            reader.join(5)
        with self._lock:
            for end in (conn, views):
                if end is not None:
                    try:
                        end.close()
                    except OSError:
                        pass
            self._conn = self._views = self._worker = self._reader = None
            if self._view.get('phase') in ('starting', 'running'):
                self._view = dict(self._view, phase='stopped', message='Sweep stopped.')

    def save(self, motors):
        """Save the confirmed joints' recommended settings as the servo calibration."""
        with self._lock:
            if self.active:
                raise ValueError('Stop the sweep before saving')
            results = deepcopy(self.results)
        if not results:
            raise ValueError('No sweep results to save')
        if not motors:
            raise ValueError('Choose the joints to save')
        document = save_servo(self.path, asset_id=self.helm.asset_id, reference=self.helm.reference,
                              results=results, motors=motors, session=self._view.get('evidence') or utc_now(),
                              existing=self.saved)
        with self._lock:
            self.saved, self.saved_error = document, None
        return document

    def snapshot(self):
        with self._lock:
            view = deepcopy(self._view)
            view.pop('results', None)
            return dict(view, receiver_choice=dict(self.receiver), joints_choice=self.joints, target=self.target,
                        results=deepcopy(self.results), saved=deepcopy(self.saved), saved_error=self.saved_error,
                        path=str(self.path))
