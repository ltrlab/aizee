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
import gc
import json
import multiprocessing
from pathlib import Path
import threading
import time

from robot_platform.control.guarded_config import arm_subset, derive_receiver, fit_motor_ranges
from robot_platform.control.servo_calibration import load_servo, save_servo, servo_path  # noqa: F401 (re-exported)
from robot_platform.control.scheduling import prefer_responsive
from .limits import utc_now
from robot_platform.control.guarded import GuardRejected
from .teleop import HELD, LOWERING_ATTEMPTS, WAIT_TO_REARM, AdvisoryCompiler, load_profile, loop_period_s

PERIOD_S = .04
RESULTS_EVERY_S = 1.


class TuneCore:
    """The sweep's loop body. Openers are injectable for tests."""

    def __init__(self, bundle, *, open_backend=None, open_harness=None, open_receiver=None, clock=time.perf_counter,
                 evidence_root=None):
        self.bundle, self.clock = deepcopy(bundle), clock
        self.evidence_root = Path(evidence_root) if evidence_root else Path.home() / 'minerva' / 'evidence'
        self.profile = load_profile(bundle['teleop_profile'])
        self.period_s = loop_period_s(self.profile)  # As teleop's: Minerva Check verifies at teleop's rate.
        receiver = bundle.get('receiver') or {'mode': 'virtual'}
        # 'verify': Minerva Check's live joint sweep. 'return': the arm from a pose the operator confirmed
        # (bundle['start_pose'], {motor: raw rad}) back to its reference, then disabled.
        self.mode = bundle.get('mode') or 'tune'
        if self.mode not in ('tune', 'verify', 'return'):
            raise ValueError('The sweep tunes, verifies or returns the arm')
        if self.mode == 'return' and not bundle.get('start_pose'):
            raise ValueError('A return starts from the pose the operator confirmed')
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
        self.stopped_by = None  # Why the receiver stopped the arm, when it did.
        self._lowerings = 0
        self.engine = self.controller = self.receiver = self.compiler = self.derived = None
        self.absent = []  # Drives left out (unplugged), once built.
        self.pending = []  # Receiver results since the engine last saw them.
        self.cycles = []
        self._closers = []
        self._frozen = False  # The garbage collector's long-lived objects are frozen while the loop runs.
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
        returning = self.mode == 'return'
        service = service_config(config, device, reference, self.bundle['receiver']['motor_models'],
                                 bus=buses.pop(), tolerance_rad=live['reference_tolerance_rad'],
                                 period_us=live['period_us'], maximum_lateness_us=live['maximum_lateness_us'],
                                 start_pose=self.bundle['start_pose'] if returning else None)
        return LiveReceiver(service, device=device, reference=reference, binary=ROOT / 'bin' / 'guarded_controller',
                            directory=self.evidence, labels=self.labels, confirmed_pose=returning).start()

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
        # A drive unplugged for a wiring check (the gripper, the end of the chain) is left out: the receiver
        # neither scans nor drives it, and the collision model holds it in its reference pose.
        absent = set(self.bundle.get('absent') or ())
        if absent:
            if self.mode != 'tune' or not absent <= {f'{self.arm}_gripper'}:
                raise ValueError('Only a tuning sweep leaves a drive out, and only the gripper')
            reference = dict(reference, motors=[m for m in reference['motors'] if m['name'] not in absent])
            steps = [step for step in steps if step['motor'] not in absent]
            transmissions = [entry for entry in transmissions if entry['motor'] not in absent]
        self.absent = sorted(absent)
        motors = [motor['name'] for motor in reference['motors']]
        sweep = [] if self.mode == 'return' else [motor for motor in (self.bundle.get('joints') or motors)
                                                  if motor in motors]
        if not sweep and self.mode != 'return':
            raise ValueError('Choose at least one joint of the arm to sweep')
        if self.mode == 'return' and set(self.bundle['start_pose']) != set(motors):
            raise ValueError(f'The confirmed pose must cover every motor of the {self.arm} arm')
        backend = self.open_backend(model)
        self._closers.append(backend.close)
        # The profile's gains as derived (and any saved servo calibration), with the sweep's ceilings raised.
        base = derive_receiver(reference=reference, steps=steps, directions=self.bundle['directions'],
                               transmissions=transmissions, model=model, limits=self.bundle['limits'],
                               profile=self.profile, backend=backend, servo=self.bundle.get('servo'))
        gains = {joint['name']: (joint['hold_kp'], joint['hold_kd']) for joint in base['config']['joints']}
        if self.mode in ('verify', 'return'):
            derived = base  # The settings teleop would use: what the check verifies, and what holds a return.
        else:
            # The session's own settings, with the saved gravity feed-forward (a new torque map refits it).
            derived = derive_receiver(reference=reference, steps=steps, directions=self.bundle['directions'],
                                      transmissions=transmissions, model=model, limits=self.bundle['limits'],
                                      profile=sweep_profile(self.profile, motors, gains, sweep=sweep),
                                      backend=backend, gravity=(self.bundle.get('servo') or {}).get('gravity'))
        # The sweep's raised ceilings stay within what each motor's wire protocol carries.
        fit_motor_ranges(derived['config']['joints'], (self.bundle.get('receiver') or {}).get('motor_models') or {})
        # Each joint's derived row today, with the settings teleop uses today (the session may loosen them).
        # The receiver's speed cap is the commanded one times OVERSPEED_HEADROOM; the card compares commanded caps.
        from robot_platform.control.approach import OVERSPEED_HEADROOM
        today = {joint['name']: joint for joint in base['config']['joints']}
        self.travel = {row['motor']: dict(row, today={
            'max_tracking_error': today[row['motor']]['max_tracking_error'],
            'stop_acceleration': today[row['motor']]['stop_acceleration'],
            'max_velocity': today[row['motor']]['max_velocity'] / OVERSPEED_HEADROOM})
            for row in base['report'] if row['motor'] in today}
        self.derived = derived
        stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        kind = {'verify': 'check-sweep', 'return': 'return'}.get(self.mode, 'sweep')
        self.evidence = (self.evidence_root / f'{kind}-{stamp}') if self.live_arm else None
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
                if now_ms > harness.now_ms:
                    # As the live receiver, which ticks every 2.5 ms between requests (see helm.teleop).
                    self.pending.append(harness.step(now_ms))
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
        # The teleop profile's collision scope, as teleop checks: the arm against the other arm. Within an arm and
        # against the body, the CAD's assembly interfaces (bearings at the joints) read as contacts that are not
        # (docs/architecture/COLLISION_RUNTIME_IMPLEMENTATION.md), and each joint moves alone from the reference
        # pose within its hand-swept range. The right J1 sweep of 2026-09-28 stopped at +-0.2 rad on bearing/0.
        checker = PathChecker(backend, derived['joint_map'], home=home, fixed=fixed, cad_limits=derived['cad_limits'],
                              clearance_m=settings[(sweep or names)[0]]['clearance_m'],
                              step_rad=settings[(sweep or names)[0]]['path_step_rad'],
                              scope=derived['config']['collision']['scope'])
        policy = derived['config']['collision']
        window_s = (policy['max_verdict_ms'] + derived['config']['feedback_timeout_ms'] + policy['reaction_ms']) / 1000
        calibration = ((self.bundle.get('servo') or {}).get('joints') or {})
        self.engine = SweepEngine(derived['config']['joints'], home=home, gains=gains, settings=settings, sweep=sweep,
                                  checker=checker, window_s=window_s, verdict_s=policy['max_verdict_ms'] / 1000,
                                  labels=self.labels, mode=self.mode, period_s=self.period_s,
                                  calibration={m: calibration[m] for m in sweep if m in calibration},
                                  travel=self.travel, receiver=self.profile['receiver'],
                                  interpolation_s=derived['config'].get('interpolation_ms', 0) / 1000,
                                  gravity={row['name']: row['gravity'] for row in derived['lifecycle']
                                           if row.get('gravity')},
                                  # A saved sweep's inertia, fed forward as teleop does (this sweep refits it).
                                  inertia={row['name']: row['inertia'] for row in base['lifecycle']
                                           if row.get('inertia')})
        if self.evidence is not None:
            self.sink = (self.evidence / 'samples.f32').open('ab')
            self.engine.recording.sink = self.sink
            (self.evidence / 'columns.json').write_text(json.dumps(
                {'dtype': 'float32', 'columns': ['t_ms'] + [f'{name}.{field}' for name in names
                                                           for field in ('position', 'velocity', 'effort', 'command')]},
                indent=1) + '\n', encoding='utf-8')
        # As teleop does (helm.teleop.TeleopCore.build): a full garbage-collector pass over the collision model's
        # object graphs took 46 ms on the Jetson, and one in the middle of a right J1 sweep (2026-09-29) let the
        # receiver's 80 ms collision verdict expire. Freeze everything long-lived; later passes walk only new objects.
        gc.collect()
        gc.freeze()
        self._frozen = True
        self.phase, self.stage = 'running', 'arming'
        self._stage_since = self.clock()
        what = {'verify': 'LIVE JOINT SWEEP (MINERVA CHECK)', 'return': 'RETURN TO REFERENCE'}.get(self.mode,
                                                                                                   'SERVO SWEEP')
        moves = ('each joint returns alone to the reference' if self.mode == 'return'
                 else 'the sweep moves the arm')
        if self.absent:
            what += ' · WITHOUT ' + ', '.join(self.labels.get(name, name).upper() for name in self.absent)
        self.message = (f'{what} · LIVE {self.arm.upper()} ARM · enabling, then {moves}. '
                        'Keep the foot switch in reach.' if self.live_arm else
                        f'{what} · VIRTUAL RECEIVER · no motors move.')

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

    def _move(self, command, offered, offer_started):
        feedback = (self.latest() or {}).get('feedback') or []
        count = len(command['targets'])
        joints = [{'position': target, 'velocity': velocity, 'effort': effort, 'kp': kp, 'kd': kd}
                  for target, velocity, effort, (kp, kd) in zip(command['targets'],
                                                                 command.get('velocities') or [0.] * count,
                                                                 command.get('efforts') or [0.] * count,
                                                                 command['gains'])]
        envelope = self.compiler.compile(offered, feedback, joints, started_at=offer_started)
        self.controller.move(joints, collision=envelope, ramp_efforts=command.get('ramp_efforts'))

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
                # Take the collision offer first: its result brings the latest rows, so the command's lead is taken
                # from where the joint is now, not a cycle ago. The move binding the offer renews the lease.
                offer_started = self.clock()
                offered = self.controller.collision_offer()
                self._feed()
                command = self.engine.command(started)
                if command is None:
                    if self.mode == 'return':
                        why = next((e['text'] for e in reversed(self.engine.events)), 'stopped')
                        self._disable('Returned: the arm is at its reference and disabled.'
                                      if self.engine.task == 'finished' else
                                      f'Return stopped ({why}): the arm was disabled where it is, and rests there.')
                    else:
                        self._disable('Sweep finished: the arm is back at the reference and disabled. Review the '
                                      'results, then save the joints you accept.' if self.engine.task == 'finished'
                                      else 'Sweep stopped: the arm is back at the reference and disabled.')
                    self.stage = 'finished'
                else:
                    self._move(command, offered, offer_started)
                    progress = self.engine.progress()
                    self.message = (progress['task'] if self.mode == 'return' else
                                    f"{progress['task']} ({progress['joints_done']}/{progress['joints']} joints done)")
            elif self.stage == 'lowering':
                self._lower(started)
        except Exception as exc:
            status = getattr(exc, 'status', None) or (self.controller.status if self.controller else None) or {}
            reason = status.get('reason')
            if self.stage in ('sweeping', 'lowering') and self.engine is not None:
                why = reason if status.get('state') in HELD else str(exc) or type(exc).__name__
                if status.get('state') in ('holding', 'executing') and self.controller.owned:
                    # This loop failed with the arm up: have the receiver brake it to a hold, then lower it.
                    try:
                        status = self.controller.stop()
                    except Exception:
                        pass
                if status.get('state') in HELD:
                    # A trip (or a lapsed lease): the receiver braked the arm to a hold and revoked this session's
                    # authority. Take it back there and lower the arm; never let it drop.
                    self._held(why)
                    return
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
        self.message = ('Holding where it rests. Each joint returns alone to the reference.' if self.mode == 'return'
                        else 'Holding. The sweep moves the arm joint by joint.')

    def _held(self, reason):
        """The receiver stopped the arm and holds it: lower it from where it is held."""
        if self.stage != 'lowering':
            self.stage, self.stopped_by, self._lowerings = 'lowering', reason, 0
        self.message = (f'The receiver stopped the arm ({reason}) and holds it; taking it back to lower it to the '
                        'reference…')

    def land(self):
        """Before the session closes (closing the receiver cuts torque): lower an arm the sweep holds up, then
        disable it. True while there is an arm to lower; the loop keeps running until it is down."""
        if self.engine is None or self.controller is None or self.fault:
            return False
        if self.stage == 'sweeping':
            self.engine.lower(None, 'the session is closing')  # From the last command, which the receiver ramps to.
            self.stage, self._lowerings = 'lowering', 0
            self.message = 'Closing: lowering the arm to the reference, then disabling it…'
        return self.stage == 'lowering'

    def _lower(self, started):
        """Re-arm an arm the receiver holds where it holds it, move every joint to its reference slowly, then
        disable it (it hangs)."""
        status = self.controller.poll()
        state = status['state']
        if state == 'stopping':
            self.message = f'The receiver stopped the arm ({self.stopped_by}) and is braking it to a hold…'
            return
        if state == 'stopped':
            if self._lowerings >= LOWERING_ATTEMPTS:
                self.stage = 'held'
                self.message = (f'The receiver stopped the arm {LOWERING_ATTEMPTS} times while it was lowered '
                                f'({status.get("reason")}): it holds the arm where it stopped. Support the arm, '
                                'then press H to disable it (it goes limp), or stop the session.')
                return
            if not self.controller.owned:
                self.controller.acquire()
            offer_started = self.clock()
            offered = self.controller.collision_offer()
            feedback = (self.latest() or {}).get('feedback') or []
            held = status['joints']  # The stop's hold: re-armed there, with its feed-forward.
            try:
                status = self.controller.arm(collision=self.compiler.compile(offered, feedback, held,
                                                                             started_at=offer_started))
            except GuardRejected as exc:
                if exc.reason not in WAIT_TO_REARM:
                    raise
                dropped = (self.latest() or {}).get('hardware_dropped')
                if dropped:
                    # Out of Run mode, that drive never answers again this session: nothing can re-arm the arm.
                    self.stage = 'held'
                    self.message = (f'{dropped}. The receiver holds the arm on the other drives and cannot re-arm '
                                    'it this session. Support the arm, then press H to disable it (it goes limp).')
                    return
                # A drive not answering, or a joint not yet settled: the receiver holds the arm meanwhile.
                self.message = (f'The receiver stopped the arm ({self.stopped_by}) and holds it; waiting to re-arm '
                                f'it ({exc.reason})…')
                return
            self._lowerings += 1
            self.engine.lower({name: joint['position'] for name, joint in zip(self.engine.names, status['joints'])},
                              f'the receiver stopped the arm ({self.stopped_by})' if self.stopped_by
                              else 'the receiver stopped the arm')
            self.message = 'Re-armed where the receiver held it; lowering the arm to the reference…'
            return
        if state not in ('holding', 'executing') or not self.controller.owned:
            raise RuntimeError(f"the receiver is {state} ({status.get('reason')}): the arm's torque is off")
        offer_started = self.clock()
        offered = self.controller.collision_offer()  # The move binding it renews the lease.
        self._feed()
        command = self.engine.command(started)
        if command is None:
            why = f'the receiver stopped it ({self.stopped_by})' if self.stopped_by else 'the session closed'
            self._disable(f'Sweep ended: {why}. The arm was lowered to the reference and disabled. Review the '
                          'results; the joint under test was not finished.')
        else:
            self._move(command, offered, offer_started)
            self.message = 'Lowering the arm to the reference…'

    def _disable(self, message):
        if self.controller is not None:
            try:
                if self.controller.owned:
                    self.controller.disable()
                elif self.stage in ('lowering', 'held'):
                    # The receiver holds the arm under no owner: an emergency stop (always accepted) removes torque.
                    self.controller.emergency_stop()
            except Exception as exc:
                message += f' (disable reported: {exc})'
        self.stage = 'disabled' if self.stage != 'finished' else self.stage
        self.message = message

    def fail(self, reason):
        self.phase, self.fault, self.stage = 'fault', reason, 'faulted'
        if self.engine is not None:
            self.engine.halt(reason)
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
        if self._frozen:
            gc.unfreeze()
            self._frozen = False
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
                'arm': self.arm, 'live': self.live_arm is not None, 'mode': self.mode, 'absent': self.absent,
                'receiver': ((last or {}).get('status') or {}).get('state'),
                'motors_enabled': (last or {}).get('hardware_enabled'),
                'progress': self.engine.progress() if self.engine else None,
                'cycle_ms': {'p50': durations[(len(durations) - 1) // 2],
                             'p99': durations[min(len(durations), -(-99 * len(durations) // 100)) - 1]}
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
    closing = False  # Asked to stop, or Helm is gone: lowering the arm before the receiver closes.
    listening = True
    try:
        views.send(('view', core.view(results=False), None))
        core.build()
        views.send(('view', core.view(), None))
        while True:
            started = clock()
            key = None
            while listening:
                try:
                    if not commands.poll():
                        break
                    message = commands.recv()
                except (EOFError, OSError):
                    listening, message = False, ('stop',)
                if message[0] == 'stop':
                    # Closing the receiver cuts torque: a raised arm is lowered first (H still disables at once).
                    if not closing:
                        if not core.land():
                            return
                        closing = True
                    continue
                if message[0] == 'key':
                    key = message[1] if key is None or message[1] == 'H' else key
            core.tick(key, started)
            if core.fault or core.stage in ('finished', 'disabled'):
                return  # Nothing more to do: close the receiver and CAN link now; the last view has the results.
            try:
                views.send(('view', core.view(), None))
            except (OSError, EOFError, ValueError):
                if not core.land():
                    return
                closing = True
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

    def __init__(self, helm, profile_path, *, target='jetson', core_factory=None, edge_factory=None,
                 on_finished=None):
        self.helm = helm
        self.on_finished = on_finished  # (mode, arm, results, fault) once a sweep's loop ends.
        self.mode = 'tune'
        self.profile_path = Path(profile_path).resolve()
        self.profile = load_profile(self.profile_path)
        self.target, self.core_factory, self.edge_factory = target, core_factory, edge_factory
        self.receiver = {'mode': 'live', 'arm': 'right'}
        self.joints = None
        self.absent = None  # Drives unplugged for a wiring check, which the next tuning sweep leaves out.
        self._swept_without = None  # Those the last sweep left out: its results are not saved.
        self.path = servo_path(helm.profile_path, helm.asset_id)
        self._lock = threading.Lock()
        self._conn = self._views = self._worker = self._reader = None
        self._view = {'phase': 'stopped', 'stage': None, 'message': 'The servo sweep is not running.',
                      'fault': None, 'progress': None, 'results': None}
        self.results = None
        self.saved = self._load()
        self.saved_error = None
        self._start_pose = None  # A return's confirmed pose, while it starts.
        self.offer = None  # The return last shown to the operator (return_offer), until confirmed or replaced.

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

    def configure(self, *, arm, joints=None, mode='live', target=None, verify=False, absent=None):
        if arm not in ('left', 'right') or mode not in ('live', 'virtual'):
            raise ValueError('Sweep the left or right arm, live or on the virtual receiver')
        # Only the gripper, the end of the chain, can be unplugged and left out: a wiring check of the rest.
        if absent and (verify or not set(absent) <= {f'{arm}_gripper'}):
            raise ValueError(f'Only a tuning sweep leaves a drive out, and only the {arm} gripper')
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
            self.absent = sorted(set(absent)) if absent else None
            self.mode = 'verify' if verify else 'tune'

    def return_offer(self, arm):
        """What a return of `arm` to its reference would do, from the stopped link's live readings: each motor's
        reading, its offset from the reference and whether it moves. The operator confirms it against the arm
        (the 3D view shows those readings). Refused unless every reading is fresh and inside its swept range."""
        from robot_platform.control.servo_sweep import RETURNED_RAD, sweep_settings
        helm = self.helm
        if arm not in ('left', 'right'):
            raise ValueError('Return the left or right arm')
        # A resting arm's reading up to 2 s old: the stopped reader's 0.3 s scans plus their round trip sometimes
        # pass the 0.75 s live-edit limit, which refused a return between two fresh frames (2026-09-30). The
        # confirm, the live start's stopped scan and the receiver's start gate each check the pose again.
        if not helm.fresh(limit=2.):
            raise ValueError('Connect to the motors first (Connect, top right): the 3D view then shows the arm '
                             'as its drives read it, to compare with the real arm')
        limits = (helm.saved_limits or {}).get('joints') or {}
        motors = [motor for motor in helm.reference['motors'] if motor['name'].startswith(f'{arm}_')]
        if any(motor['name'] not in limits for motor in motors):
            raise ValueError('Save joint limits for every motor in the Limits workspace first')
        raw = dict(helm.raw or {})
        joints, outside = [], []
        for motor in motors:
            name = motor['name']
            q, home = raw.get(name), motor['raw_reference_rad']
            if q is None:
                raise ValueError(f'No live reading for {helm.labels.get(name, name)}')
            low, high = limits[name]['min_rad'], limits[name]['max_rad']
            if not low - .05 <= q <= high + .05:
                outside.append(f'{helm.labels.get(name, name)} reads {q:+.2f} rad, outside its swept '
                               f'{low:+.2f} to {high:+.2f}')
            settings = sweep_settings(self.profile, name)
            joints.append({'motor': name, 'label': helm.labels.get(name, name), 'position': q, 'offset': q - home,
                           'moves': abs(q - home) > RETURNED_RAD, 'speed': settings['characterize_speed'],
                           'rest_s': settings['rest_s']})
        if outside:
            raise ValueError('A reading outside its swept range cannot be trusted to move from: ' + '; '.join(outside)
                             + '. Turn that joint back by hand.')
        moving = [j for j in joints if j['moves']]
        # The joints move together along one line: the slowest to arrive at its own speed sets the time.
        return {'arm': arm, 'joints': joints, 'speed': max(j['speed'] for j in joints),
                'seconds': round(max((abs(j['offset']) / j['speed'] + j['rest_s'] for j in moving), default=0.), 1)}

    def start_return(self, arm, pose):
        """Return `arm` from `pose` ({motor: raw rad}, confirmed by the operator) to its reference, then disable."""
        with self._lock:
            if self.active:
                raise ValueError('The sweep is already running')
            previous = self.receiver, self.joints, self.mode, self.absent
            self.receiver, self.joints, self.mode, self.absent = {'mode': 'live', 'arm': arm}, None, 'return', None
            self._start_pose = dict(pose)
        try:
            self.start(confirm=True)
        finally:
            with self._lock:  # A later start sweeps again, as configured before.
                self.receiver, self.joints, self.mode, self.absent = previous
                self._start_pose = None

    def _bundle(self):
        helm = self.helm
        return {'kind': 'tune', 'mode': self.mode, 'calibration_profile': str(helm.profile_path),
                'bindings': deepcopy(helm.bindings),
                'directions': deepcopy(helm.mapping), 'limits': deepcopy(helm.saved_limits),
                'teleop_profile': str(self.profile_path), 'asset_id': helm.asset_id,
                'receiver': dict(self.receiver), 'arm': self.receiver['arm'], 'joints': self.joints,
                'servo': deepcopy(self.saved),
                **({'absent': list(self.absent)} if self.absent and self.mode == 'tune' else {}),
                **({'start_pose': dict(self._start_pose)} if self.mode == 'return' else {})}

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
            self._swept_without = bundle.get('absent')
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
            self._view = dict(self._view, phase='starting', fault=None, results=None, mode=self.mode,
                              message=(f"Returning the {self.receiver['arm']} arm: checking it is still where you "
                                       'confirmed it, then starting the receiver…' if self.mode == 'return' else
                                       f"Starting the sweep on the {self.receiver['arm']} arm: checking it is still and "
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
                finished = (self.mode, self.receiver['arm'], deepcopy(self.results), self._view.get('fault'))
            if self.on_finished is not None:
                try:
                    self.on_finished(*finished)
                except Exception:  # Recording is best effort; the sweep itself has ended safely.
                    pass

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

    def stop(self, timeout=45.):
        # The loop lowers a raised arm before it closes the receiver: about 11 s for J2 from horizontal.
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
        if self._swept_without:
            # The arm runs with every drive: a wrist swept without its gripper's load is not what teleop drives.
            raise ValueError('This sweep left out ' + ', '.join(self._swept_without) + ' to check the wiring; '
                             'sweep again with every drive plugged in to save settings')
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
            running = view.get('phase') in ('starting', 'running')
            return dict(view, receiver_choice=dict(self.receiver), joints_choice=self.joints, target=self.target,
                        absent_choice=self.absent,
                        mode=view.get('mode', self.mode) if running else self.mode,
                        results=deepcopy(self.results), saved=deepcopy(self.saved), saved_error=self.saved_error,
                        return_offer=deepcopy(self.offer),
                        path=str(self.path))
