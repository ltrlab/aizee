"""Helm teleoperation: the existing GUI lifecycle through the guarded receiver.

Idle (I) -> Enable (E) -> Engage/Align (T) -> Teleop; T again holds; Shutdown
(X) returns slowly to the reference; Disable (H) removes torque. The receiver is
the virtual (native harness) receiver, or, on the robot computer, the live
guarded service driving one arm (control.live_receiver) with advisory collision
checks until a two-arm bound exists.

The 25 Hz loop runs in its own process and owns the leader USB, the receiver and
the collision model. The web server only relays keys and route edits and reads
back the latest state, so browser traffic cannot stall a control cycle.
Receiver limits come from the saved joint limits. Each leader channel's saved
(hand-swept) range spans its robot joint's usable range; route signs come from
the teleop profile plus local operator edits.
"""
from copy import deepcopy
import gc
import itertools
import json
import multiprocessing
from pathlib import Path
import threading
import time

from robot_platform.control.guarded_config import arm_subset, derive_receiver, fit_motor_ranges, to_cad
from robot_platform.control.joint_calibration import save_json
from robot_platform.control.live_receiver import fault_detail
from robot_platform.control.scheduling import prefer_responsive
from .limits import utc_now

KEYS = frozenset('IETHX')
ENGAGED = ('ENGAGING', 'TELEOP')
HELD = ('stopping', 'stopped')  # Receiver states in which it brakes or holds the arm itself.
LOWERING_ATTEMPTS = 3  # Re-arms after the receiver stops the arm, before it is left held where it stopped.
# A re-arm the receiver refuses for these waits (it holds the arm meanwhile): a drive not answering, a joint over
# a limit, or one still moving.
WAIT_TO_REARM = ('feedback_stale', 'limit_violation', 'invalid_state')


def load_profile(path):
    profile = json.loads(Path(path).read_text(encoding='utf-8'))
    if (profile.get('kind') != 'guarded_teleop_profile' or profile.get('backend') != 'virtual'
            or profile.get('runtime_approved') is not False):
        raise ValueError('Helm teleop needs a virtual, unapproved guarded teleop profile')
    mode = profile['receiver'].get('collision_mode', 'enforced')
    if mode not in ('enforced', 'advisory'):
        raise ValueError('collision_mode must be enforced or advisory')
    if mode == 'advisory' and profile['backend'] != 'virtual':
        raise ValueError('Advisory collision checks are only allowed with the virtual receiver')
    receiver = profile['receiver']
    interpolation = receiver.get('interpolation_ms', 40)
    if (type(interpolation) is not int or not 10 <= interpolation <= 40
            or not 0 < receiver.get('compute_budget_s', 0) <= .6 * interpolation / 1000):
        raise ValueError('The receiver interpolates over 10-40 ms (the loop period), with compute under 60% of it')
    live = profile.get('live')
    if live is not None and (
            set(live) - {'reference_tolerance_rad', 'period_us', 'maximum_lateness_us', 'note'}
            or not 0 < live.get('reference_tolerance_rad', 0) <= .1
            or not 1000 <= live.get('period_us', 0) <= 10000
            or not live['period_us'] <= live.get('maximum_lateness_us', 0) <= 20000):
        raise ValueError('The live block needs a reference tolerance of at most 0.1 rad and receiver timing')
    return profile


def loop_period_s(profile):
    """The loop's period: the receiver ramps each command over exactly one, so commands neither stall between
    ramps nor cut one short."""
    return profile['receiver'].get('interpolation_ms', 40) / 1000


def source_misses(period_s):
    """Missed leader samples an engaged arm rides out: 80 ms of them, whatever the loop rate."""
    return max(1, round(.08 / period_s))


CYCLE_BUDGET_S = .035  # A cycle not dispatched this long after it started faults the session; a slower one
# than its period only starts the next late (the receiver's lease and collision verdict still bound it).


def routes_path(runs, asset_id):
    return Path(runs).resolve() / (asset_id + '-teleop') / 'routes.json'


def load_route_edits(path, profile, asset_id):
    """Operator route edits ({motor: {sign, verified, history}}) saved by Helm for this profile and robot."""
    path = Path(path)
    if not path.is_file():
        return {}
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('profile') != profile['id'] or data.get('asset_id') != asset_id:
        raise ValueError(f'{path} belongs to another teleop profile')
    # Trigger edits from the old released-opens mapping ('squeeze_sign') meant
    # something else; those grippers need verifying again.
    return {motor: edit for motor, edit in data.get('routes', {}).items() if 'squeeze_sign' not in edit}


def save_route_edits(path, profile, asset_id, edits):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    save_json(path, {'schema': 1, 'profile': profile['id'], 'asset_id': asset_id, 'runtime_approved': False,
                     'routes': edits})


def merge_routes(routes, edits):
    """The profile's routes with the operator's sign and verification edits applied."""
    merged = deepcopy(routes)
    for motor, edit in edits.items():
        if motor in merged:
            merged[motor].update({k: v for k, v in edit.items() if k in ('sign', 'verified')})
    return merged


class AdvisoryCompiler:
    """Report clearance at the commanded pose; never block.

    The conservative swept-box check cannot certify teleop speeds (and is slow
    when it fails), so this checks the target pose itself against the same
    keep-out geometry and scope. The receiver gets its whole operational interval
    and still enforces it, the stopping reserve inside it and the single-use
    offer. Used with the virtual receiver, and with one live arm until a two-arm
    bound exists: model joints outside this receiver (the other, disabled arm)
    are taken at `fixed`, their parked reference pose.
    """
    def __init__(self, backend, config, joint_map, *, fixed=None, cad_limits=None, clearance_m=.005,
                 warn_m=.05, compute_budget_s=.025):
        self.backend, self.config = backend, config
        self.names = tuple(joint['name'] for joint in config['joints'])
        self.joint_map, self.fixed, self.cad_limits = dict(joint_map), dict(fixed or {}), dict(cad_limits or {})
        self.clearance_m, self.warn_m, self.compute_budget_s = clearance_m, warn_m, compute_budget_s
        scene = getattr(getattr(backend, 'runtime_model', None), 'scene', None)
        if scene is not None and set(self.joint_map) | set(self.fixed) != set(scene.independent_joints):
            raise ValueError('Every model joint must follow this receiver or have a fixed pose')
        self.last_verdict = None
        self.blocked = 0

    def compile(self, offered, feedback, joints=None, *, started_at=None):
        target = [row['position'] for row in (joints if joints is not None else feedback)]
        cad = dict(self.fixed)
        cad.update(to_cad(self.joint_map, dict(zip(self.names, target)), self.cad_limits))
        policy = self.config['collision']
        result = self.backend.proximity(cad, threshold=self.warn_m, scope=policy['scope'])
        pairs = sorted(result['near_pairs'], key=lambda pair: pair['distance_m'])
        clear = not pairs or pairs[0]['distance_m'] > self.clearance_m
        if not clear:
            self.blocked += 1
        self.last_verdict = {'advisory': True, 'clear': clear,
                             'reason': 'target_clear' if not pairs else 'target_near' if clear else 'target_contact',
                             'distance_m': pairs[0]['distance_m'] if pairs else None,
                             'shapes': pairs[0]['shapes'] if pairs else None}
        return {'offer': offered['offer']['id'], 'world_revision': policy['world_revision'],
                'scope': policy['scope'],
                'lower': [joint['lower'] for joint in self.config['joints']],
                'upper': [joint['upper'] for joint in self.config['joints']]}


class TeleopCore:
    """The loop body, built from plain data. Openers are injectable for tests."""

    def __init__(self, bundle, *, open_leaders=None, open_backend=None, open_harness=None, open_receiver=None,
                 clock=time.perf_counter):
        self.bundle, self.clock = deepcopy(bundle), clock
        self.profile = load_profile(bundle['teleop_profile'])
        self.period_s = loop_period_s(self.profile)
        self.collision_mode = self.profile['receiver'].get('collision_mode', 'enforced')
        receiver = bundle.get('receiver') or {'mode': 'virtual'}
        self.live_arm = receiver.get('arm') if receiver.get('mode') == 'live' else None
        if receiver.get('mode') not in ('virtual', 'live') or (receiver['mode'] == 'live'
                                                                 and self.live_arm not in ('left', 'right')):
            raise ValueError('The receiver is virtual, or live on the left or right arm')
        self.open_receiver = open_receiver or self._open_receiver
        self._powered = False  # Live: the receiver's enable permit is single-use.
        # No routes_path: a loop on the robot computer, where Helm keeps the file (route_edits).
        self.routes_path = Path(bundle['routes_path']) if bundle.get('routes_path') else None
        self.route_edits = self._load_route_edits()
        self._hold = False  # Set with the zero: the next cycle holds an engaged arm.
        if open_leaders is None and bundle.get('leader_factory'):
            # Previews/tests only: 'module:function' building a scripted leader source.
            import importlib
            module, _, function = bundle['leader_factory'].partition(':')
            open_leaders = getattr(importlib.import_module(module), function)
        self.open_leaders = open_leaders or self._open_leaders
        self.open_backend = open_backend or self._open_backend
        self.open_harness = open_harness or self._open_harness
        self.phase = 'starting'
        self.message = ('Checking the arm, then starting the live receiver…' if self.live_arm
                        else 'Loading the collision model and virtual receiver; opening the leaders…')
        self.derived = self.session = self.mapper = self.leaders = self.harness = self.compiler = None
        self.scales = {}
        self.fault = self.input_error = self.last_target = None
        # After a fault the receiver holds the arm: it is taken back and lowered (the lifecycle's shutdown).
        self._lowering, self._lowerings = False, 0
        self._land = False  # Closing: shut down (lower, then disable) before the receiver closes.
        self.late_targets, self.sequence, self.overruns = 0, 0, 0
        self.slowest_ms = self.largest_gap_ms = 0.
        self._last_start = None
        self.cycles = []
        self._closers = []
        self._frozen = False
        self.scheduling = {}

    # -- resources -------------------------------------------------------

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
        evidence = Path.home() / 'minerva' / 'evidence' / time.strftime('live-%Y%m%dT%H%M%SZ', time.gmtime())
        return LiveReceiver(service, device=device, reference=reference, binary=ROOT / 'bin' / 'guarded_controller',
                            directory=evidence, labels=self.labels).start()

    def _open_leaders(self, profile):
        from robot_platform.teleop.leader_bridge import SAMPLER_HZ, LatestLeaderInputs
        from robot_platform.teleop.openrb import OpenRBInputs
        leaders = LatestLeaderInputs(OpenRBInputs(profile), rate_hz=SAMPLER_HZ)
        try:
            leaders.connect()
        except Exception as exc:
            raise RuntimeError(f'Could not open the leader arms ({exc}). Close the leader viewer or any other '
                               'program using their USB ports, then start again.') from exc
        return leaders

    # -- route edits -----------------------------------------------------

    def _load_route_edits(self):
        if self.bundle.get('route_edits') is not None:
            return deepcopy(self.bundle['route_edits'])
        return load_route_edits(self.routes_path, self.profile, self.bundle['asset_id'])

    def _save_edits(self):
        if self.routes_path is not None:
            save_route_edits(self.routes_path, self.profile, self.bundle['asset_id'], self.route_edits)

    def routes(self):
        return merge_routes(self.profile['routes'], self.route_edits)

    def route(self, action, motor):
        if self.session is not None and self.session.lifecycle.state in ENGAGED:
            raise ValueError('Hold (T) before changing a leader route')
        current = self.routes().get(motor)
        if current is None or action not in ('flip', 'accept'):
            raise ValueError('Use flip or accept with a routed motor')
        edit = self.route_edits.setdefault(motor, {})
        edit['sign'] = -current['sign'] if action == 'flip' else current['sign']
        edit['verified'] = action == 'accept'
        edit.setdefault('history', []).append({'action': action, 'at': utc_now()})
        self._save_edits()
        if self.derived is not None:
            self.mapper = self._mapper()
        return deepcopy(self.route_edits)

    def set_zero(self):
        """The leaders' pose now becomes the zero, mapping onto the robot's reference pose.

        Allowed at any time. An engaged arm holds on the next cycle rather than
        jumping to the new mapping; Engage (T) aligns to it slowly. Returns the
        zero as {leader: {channel name: reading_rad}} for saving.
        """
        if self.derived is None or self.phase != 'running' or self.fault:
            raise ValueError('Set the zero in a running session')
        samples = self.leaders.poll()
        self.references = {name: list(sample.positions_rad) for name, sample in samples.items()}
        self.mapper = self._mapper()
        self._hold = self.session.lifecycle.state in ENGAGED
        self.message = 'Leader zero set. Engage (T) to follow from it.'
        return {name: {joint['name']: q for joint, q in zip(self.leader_profile['leaders'][name]['joints'],
                                                            sample.positions_rad)}
                for name, sample in samples.items()}

    # -- build -------------------------------------------------------------

    def _mapper(self):
        from robot_platform.teleop.absolute_mapping import AbsoluteLeaderMapper
        from robot_platform.teleop.leader_routes import build_routes
        report = self.derived['report']
        usable = {row['motor']: tuple(row['usable']) for row in report}
        opens = {row['motor']: row['opens'] for row in report if row['opens']}
        # The leader zero lines up with the robot's reference pose.
        home = {row['motor']: row['home'] for row in report}
        routes = {motor: route for motor, route in self.routes().items() if motor in home}
        anchors = {motor: (self.references[route['leader']][route['channel'] - 1], home[motor])
                   for motor, route in routes.items()}
        routes = build_routes(routes, self.ranges, self.derived['joint_map'], usable, opens, anchors)
        # One live arm routes from one leader; the mapper takes exactly the leaders it routes.
        self.route_leaders = {route['leader'] for route in routes}
        self.scales = {route['actuator']: {key: route[key] for key in ('scale', 'reach', 'overtravel')}
                       for route in routes}
        # 20 rad/s: a quick trigger squeeze is not an encoder glitch (those jump much further).
        return AbsoluteLeaderMapper(self.derived['config']['joints'], routes,
                                    max_age_s=self.leaders.inputs.max_age_s, max_skew_s=self.leaders.inputs.max_skew_s,
                                    max_leader_speed_rad_s=20.)

    def build(self):
        from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler
        from robot_platform.control.guarded import GuardedControllerClient
        from robot_platform.control.joint_calibration import load_plan, validate_preview_transmissions
        from robot_platform.teleop.guarded_lifecycle import GuardedLifecycleSession
        from robot_platform.teleop.leader_ranges import range_table, reference_table, validate_ranges
        from robot_platform.teleop.lifecycle import TeleopLifecycle
        plan, _paths, device, reference, model, pose, bindings = load_plan(self.bundle['calibration_profile'])
        if bindings != self.bundle['bindings']:
            raise ValueError('Calibration files changed since Helm started; restart Helm')
        transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), plan['steps'],
                                                       reference, model)
        self.model, self.pose = model, pose
        self.labels = {s['motor']: s['label'] for s in plan['steps']}
        self.labels.update({t['motor']: t['motor'].replace('_', ' ').capitalize() for t in transmissions})
        steps = plan['steps']
        if self.live_arm:
            # One arm on physical motors: the other arm is not in this receiver and is never enabled.
            if self.collision_mode != 'advisory' or 'live' not in self.profile:
                raise ValueError('Live teleop needs the profile\'s live block and advisory collision checks: '
                                 'one arm, until a two-arm collision bound exists')
            reference, steps, transmissions = arm_subset(reference, steps, transmissions, self.live_arm)
        backend = self.open_backend(model)
        self._closers.append(backend.close)
        derived = derive_receiver(reference=reference, steps=steps, directions=self.bundle['directions'],
                                  transmissions=transmissions, model=model, limits=self.bundle['limits'],
                                  profile=self.profile, backend=backend, servo=self.bundle.get('servo'))
        fit_motor_ranges(derived['config']['joints'], (self.bundle.get('receiver') or {}).get('motor_models') or {})
        base = Path(self.bundle['teleop_profile']).resolve().parent
        leader_profile = json.loads((base / self.profile['leader_profile']).read_text(encoding='utf-8'))
        if self.bundle.get('leader_ranges') is None:
            raise ValueError('Sweep and save the leader ranges in the Leaders tab first')
        document = validate_ranges(self.bundle['leader_ranges'], leader_profile)
        self.ranges = range_table(document, leader_profile)
        self.references = reference_table(document, leader_profile)
        if self.references is None:
            raise ValueError('Set the leader zero in the Leaders tab first')
        self.leader_profile = leader_profile
        self.centers = {leader: [None if row is None else row[0] for row in rows]
                        for leader, rows in self.ranges.items()}
        # The receiver: the native harness (virtual) or the live service; both keep `.last`.
        if self.live_arm:
            harness = self.open_receiver(derived['config'], device, reference)
            self._closers.append(harness.close)
            exchange = harness.exchange
        else:
            harness = self.open_harness(derived['config'], derived['initial'])
            self._closers.append(harness.close)
            epoch = self.clock()

            def exchange(request):
                now_ms = max(harness.now_ms, int((self.clock() - epoch) * 1000))
                if now_ms > harness.now_ms:
                    # The live receiver ticks every 2.5 ms between requests, so the joints have taken the latest
                    # command before a request reads them; the harness ticks only when stepped.
                    harness.step(now_ms)
                return harness.step(now_ms, request)['reply']

        names = tuple(joint['name'] for joint in derived['config']['joints'])
        controller = GuardedControllerClient(exchange, group=derived['config']['group'],
                                             identity=derived['config']['identity'], joint_names=names)
        budget = self.profile['receiver']['compute_budget_s']
        if self.collision_mode == 'advisory':
            # Model joints outside this receiver (the other arm, if live) stay at the reference pose.
            fixed = {name: pose[name] for name in pose if name not in derived['joint_map']}
            compiler = AdvisoryCompiler(backend, derived['config'], derived['joint_map'], fixed=fixed,
                                        cad_limits=derived['cad_limits'], compute_budget_s=budget)
        else:
            compiler = CollisionEnvelopeCompiler(backend, derived['config'], derived['joint_map'],
                                                 compute_budget_s=budget, cad_limits=derived['cad_limits'])
        lifecycle = TeleopLifecycle(derived['lifecycle'], clock=self.clock,
                                    max_source_misses=source_misses(self.period_s))
        session = GuardedLifecycleSession(controller, compiler, lifecycle, lambda: harness.last,
                                          period_s=self.period_s, cycle_budget_s=CYCLE_BUDGET_S, clock=self.clock)
        self.derived, self.harness, self.session, self.compiler = derived, harness, session, compiler
        self.leaders = self.open_leaders(leader_profile)
        self._closers.append(self.leaders.close)
        self.mapper = self._mapper()
        # Full garbage-collector passes over the CAD/collision object graphs took
        # 30+ ms, longer than a cycle's budget. Freeze everything long-lived so
        # later passes only walk new objects; reference counting still frees.
        gc.collect()
        gc.freeze()
        self._frozen = True
        self.phase = 'running'
        self.message = (f'LIVE {self.live_arm.upper()} ARM · motors are enabled from Idle (I). Keep the foot '
                        'switch in reach; then Enable (E) and Engage (T).' if self.live_arm else
                        'VIRTUAL RECEIVER · no motors move. Idle (I) first, then Enable (E) and Engage (T).')

    def close(self):
        if self.session is not None:
            try:
                self.session.disable()
            except Exception:
                self.session.fault('helm_teleop_closed')
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
        if self.phase != 'fault':
            self.phase = 'stopped'
            self.message = ('Teleop stopped. The live receiver disabled the arm and its CAN link is down.'
                            if self.live_arm else 'Teleop stopped. The virtual receiver is closed.')
        if failures:
            self.message += ' Closing reported: ' + '; '.join(failures)

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason
        self.message = f'Teleop stopped: {reason}'

    # -- cycle -------------------------------------------------------------

    def _recent_cycle_s(self):
        durations = sorted(c['completed_at'] - c['started_at'] for c in self.cycles[-25:]
                           if c.get('completed_at') is not None)
        return durations[int(.9 * (len(durations) - 1))] if durations else 0.

    def _leader_target(self, engaged):
        from robot_platform.teleop.leader_routes import unwrap_samples
        if self.mapper.fault and not engaged:
            self.mapper.reset()  # Leaders only preview outside engagement.
        try:
            samples = unwrap_samples(self.leaders.poll(), self.centers)
            samples = {name: sample for name, sample in samples.items() if name in self.route_leaders}
            target = self.mapper.map(samples, now=self.clock())
        except Exception as exc:
            return None, str(exc) or type(exc).__name__
        self.last_target = target
        margin = max(self._recent_cycle_s(), .01)
        age = self.clock() - min(row['sampled_at'] for row in target)
        if age + margin >= self.session.lifecycle.target_max_age_s:
            self.late_targets += 1
            return None, None
        return target, None

    def tick(self, key, started):
        from robot_platform.teleop.guarded_lifecycle import minerva_action
        if self.fault:
            # The receiver stopped itself and revoked the lease. It holds the arm: take it back there and
            # lower it, keeping the original reason on screen; a new session is needed to follow again.
            if key == 'H' and self._lowering:
                self._lowering = False
                self.session.disable_now()
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · Disabled at your request: the arm '
                                'is limp where it was. Stop the session and start it again.')
            elif self._lowering:
                self._lower(started)
            return
        lifecycle = self.session.lifecycle
        previous = lifecycle.state
        engaged = previous in ENGAGED
        action = minerva_action(key, previous) if key else None
        notice = None
        if self._land:
            # Closing: lower the arm and disable it (as X) before the receiver closes; H still disables at once.
            self._land = False
            if previous in ('HOLD', 'ENGAGING', 'TELEOP') and action != 'disable':
                action = 'shutdown'
        if self._hold:
            self._hold = False
            if engaged:
                action = 'disengage'  # A new zero never moves an engaged arm.
                notice = 'Leader zero set. Holding: engage (T) to follow from it.'
        target = None
        if key != 'H':
            target, self.input_error = self._leader_target(engaged)
        allowed = {'enable': ('IDLE',), 'engage': ('IDLE', 'HOLD'), 'disengage': ENGAGED,
                   'shutdown': ('IDLE', 'HOLD', 'ENGAGING', 'TELEOP')}
        if action in allowed and previous not in allowed[action]:
            notice = f'[{key}] Not available from {previous}. Start with Idle (I), then Enable (E).'
            action = None
        if self.live_arm and self._powered and previous == 'DISABLED' and action not in (None, 'disable'):
            # The live receiver spent its single-use enable permit; enabling again needs a
            # new session, which checks the reference pose and the watchdogs again.
            notice = f'[{key}] The arm was disabled. Stop the session and start a new one to enable it again.'
            action = None
        if action == 'engage' and target is None:
            notice = f'[{key}] Engage needs live leaders: {self.input_error or "sample arrived late; press T again"}'
            action = None
        if self._last_start is not None:
            self.largest_gap_ms = max(self.largest_gap_ms, (started - self._last_start) * 1000)
        self._last_start = started
        try:
            record = self.session.cycle(action=action, target=target, started_at=started)
        except Exception as exc:
            self._record(self.session.last_cycle or {}, failure=str(exc))
            # Lead with the receiver's own reason when it stopped first (a late
            # cycle shows up client-side only as lost ownership).
            status = getattr(exc, 'status', None) or self.session.controller.status or {}
            reason = status.get('reason')
            detail = fault_detail(self.harness.last, self.derived['config'], self.labels) if self.harness else ''
            self.fault = (f"Receiver {status.get('state')}: {reason}" + (f' ({detail})' if detail else '')
                          + f' · {exc}') if reason not in (None, 'ok') else str(exc)
            if self._held():
                self._lowering, self._lowerings = True, 0
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · The receiver braked the arm to a '
                                'hold; lowering it to the reference, then disabling it…')
            else:
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · The receiver stopped itself'
                                + (' and disabled the arm' if self.live_arm else '')
                                + '. Stop the session and start it again.')
            return
        self._record(record)
        self.sequence += 1
        current = lifecycle.state
        self._powered = self._powered or current != 'DISABLED'
        if notice:
            self.message = notice
        elif current != previous:
            self.message = f'{self._receiver_name} · {current}'
            if current == 'HOLD' and engaged and lifecycle.reason:
                self.message += f' · {lifecycle.reason} · last input problem: {self.input_error}'
            if current == 'DISABLED' and self.live_arm:
                self.message += ' · the arm is limp. Stop the session; a new one checks the reference pose again.'

    def _held(self):
        """Whether the receiver brakes or holds the arm itself (after a trip, or the stop a failed cycle asked
        for), rather than having cut its torque."""
        try:
            return self.session.controller.poll()['state'] in HELD
        except Exception:
            return False

    def _lower(self, started):
        """After a fault: take the arm back where the receiver holds it, then step the lifecycle's shutdown down
        to the reference, which ends disabled with the arm hanging."""
        session = self.session
        try:
            if session.lifecycle.state == 'FAULT':
                if self._lowerings >= LOWERING_ATTEMPTS:
                    self._lowering = False
                    self.message = (f'{self._receiver_name} FAULT · {self.fault} · The receiver stopped the arm '
                                    f'{LOWERING_ATTEMPTS} times while it was lowered and holds it where it stopped. '
                                    'Support the arm, then stop the session (it goes limp).')
                    return
                if session.recover():
                    self._lowerings += 1
                    self.message = (f'{self._receiver_name} FAULT · {self.fault} · Re-armed where the receiver '
                                    'held it; lowering the arm to the reference…')
                return
            record = session.cycle(action=None, target=None, started_at=started)
            self._record(record)
            if session.lifecycle.state == 'DISABLED':
                self._lowering = False
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · The arm was lowered to the '
                                'reference and disabled. Stop the session and start it again.')
        except Exception as exc:
            self._record(session.last_cycle or {}, failure=str(exc))
            if getattr(exc, 'reason', None) in WAIT_TO_REARM and self._held():
                dropped = (self.harness.last or {}).get('hardware_dropped') if self.harness else None
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · ' + (
                    f'{dropped}. The receiver holds the arm on the other drives and cannot re-arm it this session. '
                    'Support the arm, then press H to disable it (it goes limp).' if dropped else
                    f'The receiver holds the arm; waiting to re-arm it ({exc.reason})…'))
            if not self._held():  # The receiver cut the arm's torque: nothing left to lower.
                self._lowering = False
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · Lowering stopped: {exc}. '
                                'Stop the session and start it again.')

    def land(self):
        """Before the session closes (closing the receiver cuts torque): lower an enabled arm to the reference
        (the lifecycle's shutdown, as X does), then disable it. True while there is an arm to lower."""
        if self.session is None:
            return False
        if not self.fault and self.session.lifecycle.state in ('HOLD', 'ENGAGING', 'TELEOP'):
            self._land = True
        return self.landing

    @property
    def landing(self):
        """Whether the loop is still lowering the arm before it may close the receiver."""
        if self.session is None:
            return False
        if self.fault:
            return self._lowering
        return self._land or self.session.lifecycle.state == 'SHUTDOWN'

    @property
    def _receiver_name(self):
        return f'LIVE {self.live_arm.upper()} ARM' if self.live_arm else 'VIRTUAL RECEIVER'

    def _record(self, record, failure=None):
        entry = {k: record.get(k) for k in ('started_at', 'completed_at', 'accepted', 'action')}
        if failure:
            entry['failure'] = failure
        if entry['started_at'] is not None and entry['completed_at'] is not None:
            duration = (entry['completed_at'] - entry['started_at']) * 1000
            self.slowest_ms = max(self.slowest_ms, duration)
            if duration > self.period_s * 1000:
                self.overruns += 1
        self.cycles.append(entry)
        if len(self.cycles) > 500:
            del self.cycles[:100]

    # -- views ---------------------------------------------------------------

    def _positions(self):
        feedback = self.session.last_feedback if self.session else None
        if not feedback:
            return None
        return {row['name']: row['position'] for row in feedback}

    def guide(self):
        measured = self._positions()
        if measured is None or self.phase != 'running':
            return None
        positions = dict(self.pose)
        positions.update(to_cad(self.derived['joint_map'], measured, self.derived['cad_limits']))
        state = self.session.lifecycle.state
        return {'model': self.model.profile.id, 'urdf_sha256': self.model.sha256,
                'positions': positions, 'live': True, 'rows': [], 'stream_id': 'helm-teleop',
                'sequence': self.sequence, 'updated_at': time.time(), 'source_age_seconds': 0.,
                'scan_seconds': self.period_s, 'feed_state': 'reading', 'reconnections': 0, 'view': 'fit',
                'badge': (f'LIVE {self.live_arm.upper()} ARM · {state} · foot switch in reach' if self.live_arm
                          else f'VIRTUAL FOLLOWER · {state} · no motors move'),
                'message': self.message}

    def view(self):
        routes = self.routes()
        receiver = (self.harness.last or {}).get('status', {}).get('state') if self.harness else None
        verdict = getattr(self.compiler, 'last_verdict', None) or {}
        last = (self.harness.last or {}) if self.harness else {}
        result = {'phase': self.phase, 'message': self.message, 'fault': self.fault,
                  'state': self.session.lifecycle.state if self.session else None, 'receiver': receiver,
                  'live': {'arm': self.live_arm, 'motors_enabled': last.get('hardware_enabled'),
                           'watchdog': last.get('watchdog'), 'reference': last.get('reference')}
                  if self.live_arm else None,
                  'input_error': self.input_error, 'late_targets': self.late_targets,
                  'collision': {'mode': self.collision_mode, 'clear': verdict.get('clear'),
                                'reason': verdict.get('reason'), 'distance_m': verdict.get('distance_m'),
                                'shapes': verdict.get('shapes'), 'blocked': getattr(self.compiler, 'blocked', 0)},
                  'scheduling': self.scheduling, 'cycle_ms': None, 'joints': []}
        durations = sorted((c['completed_at'] - c['started_at']) * 1000 for c in self.cycles[-100:]
                           if c.get('completed_at') is not None)
        if durations:
            result['cycle_ms'] = {'p50': durations[(len(durations) - 1) // 2],
                                  'p99': durations[min(len(durations), -(-99 * len(durations) // 100)) - 1],
                                  'max': self.slowest_ms, 'overruns': self.overruns,
                                  'largest_gap': self.largest_gap_ms}
        if self.derived is None:
            return result
        names = [joint['name'] for joint in self.derived['config']['joints']]
        measured = self._positions() or {}
        command = dict(zip(names, self.session.lifecycle.target or ()))
        leader = {} if self.input_error else {row['name']: row['position'] for row in (self.last_target or [])}
        for row in self.derived['report']:
            motor = row['motor']
            route = routes.get(motor, {})
            result['joints'].append({'motor': motor, 'label': self.labels.get(motor, motor),
                'usable': row['usable'], 'operational': row['operational'],
                'leader': leader.get(motor), 'command': command.get(motor), 'measured': measured.get(motor),
                'route': {'sign': route.get('sign'), 'trigger': motor.endswith('gripper'),
                          'verified': route.get('verified', False), **self.scales.get(motor, {}),
                          'leader': route.get('leader'), 'channel': route.get('channel')}})
        return result


def run_loop(commands, views, core):
    """Serve one session: keys and route edits arrive on `commands`, state leaves on `views`.

    The control loop only drops its latest state into a slot; a sender thread does
    the (possibly blocking) pipe writes, so a slow reader drops stale views
    instead of stalling a cycle.
    """
    clock = core.clock
    keys = []
    ready = threading.Condition()
    outbox = {'view': None, 'replies': [], 'closing': False}

    def sender():
        while True:
            with ready:
                ready.wait_for(lambda: outbox['view'] is not None or outbox['replies'] or outbox['closing'])
                view, replies = outbox['view'], outbox['replies']
                outbox['view'], outbox['replies'] = None, []
                finished = outbox['closing'] and view is None and not replies
            if finished:
                return
            try:
                for reply in replies:
                    views.send(('reply', *reply))
                if view is not None:
                    views.send(('view', *view))
            except (OSError, EOFError, ValueError):
                return

    def publish(guide=True):
        state = (core.view(), core.guide() if guide else None)
        with ready:
            outbox['view'] = state
            ready.notify()

    def reply(request, error, result=None):
        with ready:
            outbox['replies'].append((request, error, result))
            ready.notify()

    thread = threading.Thread(target=sender, daemon=True, name='helm-teleop-sender')
    thread.start()
    closing = False  # Asked to stop, or Helm went away: lowering the arm before the receiver closes.
    listening = True
    try:
        publish(False)
        core.build()
        publish()
        while True:
            started = clock()
            while listening:
                try:
                    if not commands.poll():
                        break
                    message = commands.recv()
                except (EOFError, OSError):
                    listening, message = False, ('stop',)  # Helm went away: stop and clean up.
                if message[0] == 'stop':
                    # Closing the receiver cuts torque: an enabled arm is lowered first (H still disables at once).
                    if not closing:
                        if not core.land():
                            return
                        closing = True
                    continue
                if message[0] == 'key':
                    keys.append(message[1])
                elif message[0] in ('route', 'zero'):
                    _, request, *arguments = message
                    try:
                        reply(request, None, (core.route if message[0] == 'route' else core.set_zero)(*arguments))
                    except Exception as exc:
                        reply(request, str(exc) or type(exc).__name__)
            core.tick(keys.pop(0) if keys else None, started)
            publish()
            if closing and not core.landing:
                return
            remaining = core.period_s - (clock() - started)
            if remaining > 0:
                time.sleep(remaining)
    except Exception as exc:
        core.fail(str(exc) or type(exc).__name__)
    finally:
        core.close()
        publish(False)
        with ready:
            outbox['closing'] = True
            ready.notify()
        thread.join(5)
        views.close()
        commands.close()


def _process_main(commands, views, bundle):
    core = TeleopCore(bundle)
    core.scheduling['loop'] = prefer_responsive()
    run_loop(commands, views, core)


class TeleopSession:
    """Web-side handle: starts the loop, relays keys, caches its views.

    The loop runs in a local process (target 'pc') or on the robot computer
    (target 'jetson', see helm.edge) with the same messages either way.
    """

    TARGETS = ('jetson', 'pc')

    def __init__(self, helm, profile_path, *, runs, leaders=None, core_factory=None, leader_factory=None,
                 target='pc', edge_factory=None):
        self.helm, self.leaders = helm, leaders  # leaders: the Leaders workspace holding saved ranges.
        self.profile_path = Path(profile_path).resolve()
        self.profile = load_profile(self.profile_path)
        self.routes_path = routes_path(runs, helm.asset_id)
        self.core_factory = core_factory  # Tests: run a core in a thread instead of a process.
        self.leader_factory = leader_factory  # Previews/tests: 'module:function' instead of USB leaders.
        if target not in self.TARGETS:
            raise ValueError(f'The loop runs on one of {", ".join(self.TARGETS)}')
        self.target, self.edge_factory = target, edge_factory  # edge_factory: tests build the EdgeWorker.
        self.receiver = {'mode': 'virtual'}  # Or live on one arm: {'mode': 'live', 'arm': 'left'}.
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._conn = self._views = self._worker = self._reader = None
        self._replies, self._ids = {}, itertools.count(1)
        self._reply_ready = threading.Condition(self._lock)
        stopped = {'phase': 'stopped', 'message': 'Teleop is not running. Choose the receiver, then start a session.',
                   'fault': None, 'state': None, 'receiver': None, 'input_error': None, 'late_targets': 0,
                   'collision': {'mode': self.profile['receiver'].get('collision_mode', 'enforced')},
                   'cycle_ms': None, 'joints': []}
        self._view, self._guide = stopped, None

    @property
    def active(self):
        worker = self._worker
        return worker is not None and worker.is_alive()

    def _bundle(self):
        helm = self.helm
        return {'calibration_profile': str(helm.profile_path), 'bindings': deepcopy(helm.bindings),
                'directions': deepcopy(helm.mapping), 'limits': deepcopy(helm.saved_limits),
                'teleop_profile': str(self.profile_path), 'routes_path': str(self.routes_path),
                'asset_id': helm.asset_id, 'leader_factory': self.leader_factory, 'receiver': dict(self.receiver),
                'leader_ranges': deepcopy(self.leaders.saved) if self.leaders is not None else None,
                'servo': self._servo()}

    def _servo(self):
        """The saved servo calibration (Helm's Tune tab), if any: the receiver takes its settings from it."""
        from robot_platform.control.servo_calibration import load_servo, servo_path
        if not hasattr(self.helm, 'profile_path') or not hasattr(self.helm, 'asset_id'):
            return None
        return load_servo(servo_path(self.helm.profile_path, self.helm.asset_id))

    def set_target(self, target):
        if target not in self.TARGETS:
            raise ValueError(f'The loop runs on one of {", ".join(self.TARGETS)}')
        with self._lock:
            if self.active:
                raise ValueError('Stop the session before choosing where the loop runs')
            if target != 'jetson' and self.receiver['mode'] == 'live':
                raise ValueError("The live receiver runs on Minerva's Jetson; choose the virtual receiver first")
            self.target = target

    def set_receiver(self, mode, arm=None):
        """The virtual receiver, or the live one driving one arm's motors (Minerva's Jetson only)."""
        if mode not in ('virtual', 'live') or (mode == 'live') != (arm in ('left', 'right')):
            raise ValueError('Choose the virtual receiver, or live on the left or right arm')
        with self._lock:
            if self.active:
                raise ValueError('Stop the session before choosing the receiver')
            if mode == 'live' and ('live' not in self.profile or self.target != 'jetson'):
                raise ValueError("Live teleop needs the profile's live block and the loop on Minerva's Jetson")
            if mode == 'live' and getattr(self.helm, 'connection', 'tether') != 'tether':
                raise ValueError('Live teleop runs over the wired tether: choose Tether at the top first')
            self.receiver = {'mode': 'live', 'arm': arm} if mode == 'live' else {'mode': 'virtual'}

    def _edge_bundle(self):
        """The bundle for the robot: release-relative profiles, and route edits Helm keeps on this PC."""
        from robot_platform.ops.release import ROOT
        bundle = self._bundle()
        for key in ('calibration_profile', 'teleop_profile'):
            path = Path(bundle[key]).resolve()
            if not path.is_relative_to(ROOT):
                raise ValueError(f'{path.name} is not part of the robot release')
            bundle[key] = path.relative_to(ROOT).as_posix()
        bundle.update(routes_path=None, leader_factory=None,
                      route_edits=load_route_edits(self.routes_path, self.profile, self.helm.asset_id))
        if self.receiver['mode'] == 'live':
            # Motor models live in the hardware profiles, which stay on this PC.
            from robot_platform.control.commissioning import motor_models
            bundle['receiver'] = dict(self.receiver, motor_models=motor_models(self.helm.device, ROOT))
        return bundle

    def start(self, *, confirm=False):
        with self._lock:
            if self.active:
                raise ValueError('Teleop is already running')
            if self.receiver['mode'] == 'live' and getattr(self.helm, 'connection', 'tether') != 'tether':
                raise ValueError('Live teleop runs over the wired tether: choose Tether at the top first')
            if self.receiver['mode'] == 'live' and confirm is not True:
                raise ValueError(f"Confirm the live session: the {self.receiver['arm']} arm's motors will be powered, "
                                 'the foot switch is in reach and the arm hangs in its reference pose')
            if self.helm.saved_limits is None:
                raise ValueError('Save joint limits for every motor in the Limits workspace first')
            if self.leaders is not None:
                missing = self.leaders.missing(self.profile['routes'])
                if missing:
                    raise ValueError('Sweep and save the leader ranges in the Leaders tab first; missing: '
                                     + ', '.join(missing))
            local = self.target == 'pc' or self.core_factory is not None
            bundle = self._bundle() if local else self._edge_bundle()
            if local:
                context = multiprocessing.get_context('spawn')
                # One-way pipes: commands to the loop, state back from it.
                command_reader, command_writer = context.Pipe(duplex=False)
                view_reader, view_writer = context.Pipe(duplex=False)
                if self.core_factory is None:
                    worker = context.Process(target=_process_main, args=(command_reader, view_writer, bundle),
                                             daemon=True, name='helm-teleop')
                else:
                    worker = threading.Thread(target=run_loop, daemon=True, name='helm-teleop',
                                              args=(command_reader, view_writer, self.core_factory(bundle)))
                message = 'Starting: loading the collision model and virtual receiver; opening the leaders…'
            else:
                from .edge import EdgeWorker
                leader_profile = json.loads((self.profile_path.parent / self.profile['leader_profile'])
                                            .read_text(encoding='utf-8'))
                worker = (self.edge_factory or EdgeWorker)(bundle, leader_profile, base_view=dict(self._view),
                                                           real_leaders=self.leader_factory is None,
                                                           address=getattr(self.helm, 'address', None))
                command_writer, view_reader = worker.commands, worker.views
                message = ("Starting on Minerva's Jetson: connecting, loading the collision model and virtual "
                           'receiver, linking the leaders…') if self.receiver['mode'] == 'virtual' else (
                    f"Starting LIVE on the {self.receiver['arm']} arm: checking it is still and in its reference "
                    'pose, then starting the receiver (its motors stay disabled until Idle)…')
            self._view = dict(self._view, phase='starting', fault=None, joints=[], message=message)
            self._guide = None
            self._conn, self._views, self._worker = command_writer, view_reader, worker
            worker.start()
            if local and self.core_factory is None:
                command_reader.close()  # The child owns its ends; EOF then means it exited.
                view_writer.close()
            self._reader = threading.Thread(target=self._read, args=(view_reader,), daemon=True,
                                            name='helm-teleop-reader')
            self._reader.start()

    def _read(self, conn):
        try:
            while True:
                message = conn.recv()
                with self._lock:
                    if message[0] == 'view':
                        self._view, self._guide = message[1], message[2]
                    elif message[0] == 'reply':
                        self._replies[message[1]] = message[2:]
                        self._reply_ready.notify_all()
        except (EOFError, OSError):
            with self._lock:
                if self._view.get('phase') in ('starting', 'running'):
                    self._view = dict(self._view, phase='fault', fault='Teleop process ended unexpectedly',
                                      message='Teleop stopped: the loop process ended unexpectedly.')
                self._guide = None

    def _send(self, message):
        with self._lock:
            conn = self._conn
        if conn is None or not self.active:
            raise ValueError('Start a teleop session first')
        try:
            with self._send_lock:  # Handlers and executor threads may both send.
                conn.send(message)
        except (OSError, ValueError) as exc:
            raise ValueError(f'Teleop session is not reachable: {exc}') from exc

    def key(self, key):
        if key not in KEYS:
            raise ValueError('Use I, E, T, H or X')
        if self._view.get('phase') != 'running':
            raise ValueError('Start a teleop session first')
        if self._view.get('fault'):
            raise ValueError('Teleop faulted: stop the session and start it again')
        self._send(('key', key))

    def _request(self, kind, *arguments, timeout=5.):
        request = next(self._ids)
        self._send((kind, request, *arguments))
        with self._lock:
            if not self._reply_ready.wait_for(lambda: request in self._replies, timeout):
                raise ValueError('Teleop did not answer the change in time')
            error, result = self._replies.pop(request)
        if error:
            raise ValueError(error)
        return result

    def route(self, action, motor, timeout=5.):
        edits = self._request('route', action, motor, timeout=timeout)
        if self.target == 'jetson' and self.core_factory is None:
            # The loop on the robot keeps no files; Helm saves the operator's edits here.
            save_route_edits(self.routes_path, self.profile, self.helm.asset_id, edits)

    def set_zero(self, timeout=5.):
        """The leaders' pose now becomes the zero; saved through the Leaders workspace."""
        zero = self._request('zero', timeout=timeout)
        if self.leaders is not None:
            self.leaders.store_reference(zero)
        return zero

    def stop(self, timeout=45.):
        # The loop lowers an enabled arm before it closes the receiver: seconds at the shutdown speed.
        with self._lock:
            conn, views, worker, reader = self._conn, self._views, self._worker, self._reader
        if worker is None:
            return
        try:
            with self._send_lock:
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
                    end.close()
            self._conn = self._views = self._worker = self._reader = None
            if self._view.get('phase') in ('starting', 'running'):
                self._view = dict(self._view, phase='stopped', message='Teleop stopped.')
            self._guide = None

    def snapshot(self):
        with self._lock:
            return dict(deepcopy(self._view), target=self.target, receiver_choice=dict(self.receiver),
                        live_available='live' in self.profile)

    def guide(self):
        with self._lock:
            return deepcopy(self._guide)
