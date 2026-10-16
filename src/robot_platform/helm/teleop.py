"""Helm teleoperation: the existing GUI lifecycle through the guarded receiver.

Idle (I) -> Enable (E) -> Engage/Align (T) -> Teleop; T again holds; Shutdown
(X) returns slowly to the reference; Disable (H) removes torque. The receiver is
the virtual (native harness) receiver, or, on the robot computer, the live
guarded service (control.live_receiver), with an explicit collision policy or
the dedicated collision-disabled direct profile.

The profile-timed loop runs in its own process and owns the leader USB, receiver
and any selected collision model. The web server relays keys and route edits and reads
back the latest state, so browser traffic cannot stall a control cycle.
Receiver limits come from the saved joint limits. Each leader channel's saved
(hand-swept) range spans its robot joint's usable range; route signs come from
the teleop profile plus local operator edits.
"""
from collections import deque
from copy import deepcopy
import gc
import hashlib
import itertools
import json
import multiprocessing
from pathlib import Path
import threading
import time

from robot_platform.control.guarded_config import CONTROL_LAW, arm_subset, derive_receiver, fit_motor_ranges, to_cad
from robot_platform.control.joint_calibration import save_json
from robot_platform.control.live_receiver import fault_detail
from robot_platform.control.scheduling import prefer_responsive
from robot_platform.control.plain_data import plain_copy
from .limits import utc_now
from .recording import RecordingError, RecordingWriter

# Recent cycle timings kept in memory: the most any reader uses (the view's last 100). The full trace goes to the
# latency file. A bounded deque drops one small entry per cycle; trimming 100 full trace entries at once stalled
# the Jetson loop about 3.5 ms every 100 cycles and, with one slow receiver reply, cost a 10 ms slot (2026-10-11).
CYCLE_HISTORY = 100

KEYS = frozenset('IETHX')
ENGAGED = ('ENGAGING', 'TELEOP')
HELD = ('stopping', 'stopped')  # Receiver states in which it brakes or holds the arm itself.
LOWERING_ATTEMPTS = 3  # Re-arms after the receiver stops the arm, before it is left held where it stopped.
# A re-arm the receiver refuses for these waits (it holds the arm meanwhile): a drive not answering, a joint over
# a limit, or one still moving.
WAIT_TO_REARM = ('feedback_stale', 'limit_violation', 'invalid_state')


def load_profile(path):
    return validate_profile(json.loads(Path(path).read_text(encoding='utf-8')))


def validate_profile(profile):
    """The checks every teleop profile passes, also applied to one resolved with operator settings."""
    if (profile.get('kind') != 'guarded_teleop_profile' or profile.get('backend') != 'virtual'
            or profile.get('runtime_approved') is not False):
        raise ValueError('Helm teleop needs a virtual, unapproved guarded teleop profile')
    mode = profile['receiver'].get('collision_mode', 'enforced')
    if mode not in ('enforced', 'advisory', 'disabled'):
        raise ValueError('collision_mode must be enforced, advisory or disabled')
    if mode == 'advisory' and profile['backend'] != 'virtual':
        raise ValueError('Advisory collision checks are only allowed with the virtual receiver')
    receiver = profile['receiver']
    interpolation = receiver.get('interpolation_ms', 40)
    if mode == 'disabled':
        hz, direct = receiver.get('loop_hz'), receiver.get('direct') or {}
        if (type(hz) is not int or not 50 <= hz <= 200 or 1000 % hz
                or type(interpolation) is not int or not 0 <= interpolation <= 1000 / hz
                or receiver.get('collision') is not None
                or set(direct) - {'command_ms', 'velocity_timeout_ms', 'limit_reaction_ms', 'position_only'}
                or any(type(direct.get(k)) is not int
                       for k in ('command_ms', 'velocity_timeout_ms', 'limit_reaction_ms'))
                or type(direct.get('position_only', False)) is not bool
                or not 2 * 1000 / hz < direct.get('command_ms', 0) <= receiver['lease_ms']
                or not interpolation <= direct.get('velocity_timeout_ms', 0) <= 2 * 1000 / hz
                or direct.get('velocity_timeout_ms', 0) <= 0
                or not 0 <= direct.get('limit_reaction_ms', -1) <= 1000):
            raise ValueError('Direct teleop needs explicit timing, limit reaction and no collision policy')
    elif receiver.get('direct') is not None or receiver.get('loop_hz') is not None:
        raise ValueError('Independent loop rate and direct policy require disabled collision mode')
    elif (type(interpolation) is not int or not 10 <= interpolation <= 40
            or not 0 < receiver.get('compute_budget_s', 0) <= .6 * interpolation / 1000):
        raise ValueError('The receiver interpolates over 10-40 ms (the loop period), with compute under 60% of it')
    from robot_platform.teleop.lifecycle import validate_trial
    trial = validate_trial(profile.get('trial'))
    if trial and (mode != 'disabled' or receiver['direct'].get('position_only') is not True):
        raise ValueError('A bounded trial requires explicit direct position-only mode')
    # Manual tuning runs a direct law: position-only, or (unramped) with velocity fed forward (guarded_config).
    if ('manual_tuning' in profile and (profile['manual_tuning'] is not True or trial or mode != 'disabled'
            or not (receiver['direct'].get('position_only') is True or interpolation == 0))):
        raise ValueError('Manual tuning requires direct position-only or unramped velocity feed-forward control '
                         'without a fixed trial')
    live = profile.get('live')
    if live is not None and (
            set(live) - {'reference_tolerance_rad', 'period_us', 'maximum_lateness_us', 'note'}
            or not 0 < live.get('reference_tolerance_rad', 0) <= .1
            or not 1000 <= live.get('period_us', 0) <= 10000
            or not live['period_us'] <= live.get('maximum_lateness_us', 0) <= 20000):
        raise ValueError('The live block needs a reference tolerance of at most 0.1 rad and receiver timing')
    return profile


def loop_period_s(profile):
    """Direct mode has an independent update rate; legacy modes use their ramp period."""
    receiver = profile['receiver']
    return 1 / receiver['loop_hz'] if receiver.get('collision_mode') == 'disabled' else receiver.get('interpolation_ms', 40) / 1000


def velocity_feedforward(profile):
    """Direct, unramped and not position-only: arm joints get a smoothed velocity fed forward (guarded_config)."""
    receiver = profile['receiver']
    return (receiver.get('collision_mode') == 'disabled' and receiver.get('interpolation_ms', 40) == 0
            and not (receiver.get('direct') or {}).get('position_only', False))


def control_law(direct, position_only, feedforward):
    """The receiver's control law, as guarded_config names it in each joint's report."""
    if not direct:
        return CONTROL_LAW
    return ('direct-position-only' if position_only else 'direct-velocity-feedforward' if feedforward
            else 'direct-feedforward')


def profile_law(profile):
    receiver = profile['receiver']
    return control_law(receiver.get('collision_mode') == 'disabled',
                       (receiver.get('direct') or {}).get('position_only', False), velocity_feedforward(profile))


def switched_profile(profile, manual):
    """The profile a session runs. Motor tuning's velocity feed-forward switch (helm.manual_tuning), on, takes a
    manual-tuning profile out of position-only: the receiver then accepts the smoothed velocity the lifecycle
    feeds forward (guarded_config). A session takes it at start."""
    if profile.get('manual_tuning') is True and isinstance(manual, dict) and manual.get('velocity_feedforward') is True:
        profile = deepcopy(profile)
        profile['receiver']['direct']['position_only'] = False
        validate_profile(profile)
    return profile


def source_misses(period_s):
    """At most 80 ms or ten missed samples before engaged following requires Engage again."""
    return min(10, max(1, round(.08 / period_s)))


CYCLE_BUDGET_S = .035  # A cycle not dispatched this long after it started faults the session; a slower one
# than its period only starts the next late (the receiver's lease and collision verdict still bound it).
# A live session records every receiver tick (91 kB/s for both arms at 400 Hz) up to this much, about 90
# minutes; its events go on being logged.
RECORDING_BYTES = 512 * 1024 * 1024
RECORDING_FLUSH_S = 1.


def routes_path(runs, asset_id):
    return Path(runs).resolve() / (asset_id + '-teleop') / 'routes.json'


def routes_sha256(profile):
    """The digest of a profile's route definitions."""
    return hashlib.sha256(json.dumps(profile.get('routes') or {}, sort_keys=True, separators=(',', ':'))
                          .encode()).hexdigest()


def load_route_edits(path, profile, asset_id):
    """Operator route edits ({motor: {sign, verified, history}}) saved by Helm for this robot and these route
    definitions. Profiles with the same routes share them (the position-only and velocity feed-forward tuning
    profiles run from one folder), so a sign verified in one Helm holds in the other. A file saved without the
    routes' digest is taken only by the profile that saved it."""
    path = Path(path)
    if not path.is_file():
        return {}
    data = json.loads(path.read_text(encoding='utf-8'))
    shared = data.get('routes_sha256') == routes_sha256(profile)
    if data.get('asset_id') != asset_id or not (shared or data.get('profile') == profile['id']):
        raise ValueError(f'{path} belongs to another teleop profile')
    # Trigger edits from the old released-opens mapping ('squeeze_sign') meant
    # something else; those grippers need verifying again.
    return {motor: edit for motor, edit in data.get('routes', {}).items() if 'squeeze_sign' not in edit}


def save_route_edits(path, profile, asset_id, edits):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    save_json(path, {'schema': 1, 'profile': profile['id'], 'routes_sha256': routes_sha256(profile),
                     'asset_id': asset_id, 'runtime_approved': False, 'routes': edits})


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

    def compile(self, offered, feedback, joints=None, *, started_at=None, hold=False):
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
                 open_simulation=None, clock=time.perf_counter):
        self.bundle, self.clock = deepcopy(bundle), clock
        self.base_profile = self.profile = load_profile(bundle['teleop_profile'])
        if bundle.get('operator_settings') is not None:
            # Operator settings only tighten the profile's timing (helm.settings); the receiver is derived from
            # the result, and build() takes the lifecycle's speeds and freshness from them.
            from .settings import resolve_profile
            self.profile = resolve_profile(self.base_profile, bundle['operator_settings'])
        self.profile = switched_profile(self.profile, bundle.get('manual_tuning'))
        self.period_s = loop_period_s(self.profile)
        self.collision_mode = self.profile['receiver'].get('collision_mode', 'enforced')
        receiver = bundle.get('receiver') or {'mode': 'virtual'}
        self.live_arm = receiver.get('arm') if receiver.get('mode') == 'live' else None
        if receiver.get('mode') not in ('virtual', 'live') or (receiver['mode'] == 'live'
                                                                 and self.live_arm not in ('left', 'right', 'both')):
            raise ValueError('The receiver is virtual, or live on the left, right or both arms')
        if self.profile.get('trial') and self.live_arm and (
                self.live_arm == 'both' or not self.profile['trial']['motor'].startswith(self.live_arm + '_')):
            raise ValueError('The trial permits only its selected single arm')
        # Both arms in one receiver need enforced checks of one arm against the other: boxes the receiver checks
        # against the motion commanded (MotionEnvelopeCompiler). The virtual receiver can rehearse them.
        if self.collision_mode != 'disabled' and (self.live_arm == 'both' or receiver.get('collision') == 'motion'):
            self.collision_mode = 'motion'
        if self.collision_mode == 'disabled' and receiver.get('collision') is not None:
            raise ValueError('Direct teleop cannot request a collision bound')
        self.open_receiver = open_receiver or self._open_receiver
        # Helm started with --sim: the virtual receiver drives a simulated Minerva (simulation.minerva_sim) whose
        # joints move under the receiver's commands, instead of joints that follow them exactly.
        self.simulation = bundle.get('simulation') if receiver.get('mode') == 'virtual' else None
        self.open_simulation = open_simulation or self._open_simulation
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
        self.message = (f'Checking {self._the_arm}, then starting the live receiver…' if self.live_arm
                        else 'Loading the simulated Minerva and its window; opening the leaders…' if self.simulation
                        else 'Loading the collision model and virtual receiver; opening the leaders…')
        self.derived = self.session = self.mapper = self.leaders = self.harness = self.compiler = None
        self.scales = {}
        self.fault = self.input_error = self.last_target = None
        # After a fault the receiver holds the arm: it is taken back and lowered (the lifecycle's shutdown).
        self._lowering, self._lowerings = False, 0
        self._land = False  # Closing: shut down (lower, then disable) before the receiver closes.
        self.late_targets, self.sequence, self.overruns = 0, 0, 0
        self._shown_block = None  # The hold on screen: why a move was not certified.
        self.slowest_ms = self.largest_gap_ms = 0.
        self._last_start = None
        self.cycles = deque(maxlen=CYCLE_HISTORY)
        self._closers = []
        self._frozen = False
        self._gc_enabled_before_direct = None
        self.scheduling = {}
        # Live evidence beside the receiver's own (_start_recording).
        self._samples = self._events = self._logged = None
        self._latency = None
        self._recorder = None
        self._recording_error = None
        self.task_capture = None
        self._capture_exchange_started = None
        self._pending_latency = None
        self._loop_timings = deque(maxlen=CYCLE_HISTORY)
        self._loop_max_ms = self._loop_misses = self._loop_skipped = 0
        self._input_timing = {}
        self._recorded, self._flushed = 0, None
        self._manual_state = None
        self._manual_rows = None
        self._tuning_pending = self._tuning_result = None
        self._tuning_confirm = None

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

    def _open_simulation(self, derived, model, reference):
        from robot_platform.simulation.minerva_sim import open_simulation
        directory = Path(self.simulation['directory']) / time.strftime('sim-%Y%m%dT%H%M%SZ', time.gmtime())
        receiver = open_simulation(derived, model=model, directions=self.bundle['directions'],
                                   motors=reference['motors'], servo=self.bundle.get('servo'), directory=directory,
                                   clock=self.clock, viewer=self.simulation.get('viewer', True),
                                   seed=self.simulation.get('seed', 1), actuators=self.simulation.get('actuators'),
                                   task_scene=self.simulation.get('task_scene', False))
        self.scheduling['receiver'] = prefer_responsive(receiver.harness.process.pid)
        return receiver

    def _open_receiver(self, config, device, reference):
        from robot_platform.control.live_receiver import LiveReceiver, service_config
        from robot_platform.ops.release import ROOT
        live = self.profile['live']
        used = {motor['bus'] for motor in reference['motors']}
        buses = [b['interface'] for b in device['buses'] if b['interface'] in used]
        service = service_config(config, device, reference, self.bundle['receiver']['motor_models'],
                                 buses=buses, tolerance_rad=live['reference_tolerance_rad'],
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
        from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler, MotionEnvelopeCompiler
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
        if self.live_arm and 'live' not in self.profile:
            raise ValueError('Live teleop needs the profile\'s live block')
        if self.live_arm in ('left', 'right'):
            # One arm on physical motors: the other arm is not in this receiver and is never enabled. Its checks
            # against the other arm (parked at its reference) are advisory.
            if self.collision_mode not in ('advisory', 'disabled'):
                raise ValueError('Live teleop of one arm needs advisory collision checks in the profile')
            reference, steps, transmissions = arm_subset(reference, steps, transmissions, self.live_arm)
        backend = None
        if self.collision_mode != 'disabled':
            backend = self.open_backend(model)
            self._closers.append(backend.close)
        derived = derive_receiver(reference=reference, steps=steps, directions=self.bundle['directions'],
                                  transmissions=transmissions, model=model, limits=self.bundle['limits'],
                                  profile=self.profile, backend=backend, servo=self.bundle.get('servo'),
                                  collision_bound='motion' if self.collision_mode == 'motion' else None)
        if self.profile.get('manual_tuning') is True:
            from .manual_tuning import receiver_rows, validate, install_initial
            from robot_platform.control.commissioning import motor_models
            from robot_platform.ops.release import ROOT
            from robot_platform.teleop.lifecycle import direct_law
            if self.profile.get('trial') or not derived['config'].get('direct') or not direct_law(derived['lifecycle']):
                raise ValueError('Manual tuning requires direct position-only or velocity feed-forward control '
                                 'without a fixed trial')
            models = (self.bundle.get('receiver') or {}).get('motor_models') or motor_models(device, ROOT)
            self._manual_rows = receiver_rows(derived, models, self.labels)
            selected = deepcopy(self.bundle.get('manual_tuning'))
            if (not isinstance(selected, dict) or set(selected) - {'velocity_feedforward'} != {'session_id', 'revision', 'joints'}
                    or type(selected.get('velocity_feedforward', False)) is not bool
                    or not isinstance(selected['session_id'], str) or not selected['session_id']
                    or type(selected['revision']) is not int or selected['revision'] < 0):
                raise ValueError('Manual tuning needs a versioned complete operator preset')
            names = {r['motor'] for r in self._manual_rows}
            rows = validate([r for r in selected['joints'] if r['motor'] in names], self._manual_rows)
            install_initial(derived, rows)
            self._manual_state = selected
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
            self._start_recording(harness, derived['config'])
            def exchange(request):
                self._capture_exchange_started = self.clock()
                return harness.exchange(request)
        elif self.simulation is not None:
            # The simulated arms tick with the receiver, as the live service does; their ticks are recorded as a
            # live session's are.
            harness = self.open_simulation(derived, model, reference)
            self._closers.append(harness.close)
            self.task_capture = getattr(harness, 'task_view', None)
            self._start_recording(harness, derived['config'])
            actuators = getattr(harness, 'actuators', None)
            self._event('simulation', nominal=harness.nominal, **({'actuators': actuators} if actuators else {}))
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
        budget = self.profile['receiver'].get('compute_budget_s')
        # Model joints outside this receiver (the other arm, if one is live) stay at the reference pose.
        fixed = {name: pose[name] for name in pose if name not in derived['joint_map']}
        if self.collision_mode == 'disabled':
            compiler = None
        elif self.collision_mode == 'advisory':
            compiler = AdvisoryCompiler(backend, derived['config'], derived['joint_map'], fixed=fixed,
                                        cad_limits=derived['cad_limits'], compute_budget_s=budget)
        elif self.collision_mode == 'motion':
            # Enforced: a move whose box could bring the arms together is held (GuardedLifecycleSession).
            compiler = MotionEnvelopeCompiler(backend, derived['config'], derived['joint_map'], fixed=fixed,
                                              cad_limits=derived['cad_limits'], compute_budget_s=budget,
                                              clock=self.clock)
        else:
            compiler = CollisionEnvelopeCompiler(backend, derived['config'], derived['joint_map'],
                                                 compute_budget_s=budget, cad_limits=derived['cad_limits'])
        rows, source = derived['lifecycle'], {'max_source_misses': source_misses(self.period_s)}
        if self.bundle.get('operator_settings') is not None:
            from .settings import validate_bundle
            rows, source = validate_bundle(self.base_profile, derived, self.bundle['operator_settings'],
                                           self._manual_state)
        lifecycle = TeleopLifecycle(rows, clock=self.clock, **source,
                                    trial=self.profile.get('trial'),
                                    allow_repeated_source=self.collision_mode == 'disabled')
        if self._manual_state is not None:
            names = set(lifecycle.names)
            lifecycle.configure_manual([r for r in self._manual_state['joints'] if r['motor'] in names])
        if self.collision_mode == 'disabled':
            from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
            session = DirectLifecycleSession(controller, derived['config'], lifecycle, lambda: harness.last,
                                             period_s=self.period_s, cycle_budget_s=CYCLE_BUDGET_S, clock=self.clock)
        else:
            session = GuardedLifecycleSession(controller, compiler, lifecycle, lambda: harness.last,
                                              period_s=self.period_s, cycle_budget_s=CYCLE_BUDGET_S, clock=self.clock)
        self.derived, self.harness, self.session, self.compiler = derived, harness, session, compiler
        self.leaders = self.open_leaders(leader_profile)
        if self.collision_mode == 'disabled' and hasattr(self.leaders, 'allow_repeated'):
            self.leaders.allow_repeated = True
        self._closers.append(self.leaders.close)
        self.mapper = self._mapper()
        # Full garbage-collector passes over the CAD/collision object graphs took
        # 30+ ms, longer than a cycle's budget. Freeze everything long-lived so
        # later passes only walk new objects; reference counting still frees.
        gc.collect()
        gc.freeze()
        self._frozen = True
        if self.collision_mode == 'disabled':
            # Even new-object generation scans exceeded the 10 ms direct-loop
            # budget on Jetson. This dedicated process keeps reference counting
            # active; restore cyclic collection only after its resources close.
            self._gc_enabled_before_direct = gc.isenabled()
            gc.disable()
        self.phase = 'running'
        self.message = (f'{self._receiver_name} · motors are enabled from Idle (I). Keep the foot '
                        'switch in reach; then Enable (E) and Engage (T).' if self.live_arm else
                        'VIRTUAL RECEIVER · no motors move. Idle (I) first, then Enable (E) and Engage (T).')

    def close(self):
        failures = []
        try:
            if self.session is not None:
                try:
                    self.session.disable()
                except Exception:
                    try:
                        self.session.fault('helm_teleop_closed')
                    except Exception as exc:
                        failures.append(str(exc) or type(exc).__name__)
            for close in reversed(self._closers):
                try:
                    close()
                except Exception as exc:
                    failures.append(str(exc) or type(exc).__name__)
            self._closers = []
            # Wait for evidence only after successful resource cleanup. If any
            # teardown is unconfirmed, initiate recorder close without waiting.
            try:
                self._stop_recording(timeout=0. if failures else 5.)
            except Exception as exc:
                reason = str(exc) or type(exc).__name__
                failures.append(reason)
                self.fail(self.fault or reason)
        finally:
            try:
                if self._frozen:
                    gc.unfreeze()
                    self._frozen = False
            finally:
                previous, self._gc_enabled_before_direct = self._gc_enabled_before_direct, None
                if previous is not None:
                    (gc.enable if previous else gc.disable)()
        if failures and self.phase != 'fault':
            self.fail(self.fault or 'Teleop cleanup could not be confirmed')
        if self.phase != 'fault':
            self.phase = 'stopped'
            self.message = (f'Teleop stopped. The live receiver disabled {self._the_arm} and its CAN link is down.'
                            if self.live_arm else 'Teleop stopped. The virtual receiver is closed.')
        if failures:
            self.message += ' Closing reported: ' + '; '.join(failures)

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason
        self.message = f'Teleop stopped: {reason}'

    # -- cycle -------------------------------------------------------------

    def _recent_cycle_s(self):
        durations = sorted(c['completed_at'] - c['started_at'] for c in itertools.islice(reversed(self.cycles), 25)
                           if c.get('completed_at') is not None)
        return durations[int(.9 * (len(durations) - 1))] if durations else 0.

    def _leader_target(self, engaged):
        from robot_platform.teleop.leader_routes import unwrap_samples
        if self.mapper.fault and not engaged:
            self.mapper.reset()  # Leaders only preview outside engagement.
        try:
            polled = self.clock()
            samples = unwrap_samples(self.leaders.poll(), self.centers)
            mapped = self.clock()
            samples = {name: sample for name, sample in samples.items() if name in self.route_leaders}
            repeated = False
            if self.collision_mode == 'disabled':
                target = self.mapper.map(samples, now=self.clock(), allow_repeated=True)
                if target is None:
                    # The mapper has validated the cached payload and original
                    # age. Reuse only that observed goal, never a failed poll.
                    target = deepcopy(self.last_target)
                    repeated = True
            else:
                target = self.mapper.map(samples, now=self.clock())
        except Exception as exc:
            return None, str(exc) or type(exc).__name__
        self.last_target = target
        self._input_timing = {'poll_ms': (mapped-polled)*1000, 'map_ms': (self.clock()-mapped)*1000,
                              'source_age_at_map_ms': (self.clock()-min(row['sampled_at'] for row in target))*1000}
        if repeated:
            self._input_timing['repeated_source'] = True
        margin = max(self._recent_cycle_s(), .01)
        age = self.clock() - min(row['sampled_at'] for row in target)
        if age + margin >= self.session.lifecycle.target_max_age_s:
            self.late_targets += 1
            return None, None
        return target, None

    def queue_tuning(self, payload):
        from .manual_tuning import validate
        if (self._manual_state is None or self.phase != 'running' or self.fault
                or self.session.lifecycle.state != 'HOLD' or self._tuning_pending is not None
                or self._tuning_confirm is not None):
            raise ValueError('Hold the running arm before applying manual tuning')
        if (not isinstance(payload, dict) or set(payload) - {'velocity_feedforward'} != {'session_id', 'revision', 'joints'}
                or payload['session_id'] != self._manual_state['session_id']
                or payload['revision'] != self._manual_state['revision']):
            raise ValueError('Tuning revision changed; review the current settings')
        if payload.get('velocity_feedforward', False) is not self._manual_state.get('velocity_feedforward', False):
            raise ValueError('Velocity feed-forward switches at the next session start; stop the session to change it')
        known = [r['motor'] for r in self._manual_state['joints']]
        if [r.get('motor') for r in payload['joints']] != known:
            raise ValueError('The tuning motor roster changed')
        names = set(self.session.lifecycle.names)
        selected = validate([r for r in payload['joints'] if r['motor'] in names], self._manual_rows)
        feedback = (self.harness.last or {}).get('feedback') or []
        settled = self.derived['config']['settled_velocity']
        if (len(feedback) != len(names)
                or any(type(row.get('velocity')) not in (int, float)
                       or not abs(row['velocity']) <= settled for row in feedback)):
            raise ValueError('The motor feedback is not settled. Wait in Hold before applying; existing settings are unchanged.')
        lifecycle = self.session.lifecycle
        before = plain_copy(lifecycle._joints), plain_copy(lifecycle._manual), lifecycle._manual_hold
        try:
            lifecycle.configure_manual(selected)  # Validate changed windows before entering the control transaction.
        finally:
            lifecycle._joints, lifecycle._manual, lifecycle._manual_hold = before
        self._tuning_pending = plain_copy(payload)
        self._tuning_result = None

    # -- evidence ----------------------------------------------------------

    def _start_recording(self, receiver, config):
        """A live session's evidence beside the receiver's own: every receiver tick (samples.f32, float32 rows
        as columns.json names them) and every key, lifecycle state, fault and message (events.jsonl). The
        receiver's -last.json keeps its final two ticks only: what tripped Engage on 2026-10-09 could not be told."""
        import numpy as np
        from robot_platform.control.servo_sweep import recorder_columns, result_rows
        transport, directory = getattr(receiver, 'transport', None), getattr(receiver, 'directory', None)
        if transport is None or directory is None:
            return
        directory = Path(directory)
        names = [joint['name'] for joint in config['joints']]
        if self.live_arm and self.bundle.get('task_capture') is not None:
            if self.live_arm != 'both':
                raise ValueError('Task capture requires both arms and both grippers')
            from .task_capture import LiveTaskCapture
            self.task_capture = LiveTaskCapture(directory / 'episodes', names, self.bundle['task_capture'],
                metadata={'source': 'real', 'asset_id': self.bundle['asset_id'],
                          'bindings': self.bundle['bindings'], 'receiver_identity': config['identity'],
                          'action_semantics': 'receiver_emitted_position_at_sample'})
            self._closers.append(self.task_capture.close)
        (directory / 'columns.json').write_text(json.dumps(
            {'dtype': 'float32', 'columns': recorder_columns(names)}, indent=1) + '\n', encoding='utf-8')
        self._recorder = RecordingWriter(directory)
        self._samples = self._recorder.stream('samples.f32', binary=True)
        self._events = self._recorder.stream('events.jsonl')
        if self.collision_mode == 'disabled':
            self._latency = self._recorder.stream('direct-latency.jsonl')

        def on_result(result):
            if self._samples is None:
                return
            rows, dropped = result_rows(result, names)
            if self.live_arm and self.task_capture is not None:
                self.task_capture.receive(result, rows, dropped=dropped,
                    started_at=self._capture_exchange_started, received_at=self.clock())
            if dropped:
                self._event('rows_dropped', count=dropped)
            if rows and self._recorded < RECORDING_BYTES:
                data = np.asarray(rows, dtype=np.float32).tobytes()
                try:
                    self._samples.write(data)
                except RecordingError as exc:
                    # A recorder callback may also run during disable. Latch its
                    # error for the loop, never interrupt the receiver stop RPC.
                    self._recording_error = str(exc)
                    return
                self._recorded += len(data)
                if self._recorded >= RECORDING_BYTES:
                    self._event('recording_full', bytes=self._recorded)
        transport.on_result = on_result
        self._event('session', receiver=self.live_arm, collision=self.collision_mode, joints=names)

    def _event(self, kind, **fields):
        if self._events is not None:
            try:
                self._events.write(json.dumps({'t': self.clock(), 'at': utc_now(), 'kind': kind, **fields},
                                              default=str) + '\n')
                self._events.flush()
            except RecordingError as exc:
                self._recording_error = str(exc)

    def _note(self):
        """Log what changed this cycle (lifecycle state, fault, message, a hold) and flush the ticks now and then."""
        if self._events is None:
            return
        status = ((self.harness.last or {}).get('status') or {}) if self.harness is not None else {}
        now = (self.session.lifecycle.state if self.session else None, self.fault, self.message,
               self.session.blocked if self.session else None, status.get('state'), status.get('reason'))
        if now != self._logged:
            self._logged = now
            self._event('state', lifecycle=now[0], fault=now[1], message=now[2], blocked=now[3], receiver=now[4],
                        reason=now[5])
        clock = self.clock()
        if self._samples is not None and (self._flushed is None or clock - self._flushed >= RECORDING_FLUSH_S):
            try:
                self._samples.flush()
                if self._latency is not None:
                    self._latency.flush()
            except RecordingError as exc:
                self._recording_error = str(exc)
            self._flushed = clock

    def task_action(self, action):
        if self.task_capture is None:
            raise ValueError('Set up the task before starting this session')
        state = self.session.lifecycle.state if self.session else None
        if action == 'reset' and (self.live_arm or state not in ('DISABLED', 'IDLE')):
            raise ValueError('Reset is only available with the simulated receiver disabled or idle')
        if action == 'start_record' and (self.fault or state not in ('HOLD', 'TELEOP')):
            raise ValueError('Hold or engage both arms before recording')
        return self.task_capture.command(action)

    def _check_recording(self):
        if self.task_capture is not None:
            task = self.task_capture.snapshot()
            recording = task.get('recording') or {}
            if task.get('failed') or (task.get('error') and (recording.get('active')
                    or recording.get('state') in ('failed', 'aborted'))):
                raise RecordingError(task.get('error') or 'Task capture failed')
        if self._recording_error is not None:
            raise RecordingError(self._recording_error)
        if self._recorder is not None:
            self._recorder.check()

    def _stop_recording(self, *, timeout=5.):
        recorder, self._recorder = self._recorder, None
        if recorder is None:
            return
        try:
            self._flush_latency()
            self._event('closed', recorded_bytes=self._recorded)
        finally:
            self._samples = self._events = self._latency = None
            # Called by close only after receiver cleanup, never by tick.
            recorder.close(timeout=timeout)
        self._check_recording()

    # -- cycle (continued) -------------------------------------------------

    def tick(self, key, started):
        # Persist the previous, finalized loop record inside this cycle's measured
        # workload. Its full-loop timestamp is only known after UI construction.
        self._check_recording()
        self._flush_latency()
        self._check_recording()
        if key:
            self._event('key', key=key, lifecycle=self.session.lifecycle.state if self.session else None)
        try:
            self._tick(key, started)
        finally:
            self._note()
        self._check_recording()

    def _tick(self, key, started):
        from robot_platform.teleop.guarded_lifecycle import minerva_action
        self._input_timing = {}
        if self.fault:
            # The receiver stopped itself and revoked the lease. It holds the arm: take it back there and
            # lower it, keeping the original reason on screen; a new session is needed to follow again.
            if key == 'H':
                self._lowering = False
                try:
                    self.session.disable_now()
                except Exception as exc:
                    self.session.stop_error = str(exc) or type(exc).__name__
                    self.message = (f'{self._receiver_name} FAULT · {self.fault} · Disable could not be confirmed: '
                                    f'{self.session.stop_error}. Use the foot switch; the session remains faulted.')
                else:
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
            tuning = self._tuning_pending
            if tuning is not None and (action is not None or previous != 'HOLD'):
                self._tuning_result = ('Control state changed; tuning was not applied', None)
                self._tuning_pending = tuning = None
            if tuning is None:
                record = self.session.cycle(action=action, target=target, started_at=started)
            else:
                names = set(lifecycle.names)
                record = self.session.cycle(action=action, target=target, started_at=started,
                    tuning=[r for r in tuning['joints'] if r['motor'] in names])
                if record.get('tuning_rejected'):
                    self._tuning_result = (record['tuning_rejected'], None)
                else:
                    self._tuning_confirm = (dict(tuning, revision=tuning['revision'] + 1), record['request_sequence'])
                self._tuning_pending = None
            if self._tuning_confirm is not None:
                candidate, sequence = self._tuning_confirm
                applied = (self.harness.last or {}).get('status', {}).get('last_applied_target') or {}
                if applied.get('sequence', -1) >= sequence:
                    self._manual_state = candidate
                    self._tuning_result = (None, deepcopy(candidate))
                    self._tuning_confirm = None
                    self._event('manual_tuning_applied', settings=candidate, applied_target=applied)
        except Exception as exc:
            if self._tuning_pending is not None or self._tuning_confirm is not None:
                self._tuning_pending = self._tuning_confirm = None
                self._tuning_result = (f'Tuning application could not be confirmed: {exc}. Restart the session.', None)
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
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · The receiver braked {self._the_arm} '
                                'to a hold; lowering to the reference, then disabling…')
            else:
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · The receiver stopped itself'
                                + (f' and disabled {self._the_arm}' if self.live_arm else '')
                                + '. Stop the session and start it again.')
            return
        self._record(record)
        self.sequence += 1
        current = lifecycle.state
        self._powered = self._powered or current != 'DISABLED'
        blocked = record.get('blocked')
        if blocked and self._shown_block != blocked.split(' (')[0]:
            # Held, not faulted: the arms follow again once the leaders part.
            self._shown_block = blocked.split(' (')[0]
            self.message = (f'{self._receiver_name} · {current} · HOLDING: {blocked}. Move the leaders apart; the '
                            'arms follow again once their path is clear.')
        elif not blocked and self._shown_block:
            self._shown_block = None
            self.message = f'{self._receiver_name} · {current} · following again.'
        if notice:
            self.message = notice
        elif current != previous:
            self.message = f'{self._receiver_name} · {current}'
            if current == 'HOLD' and engaged and lifecycle.reason:
                self.message += f' · {lifecycle.reason} · last input problem: {self.input_error}'
            if current == 'DISABLED' and self.live_arm:
                self.message += (f' · {self._the_arm} {"are" if self.live_arm == "both" else "is"} limp. Stop the '
                                 'session; a new one checks the reference pose again.')

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
                    self.message = (f'{self._receiver_name} FAULT · {self.fault} · {self._set_aside()}Re-armed '
                                    'where the receiver held it; lowering the arm to the reference…')
                return
            record = session.cycle(action=None, target=None, started_at=started)
            self._record(record)
            if session.lifecycle.state == 'DISABLED':
                self._lowering = False
                aside = self._set_aside()
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · {aside}The arm was lowered to the '
                                'reference and disabled. ' + ('Check that drive\'s power and wiring, then stop the '
                                                             'session and start it again.' if aside else
                                                             'Stop the session and start it again.'))
        except Exception as exc:
            self._record(session.last_cycle or {}, failure=str(exc))
            if getattr(exc, 'reason', None) in WAIT_TO_REARM and self._held():
                dropped = (self.harness.last or {}).get('hardware_dropped') if self.harness else None
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · ' + (
                    f'{dropped}. The receiver holds the arm on the other drives and cannot re-arm it this session. '
                    'Support the arm, then press H to disable it (it goes limp).' if dropped else
                    f'The receiver holds the arm; waiting to re-arm it ({exc.reason}). If it stays held, support '
                    'the arm, then press H to disable it (it goes limp).'))
            if not self._held():  # The receiver cut the arm's torque: nothing left to lower.
                self._lowering = False
                self.message = (f'{self._receiver_name} FAULT · {self.fault} · Lowering stopped: {exc}. '
                                'Stop the session and start it again.')

    def _set_aside(self):
        """The grippers the receiver set aside (they stopped answering; the arm is lowered without them), as the
        start of a sentence, or ''."""
        names = (((self.harness.last or {}).get('status') or {}).get('set_aside') or []) if self.harness else []
        if not names:
            return ''
        labels = ' and '.join(self.labels.get(name, name) for name in names)
        return f'{labels} stopped answering and {"is" if len(names) == 1 else "are"} set aside. '

    def helm_gone(self, reason):
        """Helm closed the channel or went silent (run_loop) without asking to stop; the loop lands and closes."""
        self._event('helm_gone', reason=reason)

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
        if self.live_arm == 'both':
            return 'LIVE BOTH ARMS'
        return (f'LIVE {self.live_arm.upper()} ARM' if self.live_arm
                else 'SIMULATED MINERVA' if self.simulation else 'VIRTUAL RECEIVER')

    @property
    def _the_arm(self):
        return 'the arms' if self.live_arm == 'both' else 'the arm'

    def _record(self, record, failure=None):
        entry = {k: record.get(k) for k in ('started_at', 'completed_at', 'accepted', 'action')}
        if self.collision_mode == 'disabled':
            from robot_platform.teleop.latency import trace_record
            entry.update(trace_record(record), **self._input_timing)
            self._pending_latency = entry
        if failure:
            entry['failure'] = failure
        if entry['started_at'] is not None and entry['completed_at'] is not None:
            duration = (entry['completed_at'] - entry['started_at']) * 1000
            self.slowest_ms = max(self.slowest_ms, duration)
            if duration > self.period_s * 1000:
                self.overruns += 1
        self.cycles.append({'started_at': entry['started_at'], 'completed_at': entry['completed_at']})

    def _flush_latency(self):
        entry, self._pending_latency = self._pending_latency, None
        if entry is not None and self._latency is not None and self._recorded < RECORDING_BYTES:
            encoded = json.dumps(entry, allow_nan=False) + '\n'
            try:
                self._latency.write(encoded)
            except RecordingError as exc:
                self._recording_error = str(exc)
                return
            self._recorded += len(encoded.encode('utf-8'))

    def finish_cycle(self, started, scheduled, completed, skipped_slots=0):
        """Measure commands, prior trace write, tick/flush and UI construction on one clock.

        This runs after publish; it performs no I/O. The next tick persists it.
        The sender thread's pipe delivery and physical motor response are separate.
        """
        if self.collision_mode != 'disabled':
            return
        fields = dict(scheduled_at=scheduled, loop_completed_at=completed,
                      full_loop_ms=(completed-started)*1000,
                      start_lateness_ms=max(0., (started-scheduled)*1000),
                      scheduled_deadline_miss=completed > scheduled+self.period_s,
                      skipped_slots=skipped_slots)
        if self._pending_latency is None:
            self._pending_latency = dict(started_at=started, operation=None, accepted=False)
        self._pending_latency.update(fields)
        self._loop_timings.append(fields['full_loop_ms'])
        self._loop_max_ms = max(self._loop_max_ms, fields['full_loop_ms'])
        self._loop_misses += fields['scheduled_deadline_miss']
        self._loop_skipped += skipped_slots

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
                'badge': (f'{self._receiver_name} · {state} · foot switch in reach' if self.live_arm
                          else f'VIRTUAL FOLLOWER · {state} · no motors move'),
                'message': self.message}

    def status_view(self):
        """Cheap status and tuning ACKs, independent of pose rows and quantiles."""
        receiver = (self.harness.last or {}).get('status', {}).get('state') if self.harness else None
        verdict = getattr(self.compiler, 'last_verdict', None) or {}
        last = (self.harness.last or {}) if self.harness else {}
        result = {'phase': self.phase, 'message': self.message, 'fault': self.fault,
                  'trial': deepcopy(self.profile.get('trial')),
                  'position_only': self.profile['receiver'].get('direct', {}).get('position_only', False),
                  'velocity_feedforward': velocity_feedforward(self.profile),
                  'state': self.session.lifecycle.state if self.session else None, 'receiver': receiver,
                  'manual_tuning_ack': self._manual_state,
                  'manual_tuning_pending': self._tuning_pending is not None or self._tuning_confirm is not None,
                  'live': {'arm': self.live_arm, 'motors_enabled': last.get('hardware_enabled'),
                           'watchdog': last.get('watchdog'), 'reference': last.get('reference')}
                  if self.live_arm else None,
                  'input_error': self.input_error, 'late_targets': self.late_targets,
                  'collision': {'mode': self.collision_mode, 'clear': verdict.get('clear'),
                                'reason': verdict.get('reason'), 'distance_m': verdict.get('distance_m'),
                                'shapes': verdict.get('shapes'), 'links': verdict.get('links'),
                                'clearance_m': verdict.get('clearance_lower_bound_m'),
                                'blocked': getattr(self.compiler, 'blocked', 0),
                                'holding': self.session.blocked if self.session else None},
                  'task': self.task_capture.snapshot() if getattr(self, 'task_capture', None) is not None else None,
                  'scheduling': self.scheduling, 'cycle_ms': None, 'joints': []}
        if self.profile.get('trial'):
            trial = self.profile['trial']
            result['message'] = (f"TRIAL: {trial['motor']} only, {trial['max_velocity']:g} rad/s, "
                                 f"±{trial['max_excursion_rad']:g} rad from Enable; other joints hold. "
                                 + self.message)
        return result


    def view(self):
        result = self.status_view()
        routes = self.routes()
        durations = sorted((c['completed_at'] - c['started_at']) * 1000 for c in self.cycles
                           if c.get('completed_at') is not None)
        if durations:
            result['cycle_ms'] = {'p50': durations[(len(durations) - 1) // 2],
                                  'p99': durations[min(len(durations), -(-99 * len(durations) // 100)) - 1],
                                  'max': self.slowest_ms, 'overruns': self.overruns,
                                  'largest_gap': self.largest_gap_ms}
        if self._loop_timings:
            from robot_platform.teleop.latency import distribution
            result['full_loop_ms'] = dict(distribution(self._loop_timings), max=self._loop_max_ms,
                                         scheduled_deadline_misses=self._loop_misses,
                                         skipped_slots=self._loop_skipped)
        if self.derived is None:
            return result
        # The model's joint telemetry: the receiver's last feedback as it reported it (Helm ages it on arrival).
        from .telemetry import native_packet
        result['telemetry'] = native_packet(self.harness.last if self.harness is not None else None, self.derived,
                                            model=self.model.profile.id, urdf_sha256=self.model.sha256,
                                            joint_types=self.model.by_name, labels=self.labels,
                                            source='measured' if self.live_arm else 'simulated', stream='teleop')
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


def _helm_gone(core, exc):
    gone = getattr(core, 'helm_gone', None)
    if gone is not None:
        gone(str(exc) or type(exc).__name__)


def run_loop(commands, views, core, *, silence_s=None):
    """Serve one session: keys and route edits arrive on `commands`, state leaves on `views`.

    The control loop only drops its latest state into a slot; a sender thread does
    the (possibly blocking) pipe writes, so a slow reader drops stale views
    instead of stalling a cycle.

    With `silence_s` (on the robot, helm.edge) Helm beats, and a session that hears nothing for that long
    ends as if Helm had closed it: a dropped tether leaves the channel open, so no end of stream comes.
    """
    clock = core.clock
    keys = []
    ready = threading.Condition()
    outbox = {'view': None, 'replies': [], 'closing': False}
    cached_view = None
    published_status = None
    view_cost_s = .003
    next_view_at = None

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

    def queue_view(view, guide):
        with ready:
            outbox['view'] = (view, guide)
            ready.notify()

    def status_key(view):
        tuning = view.get('manual_tuning_ack') or {}
        # Compare immutable revision values, not a reference to a mutable payload.
        return (tuple(view.get(k) for k in ('phase', 'state', 'fault', 'message', 'receiver', 'input_error'))
                + (tuning.get('session_id'), tuning.get('revision'), view.get('manual_tuning_pending')))

    def publish(guide=True):
        nonlocal cached_view, published_status
        cached_view = core.view()
        cached_view['observed_at'] = clock()
        published_status = status_key(cached_view)
        queue_view(cached_view, core.guide() if guide else None)

    def publish_status(status):
        nonlocal published_status
        # Keep the last observed joint rows and timestamp; status and tuning ACKs
        # are current even when there is no time to rebuild observations.
        view = dict(cached_view or {}, **{k: v for k, v in status.items() if k not in ('joints', 'cycle_ms')})
        view['status_at'] = clock()
        published_status = status_key(status)
        queue_view(view, None)

    def publish_in_budget(deadline, changed):
        nonlocal next_view_at, view_cost_s
        now = clock()
        if (changed or now >= next_view_at) and deadline-now >= view_cost_s + .0005:
            publish()
            completed = clock()
            # An unforeseen spike remains in full-loop deadline accounting and
            # raises future admission cost. No command/recorder work is skipped.
            view_cost_s = max(view_cost_s, completed-now)
            next_view_at = completed + .1

    def reply(request, error, result=None):
        with ready:
            outbox['replies'].append((request, error, result))
            ready.notify()

    thread = threading.Thread(target=sender, daemon=True, name='helm-teleop-sender')
    thread.start()
    closing = False  # Asked to stop, or Helm went away: lowering the arm before the receiver closes.
    tuning_request = None
    listening = True
    try:
        publish(False)
        core.build()
        publish()
        direct = getattr(core, 'collision_mode', None) == 'disabled'
        if direct:
            from robot_platform.teleop.latency import next_scheduled_slot
        scheduled = clock()
        budget_views = direct and hasattr(core, 'status_view')
        next_view_at = scheduled + .1
        heard = clock()
        while True:
            started = clock()
            view_changed = False
            while listening:
                try:
                    if not commands.poll():
                        if silence_s is None or started - heard <= silence_s:
                            break
                        raise EOFError(f'Helm went silent for {silence_s:g} s')
                    message = commands.recv()
                    heard = clock()
                except (EOFError, OSError) as exc:
                    listening, message = False, ('stop',)  # Helm went away: stop and clean up.
                    if not closing:
                        _helm_gone(core, exc)
                if message[0] == 'beat':
                    continue  # Helm is still there.
                if message[0] == 'stop':
                    # Closing the receiver cuts torque: an enabled arm is lowered first (H still disables at once).
                    if not closing:
                        if not core.land():
                            return
                        closing = True
                    continue
                if message[0] == 'key':
                    keys.append(message[1])
                elif message[0] == 'task':
                    view_changed = True
                    _, request, action = message
                    try:
                        if closing:
                            raise ValueError('The session is shutting down')
                        reply(request, None, core.task_action(action))
                    except Exception as exc:
                        reply(request, str(exc) or type(exc).__name__)
                elif message[0] in ('route', 'zero'):
                    view_changed = True
                    _, request, *arguments = message
                    try:
                        reply(request, None, (core.route if message[0] == 'route' else core.set_zero)(*arguments))
                    except Exception as exc:
                        reply(request, str(exc) or type(exc).__name__)
                elif message[0] == 'manual_tuning':
                    view_changed = True
                    _, request, payload = message
                    try:
                        if closing or tuning_request is not None:
                            raise ValueError('A tuning change or shutdown is already in progress')
                        core.queue_tuning(payload)
                        tuning_request = request
                    except Exception as exc:
                        reply(request, str(exc) or type(exc).__name__)
            key = keys.pop(0) if keys else None
            core.tick(key, started)
            if tuning_request is not None and core._tuning_result is not None:
                error, result = core._tuning_result
                reply(tuning_request, error, result)
                tuning_request = None
            if budget_views:
                status = core.status_view()
                if key is not None or view_changed or status_key(status) != published_status:
                    publish_status(status)
                publish_in_budget(scheduled + core.period_s, view_changed)
            else:
                publish()
            completed = clock()
            if direct:
                next_start, skipped = next_scheduled_slot(scheduled, completed, core.period_s)
                core.finish_cycle(started, scheduled, completed, skipped)
            if closing and not core.landing:
                return
            scheduled = next_start if direct else started + core.period_s
            remaining = scheduled - clock()
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
                 target='pc', edge_factory=None, simulation=None, capture_profile=None):
        self.helm, self.leaders = helm, leaders  # leaders: the Leaders workspace holding saved ranges.
        self.profile_path = Path(profile_path).resolve()
        self.profile = load_profile(self.profile_path)
        self.routes_path = routes_path(runs, helm.asset_id)
        # The PC end of each Jetson session's leader link reports its request timing here (helm.edge).
        self.link_evidence = Path(runs) / (helm.asset_id + '-teleop') / 'leader-link'
        self.core_factory = core_factory  # Tests: run a core in a thread instead of a process.
        self.leader_factory = leader_factory  # Previews/tests: 'module:function' instead of USB leaders.
        if target not in self.TARGETS:
            raise ValueError(f'The loop runs on one of {", ".join(self.TARGETS)}')
        # {'viewer': bool, 'seed': int, 'actuators': the simulator's actuator file} or None: the virtual receiver
        # drives a simulated Minerva on this PC.
        self.simulation = dict(simulation) if simulation is not None else None
        if self.simulation is not None and target != 'pc':
            raise ValueError('The simulated Minerva runs on this PC')
        self.sim_directory = Path(runs) / (helm.asset_id + '-sim')
        self.capture_profile = Path(capture_profile).resolve() if capture_profile is not None else None
        self.task_configured = False
        self.task_capture_config = None
        self.target, self.edge_factory = target, edge_factory  # edge_factory: tests build the EdgeWorker.
        # Or live on one arm, or both ({'mode': 'live', 'arm': 'left' | 'right' | 'both'}); virtual may enforce the
        # two-arm bound ({'mode': 'virtual', 'collision': 'motion'}).
        self.receiver = {'mode': 'virtual'}
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
        self._view_received_at = time.perf_counter()
        self._tuning_lock = threading.Lock()
        self.manual = None
        if self.profile.get('manual_tuning') is True:
            from .manual_tuning import ManualTuning
            self.manual = ManualTuning(helm, self.profile, runs, self._servo())
        # Operator session settings (helm.settings): editable with the direct profile, a read-only catalog
        # otherwise. Helm still starts when they cannot be derived (no saved limits yet); Settings says why.
        self.settings, self.settings_error = None, None
        try:
            from .settings import OperatorSettings
            self.settings = OperatorSettings(helm, self.profile, runs, self._servo(), self.manual)
        except (ValueError, KeyError, TypeError, AttributeError, OSError) as exc:
            self.settings_error = f'Settings are unavailable: {exc}'

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
                'servo': self._servo(),
                **({'task_capture': deepcopy(self.task_capture_config)} if self.task_capture_config is not None else {}),
                **({'simulation': dict(self.simulation, directory=str(self.sim_directory))}
                   if self.simulation is not None and self.receiver['mode'] == 'virtual' else {}),
                **({'manual_tuning': self.manual.bundle()} if self.manual else {}),
                **({'operator_settings': self.settings.bundle()} if self.settings and self.settings.enabled else {})}

    def _servo(self):
        """The saved servo calibration (Helm's Tune tab), if any: the receiver takes its settings from it."""
        from robot_platform.control.servo_calibration import load_servo, servo_path
        if not hasattr(self.helm, 'profile_path') or not hasattr(self.helm, 'asset_id'):
            return None
        return load_servo(servo_path(self.helm.profile_path, self.helm.asset_id))

    def task_action(self, action):
        if action == 'setup':
            with self._lock:
                if self.active:
                    raise ValueError('Stop the session before task setup')
                if self.simulation is not None:
                    self.simulation['task_scene'] = True
                else:
                    if self.receiver != {'mode': 'live', 'arm': 'both'}:
                        raise ValueError('Select Live both arms before task setup')
                    if self.capture_profile is None:
                        raise ValueError('Live capture needs --capture-profile with verified camera identities and clocks')
                    from .task_capture import validate_config
                    self.task_capture_config = validate_config(json.loads(self.capture_profile.read_text(encoding='utf-8')))
                self.task_configured = True
            return
        if action not in ('reset', 'start_record', 'stop_record'):
            raise ValueError('Unknown task action')
        return self._request('task', action)

    def set_target(self, target):
        if target not in self.TARGETS:
            raise ValueError(f'The loop runs on one of {", ".join(self.TARGETS)}')
        with self._lock:
            if self.active:
                raise ValueError('Stop the session before choosing where the loop runs')
            if target != 'jetson' and self.receiver['mode'] == 'live':
                raise ValueError("The live receiver runs on Minerva's Jetson; choose the virtual receiver first")
            if target != 'pc' and self.simulation is not None:
                raise ValueError('The simulated Minerva runs on this PC (Helm was started with --sim)')
            self.target = target

    def set_receiver(self, mode, arm=None, collision=None):
        """The virtual receiver (`collision` 'motion': enforcing the two-arm bound), or the live one driving one arm's
        motors or both arms' (Minerva's Jetson only)."""
        if (mode not in ('virtual', 'live') or (mode == 'live') != (arm in ('left', 'right', 'both'))
                or collision not in (None, 'motion') or (collision and mode != 'virtual')):
            raise ValueError('Choose the virtual receiver (the two-arm bound enforced or not), or live on the left, '
                             'right or both arms')
        if self.simulation is not None and mode != 'virtual':
            raise ValueError('Simulation mode cannot select a physical receiver')
        with self._lock:
            if self.active:
                raise ValueError('Stop the session before choosing the receiver')
            trial = self.profile.get('trial')
            if mode == 'live' and trial and arm != trial['motor'].split('_')[0]:
                raise ValueError('Choose the arm selected by this trial profile')
            if collision and self.profile['receiver'].get('collision_mode') == 'disabled':
                raise ValueError('This direct profile has collision detection disabled')
            if mode == 'live' and ('live' not in self.profile or self.target != 'jetson'):
                raise ValueError("Live teleop needs the profile's live block and the loop on Minerva's Jetson")
            if mode == 'live' and getattr(self.helm, 'connection', 'tether') != 'tether':
                raise ValueError('Live teleop runs over the wired tether: choose Tether at the top first')
            self.receiver = ({'mode': 'live', 'arm': arm} if mode == 'live'
                             else dict({'mode': 'virtual'}, **({'collision': collision} if collision else {})))

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
        if self.receiver['mode'] == 'live' or self.manual:
            # Motor models live in the hardware profiles, which stay on this PC.
            from robot_platform.control.commissioning import motor_models
            bundle['receiver'] = dict(self.receiver, motor_models=motor_models(self.helm.device, ROOT))
        return bundle

    def start(self, *, confirm=False):
        with self._lock:
            if self.active:
                raise ValueError('Teleop is already running')
            if self.simulation is not None and (self.target != 'pc' or self.receiver['mode'] != 'virtual'):
                raise ValueError('Simulation mode needs the virtual receiver on this PC')
            if self.task_capture_config is not None and self.receiver != {'mode': 'live', 'arm': 'both'}:
                raise ValueError('Configured task capture requires Live both arms')
            if self._tuning_lock.locked():
                raise ValueError('Wait for the tuning operation before starting')
            if self.receiver['mode'] == 'live' and getattr(self.helm, 'connection', 'tether') != 'tether':
                raise ValueError('Live teleop runs over the wired tether: choose Tether at the top first')
            if self.receiver['mode'] == 'live' and confirm is not True:
                arms = 'both arms\'' if self.receiver['arm'] == 'both' else f"the {self.receiver['arm']} arm's"
                raise ValueError(f'Confirm the live session: {arms} motors will be powered, the foot switch is in '
                                 'reach and the arms hang in their reference pose')
            if self.helm.saved_limits is None:
                raise ValueError('Save joint limits for every motor in the Limits workspace first')
            if self.leaders is not None:
                missing = self.leaders.missing(self.profile['routes'])
                if missing:
                    raise ValueError('Sweep and save the leader ranges in the Leaders tab first; missing: '
                                     + ', '.join(missing))
            local = self.target == 'pc' or self.core_factory is not None
            if self.manual:
                self.manual.new_session()
            if self.settings:
                self.settings.new_session()  # A draft made before Start cannot be applied to this session.
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
                message = ('Starting: loading the simulated Minerva and its window; opening the leaders…'
                           if 'simulation' in bundle else
                           'Starting: loading the collision model and virtual receiver; opening the leaders…')
            else:
                from .edge import EdgeWorker
                leader_profile = json.loads((self.profile_path.parent / self.profile['leader_profile'])
                                            .read_text(encoding='utf-8'))
                worker = (self.edge_factory or EdgeWorker)(bundle, leader_profile, base_view=dict(self._view),
                                                           real_leaders=self.leader_factory is None,
                                                           address=getattr(self.helm, 'address', None),
                                                           evidence=self.link_evidence)
                command_writer, view_reader = worker.commands, worker.views
                message = ("Starting on Minerva's Jetson: connecting, loading the collision model and virtual "
                           'receiver, linking the leaders…') if self.receiver['mode'] == 'virtual' else (
                    ('Starting LIVE on both arms: checking they are still and in their reference pose, then '
                     'starting the receiver (the motors stay disabled until Idle)…') if self.receiver['arm'] == 'both'
                    else f"Starting LIVE on the {self.receiver['arm']} arm: checking it is still and in its reference "
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
                        self._view_received_at = time.perf_counter()
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
        if self._view.get('fault') and key != 'H':
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
            self._sync_tuning()
            allowed, reason = self._tuning_gate()
            choices = ['virtual'] + ([self.profile['trial']['motor'].split('_')[0]] if self.profile.get('trial') else
                                     ['left', 'right', 'both'])
            observed = self._view.get('observed_at')
            age = (max(observed, self._view.get('status_at', observed)) - observed
                   + time.perf_counter() - self._view_received_at) * 1000 if observed is not None else None
            view = deepcopy(self._view)
            if view.get('telemetry') is not None:
                # Aged since it was observed; a cached packet never looks fresher than its source.
                from .telemetry import age_packet
                view['telemetry'] = age_packet(view['telemetry'], age / 1000 if age is not None else None)
            law, upcoming = self._control_law(view)
            return dict(view, target=self.target, receiver_choice=dict(self.receiver),
                        live_available=self.simulation is None and 'live' in self.profile,
                        receiver_choices=choices if self.simulation is None else ['virtual'],
                        simulation=self.simulation is not None, task_configured=self.task_configured,
                        observation_age_ms=age, control_law=law, control_law_next=upcoming,
                        tuning=self.manual.snapshot(can_apply=allowed, reason=reason) if self.manual else
                        {'enabled': False})

    def _control_law(self, view):
        """The receiver's law for the status row, and whether it is the next session's: a running session's as
        its core reports it (a release that ignores the switch shows), else the law a session starts with."""
        if self.active and view.get('phase') == 'running' and 'position_only' in view:
            direct = self.profile['receiver'].get('collision_mode') == 'disabled'
            return control_law(direct, view['position_only'], view.get('velocity_feedforward', False)), False
        manual = {'velocity_feedforward': self.manual.feedforward} if self.manual else None
        return profile_law(switched_profile(self.profile, manual)), not self.active

    def _sync_tuning(self):
        """Recover an applied acknowledgement even if the HTTP caller timed out."""
        ack = self._view.get('manual_tuning_ack')
        if (self.manual and ack and ack['session_id'] == self.manual.session_id
                and ack['revision'] > self.manual.revision):
            from .manual_tuning import validate
            self.manual.active = validate(ack['joints'], self.manual.rows)
            self.manual.revision = ack['revision']

    def _tuning_gate(self):
        if not self.manual:
            return False, 'Start Helm with the manual tuning profile to edit settings'
        if self._tuning_lock.locked():
            return False, 'Waiting for the tuning operation to finish'
        if not self.active:
            return True, 'Apply prepares the next session; it does not enable motors'
        if self._view.get('manual_tuning_pending'):
            return False, 'Waiting for the receiver to confirm the pending tuning change'
        if self._view.get('phase') == 'running' and self._view.get('state') == 'HOLD' and not self._view.get('fault'):
            return True, 'Apply updates held gains; Engage resumes following'
        return False, 'Hold to edit. Before enabling, stop the session to prepare settings.'

    def manual_tuning(self, action, payload):
        """Serialize edits without blocking emergency keys or the receive thread."""
        expected = {'session_id', 'revision'} | ({'joints'} if action == 'apply' else set())
        if (action not in ('apply', 'save', 'restore') or not isinstance(payload, dict)
                or set(payload) - ({'velocity_feedforward'} if action == 'apply' else set()) != expected
                or type(payload.get('velocity_feedforward', False)) is not bool):
            raise ValueError('Invalid manual tuning request')
        with self._lock:
            self._sync_tuning()
            allowed, reason = self._tuning_gate()
            if not allowed:
                raise ValueError(reason)
            self.manual.check_revision(payload['session_id'], payload['revision'])
            if not self._tuning_lock.acquire(blocking=False):
                raise ValueError('A tuning change is already in progress')
        try:
            from .manual_tuning import validate
            if action == 'save':
                self.manual.save()
                return
            if action == 'restore' and self.manual.saved is None:
                raise ValueError('No manual preset has been saved')
            rows = validate(payload['joints'] if action == 'apply' else self.manual.saved['joints'], self.manual.rows)
            feedforward = (payload.get('velocity_feedforward') if action == 'apply'
                           else self.manual.saved.get('velocity_feedforward') is True)
            if self.settings:
                self.settings.validate_manual(rows)  # Within the operator's following-speed ceilings.
            if self.active:
                if feedforward is not None and feedforward != self.manual.feedforward:
                    raise ValueError('Velocity feed-forward switches at the next session start; '
                                     'stop the session to change it')
                ack = self._request('manual_tuning', dict(self.manual.bundle(), joints=rows))
                with self._lock:
                    self.manual.active = validate(ack['joints'], self.manual.rows)
                    self.manual.revision = ack['revision']
            else:
                with self._lock:
                    self.manual.accept(rows, feedforward=feedforward)
        finally:
            self._tuning_lock.release()

    def settings_snapshot(self):
        """The Settings workspace: every setting with its value, default, bounds and when it takes effect."""
        with self._lock:
            if self.settings is None:
                return {'enabled': False, 'session_id': None, 'revision': 0, 'can_apply': False,
                        'reason': self.settings_error or 'Settings are unavailable', 'sections': []}
            active = self.active
            reason = ('Stop the teleop session to change settings; they apply from the next Start' if active
                      else 'Applied values are saved for the next session; no motor commands are sent')
            return self.settings.snapshot(can_apply=not active, reason=reason, manual=self.manual)

    def settings_update(self, action, payload):
        """Apply edits to, or reset, the session settings for the next Start (never a running session)."""
        with self._lock:
            if self.settings is None:
                raise ValueError(self.settings_error or 'Settings are unavailable')
            if self._tuning_lock.locked():
                raise ValueError('Wait for the tuning operation to finish')
            self.settings.update(action, payload, running=self.active, manual=self.manual)
        return self.settings_snapshot()

    def guide(self):
        with self._lock:
            return deepcopy(self._guide)
