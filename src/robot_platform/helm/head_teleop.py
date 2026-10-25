"""The head in a teleop session: its own receiver beside the arms', driven through the same lifecycle (Idle, Enable,
Engage, Return, Disable) as the arms, one session for both.

Its target is the operator's headset (helm.xr: yaw turns neck_z, pitch turns neck_y, one to one from the headset's
zero) while a pose is fresh. Without one it holds its last target for a moment, then goes back to its zero pose, so
with no headset at all the head holds still at zero. Every target goes through the arms' lifecycle classes (the
direct law: speed caps, alignment, braking near the ends) and the head receiver's own limits.
"""
from copy import deepcopy
import math
import time

PERIOD_S = .04  # The head's commands at 25 Hz (its profile's loop_hz); its receiver ramps each over 40 ms.
CYCLE_BUDGET_S = .035
POSE_FRESH_S = .2  # A headset pose this recent steers the head;
RETURN_AFTER_S = 1.  # without one the head holds its last target this long, then returns to its zero pose.


class HeadTeleop:
    """`bundle`: the head's profile path, reference, limits and servo calibration (helm.tune's head bundle shape).
    Openers are injectable for tests, as the arms' are."""

    def __init__(self, bundle, *, live, clock=time.perf_counter, open_receiver=None, open_harness=None):
        self.bundle, self.live, self.clock = deepcopy(bundle), live, clock
        self.open_receiver = open_receiver or self._open_receiver
        self.open_harness = open_harness or self._open_harness
        self.derived = self.session = self.receiver = self.model = None
        self.fault = None
        self._closers = []
        self._pose = None  # (yaw, pitch, received at) from the headset, on this loop's clock.
        self._poses = 0
        self._held = None  # The last target followed, held while the headset is briefly quiet.
        self._last_cycle = None
        self._sequence = 0

    # -- resources -------------------------------------------------------------------------

    def _open_receiver(self, config, reference):
        from robot_platform.neck.head import HeadReceiver, service_config
        from robot_platform.ops.release import ROOT
        service = service_config(self.profile, reference, self.derived, mode='teleop_direct')
        return HeadReceiver(service, binary=ROOT / 'bin' / 'head_controller', directory=self.bundle['evidence'],
                            labels=self.labels).start()

    def _open_harness(self, config, initial):
        from robot_platform.simulation.control_harness import NativeControlHarness
        harness = NativeControlHarness(config, initial)
        harness.step(0)
        return harness

    def build(self):
        from robot_platform.control.guarded import GuardedControllerClient
        from robot_platform.models import ModelProfile, UrdfModel
        from robot_platform.neck import head
        from robot_platform.ops.release import ROOT
        from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
        from robot_platform.teleop.lifecycle import TeleopLifecycle
        self.profile = head.load_profile(self.bundle['teleop_profile'])
        reference = self.bundle['head_reference']
        self.labels = {motor['name']: motor['label'] for motor in self.profile['motors']}
        self.model = UrdfModel(ModelProfile.load(ROOT / self.profile['model_profile']))
        self.derived = derived = head.derive(self.profile, reference, self.bundle['limits'], self.model,
                                             servo=self.bundle.get('servo'))
        config = derived['config']
        if self.live:
            receiver = self.open_receiver(config, reference)
            exchange = receiver.exchange
        else:
            receiver = self.open_harness(config, derived['initial'])
            epoch = self.clock()

            def exchange(request):
                now_ms = max(receiver.now_ms, int((self.clock() - epoch) * 1000))
                if now_ms > receiver.now_ms:
                    receiver.step(now_ms)
                return receiver.step(now_ms, request)['reply']
        self._closers.append(receiver.close)
        self.receiver = receiver
        names = tuple(joint['name'] for joint in config['joints'])
        controller = GuardedControllerClient(exchange, group=config['group'], identity=config['identity'],
                                             joint_names=names)
        # Each cycle reads the reply of the cycle before (DirectLifecycleSession), so its feedback and target ages
        # are bounded by the head's period, not the arms' 10 ms one.
        lifecycle = TeleopLifecycle(derived['lifecycle'], clock=self.clock, max_step_s=2 * PERIOD_S,
                                    feedback_max_age_s=1.5 * PERIOD_S, target_max_age_s=2.5 * PERIOD_S,
                                    allow_repeated_source=True)
        self.session = DirectLifecycleSession(controller, config, lifecycle, lambda: receiver.last,
                                              period_s=PERIOD_S, cycle_budget_s=CYCLE_BUDGET_S, clock=self.clock)
        self.names = names
        self.zero = {row['name']: row['zero_position'] for row in derived['lifecycle']}
        self.usable = {row['motor']: row['usable'] for row in derived['report']}
        # The headset's yaw and pitch are the CAD angles of neck_z and neck_y; each motor maps from its CAD joint.
        self.axes = {row['actuator']: (joint, row['scale'], row['offset'])
                     for joint, row in derived['joint_map'].items()}

    # -- the headset -------------------------------------------------------------------------

    def pose(self, yaw, pitch, *, now=None):
        """The headset's latest yaw and pitch (radians, from its zero), as it arrived in this loop."""
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            raise ValueError('A headset pose is finite yaw and pitch')
        self._pose = (float(yaw), float(pitch), self.clock() if now is None else now)
        self._poses += 1

    def _following(self, now):
        """The positions the head should take now ({motor: raw rad}) and where they come from."""
        if self._pose is not None and now - self._pose[2] <= POSE_FRESH_S:
            angles = {'neck_z': self._pose[0], 'neck_y': self._pose[1]}
            positions = {}
            for motor, (joint, scale, offset) in self.axes.items():
                low, high = self.usable[motor]
                positions[motor] = min(max((angles[joint] - offset) / scale, low), high)
            self._held = positions
            return positions, 'headset'
        if self._held is not None and self._pose is not None and now - self._pose[2] <= RETURN_AFTER_S:
            return self._held, 'holding'
        self._held = None
        return dict(self.zero), 'zero'

    def _target(self, now):
        positions, self.source = self._following(now)
        self._sequence += 1
        return [{'name': name, 'position': positions[name], 'sampled_at': now, 'sequence': self._sequence}
                for name in self.names]

    # -- the cycle -----------------------------------------------------------------------------

    @property
    def state(self):
        return self.session.lifecycle.state if self.session is not None else None

    def cycle(self, action, started):
        """One head cycle with the arms' action (None between keys), at most every PERIOD_S unless a key came.
        Returns the cycle's record, or None when it was not due. A failure faults the head (raised to the caller,
        which holds the arms)."""
        if self.fault or self.session is None:
            return None
        if action is None and self._last_cycle is not None and started - self._last_cycle < PERIOD_S:
            return None
        allowed = {'enable': ('IDLE',), 'engage': ('IDLE', 'HOLD'), 'disengage': ('ENGAGING', 'TELEOP'),
                   'shutdown': ('IDLE', 'HOLD', 'ENGAGING', 'TELEOP')}
        if action in allowed and self.state not in allowed[action]:
            action = None  # The head follows the arms' lifecycle; a transition it is not in is skipped.
        self._last_cycle = started
        try:
            return self.session.cycle(action=action, target=self._target(started), started_at=started)
        except Exception as exc:
            status = getattr(exc, 'status', None) or self.session.controller.status or {}
            reason = status.get('reason')
            self.fault = f"Head receiver {status.get('state')}: {reason} · {exc}" if reason not in (None, 'ok') \
                else f'Head: {exc}'
            raise

    def status(self):
        now = self.clock()
        pose = None if self._pose is None else {'yaw': self._pose[0], 'pitch': self._pose[1],
                                                'age_s': now - self._pose[2], 'count': self._poses}
        last = (self.receiver.last or {}) if self.receiver is not None else {}
        return {'state': self.state, 'fault': self.fault, 'source': getattr(self, 'source', None), 'headset': pose,
                'positions': {row['name']: row['position'] for row in (self.session.last_feedback or [])}
                if self.session is not None else {}, 'motors_enabled': last.get('hardware_enabled')}

    def close(self):
        if self.session is not None and self.session.owned:
            try:
                self.session.controller.disable()
            except Exception:
                pass
        failures = []
        for close in reversed(self._closers):
            try:
                close()
            except Exception as exc:
                failures.append(str(exc) or type(exc).__name__)
        self._closers = []
        if failures:
            raise RuntimeError('Closing the head: ' + '; '.join(failures))
