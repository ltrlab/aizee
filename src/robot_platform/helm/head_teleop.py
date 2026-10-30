"""The head in a teleop session: its own receiver beside the arms', driven through the same lifecycle (Idle, Enable,
Engage, Return, Disable) as the arms, one session for both.

Its target is the operator's headset (helm.xr: yaw turns neck_z, pitch turns neck_y, one to one from the headset's
zero) while a pose is fresh. Without one it holds its last target for a moment, then goes back to its zero pose, so
with no headset at all the head holds still at zero. Every target goes through the arms' lifecycle classes (the
direct law: speed caps, alignment, braking near the ends) and the head receiver's own limits. With its shoulder
clearance saved (neck.clearance), each target keeps clear of the shoulder too, and steps there so its path does.
"""
from copy import deepcopy
import math
from pathlib import Path
import time

from robot_platform.neck import clearance

PERIOD_S = .04  # The head's commands at 25 Hz (its profile's loop_hz); its receiver ramps each over 40 ms.
# The head cycles on the arms' ticks, on a grid of PERIOD_S slots: a tick this early still takes its slot (the ticks
# meet the grid only to float precision), and a late one does not push the next slot. Each command takes the offer of
# the reply before it, so two consecutive gaps must stay under the direct policy's command lifetime (100 ms).
DUE_SLACK_S = PERIOD_S / 4
CYCLE_BUDGET_S = .035
POSE_FRESH_S = .2  # A headset pose this recent steers the head;
RETURN_AFTER_S = 1.  # without one the head holds its last target this long, then returns to its zero pose.
# With a shoulder clearance, the head's targets step at this share of its speeds: steps the lifecycle takes whole.
SPEED_SHARE = .9


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
        self._due = None  # The next slot on the head's grid (helm clock); None before the first cycle.
        self._sequence = 0
        self.clearance = None
        self._stepped, self._aligned, self.limited = None, False, False  # Stepping clear of the shoulder (_clear).

    # -- resources -------------------------------------------------------------------------

    def _open_receiver(self, config, reference):
        from pathlib import Path
        from robot_platform.neck.head import HeadReceiver, service_config
        from robot_platform.ops.release import ROOT
        service = service_config(self.profile, reference, self.derived, mode='teleop_direct')
        evidence = Path.home() / 'minerva' / 'evidence' / time.strftime('live-head-%Y%m%dT%H%M%SZ', time.gmtime())
        return HeadReceiver(service, binary=ROOT / 'bin' / 'head_controller', directory=evidence,
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
        path = Path(self.bundle['teleop_profile'])
        self.profile = head.load_profile(path if path.is_absolute() else ROOT / path)  # Release-relative on the robot.
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
        # The shoulder clearance (neck.clearance), when one is saved for this reference: its targets then step.
        self.clearance = clearance.from_limits(self.bundle['limits'], reference)
        by_joint = {m['joint']: m for m in reference['motors']}
        self.pan, self.tilt = by_joint[clearance.PAN]['name'], by_joint[clearance.TILT]['name']
        self.centre = {m['name']: float(m['raw_reference_rad']) for m in reference['motors']}
        self.speeds = {row['name']: (row['align_speed'], row['max_velocity']) for row in derived['lifecycle']}

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

    def _clear(self, desired):
        """Toward `desired` in steps the lifecycle takes whole, each clear of the shoulder: the lifecycle moves each
        joint at its own speed, so one long move toward a clear pose could cut through the corner below it. Aligning
        (ENGAGING), the head holds where it is; following, it goes at the alignment speed until it first reaches its
        target, then at the head's speed. Without a clearance saved, `desired` as it is."""
        self.limited = False
        if self.clearance is None:
            return desired
        measured = {row['name']: row['position'] for row in (self.session.last_feedback or [])}
        if self.state not in ('ENGAGING', 'TELEOP') or self._stepped is None:
            self._stepped = {name: measured.get(name, self.zero[name]) for name in self.names}
            self._aligned = False
        if self.state != 'TELEOP':
            return dict(self._stepped)
        pan, tilt, centre = self.pan, self.tilt, self.centre
        goal = (desired[pan] - centre[pan], desired[tilt] - centre[tilt])
        cleared = self.clearance.clamp(*goal)
        self.limited = cleared != goal
        limit = tuple(SPEED_SHARE * self.speeds[name][self._aligned] * PERIOD_S for name in (pan, tilt))
        step = self.clearance.step((self._stepped[pan] - centre[pan], self._stepped[tilt] - centre[tilt]), cleared,
                                   limit)
        self._stepped = {pan: centre[pan] + step[0], tilt: centre[tilt] + step[1]}
        self._aligned = self._aligned or max(abs(a - b) for a, b in zip(step, cleared)) < 1e-6
        return dict(self._stepped)

    def _target(self, now):
        positions, self.source = self._following(now)
        positions = self._clear(positions)
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
        allowed = {'enable': ('IDLE',), 'engage': ('IDLE', 'HOLD'), 'disengage': ('ENGAGING', 'TELEOP'),
                   'shutdown': ('IDLE', 'HOLD', 'ENGAGING', 'TELEOP')}
        if action in allowed and self.state not in allowed[action]:
            action = None  # The head follows the arms' lifecycle; a transition it is not in is skipped.
        if action is None and self._due is not None and started < self._due - DUE_SLACK_S:
            return None
        # On the grid, the next slot follows this one; a key between slots, or a cycle half a period late (a
        # stall), starts the grid again from now.
        on_grid = action is None and self._due is not None and started - self._due < PERIOD_S / 2
        self._due = (self._due if on_grid else started) + PERIOD_S
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
        positions = ({row['name']: row['position'] for row in (self.session.last_feedback or [])}
                     if self.session is not None else {})
        # The neck's CAD angles (neck_z, neck_y), for the 3D view: each motor's measured position through its axis.
        joints = {joint: positions[motor] * scale + offset for motor, (joint, scale, offset)
                  in getattr(self, 'axes', {}).items() if motor in positions}
        return {'state': self.state, 'fault': self.fault, 'source': getattr(self, 'source', None), 'headset': pose,
                'positions': positions, 'joints': joints, 'motors_enabled': last.get('hardware_enabled'),
                # Whether the shoulder clearance holds the head above where the headset looks, this cycle.
                'clearance': None if self.clearance is None else {'limited': self.limited}}

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
