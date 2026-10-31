"""The head in a teleop session: its own receiver beside the arms', driven through the same lifecycle (Idle, Enable,
Engage, Return, Disable) as the arms, one session for both.

Its target is the operator's headset (helm.xr: yaw turns neck_z, pitch turns neck_y, one to one from the headset's
zero) while a pose is fresh. Without one it holds its last target for a moment, then goes back to its zero pose, so
with no headset at all the head holds still at zero. Every target goes through the arms' lifecycle classes (the
direct law: speed caps, alignment, braking near the ends) and the head receiver's own limits. With its shoulder
clearance saved (neck.clearance), each target keeps clear of the shoulder too, and steps there so its path does.

The head cannot stay at its reference: limp, it droops to where it rests (the user, 2026-10-30). A session starts
there (its receiver checks the head is still where it was read resting, neck.head.read_rest), Engage raises it
slowly, and Return (X), or the session's end, takes it back to its reference, then down to where it rested, before
its torque goes: it is never dropped from straight ahead, and never crosses the shoulder corner on the way.
"""
from copy import deepcopy
import json
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
        self.rest = None  # Where the head rested when the session began ({motor: raw rad}): Return ends there.
        self.returning = None  # Return's stage: 'centre' (to the reference), then 'rest'; None otherwise.
        self._disable_next = False

    # -- resources -------------------------------------------------------------------------

    def _open_receiver(self, config, reference):
        from pathlib import Path
        from robot_platform.neck.head import HeadReceiver, read_rest, service_config
        from robot_platform.ops.release import ROOT
        self.rest = read_rest(self.profile, self.bundle['limits'])
        service = service_config(self.profile, reference, self.derived, mode='teleop_direct', start=self.rest)
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
        self.usable = {row['motor']: row['usable'] for row in derived['report']}
        # The shoulder clearance (neck.clearance), when one is saved for this reference: its targets then step.
        self.clearance = clearance.from_limits(self.bundle['limits'], reference)
        by_joint = {m['joint']: m for m in reference['motors']}
        self.pan, self.tilt = by_joint[clearance.PAN]['name'], by_joint[clearance.TILT]['name']
        self.centre = {m['name']: float(m['raw_reference_rad']) for m in reference['motors']}
        if self.live:
            receiver = self.open_receiver(config, reference)
            exchange = receiver.exchange
        else:
            # No head to read: the virtual one rests where the real one droops, limp, tilted down to the end of its
            # range on the shoulder's down side (its saved clearance says which way that is), so Return shows the
            # real path, never ending straight ahead, where the real head would fall (the user, 2026-10-31). A
            # rehearsal may say where (`rest_pose`); with no clearance saved yet, it rests at its zero.
            joints = [joint['name'] for joint in config['joints']]
            self.rest = dict(zip(joints, derived['initial']))
            if self.clearance is not None:
                low, high = self.usable[self.tilt]
                self.rest[self.tilt] = low if self.clearance.down < 0 else high
            self.rest.update(self.bundle.get('rest_pose') or {})
            receiver = self.open_harness(config, [self.rest[name] for name in joints])
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
        # A cycle following nothing (Idle, Enable, Hold) takes the receiver, renews it or takes a fresh offer at
        # the head service's 10 ms ticks before its command: it has that command's lifetime, which the receiver
        # bounds it by, as the arms' holds have their verdict's. Following cycles keep the cycle budget.
        self.session = DirectLifecycleSession(controller, config, lifecycle, lambda: receiver.last,
                                              period_s=PERIOD_S, cycle_budget_s=CYCLE_BUDGET_S,
                                              hold_budget_s=config['direct']['command_ms'] / 1000, clock=self.clock)
        self.names = names
        self.zero = {row['name']: row['zero_position'] for row in derived['lifecycle']}
        # The headset's yaw and pitch are the CAD angles of neck_z and neck_y; each motor maps from its CAD joint.
        self.axes = {row['actuator']: (joint, row['scale'], row['offset'])
                     for joint, row in derived['joint_map'].items()}
        self.speeds = {row['name']: {'align': row['align_speed'], 'follow': row['max_velocity'],
                                     'return': row['shutdown_speed']} for row in derived['lifecycle']}
        self.tolerance = {row['name']: row['tolerance'] for row in derived['lifecycle']}
        # Return ends where the head rested, inside its usable range and clear of the shoulder: the last of a droop
        # past them happens limp.
        rest = dict(self.zero, **(self.rest or {}))
        rest = {name: min(max(rest[name], self.usable[name][0]), self.usable[name][1]) for name in names}
        if self.clearance is not None:
            pan, tilt = self.clearance.clamp(rest[self.pan] - self.centre[self.pan],
                                             rest[self.tilt] - self.centre[self.tilt])
            rest.update({self.pan: self.centre[self.pan] + pan, self.tilt: self.centre[self.tilt] + tilt})
        self.rest_target = rest

    # -- the headset -------------------------------------------------------------------------

    def pose(self, yaw, pitch, *, now=None):
        """The headset's latest yaw and pitch (radians, from its zero), as it arrived in this loop."""
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            raise ValueError('A headset pose is finite yaw and pitch')
        self._pose = (float(yaw), float(pitch), self.clock() if now is None else now)
        self._poses += 1

    def _following(self, now):
        """The positions the head should take now ({motor: raw rad}) and where they come from."""
        if self.returning is not None:
            return dict(self.zero if self.returning == 'centre' else self.rest_target), 'returning'
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
        target, then at the head's speed; returning, at its shutdown speed. Without a clearance saved, `desired` as
        it is, except returning: those steps keep the two stages a straight line each."""
        self.limited = False
        floor = self.clearance or (clearance.unbounded() if self.returning is not None else None)
        if floor is None:
            return desired
        measured = {row['name']: row['position'] for row in (self.session.last_feedback or [])}
        if self.state not in ('ENGAGING', 'TELEOP') or self._stepped is None:
            self._stepped = {name: measured.get(name, self.zero[name]) for name in self.names}
            self._aligned = False
        if self.state != 'TELEOP':
            return dict(self._stepped)
        pan, tilt, centre = self.pan, self.tilt, self.centre
        goal = (desired[pan] - centre[pan], desired[tilt] - centre[tilt])
        cleared = floor.clamp(*goal)
        self.limited = cleared != goal
        pace = 'return' if self.returning is not None else 'follow' if self._aligned else 'align'
        limit = tuple(SPEED_SHARE * self.speeds[name][pace] * PERIOD_S for name in (pan, tilt))
        step = floor.step((self._stepped[pan] - centre[pan], self._stepped[tilt] - centre[tilt]), cleared, limit)
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
        if self._disable_next:  # Back where it rested: its torque goes.
            self._disable_next, action = False, 'disable'
        if action == 'shutdown':
            # Return: limp already (Idle), it rests where it is; powered, it goes back to its reference, then down
            # to where it rested, following (from Hold it engages first, where it is), and is disabled there.
            if self.state == 'IDLE':
                action = 'disable'
            elif self.state in ('HOLD', 'ENGAGING', 'TELEOP') and self.returning is None:
                self.returning = 'centre'
                action = 'engage' if self.state == 'HOLD' else None
            else:
                action = None
        elif self.returning is not None and action != 'disable':
            action = None  # Returning: only Disable (H) acts.
        if action == 'disable':
            self.returning = None
        allowed = {'enable': ('IDLE',), 'engage': ('IDLE', 'HOLD'), 'disengage': ('ENGAGING', 'TELEOP')}
        if action in allowed and self.state not in allowed[action]:
            action = None  # The head follows the arms' lifecycle; a transition it is not in is skipped.
        if action is None and self._due is not None and started < self._due - DUE_SLACK_S:
            return None
        # On the grid, the next slot follows this one; a key between slots, or a cycle half a period late (a
        # stall), starts the grid again from now.
        on_grid = action is None and self._due is not None and started - self._due < PERIOD_S / 2
        self._due = (self._due if on_grid else started) + PERIOD_S
        try:
            target = self._target(started)
            # Only Engage and following use it; any other cycle follows nothing (the session's hold budget).
            following = action == 'engage' or (action is None and self.state in ('ENGAGING', 'TELEOP'))
            record = self.session.cycle(action=action, target=target if following else None, started_at=started)
            self._advance_return()
            return record
        except Exception as exc:
            status = getattr(exc, 'status', None) or self.session.controller.status or {}
            reason = status.get('reason')
            self.fault = f"Head receiver {status.get('state')}: {reason} · {exc}" if reason not in (None, 'ok') \
                else f'Head: {exc}'
            self._keep_fault(exc, status)
            raise

    def _keep_fault(self, exc, status):
        """The failing cycle, beside the live head receiver's evidence (fault-<UTC>.json): what was asked of the
        receiver, what it read and why the cycle failed. A command the receiver refuses leaves no trace there, and
        "invalid_command" alone does not say which joint or limit (2026-10-31)."""
        directory = getattr(self.receiver, 'directory', None)
        if directory is None:
            return
        try:
            stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
            record = {'fault': self.fault, 'error': f'{type(exc).__name__}: {exc}', 'status': status,
                      'cycle': self.session.last_cycle, 'returning': self.returning, 'stepped': self._stepped,
                      'rest_target': self.rest_target, 'limited': self.limited}
            (Path(directory) / f'fault-{stamp}.json').write_text(
                json.dumps(record, indent=1, default=repr) + '\n', encoding='utf-8')
        except (OSError, TypeError, ValueError):
            pass  # Evidence only: never stands in the way of the fault itself.

    def _advance_return(self):
        """Return's next stage once the head has reached this one's goal (its steps there, and its reading)."""
        if self.returning is None or self.state != 'TELEOP' or self._stepped is None:
            return
        goal = self.zero if self.returning == 'centre' else self.rest_target
        measured = {row['name']: row['position'] for row in (self.session.last_feedback or [])}
        if all(name in measured and abs(self._stepped[name] - goal[name]) < 1e-6
               and abs(measured[name] - goal[name]) <= self.tolerance[name] for name in self.names):
            if self.returning == 'centre':
                self.returning = 'rest'
            else:
                self._disable_next = True

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
                'returning': self.returning,
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
