"""The operator's headset as the teleop loop's target, in place of the leaders (Helm's Headset input; VR arm control,
docs/architecture/VR_ARM_TELEOP_PLAN.md, phase 4).

The controllers grab and drag the arms in a worker process (teleop.vr_worker runs teleop.vr_arms); this side turns
each arm's commanded CAD angles into the receiver's motor positions (cad = scale * motor + offset, the derived
report's) and each trigger into its gripper's position between open and closed, and seeds the drag from the pose
Engage aligned to.

What the arms do, by lifecycle state:
- IDLE, HOLD: the target is where the arms are, each gripper at its trigger (at its measured position without a fresh
  hand).
- ENGAGING: the pose taken as Engage began, held, except that an arm hanging at its reference rises to its ready pose
  (READY: forearm forward, the gripper level ahead of the robot). The hanging arm is singular: there a 3 cm sideways
  drag swung J1 and J5 against each other by over a radian, where the ready pose is well conditioned (the user's
  choice, 2026-11-02). The lifecycle aligns to it at the alignment speed, as to a replay's first frame.
- TELEOP: the worker is seeded with the aligned pose's arm joints as following begins (a grip held through the raise
  must be let go first, so nothing jumps); then its newest answer computed from that seed, while it is fresh (within
  `max_age_s` of its arrival on the loop's clock); until the first, the aligned pose for at most `start_s`. Without
  one the source gives no target, and the lifecycle treats that as it treats lost leaders. A gripper whose hand is
  lost keeps its last target.
- Anything else: no target; the next Engage aligns and seeds again, so after a hold, fault or pause the drag starts
  from where the arms are (raised again only if they hang), never from where it was commanded before.

Each B/Y press the worker counts asks for Hold while engaged: never Engage, never Disable.

The loop sees an ordinary target: every move still goes through the lifecycle (alignment, speed limits) and the
receiver's limits, and the operator's Hold, Return, Disable and the foot switch stay in charge.
"""

import math

from .arm_ik import ARM_JOINTS, ARMS, cad_limits

HELD = ('IDLE', 'HOLD')
ENGAGED = ('ENGAGING', 'TELEOP')
# Each arm's ready pose (CAD joints, rad): the upper arm hanging, the forearm folded forward and the gripper level,
# 28 cm ahead of the shoulder and 24 cm above where it hangs, the hands 48 cm apart. The Jacobian's smallest singular
# value is 0.16 (the phase 2 working poses'), and J2-J4 sit 0.42 rad or more inside Minerva's swept limits, so a 6 cm
# drag any way converges with no joint at a limit (2026-11-02). The arms mirror each other: J2 turns the other way on
# the right. Rising, the forearm swings forward first: halfway the hand is 18 cm ahead and 7 cm up.
READY = {'left': (0.0, -0.3, -1.6, -0.7, 0.0, 0.0), 'right': (0.0, 0.3, -1.6, -0.7, 0.0, 0.0)}
HANGING_RAD = 0.25  # An arm hangs while J2, J3 and J4 (its parallel pitch joints) are this close to the reference.


def validate_request(payload):
    """Helm's headset request: {'kind': 'headset'}."""
    if not isinstance(payload, dict) or set(payload) - {'kind'} or payload.get('kind', 'headset') != 'headset':
        raise ValueError('A headset session takes no options')
    return {'kind': 'headset'}


def worker_limits(report, names):
    """{arm: {cad joint: (low, high)}} for the arms this receiver drives whole (all six joints in `names`)."""
    driven = {row['cad_joint'] for row in report if row['motor'] in set(names)}
    return {
        arm: cad_limits(report, arm)
        for arm in ARMS
        if all(f'{arm}_arm_j{n}' in driven for n in range(1, ARM_JOINTS + 1))
    }


class HeadsetSource:
    """`reference` ({motor: position}, the lifecycle's zero positions): where a hanging arm hangs. Without it no arm
    rises on Engage."""

    def __init__(self, names, report, *, reference=None, max_age_s=0.2, start_s=0.5):
        for value, label in ((max_age_s, 'max_age_s'), (start_s, 'start_s')):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{label} must be a positive number of seconds')
        self.names, self.max_age_s, self.start_s = tuple(names), float(max_age_s), float(start_s)
        rows = {row['motor']: row for row in report if row['motor'] in set(self.names)}
        by_cad = {row['cad_joint']: row for row in rows.values()}
        self.arms = {
            arm: [
                (
                    by_cad[f'{arm}_arm_j{n}']['motor'],
                    by_cad[f'{arm}_arm_j{n}']['scale'],
                    by_cad[f'{arm}_arm_j{n}']['offset'],
                )
                for n in range(1, ARM_JOINTS + 1)
            ]
            for arm in worker_limits(report, self.names)
        }
        if not self.arms:
            raise ValueError('The headset drives whole arms: this receiver has none')
        self.grippers = {}  # {arm: (motor, open, closed)}: the opening direction picks the usable range's end.
        for arm in self.arms:
            row = next((r for r in rows.values() if r['motor'] == f'{arm}_gripper' and r.get('opens') in (-1, 1)), None)
            if row is not None:
                low, high = row['usable']
                self.grippers[arm] = (row['motor'], high, low) if row['opens'] > 0 else (row['motor'], low, high)
        self.reference = None  # {arm: CAD joints} where each arm hangs.
        if reference is not None:
            self.reference = {arm: self._cad(arm, reference) for arm in self.arms}
            for arm in self.arms:
                limits = cad_limits(report, arm)
                outside = [
                    n
                    for n, q in enumerate(READY[arm], 1)
                    if not limits[f'{arm}_arm_j{n}'][0] < q < limits[f'{arm}_arm_j{n}'][1]
                ]
                if outside:
                    raise ValueError(
                        f"The headset's ready pose is outside the {arm} arm's limits at "
                        + ', '.join(f'J{n}' for n in outside)
                    )
        self.raised = ()  # The arms this Engage raises to READY.
        self.seed = 0  # The latest seed's number: answers computed from another are not followed.
        self.pending_seed = None  # (number, {arm: CAD positions}) for the loop to hand the worker.
        self.aligned = None  # ENGAGING/TELEOP: the pose Engage aligned to, {motor: position}.
        self.grip_targets = {}  # Each gripper's last target while following.
        self.answer = None  # The newest answer and its arrival, as given to `receive`.
        self.holds_seen = {}
        self.sequence = self.answers = self.stale_cycles = self.hold_requests = 0
        self.last_error = None
        self._seeded_at = None
        self._followed = False  # An answer from the latest seed has been followed: the seed is no fallback now.
        self._state = None

    # -- the worker's side ------------------------------------------------------------------

    def receive(self, latest):
        """The worker's newest answer, (message, arrival on the loop's clock), or None."""
        if latest is None or latest is self.answer:
            return
        self.answer = latest
        self.answers += 1
        message = latest[0]
        self.last_error = message.get('error')

    def take_seed(self):
        """The seed to hand the worker now, (number, {arm: CAD positions}), once; else None."""
        seed, self.pending_seed = self.pending_seed, None
        return seed

    def hold_wanted(self, state):
        """Whether a B/Y press since the last answer asks for Hold: only while engaged."""
        commands = self._commands()
        if not commands:
            return False
        pressed = False
        for arm, command in commands.items():
            count = command.get('holds', 0)
            pressed |= count > self.holds_seen.get(arm, count)
            self.holds_seen[arm] = count
        if pressed and state in ENGAGED:
            self.hold_requests += 1
            return True
        return False

    # -- the loop's side ---------------------------------------------------------------------

    def _commands(self, now=None):
        if self.answer is None:
            return None
        message, arrived = self.answer
        if now is not None and not 0 <= now - arrived <= self.max_age_s:
            return None
        return message.get('commands')

    def _motor(self, arm, positions):
        return {motor: (cad - offset) / scale for (motor, scale, offset), cad in zip(self.arms[arm], positions)}

    def _cad(self, arm, measured):
        return [scale * measured[motor] + offset for motor, scale, offset in self.arms[arm]]

    def _gripper(self, arm, closure):
        motor, opened, closed = self.grippers[arm]
        return opened + min(1.0, max(0.0, closure)) * (closed - opened)

    def _held_pose(self, now, measured):
        """Where the arms are, each gripper at its trigger if a hand is fresh."""
        pose = dict(measured)
        commands = self._commands(now) or {}
        for arm in self.grippers:
            trigger = (commands.get(arm) or {}).get('trigger')
            if trigger is not None:
                pose[self.grippers[arm][0]] = self._gripper(arm, trigger)
        return pose

    def pause(self):
        """Hold, a fault or a pause ended following: the next Engage aligns and seeds again from where the arms are."""
        self.aligned, self.grip_targets, self._seeded_at, self._followed = None, {}, None, False
        self.raised = ()

    def _hanging(self, arm, measured):
        if self.reference is None:
            return False
        cad = self._cad(arm, measured)
        return all(abs(cad[n] - self.reference[arm][n]) < HANGING_RAD for n in (1, 2, 3))

    def target(self, state, now, measured):
        """This cycle's target rows from the receiver's measured motor positions ({motor: rad}), or None."""
        if any(name not in measured for name in self.names):
            return None  # No complete reading yet.
        if self._state in ENGAGED and state not in ENGAGED:
            self.pause()
        self._state = state
        if state in HELD:
            self.pause()
            pose = self._held_pose(now, measured)
        elif state in ENGAGED:
            if self.aligned is None:
                # Engage began: the pose it aligns to, where the arms are but for a hanging arm, raised.
                self.aligned = self._held_pose(now, measured)
                self.raised = tuple(arm for arm in self.arms if self._hanging(arm, measured))
                for arm in self.raised:
                    self.aligned.update(self._motor(arm, READY[arm]))
                self.grip_targets = {arm: self.aligned[motor] for arm, (motor, _, _) in self.grippers.items()}
            pose = dict(self.aligned)
            if state == 'TELEOP':
                if self._seeded_at is None:
                    # Aligned: the drag starts from the pose the arms settled to.
                    self.seed += 1
                    self.pending_seed = (self.seed, {arm: self._cad(arm, self.aligned) for arm in self.arms})
                    self._seeded_at = now
                commands = self._commands(now) if self.answer and self.answer[0].get('seed') == self.seed else None
                if commands is None:
                    # Never back to the seed once following has begun: the arm would be pulled to where Engage
                    # began.
                    if self._followed or now - self._seeded_at > self.start_s:
                        self.stale_cycles += 1
                        return None
                else:
                    self._followed = True
                    for arm in self.arms:
                        command = commands.get(arm) or {}
                        if command.get('positions') is not None:
                            pose.update(self._motor(arm, command['positions']))
                        if arm in self.grippers and command.get('closure') is not None:
                            self.grip_targets[arm] = self._gripper(arm, command['closure'])
                    pose.update({self.grippers[arm][0]: value for arm, value in self.grip_targets.items()})
        else:
            self.pause()
            return None
        self.sequence += 1
        return [
            dict(name=name, position=float(pose[name]), sampled_at=now, sequence=self.sequence) for name in self.names
        ]

    def snapshot(self, now=None):
        commands = (self.answer[0].get('commands') if self.answer else None) or {}
        arms = {
            arm: {
                key: (commands.get(arm) or {}).get(key)
                for key in (
                    'gripped',
                    'tracking',
                    'clamped',
                    'converged',
                    'near_singular',
                    'at_limit',
                    'sigma',
                    'trigger',
                )
            }
            for arm in self.arms
        }
        return dict(
            arms=arms,
            answers=self.answers,
            stale_cycles=self.stale_cycles,
            seed=self.seed,
            following=self.aligned is not None,
            raised=list(self.raised),
            hold_requests=self.hold_requests,
            error=self.last_error,
            compute_ms=self.answer[0].get('compute_ms') if self.answer else None,
            answer_age_s=None if self.answer is None or now is None else round(now - self.answer[1], 3),
        )
