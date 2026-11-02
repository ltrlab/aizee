"""The headset's controllers as targets for Minerva's arms: a grab and drag per arm (VR arm control,
docs/development/VR_ARM_TELEOP_PLAN.md, phase 2).

Holding a controller's grip attaches that hand to its arm. From the grip on, the arm's tool point (teleop.arm_ik) moves
as the hand has moved since the grip, turned into the robot's frame and scaled, and turns as the hand has turned, about
the same axes in the world. Releasing holds the arm where it was commanded; the next grip goes on from there. The
trigger closes the gripper (0 open, 1 closed). B or Y asks for Hold. A hand working the headset's screens (moving one
with its grip, or pressing the bar with its trigger) lets go of its arm and leaves its gripper as it was.

The VR world is WebXR's `local` space (x right, y up, -z ahead where the session started). With the forward yaw (the
yaw of Helm's head zero, left positive) taken out, its right is the robot's right (-x), its up is up (+z) and its ahead
is the robot's front (-y). Only the hand's change since the grip is used, so the controller's own axes never matter.

Each hand's poses pass through:
- a quaternion sign fix (q and -q are one rotation; a controller flips between them on fast turns);
- a jump check: a pose more than JUMP_M or JUMP_RAD from the last is dropped, and after JUMP_GRACE such poses in a row
  taken as the new place of the hand: a hand that is dragging is attached there again, so a tracking glitch moves no
  arm;
- a low-pass filter with a time constant (poses arrive unevenly over Wi-Fi).
A hand with no pose for `stale_s` lets go (the arm holds); its next fresh pose with the grip held attaches it again
where it is, so the arm does not jump. A grip already held when the arm is seeded (Engage, or after a hold) must be
let go first. A recenter (WebXR's reset) or a new forward yaw or scale attaches dragging hands again where they are.

The tool's target moves at most `max_tool_speed_m_s` and stays inside the arm's workspace box (when one is given). Each
joint moves at most `max_joint_speed_rad_s` an update, so near a singular pose (the reference pose among them: the arms
hang straight, J1 and J5 both vertical) the target lags rather than a joint whipping round; the arm's smallest
singular value goes out with its state.

Adapted from AIZEE's python/teleop/quest_leader.py (June 2026), which drove one arm the same way outside the guarded
path. Plain data in and out: the teleop loop's source (phase 4) turns the CAD angles into motor targets, and every
target still passes the lifecycle's and the receiver's limits. Nothing here commands motion.
"""

from dataclasses import dataclass, field
import math

import numpy as np

from .arm_ik import ARMS

# The VR world (yaw taken out) to the robot frame: right -> -x, up -> +z, ahead (-z) -> -y.
VR_TO_ROBOT = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])


def quaternion_matrix(q):
    """The rotation matrix of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def about_up(yaw):
    """A turn of `yaw` about the VR world's up (+y), left positive."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


@dataclass(frozen=True)
class HandSample:
    """One controller's latest state, `at` when Helm received it (the loop's clock, seconds): position (metres) and
    orientation (x, y, z, w) in the VR world, grip and trigger 0 to 1, B/Y held, and `ui`: it is working the
    headset's screens (holding one, or pressing the bar), so its arm lets go and its gripper stays as it was."""

    position: tuple
    orientation: tuple
    grip: float
    trigger: float
    hold: bool
    at: float
    ui: bool = False

    def __post_init__(self):
        if len(self.position) != 3 or len(self.orientation) != 4:
            raise ValueError(
                "A hand sample has a position (x, y, z) and an orientation (x, y, z, w)"
            )
        values = (*self.position, *self.orientation, self.grip, self.trigger, self.at)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            raise ValueError("A hand sample is finite numbers")
        if not 0.9 < math.sqrt(sum(v * v for v in self.orientation)) < 1.1:
            raise ValueError("A hand orientation is a unit quaternion")


@dataclass
class DragSettings:
    scale: float = 1.0  # Tool motion per hand motion.
    stale_s: float = 0.1  # A hand with no newer pose lets go.
    smoothing_s: float = (
        0.02  # The pose filter's time constant: about one 60 Hz frame of lag.
    )
    jump_m: float = 0.3
    jump_rad: float = 0.5
    jump_grace: int = 3
    grip_on: float = 0.6  # The grip attaches above this and lets go below grip_off.
    grip_off: float = 0.4
    max_tool_speed_m_s: float = (
        0.6  # AIZEE's: tracking jumps cut, a reach's speed kept.
    )
    max_joint_speed_rad_s: float = 3.0  # AIZEE's following speed for the Quest.
    max_step_s: float = (
        1 / 30
    )  # A longer gap between updates grants no more motion than this.
    singular_sigma: float = (
        0.02  # Below this smallest singular value the arm is near a singular pose.
    )
    # A solve that fails with the elbow straight (|J3| below this) is tried again from this bend, and the better kept.
    # The hanging arm is singular, and from straight the IK folds the elbow either way: unbent it chose the forearm
    # back, the elbow ahead of the hand (toward the table). Minerva's forearm folds forward with CAD J3 negative, both
    # arms (2026-11-02, the virtual receiver). A grip without a move bends nothing: that solve succeeds. 0: off.
    elbow_bend: float = -0.05
    # Each update solves from the last answer, so a solve not done by then goes on at the next: a converging solve
    # takes 1-2, and an arm held at a joint limit no longer spends 12 iterations a frame (on the Jetson, 4 ms of an
    # arm's update).
    ik_iterations: int = 4
    workspace: dict = field(
        default_factory=dict
    )  # {arm: ((x, y, z) low, (x, y, z) high)} in the robot frame.

    def __post_init__(self):
        positive = (
            "scale",
            "stale_s",
            "smoothing_s",
            "jump_m",
            "jump_rad",
            "max_tool_speed_m_s",
            "max_joint_speed_rad_s",
            "max_step_s",
            "singular_sigma",
        )
        if any(
            not math.isfinite(getattr(self, k)) or getattr(self, k) <= 0
            for k in positive
        ):
            raise ValueError(f"{', '.join(positive)} are positive")
        if (
            not 0 < self.grip_off < self.grip_on < 1
            or self.jump_grace < 1
            or self.ik_iterations < 1
        ):
            raise ValueError("The grip lets go below where it attaches, inside 0 to 1")
        if not math.isfinite(self.elbow_bend) or abs(self.elbow_bend) > 0.3:
            raise ValueError("elbow_bend is a small bend, at most 0.3 rad either way")
        for arm, box in self.workspace.items():
            low, high = (np.asarray(corner, dtype=float) for corner in box)
            if (
                arm not in ARMS
                or low.shape != (3,)
                or high.shape != (3,)
                or not np.all(low < high)
            ):
                raise ValueError(
                    "A workspace box is {arm: ((x, y, z) low, (x, y, z) high)}"
                )


@dataclass(frozen=True)
class ArmCommand:
    """One arm's state after an update. `positions`: the commanded CAD angles (arm_ik joints order), None until the arm
    is seeded; `closure`: the gripper's 0 (open) to 1 (closed), None while its hand is not tracked."""

    positions: tuple
    closure: float
    gripped: bool
    tracking: bool
    clamped: bool  # The target was held back: the workspace box, the tool's speed or a joint's speed.
    converged: bool
    at_limit: tuple
    sigma: float  # The smallest singular value of the arm's Jacobian at its commanded positions.
    near_singular: bool  # sigma below DragSettings.singular_sigma.
    hold: bool  # B or Y was pressed since the last update.


class _Hand:
    """A hand's filtered pose: the sign fix, the jump check and the low-pass filter."""

    def __init__(self, settings):
        self.settings = settings
        self.reset()

    def reset(self):
        self.position = self.orientation = self.at = None
        self.jumps = 0

    def add(self, sample):
        """Filter in `sample`: 'new' for the first pose (or the jump check giving way), 'kept', 'dropped' (a jump),
        or 'same' (no newer pose than the last)."""
        if self.at is not None and sample.at <= self.at:
            return "same"
        p, q = (
            np.asarray(sample.position, dtype=float),
            np.asarray(sample.orientation, dtype=float),
        )
        q = q / np.linalg.norm(q)
        if self.position is None:
            self.position, self.orientation, self.at, self.jumps = p, q, sample.at, 0
            return "new"
        if q @ self.orientation < 0:
            q = -q
        moved = float(np.linalg.norm(p - self.position))
        turned = 2 * math.acos(min(1.0, abs(float(q @ self.orientation))))
        if moved > self.settings.jump_m or turned > self.settings.jump_rad:
            self.jumps += 1
            if self.jumps < self.settings.jump_grace:
                return "dropped"
            self.position, self.orientation, self.at, self.jumps = p, q, sample.at, 0
            return "new"
        self.jumps = 0
        a = 1 - math.exp(-(sample.at - self.at) / self.settings.smoothing_s)
        self.position = self.position + a * (p - self.position)
        blended = self.orientation + a * (q - self.orientation)
        self.orientation = blended / np.linalg.norm(blended)
        self.at = sample.at
        return "kept"

    def rotation(self):
        return quaternion_matrix(self.orientation)


class _Arm:
    def __init__(self, kinematics, settings):
        self.kinematics, self.settings = kinematics, settings
        self.hand = _Hand(settings)
        self.q = None
        self.gripped = self.armed = self.pressed = self.held_button = False
        self.anchor_hand = self.anchor_tool = self.target = None
        self.updated_at = self.closure = None
        self.converged, self.at_limit = True, ()

    def attach(self):
        """The hand, where it is now, holds the tool where it is commanded."""
        self.anchor_hand = (self.hand.position.copy(), self.hand.rotation())
        self.anchor_tool = self.kinematics.pose(self.q)
        self.target = self.anchor_tool.copy()
        self.gripped = True

    def let_go(self):
        self.gripped = False
        self.anchor_hand = self.anchor_tool = self.target = None


class VRArms:
    """Both arms' grab and drag. `kinematics`: {arm: teleop.arm_ik.ArmKinematics}; `settings`: DragSettings."""

    def __init__(self, kinematics, settings=None, *, forward_yaw=0.0):
        if (
            not kinematics
            or set(kinematics) - set(ARMS)
            or any(k.arm != arm for arm, k in kinematics.items())
        ):
            raise ValueError(
                "Kinematics are {arm: ArmKinematics} for the left and/or right arm"
            )
        self.settings = settings or DragSettings()
        self.arms = {arm: _Arm(k, self.settings) for arm, k in kinematics.items()}
        self.forward_yaw = float(forward_yaw)
        self._map = VR_TO_ROBOT @ about_up(-self.forward_yaw)

    def seed(self, arm, positions):
        """`arm`'s commanded positions are now `positions` (its measured joints: at Engage, or after a hold, a fault or
        a pause). A grip already held must be let go before it attaches."""
        state = self.arms[arm]
        state.kinematics.pose(positions)  # Six finite angles, or ValueError.
        state.q = np.asarray(positions, dtype=float).copy()
        state.let_go()
        state.armed = False
        state.updated_at = None
        state.converged, state.at_limit = True, ()

    def release(self, arm=None):
        """Let go of `arm` (both when None): the arm holds where it was commanded."""
        for state in [self.arms[arm]] if arm else self.arms.values():
            state.let_go()

    def recenter(self):
        """The VR world started again (WebXR's reset): each hand's filter starts afresh, and a hand still gripping
        attaches again where its next pose puts it."""
        for state in self.arms.values():
            state.hand.reset()
            state.let_go()

    def set_forward(self, yaw):
        """The forward yaw (Helm's head zero, left positive); dragging hands attach again where they are."""
        self.forward_yaw = float(yaw)
        self._map = VR_TO_ROBOT @ about_up(-self.forward_yaw)
        self._reattach()

    def set_scale(self, scale):
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError("The scale is positive")
        self.settings.scale = float(scale)
        self._reattach()

    def _reattach(self):
        for state in self.arms.values():
            if state.gripped:
                state.attach()

    def update(self, now, samples):
        """Each arm's ArmCommand at `now` (the loop's clock), from `samples` ({arm: HandSample or None}, each hand's
        latest)."""
        return {
            arm: self._update(state, now, samples.get(arm))
            for arm, state in self.arms.items()
        }

    def _update(self, state, now, sample):
        s = self.settings
        hold = bool(sample is not None and sample.hold and not state.held_button)
        state.held_button = bool(sample is not None and sample.hold)
        if state.q is None:
            return ArmCommand(
                None, None, False, False, False, True, (), math.nan, False, hold
            )
        step_s = (
            s.max_step_s
            if state.updated_at is None
            else min(max(now - state.updated_at, 1e-3), s.max_step_s)
        )
        state.updated_at = now
        if sample is None or now - sample.at > s.stale_s:
            # Lost: the arm holds; the hand's next fresh pose with the grip held attaches it again where it is.
            state.let_go()
            state.hand.reset()
            return self._command(state, False, None, False, hold)
        added = state.hand.add(sample)
        if sample.grip >= s.grip_on:
            state.pressed = True
        elif sample.grip <= s.grip_off:
            state.pressed = False
        if sample.ui:
            # Working the screens: the arm lets go, and its grip must be let go before it attaches again.
            state.armed = False
            state.let_go()
            return self._command(state, True, state.closure, False, hold)
        if not state.pressed:
            state.armed = True
            state.let_go()
        elif state.armed and (not state.gripped or added == "new"):
            state.attach()  # A new grip, a hand back after a gap or a jump: attached where it is, nothing moves.
        clamped = self._drag(state, step_s) if state.gripped else False
        state.closure = min(1.0, max(0.0, float(sample.trigger)))
        return self._command(state, True, state.closure, clamped, hold)

    def _drag(self, state, step_s):
        s = self.settings
        anchor_position, anchor_rotation = state.anchor_hand
        moved = self._map @ (state.hand.position - anchor_position) * s.scale
        turned = self._map @ (state.hand.rotation() @ anchor_rotation.T) @ self._map.T
        target = np.eye(4)
        target[:3, :3] = turned @ state.anchor_tool[:3, :3]
        target[:3, 3] = state.anchor_tool[:3, 3] + moved
        clamped = False
        box = s.workspace.get(state.kinematics.arm)
        if box is not None:
            inside = np.clip(target[:3, 3], box[0], box[1])
            clamped = not np.allclose(inside, target[:3, 3])
            target[:3, 3] = inside
        change = target[:3, 3] - state.target[:3, 3]
        reach = s.max_tool_speed_m_s * step_s
        if np.linalg.norm(change) > reach:
            target[:3, 3] = state.target[:3, 3] + change * (
                reach / np.linalg.norm(change)
            )
            clamped = True
        state.target = target
        solution = state.kinematics.solve(target, state.q, max_iterations=s.ik_iterations)
        if s.elbow_bend and not solution.converged and abs(state.q[2]) < abs(s.elbow_bend):
            seed = state.q.copy()
            seed[2] = s.elbow_bend  # J3: again from the forearm folded forward (elbow_bend).
            bent = state.kinematics.solve(target, seed, max_iterations=s.ik_iterations)
            if bent.converged or bent.position_error_m < solution.position_error_m:
                solution = bent
        step = np.asarray(solution.positions) - state.q
        largest = s.max_joint_speed_rad_s * step_s
        if np.max(np.abs(step)) > largest:
            step = np.clip(step, -largest, largest)
            clamped = True
        state.q = state.q + step
        state.converged, state.at_limit = solution.converged, solution.at_limit
        return clamped

    def _command(self, state, tracking, closure, clamped, hold):
        sigma = float(
            np.linalg.svd(state.kinematics.jacobian(state.q), compute_uv=False)[-1]
        )
        return ArmCommand(
            tuple(float(v) for v in state.q),
            closure,
            state.gripped,
            tracking,
            clamped,
            state.converged,
            state.at_limit,
            sigma,
            sigma < self.settings.singular_sigma,
            hold,
        )
