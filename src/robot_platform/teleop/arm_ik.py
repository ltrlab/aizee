"""Inverse kinematics for Minerva's arms: the joint angles that put an arm's tool point at a pose, for driving the
arms from the headset's controllers (docs/architecture/VR_ARM_TELEOP_PLAN.md).

Each arm is its URDF chain from the robot's root to its gripper link (the child of `<arm>_arm_j6`). The tool point
sits midway between the two finger carriages in that link (their slide joints' origins) until the fingertip pads are
measured from CAD. Angles are CAD angles, unwrapped: scale * motor + offset (control.guarded_config.cad_map), never
taken a turn round, so a wrist swept past a full turn stays continuous and its limits mean what they say.

The solver is damped least squares on the geometric Jacobian, from a seed (the last answer): position and orientation
weighted against each other (metres against radians), each step capped, every iterate inside the limits, and a joint
pinned at a limit left out of the step that would push it further. The damping adapts (Levenberg-Marquardt): a step
that lowers the error is taken and the damping eased, one that does not is refused and the damping raised. A fixed
damping crawled near the arm's singular poses (1 in 30 random round trips unconverged after 100 iterations). A target
out of reach returns the nearest answer found, marked unconverged: the caller holds or clamps. Nothing here commands
motion.
"""

from dataclasses import dataclass
import math

import numpy as np

from robot_platform.vision.geometry import rotvec, rpy_matrix, transform

ARMS = ("left", "right")
ARM_JOINTS = 6
STEP_CAP_RAD = 0.25  # The most one iteration moves a joint: a far target is approached, not leapt at.
# The damping's range, and its change after a step taken (EASE) or refused (RAISE).
DAMPING_MIN, DAMPING_MAX = 1e-4, 10.0
EASE, RAISE = 1 / 3, 4.0


@dataclass(frozen=True)
class Solution:
    positions: tuple  # CAD angles in ArmKinematics.joints order.
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    converged: bool
    at_limit: tuple  # The joints pinned at a limit.

    def by_name(self, joints):
        return dict(zip(joints, self.positions))


def cad_limits(report, arm):
    """{cad joint: (low, high)} of `arm`'s joints: the usable range of each motor (a derived receiver's per-joint
    report, control.guarded_config.derive_receiver), mapped to CAD angles."""
    limits = {}
    for row in report:
        if row["cad_joint"].startswith(f"{arm}_arm_j"):
            low, high = (
                row["scale"] * value + row["offset"] for value in row["usable"]
            )
            limits[row["cad_joint"]] = (min(low, high), max(low, high))
    return limits


class ArmKinematics:
    """One arm's chain (`model`: models.UrdfModel; `arm`: 'left' or 'right'), with `limits` ({cad joint: (low,
    high)}, every arm joint) and its tool point (`tool`, metres in the gripper link; default the finger carriages'
    midpoint). Positions go in and come out in `joints` order."""

    def __init__(self, model, arm, limits, *, tool=None):
        if arm not in ARMS:
            raise ValueError(f"The arm is left or right, not {arm!r}")
        self.arm = arm
        self.joints = tuple(f"{arm}_arm_j{n}" for n in range(1, ARM_JOINTS + 1))
        by_name, by_child = (
            model.by_name,
            {joint["child"]: joint for joint in model.joints},
        )
        if any(name not in by_name for name in self.joints):
            raise ValueError(f"The model has no {arm} arm")
        gripper = by_name[self.joints[-1]]["child"]
        path, link = [], gripper
        while link != model.root:
            joint = by_child[link]
            path.append(joint)
            link = joint["parent"]
        path.reverse()
        # Each arm joint turns about its own z (as onshape-to-robot exports them): the chain is the fixed transform up
        # to each joint (any fixed joints before it folded in), then a turn about z; then fixed joints to the gripper.
        self._origins, fixed, moving = [], np.eye(4), []
        for joint in path:
            fixed = fixed @ transform(
                rpy_matrix(*joint["origin"]["rpy"]), joint["origin"]["xyz"]
            )
            if joint["type"] == "fixed":
                continue
            if joint["type"] not in ("revolute", "continuous") or not np.allclose(
                joint["axis"], (0.0, 0.0, 1.0)
            ):
                raise ValueError(f"{joint['name']}: an arm joint turns about its own z")
            moving.append(joint["name"])
            self._origins.append(fixed)
            fixed = np.eye(4)
        if tuple(moving) != self.joints:
            raise ValueError(
                f"The {arm} arm chain moves {moving}, not {list(self.joints)}"
            )
        if tool is None:
            slides = [
                j
                for j in model.joints
                if j["parent"] == gripper and j["type"] == "prismatic"
            ]
            if len(slides) != 2:
                raise ValueError(
                    f"The {arm} gripper link carries {len(slides)} finger slides, not 2"
                )
            tool = np.mean([slide["origin"]["xyz"] for slide in slides], axis=0)
        self.tool = np.asarray(tool, dtype=float)
        if self.tool.shape != (3,) or not np.all(np.isfinite(self.tool)):
            raise ValueError("The tool point is three finite numbers")
        self._end = fixed @ transform(np.eye(3), self.tool)
        if set(limits) != set(self.joints):
            raise ValueError(f"Limits are needed for exactly the {arm} arm joints")
        self.lower = np.array([float(limits[name][0]) for name in self.joints])
        self.upper = np.array([float(limits[name][1]) for name in self.joints])
        if not np.all(np.isfinite(self.lower) & np.isfinite(self.upper)) or np.any(
            self.lower >= self.upper
        ):
            raise ValueError("Each limit is a finite (low, high) with low below high")

    def _frames(self, q):
        """The tool's pose and each joint's axis and origin (the robot frame) at positions `q`. Few, small numpy calls:
        on the Jetson a solve is mostly their overhead."""
        r, p = np.eye(3), np.zeros(3)
        axes, points = np.empty((ARM_JOINTS, 3)), np.empty((ARM_JOINTS, 3))
        for i, origin in enumerate(self._origins):
            p = p + r @ origin[:3, 3]
            r = r @ origin[:3, :3]
            axes[i], points[i] = r[:, 2], p
            c, s = math.cos(q[i]), math.sin(q[i])
            r[:, :2] = r[:, :2] @ np.array(((c, -s), (s, c)))  # The turn about the joint's own z.
        pose = np.eye(4)
        pose[:3, :3], pose[:3, 3] = r @ self._end[:3, :3], p + r @ self._end[:3, 3]
        return pose, axes, points

    @staticmethod
    def _columns(pose, axes, points):
        """The 6x6 geometric Jacobian from `_frames`: each joint's axis crossed with the lever to the tool, then the
        axis (written out: numpy's cross costs more than the rest)."""
        lever = pose[:3, 3] - points
        jacobian = np.empty((6, ARM_JOINTS))
        jacobian[0] = axes[:, 1] * lever[:, 2] - axes[:, 2] * lever[:, 1]
        jacobian[1] = axes[:, 2] * lever[:, 0] - axes[:, 0] * lever[:, 2]
        jacobian[2] = axes[:, 0] * lever[:, 1] - axes[:, 1] * lever[:, 0]
        jacobian[3:] = axes.T
        return jacobian

    def pose(self, positions):
        """The tool point's 4x4 pose in the robot frame at `positions` (joints order)."""
        return self._frames(self._vector(positions))[0]

    def jacobian(self, positions):
        """The 6x6 geometric Jacobian at the tool point: linear velocity over angular, each column one joint."""
        return self._columns(*self._frames(self._vector(positions)))

    def solve(
        self,
        target,
        seed,
        *,
        position_weight=1.0,
        orientation_weight=0.5,
        damping=0.05,
        position_tolerance_m=2e-4,
        orientation_tolerance_rad=2e-3,
        max_iterations=12,
    ):
        """Positions putting the tool at `target` (4x4, the robot frame), starting from `seed` (joints order, clipped
        into the limits) with `damping` at first: a Solution, converged when both errors are inside their
        tolerances. Each iteration takes or refuses one step."""
        target = np.asarray(target, dtype=float)
        if target.shape != (4, 4) or not np.all(np.isfinite(target)):
            raise ValueError("The target is a finite 4x4 pose")
        weights = np.array([position_weight] * 3 + [orientation_weight] * 3)
        q = np.clip(self._vector(seed), self.lower, self.upper)
        current = self._residual(q, target, weights)
        iterations = 0
        while True:
            pose, axes, points, error, position_error, orientation_error = current
            converged = (
                position_error <= position_tolerance_m
                and orientation_error <= orientation_tolerance_rad
            )
            if converged or iterations == max_iterations:
                break
            iterations += 1
            jacobian = self._columns(pose, axes, points) * weights[:, None]
            step = self._step(jacobian, error, damping, np.ones(ARM_JOINTS, dtype=bool))
            # A joint at a limit that the step pushes further takes no part in it: the others move instead.
            pinned = ((q <= self.lower) & (step < 0)) | ((q >= self.upper) & (step > 0))
            if pinned.any():
                step = self._step(jacobian, error, damping, ~pinned)
            largest = float(np.max(np.abs(step)))
            if largest > STEP_CAP_RAD:
                step *= STEP_CAP_RAD / largest
            candidate = np.clip(q + step, self.lower, self.upper)
            trial = self._residual(candidate, target, weights)
            if trial[3] @ trial[3] < error @ error:
                q, current, damping = candidate, trial, max(DAMPING_MIN, damping * EASE)
            else:
                damping = min(DAMPING_MAX, damping * RAISE)
        at_limit = tuple(
            name
            for name, value, low, high in zip(self.joints, q, self.lower, self.upper)
            if value <= low or value >= high
        )
        return Solution(
            tuple(float(v) for v in q),
            position_error,
            orientation_error,
            iterations,
            converged,
            at_limit,
        )

    def _residual(self, q, target, weights):
        """(pose, axes, points, weighted error, position error, orientation error) at `q`."""
        pose, axes, points = self._frames(q)
        linear = target[:3, 3] - pose[:3, 3]
        angular = rotvec(target[:3, :3] @ pose[:3, :3].T)
        error = np.concatenate((linear, angular)) * weights
        return (
            pose,
            axes,
            points,
            error,
            float(np.linalg.norm(linear)),
            float(np.linalg.norm(angular)),
        )

    @staticmethod
    def _step(jacobian, error, damping, free):
        j = jacobian * free
        step = np.linalg.solve(
            j.T @ j + damping * damping * np.eye(ARM_JOINTS), j.T @ error
        )
        return step * free

    def _vector(self, positions):
        q = np.asarray(positions, dtype=float)
        if q.shape != (ARM_JOINTS,) or not np.all(np.isfinite(q)):
            raise ValueError(
                f"Positions are {ARM_JOINTS} finite angles, in joints order"
            )
        return q
