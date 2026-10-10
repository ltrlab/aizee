"""Certify a complete joint-coordinate box for the guarded receiver.

The box includes reaction, tracking and stopping reserve and is checked without
subdivision, so joints may move or brake at different times. One receiver must
own every actuator represented by this mechanical model. Physical qualification
of calibration, stopping bounds and scope remains a separate deployment gate.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
import time

from .approach import (approach_speed, limit_reaction_s, limit_reserve, motion_reserve, reserve_toward,
                       speed_bounds)
from .collision_guard import JointBoxBound, JointCalibration, check_joint_box, check_linear_motion

# How far a joint's reading may move between an offer and the receiver's check of the box built on it, while
# nothing drives it: a RobStride count is 0.00038 rad and readings at rest flicker by one or two.
READING_FLICKER = .005

FULL_TURN = 2 * math.pi


def calibration_revision(mapping):
    normalized = {name: asdict(value) if isinstance(value, JointCalibration) else value
                  for name, value in mapping.items()}
    return hashlib.sha256(json.dumps(normalized, sort_keys=True, allow_nan=False).encode()).hexdigest()


class CollisionEnvelopeCompiler:
    def __init__(self, backend, config, joint_map, *, clearance_m=.005,
                 max_skew_ms=10, compute_budget_s=.03, cad_limits=None, clock=time.perf_counter):
        self.backend, self.config = backend, deepcopy(config)
        # Gripper slides travel a little past the model's closed limit; clamp them for queries.
        self.cad_limits = dict(cad_limits or {})
        self.clock, self.compute_budget_s = clock, compute_budget_s
        self.clearance_m, self.max_skew_ms = clearance_m, max_skew_ms
        self.mapping = {name: value if isinstance(value, JointCalibration) else JointCalibration(**value)
                        for name, value in joint_map.items()}
        self.names = tuple(joint["name"] for joint in config["joints"])
        if (len(set(self.names)) != len(self.names)
                or set(self.mapping) != set(backend.runtime_model.scene.independent_joints)
                or {value.actuator for value in self.mapping.values()} != set(self.names)):
            raise ValueError("One receiver must cover every mapped mechanical joint and actuator")
        if any(not math.isfinite(v.scale) or not v.scale or not math.isfinite(v.offset)
               for v in self.mapping.values()):
            raise ValueError("Invalid mechanical transmission map")
        if (not math.isfinite(clearance_m) or clearance_m < 0 or not math.isfinite(compute_budget_s)
                or compute_budget_s <= 0 or not math.isfinite(max_skew_ms) or max_skew_ms <= 0):
            raise ValueError("Invalid clearance, skew or compute budget")
        policy = self.config.get("collision")
        if (not policy or policy["scope"] not in ("all", "inter_branch")
                or not 0 < policy["max_verdict_ms"] <= config["lease_ms"]
                or not 0 <= policy["reaction_ms"] <= 1000):
            raise ValueError("Explicit receiver collision policy required")
        if compute_budget_s * 1000 >= policy["max_verdict_ms"]:
            raise ValueError("Collision computation must leave time for dispatch")
        for joint in self.config["joints"]:
            if any(not math.isfinite(joint[key]) or joint[key] <= 0
                   for key in ("max_velocity", "max_tracking_error", "stop_acceleration")):
                raise ValueError("Finite positive motion/stopping bounds required")
        self._check_revisions()
        self.last_verdict = None

    def _check_revisions(self):
        config, backend = self.config, self.backend
        if (config["identity"]["model"] != backend.runtime_model.revision
                or config["identity"]["calibration"] != calibration_revision(self.mapping)
                or config["identity"]["collision_policy"] != backend.policy_revision
                or config["collision"]["world_revision"] != backend.runtime_model.world_revision):
            raise ValueError("Model, calibration, collision policy or world revision mismatch")

    def compile(self, offered, feedback, joints=None, *, started_at=None):
        """Use the feedback returned with an offer; None joints means current hold.

        started_at should be sampled just before requesting the receiver offer.
        All CAN timestamps are compared only to that receiver's clock. The Rust
        receiver independently checks actual age and current state on dispatch.
        Returns a wire envelope only for a clear, timely complete box.
        """
        started = self.clock() if started_at is None else started_at
        deadline = started + self.compute_budget_s
        verdict = {"clear": False, "reason": "unavailable", "queries": 0,
                   "query_scope": self.config["collision"]["scope"],
                   "ignored_links": dict(self.backend.ignored_links), "started_at": started}
        self.last_verdict = verdict
        try:
            self._check_revisions()
            status, offer = offered["status"], offered["offer"]
            now = status["now_ms"]
            if (not offered["accepted"] or status["identity"] != self.config["identity"]
                    or status["group"] != self.config["group"]
                    or tuple(status["joint_names"]) != self.names
                    or len(feedback) != len(self.names)
                    or not 0 <= offer["issued_at_ms"] <= now
                    or now >= offer["issued_at_ms"] + self.config["collision"]["max_verdict_ms"]):
                raise ValueError("Receiver offer, revision, roster or age mismatch")
            stamps = [sample["sampled_at_ms"] for sample in feedback]
            if max(stamps) - min(stamps) > self.max_skew_ms:
                raise ValueError("Unsynchronized joint feedback")
            for sample, limit in zip(feedback, self.config["joints"]):
                if (sample["sequence"] <= 0 or sample["fault"]
                        or not 0 <= now - sample["sampled_at_ms"] < self.config["feedback_timeout_ms"]
                        or not math.isfinite(sample["position"]) or not math.isfinite(sample["velocity"])
                        or abs(sample["velocity"]) > limit["max_velocity"]):
                    raise ValueError("Stale, faulted or invalid joint feedback")
            target = [sample["position"] for sample in feedback] if joints is None else [j["position"] for j in joints]
            if len(target) != len(self.names) or any(not math.isfinite(value) for value in target):
                raise ValueError("Complete finite joint targets required")
            # The receiver ramps each command from where its velocity says the stream came from.
            period = self.config.get("interpolation_ms", 0) / 1000
            starts = target if joints is None else [j["position"] - j["velocity"] * period for j in joints]
            if any(not math.isfinite(value) for value in starts):
                raise ValueError("Complete finite joint targets required")
            lower, upper = [], []
            policy = self.config["collision"]
            for sample, goal, start, limit in zip(feedback, target, starts, self.config["joints"]):
                lo, hi = min(sample["position"], goal, start), max(sample["position"], goal, start)
                velocity = limit["max_velocity"]
                # Allow state progression while collision computation/transport run.
                padding = velocity * policy["max_verdict_ms"] / 1000 + 1e-9
                if limit.get("gripper"):
                    # No stopping reserve: the receiver keeps a gripper's last target
                    # instead of braking, and a gripper never leaves its interval.
                    low = max(limit["lower"], lo - padding)
                    high = min(limit["upper"], hi + padding)
                else:
                    # Each side's reserve at the fastest the joint may approach that limit (the
                    # receiver's braking curve): near a limit the box reaches it, no further. The
                    # reference the receiver drives (the command's ramp) must stop at the limit on its
                    # limit curve, as the receiver checks; the measured position it holds every tick.
                    reaction = (policy["max_verdict_ms"] + self.config["feedback_timeout_ms"]
                                + policy["reaction_ms"]) / 1000
                    down, up = lo - limit["lower"], limit["upper"] - hi
                    reserve_low, reserve_high = reserve_toward(limit, reaction, down), reserve_toward(limit, reaction, up)
                    stop = limit_reaction_s(self.config)
                    driven_lo, driven_hi = min(goal, start), max(goal, start)
                    if (driven_lo - limit_reserve(limit, stop, driven_lo - limit["lower"]) < limit["lower"] - 1e-9
                            or driven_hi + limit_reserve(limit, stop, limit["upper"] - driven_hi)
                            > limit["upper"] + 1e-9):
                        raise ValueError("Stopping envelope exceeds operational interval")
                    verdict_s = policy["max_verdict_ms"] / 1000
                    low = max(limit["lower"], lo - reserve_low - approach_speed(limit, reaction, down) * verdict_s - 1e-9)
                    high = min(limit["upper"], hi + reserve_high + approach_speed(limit, reaction, up) * verdict_s + 1e-9)
                if low < limit["lower"] or high > limit["upper"]:
                    raise ValueError("Stopping envelope exceeds operational interval")
                lower.append(low)
                upper.append(high)
            bounds = dict(zip(self.names, zip(lower, upper)))
            a, b = {}, {}
            for name, calibration in self.mapping.items():
                values = [q * calibration.scale + calibration.offset for q in bounds[calibration.actuator]]
                if name in self.cad_limits and self.cad_limits[name][1] - self.cad_limits[name][0] < 2 * math.pi - 1e-9:
                    # A gripper slide passes the model's closed limit; clamp it. A box of a full-turn joint
                    # past the model's range is not wrapped: the model refuses it and no envelope is made.
                    low, high = self.cad_limits[name]
                    values = [min(high, max(low, value)) for value in values]
                a[name], b[name] = min(values), max(values)
            result = check_linear_motion(self.backend, a, b, clearance_m=self.clearance_m,
                                         deadline=deadline, clock=self.clock, subdivide=False,
                                         scope=policy["scope"])
            verdict.update(result)
            self._check_revisions()
            if not result["clear"] or self.clock() >= deadline:
                raise ValueError(result["reason"] if not result["clear"] else "collision_deadline")
            verdict["completed_at"] = self.clock()
            return {"offer": offer["id"], "world_revision": policy["world_revision"],
                    "scope": policy["scope"], "lower": lower, "upper": upper}
        except (ValueError, KeyError, TypeError, RuntimeError, OSError) as exc:
            verdict.update(clear=False, reason=str(exc), completed_at=self.clock())
            raise ValueError(f"Collision envelope unavailable: {exc}") from exc


class CollisionBlocked(ValueError):
    """A command's box could not be certified clear: a pair could come too close within it, or the check ran out
    of time. The adapter holds the arms instead of faulting. `verdict` has the check's details."""

    def __init__(self, message, verdict=None):
        super().__init__(message)
        self.verdict = dict(verdict or {})


class MotionEnvelopeCompiler:
    """Certify a command's motion-bound box (the receiver's collision::Bound::Motion).

    The box holds the reference the receiver drives from now on: the command in force, carried on along its ramp,
    then this command's ramp; widened by each joint's tracking bound and its stop from its speed bound, both of
    which the receiver enforces every tick. One proximity query at its centre certifies every pose in it
    (collision_guard.JointBoxBound), joints moving or stopping at different times. This mirrors the receiver's
    check (lib.rs validate_collision) from the offer's status, conservatively: the speed bounds it reports only
    fall, and its reference only moves on along its ramp, before the receiver checks the command.

    A box that cannot be certified raises CollisionBlocked, and the adapter holds instead. Model joints outside
    this receiver (none when it drives both arms) are taken at `fixed`, their parked pose.

    Holding where a box just certifies would leave the arms there: a move away certifies no more easily (a box
    reaching from here to there is no easier than here). So a box certified within `approach_margin_m` of the
    clearance passes only if the arms could rest at its target with the whole margin, or its move keeps the
    parts that limit the box at least as far apart (nominally, at its target against the reference in force): the
    arms stop approaching where resting keeps the margin, and leave from there.
    """

    def __init__(self, backend, config, joint_map, *, clearance_m=.005, approach_margin_m=.02, max_skew_ms=10,
                 compute_budget_s=.012, cad_limits=None, fixed=None, clock=time.perf_counter):
        self.backend, self.config = backend, deepcopy(config)
        self.cad_limits, self.fixed = dict(cad_limits or {}), dict(fixed or {})
        self.clock, self.compute_budget_s = clock, compute_budget_s
        self.clearance_m, self.max_skew_ms = clearance_m, max_skew_ms
        self.approach_margin_m = approach_margin_m
        self.mapping = {name: value if isinstance(value, JointCalibration) else JointCalibration(**value)
                        for name, value in joint_map.items()}
        self.names = tuple(joint["name"] for joint in config["joints"])
        runtime = backend.runtime_model
        if (len(set(self.names)) != len(self.names)
                or set(self.mapping) | set(self.fixed) != set(runtime.scene.independent_joints)
                or set(self.mapping) & set(self.fixed)
                or {value.actuator for value in self.mapping.values()} != set(self.names)):
            raise ValueError("Every model joint must follow this receiver or have a fixed pose")
        if any(not math.isfinite(v.scale) or not v.scale or not math.isfinite(v.offset)
               for v in self.mapping.values()) or any(not math.isfinite(q) for q in self.fixed.values()):
            raise ValueError("Invalid mechanical transmission map or fixed pose")
        if (not math.isfinite(clearance_m) or clearance_m < 0 or not math.isfinite(compute_budget_s)
                or compute_budget_s <= 0 or not math.isfinite(max_skew_ms) or max_skew_ms <= 0
                or not math.isfinite(approach_margin_m) or approach_margin_m < 0):
            raise ValueError("Invalid clearance, approach margin, skew or compute budget")
        policy = self.config.get("collision")
        if (not policy or policy.get("bound") != "motion" or policy["scope"] not in ("all", "inter_branch")
                or not 0 < policy["max_verdict_ms"] <= config["lease_ms"]
                or not 0 <= policy["reaction_ms"] <= 1000 or not config.get("interpolation_ms", 0) > 0):
            raise ValueError("A motion-bound receiver collision policy, with ramped commands, is required")
        if compute_budget_s * 1000 >= policy["max_verdict_ms"]:
            raise ValueError("Collision computation must leave time for dispatch")
        for joint in self.config["joints"]:
            if any(not math.isfinite(joint[key]) or joint[key] <= 0
                   for key in ("max_velocity", "max_tracking_error", "stop_acceleration")):
                raise ValueError("Finite positive motion/stopping bounds required")
        self._check_revisions()
        self.bound = JointBoxBound(runtime.scene, runtime.keepout_motion_corners)
        self._joints = [(name, self.names.index(value.actuator), value) for name, value in self.mapping.items()]
        self.last_verdict = None
        self.blocked = 0

    def _check_revisions(self):
        config, backend = self.config, self.backend
        if (config["identity"]["model"] != backend.runtime_model.revision
                or config["identity"]["calibration"] != calibration_revision(self.mapping)
                or config["identity"]["collision_policy"] != backend.policy_revision
                or config["collision"]["world_revision"] != backend.runtime_model.world_revision):
            raise ValueError("Model, calibration, collision policy or world revision mismatch")

    def box(self, status, feedback, joints=None):
        """Per actuator, the box ((lower, ...), (upper, ...)) the receiver requires of this command, from the
        offer's `status` and `feedback`; None joints: the hold an Arm takes."""
        period = self.config["interpolation_ms"] / 1000
        reaction = limit_reaction_s(self.config)
        verdict_s = self.config["collision"]["max_verdict_ms"] / 1000
        active = status["state"] in ("holding", "executing", "stopping", "stopped")
        bounds = status["speed_bounds"]
        lower, upper = [], []
        for i, (sample, limit) in enumerate(zip(feedback, self.config["joints"])):
            command = None if joints is None else joints[i]
            target = sample["position"] if command is None else command["position"]
            velocity = 0. if command is None else command["velocity"]
            start = target - velocity * period
            if active:
                # The receiver's reference moves on along the ramp in force toward its end before it checks this
                # command.
                points = (status["joints"][i]["position"], status["reference_ends"][i], start, target)
            else:
                # Not yet driven, the receiver boxes its own newest reading, taken after the offer's: a drive's
                # reading flickers at rest, and the first live two-arm Enable was refused for one count
                # (2026-10-09).
                points = (sample["position"] - READING_FLICKER, sample["position"] + READING_FLICKER, start,
                          target)
            lo, hi = min(points), max(points)
            if not all(math.isfinite(value) for value in (lo, hi, *bounds[i])):
                raise ValueError("Complete finite joint targets and speed bounds required")
            if limit.get("gripper"):
                # It may trail its command (stalled on an object) and move on toward it before the receiver checks:
                # its measured position, padded by its speed over the verdict's lifetime. The keep-out model ignores
                # the fingers, so this costs the check nothing.
                pad = limit["max_velocity"] * verdict_s
                lo, hi = min(lo, sample["position"]) - pad, max(hi, sample["position"]) + pad
                below = above = 0.
            else:
                # The reference must stop on the joint's limit curve, as the receiver checks.
                low, high = min(start, target), max(start, target)
                if (low - limit_reserve(limit, reaction, low - limit["lower"]) < limit["lower"] - 1e-9
                        or high + limit_reserve(limit, reaction, limit["upper"] - high) > limit["upper"] + 1e-9):
                    raise ValueError("Stopping envelope exceeds operational interval")
                # A joint moving away from something keeps its stop on the far side.
                commanded = speed_bounds(limit, velocity)
                below, above = motion_reserve(limit, reaction, (max(bounds[i][0], commanded[0]),
                                                                max(bounds[i][1], commanded[1])))
            lower.append(max(limit["lower"], lo - below - 1e-9))
            upper.append(min(limit["upper"], hi + above + 1e-9))
        return lower, upper

    def model_box(self, lower, upper):
        """The box in model joints: (centre, half-width) per model joint, the fixed ones at their pose."""
        center, half = dict(self.fixed), {name: 0. for name in self.fixed}
        for name, index, calibration in self._joints:
            a, b = sorted(q * calibration.scale + calibration.offset for q in (lower[index], upper[index]))
            limits = self.cad_limits.get(name)
            if limits and limits[1] - limits[0] >= FULL_TURN - 1e-9:
                # A joint the model turns a full circle is taken a turn round into its range (the same pose).
                mid = (a + b) / 2
                center[name], half[name] = limits[0] + (mid - limits[0]) % FULL_TURN, (b - a) / 2
                continue
            if limits:
                # A gripper slide passes the model's closed limit: clamp it.
                a, b = (min(limits[1], max(limits[0], value)) for value in (a, b))
            center[name], half[name] = (a + b) / 2, (b - a) / 2
        return center, half

    def compile(self, offered, feedback, joints=None, *, started_at=None):
        """A wire envelope for this command (None joints: an Arm's hold), from the offer's status and feedback;
        CollisionBlocked if its box is not clear in time, ValueError if the inputs are unusable."""
        started = self.clock() if started_at is None else started_at
        deadline = started + self.compute_budget_s
        policy = self.config["collision"]
        verdict = {"clear": False, "reason": "unavailable", "queries": 0, "query_scope": policy["scope"],
                   "bound": "motion", "started_at": started}
        self.last_verdict = verdict
        try:
            self._check_revisions()
            status, offer = offered["status"], offered["offer"]
            now = status["now_ms"]
            if (not offered["accepted"] or status["identity"] != self.config["identity"]
                    or status["group"] != self.config["group"]
                    or tuple(status["joint_names"]) != self.names
                    or len(feedback) != len(self.names)
                    or not 0 <= offer["issued_at_ms"] <= now
                    or now >= offer["issued_at_ms"] + policy["max_verdict_ms"]):
                raise ValueError("Receiver offer, revision, roster or age mismatch")
            stamps = [sample["sampled_at_ms"] for sample in feedback]
            if max(stamps) - min(stamps) > self.max_skew_ms:
                raise ValueError("Unsynchronized joint feedback")
            for sample, limit in zip(feedback, self.config["joints"]):
                if (sample["sequence"] <= 0 or sample["fault"]
                        or not 0 <= now - sample["sampled_at_ms"] < self.config["feedback_timeout_ms"]
                        or not math.isfinite(sample["position"]) or not math.isfinite(sample["velocity"])):
                    raise ValueError("Stale, faulted or invalid joint feedback")
            if joints is not None and (len(joints) != len(self.names) or any(
                    not math.isfinite(joint["position"]) or not math.isfinite(joint["velocity"]) for joint in joints)):
                raise ValueError("Complete finite joint targets required")
            lower, upper = self.box(status, feedback, joints)
            center, half = self.model_box(lower, upper)
            result = check_joint_box(self.backend, self.bound, center, half, clearance_m=self.clearance_m,
                                     margin_m=self.approach_margin_m, scope=policy["scope"], deadline=deadline,
                                     clock=self.clock)
            verdict.update(result)
            if result.get("shapes"):
                shapes = self.backend.runtime_model.shapes
                index = self.backend._motion_shape_indices
                verdict["links"] = [shapes[index[name]]["link"] for name in result["shapes"]]
            margin = self.clearance_m + self.approach_margin_m
            if result["clear"] and joints is not None and result["clearance_lower_bound_m"] < margin:
                # Within the approach margin: a move whose target the arms could rest at with the whole margin, or
                # one that keeps their closest parts as far apart.
                target = [joint["position"] for joint in joints]
                rest = check_joint_box(self.backend, self.bound, *self.model_box(*self.rest_box(feedback, target)),
                                       clearance_m=margin, scope=policy["scope"], deadline=deadline, clock=self.clock)
                verdict["rest_clearance_m"] = rest.get("clearance_lower_bound_m")
                if not rest["clear"]:
                    held = [row["position"] for row in status["joints"]] if status["state"] in (
                        "holding", "executing", "stopping", "stopped") else [row["position"] for row in feedback]
                    pair = tuple(sorted(result["shapes"]))
                    now_m, next_m = (self._apart(dict(zip(self.names, pose)), pair, policy["scope"])
                                     for pose in (held, target))
                    verdict.update(nearest_m=now_m, next_nearest_m=next_m)
                    if next_m < now_m - 1e-4:
                        verdict.update(clear=False, reason="approaching_within_margin")
            self._check_revisions()
            if not verdict["clear"] or self.clock() >= deadline:
                if verdict["clear"]:
                    verdict.update(clear=False, reason="collision_deadline")
                self.blocked += 1
                raise CollisionBlocked(self.describe(verdict), verdict)
            verdict["completed_at"] = self.clock()
            return {"offer": offer["id"], "world_revision": policy["world_revision"],
                    "scope": policy["scope"], "lower": lower, "upper": upper}
        except CollisionBlocked:
            verdict["completed_at"] = self.clock()
            raise
        except (ValueError, KeyError, TypeError, RuntimeError, OSError) as exc:
            verdict.update(clear=False, reason=str(exc), completed_at=self.clock())
            raise ValueError(f"Collision envelope unavailable: {exc}") from exc

    def rest_box(self, feedback, positions):
        """The box of the arms at rest at `positions`: each joint's rest reserve (its tracking bound at rest and a
        stop from its approach floor) around it; a gripper's padded as in box()."""
        reaction = limit_reaction_s(self.config)
        verdict_s = self.config["collision"]["max_verdict_ms"] / 1000
        lower, upper = [], []
        for q, sample, limit in zip(positions, feedback, self.config["joints"]):
            if limit.get("gripper"):
                pad = limit["max_velocity"] * verdict_s
                lo, hi = min(q, sample["position"]) - pad, max(q, sample["position"]) + pad
            else:
                below, above = motion_reserve(limit, reaction, speed_bounds(limit, 0.))
                lo, hi = q - below, q + above
            lower.append(max(limit["lower"], lo))
            upper.append(min(limit["upper"], hi))
        return lower, upper

    def _apart(self, motors, pair, scope):
        """How far apart (m) the two parts `pair` are with the receiver's joints at `motors`, nominally; 1 m when
        farther."""
        center, _ = self.model_box([motors[name] for name in self.names], [motors[name] for name in self.names])
        pairs = self.backend.proximity(center, threshold=1., scope=scope)["near_pairs"]
        return next((row["distance_m"] for row in pairs if tuple(sorted(row["shapes"])) == pair), 1.)

    @staticmethod
    def describe(verdict):
        """In words, why a box was not certified."""
        links = verdict.get("links") or verdict.get("shapes") or ["two parts"]
        if verdict.get("reason") == "insufficient_swept_clearance":
            return (f"{' and '.join(links)} could come within {max(0., verdict['clearance_lower_bound_m']) * 1000:.0f}"
                    f" mm of each other ({verdict['distance_m'] * 1000:.0f} mm apart now)")
        if verdict.get("reason") == "approaching_within_margin":
            return (f"the arms are as close as they may come ({' and '.join(links)}, "
                    f"{verdict['nearest_m'] * 1000:.0f} mm apart): only moves apart")
        return f"collision check not clear: {verdict.get('reason')}"
