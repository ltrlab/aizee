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

from .collision_guard import JointCalibration, check_linear_motion


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
            if joints is not None and any(j["velocity"] != 0 or j["effort"] != 0 for j in joints):
                raise ValueError("Collision envelopes currently require position targets with zero feedforward")
            lower, upper = [], []
            policy = self.config["collision"]
            for sample, goal, limit in zip(feedback, target, self.config["joints"]):
                velocity = limit["max_velocity"]
                # Allow state progression while collision computation/transport run.
                padding = velocity * policy["max_verdict_ms"] / 1000 + 1e-9
                if limit.get("gripper"):
                    # No stopping reserve: the receiver keeps a gripper's last target
                    # instead of braking, and a gripper never leaves its interval.
                    low = max(limit["lower"], min(sample["position"], goal) - padding)
                    high = min(limit["upper"], max(sample["position"], goal) + padding)
                else:
                    reserve = (limit["max_tracking_error"] + velocity * (
                        policy["max_verdict_ms"] + self.config["feedback_timeout_ms"] + policy["reaction_ms"]) / 1000
                        + velocity ** 2 / (2 * limit["stop_acceleration"]))
                    low = min(sample["position"], goal) - reserve - padding
                    high = max(sample["position"], goal) + reserve + padding
                if low < limit["lower"] or high > limit["upper"]:
                    raise ValueError("Stopping envelope exceeds operational interval")
                lower.append(low)
                upper.append(high)
            bounds = dict(zip(self.names, zip(lower, upper)))
            a, b = {}, {}
            for name, calibration in self.mapping.items():
                values = [q * calibration.scale + calibration.offset for q in bounds[calibration.actuator]]
                if name in self.cad_limits:
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
