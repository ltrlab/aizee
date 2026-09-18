"""Synthetic joint plant and approximate controller for application tests.

Parameters are deliberately synthetic, not RobStride specifications. This fast
tier is not evidence about production Rust states or physical torque accuracy.
"""
from dataclasses import dataclass, asdict
import math
import random

from robot_platform.control import RobotSession


@dataclass
class Joint:
    q: float = 0.0
    v: float = 0.0
    target: float = 0.0
    target_v: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    tau_ff: float = 0.0
    tau_applied: float = 0.0
    tau_reported: float = 0.0
    enabled: bool = False
    fault: bool = False
    sequence: int = 0
    sampled_at: float = 0.0
    measured_q: float = 0.0
    measured_v: float = 0.0
    frozen: bool = False
    commanded_at: float = 0.0


class SimulatedRobot:
    """An explicitly stepped, seeded simulation with no wall clock or I/O."""
    def __init__(self, spec, settings, *, seed=0, producer_id="simulation-0"):
        self.session = RobotSession(spec, settings)
        self.spec = spec
        self.t = 0.0
        self.rng = random.Random(seed)
        self.producer_id = producer_id
        self.joints = {name: Joint() for name in spec.active_joint_names}
        self.estopped = {arm: False for arm in self.session.arms}
        self.drop_arms = set()
        self.malformed_arms = set()
        self.drive = {"linear": 0.0, "angular": 0.0}
        self.drive_at = 0.0
        self.drive_motors = ({name: {"enabled": False, "fault": False}
                              for name in ("left_wheel", "right_wheel")}
                             if "drive" in spec.capabilities else {})
        self.base_pose = [0.0, 0.0, 0.0]
        self.watchdog_s = 0.5
        # Exposed in manifests; changing these changes the synthetic profile.
        self.parameters = {"inertia": 0.1, "damping": 0.1, "torque_limit": 2.0,
                           "response_s": 0.02, "position_limit": 2.0,
                           "torque_noise_std": 0.001, "max_step_s": 0.001}

    def command(self, arm, message):
        message = self.session.validate_commands({arm: message})[arm]
        kind = message["type"]
        names = self.session.arms[arm].joints
        motion = message.get("arm_joints", {}) if kind == "bundle" else message if kind == "arm_joints" else {}
        if any(abs(q) > self.parameters["position_limit"] for q in motion.get("positions", [])):
            raise ValueError("target exceeds synthetic position envelope")
        if kind == "bundle":
            # Validate all subcommands before applying either part.
            if "drive" in message and "drive" not in self.spec.capabilities:
                raise ValueError("robot has no drive capability")
            for key, value in message.items():
                if key != "type":
                    self.command(arm, {"type": key, **value})
            return
        if kind == "drive":
            if "drive" not in self.spec.capabilities:
                raise ValueError("robot has no drive capability")
            if not self.estopped[arm] and all(m["enabled"] and not m["fault"] for m in self.drive_motors.values()):
                self.drive = {k: message[k] for k in ("linear", "angular")}
                self.drive_at = self.t
            return
        if kind in ("emergency_stop", "clear_emergency_stop"):
            self.estopped[arm] = kind == "emergency_stop"
            if self.estopped[arm]:
                self.drive = {"linear": 0.0, "angular": 0.0}
                for motor in self.drive_motors.values():
                    motor["enabled"] = False
                for name in names:
                    self.joints[name].enabled = False
                    self.joints[name].tau_applied = 0.0
            return
        if kind == "arm_joints":
            for i, name in enumerate(names):
                j = self.joints[name]
                if not j.enabled or j.fault or self.estopped[arm]:
                    continue
                def value(key, default):
                    values = message.get(key, [])
                    return values[i] if values else default
                j.target, j.target_v = message["positions"][i], message["velocities"][i]
                j.kp, j.kd, j.tau_ff = value("kp", 0.0), value("kd", 0.0), value("torques", 0.0)
                j.commanded_at = self.t
            return
        ids = message.get("motor_ids", [])
        if set(ids) - set(names) - set(self.drive_motors):
            raise ValueError("motor does not belong to selected arm")
        if kind == "mech_zero" and any(self.joints[n].enabled if n in self.joints else self.drive_motors[n]["enabled"] for n in ids):
            raise ValueError("mechanical zero requires disabled motors")
        for name in ids:
            if name in self.drive_motors:
                motor = self.drive_motors[name]
                if kind == "enable" and not self.estopped[arm] and not motor["fault"]:
                    motor["enabled"] = True
                elif kind in ("disable", "trigger_fault", "clear_fault"):
                    motor["enabled"] = False
                    if kind != "disable":
                        motor["fault"] = kind == "trigger_fault"
                    self.drive = {"linear": 0.0, "angular": 0.0}
                continue
            j = self.joints[name]
            if kind == "enable" and not self.estopped[arm] and not j.fault:
                j.enabled = True
                j.target, j.target_v, j.kp, j.kd, j.tau_ff = j.q, 0.0, 0.0, 0.0, 0.0
                j.commanded_at = self.t
            elif kind in ("disable", "trigger_fault"):
                j.enabled = False
                j.tau_applied = 0.0
                j.fault = kind == "trigger_fault" or j.fault
            elif kind == "clear_fault":
                j.fault = False
                j.enabled = False
            elif kind == "zero_position":
                j.target = j.q
            elif kind == "mech_zero":
                j.q = j.target = 0.0

    def step(self, dt):
        if type(dt) not in (int, float) or not math.isfinite(dt) or not 0 < dt <= 1:
            raise ValueError("step duration must be finite in (0, 1]")
        count = math.ceil(dt / self.parameters["max_step_s"])
        h = dt / count
        p = self.parameters
        for _ in range(count):
            self.t += h
            for j in self.joints.values():
                if self.t - j.commanded_at > self.watchdog_s:
                    j.target_v = j.tau_ff = 0.0  # approximate command-timeout hold
                request = j.kp * (j.target - j.q) + j.kd * (j.target_v - j.v) + j.tau_ff
                request = max(-p["torque_limit"], min(p["torque_limit"], request)) if j.enabled and not j.fault else 0.0
                j.tau_applied += (request - j.tau_applied) * (1 - math.exp(-h / p["response_s"]))
                j.v += h * (j.tau_applied - p["damping"] * j.v) / p["inertia"]
                j.q += h * j.v
                if abs(j.q) > p["position_limit"]:
                    j.q = math.copysign(p["position_limit"], j.q)
                    j.v = 0.0
                if not j.frozen:
                    j.sequence += 1
                    j.sampled_at = self.t
                    j.measured_q, j.measured_v = j.q, j.v
                    j.tau_reported = j.tau_applied + self.rng.gauss(0, p["torque_noise_std"])
            if self.t - self.drive_at > self.watchdog_s:
                self.drive = {"linear": 0.0, "angular": 0.0}
            self.base_pose[2] += h * self.drive["angular"]
            self.base_pose[0] += h * self.drive["linear"] * math.cos(self.base_pose[2])
            self.base_pose[1] += h * self.drive["linear"] * math.sin(self.base_pose[2])

    def telemetry(self, arm):
        if arm in self.drop_arms:
            return None
        if arm in self.malformed_arms:
            return {"motors": "injected malformed data"}
        motors = {}
        for name in self.session.arms[arm].joints:
            j = self.joints[name]
            motors[name] = {"position": j.measured_q, "velocity": j.measured_v,
                            "torque": j.tau_reported, "temperature": 25.0,
                            "state": "error" if j.fault else "running" if j.enabled else "disabled",
                            "mode": "run" if j.enabled else "reset", "error": "injected" if j.fault else None,
                            "feedback": {"sequence": j.sequence, "age_s": max(0.0, self.t-j.sampled_at),
                                         "valid": j.sequence > 0 and not j.fault, "provenance": "simulation"}}
        return {"telemetry_schema_version": 2, "producer_id": self.producer_id,
                "timestamp": self.t, "motors": motors, "emergency_stop": self.estopped[arm],
                "simulation": {"evidence": "synthetic", "clock": "simulation", "base_pose": self.base_pose[:]}}

    def snapshot(self):
        return {"time_s": self.t, "joints": {n: asdict(j) for n, j in self.joints.items()},
                "base_pose": self.base_pose[:], "drive_motors": {n: dict(m) for n, m in self.drive_motors.items()},
                "evidence": "synthetic"}
