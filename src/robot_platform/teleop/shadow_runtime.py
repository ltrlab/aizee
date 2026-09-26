"""Live GELLO input and the existing GUI lifecycle with a virtual receiver only.

The calibration in this preview defines illustrative CAD targets. It cannot be
used to authorize physical actuation, loaded gains, stopping or joint travel.
"""
from copy import deepcopy
import json
from pathlib import Path
import time

from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler
from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.control_harness import NativeControlHarness
from robot_platform.simulation.guarded_fixture import synthetic_config
from robot_platform.simulation.native_collision import NativeCollisionBackend
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
from .absolute_mapping import AbsoluteLeaderMapper
from .guarded_lifecycle import GuardedLifecycleSession, minerva_action
from .lifecycle import TeleopLifecycle
from .viewer_input import ViewerLeaderInput


class ShadowTeleopRuntime:
    period_s = .04

    def __init__(self, model_path, calibration_path, *, leader_url="http://127.0.0.1:8767/"):
        self.source = self.backend = self.harness = self.session = None
        self.fault = None
        self.input_error = None
        self.late_targets = 0
        self.last_target = None
        self.message = "VIRTUAL RECEIVER · Live GELLO input; synthetic follower parameters. Idle (I) first."
        self.cycles = []
        self.cfg = None
        try:
            profile = ModelProfile.load(Path(model_path))
            self.model = UrdfModel(profile)
            self.backend = NativeCollisionBackend(RuntimeCollisionModel(profile), geometry_mode="keepout")
            self.cfg, mapping, _ = synthetic_config(self.model, self.backend)
            # Use the model's interior reference in the fixture. Closed finger
            # endpoints cannot contain a two-sided conservative stopping reserve.
            self.initial = [self.model.by_name[n]["default"] for n in self.model.independent]
            for _ in range(10):
                self.backend.proximity(dict(zip(self.model.independent, self.initial)), scope="inter_branch")
            self.source = ViewerLeaderInput(leader_url)
            calibration = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
            live_profile = self.source._request("GET", "/api/profile")
            if (calibration.get("kind") != "leader_viewer_display_calibration"
                    or calibration.get("display_only") is not True
                    or calibration.get("runtime_approved") is not False
                    or calibration.get("profile_sha256") != live_profile["profile_sha256"]
                    or calibration.get("pins") != self.source.pins):
                raise ValueError("Saved display reference must match the current leader profile and USB identities")
            self.routes = []
            lifecycle_joints = []
            for limit, initial in zip(self.cfg["joints"], self.initial):
                name = limit["name"]
                side = "left" if name.startswith("left_") else "right" if name.startswith("right_") else None
                if side not in calibration["leaders"]:
                    raise ValueError("Minerva shadow adapter needs explicit left/right arm routes")
                sliding = self.model.by_name[name]["type"] == "prismatic"
                channel = 6 if sliding else int(name.rsplit("j", 1)[1]) - 1
                ref = calibration["leaders"][side][channel]
                # A narrow illustrative range around the existing model reference
                # leaves the receiver's stopping reserve inside its interval.
                lower = max(limit["lower"] + 4 * limit["max_tracking_error"], initial - (.004 if sliding else .3))
                upper = min(limit["upper"] - 4 * limit["max_tracking_error"], initial + (.004 if sliding else .3))
                if not lower < initial < upper:
                    raise ValueError("Shadow model reference must be inside the stopping interval")
                scale = (-.01 if side == "right" else .01) if sliding else 1.
                sign = ref["direction"] * (1 if scale > 0 else -1)
                ends = [ref["zero_rad"] + (q - initial) / (sign * abs(scale)) for q in (lower, upper)]
                self.routes.append(dict(actuator=name, leader=side, channel=channel,
                    raw_min=min(ends), raw_max=max(ends), follower_min=lower, follower_max=upper, direction=sign))
                lifecycle_joints.append({**limit, "align_speed": limit["max_velocity"],
                    "shutdown_speed": limit["max_velocity"] / 2,
                    "max_lead": limit["max_tracking_error"],
                    "tolerance": limit["max_tracking_error"] / 2,
                    "settled_velocity": limit["max_velocity"] / 10,
                    "zero_position": initial})
            self.mapper = AbsoluteLeaderMapper(self.cfg["joints"], self.routes,
                max_age_s=self.source.max_age_s, max_skew_s=self.source.max_skew_s)
            lifecycle = TeleopLifecycle(lifecycle_joints)
            compiler = CollisionEnvelopeCompiler(self.backend, self.cfg, mapping, compute_budget_s=.025)
            self.harness = NativeControlHarness(self.cfg, self.initial)
            self.harness.step(0)
            self.harness.timeout_s = .035
            self.epoch = time.perf_counter()

            def exchange(request):
                now_ms = max(self.harness.now_ms, int((time.perf_counter() - self.epoch) * 1000))
                return self.harness.step(now_ms, request)["reply"]

            controller = GuardedControllerClient(exchange, group=self.cfg["group"],
                identity=self.cfg["identity"], joint_names=tuple(self.model.independent))
            self.session = GuardedLifecycleSession(controller, compiler, lifecycle, lambda: self.harness.last)
        except BaseException:
            self.close()
            raise

    def _recent_cycle_s(self):
        """90th-percentile duration of recent cycles, used as a dispatch margin."""
        durations = sorted(c["completed_at"] - c["started_at"] for c in self.cycles[-25:]
                           if c.get("completed_at") is not None)
        return durations[int(.9 * (len(durations) - 1))] if durations else 0.

    def _leader_target(self, engaged):
        """A fresh mapped target, or None and the reason. Input never faults the robot."""
        if self.mapper.fault and not engaged:
            # Outside engagement leaders only preview; Engage (T) stays explicit.
            self.mapper.reset()
        try:
            target = self.mapper.map(self.source.poll(), now=time.perf_counter())
        except Exception as exc:
            return None, str(exc) or type(exc).__name__
        self.last_target = target
        # A sample that cannot reach the receiver within its freshness limit is a
        # missed cycle, not a stale command.
        margin = max(self._recent_cycle_s(), .01)
        age = time.perf_counter() - min(row["sampled_at"] for row in target)
        if age + margin >= self.session.lifecycle.target_max_age_s:
            self.late_targets += 1
            return None, None
        return target, None

    def tick(self, key=None):
        started = time.perf_counter()
        if self.fault and key != "H":
            return self.snapshot()
        try:
            previous = self.session.lifecycle.state
            engaged = previous in ("ENGAGING", "TELEOP")
            action = minerva_action(key, previous)
            target = None
            # Disable must work independently of a failed input source.
            if key != "H":
                target, self.input_error = self._leader_target(engaged)
            notice = None
            if key and action is None:
                notice = f"[{key}] Unavailable in this virtual receiver preview. Use I / E / T / H / X."
            # An invalid GUI phase action should leave the current phase intact.
            allowed = {"enable": ("IDLE",), "engage": ("IDLE", "HOLD"),
                "disengage": ("ENGAGING", "TELEOP"),
                "shutdown": ("IDLE", "HOLD", "ENGAGING", "TELEOP")}
            if action in allowed and previous not in allowed[action]:
                notice = f"[{key}] Unavailable from {previous}; use Idle before Enable."
                action = None
            if action == "engage" and target is None:
                reason = self.input_error or "sample arrived too late for this cycle; press T again"
                notice = f"[{key}] Engage needs a live leader feed: {reason}"
                action = None
            record = self.session.cycle(action=action, target=target, started_at=started)
            self.cycles.append(deepcopy(record))
            if len(self.cycles) > 1500:
                self.cycles.pop(0)
            current = self.session.lifecycle.state
            if notice:
                self.message = notice
            elif current != previous:
                self.message = f"VIRTUAL RECEIVER · {current} · coarse inter-arm collision checks active for holds/motion."
                if current == "HOLD" and engaged and self.session.lifecycle.reason:
                    self.message = (f"VIRTUAL RECEIVER · HOLD · {self.session.lifecycle.reason} · "
                                    f"last input problem: {self.input_error}")
        except Exception as exc:
            self.fault = str(exc)
            self.message = f"VIRTUAL RECEIVER FAULT · {self.fault} · Explicit restart required. H disables."
            self.session.fault(exc)
        return self.snapshot()

    def _display_vector(self, rows):
        values = {row["name"]: row["position"] for row in rows or []}
        result = []
        for side in ("left", "right"):
            result.extend(values.get(f"{side}_arm_j{i}", float("nan")) for i in range(1, 7))
            # Existing GUI expects gripper motor radians, not CAD metres. The
            # illustrative conversion is presentation only; absent axes stay NaN.
            q = values.get(f"{side}_gripper_finger_a_slide", float("nan"))
            result.append(q / (.01 if side == "left" else -.01))
        return result + [float("nan")] * 3

    def snapshot(self):
        state = self.session.lifecycle.state
        target = self.session.lifecycle.target
        rows = None if target is None else [dict(name=n, position=q) for n, q in zip(self.model.independent, target)]
        feedback = self.session.last_feedback
        return dict(state=state, qpos=self._display_vector(feedback),
            target=self._display_vector(rows),
            leader=self._display_vector(None if self.input_error else self.last_target),
            torque=[float("nan")] * 17, temp=[float("nan")] * 17,
            recording=False, rec_steps=0, dropped=0, kp_scale=0.,
            grip_ff_ma={"left": 0, "right": 0}, grav_on=False, grav_peak=0.,
            battery=None, estop=None, jetson="VIRTUAL RECEIVER · PC", host=None,
            telem_age=0. if feedback and not self.fault else float("inf"), cam_ages={},
            arm_ages={s: 0. if feedback and not self.fault else float("inf") for s in ("left", "right")},
            present={s: bool(feedback and not self.fault) for s in ("left", "right")},
            leaders={s: self.last_target is not None and not self.input_error for s in ("left", "right")},
            message=self.message, last_saved=None, language_instruction="", current_label="", seg_count=0,
            robot_ok=bool(feedback and not self.fault), both_ok=bool(feedback and not self.fault),
            simulation_only=True, physical_validation=False)

    def close(self):
        if self.session is not None:
            try:
                self.session.disable()
            except Exception:
                self.session.fault("shadow_gui_closed")
        for obj in (self.source, self.harness, self.backend):
            if obj is not None:
                obj.close()
        self.source = self.harness = self.backend = None
