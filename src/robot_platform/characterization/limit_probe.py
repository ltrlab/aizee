"""Offline contact-candidate detector, not a motor command or limit writer.

All thresholds must be supplied for a particular experiment. A candidate can
be an obstruction, cable load or friction; it never establishes a hard stop.
Times must share one monotonic clock and sequences identify new source samples.
"""
from collections import deque
from dataclasses import asdict, dataclass
import math
from statistics import median


@dataclass(frozen=True)
class ProbeConfig:
    direction: int
    baseline_torque_nm: float
    torque_rise_nm: float
    torque_abort_nm: float
    window_s: float
    min_samples: int
    stall_span_rad: float
    stall_velocity_rad_s: float
    tracking_error_rad: float
    min_command_travel_rad: float
    max_duration_s: float
    max_travel_rad: float
    max_velocity_rad_s: float
    max_feedback_age_s: float
    max_sample_gap_s: float
    max_temperature_c: float

    def __post_init__(self):
        if type(self.direction) is not int or self.direction not in (-1, 1):
            raise ValueError("direction must be -1 or +1")
        if type(self.min_samples) is not int or self.min_samples < 3:
            raise ValueError("min_samples must be at least 3")
        for name, value in asdict(self).items():
            if name in ("direction", "min_samples"):
                continue
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            if name != "baseline_torque_nm" and value <= 0:
                raise ValueError(f"{name} must be positive")
        if abs(self.baseline_torque_nm) + self.torque_rise_nm >= self.torque_abort_nm:
            raise ValueError("contact threshold must leave room below the absolute torque abort")
        if self.window_s >= self.max_duration_s:
            raise ValueError("detection window must be shorter than the experiment")
        if self.stall_velocity_rad_s >= self.max_velocity_rad_s:
            raise ValueError("stall velocity must be below the velocity cap")


@dataclass(frozen=True)
class ProbeSample:
    time_s: float
    source_time_s: float
    sequence: int
    producer_id: str
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    target_rad: float
    temperature_c: float
    collision_clear: bool
    fault: bool = False


@dataclass(frozen=True)
class ProbeResult:
    state: str
    reason: str
    position_rad: float | None = None
    observed_span_rad: float | None = None
    accepted_limit: bool = False


class ContactDetector:
    """Fail closed on invalid samples; terminal decisions latch until a new run.

    A caller must separately stop on absent messages/timeouts. This offline
    classifier has no clock thread, control authority or collision engine.
    ``collision_clear`` is an input from a separately checked scene, not evidence
    that this detector has checked any geometry.
    """
    def __init__(self, config: ProbeConfig):
        self.config = config
        self.samples = deque()
        self.first = None
        self.previous = None
        self.result = ProbeResult("probing", "insufficient_evidence")

    def abort(self, reason: str) -> ProbeResult:
        if self.result.state == "probing":
            self.result = ProbeResult("aborted", reason)
        return self.result

    def observe(self, sample: ProbeSample) -> ProbeResult:
        if self.result.state != "probing":
            return self.result
        c = self.config
        for name in ("time_s", "source_time_s", "position_rad", "velocity_rad_s",
                     "torque_nm", "target_rad", "temperature_c"):
            value = getattr(sample, name)
            if type(value) not in (int, float) or not math.isfinite(value):
                return self.abort("invalid_numeric_sample")
        if (type(sample.sequence) is not int or sample.sequence < 0
                or not isinstance(sample.producer_id, str) or not sample.producer_id):
            return self.abort("invalid_source_identity")
        if type(sample.fault) is not bool or sample.fault:
            return self.abort("motor_fault_or_unknown")
        if sample.collision_clear is not True:
            return self.abort("collision_or_unknown_scene")
        age = sample.time_s - sample.source_time_s
        if age < 0 or age > c.max_feedback_age_s:
            return self.abort("stale_or_future_feedback")
        if abs(sample.torque_nm) >= c.torque_abort_nm:
            return self.abort("torque_cap")
        if abs(sample.velocity_rad_s) >= c.max_velocity_rad_s:
            return self.abort("velocity_cap")
        if sample.temperature_c >= c.max_temperature_c:
            return self.abort("temperature_cap")
        if self.previous is not None:
            p = self.previous
            if sample.producer_id != p.producer_id:
                return self.abort("producer_changed")
            if sample.sequence <= p.sequence or sample.source_time_s <= p.source_time_s:
                return self.abort("feedback_not_advancing")
            if sample.time_s <= p.time_s:
                return self.abort("clock_not_advancing")
            if max(sample.time_s - p.time_s, sample.source_time_s - p.source_time_s) > c.max_sample_gap_s:
                return self.abort("feedback_gap")
            if c.direction * (sample.target_rad - p.target_rad) < -1e-12:
                return self.abort("command_reversed")
        if self.first is None:
            self.first = sample
        if sample.time_s - self.first.time_s >= c.max_duration_s:
            return self.abort("duration_cap")
        if max(abs(sample.position_rad - self.first.position_rad),
               abs(sample.target_rad - self.first.position_rad)) >= c.max_travel_rad:
            return self.abort("travel_cap")
        self.previous = sample
        self.samples.append(sample)
        # Retain the sample immediately before the window boundary, so a
        # non-integral sampling period cannot prevent a full-duration decision.
        while len(self.samples) > 1 and self.samples[1].source_time_s <= sample.source_time_s - c.window_s:
            self.samples.popleft()
        window = list(self.samples)
        if len(window) < c.min_samples or sample.source_time_s - window[0].source_time_s < c.window_s:
            return self.result
        positions = [s.position_rad for s in window]
        span = max(positions) - min(positions)
        stalled = span <= c.stall_span_rad and all(abs(s.velocity_rad_s) <= c.stall_velocity_rad_s for s in window)
        loaded = all(c.direction * (s.torque_nm - c.baseline_torque_nm) >= c.torque_rise_nm for s in window)
        tracking = all(c.direction * (s.target_rad - s.position_rad) >= c.tracking_error_rad for s in window)
        advancing = c.direction * (sample.target_rad - window[0].target_rad) >= c.min_command_travel_rad
        if stalled and loaded and tracking and advancing:
            self.result = ProbeResult("contact_candidate", "sustained_load_and_stall", median(positions), span)
        return self.result


def synthetic_validation() -> dict:
    """Replay fabricated signals to test decisions, not actuator physics."""
    cases = {}
    for scenario in ("upper_stop", "lower_stop", "free_motion", "load_without_stall",
                     "stall_without_load", "torque_spike", "stale_feedback", "collision",
                     "over_torque", "unknown_scene"):
        direction = -1 if scenario == "lower_stop" else 1
        config = ProbeConfig(direction, 0, .4, 1.5, .2, 5, .001, .005, .005, .002,
                             4, .5, .2, .04, .05, 50)
        detector = ContactDetector(config)
        for i in range(151):
            t = i * .02
            # A slowly increasing command meets a fabricated stop at 0.05 rad.
            travel = .04 * t
            q, v, tau = min(travel, .05), .04 if travel < .05 else 0, .1 if travel < .05 else .6
            if scenario in ("free_motion", "load_without_stall"):
                q, v, tau = travel, .04, .6 if scenario == "load_without_stall" else .1
            if scenario == "stall_without_load":
                tau = .1
            if scenario == "torque_spike":
                tau = .6 if i == 90 else .1
            if scenario == "over_torque" and i >= 65:
                tau = 1.6
            sample = ProbeSample(t, t - (.1 if scenario == "stale_feedback" and i >= 65 else 0),
                                 i, "synthetic", direction*q, direction*v, direction*tau,
                                 direction*travel, 25,
                                 None if scenario == "unknown_scene" and i >= 65 else not (scenario == "collision" and i >= 65))
            result = detector.observe(sample)
            if result.state != "probing":
                break
        expected = ("contact_candidate" if scenario in ("upper_stop", "lower_stop") else
                    "aborted" if scenario in ("stale_feedback", "collision", "over_torque", "unknown_scene") else "probing")
        cases[scenario] = {**asdict(result), "expected_state": expected, "passed": result.state == expected}
    return {"evidence": "synthetic_signal_replay", "physical_model_validated": False,
            "runtime_interlock_active": False,
            "hardware_commands_sent": 0, "config_values_are_hardware_defaults": False,
            "passed": all(c["passed"] for c in cases.values()), "cases": cases}
