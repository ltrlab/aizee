"""Explicit relative registration and slew-limited whole-robot leader targets."""
from copy import deepcopy
import math


class LeaderTargetMapper:
    def __init__(self, joint_limits, routes, *, max_age_s=.05, max_skew_s=.02,
                 max_step_s=.08, max_leader_speed_rad_s=8.):
        self.limits, self.routes = deepcopy(joint_limits), deepcopy(routes)
        self.names = tuple(j["name"] for j in self.limits)
        if (len(set(self.names)) != len(self.names) or len(routes) != len(self.names)
                or tuple(route["actuator"] for route in routes) != self.names):
            raise ValueError("Leader routes must cover the complete receiver roster in order")
        for route in routes:
            if (not route["leader"] or type(route["channel"]) is not int or route["channel"] < 0
                    or not math.isfinite(route["scale"]) or route["scale"] == 0):
                raise ValueError("Invalid leader route or transmission scale")
        if any(not math.isfinite(j["max_velocity"]) or j["max_velocity"] <= 0
               or not math.isfinite(j["lower"]) or not math.isfinite(j["upper"])
               or j["lower"] >= j["upper"] for j in self.limits):
            raise ValueError("Finite operational intervals and positive speed bounds required")
        values = (max_age_s, max_skew_s, max_step_s, max_leader_speed_rad_s)
        if any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError("Positive finite input timing/speed bounds required")
        self.max_age_s, self.max_skew_s, self.max_step_s, self.max_leader_speed = values
        self.registered = False
        self.fault = None

    def _read(self, samples, now):
        if not math.isfinite(now) or set(samples) != {route["leader"] for route in self.routes}:
            raise ValueError("Complete leader pair and finite clock required")
        stamps = [sample.sampled_at for sample in samples.values()]
        if any(not 0 <= now - stamp < self.max_age_s for stamp in stamps):
            raise ValueError("Stale or future leader sample")
        if max(stamps) - min(stamps) > self.max_skew_s:
            raise ValueError("Leader samples are not synchronized")
        positions = []
        for route in self.routes:
            sample = samples[route["leader"]]
            if sample.sequence <= 0 or route["channel"] >= len(sample.positions_rad):
                raise ValueError("Missing leader channel/source sequence")
            value = sample.positions_rad[route["channel"]]
            if not math.isfinite(value):
                raise ValueError("Nonfinite leader position")
            positions.append(value)
        return positions

    def register(self, samples, measured_positions, *, now):
        """Explicit new registration after a stop; capture offsets without motion."""
        self.stop("registration_incomplete")
        if len(measured_positions) != len(self.names) or any(
                not math.isfinite(q) or not j["lower"] <= q <= j["upper"]
                for q, j in zip(measured_positions, self.limits)):
            raise ValueError("Complete measured robot pose inside operational limits required")
        self.origin_leader = self._read(samples, now)
        self.origin_robot = list(measured_positions)
        self.target = list(measured_positions)
        self.previous = dict(samples)
        self.last_time = now
        self.fault = None
        self.registered = True
        return list(self.target)

    def stop(self, reason="operator_stop"):
        self.registered = False
        self.fault = reason

    def validate_fresh(self, samples, *, now):
        """Recheck source age after collision computation, without advancing targets."""
        self._read(samples, now)

    def propose(self, samples, *, now, engaged):
        if not self.registered:
            raise ValueError("Explicit leader registration required")
        try:
            if engaged is not True:
                raise ValueError("Operator tracking disengaged")
            dt = now - self.last_time
            if not 0 < dt <= self.max_step_s:
                raise ValueError("Teleoperation scheduler stalled or clock regressed")
            positions = self._read(samples, now)
            for key, sample in samples.items():
                previous = self.previous[key]
                delta = (sample.sequence - previous.sequence) & 0xffffffff
                elapsed = sample.sampled_at - previous.sampled_at
                if not 0 < delta < 0x80000000 or elapsed <= 0:
                    raise ValueError("Leader source did not advance")
                if len(sample.positions_rad) != len(previous.positions_rad) or any(
                        not math.isfinite(a) or not math.isfinite(b)
                        or abs(a - b) > self.max_leader_speed * elapsed
                        for a, b in zip(sample.positions_rad, previous.positions_rad)):
                    raise ValueError("Leader encoder jump or excessive speed")
            target = []
            for q, origin, base, previous, route, limit in zip(
                    positions, self.origin_leader, self.origin_robot, self.target, self.routes, self.limits):
                desired = base + route["scale"] * (q - origin)
                desired = max(limit["lower"], min(limit["upper"], desired))
                step = limit["max_velocity"] * dt
                target.append(max(previous - step, min(previous + step, desired)))
            self.previous, self.last_time, self.target = dict(samples), now, target
            return list(target)
        except (ValueError, KeyError, TypeError) as exc:
            self.stop(str(exc))
            raise
