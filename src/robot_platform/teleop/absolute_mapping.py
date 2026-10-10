"""Calibrated absolute range-to-range mapping used by the existing GUI.

A route may instead be anchored with a straight scale: raw_zero maps to
follower_zero and the follower moves `gain` times the leader either way from
it. The leader may go past its calibrated range; the follower stops at its
ends. Engagement never recaptures leader
offsets. This module has no device or motor authority; deployment must
independently qualify its calibration and bounds.
"""
from copy import deepcopy
import math


class AbsoluteLeaderMapper:
    def __init__(self, joints, routes, *, max_age_s=.05, max_skew_s=.02,
                 max_leader_speed_rad_s=8.):
        self.names = tuple(j["name"] for j in joints)
        self.routes = deepcopy(routes)
        self.max_age_s, self.max_skew_s = max_age_s, max_skew_s
        self.max_speed = max_leader_speed_rad_s
        self.previous = None
        self.fault = None
        if (not self.names or len(set(self.names)) != len(self.names)
                or tuple(r["actuator"] for r in routes) != self.names
                or any(not math.isfinite(v) or v <= 0 for v in
                       (max_age_s, max_skew_s, max_leader_speed_rad_s))):
            raise ValueError("Complete ordered routes and positive timing bounds required")
        for limit, route in zip(joints, routes):
            if (not isinstance(route["leader"], str) or not route["leader"]
                    or type(route["channel"]) is not int or not 0 <= route["channel"] < 7
                    or route["direction"] not in (-1, 1)
                    or any(not math.isfinite(route[k]) for k in
                           ("raw_min", "raw_max", "follower_min", "follower_max"))
                    or route["raw_min"] >= route["raw_max"]
                    or not limit["lower"] <= route["follower_min"] < route["follower_max"] <= limit["upper"]):
                raise ValueError("Invalid calibrated leader/follower interval")
            if ("raw_zero" in route) != ("follower_zero" in route) or ("raw_zero" in route and (
                    not all(math.isfinite(route[k]) for k in ("raw_zero", "follower_zero"))
                    or not route["raw_min"] <= route["raw_zero"] <= route["raw_max"]
                    or not route["follower_min"] <= route["follower_zero"] <= route["follower_max"])):
                raise ValueError("Invalid leader/follower anchor")
            if "raw_zero" in route and (not math.isfinite(route.get("gain", math.nan)) or route["gain"] <= 0):
                raise ValueError("An anchored route needs a positive scale")

    def reset(self):
        """Explicit recovery before a deliberate new engagement; forgets history."""
        self.previous = None
        self.fault = None

    def map(self, samples, *, now, allow_repeated=False):
        if self.fault:
            raise ValueError(f"Leader mapping fault requires explicit recovery: {self.fault}")
        try:
            if not math.isfinite(now) or set(samples) != {r["leader"] for r in self.routes}:
                raise ValueError("Complete synchronized leader roster required")
            stamps = [s.sampled_at for s in samples.values()]
            if (any(not 0 <= now - stamp < self.max_age_s for stamp in stamps)
                    or max(stamps) - min(stamps) > self.max_skew_s):
                raise ValueError("Stale or unsynchronized leader target")
            repeated = False
            for name, sample in samples.items():
                if (type(sample.sequence) is not int or sample.sequence <= 0
                        or len(sample.positions_rad) != 7
                        or any(not math.isfinite(q) for q in sample.positions_rad)):
                    raise ValueError("Invalid leader channels or source sequence")
                if self.previous is not None:
                    previous = self.previous[name]
                    delta = (sample.sequence - previous.sequence) & 0xffffffff
                    dt = sample.sampled_at - previous.sampled_at
                    if allow_repeated and delta == 0:
                        if sample.positions_rad != previous.positions_rad or sample.sampled_at != previous.sampled_at:
                            raise ValueError('Leader source changed without a new sequence')
                        repeated = True
                        continue
                    if not 0 < delta < 0x80000000 or dt <= 0:
                        raise ValueError("Leader source did not advance")
                    if any(abs(a-b) > self.max_speed * dt for a, b in
                           zip(sample.positions_rad, previous.positions_rad)):
                        raise ValueError("Leader encoder jump or excessive speed")
            if repeated:
                return None  # A cached sample never becomes a fresh command or timestamp.
            target = []
            for route in self.routes:
                sample = samples[route["leader"]]
                target.append(dict(name=route["actuator"], position=follow(route, sample.positions_rad[route["channel"]]),
                                   sampled_at=sample.sampled_at, sequence=sample.sequence))
            self.previous = deepcopy(samples)
            return target
        except Exception as exc:
            self.fault = str(exc)
            raise


def follow(route, raw):
    """Follower position for one leader reading: range to range, or anchored with a straight scale."""
    if "raw_zero" in route:
        q = route["follower_zero"] + route["direction"] * route["gain"] * (raw - route["raw_zero"])
        return min(route["follower_max"], max(route["follower_min"], q))
    low, high = route["raw_min"], route["raw_max"]
    x = min(high, max(low, raw))
    near, far = route["follower_min"], route["follower_max"]
    if route["direction"] < 0:
        near, far = far, near
    return near + (x - low) / (high - low) * (far - near)
