"""Source sample age tracking, independent of wall-clock synchronization.

Age is a lower bound: publisher age plus local elapsed time cannot account for
unknown transit delay. Legacy packets explicitly retain unknown source age.
"""
from collections.abc import Mapping
import math


class SourceFreshness:
    def __init__(self):
        self.producer = None
        self.retired = set()
        self.samples = {}
        self.seen_versioned = False

    def ingest(self, message, names, received_at, *, required=False):
        versioned = message.get("telemetry_schema_version") == 2
        has_metadata = "telemetry_schema_version" in message or "producer_id" in message
        if not versioned:
            invalid = required or self.seen_versioned or has_metadata
            return tuple(-math.inf if invalid else received_at for _ in names), False
        self.seen_versioned = True
        producer = message.get("producer_id")
        if not isinstance(producer, str) or not producer or producer in self.retired:
            return tuple(-math.inf for _ in names), True
        if producer != self.producer:
            if self.producer is not None:
                self.retired.add(self.producer)
            # Bounded memory; an unreasonable number of restarts fails closed.
            if len(self.retired) > 64:
                return tuple(-math.inf for _ in names), True
            self.producer = producer
            self.samples.clear()
        motors = message.get("motors", {})
        if not isinstance(motors, Mapping):
            motors = {}
        origins = []
        for name in names:
            motor = motors.get(name, {})
            meta = motor.get("feedback", {}) if isinstance(motor, Mapping) else {}
            origin = -math.inf
            if isinstance(meta, Mapping):
                seq, age = meta.get("sequence"), meta.get("age_s")
                valid = (type(seq) is int and seq >= 1 and
                         type(age) in (int, float) and math.isfinite(age) and age >= 0 and
                         meta.get("valid") is True and meta.get("provenance") in ("can", "simulation"))
                if valid:
                    previous = self.samples.get(name)
                    if previous is None or seq >= previous[0]:
                        origin = received_at - age
                        if previous is not None and seq == previous[0]:
                            origin = min(origin, previous[1])
                        self.samples[name] = (seq, origin)
            origins.append(origin)
        return tuple(origins), True
