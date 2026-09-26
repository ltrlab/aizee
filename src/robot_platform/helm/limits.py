"""Hand-swept joint limits from stopped motor readings; motors are never enabled.

Every fresh complete scan updates every motor, so joints can be swept in any
order. An end of travel counts only when two consecutive readings agree on it:
the upper limit is the largest min(q[k-1], q[k]) and the lower limit the smallest
max(q[k-1], q[k]). A single glitch or a flick past a stop cannot widen a limit,
and recorded values never exceed the physical travel; an incomplete sweep is
only too narrow, which is why the operator confirms each joint.

Limits are raw motor radians from the recorded mechanical zero. They are bound to
that reference and to each motor's UID/firmware; a new zero invalidates them.
"""
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path

SCHEMA = 1
KIND = 'joint_limit_calibration'
UNITS = 'motor_rad_from_mechanical_zero'
METHOD = ('Operator hand sweep with motors disabled (stopped reads only). An end of '
          'travel counts after two consecutive complete scans agree on it.')
IDENTITY = ('bus', 'id', 'uid', 'firmware')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class LimitRecorder:
    def __init__(self, motors, *, jump_rad=1.5, min_span_rad=.1, reference_tolerance_rad=.05):
        names = [m['name'] for m in motors]
        if not names or len(set(names)) != len(names):
            raise ValueError('A unique motor roster is required')
        if not all(_finite(v) and v > 0 for v in (jump_rad, min_span_rad, reference_tolerance_rad)):
            raise ValueError('Positive finite sweep thresholds are required')
        self.motors = {m['name']: deepcopy(m) for m in motors}
        self.names = tuple(names)
        self.jump_rad, self.min_span_rad = jump_rad, min_span_rad
        self.reference_tolerance_rad = reference_tolerance_rad
        self.stream, self.sequence = None, None
        self._stats = {name: self._fresh() for name in names}
        self.confirmed = {}

    @staticmethod
    def _fresh(discarded=0):
        return dict(samples=0, previous=None, current=None, low=None, high=None,
                    seen_low=None, seen_high=None, jumps=0, discarded=discarded)

    def observe(self, positions, *, sequence, stream):
        """One complete scan of raw motor radians; readings never pair across streams."""
        if set(positions) != set(self.names):
            raise ValueError('A complete motor roster is required')
        if any(not _finite(positions[name]) for name in self.names):
            raise ValueError('Nonfinite motor position')
        if stream != self.stream:
            reconnect = self.stream is not None
            self.stream, self.sequence = stream, None
            for name, stats in self._stats.items():
                stats['previous'] = None
                if (reconnect and stats['current'] is not None
                        and abs(positions[name] - stats['current']) > self.jump_rad):
                    # The reading moved while the reader was away (for example a
                    # power cycle). Never mix readings from either side of it.
                    self._stats[name] = self._fresh(stats['discarded'] + 1)
                    self.confirmed.pop(name, None)
        if self.sequence is not None and sequence <= self.sequence:
            raise ValueError('Repeated or reversed scan sequence')
        self.sequence = sequence
        for name in self.names:
            q, stats = positions[name], self._stats[name]
            stats['samples'] += 1
            stats['current'] = q
            stats['seen_low'] = q if stats['seen_low'] is None else min(stats['seen_low'], q)
            stats['seen_high'] = q if stats['seen_high'] is None else max(stats['seen_high'], q)
            previous = stats['previous']
            if previous is not None:
                if abs(q - previous) > self.jump_rad:
                    stats['jumps'] += 1
                else:
                    held_high, held_low = min(q, previous), max(q, previous)
                    stats['high'] = held_high if stats['high'] is None else max(stats['high'], held_high)
                    stats['low'] = held_low if stats['low'] is None else min(stats['low'], held_low)
            stats['previous'] = q

    def _check(self, name):
        if name not in self._stats:
            raise ValueError(f'Unknown motor: {name}')
        return self._stats[name]

    def problem(self, name):
        """Why this joint cannot be confirmed yet, or None."""
        stats = self._check(name)
        low, high = stats['low'], stats['high']
        if low is None or high is None:
            return 'No held readings yet: move to each end and hold it for a moment'
        if high - low < self.min_span_rad:
            return f'Swept range is only {high - low:.3f} rad; reach both ends of travel'
        reference = self.motors[name].get('raw_reference_rad', 0.)
        tolerance = self.reference_tolerance_rad
        if not low - tolerance <= reference <= high + tolerance:
            return f'The mechanical reference ({reference:+.3f} rad) is outside the swept range'
        return None

    def confirm(self, name, *, at=None):
        problem = self.problem(name)
        if problem:
            raise ValueError(f'{name}: {problem}')
        stats = self._stats[name]
        self.confirmed[name] = dict(min_rad=stats['low'], max_rad=stats['high'],
            span_rad=stats['high'] - stats['low'], samples=stats['samples'],
            jumps=stats['jumps'], confirmed_at=at or utc_now())
        return deepcopy(self.confirmed[name])

    def unconfirm(self, name):
        self._check(name)
        self.confirmed.pop(name, None)

    def reset(self, name):
        self._check(name)
        self._stats[name] = self._fresh(self._stats[name]['discarded'])
        self.confirmed.pop(name, None)

    def rows(self):
        result = []
        for name in self.names:
            stats, confirmed = self._stats[name], self.confirmed.get(name)
            low, high = stats['low'], stats['high']
            changed = bool(confirmed and (low < confirmed['min_rad'] - .005 or high > confirmed['max_rad'] + .005))
            result.append(dict(name=name, current_rad=stats['current'], min_rad=low, max_rad=high,
                span_rad=None if low is None or high is None else high - low,
                samples=stats['samples'], jumps=stats['jumps'], discarded=stats['discarded'],
                problem=self.problem(name),
                confirmed=deepcopy(confirmed), changed_since_confirmed=changed))
        return result

    def document(self, *, asset_id, reference, reference_sha256, existing=None, session=None, at=None):
        """Merge newly confirmed joints over a previously valid saved calibration."""
        if not self.confirmed:
            raise ValueError('Confirm at least one joint before saving')
        joints = deepcopy(existing['joints']) if existing else {}
        for name, entry in self.confirmed.items():
            motor = self.motors[name]
            joints[name] = {**{key: motor[key] for key in IDENTITY}, **deepcopy(entry), 'session': session}
        return {'schema': SCHEMA, 'kind': KIND, 'asset_id': asset_id, 'reference': reference['id'],
                'reference_sha256': reference_sha256, 'model_revision': reference['model_revision'],
                'units': UNITS, 'method': METHOD, 'runtime_approved': False, 'updated_at': at or utc_now(),
                'joints': dict(sorted(joints.items()))}


def validate_limits(document, *, asset_id, reference, reference_sha256):
    """Accept saved limits only for this device, mechanical zero and motor roster."""
    if not isinstance(document, dict):
        raise ValueError('Limit calibration must be an object')
    expected = {'schema': SCHEMA, 'kind': KIND, 'asset_id': asset_id, 'reference': reference['id'],
                'reference_sha256': reference_sha256, 'units': UNITS, 'runtime_approved': False}
    for key, value in expected.items():
        if document.get(key) != value:
            raise ValueError(f'Saved limits have a different {key}; they belong to another device or zero')
    motors = {m['name']: m for m in reference['motors']}
    joints = document.get('joints')
    if not isinstance(joints, dict):
        raise ValueError('Saved limits need a joints object')
    for name, entry in joints.items():
        motor = motors.get(name)
        if motor is None or any(entry.get(key) != motor[key] for key in IDENTITY):
            raise ValueError(f'Saved limits for {name} belong to another motor identity')
        low, high = entry.get('min_rad'), entry.get('max_rad')
        if not _finite(low) or not _finite(high) or low >= high:
            raise ValueError(f'Saved limits for {name} are invalid')
    return document


def load_limits(path, **binding):
    """(document, None) when valid, (None, reason) when absent or not applicable."""
    path = Path(path)
    if not path.is_file():
        return None, None
    try:
        return validate_limits(json.loads(path.read_text(encoding='utf-8')), **binding), None
    except (ValueError, OSError) as exc:
        return None, str(exc)
