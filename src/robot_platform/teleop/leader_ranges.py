"""Hand-swept leader ranges: each channel's full travel, for range-to-range teleop.

The operator sweeps every leader joint and trigger through its travel. Readings
are unwrapped continuously, because an encoder may cross its 0/2pi seam
mid-sweep. An end counts once three consecutive readings reach it (the upper
end is the largest minimum over any three in a row, the lower end the smallest
maximum), so sweeping through an end records it without pausing, but one or two
glitch readings cannot widen a range. A single reading that jumps is ignored; a
jump that persists, or any jump across a pause in the readings, discards that
channel's unsaved sweep rather than mixing readings from either side of it.

A saved range is center-anchored: at runtime a reading r becomes
center + wrap(r - center). That is unambiguous for any range under a full turn
and independent of the servo's turn count after a power cycle (the two-endpoint
form could not tell a wide sweep from a wrapped one). The file also keeps the
leader zero: every channel's reading, captured at once, in the pose that should
map onto the robot's reference pose. The operator can set it again at any time.
Ranges belong to the leader profile and its pinned USB serials.
"""
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path

SCHEMA = 1
KIND = 'leader_range_calibration'
UNITS = 'leader_rad_center_and_span'
# Stay clear of a full turn so a reading at either end still unwraps to the right side.
MAX_SPAN_RAD = 2 * math.pi - .35


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def channels(profile):
    """(key, leader, channel name, index, label) for every leader channel, in profile order."""
    return [(f"{leader}.{joint['name']}", leader, joint['name'], index, joint.get('label', joint['name']))
            for leader, item in profile['leaders'].items() for index, joint in enumerate(item['joints'])]


class LeaderRangeRecorder:
    def __init__(self, profile, *, saved=None, agree=3, jump_rad=1., gap_s=.25, persist=3, min_span_rad=.1):
        if (not all(_finite(v) and v > 0 for v in (jump_rad, gap_s, min_span_rad))
                or type(agree) is not int or agree < 2 or persist < 1):
            raise ValueError('Positive finite sweep thresholds are required')
        self.channels = channels(profile)
        self.keys = tuple(key for key, *_ in self.channels)
        self.agree, self.jump_rad = agree, jump_rad
        self.gap_s, self.persist, self.min_span_rad = gap_s, persist, min_span_rad
        self.saved = dict(saved or {})  # {key: (center_rad, span_rad)} for display
        self._stats = {key: self._fresh() for key in self.keys}
        self._sequences = None
        self._restart = True
        self.confirmed = {}

    @staticmethod
    def _fresh(discarded=0):
        return dict(samples=0, raw=None, value=None, at=None, seen_low=None, seen_high=None, low=None, high=None,
                    window=[], jumps=0, pending=0, discarded=discarded)

    def restart(self):
        """A new reading stream: each channel must continue from its last reading."""
        self._restart, self._sequences = True, None

    def observe(self, samples, *, now):
        """One synchronized pair {leader: LeaderSample}; a pair with no new data is ignored."""
        leaders = {leader for _, leader, *_ in self.channels}
        if set(samples) != leaders:
            raise ValueError('A complete leader roster is required')
        sequences = {leader: samples[leader].sequence for leader in leaders}
        if sequences == self._sequences:
            return
        self._sequences = sequences
        restart, self._restart = self._restart, False
        for key, leader, _name, index, _label in self.channels:
            q = samples[leader].positions_rad[index]
            if not _finite(q):
                raise ValueError(f'Nonfinite reading on {key}')
            stats = self._stats[key]
            if restart:
                stats['window'] = []  # Readings never agree across streams.
            if stats['raw'] is None:
                stats.update(raw=q, value=q, at=now)
            else:
                delta = wrap(q - stats['raw'])
                if abs(delta) > self.jump_rad:
                    paused = restart or now - stats['at'] > self.gap_s
                    stats['pending'] += 1
                    if not paused and stats['pending'] < self.persist:
                        stats['jumps'] += 1  # One bad reading: ignore it.
                        continue
                    # The channel really is elsewhere: continuity is lost.
                    stats = self._stats[key] = self._fresh(stats['discarded'] + 1)
                    self.confirmed.pop(key, None)
                    stats.update(raw=q, value=q, at=now)
                else:
                    stats.update(raw=q, value=stats['value'] + delta, at=now, pending=0)
            value = stats['value']
            stats['samples'] += 1
            stats['seen_low'] = value if stats['seen_low'] is None else min(stats['seen_low'], value)
            stats['seen_high'] = value if stats['seen_high'] is None else max(stats['seen_high'], value)
            window = stats['window']
            window.append(value)
            del window[:-self.agree]
            if len(window) == self.agree:
                reached_high, reached_low = min(window), max(window)
                stats['high'] = reached_high if stats['high'] is None else max(stats['high'], reached_high)
                stats['low'] = reached_low if stats['low'] is None else min(stats['low'], reached_low)

    def readings(self):
        """{key: latest raw reading} for every channel, or None before every channel has one."""
        values = {key: self._stats[key]['raw'] for key in self.keys}
        return None if any(value is None for value in values.values()) else values

    def _check(self, key):
        if key not in self._stats:
            raise ValueError(f'Unknown leader channel: {key}')
        return self._stats[key]

    def problem(self, key):
        """Why this channel cannot be confirmed yet, or None."""
        stats = self._check(key)
        low, high = stats['low'], stats['high']
        if low is None or high - low < self.min_span_rad:
            return 'Move it all the way to both ends of its travel'
        if high - low > MAX_SPAN_RAD:
            return 'The range is nearly a full turn; it cannot be unwrapped reliably'
        return None

    def confirm(self, key, *, at=None):
        problem = self.problem(key)
        if problem:
            raise ValueError(f'{key}: {problem}')
        stats = self._stats[key]
        low, high = stats['low'], stats['high']
        self.confirmed[key] = dict(center_rad=((low + high) / 2) % (2 * math.pi), span_rad=high - low,
                                   samples=stats['samples'], jumps=stats['jumps'], confirmed_at=at or utc_now())
        return deepcopy(self.confirmed[key])

    def unconfirm(self, key):
        self._check(key)
        self.confirmed.pop(key, None)

    def reset(self, key):
        stats = self._check(key)
        self._stats[key] = self._fresh(stats['discarded'])
        self.confirmed.pop(key, None)

    def rows(self):
        """Per channel, in the current sweep's continuous angle; saved ranges are shown in it too."""
        result = []
        for key, leader, name, index, label in self.channels:
            stats, confirmed = self._stats[key], self.confirmed.get(key)
            value, low, high = stats['value'], stats['low'], stats['high']
            saved = None
            if key in self.saved:
                center, span = self.saved[key]
                if value is not None:
                    center = value + wrap(center - value)  # The saved range's branch nearest this reading.
                saved = dict(low=center - span / 2, high=center + span / 2, span_rad=span)
            reference = (low, high) if low is not None and high - low > 0 else (
                (saved['low'], saved['high']) if saved else None)
            fraction = None
            if value is not None and reference:
                fraction = (value - reference[0]) / (reference[1] - reference[0])
            changed = bool(confirmed and low is not None and high - low > confirmed['span_rad'] + .01)
            result.append(dict(key=key, leader=leader, name=name, channel=index + 1, label=label,
                               current=value, seen_low=stats['seen_low'], seen_high=stats['seen_high'],
                               low=low, high=high, span_rad=None if low is None else high - low,
                               fraction=fraction, samples=stats['samples'], jumps=stats['jumps'],
                               discarded=stats['discarded'], problem=self.problem(key),
                               confirmed=deepcopy(confirmed), changed_since_confirmed=changed, saved=saved))
        return result

    def document(self, profile, *, existing=None, session=None, at=None):
        """Merge newly confirmed channels over a previously valid calibration, keeping its zero."""
        if not self.confirmed:
            raise ValueError('Confirm at least one leader channel before saving')
        leaders = deepcopy(existing['leaders']) if existing else {}
        for key, leader, name, index, label in self.channels:
            if key in self.confirmed:
                leaders.setdefault(leader, {})[name] = {'channel': index + 1, 'label': label,
                                                        **deepcopy(self.confirmed[key]), 'session': session}
        document = {'schema': SCHEMA, 'kind': KIND, 'profile': profile['id'], 'pins': pins(profile), 'units': UNITS,
                    'method': (f'Operator hand sweep of each leader channel through its full travel. Readings are '
                               f'unwrapped continuously; an end counts once {self.agree} consecutive readings reach '
                               f'it. At runtime a reading r is center + wrap(r - center).'),
                    'runtime_approved': False, 'updated_at': at or utc_now(),
                    'leaders': {leader: dict(sorted(rows.items())) for leader, rows in sorted(leaders.items())}}
        if existing and 'reference' in existing:
            document['reference'] = deepcopy(existing['reference'])
        return document


def pins(profile):
    return {name: item['usb_serial'] for name, item in profile['leaders'].items()}


def validate_ranges(document, profile):
    """Accept saved ranges only for this leader profile and these USB serials."""
    if not isinstance(document, dict):
        raise ValueError('Leader ranges must be an object')
    expected = {'schema': SCHEMA, 'kind': KIND, 'profile': profile['id'], 'units': UNITS, 'runtime_approved': False}
    for key, value in expected.items():
        if document.get(key) != value:
            raise ValueError(f'Saved leader ranges have a different {key}; they belong to another leader setup')
    if document.get('pins') != pins(profile):
        raise ValueError('Saved leader ranges belong to other leader boards (their USB serials differ)')
    leaders = document.get('leaders')
    if not isinstance(leaders, dict):
        raise ValueError('Saved leader ranges need a leaders object')
    if 'reference' in document:
        captured = (document['reference'] or {}).get('leaders')
        if (not isinstance(captured, dict) or set(captured) != set(profile['leaders'])
                or any(set(captured[name]) != {joint['name'] for joint in item['joints']}
                       or not all(_finite(v) for v in captured[name].values())
                       for name, item in profile['leaders'].items())):
            raise ValueError('The saved leader zero must cover every leader channel')
    for leader, rows in leaders.items():
        names = {joint['name'] for joint in profile['leaders'].get(leader, {}).get('joints', [])}
        if not isinstance(rows, dict) or not names:
            raise ValueError(f'Saved leader ranges name an unknown leader: {leader}')
        for name, entry in rows.items():
            center, span = entry.get('center_rad'), entry.get('span_rad')
            if name not in names or not _finite(center) or not _finite(span) or not 0 < span <= MAX_SPAN_RAD:
                raise ValueError(f'Saved leader range for {leader} {name} is invalid')
    return document


def load_ranges(path, profile):
    """(document, None) when valid, (None, None) when absent, (None, reason) otherwise."""
    path = Path(path)
    if not path.is_file():
        return None, None
    try:
        return validate_ranges(json.loads(path.read_text(encoding='utf-8')), profile), None
    except (ValueError, OSError) as exc:
        return None, str(exc)


def with_reference(document, profile, captured, *, session=None, at=None):
    """A copy of a saved calibration (or a new, empty one) with this leader zero.

    captured: {leader: {channel name: reading_rad}} for every channel.
    """
    at = at or utc_now()
    result = deepcopy(document) if document else {
        'schema': SCHEMA, 'kind': KIND, 'profile': profile['id'], 'pins': pins(profile), 'units': UNITS,
        'runtime_approved': False, 'leaders': {}}
    result.update(updated_at=at, reference={'captured_at': at, 'session': session, 'leaders': {
        leader: {name: value % (2 * math.pi) for name, value in rows.items()} for leader, rows in captured.items()}})
    return validate_ranges(result, profile)


def reference_table(document, profile):
    """{leader: [zero reading per channel]} from the leader zero, or None when not set."""
    captured = ((document or {}).get('reference') or {}).get('leaders')
    if not captured:
        return None
    return {leader: [captured[leader][joint['name']] for joint in item['joints']]
            for leader, item in profile['leaders'].items()}


def range_table(document, profile):
    """{leader: [(center_rad, span_rad) or None per channel]} in profile channel order."""
    saved = (document or {}).get('leaders', {})
    return {leader: [(saved[leader][joint['name']]['center_rad'], saved[leader][joint['name']]['span_rad'])
                     if joint['name'] in saved.get(leader, {}) else None for joint in item['joints']]
            for leader, item in profile['leaders'].items()}
