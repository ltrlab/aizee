"""Read-only display telemetry, preserving measurement identity, units and age.

No motor access, commanded-value fallback, effort conversion or freshness reset
occurs here. ``mapping_verified`` means the saved direction was verified; it is
not a claim that a transmission's CAD scale was physically calibrated.
"""
from copy import deepcopy
import math

STALE_AFTER_S = .75  # Display freshness only; never changes receiver watchdogs.


def _number(value):
    return float(value) if type(value) in (int, float) and math.isfinite(value) else None


def _age(value):
    value = _number(value)
    return value if value is not None and value >= 0 else None


def _mapped_range(scale, offset, lo, hi):
    """A motor-frame interval in joint coordinates, low end first; None when any part is missing."""
    if None in (scale, offset, lo, hi):
        return None
    return sorted([scale * lo + offset, scale * hi + offset])


def _unit(value):
    kind = value.get('type') if isinstance(value, dict) else value
    return 'm' if kind == 'prismatic' else 'rad' if kind in ('revolute', 'continuous') else None


def empty_packet(*, model=None, urdf_sha256=None, stream_id='unavailable', stale_after_s=STALE_AFTER_S):
    return dict(model=model, urdf_sha256=urdf_sha256, stream_id=stream_id, sequence=None,
                age_seconds=None, stale_after_s=stale_after_s, stale=True, rows=[])


def _packet(rows, *, model, urdf_sha256, stream_id, sequence, stale_after_s):
    ages = [r['age_seconds'] for r in rows if r['age_seconds'] is not None]
    return dict(model=model, urdf_sha256=urdf_sha256, stream_id=stream_id, sequence=sequence,
                age_seconds=max(ages) if ages else None, stale_after_s=stale_after_s,
                stale=not rows or any(r['stale'] for r in rows), rows=rows)


def native_packet(last, derived, *, model, urdf_sha256, joint_types, labels=None,
                  source='measured', cache_age_s=0., stream='receiver', stale_after_s=STALE_AFTER_S):
    """Transform an existing native snapshot; effort remains drive-reported torque.

    Native ``status.now_ms`` and ``feedback.sampled_at_ms`` share one clock.
    ``cache_age_s`` is local elapsed time since that snapshot was received. No
    local/remote absolute clocks are subtracted. Missing data remains None.
    Each row also carries the receiver's interval and reference pose, mapped like
    positions, and its effort and temperature limits, for gauges on the model.
    """
    if source not in ('measured', 'simulated'):
        raise ValueError('Telemetry source must be measured or simulated')
    last, derived, labels = last or {}, derived or {}, labels or {}
    status = last.get('status') or {}
    now, cache_age = _number(status.get('now_ms')), _age(cache_age_s)
    names = status.get('joint_names') or [row['name'] for row in (derived.get('config') or {}).get('joints', [])]
    feedback = dict(zip(names, last.get('feedback') or []))
    verified = {r['motor']: r.get('direction_verified') is True for r in derived.get('report', [])}
    homes = {r['motor']: _number(r.get('home')) for r in derived.get('report', [])}
    limits = {j['name']: j for j in (derived.get('config') or {}).get('joints', [])}
    rows = []
    for joint, mapping in (derived.get('joint_map') or {}).items():
        motor = mapping['actuator']
        sample = feedback.get(motor) or {}
        unit = _unit(joint_types.get(joint))
        scale, offset = _number(mapping.get('scale')), _number(mapping.get('offset'))
        raw_position, raw_velocity = _number(sample.get('position')), _number(sample.get('velocity'))
        position = scale * raw_position + offset if None not in (unit, scale, offset, raw_position) else None
        velocity = scale * raw_velocity if None not in (unit, scale, raw_velocity) else None
        sampled = _number(sample.get('sampled_at_ms'))
        age = ((now - sampled) / 1000 + cache_age
               if now is not None and sampled is not None and now >= sampled and cache_age is not None else None)
        effort = _number(sample.get('effort'))
        config = limits.get(motor) or {}
        joint_range = _mapped_range(scale, offset, _number(config.get('lower')), _number(config.get('upper')))
        home = homes.get(motor)
        rows.append(dict(motor=motor, joint=joint, label=labels.get(motor, motor),
            position=_number(position), position_unit=unit, velocity=_number(velocity),
            velocity_unit=unit + '/s' if unit else None, effort=effort,
            effort_unit='Nm' if effort is not None else None,
            effort_label='Drive torque' if source == 'measured' else 'Simulated drive torque',
            temperature_c=_number(sample.get('temperature')), age_seconds=age,
            stale=age is None or age > stale_after_s or position is None or sample.get('fault') is True,
            source=source, mapping_verified=verified.get(motor, False),
            range=joint_range if unit else None,
            reference=scale * home + offset if None not in (unit, scale, offset, home) else None,
            effort_limit=_number(config.get('max_effort')), temperature_limit_c=_number(config.get('max_temperature'))))
    return _packet(rows, model=model, urdf_sha256=urdf_sha256,
                   stream_id=f"{stream}:{status.get('epoch') or 'unavailable'}", sequence=status.get('now_ms'),
                   stale_after_s=stale_after_s)


def stopped_packet(frame, mapping, transmissions, *, model, urdf_sha256, joint_types, labels=None,
                   cache_age_s=0., reading=False, stale_after_s=STALE_AFTER_S, ranges=None, references=None):
    """Only stopped-reader positions are available; never invent velocity or load.

    ``ranges`` (motor -> swept low and high, motor radians) and ``references``
    (motor -> reference, motor radians) are mapped like positions for gauges.
    """
    frame, labels, ranges, references = frame or {}, labels or {}, ranges or {}, references or {}
    raw = {r['name']: _number(r.get('position_rad')) for r in frame.get('motors', [])}
    observed, cache = _age(frame.get('roundtrip_seconds')), _age(cache_age_s)
    age = observed + cache if observed is not None and cache is not None else None
    mappings = {joint: dict(actuator=row['motor'], scale=row['sign'], offset=row['offset_rad'],
                           verified=row.get('direction_verified') is True) for joint, row in mapping.items()}
    for entry in transmissions:
        scale = entry['metres_per_motor_rad']
        mappings[entry['joint']] = dict(actuator=entry['motor'], scale=scale,
            offset=entry['joint_reference_m'] - scale * entry['motor_reference_rad'],
            verified=entry.get('direction_verified') is True)
    rows = []
    for joint, row in mappings.items():
        motor, unit = row['actuator'], _unit(joint_types.get(joint))
        value = raw.get(motor)
        position = _number(row['scale'] * value + row['offset']) if value is not None and unit else None
        lo, hi = (ranges.get(motor) or (None, None))
        reference = _number(references.get(motor))
        rows.append(dict(motor=motor, joint=joint, label=labels.get(motor, motor), position=position,
            position_unit=unit, velocity=None, velocity_unit=unit + '/s' if unit else None,
            effort=None, effort_unit=None, effort_label='Drive torque', temperature_c=None,
            age_seconds=age, stale=not reading or age is None or age > stale_after_s or position is None,
            source='measured', mapping_verified=row['verified'],
            range=_mapped_range(row['scale'], row['offset'], _number(lo), _number(hi)) if unit else None,
            reference=row['scale'] * reference + row['offset'] if unit and reference is not None else None,
            effort_limit=None, temperature_limit_c=None))
    return _packet(rows, model=model, urdf_sha256=urdf_sha256,
                   stream_id='stopped:' + str(frame.get('request_id') or 'unavailable'),
                   sequence=frame.get('sequence'), stale_after_s=stale_after_s)


def age_packet(packet, elapsed_s):
    """Age a cached packet without mutating it or refreshing its source identity."""
    result = deepcopy(packet) if packet else empty_packet()
    elapsed, threshold = _age(elapsed_s), result['stale_after_s']
    for row in [result, *result['rows']]:
        age = _age(row.get('age_seconds'))
        row['age_seconds'] = age + elapsed if age is not None and elapsed is not None else None
        row['stale'] = bool(row.get('stale') or row['age_seconds'] is None or row['age_seconds'] > threshold)
    return result


def select_telemetry(state):
    """An active owner's missing data clears the overlay instead of mixing arms."""
    baseline = state.get('telemetry') or empty_packet()
    for owner in ('teleop', 'tune'):
        view = state.get(owner) or {}
        if view.get('active') is True or view.get('phase') in ('starting', 'running', 'fault'):
            packet = view.get('telemetry')
            if packet is not None:
                return deepcopy(packet)
            return empty_packet(model=baseline.get('model'), urdf_sha256=baseline.get('urdf_sha256'),
                                stream_id=owner + ':unavailable')
    return deepcopy(baseline)
