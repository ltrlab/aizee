"""Guarded receiver settings derived from a robot's recorded calibration.

The receiver works in motor coordinates, the space the joint limits were swept
in. Each joint's operational interval is the swept range intersected with the
CAD viewing range (mapped through the motor-to-CAD direction or transmission),
inset by a margin. Commands must also keep the receiver's stopping reserve
inside that interval, so the usable target range is narrower still; both are
reported. Grippers (motion setting "gripper": true) touch and hold objects:
their interval is their hand-swept physical travel (jaws close a little past the
CAD slide's closed limit, so the model's range would stop them short), with no
margin and no reserve, since the receiver keeps their last target on a stop
instead of braking, and their command may lead the measured position by their
whole range (a grasp pushes past contact). Their CAD values are clamped into the
model's range wherever they meet the model (`cad_limits`). A closed gripper rests
on its hand-swept end, so the receiver's fault interval for a gripper extends
GRIPPER_TOLERANCE_RAD past each end; its targets (`usable`) stay within the swept travel.
Motion values come from a teleop profile and are not qualified by this derivation.
"""
from fnmatch import fnmatchcase
import math

from .collision_envelope import calibration_revision

RECEIVER_KEYS = ('max_velocity', 'max_effort', 'max_kp', 'max_kd', 'stop_acceleration',
                 'hold_kp', 'hold_kd', 'max_tracking_error', 'max_temperature')
LIFECYCLE_KEYS = ('align_speed', 'shutdown_speed', 'tolerance', 'settled_velocity')
# A jaw resting on its stop reads at or just past its hand-swept end (encoder noise
# and 16-bit feedback steps of ~0.0004 rad), and a fast close flexes it a little
# further. The receiver faults on any reading outside its interval, even disarmed,
# so a gripper's interval extends this far past each swept end (about 0.5 mm of jaw
# travel). The stops keep the jaws from going further; targets stay swept.
GRIPPER_TOLERANCE_RAD = .05


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def motion_settings(motion, motor):
    """Defaults, then every matching pattern entry in order."""
    merged = dict(motion.get('defaults', {}))
    for entry in motion.get('joints', []):
        if fnmatchcase(motor, entry['match']):
            merged.update({k: v for k, v in entry.items() if k != 'match'})
    missing = [k for k in (*RECEIVER_KEYS, *LIFECYCLE_KEYS, 'margin_rad') if k not in merged]
    if missing:
        raise ValueError(f'{motor}: motion settings missing {", ".join(missing)}')
    if any(not _finite(merged[k]) or merged[k] < 0 for k in (*RECEIVER_KEYS, *LIFECYCLE_KEYS, 'margin_rad')):
        raise ValueError(f'{motor}: motion settings must be finite and non-negative')
    return merged


def cad_map(reference, steps, directions, transmissions):
    """Per CAD joint: the motor driving it and cad = scale * motor + offset."""
    motors = {m['name'] for m in reference['motors']}
    result = {}
    for step in steps:
        row = directions[step['joint']]
        if row['motor'] != step['motor'] or row['sign'] not in (-1, 1) or not _finite(row['offset_rad']):
            raise ValueError(f"Invalid direction mapping for {step['joint']}")
        result[step['joint']] = dict(actuator=step['motor'], scale=float(row['sign']), offset=float(row['offset_rad']))
    for entry in transmissions:
        k = entry['metres_per_motor_rad']
        result[entry['joint']] = dict(actuator=entry['motor'], scale=float(k),
                                      offset=float(entry['joint_reference_m'] - k * entry['motor_reference_rad']))
    if {row['actuator'] for row in result.values()} != motors or len(result) != len(motors):
        raise ValueError('Every motor needs exactly one mapped CAD joint')
    return result


def arm_subset(reference, steps, transmissions, arm):
    """(reference, steps, transmissions) for one arm's motors, which must share one CAN bus."""
    motors = [motor for motor in reference['motors'] if motor['name'].startswith(arm + '_')]
    if not motors or len({motor['bus'] for motor in motors}) != 1:
        raise ValueError(f'The {arm} arm must be the motors of one CAN bus')
    names = {motor['name'] for motor in motors}
    return (dict(reference, motors=motors), [step for step in steps if step['motor'] in names],
            [entry for entry in transmissions if entry['motor'] in names])


def derive_receiver(*, reference, steps, directions, transmissions, model, limits, profile, backend):
    """Receiver config, CAD map, lifecycle joints, initial pose and a per-joint report."""
    joint_map = cad_map(reference, steps, directions, transmissions)
    by_motor = {row['actuator']: (joint, row) for joint, row in joint_map.items()}
    receiver = profile['receiver']
    policy = receiver['collision']
    window_s = (policy['max_verdict_ms'] + receiver['feedback_timeout_ms'] + policy['reaction_ms']) / 1000
    verified = {row['motor']: row.get('direction_verified') is True for row in directions.values()}
    verified.update({t['motor']: t.get('direction_verified') is True for t in transmissions})
    grippers = {t['motor'] for t in transmissions}
    joints, lifecycle, initial, report = [], [], [], []
    cad_limits = {}
    for motor in reference['motors']:
        name = motor['name']
        saved = limits['joints'].get(name)
        if saved is None:
            raise ValueError(f'{name}: no saved joint limits; sweep it in Helm first')
        settings = motion_settings(profile['motion'], name)
        gripper = settings.get('gripper') is True
        cad_joint, row = by_motor[name]
        cad_lo, cad_hi = model.by_name[cad_joint]['control_bounds']
        cad_motor = sorted(((cad_lo - row['offset']) / row['scale'], (cad_hi - row['offset']) / row['scale']))
        if gripper:
            lower, upper = saved['min_rad'], saved['max_rad']
            cad_limits[cad_joint] = tuple(model.by_name[cad_joint].get('bounds') or (cad_lo, cad_hi))
        else:
            lower = max(saved['min_rad'], cad_motor[0]) + settings['margin_rad']
            upper = min(saved['max_rad'], cad_motor[1]) - settings['margin_rad']
        speed = settings['max_velocity']
        if gripper:
            reserve = padding = 0.
        else:
            reserve = (settings['max_tracking_error'] + speed * window_s
                       + speed ** 2 / (2 * settings['stop_acceleration']))
            # Matches CollisionEnvelopeCompiler's box (it adds 1e-9); stay a hair
            # inside so a pose exactly on the usable edge still certifies.
            padding = speed * policy['max_verdict_ms'] / 1000 + 1e-6
        usable = (lower + reserve + padding, upper - reserve - padding)
        if not lower < upper or not usable[0] < usable[1]:
            raise ValueError(f'{name}: no usable range after margins and the stopping reserve; '
                             'reduce its speed or stopping distance')
        if settings['hold_kp'] > settings['max_kp'] or settings['hold_kd'] > settings['max_kd']:
            raise ValueError(f'{name}: hold gains exceed their maxima')
        if settings['max_tracking_error'] <= 0 or settings['max_velocity'] <= 0 or settings['stop_acceleration'] <= 0:
            raise ValueError(f'{name}: speed, tracking and stopping bounds must be positive')
        home = min(max(motor.get('raw_reference_rad', 0.), usable[0]), usable[1])
        tolerance = GRIPPER_TOLERANCE_RAD if gripper else 0.
        bounds = {'lower': lower - tolerance, 'upper': upper + tolerance}  # Receiver and lifecycle share these.
        joints.append({'name': name, 'unit': 'radians', **bounds,
                       **{key: float(settings[key]) for key in RECEIVER_KEYS}, **({'gripper': True} if gripper else {})})
        lifecycle.append({'name': name, **bounds,
                          **{key: float(settings[key]) for key in ('hold_kp', 'hold_kd', 'max_velocity')},
                          **{key: float(settings[key]) for key in LIFECYCLE_KEYS},
                          'max_lead': bounds['upper'] - bounds['lower'] if gripper
                          else float(settings['max_tracking_error']),
                          'zero_position': home})
        initial.append(home)
        report.append({'motor': name, 'cad_joint': cad_joint, 'scale': row['scale'], 'offset': row['offset'],
                       'swept': [saved['min_rad'], saved['max_rad']], 'cad': cad_motor,
                       'operational': [lower, upper], 'usable': list(usable), 'reserve': reserve + padding,
                       'receiver': [bounds['lower'], bounds['upper']],
                       'binding': ['swept', 'swept'] if gripper else [
                           'cad' if cad_motor[0] > saved['min_rad'] else 'swept',
                           'cad' if cad_motor[1] < saved['max_rad'] else 'swept'],
                       'home': home, 'direction_verified': verified.get(name, False),
                       # Gripper slides: the motor direction that opens (away from the bound nearest zero).
                       # The CAD axis may be mirrored between arms; physical opening is what an operator sees.
                       'opens': ((1 if row['scale'] > 0 else -1) * (1 if abs(cad_hi) >= abs(cad_lo) else -1)
                                 if name in grippers else None)})
    config = {'group': profile['id'],
              'identity': {'model': backend.runtime_model.revision, 'calibration': calibration_revision(joint_map),
                           'collision_policy': backend.policy_revision},
              'lease_ms': receiver['lease_ms'], 'feedback_timeout_ms': receiver['feedback_timeout_ms'],
              'settled_velocity': receiver['settled_velocity'], 'joints': joints,
              'collision': {'world_revision': backend.runtime_model.world_revision, 'scope': policy['scope'],
                            'max_verdict_ms': policy['max_verdict_ms'], 'reaction_ms': policy['reaction_ms']}}
    return {'config': config, 'joint_map': joint_map, 'lifecycle': lifecycle,
            'initial': initial, 'report': report, 'cad_limits': cad_limits}


def to_cad(joint_map, motors, limits=None):
    """CAD joint positions for a {motor: position} mapping (display and collision).

    `limits` ({cad joint: (low, high)}) clamps gripper slides, whose physical
    travel passes the model's closed limit, into the model's range.
    """
    cad = {joint: row['scale'] * motors[row['actuator']] + row['offset']
           for joint, row in joint_map.items() if row['actuator'] in motors}
    for joint, (low, high) in (limits or {}).items():
        if joint in cad:
            cad[joint] = min(high, max(low, cad[joint]))
    return cad
