"""Guarded receiver settings derived from a robot's recorded calibration.

The receiver works in motor coordinates, the space the joint limits were swept
in. An arm joint's operational interval is its hand-swept range, extended
OVERTRAVEL_RAD past each end (control.approach): the swept range is the joint's
travel, and the CAD's nominal limits (reported, mapped through the motor-to-CAD
direction or transmission) do not narrow it. Near an end a joint slows on the
receiver's braking curve, so its targets stay only the stopping margin of its
approach floor inside the interval (`usable`, reported). The receiver ramps each
arm command over the profile's interpolation period with its velocity fed
forward, and a joint with a saved servo sweep also gets its friction fed forward.
Grippers (motion setting "gripper": true) touch and hold objects:
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
from .gravity import gravity_model
from robot_platform.control.approach import OVERSPEED_HEADROOM, OVERTRAVEL_RAD, end_margin, limit_reaction_s

RECEIVER_KEYS = ('max_velocity', 'max_effort', 'max_kp', 'max_kd', 'stop_acceleration',
                 'hold_kp', 'hold_kd', 'max_tracking_error', 'max_temperature')
LIFECYCLE_KEYS = ('align_speed', 'shutdown_speed', 'tolerance', 'settled_velocity')
# A jaw resting on its stop reads at or just past its hand-swept end (encoder noise
# and 16-bit feedback steps of ~0.0004 rad), and a fast close flexes it a little
# further. The receiver faults on any reading outside its interval, even disarmed,
# so a gripper's interval extends this far past each swept end (about 0.5 mm of jaw
# travel). The stops keep the jaws from going further; targets stay swept.
GRIPPER_TOLERANCE_RAD = .05
FULL_TURN = 2 * math.pi
# What a servo calibration (the live servo sweep, saved in Helm) sets per joint.
SERVO_KEYS = ('hold_kp', 'hold_kd', 'max_velocity', 'stop_acceleration', 'max_tracking_error')
# Under a motion collision bound, a joint's tracking bound at rest, as a share of its whole bound
# (collision::tracking_bound).
REST_TRACKING_SHARE = .5
# The control law a servo calibration was measured under: the receiver ramps each command over its
# interpolation period with its velocity (and friction) fed forward. A speed measured under the old
# position-only commands, where damping braked every motion, does not carry over: a calibration made
# under another law keeps its gains, braking and tracking bound, not its speed.
CONTROL_LAW = 'interpolated-feedforward'
# How far an arm joint's command may lead its measured position, as a share of its
# tracking bound: a joint that cannot keep up lags and catches up, short of its fault
# threshold (right J4 lagged 0.088 rad against 0.08 in the first live session, when
# commands were position-only and the lag grew with speed as hold_kd * speed / hold_kp).
LEAD_FRACTION = .6


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def motion_settings(motion, motor):
    """Defaults, then every matching pattern entry in order."""
    merged = dict(motion.get('defaults', {}))
    for entry in motion.get('joints', []):
        if fnmatchcase(motor, entry['match']):
            merged.update({k: v for k, v in entry.items() if k != 'match'})
    missing = [k for k in (*RECEIVER_KEYS, *LIFECYCLE_KEYS) if k not in merged]
    if missing:
        raise ValueError(f'{motor}: motion settings missing {", ".join(missing)}')
    if any(not _finite(merged[k]) or merged[k] < 0 for k in (*RECEIVER_KEYS, *LIFECYCLE_KEYS)):
        raise ValueError(f'{motor}: motion settings must be finite and non-negative')
    return merged


def usable_margin(settings, receiver):
    """How far inside its operational range a non-gripper joint's targets stay at each end, for motion
    `settings` (the commanded cap, tracking bound and stop acceleration) under the teleop profile's
    `receiver` block: the stopping margin of its approach floor (control.approach), whatever its cap."""
    joint = {'max_velocity': settings['max_velocity'] * OVERSPEED_HEADROOM,
             'max_tracking_error': settings['max_tracking_error'], 'stop_acceleration': settings['stop_acceleration']}
    return end_margin(joint, limit_reaction_s(receiver))


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


def fit_motor_ranges(joints, models):
    """Keep each receiver joint's command limits (a derived config's joints, changed in place) within what its
    motor's wire protocol carries (`models`: {motor: RobStride model}, commissioning.wire_ranges); the receiver
    refuses a config beyond them. A damping ceiling of 5.6 on a motor carrying 5.0 failed to encode mid-sweep, a
    drive fault that cut torque (right J4, 2026-09-28 and -30). Hold gains beyond them are a calibration error."""
    from .commissioning import wire_ranges
    for joint in joints:
        if joint['name'] not in models:
            continue
        ranges = wire_ranges(models[joint['name']])
        for key, wire in (('max_kp', 'kp'), ('max_kd', 'kd'), ('max_effort', 'effort'), ('max_velocity', 'velocity')):
            joint[key] = min(joint[key], ranges[wire])
        if joint['hold_kp'] > joint['max_kp'] or joint['hold_kd'] > joint['max_kd']:
            raise ValueError(f"{joint['name']}: its hold gains exceed what its motor ({models[joint['name']]}) "
                             'carries')
    return joints


# The most a profile may add to every joint's calibrated speed (extra_speed_rad_s).
MAX_EXTRA_SPEED_RAD_S = 5.


def extra_speed(profile):
    """What the manual-tuning profile adds to every joint's speed over its calibration, in rad/s (0 by default).

    An operator's choice for direct teleop (2026-10-12: +2 rad/s), never a measured speed: the servo
    calibration, its sweeps and Minerva Check keep their own. Only the manual-tuning profile may set it."""
    extra = profile['motion'].get('extra_speed_rad_s', 0.)
    if not _finite(extra) or not 0 <= extra <= MAX_EXTRA_SPEED_RAD_S:
        raise ValueError(f'extra_speed_rad_s must be 0 to {MAX_EXTRA_SPEED_RAD_S:g} rad/s')
    if extra and profile.get('manual_tuning') is not True:
        raise ValueError('extra_speed_rad_s is for the manual-tuning profile only')
    return float(extra)


def servo_settings(settings, motor, servo):
    """A joint's motion settings with its saved servo calibration applied, if it has one.

    The calibration is bound to the motor's identity: a replaced motor must be swept again.
    """
    entry = ((servo or {}).get('joints') or {}).get(motor['name'])
    if entry is None:
        return settings, False
    if entry.get('uid') != motor.get('uid') or entry.get('firmware') != motor.get('firmware'):
        raise ValueError(f"{motor['name']}: the servo calibration was made with another motor or firmware; "
                         'sweep it again')
    keys = [key for key in SERVO_KEYS if key != 'max_velocity' or entry.get('control') == CONTROL_LAW]
    merged = dict(settings, **{key: float(entry[key]) for key in keys if entry.get(key) is not None})
    merged['max_kp'] = max(merged['max_kp'], merged['hold_kp'])
    merged['max_kd'] = max(merged['max_kd'], merged['hold_kd'])
    return merged, True


def derive_receiver(*, reference, steps, directions, transmissions, model, limits, profile, backend, servo=None,
                    gravity=None, collision_bound=None):
    """Receiver config, CAD map, lifecycle joints, initial pose and a per-joint report.

    `servo` is a saved servo calibration: its joints take their gains, speed,
    braking and tracking bound from it instead of the profile. Each arm's pitch chain
    gets gravity feed-forward from `gravity` (gravity slices, {motor: {'A': ...}};
    default: the servo calibration's), see control.gravity. `collision_bound` 'motion'
    has the receiver check collision boxes against the motion commanded (two arms;
    collision_envelope.MotionEnvelopeCompiler); by default it keeps its reserve bound.
    """
    if collision_bound not in (None, 'reserve', 'motion'):
        raise ValueError('The collision bound is reserve or motion')
    joint_map = cad_map(reference, steps, directions, transmissions)
    by_motor = {row['actuator']: (joint, row) for joint, row in joint_map.items()}
    receiver = profile['receiver']
    disabled = receiver.get('collision_mode') == 'disabled'
    position_only = disabled and receiver.get('direct', {}).get('position_only', False)
    feedforward = disabled and not position_only and int(receiver.get('interpolation_ms', 0)) == 0
    if disabled and (backend is not None or collision_bound is not None or receiver.get('collision')):
        raise ValueError('Disabled collision mode must not carry a collision backend or policy')
    if not disabled and backend is None:
        raise ValueError('Collision checking requires its backend')
    policy = None if disabled else receiver['collision']
    interpolation_ms = int(receiver.get('interpolation_ms', 0))
    verified = {row['motor']: row.get('direction_verified') is True for row in directions.values()}
    verified.update({t['motor']: t.get('direction_verified') is True for t in transmissions})
    grippers = {t['motor'] for t in transmissions}
    fits = gravity if gravity is not None else (servo or {}).get('gravity')
    terms = gravity_model(model, directions, reference['motors'], fits) if fits else {}
    joints, lifecycle, initial, report = [], [], [], []
    cad_limits = {}
    extra = extra_speed(profile)
    for motor in reference['motors']:
        name = motor['name']
        saved = limits['joints'].get(name)
        if saved is None:
            raise ValueError(f'{name}: no saved joint limits; sweep it in Helm first')
        settings, calibrated = servo_settings(motion_settings(profile['motion'], name), motor, servo)
        # The commanded cap, the receiver's cap over it and its braking curve all follow from this speed.
        settings['max_velocity'] += extra
        gripper = settings.get('gripper') is True
        # The friction its servo sweep measured, for friction feed-forward (arm joints only: a gripper's
        # squeeze is its stiffness times its lead).
        measured = ((((servo or {}).get('joints') or {}).get(name) or {}).get('measured') or {}).get('torque') or {}
        friction = 0. if gripper or not calibrated else min(float(measured.get('friction') or 0.),
                                                              float(settings['max_effort']))
        # And its inertia, for acceleration feed-forward (control.approach.ramp_effort).
        inertia = 0. if gripper or not calibrated else float(measured.get('inertia') or 0.)
        cad_joint, row = by_motor[name]
        cad_lo, cad_hi = model.by_name[cad_joint]['control_bounds']
        cad_motor = sorted(((cad_lo - row['offset']) / row['scale'], (cad_hi - row['offset']) / row['scale']))
        if gripper:
            lower, upper = saved['min_rad'], saved['max_rad']
            cad_limits[cad_joint] = tuple(model.by_name[cad_joint].get('bounds') or (cad_lo, cad_hi))
        else:
            # A joint the model lets turn a full circle (the wrist swivels were swept 6.65 rad) is taken a turn
            # round past its model limits (to_cad): the same pose, which the collision model accepts.
            bounds = model.by_name[cad_joint].get('bounds')
            if bounds and bounds[1] - bounds[0] >= FULL_TURN - 1e-9:
                cad_limits[cad_joint] = tuple(bounds)
            # The hand-swept range is the joint's travel: the receiver lets it pass a swept end by
            # OVERTRAVEL_RAD before it faults.
            lower, upper = saved['min_rad'] - OVERTRAVEL_RAD, saved['max_rad'] + OVERTRAVEL_RAD
        # The receiver enforces its cap (and near the ends its braking curve) with headroom over what is
        # commanded: a servo overshoots the speed it follows. Near an end a joint slows (control.approach),
        # so its targets stay only the stopping margin of its approach floor inside, at any speed cap.
        speed = settings['max_velocity'] * OVERSPEED_HEADROOM
        margin = 0. if gripper else usable_margin(settings, receiver)
        usable = (lower + margin, upper - margin)
        if not lower < upper or not usable[0] < usable[1]:
            raise ValueError(f'{name}: no usable range inside the stopping margin; sweep its range again')
        if settings['hold_kp'] > settings['max_kp'] or settings['hold_kd'] > settings['max_kd']:
            raise ValueError(f'{name}: hold gains exceed their maxima')
        if settings['max_tracking_error'] <= 0 or settings['max_velocity'] <= 0 or settings['stop_acceleration'] <= 0:
            raise ValueError(f'{name}: speed, tracking and stopping bounds must be positive')
        home = min(max(motor.get('raw_reference_rad', 0.), usable[0]), usable[1])
        tolerance = GRIPPER_TOLERANCE_RAD if gripper else 0.
        bounds = {'lower': lower - tolerance, 'upper': upper + tolerance}  # Receiver and lifecycle share these.
        joints.append({'name': name, 'unit': 'radians', **bounds,
                       **{key: float(settings[key]) for key in RECEIVER_KEYS}, 'max_velocity': float(speed),
                       **({'gripper': True} if gripper else {})})
        if collision_bound == 'motion' and not gripper:
            # At rest a joint holds within half its tracking bound: its servo sweep sizes the bound at twice the
            # lag at full speed, and the stiffness to hold within half of it. Two arms keep tighter boxes for it.
            joints[-1]['rest_tracking_error'] = REST_TRACKING_SHARE * float(settings['max_tracking_error'])
        lifecycle.append({'name': name, **bounds,
                          **{key: float(settings[key]) for key in ('hold_kp', 'hold_kd', 'max_velocity')},
                          **{key: float(settings[key]) for key in LIFECYCLE_KEYS},
                          **({'gripper': True} if gripper else {}),
                          'max_lead': bounds['upper'] - bounds['lower'] if gripper
                          else LEAD_FRACTION * float(settings['max_tracking_error']),
                          'zero_position': home,
                          # The receiver ramps each command over this period, with its velocity fed forward.
                          # Not a gripper's: fed-forward velocity would add to its squeeze while it closes on
                          # an object (kd x speed), and its command already leads by its whole range.
                          'interpolation_s': 0. if gripper else interpolation_ms / 1000, 'friction_nm': friction,
                          'direct_velocity': disabled and not gripper,
                          'position_only': position_only,
                          # Direct and unramped, not position-only: a smoothed velocity is fed forward, so Kd damps
                          # the joint about its command's speed instead of dragging it (teleop.lifecycle FEED_SAMPLES).
                          'velocity_feedforward': feedforward and not gripper,
                          'inertia': inertia,
                          # Gravity feed-forward: [(coefficient, {motor: multiplier})], control.gravity.
                          'gravity': [[coefficient, angle] for coefficient, angle in terms.get(name, [])],
                          'max_effort': float(settings['max_effort']),
                          # Commands slow toward an end within the receiver's braking curve.
                          **({} if gripper else {'approach': {
                              **bounds, 'max_velocity': float(speed),
                              'max_tracking_error': float(settings['max_tracking_error']),
                              'stop_acceleration': float(settings['stop_acceleration']),
                              'reaction_s': limit_reaction_s(receiver)}})})
        initial.append(home)
        report.append({'motor': name, 'cad_joint': cad_joint, 'scale': row['scale'], 'offset': row['offset'],
                       'swept': [saved['min_rad'], saved['max_rad']], 'cad': cad_motor,
                       'operational': [lower, upper], 'usable': list(usable), 'reserve': margin,
                       'receiver_cap': float(speed), 'friction_ff': 0. if position_only or feedforward else friction,
                       'gravity_ff': bool(terms.get(name)),
                       'receiver': [bounds['lower'], bounds['upper']],
                       'home': home, 'direction_verified': verified.get(name, False), 'servo_calibrated': calibrated,
                       'control_law': 'direct-position-only' if position_only else
                                      'direct-velocity-feedforward' if feedforward else
                                      'direct-feedforward' if disabled else CONTROL_LAW,
                       'speed_qualified_for_control_law': bool(calibrated and not disabled and
                           (((servo or {}).get('joints') or {}).get(name) or {}).get('control') == CONTROL_LAW),
                       # Gripper slides: the motor direction that opens (away from the bound nearest zero).
                       # The CAD axis may be mirrored between arms; physical opening is what an operator sees.
                       'opens': ((1 if row['scale'] > 0 else -1) * (1 if abs(cad_hi) >= abs(cad_lo) else -1)
                                 if name in grippers else None)})
    config = {'group': profile['id'],
              'identity': {'model': model.sha256 if disabled else backend.runtime_model.revision,
                           'calibration': calibration_revision(joint_map),
                           'collision_policy': 'disabled-v1' if disabled else backend.policy_revision},
              'lease_ms': receiver['lease_ms'], 'feedback_timeout_ms': receiver['feedback_timeout_ms'],
              'settled_velocity': receiver['settled_velocity'], 'joints': joints, 'interpolation_ms': interpolation_ms}
    if disabled:
        config['direct'] = dict(receiver['direct'])
    else:
        config['collision'] = {'world_revision': backend.runtime_model.world_revision, 'scope': policy['scope'],
                               'max_verdict_ms': policy['max_verdict_ms'], 'reaction_ms': policy['reaction_ms']}
    if collision_bound == 'motion':
        config['collision']['bound'] = 'motion'
    return {'config': config, 'joint_map': joint_map, 'lifecycle': lifecycle,
            'initial': initial, 'report': report, 'cad_limits': cad_limits}


def to_cad(joint_map, motors, limits=None):
    """CAD joint positions for a {motor: position} mapping (display and collision).

    `limits` ({cad joint: (low, high)}) brings each position into the model's range:
    a joint whose range is a full turn is taken a turn round (the same pose); a
    gripper slide, whose physical travel passes the model's closed limit, is clamped.
    """
    cad = {joint: row['scale'] * motors[row['actuator']] + row['offset']
           for joint, row in joint_map.items() if row['actuator'] in motors}
    for joint, (low, high) in (limits or {}).items():
        if joint in cad:
            if high - low >= FULL_TURN - 1e-9:
                cad[joint] = low + (cad[joint] - low) % FULL_TURN
            cad[joint] = min(high, max(low, cad[joint]))
    return cad
