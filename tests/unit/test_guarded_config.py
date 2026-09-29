"""Receiver settings derived from swept joint limits, in motor coordinates."""
import math

import pytest

from robot_platform.control.collision_envelope import calibration_revision
from robot_platform.control.approach import OVERTRAVEL_RAD, friction_feedforward
from robot_platform.control.guarded_config import (CONTROL_LAW, GRIPPER_TOLERANCE_RAD, cad_map, derive_receiver,
                                                   motion_settings, to_cad)

REFERENCE = {'motors': [{'name': 'arm_j1', 'raw_reference_rad': .9}, {'name': 'gripper', 'raw_reference_rad': 0.}]}
STEPS = [{'joint': 'shoulder', 'motor': 'arm_j1'}]
DIRECTIONS = {'shoulder': {'motor': 'arm_j1', 'sign': -1, 'offset_rad': .1, 'direction_verified': True}}
# 10 mm of slide per motor radian; the slide is 2 mm open at 0.2 rad, so cad = 0.01 * motor.
TRANSMISSIONS = [{'joint': 'slide', 'motor': 'gripper', 'metres_per_motor_rad': .01,
                  'joint_reference_m': .002, 'motor_reference_rad': .2}]
LIMITS = {'joints': {'arm_j1': {'min_rad': -2., 'max_rad': .5}, 'gripper': {'min_rad': -.5, 'max_rad': 2.}}}
DEFAULTS = {'max_velocity': .5, 'max_effort': 10., 'max_kp': 20., 'max_kd': 2.,
            'stop_acceleration': 5., 'hold_kp': 10., 'hold_kd': 1., 'max_tracking_error': .05,
            'max_temperature': 70., 'align_speed': .3, 'shutdown_speed': .2, 'tolerance': .03,
            'settled_velocity': .05}


class Model:
    by_name = {'shoulder': {'control_bounds': (-1., 1.)}, 'slide': {'control_bounds': (0., .03)}}


class Backend:
    class runtime_model:
        revision, world_revision = 'model-rev', 'world-rev'
    policy_revision = 'policy-rev'


def profile(interpolation_ms=None, **gripper):
    return {'id': 'fixture-teleop',
            'receiver': {'lease_ms': 150, 'feedback_timeout_ms': 30, 'settled_velocity': .15,
                         'collision': {'scope': 'inter_branch', 'max_verdict_ms': 80, 'reaction_ms': 10},
                         **({} if interpolation_ms is None else {'interpolation_ms': interpolation_ms})},
            'motion': {'defaults': dict(DEFAULTS),
                       'joints': [{'match': 'grip*', 'max_velocity': 1., **gripper}]}}


def derive(**changes):
    args = dict(reference=REFERENCE, steps=STEPS, directions=DIRECTIONS, transmissions=TRANSMISSIONS,
                model=Model, limits=LIMITS, profile=profile(), backend=Backend)
    return derive_receiver(**{**args, **changes})


def test_motion_settings_apply_matching_patterns_in_order():
    motion = {'defaults': dict(DEFAULTS), 'joints': [{'match': '*_j1', 'max_kp': 30.},
                                                     {'match': 'left_*', 'max_kp': 40., 'hold_kp': 5.}]}
    assert motion_settings(motion, 'left_arm_j1')['max_kp'] == 40.
    assert motion_settings(motion, 'right_arm_j1')['max_kp'] == 30.
    assert motion_settings(motion, 'right_arm_j2')['max_kp'] == 20.
    with pytest.raises(ValueError, match='missing max_temperature'):
        motion_settings({'defaults': {k: v for k, v in DEFAULTS.items() if k != 'max_temperature'}}, 'x')
    with pytest.raises(ValueError, match='non-negative'):
        motion_settings({'defaults': {**DEFAULTS, 'max_velocity': -1.}}, 'x')


def test_cad_map_covers_every_motor_once():
    mapping = cad_map(REFERENCE, STEPS, DIRECTIONS, TRANSMISSIONS)
    assert mapping['shoulder'] == {'actuator': 'arm_j1', 'scale': -1., 'offset': .1}
    assert mapping['slide']['actuator'] == 'gripper'
    assert mapping['slide']['scale'] == .01 and mapping['slide']['offset'] == pytest.approx(0.)
    assert to_cad(mapping, {'arm_j1': .3, 'gripper': 2.}) == pytest.approx({'shoulder': -.2, 'slide': .02})
    with pytest.raises(ValueError, match='Every motor'):
        cad_map(REFERENCE, STEPS, DIRECTIONS, [])
    with pytest.raises(ValueError, match='Invalid direction'):
        cad_map(REFERENCE, STEPS, {'shoulder': {**DIRECTIONS['shoulder'], 'sign': 0}}, TRANSMISSIONS)


def test_an_arm_joints_operational_interval_is_its_swept_range_and_overtravel():
    report = {row['motor']: row for row in derive()['report']}
    arm = report['arm_j1']
    # CAD -1..1 rad through cad = -motor + 0.1 is motor -0.9..1.1; swept is -2..0.5. The swept range is the
    # joint's travel: the CAD's nominal limits are reported, and narrow nothing.
    assert arm['cad'] == pytest.approx([-.9, 1.1])
    assert arm['operational'] == pytest.approx([-2. - OVERTRAVEL_RAD, .5 + OVERTRAVEL_RAD])
    grip = report['gripper']
    assert grip['cad'] == pytest.approx([0., 3.])
    assert grip['operational'] == pytest.approx([-.5 - OVERTRAVEL_RAD, 2. + OVERTRAVEL_RAD])
    assert arm['direction_verified'] is True and grip['direction_verified'] is False
    assert arm['opens'] is None and grip['opens'] == 1


def test_a_mirrored_gripper_slide_still_opens_with_its_motor_angle():
    class Mirrored(Model):
        by_name = {**Model.by_name, 'slide': {'control_bounds': (-.03, 0.)}}
    mirrored = [{**TRANSMISSIONS[0], 'metres_per_motor_rad': -.01, 'joint_reference_m': -.002}]
    grip = derive(model=Mirrored, transmissions=mirrored)['report'][1]
    assert grip['scale'] == -.01 and grip['cad'] == pytest.approx([0., 3.]) and grip['opens'] == 1


def test_usable_targets_keep_the_stopping_margin_of_the_approach_floor_inside():
    derived = derive()
    arm, grip = derived['report']
    # v * (feedback timeout + reaction) + v^2 / 2a at the approach floor (0.5 rad/s): near an end a joint
    # slows, whatever its cap (1 rad/s for the second joint), and the tracking bound costs no travel.
    reserve = .5 * .04 + .5 ** 2 / 10 + 1e-6
    assert arm['reserve'] == grip['reserve'] == pytest.approx(reserve)
    assert arm['usable'] == pytest.approx([arm['operational'][0] + reserve, arm['operational'][1] - reserve])
    assert grip['usable'] == pytest.approx([grip['operational'][0] + reserve, grip['operational'][1] - reserve])
    # The reference pose is clamped into the usable range; the lifecycle returns there.
    assert derived['initial'] == pytest.approx([arm['usable'][1], 0.])
    assert [row['zero_position'] for row in derived['lifecycle']] == derived['initial']
    # Commands lead by at most 60% of the tracking bound, so a lagging joint is not at its fault threshold.
    assert [row['max_lead'] for row in derived['lifecycle']] == pytest.approx([.6 * .05, .6 * .05])


def test_receiver_config_is_bound_to_model_calibration_and_policy():
    derived = derive()
    config = derived['config']
    assert config['group'] == 'fixture-teleop'
    assert config['identity'] == {'model': 'model-rev', 'calibration': calibration_revision(derived['joint_map']),
                                  'collision_policy': 'policy-rev'}
    assert config['collision'] == {'world_revision': 'world-rev', 'scope': 'inter_branch',
                                   'max_verdict_ms': 80, 'reaction_ms': 10}
    assert [joint['name'] for joint in config['joints']] == ['arm_j1', 'gripper']
    # The receiver enforces 25% over the commanded cap (servos overshoot); commands keep the cap.
    assert config['joints'][1]['max_velocity'] == 1.25 and derived['lifecycle'][1]['max_velocity'] == 1.
    assert (config['joints'][0]['lower'], config['joints'][0]['upper']) == pytest.approx((-2.02, .52))
    # Commands are applied as sent unless the profile sets an interpolation period.
    assert config['interpolation_ms'] == 0 and derived['lifecycle'][0]['interpolation_s'] == 0.


def test_the_receiver_ramps_commands_over_the_profiles_interpolation_period():
    derived = derive(profile=profile(interpolation_ms=40))
    assert derived['config']['interpolation_ms'] == 40
    assert [row['interpolation_s'] for row in derived['lifecycle']] == [.04, .04]
    # Commands slow toward an end on the receiver's limit curve: it reacts in its feedback timeout and
    # reaction allowance (the collision verdict's lifetime bounds a box, not a limit).
    assert derived['lifecycle'][0]['approach']['reaction_s'] == pytest.approx(.04)
    # No friction feed-forward without a servo sweep that measured it.
    assert [row['friction_nm'] for row in derived['lifecycle']] == [0., 0.]


def test_derivation_refuses_unsafe_or_incomplete_settings():
    with pytest.raises(ValueError, match='no saved joint limits'):
        derive(limits={'joints': {'arm_j1': LIMITS['joints']['arm_j1']}})
    with pytest.raises(ValueError, match='no usable range'):
        # 0.08 rad of travel with its overtravel cannot keep 0.045 at each end.
        derive(limits={'joints': {**LIMITS['joints'], 'arm_j1': {'min_rad': 0., 'max_rad': .04}}})
    # A fast cap and a loose tracking bound cost no travel: near its ends a joint slows down, and the receiver
    # checks the limits on feedback.
    assert derive(profile=profile(max_velocity=6.))['report'][1]['usable'] == derive()['report'][1]['usable']
    assert derive(profile=profile(max_tracking_error=1.))['report'][1]['usable'] == derive()['report'][1]['usable']
    with pytest.raises(ValueError, match='hold gains'):
        derive(profile=profile(hold_kp=25.))
    with pytest.raises(ValueError, match='must be positive'):
        derive(profile=profile(max_tracking_error=0.))


def test_a_gripper_gets_its_whole_physical_range_without_margin_reserve_or_lead_limit():
    derived = derive(profile=profile(gripper=True))
    arm, grip = derived['report']
    # CAD is 0..3 rad in motor terms, but jaws physically close past the model's
    # closed limit: the hand-swept -0.5..2 stands, with no inset.
    assert grip['operational'] == pytest.approx([-.5, 2.]) and grip['usable'] == pytest.approx([-.5, 2.])
    assert grip['cad'] == pytest.approx([0., 3.])
    assert grip['reserve'] == 0. and grip['home'] == pytest.approx(0.)
    assert arm['reserve'] > 0  # Arm joints keep their stopping margin.
    joints = derived['config']['joints']
    assert joints[1]['gripper'] is True and 'gripper' not in joints[0]
    # A closed jaw rests on its swept end and reads a step past it: the receiver's
    # fault interval extends past each end, while targets stay within the swept travel.
    assert (joints[1]['lower'], joints[1]['upper']) == pytest.approx((-.5 - GRIPPER_TOLERANCE_RAD,
                                                                       2. + GRIPPER_TOLERANCE_RAD))
    assert grip['receiver'] == pytest.approx([joints[1]['lower'], joints[1]['upper']])
    lifecycle = derived['lifecycle'][1]
    assert (lifecycle['lower'], lifecycle['upper']) == (joints[1]['lower'], joints[1]['upper'])
    assert (joints[0]['lower'], joints[0]['upper']) == pytest.approx(arm['operational'])  # Arms: no tolerance.
    # A grasp may push past contact: the command may lead by the whole interval.
    assert lifecycle['max_lead'] == pytest.approx(2.5 + 2 * GRIPPER_TOLERANCE_RAD)
    # Its commands carry no velocity: fed forward, it would add to the squeeze while the jaws close on an object.
    ramped = derive(profile=profile(interpolation_ms=40, gripper=True))['lifecycle']
    assert ramped[0]['interpolation_s'] == .04 and ramped[1]['interpolation_s'] == 0.
    # Wherever it meets the model, the slide is clamped into its CAD range.
    assert derived['cad_limits'] == {'slide': (0., .03)}
    assert to_cad(derived['joint_map'], {'arm_j1': 0., 'gripper': -.5}, derived['cad_limits'])['slide'] == 0.


def test_a_servo_calibration_sets_its_joints_settings_and_is_bound_to_the_motor():
    reference = {'motors': [dict(REFERENCE['motors'][0], uid='aa', firmware='0.1'),
                            dict(REFERENCE['motors'][1], uid='bb', firmware='0.2')]}
    servo = {'joints': {'arm_j1': {'uid': 'aa', 'firmware': '0.1', 'hold_kp': 12., 'hold_kd': 1.5,
                                   'max_velocity': .3, 'stop_acceleration': 6., 'max_tracking_error': .04,
                                   'control': CONTROL_LAW, 'measured': {'torque': {'friction': .3}}}}}
    derived = derive(reference=reference, servo=servo)
    arm, grip = derived['config']['joints']
    assert (arm['hold_kp'], arm['hold_kd'], arm['max_velocity'], arm['stop_acceleration'],
            arm['max_tracking_error']) == (12., 1.5, .3 * 1.25, 6., .04)
    assert arm['max_kp'] >= 12. and derived['report'][0]['servo_calibrated'] is True
    assert derived['report'][1]['servo_calibrated'] is False and grip['hold_kp'] == derive()['config']['joints'][1]['hold_kp']
    # A harder stop shrinks the stopping margin, so more of the range is usable.
    assert derived['report'][0]['reserve'] < derive()['report'][0]['reserve']
    # Its measured friction is fed forward in the direction of motion.
    assert derived['lifecycle'][0]['friction_nm'] == .3 and derived['lifecycle'][1]['friction_nm'] == 0.
    assert derived['report'][0]['friction_ff'] == .3 and friction_feedforward(.3, 1.) == pytest.approx(.21)
    # A speed measured under the old position-only commands does not carry over; the rest does.
    old = {'joints': {'arm_j1': {k: v for k, v in servo['joints']['arm_j1'].items() if k != 'control'}}}
    arm = derive(reference=reference, servo=old)['config']['joints'][0]
    assert arm['max_velocity'] == DEFAULTS['max_velocity'] * 1.25 and arm['hold_kp'] == 12.
    assert arm['max_tracking_error'] == .04 and arm['stop_acceleration'] == 6.
    swapped = {'joints': {'arm_j1': dict(servo['joints']['arm_j1'], uid='cc')}}
    with pytest.raises(ValueError, match='another motor or firmware; sweep it again'):
        derive(reference=reference, servo=swapped)


def test_a_full_turn_joint_is_taken_a_turn_round_for_the_model_and_a_slide_clamped():
    mapping = {'swivel': {'actuator': 'w', 'scale': 1., 'offset': 0.}, 'slide': {'actuator': 'g', 'scale': .01, 'offset': 0.}}
    limits = {'swivel': (-math.pi, math.pi), 'slide': (0., .03)}
    cad = to_cad(mapping, {'w': -3.4, 'g': -.5}, limits)
    assert cad['swivel'] == pytest.approx(-3.4 + 2 * math.pi) and cad['slide'] == 0.
    assert to_cad(mapping, {'w': 1., 'g': 1.}, limits)['swivel'] == pytest.approx(1.)
