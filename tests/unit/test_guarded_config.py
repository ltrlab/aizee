"""Receiver settings derived from swept joint limits, in motor coordinates."""
import pytest

from robot_platform.control.collision_envelope import calibration_revision
from robot_platform.control.guarded_config import cad_map, derive_receiver, motion_settings, to_cad

REFERENCE = {'motors': [{'name': 'arm_j1', 'raw_reference_rad': .9}, {'name': 'gripper', 'raw_reference_rad': 0.}]}
STEPS = [{'joint': 'shoulder', 'motor': 'arm_j1'}]
DIRECTIONS = {'shoulder': {'motor': 'arm_j1', 'sign': -1, 'offset_rad': .1, 'direction_verified': True}}
# 10 mm of slide per motor radian; the slide is 2 mm open at 0.2 rad, so cad = 0.01 * motor.
TRANSMISSIONS = [{'joint': 'slide', 'motor': 'gripper', 'metres_per_motor_rad': .01,
                  'joint_reference_m': .002, 'motor_reference_rad': .2}]
LIMITS = {'joints': {'arm_j1': {'min_rad': -2., 'max_rad': .5}, 'gripper': {'min_rad': -.5, 'max_rad': 2.}}}
DEFAULTS = {'margin_rad': .05, 'max_velocity': .5, 'max_effort': 10., 'max_kp': 20., 'max_kd': 2.,
            'stop_acceleration': 5., 'hold_kp': 10., 'hold_kd': 1., 'max_tracking_error': .05,
            'max_temperature': 70., 'align_speed': .3, 'shutdown_speed': .2, 'tolerance': .03,
            'settled_velocity': .05}


class Model:
    by_name = {'shoulder': {'control_bounds': (-1., 1.)}, 'slide': {'control_bounds': (0., .03)}}


class Backend:
    class runtime_model:
        revision, world_revision = 'model-rev', 'world-rev'
    policy_revision = 'policy-rev'


def profile(**gripper):
    return {'id': 'fixture-teleop',
            'receiver': {'lease_ms': 150, 'feedback_timeout_ms': 30, 'settled_velocity': .15,
                         'collision': {'scope': 'inter_branch', 'max_verdict_ms': 80, 'reaction_ms': 10}},
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


def test_operational_interval_is_swept_range_within_cad_less_margin():
    report = {row['motor']: row for row in derive()['report']}
    arm = report['arm_j1']
    # CAD -1..1 rad through cad = -motor + 0.1 is motor -0.9..1.1; swept is -2..0.5.
    assert arm['cad'] == pytest.approx([-.9, 1.1])
    assert arm['operational'] == pytest.approx([-.85, .45])
    assert arm['binding'] == ['cad', 'swept']
    grip = report['gripper']
    assert grip['cad'] == pytest.approx([0., 3.])
    assert grip['operational'] == pytest.approx([.05, 1.95])
    assert grip['binding'] == ['cad', 'swept']
    assert arm['direction_verified'] is True and grip['direction_verified'] is False
    assert arm['opens'] is None and grip['opens'] == 1


def test_a_mirrored_gripper_slide_still_opens_with_its_motor_angle():
    class Mirrored(Model):
        by_name = {**Model.by_name, 'slide': {'control_bounds': (-.03, 0.)}}
    mirrored = [{**TRANSMISSIONS[0], 'metres_per_motor_rad': -.01, 'joint_reference_m': -.002}]
    grip = derive(model=Mirrored, transmissions=mirrored)['report'][1]
    assert grip['scale'] == -.01 and grip['cad'] == pytest.approx([0., 3.]) and grip['opens'] == 1


def test_usable_targets_keep_the_receivers_stopping_reserve_inside():
    derived = derive()
    arm, grip = derived['report']
    # tracking + v * (verdict + feedback + reaction) + v^2 / 2a, then the compiler's padding.
    arm_reserve = .05 + .5 * .12 + .5 ** 2 / 10 + .5 * .08 + 1e-6
    grip_reserve = .05 + 1. * .12 + 1. / 10 + .08 + 1e-6
    assert arm['reserve'] == pytest.approx(arm_reserve)
    assert arm['usable'] == pytest.approx([-.85 + arm_reserve, .45 - arm_reserve])
    assert grip['usable'] == pytest.approx([.05 + grip_reserve, 1.95 - grip_reserve])
    # The reference pose is clamped into the usable range; the lifecycle returns there.
    assert derived['initial'] == pytest.approx([arm['usable'][1], grip['usable'][0]])
    assert [row['zero_position'] for row in derived['lifecycle']] == derived['initial']
    assert [row['max_lead'] for row in derived['lifecycle']] == [.05, .05]


def test_receiver_config_is_bound_to_model_calibration_and_policy():
    derived = derive()
    config = derived['config']
    assert config['group'] == 'fixture-teleop'
    assert config['identity'] == {'model': 'model-rev', 'calibration': calibration_revision(derived['joint_map']),
                                  'collision_policy': 'policy-rev'}
    assert config['collision'] == {'world_revision': 'world-rev', 'scope': 'inter_branch',
                                   'max_verdict_ms': 80, 'reaction_ms': 10}
    assert [joint['name'] for joint in config['joints']] == ['arm_j1', 'gripper']
    assert config['joints'][1]['max_velocity'] == 1.
    assert (config['joints'][0]['lower'], config['joints'][0]['upper']) == pytest.approx((-.85, .45))


def test_derivation_refuses_unsafe_or_incomplete_settings():
    with pytest.raises(ValueError, match='no saved joint limits'):
        derive(limits={'joints': {'arm_j1': LIMITS['joints']['arm_j1']}})
    with pytest.raises(ValueError, match='no usable range'):
        derive(profile=profile(max_velocity=6.))
    with pytest.raises(ValueError, match='hold gains'):
        derive(profile=profile(hold_kp=25.))
    with pytest.raises(ValueError, match='must be positive'):
        derive(profile=profile(max_tracking_error=0.))


def test_a_gripper_gets_its_whole_range_without_margin_reserve_or_lead_limit():
    derived = derive(profile=profile(gripper=True))
    arm, grip = derived['report']
    # CAD 0..3 rad in motor terms, swept -0.5..2: the whole overlap, no inset.
    assert grip['operational'] == pytest.approx([0., 2.]) and grip['usable'] == pytest.approx([0., 2.])
    assert grip['reserve'] == 0. and grip['home'] == pytest.approx(0.)
    assert arm['reserve'] > 0  # Arm joints keep their margin and reserve.
    joints = derived['config']['joints']
    assert joints[1]['gripper'] is True and 'gripper' not in joints[0]
    assert derived['lifecycle'][1]['max_lead'] == pytest.approx(2.)  # A grasp may push past contact.
