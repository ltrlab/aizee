"""Gravity feed-forward for an arm's pitch chain, from each joint's measured gravity slice."""
import math

import pytest

from robot_platform.control.gravity import gravity_effort, gravity_model, gravity_terms, pitch_chains, world_axes


def joint(name, parent, child, rpy=(0., 0., 0.), axis=(0., 0., 1.)):
    return {'name': name, 'parent': parent, 'child': child, 'origin': {'xyz': [0., 0., 0.], 'rpy': list(rpy)},
            'axis': list(axis)}


class Arm:
    """A base yaw, three pitches (the third mounted flipped) and a wrist roll."""
    joints = [joint('yaw', 'base', 'l1'),
              joint('shoulder', 'l1', 'l2', rpy=(0., math.pi / 2, 0.)),  # z turned onto x: horizontal.
              joint('elbow', 'l2', 'l3'),
              joint('wrist', 'l3', 'l4', rpy=(math.pi, 0., 0.)),  # Its axis flipped.
              joint('roll', 'l4', 'l5', rpy=(0., -math.pi / 2, 0.))]


MOTORS = [{'name': f'arm_{n}', 'bus': 'can0'} for n in ('j1', 'j2', 'j3', 'j4', 'j5')]
DIRECTIONS = {cad: {'motor': f'arm_{m}', 'sign': sign} for cad, m, sign in (
    ('yaw', 'j1', 1), ('shoulder', 'j2', 1), ('elbow', 'j3', -1), ('wrist', 'j4', -1), ('roll', 'j5', 1))}
# Each joint's slice at the reference pose (the others at 0): the whole arm beyond it at 0.
FITS = {'arm_j2': {'A': 16.}, 'arm_j3': {'A': 6.}, 'arm_j4': {'A': 2.5}}


def test_the_pitch_chain_is_the_run_of_parallel_horizontal_axes_with_each_motors_sense():
    axes = world_axes(Arm)
    assert axes['yaw'] == pytest.approx([0., 0., 1.]) and axes['shoulder'] == pytest.approx([1., 0., 0.], abs=1e-12)
    assert axes['wrist'] == pytest.approx([-1., 0., 0.], abs=1e-12)
    # The elbow's motor is reversed against the CAD; the wrist's axis is flipped and its motor reversed.
    assert pitch_chains(Arm, DIRECTIONS, MOTORS) == {'can0': [('arm_j2', 1), ('arm_j3', -1), ('arm_j4', 1)]}


def test_each_joints_own_slice_is_what_its_sweep_measured():
    terms = gravity_model(Arm, DIRECTIONS, MOTORS, FITS)
    pose = {m['name']: 0. for m in MOTORS}
    for motor, fit in FITS.items():
        for q in (-1.2, -.4, .3, 1.5):
            assert gravity_effort(terms[motor], dict(pose, **{motor: q})) == pytest.approx(fit['A'] * math.sin(q))
    assert 'arm_j1' not in terms and 'arm_j5' not in terms


def test_the_links_beyond_a_joint_load_it_at_their_own_angles():
    terms = gravity_model(Arm, DIRECTIONS, MOTORS, FITS)
    pose = {m['name']: 0. for m in MOTORS}
    # Upper arm out 0.5 rad and the elbow's motor (reversed) turning it back 0.5: the forearm hangs straight
    # down again, so the elbow holds nothing and the shoulder only its upper arm (a2 = A2 - A3).
    bent = dict(pose, arm_j2=.5, arm_j3=.5)
    assert gravity_effort(terms['arm_j3'], bent) == pytest.approx(0., abs=1e-12)
    assert gravity_effort(terms['arm_j4'], bent) == pytest.approx(0., abs=1e-12)
    assert gravity_effort(terms['arm_j2'], bent) == pytest.approx((16. - 6.) * math.sin(.5))
    # Straight out: every joint holds the links beyond it level.
    level = dict(pose, arm_j2=math.pi / 2)
    assert [gravity_effort(terms[m], level) for m in ('arm_j2', 'arm_j3', 'arm_j4')] == pytest.approx([16., -6., 2.5])


def test_only_the_chains_fitted_prefix_is_fed_forward_the_last_carrying_the_rest():
    terms = gravity_terms([('arm_j2', 1), ('arm_j3', -1), ('arm_j4', 1)], {'arm_j2': {'A': 16.}, 'arm_j3': {'A': 6.}})
    assert set(terms) == {'arm_j2', 'arm_j3'}
    assert gravity_effort(terms['arm_j3'], {'arm_j2': 0., 'arm_j3': .7, 'arm_j4': 1.}) == pytest.approx(6. * math.sin(.7))
    assert gravity_terms([('arm_j2', 1), ('arm_j3', -1)], {'arm_j3': {'A': 6.}}) == {}  # No shoulder, no chain.


def test_minervas_arms_are_j2_to_j4_about_parallel_axes():
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.ops.release import ROOT
    model = UrdfModel(ModelProfile.load(ROOT / 'urdf/minerva/model.json'))
    # The motor-to-CAD directions verified in Helm (2026-09-24).
    signs = {'left': {'j1': 1, 'j2': 1, 'j3': 1, 'j4': 1, 'j5': 1, 'j6': 1},
             'right': {'j1': -1, 'j2': 1, 'j3': -1, 'j4': -1, 'j5': 1, 'j6': 1}}
    directions = {f'{arm}_arm_{j}': {'motor': f'{arm}_arm_{j}', 'sign': sign}
                  for arm, row in signs.items() for j, sign in row.items()}
    motors = [{'name': f'{arm}_arm_{j}', 'bus': 'can' + arm[0].upper()} for arm in signs for j in signs[arm]]
    chains = pitch_chains(model, directions, motors)
    for arm, bus in (('left', 'canL'), ('right', 'canR')):
        assert chains[bus] == [(f'{arm}_arm_j2', 1), (f'{arm}_arm_j3', -1), (f'{arm}_arm_j4', 1)]
