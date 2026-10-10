"""Manual following masks, fixed travel origins and bounded position commands."""
from copy import deepcopy

import pytest

from robot_platform.teleop.lifecycle import TeleopLifecycle


NAMES = ('left_arm_j1', 'left_arm_j2')


def joints():
    return [dict(name=name, lower=-2., upper=2., hold_kp=10., hold_kd=1., max_velocity=.5,
                 align_speed=.3, shutdown_speed=.1, max_lead=.2, tolerance=.001,
                 settled_velocity=.01, zero_position=0., position_only=True,
                 direct_velocity=True, max_effort=5.) for name in NAMES]


def settings():
    return [dict(motor=name, kp=12., kd=1.5, speed=.1, follow=i == 0,
                 travel_rad=.03 if i == 0 else None) for i, name in enumerate(NAMES)]


def observations(now, positions, *, source=False):
    rows = [dict(name=name, position=q, velocity=0., sampled_at=now, sequence=round(now*10000)+1)
            for name, q in zip(NAMES, positions)]
    return [{k: v for k, v in row.items() if k != 'velocity'} for row in rows] if source else rows


def held(*, configured=True, origin=(.1, -.2)):
    lifecycle = TeleopLifecycle(joints(), allow_repeated_source=True, settle_time_s=0.)
    if configured: lifecycle.configure_manual(settings())
    lifecycle.action('idle', observations(0., origin), now=0.)
    lifecycle.action('enable', observations(0., origin), now=0.)
    return lifecycle


def engage(lifecycle, now):
    pose = lifecycle.target
    lifecycle.action('engage', observations(now, pose), target=observations(now, pose, source=True), now=now)
    for index in range(1, 7):
        at = now + index*.01
        lifecycle.step(observations(at, pose), target=observations(at, pose, source=True), now=at)
    assert lifecycle.state == 'TELEOP'
    return at


def follow(lifecycle, now, goal, count=50):
    trace = []
    for _ in range(count):
        now += .01
        before = lifecycle.target
        result = lifecycle.step(observations(now, before), target=observations(now, goal, source=True), now=now)
        trace.append((before, result))
    return now, trace


def test_follow_false_holds_its_target_while_another_joint_moves():
    lifecycle = held()
    now = engage(lifecycle, .01)
    _, trace = follow(lifecycle, now, (1., 1.))
    assert trace[-1][1]['positions'][0] == pytest.approx(.13)
    assert all(result['positions'][1] == -.2 for _, result in trace)
    assert any(result['positions'][0] > before[0] for before, result in trace)
    assert all(command['velocity'] == 0. for _, result in trace for command in result['joints'])
    assert all(result['joints'][1]['kp'] == 12. and result['joints'][1]['kd'] == 1.5 for _, result in trace)


def test_follow_toggle_captures_current_target_without_retargeting_or_moving_origin():
    lifecycle = held()
    now = engage(lifecycle, .01)
    now, _ = follow(lifecycle, now, (1., 1.), count=20)
    lifecycle.action('disengage', observations(now+.01, lifecycle.target), now=now+.01)
    before = lifecycle.target
    changed = settings()
    changed[0].update(follow=False, kp=14.)
    changed[1].update(follow=True, travel_rad=.03)
    lifecycle.configure_manual(changed)
    assert lifecycle.target == before
    now = engage(lifecycle, now+.02)
    _, trace = follow(lifecycle, now, (-1., 1.))
    assert all(result['positions'][0] == before[0] for _, result in trace)
    assert trace[-1][1]['positions'][1] == pytest.approx(-.17)


def test_travel_origin_survives_hold_reengage_and_gain_edits():
    lifecycle = held()
    now = engage(lifecycle, .01)
    now, _ = follow(lifecycle, now, (1., -.2))
    assert lifecycle.target[0] == pytest.approx(.13)
    lifecycle.action('disengage', observations(now+.01, lifecycle.target), now=now+.01)
    edited = settings()
    edited[0].update(kp=15., kd=2.)
    lifecycle.configure_manual(edited)
    now = engage(lifecycle, now+.02)
    now, forward = follow(lifecycle, now, (1., -.2))
    assert max(result['positions'][0] for _, result in forward) == pytest.approx(.13)
    _, reverse = follow(lifecycle, now, (-1., -.2), count=80)
    assert reverse[-1][1]['positions'][0] == pytest.approx(.07)


def test_follow_speed_caps_each_step_and_can_increase_again_within_original_cap():
    lifecycle = held()
    edited = settings()
    edited[0].update(speed=.02, travel_rad=None)
    lifecycle.configure_manual(edited)
    assert all(lifecycle.joints[0][key] == .02 for key in ('max_velocity', 'align_speed'))
    assert lifecycle.joints[0]['shutdown_speed'] == .1
    now = engage(lifecycle, .01)
    now, trace = follow(lifecycle, now, (1., -.2))
    assert all(abs(result['positions'][0]-before[0]) <= .02*.01 + 1e-12 for before, result in trace)
    lifecycle.action('disengage', observations(now+.01, lifecycle.target), now=now+.01)
    edited[0]['speed'] = .4
    lifecycle.configure_manual(edited)
    assert {key: lifecycle.joints[0][key] for key in ('max_velocity', 'align_speed', 'shutdown_speed')} == {
        'max_velocity': .4, 'align_speed': .3, 'shutdown_speed': .1}


def test_shrinking_window_outside_current_target_rejects_all_changes_atomically():
    lifecycle = held()
    now = engage(lifecycle, .01)
    now, _ = follow(lifecycle, now, (1., -.2))
    lifecycle.action('disengage', observations(now+.01, lifecycle.target), now=now+.01)
    before = deepcopy(lifecycle.__dict__)
    edited = settings()
    edited[0].update(travel_rad=.01, kp=15., speed=.04)
    edited[1].update(follow=True, kd=2.)
    with pytest.raises(ValueError, match='return inside the new travel window'):
        lifecycle.configure_manual(edited)
    assert lifecycle.__dict__ == before
    edited[0]['travel_rad'] = .03
    lifecycle.configure_manual(edited)
    assert lifecycle.target == before['target']


@pytest.mark.parametrize('state', ['IDLE', 'ENGAGING', 'TELEOP', 'SHUTDOWN', 'FAULT'])
def test_manual_edits_reject_non_hold_active_states_atomically(state):
    lifecycle = held()
    if state == 'IDLE': lifecycle.action('idle', observations(.01, lifecycle.target), now=.01)
    elif state == 'ENGAGING':
        lifecycle.action('engage', observations(.01, lifecycle.target),
                         target=observations(.01, lifecycle.target, source=True), now=.01)
    elif state == 'TELEOP': engage(lifecycle, .01)
    elif state == 'SHUTDOWN': lifecycle.action('shutdown', observations(.01, lifecycle.target), now=.01)
    else: lifecycle.fault('fixture')
    before = deepcopy(lifecycle.__dict__)
    with pytest.raises(ValueError, match='DISABLED or HOLD'):
        lifecycle.configure_manual(settings())
    assert lifecycle.__dict__ == before


@pytest.mark.parametrize('case', ['missing', 'reverse', 'nonbool_follow', 'negative_gain', 'nan_gain',
                                  'zero_speed', 'speed_cap', 'zero_travel'])
def test_lifecycle_defensive_validation_is_atomic(case):
    lifecycle = held()
    before = deepcopy(lifecycle.__dict__)
    edited = settings()
    edited[0]['kp'] = 19.
    if case == 'missing': edited.pop()
    elif case == 'reverse': edited.reverse()
    else:
        key, value = {'nonbool_follow': ('follow', 1), 'negative_gain': ('kd', -.1),
            'nan_gain': ('kp', float('nan')), 'zero_speed': ('speed', 0.), 'speed_cap': ('speed', .501),
            'zero_travel': ('travel_rad', 0.)}[case]
        edited[-1][key] = value
    with pytest.raises(ValueError): lifecycle.configure_manual(edited)
    assert lifecycle.__dict__ == before


def test_manual_mode_requires_position_only_and_cannot_override_fixed_trial():
    ordinary = joints()
    ordinary[-1]['position_only'] = False
    with pytest.raises(ValueError, match='position-only'):
        TeleopLifecycle(ordinary).configure_manual(settings())
    trial = TeleopLifecycle(joints(), trial={'motor': NAMES[0], 'max_velocity': .05, 'max_excursion_rad': .03})
    with pytest.raises(ValueError, match='no fixed trial'):
        trial.configure_manual(settings())


def test_source_loss_holds_and_disable_is_the_only_way_to_start_a_new_window():
    lifecycle = held()
    now = engage(lifecycle, .01)
    now, _ = follow(lifecycle, now, (1., -.2), count=20)
    for _ in range(lifecycle.max_source_misses+1):
        now += .01
        lifecycle.step(observations(now, lifecycle.target), target=None, now=now)
    assert lifecycle.state == 'HOLD'
    old = lifecycle.target
    now += .01
    lifecycle.step(observations(now, old), target=observations(now, (1., 1.), source=True), now=now)
    assert lifecycle.state == 'HOLD' and lifecycle.target == old
    lifecycle.action('disable', now=now+.01)
    lifecycle.action('idle', observations(now+.02, (.5, .1)), now=now+.02)
    lifecycle.action('enable', observations(now+.02, (.5, .1)), now=now+.02)
    now = engage(lifecycle, now+.03)
    _, trace = follow(lifecycle, now, (1., 1.))
    assert trace[-1][1]['positions'] == pytest.approx([.53, .1])


def test_caller_mutation_cannot_change_committed_manual_settings():
    lifecycle = held()
    candidate = settings()
    lifecycle.configure_manual(candidate)
    candidate[0].update(kp=500., speed=.5, travel_rad=1.)
    candidate[1]['follow'] = True
    now = engage(lifecycle, .01)
    _, trace = follow(lifecycle, now, (1., 1.))
    assert trace[-1][1]['positions'] == pytest.approx([.13, -.2])
    assert trace[-1][1]['joints'][0]['kp'] == 12.


def test_tracking_correction_cannot_escape_manual_following_speed_cap():
    lifecycle = held()
    candidate = settings()
    candidate[0].update(speed=.01, travel_rad=None)
    lifecycle.configure_manual(candidate)
    now = engage(lifecycle, .01)
    before = lifecycle.target
    now += .01
    # The measured joint jumped far enough that the normal lead correction would
    # move the commanded position faster than the operator's selected speed.
    with pytest.raises(ValueError, match='manual following speed bound'):
        lifecycle.step(observations(now, (.5, before[1])),
                       target=observations(now, (.8, before[1]), source=True), now=now)
    assert lifecycle.state == 'FAULT' and lifecycle.target is None
