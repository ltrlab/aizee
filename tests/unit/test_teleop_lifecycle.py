"""Existing GUI intent, measured alignment, and bounded zero-return semantics."""
from copy import deepcopy
import math

import pytest

from robot_platform.teleop.lifecycle import TeleopLifecycle


def joints():
    return [dict(name='arm', lower=-2., upper=2., hold_kp=10., hold_kd=1., max_velocity=.5,
                 align_speed=.1, shutdown_speed=.05, max_lead=.2, tolerance=.005,
                 settled_velocity=.01, zero_position=0.),
            dict(name='slide', lower=-.05, upper=.05, hold_kp=5., hold_kd=.5, max_velocity=.01,
                 align_speed=.005, shutdown_speed=.002, max_lead=.01, tolerance=.0002,
                 settled_velocity=.001, zero_position=0.)]


def feedback(now, positions=(.3, .02), velocity=(0., 0.)):
    return [dict(name=joint['name'], position=q, velocity=v, sampled_at=now-.001,
                 sequence=round(now*1000)+1) for joint, q, v in zip(joints(), positions, velocity)]


def target(now, positions=(.6, .04)):
    return [{k: v for k, v in row.items() if k != 'velocity'} for row in feedback(now, positions)]


def in_state(state, *, positions=(.3, .02), settle_time_s=.1):
    lifecycle = TeleopLifecycle(joints(), settle_time_s=settle_time_s)
    lifecycle.action('idle', feedback(0., positions), now=0.)
    if state == 'IDLE': return lifecycle
    lifecycle.action('enable', feedback(0., positions), now=0.)
    if state == 'HOLD': return lifecycle
    lifecycle.action('engage', feedback(0., positions), target=target(0., positions), now=0.)
    if state == 'ENGAGING': return lifecycle
    lifecycle.step(feedback(.04, positions), target=target(.04, positions), now=.04)
    if settle_time_s:
        for now in (.08, .12, .16):
            lifecycle.step(feedback(now, positions), target=target(now, positions), now=now)
    assert lifecycle.state == 'TELEOP'
    return lifecycle


def test_idle_enable_and_hold_preserve_existing_gui_semantics():
    lifecycle = TeleopLifecycle(joints())
    with pytest.raises(ValueError, match='DISABLED'):
        lifecycle.action('enable', now=0.)
    assert lifecycle.state == 'DISABLED' and lifecycle.target is None
    idle = lifecycle.action('idle', feedback(0.), now=0.)
    assert idle['state'] == 'IDLE' and idle['operation'] == 'idle'
    assert idle['positions'] == [.3, .02]
    assert all(command['kp'] == command['kd'] == command['velocity'] == command['effort'] == 0 for command in idle['joints'])
    moved = lifecycle.step(feedback(.04, (.31, .021)), now=.04)
    assert moved['positions'] == [.31, .021] and moved['operation'] == 'idle'
    held = lifecycle.action('enable', feedback(.04, (.31, .021)), now=.04)
    assert held['state'] == 'HOLD' and held['operation'] == 'enable'
    assert held['joints'][0] == dict(position=.31, velocity=0., effort=0., kp=10., kd=1.)
    assert all(set(command) == {'position', 'velocity', 'effort', 'kp', 'kd'} for command in held['joints'])
    for now, leader in [(.08, target(.08, (-1., -.04))), (.12, None), (.16, {'bad': 'source'})]:
        proposal = lifecycle.step(feedback(now, (.30, .020)), target=leader, now=now)
        assert proposal['positions'] == [.31, .021] and proposal['state'] == 'HOLD'


@pytest.mark.parametrize('state', ['IDLE', 'HOLD'])
def test_engage_starts_at_measured_pose_and_uses_absolute_target(state):
    lifecycle = in_state(state)
    start = lifecycle.action('engage', feedback(.01), target=target(.01), now=.01)
    assert start['state'] == 'ENGAGING' and start['positions'] == [.3, .02]
    assert start['operation'] == ('enable' if state == 'IDLE' else 'move')
    proposal = lifecycle.step(feedback(.05), target=target(.05), now=.05)
    assert proposal['positions'] == pytest.approx([.304, .0202])
    assert proposal['state'] == 'ENGAGING'
    # Source values stay fixed: an absolute goal still causes approach; relative
    # registration would incorrectly keep the initial measured pose forever.
    assert proposal['positions'][0] > .3


def test_alignment_cannot_finish_just_because_time_elapsed_or_command_arrived():
    lifecycle = in_state('HOLD')
    lifecycle.action('engage', feedback(0.), target=target(0.), now=0.)
    for index in range(1, 51):
        now = index*.04
        proposal = lifecycle.step(feedback(now), target=target(now), now=now)
        assert proposal['state'] == 'ENGAGING'
        assert proposal['positions'][0] <= .5 + 1e-12  # Measured position + max lead.
        assert proposal['positions'][1] <= .03 + 1e-12


def test_alignment_requires_every_measured_joint_and_settled_velocity():
    lifecycle = in_state('HOLD', positions=(.6, .04))
    lifecycle.action('engage', feedback(0., (.6, .04)), target=target(0.), now=0.)
    # Within tolerance of the goal but still moving (0.05-0.1 rad/s from the positions).
    for now, q in ((.04, .598), (.08, .602), (.12, .598), (.16, .6)):
        proposal = lifecycle.step(feedback(now, (q, .04)), target=target(now), now=now)
        assert proposal['state'] == 'ENGAGING'
    for now in (.20, .24, .28):
        assert lifecycle.step(feedback(now, (.6, .04)), target=target(now), now=now)['state'] == 'ENGAGING'
    assert lifecycle.step(feedback(.32, (.6, .04)), target=target(.32), now=.32)['state'] == 'TELEOP'


def test_a_drives_speed_noise_at_rest_does_not_hold_up_settling():
    # Minerva's drives report speeds up to ~0.2 rad/s at rest; positions decide.
    lifecycle = in_state('HOLD', positions=(.6, .04))
    lifecycle.action('engage', feedback(0., (.6, .04)), target=target(0.), now=0.)
    states = [lifecycle.step(feedback(now, (.6, .04), velocity=(.06, .002)), target=target(now), now=now)['state']
              for now in (.04, .08, .12, .16)]
    assert states == ['ENGAGING', 'ENGAGING', 'ENGAGING', 'TELEOP']


@pytest.mark.parametrize('state', ['ENGAGING', 'TELEOP', 'SHUTDOWN'])
def test_disengage_captures_current_measured_hold_and_ignores_leaders(state):
    lifecycle = in_state('TELEOP' if state == 'TELEOP' else 'ENGAGING', settle_time_s=0.)
    now = .04 if state == 'TELEOP' else 0.
    if state == 'SHUTDOWN': lifecycle.action('shutdown', feedback(now), now=now)
    held = lifecycle.action('disengage', feedback(now+.01, (.29, .019)), now=now+.01)
    assert held['state'] == 'HOLD' and held['positions'] == [.29, .019]
    assert lifecycle.step(feedback(now+.05, (.29, .019)), target=None, now=now+.05)['positions'] == [.29, .019]


@pytest.mark.parametrize('state', ['IDLE', 'HOLD', 'ENGAGING', 'TELEOP'])
def test_shutdown_starts_measured_from_every_enabled_gui_phase(state):
    lifecycle = in_state(state, settle_time_s=0.)
    now = .05 if state == 'TELEOP' else .01
    proposal = lifecycle.action('shutdown', feedback(now, (.29, .019)), now=now)
    assert proposal['state'] == 'SHUTDOWN' and proposal['positions'] == [.29, .019]
    assert proposal['operation'] == ('enable' if state == 'IDLE' else 'move')
    proposal = lifecycle.step(feedback(now+.04, (.29, .019)), target={'unavailable': True}, now=now+.04)
    assert proposal['positions'] == pytest.approx([.288, .01892])


def test_shutdown_speed_is_elapsed_time_based_for_different_loop_rates():
    def run(dt):
        lifecycle = in_state('HOLD')
        lifecycle.action('shutdown', feedback(0.), now=0.)
        measured = [.3, .02]
        for index in range(1, round(1/dt)+1):
            now = index*dt
            measured = lifecycle.step(feedback(now, measured), now=now)['positions']
        return measured
    assert run(.01) == pytest.approx([.25, .018])
    assert run(.04) == pytest.approx(run(.01))


def test_shutdown_disables_only_after_measured_arrival_and_settling():
    lifecycle = in_state('HOLD', positions=(0., 0.))
    lifecycle.action('shutdown', feedback(0., (0., 0.)), now=0.)
    # The slide is within tolerance of zero but still moving (0.0025-0.005 m/s).
    for now, q in ((.04, .0001), (.08, -.0001), (.12, .0001), (.16, 0.)):
        assert lifecycle.step(feedback(now, (0., q)), now=now)['state'] == 'SHUTDOWN'
    for now in (.20, .24, .28):
        assert lifecycle.step(feedback(now, (0., 0.)), now=now)['state'] == 'SHUTDOWN'
    finished = lifecycle.step(feedback(.32, (0., 0.)), now=.32)
    assert finished['state'] == 'DISABLED' and finished['operation'] == 'disable'
    assert finished['positions'] is finished['joints'] is None
    assert lifecycle.step()['operation'] == 'none'


@pytest.mark.parametrize('state', ['IDLE', 'HOLD', 'ENGAGING', 'TELEOP', 'SHUTDOWN'])
def test_disable_clears_every_pending_target_without_requiring_readings(state):
    lifecycle = in_state('ENGAGING' if state == 'SHUTDOWN' else state, settle_time_s=0.)
    if state == 'SHUTDOWN': lifecycle.action('shutdown', feedback(0.), now=0.)
    disabled = lifecycle.action('disable')
    assert disabled['operation'] == 'disable' and disabled['state'] == 'DISABLED'
    assert disabled['positions'] is disabled['joints'] is None and lifecycle.target is None


@pytest.mark.parametrize('state', ['ENGAGING', 'TELEOP'])
def test_missing_source_holds_last_target_then_drops_to_measured_hold(state):
    lifecycle = in_state(state, settle_time_s=0.)
    now = .04 if state == 'TELEOP' else 0.
    last = lifecycle.target
    for _ in range(lifecycle.max_source_misses):
        now += .04
        proposal = lifecycle.step(feedback(now), target=None, now=now)
        assert proposal['state'] == state and proposal['operation'] == 'move'
        assert tuple(proposal['positions']) == last
    now += .04
    held = lifecycle.step(feedback(now, (.31, .021)), target=None, now=now)
    assert held['state'] == 'HOLD' and held['positions'] == [.31, .021]
    assert 'Leader input lost' in held['reason']
    # Fresh input alone never resumes following; Engage must be explicit.
    now += .04
    assert lifecycle.step(feedback(now, (.31, .021)), target=target(now), now=now)['positions'] == [.31, .021]
    assert lifecycle.state == 'HOLD'
    now += .04
    resumed = lifecycle.action('engage', feedback(now, (.31, .021)), target=target(now), now=now)
    assert resumed['state'] == 'ENGAGING' and resumed['reason'] is None


def test_fresh_source_resets_the_miss_budget():
    lifecycle = in_state('TELEOP', settle_time_s=0.)
    now = .04
    for _ in range(4):
        for _ in range(lifecycle.max_source_misses):
            now += .04
            assert lifecycle.step(feedback(now), target=None, now=now)['state'] == 'TELEOP'
        now += .04
        assert lifecycle.step(feedback(now), target=target(now, (.3, .02)), now=now)['state'] == 'TELEOP'


def test_missing_source_cannot_hold_a_target_the_arm_has_left():
    lifecycle = in_state('TELEOP', settle_time_s=0.)
    with pytest.raises(ValueError, match='lead bound'):
        lifecycle.step(feedback(.08, (.6, .02)), target=None, now=.08)
    assert lifecycle.state == 'FAULT' and lifecycle.target is None


@pytest.mark.parametrize('misses', [-1, 11, 1.5, True])
def test_miss_budget_must_be_a_small_integer(misses):
    with pytest.raises(ValueError):
        TeleopLifecycle(joints(), max_source_misses=misses)


@pytest.mark.parametrize('failure', ['missing_feedback', 'stale_feedback', 'partial_feedback', 'nonfinite',
                                   'wrong_order', 'future', 'source_stale', 'source_repeated', 'stall'])
def test_active_validation_failures_latch_fault_and_clear_targets(failure):
    lifecycle = in_state('ENGAGING')
    now, measured, mapped = .04, feedback(.04), target(.04)
    if failure == 'missing_feedback': measured = None
    elif failure == 'stale_feedback': measured = feedback(0.)
    elif failure == 'partial_feedback': measured.pop()
    elif failure == 'nonfinite': measured[0]['position'] = math.nan
    elif failure == 'wrong_order': measured.reverse()
    elif failure == 'future': measured[0]['sampled_at'] = .05
    elif failure == 'source_stale': mapped = target(-.1)
    elif failure == 'source_repeated': mapped = target(0., (.3, .02))
    elif failure == 'stall': now = .2; measured = feedback(now); mapped = target(now)
    with pytest.raises(ValueError): lifecycle.step(measured, target=mapped, now=now)
    assert lifecycle.state == 'FAULT' and lifecycle.target is None
    assert lifecycle.step(feedback(.21), target=target(.21), now=.21)['operation'] == 'disable'
    lifecycle.action('disable')
    assert lifecycle.state == 'FAULT'
    with pytest.raises(ValueError): lifecycle.action('idle', feedback(.21), now=.21)


def test_invalid_operator_actions_do_not_disengage_or_change_state():
    lifecycle = in_state('ENGAGING')
    previous = (lifecycle.state, lifecycle.target, lifecycle.reason)
    for action in ('enable', 'engage', 'reset', 'bogus'):
        with pytest.raises(ValueError): lifecycle.action(action)
        assert (lifecycle.state, lifecycle.target, lifecycle.reason) == previous


def test_guarded_collision_or_ack_failure_can_latch_the_same_disable_proposal():
    lifecycle = in_state('HOLD')
    result = lifecycle.fault('Zero-return path blocked by collision')
    assert result['state'] == 'FAULT' and result['operation'] == 'disable'
    assert result['positions'] is None and 'collision' in result['reason']


def test_configuration_copies_prevent_live_parameter_changes():
    config = joints()
    lifecycle = TeleopLifecycle(config)
    config[0]['align_speed'] = 50.
    external = lifecycle.joints
    external[0]['hold_kp'] = 1000.
    assert lifecycle.joints[0]['align_speed'] == .1 and lifecycle.joints[0]['hold_kp'] == 10.


@pytest.mark.parametrize('change', [
    {'name': ''}, {'align_speed': 1.}, {'shutdown_speed': 1.}, {'zero_position': 3.},
    {'tolerance': 0.}, {'max_lead': True}, {'hold_kp': math.nan}, {'settled_velocity': -1.},
])
def test_invalid_joint_configuration_is_rejected(change):
    config = joints(); config[0].update(change)
    with pytest.raises(ValueError): TeleopLifecycle(config)


def test_hold_faults_instead_of_sending_an_unbounded_lead():
    lifecycle = in_state('HOLD')
    with pytest.raises(ValueError, match='lead'):
        lifecycle.step(feedback(.04, (-1., .02)), now=.04)
    assert lifecycle.target is None


def test_commands_slow_toward_an_end_within_the_receivers_braking_curve():
    from robot_platform.control.approach import COMMAND_HEADROOM, command_step, limit_speed
    curve = {'lower': -2., 'upper': 2., 'max_velocity': 2.5, 'max_tracking_error': .05, 'stop_acceleration': 5.,
             'reaction_s': .04}
    curved = joints()
    curved[0].update(max_velocity=2., approach=curve)
    positions = (1.7, .02)
    lifecycle = TeleopLifecycle(curved, settle_time_s=0.)
    lifecycle.action('idle', feedback(0., positions), now=0.)
    lifecycle.action('enable', feedback(0., positions), now=0.)
    lifecycle.action('engage', feedback(0., positions), target=target(0., positions), now=0.)
    lifecycle.step(feedback(.04, positions), target=target(.04, positions), now=.04)
    assert lifecycle.state == 'TELEOP'
    # 0.3 rad below the upper end, toward it: only the curve's speed there, with headroom (about 1.23 rad/s),
    # and no more than it allows where the step ends (about 1.12 rad/s, 0.25 rad from the end).
    toward = lifecycle.step(feedback(.08, positions), target=target(.08, (1.95, .02)), now=.08)['positions'][0]
    allowed = COMMAND_HEADROOM * limit_speed(curve, .04, .3)
    assert allowed < 1.5 and toward == pytest.approx(1.7 + command_step(curve, .04, 1.7, 1, 2., .04))
    assert toward < 1.7 + allowed * .04
    # Away from it: the full 2 rad/s.
    away = lifecycle.step(feedback(.12, (toward, .02)), target=target(.12, (1., .02)), now=.12)['positions'][0]
    assert away == pytest.approx(toward - 2. * .04)


def test_a_lagging_joint_near_an_end_is_pulled_only_as_hard_as_its_curve_allows():
    from robot_platform.control.approach import command_speed, command_step
    curve = {'lower': -2., 'upper': 2., 'max_velocity': 2.5, 'max_tracking_error': .05, 'stop_acceleration': 5.,
             'reaction_s': .04}
    curved = joints()
    curved[0].update(max_velocity=2., approach=curve)
    lifecycle = TeleopLifecycle(curved, settle_time_s=0.)
    positions = (1.7, .02)
    lifecycle.action('idle', feedback(0., positions), now=0.)
    lifecycle.action('enable', feedback(0., positions), now=0.)
    lifecycle.action('engage', feedback(0., positions), target=target(0., positions), now=0.)
    lifecycle.step(feedback(.04, positions), target=target(.04, positions), now=.04)
    # The joint lags at 1.55 behind its 1.7 target, moving toward the end at 1.5 rad/s: it may lead by
    # 0.2 x (allowed / 2 rad/s) only.
    moving = lifecycle.step(feedback(.08, (1.55, .02), (1.5, 0.)), target=target(.08, (1.95, .02)), now=.08)
    lead = .2 * command_speed(curve, .04, 1.55, 1, 2.) / 2.
    assert lead < .2 and moving['positions'][0] == pytest.approx(1.55 + lead)
    assert 1.55 + lead < 1.7 + command_step(curve, .04, 1.7, 1, 2., .04)  # The lead binds, not the slew.
    # Stuck (reading still, as on static friction), the same: a joint breaks free with the pull it has.
    stuck = lifecycle.step(feedback(.12, (1.55, .02)), target=target(.12, (1.95, .02)), now=.12)
    assert stuck['positions'][0] == pytest.approx(1.55 + lead)


def test_a_slewed_move_carries_its_velocity_and_friction_feed_forward():
    from robot_platform.control.approach import friction_feedforward
    ramped = joints()
    for joint in ramped:
        joint.update(interpolation_s=.04)
    ramped[0].update(friction_nm=.4)
    lifecycle = TeleopLifecycle(ramped, settle_time_s=0.)
    positions = (.3, .02)
    lifecycle.action('idle', feedback(0., positions), now=0.)
    lifecycle.action('enable', feedback(0., positions), now=0.)
    engaged = lifecycle.action('engage', feedback(0., positions), target=target(0., positions), now=0.)
    # A snap to the measured pose carries no velocity.
    assert all(command['velocity'] == 0. and command['effort'] == 0. for command in engaged['joints'])
    lifecycle.step(feedback(.04, positions), target=target(.04, positions), now=.04)
    assert lifecycle.state == 'TELEOP'
    # The leader moves away: the arm follows at its 0.5 rad/s cap and the slide at 0.01, and the receiver ramps
    # each command from the last one over its 40 ms with that velocity fed forward.
    arm, slide = lifecycle.step(feedback(.08, positions), target=target(.08, (.6, .04)), now=.08)['joints']
    assert arm['position'] == pytest.approx(.3 + .5 * .04) and arm['velocity'] == pytest.approx(.5)
    assert slide['position'] == pytest.approx(.02 + .01 * .04) and slide['velocity'] == pytest.approx(.01)
    # Friction feed-forward in the direction of motion, for a joint whose sweep measured its friction.
    assert arm['effort'] == pytest.approx(friction_feedforward(.4, .5)) and slide['effort'] == 0.
    # A late cycle moves no more than the receiver's period's worth: the ramp never outruns the cap.
    late = lifecycle.step(feedback(.14, positions), target=target(.14, (.6, .04)), now=.14)['joints'][0]
    assert late['position'] == pytest.approx(arm['position'] + .5 * .04) and late['velocity'] == pytest.approx(.5)
    # The leader stops: so does the command, with no velocity.
    here = (late['position'], .02 + .01 * .08)
    still = lifecycle.step(feedback(.18, positions), target=target(.18, here), now=.18)['joints'][0]
    assert still['position'] == pytest.approx(late['position']) and still['velocity'] == 0. and still['effort'] == 0.
    # Holding sends no velocity.
    held = lifecycle.action('disengage', feedback(.22, positions), now=.22)
    assert all(command['velocity'] == 0. and command['effort'] == 0. for command in held['joints'])


def test_without_an_interpolation_period_commands_carry_no_velocity():
    lifecycle = in_state('TELEOP', settle_time_s=0.)
    moving = lifecycle.step(feedback(.08), target=target(.08, (.6, .04)), now=.08)
    assert moving['positions'][0] > .3
    assert all(command['velocity'] == 0. and command['effort'] == 0. for command in moving['joints'])


def test_commands_hold_the_arm_against_gravity_at_the_commanded_pose():
    held = joints()
    # The arm joint holds 3 sin(q) against gravity; the slide's load hangs on the arm's angle plus its own.
    held[0].update(gravity=[[3., {'arm': 1}]], max_effort=10.)
    held[1].update(gravity=[[-1., {'arm': 1, 'slide': 1}]], max_effort=.2)
    lifecycle = TeleopLifecycle(held, settle_time_s=0.)
    positions = (.3, .02)
    idle = lifecycle.action('idle', feedback(0., positions), now=0.)
    assert all(command['effort'] == 0. for command in idle['joints'])  # Zero gains: no effort.
    lifecycle.action('enable', feedback(0., positions), now=0.)
    hold = lifecycle.step(feedback(.04, positions), now=.04)['joints']
    assert hold[0]['effort'] == pytest.approx(3. * math.sin(.3))
    assert hold[1]['effort'] == pytest.approx(-.18)  # Within 90% of the slide's 0.2 Nm effort limit.
    with pytest.raises(ValueError, match='Gravity terms'):
        TeleopLifecycle([dict(held[0], gravity=[[3., {'elsewhere': 1}]]), held[1]])


def test_a_hold_at_the_measured_pose_stays_inside_the_usable_range():
    # A joint may settle a hair past a command at the edge of its usable range; the receiver refuses a command
    # within the stopping margin (0.045 rad here), so holding it there would fault the session.
    from robot_platform.control.approach import end_margin
    curve = {'lower': -2., 'upper': 2., 'max_velocity': 2.5, 'max_tracking_error': .05, 'stop_acceleration': 5.,
             'reaction_s': .04}
    curved = joints()
    curved[0].update(max_velocity=2., approach=curve)
    lifecycle = TeleopLifecycle(curved, settle_time_s=0.)
    lifecycle.action('idle', feedback(0., (1.98, .02)), now=0.)
    held = lifecycle.action('enable', feedback(0., (1.98, .02)), now=0.)
    assert held['positions'][0] == pytest.approx(2. - end_margin(curve, .04)) and held['positions'][1] == .02
    assert lifecycle.step(feedback(.04, (1.98, .02)), now=.04)['positions'][0] == pytest.approx(held['positions'][0])



def test_a_moving_joint_is_led_from_where_it_will_be_when_the_receiver_has_ramped():
    # The receiver ramps each command over 40 ms, and the feedback is a sample old: taken from the measured
    # position alone, the lead held right J1 to 0.57 rad/s at a 0.048 rad lead (80 ms of delay). A joint moving
    # at 1.25 rad/s is led from where it will be once the ramp ends: its 10 ms old sample, 50 ms on.
    ramped = joints()
    for joint in ramped:
        joint.update(interpolation_s=.04)
    ramped[0].update(max_velocity=10., max_lead=.03)
    lifecycle = TeleopLifecycle(ramped, settle_time_s=0.)
    lifecycle.action('idle', feedback(0., (.3, .02)), now=0.)
    lifecycle.action('enable', feedback(0., (.3, .02)), now=0.)
    lifecycle.action('engage', feedback(0., (.3, .02)), target=target(0., (.3, .02)), now=0.)
    lifecycle.step(feedback(.04, (.3, .02)), target=target(.04, (.3, .02)), now=.04)
    assert lifecycle.state == 'TELEOP'
    # At rest, the lead from where it is.
    assert lifecycle.step(feedback(.08, (.3, .02)), target=target(.08, (1.5, .04)), now=.08)['positions'][0] ==         pytest.approx(.33)
    rows = feedback(.12, (.35, .02))
    for row in rows:
        row['sampled_at'] = .11
    moving = lifecycle.step(rows, target=target(.12, (1.5, .04)), now=.12)['positions'][0]
    assert moving == pytest.approx(.35 + (.35 - .3) / .04 * (.01 + .04) + .03)
    assert moving > .35 + .03
