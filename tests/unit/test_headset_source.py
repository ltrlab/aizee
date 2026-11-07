"""The headset as the teleop loop's target (teleop.headset_source): the arms' CAD angles from the worker become motor
positions through the derived report's scale and offset, each trigger its gripper's position between open and
closed; HOLD aims where the arms are, Engage holds that pose but raises a hanging arm to its ready pose, following
seeds the worker from the aligned pose and takes only answers computed from the latest seed while they are fresh,
and B/Y asks for Hold only while engaged."""

import pytest

from robot_platform.teleop.headset_source import READY, HeadsetSource, validate_request, worker_limits

# Every other joint's motor turns the other way from its CAD joint, with an offset: the mapping must invert both.
SCALES = {1: 1.0, 2: -1.0, 3: 1.0, 4: -1.0, 5: 1.0, 6: -1.0}
OFFSETS = {1: 0.1, 2: -0.2, 3: 0.3, 4: 0.0, 5: -0.5, 6: 0.6}


def report(usable=(-3.0, 3.0)):
    rows = []
    for arm in ('left', 'right'):
        for n in range(1, 7):
            rows.append(
                {
                    'motor': f'{arm}_arm_j{n}',
                    'cad_joint': f'{arm}_arm_j{n}',
                    'scale': SCALES[n],
                    'offset': OFFSETS[n],
                    'usable': list(usable),
                    'opens': None,
                }
            )
        # The left gripper opens toward its upper end, the right toward its lower.
        rows.append(
            {
                'motor': f'{arm}_gripper',
                'cad_joint': f'{arm}_finger',
                'scale': 0.01,
                'offset': 0.0,
                'usable': [-1.0, 1.5],
                'opens': 1 if arm == 'left' else -1,
            }
        )
    return rows


NAMES = [row['motor'] for row in report()]
MEASURED = {name: 0.05 * i for i, name in enumerate(NAMES)}


def cad(arm, measured=MEASURED):
    return [SCALES[n] * measured[f'{arm}_arm_j{n}'] + OFFSETS[n] for n in range(1, 7)]


def motors(arm, positions):
    """The motor positions of an arm's CAD joints."""
    return {f'{arm}_arm_j{n}': (q - OFFSETS[n]) / SCALES[n] for n, q in enumerate(positions, 1)}


# Where the arms hang: every CAD joint at 0.
REFERENCE = dict(MEASURED, **motors('left', [0.0] * 6), **motors('right', [0.0] * 6))


def answer(seed, at, *, left=None, right=None, closure=None, trigger=None, holds=0):
    commands = {}
    for arm, positions in (('left', left), ('right', right)):
        commands[arm] = {
            'positions': positions,
            'closure': closure,
            'gripped': positions is not None,
            'tracking': True,
            'clamped': False,
            'converged': True,
            'at_limit': [],
            'sigma': 0.15,
            'near_singular': False,
            'holds': holds if arm == 'left' else 0,
            'trigger': trigger,
        }
    return ({'seed': seed, 'commands': commands, 'compute_ms': 3.0}, at)


def rows(target):
    return {row['name']: row['position'] for row in target}


def test_hold_aims_where_the_arms_are_with_each_gripper_at_its_trigger():
    source = HeadsetSource(NAMES, report())
    assert rows(source.target('HOLD', 1.0, MEASURED)) == MEASURED  # No answer yet: the grippers stay too.
    source.receive(answer(0, 1.0, trigger=1.0))
    held = rows(source.target('HOLD', 1.01, MEASURED))
    assert held['left_gripper'] == -1.0 and held['right_gripper'] == 1.5  # Closed: the end away from opening.
    assert all(held[name] == MEASURED[name] for name in NAMES if 'gripper' not in name)
    source.receive(answer(0, 1.02, trigger=0.0))
    held = rows(source.target('IDLE', 1.03, MEASURED))
    assert held['left_gripper'] == 1.5 and held['right_gripper'] == -1.0  # Open.
    assert source.take_seed() is None  # Nothing seeds before Engage.
    assert source.target('HOLD', 1.04, {'left_arm_j1': 0.0}) is None  # An incomplete reading gives no target.


def test_engage_holds_the_pose_it_began_with_and_seeds_the_worker_once_aligned():
    source = HeadsetSource(NAMES, report(), reference=REFERENCE)  # Neither arm hangs: neither rises.
    source.receive(answer(0, 2.0, trigger=0.5))
    aligned = rows(source.target('ENGAGING', 2.01, MEASURED))
    assert aligned['left_gripper'] == pytest.approx(0.25)  # Half closed: halfway from 1.5 to -1.
    assert source.raised == () and all(aligned[name] == MEASURED[name] for name in NAMES if 'gripper' not in name)
    moved = {name: value + 0.1 for name, value in MEASURED.items()}
    assert rows(source.target('ENGAGING', 2.03, moved)) == aligned  # Held while it aligns.
    assert source.take_seed() is None  # Nothing seeds while the arms align: a drag then would jump at Teleop.
    assert rows(source.target('TELEOP', 2.05, moved)) == aligned
    number, seed = source.take_seed()
    assert number == 1 and source.take_seed() is None  # Once.
    assert seed['left'] == pytest.approx(cad('left')) and seed['right'] == pytest.approx(cad('right'))


def test_engage_raises_a_hanging_arm_to_its_ready_pose_and_the_drag_starts_there():
    # The left arm hangs (its swivels turned, J2-J4 at the reference); the right's elbow is bent.
    measured = dict(
        REFERENCE,
        **motors('left', [0.4, 0.1, -0.1, 0.2, -0.4, 0.0]),
        **motors('right', [0.0, 0.0, -1.0, 0.0, 0.0, 0.0]),
    )
    source = HeadsetSource(NAMES, report(), reference=REFERENCE)
    aligned = rows(source.target('ENGAGING', 3.0, measured))
    assert source.raised == ('left',)
    assert cad('left', aligned) == pytest.approx(READY['left'])
    assert cad('right', aligned) == pytest.approx(cad('right', measured))  # Not hanging: where it is.
    assert source.snapshot()['raised'] == ['left']
    source.target('TELEOP', 6.0, dict(measured, **motors('left', READY['left'])))
    number, seed = source.take_seed()
    assert seed['left'] == pytest.approx(READY['left'])  # From the pose aligned to.
    # Held there and engaged again: the arm no longer hangs, so it stays.
    source.target('HOLD', 7.0, aligned)
    assert cad('left', rows(source.target('ENGAGING', 7.1, aligned))) == pytest.approx(READY['left'])
    assert source.raised == ()
    # Without the reference no arm rises.
    plain = HeadsetSource(NAMES, report())
    assert cad('left', rows(plain.target('ENGAGING', 8.0, measured))) == pytest.approx(cad('left', measured))


def test_a_ready_pose_outside_the_limits_refuses_the_session():
    with pytest.raises(ValueError, match="ready pose is outside the left arm's limits at J3"):
        HeadsetSource(NAMES, report(usable=(-1.5, 1.5)), reference=REFERENCE)
    HeadsetSource(NAMES, report(usable=(-1.5, 1.5)))  # Without the reference nothing rises: no ready pose to check.


def test_following_takes_answers_from_the_latest_seed_and_turns_cad_angles_into_motor_positions():
    source = HeadsetSource(NAMES, report(), max_age_s=0.2, start_s=0.5)
    source.target('ENGAGING', 3.0, MEASURED)
    source.take_seed()
    left = [0.3, -0.4, 0.5, -0.6, 0.7, -0.8]
    source.receive(answer(0, 3.02, left=left, right=cad('right'), closure=1.0))  # From before this Engage: ignored.
    assert rows(source.target('TELEOP', 3.03, MEASURED)) == rows(source.target('ENGAGING', 3.03, MEASURED))
    source.receive(answer(1, 3.04, left=left, right=cad('right'), closure=1.0))
    followed = rows(source.target('TELEOP', 3.05, MEASURED))
    for n in range(1, 7):
        assert SCALES[n] * followed[f'left_arm_j{n}'] + OFFSETS[n] == pytest.approx(left[n - 1])
        assert followed[f'right_arm_j{n}'] == pytest.approx(MEASURED[f'right_arm_j{n}'])
    assert followed['left_gripper'] == -1.0 and followed['right_gripper'] == 1.5
    # A hand lost (no closure) keeps its gripper's last target.
    source.receive(answer(1, 3.06, left=left, right=cad('right'), closure=None))
    assert rows(source.target('TELEOP', 3.07, MEASURED))['left_gripper'] == -1.0
    # An answer older than max_age_s is not followed: no target, which the lifecycle treats as a lost input. Never
    # the seed again, though Engage began only 0.3 s ago (inside start_s): that would pull the arm back to it.
    assert source.target('TELEOP', 3.3, MEASURED) is None and source.stale_cycles == 1


def test_until_the_first_answer_following_holds_the_seed_only_briefly():
    source = HeadsetSource(NAMES, report(), start_s=0.5)
    aligned = rows(source.target('ENGAGING', 4.0, MEASURED))
    # Seeded as following begins, however long it took to align.
    assert rows(source.target('TELEOP', 6.0, MEASURED)) == aligned
    assert rows(source.target('TELEOP', 6.4, MEASURED)) == aligned
    assert source.target('TELEOP', 6.6, MEASURED) is None


def test_a_hold_ends_following_and_the_next_engage_seeds_again_from_where_the_arms_are():
    source = HeadsetSource(NAMES, report())
    source.target('ENGAGING', 5.0, MEASURED)
    source.target('TELEOP', 5.01, MEASURED)
    assert source.take_seed()[0] == 1
    source.target('HOLD', 5.02, MEASURED)
    assert source.aligned is None
    moved = {name: value - 0.2 for name, value in MEASURED.items()}
    source.target('ENGAGING', 5.03, moved)
    source.target('TELEOP', 5.04, moved)
    number, seed = source.take_seed()
    assert number == 2 and seed['left'] == pytest.approx(cad('left', moved))
    source.target('DISABLED', 5.04, moved)
    assert source.aligned is None and source.target('DISABLED', 5.05, moved) is None


def test_b_or_y_asks_for_hold_only_while_engaged():
    source = HeadsetSource(NAMES, report())
    source.receive(answer(0, 6.0, holds=0))
    assert not source.hold_wanted('HOLD')
    source.receive(answer(0, 6.01, holds=1))
    assert not source.hold_wanted('HOLD')  # Pressed while holding: no Engage, and it is not saved for later.
    assert not source.hold_wanted('TELEOP')
    source.receive(answer(0, 6.02, holds=2))
    assert source.hold_wanted('TELEOP') and source.hold_requests == 1
    assert not source.hold_wanted('TELEOP')  # Once a press.


def test_the_request_and_the_arms_it_drives():
    assert validate_request({'kind': 'headset'}) == {'kind': 'headset'}
    for bad in ({'kind': 'headset', 'scale': 2}, {'kind': 'policy'}, 'headset'):
        with pytest.raises(ValueError, match='no options'):
            validate_request(bad)
    one = [row for row in report() if row['motor'].startswith('right')]
    assert set(worker_limits(one, [row['motor'] for row in one])) == {'right'}
    assert set(HeadsetSource([row['motor'] for row in one], report()).arms) == {'right'}
    with pytest.raises(ValueError, match='whole arms'):
        HeadsetSource(['left_arm_j1', 'left_gripper'], report())
    assert worker_limits(report(), NAMES)['left']['left_arm_j2'] == (-3.2, 2.8)  # Usable, through the CAD map.
