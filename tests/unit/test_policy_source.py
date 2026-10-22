"""A policy's chunks as the loop's target: aligning to a preview, following in real time from the row for now,
crossfading between chunks, and holding (no target) when chunks go stale."""
import numpy as np
import pytest

from robot_platform.teleop.policy_source import PolicySource


def positions(rows):
    return None if rows is None else [round(row['position'], 6) for row in rows]


def ramp(start, rows=11, step=.1):
    """Two joints moving +step and -step a row from `start`: row k is meant for k * 0.1 s after the observation."""
    return [[start + k * step, -(start + k * step)] for k in range(rows)]


def source(**options):
    return PolicySource(('a', 'b'), action_dt=.1, **options)


def test_hold_previews_the_first_row_engage_aligns_to_it_and_following_holds_it_until_the_first_chunk():
    policy = source()
    assert policy.target('IDLE', 0.) is None and policy.wants('IDLE') is None  # Nothing asked before Enable.
    assert policy.target('HOLD', 1.) is None and policy.wants('HOLD') == 'preview'
    policy.asked(1, 1., 'preview')
    assert policy.wants('HOLD') is None  # One request at a time.
    assert policy.receive(1, ramp(.5), 1.3)
    assert positions(policy.target('HOLD', 1.4)) == [.5, -.5]  # The preview's first row, unmoving.
    assert positions(policy.target('ENGAGING', 2.)) == [.5, -.5]  # Engage aligns to it.
    assert policy.wants('HOLD') is None  # Previewed once.
    assert positions(policy.target('TELEOP', 3.)) == [.5, -.5]  # Following begins: hold it until a chunk arrives.
    assert policy.wants('TELEOP') == 'rollout'
    policy.asked(2, 3., 'rollout')
    assert positions(policy.target('TELEOP', 3.5)) == [.5, -.5]


def test_a_late_chunk_starts_at_the_row_for_now_and_the_next_crossfades_in():
    policy = source(blend_s=.2)
    policy.target('HOLD', 0.)
    policy.asked(1, 0., 'preview')
    policy.receive(1, ramp(0.), .2)
    policy.target('TELEOP', 1.)
    policy.asked(2, 1., 'rollout')  # Observed at 1.0 s...
    assert policy.receive(2, ramp(1.), 1.25)  # ...answered 0.25 s later.
    assert positions(policy.target('TELEOP', 1.25)) == [1.25, -1.25]  # Row 2.5: the rows already past are skipped.
    assert positions(policy.target('TELEOP', 1.3)) == [1.3, -1.3]
    policy.asked(3, 1.3, 'rollout')
    policy.receive(3, ramp(2.), 1.4)  # It disagrees: 2.0 + 0.1 * 1 at 1.4 s against 1.4 from the first.
    assert positions(policy.target('TELEOP', 1.4)) == [1.4, -1.4]  # Arrival: still all the old chunk,
    assert positions(policy.target('TELEOP', 1.5)) == pytest.approx([1.85, -1.85])  # then half way at 0.1 s,
    assert positions(policy.target('TELEOP', 1.6)) == pytest.approx([2.3, -2.3])  # then only the new one.
    sequences = [policy.target('TELEOP', 1.6 + i / 100)[0]['sequence'] for i in range(5)]
    assert sequences == sorted(set(sequences))  # A new sample every cycle: never counted as a miss.


def test_a_stale_chunk_gives_no_target_and_a_dead_worker_is_given_up():
    policy = source(max_age_s=.5)
    policy.target('HOLD', 0.)
    policy.asked(1, 0., 'preview')
    policy.receive(1, ramp(0.), .1)
    policy.target('TELEOP', 1.)
    policy.asked(2, 1., 'rollout')
    policy.receive(2, ramp(1.), 1.1)
    assert policy.target('TELEOP', 1.4) is not None
    assert policy.target('TELEOP', 1.6) is None  # Past max_age_s from its observation: hold, as for lost leaders.
    assert policy.stale_cycles == 1
    policy.asked(3, 1.6, 'rollout')
    assert policy.wants('TELEOP') is None
    policy.target('TELEOP', 2.2)  # Unanswered for over max_age_s: given up, asked again.
    assert policy.wants('TELEOP') == 'rollout' and 'did not answer' in policy.last_error
    assert not policy.receive(3, ramp(9.), 2.3)  # Its late answer is dropped.


def test_following_before_any_chunk_holds_the_aligned_row_only_briefly():
    policy = source(max_age_s=.5)
    policy.target('HOLD', 0.)
    policy.asked(1, 0., 'preview')
    policy.receive(1, ramp(.2), .1)
    assert positions(policy.target('TELEOP', 1.)) == [.2, -.2]
    assert positions(policy.target('TELEOP', 1.4)) == [.2, -.2]
    assert policy.target('TELEOP', 1.6) is None  # No rollout chunk within max_age_s of following: hold.


def test_hold_ends_the_rollout_and_engage_aligns_anew_from_where_the_arms_are():
    policy = source()
    policy.target('HOLD', 0.)
    policy.asked(1, 0., 'preview')
    policy.receive(1, ramp(0.), .1)
    policy.target('TELEOP', 1.)
    policy.asked(2, 1., 'rollout')
    policy.receive(2, ramp(1.), 1.1)
    assert policy.target('TELEOP', 1.2) is not None
    assert policy.target('HOLD', 1.3) is None  # Paused: the rollout's chunks are not followed again,
    assert policy.wants('HOLD') == 'preview'  # and a new preview is asked for from where the arms hold.
    policy.asked(3, 1.3, 'preview')
    policy.receive(3, ramp(1.3), 1.5)
    assert positions(policy.target('ENGAGING', 1.6)) == [1.3, -1.3]
    assert policy.target('DISABLED', 2.) is None and policy.wants('HOLD') == 'preview'  # Disabled: all forgotten.


def test_answers_are_matched_to_their_request_and_checked():
    policy = source()
    policy.target('HOLD', 0.)
    assert not policy.receive(7, ramp(0.), .1)  # Nothing asked.
    policy.asked(1, 0., 'preview')
    assert not policy.receive(2, ramp(0.), .1)  # Another request's answer.
    with pytest.raises(ValueError, match=r'finite \(rows, 2\) chunk'):
        policy.receive(1, [[0., 1., 2.]], .1)
    policy.asked(2, .2, 'preview')
    with pytest.raises(ValueError, match='finite'):
        policy.receive(2, [[0., float('nan')]], .3)
    policy.asked(3, .4, 'preview')
    policy.failed(3, 'CUDA out of memory')
    assert policy.wants('HOLD') == 'preview' and policy.last_error == 'CUDA out of memory'
    policy.asked(4, .5, 'preview')  # A one-row chunk is a pose: held.
    assert policy.receive(4, np.array([.3, -.3]), .6)
    assert positions(policy.target('HOLD', .7)) == [.3, -.3]
    for bad in ({'action_dt': 0}, {'action_dt': .1, 'max_age_s': -1}, {'action_dt': .1, 'blend_s': float('nan')}):
        with pytest.raises(ValueError):
            PolicySource(('a',), **bad)
