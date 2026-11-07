"""The harness clips a policy's commands into what the direct receiver accepts."""

import pytest

from robot_platform.evaluation.plants import _Stack, derive_stack


def test_commands_are_clipped_inside_the_receivers_stopping_margin():
    """The direct receiver refuses an arm joint's command within the stopping margin of either end of its range
    (it aborted a trial at right J5 3.2348 against 3.2585 - 0.0239); the harness clips into what it accepts."""
    from robot_platform.control.approach import end_margin

    stack = derive_stack()
    limits = _Stack(stack)
    config = {joint['name']: joint for joint in stack['derived']['config']['joints']}
    rows = {row['name']: row for row in stack['derived']['lifecycle']}
    reaction = (
        stack['derived']['config']['feedback_timeout_ms'] + stack['derived']['config']['direct']['limit_reaction_ms']
    ) / 1000
    for i, name in enumerate(limits.names):
        margin = 0.0 if config[name].get('gripper') else end_margin(config[name], reaction)
        assert limits.lower[i] == pytest.approx(rows[name]['lower'] + margin)
        assert limits.upper[i] == pytest.approx(rows[name]['upper'] - margin)
    j5 = limits.names.index('right_arm_j5')
    command = limits.home.copy()
    command[j5] = rows['right_arm_j5']['upper'] - 0.001
    assert limits.clip(command)[j5] < rows['right_arm_j5']['upper'] - 0.02
