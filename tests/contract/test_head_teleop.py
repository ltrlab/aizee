"""The head in a teleop session (helm.head_teleop): its own receiver (the native harness here) through the arms'
lifecycle, holding still at its zero without a headset, following a fresh headset pose, holding briefly when the
headset goes quiet, then returning to zero."""
import math
from pathlib import Path

import pytest

from robot_platform.helm.head_teleop import PERIOD_S, RETURN_AFTER_S, HeadTeleop
from robot_platform.neck import head
from robot_platform.simulation.control_harness import default_binary

ROOT = Path(__file__).resolve().parents[2]
pytestmark = [pytest.mark.integration, pytest.mark.skipif(not default_binary().is_file(),
                                                          reason='Build control_harness first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


@pytest.fixture
def teleop():
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    clock = Clock()
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference,
                       'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None}, live=False, clock=clock)
    core.build()
    yield core, clock
    core.close()


def run(core, clock, seconds, action=None, pose=None):
    """Cycle the head for `seconds` at the arms' 10 ms rate (the head cycles every PERIOD_S), the first cycle
    taking `action`, a headset `pose` (yaw, pitch) arriving each cycle when given."""
    for _ in range(int(seconds / .01)):
        if pose is not None:
            core.pose(*pose, now=clock.now)
        core.cycle(action, clock.now)
        action = None
        clock.now += .01


def positions(core):
    return {name: value for name, value in core.status()['positions'].items()}


def test_without_a_headset_the_head_holds_still_at_its_zero(teleop):
    core, clock = teleop
    zero = dict(core.zero)
    run(core, clock, .1, 'idle')
    run(core, clock, .1, 'enable')
    assert core.state == 'HOLD'
    run(core, clock, .5, 'engage')
    run(core, clock, 1.)
    assert core.state == 'TELEOP' and core.source == 'zero'
    assert positions(core) == pytest.approx(zero, abs=1e-6)
    assert core.status()['headset'] is None


def test_the_head_follows_a_fresh_headset_pose_then_holds_and_returns_to_zero(teleop):
    core, clock = teleop
    for action in ('idle', 'enable', 'engage'):
        run(core, clock, .2, action)
    run(core, clock, 2., pose=(.3, -.2))  # Look left and down.
    assert core.state == 'TELEOP' and core.source == 'headset'
    pan, tilt = (positions(core)[name] - core.zero[name] for name in ('head_pan', 'head_tilt'))
    assert pan == pytest.approx(.3, abs=.02) and tilt == pytest.approx(-.2, abs=.02)
    run(core, clock, .5)  # The headset goes quiet: the head holds where it was.
    assert core.source == 'holding' and positions(core)['head_pan'] - core.zero['head_pan'] == pytest.approx(.3, abs=.02)
    run(core, clock, RETURN_AFTER_S + 2.)  # Then it goes back to its zero, at the lifecycle's speed.
    assert core.source == 'zero' and positions(core) == pytest.approx(core.zero, abs=.01)
    run(core, clock, 2., pose=(math.radians(170), 0.))  # Past its range: clamped to the usable end.
    low, high = core.usable['head_pan']
    assert positions(core)['head_pan'] == pytest.approx(high, abs=.02)


def test_disable_leaves_the_head_limp_and_a_fault_stops_its_cycles(teleop):
    core, clock = teleop
    for action in ('idle', 'enable'):
        run(core, clock, .2, action)
    run(core, clock, .1, 'disable')
    assert core.state == 'DISABLED' and core.status()['motors_enabled'] is False
    core.fault = 'test'
    assert core.cycle('idle', clock.now) is None
