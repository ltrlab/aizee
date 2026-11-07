"""Live · Head (helm.head_only): the head alone under the session's keys and pedals, steered by the headset, with none
of the arms' motors. Here on the virtual head receiver (the native harness), a stepped clock and the loop's period."""

import itertools
from pathlib import Path

import pytest

from robot_platform.helm.head_only import HeadOnlyCore
from robot_platform.helm.head_teleop import PERIOD_S, HeadTeleop
from robot_platform.neck import head
from robot_platform.simulation.control_harness import default_binary

ROOT = Path(__file__).resolve().parents[2]
pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(not default_binary().is_file(), reason='Build control_harness first'),
]
KEYS = {'idle': 'I', 'enable': 'E', 'engage': 'T', 'pause': 'T', 'resume': 'T', 'shutdown': 'X'}  # run_loop's map.


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def bundle(mode='virtual'):
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    return {
        'receiver': {'mode': mode, 'arm': 'head'},
        'head': {
            'teleop_profile': str(ROOT / head.PROFILE),
            'head_reference': reference,
            'limits': head.virtual_limits(reference, span_rad=0.8),
            'servo': None,
        },
    }


@pytest.fixture
def rig():
    clock = Clock()
    core = HeadOnlyCore(bundle(), clock=clock)
    core.build()
    yield core, clock
    core.close()


def run(core, clock, seconds, key=None, pose=None):
    """The loop's cycles for `seconds`, one each period, the first taking `key`; a headset pose (yaw, pitch) arriving
    before each cycle when given."""
    for _ in range(max(1, round(seconds / PERIOD_S))):
        if pose is not None:
            core.head_pose(*pose)
        core.tick(key, clock.now)
        key = None
        clock.now += PERIOD_S


def until(core, clock, state, seconds=10.0):
    for _ in range(round(seconds / PERIOD_S)):
        if core.head.state == state:
            return
        run(core, clock, PERIOD_S)
    pytest.fail(f'Expected {state}: {core.message}')


def test_the_keys_drive_the_head_alone_and_it_follows_the_headset(rig):
    core, clock = rig
    assert core.phase == 'running' and core.view()['live'] is None and core.guide() is None
    run(core, clock, 0.2, 'I')
    assert core.head.state == 'IDLE' and 'Enable (E) holds the head' in core.message
    run(core, clock, 0.2, 'E')
    assert core.head.state == 'HOLD'
    run(core, clock, 0.4, 'T', pose=(0.3, -0.2))  # Look left and down.
    run(core, clock, 2.0, pose=(0.3, -0.2))
    view = core.view()
    assert view['state'] == 'TELEOP' and view['head']['source'] == 'headset' and view['fault'] is None
    # The neck's CAD angles (the 3D view's) are the headset's yaw and pitch.
    assert view['head']['joints'] == pytest.approx({'neck_z': 0.3, 'neck_y': -0.2}, abs=0.02)
    assert view['head_only'] is True and view['joints'] == [] and view['cycle_ms']['p50'] >= 0
    run(core, clock, 0.2, 'T', pose=(0.3, -0.2))  # T again holds it there.
    assert core.head.state == 'HOLD'
    run(core, clock, 0.1, 'X')  # Return: back to its zero, then disabled.
    until(core, clock, 'DISABLED')
    assert core.head.status()['positions'] == pytest.approx(core.head.zero, abs=0.01)
    assert core.fault is None


def test_a_key_not_available_is_said_never_sent(rig):
    core, clock = rig
    run(core, clock, 0.1, 'E')  # Enable before Idle: the lifecycle would refuse it, and that faults the head.
    assert core.fault is None and core.head.state == 'DISABLED' and 'Not available from DISABLED' in core.message


def test_closing_returns_a_powered_head_to_its_zero_before_its_receiver_closes(rig):
    core, clock = rig
    for key in 'IET':
        run(core, clock, 0.2, key, pose=(0.2, 0.1))
    run(core, clock, 1.0, pose=(0.2, 0.1))
    assert core.head.state == 'TELEOP' and core.land() is True and core.landing
    until(core, clock, 'DISABLED')
    assert not core.landing and core.fault is None
    assert core.head.status()['positions'] == pytest.approx(core.head.zero, abs=0.01)


def test_an_unpowered_head_closes_at_once():
    clock = Clock()
    core = HeadOnlyCore(bundle(), clock=clock)
    core.build()
    try:
        run(core, clock, 0.2, 'I')
        assert core.land() is False and not core.landing  # Idle: torque off, nothing to return.
    finally:
        core.close()
    assert core.phase == 'stopped' and 'closed' in core.message


def test_a_live_head_disabled_after_it_was_powered_needs_a_new_session():
    clock = Clock()
    core = HeadOnlyCore(bundle('live'), clock=clock, open_head=lambda b: HeadTeleop(b, live=False, clock=clock))
    core.build()
    try:
        for key in 'IEH':
            run(core, clock, 0.2, key)
        assert core.head.state == 'DISABLED' and 'a new one powers it again' in core.message
        run(core, clock, 0.2, 'I')
        assert core.head.state == 'DISABLED' and 'Stop the session and start a new one' in core.message
        assert core.view()['live']['arm'] == 'head' and core.view()['pedals']['actions']['c']['enabled'] is False
    finally:
        core.close()


def test_a_head_fault_holds_every_key_but_disable(rig, monkeypatch):
    core, clock = rig
    for key in 'IE':
        run(core, clock, 0.2, key)

    def broken(action, started):
        core.head.fault = 'Head: the receiver stopped answering'
        raise RuntimeError('the receiver stopped answering')

    monkeypatch.setattr(core.head, 'cycle', broken)
    run(core, clock, PERIOD_S)
    assert core.fault == 'Head: the receiver stopped answering' and 'FAULT' in core.message
    assert core.land() is False and not core.landing  # Nothing returns a faulted head: H, or the session's end.
    disabled = []
    monkeypatch.setattr(core.head.session, 'disable_now', lambda: disabled.append(True))
    run(core, clock, PERIOD_S, 'T')
    assert not disabled
    run(core, clock, PERIOD_S, 'H')
    assert disabled == [True] and 'Disabled at your request' in core.message


def test_the_pedals_step_the_head_and_record_nothing(rig):
    core, clock = rig
    presses = itertools.count(1)

    def press(pedal):
        offer = core.view()['pedals']
        intent = core.prepare_pedal(
            {
                'session_id': offer['session_id'],
                'pedal': pedal,
                'press_id': next(presses),
                'revision': offer['actions'][pedal]['revision'],
            }
        )
        run(core, clock, PERIOD_S, KEYS[intent['action']], pose=(0.1, 0.0))
        result = core.finish_pedal(intent)
        assert result['accepted'], result
        return intent['action']

    assert core.view()['pedals']['actions']['a']['enabled'] is False  # No task: nothing to record.
    assert [press('c'), press('c'), press('c')] == ['idle', 'enable', 'engage']
    run(core, clock, 1.0, pose=(0.1, 0.0))
    assert core.head.state == 'TELEOP'
    assert press('b') == 'pause' and core.head.state == 'HOLD'
    assert press('b') == 'resume' and core.head.state in ('ENGAGING', 'TELEOP')
    run(core, clock, 1.0, pose=(0.1, 0.0))
    assert press('c') == 'shutdown'
    until(core, clock, 'DISABLED')


def test_a_head_only_session_needs_the_head_set_up_and_the_head_receiver():
    with pytest.raises(ValueError, match='Limits → Head'):
        HeadOnlyCore({'receiver': {'mode': 'live', 'arm': 'head'}, 'head': None})
    with pytest.raises(ValueError, match='head alone'):
        HeadOnlyCore(dict(bundle(), receiver={'mode': 'live', 'arm': 'both'}))
    core = HeadOnlyCore(bundle())
    for method, arguments in (
        ('task_action', ('start_record',)),
        ('route', ('accept', 'head_pan')),
        ('set_zero', ()),
        ('set_grip_feedback', ({},)),
        ('queue_tuning', ({},)),
    ):
        with pytest.raises(ValueError):
            getattr(core, method)(*arguments)
