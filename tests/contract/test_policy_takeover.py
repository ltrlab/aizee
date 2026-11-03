"""Policy corrections on the virtual receiver: a policy session with takeover opens the leaders too. The policy drives;
the operator takes over (O, or pedal B through the loop worker) and the arms hold where they are; Resume aligns them
to the leaders and they follow; handing back holds again, previews the policy from there and aligns to it. Each frame
the capture records says who drove it: the policy, or a correction."""
from collections import deque
from types import SimpleNamespace

import pytest

from robot_platform.helm import teleop
from robot_platform.helm.teleop import TeleopCore
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.native_collision import library_path
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders  # noqa: F401
from tests.contract.test_policy_mode import ScriptedWorker, Sway, SWAY

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


def build(data, clock, leaders):
    sway, workers = Sway(), []

    def open_policy(spec, names, context, cameras=None):
        workers.append(ScriptedWorker(sway, names, context))
        return workers[-1]
    core = TeleopCore(dict(data, source={'kind': 'policy', 'policy': 'sway', 'takeover': True}),
                      open_leaders=lambda _profile: leaders, open_policy=open_policy, clock=clock)
    core.build()
    return core, sway


def state(core):
    return core.session.lifecycle.state


def run(core, clock, key=None, *, until=None, limit=1):
    """`limit` cycles, or until `until(core)`; every cycle fault-free."""
    for _ in range(limit):
        core.tick(key, clock())
        key = None
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if until is not None and until(core):
            return
    assert until is None, f'still {state(core)}: {core.message} ({core.input_error})'


def measured(core):
    last = core.harness.last
    names = last['status'].get('joint_names') or [joint['name'] for joint in core.derived['config']['joints']]
    return {name: row['position'] for name, row in zip(names, last['feedback'])}


def test_take_over_with_the_leaders_then_hand_back_to_the_policy(bundle):
    data, _profile = bundle
    clock = Clock()
    leaders = Leaders(clock)
    core, sway = build(data, clock, leaders)
    names = [joint['name'] for joint in core.derived['config']['joints']]
    try:
        assert core.leaders is leaders and core.driver == 'policy'
        assert core.grip_triggers == {}  # No force feedback in a policy session.
        run(core, clock, 'I', until=lambda c: state(c) == 'IDLE', limit=50)
        run(core, clock, 'E', until=lambda c: state(c) == 'HOLD', limit=50)
        run(core, clock, until=lambda c: c.source.aligned is not None, limit=20)
        run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP', limit=2000)
        run(core, clock, limit=50)
        assert core._capture_label() == {'authority': 'policy', 'lifecycle': 'TELEOP'}
        view = core.status_view()
        assert view['policy']['takeover'] and view['policy']['driver'] == 'policy'
        assert view['pedals']['actions']['b']['action'] == 'take_over'

        # Take over: the arms hold where the policy left them, and the policy is asked nothing more.
        run(core, clock, 'O')
        assert state(core) == 'HOLD' and core.driver == 'leaders' and 'You drive next' in core.message
        assert core._capture_label() == {'authority': 'correction', 'lifecycle': 'HOLD'}
        assert core.source.chunk is None and core.source.aligned is None
        asked, held = core.source.requests, measured(core)
        run(core, clock, limit=30)
        assert core.source.requests == asked
        assert max(abs(measured(core)[n] - held[n]) for n in names) < .01

        # Resume aligns the arms to the leaders (at their zero: the reference pose), then they follow them.
        run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP', limit=2000)
        assert core._capture_label() == {'authority': 'correction', 'lifecycle': 'TELEOP'}
        assert core.status_view()['pedals']['actions']['b']['action'] == 'hand_back'
        reference = {row['name']: row['position'] for row in core.last_target}
        for _ in range(20):  # Raise the left leader's first joint 0.2 rad at 1 rad/s: a correction.
            leaders.offsets['left'][0] += .01
            run(core, clock)
        run(core, clock, limit=100)
        target = {row['name']: row['position'] for row in core.last_target}
        assert abs(target['left_arm_j1'] - reference['left_arm_j1']) > .05  # The leader moved its joint...
        assert all(abs(target[n] - reference[n]) < 1e-6 for n in names if n != 'left_arm_j1')  # ...and only it.
        assert max(abs(measured(core)[n] - target[n]) for n in names) < .02  # The arms followed the leaders.
        assert core.source.requests == asked

        # Hand back: hold again, preview the policy from where the arms are, align to it, and the policy drives.
        run(core, clock, 'O')
        assert state(core) == 'HOLD' and core.driver == 'policy' and 'The policy drives next' in core.message
        assert core._capture_label() == {'authority': 'policy', 'lifecycle': 'HOLD'}
        run(core, clock, until=lambda c: c.source.aligned is not None, limit=20)
        assert core.source.requests == asked + 1  # One preview, from the hold.
        run(core, clock, 'T', until=lambda c: state(c) == 'TELEOP', limit=2000)
        run(core, clock, limit=50)
        errors = []
        for _ in range(10):
            run(core, clock, limit=10)
            expected = dict(zip(names, sway.at(clock.now)))
            errors.append(max(abs(measured(core)[n] - expected[n]) for n in SWAY))
        assert max(errors) < .05 and core._capture_label()['authority'] == 'policy'

        # Return lowers the arms as in teleop. Disabled, O only chooses who drives once the arms are engaged again.
        run(core, clock, 'X', until=lambda c: state(c) == 'DISABLED', limit=4000)
        run(core, clock, 'O')
        assert core.driver == 'leaders' and core.message.startswith('You drive: Engage (T)')
    finally:
        core.close()


def test_a_policy_session_without_takeover_opens_no_leaders_and_refuses_o(bundle):
    data, _profile = bundle
    clock = Clock()
    workers = []

    def open_policy(spec, names, context, cameras=None):
        workers.append(ScriptedWorker(Sway(), names, context))
        return workers[-1]
    core = TeleopCore(dict(data, source={'kind': 'policy', 'policy': 'sway'}), clock=clock,
                      open_leaders=lambda _profile: pytest.fail('a policy without takeover opened the leaders'),
                      open_policy=open_policy)
    core.build()
    try:
        assert core.driver is None and core.leaders is None
        run(core, clock, 'I', until=lambda c: state(c) == 'IDLE', limit=50)
        run(core, clock, 'O')
        assert core.driver is None and 'Take over in a policy session' in core.message
        assert core.status_view()['policy'].get('takeover') is None
    finally:
        core.close()


def test_pedal_b_takes_over_and_hands_back_through_the_loop_worker(bundle, monkeypatch):
    data, _profile = bundle
    clock = Clock()
    leaders = Leaders(clock)
    core, _sway = build(data, clock, leaders)
    core.build = lambda: None
    ticks, sent, queue, labels = [], [], deque(), []
    stage, identity, last_submit = 0, 0, -1
    original = core.tick

    def tick(key, started):
        original(key, started)
        ticks.append((state(core), key, core.driver, core.pedals.paused))
        assert core.fault is None, core.message
        assert len(ticks) < 6000, f'Pedal workflow stuck at stage {stage}'
    core.tick = tick

    def add(pedal):
        nonlocal identity
        view = core.status_view()['pedals']
        identity += 1
        labels.append(view['actions'][pedal]['label'])
        queue.append(('pedal', identity, dict(session_id=view['session_id'], press_id=identity, pedal=pedal,
                                              revision=view['actions'][pedal]['revision'])))

    class Commands:
        def poll(self):
            nonlocal stage, last_submit
            now, aligned = state(core), core.source.aligned is not None
            if queue:
                return True
            if last_submit == len(ticks):
                return False
            if stage == 0:
                add('c')
                stage = 1
            elif stage == 1 and now == 'IDLE':
                add('c')
                stage = 2
            elif stage == 2 and now == 'HOLD' and aligned:
                add('c')  # Engage: align to the policy's first action.
                stage = 3
            elif stage == 3 and now == 'TELEOP' and len(ticks) > 300:
                add('b')  # Take over.
                stage = 4
            elif stage == 4 and now == 'HOLD' and core.pedals.paused:
                add('b')  # Resume: you drive.
                stage = 5
            elif stage == 5 and now == 'TELEOP' and core.driver == 'leaders':
                add('b')  # Hand back.
                stage = 6
            elif stage == 6 and now == 'HOLD' and core.pedals.paused and aligned:
                add('b')  # Resume: the policy drives.
                stage = 7
            elif stage == 7 and now == 'TELEOP' and core.driver == 'policy':
                add('c')  # Shutdown to the reference.
                stage = 8
            elif stage == 8 and now == 'DISABLED':
                queue.append(('stop',))
                stage = 9
            if queue:
                last_submit = len(ticks)
            return bool(queue)

        def recv(self):
            return queue.popleft()

        def close(self):
            pass
    monkeypatch.setattr(teleop, 'time', SimpleNamespace(time=clock, sleep=lambda s: setattr(clock, 'now', clock.now + s)))
    try:
        teleop.run_loop(Commands(), SimpleNamespace(send=sent.append, close=lambda: None), core, silence_s=None)
    finally:
        core.close()
    assert stage == 9
    assert labels == ['Idle', 'Enable / Hold', 'Engage / Align', 'Take over', 'Resume: you drive',
                      'Hand back to policy', 'Resume: policy drives', 'Shutdown to reference']
    replies = {m[1]: m[2:] for m in sent if m[0] == 'reply'}
    assert all(replies[i][0] is None for i in range(1, identity + 1)), replies
    switches = [(s, k, d) for s, k, d, _ in ticks if k == 'O']
    assert switches == [('HOLD', 'O', 'leaders'), ('HOLD', 'O', 'policy')]  # Each held as it switched.
