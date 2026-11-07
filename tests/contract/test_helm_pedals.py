"""Pedal worker dispatch against the actual native fourteen-axis virtual receiver.

Deterministic clock and scripted leaders: no USB, CAN, physical arm, or timing
qualification. Capture commands use an explicit asynchronous in-memory boundary.
"""

from collections import deque
import math
from types import SimpleNamespace

import pytest

from robot_platform.helm import teleop
from robot_platform.simulation.control_harness import default_binary
from tests.contract.test_direct_teleop import direct_profile
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders, run  # noqa: F401

pytestmark = pytest.mark.integration


class Capture:
    def __init__(self):
        self.active = False
        self.pending = None
        self.identity = 0
        self.last = None
        self.commands = []

    def command(self, action):
        self.identity += 1
        self.pending = (self.identity, action, 5)
        self.last = dict(id=self.identity, action=action, state='pending')
        self.commands.append(action)
        return dict(id=self.identity, accepted=True)

    def tick(self):
        if self.pending:
            identity, action, left = self.pending
            self.pending = (identity, action, left - 1) if left else None
            if not left:
                self.active = action == 'start_record'
                self.last = dict(id=identity, action=action, state='complete')

    def health(self):
        return dict(
            available=True,
            capture_ready=True,
            failed=False,
            error=None,
            recording=dict(
                active=self.active,
                state='recording' if self.active else 'idle',
                complete=bool(self.last and self.last['action'] == 'stop_record' and not self.pending),
            ),
            last_command=self.last,
        )

    snapshot = health


@pytest.mark.parametrize('mode', ['direct', 'motion'])
def test_worker_pedals_run_both_arms_through_pause_resume_and_return(bundle, monkeypatch, mode):
    assert default_binary().is_file(), 'Build the native control harness for this required contract'
    data, path = bundle
    if mode == 'direct':
        direct_profile(path)
    clock = Clock()
    leaders = Leaders(clock)
    receiver = {'mode': 'virtual'} if mode == 'direct' else {'mode': 'virtual', 'collision': 'motion'}
    core = teleop.TeleopCore(dict(data, receiver=receiver), open_leaders=lambda _: leaders, clock=clock)
    core.build()
    core.build = lambda: None
    assert len(core.session.lifecycle.names) == 14
    capture = core.task_capture = Capture()
    ticks = []
    sent = []
    requests = []
    queue = deque()
    stage = 0
    identity = 0
    last_submit = -1
    original = core.tick

    def tick(key, started):
        original(key, started)
        capture.tick()
        ticks.append((core.session.lifecycle.state, key, core.pedals.paused))
        assert core.fault is None, core.message
        assert len(ticks) < 2000, 'Pedal workflow did not finish'

    core.tick = tick

    def add(pedal):
        nonlocal identity
        view = core.status_view()['pedals']
        identity += 1
        payload = dict(
            session_id=view['session_id'], revision=view['actions'][pedal]['revision'], press_id=identity, pedal=pedal
        )
        requests.append(payload)
        queue.append(('pedal', identity, payload))

    class Commands:
        def poll(self):
            nonlocal stage, last_submit
            state = core.session.lifecycle.state
            if queue:
                return True
            if last_submit == len(ticks):
                return False
            if stage == 0:
                add('c')
                stage = 1
            elif stage == 1 and state == 'IDLE':
                queue.append(('pedal', 1001, requests[0]))  # Lost reply retry must not enable.
                stage = 2
            elif stage == 2 and ticks and ticks[-1][0] == 'IDLE':
                add('c')
                stage = 3
            elif stage == 3 and state == 'HOLD':
                add('c')
                stage = 4
            elif stage == 4 and state == 'ENGAGING':
                add('b')
                stage = 5
            elif stage == 5 and state == 'HOLD' and core.pedals.paused:
                add('b')
                stage = 6
            elif stage == 6 and state == 'TELEOP':
                add('a')
                stage = 7
            elif stage == 7 and capture.pending:
                add('b')
                stage = 8  # Pause is allowed while recording's ACK is pending.
            elif stage == 8 and state == 'HOLD' and capture.active and not capture.pending:
                add('a')
                stage = 9
            elif stage == 9 and capture.pending:
                add('b')
                stage = 10  # Resume while writer finalizes, without changing recording.
            elif stage == 10 and state == 'TELEOP' and not capture.pending:
                add('c')
                stage = 11
            elif stage == 11 and state == 'DISABLED':
                queue.append(('stop',))
                stage = 12
            if queue:
                last_submit = len(ticks)
            return bool(queue)

        def recv(self):
            return queue.popleft()

        def close(self):
            pass

    monkeypatch.setattr(
        teleop, 'time', SimpleNamespace(time=clock, sleep=lambda s: setattr(clock, 'now', clock.now + s))
    )
    try:
        teleop.run_loop(Commands(), SimpleNamespace(send=sent.append, close=lambda: None), core, silence_s=None)
    finally:
        core.close()
    assert stage == 12
    assert capture.commands == ['start_record', 'stop_record']
    assert any(state == 'ENGAGING' and key == 'T' for state, key, _ in ticks)
    assert sum(state == 'HOLD' and key == 'T' for state, key, _ in ticks) == 2
    assert any(state == 'SHUTDOWN' and key == 'X' for state, key, _ in ticks)
    assert core.session.lifecycle.state == 'DISABLED'
    replies = {m[1]: m[2:] for m in sent if m[0] == 'reply'}
    assert 'Duplicate' in replies[1001][0]
    assert all(replies[i][0] is None for i in range(1, identity + 1)), replies


@pytest.mark.parametrize('key', ['H', 'X'])
def test_native_immediate_control_cannot_be_replaced_by_queued_zero_hold(bundle, key):
    data, path = bundle
    direct_profile(path)
    clock = Clock()
    leaders = Leaders(clock)
    core = teleop.TeleopCore(dict(data, receiver={'mode': 'virtual'}), open_leaders=lambda _: leaders, clock=clock)
    try:
        core.build()
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=500)
        stale = core.status_view()['pedals']
        core.set_zero()
        assert core._hold
        run(core, clock, key)
        assert core.session.lifecycle.state == ('DISABLED' if key == 'H' else 'SHUTDOWN')
        with pytest.raises(ValueError, match='stale'):
            core.prepare_pedal(
                dict(session_id=stale['session_id'], revision=stale['actions']['b']['revision'], press_id=1, pedal='b')
            )
        assert not core.status_view()['pedals']['actions']['b']['enabled']
        if key == 'X':
            seconds = (
                max(
                    abs(q - j['zero_position']) / j['shutdown_speed']
                    for q, j in zip(core.session.lifecycle.target, core.session.lifecycle.joints)
                )
                + 2
            )
            run(core, clock, until='DISABLED', limit=math.ceil(seconds / core.period_s))
    finally:
        core.close()
