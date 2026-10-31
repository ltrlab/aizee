"""The head in a teleop session with the arms: one TeleopCore, the arms' receiver and the head's (both the native
harness), the same keys for both. Without a headset the head holds at its zero; a headset pose steers it while the
arms follow the leaders; a fault in the head holds the arms."""
import time

import pytest

from robot_platform.helm.teleop import TeleopCore
from robot_platform.neck import head
from robot_platform.simulation.control_harness import NativeControlHarness
from tests.contract.test_direct_teleop import direct_profile
from tests.contract.test_helm_teleop_live import ROOT, Clock, Leaders, bundle, run  # noqa: F401


def head_bundle():
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    return {'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference,
            'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None}


@pytest.fixture
def core(bundle):
    data, path = bundle
    direct_profile(path, 100)
    clock = Clock()
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(clock), clock=clock)
    core.build()
    yield core, clock
    core.close()


def test_the_head_takes_the_arms_keys_and_holds_still_without_a_headset(core):
    core, clock = core
    assert core.head.state == 'DISABLED'
    run(core, clock, 'I')
    run(core, clock, 'E')
    assert (core.session.lifecycle.state, core.head.state) == ('HOLD', 'HOLD')
    run(core, clock, 'T', until='TELEOP', limit=1000)
    run(core, clock, limit=100)
    assert core.head.state == 'TELEOP' and core.head.source == 'zero'
    view = core.view()['head']
    assert view['state'] == 'TELEOP' and view['headset'] is None
    assert view['positions'] == pytest.approx(core.head.zero, abs=1e-6)


def test_a_headset_pose_steers_the_head_while_the_arms_follow_the_leaders(core):
    core, clock = core
    for key in ('I', 'E'):
        run(core, clock, key)
    run(core, clock, 'T', until='TELEOP', limit=1000)
    run(core, clock, limit=100)
    for _ in range(300):
        core.head_pose(.25, .1)
        run(core, clock)
    assert core.head.source == 'headset'
    pan = core.view()['head']['positions']['head_pan'] - core.head.zero['head_pan']
    assert pan == pytest.approx(.25, abs=.02)
    assert core.view()['head']['joints'] == pytest.approx({'neck_z': .25, 'neck_y': .1}, abs=.02)  # Helm's 3D view.
    assert core.session.lifecycle.state == 'TELEOP'  # The arms follow on.
    run(core, clock, 'X', until='DISABLED', limit=3000)
    run(core, clock, limit=300)  # The head returns to its zero and is disabled with the arms.
    assert core.head.state == 'DISABLED'


def test_a_head_fault_holds_the_arms(core):
    core, clock = core
    for key in ('I', 'E'):
        run(core, clock, key)
    run(core, clock, 'T', until='TELEOP', limit=1000)
    core.head.receiver.close()  # The head's receiver goes away mid-session.
    run(core, clock, limit=20)
    assert core.head.fault and core.session.lifecycle.state == 'HOLD'
    assert 'The arms hold' in core.message
    run(core, clock, 'T', until='TELEOP', limit=1000)  # The arms follow again, without the head.


class Ticking:
    """The head's harness answering each request 10 ms after it was sent, as the live head service does at its
    ticks."""

    def __init__(self, config, initial):
        self.inner, self.requests = NativeControlHarness(config, initial), 0
        self.close = self.inner.close

    now_ms = property(lambda self: self.inner.now_ms)
    last = property(lambda self: self.inner.last)

    def step(self, now_ms, request=None):
        if request is not None:
            self.requests += 1
            time.sleep(.01)
        return self.inner.step(max(now_ms, self.inner.now_ms), request)


def test_a_live_head_cycles_on_a_thread_of_its_own_so_the_arms_never_wait_for_it(bundle):
    # Live, each exchange with the head's service waits for its next 10 ms tick: cycled in the arms' loop, the head
    # (two exchanges a cycle) took 22 ms of their 10 ms period every 40 ms, their commands went out 10 and 30 ms
    # apart, each 30 ms long, and Enable faulted ("command_expired · wrong_owner", 2026-10-31). A live head cycles
    # on a thread of its own: here, in real time, its receiver answers as the service does, and the arms' ticks
    # still take only their own few ms.
    data, path = bundle
    direct_profile(path, 100)
    clock = time.perf_counter
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(clock), clock=clock)
    core.head.open_harness = Ticking
    core.head_thread = True  # As a live session's.
    core.build()
    try:
        durations = []
        for key in ['I'] + [None] * 99:  # A second of the arms' 10 ms ticks.
            started = clock()
            core.tick(key, started)
            durations.append(clock() - started)
            time.sleep(max(0., started + core.period_s - clock()))
        assert core.fault is None and core.head.fault is None
        assert (core.session.lifecycle.state, core.head.state) == ('IDLE', 'IDLE')
        assert core.head.receiver.requests > 15  # The head cycled on its own grid meanwhile.
        # In the arms' loop, every fourth tick waited 20 ms or more for the head; beside it, none does (but for an
        # odd stall of the machine running this).
        assert sum(duration > .018 for duration in durations) <= 2, durations
    finally:
        core.close()


def test_a_fault_on_the_heads_own_thread_holds_the_arms(bundle, monkeypatch):
    # A fault ends the live head's thread; the arms' next tick holds the arms, as after a failed inline head cycle.
    # The thread stands in here: each post cycles the head at once, on this test's clock, its fault kept on its side.
    from robot_platform.helm import head_teleop

    class Beside:
        def __init__(self, head, *, clock):
            self.head, self.clock, self.pending = head, clock, False

        def start(self):
            return self

        def post(self, action):
            try:
                self.head.cycle(action, self.clock())
            except Exception:
                pass

        def stop(self):
            pass
    monkeypatch.setattr(head_teleop, 'HeadLoop', Beside)
    data, path = bundle
    direct_profile(path, 100)
    clock = Clock()
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(clock), clock=clock)
    core.head_thread = True
    core.build()
    try:
        assert isinstance(core._head_loop, Beside)
        for key in ('I', 'E'):
            run(core, clock, key)
        run(core, clock, 'T', until='TELEOP', limit=1000)
        core.head.receiver.close()  # The head's receiver goes away mid-session.
        run(core, clock, limit=20)
        assert core.head.fault and core.session.lifecycle.state == 'HOLD'
        assert 'The arms hold' in core.message
    finally:
        core.close()
