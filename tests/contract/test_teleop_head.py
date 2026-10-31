"""The head in a teleop session with the arms: one TeleopCore, the arms' receiver and the head's (both the native
harness), the same keys for both. Without a headset the head holds at its zero; a headset pose steers it while the
arms follow the leaders; a fault in the head holds the arms."""
import os
import time

import pytest

from robot_platform.helm.teleop import TeleopCore
from robot_platform.neck import head
from robot_platform.simulation.control_harness import NativeControlHarness
from tests.contract.test_direct_teleop import direct_profile
from tests.contract.test_helm_teleop_live import ROOT, Clock, Leaders, Receiver, bundle, run  # noqa: F401


def head_bundle():
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    return {'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference,
            'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None}


def drooped_head():
    """As on Minerva: its shoulder clearance saved, and the head resting limp below the lowest tilt its receiver
    takes."""
    from robot_platform.neck.clearance import section
    data = head_bundle()
    centre = {m['name']: m['raw_reference_rad'] for m in data['head_reference']['motors']}
    data['limits'] = dict(data['limits'], reference_id=data['head_reference']['id'],
                          clearance=section([{'pan': -.6, 'tilt': -.4}, {'pan': .6, 'tilt': -.4}]))
    return dict(data, rest_pose={'head_tilt': centre['head_tilt'] - .8})


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
    ticks. Module-level, so a head process (helm.head_teleop.HeadProcess) builds it in its own interpreter."""

    def __init__(self, config, initial):
        self.inner, self.requests = NativeControlHarness(config, initial), 0
        self.close = self.inner.close

    now_ms = property(lambda self: self.inner.now_ms)
    last = property(lambda self: self.inner.last)

    def step(self, now_ms, request=None):
        if request is not None:
            self.requests += 1
            self.wait()
        return self.inner.step(max(now_ms, self.inner.now_ms), request)

    def wait(self):
        time.sleep(.01)


class Greedy(Ticking):
    """A head holding its interpreter for 8 ms of each 10 ms answer, as a thread stalled (or descheduled) while it
    holds the interpreter does: in the arms' process, the arms' loop would wait each time."""

    def wait(self):
        end = time.perf_counter() + .008
        while time.perf_counter() < end:
            pass
        time.sleep(.002)


class Failing(Ticking):
    """A head whose receiver goes away mid-session: once the file HEAD_FAIL_FLAG names exists (the test's process
    and the head's share the environment, not their objects)."""

    def step(self, now_ms, request=None):
        if request is not None and os.path.exists(os.environ['HEAD_FAIL_FLAG']):
            raise RuntimeError('the head receiver went away')
        return super().step(now_ms, request)


def live(core, key=None, *, until=None, limit=1):
    """The arms' ticks in real time, as run_loop gives them: a head in its own process cycles on the wall clock."""
    for _ in range(limit):
        started = time.perf_counter()
        core.tick(key, started)
        key = None
        assert core.fault is None, core.fault
        if until is not None and until(core):
            return
        time.sleep(max(0., started + core.period_s - time.perf_counter()))
    assert until is None, f'still {core.session.lifecycle.state}: {core.message}'


def test_a_live_head_cycles_in_a_process_of_its_own_so_the_arms_never_wait_for_it(bundle):
    # Cycled in the arms' loop, the live head (each exchange waits for its service's 10 ms tick) took 22 ms of their
    # 10 ms period every 40 ms, and Enable faulted. On a thread beside them it shared their interpreter: their slow
    # cycles kept time with its 40 ms cycle (R 0.7) and one command expired (2026-10-31, live-20261031T195313Z,
    # "command_expired · wrong_owner"). In a process of its own, even a head that holds its interpreter for most of
    # each answer leaves the arms' ticks their own few ms.
    from robot_platform.helm.head_teleop import HeadProcess
    data, path = bundle
    direct_profile(path, 100)
    clock = time.perf_counter
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(clock), clock=clock)
    core.head.open_harness = Greedy
    core.head_process = True  # As a live session's.
    core.build()
    try:
        assert isinstance(core.head, HeadProcess) and core.head.session and core._head_loop is core.head
        durations = []
        for key in ['I'] + [None] * 199:  # Two seconds of the arms' 10 ms ticks.
            started = clock()
            core.tick(key, started)
            durations.append(clock() - started)
            time.sleep(max(0., started + core.period_s - clock()))
        assert core.fault is None and core.head.fault is None
        assert (core.session.lifecycle.state, core.head.state) == ('IDLE', 'IDLE')
        assert core.head.status()['positions']  # Its cycles' word, from its process.
        assert sum(duration > .008 for duration in durations) <= 3, sorted(durations)[-5:]
    finally:
        core.close()
    assert not core.head._process.is_alive()  # Closed with the session.


def test_beside_a_live_head_the_interpreter_changes_hands_within_a_millisecond(bundle):
    # With the head's thread, the arms' exchanges took up to 10.6 ms (2-3 ms alone): CPython hands its interpreter
    # between threads after 5 ms. The arms' process still runs threads beside their loop (the views' sender, the
    # recorder, the head process's proxy): beside a live head it hands over after 1 ms, until the session closes.
    import sys
    data, path = bundle
    direct_profile(path, 100)
    before = sys.getswitchinterval()
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(time.perf_counter))
    core.head_process = True  # As a live session's.
    core.build()
    try:
        assert sys.getswitchinterval() == pytest.approx(.001)
    finally:
        core.close()
    assert sys.getswitchinterval() == pytest.approx(before)
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(time.perf_counter))
    core.build()  # Its head cycles on the arms' ticks: nothing changes.
    try:
        assert sys.getswitchinterval() == pytest.approx(before)
    finally:
        core.close()


def test_the_heads_process_moves_itself_off_the_arms_cpus(monkeypatch):
    # On the arms' CPUs, beside the head, the arms' receiver started its ticks up to 9.2 ms late (2026-10-31, 15:15).
    # The head's process moves itself (control.scheduling.prefer_background, every thread of it) before it builds
    # the head, so its receiver's service starts there too, and says so to the arms' side.
    import os
    from robot_platform.control import scheduling
    from robot_platform.helm.head_teleop import serve_head
    moved = []
    monkeypatch.setattr(scheduling, 'prefer_background', lambda pid=None: moved.append(pid) or ['background_cpus'])

    class Conn:
        def __init__(self):
            self.sent = []

        def send(self, message):
            self.sent.append(message)

        def poll(self, timeout):
            return True

        def recv(self):
            return ('close',)
    conn = Conn()
    serve_head(head_bundle(), False, conn)
    assert moved == [os.getpid()]
    assert conn.sent[0] == ('built', ['background_cpus']) and conn.sent[-1] == ('closed', None)


def test_the_heads_process_loads_the_head_model_by_the_releases_digests(monkeypatch):
    # On the robot there are no CAD files: a process loads the model by the digests its verified release vouches for
    # (ops.release.activate). The head's process got none and failed to build in its rehearsal on Minerva
    # (2026-10-31): it registers those the arms' process holds.
    from robot_platform.helm import head_teleop
    from robot_platform.models import urdf
    registered = []
    monkeypatch.setattr(urdf, 'register_verified_assets', lambda digests: registered.append(dict(digests)))
    monkeypatch.setattr(urdf, '_VERIFIED_ASSETS', {ROOT / 'urdf/part.stl': 'abc'})
    assert urdf.verified_assets() == {str(ROOT / 'urdf/part.stl'): 'abc'}

    class Conn:
        sent = []

        def send(self, message):
            self.sent.append(message)

        def poll(self, timeout):
            return True

        def recv(self):
            return ('close',)
    head_teleop.serve_head(head_bundle(), False, Conn(), verified={'urdf/part.stl': 'abc'})
    assert registered == [{'urdf/part.stl': 'abc'}]
    head_teleop.serve_head(head_bundle(), False, Conn())  # A source checkout: none to register.
    assert len(registered) == 1


def test_a_fault_in_the_heads_own_process_holds_the_arms(bundle, tmp_path, monkeypatch):
    # A fault ends the head's cycles in its process; the arms' next tick after its word holds the arms, as after a
    # failed inline head cycle.
    flag = tmp_path / 'head-fails'
    monkeypatch.setenv('HEAD_FAIL_FLAG', str(flag))
    data, path = bundle
    direct_profile(path, 100)
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(time.perf_counter), clock=time.perf_counter)
    core.head.open_harness = Failing
    core.head_process = True
    core.build()
    try:
        live(core, 'I', until=lambda c: c.head.state == 'IDLE', limit=200)
        live(core, 'E', until=lambda c: c.session.lifecycle.state == 'HOLD', limit=200)
        live(core, 'T', until=lambda c: c.session.lifecycle.state in ('ENGAGING', 'TELEOP'), limit=200)
        flag.write_text('gone')
        live(core, until=lambda c: c.head.fault and c.session.lifecycle.state == 'HOLD', limit=300)
        assert 'went away' in core.head.fault and 'The arms hold' in core.message
    finally:
        core.close()


def test_the_heads_process_ending_on_its_own_holds_the_arms_too(bundle):
    # A head process that dies (killed, or crashed) says nothing more: the arms' side takes its silence as a fault.
    data, path = bundle
    direct_profile(path, 100)
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(time.perf_counter), clock=time.perf_counter)
    core.head_process = True
    core.build()
    try:
        live(core, 'I', until=lambda c: c.head.state == 'IDLE', limit=200)
        core.head._process.kill()
        live(core, until=lambda c: c.head.fault, limit=200)
        assert 'ended unexpectedly' in core.head.fault
    finally:
        try:
            core.close()
        except RuntimeError:
            pass  # Closing a head process that is already gone may say so.


def test_an_arms_fault_takes_the_head_back_to_its_rest_before_its_torque_goes(bundle):
    # Live, the arms faulted at Enable ("command_expired · wrong_owner", 2026-10-31) and the head only held: stopping
    # the session then cuts its torque wherever it is held, and a raised head falls. After the arms' fault the head
    # goes back as Return (X) takes it, to its reference, then down to where it rested, and is disabled there; the
    # session lands only once it is.
    data, path = bundle
    direct_profile(path, 100)
    clock = Clock()
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=drooped_head()),
                      open_leaders=lambda _: Leaders(clock), clock=clock)
    core.build()
    try:
        for key in ('I', 'E'):
            run(core, clock, key)
        run(core, clock, 'T', until='TELEOP', limit=1000)
        for _ in range(800):  # The operator looks straight ahead: the head rises from its rest.
            core.head_pose(0., 0.)
            run(core, clock)
        tilt = core.view()['head']['positions']['head_tilt']
        assert core.head.state == 'TELEOP' and tilt == pytest.approx(core.head.centre['head_tilt'], abs=.02)
        due = core.head.due
        while core.head.due == due:  # Just after the head's cycle, whose own command lives on,
            run(core, clock)
        clock.now += .035  # a late arms' cycle: their command expires.
        core.tick(None, clock())
        assert core.fault and 'command_expired' in core.fault and core.head.fault is None
        assert core.land()  # The session is stopped: it lands first.
        for _ in range(3000):
            if not core.landing:
                break
            clock.now += core.period_s
            core.tick(None, clock())
        assert not core.landing and core.head.fault is None
        assert (core.session.lifecycle.state, core.head.state) == ('DISABLED', 'DISABLED')
        tilt = core.view()['head']['positions']['head_tilt']
        assert tilt == pytest.approx(core.head.rest_target['head_tilt'], abs=.03)  # Disabled at its rest.
    finally:
        core.close()


class HeadService(Receiver):
    """A live head's receiver: its replies carry the service's rows (`recent`), as bin/head_controller's do."""

    @property
    def last(self):
        from robot_platform.control.servo_sweep import result_rows
        last = self.harness.last
        return dict(last, recent=result_rows(last, ())[0]) if last else last


class Feed:
    def __init__(self, camera_pose):
        self.camera_pose, self.replies = camera_pose, []

    def publish(self, last, wall_now):
        self.replies.append((last['recent'][-1][0], wall_now))


def test_a_live_head_publishes_every_reply_for_the_headsets_world_locked_view():
    # The headset keeps the head camera's depth where it is in the room from the head's measured pose at each
    # frame's capture (neck.head_pose): a live head puts each of its service's replies on the feed, and the relay
    # stamps the frames from it. A failing feed ends; the head goes on.
    from robot_platform.helm.head_teleop import HeadTeleop
    feeds = []
    live = HeadTeleop(head_bundle(), live=True, open_receiver=lambda config, reference: HeadService(
        config, reference, time.perf_counter), open_feed=lambda pose: feeds.append(Feed(pose)) or feeds[-1])
    live.build()
    try:
        (feed,) = feeds
        assert feed.camera_pose.joints == ['neck_z', 'neck_y'] and live.feed is feed
        exchanges = []
        inner = live.receiver.exchange
        live.receiver.exchange = lambda request: exchanges.append(request) or inner(request)
        for key in ('enable', None, None):
            live.cycle(key, time.perf_counter() + 1)
        assert len(feed.replies) == len(exchanges) > 0  # Every reply.
        assert all(a[0] <= b[0] and a[1] <= b[1] for a, b in zip(feed.replies, feed.replies[1:]))

        def broken(last, wall_now):
            raise OSError('feed gone')
        feed.publish = broken
        live.cycle(None, time.perf_counter() + 2)
        assert live.feed is None and live.fault is None
    finally:
        live.close()
    virtual = HeadTeleop(head_bundle(), live=False, open_feed=lambda pose: pytest.fail('A virtual head has no feed'))
    virtual.build()
    virtual.close()
