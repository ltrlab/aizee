"""Helm's live view: the stopped reader runs whenever Minerva is online and nothing else holds the motors."""
from types import SimpleNamespace

from robot_platform.helm.live_view import LiveView


class Link:
    def __init__(self):
        self.running, self.connects, self.disconnects = False, 0, 0

    def connect(self):
        self.running, self.connects = True, self.connects + 1

    def disconnect(self):
        self.running, self.disconnects = False, self.disconnects + 1


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now


def setup():
    session = SimpleNamespace(connection='tether', feed={'state': 'disconnected'})
    link, clock, holder = Link(), Clock(), [None]
    view = LiveView(session, link, lambda: holder[0], clock=clock, retry_s=10., handoff_s=5.)
    return view, session, link, clock, holder


def test_the_reader_starts_on_its_own_and_again_after_a_session_frees_the_motors():
    view, session, link, clock, holder = setup()
    assert view.tick() and link.connects == 1
    assert not view.tick()  # Running already.
    # A sweep takes the motors: the reader steps aside and stays off while the sweep holds them.
    view.yield_motors()
    holder[0] = 'the servo sweep'
    clock.now += 60
    assert not link.running and not view.tick()
    holder[0] = None  # It ended.
    assert view.tick() and link.connects == 2
    # A session that never took the motors (refused) lets it start again after the handoff.
    view.yield_motors()
    clock.now += 1
    assert not view.tick()
    clock.now += 5
    assert view.tick() and link.connects == 3


def test_offline_it_tries_again_every_while_and_only_over_the_tether():
    view, session, link, clock, _holder = setup()
    session.feed['state'] = 'fault'  # The last try failed: Minerva not reached.
    assert not view.tick()
    clock.now += 9
    assert not view.tick()
    clock.now += 2
    assert view.tick() and link.connects == 1
    link.running = False  # It failed again.
    assert not view.tick()
    clock.now += 11
    assert view.tick() and link.connects == 2
    link.running, session.feed['state'], session.connection = False, 'disconnected', 'wifi'
    assert not view.tick()  # Over Wi-Fi only on Connect.


def test_disconnect_turns_the_live_view_off_until_connect():
    view, _session, link, _clock, _holder = setup()
    view.set_enabled(False)
    assert not view.tick() and link.connects == 0
    assert view.snapshot() == {'enabled': False, 'owner': None}
    view.set_enabled(True)
    assert view.tick() and link.connects == 1
