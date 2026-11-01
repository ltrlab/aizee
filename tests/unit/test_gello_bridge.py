"""The GELLO bridge for Helm on Minerva: the PC's bridge reaches Helm's hub through the relay, serves a session's
leader link (the same PC-end process as before) so the robot's loop gets signed samples, streams restamped samples
to the Leaders workspace, and refuses a wrong robot, a wrong token and a silent Helm."""
import io
import os
import threading
import time

import pytest

from robot_platform.helm import bridge as hub_module
from robot_platform.helm.bridge import BridgeError, BridgeHub, relay
from robot_platform.helm.edge import read_frame, write_frame
from robot_platform.teleop import gello_bridge
from robot_platform.teleop.gello_bridge import BridgeSession
from robot_platform.teleop.leader_link import LeaderLinkInput

PROFILE = {'leaders': {'left': {'usb_serial': 'A1'}, 'right': {'usb_serial': 'B2'}},
           'max_age_s': .05, 'max_skew_s': .02}
KEY = b'k' * 32
MACHINE = 'minerva-machine-id'


def wait_until(predicate, timeout=10.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(.02)


@pytest.fixture
def hub(tmp_path):
    hub = BridgeHub(port=0, token_file=tmp_path / 'run' / 'helm-bridge-token', machine_id=MACHINE)
    yield hub
    hub.close()


def attach(hub, tmp_path, *, machine_id=MACHINE, **options):
    """The PC's bridge on one end of two pipes, the Jetson's relay on the other, as the forced SSH command runs it."""
    to_relay, to_bridge = os.pipe(), os.pipe()
    relay_in, bridge_out = os.fdopen(to_relay[0], 'rb'), os.fdopen(to_relay[1], 'wb')
    bridge_in, relay_out = os.fdopen(to_bridge[0], 'rb'), os.fdopen(to_bridge[1], 'wb')
    threading.Thread(target=relay, args=(relay_in, relay_out), daemon=True,
                     kwargs={'port': hub.port, 'token_file': hub.token_file}).start()
    session = BridgeSession(bridge_in, bridge_out, machine_id=machine_id, synthetic=True,
                            route=lambda address: '127.0.0.1', **options)
    thread = threading.Thread(target=session.run, daemon=True)
    thread.start()
    return session, thread


def test_a_session_link_served_through_the_bridge_gets_the_robot_signed_samples(hub, tmp_path):
    with pytest.raises(BridgeError, match='not connected: start it on the PC'):
        hub.serve_link(PROFILE, KEY, ('127.0.0.1', 9))
    session, _ = attach(hub, tmp_path)
    wait_until(lambda: hub.connected)
    assert hub.snapshot()['host'] == session.host
    robot = LeaderLinkInput(KEY, PROFILE, bind=('127.0.0.1', 0), timeout_s=.5)
    try:
        link = hub.serve_link(PROFILE, KEY, robot.address)  # The PC end, in its own process, via the bridge.
        robot.wait_for_pc(15.)
        first, second = robot.poll(), robot.poll()
        assert second['left'].sequence > first['left'].sequence and robot.fault is None
        report = link.close()
    finally:
        robot.close()
    assert report['served'] >= 2 and report['failure'] is None and report['rejected'] == 0
    assert link.close() is report  # Closing twice is harmless.


def test_the_leaders_workspace_reads_samples_restamped_on_arrival(hub, tmp_path):
    attach(hub, tmp_path)
    wait_until(lambda: hub.connected)
    leaders = hub.open_leaders(PROFILE, rate_hz=50.)
    try:
        from robot_platform.teleop.openrb import LeaderSampleSkipped
        samples = None

        def fresh():
            nonlocal samples
            try:
                samples = leaders.poll()
                return True
            except LeaderSampleSkipped:
                return False
        wait_until(fresh)
        assert set(samples) == {'left', 'right'}
        assert all(0 <= hub.clock() - s.sampled_at < .5 for s in samples.values())  # This computer's clock.
        with pytest.raises(BridgeError, match='Leaders tab'):
            hub.serve_link(PROFILE, KEY, ('127.0.0.1', 9))  # One owner of the leader USB at a time.
    finally:
        leaders.close()
    robot = LeaderLinkInput(KEY, PROFILE, bind=('127.0.0.1', 0), timeout_s=.5)
    try:
        wait_until(lambda: not hub._leaders)
        time.sleep(.2)
        hub.serve_link(PROFILE, KEY, robot.address).close()  # Once closed, a session may have them.
    finally:
        robot.close()


def test_a_wrong_robot_or_token_is_refused(hub, tmp_path):
    session, thread = attach(hub, tmp_path, machine_id='another-computer')
    thread.join(10.)
    assert not thread.is_alive()  # The bridge left: the welcome named another machine,
    wait_until(lambda: not hub.connected)  # and the hub sees it gone.
    import socket
    sock = socket.create_connection(('127.0.0.1', hub.port))
    writer = sock.makefile('wb')
    write_frame(writer, {'relay': hub_module.VERSION, 'token': 'not-the-token', 'peer': None})
    write_frame(writer, ['hello', {'version': hub_module.VERSION, 'host': 'impostor'}])
    reader = sock.makefile('rb')
    with pytest.raises((EOFError, OSError)):
        read_frame(reader)  # Dropped without a word.
    sock.close()
    assert not hub.connected


def test_a_silent_helm_ends_the_connection_and_stops_what_the_bridge_served(monkeypatch):
    monkeypatch.setattr(gello_bridge, 'SILENCE_S', .4)
    monkeypatch.setattr(gello_bridge, 'BEAT_S', .1)
    closed = threading.Event()

    class Link:
        def close(self):
            closed.set()
            return {'served': 0}
    import socket
    bridge_end, helm_end = socket.socketpair()
    helm_in, helm_out = helm_end.makefile('rb'), helm_end.makefile('wb')
    # As paramiko's client.close() on the SSH channel: the bridge's blocked read ends at end of stream.
    session = BridgeSession(bridge_end.makefile('rb'), bridge_end.makefile('wb'), machine_id=MACHINE, synthetic=True,
                            route=lambda address: '127.0.0.1', serve_link=lambda *a, **k: Link(),
                            hangup=lambda: helm_end.shutdown(socket.SHUT_WR))
    thread = threading.Thread(target=session.run, daemon=True)
    thread.start()
    assert read_frame(helm_in)[0] == 'hello'
    write_frame(helm_out, ['welcome', {'version': hub_module.VERSION, 'machine_id': MACHINE, 'host': 'minerva'}])
    write_frame(helm_out, ['link', 7, {'profile': PROFILE, 'key': KEY.hex(), 'robot': ['127.0.0.1', 9], 'real': True}])
    while (message := read_frame(helm_in))[0] == 'beat':
        pass
    assert message == ['link_up', 7]
    thread.join(5.)  # Helm says nothing more: after 0.4 s the bridge ends the connection,
    assert not thread.is_alive() and closed.is_set()  # and the link it served stops with it.


def test_the_logon_task_logs_to_its_own_file_and_a_crash_says_why(tmp_path, monkeypatch):
    import logging
    path = tmp_path / 'gello-bridge.log'
    monkeypatch.setattr(gello_bridge, 'LOG_FILE', None)
    monkeypatch.setattr(logging, 'basicConfig', lambda **options: None)  # Leave pytest's logging alone.
    monkeypatch.setattr(gello_bridge, 'run', lambda **options: gello_bridge.log('cannot reach Helm on 10.42.0.1: timed out'))
    assert gello_bridge.main(['--key', 'k', '--log', str(path)]) == 0
    assert path.read_text(encoding='utf-8').endswith('cannot reach Helm on 10.42.0.1: timed out\n')

    def broken(**options):
        raise RuntimeError('no profile')
    monkeypatch.setattr(gello_bridge, 'run', broken)
    with pytest.raises(RuntimeError):
        gello_bridge.main(['--key', 'k', '--log', str(path)])
    assert 'stopped: Traceback' in path.read_text(encoding='utf-8') and 'no profile' in path.read_text(encoding='utf-8')


def test_the_relay_says_where_the_bridge_reached_minerva_and_the_bridge_remembers_its_addresses(tmp_path,
                                                                                                monkeypatch):
    monkeypatch.setenv('SSH_CLIENT', '192.168.8.225 51234 22')
    monkeypatch.setenv('SSH_CONNECTION', '192.168.8.225 51234 192.168.8.43 22')
    hub = BridgeHub(port=0, token_file=tmp_path / 'run' / 'helm-bridge-token', machine_id=MACHINE,
                    addresses=lambda: ['192.168.8.43', '10.42.0.1'],
                    path_of=lambda peer: 'wired' if peer == '192.168.8.225' else None)
    try:
        session, _ = attach(hub, tmp_path)
        wait_until(lambda: hub.connected)
        snapshot = hub.snapshot()
        assert (snapshot['peer'], snapshot['robot_address'], snapshot['path']) == ('192.168.8.225', '192.168.8.43',
                                                                                   'wired')
        assert hub.robot_address == '192.168.8.43'  # A session's leader link binds there (helm.teleop).
        wait_until(lambda: session.addresses == ['192.168.8.43', '10.42.0.1'])
    finally:
        hub.close()


def test_the_relay_keeps_off_the_arms_loops_cpus_before_it_joins_helm(tmp_path, monkeypatch):
    """The bridge's SSH session may run on every CPU, the arms' loop's too (2026-11-01). The relay moves itself onto
    the background CPUs first, so the thread it starts goes there as well."""
    from robot_platform.control import scheduling
    events = []
    monkeypatch.setattr(scheduling, 'prefer_background_thread', lambda: events.append('placed') or ['background_cpus'])

    def connect(*_args, **_kwargs):
        events.append('connect')
        raise OSError('no hub')
    monkeypatch.setattr(hub_module.socket, 'create_connection', connect)
    token = tmp_path / 'helm-bridge-token'
    token.write_text('t')
    assert relay(io.BytesIO(), io.BytesIO(), port=9, token_file=token) == 1  # Helm not running: it says so, ends.
    assert events == ['placed', 'connect']


def test_the_bridge_tries_minervas_addresses_in_order_and_remembers_where_it_was(monkeypatch):
    from pathlib import Path
    profile = str(Path(__file__).resolve().parents[2] / 'config/devices/minerva-joint-calibration.json')
    assert gello_bridge.candidates(profile) == ['10.42.0.1', 'aizee-rover.local', '192.168.8.180']
    assert gello_bridge.candidates(profile, remembered=['192.168.8.43', '10.42.0.1']) == [
        '10.42.0.1', '192.168.8.43', 'aizee-rover.local', '192.168.8.180']
    assert gello_bridge.candidates(profile, '10.0.0.5, 10.0.0.6') == ['10.0.0.5', '10.0.0.6']
    monkeypatch.setattr(gello_bridge, 'RECONNECT_S', (0.,))
    stop, attempts, sessions = threading.Event(), [], []

    class Session:
        def __init__(self, reader, writer, **options):
            self.addresses = ['192.168.8.43', '10.42.0.1']  # As Helm's welcome reports them.

        def run(self):
            sessions.append(attempts[-1])
            if len(sessions) == 2:
                stop.set()
    monkeypatch.setattr(gello_bridge, 'BridgeSession', Session)

    def connect(address):
        attempts.append(address)
        if address == 'aizee-rover.local':
            raise RuntimeError('SSH host fingerprint mismatch')  # Something answers there, but not Minerva.
        return None, None, lambda: None
    reachable = {'aizee-rover.local', '192.168.8.180', '192.168.8.43'}  # Not the tether: no adapter.
    gello_bridge.run(profile=profile, key='k', connect=connect, stop=stop, probe=lambda address: address in reachable,
                     resolve=lambda name: name)
    # The tether does not answer; the name answers but is not Minerva; the profile's address is. The next time,
    # the wired address Helm reported comes before the name.
    assert attempts == ['aizee-rover.local', '192.168.8.180', '192.168.8.43']
    assert sessions == ['192.168.8.180', '192.168.8.43']


def test_names_resolve_to_ipv4_only(monkeypatch):
    """Windows answers aizee-rover.local with an IPv6 link-local address first; the bridge key allows IPv4 networks."""
    import socket
    families = []

    def getaddrinfo(host, port, family=0, kind=0):
        families.append(family)
        if host == 'nowhere.local':
            raise socket.gaierror('not found')
        return [(socket.AF_INET, socket.SOCK_STREAM, 6, '', ('192.168.8.43', port))]
    monkeypatch.setattr(gello_bridge.socket, 'getaddrinfo', getaddrinfo)
    assert gello_bridge.ipv4('aizee-rover.local') == '192.168.8.43'
    assert gello_bridge.ipv4('nowhere.local') is None
    assert set(families) == {socket.AF_INET}


def test_a_newer_bridge_cuts_the_older_relay_at_once(hub):
    """A bridge that reconnects (Minerva moved from Wi-Fi to a cable) replaces its old relay: the hub ends the old
    connection now, not when TCP gives up on it (2026-10-25: close() left it open under its reader thread)."""
    import socket

    def relay_socket(host):
        sock = socket.create_connection(('127.0.0.1', hub.port))
        writer, reader = sock.makefile('wb'), sock.makefile('rb')
        write_frame(writer, {'relay': hub_module.VERSION, 'token': hub.token, 'peer': None, 'server': None})
        write_frame(writer, ['hello', {'version': hub_module.VERSION, 'host': host}])
        assert read_frame(reader)[0] == 'welcome'
        return sock, reader
    old, old_reader = relay_socket('old')
    new, _ = relay_socket('new')
    try:
        old.settimeout(3.)
        with pytest.raises(EOFError):
            while True:
                read_frame(old_reader)  # Beats sent before the switch, then the end: never a timeout.
        wait_until(lambda: hub.connected and hub.snapshot()['host'] == 'new')
    finally:
        old.close()
        new.close()
