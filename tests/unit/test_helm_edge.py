"""The teleop loop on the robot computer: framing, and a whole session over pipes and a loopback link."""
import io
import json
import os
import threading
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm import edge
from robot_platform.helm.edge import EdgeWorker, Inbox, Outbox, pump, read_frame, serve, write_frame
from robot_platform.helm.teleop import TeleopSession, routes_path
from robot_platform.ops.release import ROOT
from robot_platform.teleop.leader_link import LeaderLinkInput, LeaderLinkServer, SyntheticLeaders
from robot_platform.teleop.openrb import LeaderSampleSkipped

TELEOP = ROOT / 'config/devices/minerva-teleop.json'
PROFILE = {'leaders': {'left': {'usb_serial': 'A1'}, 'right': {'usb_serial': 'B2'}}, 'max_age_s': .05,
           'max_skew_s': .02}
ASSET = 'minerva-production-01'


def test_frames_round_trip_and_bad_streams_end():
    stream = io.BytesIO()
    write_frame(stream, ['view', {'phase': 'running', 'q': [1.5, None]}, None])
    stream.seek(0)
    assert read_frame(stream) == ['view', {'phase': 'running', 'q': [1.5, None]}, None]
    with pytest.raises(EOFError):
        read_frame(stream)
    with pytest.raises(ValueError, match='exceeds the limit'):
        read_frame(io.BytesIO((1 << 30).to_bytes(4, 'big')))
    inbox = Inbox()
    assert pump(io.BytesIO(b'\x00\x00\x00\x09abc'), inbox) == 'The channel closed' and not inbox.poll()


class Core:
    """A loop core that reads the leaders over the link each cycle, like TeleopCore."""
    period_s = .01
    clock = staticmethod(time.perf_counter)

    def __init__(self, bundle, *, open_leaders):
        self.bundle, self.open_leaders = bundle, open_leaders
        self.keys, self.samples, self.closed, self.fault, self.scheduling = [], 0, False, None, {}

    def build(self):
        self.leaders = self.open_leaders(None)

    def tick(self, key, started):
        if key:
            self.keys.append(key)
        try:
            self.samples += bool(self.leaders.poll())
        except LeaderSampleSkipped:
            pass

    def view(self):
        return {'phase': 'stopped' if self.closed else 'running', 'fault': self.fault, 'message': '',
                'keys': list(self.keys), 'samples': self.samples, 'joints': [], 'bundle': self.bundle}

    def guide(self):
        return {'sequence': len(self.keys)}

    def route(self, action, motor):
        if motor == 'nope':
            raise ValueError('Use flip or accept with a routed motor')
        return {motor: {'sign': -1, 'verified': action == 'accept'}}

    def set_zero(self):
        return {'left': {'channel_1': 1.5}}

    def fail(self, reason):
        self.fault = reason

    def close(self):
        self.closed = True
        self.leaders.close()


def fake_prepare(start, root):
    return start['bundle'], LeaderLinkInput(bytes.fromhex(start['key']), PROFILE, bind=(start['bind'], 0),
                                            timeout_s=.2)


class Channel:
    def __init__(self, robot, writer):
        self.robot, self.writer, self.status = robot, writer, None

    def recv_exit_status(self):
        self.robot.join(5)
        return self.status

    def shutdown_write(self):
        self.writer.close()

    def close(self):
        self.writer.close()


class End:
    """One end of a pipe, carrying `channel` like a paramiko channel file."""

    def __init__(self, stream, channel=None):
        self.stream, self.channel = stream, channel

    def write(self, data):
        self.stream.write(data)

    def flush(self):
        self.stream.flush()

    def read(self, size=-1):
        return self.stream.read(size)

    def close(self):
        self.stream.close()


class Robot:
    """Stands in for SSH: each exec runs edge.serve in a thread behind OS pipes."""

    def __init__(self):
        self.commands = []

    def exec_command(self, command):
        self.commands.append(command)
        to_robot, from_robot = os.pipe(), os.pipe()
        pc_in, robot_in = os.fdopen(to_robot[1], 'wb'), os.fdopen(to_robot[0], 'rb')
        robot_out, pc_out = os.fdopen(from_robot[1], 'wb'), os.fdopen(from_robot[0], 'rb')

        def run():
            start = read_frame(robot_in)
            inbox = Inbox()
            threading.Thread(target=lambda: inbox.end(pump(robot_in, inbox)), daemon=True).start()
            channel.status = serve(start, inbox, Outbox(robot_out), core_factory=Core)

        robot = threading.Thread(target=run, daemon=True)
        channel = Channel(robot, pc_in)
        robot.start()
        return End(pc_in, channel), End(pc_out, channel), End(io.BytesIO(b'traceback on stderr'))

    def close(self):
        pass


def serve_in_process(profile, key, robot, *, local, real):
    server = LeaderLinkServer(SyntheticLeaders(profile), key, robot, local=local).start()
    return SimpleNamespace(close=server.close)


def session(tmp_path, robot, stored=None):
    device = json.loads((ROOT / 'config/devices/minerva-production-01.json').read_text(encoding='utf-8'))
    helm = SimpleNamespace(profile_path=ROOT / 'config/devices/minerva-joint-calibration.json', bindings={},
                           mapping={}, saved_limits={'joints': {}}, asset_id=ASSET, device=device)
    stored = [] if stored is None else stored
    leaders = SimpleNamespace(saved=None, missing=lambda routes: [], store_reference=stored.append)

    def factory(bundle, leader_profile, **options):
        return EdgeWorker(bundle, PROFILE, open_ssh=lambda *a, **k: robot, serve_link=serve_in_process,
                          **dict(options, address='127.0.0.1'))
    return TeleopSession(helm, TELEOP, runs=tmp_path, leaders=leaders, target='jetson', edge_factory=factory)


def wait_until(predicate, timeout=5.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(.01)


def test_a_session_on_the_robot_relays_keys_routes_and_the_zero(tmp_path, monkeypatch):
    monkeypatch.setattr(edge, 'prepare', fake_prepare)
    robot, stored = Robot(), []
    teleop = session(tmp_path, robot, stored)
    teleop.start()
    try:
        wait_until(lambda: teleop.snapshot()['phase'] == 'running')
        view = teleop.snapshot()
        assert view['target'] == 'jetson' and 'helm.edge' in robot.commands[0]
        assert view['bundle']['teleop_profile'] == 'config/devices/minerva-teleop.json'
        assert view['bundle']['routes_path'] is None and view['bundle']['route_edits'] == {}
        teleop.key('I')
        teleop.key('E')
        wait_until(lambda: teleop.snapshot()['keys'] == ['I', 'E'])
        wait_until(lambda: teleop.snapshot()['samples'] > 3)  # Leader pairs over the loopback link.
        guide = teleop.guide()
        assert guide['sequence'] == 2 and abs(guide['updated_at'] - time.time()) < 1  # Stamped on arrival.
        teleop.route('accept', 'left_arm_j1')
        saved = json.loads(routes_path(tmp_path, ASSET).read_text(encoding='utf-8'))
        assert saved['routes'] == {'left_arm_j1': {'sign': -1, 'verified': True}}  # Helm keeps the file.
        with pytest.raises(ValueError, match='routed motor'):
            teleop.route('flip', 'nope')
        assert teleop.set_zero() == {'left': {'channel_1': 1.5}} and stored == [{'left': {'channel_1': 1.5}}]
        with pytest.raises(ValueError, match='Stop the session'):
            teleop.set_target('pc')
    finally:
        teleop.stop()
    assert not teleop.active and teleop.snapshot()['phase'] == 'stopped'
    teleop.set_target('pc')
    assert teleop.snapshot()['target'] == 'pc'


def test_a_robot_that_cannot_start_the_loop_says_why(tmp_path, monkeypatch):
    def mismatch(start, root):
        raise ValueError("The robot's calibration files differ from this PC's. Deploy and start again.")
    monkeypatch.setattr(edge, 'prepare', mismatch)
    teleop = session(tmp_path, Robot())
    teleop.start()
    wait_until(lambda: teleop.snapshot()['phase'] == 'fault')
    assert 'calibration files differ' in teleop.snapshot()['fault']
    teleop.stop()
    unreachable = session(tmp_path, None)
    unreachable.edge_factory = lambda bundle, profile, **options: EdgeWorker(
        bundle, PROFILE, open_ssh=lambda *a, **k: (_ for _ in ()).throw(OSError('No route to host')),
        **dict(options, address='127.0.0.1'))
    unreachable.start()
    wait_until(lambda: unreachable.snapshot()['phase'] == 'fault')
    assert unreachable.snapshot()['message'] == "Teleop on Minerva's Jetson stopped: No route to host"
    unreachable.stop()


def test_a_live_session_needs_the_jetson_a_confirmation_and_carries_the_motor_models(tmp_path, monkeypatch):
    monkeypatch.setattr(edge, 'prepare', fake_prepare)
    teleop = session(tmp_path, Robot())
    with pytest.raises(ValueError, match='left, right or both arms'):
        teleop.set_receiver('live', 'middle')
    with pytest.raises(ValueError, match='left, right or both arms'):
        teleop.set_receiver('live', 'both', 'motion')  # Implied live: only the virtual receiver chooses it.
    teleop.set_receiver('live', 'both')
    assert teleop.snapshot()['receiver_choice'] == {'mode': 'live', 'arm': 'both'}
    with pytest.raises(ValueError, match="Confirm the live session: both arms' motors will be powered"):
        teleop.start()
    teleop.set_receiver('live', 'left')
    with pytest.raises(ValueError, match='virtual receiver first'):
        teleop.set_target('pc')
    with pytest.raises(ValueError, match="Confirm the live session: the left arm's motors will be powered"):
        teleop.start()
    teleop.start(confirm=True)
    try:
        wait_until(lambda: teleop.snapshot()['phase'] == 'running')
        receiver = teleop.snapshot()['bundle']['receiver']
        assert receiver['mode'] == 'live' and receiver['arm'] == 'left'
        assert receiver['motor_models']['left_arm_j2'] == 'ROBSTRIDE04' and len(receiver['motor_models']) == 14
        assert teleop.snapshot()['receiver_choice'] == {'mode': 'live', 'arm': 'left'}
        with pytest.raises(ValueError, match='Stop the session'):
            teleop.set_receiver('virtual')
    finally:
        teleop.stop()
    teleop.set_receiver('virtual', collision='motion')
    assert teleop.snapshot()['receiver_choice'] == {'mode': 'virtual', 'collision': 'motion'}
    teleop.set_receiver('virtual')
    assert teleop.snapshot()['receiver_choice'] == {'mode': 'virtual'}
    teleop.set_target('pc')
    with pytest.raises(ValueError, match="loop on Minerva's Jetson"):
        teleop.set_receiver('live', 'right')
