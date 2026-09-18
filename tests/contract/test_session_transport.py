"""Transport lifecycle with fake sockets only: no network or motor commands."""

import sys
from types import SimpleNamespace

import pytest

from robot_platform.control import PartialSendError, RobotSession, StaleTelemetryError, ZmqRobotClient
from robot_platform.transport.wire import pack_msg, unpack_msg


class Again(Exception):
    pass


class FakeSocket:
    def __init__(self):
        self.frames = []
        self.sent = []
        self.closed = False
        self.failure = None
        self.options = {}

    def setsockopt(self, name, value):
        self.options[name] = value

    def connect(self, address):
        self.address = address

    def recv(self, flags):
        if not self.frames:
            raise Again()
        return self.frames.pop(0)

    def send(self, frame, flags):
        if self.failure:
            raise self.failure
        self.sent.append(frame)

    def close(self, linger):
        self.closed = True


class FakeContext:
    def __init__(self):
        self.sockets = []
        self.terminated = False

    def socket(self, kind):
        socket = FakeSocket()
        self.sockets.append(socket)
        return socket

    def term(self):
        self.terminated = True


@pytest.fixture
def backend(monkeypatch):
    context = FakeContext()
    zmq = SimpleNamespace(PUSH=1, SUB=2, LINGER=3, IMMEDIATE=4, SNDHWM=5,
                          CONFLATE=6, SUBSCRIBE=7, NOBLOCK=8, Again=Again,
                          Context=lambda: context)
    monkeypatch.setitem(sys.modules, "zmq", zmq)
    names = ("left_joint", "right_joint")
    spec = SimpleNamespace(joints=tuple(SimpleNamespace(name=name, active=True) for name in names))
    settings = {"endpoints": {"arms": {
        arm: {"command": f"inproc://{arm}-cmd", "telemetry": f"inproc://{arm}-telem", "joints": [name]}
        for arm, name in zip(("left", "right"), names)
    }}}
    session = RobotSession(spec, settings)
    return session, context


def feed(context):
    for index, name in ((1, "left_joint"), (3, "right_joint")):
        context.sockets[index].frames.append(pack_msg({"motors": {name: {"position": 1.0, "torque": 0.1}}}))


def test_construction_is_offline_and_connect_never_enables(backend):
    session, context = backend
    client = ZmqRobotClient(session, context=context)
    assert context.sockets == []
    with pytest.raises(RuntimeError, match="connect"):
        client.poll()
    with pytest.raises(RuntimeError, match="connect"):
        client.send_target([0.0, 0.0])
    client.connect()
    client.connect()
    assert len(context.sockets) == 4
    assert all(socket.sent == [] for socket in context.sockets)
    with pytest.raises(StaleTelemetryError):
        client.send_target([0.0, 0.0])
    feed(context)
    assert client.poll().ready
    client.send_target([0.2, 0.3])
    assert unpack_msg(context.sockets[0].sent[0])["positions"] == [0.2]
    assert unpack_msg(context.sockets[2].sent[0])["positions"] == [0.3]
    client.close()
    assert all(socket.closed for socket in context.sockets)
    assert not context.terminated  # externally provided context belongs to caller
    assert sum(len(socket.sent) for socket in context.sockets) == 2


def test_decode_failure_invalidates_prior_healthy_sample(backend):
    session, context = backend
    with ZmqRobotClient(session, context=context) as client:
        feed(context)
        assert client.poll().ready
        context.sockets[1].frames.append(b"\xc1")  # reserved MessagePack byte
        with pytest.raises(Exception):
            client.poll()
        assert not session.observation().ready
        with pytest.raises(StaleTelemetryError):
            client.send_target([0.1, 0.2])
    assert all(not socket.sent for socket in context.sockets)


def test_partial_send_failure_reports_which_arms_received_target(backend):
    session, context = backend
    with ZmqRobotClient(session, context=context) as client:
        feed(context)
        client.poll()
        context.sockets[2].failure = Again()
        with pytest.raises(PartialSendError) as error:
            client.send_target([0.1, 0.2])
        assert error.value.sent_arms == ("left",)
        assert error.value.failed_arm == "right"
        assert len(context.sockets[0].sent) == 1
        assert not context.sockets[2].sent


def test_owned_context_is_closed_and_clock_override_is_rejected(backend):
    session, context = backend
    with ZmqRobotClient(session) as client:
        feed(context)
        client.poll()
        with pytest.raises(TypeError, match="clock override"):
            client.send_target([0.1, 0.2], now=0.0)
    assert context.terminated
    assert all(not socket.sent for socket in context.sockets)


def test_malformed_left_does_not_starve_right_telemetry(backend):
    session, context = backend
    with ZmqRobotClient(session, context=context) as client:
        context.sockets[1].frames.append(b"\xc1")
        context.sockets[3].frames.append(pack_msg({"motors": {"right_joint": {"position": 2.0, "torque": 0.2}}}))
        with pytest.raises(Exception):
            client.poll()
        assert session.arm_ready() == {"left": False, "right": True}


def test_receive_only_constructs_no_command_sockets(backend):
    session, context = backend
    with ZmqRobotClient(session, context=context, receive_only=True) as client:
        assert len(context.sockets) == 2
        assert all(socket.address.endswith("-telem") for socket in context.sockets)
        with pytest.raises(RuntimeError, match="receive-only"):
            client.send_target([0.1, 0.2])
        with pytest.raises(RuntimeError, match="receive-only"):
            client.send_commands({"left": {"type": "enable", "motor_ids": ["left_joint"]}})
    assert not any(socket.sent for socket in context.sockets)


def test_strict_target_does_not_pass_through_partial_raw_command_filter(backend, monkeypatch):
    session, context = backend
    with ZmqRobotClient(session, context=context) as client:
        feed(context)
        client.poll()
        def forbidden(*args, **kwargs):
            raise AssertionError("strict targets must not silently become partial commands")
        monkeypatch.setattr(session, "prepare_commands", forbidden)
        client.send_target([0.1, 0.2])
        assert len(context.sockets[0].sent) == len(context.sockets[2].sent) == 1
