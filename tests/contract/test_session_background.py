"""Deterministic worker control tests; every transport is a fake, never hardware."""

from copy import deepcopy
from types import SimpleNamespace
import threading

import pytest

from robot_platform.control import RobotSession, ThreadedRobotClient
import robot_platform.control.background as background
import robot_platform.control.session as session_module
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings


class FakeIO:
    def __init__(self, session, *, context=None, receive_only=False):
        self.session = session
        self.receive_only = receive_only
        self.sent = []
        self.failures = {}
        self.calls = []
        self.polled = threading.Event()

    def connect(self):
        self.calls.append(("connect", threading.get_ident()))

    def poll(self):
        self.calls.append(("poll", threading.get_ident()))
        self.polled.set()
        return self.session.observation()

    def send_commands(self, commands):
        self.calls.append(("send", threading.get_ident()))
        for arm, command in commands.items():
            key = (arm, command["type"])
            if self.failures.get(key, 0):
                self.failures[key] -= 1
                raise RuntimeError("simulated socket backpressure")
            self.sent.extend(self.session.prepare_commands({arm: command}).items())

    def close(self):
        self.calls.append(("close", threading.get_ident()))


def feed(session, arm, now):
    session.ingest_telemetry(arm, {"motors": {name: {"position": 0.0, "torque": 0.1}
                                             for name in session.arms[arm].joints}}, received_at=now)


def wire(value=0.1):
    return {"type": "arm_joints", "positions": [value] * 7, "velocities": [0.0] * 7}


@pytest.fixture
def worker(monkeypatch):
    clock = SimpleNamespace(now=1.0)
    fake_time = SimpleNamespace(monotonic=lambda: clock.now)
    monkeypatch.setattr(background, "time", fake_time)
    monkeypatch.setattr(session_module, "time", fake_time)
    monkeypatch.setattr(background, "ZmqRobotClient", FakeIO)
    spec = load_robot("minerva")
    session = RobotSession(spec, load_settings(spec))
    for arm in session.arms:
        feed(session, arm, clock.now)
    return ThreadedRobotClient(session), clock


def tick(client):
    with client._condition:
        client._step()


def target(value=0.2):
    return [value] * 14 + [0.0] * 3


def test_keepalive_rechecks_each_arm_and_forgets_stale_target(worker):
    client, clock = worker
    client.session.target_lease_s = 2.0  # isolate feedback expiry from producer expiry
    client.set_target(target())
    tick(client)
    assert [arm for arm, _ in client._client.sent] == ["left", "right"]
    clock.now = 1.6
    feed(client.session, "right", clock.now)
    tick(client)
    assert [arm for arm, _ in client._client.sent] == ["left", "right", "right"]
    feed(client.session, "left", clock.now)
    tick(client)
    assert client._client.sent[-1][0] == "right"
    assert "left" not in client._held  # old target must not revive on recovery
    client.set_target(target(0.3))
    tick(client)
    assert [arm for arm, _ in client._client.sent[-2:]] == ["left", "right"]


def test_producer_lease_expires_despite_healthy_telemetry(worker):
    client, clock = worker
    client.set_target(target())
    tick(client)
    count = len(client._client.sent)
    clock.now += 0.6
    for arm in client.session.arms:
        feed(client.session, arm, clock.now)
    tick(client)
    assert len(client._client.sent) == count
    assert not client._held
    client.set_target(target(0.3))
    tick(client)
    assert len(client._client.sent) == count + 2


def test_queued_motion_expires_before_worker_first_send(worker):
    client, clock = worker
    client.submit({"left": {"type": "arm_joints", "positions": [0.2]*7, "velocities": [0.0]*7}})
    clock.now += 0.6
    feed(client.session, "left", clock.now)
    tick(client)
    assert not client._client.sent


def test_one_shot_and_held_targets_obey_latest_submission_order(worker):
    client, _ = worker
    client.set_target(target(0.1))
    client.submit({"left": wire(0.7)})
    tick(client)
    assert [command["positions"][0] for arm, command in client._client.sent if arm == "left"] == [0.7]
    tick(client)
    assert [command["positions"][0] for arm, command in client._client.sent if arm == "left"] == [0.7]
    client._client.sent.clear()
    client.submit({"left": wire(0.8)})
    client.set_target(target(0.2))
    tick(client)
    assert [command["positions"][0] for arm, command in client._client.sent if arm == "left"] == [0.2]


def test_submissions_coalesce_and_cannot_mutate_queued_commands(worker):
    client, _ = worker
    for i in range(1000):
        message = wire(i / 1000)
        client.submit({"left": message})
    message["positions"][0] = 999.0
    assert len(client._pending_motion) == 1
    tick(client)
    assert client._client.sent == [("left", wire(0.999))]


def test_disable_priority_and_enable_preserve_state_order(worker):
    client, _ = worker
    names = list(client.session.arms["left"].joints)
    client.set_target(target())
    client.submit({"left": {"type": "disable", "motor_ids": names}})
    client.submit({"left": wire(0.4)})
    tick(client)
    assert [command["type"] for arm, command in client._client.sent if arm == "left"] == ["disable"]
    client._client.sent.clear()
    client.submit({"left": {"type": "enable", "motor_ids": names}})
    client.submit({"left": wire(0.5)})
    tick(client)
    assert [command["type"] for arm, command in client._client.sent if arm == "left"] == ["enable", "arm_joints"]


def test_wheel_disable_does_not_disable_arm_target(worker):
    client, _ = worker
    client.submit({"left": {"type": "disable", "motor_ids": ["left_wheel", "right_wheel"]}})
    client.submit({"left": wire()})
    tick(client)
    assert [command["type"] for _, command in client._client.sent] == ["disable", "arm_joints"]


def test_emergency_stop_blocks_arm_and_drive_until_clear_and_enable(worker):
    client, _ = worker
    client.submit({"left": {"type": "emergency_stop"}})
    client.submit({"left": {"type": "bundle", "arm_joints": wire(), "drive": {"linear": 0.1, "angular": 0.0}}})
    tick(client)
    assert client._client.sent == [("left", {"type": "emergency_stop"})]
    client.submit({"left": {"type": "clear_emergency_stop"}})
    client.submit({"left": wire()})
    tick(client)
    assert client._client.sent[-1] == ("left", {"type": "clear_emergency_stop"})


def test_clear_target_drops_queued_motion_and_optionally_drive(worker):
    client, _ = worker
    command = {"type": "bundle", "arm_joints": wire(), "drive": {"linear": 0.1, "angular": 0.0}}
    client.submit({"left": command})
    client.clear_target()
    tick(client)
    assert client._client.sent[-1] == ("left", {"type": "bundle", "drive": command["drive"]})
    client._client.sent.clear()
    client.submit({"left": command})
    client.clear_target(drop_drive=True)
    tick(client)
    assert not client._client.sent


def test_backpressured_disable_retries_and_does_not_starve_other_arm(worker):
    client, _ = worker
    client._client.failures[("left", "disable")] = 1
    commands = {arm: {"type": "disable", "motor_ids": list(endpoint.joints)} for arm, endpoint in client.session.arms.items()}
    client.submit(commands)
    tick(client)
    assert client._client.sent == [("right", commands["right"])]
    tick(client)
    assert client._client.sent[-1] == ("left", commands["left"])
    assert not client._controls


def test_service_queue_deadline_is_reported_not_silently_dropped(worker):
    client, clock = worker
    client._client.failures[("left", "disable")] = 10
    client.submit({"left": {"type": "disable", "motor_ids": list(client.session.arms["left"].joints)}})
    tick(client)
    clock.now = 2.1
    tick(client)
    assert "delivery deadline" in str(client.error)
    with pytest.raises(RuntimeError, match="delivery failed"):
        client.close()


def test_close_keeps_stop_before_disable_but_drops_activation_and_motion(worker):
    client, _ = worker
    names = list(client.session.arms["left"].joints)
    client.submit({"left": {"type": "drive", "linear": 0.0, "angular": 0.0}})
    client.submit({"left": {"type": "disable", "motor_ids": names}})
    client.submit({"right": {"type": "enable", "motor_ids": list(client.session.arms["right"].joints)}})
    client.submit({"right": {"type": "mech_zero", "motor_ids": list(client.session.arms["right"].joints), "save": True}})
    client.submit({"right": wire()})
    client._thread = SimpleNamespace(join=lambda timeout: tick(client), is_alive=lambda: False)
    client.close()
    assert [command["type"] for _, command in client._client.sent] == ["drive", "disable"]
    assert not client._pending_motion and not client._held


def test_worker_owns_entire_transport_lifecycle(monkeypatch):
    monkeypatch.setattr(background, "ZmqRobotClient", FakeIO)
    spec = load_robot("aizee")
    client = ThreadedRobotClient(RobotSession(spec, load_settings(spec)))
    io = client._client
    assert not io.calls
    client.start()
    assert io.polled.wait(1.0)
    client.submit({"main": {"type": "disable", "motor_ids": list(spec.joint_names)}})
    client.close()
    owner = client._thread.ident
    assert owner != threading.get_ident()
    assert {name for name, _ in io.calls} == {"connect", "poll", "send", "close"}
    assert all(thread_id == owner for _, thread_id in io.calls)
