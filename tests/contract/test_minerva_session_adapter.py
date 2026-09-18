"""Minerva collector exercises the shared session through an offline worker."""

from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np
import pytest

from robot_platform.control import ThreadedRobotClient
import robot_platform.control.background as background
import robot_platform.control.session as session_module


@pytest.fixture
def follower(monkeypatch):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root / "python"))
    monkeypatch.syspath_prepend(str(root / "python/scripts"))
    from collect_minerva_app import follower as module

    clock = SimpleNamespace(now=1.0)
    fake_time = SimpleNamespace(monotonic=lambda: clock.now)
    monkeypatch.setattr(background, "time", fake_time)
    monkeypatch.setattr(session_module, "time", fake_time)
    monkeypatch.setenv("ROBOT_PLATFORM_SETTINGS_JSON", json.dumps({"control": {"stale_after_s": 0.25}}))

    class FakeIO:
        def __init__(self, session, *, context=None, receive_only=False):
            self.session, self.receive_only = session, receive_only
            self.sent = []

        def poll(self):
            return self.session.observation()

        def send_commands(self, commands):
            self.sent.extend(self.session.prepare_commands(commands).items())

    class OfflineWorker(ThreadedRobotClient):
        def start(self):
            return self

    monkeypatch.setattr(background, "ZmqRobotClient", FakeIO)
    monkeypatch.setattr(module, "ThreadedRobotClient", OfflineWorker)
    transport = module.DualArmTransport(None, left_cmd="tcp://left:5555", left_telem="tcp://left:5556",
                                       right_cmd="tcp://right:5575", right_telem="tcp://right:5576")
    return transport, clock, module


def message(session, arm, *, voltage=24.0, estop=False):
    return {"battery_voltage": voltage, "emergency_stop": estop, "motors": {
        name: {"position": float(session.joint_names.index(name)), "torque": 0.1,
               "temperature": 35.0, "state": "running", "mode": "run", "error": None}
        for name in session.arms[arm].joints
    }}


def feed_all(transport, now):
    for arm in transport.session.arms:
        transport.session.ingest_telemetry(arm, message(transport.session, arm), received_at=now)


def tick(transport):
    with transport.client._condition:
        transport.client._step()


def test_legacy_vectors_only_zero_inactive_placeholders_and_preserve_metadata(follower):
    transport, clock, _ = follower
    assert transport.qpos() is None and transport.qpos_partial() is None
    feed_all(transport, clock.now)
    assert transport.session.max_age_s == 0.25
    np.testing.assert_array_equal(transport.qpos(), list(range(14)) + [0.0] * 3)
    np.testing.assert_allclose(transport.temps_partial(), [35.0] * 14 + [0.0] * 3)
    observation = transport.observation()
    assert observation.ready
    assert all(np.isnan(observation.positions[14:]))
    positions, torques = transport.recording_sample()
    np.testing.assert_array_equal(positions, transport.qpos())
    np.testing.assert_allclose(torques, [0.1] * 14 + [0.0] * 3)
    transport.session.ingest_telemetry("right", message(transport.session, "right", voltage=23.5, estop=True), received_at=clock.now)
    assert transport.battery_voltage() == 23.5 and transport.estop() is True
    assert transport.client.telemetry("right")["motors"]["right_arm_j1"]["state"] == "running"


def test_missing_torque_is_explicit_and_only_blocks_strict_consumers(follower):
    transport, clock, _ = follower
    feed_all(transport, clock.now)
    left = message(transport.session, "left")
    left["motors"]["left_arm_j1"].pop("torque")
    transport.session.ingest_telemetry("left", left, received_at=clock.now)
    assert transport.arm_ok() == {"left": True, "right": True}
    assert transport.qpos() is not None and transport.torques() is None
    assert transport.recording_sample(require_torque=True) is None
    recorded_positions, recorded_torques = transport.recording_sample()
    assert np.isfinite(recorded_positions).all()
    assert np.isnan(recorded_torques[0])
    np.testing.assert_allclose(recorded_torques[1:14], [0.1] * 13)
    assert np.isnan(transport.torques_partial()[:7]).all()
    transport.set_target(np.array([0.2] * 14 + [0.0] * 3))
    tick(transport)
    assert [arm for arm, _ in transport.client._client.sent] == ["left", "right"]


def test_one_arm_drop_preserves_other_and_does_not_return_fake_zero(follower):
    transport, clock, _ = follower
    feed_all(transport, clock.now)
    clock.now = 1.3
    transport.session.ingest_telemetry("right", message(transport.session, "right", voltage=22.0), received_at=clock.now)
    assert transport.qpos() is None
    partial = transport.qpos_partial()
    assert np.isnan(partial[:7]).all()
    np.testing.assert_array_equal(partial[7:], list(range(7, 14)) + [0.0] * 3)
    assert transport.arm_ok() == {"left": False, "right": True}
    assert transport.battery_voltage() == 22.0
    assert transport.telem_age()["left"] == pytest.approx(0.3)
    transport.set_target(np.array([0.2] * 14 + [0.0] * 3))
    tick(transport)
    assert [arm for arm, _ in transport.client._client.sent] == ["right"]


def test_gain_compatibility_does_not_mutate_inputs_or_drop_inactive_motion(follower):
    transport, clock, _ = follower
    feed_all(transport, clock.now)
    kp, kd = [1.0] * 17, [0.1] * 17
    target = np.array([0.2] * 14 + [0.0] * 3)
    before = target.copy()
    transport.set_target(target, kp, kd)
    tick(transport)
    assert kp == [1.0] * 17 and kd == [0.1] * 17
    np.testing.assert_array_equal(target, before)
    for _, command in transport.client._client.sent:
        assert command["kp"] == [1.0] * 7
        assert len(command["positions"]) == 7
    target[16] = 0.1
    with pytest.raises(ValueError, match="inactive"):
        transport.set_target(target, kp, kd)


def test_nan_target_skips_only_its_arm_and_services_keep_named_wire_order(follower):
    transport, clock, _ = follower
    feed_all(transport, clock.now)
    transport.enable()
    tick(transport)
    assert [command["motor_ids"] for _, command in transport.client._client.sent] == [
        list(transport.session.arms[arm].joints) for arm in ("left", "right")]
    transport.client._client.sent.clear()
    target = np.array([0.2] * 14 + [0.0] * 3)
    target[0] = np.nan
    transport.set_target(target)
    tick(transport)
    assert [arm for arm, _ in transport.client._client.sent] == ["right"]
    transport.disable()
    tick(transport)
    assert not transport.client._held
    transport.mech_zero(save=True)
    tick(transport)
    assert all(command["type"] == "mech_zero" and command["save"] for _, command in transport.client._client.sent[-2:])


def test_dry_run_adapter_rejects_motor_submission(follower):
    _, _, module = follower
    readonly = module.DualArmTransport(None, left_cmd="tcp://left:5555", left_telem="tcp://left:5556",
                                      right_cmd="tcp://right:5575", right_telem="tcp://right:5576", receive_only=True)
    with pytest.raises(RuntimeError, match="receive-only"):
        readonly.enable()
    with pytest.raises(RuntimeError, match="receive-only"):
        readonly.set_target([0.0] * 17)
    assert not readonly.client._client.sent
