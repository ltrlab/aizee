"""Collector compatibility contracts; no robot sockets or motors."""
from pathlib import Path
import threading

import numpy as np
import pytest


@pytest.fixture
def legacy(monkeypatch):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root / "python"))
    monkeypatch.syspath_prepend(str(root / "python" / "scripts"))
    from collect_demo_app import alignment, session, telem, zmq_io
    monkeypatch.setattr(alignment, "_ALIGN_SIGNS", np.array([-1, 1, -1, 1, -1, 1, -1]))
    return session, telem, zmq_io


def test_collector_uses_resolved_endpoints_and_shared_freshness(legacy, monkeypatch):
    module, _, _ = legacy

    class Client:
        def __init__(self, session, *, context):
            self.session, self.context = session, context
            self.messages = []
            self.started = self.closed = False

        def start(self):
            self.started = True

        def close(self):
            self.closed = True

        def submit(self, commands):
            self.messages.append(commands)

    monkeypatch.setattr(module, "ThreadedRobotClient", Client)
    monkeypatch.setenv("ROBOT_PLATFORM_SETTINGS_JSON", '{"control":{"stale_after_s":0.125}}')
    context = object()
    transport = module.AizeeSessionTransport(context, command="tcp://127.0.0.1:6011",
                                              telemetry="tcp://127.0.0.1:6012")
    assert transport.client.context is context
    assert transport.client.started
    assert transport.session.max_age_s == 0.125
    assert transport.session.arms["main"].command == "tcp://127.0.0.1:6011"
    assert transport.session.arms["main"].telemetry == "tcp://127.0.0.1:6012"
    transport.send_message({"type": "disable", "motor_ids": ["gripper"]})
    assert transport.client.messages == [{"main": {"type": "disable", "motor_ids": ["gripper"]}}]
    transport.close()
    assert transport.client.closed


def test_cache_keeps_original_receive_time_when_read_repeatedly(legacy, monkeypatch):
    module, _, _ = legacy

    class Client:
        def telemetry_sample(self, arm):
            assert arm == "main"
            return {"motors": {}}, 10.0

    monkeypatch.setattr(module.time, "monotonic", lambda: 10.2)
    monkeypatch.setattr(module.time, "time", lambda: 1000.0)
    cache, lock = {}, threading.Lock()
    module._refresh_telem_cache(Client(), lock, cache)
    assert cache["received_at"] == 10.0
    assert cache["time"] == pytest.approx(999.8)
    monkeypatch.setattr(module.time, "monotonic", lambda: 11.0)
    monkeypatch.setattr(module.time, "time", lambda: 5000.0)
    module._refresh_telem_cache(Client(), lock, cache)
    assert cache["time"] == pytest.approx(999.8)


@pytest.mark.parametrize("kind", ["arm", "bundle", "helper"])
def test_calibration_applied_once_before_session_submit(legacy, kind):
    _, _, module = legacy

    class Sink:
        def send_message(self, message):
            self.message = message

    arm = {"type": "arm_joints", "positions": [0.1] * 7,
           "velocities": [0.2] * 7, "torques": [0.3] * 7, "kp": [1.0] * 7}
    drive = {"linear": 0.4, "angular": 0.0}
    sink = Sink()
    if kind == "arm":
        module._send(sink, arm)
        actual = sink.message
    elif kind == "bundle":
        module._send(sink, {"type": "bundle", "arm_joints": arm, "drive": drive})
        actual = sink.message["arm_joints"]
        assert sink.message["drive"] == drive
    else:
        module._send_bundle(sink, arm, drive)
        actual = sink.message["arm_joints"]
        assert sink.message["drive"] == drive
    signs = np.array([-1, 1, -1, 1, -1, 1, -1])
    for field, value in (("positions", 0.1), ("velocities", 0.2), ("torques", 0.3)):
        assert actual[field] == pytest.approx(signs * value)
        assert arm[field] == [value] * 7  # caller's URDF data was not mutated
    assert actual["kp"] == [1.0] * 7


@pytest.mark.parametrize("bad", [None, True, "0", float("nan"), float("inf")])
def test_partial_or_invalid_positions_never_become_measured_zeros(legacy, bad):
    _, module, _ = legacy
    from common.arm_constants import ARM_JOINTS
    message = {"motors": {name: {"position": 0.25} for name in ARM_JOINTS}}
    message["motors"][ARM_JOINTS[-1]]["position"] = bad
    assert module._qpos(message) is None
    del message["motors"][ARM_JOINTS[-1]]
    assert module._qpos_motor(message) is None
    message["motors"][ARM_JOINTS[-1]] = {"position": 0.25}
    assert module._qpos(message) == pytest.approx(np.array([-1, 1, -1, 1, -1, 1, -1]) * 0.25)


def test_clear_during_tracking_calculation_does_not_submit_old_tick(legacy, monkeypatch):
    _, _, module = legacy
    calculating, release = threading.Event(), threading.Event()

    def compute(*args):
        calculating.set()
        assert release.wait(2.0)
        return {"type": "bundle", "arm_joints": {"positions": [0.1] * 7}}

    class Sink:
        messages = []

        def send_message(self, message):
            self.messages.append(message)

    monkeypatch.setattr(module, "_compute_tracking_bundle", compute)
    sink = Sink()
    stop, thread, lock, holder = module._start_cmd_sender(
        sink, threading.Lock(), {}, {}, threading.Lock(), {})
    try:
        with lock:
            holder.update(mode="tracking", tracking={})
        assert calculating.wait(2.0)
        with lock:
            holder.update(mode=None, tracking=None, bundle=None)
        stop.set()
        release.set()
        thread.join(2.0)
        assert not thread.is_alive()
        assert sink.messages == []
    finally:
        release.set()
        stop.set()
        thread.join(2.0)


def test_invalid_display_fields_remain_missing_without_crashing(legacy):
    _, module, _ = legacy
    from common.arm_constants import ARM_JOINTS
    motors = {ARM_JOINTS[0]: {"torque": 1.5, "temperature": 30.0},
              ARM_JOINTS[1]: {"torque": "bad", "temperature": float("inf")},
              ARM_JOINTS[2]: None}
    torque = module._qtorque({"motors": motors})
    temperature = module._qtemp({"motors": motors})
    assert torque[0] == 1.5 and temperature[0] == 30.0
    assert np.isnan(torque[1:]).all() and np.isnan(temperature[1:]).all()
