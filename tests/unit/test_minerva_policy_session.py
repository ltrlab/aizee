"""Policy/backend integration with synthetic telemetry and fake sockets only."""

from copy import deepcopy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

torch = pytest.importorskip("torch")
yaml = pytest.importorskip("yaml")
zmq = pytest.importorskip("zmq")

from robot_platform.control import RobotSession
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings
from robot_platform.transport.wire import pack_camera, pack_msg, unpack_msg


REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def policy_node():
    spec = importlib.util.spec_from_file_location("tested_minerva_policy", REPO / "python/nodes/minerva_policy_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def policy_config(monkeypatch):
    monkeypatch.delenv("ROBOT_PLATFORM_SETTINGS_JSON", raising=False)
    return yaml.safe_load((REPO / "config/minerva.yaml").read_text(encoding="utf-8"))


def new_session(max_age_s=0.2):
    spec = load_robot("minerva")
    return RobotSession(spec, load_settings(spec, overrides={"host": "localhost"}), max_age_s=max_age_s)


def arm_message(session, side, *, torque=True, position=0.0):
    return {"motors": {
        name: {"position": position + index / 100, **({"torque": index / 10} if torque else {})}
        for index, name in enumerate(session.arms[side].joints)
    }}


def test_actual_dual_arm_telemetry_becomes_model_state(policy_node):
    session = new_session()
    session.ingest_telemetry("left", arm_message(session, "left"), received_at=10.0)
    session.ingest_telemetry("right", arm_message(session, "right", position=1.0), received_at=10.1)
    observation = session.observation(now=10.15)
    qpos, torques = policy_node.model_observation(observation, "qpos_qcmd_tq")
    np.testing.assert_allclose(qpos[:7], np.arange(7) / 100)
    np.testing.assert_allclose(qpos[7:14], 1 + np.arange(7) / 100)
    np.testing.assert_array_equal(qpos[14:], np.zeros(3))
    np.testing.assert_allclose(torques[:7], np.arange(7) / 10)
    assert all(np.isnan(value) for value in observation.positions[14:])


@pytest.mark.parametrize("mode", ["qpos", "qpos_qcmd", "qpos_qcmd_tq"])
def test_either_missing_or_stale_arm_rejects_entire_policy_state(policy_node, mode):
    session = new_session()
    session.ingest_telemetry("left", arm_message(session, "left"), received_at=10.0)
    assert policy_node.model_observation(session.observation(now=10.1), mode) is None
    session.ingest_telemetry("right", arm_message(session, "right"), received_at=10.0)
    session.ingest_telemetry("left", arm_message(session, "left"), received_at=10.3)
    assert policy_node.model_observation(session.observation(now=10.31), mode) is None


def test_position_only_models_do_not_require_torque_and_missing_position_is_never_zero_filled(policy_node):
    session = new_session()
    for side in session.arms:
        session.ingest_telemetry(side, arm_message(session, side, torque=False), received_at=10.0)
    observation = session.observation(now=10.1)
    assert policy_node.model_observation(observation, "qpos") is not None
    assert policy_node.model_observation(observation, "qpos_qcmd") is not None
    assert policy_node.model_observation(observation, "qpos_qcmd_tq") is None
    broken = arm_message(session, "right")
    del broken["motors"]["right_arm_j3"]["position"]
    session.ingest_telemetry("right", broken, received_at=10.1)
    assert policy_node.model_observation(session.observation(now=10.1), "qpos") is None


def test_policy_targets_reject_unsupported_motion_and_nonfinite_values(policy_node):
    session = new_session()
    action = np.zeros(17, dtype=np.float32)
    action[14:] = 5e-7
    rounded = policy_node.policy_target(action, session)
    np.testing.assert_array_equal(rounded[14:], [0, 0, 0])
    assert np.all(action[14:] != 0)  # caller's chunk is not modified
    action[14] = 0.1
    with pytest.raises(ValueError, match="head_pan"):
        policy_node.policy_target(action, session)
    action[14] = np.nan
    with pytest.raises(ValueError, match="finite"):
        policy_node.policy_target(action, session)


def test_default_policy_config_addresses_both_real_arm_processes(policy_node, policy_config):
    original = deepcopy(policy_config)
    spec, settings = policy_node.policy_settings(policy_config)
    assert spec.name == "minerva"
    assert settings["endpoints"]["arms"]["left"]["command"] == "tcp://localhost:5555"
    assert settings["endpoints"]["arms"]["right"]["command"] == "tcp://localhost:5575"
    assert settings["endpoints"]["arms"]["right"]["telemetry"] == "tcp://localhost:5576"
    assert settings["control"]["stale_after_s"] == 0.2
    assert policy_config == original


def test_host_and_endpoint_overrides_preserve_separate_camera_hosts(policy_node, policy_config):
    policy_config["endpoints"]["cameras"]["head"] = "tcp://scene-host:6004"
    _, settings = policy_node.policy_settings(policy_config, host="bench", endpoint_overrides={
        "right": {"command": "tcp://second-controller:6675", "telemetry": "tcp://second-controller:6676"}})
    assert settings["endpoints"]["arms"]["left"]["command"] == "tcp://bench:5555"
    assert settings["endpoints"]["arms"]["right"]["command"] == "tcp://second-controller:6675"
    assert settings["endpoints"]["cameras"]["left_wrist"] == "tcp://bench:5563"
    assert settings["endpoints"]["cameras"]["head"] == "tcp://scene-host:6004"


def test_policy_yaml_can_explicitly_define_both_arm_endpoints(policy_node, policy_config):
    policy_config["endpoints"] = {
        "arms": {
            "left": {"command": "tcp://robot-a:6000", "telemetry": "tcp://robot-a:6001"},
            "right": {"command": "tcp://robot-a:6010", "telemetry": "tcp://robot-a:6011"},
        },
        "cameras": {name: f"tcp://robot-a:{port}" for name, port in zip(policy_node._CAMERAS, (5563, 5565, 5564))},
    }
    _, settings = policy_node.policy_settings(policy_config, host="robot-b")
    assert settings["endpoints"]["arms"]["left"]["command"] == "tcp://robot-b:6000"
    assert settings["endpoints"]["arms"]["right"]["telemetry"] == "tcp://robot-b:6011"
    assert settings["endpoints"]["cameras"]["head"] == "tcp://robot-b:5564"


def test_launcher_settings_reach_policy_and_explicit_host_retargets_them(policy_node, policy_config, monkeypatch):
    launcher = load_settings(load_robot("minerva"), overrides={"host": "selected", "control": {"stale_after_s": 0.15}})
    monkeypatch.setenv("ROBOT_PLATFORM_SETTINGS_JSON", json.dumps(launcher))
    _, settings = policy_node.policy_settings(policy_config)
    assert settings == launcher
    _, settings = policy_node.policy_settings(policy_config, host="explicit")
    assert settings["endpoints"]["arms"]["right"]["command"] == "tcp://explicit:5575"


def test_ordered_settings_overlays_control_both_arms(policy_node, policy_config, monkeypatch):
    overlays = {"one.json": {"host": "first"}, "two.json": {"host": "second", "control": {"stale_after_s": 0.12}}}
    read_text = Path.read_text
    monkeypatch.setattr(Path, "read_text", lambda path, *a, **kw: json.dumps(overlays[path.name]) if path.name in overlays else read_text(path, *a, **kw))
    _, settings = policy_node.policy_settings(policy_config, settings_paths=["one.json", "two.json"])
    assert settings["host"] == "second"
    assert settings["endpoints"]["arms"]["right"]["command"] == "tcp://second:5575"
    assert settings["control"]["stale_after_s"] == 0.12


class FakeSocket:
    def __init__(self, context, kind):
        self.context, self.kind = context, kind
        self.address = None
        self.sent = []
        self.frames = []
        self.closed = False

    def setsockopt(self, *args):
        pass

    def setsockopt_string(self, *args):
        pass

    def connect(self, address):
        self.address = address
        port = int(address.rsplit(":", 1)[1])
        if self.kind == zmq.SUB:
            if port in (5556, 5576):
                side = "left" if port == 5556 else "right"
                if side not in self.context.missing:
                    self.frames.append(pack_msg(arm_message(new_session(), side, torque=self.context.torque)))
            else:
                self.frames.append(pack_camera({"color": {"format": "jpeg"}}, b"fake"))

    def recv(self, flags):
        if not self.frames:
            raise zmq.Again()
        return self.frames.pop(0)

    def recv_multipart(self, flags):
        return self.recv(flags)

    def send(self, frame, flags):
        self.sent.append(unpack_msg(frame))

    def close(self, *args, **kwargs):
        self.closed = True


class FakeContext:
    def __init__(self, missing=(), torque=True):
        self.sockets = []
        self.missing = missing
        self.torque = torque
        self.terminated = False

    def socket(self, kind):
        sock = FakeSocket(self, kind)
        self.sockets.append(sock)
        return sock

    def term(self):
        self.terminated = True


def run_fake_tick(policy_node, monkeypatch, *, dry_run=False, missing=(), torque=True,
                  mode="qpos_qcmd", lose_during_inference=False,
                  refresh_telemetry_after_inference=False):
    context = FakeContext(missing=missing, torque=torque)
    clock = [10.0]
    calls = []

    class FakePolicy:
        def select_action(self, state, images, **kwargs):
            calls.append(state)
            if lose_during_inference:
                clock[0] += 0.3
                if refresh_telemetry_after_inference:
                    for sock in context.sockets:
                        if sock.kind == zmq.SUB and sock.address.endswith((":5556", ":5576")):
                            side = "left" if sock.address.endswith(":5556") else "right"
                            sock.frames.append(pack_msg(arm_message(new_session(), side, torque=torque)))
            return torch.zeros((1, 2, 17))

    stats = {f"{field}_{stat}": np.full(17, value, dtype=np.float32)
             for field in ("qpos", "qcmd", "torque") for stat, value in (("mean", 0), ("std", 1))}
    stats.update(action_lo=np.zeros(17, np.float32), action_hi=np.zeros(17, np.float32))
    checkpoint = {"num_joints": 17, "state_mode": mode, "action_mode": "absolute", "chunk_size": 2, "lang_dim": 0}
    monkeypatch.setattr(policy_node, "load_minerva_checkpoint", lambda *args: (FakePolicy(), stats, checkpoint))
    monkeypatch.setattr(policy_node.zmq, "Context", lambda: context)
    monkeypatch.setattr(policy_node, "decode_image", lambda *args: np.zeros((4, 4, 3), dtype=np.uint8))
    monkeypatch.setattr(policy_node.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(policy_node.time, "sleep", lambda duration: (_ for _ in ()).throw(KeyboardInterrupt()))
    argv = ["--config", str(REPO / "config/minerva.yaml"), "--checkpoint", "fake.pt", "--device", "cpu"]
    if dry_run:
        argv.append("--dry-run")
    policy_node.main(argv)
    assert context.terminated and all(sock.closed for sock in context.sockets)
    return context, calls


def test_policy_main_sends_two_seven_joint_commands_through_shared_backend(policy_node, policy_config, monkeypatch):
    context, calls = run_fake_tick(policy_node, monkeypatch)
    assert len(calls) == 1
    pushes = [sock for sock in context.sockets if sock.kind == zmq.PUSH]
    assert [sock.address for sock in pushes] == ["tcp://localhost:5555", "tcp://localhost:5575"]
    for index, sock in enumerate(pushes):
        assert len(sock.sent) == 1
        command = sock.sent[0]
        assert command["type"] == "arm_joints"
        assert len(command["positions"]) == 7
        assert command["kp"] == policy_node.KP[index * 7:index * 7 + 7]
        assert "joint_names" not in command  # Rust consumes positional arrays


def test_policy_dry_run_constructs_no_command_sockets(policy_node, policy_config, monkeypatch):
    context, calls = run_fake_tick(policy_node, monkeypatch, dry_run=True)
    assert len(calls) == 1
    assert len(context.sockets) == 5  # two telemetry + three cameras
    assert all(sock.kind == zmq.SUB and not sock.sent for sock in context.sockets)


@pytest.mark.parametrize("missing", [("left",), ("right",)])
def test_policy_loop_never_infers_or_sends_with_one_arm_missing(policy_node, policy_config, monkeypatch, missing):
    context, calls = run_fake_tick(policy_node, monkeypatch, missing=missing)
    assert not calls
    assert all(not sock.sent for sock in context.sockets)


def test_policy_rechecks_freshness_after_inference(policy_node, policy_config, monkeypatch):
    context, calls = run_fake_tick(policy_node, monkeypatch, lose_during_inference=True)
    assert len(calls) == 1
    assert all(not sock.sent for sock in context.sockets)


def test_policy_rejects_expired_images_even_when_arm_telemetry_refreshes(policy_node, policy_config, monkeypatch):
    context, calls = run_fake_tick(policy_node, monkeypatch, lose_during_inference=True,
                                   refresh_telemetry_after_inference=True)
    assert len(calls) == 1
    assert all(not sock.sent for sock in context.sockets)


@pytest.mark.parametrize(("mode", "sends"), [("qpos_qcmd", True), ("qpos_qcmd_tq", False)])
def test_policy_loop_respects_models_torque_requirement(policy_node, policy_config, monkeypatch, mode, sends):
    context, calls = run_fake_tick(policy_node, monkeypatch, torque=False, mode=mode)
    assert bool(calls) is sends
    assert any(sock.sent for sock in context.sockets) is sends
