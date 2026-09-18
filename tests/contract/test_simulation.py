import io
import json
import math
from pathlib import Path
import time

import pytest

from robot_platform.control import RobotSession, ZmqRobotClient
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings
from robot_platform.simulation import SimulatedRobot
from robot_platform.simulation.server import ProtocolServer
from robot_platform.data.traces import TraceWriter, read_events
from robot_platform.transport.wire import unpack_camera, unpack_msg


@pytest.mark.parametrize("robot", ["aizee", "minerva"])
def test_deterministic_dynamics_and_no_implicit_enable(robot):
    spec = load_robot(robot)
    plants = [SimulatedRobot(spec, load_settings(spec), seed=12) for _ in range(2)]
    for plant in plants:
        arm = next(iter(plant.session.arms))
        names = plant.session.arms[arm].joints
        cmd = {"type": "arm_joints", "positions": [0.3]*len(names), "velocities": [0.0]*len(names),
               "kp": [2.0]*len(names), "kd": [0.4]*len(names)}
        plant.command(arm, cmd)
        plant.step(0.1)
        assert all(j.q == 0 for j in plant.joints.values())
        plant.command(arm, {"type": "enable", "motor_ids": list(names)})
        plant.command(arm, cmd)
        plant.step(0.5)
        assert 0 < plant.joints[names[0]].q < 0.5
        plant.command(arm, {"type": "emergency_stop"})
        plant.command(arm, {"type": "clear_emergency_stop"})
        assert not any(plant.joints[n].enabled for n in names)
    assert plants[0].snapshot() == plants[1].snapshot()


@pytest.mark.parametrize("robot", ["aizee", "minerva"])
def test_real_loopback_commands_telemetry_cameras_and_trace(tmp_path, robot, monkeypatch):
    import zmq
    from PIL import Image
    monkeypatch.setattr("robot_platform.simulation.server.source_identity", lambda: {"test": True})
    output = tmp_path / robot
    with ProtocolServer(robot, output) as server:
        session = RobotSession(server.spec, server.settings)
        client = ZmqRobotClient(session)
        camera = server.context.socket(zmq.SUB)
        camera.setsockopt(zmq.SUBSCRIBE, b"")
        camera.connect(next(iter(server.settings["endpoints"]["cameras"].values())))
        client.connect()
        try:
            deadline = time.monotonic() + 3
            while not session.observation().ready and time.monotonic() < deadline:
                server.cycle()
                time.sleep(0.005)
                client.poll()
            assert session.observation().ready
            assert all(not j.enabled for j in server.plant.joints.values())
            for arm, ep in session.arms.items():
                client.send_commands({arm: {"type": "enable", "motor_ids": list(ep.joints)}})
            for _ in range(10):
                server.cycle()
                time.sleep(0.005)
                client.poll()
            q = [0.15 if j.active else 0.0 for j in server.spec.joints]
            gains = [2.0 if j.active else 0.0 for j in server.spec.joints]
            client.send_target(q, kp=gains)
            for _ in range(10):
                server.cycle()
                time.sleep(0.005)
                client.poll()
            assert all(j.q > 0 for j in server.plant.joints.values())
            assert camera.poll(1000)
            msg = unpack_camera(camera.recv_multipart())
            assert msg["simulation"] is True
            assert Image.open(io.BytesIO(msg["color"]["data_bytes"])).size == (640, 480)
            assert all(url.startswith("tcp://127.0.0.1:") for url in server.settings["endpoints"]["cameras"].values())
            assert str(output) in server.settings["applications"]["collect"]["output_dir"]
        finally:
            client.close()
            camera.close(linger=0)
    events = list(read_events(output / "trace.h5"))
    assert any(e["kind"] == "command_receive" for e in events)
    assert any(e["kind"] == "plant_state" for e in events)
    assert json.loads((output / "manifest.json").read_text())["evidence"] == "synthetic"


@pytest.mark.parametrize("scenario", ["feedback-loss", "frozen-feedback", "malformed-feedback"])
def test_fault_scenarios_make_only_affected_arm_unready(tmp_path, monkeypatch, scenario):
    monkeypatch.setattr("robot_platform.simulation.server.source_identity", lambda: {})
    with ProtocolServer("minerva", tmp_path/scenario, scenario=scenario, cameras=False) as server:
        session = RobotSession(server.spec, server.settings)
        for _ in range(100):
            server.cycle()
            for arm in session.arms:
                msg = server.plant.telemetry(arm)
                if msg is not None:
                    session.ingest_telemetry(arm, msg, received_at=server.plant.t)
        assert session.arm_ready(now=server.plant.t) == {"left": False, "right": True}


def test_trace_preserves_event_times_and_rejects_overwrite(tmp_path):
    path = tmp_path / "trace.h5"
    writer = TraceWriter(path, clock_domain="test", clock=lambda: 12.5)
    writer.record("sample", {"raw_hex": "ff00"}, time_s=12.25)
    writer.record("boundary", {})
    writer.close()
    assert [event["time_s"] for event in read_events(path)] == [12.25, 12.5]
    with pytest.raises(RuntimeError):
        TraceWriter(path, clock_domain="test")


def test_simulated_minerva_episode_loads_in_training_reader(tmp_path, monkeypatch):
    import numpy as np
    import h5py
    from robot_platform.data.formats.minerva import save_minerva_episode
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root))
    from python.training.minerva_dataset import MinervaEpisodeDataset
    spec = load_robot("minerva")
    plant = SimulatedRobot(spec, load_settings(spec))
    positions, torques = [], []
    for _ in range(8):
        plant.step(0.02)
        for arm in plant.session.arms:
            plant.session.ingest_telemetry(arm, plant.telemetry(arm), received_at=plant.t)
        obs = plant.session.observation(now=plant.t)
        positions.append(np.nan_to_num(obs.positions).astype(np.float32))
        torques.append(np.nan_to_num(obs.torques).astype(np.float32))
    cameras = {c.name: [np.zeros((16, 24, 3), np.uint8) for _ in positions] for c in spec.cameras}
    monkeypatch.setenv("ROBOT_PLATFORM_SETTINGS_JSON", json.dumps({"simulation": {"run_id": "test", "evidence": "synthetic"}}))
    path, count = save_minerva_episode(tmp_path, positions, cameras, torque_buf=torques)
    with h5py.File(path, "r") as file:
        assert file.attrs["data_source"] == "simulation"
    dataset = MinervaEpisodeDataset([path], chunk_size=4, state_mode="qpos", augment=False)
    obs, actions = dataset[0]
    assert count == 8 and actions.shape == (4, 17)
    assert obs["state"].shape == (17,)
    assert set(obs["images"]) == set(cameras)


def test_estop_fanout_retries_failed_peer_and_uses_msgpack():
    import zmq
    from robot_platform.control.estop import EstopFanout, parse_state
    class Socket:
        def __init__(self, fail=False):
            self.fail, self.sent = fail, []
        def send(self, frame, flags):
            if self.fail:
                raise zmq.Again()
            self.sent.append(unpack_msg(frame))
    left, right = Socket(True), Socket()
    bridge = EstopFanout({"left": left, "right": right})
    bridge.update(True)
    assert "left" in bridge.flush()
    assert right.sent == [{"type": "emergency_stop"}]
    left.fail = False
    bridge.flush()
    assert left.sent == [{"type": "emergency_stop"}]
    bridge.update(False)
    bridge.flush()
    assert left.sent[-1] == right.sent[-1] == {"type": "clear_emergency_stop"}
    assert parse_state('{"estop": "false"}') is None
    assert parse_state('# nc=0 no=1') is True


def test_simulation_cannot_be_repointed_to_hardware():
    from robot_platform.contracts import ConfigurationError
    spec = load_robot("minerva")
    sim = {"run_id": "test", "evidence": "synthetic"}
    with pytest.raises(ConfigurationError, match="host=127.0.0.1"):
        load_settings(spec, overrides={"simulation": sim})
    with pytest.raises(ConfigurationError, match="simulation endpoints"):
        load_settings(spec, overrides={"host": "127.0.0.1", "simulation": sim,
            "endpoints": {"arms": {"left": {"command": "tcp://192.168.0.27:5555"}}}})


def test_simulated_teleop_never_opens_leaders_even_on_reconnect(monkeypatch):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root / "python"))
    monkeypatch.syspath_prepend(str(root / "python/scripts"))
    from collect_minerva_app import teleop
    def forbidden():
        pytest.fail("simulation must not import or discover hardware drivers")
    monkeypatch.setattr(teleop, "_import_openrb", forbidden)
    instance = teleop.MinervaTeleop(left_port="COM1", hardware_enabled=False)
    assert instance.connect() == instance.connect() == (False, False)


def test_invalid_bundle_does_not_partially_apply_drive():
    spec = load_robot("aizee")
    plant = SimulatedRobot(spec, load_settings(spec))
    plant.command("main", {"type": "enable", "motor_ids": ["left_wheel", "right_wheel"]})
    with pytest.raises(ValueError, match="envelope"):
        plant.command("main", {"type": "bundle", "drive": {"linear": 1, "angular": 0},
            "arm_joints": {"positions": [3.0]*7, "velocities": [0.0]*7}})
    assert plant.drive == {"linear": 0.0, "angular": 0.0}


def test_aizee_collector_motor_roster_and_drive_lifecycle():
    spec = load_robot("aizee")
    plant = SimulatedRobot(spec, load_settings(spec))
    drive = {"type": "drive", "linear": 0.1, "angular": 0.0}
    plant.command("main", drive)
    plant.step(0.1)
    assert plant.base_pose == [0, 0, 0]
    plant.command("main", {"type": "enable", "motor_ids": ["left_wheel", "right_wheel", *spec.active_joint_names]})
    plant.command("main", drive)
    plant.step(0.1)
    assert plant.base_pose[0] > 0
    plant.command("main", {"type": "emergency_stop"})
    plant.command("main", {"type": "clear_emergency_stop"})
    before = plant.base_pose[:]
    plant.command("main", drive)
    plant.step(0.1)
    assert plant.base_pose == before
