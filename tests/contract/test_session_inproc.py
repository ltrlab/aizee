"""Real ZMQ in-process transport: no network interfaces or physical hardware."""
from pathlib import Path
import time

import pytest
import zmq

from robot_platform.control import RobotSession, ThreadedRobotClient
from robot_platform.robots import load_robot
from robot_platform.transport.wire import pack_msg, unpack_msg


def test_aizee_calibration_and_freshness_through_real_worker(monkeypatch):
    root = Path(__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(root / "python"))
    monkeypatch.syspath_prepend(str(root / "python/scripts"))
    from collect_demo_app import alignment, zmq_io
    import numpy as np
    monkeypatch.setattr(alignment, "_ALIGN_SIGNS", np.array([-1, 1, -1, 1, -1, 1, -1]))
    spec = load_robot("aizee")
    session = RobotSession(spec, {"endpoints": {"arms": {"main": {
        "command": "inproc://test-command", "telemetry": "inproc://test-telemetry",
        "joints": list(spec.joint_names),
    }}}, "control": {"stale_after_s": 0.1}})
    context = zmq.Context()
    pull, pub = context.socket(zmq.PULL), context.socket(zmq.PUB)
    pull.bind("inproc://test-command")
    pub.bind("inproc://test-telemetry")
    client = ThreadedRobotClient(session, context=context)

    class Boundary:
        def send_message(self, message):
            client.submit({"main": message})

    frame = pack_msg({"motors": {name: {"position": 0.25, "torque": 0.1}
                                  for name in spec.joint_names}})
    try:
        client.start()
        deadline = time.monotonic() + 3.0
        while not client.arm_ready()["main"] and time.monotonic() < deadline:
            pub.send(frame)
            time.sleep(0.005)
        assert client.arm_ready()["main"]
        assert not pull.poll(0)  # connect/telemetry cannot enable or move anything
        zmq_io._send_bundle(Boundary(), {"positions": [0.2] * 7, "velocities": [0.0] * 7,
                                         "kp": [1.0] * 7, "kd": [0.1] * 7})
        assert pull.poll(1000)
        command = unpack_msg(pull.recv())
        assert command["arm_joints"]["positions"] == pytest.approx([-0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2])
        # Let the original receive time expire, then prove the actual socket
        # boundary strips arm motion while still transmitting a drive stop.
        time.sleep(0.13)
        assert not client.arm_ready()["main"]
        zmq_io._send_bundle(Boundary(), {"positions": [0.3] * 7, "velocities": [0.0] * 7},
                            {"linear": 0.0, "angular": 0.0, "kp": 0.0, "kd": 3.0})
        assert pull.poll(1000)
        command = unpack_msg(pull.recv())
        assert command == {"type": "bundle", "drive": {"linear": 0.0, "angular": 0.0, "kp": 0.0, "kd": 3.0}}
        assert client.error is None
    finally:
        client.close()
        pull.close(linger=0)
        pub.close(linger=0)
        context.term()
