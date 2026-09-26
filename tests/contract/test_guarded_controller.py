"""Python-to-Rust MessagePack contract and coordinated session fault tests."""
from contextlib import ExitStack

import pytest

from robot_platform.control.session import RobotSession
from robot_platform.control.guarded import GuardedBatchError, GuardedControllerClient, GuardedSessionClient, GuardRejected
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary


pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file(), reason="Build control_harness with scripts/test_control_core.ps1 or cargo build -p robot_control")]


def config(names=("joint",), group="fixture"):
    return {"group": group,
            "identity": {"model": "synthetic-model", "calibration": "synthetic-calibration", "collision_policy": "synthetic-policy"},
            "lease_ms": 100, "feedback_timeout_ms": 20, "settled_velocity": 0.001,
            "joints": [{"name": name, "unit": "radians", "lower": -3.0, "upper": 3.0,
                        "max_velocity": 2.0, "max_effort": 5.0, "max_kp": 20.0, "max_kd": 5.0,
                        "stop_acceleration": 2.0, "hold_kp": 10.0, "hold_kd": 1.0,
                        "max_tracking_error": 0.5, "max_temperature": 70.0} for name in names]}


def client(harness, cfg):
    return GuardedControllerClient(harness.exchange, group=cfg["group"], identity=cfg["identity"],
                                   joint_names=tuple(j["name"] for j in cfg["joints"]))


def command(count=1, position=0.1):
    return [{"position": position, "velocity": 0.2, "effort": 0.0, "kp": 10.0, "kd": 1.0} for _ in range(count)]


def test_native_lease_expires_without_any_more_client_commands():
    cfg = config()
    with NativeControlHarness(cfg, [0.0]) as h:
        c = client(h, cfg)
        assert c.acquire()["state"] == "disarmed"
        assert not h.last["hardware_enabled"]
        c.arm()
        c.move(command())
        assert h.step(100)["status"]["state"] == "stopping"
        for now in (150, 200, 201):
            result = h.step(now)
        assert result["status"]["state"] == "stopped"
        assert result["status"]["joints"][0]["position"] == pytest.approx(0.11)
        c.poll()
        c.acquire()
        with pytest.raises(GuardRejected, match="invalid_state"):
            c.arm()
        c.reset()
        assert not h.last["hardware_enabled"]
        assert c.arm()["state"] == "holding"


def test_native_renewal_is_not_a_target_keepalive():
    cfg = config()
    with NativeControlHarness(cfg, [0.0]) as h:
        c = client(h, cfg)
        c.acquire(); c.arm(); c.move(command())
        h.step(50)
        renewed = c.renew()
        assert renewed["lease"]["expires_at_ms"] == 150
        assert renewed["command_expires_at_ms"] == 100
        assert h.step(100)["status"]["reason"] == "command_expired"


def test_native_epoch_and_schema_reject_old_or_legacy_traffic():
    cfg = config()
    with NativeControlHarness(cfg, [0.0]) as old:
        c = client(old, cfg); c.acquire(); c.arm()
        authority = c._authority()
    with NativeControlHarness(cfg, [0.0]) as new:
        reply = new.step(0, {"type": "arm", "authority": authority})["reply"]
        assert not reply["accepted"] and reply["reason"] == "wrong_epoch"
        with pytest.raises(ValueError, match="unknown variant"):
            new.step(0, {"type": "arm_joints", "positions": [0.1], "velocities": [0.0]})
        assert new.last["status"]["state"] == "faulted"
        assert not new.last["hardware_enabled"]


@pytest.mark.parametrize("fault,reason", [("frozen", "feedback_stale"), ("fail_read", "hardware_fault"),
                                         ("fail_write", "hardware_fault"), ("motor_fault", "hardware_fault")])
def test_native_faults_latch(fault, reason):
    cfg = config()
    with NativeControlHarness(cfg, [0.0]) as h:
        c = client(h, cfg); c.acquire(); c.arm()
        output = h.step(20, faults={fault: True})
        assert output["status"]["reason"] == reason
        assert output["status"]["lease"] is None
        assert not output["hardware_enabled"]
        assert h.step(21, faults={})["status"]["state"] == "faulted"


def session_fixture(stack, robot):
    spec = load_robot(robot)
    settings = load_settings(spec, overrides={"host": "127.0.0.1"})
    settings["simulation"] = {"evidence": "synthetic"}
    settings["control"]["require_source_freshness"] = True
    session = RobotSession(spec, settings)
    harnesses, clients = {}, {}
    for arm, endpoint in session.arms.items():
        cfg = config(endpoint.joints, arm)
        h = stack.enter_context(NativeControlHarness(cfg, [0.0] * len(endpoint.joints)))
        harnesses[arm], clients[arm] = h, client(h, cfg)
        session.ingest_telemetry(arm, h.telemetry(), received_at=0.0)
    return session, harnesses, clients, GuardedSessionClient(session, clients)


def routed_commands(session):
    active = session.active_mask
    return session.route_target([0.1 if a else 0.0 for a in active], now=0,
                                velocities=[0.0] * len(active),
                                kp=[10.0 if a else 0.0 for a in active],
                                kd=[1.0 if a else 0.0 for a in active], torques=[0.0] * len(active))


@pytest.mark.parametrize("robot", ["aizee", "minerva"])
def test_existing_robot_session_routes_into_native_generic_core(robot):
    with ExitStack() as stack:
        session, harnesses, _, guarded = session_fixture(stack, robot)
        guarded.acquire(); guarded.arm()
        results = guarded.send_commands(routed_commands(session), now=0)
        assert all(result["state"] == "executing" for result in results.values())
        assert all(all(j["position"] == pytest.approx(0.1) for j in h.last["status"]["joints"])
                   for h in harnesses.values())


def test_partial_acceptance_attempts_stop_on_both_arms():
    with ExitStack() as stack:
        session, harnesses, clients, guarded = session_fixture(stack, "minerva")
        guarded.acquire(); guarded.arm()
        arms = list(clients)
        # Second controller faults after acquisition; the first accepts motion.
        harnesses[arms[1]].step(1, faults={"motor_fault": True})
        with pytest.raises(GuardedBatchError) as caught:
            guarded.send_commands(routed_commands(session), now=0)
        assert caught.value.accepted_groups == (arms[0],)
        assert not caught.value.stop_failures
        assert all(not h.last["hardware_enabled"] for h in harnesses.values())


def test_lost_reply_stops_even_receiver_that_may_have_accepted():
    with ExitStack() as stack:
        session, harnesses, clients, guarded = session_fixture(stack, "minerva")
        guarded.acquire(); guarded.arm()
        arm = list(clients)[1]
        original = clients[arm].exchange
        def lose_move_reply(request):
            result = original(request)
            if request["type"] == "move":
                raise TimeoutError("simulated lost acknowledgement")
            return result
        clients[arm].exchange = lose_move_reply
        with pytest.raises(GuardedBatchError):
            guarded.send_commands(routed_commands(session), now=0)
        assert all(not h.last["hardware_enabled"] for h in harnesses.values())


def test_collision_verdict_rechecked_before_each_receiver():
    with ExitStack() as stack:
        session, harnesses, _, guarded = session_fixture(stack, "minerva")
        guarded.acquire(); guarded.arm()
        class ExpiringGate:
            calls = 0
            mode = "enforce_simulation"
            def filter_commands(self, _session, commands):
                return commands
            def dispatch_valid(self, *_):
                self.calls += 1
                return self.calls < 3
        session.command_gate = ExpiringGate()
        with pytest.raises(GuardedBatchError):
            guarded.send_commands(routed_commands(session), now=0)
        assert all(not h.last["hardware_enabled"] for h in harnesses.values())


def test_adapter_refuses_physical_session():
    spec = load_robot("aizee")
    with pytest.raises(ValueError, match="simulation_only"):
        GuardedSessionClient(RobotSession(spec, load_settings(spec)), {})


def test_busy_acquisition_rolls_back_without_stopping_other_owner():
    with ExitStack() as stack:
        _, harnesses, clients, guarded = session_fixture(stack, "minerva")
        arms = list(clients)
        other = client(harnesses[arms[1]], config(clients[arms[1]].joint_names, arms[1]))
        other.acquire(); other.arm()
        with pytest.raises(GuardedBatchError) as caught:
            guarded.acquire()
        assert caught.value.accepted_groups == (arms[0],)
        assert harnesses[arms[0]].last["status"]["lease"] is None
        assert harnesses[arms[1]].last["hardware_enabled"]
        assert other.poll()["lease"]["session"] == other.session_id


def test_native_feedback_preserves_source_age_when_republished():
    cfg = config()
    with NativeControlHarness(cfg, [0.0]) as h:
        h.step(5)
        first = h.telemetry()["motors"]["joint"]["feedback"]
        h.step(15, faults={"frozen": True})
        repeated = h.telemetry()["motors"]["joint"]["feedback"]
        assert repeated["sequence"] == first["sequence"]
        assert repeated["age_s"] == 0.01
