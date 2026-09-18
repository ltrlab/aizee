"""Offline contract checks for the current 7- and 17-joint robot layouts."""

from copy import deepcopy
import math
from types import SimpleNamespace

import pytest

from robot_platform.control import RobotSession, StaleTelemetryError, route_joint_target


AIZEE = ("swivel", "gantry_base", "gantry_mid", "gantry_end", "wrist_pitch", "wrist_roll", "gripper")
LEFT = tuple(f"left_arm_j{i}" for i in range(1, 7)) + ("left_gripper",)
RIGHT = tuple(f"right_arm_j{i}" for i in range(1, 7)) + ("right_gripper",)
MINERVA = LEFT + RIGHT + ("head_pan", "head_tilt", "lift")


def make_spec(names=MINERVA):
    return SimpleNamespace(joints=tuple(SimpleNamespace(name=name, active=index < 14, unit="rad") for index, name in enumerate(names)))


def make_settings(arms=None):
    arms = {"left": LEFT, "right": RIGHT} if arms is None else arms
    return {"endpoints": {"arms": {
        arm: {"command": f"inproc://{arm}-cmd", "telemetry": f"inproc://{arm}-telem", "joints": list(names)}
        for arm, names in arms.items()
    }}}


def telemetry(names, *, offset=0.0):
    return {"timestamp": 1_000_000.0, "motors": {
        name: {"position": offset + i, "torque": offset + i + 0.5, "velocity": 0.0,
               "temperature": 20.0, "state": "disabled", "mode": "run", "error": None}
        for i, name in enumerate(names)
    }}


@pytest.mark.parametrize("names,arms", [(AIZEE, {"main": AIZEE}), (MINERVA, {"left": LEFT, "right": RIGHT})])
def test_real_robot_shapes_split_by_endpoint_joint_order(names, arms):
    # Deliberately invert both endpoint and wire order: never assume contiguous
    # slices or that the first configured endpoint corresponds to joint zero.
    reversed_arms = {arm: tuple(reversed(joints)) for arm, joints in reversed(tuple(arms.items()))}
    spec, settings = make_spec(names), make_settings(reversed_arms)
    positions = [float(i) if i < 14 else 0.0 for i in range(len(names))]
    gains = [1.0 if i < 14 else 0.0 for i in range(len(names))]
    before = deepcopy((positions, gains, settings))
    routed = route_joint_target(spec, settings["endpoints"]["arms"], positions, kp=gains, kd=gains, torques=positions)
    assert list(routed) == list(reversed_arms)
    for arm, joints in reversed_arms.items():
        expected = [positions[names.index(name)] for name in joints]
        assert routed[arm] == {"type": "arm_joints", "positions": expected,
                               "velocities": [0.0] * 7, "kp": [1.0] * 7,
                               "kd": [1.0] * 7, "torques": expected}
    assert (positions, gains, settings) == before
    routed[next(iter(routed))]["positions"][0] = -999
    assert positions == before[0]


@pytest.mark.parametrize("field", ["positions", "velocities", "kp", "kd", "torques"])
@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf, "1.0", None, True])
def test_malformed_targets_are_rejected(field, bad_value):
    vector = [0.0] * 17
    vector[3] = bad_value
    kwargs = {"positions": [0.0] * 17, field: vector}
    with pytest.raises(ValueError):
        route_joint_target(make_spec(), make_settings()["endpoints"]["arms"], **kwargs)


@pytest.mark.parametrize("field", ["positions", "velocities", "kp", "kd", "torques"])
def test_inactive_targets_are_rejected_instead_of_discarded(field):
    vector = [0.0] * 17
    vector[16] = 0.1
    with pytest.raises(ValueError, match="inactive joint 'lift'"):
        route_joint_target(make_spec(), make_settings()["endpoints"]["arms"], **{"positions": [0.0] * 17, field: vector})


@pytest.mark.parametrize("length", [0, 7, 14, 16, 18])
def test_wrong_vector_size_rejected(length):
    with pytest.raises(ValueError, match="17 values"):
        route_joint_target(make_spec(), make_settings()["endpoints"]["arms"], [0.0] * length)


@pytest.mark.parametrize("mutation", ["duplicate_joint", "inactive", "unknown", "missing_active", "duplicate_endpoint"])
def test_invalid_endpoint_ownership_rejected(mutation):
    settings = make_settings()
    arms = settings["endpoints"]["arms"]
    if mutation == "duplicate_joint":
        arms["right"]["joints"][0] = LEFT[0]
    elif mutation == "inactive":
        arms["right"]["joints"][0] = "lift"
    elif mutation == "unknown":
        arms["right"]["joints"][0] = "not_a_joint"
    elif mutation == "missing_active":
        arms["right"]["joints"].pop()
    else:
        arms["right"]["command"] = arms["left"]["command"]
    with pytest.raises(ValueError):
        RobotSession(make_spec(), settings)


def test_session_snapshots_inputs_and_does_not_invent_missing_telemetry():
    settings = make_settings()
    session = RobotSession(make_spec(), settings)
    settings["endpoints"]["arms"]["left"]["joints"].clear()
    empty = session.observation(now=10.0)
    assert not empty.ready
    assert not any(empty.valid_mask)
    assert all(math.isnan(value) for value in empty.positions)
    assert math.isinf(empty.arm_freshness["left"].age_s)
    message = telemetry(LEFT)
    session.ingest_telemetry("left", message, received_at=10.0)
    message["motors"][LEFT[0]]["position"] = 9000
    partial = session.observation(now=10.1)
    assert partial.positions[:7] == tuple(float(i) for i in range(7))
    assert all(math.isnan(value) for value in partial.positions[7:])
    assert partial.active_mask == (True,) * 14 + (False,) * 3
    assert partial.valid_mask == (True,) * 7 + (False,) * 10
    assert not partial.ready
    with pytest.raises(StaleTelemetryError, match="right"):
        session.route_target([0.0] * 17, now=10.1)
    session.ingest_telemetry("right", telemetry(RIGHT, offset=7.0), received_at=10.2)
    complete = session.observation(now=10.3)
    assert complete.ready
    assert complete.positions[:14] == tuple(float(i) for i in range(14))
    assert complete.torques[:14] == tuple(float(i) + 0.5 for i in range(14))
    assert all(math.isnan(value) for value in complete.positions[14:])
    assert len(session.route_target([0.0] * 17, now=10.3)) == 2
    with pytest.raises(TypeError):
        complete.arm_freshness["left"] = None
    # One old arm invalidates readiness, even while the other remains fresh.
    stale = session.observation(now=10.6)
    assert not stale.arm_freshness["left"].fresh
    assert stale.arm_freshness["right"].fresh
    assert all(math.isnan(value) for value in stale.positions[:7])
    assert not stale.ready
    with pytest.raises(StaleTelemetryError, match="left"):
        session.route_target([0.0] * 17, now=10.6)


@pytest.mark.parametrize("corruption", ["missing_motor", "missing_position", "missing_torque", "nonfinite", "invalid_map"])
def test_bad_latest_frame_never_falls_back_to_old_or_zero_data(corruption):
    session = RobotSession(make_spec(AIZEE), make_settings({"main": AIZEE}))
    session.ingest_telemetry("main", telemetry(AIZEE), received_at=1.0)
    message = telemetry(AIZEE)
    if corruption == "missing_motor":
        message["motors"].pop(AIZEE[0])
    elif corruption == "missing_position":
        message["motors"][AIZEE[0]].pop("position")
    elif corruption == "missing_torque":
        message["motors"][AIZEE[0]].pop("torque")
    elif corruption == "nonfinite":
        message["motors"][AIZEE[0]]["position"] = math.nan
    else:
        message["motors"] = []
    session.ingest_telemetry("main", message, received_at=1.1)
    observation = session.observation(now=1.2)
    assert observation.arm_freshness["main"].fresh
    assert not observation.arm_freshness["main"].complete
    assert not observation.valid_mask[0]
    assert not observation.ready
    with pytest.raises(StaleTelemetryError):
        session.route_target([0.0] * 7, now=1.2)


def test_monotonic_times_cannot_run_backwards_or_mask_future_receipts():
    session = RobotSession(make_spec(AIZEE), make_settings({"main": AIZEE}))
    session.ingest_telemetry("main", telemetry(AIZEE), received_at=10.0)
    with pytest.raises(ValueError, match="Out-of-order"):
        session.ingest_telemetry("main", telemetry(AIZEE), received_at=9.0)
    assert not session.observation(now=9.0).ready


@pytest.mark.parametrize("robot_name,expected_size,expected_arms", [("aizee", 7, 1), ("minerva", 17, 2)])
def test_repository_profiles_use_same_session_api(robot_name, expected_size, expected_arms):
    from robot_platform.robots import load_robot
    from robot_platform.settings import load_settings

    spec = load_robot(robot_name)
    settings = load_settings(spec, overrides={"control": {"stale_after_s": 0.25}})
    session = RobotSession(spec, settings)
    assert len(session.joint_names) == expected_size
    assert len(session.arms) == expected_arms
    assert session.max_age_s == 0.25
    for arm, endpoint in session.arms.items():
        session.ingest_telemetry(arm, telemetry(endpoint.joints), received_at=1.0)
    assert session.observation(now=1.2).ready
    assert len(session.route_target([0.0] * expected_size, now=1.2)) == expected_arms
    assert not session.observation(now=1.3).ready


def test_partial_targeting_uses_position_readiness_without_inventing_torque():
    session = RobotSession(make_spec(), make_settings())
    left = telemetry(LEFT)
    left["motors"][LEFT[0]].pop("torque")
    session.ingest_telemetry("left", left, received_at=1.0)
    assert session.arm_ready(now=1.1) == {"left": True, "right": False}
    assert not session.observation(now=1.1).ready
    target = [0.2] * 7 + [math.nan] * 7 + [0.0] * 3
    routed = session.route_target(target, now=1.1, allow_partial=True, require_torque=False)
    assert set(routed) == {"left"}
    assert session.route_target(target, now=1.1, allow_partial=True) == {}
    assert session.route_target(target, now=1.6, allow_partial=True, require_torque=False) == {}


def test_raw_metadata_and_numeric_fields_retain_original_age_and_no_mutable_aliases():
    session = RobotSession(make_spec(AIZEE), make_settings({"main": AIZEE}))
    message = telemetry(AIZEE)
    message.update(battery_voltage=24.0, emergency_stop=True)
    session.ingest_telemetry("main", message, received_at=1.0)
    message["battery_voltage"] = 0.0
    assert session.joint_field("temperature", now=1.2) == (20.0,) * 7
    assert session.telemetry("main", now=1.2)["battery_voltage"] == 24.0
    copy, received_at = session.telemetry_sample("main")
    copy["motors"].clear()
    assert len(session.telemetry_sample("main")[0]["motors"]) == 7
    assert received_at == 1.0
    assert session.telemetry("main", now=2.0) is None
    assert all(math.isnan(value) for value in session.joint_field("temperature", now=2.0))


def test_stale_raw_bundle_retains_drive_stop_and_normalizes_legacy_payload():
    session = RobotSession(make_spec(AIZEE), make_settings({"main": AIZEE}))
    command = {"type": "bundle", "arm_joints": {"type": "arm_joints", "positions": [0.2] * 7,
               "velocities": [0.0] * 7, "kp": None, "kd": None},
               "drive": {"type": "drive", "linear": 0.0, "angular": 0.0}}
    before = deepcopy(command)
    assert session.prepare_commands({"main": command}, now=1.0) == {
        "main": {"type": "bundle", "drive": {"linear": 0.0, "angular": 0.0}}}
    session.ingest_telemetry("main", telemetry(AIZEE), received_at=1.0)
    arm = session.prepare_commands({"main": command}, now=1.1)["main"]["arm_joints"]
    assert "type" not in arm and "kp" not in arm and "kd" not in arm
    assert command == before


@pytest.mark.parametrize("kind", ["enable", "disable", "zero_position", "mech_zero", "clear_fault", "trigger_fault"])
def test_rust_service_commands_allow_named_wheels_without_arm_telemetry(kind):
    session = RobotSession(make_spec(AIZEE), make_settings({"main": AIZEE}))
    command = {"type": kind, "motor_ids": ["left_wheel", "right_wheel"]}
    if kind == "mech_zero":
        command["save"] = True
    assert session.prepare_commands({"main": command}, now=1.0) == {"main": command}
