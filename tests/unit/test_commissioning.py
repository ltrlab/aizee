import struct
import copy
from pathlib import Path

import pytest

from robot_platform.control.commissioning import profile
from robot_platform.control.parameter_probe import matching_observation
from robot_platform.control.motor_diagnostics import matching_diagnostic
from robot_platform.control.watchdog_trial import trial_profile


def test_profile_pins_roster_and_refuses_motion_modes():
    root = Path(__file__).resolve().parents[2]
    device = root / "config/devices/minerva-production-01.json"
    config = profile(device, root)
    assert config["mode"] == "observe"
    assert len(config["motors"]) == 14
    assert config["motors"][0]["bus"] != config["motors"][7]["bus"]
    assert config["motors"][0]["id"] == config["motors"][7]["id"] == 4
    assert all(bus["usb_serial"] for bus in config["buses"])
    for mode in ("virtual_can", "zero_torque_trial", "production", "bogus"):
        with pytest.raises(ValueError):
            profile(device, root, mode=mode)


def test_parameter_response_requires_matching_bus_motor_host_parameter_and_freshness():
    row = {"bus": 1, "id": 0x110004aa, "data": list(b"\x19\x70\0\0" + struct.pack("<f", 1.5)), "received_at_ms": 20}
    match = dict(bus=1, motor_id=4, host_id=170, parameter=0x7019, after_ms=20)
    assert matching_observation([row], **match)["value"] == 1.5
    for key, value in {"bus": 0, "motor_id": 5, "host_id": 187, "parameter": 0x701b, "after_ms": 21}.items():
        assert matching_observation([row], **{**match, key: value}) is None
    for value in (float("nan"), float("inf")):
        invalid = {**row, "data": list(b"\x19\x70\0\0" + struct.pack("<f", value))}
        assert matching_observation([invalid], **match) is None
    assert matching_observation([{**row, "id": row["id"] | (1 << 16)}], **match) is None


def test_diagnostics_distinguish_version_status_and_typed_timeout():
    match = dict(bus=1, motor_id=4, host_id=170, after_ms=20)
    row = {"bus": 1, "id": 0x020004aa, "data": [0, 0xc4, 0x56, 0, 3, 1, 41, 0], "received_at_ms": 20}
    assert matching_diagnostic([row], diagnostic={"kind": "version"}, **match)["firmware"] == "0.3.1.41"
    assert matching_diagnostic([row], diagnostic={"kind": "stop_feedback"}, **match) is None
    for key, value in {"bus": 0, "motor_id": 5, "host_id": 187, "after_ms": 21}.items():
        assert matching_diagnostic([row], diagnostic={"kind": "version"}, **{**match, key: value}) is None
    stopped = {**row, "data": [0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff, 1, 44]}
    assert matching_diagnostic([stopped], diagnostic={"kind": "stop_feedback"}, **match)["mode"] == 0
    assert matching_diagnostic([stopped], diagnostic={"kind": "version"}, **match) is None
    timeout = {**row, "id": 0x110004aa, "data": list(b"\x28\x70\0\0" + struct.pack("<I", 4000))}
    assert matching_diagnostic([timeout], diagnostic={"kind": "parameter", "parameter": 0x7028}, **match)["value"] == 4000
    # A parameter error can contain zero bytes. It must not prove a zero watchdog.
    assert matching_diagnostic([{**timeout, "id": 0x110104aa}], diagnostic={"kind": "parameter", "parameter": 0x7028}, **match) is None


def test_identity_and_real_parameter_fixture():
    match = dict(bus=0, motor_id=4, host_id=170, after_ms=20)
    identity = {"bus": 0, "id": 0x000004fe, "data": list(range(8)), "received_at_ms": 20}
    assert matching_diagnostic([identity], diagnostic={"kind": "identity"}, **match)["mcu_uid_hex"] == "0001020304050607"
    assert matching_diagnostic([{**identity, "id": 0x000004aa}], diagnostic={"kind": "identity"}, **match) is None
    # Captured on Minerva's right arm, ID 4, 2026-09-24; no model-dependent scaling.
    position = {"bus": 1, "id": 0x110004aa, "data": [25, 112, 0, 0, 142, 189, 43, 188], "received_at_ms": 1755}
    actual = matching_diagnostic([position], diagnostic={"kind": "parameter", "parameter": 0x7019}, **{**match, "bus": 1})
    assert actual["value"] == pytest.approx(-0.01048220507800579)


def test_single_motor_trial_profile_pins_route_uid_and_firmware():
    root = Path(__file__).resolve().parents[2]
    device = root / "config/devices/minerva-production-01.json"
    row = {"name": "right_gripper", "bus": "canR", "can_id": 10,
           "identity": {"mcu_uid_hex": "632f93c79c90b00e"}, "version": {"firmware": "0.0.3.32"},
           "reset_confirmed": True, "stop_feedback": {"error_bits": 0}, "parameters": {"run_mode": {"value": 0}}}
    diagnostics = {"evidence": "hardware-stopped-motor-diagnostics", "motors": [row]}
    result = trial_profile(device, root, diagnostics, "right_gripper")
    assert result["service"]["buses"][0]["interface"] == "canR"
    assert len(result["service"]["motors"]) == 1
    assert result["service"]["motors"][0]["bus"] == 0
    assert result["motor_identity"]["firmware"] == [0, 0, 3, 32]
    assert result["service"]["watchdog_identities"] == [result["motor_identity"]]
    assert result["service"]["motors"][0]["ranges"]["effort"] == 14.0
    jog = trial_profile(device, root, diagnostics, "right_gripper", jog_radians=.025)
    assert jog["service"]["mode"] == "bounded_jog_trial"
    assert jog["jog_radians"] == .025
    for key, bad in {"bus": "canL", "can_id": 9, "reset_confirmed": False, "parameters": {"run_mode": None}}.items():
        altered = copy.deepcopy(diagnostics)
        altered["motors"][0][key] = bad
        with pytest.raises(ValueError):
            trial_profile(device, root, altered, "right_gripper")
    with pytest.raises(ValueError):
        trial_profile(device, root, diagnostics, "right_gripper", jog_radians=.5)
    with pytest.raises(ValueError, match="starting position"):
        trial_profile(device, root, diagnostics, "right_gripper", breakaway_direction=1)
    row["parameters"]["motor_position_rad"] = {"value": 2.46}
    for direction in [-1, 1]:
        ramp = trial_profile(device, root, diagnostics, "right_gripper", breakaway_direction=direction)
        assert ramp["service"]["mode"] == "breakaway_trial"
        assert ramp["breakaway"] == {"direction": direction, "expected_position_rad": 2.46}
        assert "watchdog_identities" not in ramp["service"]
        damped = trial_profile(device, root, diagnostics, "right_gripper", breakaway_direction=direction, damped_stop=True)
        assert damped["breakaway"]["stop"] == "damped"
    for options in ({"breakaway_direction": 0}, {"breakaway_direction": 1, "jog_radians": .025}):
        with pytest.raises(ValueError):
            trial_profile(device, root, diagnostics, "right_gripper", **options)
    with pytest.raises(ValueError, match="requires a breakaway"):
        trial_profile(device, root, diagnostics, "right_gripper", damped_stop=True)
