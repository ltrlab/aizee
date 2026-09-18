from copy import deepcopy
import json

import pytest

from robot_platform.contracts import ConfigurationError, RobotSpec
from robot_platform.robots import available_robots, load_robot
from robot_platform.settings import load_settings, merge_settings, validate_settings


def test_builtins_have_explicit_active_capabilities():
    assert available_robots() == ("aizee", "minerva")
    a, m = load_robot("aizee"), load_robot("minerva")
    assert len(a.active_joint_names) == 7
    assert len(m.joints) == 17 and len(m.active_joint_names) == 14
    assert not any(j.active for j in m.joints[14:])
    assert "lift" not in m.capabilities
    assert RobotSpec.from_dict(m.to_dict()) == m


@pytest.mark.parametrize("name", ["aizee", "minerva"])
def test_settings_host_replacement_and_input_immutability(name):
    override = {"host": "10.42.0.1"}
    before = deepcopy(override)
    resolved = load_settings(load_robot(name), overrides=override)
    assert override == before
    assert all(ep["command"].startswith("tcp://10.42.0.1:") for ep in resolved["endpoints"]["arms"].values())


def test_ipv6_endpoint_host():
    result = load_settings(load_robot("aizee"), overrides={"host": "::1"})
    assert result["endpoints"]["arms"]["main"]["command"] == "tcp://[::1]:5555"


def test_layers_and_explicit_endpoint_hosts_are_preserved(tmp_path):
    p1 = tmp_path / "machine.json"
    p2 = tmp_path / "operator.json"
    p1.write_text(json.dumps({"host": "10.42.0.1", "applications": {"collect": {"gui": False}}}))
    p2.write_text(json.dumps({"applications": {"collect": {"gui": True}}, "endpoints": {"cameras": {"head": "tcp://scene-host:5564"}}}))
    result = load_settings(load_robot("minerva"), [p1, p2], {"host": "192.168.50.1"})
    assert result["applications"]["collect"]["gui"] is True
    assert result["endpoints"]["arms"]["right"]["command"] == "tcp://192.168.50.1:5575"
    assert result["endpoints"]["cameras"]["head"] == "tcp://scene-host:5564"


@pytest.mark.parametrize("overlay", [
    {"robot": "minerva"}, {"hosst": "localhost"}, {"host": None},
    {"control": {"stale_after_s": float("nan")}}, {"control": {"stale_after_s": True}},
    {"applications": {"collect": {"gui": "false"}}},
    {"endpoints": {"arms": {"main": {"command": "tcp://localhost:0"}}}},
    {"endpoints": {"arms": {"main": {"command": "http://localhost:5555"}}}},
    {"endpoints": {"cameras": {"typo": "tcp://localhost:9000"}}},
])
def test_invalid_overrides_fail_before_launch(overlay):
    with pytest.raises(ConfigurationError):
        load_settings(load_robot("aizee"), overrides=overlay)


def test_missing_settings_file_does_not_fall_back(tmp_path):
    with pytest.raises(ConfigurationError, match="cannot load settings"):
        load_settings(load_robot("aizee"), [tmp_path / "missing.json"])


def test_required_camera_cannot_be_disabled():
    with pytest.raises(ConfigurationError):
        load_settings(load_robot("minerva"), overrides={"endpoints": {"cameras": {"head": None}}})


def test_joint_order_and_duplicate_ports_are_rejected():
    spec = load_robot("minerva")
    data = load_settings(spec)
    data["endpoints"]["arms"]["right"]["joints"].reverse()
    with pytest.raises(ConfigurationError, match="ordered"):
        validate_settings(spec, data)
    data = load_settings(spec)
    data["endpoints"]["arms"]["right"]["command"] = data["endpoints"]["arms"]["left"]["command"]
    with pytest.raises(ConfigurationError, match="distinct"):
        validate_settings(spec, data)


def test_custom_robot_definition_with_custom_settings(tmp_path):
    raw = {"schema_version": 1, "name": "bench", "joints": [{"name": "j1", "unit": "rad"}],
           "groups": {"arm": ["j1"]}, "cameras": [], "capabilities": ["arm"]}
    path = tmp_path / "robot.json"
    path.write_text(json.dumps(raw))
    spec = load_robot(path)
    data = load_settings(spec, overrides={"endpoints": {"arms": {"arm": {
        "command": "tcp://{host}:6000", "telemetry": "tcp://{host}:6001", "joints": ["j1"]}}}})
    assert spec.name == "bench" and data["endpoints"]["arms"]["arm"]["command"] == "tcp://localhost:6000"


def test_merge_replaces_lists_without_mutating_layers():
    original = {"application": {"options": ["--old"], "gui": True}}
    merged = merge_settings(original, {"application": {"options": ["--new"]}})
    assert original["application"]["options"] == ["--old"]
    assert merged == {"application": {"options": ["--new"], "gui": True}}


@pytest.mark.parametrize("edit", [
    lambda d: d["joints"].append(d["joints"][0]),
    lambda d: d["joints"][0].update(unit="degrees"),
    lambda d: d["groups"].update(main=["unknown"]),
    lambda d: d.update(schema_version=7),
])
def test_bad_robot_definition(edit):
    data = load_robot("aizee").to_dict()
    edit(data)
    with pytest.raises(ConfigurationError):
        RobotSpec.from_dict(data)
