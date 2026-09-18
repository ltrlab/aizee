import json
from pathlib import Path

import pytest

from robot_platform.apps.launcher import build_launch_plan
from robot_platform.cli import main
from robot_platform.contracts import ConfigurationError
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings

REPO = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("robot", ["aizee", "minerva"])
@pytest.mark.parametrize("command", ["collect", "train"])
def test_both_robots_use_the_same_launcher_without_running_apps(robot, command, tmp_path):
    spec = load_robot(robot)
    settings = load_settings(spec, overrides={"host": "10.42.0.1"})
    plan = build_launch_plan(spec, settings, command, repo=REPO, caller_cwd=tmp_path)
    assert plan.application == f"{robot}.{command}"
    assert Path(plan.cwd) == REPO
    output = plan.argv[plan.argv.index("--output-dir") + 1]
    assert Path(output).is_relative_to(tmp_path)
    if command == "collect":
        flag = "--right-cmd" if robot == "minerva" else "--cmd"
        assert plan.argv[plan.argv.index(flag) + 1] == f"tcp://10.42.0.1:{5575 if robot == 'minerva' else 5555}"


def test_incompatible_adapter_does_not_construct_command():
    spec = load_robot("aizee")
    with pytest.raises(ConfigurationError, match="incompatible"):
        build_launch_plan(spec, load_settings(spec), "collect", adapter="minerva.collect", repo=REPO)


@pytest.mark.parametrize("args", [["--cmd", "tcp://bad:5555"], ["--config=other.yaml"], ["--output-dir", "elsewhere"], ["--left-cm", "tcp://bad:9000"], ["--output-d=elsewhere"]])
def test_forwarded_arguments_cannot_bypass_resolved_settings(args):
    spec = load_robot("aizee")
    with pytest.raises(ConfigurationError, match="managed by settings"):
        build_launch_plan(spec, load_settings(spec), "collect", repo=REPO, extra_args=args)


def test_settings_and_extra_options_reach_application_in_order():
    spec = load_robot("minerva")
    settings = load_settings(spec, overrides={"applications": {"collect": {"options": ["--kp-scale", "0.2"], "gui": False}}})
    plan = build_launch_plan(spec, settings, "collect", repo=REPO, extra_args=["--dry-run"])
    assert plan.argv[-3:] == ("--kp-scale", "0.2", "--dry-run")
    assert "--no-gui" in plan.argv


def test_cli_plan_is_json_and_does_not_spawn(monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        raise AssertionError("plan must not spawn the application")
    monkeypatch.setattr("subprocess.call", forbidden)
    assert main(["run", "collect", "--robot", "minerva", "--host", "10.42.0.1", "--repo", str(REPO), "--plan"]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["settings"]["host"] == "10.42.0.1"


def test_execution_passes_profile_to_child_without_changing_process_env(monkeypatch):
    monkeypatch.delenv("ROBOT_PLATFORM_SETTINGS_JSON", raising=False)
    spec = load_robot("minerva")
    settings = load_settings(spec)
    seen = {}
    def fake_call(argv, *, cwd, env):
        seen.update(argv=argv, cwd=cwd, settings=json.loads(env["ROBOT_PLATFORM_SETTINGS_JSON"]))
        return 4
    monkeypatch.setattr("subprocess.call", fake_call)
    plan = build_launch_plan(spec, settings, "collect", repo=REPO)
    assert plan.execute() == 4 and seen["settings"] == settings
    import os
    assert "ROBOT_PLATFORM_SETTINGS_JSON" not in os.environ


def test_describe_and_config_support_standard_library_only_interfaces(capsys):
    assert main(["describe", "--robot", "minerva"]) == 0
    assert len(json.loads(capsys.readouterr().out)["joints"]) == 17
    assert main(["config", "--robot", "aizee", "--set", "applications.collect.gui=false"]) == 0
    assert json.loads(capsys.readouterr().out)["applications"]["collect"]["gui"] is False


def test_launch_plan_is_snapshot_of_settings():
    spec = load_robot("minerva")
    settings = load_settings(spec)
    plan = build_launch_plan(spec, settings, "collect", repo=REPO)
    original = plan.settings
    settings["host"] = "changed"
    plan.to_dict()["settings"]["host"] = "also-changed"
    assert plan.settings == original


def test_optional_scene_can_be_disabled_in_launch():
    spec = load_robot("aizee")
    settings = load_settings(spec, overrides={"endpoints": {"cameras": {"scene": None}}})
    plan = build_launch_plan(spec, settings, "collect", repo=REPO)
    assert plan.argv[plan.argv.index("--scene-cam") + 1] == ""


def _minerva_legacy_config():
    import importlib.util
    module_spec = importlib.util.spec_from_file_location("minerva_legacy_config", REPO / "python/scripts/collect_minerva_app/config.py")
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    return module


def test_minerva_launcher_settings_reach_real_camera_config(monkeypatch):
    config = _minerva_legacy_config()
    monkeypatch.delenv("ROBOT_PLATFORM_SETTINGS_JSON", raising=False)
    original = config.load_config(str(REPO / "config/minerva.yaml"))
    settings = load_settings(load_robot("minerva"), overrides={
        "host": "10.42.0.1", "endpoints": {"cameras": {"head": "tcp://scene-host:6004"}}})
    monkeypatch.setenv("ROBOT_PLATFORM_SETTINGS_JSON", json.dumps(settings))
    cfg = config.load_config(str(REPO / "config/minerva.yaml"))
    assert cfg["endpoints"]["cameras"] == settings["endpoints"]["cameras"]
    assert cfg["cameras"] == original["cameras"]
    assert cfg["safety"] == original["safety"]


def test_explicit_missing_minerva_config_is_not_silently_replaced(monkeypatch, tmp_path):
    monkeypatch.delenv("ROBOT_PLATFORM_SETTINGS_JSON", raising=False)
    with pytest.raises(FileNotFoundError):
        _minerva_legacy_config().load_config(str(tmp_path / "missing.yaml"))


def test_legacy_joint_vocabulary_uses_packaged_definitions():
    import importlib.util
    for robot, filename, attribute in (
        ("aizee", "arm_constants.py", "ARM_JOINTS"),
        ("minerva", "minerva_constants.py", "MINERVA_JOINTS"),
    ):
        mspec = importlib.util.spec_from_file_location("legacy_" + robot, REPO / "python/common" / filename)
        module = importlib.util.module_from_spec(mspec)
        mspec.loader.exec_module(module)
        assert tuple(getattr(module, attribute)) == load_robot(robot).joint_names
