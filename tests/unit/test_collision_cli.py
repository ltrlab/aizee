import json

import pytest

pytest.importorskip("fcl")
from robot_platform.simulation.cli import main


def files(tmp_path, request):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text('<robot name="fixture"><link name="base"><collision><geometry>'
                    '<box size="1 1 1"/></geometry></collision></link></robot>')
    scene = tmp_path / "scene.json"
    scene.write_text(json.dumps(request))
    return ["check-collision", "--urdf", str(urdf), "--scene", str(scene)]


def test_cli_clear_state_is_explicitly_offline(tmp_path, capsys):
    args = files(tmp_path, {"positions": {}})
    output = tmp_path / "report.json"
    assert main(args + ["--output", str(output)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result == json.loads(output.read_text())
    assert result["checked_samples_clear"]
    assert not result["runtime_interlock_active"]
    assert not result["continuous_collision_proof"]


def test_cli_colliding_environment_returns_failure(tmp_path, capsys):
    args = files(tmp_path, {"positions": {}, "environment": [{"name": "obstacle", "size": [1, 1, 1]}]})
    assert main(args) == 2
    result = json.loads(capsys.readouterr().out)
    assert result["colliding_pairs"] == [["base", "environment:obstacle"]]


@pytest.mark.parametrize("scene_request", [{"positions": {}, "typo": []},
    {"positions": {}, "end_positions": {}}, {"positions": {}, "max_joint_step": .1},
    {"positions": {"unknown": 0}}, {"positions": {}, "environment": [{}]}])
def test_cli_rejects_incomplete_or_unknown_scene(tmp_path, scene_request):
    with pytest.raises(SystemExit) as error:
        main(files(tmp_path, scene_request))
    assert error.value.code == 2


def test_probe_cli_report_is_synthetic_and_never_overwrites(tmp_path, capsys):
    output = tmp_path / "probe.json"
    assert main(["validate-limit-probe", "--output", str(output)]) == 0
    original = output.read_bytes()
    result = json.loads(capsys.readouterr().out)
    assert result["passed"] and result["hardware_commands_sent"] == 0
    with pytest.raises(SystemExit):
        main(["validate-limit-probe", "--output", str(output)])
    assert output.read_bytes() == original


def test_interrupted_collision_check_cannot_report_success(tmp_path, monkeypatch):
    from robot_platform.simulation.collision import CollisionScene
    def interrupted(*args, **kwargs):
        raise KeyboardInterrupt
    monkeypatch.setattr(CollisionScene, "from_urdf", interrupted)
    assert main(files(tmp_path, {"positions": {}})) == 130


def test_interrupted_probe_validation_cannot_report_success(monkeypatch):
    from robot_platform.characterization import limit_probe
    def interrupted():
        raise KeyboardInterrupt
    monkeypatch.setattr(limit_probe, "synthetic_validation", interrupted)
    assert main(["validate-limit-probe"]) == 130
