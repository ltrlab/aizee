import json

import pytest

from robot_platform.simulation.cli import main


def inputs(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text('<robot name="test"><link name="base"/></robot>', encoding="utf-8")
    return ["audit-model", "--urdf", str(urdf), "--package-root", str(tmp_path)]


def test_cli_reports_failure_against_manifest_without_mutating_model(tmp_path, capsys):
    args = inputs(tmp_path)
    before = (tmp_path / "robot.urdf").read_bytes()
    manifest = tmp_path / "source.json"
    manifest.write_text(json.dumps({"expected_joints": {"revolute": ["arm"], "prismatic": []}}))
    output = tmp_path / "audit.json"
    assert main(args + ["--expectations", str(manifest), "--output", str(output)]) == 2
    report = json.loads(capsys.readouterr().out)
    assert report == json.loads(output.read_text())
    assert report["physical_model_validated"] is False
    assert any(item["code"] == "missing_expected_joint" for item in report["findings"])
    assert (tmp_path / "robot.urdf").read_bytes() == before


def test_static_success_does_not_claim_physical_validation(tmp_path, capsys):
    assert main(inputs(tmp_path)) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["static_checks_passed"]
    assert not report["physical_model_validated"]


def test_cli_preserves_existing_report(tmp_path):
    args = inputs(tmp_path)
    output = tmp_path / "audit.json"
    output.write_text("previous evidence")
    with pytest.raises(SystemExit) as caught:
        main(args + ["--output", str(output)])
    assert caught.value.code == 2
    assert output.read_text() == "previous evidence"


@pytest.mark.parametrize("invalid", [[], {}, {"revolute": "arm"}, {"expected_joints": None}])
def test_cli_rejects_invalid_expectations(tmp_path, invalid):
    args = inputs(tmp_path)
    path = tmp_path / "expectations.json"
    path.write_text(json.dumps(invalid))
    with pytest.raises(SystemExit) as caught:
        main(args + ["--expectations", str(path)])
    assert caught.value.code == 2
