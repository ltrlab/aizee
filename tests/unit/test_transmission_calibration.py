"""Offline calibration criteria: synthetic measurements are not hardware evidence."""
import copy
import json

import pytest

from robot_platform.control.transmission_calibration import (
    estimate_aperture, fit_samples, main, make_sample,
)


def diagnostic(position=2., *, capture=0):
    return {"started_at": f"2026-09-24T20:{capture:02d}:00+00:00", "diagnostics": {
        "evidence": "hardware-stopped-motor-diagnostics", "complete": True,
        "enable_requested": False, "stop_requested": True, "parameter_writes_requested": False,
        "motors": [{"name": "test_gripper", "bus": "test_bus", "can_id": 7,
                    "identity": {"mcu_uid_hex": "0123456789abcdef"},
                    "version": {"firmware": "0.0.3.32", "error_bits": 0},
                    "reset_confirmed": True, "stop_feedback": {"mode": 0, "error_bits": 0},
                    "parameters": {"motor_position_rad": {"value": position},
                                   "motor_velocity_rad_s": {"value": .01}}}]}}


def sample(position, gap_mm, direction, capture):
    return make_sample(json.dumps(diagnostic(position, capture=capture)).encode(),
                       source=f"synthetic-{capture}.json", motor="test_gripper",
                       aperture_mm=gap_mm, approach=direction, instrument="synthetic calipers",
                       uncertainty_mm=.1, pairing_confirmed=True)


def samples(*, scale=20., backlash=0.):
    return [sample(x, 60. + scale * x + (backlash if direction == "closing" else 0),
                   direction, i * 3 + j)
            for i, direction in enumerate(("opening", "closing"))
            for j, x in enumerate((1., 2., 3.))]


@pytest.mark.parametrize("scale", [20., -15.])
def test_fit_handles_both_motor_signs_and_total_aperture_units(scale):
    result = fit_samples(samples(scale=scale))
    assert result["status"] == "candidate"
    assert result["candidate"]["scale_m_per_rad"] == pytest.approx(scale / 1000)
    assert result["candidate"]["offset_m"] == pytest.approx(.06)
    assert estimate_aperture(result, 2.) == pytest.approx((60. + 2 * scale) / 1000)
    assert result["candidate"]["runtime_approved"] is False
    assert result["fit"]["max_leave_one_out_error_m"] < 1e-12
    with pytest.raises(ValueError, match="Extrapolation"):
        estimate_aperture(result, 3.001)


def test_pairing_preserves_raw_evidence_and_unknown_measurement_metadata():
    report = diagnostic()
    result = make_sample(json.dumps(report).encode(), source="stopped.json", motor="test_gripper",
                         aperture_mm=48.)
    assert result["measurement"]["total_aperture_m"] == .048
    assert result["diagnostics"] == report["diagnostics"]
    assert result["measurement"]["uncertainty_m"] is None
    status = fit_samples([result])
    assert status["status"] == "insufficient_data" and status["candidate"] is None
    assert any("unconfirmed" in issue for issue in status["issues"])
    assert any("uncertainty" in issue for issue in status["issues"])
    with pytest.raises(ValueError, match="No passing"):
        estimate_aperture(status, 2.)


@pytest.mark.parametrize("changes", [
    {"enable_requested": True}, {"parameter_writes_requested": True},
    {"complete": False}, {"stop_requested": False},
])
def test_non_stopped_diagnostic_modes_are_rejected(changes):
    report = diagnostic()
    report["diagnostics"].update(changes)
    with pytest.raises(ValueError, match="stopped diagnostics"):
        make_sample(json.dumps(report).encode(), source="bad.json", motor="test_gripper", aperture_mm=40)


@pytest.mark.parametrize("failure", ["fault", "enabled", "speed", "nan", "duplicate", "firmware_fault"])
def test_faults_motion_nonfinite_and_ambiguous_motor_rejected(failure):
    report = diagnostic()
    motor = report["diagnostics"]["motors"][0]
    if failure == "fault":
        motor["stop_feedback"]["error_bits"] = 1
    elif failure == "enabled":
        motor["stop_feedback"]["mode"] = 2
    elif failure == "speed":
        motor["parameters"]["motor_velocity_rad_s"]["value"] = .101
    elif failure == "nan":
        motor["parameters"]["motor_position_rad"]["value"] = float("nan")
    elif failure == "firmware_fault":
        motor["version"]["error_bits"] = 2
    else:
        report["diagnostics"]["motors"].append(copy.deepcopy(motor))
    with pytest.raises(ValueError):
        make_sample(json.dumps(report).encode(), source="bad.json", motor="test_gripper", aperture_mm=40)


def test_plain_diagnostic_requires_an_explicit_timestamp():
    raw = json.dumps(diagnostic()["diagnostics"]).encode()
    with pytest.raises(ValueError, match="timestamp"):
        make_sample(raw, source="plain.json", motor="test_gripper", aperture_mm=40)
    result = make_sample(raw, source="plain.json", motor="test_gripper", aperture_mm=40,
                         captured_at="2026-09-24T16:30:00-04:00")
    assert result["source"]["captured_at"] == "2026-09-24T20:30:00+00:00"


def test_duplicate_evidence_and_mixed_identity_rejected():
    rows = samples()
    with pytest.raises(ValueError, match="unique diagnostic"):
        fit_samples(rows + [rows[0]])
    rows[0]["identity"]["mcu_uid_hex"] = "fedcba9876543210"
    rows[0]["diagnostics"]["motors"][0]["identity"]["mcu_uid_hex"] = "fedcba9876543210"
    with pytest.raises(ValueError, match="Cannot mix"):
        fit_samples(rows)
    rows = samples()
    rows[0]["motor_position_rad"] += .1
    with pytest.raises(ValueError, match="differs"):
        fit_samples(rows)


def test_tiny_compliance_samples_and_single_direction_are_insufficient():
    rows = [sample(2. + i * .001, 40. + i * .02, direction, j * 3 + i)
            for j, direction in enumerate(("opening", "closing")) for i in range(3)]
    assert fit_samples(rows)["status"] == "insufficient_data"
    assert fit_samples(samples()[:3])["candidate"] is None


def test_large_backlash_and_nonlinearity_fail_affine_model():
    report = fit_samples(samples(backlash=3.))
    assert report["status"] == "rejected"
    assert any("backlash" in issue for issue in report["issues"])
    rows = samples()
    for row in (rows[1], rows[4]):
        row["measurement"]["total_aperture_m"] += .004
    assert fit_samples(rows)["status"] == "rejected"


def test_direction_overlap_restricts_interpolation():
    rows = [sample(x, 20. * x + 10, direction, i * 3 + j)
            for i, direction in enumerate(("opening", "closing"))
            for j, x in enumerate((1. + i * .25, 2. + i * .25, 3. + i * .25))]
    report = fit_samples(rows)
    assert report["candidate"]["interpolation_motor_rad"] == [1.25, 3.]
    with pytest.raises(ValueError, match="Extrapolation"):
        estimate_aperture(report, 1.)


def test_uncertainty_is_explicit_and_must_resolve_the_fit():
    rows = samples()
    rows[0]["measurement"]["uncertainty_m"] = .002
    assert fit_samples(rows)["candidate"] is None
    with pytest.raises(ValueError):
        fit_samples(samples(), max_error_mm=float("nan"))


def test_cli_writes_report_without_hardware_and_preserves_existing_output(tmp_path):
    paths = []
    for index, row in enumerate(samples()):
        path = tmp_path / f"sample-{index}.json"
        path.write_text(json.dumps(row), encoding="utf-8")
        paths.append(str(path))
    output = tmp_path / "fit.json"
    assert main(["fit", *paths, "--output", str(output)]) == 0
    original = output.read_bytes()
    with pytest.raises(SystemExit):
        main(["fit", *paths, "--output", str(output)])
    assert output.read_bytes() == original
