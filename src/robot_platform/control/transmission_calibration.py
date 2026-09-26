"""Offline motor-angle to measured aperture calibration; never connects to hardware.

Samples bind a manually measured total jaw gap to stopped-motor diagnostics.
The affine fit is an interpolation candidate, not a mechanical-limit, force,
CAD-zero or powered-motion qualification. No settings are installed by this tool.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from statistics import mean


SCHEMA = "robot-platform.aperture-sample.v1"
MAX_STOPPED_SPEED_RAD_S = .1


def _number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{name} must be finite" + (" and positive" if positive else ""))
    return float(value)


def _timestamp(value):
    if not isinstance(value, str):
        raise ValueError("Diagnostics require a timezone-aware capture timestamp")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError("Diagnostics require a timezone-aware capture timestamp")
    return parsed.astimezone(timezone.utc).isoformat()


def _motor(report, name):
    if (report.get("evidence") != "hardware-stopped-motor-diagnostics"
            or report.get("complete") is not True
            or report.get("enable_requested") is not False
            or report.get("stop_requested") is not True
            or report.get("parameter_writes_requested") is not False):
        raise ValueError("Requires complete stopped diagnostics without enable or parameter writes")
    matches = [row for row in report["motors"] if row["name"] == name]
    if len(matches) != 1:
        raise ValueError("Diagnostics must identify exactly one matching motor")
    row = matches[0]
    if (row.get("reset_confirmed") is not True or row["stop_feedback"]["mode"] != 0
            or row["stop_feedback"]["error_bits"] != 0 or row["version"]["error_bits"] != 0):
        raise ValueError("Motor must report Reset with no fault bits")
    uid = row["identity"]["mcu_uid_hex"]
    firmware = row["version"]["firmware"]
    if not re.fullmatch(r"[0-9a-f]{16}", uid) or not re.fullmatch(r"\d+\.\d+\.\d+\.\d+", firmware):
        raise ValueError("Missing or invalid motor UID/firmware")
    parameters = row["parameters"]
    position = _number(parameters["motor_position_rad"]["value"], "motor position")
    velocity = _number(parameters["motor_velocity_rad_s"]["value"], "motor velocity")
    if abs(velocity) > MAX_STOPPED_SPEED_RAD_S:
        raise ValueError("Reported speed exceeds the stopped-sample limit")
    identity = {key: row[key] for key in ("name", "bus", "can_id")}
    identity.update(mcu_uid_hex=uid, firmware=firmware)
    return identity, position, velocity


def make_sample(source_bytes, *, source, motor, aperture_mm, approach="unknown",
                instrument=None, uncertainty_mm=None, pairing_confirmed=False,
                captured_at=None, note=""):
    """Preserve a measurement even if its instrument/uncertainty is still unknown.

    pairing_confirmed means the operator left the jaws stationary between gap
    measurement and diagnostic capture. Reset alone does not establish this.
    Plain diagnostic-client JSON needs an explicit capture time; SSH envelopes
    already carry started_at. A hash identifies the original source bytes.
    """
    aperture = _number(aperture_mm, "aperture_mm", positive=True) / 1000
    if approach not in ("opening", "closing", "unknown"):
        raise ValueError("Approach must be opening, closing or unknown")
    if uncertainty_mm is not None:
        uncertainty_mm = _number(uncertainty_mm, "uncertainty_mm", positive=True)
    wrapper = json.loads(source_bytes)
    report = wrapper.get("diagnostics", wrapper)
    identity, position, velocity = _motor(report, motor)
    # Keep only the selected motor's raw diagnostic replies in the portable sample.
    selected = {**report, "motors": [row for row in report["motors"] if row["name"] == motor]}
    timestamp = _timestamp(captured_at or wrapper.get("started_at"))
    return {
        "schema": SCHEMA, "identity": identity,
        "source": {"path": str(source), "sha256": hashlib.sha256(source_bytes).hexdigest(),
                   "captured_at": timestamp, "remote_trial": wrapper.get("trial")},
        "measurement": {"total_aperture_m": aperture, "approach": approach,
                        "instrument": instrument,
                        "uncertainty_m": None if uncertainty_mm is None else uncertainty_mm / 1000,
                        "pairing_confirmed": pairing_confirmed, "note": note},
        "motor_position_rad": position, "reported_velocity_rad_s": velocity,
        "diagnostics": selected,
    }


def _line(points):
    mx, my = mean(x for x, _ in points), mean(y for _, y in points)
    denominator = sum((x - mx) ** 2 for x, _ in points)
    if denominator <= 1e-15:
        raise ValueError("Encoder positions do not span a fit")
    slope = sum((x - mx) * (y - my) for x, y in points) / denominator
    return slope, my - slope * mx


def fit_samples(samples, *, min_span_mm=10., max_error_mm=1.):
    """Check both approach directions and produce a bounded offline candidate.

    Default gates are commissioning measurement criteria, not robot limits.
    Opening/closing describe total aperture, independently of motor sign.
    """
    min_span = _number(min_span_mm, "min_span_mm", positive=True) / 1000
    tolerance = _number(max_error_mm, "max_error_mm", positive=True) / 1000
    issues, points, directions, hashes = [], [], {}, set()
    identity = None
    for sample in samples:
        if sample.get("schema") != SCHEMA:
            raise ValueError("Unsupported sample schema")
        found, position, velocity = _motor(sample["diagnostics"], sample["identity"]["name"])
        if (found != sample["identity"] or position != sample["motor_position_rad"]
                or velocity != sample["reported_velocity_rad_s"]):
            raise ValueError("Sample differs from its diagnostic evidence")
        if identity is not None and found != identity:
            raise ValueError("Cannot mix motor routes, UIDs or firmware")
        identity = found
        source = sample["source"]
        _timestamp(source["captured_at"])
        digest = source["sha256"]
        if not re.fullmatch(r"[0-9a-f]{64}", digest) or digest in hashes:
            raise ValueError("Each sample needs a unique diagnostic source hash")
        hashes.add(digest)
        measurement = sample["measurement"]
        aperture = _number(measurement["total_aperture_m"], "aperture", positive=True)
        uncertainty = measurement.get("uncertainty_m")
        if uncertainty is None:
            issues.append("Measurement uncertainty is missing")
        else:
            uncertainty = _number(uncertainty, "uncertainty", positive=True)
            if uncertainty > tolerance / 2 or 10 * uncertainty > min_span:
                issues.append("Measurement uncertainty is too large for the selected fit criteria")
        if not isinstance(measurement.get("instrument"), str) or not measurement["instrument"].strip():
            issues.append("Measurement instrument is missing")
        if measurement.get("pairing_confirmed") is not True:
            issues.append("Stationary pairing of measurement and encoder is unconfirmed")
        direction = measurement["approach"]
        if direction not in ("opening", "closing", "unknown"):
            raise ValueError("Invalid approach direction")
        point = (position, aperture)
        points.append(point)
        directions.setdefault(direction, []).append(point)

    report = {"schema": "robot-platform.aperture-fit.v1", "status": "insufficient_data",
              "identity": identity, "sample_count": len(samples), "source_sha256": sorted(hashes),
              "criteria": {"minimum_aperture_span_m": min_span,
                           "maximum_error_m": tolerance, "samples_per_direction": 3,
                           "minimum_encoder_span_rad": .1},
              "issues": issues, "candidate": None}
    for direction in ("opening", "closing"):
        group = directions.get(direction, [])
        if len({x for x, _ in group}) < 3:
            issues.append(f"Need three distinct encoder positions approached while {direction}")
        elif (max(y for _, y in group) - min(y for _, y in group) < min_span
              or max(x for x, _ in group) - min(x for x, _ in group) < .1):
            issues.append(f"Insufficient measured aperture/encoder span while {direction}")
    if issues:
        report["issues"] = sorted(set(issues))
        return report

    slope, intercept = _line(points)
    residuals = [y - (slope * x + intercept) for x, y in points]
    loo = []
    for i, (x, y) in enumerate(points):
        a, b = _line(points[:i] + points[i + 1:])
        loo.append(abs(y - (a * x + b)))
    lines = {direction: _line(directions[direction]) for direction in ("opening", "closing")}
    low = max(min(x for x, _ in directions[direction]) for direction in lines)
    high = min(max(x for x, _ in directions[direction]) for direction in lines)
    separation = max(abs((lines["opening"][0] - lines["closing"][0]) * x
                         + lines["opening"][1] - lines["closing"][1]) for x in (low, high))
    report["fit"] = {"scale_m_per_rad": slope, "offset_m": intercept,
                     "max_residual_m": max(abs(r) for r in residuals),
                     "rms_residual_m": math.sqrt(mean(r * r for r in residuals)),
                     "max_leave_one_out_error_m": max(loo),
                     "direction_separation_m": separation,
                     "direction_fits": {key: {"scale_m_per_rad": value[0], "offset_m": value[1]}
                                        for key, value in lines.items()}}
    if low >= high or abs(slope) * (high - low) < min_span:
        issues.append("Opening and closing samples lack a shared interpolation span")
    if any(a * slope <= 0 for a, _ in lines.values()):
        issues.append("Opening and closing fits disagree on transmission sign")
    if max(abs(r) for r in residuals) > tolerance or max(loo) > tolerance:
        issues.append("Affine model residual or held-out error exceeds tolerance")
    if separation > tolerance:
        issues.append("Opening/closing separation exceeds the backlash criterion")
    if min(slope * low + intercept, slope * high + intercept) <= 0:
        issues.append("Fit predicts a nonpositive aperture in the sampled range")
    report["status"] = "rejected" if issues else "candidate"
    if not issues:
        report["candidate"] = {
            "scale_m_per_rad": slope, "offset_m": intercept,
            "interpolation_motor_rad": [low, high],
            "interpolation_aperture_m": sorted([slope * low + intercept, slope * high + intercept]),
            "quantity": "total_inner_jaw_gap", "runtime_approved": False,
            "limitations": ["Interpolation only; range endpoints are not mechanical stops",
                            "No motor zero, CAD slider zero, jaw force or loaded-motion validation",
                            "Invalid after encoder zero changes or transmission maintenance"],
        }
    return report


def estimate_aperture(report, motor_position_rad):
    """Evaluate an offline candidate only inside its measured overlap range."""
    if report.get("status") != "candidate" or report.get("candidate") is None:
        raise ValueError("No passing calibration candidate")
    position = _number(motor_position_rad, "motor position")
    candidate = report["candidate"]
    low, high = candidate["interpolation_motor_rad"]
    if not low <= position <= high:
        raise ValueError("Extrapolation beyond measured range is not allowed")
    return candidate["scale_m_per_rad"] * position + candidate["offset_m"]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    sample = commands.add_parser("sample", help="Pair existing diagnostic evidence with a manual gap")
    sample.add_argument("--diagnostics", type=Path, required=True)
    sample.add_argument("--motor", required=True)
    sample.add_argument("--aperture-mm", type=float, required=True)
    sample.add_argument("--approach", choices=("opening", "closing", "unknown"), default="unknown")
    sample.add_argument("--instrument")
    sample.add_argument("--uncertainty-mm", type=float)
    sample.add_argument("--pairing-confirmed", action="store_true")
    sample.add_argument("--captured-at", help="Required for plain diagnostic JSON without started_at")
    sample.add_argument("--note", default="")
    fit = commands.add_parser("fit", help="Check a set of samples; writes a candidate or unmet criteria")
    fit.add_argument("samples", type=Path, nargs="+")
    fit.add_argument("--min-span-mm", type=float, default=10.)
    fit.add_argument("--max-error-mm", type=float, default=1.)
    for command in (sample, fit):
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError("Use a new output file to preserve evidence")
        if args.command == "sample":
            result = make_sample(args.diagnostics.read_bytes(), source=args.diagnostics.name,
                                 motor=args.motor, aperture_mm=args.aperture_mm, approach=args.approach,
                                 instrument=args.instrument, uncertainty_mm=args.uncertainty_mm,
                                 pairing_confirmed=args.pairing_confirmed, captured_at=args.captured_at,
                                 note=args.note)
        else:
            result = fit_samples([json.loads(path.read_text(encoding="utf-8")) for path in args.samples],
                                 min_span_mm=args.min_span_mm, max_error_mm=args.max_error_mm)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps({"output": str(args.output), "status": result.get("status", "sample-recorded"),
                      "issues": result.get("issues", [])}))
    if args.command == "fit" and result["status"] != "candidate":
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
