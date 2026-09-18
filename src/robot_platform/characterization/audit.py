"""Hardware readiness is evidence, never inferred from synthetic profiles."""
import hashlib
import json
import math
from pathlib import Path


# Section 4.1.2, 29-bit private protocol. Not the manuals' 11-bit MIT chapter.
DOCUMENTED = {
    "RS00": {"velocity": 33, "torque": 14, "kp": 500, "kd": 5, "pdf_page": 39},
    "RS02": {"velocity": 44, "torque": 17, "kp": 500, "kd": 5, "pdf_page": 43},
    "RS03": {"velocity": 20, "torque": 60, "kp": 5000, "kd": 100, "pdf_page": 42},
    "RS04": {"velocity": 15, "torque": 120, "kp": 5000, "kd": 100, "pdf_page": 45},
}


def audit_repository(repo):
    import yaml
    repo = Path(repo)
    issues, manuals, controllers = [], {}, {}
    for model, reference in DOCUMENTED.items():
        path = repo / "docs" / f"{model}User Manual260112.pdf"
        manuals[model] = {**reference, "path": str(path.relative_to(repo)),
                          "sha256": hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None}
    # Keep mismatch evidence explicit; do not silently alter a deployed codec.
    issues.extend([
        {"id": "RS00_SCALE", "detail": "Deployed velocity/torque/Kp = 30/2/100; bundled manual = 33/14/500. Firmware match required."},
        {"id": "RS03_SCALE", "detail": "Deployed velocity = 50; bundled manual = 20. Firmware match required."},
        {"id": "PHYSICAL_IDENTITY", "detail": "Per-actuator model, firmware, signs, and shaft reference need device evidence."},
        {"id": "STOP_VALIDATION", "detail": "Dual-controller bridge is implemented; physical stop response has not been measured."},
        {"id": "TIMING_VALIDATION", "detail": "Configured rates are not measurements; collect source-timestamped CAN traces."},
        {"id": "TORQUE_REFERENCE", "detail": "Initial torque source is motor telemetry; independent output torque is unavailable."},
    ])
    for side in ("left", "right"):
        path = repo / "config" / f"hardware_minerva_{side}.yaml"
        cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
        controllers[side] = {"control": cfg["control"], "interfaces": cfg["can"]["interfaces"],
                             "endpoints": cfg["network"]["device"]["zmq"],
                             "motors": [cfg["motors"]["swivel"], *cfg["motors"]["arm"]]}
    calibration = json.loads((repo / "config/minerva_calibration.json").read_text(encoding="utf-8"))
    for name, joint in calibration.get("joints", {}).items():
        lo, hi = joint.get("min_rad"), joint.get("max_rad")
        if type(lo) not in (int, float) or type(hi) not in (int, float) or not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
            issues.append({"id": "DIRECTIONAL_BOUNDS", "joint": name,
                           "detail": "Bounds are not a finite increasing interval; confirm sign and usable travel before conversion.",
                           "recorded_min": lo, "recorded_max": hi})
    issues.append({"id": "APPROVED_ENVELOPE", "detail": "YAML ranges remain placeholders until a verified bench profile is supplied."})
    return {"schema_version": 1, "hardware_ready": False, "simulation_ready": True,
            "evidence": "repository/manual audit only", "manuals": manuals,
            "controllers": controllers, "issues": issues}


def validate_bench_profile(profile, joint):
    """Require an explicitly verified per-device envelope before opening CAN.

    This validates the record, not the truth of the operator's measurements.
    The present bench codec is legacy_deployed; conflicting manuals cannot be
    selected implicitly. Matching firmware/captured-frame evidence is mandatory.
    """
    if profile.get("schema_version") != 1:
        raise ValueError("bench profile requires schema_version=1")
    if profile.get("protocol") != "legacy_deployed":
        raise ValueError("bench runner currently supports verified legacy_deployed codec only")
    for key in ("identity", "protocol", "limits", "stop"):
        evidence = profile.get("verification", {}).get(key, {})
        if evidence.get("verified") is not True or not evidence.get("evidence"):
            raise ValueError(f"bench profile needs verified {key} and evidence")
    for key in ("name", "model", "can_id", "bus"):
        if profile.get(key) != getattr(joint, key):
            raise ValueError(f"bench profile {key} does not match selected device")
    if not profile.get("firmware") or not profile.get("coordinate_reference"):
        raise ValueError("firmware and coordinate_reference must be recorded")
    limits = profile.get("limits", {})
    for key in ("min_position", "max_position", "max_velocity", "max_reported_torque", "max_feedforward", "max_kp", "max_kd", "feedback_timeout_s"):
        value = limits.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"finite limit required: {key}")
        if key not in ("min_position", "max_position") and value <= 0:
            raise ValueError(f"positive limit required: {key}")
    if limits["min_position"] >= limits["max_position"]:
        raise ValueError("position limits must be increasing; do not sort directional calibration bounds blindly")
    return limits
