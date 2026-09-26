"""Generate observation/diagnostic profiles; never turn guessed hardware limits into motion approval."""
import argparse
import hashlib
import json
from pathlib import Path

from robot_platform.robots import load_robot

# Official RS-series manuals (260713): wire velocity (rad/s), torque (Nm), Kp and Kd
# spans per model. Generating a profile from them does not verify hardware. The
# inherited RS00 (2 Nm/100 Kp) and RS03 (50 rad/s) ranges were incorrect.
RANGES = {"ROBSTRIDE00": (33., 14., 500., 5.), "ROBSTRIDE02": (44., 17., 500., 5.),
          "ROBSTRIDE03": (20., 60., 5000., 100.), "ROBSTRIDE04": (15., 120., 5000., 100.)}
POSITION_SPAN = 4 * 3.141592653589793


def wire_ranges(model):
    """The RobStride MIT frame scaling for one motor model."""
    velocity, effort, kp, kd = RANGES[model]
    return {"position": POSITION_SPAN, "velocity": velocity, "effort": effort, "kp": kp, "kd": kd}


def motor_models(device, root):
    """{motor name: RobStride model} from each of the device's bus hardware profiles."""
    import yaml
    models = {}
    for bus in device["buses"]:
        source = yaml.safe_load((Path(root) / bus["hardware_profile"]).read_text(encoding="utf-8"))
        roster = [*source["motors"].get("wheels", []), source["motors"]["swivel"], *source["motors"]["arm"]]
        models.update({motor["id"]: motor["type"] for motor in roster})
    if any(model not in RANGES for model in models.values()):
        raise ValueError("Unknown RobStride model in the hardware profiles")
    return models


def profile(device_path, root, *, mode="observe", endpoint="tcp://127.0.0.1:5590"):
    import yaml
    if mode not in ("observe", "parameter_probe", "motor_diagnostics"):
        raise ValueError("Generator supports observation, parameter reads and stopped-motor diagnostics only")
    device = json.loads(Path(device_path).read_text())
    root = Path(root).resolve()
    spec = load_robot(device["robot"])
    motors, joints, buses, evidence = [], [], [], bytearray()
    # These diagnostic modes still prohibit enabling or commanding motion.
    for bus_index, bus in enumerate(device["buses"]):
        path = (root / bus["hardware_profile"]).resolve()
        if not path.is_relative_to(root):
            raise ValueError("Hardware profile must be within repository")
        raw = path.read_bytes()
        evidence.extend(raw)
        source = yaml.safe_load(raw)
        buses.append({"interface": bus["interface"], "usb_serial": bus["usb_serial"]})
        roster = [*source["motors"].get("wheels", []), source["motors"]["swivel"], *source["motors"]["arm"]]
        for motor in roster:
            velocity, effort, kp, kd = RANGES[motor["type"]]
            name = motor["id"]
            motors.append({"name": name, "bus": bus_index, "id": motor["can_id"],
                           "ranges": wire_ranges(motor["type"]), "scale": 1., "zero": 0.})
            joints.append({"name": name, "unit": "radians", "lower": -4 * 3.141592653589793,
                           "upper": 4 * 3.141592653589793, "max_velocity": velocity, "max_effort": effort,
                           "max_kp": kp, "max_kd": kd, "stop_acceleration": 1., "hold_kp": 0., "hold_kd": 0.,
                           "max_tracking_error": 0.1, "max_temperature": 70.})
    # Validate roster against shared robot definition without depending on arm count.
    expected = {joint.name for joint in spec.joints if joint.active}
    if {m["name"] for m in motors} != expected:
        raise ValueError("Device roster does not match active robot joints")
    return {"mode": mode, "endpoint": endpoint, "period_us": 2500, "maximum_lateness_us": 10000,
            "host_id": 170, "buses": buses, "conflicting_services": device["conflicting_services"], "motors": motors,
            "controller": {"group": device["asset_id"],
                "identity": {"model": hashlib.sha256(evidence).hexdigest(),
                             "calibration": "unverified-observation-only", "collision_policy": "physical-motion-blocked"},
                "lease_ms": 200, "feedback_timeout_ms": 80, "settled_velocity": 0.01, "joints": joints}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--root", default=".")
    parser.add_argument("--mode", choices=("observe", "parameter_probe", "motor_diagnostics"), default="observe")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    result = profile(args.device, args.root, mode=args.mode)
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
    print(f"Wrote {args.mode} profile: {args.output}. Joint motion is blocked.")


if __name__ == "__main__":
    main()
