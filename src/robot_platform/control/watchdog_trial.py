"""Prepare a pinned single-motor watchdog, bounded jog or breakaway profile.

This generates a file only. The Linux motor_watchdog_trial executable conducts
the finite experiment, writes a volatile nonzero watchdog and never saves flash.
Motion experiments require a separate explicit flag and a successful watchdog
test first. Zero-torque profiles include the guarded service's startup pins;
use their service object for that executable. Firmware behavior qualification
must be established separately; diagnostic readback alone is not qualification.
"""
import argparse
import copy
import json
import math
from pathlib import Path

from robot_platform.control.commissioning import profile


def trial_profile(device_path, root, diagnostics, motor_name, *, jog_radians=None, breakaway_direction=None, damped_stop=False):
    if damped_stop and breakaway_direction is None:
        raise ValueError("Damped stop requires a breakaway experiment")
    if breakaway_direction is not None and (breakaway_direction not in (-1, 1) or jog_radians is not None):
        raise ValueError("Choose a jog OR a breakaway direction of -1 or 1")
    if jog_radians is not None and (not math.isfinite(jog_radians) or not .005 <= abs(jog_radians) <= .03):
        raise ValueError("Jog magnitude must be finite and in .005..0.03 motor radians")
    device = json.loads(Path(device_path).read_text())
    if device.get("motor_models_verified") is not True:
        raise ValueError("Confirm installed motor models before preparing a torque trial")
    if diagnostics.get("evidence") != "hardware-stopped-motor-diagnostics":
        raise ValueError("Requires captured hardware diagnostics")
    config = profile(device_path, root)
    matches = [(i, m) for i, m in enumerate(config["motors"]) if m["name"] == motor_name]
    if len(matches) != 1:
        raise ValueError("Select exactly one configured motor by name")
    index, motor = matches[0]
    bus = config["buses"][motor["bus"]]
    evidence = [m for m in diagnostics["motors"] if m["name"] == motor_name
                and m["bus"] == bus["interface"] and m["can_id"] == motor["id"]]
    if len(evidence) != 1:
        raise ValueError("Diagnostic motor identity/route does not match profile")
    observed = evidence[0]
    if (not observed.get("reset_confirmed") or not observed.get("identity") or not observed.get("version")
            or observed["stop_feedback"]["error_bits"] != 0
            or (observed["parameters"].get("run_mode") or {}).get("value") != 0):
        raise ValueError("Requires identified, fault-free stopped motor in motion-control mode")
    uid = bytes.fromhex(observed["identity"]["mcu_uid_hex"])
    version = [int(n) for n in observed["version"]["firmware"].split(".")]
    if len(uid) != 8 or len(version) != 4 or any(not 0 <= n <= 255 for n in version):
        raise ValueError("Invalid hardware UID or firmware version")
    selected = copy.deepcopy(motor)
    selected["bus"] = 0
    config.update(mode="zero_torque_trial", buses=[bus], motors=[selected])
    config["controller"]["joints"] = [config["controller"]["joints"][index]]
    result = {"service": config, "motor_identity": {"uid": list(uid), "firmware": version}}
    if jog_radians is not None:
        config["mode"] = "bounded_jog_trial"
        result["jog_radians"] = jog_radians
    elif breakaway_direction is not None:
        position = (observed["parameters"].get("motor_position_rad") or {}).get("value")
        if position is None or not math.isfinite(position):
            raise ValueError("Breakaway requires a freshly captured starting position")
        config["mode"] = "breakaway_trial"
        result["breakaway"] = {"direction": breakaway_direction, "expected_position_rad": position}
        if damped_stop:
            result["breakaway"]["stop"] = "damped"
    else:
        config["watchdog_identities"] = [result["motor_identity"]]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--root", default=".")
    parser.add_argument("--diagnostics", required=True)
    parser.add_argument("--motor", required=True)
    parser.add_argument("--jog-radians", type=float, help="Prepare a separate explicit, operator-cleared motor-space micro-jog")
    parser.add_argument("--breakaway-direction", type=int, choices=(-1, 1), help="Prepare a bounded motor-space friction experiment")
    parser.add_argument("--damped-stop", action="store_true", help="Taper drive and damping before disabling after breakaway")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    target = Path(args.output)
    if target.exists():
        parser.error("Use a new output filename")
    result = trial_profile(args.device, args.root, json.loads(Path(args.diagnostics).read_text()), args.motor, jog_radians=args.jog_radians, breakaway_direction=args.breakaway_direction, damped_stop=args.damped_stop)
    target.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Prepared {args.motor} watchdog test profile; no connection or motor command sent.")


if __name__ == "__main__":
    main()
