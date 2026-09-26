"""Bounded read-only motor parameter trial through the guarded receiver.

Requires a separately started parameter_probe service and explicitly selected
profile. Never sends enable, zero, gain, flash-write or motion requests.
"""
import argparse
import json
from pathlib import Path
import struct
import time

from robot_platform.control.guarded_transport import GuardedZmqTransport

PARAMETERS = {0x7019: "motor_position_rad", 0x701B: "motor_velocity_rad_s", 0x701C: "bus_voltage_v"}


def matching_observation(observations, *, bus, motor_id, host_id, parameter, after_ms):
    valid = []
    for row in observations:
        arb = row["id"]
        data = bytes(row["data"])
        if (row["bus"] == bus and arb >> 24 == 17 and (arb & 255) == host_id and ((arb >> 16) & 255) == 0
                and ((arb >> 8) & 255) == motor_id and row["received_at_ms"] >= after_ms
                and len(data) == 8 and int.from_bytes(data[:2], "little") == parameter):
            value = struct.unpack("<f", data[4:])[0]
            import math
            if math.isfinite(value):
                valid.append({"value": value, "received_at_ms": row["received_at_ms"], "raw": row})
    return max(valid, key=lambda row: row["received_at_ms"]) if valid else None


def probe(config, *, timeout=.15):
    if config["mode"] != "parameter_probe":
        raise ValueError("Requires a parameter_probe profile")
    result = {"evidence": "hardware-parameter-read", "enable_requested": False, "motors": []}
    with GuardedZmqTransport(config["endpoint"]) as t:
        status = t.request({"type": "status"})
        if (status["mode"] != "parameter_probe" or status["hardware_enabled"]
                or status["status"]["identity"] != config["controller"]["identity"]
                or status["status"]["joint_names"] != [m["name"] for m in config["motors"]]):
            raise RuntimeError("Receiver/profile mismatch or unexpected enabled state")
        epoch = status["status"]["epoch"]
        for index, motor in enumerate(config["motors"]):
            row = {"name": motor["name"], "can_id": motor["id"], "bus": config["buses"][motor["bus"]]["interface"], "parameters": {}}
            for parameter, name in PARAMETERS.items():
                sent = t.request({"type": "probe_parameter", "motor_index": index, "parameter": parameter})
                after = sent["status"]["now_ms"]
                deadline = time.monotonic() + timeout
                observed = None
                while time.monotonic() < deadline:
                    snapshot = t.request({"type": "status"})
                    if snapshot["status"]["epoch"] != epoch or snapshot["hardware_enabled"]:
                        raise RuntimeError("Receiver restarted or unexpectedly enabled during parameter trial")
                    observed = matching_observation(snapshot["observations"], bus=motor["bus"], motor_id=motor["id"],
                        host_id=config["host_id"], parameter=parameter, after_ms=after)
                    if observed is not None:
                        break
                    time.sleep(.005)
                row["parameters"][name] = observed
            result["motors"].append(row)
        result["tx_frames"] = t.last["tx_frames"]
        result["rx_frames"] = t.last["rx_frames"]
    result["complete"] = all(all(value is not None for value in row["parameters"].values()) for row in result["motors"])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    target = Path(args.output)
    if target.exists():
        parser.error("Use a new output file to preserve trial evidence")
    config = json.loads(Path(args.profile).read_text())
    result = probe(config)
    target.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not result["complete"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
