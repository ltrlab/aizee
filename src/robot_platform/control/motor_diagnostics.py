"""Bounded stopped-motor diagnostics through the guarded receiver.

Identity reads, stop-feedback requests, firmware reads and allowlisted parameter
reads only. Stop/firmware requests can remove torque. No enable, fault clear,
zeroing, gain changes, timeout changes or flash writes. Raw frames are preserved;
parameter units and protocol ranges still require installed-model verification.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import time

from robot_platform.control.guarded_transport import GuardedZmqTransport

PARAMETERS = {
    0x7005: ("run_mode", "B"),
    0x700B: ("torque_limit_nm", "f"),
    0x7017: ("speed_limit_rad_s", "f"),
    0x7018: ("current_limit_a", "f"),
    0x7019: ("motor_position_rad", "f"),
    0x701A: ("filtered_iq_a", "f"),
    0x701B: ("motor_velocity_rad_s", "f"),
    0x701C: ("bus_voltage_v", "f"),
    0x7026: ("report_interval_setting", "H"),
    0x7028: ("can_timeout_ticks", "I"),
}


def matching_diagnostic(observations, *, bus, motor_id, host_id, diagnostic, after_ms):
    """Match complete, correctly addressed replies; never decode errors as values."""
    matches = []
    for row in observations:
        arb, data = row["id"], bytes(row["data"])
        if row["bus"] != bus or row["received_at_ms"] < after_ms or len(data) != 8:
            continue
        kind = diagnostic["kind"]
        value = None
        if kind == "identity":
            if arb == (motor_id << 8) | 0xFE:
                value = {"mcu_uid_hex": data.hex()}
        elif kind in ("version", "stop_feedback"):
            if arb >> 24 != 2 or ((arb >> 8) & 255) != motor_id:
                continue
            version = data[:3] == b"\x00\xc4\x56"
            if kind == "version" and version and (arb & 255) in (host_id, motor_id):
                value = {"firmware": ".".join(str(n) for n in data[3:7]),
                         "mode": (arb >> 22) & 3, "error_bits": (arb >> 16) & 63}
            elif kind == "stop_feedback" and not version and (arb & 255) == host_id:
                # Retain wire values; do not treat unverified torque scaling as evidence.
                value = {"mode": (arb >> 22) & 3, "error_bits": (arb >> 16) & 63,
                         "position_u16": int.from_bytes(data[:2], "big"),
                         "velocity_u16": int.from_bytes(data[2:4], "big"),
                         "effort_u16": int.from_bytes(data[4:6], "big"),
                         "temperature_c": int.from_bytes(data[6:], "big") / 10.}
        elif kind == "parameter":
            parameter = diagnostic["parameter"]
            if parameter not in PARAMETERS:
                raise ValueError("Unapproved diagnostic parameter")
            if (arb == (17 << 24) | (motor_id << 8) | host_id
                    and int.from_bytes(data[:2], "little") == parameter):
                decoded = struct.unpack_from("<" + PARAMETERS[parameter][1], data, 4)[0]
                if math.isfinite(decoded):
                    value = {"value": decoded}
        else:
            raise ValueError("Unknown diagnostic")
        if value is not None:
            matches.append({**value, "received_at_ms": row["received_at_ms"], "raw": row})
    return max(matches, key=lambda row: row["received_at_ms"]) if matches else None


def diagnose(config, *, timeout=.15):
    if config["mode"] != "motor_diagnostics":
        raise ValueError("Requires an explicitly selected motor_diagnostics profile")
    if not math.isfinite(timeout) or not 0 < timeout <= .25:
        raise ValueError("Diagnostic reply timeout must be in (0, .25] seconds")
    result = {"evidence": "hardware-stopped-motor-diagnostics", "enable_requested": False,
              "stop_requested": True, "parameter_writes_requested": False, "motors": []}
    with GuardedZmqTransport(config["endpoint"]) as transport:
        initial = transport.request({"type": "status"})
        if (initial["mode"] != "motor_diagnostics" or initial["hardware_enabled"]
                or initial["status"]["identity"] != config["controller"]["identity"]
                or initial["status"]["joint_names"] != [m["name"] for m in config["motors"]]):
            raise RuntimeError("Receiver/profile mismatch or unexpected enabled state")
        epoch = initial["status"]["epoch"]

        def check(snapshot):
            if (snapshot["status"]["epoch"] != epoch or snapshot["hardware_enabled"]
                    or snapshot["mode"] != "motor_diagnostics"):
                raise RuntimeError("Receiver changed or unexpectedly enabled during diagnostics")

        for index, motor in enumerate(config["motors"]):
            row = {"name": motor["name"], "can_id": motor["id"],
                   "bus": config["buses"][motor["bus"]]["interface"], "parameters": {}}
            result["motors"].append(row)
            operations = [(name, {"kind": name}) for name in ("identity", "stop_feedback", "version")]
            operations += [(name, {"kind": "parameter", "parameter": parameter})
                           for parameter, (name, _) in PARAMETERS.items()]
            for name, diagnostic in operations:
                sent = transport.request({"type": "diagnose_motor", "motor_index": index,
                                          "diagnostic": diagnostic})
                check(sent)
                deadline = time.monotonic() + timeout
                observed = None
                while time.monotonic() < deadline:
                    snapshot = transport.request({"type": "status"})
                    check(snapshot)
                    observed = matching_diagnostic(snapshot["observations"], bus=motor["bus"],
                        motor_id=motor["id"], host_id=config["host_id"], diagnostic=diagnostic,
                        after_ms=sent["status"]["now_ms"])
                    if observed is not None:
                        break
                    time.sleep(.005)
                target = row["parameters"] if diagnostic["kind"] == "parameter" else row
                target[name] = observed
            row["reset_confirmed"] = bool(row["stop_feedback"] and row["stop_feedback"]["mode"] == 0)
        result["tx_frames"] = transport.last["tx_frames"]
        result["rx_frames"] = transport.last["rx_frames"]
        result["timing"] = transport.last["timing"]
    result["complete"] = all(
        row["identity"] and row["version"] and row["reset_confirmed"]
        and all(v is not None for v in row["parameters"].values()) for row in result["motors"])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    target = Path(args.output)
    if target.exists():
        parser.error("Use a new output file to preserve trial evidence")
    result = diagnose(json.loads(Path(args.profile).read_text()))
    target.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"complete": result["complete"], "motors": len(result["motors"]),
                      "tx_frames": result["tx_frames"], "rx_frames": result["rx_frames"]}))
    if not result["complete"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
