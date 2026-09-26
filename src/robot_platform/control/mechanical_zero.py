"""Explicit, finite ROBSTRIDE reference maintenance while every drive is disabled.

This is not homing by motion or magnetic encoder calibration. An operator must
already have placed the robot in a known physical reference pose. No enable,
control, mode change, fault clear, or arbitrary parameter write exists here.
"""
import math
import socket
import struct
import time
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import os
from datetime import datetime, timezone


ZERO_DATA = bytes([1, 0, 0, 0, 0, 0, 0, 0])
SAVE_DATA = bytes(range(1, 9))
READ_PARAMETERS = {0x7005, 0x7019, 0x701B, 0x2021, 0x2024}


def maintenance_frame(motor_id, host, kind, parameter=None):
    if not 0 < motor_id < 128 or not 0 < host < 256 or motor_id == host:
        raise ValueError("Invalid motor/host identity")
    data = bytes(8)
    if kind == "identity":
        code = 0
    elif kind == "stop":
        code = 4
    elif kind == "version":
        code, data = 4, bytes([0, 0xC4, 0, 0, 0, 0, 0, 0])
    elif kind == "read" and parameter in READ_PARAMETERS:
        code, data = 17, struct.pack("<H", parameter) + bytes(6)
    elif kind == "zero":
        code, data = 6, ZERO_DATA
    elif kind == "save":
        code, data = 22, SAVE_DATA
    else:
        raise ValueError("Unsupported maintenance operation")
    return (code << 24) | (host << 8) | motor_id, data


class StoppedMaintenance:
    """Injected exchange returns fresh replies; zero/save writes are never retried."""
    def __init__(self, exchange, *, host=0xAA, sleep=time.sleep):
        self.exchange, self.host, self.sleep = exchange, host, sleep

    def request(self, motor, kind, parameter=None):
        frame = maintenance_frame(motor["id"], self.host, kind, parameter)
        try:
            arb, data = self.exchange(motor["bus"], frame, kind, parameter)
        except TimeoutError:
            if kind in ('zero', 'save'):
                raise
            # One bounded repeat of an idempotent stopped diagnostic. Transport
            # drains prior queued frames before each transaction. This never
            # repeats mechanical zero, flash save, or any powered command.
            self.sleep(.02)
            arb, data = self.exchange(motor["bus"], frame, kind, parameter)
        if len(data) != 8:
            raise RuntimeError("Incomplete maintenance reply")
        if kind == "identity":
            if arb != (motor["id"] << 8) | 0xFE:
                raise RuntimeError("Wrong identity reply")
            return data.hex()
        if kind == 'save' and arb == (motor['id'] << 8) | 0xFE:
            # RS03 0.3.1.30 was observed returning its UID after type 22,
            # unlike the manual's type-2 reply. This is a pinned response,
            # NOT evidence that flash survives power loss. Recheck Reset/q
            # below, and require a separate physical power-cycle test.
            if data.hex() != motor['uid']:
                raise RuntimeError('Wrong UID in save response')
            return {'kind': 'identity', 'uid': data.hex(), 'persistence_verified': False}
        if kind == "read":
            if (arb != (17 << 24) | (motor["id"] << 8) | self.host
                    or int.from_bytes(data[:2], "little") != parameter):
                raise RuntimeError("Wrong parameter reply")
            value = data[4] if parameter == 0x7005 else struct.unpack_from("<f", data, 4)[0]
            if not math.isfinite(value):
                raise RuntimeError("Nonfinite parameter")
            return value
        if (arb >> 24 != 2 or ((arb >> 8) & 255) != motor["id"]
                or (arb & 255) not in ((self.host, motor["id"]) if kind == "version" else (self.host,))
                or (arb >> 22) & 3 or (arb >> 16) & 63):
            raise RuntimeError("Drive did not confirm fault-free Reset")
        version = data[:3] == bytes([0, 0xC4, 0x56])
        if kind == "version":
            if not version:
                raise RuntimeError("Not a version reply")
            return ".".join(str(x) for x in data[3:7])
        if version:
            raise RuntimeError("Version is not position feedback")
        return {"position_u16": int.from_bytes(data[:2], "big"), "mode": 0, "fault": 0}

    def stationary(self, motor):
        samples = []
        # Reset the quiet interval on an outlier; never widen its bounds. A
        # bounded observation window allows velocity-estimator noise to settle
        # without retrying any zero/save operation or accepting a moving pose.
        for _ in range(20):
            self.request(motor, "stop")
            q = self.request(motor, "read", 0x7019)
            v = self.request(motor, "read", 0x701B)
            if abs(v) > .2:
                samples = []
            else:
                samples.append(q)
                if max(samples) - min(samples) > .002:
                    samples = [q]
                if len(samples) >= 6:
                    return samples
            self.sleep(.02)
        raise RuntimeError(f"{motor['name']}: no continuous quiet reference interval; leave disabled")

    def run(self, motors, report, *, reference_confirmed=False, save=False, previous=None, checkpoint=lambda: None):
        if not reference_confirmed or not motors or len(motors) > 32:
            raise ValueError("Explicit operator reference-pose confirmation and roster required")
        names, addresses = set(), set()
        for m in motors:
            if (m["name"] in names or (m["bus"], m["id"]) in addresses
                    or len(bytes.fromhex(m["uid"])) != 8 or not m["firmware"]):
                raise ValueError("Invalid or duplicate pinned motor identity")
            maintenance_frame(m["id"], self.host, "identity")
            names.add(m["name"])
            addresses.add((m["bus"], m["id"]))
        report.update(success=False, enable_requested=False, save_requested=save,
                      reference_confirmed=True, motors=[], error=None,
                      power_cycle_verified=False)
        prior = {}
        if previous is not None:
            previous_motors = previous.get('motors', [])
            if len(previous_motors) != len(motors):
                raise ValueError('Resume requires complete prior roster verification')
            for m, old in zip(motors, previous_motors):
                if any(old.get(k) != m[k] for k in ('name', 'bus', 'id', 'uid', 'firmware')):
                    raise ValueError('Resume identity/roster mismatch')
                if old.get('zero_requested') and not old.get('zero_reply'):
                    raise ValueError('Ambiguous write cannot be automatically resumed')
                prior[m['name']] = old
        try:
            # Verify the ENTIRE roster before the first reference/flash write.
            for m in motors:
                row = {"name": m["name"], "bus": m["bus"], "id": m["id"]}
                report["motors"].append(row)
                row["uid"] = self.request(m, "identity")
                row["firmware"] = self.request(m, "version")
                if row["uid"] != m["uid"] or row["firmware"] != m["firmware"]:
                    raise RuntimeError(f"{m['name']}: UID/firmware mismatch")
                self.request(m, "stop")
                row["run_mode"] = self.request(m, "read", 0x7005)
                if row["run_mode"] != 0:
                    raise RuntimeError("Mechanical zero requires verified operation-control mode")
                row["before_rad"] = self.stationary(m)
            checkpoint()
            for m, row in zip(motors, report["motors"]):
                current = self.stationary(m)
                if abs(current[-1] - row["before_rad"][-1]) > .002:
                    raise RuntimeError(f"{m['name']}: pose changed after preflight")
                old = prior.get(m['name'], {})
                if old.get('zero_reply'):
                    if max(abs(q) for q in current) > .01:
                        raise RuntimeError('Previously zeroed motor reference changed; cannot resume')
                    row['zero_already_acknowledged'] = True
                    row['zero_reply'] = old['zero_reply']
                else:
                    row["zero_requested"] = True  # Preserve ambiguous sends; never automatically retry.
                    checkpoint()
                    row["zero_reply"] = self.request(m, "zero")
                    checkpoint()
                    self.sleep(.25)
                row["after_zero_rad"] = self.stationary(m)
                if max(abs(q) for q in row["after_zero_rad"]) > .01:
                    raise RuntimeError(f"{m['name']}: mechanical zero readback failed")
                if save:
                    if old.get('save_requested') or old.get('save_already_attempted') or old.get('save_reply'):
                        # Never repeat a flash write, including one with a lost
                        # response. Fresh stopped readback checks RAM only;
                        # persistence remains pending the physical power cycle.
                        row['save_already_attempted'] = True
                        if old.get('save_reply'):
                            row['save_already_acknowledged'] = True
                            row['save_reply'] = old['save_reply']
                        else:
                            row['save_response_missing'] = True
                    else:
                        row["save_requested"] = True
                        checkpoint()
                        try:
                            row["save_reply"] = self.request(m, "save")
                        except TimeoutError:
                            row['save_response_missing'] = True
                        checkpoint()
                        self.sleep(.25)
                    row["after_save_rad"] = self.stationary(m)
                    if max(abs(q) for q in row["after_save_rad"]) > .01:
                        raise RuntimeError(f"{m['name']}: reference changed after flash save")
            report["success"] = True
        except Exception as exc:
            report["error"] = str(exc)
        finally:
            report["final_stops"] = {}
            for m in motors:
                try:
                    report["final_stops"][m["name"]] = self.request(m, "stop")
                except Exception as exc:
                    report["final_stops"][m["name"]] = {"error": str(exc)}
                    report["success"] = False
        return report


class SocketCanMaintenance:
    """Linux-only bounded transactions. Caller verifies host, adapters and ownership.

    Socket creation does not bring interfaces up. All transmitted frames pass the
    maintenance allowlist again at this boundary. No queued response is freshened.
    """
    def __init__(self, interfaces, events, *, host=0xAA):
        self.sockets, self.events, self.host = {}, events, host
        self.last_tx = 0.
        try:
            for interface in interfaces:
                sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sockets[interface] = sock
                sock.settimeout(.2)
                sock.bind((interface,))
        except Exception:
            self.close()
            raise

    def close(self):
        for sock in self.sockets.values():
            sock.close()
        self.sockets.clear()

    def __call__(self, bus, frame, kind, parameter):
        arb, data = frame
        motor = arb & 255
        if frame != maintenance_frame(motor, self.host, kind, parameter):
            raise ValueError("Frame not in maintenance allowlist")
        # Maintenance is not a control loop. Allow firmware time between
        # parameter reads instead of bursting transactions at USB/CAN speed.
        time.sleep(max(0., self.last_tx + .005 - time.perf_counter()))
        sock = self.sockets[bus]
        sock.setblocking(False)
        for _ in range(256):
            try:
                sock.recv(16)
            except BlockingIOError:
                break
        else:
            raise RuntimeError("CAN backlog before maintenance transaction")
        sock.settimeout(.2)
        self.events.append({"direction": "tx", "bus": bus, "id": arb, "data": list(data),
                            "at": time.perf_counter(), "kind": kind})
        if sock.send(struct.pack("=IB3x8s", arb | 0x80000000, 8, data)) != 16:
            raise RuntimeError("Incomplete CAN write")
        self.last_tx = time.perf_counter()
        deadline = time.perf_counter() + .2
        for _ in range(256):
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                break
            sock.settimeout(remaining)
            raw = sock.recv(16)
            if len(raw) != 16:
                raise RuntimeError("Incomplete CAN frame")
            rxid, size, rxdata = struct.unpack("=IB3x8s", raw)
            self.events.append({"direction": "rx", "bus": bus, "id": rxid,
                                "data": list(rxdata), "at": time.perf_counter()})
            if rxid & 0x60000000 or not rxid & 0x80000000 or size != 8:
                raise RuntimeError("Unexpected CAN flags/length")
            rxid &= 0x1FFFFFFF
            if kind == "identity":
                matches = rxid == (motor << 8) | 0xFE
            elif kind == "read":
                matches = (rxid == (17 << 24) | (motor << 8) | self.host
                           and int.from_bytes(rxdata[:2], "little") == parameter)
            else:
                matches = ((kind == 'save' and rxid == (motor << 8) | 0xFE) or
                           (rxid >> 24 == 2 and (rxid >> 8) & 255 == motor
                           and (rxdata[:3] == bytes([0, 0xC4, 0x56])) == (kind == "version"))
                           )
            if matches:
                return rxid, rxdata
        raise TimeoutError("Maintenance reply deadline exceeded")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="Pinned Linux host and CAN adapter identity JSON")
    parser.add_argument("--roster", required=True, help="Ordered motors with pinned UID and firmware")
    parser.add_argument("--output", required=True)
    parser.add_argument("--confirm-reference-pose", action="store_true")
    parser.add_argument("--save", action="store_true", help="Explicitly save the current motor parameters after zeroing")
    parser.add_argument("--resume-evidence", help="Explicit prior JSON report; acknowledged writes are never repeated")
    args = parser.parse_args()
    if not args.confirm_reference_pose:
        parser.error("Place every motor at its documented reference pose and explicitly confirm it")
    device = json.loads(Path(args.device).read_text())
    roster_data = json.loads(Path(args.roster).read_text())
    motors = roster_data["motors"]
    previous = json.loads(Path(args.resume_evidence).read_text()) if args.resume_evidence else None
    if previous is not None and previous.get('reference') != roster_data:
        parser.error('Resume reference definition differs from the original action')
    if not roster_data.get("reference_description") or not roster_data.get("model_revision"):
        parser.error("Roster must identify the physical reference and CAD model revision")
    if Path('/etc/machine-id').read_text().strip() != device['machine_id']:
        raise RuntimeError("Wrong host machine identity")
    if Path('/proc/device-tree/serial-number').read_bytes().rstrip(b'\0\n').decode() != device['module_serial']:
        raise RuntimeError("Wrong Jetson module identity")
    interfaces = [b["interface"] for b in device["buses"]]
    if any(m['bus'] not in interfaces for m in motors):
        raise RuntimeError("Unknown CAN bus in roster")
    for bus in device["buses"]:
        target = (Path('/sys/class/net') / bus['interface'] / 'device').resolve()
        actual = next((p / 'serial').read_text().strip() for p in target.parents if (p / 'serial').is_file())
        if actual != bus['usb_serial']:
            raise RuntimeError("Wrong USB CAN adapter identity")
    for service in device['conflicting_services']:
        pid = subprocess.check_output(['systemctl', 'show', service, '--property=MainPID', '--value'], text=True, timeout=2).strip()
        if pid != '0':
            raise RuntimeError("Conflicting motor service is running")
    if subprocess.check_output(['ss', '-H', '-ltn', 'sport', '=', ':5590'], text=True, timeout=2).strip():
        raise RuntimeError("Guarded receiver already running")
    # Process-local operator action, never a network command or startup behavior.
    import fcntl
    with open('/tmp/robot-platform-mechanical-reference.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with Path(args.output).open('x', encoding='utf-8') as output:
            result = {"schema": 1, "started_at": datetime.now(timezone.utc).isoformat(),
                      "device": device['asset_id'], "reference": roster_data,
                      "roster_sha256": hashlib.sha256(Path(args.roster).read_bytes()).hexdigest(),
                      "events": [], "success": False, "enable_requested": False}
            if previous is not None:
                result['resumed_from_sha256'] = hashlib.sha256(Path(args.resume_evidence).read_bytes()).hexdigest()
            def checkpoint():
                output.seek(0)
                output.write(json.dumps(result, indent=2) + '\n')
                output.truncate()
                output.flush()
                os.fsync(output.fileno())
            io = None
            try:
                io = SocketCanMaintenance(interfaces, result['events'])
                StoppedMaintenance(io).run(motors, result, reference_confirmed=True,
                                          save=args.save, previous=previous, checkpoint=checkpoint)
            except Exception as exc:
                result['error'] = repr(exc)
            finally:
                if io is not None:
                    io.close()
                result['finished_at'] = datetime.now(timezone.utc).isoformat()
                checkpoint()
            print(json.dumps({k: result.get(k) for k in ('success', 'error', 'enable_requested', 'power_cycle_verified')}))
            if not result['success']:
                raise SystemExit(1)


if __name__ == '__main__':
    main()
