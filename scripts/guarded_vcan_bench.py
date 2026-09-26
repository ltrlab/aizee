"""Exercise the real Linux guarded service on explicitly verified virtual CAN only.

Uses synthetic motor feedback. This is transport/scheduling evidence, not actuator
dynamics, braking, collision-supervisor or physical firmware qualification.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import struct
import subprocess
import threading
import time

from robot_platform.control.guarded import GuardedControllerClient, GuardRejected
from robot_platform.control.guarded_transport import GuardedZmqTransport
from robot_platform.robots import load_robot


def virtual_config(robot="minerva"):
    spec = load_robot(robot)
    names = spec.active_joint_names
    joints = [{"name": n, "unit": "radians", "lower": -3., "upper": 3., "max_velocity": 2.,
               "max_effort": 5., "max_kp": 20., "max_kd": 5., "stop_acceleration": 2.,
               "hold_kp": 10., "hold_kd": 1., "max_tracking_error": .5, "max_temperature": 70.} for n in names]
    # Repeat IDs on different buses intentionally exercises routing isolation.
    motors = [{"name": n, "bus": i // 7, "id": i % 7 + 4,
               "ranges": {"position": 12.5, "velocity": 44., "effort": 17., "kp": 500., "kd": 5.},
               "scale": 1., "zero": 0.} for i, n in enumerate(names)]
    return {"mode": "virtual_can", "endpoint": "tcp://127.0.0.1:5590", "period_us": 2500,
            "maximum_lateness_us": 10000, "host_id": 170, "conflicting_services": [],
            "buses": [{"interface": n, "usb_serial": None} for n in ("vcanGuardL", "vcanGuardR")][:max(m["bus"] for m in motors)+1],
            "motors": motors, "controller": {"group": "virtual_robot", "identity": {
                "model": "synthetic-" + robot, "calibration": "synthetic", "collision_policy": "synthetic-no-collision-enforcement"},
                "lease_ms": 200, "feedback_timeout_ms": 80, "settled_velocity": .01, "joints": joints}}


class Motors:
    def __init__(self, cfg):
        self.cfg, self.sockets = cfg, []
        for bus in cfg["buses"]:
            link = json.loads(subprocess.check_output(["ip", "-details", "-json", "link", "show", bus["interface"]]))[0]
            if link["linkinfo"]["info_kind"] != "vcan":
                raise RuntimeError("Refusing to emulate motors on a physical CAN interface")
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((bus["interface"],)); sock.setblocking(False)
            self.sockets.append(sock)
        self.state = {(m["bus"], m["id"]): {"position": 0., "velocity": 0., "effort": 0., "enabled": False, "fault": False} for m in cfg["motors"]}
        self.freeze, self.done, self.error, self.last_disable = False, False, None, None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.done:
                for bus, sock in enumerate(self.sockets):
                    for _ in range(512):
                        try:
                            raw = sock.recv(16)
                        except BlockingIOError:
                            break
                        arb, length, data = struct.unpack("=IB3x8s", raw)
                        arb &= 0x1fffffff
                        kind, motor_id = arb >> 24, arb & 255
                        state = self.state.get((bus, motor_id))
                        if state is None:
                            continue
                        if kind == 3:
                            state["enabled"] = True
                        elif kind == 4:
                            state.update(enabled=False, velocity=0., effort=0.)
                            self.last_disable = time.monotonic()
                        elif kind == 1 and state["enabled"]:
                            q, v, _, _ = struct.unpack(">4H", data)
                            state.update(position=q / 65535. * 25. - 12.5, velocity=v / 65535. * 88. - 44.,
                                         effort=((arb >> 8) & 65535) / 65535. * 34. - 17.)
                    else:
                        raise RuntimeError("Emulator receive budget exhausted")
                if not self.freeze:
                    for (bus, motor_id), state in self.state.items():
                        values = [(state["position"] + 12.5) / 25., (state["velocity"] + 44.) / 88., (state["effort"] + 17.) / 34.]
                        data = struct.pack(">4H", *(round(max(0., min(1., x)) * 65535) for x in values), 300)
                        arb = (2 << 24) | (int(state["enabled"]) * 2 << 22) | (int(state["fault"]) << 16) | (motor_id << 8) | 170
                        self.sockets[bus].send(struct.pack("=IB3x8s", arb | 0x80000000, 8, data))
                time.sleep(.005)
        except BaseException as exc:
            self.error = repr(exc)

    def close(self):
        self.done = True
        self.thread.join(2)
        for sock in self.sockets:
            sock.close()
        if self.error:
            raise RuntimeError(self.error)


def run(binary, output, cycles, robot):
    cfg = virtual_config(robot)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    config_path = output / "virtual-config.json"
    config_path.write_text(json.dumps(cfg, indent=2))
    subprocess.run([binary, str(config_path), "--validate"], check=True)
    # Verify startup gates before starting the virtual controller.
    negative_checks = {}
    def unpinned_physical_trial(v):
        v.update(mode="zero_torque_trial", motors=v["motors"][:1])
        for b in v["buses"]:
            b.update(usb_serial="fixture")
        v["controller"]["joints"] = v["controller"]["joints"][:1]
        v["controller"]["lease_ms"] = 200
        v["controller"]["joints"][0].update(hold_kp=0., hold_kd=0.)
    for label, mutate, flags, expected in (
        ("virtual_mode_refuses_physical_can", lambda v: v["buses"][0].update(interface="canL"), ["--seconds", ".1"], "virtual mode requires"),
        ("probe_requires_explicit_trial_flag", lambda v: (v.update(mode="parameter_probe"), [b.update(usb_serial="fixture") for b in v["buses"]]), ["--seconds", ".1"], "powered commissioning requires"),
        ("diagnostics_require_explicit_trial_flag", lambda v: (v.update(mode="motor_diagnostics"), [b.update(usb_serial="fixture") for b in v["buses"]]), ["--seconds", ".1"], "powered commissioning requires"),
        ("zero_torque_rejects_multiple_motors", lambda v: (v.update(mode="zero_torque_trial"), [b.update(usb_serial="fixture") for b in v["buses"]]), ["--validate"], "zero-torque trial requires"),
        ("physical_enable_requires_watchdog_pins", unpinned_physical_trial, ["--validate"], "physical enable requires pinned watchdog identities"),
    ):
        candidate = copy.deepcopy(cfg); mutate(candidate)
        rejected_path = output / (label + ".json")
        rejected_path.write_text(json.dumps(candidate))
        rejected = subprocess.run([binary, str(rejected_path), *flags], capture_output=True, text=True, timeout=5)
        assert rejected.returncode != 0 and expected in rejected.stderr, rejected.stderr
        negative_checks[label] = True
    process = subprocess.Popen([binary, str(config_path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    emulator = Motors(cfg)
    result = {"robot": robot, "evidence": "synthetic-vcan", "pid": process.pid, "checks": negative_checks, "cycles_requested": cycles}
    t = GuardedZmqTransport(cfg["endpoint"], timeout_ms=1000)
    c = GuardedControllerClient(t.exchange, group=cfg["controller"]["group"], identity=cfg["controller"]["identity"],
                               joint_names=tuple(j["name"] for j in cfg["controller"]["joints"]))
    def recover():
        state = c.poll()
        if state["lease"] is not None:
            c.emergency_stop()
        c.acquire()
        if c.poll()["state"] in ("faulted", "stopped"):
            c.reset()
        c.arm()
    try:
        for _ in range(50):
            if process.poll() is not None:
                raise RuntimeError(process.stderr.read())
            try:
                c.poll()
                if all(f["sequence"] > 0 for f in t.last["feedback"]):
                    break
            except TimeoutError:
                pass
            time.sleep(.02)
        else:
            raise RuntimeError("Service did not produce fresh complete feedback")
        assert not t.last["hardware_enabled"]
        result["checks"]["startup_disarmed"] = True
        try:
            t.request({"type": "diagnose_motor", "motor_index": 0, "diagnostic": {"kind": "version"}})
            raise AssertionError("diagnostic accepted outside its explicit mode")
        except RuntimeError as exc:
            assert "motor_diagnostic_rejected" in str(exc)
        result["checks"]["diagnostics_rejected_outside_explicit_mode"] = True
        recover()
        q = [0.] * len(cfg["motors"])
        def command(position):
            return [{"position": position if i < 7 else -position, "velocity": 0., "effort": 0., "kp": 10., "kd": 1.} for i in range(len(q))]
        c.move(command(.1))
        # Disconnected producer: no requests while independent receiver ticks expire.
        time.sleep(.3)
        assert c.poll()["state"] == "stopped", t.last
        assert c.status["lease"] is None
        result["checks"]["silent_producer_expires"] = True
        recover()
        authority = c._authority()
        c.move(command(.05))
        replay = t.exchange({"type": "move", "authority": authority, "identity": c.identity, "joints": command(.2)})
        assert not replay["accepted"] and replay["reason"] == "sequence"
        result["checks"]["replay_rejected"] = True
        old_position = replay["status"]["joints"][0]["position"]
        bad = command(.1); bad[-1]["position"] = 200.
        try:
            c.move(bad)
            raise AssertionError("invalid batch accepted")
        except GuardRejected as exc:
            assert exc.reason == "invalid_command"
            assert exc.status["joints"][0]["position"] == old_position
        result["checks"]["batch_rejection_is_complete"] = True
        emulator.freeze = True
        time.sleep(.12)
        assert c.poll()["state"] == "faulted" and not t.last["hardware_enabled"], t.last
        result["checks"]["missing_feedback_disables"] = True
        emulator.freeze = False
        time.sleep(.03)
        recover()
        os.kill(process.pid, signal.SIGSTOP)
        time.sleep(.15)
        os.kill(process.pid, signal.SIGCONT)
        time.sleep(.025)
        assert c.poll()["state"] == "faulted" and not t.last["hardware_enabled"], t.last
        result["checks"]["scheduler_stall_latches_fault"] = True
        recover()
        try:
            t.request({"type": "arm_joints", "positions": [0.] * len(q)})
            raise AssertionError("legacy frame accepted")
        except RuntimeError as exc:
            assert "invalid_guarded_protocol_frame" in str(exc)
        assert c.poll()["state"] == "faulted" and not t.last["hardware_enabled"]
        result["checks"]["legacy_motion_rejected"] = True
        recover()
        old_authority = c._authority()
        old_epoch = c.status["epoch"]
        process.terminate()
        previous_stdout, previous_stderr = process.communicate(timeout=5)
        assert process.returncode == 0, previous_stderr
        time.sleep(.03)
        assert all(not state["enabled"] for state in emulator.state.values())
        result["checks"]["sigterm_disables_virtual_motors"] = True
        result["fault_trial_summary"] = json.loads(previous_stdout)
        t.close()
        process = subprocess.Popen([binary, str(config_path)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        t = GuardedZmqTransport(cfg["endpoint"], timeout_ms=1000)
        for _ in range(50):
            status = t.exchange({"type": "status"})["status"]
            if all(f["sequence"] > 0 for f in t.last["feedback"]): break
            time.sleep(.02)
        assert status["epoch"] != old_epoch
        reply = t.exchange({"type": "arm", "authority": old_authority})
        assert not reply["accepted"] and reply["reason"] == "wrong_epoch"
        result["checks"]["restart_rejects_old_authority"] = True
        c = GuardedControllerClient(t.exchange, group=cfg["controller"]["group"], identity=cfg["controller"]["identity"],
                                   joint_names=tuple(j["name"] for j in cfg["controller"]["joints"]))
        recover()
        before = dict(t.last["timing"])
        durations, misses, faults = [], 0, 0
        start = time.monotonic()
        for i in range(cycles):
            target = start + i / 25.
            time.sleep(max(0., target - time.monotonic()))
            began = time.monotonic()
            c.renew()
            c.move(command(.05 * math.sin(i * .01)))
            ended = time.monotonic()
            durations.append((ended - began) * 1000.)
            misses += ended > target + .04
            if c.status["state"] != "executing": faults += 1
        c.emergency_stop()
        c.poll()
        result.update(cycles_completed=len(durations), cycle_p50_ms=statistics.median(durations),
                      cycle_p99_ms=sorted(durations)[min(len(durations)-1, int(len(durations)*.99))],
                      cycle_max_ms=max(durations), scheduled_40ms_misses=misses, nominal_faults=faults,
                      control_timing_before=before, control_timing_after=t.last["timing"],
                      tx_frames=t.last["tx_frames"], rx_frames=t.last["rx_frames"])
        assert faults == 0
        result["checks"]["25hz_guarded_commands"] = misses == 0
        result["success"] = all(result["checks"].values())
    except BaseException as exc:
        result["success"], result["error"] = False, repr(exc)
        result["last"] = t.last
        raise
    finally:
        t.close()
        if process.poll() is None:
            os.kill(process.pid, signal.SIGCONT)
            process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); stdout, stderr = process.communicate()
        result.update(service_stdout=stdout, service_stderr=stderr, service_exit=process.returncode)
        emulator.close()
        (output / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cycles", type=int, default=1000)
    parser.add_argument("--robot", default="minerva")
    args = parser.parse_args()
    if args.cycles < 2:
        parser.error("at least two cycles required")
    if not run(str(Path(args.binary).resolve()), args.output, args.cycles, args.robot)["success"]:
        raise SystemExit(1)
