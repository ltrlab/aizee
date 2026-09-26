"""Full local collision supervisor and independently clocked Linux receiver on vcan.

Uses synthetic leaders and motors, never physical CAN. Not a loaded robot trial.
The model/stop parameters come from the explicitly synthetic benchmark fixture.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np

from benchmark_guarded_teleop import synthetic_config
from guarded_vcan_bench import Motors
from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler
from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.control.guarded_transport import GuardedZmqTransport
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.native_collision import NativeCollisionBackend
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
from robot_platform.teleop.leader_mapping import LeaderTargetMapper
from robot_platform.teleop.openrb import LeaderSample
from robot_platform.teleop.supervisor import GuardedTeleopSupervisor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=1000)
    args = parser.parse_args()
    if args.output.exists() or not 1 <= args.cycles <= 45000:
        parser.error("New output directory and 1-45000 cycles required")
    args.output.mkdir(parents=True)
    profile = ModelProfile.load(args.model)
    model = UrdfModel(profile)
    backend = NativeCollisionBackend(RuntimeCollisionModel(profile), geometry_mode="keepout")
    cfg, mapping, routes = synthetic_config(model, backend)
    service = dict(mode="virtual_can", endpoint="tcp://127.0.0.1:5591", period_us=2500,
        maximum_lateness_us=10000, host_id=170, conflicting_services=[],
        buses=[dict(interface=name, usb_serial=None) for name in ("vcanGuardL", "vcanGuardR")],
        controller=cfg, motors=[dict(name=j["name"], bus=i // 7, id=i % 7 + 4,
            ranges=dict(position=12.5, velocity=44., effort=17., kp=500., kd=5.),
            scale=.01 if j["unit"] == "metres" else 1., zero=0.) for i, j in enumerate(cfg["joints"])])
    config_path = args.output / "virtual-config.json"
    config_path.write_text(json.dumps(service, indent=2))
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping, compute_budget_s=.025)
    initial = {name: model.by_name[name]["default"] for name in model.independent}
    for _ in range(10):
        backend.proximity(initial, scope="inter_branch")
    emulator = Motors(service)  # Verifies kernel interface kind=vcan BEFORE sockets.
    for m in service["motors"]:
        emulator.state[(m["bus"], m["id"])]["position"] = initial[m["name"]] / m["scale"]
    process, transport, supervisor = None, None, None
    report = dict(physical_validation=False, limitation=__doc__, config=service,
                  ignored_links=backend.ignored_links, query_scope="inter_branch", cycles=[], failure=None)
    try:
        process = subprocess.Popen([str(args.binary.resolve()), str(config_path.resolve()),
                                    "--seconds", str(args.cycles * .04 + 20)],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        transport = GuardedZmqTransport(service["endpoint"], timeout_ms=100)
        control = GuardedControllerClient(transport.exchange, group=cfg["group"],
            identity=cfg["identity"], joint_names=tuple(model.independent))
        for _ in range(50):
            if process.poll() is not None:
                raise RuntimeError(process.stderr.read())
            try:
                control.poll()
                if all(f["sequence"] > 0 for f in transport.last["feedback"]):
                    break
            except TimeoutError:
                pass
            time.sleep(.02)
        else:
            raise RuntimeError("No complete virtual feedback")
        # vcan startup can tick before its first complete frame roster. Recover
        # this specific initialization fault once, deliberately in the fixture;
        # the live supervisor has no automatic reset/rearm path.
        if control.status["state"] == "faulted" and control.status["reason"] == "feedback_invalid":
            report["startup_fault"] = control.status
            control.acquire()
            control.reset()
            control.release()
        if control.status["state"] != "disarmed":
            raise RuntimeError("Receiver startup is not disarmed")
        supervisor = GuardedTeleopSupervisor(control, compiler,
            LeaderTargetMapper(cfg["joints"], routes), lambda: transport.last)
        def sample(index):
            now = time.perf_counter()
            q = (.02 * math.sin(index * .04 * .4),) * len(model.independent)
            return {"synthetic": LeaderSample(q, (0,) * len(q), index + 1, now, now, (0, 0, 1, 1), 0, 0)}
        supervisor.begin(sample(0), engaged=True)
        due = time.perf_counter() + .04
        for i in range(1, args.cycles + 1):
            time.sleep(max(0., due - time.perf_counter()))
            record = supervisor.step(sample(i), engaged=True)
            record["scheduled_at"] = due
            report["cycles"].append(record)
            due += .04
        # No requests, renewal, collision or client-side ticking during silence.
        time.sleep(.12)
        status = control.poll()
        report["silent_producer"] = dict(status=status, feedback=transport.last["feedback"])
        assert status["state"] == "stopped" and status["reason"] == "collision_expired", status
        report["timing"] = transport.last["timing"]
    except Exception as exc:
        report["failure"] = repr(exc)
        if supervisor is not None and supervisor.last_cycle not in report["cycles"]:
            if "due" in locals():
                supervisor.last_cycle["scheduled_at"] = due
            report["failed_cycle"] = supervisor.last_cycle
        if transport is not None:
            report["last_receiver"] = transport.last
    finally:
        if supervisor is not None:
            supervisor.stop("benchmark_complete")
        if transport is not None:
            transport.close()
        if process is not None:
            process.terminate()
            out, err = process.communicate(timeout=5)
            report["service_exit"] = dict(code=process.returncode, stdout=out, stderr=err)
        emulator.close()
        backend.close()
    rows = report["cycles"]
    durations = [(r["completed_at"] - r["started_at"]) * 1000 for r in rows]
    report["summary"] = dict(cycles=len(rows), accepted=sum(r["accepted"] is True for r in rows),
        p50_ms=float(np.percentile(durations, 50)) if rows else None,
        p99_ms=float(np.percentile(durations, 99)) if rows else None,
        maximum_ms=max(durations, default=None), failure=report["failure"],
        scheduled_deadline_misses=sum(r["completed_at"] > r["scheduled_at"] + .04 for r in rows),
        silent_producer_stop_verified=report.get("silent_producer", {}).get("status", {}).get("reason") == "collision_expired")
    (args.output / "result.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(report["summary"], indent=2))
    if report["failure"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
