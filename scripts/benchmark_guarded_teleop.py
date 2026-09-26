"""Paced model/leader-mapper/collision/receiver test; no physical device access.

The native harness has an ideal plant and an explicitly advanced receiver clock.
This measures local software work, not loaded motor behavior or Linux scheduling.
All identity mappings and motion bounds generated here are synthetic fixtures.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import time

import numpy as np

from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler, calibration_revision
from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
from robot_platform.teleop.leader_mapping import LeaderTargetMapper
from robot_platform.teleop.openrb import LeaderSample
from robot_platform.teleop.supervisor import GuardedTeleopSupervisor
from robot_platform.simulation.guarded_fixture import synthetic_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=1000)
    args = parser.parse_args()
    if args.output.exists() or not 1 <= args.cycles <= 45000:
        parser.error("Choose a new output and 1-45000 cycles")
    profile = ModelProfile.load(args.model)
    model, runtime = UrdfModel(profile), RuntimeCollisionModel(profile)
    backend = NativeCollisionBackend(runtime, geometry_mode="keepout")
    cfg, mapping, routes = synthetic_config(model, backend)
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping, compute_budget_s=.025)
    initial = [model.by_name[name]["default"] for name in model.independent]
    records, failure, expiry = [], None, None
    try:
        with NativeControlHarness(cfg, initial) as h:
            epoch = time.perf_counter()
            def exchange(request):
                return h.step(int((time.perf_counter() - epoch) * 1000), request)["reply"]
            c = GuardedControllerClient(exchange, group=cfg["group"], identity=cfg["identity"],
                                        joint_names=tuple(model.independent))
            mapper = LeaderTargetMapper(cfg["joints"], routes)
            supervisor = GuardedTeleopSupervisor(c, compiler, mapper, lambda: h.last)
            def sample(index):
                now = time.perf_counter()
                q = tuple(.02 * math.sin(index * .04 * .4) for _ in model.independent)
                return {"synthetic": LeaderSample(q, (0,) * len(q), index + 1, now, now,
                                                  (0, 0, 1, 1), 0, 0)}
            # Warm caches before starting freshness windows.
            for _ in range(10):
                backend.proximity(dict(zip(model.independent, initial)), scope="inter_branch")
            supervisor.begin(sample(0), engaged=True)
            due = time.perf_counter() + .04
            try:
                for i in range(1, args.cycles + 1):
                    time.sleep(max(0., due - time.perf_counter()))
                    record = supervisor.step(sample(i), engaged=True)
                    record["scheduled_at"] = due
                    records.append(record)
                    due += .04
                # Withhold producer work; advance the receiver beyond its last
                # verdict deadline and require local stop without a new request.
                status = h.last["status"]
                expiry = h.step(status["collision_expires_at_ms"] + 1)["status"]
                assert expiry["reason"] == "collision_expired" and expiry["state"] == "stopped"
            except Exception as exc:
                failure = repr(exc)
                if supervisor.last_cycle not in records:
                    supervisor.last_cycle["scheduled_at"] = due
                    records.append(supervisor.last_cycle)
            finally:
                supervisor.stop("benchmark_complete")
    except Exception as exc:
        failure = repr(exc)
    finally:
        backend.close()
    times = [(r["completed_at"] - r["started_at"]) * 1000 for r in records]
    summary = dict(completed_cycles=len(records), accepted_cycles=sum(r["accepted"] is True for r in records),
                   scheduled_deadline_misses=sum(r["completed_at"] > r.get("scheduled_at", r["started_at"]) + .04 for r in records),
                   p50_ms=float(np.percentile(times, 50)) if times else None,
                   p99_ms=float(np.percentile(times, 99)) if times else None, maximum_ms=max(times, default=None),
                   failure=failure, silent_producer_stop_verified=expiry is not None)
    report = dict(schema_version=1, platform=platform.platform(), physical_validation=False,
                  limitation=__doc__, model_id=profile.id, config=cfg, mapping=mapping,
                  ignored_links=backend.ignored_links, query_scope="inter_branch",
                  harness_sha256=hashlib.sha256(default_binary().read_bytes()).hexdigest(),
                  collision_library_sha256=hashlib.sha256(library_path().read_bytes()).hexdigest(),
                  summary=summary, expiry=expiry, cycles=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps(summary, indent=2))
    if failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
