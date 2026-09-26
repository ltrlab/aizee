"""25 Hz real-leader shadow test with a local, ideal virtual receiver.

Read snapshots from the already-running leader viewer; never open USB, CAN or a
physical controller transport. All follower coordinates, gains and motion bounds
are synthetic. Passing this test does not qualify powered operation or dynamics.
The native harness advances its clock only when called, not on an independent
receiver thread. Engagement below is an explicit simulation fixture, not a
qualified operator deadman. Any input, collision or timing failure ends the run.
"""
from copy import deepcopy
import argparse
import hashlib
import http.client
import json
import math
from pathlib import Path
import platform
import time
from urllib.parse import urlsplit

import numpy as np

from benchmark_guarded_teleop import synthetic_config
from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler
from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
from robot_platform.teleop.viewer_input import ViewerLeaderInput, validate_viewer_url
from robot_platform.teleop.leader_mapping import LeaderTargetMapper
from robot_platform.teleop.supervisor import GuardedTeleopSupervisor


PERIOD_S = .04
CYCLE_BUDGET_S = .035


def validate_routes(profile, joint_names, leader_names):
    """Require complete, explicitly synthetic CAD routes; never motor mappings."""
    if (profile.get("schema_version") != 1 or profile.get("simulation_only") is not True
            or profile.get("runtime_approved") is not False):
        raise ValueError("Routes require schema 1, simulation_only=true and runtime_approved=false")
    routes = profile.get("routes")
    if (not isinstance(routes, list) or len(routes) != len(joint_names)
            or any(not isinstance(row, dict) for row in routes)
            or tuple(row.get("actuator") for row in routes) != tuple(joint_names)):
        raise ValueError("Shadow routes must cover every independent CAD joint in model order")
    for row in routes:
        if (set(row) != {"actuator", "leader", "channel", "scale"}
                or not isinstance(row["leader"], str) or row["leader"] not in leader_names
                or type(row["channel"]) is not int or not 0 <= row["channel"] < 7
                or type(row["scale"]) not in (int, float)
                or not math.isfinite(row["scale"]) or row["scale"] == 0):
            raise ValueError("Each shadow route needs a known leader, channel 0..6 and finite nonzero scale")
    if {row["leader"] for row in routes} != set(leader_names):
        raise ValueError("Shadow routes must use the complete leader roster")
    return deepcopy(routes)


class ShadowSupervisor(GuardedTeleopSupervisor):
    """Include HTTP input time in the existing pre/post-dispatch budget checks."""
    input_started_at = None
    scheduled_at = None

    def _before_dispatch(self, samples, started):
        super()._before_dispatch(samples, started)
        if self.input_started_at is None or self.clock() - self.input_started_at >= self.cycle_budget_s:
            raise TimeoutError("Full shadow cycle including leader input exceeded 35 ms")
        if self.scheduled_at is None or self.clock() > self.scheduled_at + self.period_s:
            raise TimeoutError("Shadow cycle exceeded its 25 Hz scheduled deadline")


class ArmAcknowledgement:
    """Keep virtual Arm acceptance even when post-ack checks stop registration."""
    def __init__(self, exchange):
        self.exchange, self.arm_result = exchange, None

    def __call__(self, request):
        is_arm = request.get("type") == "arm"
        if is_arm:
            self.arm_result = {"accepted": None}  # Unknown until a reply arrives.
        reply = self.exchange(request)
        if is_arm:
            self.arm_result = deepcopy(reply)
        return reply


def run_cycle(source, supervisor, *, registration=False, scheduled_at, clock=time.perf_counter):
    """Record rejected input as well as rejected/late virtual receiver commands."""
    started = clock()
    record = dict(kind="registration" if registration else "motion", accepted=False,
                  scheduled_at=scheduled_at, cycle_started_at=started)
    previous = supervisor.last_cycle
    previous_verdict = supervisor.compiler.last_verdict
    samples = None
    supervisor.input_started_at = started
    supervisor.scheduled_at = scheduled_at
    try:
        samples = source.poll()
        received = clock()
        record.update(input_completed_at=received, input_ms=(received - started) * 1000,
                      leaders={name: dict(sequence=s.sequence, raw_positions=list(s.raw_positions),
                                           source_age_ms=(received - s.sampled_at) * 1000)
                               for name, s in samples.items()},
                      pair_skew_ms=(max(s.sampled_at for s in samples.values())
                                    - min(s.sampled_at for s in samples.values())) * 1000)
        action = supervisor.begin if registration else supervisor.step
        record.update(deepcopy(action(samples, engaged=True)))
    except Exception as exc:
        if supervisor.last_cycle is not None and supervisor.last_cycle is not previous:
            record.update(deepcopy(supervisor.last_cycle))
        record.update(failure=str(exc) or type(exc).__name__, failure_type=type(exc).__name__)
        supervisor.stop(record["failure"])
    finally:
        if registration:
            exchange = getattr(getattr(supervisor, "controller", None), "exchange", None)
            acknowledgement = getattr(exchange, "arm_result", None)
            if acknowledgement is not None:
                record["accepted"] = acknowledgement["accepted"]
                record["arm_acknowledgement"] = deepcopy(acknowledgement)
        if supervisor.compiler.last_verdict is not previous_verdict:
            record["collision"] = deepcopy(supervisor.compiler.last_verdict)
        completed = clock()
        record.update(cycle_completed_at=completed, full_cycle_ms=(completed - started) * 1000,
                      stop_error=supervisor.stop_error)
        if "input_ms" not in record:
            record["input_ms"] = (completed - started) * 1000
        if samples is not None:
            record["maximum_source_age_at_completion_ms"] = max(
                completed - s.sampled_at for s in samples.values()) * 1000
        if completed - started >= CYCLE_BUDGET_S or completed > scheduled_at + PERIOD_S:
            record.setdefault("failure", "Full shadow cycle or 25 Hz scheduled deadline exceeded")
            supervisor.stop(record["failure"])
    return record


def summarize(records, registration, failure, expiry):
    values = [r["full_cycle_ms"] for r in records]
    inputs = [r["input_ms"] for r in records]
    ages = [row["source_age_ms"] for r in records for row in r.get("leaders", {}).values()]
    completion_ages = [r["maximum_source_age_at_completion_ms"] for r in records
                       if "maximum_source_age_at_completion_ms" in r]
    all_records = ([registration] if registration else []) + records
    return dict(completed_cycles=len(records), accepted_cycles=sum(r["accepted"] is True for r in records),
                scheduled_deadline_misses=sum(r["cycle_completed_at"] > r["scheduled_at"] + PERIOD_S for r in records),
                cycle_budget_misses=sum(r["full_cycle_ms"] >= CYCLE_BUDGET_S * 1000 for r in records),
                p50_ms=float(np.percentile(values, 50)) if values else None,
                p99_ms=float(np.percentile(values, 99)) if values else None,
                maximum_ms=max(values, default=None), input_maximum_ms=max(inputs, default=None),
                input_p99_ms=float(np.percentile(inputs, 99)) if inputs else None,
                maximum_source_age_ms=max(ages, default=None),
                maximum_source_age_at_completion_ms=max(completion_ages, default=None),
                collision_rejections=sum(r.get("collision", {}).get("clear") is False for r in all_records),
                failure=failure, silent_producer_stop_verified=expiry is not None
                and expiry.get("reason") == "collision_expired" and expiry.get("state") == "stopped")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--routes", type=Path, required=True)
    parser.add_argument("--leader-url", default="http://127.0.0.1:8767/")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=250)
    args = parser.parse_args()
    if args.output.exists() or not 1 <= args.cycles <= 1500:
        parser.error("Choose a new output file and 1-1500 cycles")
    try:
        validate_viewer_url(args.leader_url)
    except ValueError as exc:
        parser.error(str(exc))
    profile = ModelProfile.load(args.model)
    model = UrdfModel(profile)
    route_profile = json.loads(args.routes.read_text(encoding="utf-8"))
    backend = source = None
    cfg = mapping = routes = leader_profile = registration = expiry = failure = None
    ignored_links, records = {}, []
    try:
        print("Loading collision geometry and warming shadow-test caches...", flush=True)
        backend = NativeCollisionBackend(RuntimeCollisionModel(profile), geometry_mode="keepout")
        cfg, mapping, _ = synthetic_config(model, backend)
        ignored_links = backend.ignored_links
        compiler = CollisionEnvelopeCompiler(backend, cfg, mapping, compute_budget_s=.025)
        initial = [model.by_name[name]["default"] for name in model.independent]
        for _ in range(10):
            backend.proximity(dict(zip(model.independent, initial)), scope="inter_branch")
        source = ViewerLeaderInput(args.leader_url)
        leader_profile = source.profile
        routes = validate_routes(route_profile, model.independent, source.pins)
        with NativeControlHarness(cfg, initial) as harness:
            # Process startup is outside the paced test, like collision-cache
            # warming. Once ready, every harness reply has the cycle timeout.
            harness.step(0)
            harness.timeout_s = CYCLE_BUDGET_S
            epoch = time.perf_counter()

            def exchange(request):
                # The final silent-producer probe deliberately advances the
                # simulated clock. Cleanup must not move that clock backwards.
                now_ms = max(harness.now_ms, int((time.perf_counter() - epoch) * 1000))
                return harness.step(now_ms, request)["reply"]

            controller = GuardedControllerClient(ArmAcknowledgement(exchange), group=cfg["group"], identity=cfg["identity"],
                                                  joint_names=tuple(model.independent))
            mapper = LeaderTargetMapper(cfg["joints"], routes, max_age_s=source.max_age_s,
                                        max_skew_s=source.max_skew_s)
            supervisor = ShadowSupervisor(controller, compiler, mapper, lambda: harness.last)
            try:
                print(f"Starting {args.cycles} shadow cycles at 25 Hz; virtual receiver only.", flush=True)
                registration = run_cycle(source, supervisor, registration=True, scheduled_at=time.perf_counter())
                if registration.get("failure"):
                    failure = registration["failure"]
                else:
                    due = time.perf_counter() + PERIOD_S
                    for _ in range(args.cycles):
                        time.sleep(max(0., due - time.perf_counter()))
                        record = run_cycle(source, supervisor, scheduled_at=due)
                        records.append(record)
                        if record.get("failure"):
                            failure = record["failure"]
                            break
                        due += PERIOD_S
                    if failure is None:
                        status = harness.last["status"]
                        expiry = harness.step(status["collision_expires_at_ms"] + 1)["status"]
                        if expiry["reason"] != "collision_expired" or expiry["state"] != "stopped":
                            raise RuntimeError("Virtual receiver failed silent-producer collision expiry check")
            finally:
                supervisor.stop("shadow_benchmark_complete")
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        if source is not None:
            source.close()
        if backend is not None:
            backend.close()
    summary = summarize(records, registration, failure, expiry)
    report = dict(schema_version=1, platform=platform.platform(), physical_validation=False,
                  simulation_only=True, limitation=__doc__, model_id=profile.id,
                  routes_sha256=hashlib.sha256(args.routes.read_bytes()).hexdigest(),
                  route_profile=route_profile, leader_profile=leader_profile, config=cfg,
                  mapping=mapping, routes=routes, ignored_links=ignored_links,
                  query_scope="inter_branch", period_s=PERIOD_S, cycle_budget_s=CYCLE_BUDGET_S,
                  harness_sha256=hashlib.sha256(default_binary().read_bytes()).hexdigest()
                  if default_binary().is_file() else None,
                  collision_library_sha256=hashlib.sha256(library_path().read_bytes()).hexdigest()
                  if library_path().is_file() else None,
                  summary=summary, registration=registration, expiry=expiry, cycles=records)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(summary, indent=2))
    if failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
