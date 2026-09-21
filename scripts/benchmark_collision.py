"""Offline repeatable whole-robot proximity benchmark; no device access.

Example: python scripts/benchmark_collision.py --model urdf/minerva/model.json
         --output simulation_runs/collision-native.json --samples 1000
"""
import argparse
import json
from pathlib import Path
import platform
import time

import numpy as np

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
from robot_platform.simulation.native_collision import NativeCollisionBackend
from robot_platform.simulation.collision_groups import SCOPES


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--samples", type=int, default=1000)
    p.add_argument("--reference-samples", type=int, default=20)
    p.add_argument("--audit-samples", type=int, default=0, help="Also compare contacts against detailed source CAD")
    p.add_argument("--seed", type=int, default=20260920)
    p.add_argument("--threshold", type=float, default=0., help="Contact queries default to zero; positive values measure CAD surface clearance")
    p.add_argument("--scope", choices=SCOPES, default="all", help="Partial scopes do not qualify whole-robot safety")
    args = p.parse_args()
    if args.samples < 2 or not 1 <= args.reference_samples <= args.samples or not 0 <= args.audit_samples <= args.samples or args.output.exists() or not np.isfinite(args.threshold) or args.threshold<0:
        p.error("Require samples >= reference-samples >= 1, samples >= 2 and a new output file")
    profile = ModelProfile.load(args.model)
    model = UrdfModel(profile)
    runtime = RuntimeCollisionModel(profile)
    native = NativeCollisionBackend(runtime)
    rng = np.random.default_rng(args.seed)
    poses = [{n: model.by_name[n]["default"] for n in model.independent}]
    poses += [{n: float(rng.uniform(*model.by_name[n]["control_bounds"])) for n in model.independent}
              for _ in range(args.samples-1)]
    # Warm geometry/query code before steady-state timing.
    for _ in range(5): native.proximity(poses[0],threshold=args.threshold,scope=args.scope)
    timings = []
    stages = []
    counters = []
    results = []
    for q in poses:
        start = time.perf_counter()
        result = native.proximity(q,threshold=args.threshold,scope=args.scope)
        timings.append((time.perf_counter()-start)*1000)
        stages.append(result['stage_ms'])
        counters.append(result['broad_phase'])
        if len(results)<max(args.reference_samples,args.audit_samples): results.append(result)
    compact=[]
    for q in poses:
        start=time.perf_counter()
        native.proximity(q,max_pairs=1,threshold=args.threshold,scope=args.scope)
        compact.append((time.perf_counter()-start)*1000)
    reference, mismatches = [], []
    for q, result in zip(poses[:args.reference_samples], results):
        other = runtime.proximity(q,threshold=args.threshold,scope=args.scope)
        reference.append(other["compute_ms"])
        a = {tuple(p["shapes"]): p["distance_m"] for p in result["near_pairs"]}
        b = {tuple(p["shapes"]): p["distance_m"] for p in other["near_pairs"]}
        errors = [(key, a.get(key), b.get(key)) for key in a.keys() | b.keys()
                  if key not in a or key not in b or abs(a[key]-b[key]) > 2e-6]
        if errors: mismatches.append(dict(positions=q, pairs=errors))
    def stats(values):
        return dict(samples=len(values), p50_ms=float(np.percentile(values,50)),
                    p95_ms=float(np.percentile(values,95)), p99_ms=float(np.percentile(values,99)),
                    maximum_ms=max(values), over_10ms=sum(v>10 for v in values), over_40ms=sum(v>40 for v in values))
    missed, extra, audit_times = [],[],[]
    if args.audit_samples:
        from robot_platform.simulation.collision import CollisionScene
        source=CollisionScene.from_urdf(profile.urdf,profile.package_root,mesh_resolver=profile.resolve_mesh)
        audit_pairs = [tuple(runtime.shapes[k]['id'] for k in pair) for pair in runtime.pairs_for_scope(args.scope)]
        for q,result in zip(poses[:args.audit_samples],results):
            start=time.perf_counter()
            contacts=set(source.check_shape_pairs(q, pairs=audit_pairs))
            audit_times.append((time.perf_counter()-start)*1000)
            proxy={tuple(sorted(p["shapes"])) for p in result["near_pairs"] if p["distance_m"]<=2e-6}
            if contacts-proxy: missed.append(dict(positions=q,missed_pairs=sorted(contacts-proxy)))
            if proxy-contacts: extra.append(dict(positions=q,extra_pairs=sorted(proxy-contacts)))
    report = dict(model_id=profile.id, model_revision=runtime.revision,
                  platform=platform.platform(), processor=platform.processor(), seed=args.seed,
                  threshold_m=args.threshold, geometry_basis="source_cad", backend="parry-bounds+fcl-cad",
                  query_scope=args.scope, whole_robot_checked=args.scope=='all', collision_policy_revision=runtime.policy_revision,
                  source_triangles=runtime.data["source_triangles"],
                  proxy_triangles=runtime.data["proxy_triangles"],
                  native=stats(timings), native_compact=stats(compact), fcl_reference=stats(reference),
                  reference_disagreements=mismatches,
                  missed_source_contacts=missed, extra_source_contacts=extra, source_audit=stats(audit_times) if audit_times else None,
                  reference_pose_candidates=results[0]['candidate_pairs'],reference_pose_contacts=len(results[0]['near_pairs']),
                  reference_pose_by_scope=results[0]['by_scope'], reference_pose_counters=counters[0],
                  stage_timings={k:stats([s[k] for s in stages]) for k in stages[0]},
                  counters={k:dict(mean=float(np.mean([c[k] for c in counters])),maximum=max(c[k] for c in counters)) for k in counters[0]},
                  physical_validation=False, positions=poses, native_timings_ms=timings)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x",encoding="utf-8") as stream:
        json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:report[k] for k in ("model_id","source_triangles","proxy_triangles","native","native_compact","fcl_reference")},indent=2))
    print(f"Reference poses with disagreements: {len(mismatches)}; see {args.output}")
    if mismatches or missed or extra:
        raise SystemExit(1)


if __name__ == "__main__": main()
