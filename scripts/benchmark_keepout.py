"""Offline keep-out latency, CAD containment audit, and synthetic motion replay.

The chosen scope and link exclusions are explicit. This never drives hardware.
The replay uses three short paths per cycle, not calibrated stopping dynamics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.runtime_model import RuntimeCollisionModel
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.control.collision_guard import check_linear_motion
from robot_platform.simulation.collision_groups import SCOPES


def stats(values):
    return dict(samples=len(values), p50_ms=float(np.percentile(values,50)),
                p99_ms=float(np.percentile(values,99)), maximum_ms=max(values),
                over_40ms=sum(v>40 for v in values))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--scope',choices=SCOPES,default='inter_branch')
    parser.add_argument('--samples',type=int,default=1000)
    parser.add_argument('--audit-samples',type=int,default=100)
    parser.add_argument('--threshold',type=float,default=.005)
    parser.add_argument('--rate',type=float,default=0.,help='Pace the three-path replay, e.g. 25 Hz; zero runs unpaced')
    args=parser.parse_args()
    if (args.output.exists() or args.samples<2 or not 1<=args.audit_samples<=args.samples
            or not np.isfinite(args.threshold) or args.threshold<0 or not np.isfinite(args.rate) or args.rate<0):
        parser.error('Require a new output, at least two samples, valid audit count and nonnegative finite gap/rate')
    profile=ModelProfile.load(args.model); model=UrdfModel(profile); runtime=RuntimeCollisionModel(profile)
    backend=NativeCollisionBackend(runtime,geometry_mode='keepout')
    rng=np.random.default_rng(20260920)
    poses=[{n:model.by_name[n]['default'] for n in model.independent}]
    poses += [{n:float(rng.uniform(*model.by_name[n]['control_bounds'])) for n in model.independent} for _ in range(args.samples-1)]
    for _ in range(30): backend.proximity(poses[0],threshold=args.threshold,scope=args.scope)
    timings=[]; results=[]; blocked=0
    for q in poses:
        start=time.perf_counter(); result=backend.proximity(q,threshold=args.threshold,scope=args.scope)
        timings.append((time.perf_counter()-start)*1000)
        blocked+=bool(result['near_pairs'])
        if len(results)<args.audit_samples: results.append(result)
        assert result['broad_phase']['fcl_collision_tests']==result['broad_phase']['fcl_distance_tests']==0
    # A source-CAD proximity must never escape its enclosing boundary, except
    # for the explicitly excluded links. Additional keep-out restrictions are expected.
    missed=[]; additional=0; excluded_cad=0
    allowed={tuple(runtime.shapes[k]['id'] for k in pair) for pair in runtime.pairs_for_scope(args.scope,geometry_mode='keepout')}
    for q,result in zip(poses,results):
        reference=runtime.proximity(q,threshold=args.threshold,scope=args.scope)
        cad={tuple(p['shapes']) for p in reference['near_pairs']}
        envelope={tuple(p['shapes']) for p in result['near_pairs']}
        expected=cad & allowed
        if expected-envelope: missed.append(dict(positions=q,pairs=sorted(expected-envelope)))
        additional+=len(envelope-expected);excluded_cad+=len(cad-allowed)
    replay=[]; accepted=0; counts=[]; deadline_misses=0; wake_lateness=[]
    next_cycle=time.monotonic()
    for q in poses:
        if args.rate:
            time.sleep(max(0.,next_cycle-time.monotonic()))
            wake_lateness.append(max(0.,time.monotonic()-next_cycle)*1000)
        started=time.perf_counter(); clear=True; queries=0
        goal={n:float(np.clip(v+.002,*model.by_name[n]['control_bounds'])) for n,v in q.items()}
        stop={n:float(np.clip(v+.001,*model.by_name[n]['control_bounds'])) for n,v in goal.items()}
        for a,b,subdivide in ((q,goal,False),(q,goal,True),(goal,stop,False)):
            result=check_linear_motion(backend,a,b,clearance_m=args.threshold,scope=args.scope,
                                       subdivide=subdivide,deadline=started+.04,clock=time.perf_counter)
            queries+=result['queries']
            if not result['clear']: clear=False;break
        elapsed=(time.perf_counter()-started)*1000
        replay.append(elapsed);counts.append(queries);accepted+=clear
        if args.rate:
            deadline_misses+=time.monotonic()>next_cycle+1/args.rate
            next_cycle+=1/args.rate
            if time.monotonic()>next_cycle: next_cycle=time.monotonic()
    report=dict(schema_version=1,model_id=profile.id,model_revision=runtime.revision,
        policy_revision=backend.policy_revision,ignored_links=backend.ignored_links,
        query_scope=args.scope,whole_robot_checked=args.scope=='all',full_geometry_coverage=args.scope=='all' and not backend.ignored_links,
        geometry_basis=backend.geometry_basis,platform=platform.platform(),seed=20260920,
        native_library_sha256=hashlib.sha256(library_path().read_bytes()).hexdigest(),
        threshold_m=args.threshold,pose=stats(timings),blocked_poses=blocked,
        reference_pose=results[0],cad_audit_samples=args.audit_samples,
        missed_nonexcluded_cad_pairs=missed,additional_envelope_pairs=additional,
        intentionally_excluded_cad_pairs=excluded_cad,
        motion_replay=stats(replay),accepted_cycles=accepted,rejected_cycles=args.samples-accepted,
        maximum_queries=max(counts),rate_hz=args.rate,scheduled_deadline_misses=deadline_misses if args.rate else None,
        wake_lateness=stats(wake_lateness) if args.rate else None,physical_validation=False,
        limitation='Synthetic short-path workload. Excluded links have no keep-out protection. Not a full hardware supervisor qualification.',
        positions=poses,pose_timings_ms=timings,motion_timings_ms=replay)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('query_scope','pose','motion_replay','accepted_cycles','rejected_cycles','scheduled_deadline_misses','additional_envelope_pairs','intentionally_excluded_cad_pairs')},indent=2))
    if missed: raise SystemExit('Keep-out query missed non-excluded CAD proximity')


if __name__=='__main__': main()
