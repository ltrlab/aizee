#!/usr/bin/env python3
"""Portable OFFLINE command qualification for quantized direct teleop.

Uses explicit synthetic control fixtures, never robot calibration, a receiver,
serial, sockets, CAN, or a physical plant. JSON/CSV are the primary artifacts.
The ideal follower returns the preceding position command at the next cycle.
Passing this command contract does not qualify physical tracking or stability.
"""
import argparse
from copy import deepcopy
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import sys


ROOT = Path(__file__).resolve().parents[1]
DT = .01
SOURCE_HZ = 90
SOURCE_DELAY = .015
QUANTUM = .0015
EPS = 1e-12


def synthetic_joint(*, position_only=False):
    """A portable contract fixture; these values are NOT Minerva calibration."""
    return dict(name='q', lower=-2., upper=2., hold_kp=100., hold_kd=20., max_velocity=1.,
                align_speed=.2, shutdown_speed=.2, max_lead=.15, tolerance=.005,
                settled_velocity=.02, zero_position=0., interpolation_s=0., direct_velocity=True,
                position_only=position_only, friction_nm=.5, inertia=.1, max_effort=30.,
                gravity=[(2., {'q': 1})],
                approach=dict(lower=-2., upper=2., max_velocity=1.25, max_tracking_error=.25,
                              stop_acceleration=4., reaction_s=.04))


def percentile(values, fraction):
    values=sorted(values)
    if not values: return None
    index=(len(values)-1)*fraction
    low=math.floor(index)
    return values[low]+(values[math.ceil(index)]-values[low])*(index-low)


def stats(values):
    values=list(values)
    if any(not math.isfinite(v) for v in values): raise ValueError('Nonfinite qualification metric')
    return dict(count=len(values),p50=percentile(values,.5),p95=percentile(values,.95),
                p99=percentile(values,.99),max=max(values,default=None))


def rows(now, position=0., *, sequence=None, sampled_at=None):
    return [dict(name='q',position=position,velocity=0.,sampled_at=now if sampled_at is None else sampled_at,
                 sequence=round(now*10000)+10000 if sequence is None else sequence)]


def following(lifecycle_class, *, position_only, repeated=True):
    joint=synthetic_joint(position_only=position_only)
    life=lifecycle_class([joint],allow_repeated_source=repeated,max_source_misses=8)
    life.action('idle',rows(0.),now=0.)
    life.action('enable',rows(0.),now=0.)
    life.action('engage',rows(0.),target=rows(0.),now=0.)
    for cycle in range(1,32):
        t=cycle*DT
        life.step(rows(t),target=rows(t),now=t)
    if life.state!='TELEOP': raise AssertionError('Fresh stationary warmup did not enter TELEOP')
    return life


def cases(seed=20261010):
    rng=random.Random(seed)
    result=[]
    for speed in (.08,.8):
        for direction in (-1.,1.):
            for phase_index in range(2):
                amplitude=.12 if speed==.08 else .36
                result.append(dict(name=f'{"slow" if speed==.08 else "fast"}-{direction:+.0f}-{phase_index}',
                                   speed=speed,direction=direction,amplitude=amplitude,
                                   phase=-rng.random()/SOURCE_HZ))
    return result


def motion(t, case):
    """Continuous ramp, immediate reversal, reached-goal hold, return, final hold."""
    ramp=case['amplitude']/case['speed']
    direction,amplitude=case['direction'],case['amplitude']
    if t<.4: return 0.,'initial_hold',False
    t-=.4
    if t<ramp: return direction*amplitude*t/ramp,'outward_ramp',True
    t-=ramp
    if t<2*ramp: return direction*amplitude*(1-t/ramp),'reversal_ramp',True
    t-=2*ramp
    if t<.3: return -direction*amplitude,'far_hold',False
    t-=.3
    if t<ramp: return direction*amplitude*(-1+t/ramp),'return_ramp',True
    return 0.,'final_hold',False


def run_case(lifecycle_class, case, *, position_only, repeated=True):
    life=following(lifecycle_class,position_only=position_only,repeated=repeated)
    joint=synthetic_joint(position_only=position_only)
    # A 30 ms transition from warmup makes the first 15 ms delayed sample newer
    # than warmup's last timestamp. Motion starts only after another 400 ms hold.
    epoch=.34
    duration=1.2+4*case['amplitude']/case['speed']
    previous_position=previous_velocity=previous_non_gravity=0.
    previous_source=None
    trace=[]
    errors=[]
    last_source=None
    for cycle in range(1,math.ceil(duration/DT)+1):
        elapsed=cycle*DT
        now=epoch+elapsed
        source_index=math.floor((elapsed-SOURCE_DELAY-case['phase'])*SOURCE_HZ+1e-10)
        stamp=case['phase']+source_index/SOURCE_HZ
        continuous_goal,segment,moving=motion(stamp,case)
        goal=round(continuous_goal/QUANTUM)*QUANTUM
        sequence=100000+source_index
        source=rows(now,goal,sequence=sequence,sampled_at=epoch+stamp)
        duplicate=sequence==previous_source
        previous_source=sequence
        before=previous_position
        try:
            proposal=life.step(rows(now,before),target=source if repeated or not duplicate else None,now=now)
        except Exception as exc:
            errors.append(dict(cycle=cycle,error=f'{type(exc).__name__}: {exc}'))
            break
        command=proposal['joints'][0]
        position,velocity,effort=command['position'],command['velocity'],command['effort']
        reference_velocity=proposal['reference_velocities'][0]
        ramp_effort=(proposal.get('ramp_efforts') or [0.])[0]
        gravity=2*math.sin(position)  # Independent analytic value for this synthetic fixture.
        non_gravity=joint['hold_kd']*velocity+effort-gravity
        delta=position-before
        if not joint['lower']<=position<=joint['upper']: errors.append(dict(cycle=cycle,error='position bound'))
        if abs(delta)>joint['max_velocity']*DT+EPS: errors.append(dict(cycle=cycle,error='position slew'))
        if abs(velocity)>joint['max_velocity']+EPS: errors.append(dict(cycle=cycle,error='velocity bound'))
        if position_only and abs(ramp_effort)>EPS: errors.append(dict(cycle=cycle,error='position-only ramp effort'))
        if not min(before,goal)-EPS<=position<=max(before,goal)+EPS:
            errors.append(dict(cycle=cycle,error='command overshot the observed goal'))
        if repeated and life._source!=source: errors.append(dict(cycle=cycle,error='source identity changed'))
        if life.state!='TELEOP': errors.append(dict(cycle=cycle,error='unexpected lifecycle state'))
        trace.append(dict(case=case['name'],policy='position_only' if position_only else
                          ('retained_feedforward' if repeated else 'legacy_missing'),cycle=cycle,time_s=elapsed,
                          segment=segment,source_moving=moving,source_sequence=sequence,source_sampled_at=epoch+stamp,
                          source_age_ms=(now-epoch-stamp)*1000,repeated_source=duplicate,
                          goal=goal,continuous_goal=continuous_goal,position=position,
                          position_step=delta,planned_velocity_from_position=delta/DT,
                          reference_velocity=reference_velocity,ramp_effort=ramp_effort,
                          commanded_velocity=velocity,commanded_effort=effort,gravity_effort=gravity,
                          non_gravity_command_term=non_gravity,
                          delta_non_gravity_command_term=non_gravity-previous_non_gravity,
                          delta_commanded_velocity=velocity-previous_velocity,
                          goal_already_reached=abs(before-goal)<=EPS,
                          zero_velocity_drop=abs(previous_velocity)>EPS and abs(velocity)<=EPS,
                          limits=';'.join(proposal['limits'].get('q',[]))))
        previous_position,previous_velocity,previous_non_gravity=position,velocity,non_gravity
        last_source=deepcopy(source)
    if life.state!='FAULT':
        life.action('disengage',rows(now,previous_position),now=now)
        life.action('disable',now=now)
    moving_trace=[r for r in trace if r['source_moving']]
    duplicates=[r for r in moving_trace if r['repeated_source']]
    final_hold=[r for r in trace if r['segment']=='final_hold']
    settled=next((r['time_s']-final_hold[0]['time_s'] for r in final_hold
                  if abs(r['position']-r['goal'])<=EPS),None) if final_hold else None
    first_goal=next((r['time_s'] for r in trace if abs(r['goal'])>EPS),None)
    first_command=next((r['time_s'] for r in trace if abs(r['position'])>EPS),None)
    # Explicit fixture envelope: continuous acceleration catch-up distance, two
    # consumer periods of motion, and two encoder quanta. This is a test bound,
    # never a replacement for runtime limits or a physical latency guarantee.
    gap_bound=case['speed']**2/(2*joint['approach']['stop_acceleration'])+2*case['speed']*DT+2*QUANTUM
    settle_bound=case['speed']/joint['approach']['stop_acceleration']+2/SOURCE_HZ+3*DT
    summary=dict(case=case,policy=trace[0]['policy'] if trace else None,cycles=len(trace),faults=errors,
                 final_state=life.state,source_identity_preserved=not any(e['error']=='source identity changed' for e in errors),
                 moving_cycles=len(moving_trace),moving_duplicates=len(duplicates),
                 zero_velocity_drops=sum(r['zero_velocity_drop'] for r in moving_trace),
                 repeat_zero_velocity_drops=sum(r['zero_velocity_drop'] for r in duplicates),
                 reached_goal_repeat_zero_velocity_drops=sum(r['zero_velocity_drop'] and r['goal_already_reached'] for r in duplicates),
                 command_velocity_abs=stats(abs(r['commanded_velocity']) for r in trace),
                 reference_velocity_abs=stats(abs(r['reference_velocity']) for r in trace),
                 non_gravity_command_abs=stats(abs(r['non_gravity_command_term']) for r in trace),
                 repeat_non_gravity_step_abs=stats(abs(r['delta_non_gravity_command_term']) for r in duplicates),
                 maximum_goal_gap_rad=max((abs(r['position']-r['goal']) for r in trace),default=math.inf),
                 goal_gap_bound_rad=gap_bound,final_goal_settle_s=settled,final_goal_settle_bound_s=settle_bound,
                 first_observed_goal_to_position_s=None if first_goal is None or first_command is None else first_command-first_goal,
                 source_age_ms=stats(r['source_age_ms'] for r in trace),last_source=last_source)
    return summary,trace


def acceptance(summary, *, require_position_only=True):
    failures=[]
    if summary['faults']: failures.append('lifecycle or invariant failure')
    if summary['final_state']!='DISABLED': failures.append('did not disable')
    if summary['maximum_goal_gap_rad']>summary['goal_gap_bound_rad']+EPS: failures.append('moving goal gap')
    if summary['final_goal_settle_s'] is None or summary['final_goal_settle_s']>summary['final_goal_settle_bound_s']+EPS:
        failures.append('final goal settle time')
    delay=summary['first_observed_goal_to_position_s']
    if delay is None or not 0<=delay<=DT+EPS: failures.append('first observed goal response')
    if require_position_only:
        if not (summary['reference_velocity_abs']['max'] or 0)>EPS: failures.append('planner did not move')
        if (summary['command_velocity_abs']['max'] or 0)>EPS: failures.append('nonzero velocity feedforward')
        if (summary['non_gravity_command_abs']['max'] or 0)>EPS: failures.append('non-gravity command torque')
        if (summary['repeat_non_gravity_step_abs']['max'] or 0)>EPS: failures.append('repeat-linked command torque pulse')
    return failures


def freshness_checks(lifecycle_class):
    results={}
    for mutation in ('stale_source','stale_feedback','changed_duplicate','changed_timestamp','backwards_sequence'):
        life=following(lifecycle_class,position_only=True)
        source=rows(.32,.3)
        first=life.step(rows(.32),target=source,now=.32)
        feedback=rows(.33,first['positions'][0])
        now=.33
        if mutation=='stale_source': now=.371;feedback=rows(now,first['positions'][0])
        elif mutation=='stale_feedback': feedback[0]['sampled_at']=.29
        elif mutation=='changed_duplicate': source[0]['position']=.31
        elif mutation=='changed_timestamp': source[0]['sampled_at']=.321
        else: source[0]['sequence']-=1
        try:
            life.step(feedback,target=source,now=now)
        except ValueError as exc:
            results[mutation]=dict(passed=life.state=='FAULT',reason=str(exc))
        else: results[mutation]=dict(passed=False,reason='invalid input was accepted')
    life=following(lifecycle_class,position_only=True)
    source=rows(.32,.3)
    first=life.step(rows(.32),target=source,now=.32)
    position=first['positions'][0]
    held=None
    for index in range(1,10):
        now=.32+index*DT
        held=life.step(rows(now,position),target=None,now=now)
        position=held['positions'][0]
    now+=DT
    resumed=life.step(rows(now,position),target=rows(now,.5),now=now)
    results['missing_requires_reengage']=dict(passed=held['state']=='HOLD' and resumed['state']=='HOLD',
                                            state=life.state,reason=life.reason)
    return results


def qualify(lifecycle_class, *, seed=20261010):
    summaries=[]
    records=[]
    for case in cases(seed):
        old,old_trace=run_case(lifecycle_class,case,position_only=False)
        new,new_trace=run_case(lifecycle_class,case,position_only=True)
        # Position-only motor references must retain the exact bounded internal
        # position planner. Otherwise zero v_command can reset acceleration state.
        same_inputs=[(r['source_sequence'],r['source_sampled_at'],r['goal']) for r in old_trace]==[
                    (r['source_sequence'],r['source_sampled_at'],r['goal']) for r in new_trace]
        position_difference=max((abs(a['position']-b['position']) for a,b in zip(old_trace,new_trace)),default=math.inf)
        failed=acceptance(new)
        if not same_inputs or len(old_trace)!=len(new_trace): failed.append('comparison inputs/completion differ')
        if position_difference>EPS: failed.append('position planning changed with motor reference policy')
        summaries.append(dict(case=case['name'],candidate=new,feedforward_reference=old,
                              identical_inputs=same_inputs,maximum_position_difference_rad=position_difference,
                              acceptance_failures=failed,passed=not failed))
        records.extend(old_trace);records.extend(new_trace)
    faults=freshness_checks(lifecycle_class)
    report=dict(schema=1,offline_only=True,hardware_opened=False,plant_model=False,seed=seed,
                fixture=synthetic_joint(position_only=True),consumer_hz=1/DT,source_hz=SOURCE_HZ,
                source_delay_s=SOURCE_DELAY,quantum_rad=QUANTUM,
                all_passed=all(s['passed'] for s in summaries) and all(x['passed'] for x in faults.values()),
                cases=summaries,freshness=faults,
                acceptance_contract=[
                    'Position-only emitted velocity and non-gravity effort are zero, including reached/repeated goals.',
                    'Identical source inputs yield identical bounded position trajectories across reference policies.',
                    'No step overshoots the current observed goal; existing position/speed bounds remain intact.',
                    'Synthetic goal lag and settle time stay inside the explicit fixture envelopes.',
                    'Original source timestamps/sequences are retained; invalid/stale input faults and loss requires Engage.'],
                limitations=[
                    'Synthetic fixture values are not measured robot properties or a replacement calibration.',
                    'Ideal next-cycle feedback has no inertia, delay, friction, compliance, noise, or force interaction.',
                    'No receiver, source-age pre-dispatch gate, CAN/USB/network, real-time scheduling, or motor is exercised.',
                    'Non-gravity command term is Kd*v_command + effort_command - gravity(command_pose), not measured torque.',
                    'Quantized position steps remain; passing does not establish physical smoothness or authorize live operation.'])
    return report,records


def source_identity():
    from robot_platform.teleop import lifecycle
    from robot_platform.control import approach,gravity
    files=[Path(__file__),Path(lifecycle.__file__),Path(approach.__file__),Path(gravity.__file__)]
    return {str(path.resolve()):hashlib.sha256(path.read_bytes()).hexdigest() for path in files}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=20261010)
    args=parser.parse_args(argv)
    sys.path.insert(0,str(ROOT/'src'))
    from robot_platform.teleop.lifecycle import TeleopLifecycle
    import robot_platform.teleop.lifecycle as lifecycle
    if Path(lifecycle.__file__).resolve()!=ROOT/'src/robot_platform/teleop/lifecycle.py':
        raise RuntimeError('Qualification imported lifecycle outside this checkout')
    identity=source_identity()
    report,trace=qualify(TeleopLifecycle,seed=args.seed)
    if identity!=source_identity(): raise RuntimeError('Qualification sources changed while running')
    report['source_sha256']=identity
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    with (args.output/'command-trace.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(trace[0]));writer.writeheader();writer.writerows(trace)
    print(json.dumps(dict(all_passed=report['all_passed'],cases=len(report['cases']),
                         report=str(args.output/'report.json')),indent=2))
    return 0 if report['all_passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
