"""Bounded actual TeleopCore/run_loop tuning rehearsal: private virtual CAN only.

Uses saved robot mappings/limits, synthetic 90 Hz quantized leaders and ideal
emulated motor feedback. This cannot qualify physical gain stability or tracking.
Install this helper under the hashed qualification tree's scripts directory.
--arm both drives both arms in one receiver: each live arm's J1 is tuned and follows
(the right J1 within the same +/-0.03 rad window as the left default); others hold.
"""
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import gc
import io
import json
import math
from multiprocessing import Pipe
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback

import benchmark_direct_loop as bench
from benchmark_direct_teleop import verify_runtime_imports
from robot_platform.control.plain_data import plain_copy
from robot_platform.control.scheduling import prefer_responsive
from robot_platform.helm.teleop import TeleopCore, run_loop
from robot_platform.teleop.latency import distribution, summarize


def assert_namespace():
    rows=json.loads(subprocess.check_output(['ip','-details','-json','link']))
    if ({row['ifname'] for row in rows}!={'lo','vcanDirectL','vcanDirectR'} or
            any(row.get('linkinfo',{}).get('info_kind')!='vcan' for row in rows if row['ifname']!='lo')):
        raise RuntimeError('Qualification requires a private namespace with only loopback and named virtual CAN')
    return rows


class StageProbe:
    """Inclusive stage timings; current-cycle details move intact into its trace."""
    def __init__(self,clock):
        self.clock=clock
        self.samples=defaultdict(list)
        self.current={}

    def call(self,name,function,*args,**kwargs):
        started=self.clock()
        try:
            return function(*args,**kwargs)
        finally:
            ended=self.clock()
            duration=(ended-started)*1000
            self.samples[name].append(duration)
            previous=self.current.get(name)
            self.current[name]=dict(started_at=previous['started_at'] if previous else started,
                ended_at=ended,total_ms=(previous['total_ms'] if previous else 0.)+duration,
                calls=(previous['calls'] if previous else 0)+1)

    def take_cycle(self):
        current,self.current=self.current,{}
        return current


class TimedLatencyStream:
    """Preserve stream calls; async stream timings are enqueue costs, never worker I/O."""
    def __init__(self,stream,probe):
        self.stream,self.probe=stream,probe
        self.kind=('file' if isinstance(stream,io.TextIOBase) else
                   'enqueue' if type(stream).__module__=='robot_platform.helm.recording' else 'stream')

    def write(self,value):
        return self.probe.call('latency_'+self.kind+'_write',self.stream.write,value)

    def flush(self):
        return self.probe.call('latency_'+self.kind+'_flush',self.stream.flush)

    def __getattr__(self,name):
        return getattr(self.stream,name)


def fault_inclusive_timing(records):
    """The generic summarize() filters moves; expose failed operation-null cycles too."""
    following=[r for r in records if r.get('qualification_stage','').startswith('follow')]
    return dict(following_all_rows_full_loop_ms=distribution(r.get('full_loop_ms') for r in following),
        following_all_rows_scheduled_misses=sum(bool(r.get('scheduled_deadline_miss')) for r in following),
        following_all_rows_skipped_slots=sum(r.get('skipped_slots',0) for r in following),
        fault_cycle_max_ms=max((r.get('full_loop_ms',0) for r in records if r.get('failure')),default=None),
        fault_cycles=[{key:r.get(key) for key in ('started_at','qualification_stage','lifecycle_before',
                      'lifecycle_after','operation','failure','full_loop_ms','cycle_ms','skipped_slots',
                      'scheduled_deadline_miss','stage_timing','gc_enabled_at_tick')}
                      for r in records if r.get('failure')],
        generic_following_timing_scope='Move operations only; following_all_rows_full_loop_ms includes failed operation-null rows.')


class Receiver(bench.VirtualReceiver):
    def __init__(self,*args,**kwargs):
        self.tuning_requests=[]
        super().__init__(*args,**kwargs)

    def exchange(self,request):
        # Evidence copies run inside the timed exchange: plain_copy keeps them independent and cheap.
        before=plain_copy((self.last or {}).get('status',{})) if request.get('update_hold_gains') else None
        try:
            return super().exchange(request)
        finally:
            if before is not None:
                self.tuning_requests.append(dict(request=plain_copy(request),before=before,
                                                after=plain_copy(self.last),at=time.perf_counter()))


class Driver:
    def __init__(self,core,seconds,arm='left'):
        self.core=core
        self.followed=[a+'_arm_j1' for a in (('left','right') if arm=='both' else (arm,))]
        self.seconds=seconds
        self.deadline=core.clock()+seconds+70
        self.plan=[('idle','key','I','IDLE'),('enable','key','E','HOLD'),
                   ('settle_initial','wait',.15,None),('apply_initial','tune','apply',None),
                   ('reject_stale','tune','stale',None),('engage','key','T','TELEOP'),
                   ('follow_long','follow',seconds,None),('hold','key','T','HOLD'),
                   ('settle_hold','wait',.15,None),('apply_again','tune','apply',None),
                   ('reject_invalid','tune','invalid',None),('reengage','key','T','TELEOP'),
                   ('follow_after_apply','follow',5.1,None),('drain','drain',None,None),
                   ('shutdown','key','X','DISABLED'),('done','stop',None,None)]
        self.index=0
        self.pending=None
        self.sent_cycle=-1
        self.entered=core.clock()
        self.issued=False
        self.request_id=0
        self.awaiting=None
        self.failure=None
        self.transitions=[]
        self.checks=[]
        self.replies={}
        self.reply_lock=threading.Lock()
        self.applied=set()
        self.last_following=None
        self.follow_started=False
        self.engaged_rejection_sent=False
        self.engaged_rejection_seen=False
        self.leaders=None
        self.phase='idle'
        self.abort_sent=False

    def receive(self,message):
        if message[0]=='reply':
            with self.reply_lock:
                self.replies[message[1]]=(message[2],message[3])

    def observe_cycle(self,record):
        applied=record.get('receiver_applied')
        if applied:
            self.applied.add((record.get('receiver_epoch'),applied['sequence'],applied['accepted_at_ms']))
        if self.phase.startswith('follow') and record.get('accepted') and record.get('request_sequence') is not None:
            self.last_following=(record.get('receiver_epoch'),record['request_sequence'],record['receiver_accepted_ms'])

    def check(self,name,passed,**evidence):
        self.checks.append(dict(name=name,passed=bool(passed),**evidence))
        if not passed: raise RuntimeError('Manual tuning qualification failed: '+name)

    def advance(self):
        self.transitions.append(dict(stage=self.phase,at=self.core.clock(),state=self.core.session.lifecycle.state))
        self.index+=1
        self.entered=self.core.clock()
        self.issued=False
        self.phase=self.plan[self.index][0]

    def tune(self,kind):
        payload=deepcopy(self.core._manual_state)
        before=deepcopy(payload)
        firsts=[row for row in payload['joints'] if row['motor'] in self.followed]
        if kind=='apply':
            for first in firsts:
                cap=next(row for row in self.core._manual_rows if row['motor']==first['motor'])
                factor=.9 if payload['revision']==0 else 1.05
                first['kp']=min(cap['bounds']['kp'][1],first['kp']*factor if first['kp'] else .1)
                first['kd']=min(cap['bounds']['kd'][1],first['kd']*factor if first['kd'] else .01)
                if payload['revision']==0 and not first['follow']:
                    # Only the left J1 follows by default; a live right J1 gets the same small window.
                    first.update(follow=True,travel_rad=.03,speed=min(.05,cap['bounds']['speed'][1]))
        elif kind=='stale': payload['revision']-=1
        elif kind=='invalid': firsts[0]['kd']=-1.
        self.request_id+=1
        self.awaiting=dict(id=self.request_id,kind=kind,before=before,payload=deepcopy(payload),
                           target=list(self.core.session.lifecycle.target),at=self.core.clock())
        return ('manual_tuning',self.request_id,payload)

    def consume_reply(self):
        wait=self.awaiting
        if wait is None: return False
        with self.reply_lock:
            reply=self.replies.pop(wait['id'],None)
        if reply is None:
            if self.core.clock()-wait['at']>2: raise TimeoutError('Tuning reply exceeded two seconds')
            return False
        error,result=reply
        current=self.core._manual_state
        if wait['kind']=='apply':
            transactions=self.core.receiver_fixture.tuning_requests
            native=transactions[-1] if transactions else {}
            status=(native.get('after') or {}).get('status') or {}
            seq=(status.get('lease') or {}).get('last_sequence')
            observed=(status.get('epoch'),seq,status.get('now_ms')) in self.applied
            expected={r['motor']:r for r in wait['payload']['joints']}
            joints=self.core.session.lifecycle.joints
            actual=(self.core.harness.last or {}).get('status',{}).get('joints',[])
            gains=len(joints)==len(actual) and all(
                row['kp']==expected[joint['name']]['kp'] and row['kd']==expected[joint['name']]['kd']
                for joint,row in zip(joints,actual))
            self.check(self.phase,error is None and result==current and current['revision']==wait['before']['revision']+1
                       and self.core.session.lifecycle.state=='HOLD' and list(self.core.session.lifecycle.target)==wait['target']
                       and observed and gains,error=error,reply=result,applied_observed=observed,native_gains_match=gains)
        else:
            self.check(self.phase+'_'+wait['kind'],bool(error) and result is None and current==wait['before'],
                       error=error,revision_unchanged=current==wait['before'])
        self.awaiting=None
        return True

    def poll(self):
        if self.pending is not None: return True
        if self.sent_cycle==self.core.driver_cycle: return False
        state=self.core.session.lifecycle.state
        try:
            if self.core.fault: raise RuntimeError(self.core.fault)
            if self.core.clock()>self.deadline: raise TimeoutError('Bounded qualification deadline exceeded')
            _,kind,value,expected=self.plan[self.index]
            message=None
            if kind=='key':
                if not self.issued: message=('key',value); self.issued=True
                elif state==expected: self.advance()
            elif kind=='wait':
                if self.core.clock()-self.entered>=value: self.advance()
            elif kind=='tune':
                if not self.issued: message=self.tune(value); self.issued=True
                elif self.consume_reply(): self.advance()
            elif kind=='follow':
                if not self.follow_started:
                    self.leaders.start_following(self.core.clock())
                    self.follow_started=True
                if self.phase=='follow_long' and not self.engaged_rejection_sent:
                    message=self.tune('engaged'); self.engaged_rejection_sent=True
                elif self.awaiting is not None:
                    self.engaged_rejection_seen=self.consume_reply() or self.engaged_rejection_seen
                if self.core.clock()-self.entered>=value and self.awaiting is None: self.advance()
            elif kind=='drain':
                if self.last_following in self.applied: self.advance()
                elif self.core.clock()-self.entered>.05: raise TimeoutError('Last measured target application not observed')
            elif kind=='stop': message=('stop',)
            if message is not None:
                self.pending=message; self.sent_cycle=self.core.driver_cycle
        except Exception as exc:
            self.failure=self.failure or str(exc)
            self.phase='abort'
            self.pending=('stop',) if self.abort_sent else ('key','H')
            self.abort_sent=True
            self.sent_cycle=self.core.driver_cycle
        return self.pending is not None

    def recv(self):
        message,self.pending=self.pending,None
        return message

    def close(self): pass


class Core(TeleopCore):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.driver_cycle=0
        self.probe=StageProbe(self.clock)
        self.stage_samples=self.probe.samples
        self.gc_enabled_after_build=None
        self.gc_enabled_any_cycle=False
        self.gc_enabled_at_tick=None
        self.latency_sink=None

    def build(self):
        result=super().build()
        self.gc_enabled_after_build=gc.isenabled()
        stream=getattr(self,'_latency',None)
        if stream is not None:
            proxy=TimedLatencyStream(stream,self.probe)
            self.latency_sink=dict(type=type(stream).__module__+'.'+type(stream).__name__,
                                  measured_operation=proxy.kind)
            self._latency=proxy
        return result

    def tick(self,key,started):
        # Startup/final observation costs remain aggregated but outside cycle details.
        self.probe.take_cycle()
        self.gc_enabled_at_tick=gc.isenabled()
        self.gc_enabled_any_cycle|=self.gc_enabled_at_tick
        return super().tick(key,started)

    def finish_cycle(self,started,scheduled,completed,skipped_slots=0):
        super().finish_cycle(started,scheduled,completed,skipped_slots)
        self.driver_cycle+=1
        row=self._pending_latency
        stages=self.probe.take_cycle()
        row.update(qualification_stage=self.driver.phase,lifecycle=self.session.lifecycle.state,
                   manual_revision=(self._manual_state or {}).get('revision'),
                   scripted_source_stage=getattr(self.driver.leaders,'stage',None),
                   command_proposed_at=(self.session.last_cycle or {}).get('mapped_at'),
                   stage_timing=stages,gc_enabled_at_tick=self.gc_enabled_at_tick)
        flush=stages.get('_flush_latency')
        if flush:
            writes=sum(stage['total_ms'] for name,stage in stages.items()
                       if name.startswith('latency_') and name.endswith('_write'))
            row['latency_json_and_bookkeeping_estimate_ms']=max(0.,flush['total_ms']-writes)
        self.driver.observe_cycle(row)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--source-manifest',type=Path,required=True)
    p.add_argument('--runs',type=Path,required=True)
    p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=125.)
    p.add_argument('--profile',default='config/devices/minerva-teleop-direct-tuning.json')
    p.add_argument('--arm',choices=('left','right','both'),default='left')
    args=p.parse_args()
    root,out=args.root.resolve(),args.output.resolve()
    if sys.platform!='linux' or not 120<=args.seconds<=240 or out.exists() or out.is_relative_to(root):
        p.error('Linux private-vcan only, 120-240 seconds, and a new output outside candidate required')
    if Path(__file__).resolve()!=root/'scripts/qualify_manual_tuning_loop.py':
        p.error('Run the helper installed under the hashed candidate scripts directory')
    binary=args.binary.resolve()
    if not binary.is_relative_to(root): p.error('Binary must be inside verified candidate')
    links=assert_namespace()
    out.mkdir(parents=True)
    report=dict(arm=args.arm,complete=False,failure=None,physical_validation=False,source='synthetic 90 Hz quantized saved-reference leaders',
                namespace_links=links,requested_following_s=args.seconds,script_sha256=bench.sha(__file__))
    core=driver=receiver=None
    messages=[]
    try:
        report['source_identity'],release=bench.prepare_candidate(root,args.source_manifest)
        report['release']=release and {k:release[k] for k in ('id','content_sha256')}
        report['binary_sha256']=bench.sha(binary)
        bundle,ranges,report['inputs']=bench.make_bundle(root,out,args.arm,args.runs.resolve(),root/args.profile)
        if not bundle.get('manual_tuning'): raise ValueError('Saved-derived, explicit manual tuning bundle is required')
        def open_source(profile):
            driver.leaders=bench.QuantizedLeaders(ranges)
            return driver.leaders
        def open_receiver(config,device,reference):
            nonlocal receiver
            (out/'controller.json').write_text(json.dumps(config,indent=2)+'\n')
            receiver=Receiver(config,reference,bundle['receiver']['motor_models'],binary,out,args.seconds+80)
            core.receiver_fixture=receiver
            return receiver
        def no_geometry(_): raise RuntimeError('Direct qualification attempted collision geometry')
        core=Core(bundle,open_leaders=open_source,open_receiver=open_receiver,open_backend=no_geometry)
        if core.period_s!=.01 or not core.profile.get('manual_tuning') or not core.profile['receiver']['direct'].get('position_only'):
            raise ValueError('Expected manual position-only 100 Hz profile')
        core.scheduling['loop']=prefer_responsive()
        driver=Driver(core,args.seconds,args.arm); core.driver=driver
        reader,writer=Pipe(duplex=False)
        def consume():
            try:
                while True:
                    message=reader.recv()
                    if message[0]=='reply': driver.receive(message); messages.append(message)
                    elif message[0]=='view':
                        v=message[1]
                        messages.append(('view',dict(observed_at=v.get('observed_at'),status_at=v.get('status_at'),
                            state=v.get('state'),pending=v.get('manual_tuning_pending'),
                            revision=(v.get('manual_tuning_ack') or {}).get('revision'))))
            except EOFError: pass
            finally: reader.close()
        reader_thread=threading.Thread(target=consume,daemon=True)
        reader_thread.start()
        # Original methods remain intact; tiny timing records join the normal trace.
        for name in ('_record','_note','_flush_latency','view','guide'):
            original=getattr(core,name)
            def timed(*a,_name=name,_fn=original,**kw):
                return core.probe.call(_name,_fn,*a,**kw)
            setattr(core,name,timed)
        run_loop(driver,writer,core)
        reader_thread.join(3)
        verify_runtime_imports(root)
        report.update(failure=driver.failure or core.fault,checks=driver.checks,transitions=driver.transitions,
                      final_state=core.session.lifecycle.state,final_snapshot=receiver.last,
                      first_fault=receiver.first_fault,startup_fault=receiver.startup_fault,emulator_error=receiver.close_error,
                      tuning_requests=receiver.tuning_requests,
                      stages_ms={name:distribution(values) for name,values in core.stage_samples.items()},
                      gc=dict(enabled_after_build=core.gc_enabled_after_build,
                              enabled_during_any_cycle=core.gc_enabled_any_cycle,enabled_after_close=gc.isenabled()),
                      latency_sink=core.latency_sink,
                      stage_timing_notes='Per-cycle inclusive stages include enqueue timings for async streams, never worker I/O. Flush minus write is an approximate JSON/bookkeeping remainder; GC state is measured, not changed by instrumentation.')
    except Exception as exc:
        report['failure']=str(exc); report['traceback']=traceback.format_exc()
        if core: core.close()
    finally:
        if core:
            report.update(stages_ms={name:distribution(values) for name,values in core.stage_samples.items()},
                          gc=dict(enabled_after_build=core.gc_enabled_after_build,
                                  enabled_during_any_cycle=core.gc_enabled_any_cycle,enabled_after_close=gc.isenabled()),
                          latency_sink=core.latency_sink)
        trace=out/'direct-latency.jsonl'
        records=[json.loads(line) for line in trace.read_text().splitlines()] if trace.exists() else []
        following=[r for r in records if r.get('qualification_stage','').startswith('follow')]
        long=[r for r in following if r.get('qualification_stage')=='follow_long']
        observations=[dict(r,operation='observation') for r in records if not r.get('qualification_stage','').startswith('follow')]
        timing=summarize(following+observations,.01)
        report['following_timing']=timing
        report.update(fault_inclusive_timing(records))
        report['full_loop_by_stage']={stage:distribution(r.get('full_loop_ms') for r in records if r.get('qualification_stage')==stage)
                                      for stage in sorted({r.get('qualification_stage','unknown') for r in records})}
        report['long_following_duration_s']=(long[-1]['started_at']-long[0]['started_at']) if len(long)>1 else 0
        report['all_scheduled_misses']=sum(bool(r.get('scheduled_deadline_miss')) for r in records)
        report['all_skipped_slots']=sum(r.get('skipped_slots',0) for r in records)
        powered=[r for r in records if r.get('lifecycle_before') in ('HOLD','ENGAGING','TELEOP','SHUTDOWN')]
        report['powered_scheduled_misses']=sum(bool(r.get('scheduled_deadline_miss')) for r in powered)
        report['powered_skipped_slots']=sum(r.get('skipped_slots',0) for r in powered)
        report['views_and_replies']=messages
        report['position_only_output']=bool(following) and all(r.get('commanded_velocity') and all(v==0 for v in r['commanded_velocity']) for r in following)
        stages=Counter(r.get('scripted_source_stage') for r in following)
        commands=[r['commanded'] for r in following if r.get('accepted') and r.get('commanded')]
        names=[j['name'] for j in core.derived['config']['joints']] if core and core.derived else []
        joints={}
        for name in (driver.followed if driver else []):
            q=[c[names.index(name)] for c in commands] if name in names else []
            steps=[b-a for a,b in zip(q,q[1:])]
            joints[name]=dict(command_range_rad=max(q)-min(q) if q else 0.,
                              reversal=any(x>1e-8 for x in steps) and any(x<-1e-8 for x in steps))
        q1=[r[0] for r in commands]
        delta=[b-a for a,b in zip(q1,q1[1:])]
        repeats=[(a,b) for a,b in zip(following,following[1:]) if a.get('source_sequence') and a.get('source_sequence')==b.get('source_sequence')]
        immutable=all(a['desired']==b['desired'] and abs((a['completed_at']-a['source_age_ms']/1000)
                      -(b['completed_at']-b['source_age_ms']/1000))<1e-8 for a,b in repeats)
        report['moving_source_coverage']=dict(stages=dict(stages),repeated_samples=len(repeats),immutable_repeats=immutable,
            j1_command_range_rad=max(q1)-min(q1) if q1 else 0.,
            reversal=any(x>1e-8 for x in delta) and any(x<-1e-8 for x in delta),followed_joints=joints)
        # Every followed J1 (each live arm) must move and reverse, not only the receiver's first joint.
        movement=bool(joints and all(j['command_range_rad']>1e-5 and j['reversal'] for j in joints.values())
                      and len(repeats)>2 and immutable and all(stages.get(stage,0)>=2 for stage in bench.QuantizedLeaders.stages))
        report['processing_target_met']=bool(timing['full_loop_ms'] and timing['full_loop_ms']['p99']<8.)
        report['scheduling_target_met']=bool(following) and timing['scheduled_deadline_misses']==0 and timing['skipped_slots']==0 and report['powered_scheduled_misses']==0 and report['powered_skipped_slots']==0
        report['complete']=bool(not report['failure'] and not report.get('emulator_error') and driver and driver.phase=='done'
            and report.get('final_state')=='DISABLED' and report.get('final_snapshot',{}).get('hardware_enabled') is False
            and report['long_following_duration_s']>=120 and len(driver.checks)==5 and all(c['passed'] for c in driver.checks)
            and driver.engaged_rejection_seen and len(receiver.tuning_requests)==2
            and timing['receiver_unobserved_targets']==0 and report['position_only_output'] and movement)
        (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('complete','failure','long_following_duration_s','processing_target_met','scheduling_target_met','fault_cycle_max_ms')},indent=2))
    return 0 if report['complete'] and report['processing_target_met'] and report['scheduling_target_met'] else 1


if __name__=='__main__': raise SystemExit(main())
