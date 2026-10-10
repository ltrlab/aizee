"""Deterministic control deadlines versus optional observation work; no hardware."""
from types import SimpleNamespace
import time as real_time
import threading

import pytest

from robot_platform.helm import teleop


class Clock:
    def __init__(self): self.now=1.
    def __call__(self): return self.now
    def sleep(self,seconds): self.now+=seconds


class Core:
    period_s=.01
    collision_mode='disabled'
    landing=False

    def __init__(self,clock,*,cycles=120,slow=False):
        self.clock=clock
        self.cycles=cycles
        self.slow=slow
        self.count=0
        self.phase='starting'
        self.state='DISABLED'
        self.fault=None
        self.message='starting'
        self.records=[]
        self.full_views=[]
        self.guide_calls=[]
        self.keys=[]
        self.edits=[]
        self.stopped=False
        self.runtime_view_cost=.002554
        self._manual_state={'session_id':'synthetic','revision':0,'joints':[]}
        self._tuning_pending=self._tuning_confirm=self._tuning_result=None

    def build(self): self.phase='running'
    def land(self): self.stopped=True; return False
    def close(self): self.phase='stopped'
    def fail(self,reason): self.fault=reason; self.phase='fault'
    def route(self,*args): self.edits.append(args); return {'saved':True}
    def set_zero(self): return {'zero':True}
    def queue_tuning(self,payload):
        self._tuning_pending=dict(payload)
        self._tuning_result=None
    def status_view(self):
        return dict(phase=self.phase,state=self.state,fault=self.fault,message=self.message,receiver='executing',input_error=None,joints=[],cycle_ms=None,
                    manual_tuning_ack=self._manual_state,manual_tuning_pending=self._tuning_pending is not None or self._tuning_confirm is not None)
    def view(self):
        self.full_views.append((self.count,self.clock()))
        self.clock.sleep(self.runtime_view_cost if 0<self.count<self.cycles else .002554)
        return dict(self.status_view(),joints=[{'sample':self.count}],cycle_ms={'p99':7.8})
    def guide(self):
        self.guide_calls.append(self.count)
        return {'sequence':self.count}
    def tick(self,key,started):
        expected={4:'following',13:'tuning pending',14:'tuning acknowledged',43:'disabled at operator request',57:'faulted'}.get(self.count)
        if expected:
            assert self.views.seen[expected].wait(1.), 'Immediate status was not queued to the sender'
        self.count+=1
        self.keys.append(key)
        if self._tuning_confirm is not None:
            self._manual_state=dict(self._tuning_confirm,revision=self._tuning_confirm['revision']+1)
            self._tuning_result=(None,self._manual_state)
            self._tuning_confirm=None
        elif self._tuning_pending is not None:
            self._tuning_confirm,self._tuning_pending=self._tuning_pending,None
        if key=='T': self.state='TELEOP'; self.message='following'
        if key=='H': self.state='DISABLED'; self.message='disabled at operator request'
        if self.count==57: self.fault='example receiver fault'; self.state='FAULT'; self.message='faulted'
        # Exactly the observed problematic mandatory duration, or a normal one.
        self.clock.sleep(.007811 if self.slow else .0055)
        # Let the real sender thread consume its mailbox without advancing the
        # synthetic control clock. Pipe blocking never enters the control path.
        real_time.sleep(0)
    def finish_cycle(self,started,scheduled,completed,skipped):
        self.records.append(dict(index=self.count,started=started,scheduled=scheduled,completed=completed,skipped=skipped))


class Commands:
    def __init__(self,core): self.core=core; self.sent=set(); self.closed=False
    def poll(self):
        return self.core.count>=self.core.cycles or self.core.count in (3,7,12,42) and self.core.count not in self.sent
    def recv(self):
        n=self.core.count
        self.sent.add(n)
        if n>=self.core.cycles: return ('stop',)
        if n==3: return ('key','T')
        if n==7: return ('route',9,'flip','left_arm_j1')
        if n==12: return ('manual_tuning',10,{'session_id':'synthetic','revision':0,'joints':[]})
        return ('key','H')
    def close(self): self.closed=True


class Views:
    def __init__(self):
        self.messages=[]; self.closed=False
        self.seen={message:threading.Event() for message in ('following','disabled at operator request','faulted','tuning pending','tuning acknowledged')}
    def send(self,message):
        self.messages.append(message)
        if message[0]=='view' and message[1].get('message') in self.seen:
            self.seen[message[1]['message']].set()
        if message[0]=='view' and message[1].get('manual_tuning_pending'):
            self.seen['tuning pending'].set()
        if message[0]=='view' and (message[1].get('manual_tuning_ack') or {}).get('revision')==1:
            self.seen['tuning acknowledged'].set()
    def close(self): self.closed=True


@pytest.mark.parametrize('slow',[False,True])
def test_full_loop_keeps_control_and_records_every_cycle_with_fast_status_and_replies(monkeypatch,slow):
    clock=Clock()
    core=Core(clock,slow=slow)
    commands,views=Commands(core),Views()
    core.views=views
    monkeypatch.setattr(teleop,'time',SimpleNamespace(sleep=clock.sleep))
    teleop.run_loop(commands,views,core)
    assert len(core.records)==120
    assert all(r['skipped']==0 and r['completed']<=r['scheduled']+.01 for r in core.records)
    assert all(b['started']-a['started']==pytest.approx(.01) for a,b in zip(core.records,core.records[1:]))
    assert core.keys[3]=='T' and core.keys[42]=='H'
    assert core.edits==[('flip','left_arm_j1')]
    assert ('reply',9,None,{'saved':True}) in views.messages
    assert ('reply',10,None,{'session_id':'synthetic','revision':1,'joints':[]}) in views.messages
    full_during_loop=[(n,t) for n,t in core.full_views if 0<n<120]
    if slow:
        assert not full_during_loop  # UI cannot take the last 2.189 ms of a cycle.
    else:
        assert 8<=len(full_during_loop)<=13
    messages=[m[1] for m in views.messages if m[0]=='view']
    immediate=[v for v in messages if v.get('status_at') is not None]
    expected=[(4,lambda v:v['state']=='TELEOP'),
              (13,lambda v:v.get('manual_tuning_pending')),
              (14,lambda v:(v.get('manual_tuning_ack') or {}).get('revision')==1),
              (43,lambda v:v['message']=='disabled at operator request'),
              (57,lambda v:v['fault']=='example receiver fault')]
    for count,matches in expected:
        cycle=core.records[count-1]
        # A due full observation may replace the partial view in the latest-only
        # mailbox; either must carry the status from this very control cycle.
        assert any(matches(v) and cycle['started']<=v.get('status_at',v['observed_at'])<=cycle['completed'] for v in messages)
    # An urgent status view preserves the last observed rows rather than claiming
    # a newly measured pose; the guide is omitted for that partial update.
    assert all(v['observed_at']<=v['status_at'] for v in immediate)
    assert messages[-1]['phase']=='stopped' and messages[-1]['fault']=='example receiver fault'
    assert core.stopped and commands.closed and views.closed


def test_observed_cost_spike_remains_counted_then_prevents_repeated_ui_overruns(monkeypatch):
    clock=Clock()
    core=Core(clock)
    core.runtime_view_cost=.005
    commands,views=Commands(core),Views()
    core.views=views
    monkeypatch.setattr(teleop,'time',SimpleNamespace(sleep=clock.sleep))
    teleop.run_loop(commands,views,core)
    assert len(core.records)==120
    assert sum(r['skipped'] for r in core.records)==1
    assert sum(r['completed']>r['scheduled']+.01 for r in core.records)==1
    assert len([(n,t) for n,t in core.full_views if 0<n<120])==1
    assert any(m[0]=='reply' and m[1]==10 for m in views.messages)
    assert core.stopped and core.fault=='example receiver fault'


def test_status_view_does_not_rebuild_routes_pose_rows_or_quantiles():
    # Exercise the real split with only status data; heavy fields are absent.
    core=object.__new__(teleop.TeleopCore)
    core.phase='running'; core.message='following'; core.fault=None; core.live_arm=None
    core.harness=None; core.compiler=None; core.session=SimpleNamespace(lifecycle=SimpleNamespace(state='TELEOP'),blocked=None)
    core.profile={'receiver':{'direct':{'position_only':True}}}
    core.collision_mode='disabled'; core.input_error=None; core.late_targets=0; core.scheduling={}
    core._manual_state={'session_id':'one','revision':3,'joints':[]}
    core._tuning_pending={}; core._tuning_confirm=None
    result=core.status_view()
    assert result['state']=='TELEOP' and result['position_only'] is True
    assert result['joints']==[] and result['cycle_ms'] is None
    assert result['manual_tuning_ack']['revision']==3 and result['manual_tuning_pending'] is True
