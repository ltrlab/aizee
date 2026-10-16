"""Recording faults hold the actual fourteen-axis receiver; no USB/hardware.

The deterministic simulation clock exercises authority/lifecycle, not physical
object retention or workstation timing. Failure sources are injected at the
capture/diagnostic boundary; the native receiver and ArmPlant are real.
"""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from robot_platform.control.joint_calibration import load_plan
from robot_platform.control.servo_calibration import load_servo
from robot_platform.helm import teleop
from robot_platform.simulation.control_harness import default_binary

ROOT=Path(__file__).resolve().parents[2]
DEV=ROOT/'config/devices'
_spec=importlib.util.spec_from_file_location('capture_fault_fixtures',Path(__file__).with_name('test_helm_sim.py'))
fixtures=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(fixtures)
pytestmark=pytest.mark.integration

class Capture:
    def __init__(self):
        self.failure=None;self.aborted=[];self.commands=[];self.stage='recording'
    def health(self):
        return {'failed':bool(self.failure),'error':self.failure,'available':True,'capture_ready':not self.failure,
                'recording':{'active':self.stage=='recording','state':'failed' if self.failure else self.stage},
                'last_command':{'action':self.stage,'state':'pending'}}
    def snapshot(self):return self.health()
    def abort(self,reason):self.aborted.append(reason)
    def command(self,action):self.commands.append(action);return {'accepted':True}

class Rig:
    def __init__(self,core,clock,leaders):self.core,self.clock,self.leaders=core,clock,leaders
    def tick(self,key=None):
        self.core.tick(key,self.clock());self.clock.now+=self.core.period_s
    def until(self,state,key=None,cycles=2500):
        for i in range(cycles):
            self.tick(key if i==0 else None)
            assert self.core.fault is None,self.core.message
            if self.core.session.lifecycle.state==state:return
        pytest.fail(f'Expected {state}: {self.core.message}')
    def engage(self):
        self.until('TELEOP','T')
        for i in range(30):
            for side in fixtures.CENTERS:self.leaders.offsets[side]=[fixtures.HANG+.015]*7
            self.tick()
        assert self.core.fault is None

@pytest.fixture
def rig(tmp_path,request):
    assert default_binary().is_file(),'Build the actual native control_harness before this required contract'
    plan,_,device,_,_,_,bindings=load_plan(DEV/'minerva-joint-calibration.json')
    directions={step['joint']:dict(motor=step['motor'],offset_rad=0.,direction_verified=True,
        sign=-1 if step['motor'] in {'right_arm_j1','right_arm_j3','right_arm_j4'} else 1) for step in plan['steps']}
    profile_name=getattr(request,'param','minerva-teleop-direct.json')
    profile=json.loads((DEV/profile_name).read_text())
    routes={name:dict(sign=entry['sign'],verified=True) for name,entry in profile['routes'].items()}
    clock=fixtures.Clock();leaders=fixtures.Leaders(clock)
    bundle=dict(calibration_profile=str(DEV/'minerva-joint-calibration.json'),bindings=bindings,directions=directions,
        limits=json.loads((DEV/'minerva-production-01-joint-limits.json').read_text()),
        teleop_profile=str(DEV/profile_name),routes_path=None,route_edits=routes,
        asset_id=device['asset_id'],receiver=({'mode':'virtual'} if profile_name=='minerva-teleop-direct.json' else {'mode':'virtual','collision':'motion'}),leader_ranges=fixtures.leader_ranges(),
        servo=load_servo(DEV/'minerva-production-01-servo.json'),
        simulation={'viewer':False,'directory':str(tmp_path/'receiver')})
    core=teleop.TeleopCore(bundle,open_leaders=lambda _:leaders,clock=clock)
    test=Rig(core,clock,leaders)
    try:
        core.build();test.until('IDLE','I');test.until('HOLD','E')
        assert len(core.session.lifecycle.names)==14
        yield test
    finally:core.close()

@pytest.mark.parametrize('state',['HOLD','ENGAGING','TELEOP'])
@pytest.mark.parametrize('stage',['recording','start_record','stop_record'])
def test_failed_capture_holds_all_axes_renews_authority_and_locks_reengagement(rig,state,stage):
    core=rig.core
    if state=='TELEOP':rig.engage()
    elif state=='ENGAGING':rig.tick('T');assert core.session.lifecycle.state=='ENGAGING'
    capture=core.task_capture=Capture();capture.stage=stage;capture.failure='camera stale / capture worker failed'
    core._tuning_pending={'must_not_apply':True}
    rig.tick('T')  # A queued Engage cannot win a pre-cycle failure.
    assert core.recording_fault==capture.failure and core.fault is None
    assert core.session.lifecycle.state=='HOLD' and core.phase=='running'
    assert core._tuning_pending is None and core._tuning_result[0]
    assert capture.aborted==[capture.failure]
    held=tuple(core.session.lifecycle.target)
    before=core.harness.last['status']['now_ms']
    sequence=rig.leaders.sequence
    for i in range(80):
        for side in fixtures.CENTERS:rig.leaders.offsets[side]=[fixtures.HANG+.5]*7
        rig.tick(('I','E','T')[i%3])
        assert core.fault is None and core.session.lifecycle.state=='HOLD'
        assert core.harness.last['hardware_enabled'] is True
        assert tuple(core.session.lifecycle.target)==held
    assert core.harness.last['status']['now_ms']-before>150
    assert rig.leaders.sequence==sequence  # Failed-session holds do not poll USB.
    assert len(capture.aborted)==1
    for action in ('start_record','stop_record','reset'):
        with pytest.raises(ValueError,match='Recording failed'):core.task_action(action)
    with pytest.raises(ValueError):core.set_zero()
    with pytest.raises(ValueError):core.route('flip','left_arm_j1')
    rig.tick('H')
    assert core.session.lifecycle.state=='DISABLED' and not core.harness.last['hardware_enabled']
    rig.tick('I');assert core.session.lifecycle.state=='DISABLED'


def test_post_cycle_diagnostic_error_holds_next_cycle_without_closing_writer(rig,monkeypatch):
    rig.engage();core=rig.core;capture=core.task_capture=Capture()
    original=core._tick
    def inject(key,started):
        original(key,started);core._recording_error='disk queue overflow'
    monkeypatch.setattr(core,'_tick',inject)
    rig.tick()
    assert core.recording_fault=='disk queue overflow' and core.fault is None
    assert core.session.lifecycle.state=='TELEOP'  # Detected after this accepted command.
    assert core._samples is core._events is core._latency is None
    rig.tick()
    assert core.session.lifecycle.state=='HOLD' and core.harness.last['hardware_enabled']
    assert capture.aborted==['disk queue overflow']
    assert core._recorder is not None  # Cleanup/draining has not run in a powered tick.

@pytest.mark.parametrize('ending_request',['H','X','stop'])
def test_capture_failure_does_not_override_an_operator_shutdown(rig,ending_request):
    rig.engage();core=rig.core;capture=core.task_capture=Capture();capture.failure='disk full'
    core.set_zero()  # Previously queued zero-hold must not replace H, X, or Stop.
    assert core._hold
    if ending_request=='stop':assert core.land()
    rig.tick(ending_request if ending_request in ('H','X') else None)
    assert core.session.lifecycle.state==('DISABLED' if ending_request=='H' else 'SHUTDOWN') and core.fault is None
    if ending_request!='H':rig.until('DISABLED')
    assert not core.harness.last['hardware_enabled']


def test_real_receiver_fault_is_not_masked_by_recording_failure(rig):
    rig.engage();core=rig.core;core.task_capture=Capture();core.task_capture.failure='camera stale'
    rig.clock.now+=.2  # Actual native command validity/lease expires.
    rig.tick()
    assert core.recording_fault=='camera stale'
    assert core.fault and core.session.lifecycle.state=='FAULT'
    assert 'Receiver' in core.message or 'FAULT' in core.message
    rig.tick('H')
    assert not core.harness.last['hardware_enabled']


def test_unconfirmed_guarded_hold_uses_existing_hard_receiver_stop(rig,monkeypatch):
    rig.engage();core=rig.core;core.task_capture=Capture();core.task_capture.failure='disk full'
    original=core.session.cycle
    def skipped(**kwargs):
        record=original(**kwargs);record['skipped']='collision hold could not be certified';return record
    monkeypatch.setattr(core.session,'cycle',skipped)
    rig.tick()
    assert core.fault and core.session.lifecycle.state=='FAULT'
    assert 'hold could not be confirmed' in core.fault
    assert core.harness.last['status']['state'] in ('stopping','stopped','faulted')

@pytest.mark.parametrize('ending',['H','X','stop','eof','silent'])
def test_real_run_loop_retains_failed_capture_hold_until_operator_or_parent_closure(rig,monkeypatch,ending):
    rig.engage();core=rig.core;capture=core.task_capture=Capture()
    core.build=lambda:None  # This fixture already built and engaged the actual receiver.
    ticks=[];original=core.tick
    def tick(key,started):
        if len(ticks)==3:capture.failure='camera worker exited'
        original(key,started)
        ticks.append((core.session.lifecycle.state,core.harness.last['hardware_enabled'],core.recording_fault))
        assert len(ticks)<2000,'run_loop did not finish shutdown'
    core.tick=tick
    class Commands:
        stage=0
        def poll(self):
            if ending=='silent':return False
            return (self.stage==0 and len(ticks)>=45) or (self.stage==1 and core.session.lifecycle.state=='DISABLED')
        def recv(self):
            if self.stage==0:
                self.stage=1 if ending in ('H','X') else 2
                if ending=='eof':raise EOFError('operator window closed')
                if ending in ('H','X'):return ('key',ending)
            self.stage=2
            return ('stop',)
        def close(self):pass
    commands=Commands()
    monkeypatch.setattr(teleop,'time',SimpleNamespace(time=rig.clock,sleep=lambda seconds:setattr(rig.clock,'now',rig.clock.now+seconds)))
    messages=[]
    teleop.run_loop(commands,SimpleNamespace(send=messages.append,close=lambda:None),core,
                    silence_s=.5 if ending=='silent' else None)
    assert any(state=='HOLD' and enabled and failed for state,enabled,failed in ticks),(core.fault,core.message,ticks[:8])
    assert all(state=='HOLD' and enabled for state,enabled,failed in ticks[4:40])
    assert not core.harness.last['hardware_enabled']
    if ending!='H':assert any(state=='SHUTDOWN' for state,_,_ in ticks)
    assert capture.aborted==['camera worker exited']


@pytest.mark.parametrize('state',['DISABLED','IDLE'])
def test_failure_while_disabled_or_idle_cannot_enable_or_request_position_hold(rig,state):
    core=rig.core;rig.tick('H')
    if state=='IDLE':rig.tick('I')
    enabled=core.harness.last['hardware_enabled']  # IDLE keeps the existing damping-only authority.
    core.task_capture=Capture();core.task_capture.failure='camera failed at startup'
    for key in ('E','T','I'):
        rig.tick(key)
        assert core.session.lifecycle.state==state and core.fault is None
        assert core.harness.last['hardware_enabled']==enabled
        assert not core._recording_hold_confirmed


@pytest.mark.parametrize('rig',['minerva-teleop.json'],indirect=True)
def test_capture_fault_retains_native_motion_collision_checks_in_hold(rig,monkeypatch):
    # The actual native collision compiler remains in every guarded HOLD cycle.
    rig.engage();core=rig.core
    assert core.collision_mode=='motion'
    checks=[];compile_original=core.compiler.compile
    def compile_checked(*args,**kwargs):
        result=compile_original(*args,**kwargs);checks.append(result);return result
    monkeypatch.setattr(core.compiler,'compile',compile_checked)
    core.task_capture=Capture();core.task_capture.failure='capture worker died'
    rig.tick('T');held=tuple(core.session.lifecycle.target)
    for _ in range(12):
        rig.tick('T')
        assert core.fault is None and core.session.lifecycle.state=='HOLD'
        assert core.harness.last['hardware_enabled'] and tuple(core.session.lifecycle.target)==held
    assert len(checks)==13
    assert core.compiler.last_verdict['clear']
    rig.until('DISABLED','X')
    assert not core.harness.last['hardware_enabled']
