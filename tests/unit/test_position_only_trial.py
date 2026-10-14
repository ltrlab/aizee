"""A selected-joint experiment cannot escape its latched software window.

Feedback is ideal next-cycle command feedback for these command contracts, not a
physical plant or a qualification of the actual robot's tracking response.
"""
from copy import deepcopy
import json
import math
from pathlib import Path

import pytest

from robot_platform.teleop.lifecycle import TeleopLifecycle


NAMES=('left_arm_j1','left_arm_j2','left_gripper')
ORIGIN=(.2,-.08,.06)
TRIAL=dict(motor=NAMES[0],max_velocity=.05,max_excursion_rad=.03)
DT=.01


def joints():
    return [dict(name=name,lower=-2.,upper=2.,hold_kp=30.,hold_kd=3.,max_velocity=1.,
                 align_speed=.2,shutdown_speed=.2,max_lead=.03,tolerance=.001,
                 settled_velocity=.01,zero_position=0.,interpolation_s=0.,
                 direct_velocity=True,position_only=True,friction_nm=.5,max_effort=20.,
                 approach=dict(lower=-2.,upper=2.,max_velocity=1.25,max_tracking_error=.05,
                               stop_acceleration=4.,reaction_s=.04)) for name in NAMES]


class TrialRun:
    def __init__(self, *, origin=ORIGIN, enable=True):
        self.life=TeleopLifecycle(joints(),trial=TRIAL,allow_repeated_source=True,max_source_misses=2)
        self.now=0.
        self.q=list(origin)
        self.feedback_sequence=0
        self.source_sequence=0
        self.source=None
        self.life.action('idle',self.feedback(),now=self.now)
        if enable:
            self.life.action('enable',self.feedback(),now=self.now)

    def feedback(self, q=None):
        self.feedback_sequence+=1
        return [dict(name=name,position=float(position),velocity=0.,sampled_at=self.now,
                     sequence=self.feedback_sequence) for name,position in zip(NAMES,self.q if q is None else q)]

    def target(self, goal):
        self.source_sequence+=1
        self.source=[dict(name=name,position=float(position),sampled_at=self.now,sequence=self.source_sequence)
                     for name,position in zip(NAMES,goal)]
        return deepcopy(self.source)

    def action(self, action, *, goal=None, measured=None):
        self.now=round(self.now+DT,9)
        if measured is not None: self.q=list(measured)
        proposal=self.life.action(action,self.feedback(),target=self.target(goal) if goal is not None else None,
                                  now=self.now)
        if proposal['positions'] is not None: self.q=list(proposal['positions'])
        return proposal

    def step(self, goal=None, *, repeat=False, measured=None, mutate=None):
        self.now=round(self.now+DT,9)
        feedback=self.feedback(measured)
        source=deepcopy(self.source) if repeat else self.target(goal) if goal is not None else None
        if mutate is not None: mutate(source,self.now)
        proposal=self.life.step(feedback,target=source,now=self.now)
        if proposal['positions'] is not None: self.q=list(proposal['positions'])
        return proposal


def assert_trial_reference(proposal, before, *, origin=ORIGIN, check_slew=True):
    assert proposal['positions'] is not None
    q=proposal['positions']
    assert abs(q[0]-origin[0])<=TRIAL['max_excursion_rad']+1e-12
    assert q[1:]==pytest.approx(origin[1:],abs=1e-12,rel=0)
    if check_slew:
        assert abs(q[0]-before[0])<=TRIAL['max_velocity']*DT+1e-12
    assert all(command['velocity']==0. for command in proposal['joints'])
    assert all(value==0. for value in (proposal['ramp_efforts'] or []))


@pytest.mark.parametrize('enable_first',[False,True])
def test_trial_full_alignment_repeated_moving_source_and_reversals_stay_in_one_window(enable_first):
    run=TrialRun(enable=enable_first)
    run.action('engage',goal=(1.,.9,1.5))
    observed_states=set()
    high=low=ORIGIN[0]
    for selected_goal in (1.,-1.,1.):
        for cycle in range(160):
            before=list(run.q)
            # Nine distinct observations and one legitimate duplicate per 100 ms.
            proposal=run.step((selected_goal,.9,1.5),repeat=cycle%10==5)
            assert_trial_reference(proposal,before)
            observed_states.add(proposal['state'])
            high=max(high,run.q[0]);low=min(low,run.q[0])
    assert {'ENGAGING','TELEOP'}<=observed_states
    assert high==pytest.approx(ORIGIN[0]+.03,abs=1e-12)
    assert low==pytest.approx(ORIGIN[0]-.03,abs=1e-12)
    assert run.life.state=='TELEOP'


@pytest.mark.parametrize('restart',['hold','idle'])
def test_hold_or_idle_then_reengage_cannot_walk_the_trial_origin(restart):
    run=TrialRun()
    run.action('engage',goal=(1.,.9,1.5))
    for _ in range(100): run.step((1.,.9,1.5))
    for _ in range(3):
        if restart=='hold':
            held=run.action('disengage')
        else:
            run.action('idle')
            held=run.action('enable')
        assert_trial_reference(held,run.q,check_slew=False)
        run.action('engage',goal=(1.,-.9,-1.5))
        for _ in range(100):
            before=list(run.q)
            assert_trial_reference(run.step((1.,-.9,-1.5)),before)
    assert run.q[0]==pytest.approx(ORIGIN[0]+.03,abs=1e-12)


def test_disable_ends_trial_window_and_next_enable_latches_new_measured_origin():
    run=TrialRun()
    run.action('engage',goal=(1.,.9,1.5))
    for _ in range(100): run.step((1.,.9,1.5))
    run.action('disable')
    new_origin=(-.25,.15,-.12)
    run.action('idle',measured=new_origin)
    run.action('enable')
    run.action('engage',goal=(-1.,.9,1.5))
    for _ in range(100):
        before=list(run.q)
        assert_trial_reference(run.step((-1.,.9,1.5)),before,origin=new_origin)
    assert run.q[0]==pytest.approx(new_origin[0]-.03,abs=1e-12)


def test_source_loss_holds_fixed_inactive_origins_and_requires_explicit_engage():
    run=TrialRun()
    run.action('engage',goal=(1.,.9,1.5))
    for _ in range(100): run.step((1.,.9,1.5))
    # Small physical drift of an inactive joint is still inside its feedback lead
    # bound. Source loss must not adopt that drift as a new inactive target.
    measured=list(run.q)
    measured[1]+=.003
    for _ in range(3):
        proposal=run.step(measured=measured)
        assert_trial_reference(proposal,run.q,check_slew=False)
    assert proposal['state']=='HOLD'
    held=proposal['positions']
    for _ in range(5):
        proposal=run.step((-1.,-.9,-1.5),measured=measured)
        assert proposal['state']=='HOLD' and proposal['positions']==held
    run.action('engage',goal=(-1.,-.9,-1.5),measured=measured)
    for _ in range(160):
        before=list(run.q)
        assert_trial_reference(run.step((-1.,-.9,-1.5)),before)
    assert run.q[0]==pytest.approx(ORIGIN[0]-.03,abs=1e-12)


@pytest.mark.parametrize('bad',['nan','out_of_range','stale','changed_duplicate'])
def test_held_joint_source_is_validated_before_trial_masking(bad):
    run=TrialRun()
    run.action('engage',goal=ORIGIN)
    run.step(ORIGIN)
    def mutate(source,now):
        if bad=='nan': source[1]['position']=math.nan
        elif bad=='out_of_range': source[1]['position']=3.
        elif bad=='stale': source[1]['sampled_at']=now-.051
        else: source[1]['position']+=.01
    with pytest.raises(ValueError):
        run.step(ORIGIN,repeat=bad=='changed_duplicate',mutate=mutate)
    assert run.life.state=='FAULT' and run.life.target is None


@pytest.mark.parametrize('index',[1,2])
def test_inactive_joint_displacement_faults_instead_of_following_its_leader(index):
    run=TrialRun()
    run.action('engage',goal=ORIGIN)
    measured=list(ORIGIN)
    measured[index]+=.031
    with pytest.raises(ValueError,match='trial holding bound'):
        run.step((1.,.9,1.5),measured=measured)
    assert run.life.state=='FAULT'


def test_incompatible_feedback_lead_and_trial_travel_faults_instead_of_large_correction():
    run=TrialRun()
    run.action('engage',goal=ORIGIN)
    measured=list(ORIGIN)
    measured[0]+=.10
    # Even the nearest lead-valid command would be origin+.07, outside the .03
    # experiment and far beyond one .0005-rad following step. No proposal is safe.
    with pytest.raises(ValueError):
        run.step((1.,.9,1.5),measured=measured)
    assert run.life.state=='FAULT' and run.life.target is None


def test_shutdown_returns_every_joint_to_reference_even_outside_trial_window():
    run=TrialRun()
    run.action('engage',goal=(1.,.9,1.5))
    for _ in range(100): run.step((1.,.9,1.5))
    run.action('shutdown')
    inactive_moved=False
    for _ in range(600):
        before=list(run.q)
        proposal=run.step()
        if proposal['state']=='DISABLED': break
        assert proposal['state']=='SHUTDOWN'
        assert all(abs(q-p)<=.05*DT+1e-12 for q,p in zip(proposal['positions'],before))
        assert all(command['velocity']==0 for command in proposal['joints'])
        inactive_moved|=abs(run.q[1]-ORIGIN[1])>.001
    assert inactive_moved
    assert proposal['state']=='DISABLED' and proposal['positions'] is None
    assert run.life._trial_origin is None
    assert run.q==pytest.approx((0.,0.,0.),abs=.001)


def test_trial_caps_apply_after_joint_settings_without_mutating_input_or_raising_lower_caps():
    config=joints()
    config[0].update(max_velocity=.02,align_speed=.01,shutdown_speed=.01)
    before=deepcopy(config)
    trial=deepcopy(TRIAL)
    life=TeleopLifecycle(config,trial=trial)
    trial['max_velocity']=1.
    assert config==before
    assert life.joints[0]['max_velocity']==.02
    assert life.joints[0]['align_speed']==life.joints[0]['shutdown_speed']==.01
    for joint in life.joints[1:]:
        assert joint['max_velocity']==joint['align_speed']==joint['shutdown_speed']==.05
    assert life.trial==TRIAL
    assert [(j['lower'],j['upper'],j['max_lead'],j['hold_kp'],j['hold_kd']) for j in life.joints]==[
            (j['lower'],j['upper'],j['max_lead'],j['hold_kp'],j['hold_kd']) for j in before]


@pytest.mark.parametrize('trial',[{},[],True,
    dict(TRIAL,motor=''),dict(TRIAL,motor='not_present'),dict(TRIAL,max_velocity=0.),
    dict(TRIAL,max_velocity=True),dict(TRIAL,max_excursion_rad=-.01),
    dict(TRIAL,max_excursion_rad=math.inf),dict(TRIAL,max_velocity=math.nan),dict(TRIAL,extra=True)])
def test_trial_rejects_malformed_or_unknown_configuration(trial):
    with pytest.raises(ValueError):
        TeleopLifecycle(joints(),trial=trial)


@pytest.mark.parametrize('index',[0,1,2])
def test_every_trial_joint_must_use_position_only_motor_references(index):
    config=joints()
    config[index]['position_only']=False
    with pytest.raises(ValueError,match='position-only'):
        TeleopLifecycle(config,trial=TRIAL)


def trial_profile_path():
    return Path(__file__).resolve().parents[2]/'config/devices/minerva-teleop-direct-trial.json'


def test_explicit_trial_profile_loads_with_position_only_and_fixed_left_joint():
    from robot_platform.helm.teleop import load_profile
    profile=load_profile(trial_profile_path())
    assert profile['trial']==TRIAL
    assert profile['receiver']['direct']['position_only'] is True
    assert profile['receiver']['collision_mode']=='disabled'
    assert profile['receiver']['loop_hz']==100
    assert profile['runtime_approved'] is False


@pytest.mark.parametrize('value',[None,0,1,'true',[],{}])
def test_direct_position_only_profile_field_must_be_boolean(tmp_path,value):
    from robot_platform.helm.teleop import load_profile
    profile=json.loads(trial_profile_path().read_text())
    profile['receiver']['direct']['position_only']=value
    path=tmp_path/'invalid-position-only.json'
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='Direct teleop'):
        load_profile(path)


@pytest.mark.parametrize('setting',['absent','false','collision_checked'])
def test_trial_profile_cannot_fall_back_to_feedforward_or_collision_checked_mode(tmp_path,setting):
    from robot_platform.helm.teleop import load_profile
    profile=json.loads(trial_profile_path().read_text())
    if setting=='absent': profile['receiver']['direct'].pop('position_only')
    elif setting=='false': profile['receiver']['direct']['position_only']=False
    else:
        profile['receiver'].update(collision_mode='advisory',interpolation_ms=40,compute_budget_s=.01)
        profile['receiver'].pop('direct')
        profile['receiver'].pop('loop_hz')
    path=tmp_path/'invalid-trial-mode.json'
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='explicit direct position-only'):
        load_profile(path)


@pytest.mark.parametrize('arm',['right','both'])
def test_trial_wrong_arm_rejected_before_any_receiver_leader_or_backend_opener(arm):
    from robot_platform.helm.teleop import TeleopCore
    def forbidden(*args,**kwargs):
        pytest.fail('An opener ran before the single-arm trial was validated')
    with pytest.raises(ValueError,match='selected single arm'):
        TeleopCore(dict(teleop_profile=str(trial_profile_path()),receiver=dict(mode='live',arm=arm)),
                   open_leaders=forbidden,open_receiver=forbidden,open_backend=forbidden,open_harness=forbidden)


def test_trial_derivation_keeps_saved_gains_bounds_and_gravity_but_reports_no_friction_feedforward(monkeypatch):
    from tests.unit.test_guarded_config import CONTROL_LAW,Model,REFERENCE,derive,profile
    import robot_platform.control.guarded_config as config_module
    class DirectModel(Model):
        sha256='synthetic-model-for-position-only-contract'
    # The gravity solver has its own model tests. This isolates the profile-to-
    # controller/lifecycle wiring with an explicit analytic synthetic gravity term.
    gravity={'arm_j1':[(2.,{'arm_j1':1})]}
    monkeypatch.setattr(config_module,'gravity_model',lambda *args:deepcopy(gravity))
    reference={'motors':[dict(REFERENCE['motors'][0],uid='aa',firmware='0.1'),
                         dict(REFERENCE['motors'][1],uid='bb',firmware='0.2')]}
    servo={'joints':{'arm_j1':dict(uid='aa',firmware='0.1',hold_kp=12.,hold_kd=1.5,
              max_velocity=.3,stop_acceleration=6.,max_tracking_error=.04,control=CONTROL_LAW,
              measured={'torque':{'friction':.3,'inertia':.1}})},'gravity':{'arm_j1':{'A':2.}}}
    ordinary=profile(interpolation_ms=0,gripper=True)
    ordinary['receiver'].pop('collision')
    ordinary['receiver'].update(collision_mode='disabled',loop_hz=100,
        direct=dict(command_ms=30,velocity_timeout_ms=15,limit_reaction_ms=10,position_only=False))
    position=deepcopy(ordinary)
    position['receiver']['direct']['position_only']=True
    position['trial']=dict(TRIAL,motor='arm_j1')
    before=derive(reference=reference,servo=servo,model=DirectModel,profile=ordinary,backend=None)
    after=derive(reference=reference,servo=servo,model=DirectModel,profile=position,backend=None)
    assert after['config']['direct']['position_only'] is True
    assert after['config']['joints']==before['config']['joints']
    assert (after['config']['joints'][0]['hold_kp'],after['config']['joints'][0]['hold_kd'])==(12.,1.5)
    assert all(j['position_only'] is True for j in after['lifecycle'])
    assert all(r['control_law']=='direct-position-only' and r['friction_ff']==0. for r in after['report'])
    assert all(r['speed_qualified_for_control_law'] is False for r in after['report'])
    # Unramped direct control without position-only feeds a smoothed velocity forward, and no friction either.
    assert before['report'][0]['control_law']=='direct-velocity-feedforward' and before['report'][0]['friction_ff']==0.
    assert [j['velocity_feedforward'] for j in before['lifecycle']]==[True,False]  # Not the gripper.
    assert not any(j['velocity_feedforward'] for j in after['lifecycle'])
    assert after['lifecycle'][0]['friction_nm']==.3  # Preserve measured data, suppress only its command term.
    assert [j['gravity'] for j in after['lifecycle']]==[j['gravity'] for j in before['lifecycle']]
    assert after['lifecycle'][0]['gravity']==[[2.,{'arm_j1':1}]]
    trial=TeleopLifecycle(after['lifecycle'],trial=position['trial'])
    assert all(j['max_velocity']<=.05 and j['align_speed']<=.05 and j['shutdown_speed']<=.05 for j in trial.joints)
    assert after['lifecycle'][0]['max_velocity']==.3  # Trial cap applies after the saved servo merge.


def test_lead_correction_inside_trial_window_still_cannot_exceed_trial_slew():
    run=TrialRun()
    run.action('engage',goal=ORIGIN)
    measured=list(ORIGIN)
    measured[0]+=.04
    # Lead requires q>=.21, which is inside the .17..23 experimental window but
    # outside .1995..2005 at .05 rad/s in this 10 ms step. Fail closed.
    with pytest.raises(ValueError):
        run.step((1.,.9,1.5),measured=measured)
    assert run.life.state=='FAULT' and run.life.target is None
