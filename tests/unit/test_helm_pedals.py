"""Pedal intent freshness, independent stop authority, and HTTP/worker plumbing."""
import asyncio
from collections import deque
from types import SimpleNamespace

import pytest

from robot_platform.helm.pedals import Pedals, OFFER_S
from robot_platform.helm.teleop import TeleopCore, run_loop
from tests.unit.test_helm_teleop import FakeCore, FakeLink, FakeTeleop, ROOT


def context(state='DISABLED', **changes):
    return dict(phase='running', state=state, fault=None, recording_fault=None,
                spent=False, closing=False, task=None, **changes)


def edge(p, c, pedal='c', identity=1, now=1.):
    view=p.view(c, now)
    return dict(session_id=view['session_id'], revision=view['actions'][pedal]['revision'],
                press_id=identity, pedal=pedal)


def accepted(p, c, name, after, identity=1, now=1.):
    intent=p.prepare(edge(p,c,name,identity,now),c,now)
    c['state']=after
    result=p.finish(intent,c)
    assert result['accepted'],result
    return result


def test_ordered_cycle_pause_resume_and_no_implicit_resume():
    p=Pedals();c=context()
    accepted(p,c,'c','IDLE')
    accepted(p,c,'c','HOLD')
    accepted(p,c,'c','ENGAGING')
    assert not p.view(c,1.)['actions']['c']['enabled']
    accepted(p,c,'b','HOLD')
    assert p.paused and p.view(c,1.)['actions']['c']['action']=='shutdown'
    accepted(p,c,'b','ENGAGING')
    c['state']='TELEOP';p.view(c,1.)
    c['state']='HOLD'  # Source loss or ordinary keyboard Hold is not B pause.
    assert not p.view(c,1.)['actions']['b']['enabled']
    assert p.view(c,1.)['actions']['c']['action']=='shutdown'
    accepted(p,c,'c','SHUTDOWN')
    c['state']='DISABLED'
    assert not any(a['enabled'] for a in p.view(c,1.)['actions'].values())
    assert 'Stop this session and start a new one' in p.view(c,1.)['actions']['c']['reason']


def test_in_a_policy_session_with_takeover_b_takes_over_and_hands_back_through_a_hold():
    p=Pedals();c=context(driver='policy')
    assert Pedals().view(context(state='TELEOP'),1.)['actions']['b']['action']=='pause'  # No takeover: B pauses.
    accepted(p,c,'c','IDLE')
    accepted(p,c,'c','HOLD')
    accepted(p,c,'c','ENGAGING')
    c['state']='TELEOP'
    b=p.view(c,1.)['actions']['b']
    assert (b['action'],b['label'],b['enabled'])==('take_over','Take over',True)
    intent=p.prepare(edge(p,c,'b',2),c,1.)
    c['state']='HOLD'  # Held, but the policy still drives: the take over is not confirmed.
    assert not p.finish(intent,c)['accepted'] and not p.paused
    c['state']='TELEOP'
    intent=p.prepare(edge(p,c,'b',3),c,1.)
    c.update(state='HOLD',driver='leaders')
    assert p.finish(intent,c)['accepted'] and p.paused
    b=p.view(c,1.)['actions']['b']
    assert (b['action'],b['label'],b['enabled'])==('resume','Resume: you drive',True)
    accepted(p,c,'b','ENGAGING',4)
    c['state']='TELEOP'
    b=p.view(c,1.)['actions']['b']
    assert (b['action'],b['label'])==('hand_back','Hand back to policy')
    intent=p.prepare(edge(p,c,'b',5),c,1.)
    c.update(state='HOLD',driver='policy')
    assert p.finish(intent,c)['accepted'] and p.paused
    assert p.view(c,1.)['actions']['b']['label']=='Resume: policy drives'
    assert p.view(c,1.)['actions']['c']['action']=='shutdown'


def test_duplicate_lost_reply_expired_offer_and_state_change_do_not_advance():
    p=Pedals();c=context();request=edge(p,c)
    intent=p.prepare(request,c,1.)
    c['state']='IDLE';p.finish(intent,c)  # Reply lost; retry cannot enable.
    with pytest.raises(ValueError,match='Duplicate'):p.prepare(request,c,1.)
    stale=edge(p,c,identity=2)
    with pytest.raises(ValueError,match='stale'):p.prepare(stale,c,1.+OFFER_S)
    changed=edge(p,c,identity=3,now=2.)
    c['state']='HOLD'
    with pytest.raises(ValueError,match='stale'):p.prepare(changed,c,2.)
    # Reloaded browser can restart its counter on a fresh offered revision.
    new=edge(p,c,identity=1,now=2.)
    assert p.prepare(new,c,2.)['action']=='engage'


@pytest.mark.parametrize('bad', [{'session_id':'old'}, {'press_id':True}, {'revision':False}, {'pedal':'x'}])
def test_invalid_or_old_session_edges_rejected(bad):
    p=Pedals();c=context();request=edge(p,c);request.update(bad)
    with pytest.raises(ValueError):p.prepare(request,c,1.)


@pytest.mark.parametrize('stage',['starting','finishing'])
def test_recorder_pending_does_not_block_pause_or_shutdown(stage):
    p=Pedals();c=context('TELEOP')
    c['task']={'capture_ready':True,'recording':{'active':True,'state':'recording'}}
    b=edge(p,c,'b');revision=b['revision']
    p.capture_command('stop_record',{'id':7})
    c['task']['recording']['state']=stage
    view=p.view(c,1.)
    assert not view['actions']['a']['enabled']
    assert view['actions']['b']['enabled'] and view['actions']['c']['enabled']
    assert view['actions']['b']['revision']==revision
    assert 'aborts' in view['actions']['c']['warning']
    intent=p.prepare(b,c,1.);c['state']='HOLD';p.finish(intent,c)
    accepted(p,c,'c','SHUTDOWN')


def test_capture_requires_matching_ack_and_is_independent_of_motion_result():
    p=Pedals();c=context('HOLD')
    c['task']={'capture_ready':True,'recording':{'active':False,'state':'idle'}}
    intent=p.prepare(edge(p,c,'a'),c,1.)
    p.capture_command('start_record',{'id':11});p.finish(intent,c,command_id=11)
    assert not p.view(c,1.)['actions']['a']['enabled']
    c['task'].update(recording={'active':True,'state':'recording'},last_command={'id':10,'state':'complete'})
    assert not p.view(c,1.)['actions']['a']['enabled']
    c['task']['last_command']['id']=11
    assert p.view(c,1.)['actions']['a']['action']=='stop_record'
    assert p.view(c,1.)['actions']['a']['enabled']


@pytest.mark.parametrize('failure',['fault','recording_fault'])
def test_fault_never_reengages_a_paused_arm(failure):
    p=Pedals();c=context('TELEOP');accepted(p,c,'b','HOLD')
    c[failure]='failed'
    view=p.view(c,1.)
    assert not view['paused'] and not view['actions']['b']['enabled']
    assert not view['actions']['a']['enabled']
    assert view['actions']['c']['enabled'] is (failure=='recording_fault')
    assert view['actions']['c']['action']=='shutdown'


class Worker(FakeCore):
    def __init__(self):
        super().__init__();self.pedals=Pedals();self.state='DISABLED';self.requests=0;self.task=None
    def prepare_pedal(self,payload):
        self.requests+=1
        c=context(self.state);c['task']=self.task
        return self.pedals.prepare(payload,c,self.clock())
    def finish_pedal(self,intent,**kwargs):
        return self.pedals.finish(intent,context(self.state),**kwargs)
    def tick(self,key,started):
        super().tick(key,started)
        if key:self.state={'I':'IDLE','E':'HOLD','T':('HOLD' if self.state in ('ENGAGING','TELEOP') else 'ENGAGING'),
                           'H':'DISABLED','X':'SHUTDOWN'}[key]


@pytest.mark.parametrize('safety',['H','X','stop'])
@pytest.mark.parametrize('order',['before','after'])
def test_run_loop_safety_cancels_pending_pedal_and_queued_enable(safety,order):
    core=Worker();payload=edge(core.pedals,context(),now=core.clock())
    pedal=('pedal',1,payload);safe=('stop',) if safety=='stop' else ('key',safety)
    items=deque([pedal,safe] if order=='after' else [safe,pedal])
    if safety!='stop':items.append(('key','E'))  # Same drained burst cannot re-enable.
    sent=[]
    class Commands:
        def poll(self):
            if not items and core.keys:items.append(('stop',))
            return bool(items)
        def recv(self):return items.popleft()
        def close(self):pass
    run_loop(Commands(),SimpleNamespace(send=sent.append,close=lambda:None),core,silence_s=.05)
    assert core.requests==0
    assert core.keys==([] if safety=='stop' else [safety])


@pytest.mark.parametrize('pedal,key',[('b','T'),('c','X')])
def test_same_batch_record_edge_cannot_block_pause_or_return(pedal,key):
    core=Worker();core.state='TELEOP'
    core.task={'capture_ready':True,'recording':{'active':False,'state':'idle'}}
    c=context('TELEOP');c['task']=core.task
    a=edge(core.pedals,c,'a',now=core.clock());motion=edge(core.pedals,c,pedal,now=core.clock())
    items=deque([('pedal',1,a),('pedal',2,motion)]);sent=[]
    core.task_action=lambda action:pytest.fail('Superseded recording must not be dispatched later')
    class Commands:
        def poll(self):
            if not items and core.keys:items.append(('stop',))
            return bool(items)
        def recv(self):return items.popleft()
        def close(self):pass
    run_loop(Commands(),SimpleNamespace(send=sent.append,close=lambda:None),core,silence_s=.05)
    assert core.keys==[key] and core.requests==1
    replies={m[1]:m[2:] for m in sent if m[0]=='reply'}
    assert 'superseded' in replies[1][0] and replies[2][0] is None


@pytest.mark.parametrize('ending',['stop','eof'])
@pytest.mark.parametrize('order',['h_first','h_last'])
def test_pending_immediate_disable_survives_session_stop_or_eof(ending,order):
    if ending=='eof' and order=='h_last':
        # EOF is terminal, so the meaningful trailing-H case is an explicit Stop.
        ending='stop'
    class Landing(Worker):
        landing=False
        def land(self):self.landing=True;return True
        def tick(self,key,started):
            super().tick(key,started);self.landing=False
    core=Landing();core.state='TELEOP'
    stop=('stop',) if ending=='stop' else EOFError('parent closed')
    items=deque([('key','H'),stop] if order=='h_first' else [stop,('key','H')])
    class Commands:
        def poll(self):return bool(items)
        def recv(self):
            item=items.popleft()
            if isinstance(item,Exception):raise item
            return item
        def close(self):pass
    run_loop(Commands(),SimpleNamespace(send=lambda message:None,close=lambda:None),core,silence_s=None)
    assert core.keys==['H'] and core.state=='DISABLED'


def test_http_pedal_is_authenticated_and_preserves_request_binding(tmp_path):
    from aiohttp.test_utils import TestClient,TestServer
    from robot_platform.helm.server import create_app
    from robot_platform.helm.session import HelmSession
    helm=HelmSession(ROOT/'config/devices/minerva-joint-calibration.json',runs=tmp_path/'runs',limits=tmp_path/'limits.json')
    follower=FakeTeleop(helm.guide);follower.active=True
    follower.pedal=lambda payload:follower.calls.append(payload)
    request={'pedal':'b','press_id':1,'session_id':'bound','revision':3}
    async def scenario():
        async with TestClient(TestServer(create_app(helm,FakeLink(),teleop=follower,token='secret'))) as client:
            assert (await client.post('/api/helm/teleop/pedal',json=request)).status==403
            response=await client.post('/api/helm/teleop/pedal',json=request,headers={'X-Helm-Token':'secret'})
            assert response.status==200
            assert 'teleop' in await response.json()
    try:asyncio.run(scenario())
    finally:helm.close()
    assert follower.calls==[request,'stop']  # Application shutdown retains existing cleanup.
