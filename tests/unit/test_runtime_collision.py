import json
import math

import numpy as np
import pytest

pytest.importorskip("fcl")
trimesh = pytest.importorskip("trimesh")

from robot_platform.models import ModelProfile
from robot_platform.simulation.collision import CollisionError
from robot_platform.simulation.runtime_model import RuntimeCollisionModel, build_runtime_model
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path


def make_runtime(tmp_path, *, meshes=False):
    shape = '<mesh filename="cube.stl"/>' if meshes else '<box size=".2 .2 .2"/>'
    if meshes:
        trimesh.creation.box(extents=[.2, .2, .2]).export(tmp_path / "cube.stl")
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(f'''<robot name="test"><link name="root"/>
    <link name="a"><collision><geometry>{shape}</geometry></collision></link>
    <link name="b"><collision><geometry>{shape}</geometry></collision></link>
    <joint name="left" type="prismatic"><parent link="root"/><child link="a"/>
    <axis xyz="1 0 0"/><limit lower="-3" upper="3"/></joint>
    <joint name="right" type="prismatic"><parent link="root"/><child link="b"/>
    <axis xyz="1 0 0"/><limit lower="-3" upper="3"/></joint></robot>''')
    profile = ModelProfile.from_urdf(urdf)
    path = tmp_path / "runtime.json"
    build_runtime_model(profile, path)
    return RuntimeCollisionModel(profile, path)


def test_source_coverage_and_repeated_mesh_instances(tmp_path):
    runtime = make_runtime(tmp_path, meshes=True)
    assert len(runtime.shapes) == 2
    assert len({id(obj) for obj in runtime.scene._objects}) == 2
    for shape in runtime.shapes:
        hull = trimesh.Trimesh(shape["vertices"], shape["faces"], process=False)
        assert hull.is_watertight and hull.is_convex and hull.volume > .2**3
        source = trimesh.creation.box(extents=[.2, .2, .2]).vertices
        excess = source @ hull.face_normals.T - np.einsum("ij,ij->i",hull.face_normals,hull.triangles[:,0])
        assert excess.max() <= 1e-8
    assert runtime.proximity({"left":0, "right":0})["minimum_distance_m"] == 0
    assert runtime.proximity({"left":-1, "right":1})["near_pairs"] == []
    assert runtime.proximity({"left":0, "right":.22})["minimum_distance_m"] == pytest.approx(.02, abs=1e-7)


def test_runtime_rejects_changed_source_and_missing_shape(tmp_path):
    runtime = make_runtime(tmp_path, meshes=True)
    data = json.loads(runtime.path.read_text())
    data["shapes"].pop()
    runtime.path.write_text(json.dumps(data))
    with pytest.raises(CollisionError, match="every collision"):
        RuntimeCollisionModel(runtime.profile,runtime.path)
    runtime.path.write_text(json.dumps(runtime.data))
    trimesh.creation.box(extents=[.3,.3,.3]).export(tmp_path/"cube.stl")
    with pytest.raises(CollisionError, match="revision changed"):
        RuntimeCollisionModel(runtime.profile,runtime.path)


def test_runtime_rejects_undersized_broad_phase_bounds(tmp_path):
    runtime=make_runtime(tmp_path,meshes=True)
    data=json.loads(runtime.path.read_text())
    data['shapes'][0]['vertices']=(np.asarray(data['shapes'][0]['vertices'])*.8).tolist()
    runtime.path.write_text(json.dumps(data))
    with pytest.raises(CollisionError,match="does not enclose source CAD"):
        RuntimeCollisionModel(runtime.profile,runtime.path)


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_hollow_bracket_rejects_hull_false_contacts_and_keeps_real_contacts(tmp_path):
    # A single source collision mesh with a U-shaped opening. Its convex
    # enclosure fills the opening, but the probe fits without touching CAD.
    pieces=[]
    for extents,center in [((.02,.2,.1),(-.09,0,0)),((.02,.2,.1),(.09,0,0)),
                           ((.16,.02,.1),(0,-.09,0))]:
        piece=trimesh.creation.box(extents=extents);piece.apply_translation(center);pieces.append(piece)
    trimesh.util.concatenate(pieces).export(tmp_path/'bracket.stl')
    path=tmp_path/'bracket.urdf'
    path.write_text('''<robot name="bracket"><link name="base"><collision><geometry>
    <mesh filename="bracket.stl"/></geometry></collision></link>
    <link name="probe"><collision><geometry><box size=".02 .02 .02"/></geometry></collision></link>
    <joint name="x" type="prismatic"><parent link="base"/><child link="probe"/>
    <axis xyz="1 0 0"/><limit lower="-1" upper="1"/></joint></robot>''')
    profile=ModelProfile.from_urdf(path); artifact=tmp_path/'bounds.json'
    build_runtime_model(profile,artifact)
    runtime=RuntimeCollisionModel(profile,artifact); native=NativeCollisionBackend(runtime)
    clear=native.proximity({'x':0},threshold=0.)
    assert clear['candidate_pairs']==1 and clear['near_pairs']==[]
    envelope = NativeCollisionBackend(runtime,geometry_mode='keepout').proximity({'x':0},threshold=0.)
    assert envelope['near_pairs'] and envelope['geometry_basis']=='conservative_envelope'
    assert envelope['verdict_kind']=='keepout_boundary'
    assert envelope['broad_phase']['fcl_collision_tests']==0
    assert native.proximity({'x':0},threshold=.1)['minimum_distance_m']==pytest.approx(.07,abs=1e-7)
    for x in np.linspace(-.15,.15,61):
        q={'x':float(x)}
        actual=native.proximity(q,threshold=0.)
        expected=runtime.scene.check_shape_pairs(q)
        assert {tuple(sorted(p['shapes'])) for p in actual['near_pairs']}==set(expected)
    assert native.proximity({'x':.085},threshold=0.)['near_pairs']
    assert native.proximity({'x':0},threshold=.1,max_pairs=1)['minimum_distance_m']==pytest.approx(.07,abs=1e-7)


@pytest.mark.skipif(not library_path().is_file(), reason="Build robot_collision release library first")
@pytest.mark.parametrize("meshes", [False, True])
def test_native_matches_fcl_distances_and_threaded_queries(tmp_path, meshes):
    from concurrent.futures import ThreadPoolExecutor
    runtime = make_runtime(tmp_path, meshes=meshes)
    native = NativeCollisionBackend(runtime)
    qs = [{"left":float(x),"right":float(-x)} for x in np.linspace(-1,1,101)]
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda q:native.proximity(q,threshold=2),qs))
    for q, actual in zip(qs,results):
        expected = runtime.proximity(q,threshold=2)
        assert actual["minimum_distance_m"] == pytest.approx(expected["minimum_distance_m"], abs=2e-6)
        assert [p["shapes"] for p in actual["near_pairs"]] == [p["shapes"] for p in expected["near_pairs"]]
    with pytest.raises(CollisionError): native.proximity({"left":math.nan,"right":0})
    native.close()
    with pytest.raises(CollisionError,match="closed"): native.proximity(qs[0])


@pytest.fixture
def guarded_session(tmp_path):
    if not library_path().is_file(): pytest.skip("Build native backend")
    from types import SimpleNamespace
    from robot_platform.control.session import RobotSession
    from robot_platform.control.collision_guard import CollisionCommandGate
    runtime=make_runtime(tmp_path)
    backend=NativeCollisionBackend(runtime)
    clock=[10.]
    gate=CollisionCommandGate(backend,
        {"left":{"actuator":"a","scale":1.,"offset":0.},"right":{"actuator":"b","scale":1.,"offset":0.}},
        braking_acceleration={"left":2.,"right":2.},max_speed={"left":2.,"right":2.},
        mode="enforce_simulation",clock=lambda:clock[0])
    spec=SimpleNamespace(joints=[SimpleNamespace(name=n,active=True) for n in ("a","b")])
    settings={"simulation":{"evidence":"synthetic"},"endpoints":{"arms":{
        "left":{"command":"tcp://127.0.0.1:5001","telemetry":"tcp://127.0.0.1:5002","joints":["a"]},
        "right":{"command":"tcp://127.0.0.1:5003","telemetry":"tcp://127.0.0.1:5004","joints":["b"]}}}}
    session=RobotSession(spec,settings,command_gate=gate)
    for arm,name,value in (("left","a",-1.),("right","b",1.)):
        session.ingest_telemetry(arm,{"telemetry_schema_version":2,"producer_id":arm,"motors":{
            name:{"position":value,"velocity":0.,"torque":0.,"feedback":{
                "sequence":1,"age_s":0.,"valid":True,"provenance":"simulation"}}}},received_at=clock[0])
    return session,gate,clock


def command(left,right):
    return {arm:{"type":"arm_joints","positions":[value],"velocities":[0.]} for arm,value in (("left",left),("right",right))}


def test_gate_checks_crossing_path_and_allows_clear_motion(guarded_session):
    session,gate,clock=guarded_session
    clear=command(-.9,.9)
    assert gate.filter_commands(session,clear)==clear
    assert gate.last_verdict["clear"]
    assert gate.dispatch_valid(session,clear)
    assert not gate.dispatch_valid(session,command(-.8,.8))
    for arm,name,value in (("left","a",-.2),("right","b",.2)):
        message,stamp=session.telemetry_sample(arm)
        message["motors"][name]["position"]=value
        message["motors"][name]["feedback"]["sequence"]=2
        session.ingest_telemetry(arm,message,received_at=stamp)
    collision=gate.filter_commands(session,command(.2,-.2))
    assert all(c["type"]=="emergency_stop" for c in collision.values())
    assert gate.last_verdict["reason"]=="insufficient_swept_clearance"


def test_gate_checks_stopping_path_not_only_target(guarded_session):
    session,gate,_=guarded_session
    for arm,name in (("left","a"),("right","b")):
        message,stamp=session.telemetry_sample(arm)
        message["motors"][name]["position"]=-.2 if arm=="left" else .2
        message["motors"][name]["velocity"]=1. if arm=="left" else -1.
        message["motors"][name]["feedback"]["sequence"]=2
        session.ingest_telemetry(arm,message,received_at=stamp)
    assert all(c["type"]=="emergency_stop" for c in gate.filter_commands(session,command(-.2,.2)).values())


def test_gate_staleness_expiry_partial_and_new_state(guarded_session):
    session,gate,clock=guarded_session
    clear=command(-.9,.9)
    gate.filter_commands(session,clear)
    message,stamp=session.telemetry_sample("left")
    session.ingest_telemetry("left",message,received_at=stamp)
    assert not gate.dispatch_valid(session,clear)
    gate.filter_commands(session,clear)
    clock[0]+=.02
    assert not gate.dispatch_valid(session,clear)
    clock[0]+=.1
    assert all(c["type"]=="emergency_stop" for c in gate.filter_commands(session,clear).values())
    assert "state" in gate.last_verdict["reason"]
    clock[0]=10.
    assert len(gate.filter_commands(session,{"left":clear["left"]}))==2
    assert gate.last_verdict["reason"]=="complete_whole_robot_arm_target_required"


def test_gate_refuses_hardware_and_observation_does_not_alter_commands(guarded_session):
    session,gate,_=guarded_session
    session.simulation_only=False
    with pytest.raises(ValueError,match="synthetic"):
        gate.filter_commands(session,command(1.,-1.))
    gate.mode="observe"
    commands=command(1.,-1.)
    assert gate.filter_commands(session,commands)==commands
    assert not gate.last_verdict["clear"]


def test_motion_bound_prevents_tunneling_with_a_single_midpoint(guarded_session):
    from robot_platform.control.collision_guard import check_linear_motion
    _,gate,_=guarded_session
    # The collision is at t=.2; neither endpoint nor the midpoint collides.
    result=check_linear_motion(gate.backend,{"left":-1.,"right":-.6},{"left":1.,"right":-.6},
                               max_segment_travel_m=10,max_queries=1)
    assert not result["clear"]
    assert result["queries"]==1


def test_transport_rechecks_expiry_between_arm_sends(guarded_session):
    from types import SimpleNamespace
    from robot_platform.control.session import ZmqRobotClient
    from robot_platform.transport.wire import unpack_msg
    session,gate,clock=guarded_session
    client=ZmqRobotClient(session)
    frames={arm:[] for arm in session.arms}
    class Socket:
        def __init__(self,arm): self.arm=arm
        def send(self,raw,flags):
            frames[self.arm].append(unpack_msg(raw))
            if self.arm=="left": clock[0]+=.02
    client._push={arm:Socket(arm) for arm in session.arms}
    client._zmq=SimpleNamespace(NOBLOCK=0)
    with pytest.raises(RuntimeError,match="expired"):
        client._send_prepared(command(-.9,.9))
    assert [f["type"] for f in frames["left"]]==["arm_joints","emergency_stop"]
    assert [f["type"] for f in frames["right"]]==["emergency_stop"]


def test_background_submits_one_whole_robot_batch(guarded_session,monkeypatch):
    import time
    from types import SimpleNamespace
    from robot_platform.control.background import ThreadedRobotClient
    from robot_platform.transport.wire import unpack_msg
    session,gate,clock=guarded_session
    monkeypatch.setattr(time,"monotonic",lambda:clock[0])
    client=ThreadedRobotClient(session)
    frames=[]
    class Socket:
        def send(self,raw,flags): frames.append(unpack_msg(raw))
    client._client._push={arm:Socket() for arm in session.arms}
    client._client._zmq=SimpleNamespace(NOBLOCK=0)
    monkeypatch.setattr(client._client,"poll",lambda:session.observation())
    client.set_target([-.9,.9])
    client._step()
    assert gate.last_verdict["clear"]
    assert len(frames)==2 and all(f["type"]=="arm_joints" for f in frames)


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_world_revision_cylinder_axis_and_environment_pair_filter(tmp_path):
    from robot_platform.simulation.collision import BoxObstacle
    path=tmp_path/"cylinder.urdf"
    path.write_text('<robot name="cylinder"><link name="base"><collision><geometry>'
                    '<cylinder radius=".05" length="2"/></geometry></collision></link></robot>')
    profile=ModelProfile.from_urdf(path)
    artifact=tmp_path/"collision.json"
    build_runtime_model(profile,artifact)
    clear=RuntimeCollisionModel(profile,artifact)
    world=RuntimeCollisionModel(profile,artifact,environment=[
        BoxObstacle("inside",(.02,.02,.02),(0,0,.8)),
        BoxObstacle("outside",(.02,.02,.02),(0,.8,0)),
        BoxObstacle("inside2",(.02,.02,.02),(0,0,.8))])
    assert world.revision!=clear.revision and world.world_revision!=clear.world_revision
    native=NativeCollisionBackend(world)
    actual=native.proximity({},threshold=1.)
    reference=world.proximity({},threshold=1.)
    assert len(actual["near_pairs"])==3
    assert [p["distance_m"] for p in actual["near_pairs"]]==pytest.approx([p["distance_m"] for p in reference["near_pairs"]],abs=2e-6)
    assert actual["near_pairs"][0]["distance_m"]==0
    assert actual["near_pairs"][1]["distance_m"]==pytest.approx(.74,abs=2e-6)


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_rotating_link_motion_bound_catches_unsampled_obstacle(tmp_path):
    from robot_platform.simulation.collision import BoxObstacle
    from robot_platform.control.collision_guard import check_linear_motion,motion_bounds
    path=tmp_path/"rotor.urdf"
    path.write_text('''<robot name="rotor"><link name="root"/><link name="arm">
    <collision><origin xyz="1 0 0"/><geometry><box size="2 .1 .1"/></geometry></collision></link>
    <joint name="rotate" type="continuous"><parent link="root"/><child link="arm"/>
    <axis xyz="0 0 1"/></joint></robot>''')
    profile=ModelProfile.from_urdf(path); artifact=tmp_path/"collision.json"
    build_runtime_model(profile,artifact)
    runtime=RuntimeCollisionModel(profile,artifact,environment=[BoxObstacle("wall",(.1,.1,.1),(1.5,.4,0))])
    native=NativeCollisionBackend(runtime)
    assert native.proximity({"rotate":0},threshold=.01)["near_pairs"]==[]
    result=check_linear_motion(native,{"rotate":-.5},{"rotate":.5},max_segment_travel_m=10)
    assert not result["clear"] and result["queries"]==1
    assert motion_bounds(runtime.scene,{"rotate":-.5},{"rotate":.5})[0]>=2.


def branched_runtime(tmp_path):
    from robot_platform.simulation.collision import BoxObstacle
    shape = '<collision><geometry><box size=".2 .2 .2"/></geometry></collision>'
    links = ''.join(f'<link name="{n}">{shape}</link>' for n in ('base','a','tip','b'))
    path = tmp_path/'branches.urdf'
    path.write_text(f'''<robot name="branches">{links}
      <joint name="slide" type="prismatic"><parent link="base"/><child link="a"/>
        <axis xyz="1 0 0"/><limit lower="-5" upper="5"/></joint>
      <joint name="coupled" type="prismatic"><parent link="a"/><child link="tip"/>
        <axis xyz="0 1 0"/><limit lower="-5" upper="5"/><mimic joint="slide" multiplier="0.01"/></joint>
      <joint name="other" type="continuous"><parent link="base"/><child link="b"/>
        <origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint></robot>''')
    profile = ModelProfile.from_urdf(path)
    artifact = tmp_path/'bounds.json'
    build_runtime_model(profile,artifact)
    return RuntimeCollisionModel(profile,artifact,environment=[BoxObstacle('wall',(.2,.2,.2),(2,0,0))])


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_scopes_partition_contacts_and_preserve_self_body_environment(tmp_path):
    runtime = branched_runtime(tmp_path)
    backend = NativeCollisionBackend(runtime)
    for threshold in (0., .04):
        for position in (-1., 0., .79, .8, 1., 2.):
            q = {'slide':position,'other':.3}
            full = backend.proximity(q,threshold=threshold)
            scoped_contacts = []
            for scope in ('inter_branch','within_branch','body','environment'):
                result = backend.proximity(q,threshold=threshold,scope=scope)
                reference = runtime.proximity(q,threshold=threshold,scope=scope)
                assert result['near_pairs'] == reference['near_pairs']
                assert result['whole_robot_checked'] is False
                assert result['query_scope'] == scope
                assert result['broad_phase']['eligible_pairs'] == len(runtime.pairs_for_scope(scope))
                assert all(p['kind'] == scope for p in result['near_pairs'])
                scoped_contacts.extend(result['near_pairs'])
            key = lambda p: p['shapes']
            assert sorted(full['near_pairs'],key=key) == sorted(scoped_contacts,key=key)
            assert full['whole_robot_checked'] is True
    q = {'slide':-1.,'other':0.}
    inter = backend.proximity(q,threshold=0.,scope='inter_branch')
    assert inter['near_pairs'] == [] and inter['candidate_pairs'] == 0
    assert inter['broad_phase']['group_rejections'] == 1
    assert inter['broad_phase']['part_aabb_tests'] == 0
    assert backend.proximity(q,threshold=0.,scope='within_branch')['near_pairs']
    with pytest.raises(CollisionError,match='scope'):
        backend.proximity(q,scope='arms_only_typo')


def test_branch_configuration_validates_roots_and_includes_descendants(tmp_path):
    from robot_platform.simulation.collision_groups import CollisionGroups
    runtime = branched_runtime(tmp_path)
    model = runtime.model
    model.profile.collision_branches = {'tool':'slide','second':'other'}
    groups = CollisionGroups(model)
    assert groups.links == {'base':'body','a':'branch:tool','tip':'branch:tool','b':'branch:second'}
    for roots in ({'bad':'missing'}, {'one':'slide','two':'slide'}, {'one':'slide','two':'coupled'}):
        model.profile.collision_branches = roots
        with pytest.raises(CollisionError): CollisionGroups(model)


def test_motion_guard_rejects_partial_scope_result(guarded_session, monkeypatch):
    from robot_platform.control.collision_guard import check_linear_motion
    _, gate, _ = guarded_session
    monkeypatch.setattr(gate.backend,'proximity',lambda *a,**k:dict(
        query_scope='inter_branch', whole_robot_checked=False, near_pairs=[]))
    result = check_linear_motion(gate.backend,{'left':-1.,'right':1.},{'left':-.9,'right':.9})
    assert not result['clear'] and result['reason']=='incomplete_collision_scope'


def finger_runtime(tmp_path):
    path=tmp_path/'fingers.urdf'
    parts=''.join(f'<link name="{n}"><collision><geometry><box size="{size} {size} {size}"/></geometry></collision></link>'
                  for n,size in [('a',.2),('b',.2),('finger_a',.1),('finger_b',.1)])
    joints=''.join(f'<joint name="{n}" type="prismatic"><parent link="root"/><child link="{n}"/>'
                   '<axis xyz="1 0 0"/><limit lower="-3" upper="3"/></joint>' for n in ('a','b'))
    path.write_text(f'''<robot name="fingers"><link name="root"/>{parts}{joints}
      <joint name="jaw" type="prismatic"><parent link="a"/><child link="finger_a"/>
        <origin xyz="1 0 0"/><axis xyz="1 0 0"/><limit lower="-.1" upper=".1"/></joint>
      <joint name="coupled_jaw" type="prismatic"><parent link="b"/><child link="finger_b"/>
        <origin xyz="-1 0 0"/><axis xyz="1 0 0"/><limit lower="-.1" upper=".1"/>
        <mimic joint="jaw" multiplier="-1"/></joint></robot>''')
    profile=ModelProfile.from_urdf(path)
    profile.keepout_ignored_links={'finger_a':'Close interaction','finger_b':'Close interaction'}
    artifact=tmp_path/'bounds.json';build_runtime_model(profile,artifact)
    return RuntimeCollisionModel(profile,artifact)


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_keepout_ignores_fingers_only_and_never_runs_fcl(tmp_path,monkeypatch):
    runtime=finger_runtime(tmp_path)
    cad=NativeCollisionBackend(runtime)
    keepout=NativeCollisionBackend(runtime,geometry_mode='keepout')
    for q in ({'a':-1.,'b':1.,'jaw':0.},{'a':-1.,'b':0.,'jaw':0.}):
        assert cad.proximity(q,threshold=0.)['near_pairs']
        result=keepout.proximity(q,threshold=.01)
        assert not result['near_pairs']
        assert result['excluded_pair_count']==5 and result['eligible_pairs']==1
        assert result['whole_robot_checked'] and not result['full_geometry_coverage']
        assert result['ignored_links']==runtime.keepout_ignored_links
    monkeypatch.setattr(runtime,'refine',lambda *a,**k:pytest.fail('Keep-out mode called FCL'))
    for scope in ('all','inter_branch'):
        result=keepout.proximity({'a':-.05,'b':.05,'jaw':.02},threshold=.01,scope=scope)
        assert len(result['near_pairs'])==1 and result['near_pairs'][0]['shapes']==['a/0','b/0']
        assert result['broad_phase']['fcl_distance_tests']==0 and result['stage_ms']['fine']==0
        assert result['collision_policy_revision']!=cad.policy_revision
    assert keepout.proximity({'a':0.,'b':.215,'jaw':0.},threshold=.01)['near_pairs']==[]
    assert keepout.proximity({'a':0.,'b':.205,'jaw':0.},threshold=.01)['near_pairs']


@pytest.mark.skipif(not library_path().is_file(), reason="Build native backend")
def test_keepout_motion_rejects_crossing_and_allows_excluded_finger_contact(tmp_path):
    from robot_platform.control.collision_guard import check_linear_motion
    runtime=finger_runtime(tmp_path); backend=NativeCollisionBackend(runtime,geometry_mode='keepout')
    assert check_linear_motion(backend,{'a':-1.,'b':1.,'jaw':0.},{'a':-.99,'b':.99,'jaw':.01})['clear']
    partial=check_linear_motion(backend,{'a':-1.,'b':1.,'jaw':0.},{'a':-.99,'b':.99,'jaw':.01},scope='inter_branch')
    assert partial['clear'] and not partial['whole_robot_checked'] and partial['ignored_links']
    result=check_linear_motion(backend,{'a':-1.,'b':-.6,'jaw':0.},{'a':1.,'b':-.6,'jaw':0.},max_segment_travel_m=10)
    assert not result['clear'] and result['reason']=='insufficient_swept_clearance'


def test_keepout_policy_is_validated_and_pins_guard_verdict(guarded_session):
    session,gate,_=guarded_session
    runtime=gate.backend.runtime_model
    gate.backend=NativeCollisionBackend(runtime,geometry_mode='keepout')
    safe=command(-.9,.9)
    assert gate.filter_commands(session,safe)==safe and gate.dispatch_valid(session,safe)
    assert gate.last_verdict['geometry_basis']=='conservative_envelope'
    gate.backend=NativeCollisionBackend(runtime)
    assert not gate.dispatch_valid(session,safe)
    runtime.profile.keepout_ignored_links={'missing':'Typo'}
    with pytest.raises(ValueError,match='existing link names'):
        RuntimeCollisionModel(runtime.profile,runtime.path)


def test_minerva_finger_policy_matches_joint_children_and_keeps_wrist():
    from pathlib import Path
    from robot_platform.models import UrdfModel
    model=UrdfModel(ModelProfile.load(Path(__file__).resolve().parents[2]/'urdf/minerva/model.json'))
    fingers=[j for j in model.joints if j['name'] in (
        'left_gripper_finger_a_slide','left_gripper_finger_b_slide',
        'right_gripper_finger_a_slide','right_gripper_finger_b_slide')]
    assert len(fingers)==4
    assert set(model.profile.keepout_ignored_links)=={j['child'] for j in fingers}
    assert not {j['parent'] for j in fingers} & set(model.profile.keepout_ignored_links)


def test_cached_motion_radii_still_include_changing_prismatic_extension(tmp_path):
    from robot_platform.control.collision_guard import _compile_motion_bounds, motion_bounds
    path=tmp_path/'extending-arm.urdf'
    path.write_text('''<robot name="arm"><link name="root"/><link name="arm"/>
      <link name="tip"><collision><geometry><box size=".2 .2 .2"/></geometry></collision></link>
      <joint name="rotate" type="continuous"><parent link="root"/><child link="arm"/>
        <axis xyz="0 0 1"/></joint>
      <joint name="extend" type="prismatic"><parent link="arm"/><child link="tip"/>
        <axis xyz="1 0 0"/><limit lower="0" upper="5"/></joint></robot>''')
    profile=ModelProfile.from_urdf(path); artifact=tmp_path/'bounds.json'
    build_runtime_model(profile,artifact); runtime=RuntimeCollisionModel(profile,artifact)
    plan=_compile_motion_bounds(runtime.scene,runtime.keepout_motion_corners)
    for extension in (0.,3.,.5,4.):
        travel=motion_bounds(runtime.scene,{'rotate':0.,'extend':extension},
                             {'rotate':1.,'extend':extension},plan=plan)
        assert travel[0]==pytest.approx(np.sqrt(3)*.1+extension)
