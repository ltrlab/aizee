import hashlib
import json
import math

import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.simulation.limit_sweep import motion_groups, scan_interval, sweep_model


def scan(check, **overrides):
    args = dict(anchor=0., bounds=[-1., 1.], step=.1, tolerance=.001, margin=.02)
    return scan_interval(check, **{**args, **overrides})


def test_first_contacts_refined_and_margin_applied_independently():
    result = scan(lambda q: [('body/0', 'arm/0')] if q <= -.73 or q >= .42 else [])
    assert result['status'] == 'candidate'
    assert result['bounds'] == pytest.approx([-.71, .40], abs=.001)
    assert result['lower']['clear'] > result['lower']['contact']
    assert result['upper']['clear'] < result['upper']['contact']


def test_no_contact_is_search_bound_and_initial_contact_locks_instead_of_fallback():
    result = scan(lambda q: [])
    assert result['bounds'] == [-1, 1]
    assert result['lower']['status'] == result['upper']['status'] == 'search_bound'
    blocked = scan(lambda q: [('a/0','b/0')])
    assert blocked['status'] == 'blocked_at_reference'
    assert blocked['bounds'] == [0, 0]


def test_contact_at_inward_margin_is_unresolved_not_a_clear_interval():
    result = scan(lambda q: [('a/0','b/0')] if .83 < q < .86 or q >= .95 else [],
                  step=.2, margin=.1)
    assert result['status'] == 'unresolved_contact'
    assert result['bounds'] == [0, 0]


@pytest.mark.parametrize('change', [{'step':0}, {'tolerance':0}, {'margin':-1},
                                    {'step':1e-10,'tolerance':1e-11}, {'anchor':math.nan}])
def test_invalid_resolution_fails(change):
    with pytest.raises(ValueError):
        scan(lambda q: [], **change)


def make_model(tmp_path):
    shape='<collision><geometry><box size=".2 .2 .2"/></geometry></collision>'
    body=f'<link name="body">{shape}</link><link name="left">{shape}</link><link name="right">{shape}</link>'
    for side, origin in [('left',.8),('right',-.8)]:
        body+=f'<joint name="{side}" type="prismatic"><parent link="body"/><child link="{side}"/><origin xyz="{origin} 0 0"/><axis xyz="1 0 0"/><limit lower="-1" upper="1"/></joint>'
    (tmp_path/'robot.urdf').write_text(f'<robot name="fixture">{body}</robot>')
    path=tmp_path/'model.json'
    path.write_text(json.dumps({'schema_version':1,'id':'fixture','title':'Fixture','urdf':'robot.urdf'}))
    return ModelProfile.load(path)


def request(p):
    return {'schema_version':1,'urdf_sha256':hashlib.sha256(p.urdf.read_bytes()).hexdigest(),
            'reference_positions':{'left':0.,'right':0.},
            'groups':[{'name':n,'root_joint':n,'joints':[n]} for n in ['left','right']],
            'resolution':{'m':{'step':.1,'tolerance':.001,'margin':.02}},'assembly_interfaces':[]}


def test_independent_branches_include_body_and_exclude_opposite_arm(tmp_path):
    pytest.importorskip('fcl')
    p=make_model(tmp_path)
    first=sweep_model(p,request(p))
    moved=request(p);moved['reference_positions']['right']=.75
    second=sweep_model(p,moved)
    assert first['results']['left']['bounds'] == second['results']['left']['bounds']
    assert first['results']['left']['bounds'][0] == pytest.approx(-.58,abs=.002)
    assert first['results']['left']['bounds'][1] == 1
    assert second['results']['right']['status']=='blocked_at_reference'
    assert first['runtime_interlock_active'] is False


def test_shape_pair_scope_does_not_exempt_other_shapes_on_same_link(tmp_path):
    pytest.importorskip('fcl')
    from robot_platform.simulation.collision import CollisionScene, CollisionError
    p=make_model(tmp_path)
    p.urdf.write_text(p.urdf.read_text().replace('<link name="left">',
        '<link name="left"><collision><origin xyz="-.8 0 0"/><geometry><box size=".2 .2 .2"/></geometry></collision>'))
    scene=CollisionScene.from_urdf(p.urdf)
    assert scene.check_shape_pairs({'left':0,'right':0}, [('body/0','left/0')]) == (('body/0','left/0'),)
    assert scene.check_shape_pairs({'left':0,'right':0}, [('body/0','left/1')]) == ()
    assert scene.check_state({'left':0,'right':0}) == (('body','left'),)
    with pytest.raises(CollisionError):
        scene.check_shape_pairs({'left':0,'right':0}, [('body/0','missing/0')])


def test_mimic_motion_keeps_opposed_jaws_in_collision_query(tmp_path):
    p=make_model(tmp_path)
    text=p.urdf.read_text()
    text=text.replace('<origin xyz="-0.8 0 0"/>','<mimic joint="left" multiplier="-1"/><origin xyz="-0.8 0 0"/>')
    p.urdf.write_text(text)
    groups=motion_groups(UrdfModel(p),'left')
    assert groups['left'] != groups['right']
    assert groups['left'] and groups['right'] and not groups['body']


def test_viewer_applies_revision_bound_report_without_editing_urdf(tmp_path):
    pytest.importorskip('fcl')
    p=make_model(tmp_path)
    report=sweep_model(p,request(p))
    report_path=tmp_path/'review.json';report_path.write_text(json.dumps(report))
    profile_path=tmp_path/'model.json';data=json.loads(profile_path.read_text());data['limit_review']='review.json';profile_path.write_text(json.dumps(data))
    loaded=ModelProfile.load(profile_path)
    model=UrdfModel(loaded)
    assert model.by_name['left']['control_bounds']==report['results']['left']['bounds']
    assert model.by_name['left']['bounds']==[-1,1]
    assert UrdfModel(loaded,apply_review=False).by_name['left']['control_bounds']==[-1,1]
    report['urdf_sha256']='stale';report_path.write_text(json.dumps(report))
    with pytest.raises(ValueError,match='revision'):
        UrdfModel(loaded)
