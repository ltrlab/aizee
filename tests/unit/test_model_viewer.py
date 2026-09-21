import asyncio
import hashlib
import json
from pathlib import Path
import struct

import pytest

from robot_platform.models import ModelProfile, UrdfModel, discover_models


def profile(tmp_path, mesh=False):
    shape = '<mesh filename="package://parts/shape.stl"/>' if mesh else '<box size=".1 .2 .3"/>'
    (tmp_path / 'machine.urdf').write_text(f'''<robot name="machine">
      <link name="world"/><link name="wheel"><visual><geometry>{shape}</geometry></visual><collision><geometry><box size=".1 .2 .3"/></geometry></collision></link>
      <link name="carriage"/><link name="follower"/>
      <joint name="spin" type="continuous"><parent link="world"/><child link="wheel"/><axis xyz="0 1 0"/><origin xyz=".4 .5 .6" rpy=".1 .2 .3"/></joint>
      <joint name="travel" type="prismatic"><parent link="wheel"/><child link="carriage"/><limit lower="0" upper="1"/><axis xyz="1 0 0"/></joint>
      <joint name="copy" type="prismatic"><parent link="carriage"/><child link="follower"/><limit lower="-.5" upper=".2"/><mimic joint="travel" multiplier="-2" offset=".2"/></joint>
    </robot>''')
    path = tmp_path / 'model.json'
    path.write_text(json.dumps({'schema_version':1,'id':'fixture','title':'Wheel and slide','urdf':'machine.urdf',
                                'packages':{'parts':'.'},'groups':{'Motion':['spin','travel']}}))
    if mesh:
        import trimesh
        trimesh.creation.box().export(tmp_path / 'shape.stl')
    return ModelProfile.load(path)


def test_arbitrary_topology_groups_mimic_and_origins(tmp_path):
    model = UrdfModel(profile(tmp_path))
    assert model.independent == ['spin','travel']
    assert model.mimic_order == ['copy']
    assert model.by_name['travel']['control_bounds'] == pytest.approx([0,.35])
    assert model.positions({'spin':10,'travel':.25})['copy'] == pytest.approx(-.3)
    assert model.by_name['spin']['origin']['rpy'] == [.1,.2,.3]
    assert model.metadata()['geometry'][0]['shape'] == 'box'


def test_chained_coupling_tightens_source_slider_and_reset(tmp_path):
    p=profile(tmp_path)
    extra='''<link name="tip"/><joint name="chain" type="prismatic"><parent link="follower"/><child link="tip"/>
    <limit lower="0" upper=".1"/><mimic joint="copy" multiplier=".5" offset=".1"/></joint>'''
    p.urdf.write_text(p.urdf.read_text().replace('</robot>',extra+'</robot>'))
    model=UrdfModel(p)
    assert model.by_name['travel']['control_bounds'] == pytest.approx([.1,.2])
    assert model.by_name['travel']['default'] == pytest.approx(.1)
    assert model.positions({'spin':0,'travel':.15})['chain'] == pytest.approx(.05)


@pytest.mark.parametrize('q',[{'spin':0}, {'spin':0,'travel':.36}, {'spin':0,'travel':float('nan')},
                            {'spin':True,'travel':0}, {'spin':0,'travel':0,'copy':0}])
def test_invalid_or_incomplete_pose_rejected(tmp_path,q):
    with pytest.raises(ValueError):
        UrdfModel(profile(tmp_path)).positions(q)


def test_external_urdf_needs_no_robot_contract(tmp_path):
    p=profile(tmp_path)
    raw=ModelProfile.from_urdf(p.urdf)
    model=UrdfModel(raw)
    assert model.groups == {'Joints':['spin','travel']}
    assert raw.id.startswith('custom-')


def test_local_package_resolution_and_no_secret_assets(tmp_path):
    p=profile(tmp_path,mesh=True)
    assert p.resolve_mesh(r'package://parts\shape.stl') == tmp_path/'shape.stl'
    for uri in ('https://host/shape.stl','../shape.stl','package://unknown/shape.stl','.env','C:/private.stl'):
        with pytest.raises((ValueError,OSError)):
            p.resolve_mesh(uri)
    (tmp_path/'shape.stl').write_text('version https://git-lfs.github.com/spec/v1\n')
    with pytest.raises(ValueError,match='unhydrated'):
        UrdfModel(p)


@pytest.mark.parametrize('change',[
    {'groups':{'Wrong':['motor_not_in_cad']}}, {'provisional_limits':['missing']},
    {'schema_version':2}, {'urdf':'https://host/robot.urdf'}, {'id':'Invalid name'}])
def test_profile_errors_fail_before_rendering(tmp_path,change):
    profile(tmp_path)
    path=tmp_path/'model.json';data=json.loads(path.read_text());data.update(change);path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        UrdfModel(ModelProfile.load(path))


@pytest.mark.parametrize('before,after',[
    ('type="continuous"','type="floating"'),
    ('joint="travel"','joint="copy"'),
    ('child link="wheel"','child link="missing"'),
    ('axis xyz="0 1 0"','axis xyz="0 2 0"'),
    ('lower="0" upper="1"','lower="2" upper="1"'),
])
def test_invalid_urdf_does_not_silently_drop_structure(tmp_path,before,after):
    p=profile(tmp_path);p.urdf.write_text(p.urdf.read_text().replace(before,after))
    with pytest.raises(ValueError):
        UrdfModel(p)


def test_mesh_conversion_and_explicit_collision_backend(tmp_path):
    pytest.importorskip('fcl')
    from robot_platform.viewer.server import ModelStore
    p=profile(tmp_path,mesh=True);store=ModelStore([p])
    binary=store.mesh(p.id,0)
    vertices,faces=struct.unpack('<II',binary[:8])
    assert vertices>0 and faces==12 and len(binary)==8+12*(vertices+faces)
    result=store.collisions(p.id,{'spin':0,'travel':.2})
    assert result['pairs']==() and result['runtime_interlock_active'] is False


def test_catalog_both_real_robots():
    root=Path(__file__).resolve().parents[2]
    models={p.id:UrdfModel(p) for p in discover_models(root/'urdf')}
    assert len(models['minerva'].independent)==14
    assert len(models['aizee'].independent)==8
    assert len(models['minerva'].mimic_order)==2


def test_display_preserves_dense_mesh_and_small_disconnected_features(tmp_path):
    import numpy as np
    import trimesh
    from robot_platform.viewer.server import ModelStore

    p = profile(tmp_path, mesh=True)
    housing = trimesh.creation.icosphere(subdivisions=5)
    tab = trimesh.creation.box(extents=[.0002, .03, .1])
    tab.apply_translation([2, 0, 0])
    source = trimesh.util.concatenate([housing, tab])
    source.export(tmp_path / 'shape.stl')
    assert len(source.faces) > 12000

    binary = ModelStore([p]).mesh(p.id, 0)
    vertex_count, face_count = struct.unpack('<II', binary[:8])
    vertices = np.frombuffer(binary, dtype='<f4', count=vertex_count*3, offset=8).reshape(-1, 3)
    faces = np.frombuffer(binary, dtype='<u4', offset=8+vertex_count*12).reshape(-1, 3)
    assert face_count == len(source.faces)
    # The actual transmitted triangles, including the tiny separate tab, must
    # match the exported source rather than a visually similar approximation.
    np.testing.assert_array_equal(vertices[faces], source.triangles.astype('<f4'))


def test_http_routes_scoped_assets_post_validation_and_embedded_mount(tmp_path):
    from aiohttp import web
    from aiohttp.test_utils import TestClient,TestServer
    from robot_platform.viewer.server import create_app
    p=profile(tmp_path,mesh=True)
    (tmp_path/'.env').write_text('secret fixture must never be served')
    async def run():
        parent=web.Application();parent.add_subapp('/preview',create_app([p]))
        async with TestClient(TestServer(parent)) as client:
            assert (await client.get('/preview/')).status==200
            assert (await client.get('/preview/static/vendor/three.module.js')).status==200
            result=await (await client.get('/preview/api/models')).json()
            assert result['selected']=='fixture'
            result=await (await client.get('/preview/api/models/fixture')).json()
            assert result['independent']==['spin','travel']
            assert (await client.get('/preview/api/models/fixture/meshes/0')).status==200
            assert (await client.get('/preview/.env')).status==404
            assert (await client.get('/preview/api/models/missing')).status==404
            assert (await client.get('/preview/api/models/fixture/meshes/-1')).status==404
            assert (await client.post('/preview/api/models/fixture/collisions',json={'positions':{}})).status==422
            assert (await client.post('/preview/api/models/fixture/collisions',json={'positions':{}},headers={'Origin':'https://unrelated.test'})).status==403
            assert (await client.get('/preview/api/models',headers={'Host':'unrelated.test'})).status==403
    asyncio.run(run())


def test_shared_profile_audit_and_saved_pose_collision_cli(tmp_path,capsys):
    pytest.importorskip('fcl')
    from robot_platform.simulation.cli import main
    p=profile(tmp_path,mesh=True)
    assert main(['audit-model','--model',str(tmp_path/'model.json')])==0
    assert json.loads(capsys.readouterr().out)['static_checks_passed']
    pose=tmp_path/'pose.json';pose.write_text(json.dumps({'schema_version':1,'model':{'urdf_sha256':hashlib.sha256(p.urdf.read_bytes()).hexdigest()},'positions':{'spin':0,'travel':.2}}))
    assert main(['check-collision','--model',str(tmp_path/'model.json'),'--scene',str(pose)])==0
    data=json.loads(pose.read_text());data['model']['urdf_sha256']='wrong';pose.write_text(json.dumps(data))
    with pytest.raises(SystemExit):
        main(['check-collision','--model',str(tmp_path/'model.json'),'--scene',str(pose)])


def twin_slides(tmp_path):
    # Arbitrary names and no presentation groups: contact detection is topology
    # driven, not dependent on left/right arm naming or a limit-sweep profile.
    shapes = ''.join(f'<collision><origin xyz="{x} 0 0"/><geometry><box size=".2 .2 .2"/></geometry></collision>'
                     for x in (0, 1))
    links = ''.join(f'<link name="{n}">{shapes}</link>' for n in ('carriage', 'tool'))
    joints = ''.join(f'<joint name="{n}_slide" type="prismatic"><parent link="base"/><child link="{n}"/>'
                     '<axis xyz="1 0 0"/><limit lower="-2" upper="2"/></joint>' for n in ('carriage', 'tool'))
    path = tmp_path / 'slides.urdf'
    path.write_text(f'<robot name="slides"><link name="base"/>{links}{joints}</robot>')
    return ModelProfile.from_urdf(path)


def test_viewer_checks_both_branches_and_clears_separated_pose(tmp_path):
    pytest.importorskip('fcl')
    from robot_platform.viewer.server import ModelStore
    p = twin_slides(tmp_path)
    store = ModelStore([p])
    q = {'carriage_slide': 0, 'tool_slide': 0}
    result = store.collisions(p.id, q)
    assert result['pairs'] == (('carriage', 'tool'),)
    assert len(result['contacts']) == 2
    assert not any(c['provisional_interface'] for c in result['contacts'])
    rendered_ids = {g['collision_id'] for g in store.model(p.id).geometry}
    assert all(s['id'] in rendered_ids for c in result['contacts'] for s in c['shapes'])
    assert store.collisions(p.id, {'carriage_slide': -1, 'tool_slide': 1})['contacts'] == []
    with pytest.raises(ValueError, match='every independent joint'):
        store.collisions(p.id, {'carriage_slide': 0})


def test_provisional_interface_classifies_only_exact_shapes_and_remains_in_result(tmp_path):
    pytest.importorskip('fcl')
    from robot_platform.viewer.server import ModelStore
    p = twin_slides(tmp_path)
    store = ModelStore([p])
    # Revision matching is covered by limit-review loader tests. Here exercise
    # classification after loading, including a reversed pair in the report.
    store.model(p.id).limit_review = {'assembly_interfaces': [
        {'shapes': ['tool/0', 'carriage/0'], 'reason': 'Provisional fixture interface.'}]}
    result = store.collisions(p.id, {'carriage_slide': 0, 'tool_slide': 0})
    assert result['pairs'] == (('carriage', 'tool'),)
    assert [c['provisional_interface'] for c in result['contacts']] == [True, False]
    assert result['contacts'][0]['reason'] == 'Provisional fixture interface.'
    assert result['contacts'][1]['reason'] is None
    assert result['runtime_interlock_active'] is False


def test_http_collision_response_contains_shape_contacts_and_invalid_pose_is_not_clear(tmp_path):
    pytest.importorskip('fcl')
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.viewer.server import create_app
    p = twin_slides(tmp_path)
    async def run():
        async with TestClient(TestServer(create_app([p]))) as client:
            url = f'/api/models/{p.id}/collisions'
            response = await client.post(url, json={'positions': {'carriage_slide': 0, 'tool_slide': 0}})
            assert response.status == 200
            assert len((await response.json())['contacts']) == 2
            response = await client.post(url, json={'positions': {'carriage_slide': 0, 'tool_slide': 0}, 'scope':'inter_branch'})
            scoped = await response.json()
            assert len(scoped['contacts']) == 2 and not scoped['whole_robot_checked']
            response = await client.post(url, json={'positions': {'carriage_slide': 0, 'tool_slide': 0}, 'scope':'within_branch'})
            assert (await response.json())['contacts'] == []
            response = await client.post(url, json={'positions': {'carriage_slide': 0, 'tool_slide': 0}, 'scope':'unknown'})
            assert response.status == 422
            response = await client.post(url, json={'positions': {'carriage_slide': 0}})
            assert response.status == 422
            assert 'contacts' not in await response.json()
    asyncio.run(run())


def test_keepout_viewer_excludes_configured_link_without_filtering_cad(tmp_path):
    pytest.importorskip('fcl')
    from robot_platform.simulation.native_collision import library_path
    from robot_platform.simulation.runtime_model import build_runtime_model
    from robot_platform.viewer.server import ModelStore
    if not library_path().is_file(): pytest.skip('Build native backend')
    p=twin_slides(tmp_path);p.keepout_ignored_links={'tool':'Close interaction permitted'}
    p.collision_model=tmp_path/'bounds.json';build_runtime_model(p,p.collision_model)
    store=ModelStore([p]);q={'carriage_slide':0,'tool_slide':0}
    assert store.collisions(p.id,q,'source')['contacts']
    assert store.collisions(p.id,q,'runtime')['contacts']
    result=store.collisions(p.id,q,'keepout',clearance_m=.005)
    assert not result['contacts'] and result['ignored_links']==p.keepout_ignored_links
    assert result['geometry_basis']=='conservative_envelope'
    assert result['threshold_m']==.005
    assert store.runtime_geometry(p.id)['keepout_ignored_links']==p.keepout_ignored_links
    for invalid in (float('nan'),-.1,None,True):
        with pytest.raises(ValueError,match='gap'):
            store.collisions(p.id,q,'keepout',clearance_m=invalid)
