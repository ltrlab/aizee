"""The MJCF export keeps URDF frames, meshes, inertials and mimic coupling exactly.

Expectations come from the project's own URDF kinematics (CollisionScene) and from
the source STL bytes, not from a second MuJoCo rendering of the same transforms.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.models import mjcf
from robot_platform.models.mjcf import build_mjcf, stl_to_obj, swept_ranges, write_mjcf
from robot_platform.models.urdf import asset_digest
from robot_platform.simulation.collision import CollisionScene
from robot_platform.simulation.mujoco_plant import JointBinding, MujocoPlant

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / 'tests/fixtures/physics'
MINERVA = ROOT / 'urdf/minerva/model.json'
MINERVA_MJCF = ROOT / 'urdf/minerva/mujoco/minerva.xml'
MINERVA_LIMITS = ROOT / 'config/devices/minerva-production-01-joint-limits.json'
# Declared before comparing: frames are copied, so only rounding error is allowed.
FRAME_TOLERANCE = 1e-9
# MuJoCo stores mesh vertices as float32 about the mesh centroid.
VERTEX_TOLERANCE_M = 2e-6


def kinematic_scene(profile):
    # Kinematics only: mesh bounds are never queried by link_transforms.
    return CollisionScene.from_urdf(profile.urdf, profile.package_root, mesh_resolver=profile.resolve_mesh,
                                    mesh_bounds=lambda _: [[-.01] * 3, [.01] * 3])


def poses(model, count, seed):
    """Seeded interior poses plus every joint at each end of its feasible range."""
    rng = np.random.default_rng(seed)
    bounds = {n: model.by_name[n]['control_bounds'] for n in model.independent}
    result = [{n: float(rng.uniform(*bounds[n])) for n in model.independent} for _ in range(count)]
    for side in (0, 1):
        result.append({n: float(bounds[n][side]) for n in model.independent})
    return result


def load(xml, directory):
    path = Path(directory) / 'model.xml'
    path.write_text(xml, encoding='utf-8')
    return mujoco.MjModel.from_xml_path(str(path))


def set_pose(model, data, positions):
    for name, value in positions.items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert joint >= 0, name
        data.qpos[model.jnt_qposadr[joint]] = value
    mujoco.mj_kinematics(model, data)


def assert_link_frames(profile, model, count, seed):
    urdf, scene = UrdfModel(profile, apply_review=False), kinematic_scene(profile)
    data = mujoco.MjData(model)
    assert model.nq == model.nv == len([j for j in urdf.joints if j['type'] != 'fixed'])
    for q in poses(urdf, count, seed):
        set_pose(model, data, urdf.positions(q))
        for link, expected in scene.link_transforms(q).items():
            body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link)
            assert body >= 0, link
            np.testing.assert_allclose(data.xpos[body], expected[:3, 3], rtol=0, atol=FRAME_TOLERANCE)
            np.testing.assert_allclose(data.xmat[body].reshape(3, 3), expected[:3, :3],
                                       rtol=0, atol=FRAME_TOLERANCE)


def test_fixture_frames_inertials_and_limits_match_the_urdf(tmp_path):
    profile = ModelProfile.from_urdf(FIXTURES / 'coupled_jaws.urdf')
    xml, manifest, derived = build_mjcf(profile, tmp_path / 'model.xml')
    model = load(xml, tmp_path)
    assert not derived and manifest['dynamics_usable'] and not manifest['placeholder_inertial_links']
    assert_link_frames(profile, model, 50, seed=3)
    for name, kind, bounds in (('hinge', mujoco.mjtJoint.mjJNT_HINGE, (-2, 2)),
                               ('slide_a', mujoco.mjtJoint.mjJNT_SLIDE, (0, .04)),
                               ('slide_b', mujoco.mjtJoint.mjJNT_SLIDE, (-.03, .01))):
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert model.jnt_type[joint] == kind and model.jnt_limited[joint]
        assert tuple(model.jnt_range[joint]) == pytest.approx(bounds, abs=1e-15)
        assert model.dof_damping[model.jnt_dofadr[joint]] == model.dof_armature[model.jnt_dofadr[joint]] == 0
        assert model.dof_frictionloss[model.jnt_dofadr[joint]] == 0
    # Independent expectation: R_urdf I R_urdf^T about the COM, expressed in the link frame.
    arm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm')
    r = mjcf.rpy_matrix((.3, -.2, .5))
    tensor = np.array([[.004, .0005, -.0003], [.0005, .006, .0002], [-.0003, .0002, .008]])
    principal = np.zeros(9)
    mujoco.mju_quat2Mat(principal, model.body_iquat[arm])
    principal = principal.reshape(3, 3)
    np.testing.assert_allclose(principal @ np.diag(model.body_inertia[arm]) @ principal.T,
                               r @ tensor @ r.T, rtol=0, atol=1e-12)
    np.testing.assert_allclose(model.body_ipos[arm], [.2, .01, -.03], rtol=0, atol=1e-15)
    assert model.body_mass[arm] == .7
    assert model.ngeom == 4 and not model.geom_contype.any() and not model.geom_conaffinity.any()


def test_mimic_becomes_a_joint_equality_that_holds_under_motion(tmp_path):
    profile = ModelProfile.from_urdf(FIXTURES / 'coupled_jaws.urdf')
    xml, manifest, _ = build_mjcf(profile, tmp_path / 'model.xml')
    (tmp_path / 'model.xml').write_text(xml, encoding='utf-8')
    assert manifest['mimic_equalities'] == [
        {'follower': 'slide_b', 'source': 'slide_a', 'multiplier': -.5, 'offset': .01}]
    # Motor angle to jaw metres is a binding (here an arbitrary fixture scale).
    plant = MujocoPlant(tmp_path / 'model.xml', [JointBinding('hinge_motor', 'hinge', 20.),
                        JointBinding('jaw_motor', 'slide_a', 2., scale=.01)],
                        positions=[0., 1.], position_step=0.)
    assert tuple(plant.model.eq_solref[0]) == (2 * plant.model.opt.timestep, 1.)
    follower = plant.model.jnt_qposadr[mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, 'slide_b')]
    leader = plant._qadr[1]
    # Constraint released from a violated start, then driven through the stroke
    # (inside both jaws' ranges, so no limit competes with the coupling).
    worst, travel = 0., []
    for tick in range(800):
        target = 2. + 1.8 * math.sin(tick * .01)
        plant.step([dict(position=0., velocity=0., effort=0., kp=200., kd=5.),
                    dict(position=target, velocity=0., effort=0., kp=.5, kd=.02)], .0025)
        travel.append(plant.data.qpos[leader])
        if tick >= 200:
            worst = max(worst, abs(plant.data.qpos[follower] - (.01 - .5 * plant.data.qpos[leader])))
    assert min(travel) < .005 and max(travel) > .035  # The jaw crossed most of its stroke.
    assert worst < 1e-4


@pytest.mark.parametrize('limit', [10, 12])
def test_oversize_stl_becomes_a_lossless_obj(tmp_path, monkeypatch, limit):
    triangles = np.array([[[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 0], [0, 0, 1], [1, 0, 0]],
                          [[0, 0, 0], [0, 1, 0], [0, 0, 1]], [[1, 0, 0], [0, 0, 1], [0, 1, 0]]] * 3,
                         dtype='<f4') * np.array([.1, .2, .3], dtype='<f4')
    triangles[4:] += np.array([.5, 0, 0], dtype='<f4')
    triangles[8:] += np.array([0, .5, 0], dtype='<f4')
    record = np.zeros(len(triangles), dtype=[('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    record['v'] = triangles
    (tmp_path / 'meshes').mkdir()
    stl = tmp_path / 'meshes/part.stl'
    stl.write_bytes(bytes(80) + len(record).to_bytes(4, 'little') + record.tobytes())
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot name="r"><link name="a"><visual><origin xyz=".1 .2 .3" rpy=".1 .2 .3"/>'
                    '<geometry><mesh filename="meshes/part.stl" scale="2 2 2"/></geometry></visual></link></robot>')
    monkeypatch.setattr(mjcf, 'MUJOCO_STL_MAX_FACES', limit)
    manifest = write_mjcf(ModelProfile.from_urdf(urdf), tmp_path / 'export/model.xml')
    assert [d['derived'] for d in manifest['derived_meshes']] == (['meshes/part.obj'] if limit < 12 else [])
    model = mujoco.MjModel.from_xml_path(str(tmp_path / 'export/model.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    world = data.geom_xpos[0] + model.mesh_vert[:model.mesh_vertnum[0]] @ data.geom_xmat[0].reshape(3, 3).T
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = mjcf.rpy_matrix((.1, .2, .3)), (.1, .2, .3)
    expected = np.unique(triangles.reshape(-1, 3).astype(float) * 2, axis=0) @ transform[:3, :3].T + transform[:3, 3]
    assert model.mesh_facenum[0] == 12 and model.mesh_vertnum[0] == len(expected)
    np.testing.assert_allclose(np.sort(world, axis=0), np.sort(expected, axis=0), rtol=0, atol=VERTEX_TOLERANCE_M)
    if limit < 12:
        content, count = stl_to_obj(stl)
        assert count == 12 and (tmp_path / 'export/meshes/part.obj').read_bytes() == content


def test_export_refuses_to_overwrite(tmp_path):
    profile = ModelProfile.from_urdf(FIXTURES / 'coupled_jaws.urdf')
    write_mjcf(profile, tmp_path / 'model.xml')
    with pytest.raises(FileExistsError):
        write_mjcf(profile, tmp_path / 'model.xml')


def test_minerva_link_frames_match_urdf_kinematics(tmp_path):
    profile = ModelProfile.load(MINERVA)
    xml, manifest, _ = build_mjcf(profile, tmp_path / 'model.xml', meshes=False)
    assert_link_frames(profile, load(xml, tmp_path), 200, seed=20261012)
    assert [m['follower'] for m in manifest['mimic_equalities']] == [
        'right_gripper_finger_b_slide', 'left_gripper_finger_b_slide']


def test_minerva_placeholder_masses_keep_the_model_out_of_dynamics(tmp_path):
    profile = ModelProfile.load(MINERVA)
    xml, manifest, _ = build_mjcf(profile, tmp_path / 'model.xml', meshes=False)
    (tmp_path / 'model.xml').write_text(xml, encoding='utf-8')
    assert not manifest['dynamics_usable'] and len(manifest['placeholder_inertial_links']) == 17
    with pytest.raises(ValueError, match='placeholder mass'):
        MujocoPlant(tmp_path / 'model.xml', [JointBinding('left_arm_j1', 'left_arm_j1', 10.)])


def numeric_tree(element):
    def value(text):
        try:
            return tuple(float(v) for v in text.split())
        except ValueError:
            return text
    return (element.tag, {k: value(v) for k, v in element.attrib.items()}, [numeric_tree(c) for c in element])


def assert_same_tree(actual, expected, path='mujoco'):
    assert actual[0] == expected[0], path
    assert set(actual[1]) == set(expected[1]), path
    for key, value in expected[1].items():
        if isinstance(value, tuple):
            assert actual[1][key] == pytest.approx(value, rel=0, abs=1e-14), f'{path}@{key}'
        else:
            assert actual[1][key] == value, f'{path}@{key}'
    assert len(actual[2]) == len(expected[2]), path
    for a, e in zip(actual[2], expected[2]):
        assert_same_tree(a, e, f"{path}/{e[1].get('name', e[0])}")


def committed_swept(manifest):
    """The swept input the committed export was built from: Helm's direction session is
    local state, so its bindings and confirmed mapping are kept in the manifest."""
    mapping = {joint: {'motor': entry['motor'], 'sign': entry['sign'], 'offset_rad': entry['offset_rad'],
                       'direction_verified': True}
               for joint, entry in manifest['joint_ranges'].items() if entry['source'] == 'helm_swept'}
    directions = {'bindings': manifest['range_source']['directions']['bindings'], 'preview_mapping': mapping}
    return swept_ranges(json.loads(MINERVA_LIMITS.read_text(encoding='utf-8')), directions)


def test_committed_minerva_mjcf_is_current_and_its_assets_match_the_manifest(tmp_path):
    profile = ModelProfile.load(MINERVA)
    manifest = json.loads(MINERVA_MJCF.with_suffix('.manifest.json').read_text(encoding='utf-8'))
    xml, expected, derived = build_mjcf(profile, MINERVA_MJCF, swept=committed_swept(manifest))
    # Text is compared numerically: last-digit libm differences are not drift.
    assert_same_tree(numeric_tree(ET.fromstring(MINERVA_MJCF.read_bytes())), numeric_tree(ET.fromstring(xml)))
    assert {k: v for k, v in manifest.items() if k != 'mjcf_sha256'} == {
        k: v for k, v in expected.items() if k != 'mjcf_sha256'}
    # The manifest describes the committed bytes (stored without line-ending conversion).
    assert hashlib.sha256(MINERVA_MJCF.read_bytes()).hexdigest() == manifest['mjcf_sha256']
    for item in manifest['mesh_assets']:
        assert asset_digest(MINERVA_MJCF.parent / item['file']) == item['sha256']
    for item in manifest['derived_meshes']:
        assert (MINERVA_MJCF.parent / item['derived']).read_bytes() == derived[item['derived']]


def test_committed_minerva_meshes_land_where_the_urdf_puts_them():
    profile = ModelProfile.load(MINERVA)
    urdf, scene = UrdfModel(profile, apply_review=False), kinematic_scene(profile)
    model = mujoco.MjModel.from_xml_path(str(MINERVA_MJCF))
    data = mujoco.MjData(model)
    q = poses(urdf, 1, seed=7)[0]
    set_pose(model, data, urdf.positions(q))
    links = scene.link_transforms(q)
    counts, checked = {}, 0
    for desc in urdf.geometry:
        role, link = desc['role'], desc['link']
        counts[(link, role)] = counts.get((link, role), 0) + 1
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'{link}/{role}/{counts[(link, role)] - 1}')
        assert geom >= 0 and desc['shape'] == 'mesh'
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        world = data.geom_xpos[geom] + model.mesh_vert[start:start + count] @ data.geom_xmat[geom].reshape(3, 3).T
        raw = urdf.assets[desc['asset']].read_bytes()
        faces = int.from_bytes(raw[80:84], 'little')
        corners = np.frombuffer(raw, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]),
                                count=faces, offset=84)['v'].reshape(-1, 3).astype(float)
        source = np.unique(corners, axis=0) * desc['scale']
        transform = links[link] @ np.vstack([np.column_stack([mjcf.rpy_matrix(desc['origin']['rpy']),
                                                             desc['origin']['xyz']]), [0, 0, 0, 1]])
        expected = source @ transform[:3, :3].T + transform[:3, 3]
        # Nothing dropped or merged beyond exact duplicates, and the same placement.
        assert model.mesh_facenum[mesh] == faces and count == len(source), desc['asset']
        np.testing.assert_allclose([world.min(axis=0), world.max(axis=0), world.mean(axis=0)],
                                   [expected.min(axis=0), expected.max(axis=0), expected.mean(axis=0)],
                                   rtol=0, atol=VERTEX_TOLERANCE_M)
        checked += 1
    assert checked == model.ngeom == 220


def fixture_swept(sign=-1, verified=True, reference='zero-a', model=None):
    urdf = UrdfModel(ModelProfile.from_urdf(FIXTURES / 'coupled_jaws.urdf'), apply_review=False)
    limits = {'kind': 'joint_limit_calibration', 'units': 'motor_rad_from_mechanical_zero',
              'reference_sha256': 'zero-a', 'model_revision': model or urdf.sha256,
              'joints': {'hinge_motor': {'min_rad': -.4, 'max_rad': 1.25}}}
    directions = {'bindings': {'reference': reference, 'model': model or urdf.sha256},
                  'preview_mapping': {'hinge': {'motor': 'hinge_motor', 'sign': sign, 'offset_rad': .1,
                                                'direction_verified': verified}}}
    return limits, directions


@pytest.mark.parametrize('sign, expected', [(1, (-.3, 1.35)), (-1, (-1.15, .5))])
def test_swept_motor_limits_become_model_ranges_through_the_confirmed_direction(tmp_path, sign, expected):
    swept = swept_ranges(*fixture_swept(sign))
    xml, manifest, _ = build_mjcf(ModelProfile.from_urdf(FIXTURES / 'coupled_jaws.urdf'), tmp_path / 'model.xml',
                                  swept=swept)
    model = load(xml, tmp_path)
    hinge = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'hinge')
    # model = sign * motor + offset; a reversed joint swaps its ends.
    assert tuple(model.jnt_range[hinge]) == pytest.approx(expected, abs=1e-15)
    entry = dict(manifest['joint_ranges']['hinge'])
    assert entry.pop('range') == pytest.approx(expected, abs=1e-15)
    assert entry == {'source': 'helm_swept', 'motor': 'hinge_motor', 'sign': sign, 'offset_rad': .1,
                     'swept_motor_rad': [-.4, 1.25]}
    slide = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'slide_a')
    assert tuple(model.jnt_range[slide]) == (0, .04) and manifest['joint_ranges']['slide_a']['source'] == 'urdf'


@pytest.mark.parametrize('change, message', [
    (dict(verified=False), 'not confirmed'),
    (dict(reference='zero-b'), 'different mechanical zeros'),
    (dict(model='another-urdf'), 'another URDF revision'),
])
def test_swept_ranges_refuse_unconfirmed_or_mismatched_calibration(tmp_path, change, message):
    with pytest.raises(ValueError, match=message):
        swept = swept_ranges(*fixture_swept(**change))
        build_mjcf(ModelProfile.from_urdf(FIXTURES / 'coupled_jaws.urdf'), tmp_path / 'model.xml', swept=swept)


def test_committed_minerva_arm_ranges_are_helms_swept_travel():
    manifest = json.loads(MINERVA_MJCF.with_suffix('.manifest.json').read_text(encoding='utf-8'))
    limits = json.loads(MINERVA_LIMITS.read_text(encoding='utf-8'))
    model = mujoco.MjModel.from_xml_path(str(MINERVA_MJCF))
    urdf = UrdfModel(ModelProfile.load(MINERVA), apply_review=False)
    swept = {j: e for j, e in manifest['joint_ranges'].items() if e['source'] == 'helm_swept'}
    assert sorted(swept) == sorted(f'{side}_arm_j{i}' for side in ('left', 'right') for i in range(1, 7))
    assert manifest['range_source']['limits']['reference_sha256'] == limits['reference_sha256']
    for joint, entry in manifest['joint_ranges'].items():
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if entry['source'] == 'helm_swept':
            saved = limits['joints'][entry['motor']]
            assert entry['swept_motor_rad'] == [saved['min_rad'], saved['max_rad']]
            ends = sorted(entry['sign'] * q + entry['offset_rad'] for q in entry['swept_motor_rad'])
        else:  # The jaws keep their CAD stroke: their motor-to-jaw scale is unverified.
            ends = urdf.by_name[joint]['bounds']
        assert tuple(model.jnt_range[index]) == pytest.approx(ends, abs=1e-12), joint
