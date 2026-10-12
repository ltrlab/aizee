"""The MJCF export keeps URDF frames, meshes, inertials and mimic coupling exactly.

Expectations come from the project's own URDF kinematics (CollisionScene) and from
the source STL bytes, not from a second MuJoCo rendering of the same transforms.
"""
import math
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.models import mjcf
from robot_platform.models.mjcf import build_mjcf, stl_to_obj, write_mjcf
from robot_platform.simulation.collision import CollisionScene
from robot_platform.simulation.mujoco_plant import JointBinding, MujocoPlant

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / 'tests/fixtures/physics'
MINERVA = ROOT / 'urdf/minerva/model.json'
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
