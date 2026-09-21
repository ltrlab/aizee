import math

import numpy as np
import pytest

pytest.importorskip("fcl")
trimesh = pytest.importorskip("trimesh")

from robot_platform.simulation.collision import BoxObstacle, CollisionError, CollisionScene


def robot(tmp_path, body):
    path = tmp_path / "robot.urdf"
    path.write_text(f'<robot name="robot">{body}</robot>')
    return path


def link(name, shape=None, origin="", extra=""):
    collision = "" if shape is None else f'<collision>{origin}<geometry>{shape}</geometry></collision>'
    return f'<link name="{name}">{collision}{extra}</link>'


def joint(name, parent, child, kind="fixed", origin="", axis="1 0 0", limits="", mimic=""):
    return (f'<joint name="{name}" type="{kind}"><parent link="{parent}"/>'
            f'<child link="{child}"/>{origin}<axis xyz="{axis}"/>{limits}{mimic}</joint>')


BOX = '<box size="0.2 0.2 0.2"/>'
LIMITS = '<limit lower="-3" upper="3"/>'


def slider_scene(tmp_path, **kwargs):
    body = link("base", BOX) + link("frame") + link("moving", BOX)
    body += joint("frame_joint", "base", "frame")
    body += joint("slide", "frame", "moving", "prismatic", limits=LIMITS)
    return CollisionScene.from_urdf(robot(tmp_path, body), **kwargs)


def test_nonzero_fk_preserves_joint_origins_rotation_and_mimic(tmp_path):
    body = link("base", BOX) + link("arm", BOX) + link("jaw", BOX) + link("opposite", BOX)
    body += joint("turn", "base", "arm", "continuous", axis="0 0 1",
                  origin=f'<origin xyz="1 0 0" rpy="0 0 {math.pi / 2}"/>')
    body += joint("slide", "arm", "jaw", "prismatic", limits=LIMITS,
                  origin='<origin xyz="2 0 0"/>')
    body += joint("follower", "base", "opposite", "prismatic", limits=LIMITS,
                  mimic='<mimic joint="slide" multiplier="-2" offset="1"/>')
    scene = CollisionScene.from_urdf(robot(tmp_path, body))
    assert scene.independent_joints == ("turn", "slide")
    assert scene.joint_units == {"turn": "rad", "slide": "m"}
    frames = scene.link_transforms({"turn": math.pi / 2, "slide": 0.25})
    np.testing.assert_allclose(frames["arm"][:3, 3], [1, 0, 0], atol=1e-14)
    np.testing.assert_allclose(frames["jaw"][:3, 3], [-1.25, 0, 0], atol=1e-14)
    np.testing.assert_allclose(frames["opposite"][:3, 3], [0.5, 0, 0], atol=1e-14)


def test_nonadjacent_self_collision_and_environment(tmp_path):
    scene = slider_scene(tmp_path, environment=[BoxObstacle("wall", (0.2, 0.2, 0.2), (2, 0, 0))])
    assert scene.check_state({"slide": -1}) == ()
    assert scene.check_state({"slide": 0}) == (("base", "moving"),)
    assert scene.check_state({"slide": 2}) == (("environment:wall", "moving"),)


def test_adjacent_links_are_checked_and_only_explicit_pairs_are_allowed(tmp_path):
    body = link("a", BOX, extra=f'<collision><geometry>{BOX}</geometry></collision>')
    body += link("b", BOX) + joint("fixed", "a", "b")
    path = robot(tmp_path, body)
    assert CollisionScene.from_urdf(path).check_state({}) == (("a", "b"),)
    assert CollisionScene.from_urdf(path, allowed_pairs=[("b", "a")]).check_state({}) == ()
    for pair in [("a", "missing"), ("a", "a"), "ab", 1, ([], "a")]:
        with pytest.raises(CollisionError, match="Allowed collision pair"):
            CollisionScene.from_urdf(path, allowed_pairs=[pair])


def test_path_checks_endpoints_and_detects_interior_collision(tmp_path):
    scene = slider_scene(tmp_path)
    result = scene.check_path({"slide": -1}, {"slide": 1}, max_joint_step={"slide": 0.1})
    assert result.samples_checked == 21
    assert not result.collision_free
    assert any(sample.fraction == 0.5 and sample.pairs == (("base", "moving"),)
               for sample in result.collisions)
    assert all(0 < sample.sample_index < 20 for sample in result.collisions)
    endpoint = scene.check_path({"slide": 0}, {"slide": 1}, max_joint_step=2)
    assert endpoint.samples_checked == 2
    assert endpoint.collisions[0].sample_index == 0
    assert scene.check_path({"slide": -2}, {"slide": -1}, max_joint_step=0.2).collision_free


def test_two_arms_move_synchronously_and_other_arm_cannot_be_omitted(tmp_path):
    body = link("root") + link("left", BOX) + link("right", BOX)
    body += joint("left_slide", "root", "left", "prismatic", limits=LIMITS)
    body += joint("right_slide", "root", "right", "prismatic", limits=LIMITS)
    scene = CollisionScene.from_urdf(robot(tmp_path, body))
    start, end = {"left_slide": -1, "right_slide": 1}, {"left_slide": 1, "right_slide": -1}
    assert scene.check_state(start) == scene.check_state(end) == ()
    result = scene.check_path(start, end, max_joint_step=0.1)
    assert any(hit.fraction == 0.5 and hit.pairs == (("left", "right"),) for hit in result.collisions)
    with pytest.raises(CollisionError, match="exactly"):
        scene.check_state({"left_slide": -1})


@pytest.mark.parametrize("state", [{}, {"slide": 0, "other_arm": 0}, {"slide": float("nan")},
                                   {"slide": float("inf")}, {"slide": 3.01}])
def test_incomplete_unknown_nonfinite_or_out_of_limits_state_fails(tmp_path, state):
    with pytest.raises(CollisionError):
        slider_scene(tmp_path).check_state(state)


@pytest.mark.parametrize("step,budget", [(0, 10), (-1, 10), (float("nan"), 10),
                                        ({}, 10), (0.001, 100), (1, 1), (1, True)])
def test_invalid_or_unbounded_path_request_fails(tmp_path, step, budget):
    with pytest.raises(CollisionError):
        slider_scene(tmp_path).check_path({"slide": -1}, {"slide": 1},
                                         max_joint_step=step, max_samples=budget)


def test_mimic_limits_checked_and_dependent_inputs_rejected(tmp_path):
    body = link("a", BOX) + link("b", BOX) + link("c", BOX)
    body += joint("source", "a", "b", "prismatic", limits=LIMITS)
    body += joint("follower", "a", "c", "prismatic", limits='<limit lower="-1" upper="1"/>',
                  mimic='<mimic joint="source" multiplier="2"/>')
    scene = CollisionScene.from_urdf(robot(tmp_path, body))
    with pytest.raises(CollisionError, match="limits: follower"):
        scene.check_state({"source": 0.6})
    with pytest.raises(CollisionError, match="exactly"):
        scene.check_state({"source": 0, "follower": 0})


@pytest.mark.parametrize("body", [
    link("a", BOX) + link("b", BOX),
    link("a", BOX) + link("a", BOX),
    link("a", BOX) + joint("bad", "a", "missing"),
    link("a", BOX) + link("b", BOX) + joint("ab", "a", "b") + joint("ba", "b", "a"),
    link("a", BOX) + link("b", BOX) + joint("ab", "a", "b") + joint("ab2", "a", "b"),
    link("a", BOX) + link("b", BOX) + joint("planar", "a", "b", "planar"),
    link("a", BOX) + link("b", BOX) + joint("slide", "a", "b", "prismatic"),
    link("a", BOX) + link("b", BOX) + joint("slide", "a", "b", "prismatic", axis="0 0 0", limits=LIMITS),
])
def test_malformed_graph_or_joint_fails(tmp_path, body):
    with pytest.raises(CollisionError):
        CollisionScene.from_urdf(robot(tmp_path, body))


@pytest.mark.parametrize("mimic_a,mimic_b", [("b", "a"), ("missing", "a")])
def test_mimic_cycle_and_missing_source_fail(tmp_path, mimic_a, mimic_b):
    body = link("root", BOX) + link("la", BOX) + link("lb", BOX)
    body += joint("a", "root", "la", "prismatic", limits=LIMITS,
                  mimic=f'<mimic joint="{mimic_a}"/>')
    body += joint("b", "root", "lb", "prismatic", limits=LIMITS,
                  mimic=f'<mimic joint="{mimic_b}"/>')
    with pytest.raises(CollisionError, match="mimic"):
        CollisionScene.from_urdf(robot(tmp_path, body))


@pytest.mark.parametrize("shape", [None, '<mesh filename="missing.stl"/>', '<box size="0 1 1"/>',
                                  '<sphere radius="nan"/>', '<capsule radius="1" length="1"/>'])
def test_missing_or_invalid_geometry_fails(tmp_path, shape):
    extra = '<visual><geometry><box size="1 1 1"/></geometry></visual>'
    with pytest.raises(CollisionError):
        CollisionScene.from_urdf(robot(tmp_path, link("base", shape, extra=extra)))


def test_lfs_pointer_is_rejected(tmp_path):
    (tmp_path / "cube.stl").write_text("version https://git-lfs.github.com/spec/v1\noid sha256:abc\nsize 200\n")
    with pytest.raises(CollisionError, match="unhydrated"):
        CollisionScene.from_urdf(robot(tmp_path, link("base", '<mesh filename="cube.stl"/>')))


def test_mesh_scale_origin_and_package_resolution(tmp_path):
    trimesh.creation.box().export(tmp_path / "cube.stl")
    body = link("base", '<mesh filename="package://robot/cube.stl" scale="2 1 1"/>',
                origin=f'<origin xyz="0 1 0" rpy="0 0 {math.pi / 2}"/>')
    scene = CollisionScene.from_urdf(robot(tmp_path, body), package_root=tmp_path,
        environment=[BoxObstacle("touching", (0.2, 0.2, 0.2), (0, 2, 0)),
                     BoxObstacle("clear", (0.2, 0.2, 0.2), (1, 1, 0))])
    assert scene.check_state({}) == (("base", "environment:touching"),)


def test_native_sphere_cylinder_and_rotated_environment(tmp_path):
    body = link("base", '<cylinder radius="0.1" length="2"/>',
                origin=f'<origin rpy="0 {math.pi / 2} 0"/>')
    body += link("ball", '<sphere radius="0.2"/>')
    body += joint("slide", "base", "ball", "prismatic", limits=LIMITS)
    scene = CollisionScene.from_urdf(robot(tmp_path, body),
        environment=[BoxObstacle("wall", (2, 0.2, 0.2), (2, 0, 0), (0, 0, math.pi / 2))])
    assert scene.check_state({"slide": 1.1}) == (("ball", "base"),)
    assert scene.check_state({"slide": 2}) == (("ball", "environment:wall"),)


def test_remote_gltf_resource_rejected_before_loading(tmp_path):
    (tmp_path / "remote.gltf").write_text('{"asset":{"version":"2.0"},"buffers":[{"uri":"https://example.com/private.bin"}]}')
    with pytest.raises(CollisionError, match="nonlocal GLTF"):
        CollisionScene.from_urdf(robot(tmp_path, link("base", '<mesh filename="remote.gltf"/>')))


def test_backend_failure_is_not_clearance(tmp_path, monkeypatch):
    scene = slider_scene(tmp_path)
    def fail(*args):
        raise RuntimeError("backend failure")
    monkeypatch.setattr(scene._fcl, "collide", fail)
    with pytest.raises(CollisionError, match="backend failure"):
        scene.check_state({"slide": 0})


def test_cached_objects_survive_alternating_and_concurrent_poses(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    scene = slider_scene(tmp_path)
    identities = tuple(map(id, scene._objects))
    def no_construction(*args):
        raise AssertionError("Collision objects must be reused")
    monkeypatch.setattr(scene._fcl, "CollisionObject", no_construction)
    values = [0., -1., 1., 0., 2.] * 20
    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(executor.map(lambda q: scene.check_state({"slide": q}), values))
    assert results == [(("base", "moving"),) if q == 0 else () for q in values]
    assert tuple(map(id, scene._objects)) == identities
