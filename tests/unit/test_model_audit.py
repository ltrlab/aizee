import hashlib
import json

import pytest

from robot_platform.simulation.model_audit import audit_urdf


INERTIA = '<inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>'
GEOMETRY = '<geometry><box size="1 1 1"/></geometry>'


def write_robot(tmp_path, body):
    path = tmp_path / "robot.urdf"
    path.write_text(f'<robot name="robot">{body}</robot>')
    return path


def link(name, extra=""):
    return f'<link name="{name}">{extra}</link>'


def joint(name, parent, child, kind="fixed", extra=""):
    return f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>{extra}</joint>'


def codes(report):
    return {f["code"] for f in report["findings"]}


def test_valid_physical_link_and_massless_frame(tmp_path):
    body = link("base") + link("arm", INERTIA + f'<visual>{GEOMETRY}</visual><collision>{GEOMETRY}</collision>')
    body += joint("j1", "base", "arm", "revolute", '<axis xyz="0 0 1"/><limit lower="-1" upper="1"/>')
    path = write_robot(tmp_path, body)
    result = audit_urdf(path, tmp_path, ("j1",))
    assert result["static_checks_passed"]
    assert result["findings"] == []
    assert result["physical_model_validated"] is False
    assert result["summary"]["total_mass_kg"] == 1
    assert result["input_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize("body, expected", [
    (link("a") + link("a"), "invalid_name"),
    (link("a") + link("b"), "disconnected_graph"),
    (link("a") + joint("bad", "a", "missing"), "invalid_link_reference"),
    (link("a") + link("b") + joint("ab", "a", "b") + joint("ba", "b", "a"), "cyclic_graph"),
    (link("a") + link("b") + joint("ab", "a", "b") + joint("ab2", "a", "b"), "multiple_parents"),
])
def test_malformed_graph(tmp_path, body, expected):
    result = audit_urdf(write_robot(tmp_path, body), tmp_path)
    assert expected in codes(result)
    assert not result["static_checks_passed"]


def test_mesh_resolution_is_local_and_contained(tmp_path):
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "valid.stl").write_text("mesh")
    uris = ("package://robot/meshes/valid.stl", "package://robot/../outside.stl",
            "package://robot/meshes/missing.stl", "https://example.com/robot.stl",
            "package://other/meshes/valid.stl", "package://robot/%2e%2e/outside.stl",
            "package://robot/%00.stl", "package://[invalid/mesh.stl")
    visuals = "".join(f'<visual><geometry><mesh filename="{uri}"/></geometry></visual>' for uri in uris)
    result = audit_urdf(write_robot(tmp_path, link("a", visuals + INERTIA)), tmp_path)
    assert {"mesh_path_escape", "missing_mesh", "unresolved_mesh", "unknown_mesh_package"} <= codes(result)
    assert sum(f["code"] == "missing_mesh" for f in result["findings"]) == 1


@pytest.mark.parametrize("tensor", [
    'ixx="-1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"',
    'ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="3"',
    'ixx="nan" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"',
])
def test_invalid_inertia(tmp_path, tensor):
    body = link("a", f'<inertial><mass value="1"/><inertia {tensor}/></inertial>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path)
    assert "invalid_inertia" in codes(result)
    assert not result["static_checks_passed"]


def test_expected_types_extra_dofs_and_bad_joint_numerics(tmp_path):
    body = link("a") + link("b") + link("c")
    body += joint("arm", "a", "b", "prismatic", '<axis xyz="0 0 2"/><limit lower="0" upper="5"/>')
    body += joint("extra", "b", "c", "revolute", '<limit lower="nan" upper="1"/>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path, ("arm", "missing"))
    assert {"wrong_joint_type", "missing_expected_joint", "unexpected_moving_joint",
            "invalid_axis", "invalid_limits", "large_slider_limits"} <= codes(result)


def test_missing_geometry_metadata_and_slider_coupling_are_warnings(tmp_path):
    body = link("a") + link("b", f'<visual>{GEOMETRY}</visual>') + link("c")
    body += joint("finger_a", "a", "b", "prismatic", '<limit lower="0" upper="0.03"/>')
    body += joint("finger_b", "a", "c", "prismatic", '<limit lower="-0.03" upper="0"/>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path, (), ("finger_a", "finger_b"))
    assert {"missing_inertial", "missing_collision", "missing_slider_coupling"} <= codes(result)
    assert all(f["severity"] == "warning" for f in result["findings"])


def test_malformed_xml_reports_input_hash(tmp_path):
    path = tmp_path / "bad.urdf"
    path.write_text("<robot>")
    result = audit_urdf(path, tmp_path)
    assert "unreadable_urdf" in codes(result)
    assert result["input_sha256"]


@pytest.mark.parametrize("mass", ["0", "-1", "nan", "inf"])
def test_invalid_physical_mass(tmp_path, mass):
    body = link("a", INERTIA.replace('value="1"', f'value="{mass}"') + f'<visual>{GEOMETRY}</visual>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path)
    assert "invalid_mass" in codes(result)
    assert not result["static_checks_passed"]
    json.dumps(result, allow_nan=False)


def test_cyclic_mimic_dependencies_fail_without_structural_cycle(tmp_path):
    body = link("a") + link("b") + link("c")
    body += joint("one", "a", "b", "continuous", '<mimic joint="two"/>')
    body += joint("two", "a", "c", "continuous", '<mimic joint="one"/>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path)
    assert "cyclic_mimic" in codes(result)
    assert "cyclic_graph" not in codes(result)


@pytest.mark.parametrize("scale, expected", [("1 0 1", "invalid_mesh_scale"),
                                           ("1 nan 1", "invalid_mesh_scale"),
                                           ("1 1", "invalid_mesh_scale"),
                                           ("-1 1 1", "negative_mesh_scale")])
def test_mesh_scale_and_unhydrated_lfs_pointer(tmp_path, scale, expected):
    (tmp_path / "mesh.stl").write_text("version https://git-lfs.github.com/spec/v1\noid sha256:abcd\nsize 42\n")
    body = link("a", f'<visual><geometry><mesh filename="package://robot/mesh.stl" scale="{scale}"/></geometry></visual>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path)
    assert {expected, "unhydrated_mesh"} <= codes(result)


@pytest.mark.parametrize("limits", ['effort="nan"', 'velocity="-1"', 'effort="inf"'])
def test_optional_joint_limit_bounds_are_numeric(tmp_path, limits):
    body = link("a") + link("b") + joint("j", "a", "b", "continuous", f'<limit {limits}/>')
    result = audit_urdf(write_robot(tmp_path, body), tmp_path)
    assert "invalid_limit_bound" in codes(result)
