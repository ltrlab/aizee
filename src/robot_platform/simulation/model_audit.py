"""Static, offline URDF checks; passing does not validate real robot dynamics."""
from __future__ import annotations

from collections import Counter, defaultdict
import hashlib
from pathlib import Path, PureWindowsPath
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import numpy as np


def audit_urdf(
    urdf_path: str | Path,
    package_root: str | Path,
    expected_revolute_joints: tuple[str, ...] = (),
    expected_prismatic_joints: tuple[str, ...] = (),
    mesh_resolver=None,
) -> dict:
    """Audit one URDF and local mesh references without loading meshes or networking.

    ``package_root`` is the directory containing this package's meshes. Its name
    comes from package.xml when present, otherwise the URDF robot name. Expected
    joints describe the complete moving joint set when either tuple is supplied.
    Massless links without visual/collision geometry are allowed as frames.
    """
    path, base = Path(urdf_path).resolve(), Path(package_root).resolve()
    findings: list[dict] = []
    summary = {"links": 0, "joints": 0, "joint_types": {}, "moving_joints": 0,
               "total_mass_kg": 0.0, "visual_links": 0, "collision_links": 0,
               "inertial_links": 0, "mimic_joints": 0, "root_links": []}
    report = {"schema_version": 1, "urdf_path": str(path), "package_root": str(base),
              "input_sha256": None, "robot_name": None, "summary": summary,
              "scope": "Static structure and numeric checks only; geometry, mass scale, "
                       "coordinate calibration, contact and real dynamics remain unvalidated.",
              "physical_model_validated": False, "findings": findings}

    def finding(code, message, *names, severity="error"):
        findings.append({"code": code, "severity": severity, "message": message,
                         "affected_names": list(names)})

    def finish():
        summary["error_count"] = sum(f["severity"] == "error" for f in findings)
        summary["warning_count"] = sum(f["severity"] == "warning" for f in findings)
        report["static_checks_passed"] = summary["error_count"] == 0
        return report

    def values(text, size):
        try:
            result = np.array([float(v) for v in text.split()])
            return result if len(result) == size and np.isfinite(result).all() else None
        except (ValueError, AttributeError, TypeError):
            return None

    try:
        raw = path.read_bytes()
        report["input_sha256"] = hashlib.sha256(raw).hexdigest()
        root = ET.fromstring(raw)
    except (OSError, ET.ParseError) as exc:
        finding("unreadable_urdf", str(exc))
        return finish()
    if root.tag != "robot":
        finding("invalid_root_element", "URDF root must be <robot>.")
        return finish()
    report["robot_name"] = root.get("name", "")
    package_name = report["robot_name"]
    if (base / "package.xml").is_file():
        try:
            package_name = ET.parse(base / "package.xml").getroot().findtext("name")
        except (OSError, ET.ParseError) as exc:
            finding("invalid_package_manifest", str(exc))

    links, joints = root.findall("link"), root.findall("joint")
    link_names = [element.get("name", "") for element in links]
    joint_names = [element.get("name", "") for element in joints]
    for kind, names in (("link", link_names), ("joint", joint_names)):
        for name, count in Counter(names).items():
            if not name or count > 1:
                finding("invalid_name", f"{kind} name is empty or duplicated ({count} occurrences).", name)
    summary.update(links=len(links), joints=len(joints))
    summary["joint_types"] = dict(Counter(j.get("type", "") for j in joints))
    summary["moving_joints"] = sum(j.get("type") != "fixed" for j in joints)

    for link in links:
        name = link.get("name", "")
        visual, collision, inertial = link.find("visual"), link.find("collision"), link.find("inertial")
        physical = visual is not None or collision is not None
        summary["visual_links"] += visual is not None
        summary["collision_links"] += collision is not None
        summary["inertial_links"] += inertial is not None
        if visual is not None and collision is None:
            finding("missing_collision", "Visual link has no collision geometry.", name, severity="warning")
        if inertial is None and physical:
            finding("missing_inertial", "Physical link has no mass/inertia description.", name, severity="warning")
        if inertial is not None:
            mass_el, tensor = inertial.find("mass"), inertial.find("inertia")
            mass = values(mass_el.get("value") if mass_el is not None else None, 1)
            if mass is None or mass[0] < 0 or (physical and mass[0] == 0):
                finding("invalid_mass", "Physical masses must be finite and positive; frame masses may be zero.", name)
            else:
                summary["total_mass_kg"] += float(mass[0])
            parts = values(" ".join(tensor.get(k, "") for k in
                           ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")) if tensor is not None else None, 6)
            if parts is None:
                finding("invalid_inertia", "Inertia requires six finite tensor components.", name)
            else:
                xx, xy, xz, yy, yz, zz = parts
                matrix = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
                # Normalize first so extreme finite inputs cannot overflow eigensolving.
                scale = float(np.max(np.abs(matrix)))
                eig = np.linalg.eigvalsh(matrix / scale if scale else matrix)
                tol = 1e-8
                if eig[0] < -tol or eig[2] > eig[0] + eig[1] + tol:
                    finding("invalid_inertia", "Inertia must be positive semidefinite and obey principal-moment triangle inequalities.", name)
                elif physical and eig[0] <= tol:
                    finding("degenerate_inertia", "Physical link has a zero or nearly degenerate principal inertia.", name, severity="warning")
        for origin in link.findall(".//origin"):
            if any(values(origin.get(key, "0 0 0"), 3) is None for key in ("xyz", "rpy")):
                finding("invalid_origin", "Origin must contain finite xyz/rpy triples.", name)
        for mesh in link.findall(".//mesh"):
            uri = mesh.get("filename", "")
            mesh_scale = values(mesh.get("scale", "1 1 1"), 3)
            if mesh_scale is None or np.any(mesh_scale == 0):
                finding("invalid_mesh_scale", "Mesh scale must contain three finite nonzero values.", name)
            elif np.any(mesh_scale < 0):
                finding("negative_mesh_scale", "Negative mesh scale reflects geometry; verify orientation and collision handling.", name, severity="warning")
            if mesh_resolver is not None:
                try:
                    mesh_resolver(uri)
                except (OSError, ValueError, RuntimeError) as exc:
                    finding("unresolved_mesh", str(exc), name)
                continue
            try:
                parsed = urlsplit(uri)
            except ValueError:
                finding("unresolved_mesh", f"Malformed mesh reference: {uri!r}.", name)
                continue
            if parsed.query or parsed.fragment or "\x00" in unquote(uri):
                finding("unresolved_mesh", f"Mesh reference contains a query, fragment or null byte: {uri!r}.", name)
                continue
            if parsed.scheme == "package":
                if parsed.netloc != package_name:
                    finding("unknown_mesh_package", f"Cannot resolve package in {uri!r} from this package root.", name)
                    continue
                mesh_path = unquote(parsed.path.lstrip("/")).replace("\\", "/")
                candidate = base / mesh_path
            elif parsed.scheme or not uri:
                finding("unresolved_mesh", f"Only package:// or relative local mesh paths are supported: {uri!r}.", name)
                continue
            else:
                mesh_path = unquote(uri).replace("\\", "/")
                candidate = path.parent / mesh_path
            try:
                resolved = candidate.resolve()
            except (OSError, ValueError, RuntimeError):
                finding("unresolved_mesh", f"Invalid local mesh path: {uri!r}.", name)
                continue
            if (PureWindowsPath(mesh_path).is_absolute() or Path(mesh_path).is_absolute()
                    or not resolved.is_relative_to(base)):
                finding("mesh_path_escape", f"Mesh reference escapes the explicit package root: {uri!r}.", name)
            elif not candidate.is_file():
                finding("missing_mesh", f"Local mesh does not exist: {uri!r}.", name)
            else:
                try:
                    with resolved.open("rb") as stream:
                        header = stream.read(64)
                    if header.startswith(b"version https://git-lfs.github.com/spec/v1"):
                        finding("unhydrated_mesh", f"Mesh is a Git LFS pointer, not the asset: {uri!r}.", name)
                except OSError as exc:
                    finding("unresolved_mesh", f"Cannot read mesh {uri!r}: {exc}", name)

    parents, children = defaultdict(list), defaultdict(list)
    mimic = {}
    expected = {name: "revolute" for name in expected_revolute_joints}
    expected.update({name: "prismatic" for name in expected_prismatic_joints})
    by_name = {j.get("name", ""): j for j in joints}
    for name, kind in expected.items():
        if name not in by_name:
            finding("missing_expected_joint", f"Expected {kind} joint is absent.", name)
        elif by_name[name].get("type") != kind:
            finding("wrong_joint_type", f"Expected {kind}, found {by_name[name].get('type')!r}.", name)
    for joint in joints:
        name, kind = joint.get("name", ""), joint.get("type", "")
        if kind not in ("fixed", "revolute", "continuous", "prismatic", "floating", "planar"):
            finding("invalid_joint_type", f"Unknown joint type {kind!r}.", name)
        if expected and kind != "fixed" and name not in expected:
            finding("unexpected_moving_joint", "Moving joint is outside the expected mechanical joint set.", name)
        refs = [joint.find(tag) for tag in ("parent", "child")]
        parent, child = [r.get("link", "") if r is not None else "" for r in refs]
        if parent not in link_names or child not in link_names:
            finding("invalid_link_reference", f"Joint references missing parent/child: {parent!r}, {child!r}.", name)
        else:
            parents[child].append(parent)
            children[parent].append(child)
        axis_el = joint.find("axis")
        if kind in ("revolute", "continuous", "prismatic", "planar"):
            axis = values(axis_el.get("xyz", "1 0 0") if axis_el is not None else "1 0 0", 3)
            if axis is None or not np.isclose(np.linalg.norm(axis), 1.0, rtol=0, atol=1e-6):
                finding("invalid_axis", "Joint axis must be a finite unit vector.", name)
        origin = joint.find("origin")
        if origin is not None and any(values(origin.get(k, "0 0 0"), 3) is None for k in ("xyz", "rpy")):
            finding("invalid_origin", "Joint origin must contain finite xyz/rpy triples.", name)
        if kind in ("revolute", "prismatic"):
            limit = joint.find("limit")
            bounds = values(" ".join(limit.get(k, "") for k in ("lower", "upper")) if limit is not None else None, 2)
            if bounds is None or bounds[0] >= bounds[1]:
                finding("invalid_limits", "Moving joint needs finite lower < upper limits.", name)
            elif kind == "prismatic" and max(abs(bounds[0]), abs(bounds[1]), bounds[1] - bounds[0]) > 1.0:
                finding("large_slider_limits", "Slider bounds/travel exceed 1 metre; verify CAD units and offsets.", name, severity="warning")
        limit = joint.find("limit")
        if limit is not None:
            for key in ("effort", "velocity"):
                if key in limit.attrib:
                    bound = values(limit.get(key), 1)
                    if bound is None or bound[0] < 0:
                        finding("invalid_limit_bound", f"Joint {key} bound must be finite and nonnegative.", name)
        m = joint.find("mimic")
        if m is not None:
            summary["mimic_joints"] += 1
            target = m.get("joint", "")
            mimic[name] = target
            if target not in by_name or target == name:
                finding("invalid_mimic_reference", f"Invalid mimic target {target!r}.", name)
            if values(m.get("multiplier", "1"), 1) is None or values(m.get("offset", "0"), 1) is None:
                finding("invalid_mimic", "Mimic multiplier and offset must be finite.", name)

    roots = sorted(set(link_names) - set(parents))
    summary["root_links"] = roots
    if len(roots) != 1:
        finding("invalid_root_count", f"Expected one root link, found {len(roots)}.", *roots)
    for child, incoming in parents.items():
        if len(incoming) > 1:
            finding("multiple_parents", "Link has multiple incoming joints.", child)
    # Kahn traversal detects cycles in every component without recursion limits.
    indegree = {n: len(parents[n]) for n in set(link_names)}
    queue, visited = list(roots), set()
    while queue:
        node = queue.pop()
        visited.add(node)
        for child in children[node]:
            indegree[child] -= 1
            if indegree[child] == 0:
                queue.append(child)
    if len(visited) != len(set(link_names)):
        finding("cyclic_graph", "Joint graph contains a cycle.", *sorted(set(link_names) - visited))
    reachable, queue = set(), roots[:1]
    while queue:
        node = queue.pop()
        if node not in reachable:
            reachable.add(node)
            queue.extend(children[node])
    if len(reachable) != len(set(link_names)):
        finding("disconnected_graph", "Not all links are reachable from one root.", *sorted(set(link_names) - reachable))
    for start in mimic:
        seen, node = set(), start
        while node in mimic and node not in seen:
            seen.add(node)
            node = mimic[node]
        if node in seen:
            finding("cyclic_mimic", "Mimic dependencies contain a cycle.", start)
    if len(expected_prismatic_joints) > 1 and not any(n in mimic for n in expected_prismatic_joints):
        finding("missing_slider_coupling", "Expected sliders have no mimic relation; verify whether their coupling requires explicit modeling.", *expected_prismatic_joints, severity="warning")
    if not np.isfinite(summary["total_mass_kg"]):
        summary["total_mass_kg"] = None
        finding("invalid_total_mass", "Summed finite masses overflow; verify model scale.")
    return finish()
