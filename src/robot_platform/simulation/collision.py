"""Offline whole-robot collision checks, not a hardware motion safety interlock.

URDF metres/radians and collision geometry are authoritative. Path checks sample
linear joint interpolation; they do not prove swept-volume clearance. Meshes use
FCL triangle surfaces (including their tessellation limits), not CAD solids.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path, PureWindowsPath
import struct
import threading
from typing import Mapping
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

import numpy as np


class CollisionError(ValueError):
    """A collision answer is unavailable; callers must not treat this as clear."""


@dataclass(frozen=True)
class BoxObstacle:
    """Fixed world-frame box, with size/xyz in metres and roll/pitch/yaw in radians."""

    name: str
    size: tuple[float, float, float]
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PathCollision:
    sample_index: int
    fraction: float
    pairs: tuple[tuple[str, str], ...]


@dataclass(frozen=True)
class PathCheck:
    samples_checked: int
    collisions: tuple[PathCollision, ...]

    @property
    def collision_free(self) -> bool:
        """Only the checked samples are clear; this is not a continuous proof."""
        return not self.collisions


@dataclass
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    limits: tuple[float, float] | None
    mimic: tuple[str, float, float] | None


@dataclass
class _Shape:
    link: str
    geometry: object
    local: np.ndarray
    corners: np.ndarray
    obstacle: bool = False


def _numbers(value, count: int, label: str, *, positive=False) -> np.ndarray:
    try:
        result = np.asarray(value.split() if isinstance(value, str) else value, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CollisionError(f"{label} requires {count} finite numbers") from exc
    if result.shape != (count,) or not np.isfinite(result).all():
        raise CollisionError(f"{label} requires {count} finite numbers")
    if positive and np.any(result <= 0):
        raise CollisionError(f"{label} must be positive")
    return result


def _transform(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
    xyz = _numbers(xyz, 3, "origin xyz")
    roll, pitch, yaw = _numbers(rpy, 3, "origin rpy")
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    result = np.eye(4)
    result[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                      [-sp, cp * sr, cp * cr]]
    result[:3, 3] = xyz
    return result


def _origin(element) -> np.ndarray:
    nodes = element.findall("origin")
    if len(nodes) > 1:
        raise CollisionError("Multiple origins on one element")
    return _transform() if not nodes else _transform(nodes[0].get("xyz", "0 0 0"),
                                                     nodes[0].get("rpy", "0 0 0"))


def _corners(bounds) -> np.ndarray:
    """The eight corners of an axis-aligned box (trimesh.bounds.corners order)."""
    (x0, y0, z0), (x1, y1, z1) = np.asarray(bounds, dtype=float)
    return np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                     [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]])


def _read_asset(path: Path) -> bytes:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise CollisionError(f"Missing or unreadable geometry: {path}") from exc
    if not raw or raw.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise CollisionError(f"Empty or unhydrated Git LFS geometry: {path}")
    return raw


def _contained_path(base: Path, value: str) -> Path:
    try:
        value = unquote(value)
        if not value or "\x00" in value or PureWindowsPath(value).is_absolute():
            raise ValueError("invalid local path")
        path = (base / value).resolve()
        if not path.is_relative_to(base):
            raise ValueError("path escapes its asset directory")
        return path
    except (ValueError, OSError) as exc:
        raise CollisionError(f"Invalid local mesh path: {value!r}") from exc


def _mesh_path(uri: str, source: Path, package_root: Path, package_name: str) -> Path:
    try:
        parsed = urlsplit(uri)
    except ValueError as exc:
        raise CollisionError(f"Invalid mesh URI: {uri!r}") from exc
    if parsed.query or parsed.fragment:
        raise CollisionError(f"Mesh URI must be a local file: {uri}")
    if parsed.scheme == "package" and parsed.netloc == package_name:
        return _contained_path(package_root, parsed.path.lstrip("/"))
    if not parsed.scheme and not parsed.netloc:
        # Plain URDF references are relative to the URDF, not the process cwd.
        return _contained_path(source.parent, parsed.path)
    raise CollisionError(f"Unsupported or unknown mesh package URI: {uri}")


def _validate_gltf(path: Path, raw: bytes) -> None:
    """Reject network/escaping resources before any geometry loader sees them."""
    try:
        if path.suffix.lower() == ".glb":
            magic, version, length = struct.unpack_from("<4sII", raw)
            chunk_length, kind = struct.unpack_from("<I4s", raw, 12)
            if magic != b"glTF" or version != 2 or length != len(raw) or kind != b"JSON":
                raise ValueError("invalid GLB header")
            data = json.loads(raw[20:20 + chunk_length])
        else:
            data = json.loads(raw)
        for item in data.get("buffers", []) + data.get("images", []):
            uri = item.get("uri")
            if uri is None or uri.startswith("data:"):
                continue
            parsed = urlsplit(uri)
            if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment:
                raise ValueError("nonlocal GLTF resource")
            _read_asset(_contained_path(path.parent, parsed.path))
    except (ValueError, TypeError, KeyError, AttributeError, struct.error) as exc:
        raise CollisionError(f"Invalid or nonlocal GLTF resource in {path}: {exc}") from exc


class CollisionScene:
    """A fixed offline scene checked using complete independent joint states.

    Adjacent links are checked. Only same-link shapes and explicitly supplied
    allowed pairs are excluded; no pairs are learned from a home pose. Use fresh,
    synchronized states of *both* arms when adapting this API to an application.
    This module does not obtain state, discover safe joint limits, plan motion,
    stop hardware, or certify an operational safety system.
    """

    @classmethod
    def from_urdf(cls, urdf_path: str | Path, package_root: str | Path | None = None,
                  *, allowed_pairs=(), environment=(), mesh_resolver=None, mesh_bounds=None) -> "CollisionScene":
        """Parse the scene. With `mesh_bounds` ({shape id: [min, max]} lookup) meshes
        are neither read nor built: kinematics and bounds only, no FCL queries."""
        fcl = trimesh = None
        if mesh_bounds is None:
            try:
                import fcl
                import trimesh
            except ImportError as exc:
                raise CollisionError("Collision checks require python-fcl and trimesh") from exc
        scene = cls()
        scene._fcl = fcl
        path = Path(urdf_path).resolve()
        base = Path(package_root).resolve() if package_root is not None else path.parent
        try:
            root = ET.fromstring(path.read_bytes())
        except (OSError, ET.ParseError) as exc:
            raise CollisionError(f"Unreadable URDF: {path}") from exc
        if root.tag != "robot" or not root.get("name"):
            raise CollisionError("URDF needs a named robot root")
        package_name = root.get("name")
        if (base / "package.xml").is_file():
            try:
                package_name = ET.parse(base / "package.xml").getroot().findtext("name")
            except (OSError, ET.ParseError) as exc:
                raise CollisionError("Invalid package.xml") from exc
            if not package_name:
                raise CollisionError("package.xml needs a package name")
        link_elements = root.findall("link")
        names = [link.get("name") for link in link_elements]
        if not names or any(not name or name.startswith("environment:") for name in names) or len(set(names)) != len(names):
            raise CollisionError("Link names must be nonempty, unique and not use environment:")
        scene.link_names = tuple(names)
        scene._joints = {}
        scene._children = {name: [] for name in names}
        child_names = set()
        for element in root.findall("joint"):
            name, kind = element.get("name"), element.get("type")
            if not name or name in scene._joints:
                raise CollisionError("Joint names must be nonempty and unique")
            if kind not in {"fixed", "revolute", "continuous", "prismatic"}:
                raise CollisionError(f"Unsupported joint type {kind!r}: {name}")
            parents, children = element.findall("parent"), element.findall("child")
            if len(parents) != 1 or len(children) != 1:
                raise CollisionError(f"Joint needs one parent and child: {name}")
            parent, child = parents[0].get("link"), children[0].get("link")
            if parent not in scene._children or child not in scene._children or parent == child or child in child_names:
                raise CollisionError(f"Invalid graph reference or multiple parents: {name}")
            child_names.add(child)
            axes = element.findall("axis")
            if len(axes) > 1:
                raise CollisionError(f"Multiple axes: {name}")
            axis = _numbers(axes[0].get("xyz", "") if axes else "1 0 0", 3, f"axis {name}")
            if kind != "fixed" and not np.isclose(np.linalg.norm(axis), 1.0, atol=1e-6):
                raise CollisionError(f"Joint axis must be a unit vector: {name}")
            limits = None
            if kind in {"revolute", "prismatic"}:
                lim = element.findall("limit")
                if len(lim) != 1:
                    raise CollisionError(f"Bounded joint needs limits: {name}")
                low, high = _numbers([lim[0].get("lower"), lim[0].get("upper")], 2, f"limits {name}")
                if low > high:
                    raise CollisionError(f"Reversed limits: {name}")
                limits = (float(low), float(high))
            mimics = element.findall("mimic")
            if len(mimics) > 1 or (mimics and kind == "fixed"):
                raise CollisionError(f"Invalid mimic joint: {name}")
            mimic = None
            if mimics:
                multiplier, offset = _numbers([mimics[0].get("multiplier", "1"),
                                              mimics[0].get("offset", "0")], 2, f"mimic {name}")
                mimic = (mimics[0].get("joint"), float(multiplier), float(offset))
            joint = _Joint(name, kind, parent, child, _origin(element), axis, limits, mimic)
            scene._joints[name] = joint
            scene._children[parent].append(joint)
        roots = set(names) - child_names
        if len(roots) != 1:
            raise CollisionError("URDF must have exactly one root and a connected tree")
        scene.root_link = roots.pop()
        scene._order = []
        visited, pending = set(), [scene.root_link]
        while pending:
            parent = pending.pop()
            if parent in visited:
                raise CollisionError("Cyclic URDF graph")
            visited.add(parent)
            for joint in scene._children[parent]:
                scene._order.append(joint)
                pending.append(joint.child)
        if visited != set(names):
            raise CollisionError("Disconnected or cyclic URDF graph")
        scene.independent_joints = tuple(j.name for j in scene._joints.values()
                                        if j.kind != "fixed" and j.mimic is None)
        scene.joint_units = {name: "m" if scene._joints[name].kind == "prismatic" else "rad"
                             for name in scene.independent_joints}
        for joint in scene._joints.values():
            seen = {joint.name}
            current = joint
            while current.mimic is not None:
                source = current.mimic[0]
                if source not in scene._joints or source in seen:
                    raise CollisionError(f"Missing source or cyclic mimic: {joint.name}")
                other = scene._joints[source]
                if other.kind == "fixed" or (other.kind == "prismatic") != (current.kind == "prismatic"):
                    raise CollisionError(f"Mimic units must match: {joint.name}")
                seen.add(source)
                current = other
        scene._shapes = []
        scene.shape_metadata = []
        shape_counts = {}

        def add_shape(link, geometry, origin, bounds, obstacle=False, source=None):
            ordinal = shape_counts.get(link, 0)
            shape_counts[link] = ordinal + 1
            scene.shape_metadata.append({"id": f"{link}/{ordinal}", "link": link, "source": source})
            scene._shapes.append(_Shape(link, geometry, origin, _corners(bounds), obstacle))

        for link in link_elements:
            name = link.get("name")
            collisions = link.findall("collision")
            if not collisions and (link.find("visual") is not None or link.find("inertial") is not None):
                raise CollisionError(f"Physical link lacks collision geometry: {name}")
            for collision in collisions:
                geometry_elements = collision.findall("geometry")
                if len(geometry_elements) != 1 or len(geometry_elements[0]) != 1:
                    raise CollisionError(f"Collision needs one geometry shape: {name}")
                shape = geometry_elements[0][0]
                transform = _origin(collision)
                if shape.tag == "mesh" and mesh_bounds is not None:
                    geometry = None
                    bounds = np.asarray(mesh_bounds(f"{name}/{shape_counts.get(name, 0)}"), dtype=float)
                elif shape.tag == "mesh":
                    mesh_path = (mesh_resolver(shape.get("filename", "")) if mesh_resolver else
                                 _mesh_path(shape.get("filename", ""), path, base, package_name))
                    raw = _read_asset(mesh_path)
                    extension = mesh_path.suffix.lower()
                    if extension not in {".stl", ".obj", ".ply", ".gltf", ".glb"}:
                        raise CollisionError(f"Unsupported collision mesh format: {mesh_path}")
                    if extension in {".gltf", ".glb"}:
                        _validate_gltf(mesh_path, raw)
                    scale = _numbers(shape.get("scale", "1 1 1"), 3, f"mesh scale {name}", positive=True)
                    try:
                        mesh = trimesh.load_scene(mesh_path, process=False, skip_materials=True,
                                                  allow_remote=False).to_geometry()
                        vertices = np.asarray(mesh.vertices, dtype=float) * scale
                        faces = np.asarray(mesh.faces, dtype=np.int32)
                        if len(vertices) < 3 or len(faces) == 0 or not np.isfinite(vertices).all() or faces.min() < 0 or faces.max() >= len(vertices):
                            raise ValueError("empty or invalid triangle mesh")
                        geometry = fcl.BVHModel()
                        geometry.beginModel(len(faces), len(vertices))
                        geometry.addSubModel(vertices, faces)
                        geometry.endModel()
                        bounds = np.array([vertices.min(axis=0), vertices.max(axis=0)])
                    except Exception as exc:
                        raise CollisionError(f"Cannot load collision mesh {mesh_path}: {exc}") from exc
                elif shape.tag == "box":
                    size = _numbers(shape.get("size", ""), 3, f"box size {name}", positive=True)
                    geometry, bounds = fcl and fcl.Box(*size), np.array([-size / 2, size / 2])
                elif shape.tag in {"sphere", "cylinder"}:
                    radius = _numbers([shape.get("radius")], 1, f"radius {name}", positive=True)[0]
                    if shape.tag == "sphere":
                        geometry, half = fcl and fcl.Sphere(radius), np.full(3, radius)
                    else:
                        length = _numbers([shape.get("length")], 1, f"length {name}", positive=True)[0]
                        geometry, half = fcl and fcl.Cylinder(radius, length), np.array([radius, radius, length / 2])
                    bounds = np.array([-half, half])
                else:
                    raise CollisionError(f"Unsupported collision geometry {shape.tag}: {name}")
                add_shape(name, geometry, transform, bounds, source=shape.get("filename", shape.tag))
        if not scene._shapes:
            raise CollisionError("URDF has no collision geometry")
        environment_names = set()
        for obstacle in environment:
            if not isinstance(obstacle, BoxObstacle) or not isinstance(obstacle.name, str) or not obstacle.name:
                raise CollisionError("Environment requires named BoxObstacle objects")
            name = f"environment:{obstacle.name}"
            if name in environment_names:
                raise CollisionError(f"Duplicate environment obstacle: {name}")
            environment_names.add(name)
            size = _numbers(obstacle.size, 3, f"obstacle size {name}", positive=True)
            add_shape(name, fcl and fcl.Box(*size), _transform(obstacle.xyz, obstacle.rpy),
                      np.array([-size / 2, size / 2]), True)
        scene._allowed = set()
        available = set(names) | environment_names
        for pair in allowed_pairs:
            if (not isinstance(pair, (tuple, list)) or len(pair) != 2
                    or any(not isinstance(n, str) or n not in available for n in pair)
                    or pair[0] == pair[1]):
                raise CollisionError(f"Allowed collision pair must name two distinct scene links: {pair!r}")
            scene._allowed.add(tuple(sorted(pair)))
        scene._shape_indices = {item["id"]: i for i, item in enumerate(scene.shape_metadata)}
        scene._pair_indices = [(i, j) for i, a in enumerate(scene._shapes)
                               for j, b in enumerate(scene._shapes[i+1:], i+1)
                               if a.link != b.link and not (a.obstacle and b.obstacle)
                               and tuple(sorted((a.link, b.link))) not in scene._allowed]
        # Objects belong to shape instances, not shared geometry identities.
        # Constructing an FCL object recomputes the mesh's expensive local AABB.
        scene._objects = [fcl.CollisionObject(shape.geometry) for shape in scene._shapes] if fcl else None
        scene._query_lock = threading.RLock()
        return scene

    def _positions(self, q: Mapping[str, float]) -> dict[str, float]:
        if not isinstance(q, Mapping) or set(q) != set(self.independent_joints):
            raise CollisionError(f"State must contain exactly the independent joints: {self.independent_joints}")
        positions = {name: float(_numbers([q[name]], 1, f"joint position {name}")[0])
                     for name in self.independent_joints}
        pending = [j for j in self._joints.values() if j.mimic is not None]
        while pending:
            for joint in pending[:]:
                source, multiplier, offset = joint.mimic
                if source in positions:
                    positions[joint.name] = positions[source] * multiplier + offset
                    pending.remove(joint)
        for name, value in positions.items():
            limits = self._joints[name].limits
            if not math.isfinite(value) or (limits is not None and not limits[0] <= value <= limits[1]):
                raise CollisionError(f"Joint position outside finite URDF limits: {name}={value}")
        return positions

    def link_transforms(self, q: Mapping[str, float]) -> dict[str, np.ndarray]:
        """World transforms for an explicitly complete independent state."""
        positions = self._positions(q)
        transforms = {self.root_link: np.eye(4)}
        for joint in self._order:
            motion = np.eye(4)
            if joint.kind == "prismatic":
                motion[:3, 3] = joint.axis * positions[joint.name]
            elif joint.kind in {"revolute", "continuous"}:
                angle = positions[joint.name]
                x, y, z = joint.axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                motion[:3, :3] = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)
            transforms[joint.child] = transforms[joint.parent] @ joint.origin @ motion
            if not np.isfinite(transforms[joint.child]).all():
                raise CollisionError(f"Nonfinite forward kinematics: {joint.name}")
        return transforms

    def check_state(self, q: Mapping[str, float]) -> tuple[tuple[str, str], ...]:
        """Return all colliding link pairs; errors raise, never imply clearance."""
        return self._check_pairs(q, self._pair_indices, detailed=False)

    def check_shape_pairs(self, q, pairs=None):
        """Return individual contacting shape ids, optionally for explicit pairs.

        Shape ids are link/ordinal within this exact URDF revision. This permits
        offline investigation of a bearing interface without exempting every
        other part on the same two links. It does not learn allowed contacts.
        """
        indices = self._pair_indices
        if pairs is not None:
            indices = []
            for pair in pairs:
                if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                        or any(not isinstance(n, str) or n not in self._shape_indices for n in pair)
                        or pair[0] == pair[1]):
                    raise CollisionError("Shape pair must name two distinct scene shapes")
                i, j = sorted(self._shape_indices[n] for n in pair)
                a, b = self._shapes[i], self._shapes[j]
                if a.link != b.link and not (a.obstacle and b.obstacle) and tuple(sorted((a.link, b.link))) not in self._allowed:
                    indices.append((i, j))
        return self._check_pairs(q, indices, detailed=True)

    def _check_pairs(self, q, indices, *, detailed):
        # A scene may be called from several viewer workers. Keep transforms and
        # queries in one critical section so results never mix different poses.
        with self._query_lock:
            return self._check_pairs_locked(q, indices, detailed=detailed)

    def _check_pairs_locked(self, q, indices, *, detailed):
        if self._objects is None:
            raise CollisionError("This scene was loaded for kinematics only; CAD contact checks are unavailable")
        transforms = self.link_transforms(q)
        try:
            worlds, lows, highs = [], [], []
            for shape in self._shapes:
                transform = shape.local if shape.obstacle else transforms[shape.link] @ shape.local
                points = shape.corners @ transform[:3, :3].T + transform[:3, 3]
                if not np.isfinite(transform).all() or not np.isfinite(points).all():
                    raise CollisionError(f"Nonfinite collision transform/bounds: {shape.link}")
                worlds.append(transform)
                lows.append(points.min(axis=0))
                highs.append(points.max(axis=0))
            if not indices:
                return ()
            indices = np.asarray(indices, dtype=int).reshape(-1, 2)
            left, right = indices.T
            lows, highs = np.asarray(lows), np.asarray(highs)
            intersects = np.all(highs[left] >= lows[right], axis=1) & np.all(highs[right] >= lows[left], axis=1)
            objects = {}
            def obj(index):
                if index not in objects:
                    t = worlds[index]
                    objects[index] = self._objects[index]
                    # python-fcl setTransform also updates the world AABB.
                    objects[index].setTransform(self._fcl.Transform(t[:3, :3], t[:3, 3]))
                return objects[index]
            pairs = set()
            for i, j in indices[intersects]:
                pair = tuple(sorted((self.shape_metadata[i]["id"], self.shape_metadata[j]["id"]))) if detailed else tuple(sorted((self._shapes[i].link, self._shapes[j].link)))
                if pair in pairs:
                    continue
                result = self._fcl.CollisionResult()
                count = self._fcl.collide(obj(i), obj(j), self._fcl.CollisionRequest(), result)
                if count < 0 or bool(count) != bool(result.is_collision):
                    raise CollisionError(f"FCL could not check {pair}")
                if result.is_collision:
                    pairs.add(pair)
            return tuple(sorted(pairs))
        except Exception as exc:
            raise CollisionError(f"Collision computation failed: {exc}") from exc

    def check_path(self, start: Mapping[str, float], end: Mapping[str, float], *,
                   max_joint_step: float | Mapping[str, float], max_samples: int = 10001) -> PathCheck:
        """Check linear synchronized interpolation, including both endpoints.

        Steps are radians for revolute/continuous joints and metres for sliders.
        A scalar uses that numerical step in each joint's own unit; a complete
        independent-joint mapping permits different steps. Continuous joints use
        the supplied unwrapped coordinates. Mimic coordinates follow their source.
        The caller's requested sample count must fit ``max_samples``; no coarser
        fallback is used. Discrete samples can miss collisions between samples.
        """
        a, b = self._positions(start), self._positions(end)
        if isinstance(max_joint_step, Mapping):
            if set(max_joint_step) != set(self.independent_joints):
                raise CollisionError("Step mapping must contain every independent joint")
            steps = {name: _numbers([max_joint_step[name]], 1, f"step {name}", positive=True)[0]
                     for name in self.independent_joints}
        else:
            step = _numbers([max_joint_step], 1, "max_joint_step", positive=True)[0]
            steps = dict.fromkeys(self.independent_joints, step)
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
            raise CollisionError("max_samples must be an integer of at least two")
        intervals = 1
        for name in self.independent_joints:
            ratio = abs(b[name] - a[name]) / steps[name]
            if not math.isfinite(ratio) or ratio > max_samples - 1:
                raise CollisionError("Requested path resolution exceeds max_samples")
            intervals = max(intervals, math.ceil(ratio))
        collisions = []
        for index in range(intervals + 1):
            fraction = index / intervals
            # Keep exact endpoint values, including joint-limit boundaries.
            state = {name: a[name] if index == 0 else b[name] if index == intervals
                     else (1 - fraction) * a[name] + fraction * b[name]
                     for name in self.independent_joints}
            pairs = self.check_state(state)
            if pairs:
                collisions.append(PathCollision(index, fraction, pairs))
        return PathCheck(intervals + 1, tuple(collisions))
