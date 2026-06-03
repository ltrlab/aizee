"""URDF mesh collision world.

Parses the full URDF (all links + all joints, not just the IK chain),
loads each link's `<visual><mesh>` STLs via trimesh, and wraps them in a
trimesh + python-fcl `CollisionManager` keyed by link name.

Use:
    world = MeshWorld.from_urdf("urdf/aizee/aizee.urdf")
    world.set_qpos({"swivel": 1.0, "gantry_base": -0.5, ...})
    if world.in_collision():
        print(world.colliding_pairs())

The collision check filters out:
  * Self-pairs (handled by CollisionManager)
  * Adjacent links (parent + child of the SAME joint — they're always
    touching at the joint axis and would always report a collision)
  * Optional: a user-supplied disable list for pairs that are physically
    near each other in CAD and produce spurious collisions

Mesh complexity is reduced via per-link convex hulls by default (fast,
slight over-approximation).  Pass `use_convex=False` to use the raw STL
geometry instead — more accurate, ~10-50x slower for our mesh sizes.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import trimesh
    import trimesh.collision  # noqa: F401  (registers CollisionManager)
    _TRIMESH_AVAILABLE = True
except ImportError:
    _TRIMESH_AVAILABLE = False


# -----------------------------------------------------------------------------
# SO(3) helpers — duplicated from ik/kinematics.py to keep this module
# importable in isolation (no circular dep on the IK Kinematics class).
# -----------------------------------------------------------------------------

def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle_to_R(axis: np.ndarray, theta: float) -> np.ndarray:
    a = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = np.cos(theta), np.sin(theta)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + (1.0 - c) * np.outer(a, a) + s * K


def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


# -----------------------------------------------------------------------------
# URDF parsing — links + joints (full tree, not just IK chain)
# -----------------------------------------------------------------------------

@dataclass
class _Visual:
    mesh_path: str          # absolute filesystem path (resolved)
    R_origin: np.ndarray    # 3x3
    t_origin: np.ndarray    # 3,
    scale: np.ndarray = field(default_factory=lambda: np.ones(3))


@dataclass
class _Link:
    name: str
    visuals: list[_Visual]


@dataclass
class _Joint:
    name: str
    jtype: str
    parent: str
    child: str
    R_static: np.ndarray
    t_static: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def _resolve_mesh(filename: str, urdf_dir: Path) -> Optional[Path]:
    """Convert a URDF mesh filename to a real filesystem path.

    Handles `package://meshes\\foo.stl` (Windows-style backslashes from
    onshape-to-robot) and absolute / relative paths.
    """
    raw = filename.replace("\\", "/")
    if raw.startswith("package://"):
        # Strip 'package://' and the package name; rest is the file.
        without = raw[len("package://"):]
        parts = without.split("/", 1)
        if len(parts) == 2:
            pkg, rel = parts
            # We follow the onshape-to-robot convention: `meshes` package is
            # the local `meshes/` dir next to the URDF.
            cand = urdf_dir / pkg / rel
            if cand.exists():
                return cand
        return None
    p = Path(raw)
    if not p.is_absolute():
        p = urdf_dir / p
    return p if p.exists() else None


def _parse_urdf_full(urdf_path: Path) -> tuple[dict[str, _Link], dict[str, _Joint]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    urdf_dir = urdf_path.parent

    links: dict[str, _Link] = {}
    for link_el in root.findall("link"):
        name = link_el.get("name", "")
        visuals: list[_Visual] = []
        for v in link_el.findall("visual"):
            origin = v.find("origin")
            if origin is not None:
                xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            else:
                xyz = np.zeros(3); rpy = np.zeros(3)
            geom = v.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None:
                continue
            fn = mesh.get("filename", "")
            resolved = _resolve_mesh(fn, urdf_dir)
            if resolved is None:
                continue
            scale_attr = mesh.get("scale")
            scale = np.fromstring(scale_attr, sep=" ") if scale_attr else np.ones(3)
            visuals.append(_Visual(
                mesh_path=str(resolved),
                R_origin=_rpy_to_R(rpy),
                t_origin=xyz.copy(),
                scale=scale,
            ))
        links[name] = _Link(name=name, visuals=visuals)

    joints: dict[str, _Joint] = {}
    for j in root.findall("joint"):
        name = j.get("name", "")
        jtype = j.get("type", "fixed")
        parent = j.find("parent").get("link", "") if j.find("parent") is not None else ""
        child = j.find("child").get("link", "") if j.find("child") is not None else ""
        origin = j.find("origin")
        if origin is not None:
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        else:
            xyz = np.zeros(3); rpy = np.zeros(3)
        ax_el = j.find("axis")
        axis = (np.fromstring(ax_el.get("xyz", "0 0 1"), sep=" ")
                if ax_el is not None else np.array([0, 0, 1.0]))
        limit = j.find("limit")
        lower = float(limit.get("lower", "-3.14159265358979")) if limit is not None else -np.pi
        upper = float(limit.get("upper", "3.14159265358979")) if limit is not None else  np.pi
        joints[name] = _Joint(
            name=name, jtype=jtype, parent=parent, child=child,
            R_static=_rpy_to_R(rpy), t_static=xyz.copy(),
            axis=axis, lower=lower, upper=upper,
        )
    return links, joints


# -----------------------------------------------------------------------------
# MeshWorld
# -----------------------------------------------------------------------------

class MeshWorld:
    """Full-tree URDF + mesh + collision wrapper.

    Holds:
      * Per-link concatenated trimesh.Trimesh (with the URDF visual origins
        baked in, so we just transform by the link's world frame at query time)
      * A trimesh.collision.CollisionManager with one object per link
      * The joint tree for FK
      * A set of "always-allowed" link pairs whose collisions are ignored

    Heavy at construction (mesh load + concat + convex-hull); cheap at query
    time (set_transform + in_collision_internal).
    """

    def __init__(
        self,
        urdf_path: Path | str,
        *,
        use_convex: bool = True,
        extra_allowed_pairs: Optional[list[tuple[str, str]]] = None,
        auto_allow_home_pairs: bool = True,
    ) -> None:
        if not _TRIMESH_AVAILABLE:
            raise RuntimeError(
                "trimesh + python-fcl are required — "
                "install via: pip install trimesh python-fcl"
            )
        self.urdf_path = Path(urdf_path)
        self._use_convex = use_convex
        self._links, self._joints = _parse_urdf_full(self.urdf_path)

        # Build the parent->children adjacency for FK + the inverse for the
        # adjacency-allowed-pairs filter.
        self._children_by_parent: dict[str, list[str]] = {}
        for j in self._joints.values():
            self._children_by_parent.setdefault(j.parent, []).append(j.child)
        all_children = {j.child for j in self._joints.values()}
        roots = [n for n in self._links if n not in all_children]
        if len(roots) != 1:
            raise ValueError(f"URDF must have exactly one root link; found {roots}")
        self.root_link: str = roots[0]

        # Load & concatenate visuals per link, optionally take convex hull.
        self._link_meshes: dict[str, "trimesh.Trimesh"] = {}
        for name, link in self._links.items():
            if not link.visuals:
                continue
            parts: list["trimesh.Trimesh"] = []
            for v in link.visuals:
                try:
                    m = trimesh.load(v.mesh_path, force="mesh")
                except Exception:
                    continue
                if not isinstance(m, trimesh.Trimesh) or len(m.vertices) == 0:
                    continue
                if not np.allclose(v.scale, 1.0):
                    m = m.copy()
                    m.apply_scale(v.scale)
                T = _T(v.R_origin, v.t_origin)
                if not np.allclose(T, np.eye(4)):
                    m = m.copy()
                    m.apply_transform(T)
                parts.append(m)
            if not parts:
                continue
            merged = trimesh.util.concatenate(parts)
            if use_convex:
                try:
                    merged = merged.convex_hull
                except Exception:
                    pass  # fall back to raw concat
            self._link_meshes[name] = merged

        # Build collision manager — one object per link, identity transform
        # initially.  Caller MUST call set_qpos before in_collision().
        self._mgr = trimesh.collision.CollisionManager()
        for name, m in self._link_meshes.items():
            self._mgr.add_object(name, m, transform=np.eye(4))

        # Adjacency-allowed pairs: skip parent-child of every joint, plus
        # any extras the caller wants (e.g. links that overlap slightly in
        # CAD and shouldn't be flagged).  Stored as frozenset for O(1) lookup.
        self._allowed_pairs: set[frozenset[str]] = set()
        for j in self._joints.values():
            self._allowed_pairs.add(frozenset((j.parent, j.child)))
        if extra_allowed_pairs:
            for a, b in extra_allowed_pairs:
                self._allowed_pairs.add(frozenset((a, b)))

        # Cache last-set world transforms for inspection / debug.
        self._world_T: dict[str, np.ndarray] = {self.root_link: np.eye(4)}

        # Convex-hull over-approximation and CAD-import artifacts can make
        # certain non-adjacent link pairs report as "colliding" even at the
        # canonical home pose (q=0).  These are by construction false
        # positives — if they collide AT HOME, they're either always-touching
        # in CAD or the hull encloses an empty pocket.  Promote them into
        # the allowed-pair set so the sweep only flags collisions that
        # actually appear when joints move away from home.
        if auto_allow_home_pairs:
            self.set_qpos({j: 0.0 for j in self._joints})
            spurious = self.colliding_pairs()  # raw pairs minus existing allowed
            for pair in spurious:
                self._allowed_pairs.add(pair)
            self._home_spurious_pairs: list[frozenset[str]] = list(spurious)
        else:
            self._home_spurious_pairs = []

    # ---- introspection -------------------------------------------------

    @property
    def link_names(self) -> list[str]:
        return list(self._link_meshes.keys())

    @property
    def joint_names(self) -> list[str]:
        return list(self._joints.keys())

    def joint_limit(self, name: str) -> tuple[float, float]:
        j = self._joints[name]
        return (j.lower, j.upper)

    def joint_type(self, name: str) -> str:
        return self._joints[name].jtype

    def ancestors_of(self, joint_name: str) -> list[str]:
        """Return the joint chain from root down to joint_name's parent link,
        in root-first order.  These are the joints whose q can move
        joint_name's parent link, i.e. whose state affects collision context
        for sweeping joint_name."""
        j = self._joints[joint_name]
        # Walk parent links back to root, recording joints along the way.
        joints_by_child = {jj.child: jj for jj in self._joints.values()}
        chain: list[str] = []
        cursor = j.parent
        while cursor in joints_by_child:
            anc = joints_by_child[cursor]
            chain.append(anc.name)
            cursor = anc.parent
        chain.reverse()
        return chain

    # ---- FK ------------------------------------------------------------

    def set_qpos(self, q: dict[str, float]) -> None:
        """Set joint positions and propagate world transforms.

        Joints not present in `q` are treated as zero.  Continuous joints
        (wheels) and fixed joints contribute only their static origin.
        """
        # BFS from root, recomputing world transforms.
        world: dict[str, np.ndarray] = {self.root_link: np.eye(4)}
        stack: list[str] = [self.root_link]
        while stack:
            parent_link = stack.pop()
            T_parent_w = world[parent_link]
            for child_link in self._children_by_parent.get(parent_link, ()):
                # Find the joint connecting parent_link -> child_link.
                joint = next(
                    (jj for jj in self._joints.values()
                     if jj.parent == parent_link and jj.child == child_link),
                    None,
                )
                if joint is None:
                    continue
                # Build joint local transform = origin_static * rot(axis, theta)
                theta = float(q.get(joint.name, 0.0)) if joint.jtype in (
                    "revolute", "continuous", "prismatic",
                ) else 0.0
                if joint.jtype == "prismatic":
                    # Translation along axis instead of rotation.
                    a = joint.axis / (np.linalg.norm(joint.axis) + 1e-12)
                    R_dyn = np.eye(3)
                    t_dyn = a * theta
                else:
                    R_dyn = _axis_angle_to_R(joint.axis, theta)
                    t_dyn = np.zeros(3)
                T_local = _T(joint.R_static, joint.t_static) @ _T(R_dyn, t_dyn)
                T_child_w = T_parent_w @ T_local
                world[child_link] = T_child_w
                stack.append(child_link)

        # Push into CollisionManager.  Skip links that weren't meshed.
        for name, Tw in world.items():
            if name in self._link_meshes:
                self._mgr.set_transform(name, Tw)
        self._world_T = world

    def link_world_transform(self, name: str) -> np.ndarray:
        return self._world_T.get(name, np.eye(4))

    # ---- queries -------------------------------------------------------

    def in_collision(self) -> bool:
        """True if any non-allowed link pair is colliding."""
        return bool(self.colliding_pairs())

    def colliding_pairs(self) -> set[frozenset[str]]:
        """All currently-colliding link pairs, EXCLUDING allowed pairs."""
        # in_collision_internal(return_names=True) returns (bool, set of (a,b) tuples)
        _, names = self._mgr.in_collision_internal(return_names=True)
        out: set[frozenset[str]] = set()
        for a, b in names:
            pair = frozenset((a, b))
            if pair in self._allowed_pairs:
                continue
            out.add(pair)
        return out
