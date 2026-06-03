"""Forward kinematics + body Jacobian on the AIZEE arm URDF.

Pure numpy: no pinocchio, no compiled deps — keeps the Windows install
to pip only.  Parses the URDF revolute-joint chain from root to EE link
and computes:

  * Per-joint local transform (origin xyz/rpy) × rotation(axis, theta)
  * Cumulative world transform of every link
  * 6×N spatial Jacobian (linear + angular columns) in the world frame
    for the EE link's frame origin

For AIZEE the chain in IK order is:
    swivel -> gantry_base -> gantry_mid -> gantry_end
           -> wrist_pitch -> wrist_roll
The gripper joint is the 7th in the control vector but is NOT part of
the IK chain — the operator drives it separately with the controller
trigger.

The arm is 6-jointed but has no wrist-yaw axis, so the end-effector
cannot achieve arbitrary 3-axis orientation.  Treat IK as 3-DoF position
+ 2-DoF achievable orientation; the DLS solver in `dls_ik.py` handles
the redundant null space and the unreachable rotation residual.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


# -----------------------------------------------------------------------------
# Small SO(3) helpers — kept inline so the module has zero external imports
# beyond numpy.
# -----------------------------------------------------------------------------

def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    """Convert URDF roll-pitch-yaw (ZYX intrinsic in URDF spec) to a 3x3 R."""
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _axis_angle_to_R(axis: np.ndarray, theta: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    a = axis / (np.linalg.norm(axis) + 1e-12)
    c, s = np.cos(theta), np.sin(theta)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + (1.0 - c) * np.outer(a, a) + s * K


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 R -> [x,y,z,w] quaternion (scalar-last, matches WebXR)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def quat_to_R(q: np.ndarray) -> np.ndarray:
    """[x,y,z,w] quaternion -> 3x3 R."""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def quat_log(q: np.ndarray) -> np.ndarray:
    """Quaternion logarithm -> 3-vector rotation (axis * angle)."""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    if w < 0:  # canonical hemisphere — shortest path
        x, y, z, w = -x, -y, -z, -w
    v = np.array([x, y, z])
    vn = np.linalg.norm(v)
    if vn < 1e-8:
        return 2.0 * v  # small-angle approx
    theta = 2.0 * np.arctan2(vn, w)
    return (theta / vn) * v


# -----------------------------------------------------------------------------
# URDF parsing
# -----------------------------------------------------------------------------

@dataclass
class _Joint:
    name: str
    jtype: str            # "revolute" | "fixed" | ...
    parent: str           # parent link name
    child: str            # child link name
    xyz: np.ndarray       # 3,
    rpy: np.ndarray       # 3,
    axis: np.ndarray      # 3, (joint axis in joint frame)
    lower: float
    upper: float
    R_static: np.ndarray  # 3x3 rpy as R, cached
    t_static: np.ndarray  # 3,  xyz copy, cached


def _parse_urdf(urdf_path: Path) -> dict[str, _Joint]:
    """Return {joint_name: _Joint} for every joint in the URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints: dict[str, _Joint] = {}
    for j in root.findall("joint"):
        name = j.get("name", "")
        jtype = j.get("type", "fixed")
        parent = j.find("parent").get("link") if j.find("parent") is not None else ""
        child = j.find("child").get("link") if j.find("child") is not None else ""
        origin = j.find("origin")
        if origin is not None:
            xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        else:
            xyz = np.zeros(3)
            rpy = np.zeros(3)
        axis_el = j.find("axis")
        axis = np.fromstring(axis_el.get("xyz", "0 0 1"), sep=" ") if axis_el is not None else np.array([0, 0, 1.0])
        limit = j.find("limit")
        lower = float(limit.get("lower", "-3.14159265358979")) if limit is not None else -np.pi
        upper = float(limit.get("upper", "3.14159265358979")) if limit is not None else np.pi
        joints[name] = _Joint(
            name=name, jtype=jtype, parent=parent, child=child,
            xyz=xyz, rpy=rpy, axis=axis, lower=lower, upper=upper,
            R_static=_rpy_to_R(rpy), t_static=xyz.copy(),
        )
    return joints


def _chain_root_to_link(joints: dict[str, _Joint], ee_link: str) -> list[_Joint]:
    """Walk parent links from ee_link back to root; return ordered chain root->ee."""
    by_child = {j.child: j for j in joints.values()}
    chain: list[_Joint] = []
    cursor = ee_link
    while cursor in by_child:
        j = by_child[cursor]
        chain.append(j)
        cursor = j.parent
    chain.reverse()
    return chain


# -----------------------------------------------------------------------------
# Kinematics
# -----------------------------------------------------------------------------

class Kinematics:
    """Forward kinematics + 6xN spatial Jacobian for a serial revolute chain.

    Construct via `Kinematics.from_urdf(urdf_path, ee_link, joint_order=...)`
    or via the AIZEE convenience helper `load_aizee_arm()`.

    The `joint_order` argument selects which revolute joints are
    "controlled" (rows of q).  Fixed joints between them are folded into
    the static transforms.  Joints in the chain that aren't listed in
    `joint_order` are also treated as static at q=0 — useful for ignoring
    e.g. the gripper finger.

    All transforms are expressed in the URDF root frame (`tophalfbase`
    for AIZEE).
    """

    def __init__(
        self,
        chain: list[_Joint],
        controlled_names: list[str],
    ) -> None:
        # Validate every controlled joint appears in the chain in order.
        chain_names = [j.name for j in chain]
        for n in controlled_names:
            if n not in chain_names:
                raise ValueError(f"Controlled joint {n!r} not in URDF chain to EE")
        # Sort controlled_names by chain order so q[] indexing is monotonic.
        controlled_names_sorted = [n for n in chain_names if n in controlled_names]
        if controlled_names_sorted != controlled_names:
            # Caller supplied a different order — respect their order but warn.
            # In practice the caller should pre-sort; this keeps a guarantee
            # of correctness either way.
            controlled_names_sorted = controlled_names
        self._chain = chain
        self._controlled = controlled_names_sorted
        self._ctrl_idx_in_chain = [chain_names.index(n) for n in controlled_names_sorted]
        # Joint limits in q order.
        self.lower = np.array(
            [chain[i].lower for i in self._ctrl_idx_in_chain], dtype=np.float64
        )
        self.upper = np.array(
            [chain[i].upper for i in self._ctrl_idx_in_chain], dtype=np.float64
        )
        # Joint axes in q order (still expressed in each joint's local frame).
        self._axes_local = [
            chain[i].axis / (np.linalg.norm(chain[i].axis) + 1e-12)
            for i in self._ctrl_idx_in_chain
        ]

    # ---- construction --------------------------------------------------

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path | str,
        ee_link: str,
        joint_order: list[str],
    ) -> "Kinematics":
        joints = _parse_urdf(Path(urdf_path))
        chain = _chain_root_to_link(joints, ee_link)
        if not chain:
            raise ValueError(f"No chain from URDF root to link {ee_link!r}")
        return cls(chain, joint_order)

    def apply_limits_overlay(self, yaml_path: Path | str, *, verbose: bool = True) -> None:
        """Tighten self.lower / self.upper using effective limits from a
        collision-sweep YAML (produced by `python -m ik.collision_sweep`).

        Only tightens — never loosens past the URDF.  Missing joints in the
        YAML are left at URDF limits.  Re-run safely; idempotent.
        """
        import yaml as _yaml
        data = _yaml.safe_load(Path(yaml_path).read_text()) or {}
        joints_block = data.get("joints", {})
        changed: list[str] = []
        for i, name in enumerate(self._controlled):
            entry = joints_block.get(name)
            if not entry:
                continue
            eff_lo = float(entry.get("effective_lower", self.lower[i]))
            eff_hi = float(entry.get("effective_upper", self.upper[i]))
            new_lo = max(self.lower[i], eff_lo)
            new_hi = min(self.upper[i], eff_hi)
            if new_lo != self.lower[i] or new_hi != self.upper[i]:
                changed.append(name)
                self.lower[i] = new_lo
                self.upper[i] = new_hi
        if verbose and changed:
            print(f"[ik] applied collision-sweep limits to {len(changed)} joints: {changed}")

    # ---- accessors -----------------------------------------------------

    @property
    def n(self) -> int:
        """Number of controlled joints (DoF of IK)."""
        return len(self._controlled)

    @property
    def joint_names(self) -> list[str]:
        return list(self._controlled)

    # ---- FK ------------------------------------------------------------

    def _per_joint_q(self, q: np.ndarray) -> list[float]:
        """Expand control q into a full per-chain-joint angle list (0 for non-controlled)."""
        out = [0.0] * len(self._chain)
        for k, idx in enumerate(self._ctrl_idx_in_chain):
            out[idx] = float(q[k])
        return out

    def fk_frames(self, q: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
        """Return [(R_world, t_world)] for each link in the chain (parent of joint),
        ending with the EE link.  Length = len(chain) + 1.
        """
        per = self._per_joint_q(q)
        R_acc = np.eye(3)
        t_acc = np.zeros(3)
        frames = [(R_acc.copy(), t_acc.copy())]
        for j, theta in zip(self._chain, per):
            R_j = j.R_static @ _axis_angle_to_R(j.axis, theta)
            t_acc = t_acc + R_acc @ j.t_static
            R_acc = R_acc @ R_j
            frames.append((R_acc.copy(), t_acc.copy()))
        return frames

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """End-effector (R_world, t_world) for control vector q."""
        frames = self.fk_frames(q)
        return frames[-1]

    def fk_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """End-effector (position[3], quaternion[xyzw]) — convenience for IK."""
        R, t = self.fk(q)
        return t, R_to_quat(R)

    # ---- Jacobian ------------------------------------------------------

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """6xN spatial Jacobian at the EE frame, world coords.

        Rows 0..2 = linear velocity contribution per joint.
        Rows 3..5 = angular velocity contribution per joint (world axis).
        """
        # Walk the chain accumulating frames, remembering the world-frame
        # position and axis of each controlled joint.
        per = self._per_joint_q(q)
        R_acc = np.eye(3)
        t_acc = np.zeros(3)
        joint_origins_world: list[np.ndarray] = []  # in q order
        joint_axes_world:    list[np.ndarray] = []  # in q order
        ctrl_set = set(self._ctrl_idx_in_chain)
        for chain_idx, (j, theta) in enumerate(zip(self._chain, per)):
            # The joint's own frame origin = parent's R_acc rotation of j.t_static
            t_acc = t_acc + R_acc @ j.t_static
            # The rotation axis is in the *joint* frame (post-rpy).  We want it
            # in the world frame, BEFORE applying theta — i.e. parent's R_acc
            # composed with j.R_static (the rpy part), but NOT the theta part.
            R_pre_theta = R_acc @ j.R_static
            if chain_idx in ctrl_set:
                joint_origins_world.append(t_acc.copy())
                joint_axes_world.append(R_pre_theta @ j.axis)
            R_acc = R_pre_theta @ _axis_angle_to_R(j.axis, theta)
        # EE position
        t_ee = t_acc
        # Assemble Jacobian columns.
        J = np.zeros((6, self.n), dtype=np.float64)
        for k in range(self.n):
            a = joint_axes_world[k]
            a = a / (np.linalg.norm(a) + 1e-12)
            o = joint_origins_world[k]
            J[0:3, k] = np.cross(a, t_ee - o)
            J[3:6, k] = a
        return J


# -----------------------------------------------------------------------------
# AIZEE convenience loader
# -----------------------------------------------------------------------------

# In control-vector order — matches the firmware-side joint vector (swivel
# through wrist twist).  Names here are URDF-side; the leader classes'
# AIZEE_JOINTS is firmware-side ("wrist_roll" instead of "wrist_swivel")
# and the two are mapped positionally by IK index, not by name.  Re-exports
# from OnShape can rename joints; update this list if the URDF changes.
AIZEE_ARM_IK_JOINTS = [
    "swivel",
    "gantry_base",
    "gantry_mid",
    "gantry_end",
    "wrist_pitch",
    "wrist_swivel",   # was "wrist_roll" in the pre-2026-05-20 URDF
]

# End-effector link in the URDF — child of the last arm joint (wrist_swivel).
AIZEE_EE_LINK = "1_1_02_eb571_960_stp_2"

_DEFAULT_URDF = Path(__file__).resolve().parents[2] / "urdf" / "aizee" / "aizee.urdf"


def load_aizee_arm(urdf_path: Optional[Path | str] = None) -> Kinematics:
    """Build a Kinematics for the AIZEE 6-joint arm (no gripper)."""
    p = Path(urdf_path) if urdf_path is not None else _DEFAULT_URDF
    return Kinematics.from_urdf(p, AIZEE_EE_LINK, AIZEE_ARM_IK_JOINTS)
