"""gravity_comp.py — Gravity compensation for the AIZEE 6-DoF arm.

Computes the gravitational torque at each joint so it can be sent as
feedforward via the `torques` field of the arm_joints command.

The arm is modelled as a serial chain of rigid links.  Each link has a
mass and a center-of-mass offset along its local X-axis (the link
direction).  Joints rotate about a specified axis (Z for yaw joints,
Y for pitch joints, X for roll joints).

Usage:
    from control.gravity_comp import ArmGravityModel
    model = ArmGravityModel()                     # uses defaults
    model = ArmGravityModel.from_urdf("aizee.urdf")  # from URDF
    tau = model.gravity_torques(q)                # q is [6] rad
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class LinkParams:
    """Physical parameters for one link in the serial chain."""
    length: float          # link length (m) — distance to the next joint
    mass: float            # link mass (kg) — includes motor, structure, etc.
    com_x: float           # center of mass X offset in link frame (m)
    com_y: float = 0.0     # center of mass Y offset in link frame (m)
    com_z: float = 0.0     # center of mass Z offset in link frame (m)


@dataclass
class JointDef:
    """Joint definition in the serial chain."""
    name: str
    axis: str              # "X", "Y", or "Z"
    link: LinkParams       # the link that follows (is moved by) this joint


# ---------------------------------------------------------------------------
# Default arm model — from physical measurements (2026-03-23)
# ---------------------------------------------------------------------------
# Link lengths from record_replay.py / CLAUDE.md.
# Masses weighed per-segment including motors, brackets, cables.

_DEFAULT_CHAIN = [
    JointDef("gantry_base",  "Z", LinkParams(length=0.5906, mass=2.00, com_x=0.3937)),  # CoM 2/3 from base
    JointDef("gantry_mid",   "Y", LinkParams(length=0.5649, mass=2.20, com_x=0.2825)),  # CoM 1/2 way
    JointDef("gantry_end",   "Y", LinkParams(length=0.100,  mass=1.00, com_x=0.050)),   # CoM center
    JointDef("wrist_pitch",  "Y", LinkParams(length=0.1063, mass=0.50, com_x=0.053)),   # CoM center
    JointDef("wrist_roll",   "X", LinkParams(length=0.132,  mass=0.50, com_x=0.066)),   # CoM center
    JointDef("gripper",      "Z", LinkParams(length=0.0,    mass=0.25, com_x=0.0)),     # gripper mechanism
]


class ArmGravityModel:
    """Compute gravity-compensation torques for a serial-chain arm.

    The arm base is assumed to be mounted vertically (gravity acts along
    the world -Z axis).  The arm mount offset (ARM_MOUNT_Z) is irrelevant
    for torque computation — only the joint angles matter.
    """

    def __init__(
        self,
        chain: Optional[list[JointDef]] = None,
        gravity: float = 9.81,
    ):
        self.chain = chain if chain is not None else list(_DEFAULT_CHAIN)
        self.g = gravity
        self.n_joints = len(self.chain)

    # ------------------------------------------------------------------
    # Core computation
    # ------------------------------------------------------------------

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """Compute gravity-compensation torques for joint angles *q*.

        Args:
            q: [n_joints] array of joint angles in radians.

        Returns:
            tau: [n_joints] array of feedforward torques (Nm).
                 Positive torque = counteracts gravity (add to command).
        """
        assert len(q) == self.n_joints
        tau = np.zeros(self.n_joints, dtype=np.float64)

        # Build cumulative rotation matrices from world frame to each
        # joint frame.  We walk the chain from base to tip.
        # R[i] rotates world-frame vectors into joint-i's frame.
        rotations: list[np.ndarray] = []
        R = np.eye(3)
        for i, jd in enumerate(self.chain):
            R = R @ _rotation_matrix(jd.axis, float(q[i]))
            rotations.append(R.copy())

        # Gravity vector in world frame (pointing down)
        g_world = np.array([0.0, 0.0, -self.g])

        # For each joint i, compute the torque due to gravity acting on
        # all links j >= i (the links distal to joint i).
        for i in range(self.n_joints):
            # Joint i's axis in world frame
            axis_local = _axis_vector(self.chain[i].axis)
            axis_world = rotations[i] @ axis_local

            total_torque = 0.0
            for j in range(i, self.n_joints):
                link = self.chain[j].link
                if link.mass < 1e-9:
                    continue

                # Position of link j's CoM relative to joint i, in world frame
                r_ij = self._com_position_world(q, rotations, i, j)

                # Gravitational force on link j
                F_g = link.mass * g_world

                # Torque about joint i's axis = axis . (r × F)
                torque_vec = np.cross(r_ij, F_g)
                total_torque += np.dot(axis_world, torque_vec)

            # Negate: we want the torque that *counteracts* gravity
            tau[i] = -total_torque

        return tau.astype(np.float32)

    def _com_position_world(
        self,
        q: np.ndarray,
        rotations: list[np.ndarray],
        from_joint: int,
        to_link: int,
    ) -> np.ndarray:
        """Position of link *to_link*'s CoM relative to joint *from_joint*,
        expressed in world frame.

        We walk the kinematic chain from joint *from_joint* to joint
        *to_link*, accumulating translations (rotated into world frame).
        """
        pos = np.zeros(3)

        # Walk from joint from_joint to joint to_link
        for k in range(from_joint, to_link):
            # Translation along link k (in link k's frame, along local X)
            link_vec_local = np.array([self.chain[k].link.length, 0.0, 0.0])
            pos += rotations[k] @ link_vec_local

        # Add CoM offset of the target link (in that link's frame)
        com_local = np.array([
            self.chain[to_link].link.com_x,
            self.chain[to_link].link.com_y,
            self.chain[to_link].link.com_z,
        ])
        pos += rotations[to_link] @ com_local

        return pos

    # ------------------------------------------------------------------
    # Factory: load from URDF
    # ------------------------------------------------------------------

    @classmethod
    def from_urdf(cls, urdf_path: str | Path, gravity: float = 9.81) -> "ArmGravityModel":
        """Build an ArmGravityModel from a URDF file.

        Extracts link masses, centers of mass, and joint axes from the
        URDF.  Requires the `urdf_parser_py` package.

        The URDF must contain joints named gantry_base .. gripper
        (matching ARM_JOINTS).
        """
        try:
            from urdf_parser_py.urdf import URDF
        except ImportError:
            raise ImportError(
                "urdf_parser_py is required for URDF loading. "
                "Install with: pip install urdf_parser_py"
            )

        robot = URDF.from_xml_file(str(urdf_path))

        # Expected joint names in order
        joint_names = [
            "gantry_base", "gantry_mid", "gantry_end",
            "wrist_pitch", "wrist_roll", "gripper",
        ]

        chain: list[JointDef] = []
        for jname in joint_names:
            # Find the URDF joint
            ujoint = None
            for j in robot.joints:
                if j.name == jname:
                    ujoint = j
                    break
            if ujoint is None:
                raise ValueError(f"Joint '{jname}' not found in URDF")

            # Determine axis
            ax = ujoint.axis
            if ax is None:
                ax = [0, 0, 1]  # default Z
            axis_str = _axis_from_vector(ax)

            # Find the child link
            child_link = None
            for link in robot.links:
                if link.name == ujoint.child:
                    child_link = link
                    break
            if child_link is None:
                raise ValueError(f"Child link '{ujoint.child}' not found in URDF")

            # Extract mass and CoM
            mass = 0.0
            com_x, com_y, com_z = 0.0, 0.0, 0.0
            if child_link.inertial is not None:
                mass = child_link.inertial.mass or 0.0
                if child_link.inertial.origin is not None:
                    xyz = child_link.inertial.origin.xyz or [0, 0, 0]
                    com_x, com_y, com_z = float(xyz[0]), float(xyz[1]), float(xyz[2])

            # Link length = distance to next joint's origin
            # Find the next joint that has this child link as parent
            length = 0.0
            next_jname_idx = joint_names.index(jname) + 1
            if next_jname_idx < len(joint_names):
                next_jname = joint_names[next_jname_idx]
                for j in robot.joints:
                    if j.name == next_jname and j.origin is not None:
                        xyz = j.origin.xyz or [0, 0, 0]
                        length = float(np.linalg.norm(xyz))
                        break

            chain.append(JointDef(
                name=jname,
                axis=axis_str,
                link=LinkParams(
                    length=length,
                    mass=mass,
                    com_x=com_x,
                    com_y=com_y,
                    com_z=com_z,
                ),
            ))

        return cls(chain=chain, gravity=gravity)

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def print_model(self) -> None:
        """Print a summary of the arm model."""
        total_mass = sum(jd.link.mass for jd in self.chain)
        print(f"Arm gravity model: {self.n_joints} joints, total mass={total_mass:.3f} kg")
        print(f"{'Joint':<16} {'Axis':>4} {'Length':>8} {'Mass':>7} {'CoM_x':>7}")
        for jd in self.chain:
            print(f"{jd.name:<16} {jd.axis:>4} {jd.link.length:8.4f} {jd.link.mass:7.3f} {jd.link.com_x:7.4f}")


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------

def _rotation_matrix(axis: str, angle: float) -> np.ndarray:
    """3×3 rotation matrix for rotation about X, Y, or Z."""
    c, s = math.cos(angle), math.sin(angle)
    if axis == "X":
        return np.array([
            [1, 0,  0],
            [0, c, -s],
            [0, s,  c],
        ])
    elif axis == "Y":
        return np.array([
            [ c, 0, s],
            [ 0, 1, 0],
            [-s, 0, c],
        ])
    elif axis == "Z":
        return np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1],
        ])
    else:
        raise ValueError(f"Unknown axis: {axis}")


def _axis_vector(axis: str) -> np.ndarray:
    """Unit vector for the given axis letter."""
    if axis == "X":
        return np.array([1.0, 0.0, 0.0])
    elif axis == "Y":
        return np.array([0.0, 1.0, 0.0])
    elif axis == "Z":
        return np.array([0.0, 0.0, 1.0])
    else:
        raise ValueError(f"Unknown axis: {axis}")


def _axis_from_vector(vec) -> str:
    """Determine dominant axis letter from a 3-element vector."""
    v = [abs(float(x)) for x in vec]
    idx = v.index(max(v))
    return ["X", "Y", "Z"][idx]


# ---------------------------------------------------------------------------
# Quick test / CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    model = ArmGravityModel()

    if len(sys.argv) > 1 and sys.argv[1].endswith(".urdf"):
        model = ArmGravityModel.from_urdf(sys.argv[1])

    model.print_model()
    print()

    # Test at a few configurations
    configs = {
        "all zeros (upright)":   np.zeros(6),
        "mid at 90° (horizontal)": np.array([0.0, math.pi/2, 0.0, 0.0, 0.0, 0.0]),
        "mid+end at 45°":        np.array([0.0, math.pi/4, math.pi/4, 0.0, 0.0, 0.0]),
        "fully extended horizontal": np.array([0.0, math.pi/2, 0.0, 0.0, 0.0, 0.0]),
    }

    for label, q in configs.items():
        tau = model.gravity_torques(q)
        print(f"{label}:")
        for i, jd in enumerate(model.chain):
            print(f"  {jd.name:<16}  q={q[i]:+6.3f} rad  tau={tau[i]:+8.3f} Nm")
        print()
