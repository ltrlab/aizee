"""IK package for AIZEE arm — URDF-driven FK + Jacobian + DLS IK.

Pure-numpy; no native dependencies.  See `kinematics.py` for FK/Jacobian
and `dls_ik.py` for the damped least-squares Cartesian IK loop used by
the WebXR / Quest teleop path.
"""

from .kinematics import Kinematics, load_aizee_arm
from .dls_ik import solve_ik, IKResult

__all__ = ["Kinematics", "load_aizee_arm", "solve_ik", "IKResult"]
