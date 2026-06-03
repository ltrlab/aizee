"""Damped least-squares (DLS) inverse kinematics for the AIZEE arm.

Solves for a 6-joint q (swivel..wrist_roll) given a target EE pose
(position + quaternion).  Singularity-robust via Levenberg-Marquardt
damping on (Jᵀ J + λ² I); also gracefully handles unreachable targets
(AIZEE has no wrist-yaw, so the achievable orientation manifold is
2-DoF) by returning the best-effort q plus a residual report.

Position vs orientation are weighted independently because they have
different units; the operator-facing config can prioritize either.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .kinematics import Kinematics, quat_log, R_to_quat


# -----------------------------------------------------------------------------
# Quaternion helpers (kept local to avoid coupling to kinematics.py internals)
# -----------------------------------------------------------------------------

def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """[x,y,z,w] * [x,y,z,w] -> [x,y,z,w]"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _ori_error_world(q_current: np.ndarray, q_target: np.ndarray) -> np.ndarray:
    """3-vector orientation error in the world frame (axis * angle)
    pointing from q_current TO q_target.  Compatible with the angular
    rows of the spatial Jacobian."""
    # delta_q maps current -> target.  Expressed in the WORLD frame because
    # the Jacobian's angular block is also world-aligned.
    dq = _quat_mul(q_target, _quat_conj(q_current))
    return quat_log(dq)


# -----------------------------------------------------------------------------
# IK solver
# -----------------------------------------------------------------------------

@dataclass
class IKResult:
    q: np.ndarray            # solved joint vector (N,)
    pos_err: float           # final position error norm [m]
    ori_err: float           # final orientation error norm [rad]
    iters: int               # iterations used
    converged: bool          # both errors under tolerance
    clamped: np.ndarray      # bool (N,) — which joints hit a limit


def solve_ik(
    kin: Kinematics,
    q_init: np.ndarray,
    target_pos: np.ndarray,
    target_quat: np.ndarray,
    *,
    pos_weight: float = 1.0,
    ori_weight: float = 0.3,
    pos_tol: float = 1e-3,        # 1 mm
    ori_tol: float = 1e-2,        # ~0.6 deg
    damping: float = 5e-2,        # λ in (JᵀJ + λ²I)
    joint_weights: Optional[np.ndarray] = None,  # length n; >1 = harder to move
    max_iter: int = 8,
    step_clip: float = 0.25,      # rad per iteration cap
) -> IKResult:
    """Iterate DLS Newton steps from q_init toward (target_pos, target_quat).

    `joint_weights` (optional, length n) scales the per-joint damping so
    some joints are "harder" for the solver to move than others.  Concretely
    the regulariser becomes diag(λ²·w²) instead of λ²·I, so weight=2
    quadruples a joint's resistance and the IK preferentially uses lower-
    weighted joints when there's a choice.  Use this to keep gantry joints
    out of wrist-rotation tasks, etc.  When None (default), all joints get
    equal damping (the original behaviour).

    Returns IKResult with the best q reached (clamped to joint limits at
    every step).  When the target is outside the reachable manifold, the
    solver returns the q that minimises the weighted error; the caller
    can decide what to do with the residual.
    """
    q = np.asarray(q_init, dtype=np.float64).copy()
    n = kin.n
    if q.shape != (n,):
        raise ValueError(f"q_init shape {q.shape} != ({n},)")
    target_pos = np.asarray(target_pos, dtype=np.float64).reshape(3)
    target_quat = np.asarray(target_quat, dtype=np.float64).reshape(4)
    target_quat = target_quat / (np.linalg.norm(target_quat) + 1e-12)

    W = np.diag([pos_weight, pos_weight, pos_weight,
                 ori_weight, ori_weight, ori_weight])
    if joint_weights is None:
        lam2_I = (damping ** 2) * np.eye(n)
    else:
        jw = np.asarray(joint_weights, dtype=np.float64).reshape(-1)
        if jw.shape != (n,):
            raise ValueError(f"joint_weights shape {jw.shape} != ({n},)")
        lam2_I = (damping ** 2) * np.diag(jw * jw)

    pos_err = ori_err = float("inf")
    converged = False
    for it in range(max_iter):
        pos, quat = kin.fk_pose(q)
        e_pos = target_pos - pos
        e_ori = _ori_error_world(quat, target_quat)
        pos_err = float(np.linalg.norm(e_pos))
        ori_err = float(np.linalg.norm(e_ori))
        if pos_err < pos_tol and ori_err < ori_tol:
            converged = True
            break
        e = np.concatenate([e_pos, e_ori])
        We = W @ e
        J = kin.jacobian(q)
        WJ = W @ J
        # DLS step: dq = (Jᵀ Wᵀ W J + λ² I)⁻¹ Jᵀ Wᵀ W e
        A = WJ.T @ WJ + lam2_I
        b = WJ.T @ We
        try:
            dq = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            dq = np.linalg.lstsq(A, b, rcond=None)[0]
        # Clip the per-iter step so a large error can't fling joints across
        # their workspace and trip the velocity-limit at the leader output.
        step_norm = float(np.linalg.norm(dq))
        if step_norm > step_clip:
            dq *= (step_clip / step_norm)
        q = q + dq
        # Hard-clamp to joint limits at every step — no point continuing
        # to optimize past a limit.
        q = np.minimum(np.maximum(q, kin.lower), kin.upper)

    clamped = (q <= kin.lower + 1e-6) | (q >= kin.upper - 1e-6)
    return IKResult(
        q=q.astype(np.float32),
        pos_err=pos_err,
        ori_err=ori_err,
        iters=it + 1,
        converged=converged,
        clamped=clamped,
    )
