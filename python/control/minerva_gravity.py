"""minerva_gravity.py — empirical gravity + friction feedforward for the Minerva
RobStride arms.

WHY NOT A LINK-MASS MODEL: AIZEE's gravity_comp.py built a serial chain from
*assumed* link lengths/masses/CoMs and solved for masses at a handful of poses.
It never worked on hardware — the assumed kinematics were wrong and it modelled
**no friction at all**, so the identified "gravity" absorbed friction and fit
nothing cleanly. Minerva has no trustworthy URDF either (and the repo is
pip-only: no pinocchio). So we go fully empirical.

WHAT WE DO INSTEAD: identify each arm joint's quasi-static holding torque
DIRECTLY from a slow bidirectional sweep of the measured motor torque. Per joint
we fit, by linear least squares:

    tau_hold(theta, thetadot) =  A*sin(theta) + B*cos(theta) + C     <- gravity
                               + fc*sign(thetadot) + fv*thetadot      <- friction

  * A*sin+B*cos+C is exact for a single rigid link swinging under gravity about a
    horizontal axis. For a serial arm it is the gravity torque *projected at the
    pose the sweep was taken in* (distal joints held fixed) — see the coupling
    note below.
  * fc (Coulomb) is the torque OFFSET between the up-sweep and down-sweep curves:
    the direction-dependent friction that makes an uncompensated arm feel "slow
    going up, fast coming down". fv (viscous) is the velocity-proportional drag.

RUNTIME FEEDFORWARD is the gravity term ONLY:

    tau_ff(theta) = A*sin(theta) + B*cos(theta) + C

fed through the already-plumbed `torques` field of the arm_joints command
(motor_control adds it to the on-motor PD loop as tau_ff). We do NOT feed the
Coulomb term forward by default — open-loop sign(thetadot) feedforward invites
limit cycles / buzzing around zero velocity. It is identified and stored for
diagnostics, and an optional fraction can be enabled later.

COUPLING CAVEAT: sweeping joint i with the distal joints held fixed captures the
gravity curve *at that held pose*. The shoulder-pitch torque genuinely depends
on the elbow angle (the distal CoM moves), so a 1-D per-joint fit is a slice, not
the full G(q). It captures the dominant effect and is the standard pragmatic
calibration when no dynamic model exists; the sweep tool records the full joint
vector every sample, so a coupled fit can be layered on later WITHOUT
re-collecting. A single global `scale` trim is provided for the residual.

Pure numpy. No pinocchio, no URDF, no compiled deps.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from common.minerva_constants import (
    ARM_INDICES,
    MINERVA_JOINTS,
    NUM_MINERVA_JOINTS,
    SAT_TORQUE,
)

# Feedforward is clamped to this fraction of each joint's nominal saturation
# torque, so a bad fit (huge A/B/C) can never command a dangerous tau_ff.
_FF_SAT_FRACTION = 0.8

# Samples slower than this |thetadot| (rad/s) are dropped from the fit: at a sweep
# endpoint the joint is (nearly) still and Coulomb friction is in its ambiguous
# stiction band [-fc, fc], so sign(thetadot) is undefined there.
_DEFAULT_V_MIN = 0.02


# ---------------------------------------------------------------------------
# Per-joint fit
# ---------------------------------------------------------------------------

@dataclass
class JointGravityFit:
    """Identified holding-torque model for ONE joint (canonical 17-vec index)."""
    index: int
    name: str
    # gravity terms
    A: float          # sin(theta) coefficient (Nm)
    B: float          # cos(theta) coefficient (Nm)
    C: float          # constant offset (Nm) — encoder-zero phase + tare
    # friction terms (identified, stored; not fed forward by default)
    fc: float         # Coulomb friction magnitude (Nm)
    fv: float         # viscous friction (Nm per rad/s)
    # diagnostics
    r2: float
    rms_nm: float
    n_samples: int
    theta_min: float
    theta_max: float
    vmax: float       # peak |thetadot| seen in the fitted samples

    def gravity_torque(self, theta: float) -> float:
        """Position-dependent gravity feedforward at angle `theta` (rad)."""
        return self.A * np.sin(theta) + self.B * np.cos(theta) + self.C


def fit_joint(
    index: int,
    theta: np.ndarray,
    thetadot: np.ndarray,
    tau: np.ndarray,
    *,
    v_min: float = _DEFAULT_V_MIN,
) -> JointGravityFit:
    """Least-squares fit of the gravity+friction model for one joint's sweep.

    Args:
        index: canonical 17-vector joint index (for labelling).
        theta, thetadot, tau: equal-length sample arrays (rad, rad/s, Nm),
            measured from telemetry over a slow bidirectional sweep.
        v_min: drop samples with |thetadot| below this (ambiguous stiction).

    Returns a JointGravityFit. Raises ValueError if too few moving samples.
    """
    theta = np.asarray(theta, dtype=np.float64).ravel()
    thetadot = np.asarray(thetadot, dtype=np.float64).ravel()
    tau = np.asarray(tau, dtype=np.float64).ravel()
    if not (len(theta) == len(thetadot) == len(tau)):
        raise ValueError("theta, thetadot, tau must be equal length")

    moving = np.abs(thetadot) >= v_min
    if int(moving.sum()) < 8:
        raise ValueError(
            f"joint {index}: only {int(moving.sum())} moving samples "
            f"(|thetadot|>={v_min}); sweep slower/longer or lower v_min")
    th, thd, ta = theta[moving], thetadot[moving], tau[moving]

    # Design matrix columns: [sin, cos, 1, sign(thetadot), thetadot]
    X = np.column_stack([
        np.sin(th), np.cos(th), np.ones_like(th), np.sign(thd), thd,
    ])
    coef, _res, _rank, _sv = np.linalg.lstsq(X, ta, rcond=None)
    A, B, C, fc, fv = (float(c) for c in coef)

    pred = X @ coef
    ss_res = float(np.sum((ta - pred) ** 2))
    ss_tot = float(np.sum((ta - ta.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    rms = float(np.sqrt(ss_res / len(ta)))

    return JointGravityFit(
        index=index, name=MINERVA_JOINTS[index],
        A=A, B=B, C=C, fc=abs(fc), fv=fv,
        r2=r2, rms_nm=rms, n_samples=int(len(ta)),
        theta_min=float(th.min()), theta_max=float(th.max()),
        vmax=float(np.abs(thd).max()),
    )


# ---------------------------------------------------------------------------
# Whole-arm model (17-vector)
# ---------------------------------------------------------------------------

class MinervaGravityModel:
    """Holds per-joint gravity fits and evaluates the 17-vector feedforward.

    Only identified joints contribute; every other index (unidentified arm
    joints, grippers, head, lift) returns 0 torque. Each joint's feedforward is
    clamped to +/-_FF_SAT_FRACTION * SAT_TORQUE[i] as a hard safety bound.
    """

    def __init__(self, fits: Dict[int, JointGravityFit], scale: float = 1.0):
        self.fits = dict(fits)
        self.scale = float(scale)
        self._cap = (_FF_SAT_FRACTION * np.asarray(SAT_TORQUE, dtype=np.float64))

    # -- evaluation ------------------------------------------------------
    def gravity_torques(self, q17: np.ndarray, scale: Optional[float] = None) -> np.ndarray:
        """Gravity feedforward [17] (Nm) for the current joint vector.

        `scale` overrides the model's stored scale for this call (a live trim knob —
        e.g. ramp 0→1 while validating on hardware). Non-finite inputs (a dropped
        arm reports NaN) yield 0 for that joint so a stale reading never injects a
        bogus torque. Output is clamped per joint to +/-_FF_SAT_FRACTION*SAT."""
        s = self.scale if scale is None else float(scale)
        q = np.asarray(q17, dtype=np.float64).ravel()
        tau = np.zeros(NUM_MINERVA_JOINTS, dtype=np.float64)
        for i, fit in self.fits.items():
            if i < len(q) and np.isfinite(q[i]):
                tau[i] = s * fit.gravity_torque(float(q[i]))
        return np.clip(tau, -self._cap, self._cap).astype(np.float32)

    # -- persistence -----------------------------------------------------
    def to_json(self, path: str | Path, *, meta: Optional[dict] = None) -> None:
        out = {
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "model": "empirical-sweep-v1",
            "form": "tau = A*sin(theta) + B*cos(theta) + C + fc*sign(thetadot) + fv*thetadot",
            "scale": self.scale,
            "ff_sat_fraction": _FF_SAT_FRACTION,
            "joints": {fit.name: asdict(fit) for fit in self.fits.values()},
        }
        if meta:
            out["meta"] = meta
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(out, indent=2))

    @classmethod
    def from_json(cls, path: str | Path) -> "MinervaGravityModel":
        data = json.loads(Path(path).read_text())
        name_to_idx = {n: i for i, n in enumerate(MINERVA_JOINTS)}
        fits: Dict[int, JointGravityFit] = {}
        for name, jd in data.get("joints", {}).items():
            idx = name_to_idx.get(name, jd.get("index"))
            if idx is None:
                continue
            fits[int(idx)] = JointGravityFit(
                index=int(idx), name=name,
                A=float(jd["A"]), B=float(jd["B"]), C=float(jd["C"]),
                fc=float(jd.get("fc", 0.0)), fv=float(jd.get("fv", 0.0)),
                r2=float(jd.get("r2", 0.0)), rms_nm=float(jd.get("rms_nm", 0.0)),
                n_samples=int(jd.get("n_samples", 0)),
                theta_min=float(jd.get("theta_min", 0.0)),
                theta_max=float(jd.get("theta_max", 0.0)),
                vmax=float(jd.get("vmax", 0.0)),
            )
        return cls(fits, scale=float(data.get("scale", 1.0)))

    # -- diagnostics -----------------------------------------------------
    def summary(self) -> str:
        lines = [
            f"{'joint':<14}{'A':>8}{'B':>8}{'C':>8}{'fc':>7}{'fv':>7}{'R2':>7}{'rms':>7}"]
        for i in sorted(self.fits):
            f = self.fits[i]
            lines.append(
                f"{f.name:<14}{f.A:>8.3f}{f.B:>8.3f}{f.C:>8.3f}"
                f"{f.fc:>7.3f}{f.fv:>7.3f}{f.r2:>7.3f}{f.rms_nm:>7.3f}")
        return "\n".join(lines)


__all__ = ["JointGravityFit", "fit_joint", "MinervaGravityModel", "ARM_INDICES"]


# ---------------------------------------------------------------------------
# Self-test: synthesise a sweep with known params + friction, recover them.
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    rng = np.random.default_rng(0)
    A_true, B_true, C_true, fc_true, fv_true = 3.5, -1.2, 0.4, 0.6, 0.15
    # bidirectional slow sweep: up then down
    up = np.linspace(-1.5, 1.5, 400)
    down = np.linspace(1.5, -1.5, 400)
    theta = np.concatenate([up, down])
    thetadot = np.concatenate([np.full(400, 0.12), np.full(400, -0.12)])
    tau = (A_true * np.sin(theta) + B_true * np.cos(theta) + C_true
           + fc_true * np.sign(thetadot) + fv_true * thetadot
           + rng.normal(0, 0.02, size=theta.shape))
    fit = fit_joint(1, theta, thetadot, tau)
    print("recovered:", f"A={fit.A:.3f} B={fit.B:.3f} C={fit.C:.3f} "
          f"fc={fit.fc:.3f} fv={fit.fv:.3f}  R2={fit.r2:.4f} rms={fit.rms_nm:.4f}")
    print("truth:    ", f"A={A_true} B={B_true} C={C_true} fc={fc_true} fv={fv_true}")
    model = MinervaGravityModel({1: fit})
    q = np.zeros(17); q[1] = 0.5
    print("tau_ff(q[j2]=0.5):", model.gravity_torques(q)[1], "Nm")
    assert abs(fit.A - A_true) < 0.1 and abs(fit.fc - fc_true) < 0.1, "fit off"
    print("OK")
