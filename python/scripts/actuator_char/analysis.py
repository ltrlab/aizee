"""
analysis.py — compute the Phase-0 capability-sheet metrics from a logged sweep.

Pure numpy, no hardware. Every function takes arrays and returns a dict, so the
same code runs on live logs and on synthetic signals in the unit test. Angles
are in radians unless a helper says otherwise; results that read better in
degrees are converted at the boundary.
"""
from __future__ import annotations

import numpy as np


def step_metrics(t, y, y0: float, y_target: float, settle_band: float = 0.02) -> dict:
    """Rise (10->90%), overshoot %, settle time (within ``settle_band`` of final),
    and steady-state error, for a step from ``y0`` to ``y_target``."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    span = y_target - y0
    if span == 0 or t.size < 3 or np.isfinite(y).sum() < 3:
        return dict(rise_s=np.nan, overshoot_pct=np.nan, settle_s=np.nan, sse=np.nan)
    norm = (y - y0) / span

    def _first_cross(level: float) -> float:
        idx = np.where(norm >= level)[0]
        return t[idx[0]] if idx.size else np.nan

    t10, t90 = _first_cross(0.1), _first_cross(0.9)
    rise = (t90 - t10) if np.isfinite(t10) and np.isfinite(t90) else np.nan
    overshoot = max(0.0, float(np.nanmax(norm)) - 1.0) * 100.0
    outside = np.where(np.abs(norm - 1.0) > settle_band)[0]
    settle = float(t[outside[-1]] - t[0]) if outside.size else 0.0
    sse = float(y_target - y[-1])
    return dict(rise_s=float(rise), overshoot_pct=float(overshoot), settle_s=settle, sse=sse)


def bode_bandwidth(t, cmd, resp, fmin: float = 0.1, fmax: float = 10.0) -> dict:
    """FFT transfer function of resp/cmd over a chirp; returns the -3 dB
    bandwidth (Hz) relative to the low-frequency gain, plus the raw curves."""
    t = np.asarray(t, float)
    cmd = np.asarray(cmd, float) - np.mean(cmd)
    resp = np.asarray(resp, float) - np.mean(resp)
    dt = float(np.median(np.diff(t)))
    n = t.size
    freqs = np.fft.rfftfreq(n, dt)
    C = np.fft.rfft(cmd)
    R = np.fft.rfft(resp)
    band = (freqs >= fmin) & (freqs <= fmax) & (np.abs(C) > 1e-6 * np.max(np.abs(C) + 1e-12))
    f = freqs[band]
    if f.size == 0:
        return dict(freqs=f, mag_db=np.array([]), phase_deg=np.array([]), bandwidth_hz=np.nan)
    H = R[band] / C[band]
    mag_db = 20.0 * np.log10(np.abs(H) + 1e-12)
    phase_deg = np.degrees(np.angle(H))
    g0 = float(np.mean(mag_db[: max(1, mag_db.size // 10)]))
    below = np.where(mag_db <= g0 - 3.0)[0]
    bandwidth = float(f[below[0]]) if below.size else float(f[-1])
    return dict(freqs=f, mag_db=mag_db, phase_deg=phase_deg, bandwidth_hz=bandwidth)


def backlash_deg(cmd, resp) -> float:
    """Lost motion at a direction reversal, estimated as the hysteresis width
    between the up-going and down-going branches of a slow triangle sweep.
    ``cmd`` and ``resp`` are both output-shaft angles (rad); result in degrees."""
    cmd = np.asarray(cmd, float)
    resp = np.asarray(resp, float)
    dc = np.gradient(cmd)
    up, dn = dc > 0, dc < 0
    if up.sum() < 5 or dn.sum() < 5:
        return float("nan")
    grid = np.linspace(np.percentile(cmd, 10), np.percentile(cmd, 90), 40)
    ru = np.interp(grid, cmd[up], resp[up], left=np.nan, right=np.nan)
    dn_order = np.argsort(cmd[dn])
    rd = np.interp(grid, cmd[dn][dn_order], resp[dn][dn_order], left=np.nan, right=np.nan)
    diff = np.abs(ru - rd)
    diff = diff[np.isfinite(diff)]
    return float(np.degrees(np.median(diff))) if diff.size else float("nan")


def friction_fit(vel, tau) -> dict:
    """Least-squares fit tau = coulomb*sign(v) + viscous*v over a slow velocity
    sweep. Coulomb term ~ kinetic friction torque; viscous ~ damping."""
    vel = np.asarray(vel, float)
    tau = np.asarray(tau, float)
    A = np.column_stack([np.sign(vel), vel])
    coef, *_ = np.linalg.lstsq(A, tau, rcond=None)
    return dict(coulomb_nm=float(coef[0]), viscous_nms=float(coef[1]))


def breakaway_torque(tau_cmd, moved) -> float:
    """Stiction: the commanded torque at the first sample where the OUTPUT shaft
    actually moved (``moved`` is a bool array from the external encoder)."""
    tau_cmd = np.asarray(tau_cmd, float)
    moved = np.asarray(moved, bool)
    idx = np.where(moved)[0]
    return float(tau_cmd[idx[0]]) if idx.size else float("nan")


def kt_fit(current_a, torque_nm) -> dict:
    """Torque constant Kt (Nm/A) from known-load torque vs measured current."""
    i = np.asarray(current_a, float)
    tau = np.asarray(torque_nm, float)
    A = np.column_stack([i, np.ones_like(i)])
    coef, *_ = np.linalg.lstsq(A, tau, rcond=None)
    return dict(kt_nm_per_a=float(coef[0]), offset_nm=float(coef[1]))


def tracking_error(cmd, resp) -> dict:
    """RMS / peak of (output - command) over a folding-speed trajectory."""
    e = np.asarray(resp, float) - np.asarray(cmd, float)
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), peak=float(np.max(np.abs(e))))


def latency_s(t, cmd, resp, max_lag_s: float = 0.2) -> dict:
    """Command->motion dead time via cross-correlation. Positive => resp lags cmd."""
    t = np.asarray(t, float)
    c = np.asarray(cmd, float) - np.mean(cmd)
    r = np.asarray(resp, float) - np.mean(resp)
    dt = float(np.median(np.diff(t)))
    xcorr = np.correlate(r, c, mode="full")
    lags = np.arange(-c.size + 1, c.size)
    keep = np.abs(lags) <= int(max_lag_s / dt)
    best = int(lags[keep][int(np.argmax(xcorr[keep]))])
    return dict(latency_s=best * dt)


def repeatability_rp(pose_errors_m) -> dict:
    """ISO-9283-style RP = mean + 3*sigma of the per-cycle position error (m).
    Feed it the Euclidean EE-pose error at the same commanded pose over >=30
    cycles (Stage 6); this single number is the reliability ceiling."""
    e = np.asarray(pose_errors_m, float)
    e = e[np.isfinite(e)]
    if e.size == 0:
        return dict(mean_m=np.nan, sigma_m=np.nan, rp_m=np.nan, n=0)
    return dict(mean_m=float(e.mean()), sigma_m=float(e.std(ddof=1) if e.size > 1 else 0.0),
                rp_m=float(e.mean() + 3.0 * (e.std(ddof=1) if e.size > 1 else 0.0)), n=int(e.size))
