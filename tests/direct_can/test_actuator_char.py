"""
test_actuator_char.py — validates the ROBSTRIDE codec + Phase-0 analysis math
WITHOUT hardware. Runnable on any box: `python tests/direct_can/test_actuator_char.py`.

Checks:
  1. build_control byte/arb layout matches the Rust driver (hand-computed).
  2. per-model scaling actually differs (RS04 vs RS02 torque encode).
  3. decode_feedback round-trips known raw values.
  4. each analysis metric recovers a known answer from a synthetic signal.
  5. run_profile drives the loop, logs rows, and always disables (dry bus).
"""
from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8")
    except Exception:
        pass

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python" / "scripts"))

import numpy as np

from actuator_char import analysis
from actuator_char import robstride_mit as rs
from actuator_char.external_encoder import NullEncoder
from actuator_char.harness import Joint, run_profile, step_profile, chirp_profile, triangle_profile


def test_control_frame_layout():
    arb, data = rs.build_control("RS02", 1, 0.0, 0.0, 0.0, 0.0, 0.0)
    # pos/vel midpoints = 0x7FFF, kp/kd = 0, torque midpoint carried in arb bits 8..23
    assert data == bytes([0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0x00, 0x00, 0x00]), data.hex()
    assert arb == ((1 << 24) | (0x7FFF << 8) | 1), hex(arb)


def test_per_model_scaling_differs():
    # 60 Nm command: RS04 (max 120) encodes to 1.5*mid; RS02 (max 17) clamps to full-scale.
    _, _ = rs.build_control("RS04", 1, 0, 0, 0, 0, 60.0)
    arb4, _ = rs.build_control("RS04", 1, 0, 0, 0, 0, 60.0)
    arb2, _ = rs.build_control("RS02", 1, 0, 0, 0, 0, 60.0)
    tau4 = (arb4 >> 8) & 0xFFFF
    tau2 = (arb2 >> 8) & 0xFFFF
    assert tau4 == int((60.0 / 120.0 + 1.0) * 0x7FFF), tau4       # 0xBFFE
    assert tau2 == 0xFFFF or tau2 == 0xFFFE, hex(tau2)            # clamped full scale
    assert tau4 < tau2, (hex(tau4), hex(tau2))


def test_decode_feedback_roundtrip():
    # motor_id=1, mode=RUN(2), err=0; raw midpoints -> ~0 pos/vel/tau; temp raw 300 -> 30.0C
    arb = (int(rs.MotorMsg.FEEDBACK) << 24) | (rs.MotorMode.RUN << 22) | (0 << 16) | (1 << 8)
    data = bytes([0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x01, 0x2C])
    assert rs.is_feedback_frame(arb)
    fb = rs.decode_feedback(arb, data, "RS02")
    assert fb.motor_id == 1 and fb.mode == rs.MotorMode.RUN
    assert abs(fb.position) < 1e-3 and abs(fb.velocity) < 1e-2 and abs(fb.torque) < 1e-2
    assert abs(fb.temperature - 30.0) < 1e-6


def test_step_metrics():
    t = np.linspace(0, 3, 3000)
    tau_c = 0.1
    y = 0.3 * (1.0 - np.exp(-t / tau_c))
    m = analysis.step_metrics(t, y, y0=0.0, y_target=0.3)
    assert 0.15 < m["rise_s"] < 0.30, m
    assert m["overshoot_pct"] < 1.0, m
    assert 0.30 < m["settle_s"] < 0.60, m
    assert abs(m["sse"]) < 0.01, m


def test_bandwidth_one_pole():
    dt = 0.002
    t = np.arange(0, 40, dt)
    fc = 2.0
    # log chirp command
    f0, f1 = 0.1, 12.0
    k = (f1 / f0) ** (1.0 / t[-1])
    phase = 2 * np.pi * f0 * ((k ** t - 1) / math.log(k))
    cmd = np.sin(phase)
    # one-pole low-pass with cutoff fc
    alpha = dt / (1.0 / (2 * np.pi * fc) + dt)
    resp = np.zeros_like(cmd)
    for n in range(1, len(cmd)):
        resp[n] = resp[n - 1] + alpha * (cmd[n] - resp[n - 1])
    bw = analysis.bode_bandwidth(t, cmd, resp, fmin=0.1, fmax=12.0)["bandwidth_hz"]
    assert 0.8 < bw < 6.0, bw   # generous: FFT-on-chirp estimate near fc=2 Hz


def test_backlash():
    t = np.linspace(0, 32, 6400)
    period = 8.0
    frac = (t % period) / period
    cmd = 0.3 * (4 * np.abs(frac - 0.5) - 1)          # triangle, rad
    b = math.radians(1.0)                              # 1 deg of lost motion
    dcmd = np.gradient(cmd)
    resp = cmd - (b / 2.0) * np.sign(dcmd)             # hysteresis of width b
    bl = analysis.backlash_deg(cmd, resp)
    assert 0.5 < bl < 1.5, bl


def test_friction_and_kt():
    v = np.linspace(-5, 5, 400)
    tau = 0.5 * np.sign(v) + 0.1 * v
    fr = analysis.friction_fit(v, tau)
    assert abs(fr["coulomb_nm"] - 0.5) < 0.05 and abs(fr["viscous_nms"] - 0.1) < 0.02, fr
    cur = np.linspace(0, 20, 100)
    kt = analysis.kt_fit(cur, 0.09 * cur)
    assert abs(kt["kt_nm_per_a"] - 0.09) < 1e-3, kt


def test_tracking_and_latency():
    tr = analysis.tracking_error(np.zeros(50), np.full(50, 0.01))
    assert abs(tr["rms"] - 0.01) < 1e-9 and abs(tr["peak"] - 0.01) < 1e-9
    dt = 0.005
    t = np.arange(0, 10, dt)
    cmd = np.sin(2 * np.pi * 0.7 * t) + 0.5 * np.sin(2 * np.pi * 1.9 * t)
    L = 6
    resp = np.concatenate([np.zeros(L), cmd[:-L]])   # resp lags cmd by L samples
    lat = analysis.latency_s(t, cmd, resp)["latency_s"]
    assert abs(lat - L * dt) < 1.6 * dt, (lat, L * dt)


class _DryBus:
    def __init__(self): self.enabled = False; self.shut = False; self.n = 0
    def enable(self): self.enabled = True
    def disable(self): self.enabled = False
    def control(self, *_): self.n += 1
    def read_feedback(self, timeout=0.0): return None
    def shutdown(self): self.shut = True; self.enabled = False


def test_run_profile_logs_and_disables():
    bus = _DryBus()
    with tempfile.TemporaryDirectory() as d:
        csv_path = str(Path(d) / "run.csv")
        rows = run_profile(bus, step_profile(0.3, 3.0, 0.3), duration=0.05,
                           ext=NullEncoder(), loop_hz=200, settle_s=0.0, csv_path=csv_path)
        assert len(rows) > 0
        assert bus.shut and not bus.enabled            # disabled/shutdown on exit
        assert bus.n >= len(rows)                       # sent at least one control per row
        assert math.isnan(rows[0]["ext_angle"])         # NullEncoder -> NaN
        assert Path(csv_path).exists()


def run():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print("ACTUATOR-CHAR TEST PASS")


if __name__ == "__main__":
    run()
