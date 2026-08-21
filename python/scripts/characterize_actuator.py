#!/usr/bin/env python3
"""
characterize_actuator.py — Phase-0 CLI: characterize ONE ROBSTRIDE joint on a
bench stand, log the sweep, and append a capability-sheet row.

Runs the ROBSTRIDE-native MIT codec (per-model scaling from robstride.rs) — NOT
the ±18 Nm packing in tests/direct_can/sine_wave_test.py, which mis-drives
RS03/RS04.

Examples (on the Jetson, one joint clamped in a stand):
  # step response of a shoulder-pitch RS04 on can1, id 2, with an external encoder
  python characterize_actuator.py --model RS04 --can-id 2 --bus can1 \
      --routine step --amp 0.35 --ext-port /dev/ttyACM0 --name L_shoulder_pitch

  # frequency response (bandwidth) of a wrist RS02
  python characterize_actuator.py --model RS02 --can-id 7 --routine chirp --amp 0.2 --f0 0.1 --f1 10

  # backlash of a gripper RS00 (slow triangle)
  python characterize_actuator.py --model RS00 --can-id 11 --routine triangle --amp 0.3 --period 8

  # stiction + Kt of any joint (needs the external encoder to detect first motion)
  python characterize_actuator.py --model RS03 --can-id 3 --routine torque_ramp --tau 5 --ext-port /dev/ttyACM0

Dry-run the plumbing off-robot (no CAN, logs NaN feedback):
  python characterize_actuator.py --model RS02 --can-id 7 --routine step --dry-run
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # make `actuator_char` importable

from actuator_char import analysis, robstride_mit as rs  # noqa: E402
from actuator_char.external_encoder import NullEncoder, SerialEncoder  # noqa: E402
from actuator_char.harness import (  # noqa: E402
    Joint, MotorBus, chirp_profile, run_profile, step_profile, torque_ramp_profile, triangle_profile,
)

SHEET_FIELDS = [
    "name", "model", "can_id", "routine",
    "rise_s", "overshoot_pct", "settle_s", "sse",
    "bandwidth_hz", "backlash_deg", "breakaway_nm", "coulomb_nm", "viscous_nms",
    "kt_nm_per_a", "tracking_rms", "tracking_peak", "latency_s", "notes",
]


def _make_bus(joint: Joint, dry_run: bool):
    if dry_run:
        return _DryBus(joint)
    return MotorBus(joint)


class _DryBus:
    """No-CAN stand-in so the loop + logging can be exercised off-robot."""

    def __init__(self, joint: Joint):
        self.joint = joint

    def enable(self): pass
    def disable(self): pass
    def zero(self): pass
    def save(self): pass
    def control(self, *_): pass
    def read_feedback(self, timeout: float = 0.0): return None
    def shutdown(self): pass


def analyze(routine: str, rows: list[dict], amp: float, model: str) -> dict:
    import numpy as np
    t = np.array([r["t"] for r in rows], float)
    cmd = np.array([r["cmd_pos"] for r in rows], float)
    ext = np.array([r["ext_angle"] for r in rows], float)
    fb_pos = np.array([r["fb_pos"] for r in rows], float)
    fb_vel = np.array([r["fb_vel"] for r in rows], float)
    fb_tau = np.array([r["fb_tau"] for r in rows], float)
    cmd_tau = np.array([r["cmd_tau"] for r in rows], float)
    # Prefer the external (output-shaft) angle; fall back to motor-side feedback.
    resp = ext if np.isfinite(ext).sum() > 0.5 * ext.size else fb_pos
    out: dict = {}
    if routine == "step":
        out.update(analysis.step_metrics(t, resp, y0=0.0, y_target=amp))
        out.update(analysis.latency_s(t, cmd, resp))
    elif routine == "chirp":
        out["bandwidth_hz"] = analysis.bode_bandwidth(t, cmd, resp)["bandwidth_hz"]
    elif routine == "triangle":
        out["backlash_deg"] = analysis.backlash_deg(cmd, resp)
    elif routine == "torque_ramp":
        moved = np.isfinite(resp) & (np.abs(resp - np.nanmin(resp)) > math.radians(0.5))
        out["breakaway_nm"] = analysis.breakaway_torque(cmd_tau, moved)
        good = np.isfinite(fb_vel) & (np.abs(fb_vel) > 1e-3)
        if good.sum() > 5:
            out.update(analysis.friction_fit(fb_vel[good], fb_tau[good]))
    elif routine == "tracking":
        out.update(analysis.tracking_error(cmd, resp))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, choices=list(rs.MODELS))
    ap.add_argument("--can-id", type=int, required=True)
    ap.add_argument("--bus", default="can1")
    ap.add_argument("--name", default="")
    ap.add_argument("--routine", required=True,
                    choices=["step", "chirp", "triangle", "torque_ramp", "tracking"])
    ap.add_argument("--amp", type=float, default=0.3, help="rad (position) amplitude")
    ap.add_argument("--f0", type=float, default=0.1)
    ap.add_argument("--f1", type=float, default=10.0)
    ap.add_argument("--period", type=float, default=8.0, help="triangle period (s)")
    ap.add_argument("--tau", type=float, default=2.0, help="torque_ramp target (Nm)")
    ap.add_argument("--dur", type=float, default=0.0, help="override duration (s); 0 = per-routine default")
    ap.add_argument("--kp", type=float, default=3.0)
    ap.add_argument("--kd", type=float, default=0.3)
    ap.add_argument("--loop-hz", type=int, default=200)
    ap.add_argument("--ext-port", default="")
    ap.add_argument("--ext-offset", type=float, default=0.0)
    ap.add_argument("--ext-invert", action="store_true")
    ap.add_argument("--out", default="phase0_runs")
    ap.add_argument("--sheet", default="phase0_runs/capability_sheet.csv")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    name = args.name or f"{args.model}_{args.can_id}"
    joint = Joint(name=name, model=args.model, can_id=args.can_id, bus=args.bus)

    if args.routine == "step":
        cmd_fn, dur = step_profile(args.amp, args.kp, args.kd), (args.dur or 3.0)
    elif args.routine == "chirp":
        dur = args.dur or 30.0
        cmd_fn = chirp_profile(args.amp, args.f0, args.f1, dur, args.kp, args.kd)
    elif args.routine == "triangle":
        dur = args.dur or (args.period * 4)
        cmd_fn = triangle_profile(args.amp, args.period, args.kp, args.kd)
    elif args.routine == "torque_ramp":
        dur = args.dur or 5.0
        cmd_fn = torque_ramp_profile(args.tau, dur)
    else:  # tracking — reuse a moderate chirp as a stand-in folding-speed trajectory
        dur = args.dur or 15.0
        cmd_fn = chirp_profile(args.amp, 0.2, 1.5, dur, args.kp, args.kd)

    ext = SerialEncoder(args.ext_port, offset_rad=args.ext_offset, invert=args.ext_invert) \
        if args.ext_port else NullEncoder()
    if isinstance(ext, NullEncoder):
        print("[warn] no --ext-port: logging motor-side feedback only (pre-gearbox); "
              "backlash/output-angle metrics need the external encoder.")

    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, f"{name}_{args.routine}.csv")
    bus = _make_bus(joint, args.dry_run)

    print(f"[run] {name}  model={args.model}  id={args.can_id}  bus={args.bus}  "
          f"routine={args.routine}  dur={dur:.1f}s  {'(DRY)' if args.dry_run else ''}")
    rows = run_profile(bus, cmd_fn, dur, ext=ext, loop_hz=args.loop_hz, csv_path=csv_path)
    try:
        ext.close()
    except Exception:
        pass

    metrics = analyze(args.routine, rows, args.amp, args.model) if rows else {}
    print(f"[log] {len(rows)} rows -> {csv_path}")
    for k, v in metrics.items():
        print(f"    {k:>16}: {v}")

    # Append the capability-sheet row.
    os.makedirs(os.path.dirname(args.sheet) or ".", exist_ok=True)
    exists = os.path.exists(args.sheet)
    row = {k: "" for k in SHEET_FIELDS}
    row.update(name=name, model=args.model, can_id=args.can_id, routine=args.routine, **metrics)
    with open(args.sheet, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SHEET_FIELDS, extrasaction="ignore")
        if not exists:
            w.writeheader()
        w.writerow(row)
    print(f"[sheet] appended -> {args.sheet}")


if __name__ == "__main__":
    main()
