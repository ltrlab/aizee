#!/usr/bin/env python3
"""
characterize_actuator.py — Phase-0 CLI: characterize ONE ROBSTRIDE joint on a
bench stand, log the sweep, and append a capability-sheet row.

Uses the legacy deployed 29-bit RobStride codec. Bundled manuals conflict with
some scales; hardware runs require --bench-profile with firmware/captured-frame
evidence and a verified per-device envelope. See robot-platform sim audit and
docs/architecture/SIMULATION_IMPLEMENTATION.md before selecting trajectories.
Motor-reported torque does not validate output torque, backlash, or Kt.

Dry-run the plumbing off-robot (no CAN, logs NaN feedback):
  python characterize_actuator.py --model RS02 --can-id 7 --routine step --dry-run
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
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


def _make_bus(joint: Joint, dry_run: bool, profile=None, trace=None):
    if dry_run:
        return _DryBus(joint)
    return MotorBus(joint, profile=profile, trace=trace)


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
    # Cached feedback is not a new observation. Current external instruments
    # have no synchronized device timestamp, so do not infer output latency.
    rows = [r for r in rows if r.get("fb_new", True) and np.isfinite(r["fb_pos"])]
    if len(rows) < 3:
        return {"notes": "insufficient independent feedback samples"}
    t = np.array([r.get("fb_t", r["t"]) for r in rows], float)
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        return {"notes": "response metrics withheld: nonfinite or unordered sample timestamps"}
    cmd = np.array([r["cmd_pos"] for r in rows], float)
    fb_pos = np.array([r["fb_pos"] for r in rows], float)
    fb_vel = np.array([r["fb_vel"] for r in rows], float)
    fb_tau = np.array([r["fb_tau"] for r in rows], float)
    resp = fb_pos
    out: dict = {"notes": "drive-reported samples; output torque/angle not independently validated"}
    if routine == "step":
        changed = np.flatnonzero(np.abs(cmd - cmd[0]) > max(1e-8, abs(amp)*0.01))
        if changed.size:
            i = max(0, changed[0]-1)
            out.update(analysis.step_metrics(t[i:], resp[i:], y0=resp[i], y_target=cmd[-1]))
        out["notes"] += "; response time includes command slew and unknown transport delay"
    elif routine == "chirp":
        dt = np.diff(t)
        if np.max(np.abs(dt - np.median(dt))) <= 0.1*np.median(dt):
            out["bandwidth_hz"] = analysis.bode_bandwidth(t, cmd, resp)["bandwidth_hz"]
        else:
            out["notes"] += "; frequency metrics withheld: irregular sampling"
    elif routine == "triangle":
        out["notes"] += "; backlash withheld pending independent synchronized output measurement"
    elif routine == "torque_ramp":
        out["notes"] += "; breakaway torque withheld pending independent output measurement"
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
    ap.add_argument("--bench-profile", help="verified per-device JSON envelope; required for hardware")
    args = ap.parse_args()
    if args.dry_run and args.ext_port:
        ap.error("--dry-run cannot open an external encoder; omit --ext-port")

    name = args.name or f"{args.model}_{args.can_id}"
    joint = Joint(name=name, model=args.model, can_id=args.can_id, bus=args.bus)
    profile = None
    if not args.dry_run:
        import json
        from robot_platform.characterization.audit import validate_bench_profile
        if not args.bench_profile:
            ap.error("hardware runs require --bench-profile; run robot-platform sim audit first")
        with open(args.bench_profile, encoding="utf-8") as file:
            profile = json.load(file)
        validate_bench_profile(profile, joint)  # before opening any device

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
        print("[warn] no --ext-port: logging drive-reported feedback only; "
              "backlash/output-angle metrics need the external encoder.")

    os.makedirs(args.out, exist_ok=True)
    from robot_platform.data.traces import TraceWriter
    import uuid
    run_name = f"{name}_{args.routine}_{uuid.uuid4().hex}"
    csv_path = os.path.join(args.out, run_name + ".csv")
    trace = TraceWriter(os.path.join(args.out, run_name + ".h5"), clock_domain="host-perf-counter", clock=time.perf_counter)
    trace.record("bench_manifest", {"profile": profile, "dry_run": args.dry_run, "arguments": vars(args),
                                   "torque_source": "motor_reported", "physical_torque_validated": False})

    print(f"[run] {name}  model={args.model}  id={args.can_id}  bus={args.bus}  "
          f"routine={args.routine}  dur={dur:.1f}s  {'(DRY)' if args.dry_run else ''}")
    try:
        bus = _make_bus(joint, args.dry_run, profile=profile, trace=trace)
        rows = run_profile(bus, cmd_fn, dur, ext=ext, loop_hz=args.loop_hz, csv_path=csv_path)
    except BaseException as exc:
        trace.record("bench_end", {"status": "aborted", "reason": str(exc)})
        raise
    else:
        trace.record("bench_end", {"status": "recorded", "rows": len(rows),
                                   "last_loop_time_s": rows[-1]["t"] if rows else None,
                                   "requested_duration_s": dur})
    finally:
        try:
            ext.close()
        finally:
            trace.close()

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
