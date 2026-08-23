#!/usr/bin/env python3
"""minerva_gravity_calibrate.py — gravity + friction sweep calibration for the
Minerva RobStride arms.

Drives each requested arm joint SLOWLY across its travel (both directions, at two
speeds) while the other joints hold a fixed base pose, records the measured motor
torque vs. angle, and fits the per-joint holding-torque model

    tau(theta, thetadot) = A*sin(theta) + B*cos(theta) + C   (gravity)
                         + fc*sign(thetadot) + fv*thetadot     (friction)

(see control/minerva_gravity.py). The result is written to
config/minerva_gravity.json for the collector to feed forward via the arm_joints
`torques` field — cancelling the "slow going up, fast coming down" gravity droop.

Two sweep speeds are used so viscous drag (fv, speed-proportional) separates from
Coulomb friction (fc, speed-independent). The full 7-joint pose is stored with
every sample so a coupled (multi-joint) model can be fit later without
re-collecting.

TOPOLOGY (Path A — one motor_control instance per arm, one CAN bus per arm),
matching minerva_calibrate.py and hardware_minerva_{left,right}.yaml. Each
instance's arm group is 7 joints: j1..j6 + gripper.

SAFETY (mirrors the idle-first rule in the Minerva bring-up notes):
  * Reads telemetry and shows the live pose BEFORE enabling anything.
  * Requires an explicit ENTER before every autonomous motion phase.
  * Ramps are delta-clamped and slow; commands stream continuously so the
    500 ms motor watchdog never trips.
  * Q aborts instantly and DISABLES the arm. Temperature is watched throughout.
  * Only the arm under calibration is enabled; the other arm is never commanded.

Usage:
    # dry-run the motion plan (no hardware, prints the sweep schedule)
    python python/scripts/minerva_gravity_calibrate.py --arm left --dry-run

    # calibrate the left arm's gravity joints (j2,j3,j4) on hardware
    python python/scripts/minerva_gravity_calibrate.py --arm left --host 10.42.0.1

    # both arms, custom joint set
    python python/scripts/minerva_gravity_calibrate.py --arm both --joints j2,j3,j4,j5

Controls:  ENTER = proceed   Q = abort (disables the arm)
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # python/ -> common.*, control.*
from common.arm_constants import setup_keyboard
from common.wire import pack_msg, unpack_msg
from common.minerva_constants import KP, KD, JOINT_LIMITS, MINERVA_JOINTS
from collect_minerva_app.config import resolve_jetson_host
from control.minerva_gravity import JointGravityFit, MinervaGravityModel, fit_joint

# --- Arm layout: instance arm group order (must match minerva_calibrate.py) ---
JOINT_SUFFIXES = ["j1", "j2", "j3", "j4", "j5", "j6", "gripper"]   # arm-local 0..6
N_ARM = len(JOINT_SUFFIXES)

DEFAULT_HOST = "10.42.0.1"
DEFAULT_PORTS = {"left": (5555, 5556), "right": (5575, 5576)}      # (cmd, telem)
DEFAULT_OUTPUT = "config/minerva_gravity.json"
CALIB_LIMITS = "config/minerva_calibration.json"                  # measured min/max travel

# --- Sweep / motion parameters ---
LOOP_HZ = 50
RAMP_SPEED = 0.25          # rad/s for point-to-point repositioning ramps
SWEEP_SPEEDS = (0.10, 0.22)  # rad/s sweep speeds (slow + medium -> separates fc/fv)
LIMIT_MARGIN = 0.12       # rad shrink from each measured limit (don't slam ends)
SETTLE_S = 1.0            # hold at base pose before sweeping
TEMP_LIMIT_C = 70.0      # abort if any arm motor exceeds this
DEFAULT_JOINTS = ["j2", "j3", "j4"]   # dominant gravity (pitch) joints

# Set by --yes: skip the interactive ENTER gates so the sweep can run headless
# (e.g. driven from an automation harness). Motion, temp guards and clamps are
# unchanged — only the confirmation prompts are auto-accepted.
ASSUME_YES = False


def _ansi_on() -> None:
    """Enable ANSI escape processing on classic Windows consoles (for the live
    cursor-rewrite display). No-op elsewhere / on already-capable terminals."""
    if sys.platform == "win32":
        try:
            import ctypes
            k32 = ctypes.windll.kernel32
            k32.SetConsoleMode(k32.GetStdHandle(-11), 7)
        except Exception:
            pass


def joint_name(side: str, suffix: str) -> str:
    return f"{side}_gripper" if suffix == "gripper" else f"{side}_arm_{suffix}"


def arm_joint_names(side: str) -> List[str]:
    return [joint_name(side, s) for s in JOINT_SUFFIXES]


def canonical_index(side: str, local_i: int) -> int:
    """Arm-local index (0..6) -> canonical 17-vector index."""
    return local_i if side == "left" else 7 + local_i


def arm_gains(side: str) -> Tuple[List[float], List[float]]:
    """Per-joint (kp, kd) for this arm's 7 joints, from minerva_constants."""
    lo = 0 if side == "left" else 7
    return list(KP[lo:lo + N_ARM]), list(KD[lo:lo + N_ARM])


# ---------------------------------------------------------------------------
# Measured travel limits
# ---------------------------------------------------------------------------
def load_measured_limits(side: str) -> Dict[str, Tuple[float, float]]:
    """{arm-local suffix: (min_rad, max_rad)} from minerva_calibration.json, falling
    back to the seeded JOINT_LIMITS when a joint is missing/zero."""
    out: Dict[str, Tuple[float, float]] = {}
    data = {}
    p = Path(CALIB_LIMITS)
    if p.exists():
        try:
            data = json.loads(p.read_text()).get("joints", {})
        except Exception:
            data = {}
    for local_i, suffix in enumerate(JOINT_SUFFIXES):
        name = joint_name(side, suffix)
        ci = canonical_index(side, local_i)
        lo_def, hi_def = float(JOINT_LIMITS[ci, 0]), float(JOINT_LIMITS[ci, 1])
        j = data.get(name, {})
        # min_rad/max_rad may be captured in either order (operator moved to "min"
        # then "max"); normalise FIRST, then decide if the span is usable.
        a, b = float(j.get("min_rad", 0.0)), float(j.get("max_rad", 0.0))
        lo, hi = min(a, b), max(a, b)
        if hi - lo < 0.2:            # missing / degenerate -> use seeded limits
            lo, hi = min(lo_def, hi_def), max(lo_def, hi_def)
        out[suffix] = (lo, hi)
    return out


# ---------------------------------------------------------------------------
# Telemetry (msgpack SUB) — caches position/velocity/torque/temperature
# ---------------------------------------------------------------------------
class TelemetryReader:
    FIELDS = ("position", "velocity", "torque", "temperature")

    def __init__(self, ctx: zmq.Context, address: str, joints: List[str]) -> None:
        self._joints = joints
        self._lock = threading.Lock()
        self._cache: Dict[str, Dict[str, float]] = {}
        self._t = 0.0
        self._sock = ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.connect(address)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._sock.close()

    def snapshot(self) -> Tuple[Optional[Dict[str, Dict[str, float]]], float]:
        with self._lock:
            return (dict(self._cache) if self._cache else None), self._t

    def field(self, field: str) -> Optional[np.ndarray]:
        with self._lock:
            if not self._cache or not all(j in self._cache for j in self._joints):
                return None
            return np.array([self._cache[j].get(field, 0.0) for j in self._joints],
                            dtype=np.float64)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                buf = self._sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.005)
                continue
            except Exception:
                break
            try:
                motors = unpack_msg(buf).get("motors", {})
                new = {}
                for j in self._joints:
                    m = motors.get(j)
                    if isinstance(m, dict):
                        new[j] = {f: float(m.get(f, 0.0)) for f in self.FIELDS}
                if new:
                    with self._lock:
                        self._cache.update(new)
                        self._t = time.time()
            except Exception:
                pass


def send_command(ctx: zmq.Context, address: str, cmd: dict) -> None:
    """One-shot PUSH (enable/disable) on a fresh socket that lingers to flush."""
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.LINGER, 1000)
    sock.connect(address)
    time.sleep(0.1)
    sock.send(pack_msg(cmd))
    sock.close()


# ---------------------------------------------------------------------------
# Motion engine — one synchronous loop owns the command stream + telem drain
# ---------------------------------------------------------------------------
class ArmMotion:
    """Streams arm_joints at LOOP_HZ for ONE arm instance. Holds all 7 joints via
    PD; the caller nudges the command vector. torques feedforward is always 0
    here — we are MEASURING the raw holding torque, not compensating it."""

    def __init__(self, ctx: zmq.Context, side: str, cmd_addr: str,
                 telem: TelemetryReader, get_key) -> None:
        self.side = side
        self.joints = arm_joint_names(side)
        self.telem = telem
        self.get_key = get_key
        self.kp, self.kd = arm_gains(side)
        self._sock = ctx.socket(zmq.PUSH)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.connect(cmd_addr)
        self._period = 1.0 / LOOP_HZ

    def _send(self, cmd_pos: np.ndarray) -> None:
        msg = {
            "type": "arm_joints",
            "positions": [float(x) for x in cmd_pos],
            "velocities": [0.0] * N_ARM,
            "kp": self.kp, "kd": self.kd,
            "torques": [0.0] * N_ARM,
        }
        try:
            self._sock.send(pack_msg(msg), zmq.NOBLOCK)
        except Exception:
            pass

    def read_positions(self, retries: int = 60) -> Optional[np.ndarray]:
        for _ in range(retries):
            p = self.telem.field("position")
            if p is not None:
                return p
            time.sleep(0.03)
        return None

    def hot(self) -> Optional[float]:
        t = self.telem.field("temperature")
        return float(np.max(t)) if t is not None else None

    def ramp_to(self, target: np.ndarray, *, speed: float = RAMP_SPEED,
                timeout: float = 30.0) -> Optional[np.ndarray]:
        """Delta-clamped ramp of ALL joints to `target`. Returns actual pos, or
        None on Q-abort / no telemetry."""
        cur = self.read_positions()
        if cur is None:
            print("  ERROR: no telemetry — refusing to move (would jump).")
            return None
        cmd = cur.copy()
        step = speed / LOOP_HZ
        t_end = time.time() + timeout
        while time.time() < t_end:
            t0 = time.time()
            if self.get_key() == "Q":
                return None
            cmd = cmd + np.clip(target - cmd, -step, step)
            self._send(cmd)
            pos = self.telem.field("position")
            if pos is not None:
                cur = pos
            if np.all(np.abs(cmd - target) < 1e-3) and np.max(np.abs(cur - target)) < 0.05:
                return cur
            _sleep_to(t0, self._period)
        return cur

    def hold(self, target: np.ndarray, seconds: float) -> bool:
        """PD-hold `target` for `seconds`. False on Q-abort."""
        t_end = time.time() + seconds
        while time.time() < t_end:
            t0 = time.time()
            if self.get_key() == "Q":
                return False
            self._send(target)
            _sleep_to(t0, self._period)
        return True


def _sleep_to(t0: float, period: float) -> None:
    dt = period - (time.time() - t0)
    if dt > 0:
        time.sleep(dt)


# ---------------------------------------------------------------------------
# The sweep
# ---------------------------------------------------------------------------
def sweep_joint(motion: ArmMotion, local_i: int, base: np.ndarray,
                lo: float, hi: float) -> Optional[Dict[str, np.ndarray]]:
    """Sweep joint `local_i` lo->hi->lo at each SWEEP_SPEED while the other joints
    hold `base`. Records (theta, thetadot, tau) of the swept joint. None on abort.
    """
    name = motion.joints[local_i]
    theta_s: List[float] = []
    thetadot_s: List[float] = []
    tau_s: List[float] = []
    pose_s: List[List[float]] = []

    # start at lo
    start = base.copy(); start[local_i] = lo
    if motion.ramp_to(start) is None:
        return None
    if not motion.hold(start, 0.5):
        return None

    step_period = 1.0 / LOOP_HZ
    cmd = start.copy()
    last_theta: Optional[float] = None
    last_t: Optional[float] = None
    _lines = 0

    for speed in SWEEP_SPEEDS:
        step = speed / LOOP_HZ
        for direction in (+1.0, -1.0):
            target = hi if direction > 0 else lo
            while True:
                t0 = time.time()
                if motion.get_key() == "Q":
                    print()
                    return None
                # advance the swept joint toward this leg's target
                nxt = cmd[local_i] + direction * step
                cmd[local_i] = min(hi, max(lo, nxt))
                motion._send(cmd)

                snap, _ = motion.telem.snapshot()
                if snap and name in snap:
                    theta = snap[name].get("position", 0.0)
                    tau = snap[name].get("torque", 0.0)
                    vel = snap[name].get("velocity", float("nan"))
                    now = time.time()
                    if not np.isfinite(vel) and last_theta is not None and last_t is not None:
                        dt = now - last_t
                        vel = (theta - last_theta) / dt if dt > 1e-4 else 0.0
                    last_theta, last_t = theta, now
                    theta_s.append(theta); thetadot_s.append(float(vel)); tau_s.append(tau)
                    pose_s.append([snap.get(j, {}).get("position", 0.0) for j in motion.joints])

                    if _lines:
                        sys.stdout.write(f"\033[{_lines}A")
                    hot = motion.hot()
                    _lines = 1
                    print(f"  sweep {name} @ {speed:.2f} rad/s dir={'+' if direction>0 else '-'}"
                          f"  theta={theta:+.3f}  tau={tau:+7.2f} Nm  "
                          f"n={len(theta_s)}  maxT={hot:.0f}C\033[K" if hot is not None else
                          f"  sweep {name} @ {speed:.2f} dir={'+' if direction>0 else '-'}"
                          f"  theta={theta:+.3f}  tau={tau:+7.2f}  n={len(theta_s)}\033[K")
                    sys.stdout.flush()
                    if hot is not None and hot > TEMP_LIMIT_C:
                        print(f"\n  ABORT: motor temperature {hot:.0f}C exceeds {TEMP_LIMIT_C}C")
                        return None

                # reached this leg's endpoint (commanded) — move to next leg
                if abs(cmd[local_i] - target) < 1e-6:
                    break
                _sleep_to(t0, step_period)
    print()
    return {
        "theta": np.array(theta_s), "thetadot": np.array(thetadot_s),
        "tau": np.array(tau_s), "pose": np.array(pose_s),
    }


# ---------------------------------------------------------------------------
# Per-arm procedure
# ---------------------------------------------------------------------------
def calibrate_arm(ctx: zmq.Context, side: str, cmd_addr: str, telem_addr: str,
                  suffixes: List[str], base_mode: str, get_key,
                  raw_store: dict) -> Optional[Dict[int, JointGravityFit]]:
    joints = arm_joint_names(side)
    limits = load_measured_limits(side)
    print("\n" + "#" * 60)
    print(f"#  {side.upper()} ARM   cmd={cmd_addr}")
    print(f"#  sweeping: {', '.join(suffixes)}")
    print("#" * 60)

    telem = TelemetryReader(ctx, telem_addr, joints)
    telem.start()
    motion: Optional[ArmMotion] = None
    enabled = False
    try:
        # idle-first: observe current state before enabling anything
        print("\nReading telemetry (motors still DISABLED)...")
        p = None
        for _ in range(100):
            p = telem.field("position")
            if p is not None:
                break
            time.sleep(0.05)
        if p is None:
            print("  no telemetry — check the instance / CAN bus. Skipping this arm.")
            return None
        print("  current pose:")
        for i, j in enumerate(joints):
            print(f"    {j:<16} {p[i]:+.4f} rad")

        # base pose (held joints) — mid-range, or the current pose
        base = np.array([(limits[s][0] + limits[s][1]) / 2.0 for s in JOINT_SUFFIXES])
        if base_mode == "current":
            base = p.copy()
        # keep the gripper where it is regardless
        base[JOINT_SUFFIXES.index("gripper")] = p[JOINT_SUFFIXES.index("gripper")]

        print(f"\n  base pose (held joints, {base_mode}):")
        for i, j in enumerate(joints):
            print(f"    {j:<16} {base[i]:+.4f} rad")
        print("\n  sweep plan:")
        for s in suffixes:
            lo, hi = limits[s]
            lo += LIMIT_MARGIN; hi -= LIMIT_MARGIN
            print(f"    {joint_name(side, s):<16} {lo:+.3f} -> {hi:+.3f} rad  "
                  f"@ {', '.join(f'{v}' for v in SWEEP_SPEEDS)} rad/s (both dirs)")

        print("\n  *** Ensure the workspace around the arm is CLEAR. ***")
        print("  The arm will move autonomously. Keep a hand near E-stop.")
        if not _confirm("  Press ENTER to ENABLE + calibrate this arm (Q=skip): ", get_key):
            return None

        send_command(ctx, cmd_addr, {"type": "enable", "motor_ids": joints})
        enabled = True
        time.sleep(1.0)
        motion = ArmMotion(ctx, side, cmd_addr, telem, get_key)

        hot = motion.hot()
        if hot is not None and hot > TEMP_LIMIT_C:
            print(f"  ABORT: motors already at {hot:.0f}C (> {TEMP_LIMIT_C}C). Let them cool.")
            return None

        print("\n  Moving to base pose...")
        if motion.ramp_to(base) is None:
            return None
        if not motion.hold(base, SETTLE_S):
            return None

        fits: Dict[int, JointGravityFit] = {}
        for s in suffixes:
            local_i = JOINT_SUFFIXES.index(s)
            lo, hi = limits[s]
            lo += LIMIT_MARGIN; hi -= LIMIT_MARGIN
            print(f"\n--- sweeping {joint_name(side, s)}  [{lo:+.3f}, {hi:+.3f}] ---")
            data = sweep_joint(motion, local_i, base, lo, hi)
            if data is None:
                print("  aborted during sweep.")
                return None
            ci = canonical_index(side, local_i)
            try:
                fit = fit_joint(ci, data["theta"], data["thetadot"], data["tau"])
            except ValueError as exc:
                print(f"  fit FAILED: {exc}")
                continue
            fits[ci] = fit
            raw_store[MINERVA_JOINTS[ci]] = {
                k: data[k].tolist() for k in ("theta", "thetadot", "tau")
            }
            print(f"  fit: A={fit.A:+.3f} B={fit.B:+.3f} C={fit.C:+.3f}  "
                  f"fc={fit.fc:.3f} fv={fit.fv:+.3f}  R2={fit.r2:.4f} "
                  f"rms={fit.rms_nm:.3f} Nm  (n={fit.n_samples})")

        print("\n  Returning to base, then disabling...")
        motion.ramp_to(base)
        return fits
    finally:
        if enabled:
            try:
                send_command(ctx, cmd_addr, {"type": "disable", "motor_ids": joints})
                print(f"  {side} motors disabled.")
            except Exception as exc:
                print(f"  [warn] could not disable {side}: {exc}")
        telem.stop()


def _confirm(prompt: str, get_key) -> bool:
    """Block for ENTER (proceed) or Q (abort). Works with the raw-key reader.
    With --yes (ASSUME_YES) it auto-proceeds without waiting for a keypress."""
    if ASSUME_YES:
        print(prompt + "[auto-yes]")
        return True
    print(prompt, end="", flush=True)
    while True:
        k = get_key()
        if k in ("\r", "\n", " "):
            print()
            return True
        if k == "Q":
            print(" [skip]")
            return False
        time.sleep(0.02)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["left", "right", "both"], default="left")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--joints", default=",".join(DEFAULT_JOINTS),
                    help="comma list of arm-local suffixes to sweep (default j2,j3,j4)")
    ap.add_argument("--base", choices=["mid", "current"], default="mid",
                    help="held-joint pose during each sweep (default mid-range)")
    for side in ("left", "right"):
        ap.add_argument(f"--{side}-cmd", default=None)
        ap.add_argument(f"--{side}-telem", default=None)
    ap.add_argument("--output", default=DEFAULT_OUTPUT)
    ap.add_argument("--yes", action="store_true",
                    help="skip the interactive ENTER gates (headless / automated runs)")
    ap.add_argument("--fresh", action="store_true",
                    help="ignore any existing calibration; do NOT merge with it")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the sweep plan without touching hardware")
    args = ap.parse_args()

    global ASSUME_YES
    ASSUME_YES = bool(args.yes)

    suffixes = [s.strip() for s in args.joints.split(",") if s.strip()]
    bad = [s for s in suffixes if s not in JOINT_SUFFIXES or s == "gripper"]
    if bad:
        ap.error(f"invalid --joints {bad}; choose from j1..j6")

    # Dry-run stays fully offline (no network probe); live runs resolve the Jetson.
    host = args.host if args.dry_run else resolve_jetson_host(args.host)
    sides = ["left", "right"] if args.arm == "both" else [args.arm]

    def ep(side: str, idx: int, override: Optional[str]) -> str:
        return override or f"tcp://{host}:{DEFAULT_PORTS[side][idx]}"

    endpoints = {s: (ep(s, 0, getattr(args, f"{s}_cmd")),
                     ep(s, 1, getattr(args, f"{s}_telem"))) for s in sides}

    print("=" * 60)
    print("Minerva gravity + friction sweep calibration")
    print(f"  arms={sides}  joints={suffixes}  base={args.base}")
    for s in sides:
        print(f"  {s}: cmd={endpoints[s][0]}  telem={endpoints[s][1]}")
    print("=" * 60)

    if args.dry_run:
        for s in sides:
            limits = load_measured_limits(s)
            print(f"\n[{s}] sweep plan:")
            for suffix in suffixes:
                lo, hi = limits[suffix]
                lo += LIMIT_MARGIN; hi -= LIMIT_MARGIN
                span = hi - lo
                # est. duration: both dirs at each speed
                dur = sum(2 * span / v for v in SWEEP_SPEEDS)
                print(f"  {joint_name(s, suffix):<16} [{lo:+.3f}, {hi:+.3f}] "
                      f"span={span:.2f} rad  ~{dur:.0f}s")
        print("\n(dry run — no hardware contacted)")
        return

    _ansi_on()
    get_key = setup_keyboard()
    ctx = zmq.Context()
    all_fits: Dict[int, JointGravityFit] = {}
    raw_store: dict = {}
    try:
        for s in sides:
            fits = calibrate_arm(ctx, s, endpoints[s][0], endpoints[s][1],
                                 suffixes, args.base, get_key, raw_store)
            if fits:
                all_fits.update(fits)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ctx.term()

    if not all_fits:
        print("\nNo joints calibrated — nothing saved.")
        return

    # Merge with any existing calibration so arms/joints run in SEPARATE invocations
    # accumulate into one file (this run's joints overwrite same-named ones). --fresh
    # opts out. Preserves the other arm's fits + raw data.
    outp = Path(args.output)
    merged_fits: Dict[int, JointGravityFit] = {}
    merged_raw: dict = {}
    if outp.exists() and not args.fresh:
        try:
            merged_fits.update(MinervaGravityModel.from_json(outp).fits)
            prev = json.loads(outp.read_text())
            merged_raw.update((prev.get("meta") or {}).get("raw", {}) or {})
            kept = [MINERVA_JOINTS[i] for i in merged_fits if i not in all_fits]
            if kept:
                print(f"\nMerging with existing {outp} (keeping: {', '.join(kept)})")
        except Exception as exc:
            print(f"\n[warn] could not read existing {outp} to merge: {exc}")
    merged_fits.update(all_fits)
    merged_raw.update(raw_store)

    model = MinervaGravityModel(merged_fits)
    print("\n" + "=" * 60)
    print("Identified model (all joints in file):")
    print(model.summary())
    print("=" * 60)
    model.to_json(args.output, meta={"host": host, "base_mode": args.base,
                                     "sweep_speeds": list(SWEEP_SPEEDS),
                                     "raw": merged_raw})
    print(f"\nSaved -> {args.output}")
    print("Feed it forward from the collector with --grav-comp (loads this file).")


if __name__ == "__main__":
    main()
