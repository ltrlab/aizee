#!/usr/bin/env python3
"""minerva_teleop.py — dual-GELLO teleop + zeroing for the two Minerva arms.

The arms-only Minerva analog of collect_demo.py: drives both 7-DoF Minerva arms
from two OpenRB-150 (GELLO) leaders, with the SAME control vocabulary and the
SAME two zero functions collect_demo exposes. Each follower arm is its own
motor_control instance / CAN bus / ZMQ port pair (Path A — see
config/hardware_minerva_{left,right}.yaml).

Controls (IDLE-FIRST — always Idle to read state + zero before applying gains):
    I   idle    -> enable at ZERO torque (backdrive; read true positions). ALWAYS FIRST.
    E   enable  -> gains: TRACKING if leaders else HOLD. ONLY from IDLE.
    H   toggle  -> HOLD <-> TRACKING (IDLE->HOLD)
    Z   leader zero  -> snapshot each leader pose as its zero; save to calib
    M   mirror       -> set each leader zero so the leader maps to the arm's
                        current actual pose; save to calib
    K   RobStride mechanical zero -> CAN ZeroPos + SaveConfig to BOTH arms
                        (arms must be DISABLED first — press X, then K)
    P   save current pose as ready pose (config/minerva_ready_pose.json)
    X   soft shutdown -> ramp both arms to zero, then disable
    Q   quit

Leader mapping is ABSOLUTE (collect_demo model):
    target = direction * (leader_rad - zero)      # per joint, per arm
so the two zeros are how you register leader<->follower. A per-step velocity
guard bounds every command, so engaging never snaps.

Safety:
  * targets clamp to the CALIBRATED follower range (config/minerva_calibration.json)
    when present, else only the motor_control config soft-limits;
  * KP is scaled DOWN by default (--kp-scale 0.3) for first motion.

Usage:
    python python/scripts/minerva_teleop.py --host 192.168.0.27
    python python/scripts/minerva_teleop.py --arm left --left-port COM7
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import zmq

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))                       # python/scripts -> minerva_calibrate
sys.path.insert(0, str(_HERE.parent))                # python/        -> common.*
sys.path.insert(0, str(_HERE.parent / "teleop"))     # python/teleop  -> openrb_leader

from common.arm_constants import setup_keyboard
from common.wire import pack_msg
from common.minerva_constants import KP as _KP17, KD as _KD17
from minerva_calibrate import (
    JOINT_SPECS, arm_joint_names, TelemetryReader, DEFAULT_HOST, DEFAULT_PORTS,
)
from collect_minerva_app.config import resolve_jetson_host

# Per-arm 7-vector gains = arm(6)+gripper(1) slice of the Minerva gain table.
ARM_KP = np.asarray(_KP17[0:7], dtype=np.float32)
ARM_KD = np.asarray(_KD17[0:7], dtype=np.float32)
# Per-step travel cap. LOOP_HZ*cap = max slew: 60*0.10 = 6 rad/s (arm).
MAX_DELTA = np.array([0.10] * 6 + [0.20], dtype=np.float32)
SHUTDOWN_STEP = 0.02          # rad/tick ramp toward zero on soft shutdown
LOOP_HZ = 60

DISABLED, IDLE, HOLD, TRACK, SHUTDOWN = "DISABLED", "IDLE", "HOLD", "TRACK", "SHUTDOWN"


def _import_openrb():
    try:
        from openrb_leader import OpenRBLeader, find_openrb_port
        return OpenRBLeader, find_openrb_port
    except Exception as e:
        print(f"[teleop] OpenRB leader unavailable ({e}) — running without leaders")
        return None, None


def load_limits(path: Path, side: str) -> Optional[np.ndarray]:
    """[7,2] (min,max) clamp array for *side* from the calibration JSON, or None."""
    if not path.exists():
        return None
    data = json.loads(path.read_text()).get("joints", {})
    out = []
    for suffix, _, _ in JOINT_SPECS:
        name = f"{side}_gripper" if suffix == "gripper" else f"{side}_arm_{suffix}"
        j = data.get(name)
        if j is None:
            return None
        mn, mx = float(j["min_rad"]), float(j["max_rad"])
        out.append((min(mn, mx), max(mn, mx)))   # calib may store min>max
    return np.asarray(out, dtype=np.float32)


class ArmLink:
    """One follower arm (its own instance/bus/ports) + its optional GELLO leader.

    Holds the absolute leader->follower registration (zero + direction) so the
    Z/M zero commands can update it live, mirroring collect_demo."""

    def __init__(self, ctx: zmq.Context, side: str, cmd_addr: str, telem_addr: str,
                 leader, limits: Optional[np.ndarray], kp_scale: float) -> None:
        self.side = side
        self.joints = arm_joint_names(side)
        self.n = len(self.joints)
        self.leader = leader
        self.limits = limits
        self.kp = ARM_KP * kp_scale
        self.kd = ARM_KD
        self.mode = DISABLED
        self.hold_target: Optional[np.ndarray] = None
        # Absolute-mapping registration, seeded from the leader's calib.
        if leader is not None:
            self.zero = np.asarray(leader.zero_offsets, dtype=np.float32).copy()
            self.dirs = np.asarray(leader.directions, dtype=np.float32).copy()
        else:
            self.zero = np.zeros(self.n, dtype=np.float32)
            self.dirs = np.ones(self.n, dtype=np.float32)
        self.telem = TelemetryReader(ctx, telem_addr, self.joints)
        self.telem.start()
        self._push = ctx.socket(zmq.PUSH)
        self._push.setsockopt(zmq.LINGER, 0)
        self._push.connect(cmd_addr)

    # -- reads -----------------------------------------------------------
    def qpos(self) -> Optional[np.ndarray]:
        pos = self.telem.get()
        if not pos or not all(j in pos for j in self.joints):
            return None
        return np.array([pos[j] for j in self.joints], dtype=np.float32)

    def leader_rad(self) -> Optional[np.ndarray]:
        if self.leader is None:
            return None
        r = self.leader.poll()
        return None if r is None else np.asarray(r, dtype=np.float32)[: self.n]

    def leader_target(self) -> Optional[np.ndarray]:
        lr = self.leader_rad()
        return None if lr is None else (self.dirs * (lr - self.zero)).astype(np.float32)

    # -- lifecycle -------------------------------------------------------
    def enable(self) -> None:
        self._cmd({"type": "enable", "motor_ids": self.joints})
        time.sleep(0.05)
        q = self.qpos()
        self.hold_target = q.copy() if q is not None else np.zeros(self.n, np.float32)
        self.mode = TRACK if self.leader is not None else HOLD

    def idle(self) -> None:
        self._cmd({"type": "enable", "motor_ids": self.joints})
        self.mode = IDLE

    def hold_here(self) -> None:
        q = self.qpos()
        if q is not None:
            self.hold_target = q.copy()
        self.mode = HOLD

    def toggle_hold(self) -> None:
        if self.mode in (TRACK,):
            self.hold_here()
        elif self.mode == HOLD:
            self.mode = TRACK if self.leader is not None else IDLE
        elif self.mode == IDLE:
            self.hold_here()

    def disable(self) -> None:
        self.mode = DISABLED
        self.hold_target = None
        self._cmd({"type": "disable", "motor_ids": self.joints})

    def begin_shutdown(self) -> None:
        if self.mode in (IDLE, HOLD, TRACK):
            q = self.qpos()
            self.hold_target = q.copy() if q is not None else np.zeros(self.n, np.float32)
            self.mode = SHUTDOWN

    # -- zero functions (collect_demo parity) ----------------------------
    def mech_zero(self) -> str:
        """RobStride hardware mechanical zero + SaveConfig. Requires DISABLED."""
        if self.mode != DISABLED:
            return f"{self.side}: disable first (X) before K"
        self._cmd({"type": "mech_zero", "motor_ids": self.joints, "save": True})
        return f"{self.side}: mech_zero sent (saved to flash)"

    def leader_zero(self) -> str:
        """Capture the current leader pose as this leader's zero; persist."""
        if self.leader is None:
            return f"{self.side}: no leader"
        lr = self.leader_rad()
        if lr is None:
            return f"{self.side}: no leader reading"
        self.zero = lr.copy()
        try:
            self.leader.save_zero(self.zero)
        except Exception as e:
            return f"{self.side}: zero set (save failed: {e})"
        return f"{self.side}: leader zeroed + saved"

    def mirror(self) -> str:
        """Set the leader zero so the leader maps to the arm's actual pose."""
        if self.leader is None:
            return f"{self.side}: no leader"
        lr = self.leader_rad()
        q = self.qpos()
        if lr is None or q is None:
            return f"{self.side}: need leader + telemetry to mirror"
        # target = dirs*(lr - zero) == q  =>  zero = lr - dirs*q  (dirs == ±1)
        self.zero = (lr - self.dirs * q).astype(np.float32)
        try:
            self.leader.save_zero(self.zero)
        except Exception as e:
            return f"{self.side}: mirrored (save failed: {e})"
        return f"{self.side}: mirrored + saved"

    def ready_pose(self) -> Optional[Dict[str, float]]:
        q = self.qpos()
        return None if q is None else {j: float(q[i]) for i, j in enumerate(self.joints)}

    # -- per-tick command ------------------------------------------------
    def step(self) -> None:
        if self.mode == DISABLED:
            return
        q = self.qpos()
        if q is None:
            return
        if self.mode == IDLE:
            # zero-torque backdrive: feed watchdog, apply no PD.
            self._send_arm(q, zero_gain=True)
            return
        if self.mode == TRACK:
            tgt = self.leader_target()
            if tgt is None:
                tgt = self.hold_target if self.hold_target is not None else q
        elif self.mode == SHUTDOWN:
            tgt = self._ramp_to_zero(self.hold_target if self.hold_target is not None else q)
            if np.all(np.abs(q) < 0.05) and np.all(np.abs(tgt) < 0.02):
                self.disable()
                return
        else:  # HOLD
            tgt = self.hold_target if self.hold_target is not None else q
        if self.limits is not None:
            tgt = np.clip(tgt, self.limits[:, 0], self.limits[:, 1])
        tgt = (q + np.clip(tgt - q, -MAX_DELTA, MAX_DELTA)).astype(np.float32)
        if self.mode == SHUTDOWN:
            self.hold_target = tgt
        self._send_arm(tgt, zero_gain=False)

    def _ramp_to_zero(self, tgt: np.ndarray) -> np.ndarray:
        out = tgt.copy()
        for i in range(self.n):
            out[i] = 0.0 if abs(out[i]) < SHUTDOWN_STEP else out[i] - np.sign(out[i]) * SHUTDOWN_STEP
        return out

    def _send_arm(self, tgt: np.ndarray, *, zero_gain: bool) -> None:
        self._cmd({
            "type": "arm_joints",
            "positions": tgt.astype(np.float32).tolist(),
            "velocities": [0.0] * self.n,
            "kp": ([0.0] * self.n) if zero_gain else self.kp.tolist(),
            "kd": ([0.0] * self.n) if zero_gain else self.kd.tolist(),
            "torques": [0.0] * self.n,
        })

    def _cmd(self, cmd: dict) -> None:
        try:
            self._push.send(pack_msg(cmd), zmq.NOBLOCK)
        except Exception:
            pass

    def close(self) -> None:
        self.telem.stop()
        self._push.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["left", "right", "both"], default="both")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--left-port", default=None, help="left OpenRB serial port (auto if omitted)")
    ap.add_argument("--right-port", default=None, help="right OpenRB serial port (auto if omitted)")
    ap.add_argument("--left-calib", default="config/openrb_left.json")
    ap.add_argument("--right-calib", default="config/openrb_right.json")
    ap.add_argument("--calibration", default="config/minerva_calibration.json",
                    help="follower min/max for target clamping")
    ap.add_argument("--ready-pose", default="config/minerva_ready_pose.json")
    ap.add_argument("--kp-scale", type=float, default=0.3,
                    help="scale on the arm KP (start low; raise once trusted)")
    ap.add_argument("--dry-run", action="store_true", help="never enable/command motors")
    args = ap.parse_args()

    host = resolve_jetson_host(args.host)
    sides = ["left", "right"] if args.arm == "both" else [args.arm]
    calib_path = Path(args.calibration)
    OpenRBLeader, find_openrb_port = _import_openrb()

    leaders: Dict[str, object] = {}
    if OpenRBLeader is not None:
        excl: List[str] = []
        for side in sides:
            port = getattr(args, f"{side}_port")
            if port is None:
                port = find_openrb_port(exclude=excl, verbose=True)
            if not port:
                print(f"[teleop] no {side} leader port — {side} arm has no leader")
                continue
            excl.append(port)
            calib = getattr(args, f"{side}_calib")
            try:
                dev = OpenRBLeader(port, calib=calib)
                if dev.connect():
                    leaders[side] = dev
                    print(f"[teleop] {side} leader on {port} (calib {calib})")
                else:
                    print(f"[teleop] {side} leader failed to connect on {port}")
            except Exception as e:
                print(f"[teleop] {side} leader error: {e}")

    ctx = zmq.Context()
    links: Dict[str, ArmLink] = {}
    for side in sides:
        cmd = f"tcp://{host}:{DEFAULT_PORTS[side][0]}"
        tel = f"tcp://{host}:{DEFAULT_PORTS[side][1]}"
        lim = load_limits(calib_path, side)
        links[side] = ArmLink(ctx, side, cmd, tel, leaders.get(side), lim, args.kp_scale)
        print(f"[teleop] {side}: cmd={cmd} telem={tel} "
              f"clamp={'calibrated' if lim is not None else 'config-limits-only'} "
              f"leader={'yes' if side in leaders else 'no'}")

    if not calib_path.exists():
        print(f"[teleop] NOTE: {calib_path} not found — run minerva_calibrate.py first "
              f"(targets are then clamped only by the motor_control config limits).")
    print(f"[teleop] KP x{args.kp_scale}. {'DRY-RUN.' if args.dry_run else ''}")
    print("=" * 64)
    print("  I idle=read+zero-torque (ALWAYS FIRST)   E enable-gains (from Idle only)   H hold/track   Z leader-zero   M mirror")
    print("  K robstride mech-zero (disable first)   P ready-pose   X shutdown   Q quit")
    print("=" * 64)

    get_key = setup_keyboard()
    period = 1.0 / LOOP_HZ
    msg, msg_until = "", 0.0
    last_print = 0.0

    def announce(text: str) -> None:
        nonlocal msg, msg_until
        msg, msg_until = text, time.monotonic() + 3.0
        print(f"\n{text}")

    try:
        while True:
            t0 = time.monotonic()
            key = get_key()
            if key == "Q":
                break
            elif key == "E" and not args.dry_run:   # gains (HOLD/TRACK) — ONLY from IDLE
                if all(lk.mode == IDLE for lk in links.values()):
                    for lk in links.values():
                        lk.enable()
                    announce("enabled (gains)")
                else:
                    announce("Idle first (I): read state + zero before applying gains")
            elif key == "I" and not args.dry_run:
                for lk in links.values():
                    lk.idle()
                announce("idle (zero torque)")
            elif key == "H" and not args.dry_run:
                for lk in links.values():
                    lk.toggle_hold()
                announce("hold/track toggled")
            elif key == "Z":
                announce(" | ".join(lk.leader_zero() for lk in links.values()))
            elif key == "M":
                announce(" | ".join(lk.mirror() for lk in links.values()))
            elif key == "K" and not args.dry_run:
                announce(" | ".join(lk.mech_zero() for lk in links.values()))
            elif key == "P":
                pose: Dict[str, float] = {}
                for lk in links.values():
                    rp = lk.ready_pose()
                    if rp:
                        pose.update(rp)
                if pose:
                    p = Path(args.ready_pose)
                    p.parent.mkdir(parents=True, exist_ok=True)
                    p.write_text(json.dumps({"positions": pose}, indent=2))
                    announce(f"ready pose saved -> {p}")
                else:
                    announce("no telemetry — ready pose not saved")
            elif key == "X" and not args.dry_run:
                for lk in links.values():
                    lk.begin_shutdown()
                announce("soft shutdown — ramping to zero")

            if not args.dry_run:
                for lk in links.values():
                    lk.step()

            now = time.monotonic()
            if now - last_print > 0.25:
                last_print = now
                if now > msg_until:
                    msg = ""
                bits = []
                for side, lk in links.items():
                    q = lk.qpos()
                    led = "L" if lk.leader is not None else "-"
                    qs = "n/a" if q is None else " ".join(f"{v:+.2f}" for v in q[:3])
                    bits.append(f"{side}[{led}/{lk.mode[:4]}] {qs}")
                tail = f"   {msg}" if msg else ""
                sys.stdout.write("\r" + "  ".join(bits) + tail + "\033[K")
                sys.stdout.flush()

            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nDisabling motors...")
        for lk in links.values():
            try:
                if not args.dry_run:
                    lk.disable()
            except Exception:
                pass
            lk.close()
        for dev in leaders.values():
            try:
                dev.close()
            except Exception:
                pass
        ctx.term()
        print("Done.")


if __name__ == "__main__":
    main()
