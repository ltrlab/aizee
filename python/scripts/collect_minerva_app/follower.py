"""follower.py — dual-instance (Path A) follower transport for collect_minerva.

Minerva's two arms each run their own motor_control instance on their own CAN
bus + ZMQ port pair (config/hardware_minerva_{left,right}.yaml). This module
hides that split behind ONE object so the collector's main loop keeps operating
on the canonical 17-DoF MINERVA vector:

    * telemetry from the two instances is merged into a 17-vector qpos/torque,
      with head/lift (indices 14..16) filled with zeros (no hardware yet);
    * a target 17-vector is split back into two 7-joint `arm_joints` commands
      (left = indices 0..6, right = 7..13; head/lift dropped);
    * enable / disable / mech_zero fan out to both instances.

Wire format is msgpack (common.wire), matching the live Rust node. A 100 Hz
re-emit thread keeps each follower's PD loop fed between main-loop ticks.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import zmq

from common.minerva_constants import KD, KP, MINERVA_JOINTS, NUM_MINERVA_JOINTS
from common.wire import pack_msg, unpack_msg
from collect_minerva_app.receivers import start_telem_receiver

LEFT_NAMES: List[str] = list(MINERVA_JOINTS[0:7])    # left_arm_j1..j6, left_gripper
RIGHT_NAMES: List[str] = list(MINERVA_JOINTS[7:14])   # right_arm_j1..j6, right_gripper
_ARM_KP_L = list(KP[0:7]); _ARM_KD_L = list(KD[0:7])
_ARM_KP_R = list(KP[7:14]); _ARM_KD_R = list(KD[7:14])


def _extract(msg: Optional[dict], names: List[str], field: str) -> Optional[List[float]]:
    if not msg:
        return None
    motors = msg.get("motors")
    if not isinstance(motors, dict) or not all(
            n in motors and isinstance(motors[n], dict) for n in names):
        return None
    try:
        return [float(motors[n].get(field, 0.0)) for n in names]
    except (TypeError, ValueError):
        return None


class DualArmTransport:
    """Two motor_control instances presented as one 17-DoF follower."""

    def __init__(self, ctx: zmq.Context, *, left_cmd: str, left_telem: str,
                 right_cmd: str, right_telem: str, reemit_hz: int = 100) -> None:
        self._ctx = ctx
        self._push = {
            "left": ctx.socket(zmq.PUSH),
            "right": ctx.socket(zmq.PUSH),
        }
        for side, addr in (("left", left_cmd), ("right", right_cmd)):
            self._push[side].setsockopt(zmq.LINGER, 0)
            self._push[side].connect(addr)
        self._send_lock = {"left": threading.Lock(), "right": threading.Lock()}
        # telemetry receivers (background SUB threads, msgpack)
        ls, lt, ll, lc = start_telem_receiver(ctx, left_telem)
        rs, rt, rl, rc = start_telem_receiver(ctx, right_telem)
        self._telem = {
            "left": (ls, lt, ll, lc),
            "right": (rs, rt, rl, rc),
        }
        # re-emit holder: latest per-arm arm_joints bundle
        self._holder_lock = threading.Lock()
        self._holder: Dict[str, Optional[dict]] = {"left": None, "right": None}
        self._stop = threading.Event()
        period = 1.0 / max(reemit_hz, 1)

        def _reemit() -> None:
            nxt = time.perf_counter() + period
            while not self._stop.is_set():
                with self._holder_lock:
                    bundles = dict(self._holder)
                for side, b in bundles.items():
                    if b is not None:
                        self._raw_send(side, b)
                dt = nxt - time.perf_counter()
                if dt > 0:
                    self._stop.wait(dt)
                nxt = max(nxt + period, time.perf_counter() + period)

        self._reemit = threading.Thread(target=_reemit, daemon=True, name="MinervaCmdTx")
        self._reemit.start()

    # -- telemetry -------------------------------------------------------
    def _msg(self, side: str) -> Tuple[Optional[dict], float]:
        _, _, lock, cache = self._telem[side]
        with lock:
            return cache.get("msg"), cache.get("time", 0.0)

    _STALE_S = 0.5   # an arm quiet longer than this counts as absent

    def _arm_field(self, side: str, field: str) -> Optional[List[float]]:
        """7 values for `side`, or None if the arm is STALE (bus down → not
        publishing fresh frames) OR its motors aren't all present in the latest
        frame. Covers both drop modes so a dropped arm is always detected."""
        msg, t = self._msg(side)
        if not t or (time.time() - t) > self._STALE_S:
            return None
        names = LEFT_NAMES if side == "left" else RIGHT_NAMES
        return _extract(msg, names, field)

    def _merged(self, field: str) -> Optional[np.ndarray]:
        l = self._arm_field("left", field)
        r = self._arm_field("right", field)
        if l is None or r is None:
            return None
        return np.array(l + r + [0.0, 0.0, 0.0], dtype=np.float32)   # head/lift = 0

    def qpos(self) -> Optional[np.ndarray]:
        """Strict 17-vec: None unless BOTH arms report. Use for control-critical
        paths (recording) that must not act on a half-present state."""
        return self._merged("position")

    def torques(self) -> Optional[np.ndarray]:
        return self._merged("torque")

    def _merged_partial(self, field: str) -> Optional[np.ndarray]:
        """Resilient merge: tolerates ONE arm being absent — its 7 slots are NaN.
        Returns None only if BOTH arms are down."""
        l = self._arm_field("left", field)
        r = self._arm_field("right", field)
        if l is None and r is None:
            return None
        nan7 = [float("nan")] * 7
        return np.array((l or nan7) + (r or nan7) + [0.0, 0.0, 0.0], dtype=np.float32)

    def qpos_partial(self) -> Optional[np.ndarray]:
        """17-vec for DISPLAY + resilient control: missing arm = NaN, head/lift = 0.
        None only if BOTH arms are down — so one arm dropping never blanks the other."""
        return self._merged_partial("position")

    def torques_partial(self) -> Optional[np.ndarray]:
        return self._merged_partial("torque")

    def temps_partial(self) -> Optional[np.ndarray]:
        """17-vec of per-joint temperature (°C); missing arm = NaN, head/lift = 0."""
        return self._merged_partial("temperature")

    def battery_voltage(self) -> Optional[float]:
        """Lowest motor-pack voltage (V) reported across the fresh arms, or None."""
        vals = []
        for side in ("left", "right"):
            msg, t = self._msg(side)
            if msg and t and (time.time() - t) <= self._STALE_S:
                v = msg.get("battery_voltage")
                if isinstance(v, (int, float)):
                    vals.append(float(v))
        return min(vals) if vals else None

    def estop(self) -> Optional[bool]:
        """True if EITHER fresh arm reports emergency_stop; None if no fresh telem."""
        seen = False
        for side in ("left", "right"):
            msg, t = self._msg(side)
            if msg and t and (time.time() - t) <= self._STALE_S:
                seen = True
                if bool(msg.get("emergency_stop")):
                    return True
        return False if seen else None

    def telem_age(self, now: Optional[float] = None) -> Dict[str, float]:
        now = time.time() if now is None else now
        out = {}
        for side in ("left", "right"):
            _, t = self._msg(side)
            out[side] = (now - t) if t else 999.0
        return out

    def arm_ok(self) -> Dict[str, bool]:
        return {"left": self._arm_field("left", "position") is not None,
                "right": self._arm_field("right", "position") is not None}

    # -- commands --------------------------------------------------------
    def _raw_send(self, side: str, msg: dict) -> None:
        try:
            with self._send_lock[side]:
                self._push[side].send(pack_msg(msg), zmq.NOBLOCK)
        except Exception:
            pass

    def enable(self) -> None:
        self._raw_send("left", {"type": "enable", "motor_ids": LEFT_NAMES})
        self._raw_send("right", {"type": "enable", "motor_ids": RIGHT_NAMES})

    def disable(self) -> None:
        with self._holder_lock:                      # stop the re-emitter first
            self._holder["left"] = None
            self._holder["right"] = None
        self._raw_send("left", {"type": "disable", "motor_ids": LEFT_NAMES})
        self._raw_send("right", {"type": "disable", "motor_ids": RIGHT_NAMES})

    def mech_zero(self, save: bool = True) -> None:
        """RobStride hardware mechanical zero + SaveConfig, both arms.
        Caller must ensure motors are DISABLED (motor_control refuses otherwise)."""
        self._raw_send("left", {"type": "mech_zero", "motor_ids": LEFT_NAMES, "save": save})
        self._raw_send("right", {"type": "mech_zero", "motor_ids": RIGHT_NAMES, "save": save})

    def set_target(self, q17: np.ndarray, kp17=None, kd17=None, tau17=None) -> None:
        """Split a 17-vector target into two 7-joint arm_joints commands (head/lift
        dropped) and hand them to the 100 Hz re-emitter.

        `tau17` is an optional 17-vector of feedforward torques (Nm) — e.g. gravity
        compensation — applied on-motor as tau_ff in the MIT PD loop
        (torque = kp·err + kd·(0−vel) + tau_ff). Omitted → zeros (plain PD, today's
        behaviour). The re-emitter reuses whatever tau was latched here.

        RESILIENT: an arm whose telemetry is absent (dropped bus) is NOT commanded
        — its holder is cleared so the re-emitter goes quiet — and it resumes
        automatically when it returns. NaN target slots are skipped too. So one arm
        dropping never stalls commands to the healthy arm."""
        q = np.asarray(q17, dtype=np.float32)
        kp = list(kp17) if kp17 is not None else list(KP)
        kd = list(kd17) if kd17 is not None else list(KD)
        tau = (np.asarray(tau17, dtype=np.float32) if tau17 is not None
               else np.zeros(NUM_MINERVA_JOINTS, dtype=np.float32))
        present = self.arm_ok()
        bundles = {}
        for side, lo in (("left", 0), ("right", 7)):
            pos = q[lo:lo + 7]
            if not present[side] or not np.all(np.isfinite(pos)):
                bundles[side] = None      # don't command a dropped / NaN arm
            else:
                tau_side = tau[lo:lo + 7]
                tau_side = np.where(np.isfinite(tau_side), tau_side, 0.0)  # never send NaN FF
                bundles[side] = {"type": "arm_joints", "positions": pos.tolist(),
                                 "velocities": [0.0] * 7,
                                 "kp": kp[lo:lo + 7], "kd": kd[lo:lo + 7],
                                 "torques": tau_side.tolist()}
        with self._holder_lock:
            self._holder.update(bundles)
        for side, b in bundles.items():
            if b is not None:
                self._raw_send(side, b)   # send now too (don't wait for re-emit tick)

    def clear_target(self) -> None:
        with self._holder_lock:
            self._holder["left"] = None
            self._holder["right"] = None

    # -- lifecycle -------------------------------------------------------
    def close(self) -> None:
        self._stop.set()
        self._reemit.join(timeout=1.0)
        for side in ("left", "right"):
            stop, thread, _, _ = self._telem[side]
            stop.set()
            thread.join(timeout=1.0)
            self._push[side].close()


__all__ = ["DualArmTransport", "LEFT_NAMES", "RIGHT_NAMES"]
