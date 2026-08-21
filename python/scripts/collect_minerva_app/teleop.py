"""teleop.py — Minerva bimanual teleop source (two OpenRB-150 leaders + jog).

Produces a 17-DoF follower target each tick from:
  - LEFT  OpenRB-150 leader  -> left arm  + gripper   (target indices 0..6)
  - RIGHT OpenRB-150 leader  -> right arm + gripper   (target indices 7..13)
  - head pan/tilt (14,15): LEFT leader joystick x/y (integrated), + jog
  - lift (16):             RIGHT leader joystick y   (integrated), + jog
  - record toggle:         either leader's joystick button (press edges)

Mapping is RELATIVE: on engage() we snapshot each leader pose and the current
follower qpos, then target = engage_qpos + (leader_now - leader_engage). This
means the follower never jumps when teleop engages and needs no absolute
leader↔follower calibration to start. Keyboard/GUI `jog()` offsets add on top,
so the whole app is fully runnable with NO leader hardware attached (jog-only).

A background reader thread polls both leaders as fast as the USB-CDC allows and
caches the latest poses + joystick snapshots under a lock; target() (called at
the main-loop rate) reads the cache and integrates head/lift by dt.

`OpenRBLeader` lives in python/teleop/ — the caller must put that dir on
sys.path (collect_minerva.py does). Absent pyserial / leaders, the app degrades
cleanly to jog-only.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from common.minerva_constants import (
    NUM_MINERVA_JOINTS, HEAD_INDICES, LIFT_INDEX, JOINT_LIMITS, clamp_positions,
)

_N = NUM_MINERVA_JOINTS


def _import_openrb():
    try:
        from openrb_leader import OpenRBLeader, find_openrb_port  # teleop/ on sys.path
        return OpenRBLeader, find_openrb_port
    except Exception:
        return None, None


class MinervaTeleop:
    def __init__(
        self,
        left_port: Optional[str] = None,
        right_port: Optional[str] = None,
        left_calib: Optional[str] = None,
        right_calib: Optional[str] = None,
        *,
        head_speed: float = 1.2,   # rad/s at full joystick deflection
        lift_speed: float = 0.05,  # m/s   at full joystick deflection
        jog_step_arm: float = 0.02,
        jog_step_head: float = 0.02,
        jog_step_lift: float = 0.005,
        swap: bool = False,
    ):
        # Leader→arm routing. False: LEFT leader→left arm, RIGHT leader→right arm.
        # True (swapped): LEFT leader→right arm, RIGHT leader→left arm. Persisted by
        # the caller (collect_minerva.py loads it from CollectorSettings, the GUI's
        # Swap button saves it) — the teleop just holds the live flag.
        self._swapped = bool(swap)
        self.left_port = left_port
        self.right_port = right_port
        self.left_calib = left_calib
        self.right_calib = right_calib
        self.head_speed = head_speed
        self.lift_speed = lift_speed
        self.jog_step_arm = jog_step_arm
        self.jog_step_head = jog_step_head
        self.jog_step_lift = jog_step_lift

        self._left = None
        self._right = None
        self._reader_stop = threading.Event()
        self._reader: Optional[threading.Thread] = None

        self._lock = threading.Lock()
        self._left_rad: Optional[np.ndarray] = None
        self._right_rad: Optional[np.ndarray] = None
        self._left_joy: dict = {}
        self._right_joy: dict = {}

        # engage anchors
        self._engage_qpos: Optional[np.ndarray] = None
        self._left_anchor: Optional[np.ndarray] = None
        self._right_anchor: Optional[np.ndarray] = None
        # Absolute leader->follower registration (collect_demo model): target =
        # dir * (leader_rad - zero). Seeded from each leader's calib zero/
        # direction in connect(); updated live by leader_zero() [Z] / mirror() [M].
        self._left_zero: Optional[np.ndarray] = None
        self._right_zero: Optional[np.ndarray] = None
        self._left_dir = np.ones(7, dtype=np.float32)
        self._right_dir = np.ones(7, dtype=np.float32)
        self._head = np.zeros(2, dtype=np.float32)   # integrated head pan/tilt
        self._lift = 0.0                             # integrated lift
        self._jog = np.zeros(_N, dtype=np.float32)   # persistent keyboard/GUI trims
        self._last_t: Optional[float] = None

        # record-toggle edge tracking (joystick button press_counter)
        self._last_press = {"left": 0, "right": 0}
        self._pending_record_edges = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def connect(self, verbose: bool = True) -> Tuple[bool, bool]:
        """Best-effort connect both leaders. Returns (left_ok, right_ok).
        Missing hardware is fine — the app runs jog-only."""
        OpenRBLeader, find_openrb_port = _import_openrb()
        if OpenRBLeader is None:
            if verbose:
                print("[teleop] OpenRB leader / pyserial unavailable — jog-only mode")
            self._start_reader()
            return (False, False)

        # Auto-detect two distinct ports if not explicitly given.
        lp, rp = self.left_port, self.right_port
        if lp is None or rp is None:
            excl: List[str] = [p for p in (lp, rp) if p]
            found = []
            for _ in range(2):
                p = find_openrb_port(exclude=excl + found, verbose=verbose)
                if p:
                    found.append(p)
            if lp is None and found:
                lp = found.pop(0)
            if rp is None and found:
                rp = found.pop(0)

        def _mk(port, calib, label):
            if not port:
                if verbose:
                    print(f"[teleop] no {label} leader port — {label} arm is jog-only")
                return None
            try:
                dev = OpenRBLeader(port, calib=calib) if calib else OpenRBLeader(port)
                if dev.connect():
                    if verbose:
                        print(f"[teleop] {label} leader connected on {port}")
                    return dev
                if verbose:
                    print(f"[teleop] {label} leader failed to connect on {port}")
            except Exception as e:
                if verbose:
                    print(f"[teleop] {label} leader error: {e}")
            return None

        self._left = _mk(lp, self.left_calib, "left")
        self._right = _mk(rp, self.right_calib, "right")
        self._seed_registration()
        self._start_reader()
        return (self._left is not None, self._right is not None)

    def _seed_registration(self) -> None:
        """Seed absolute-mapping zero/direction from each connected leader's calib."""
        if self._left is not None:
            self._left_zero = np.asarray(self._left.zero_offsets, dtype=np.float32).copy()
            self._left_dir = np.asarray(self._left.directions, dtype=np.float32).copy()
        if self._right is not None:
            self._right_zero = np.asarray(self._right.zero_offsets, dtype=np.float32).copy()
            self._right_dir = np.asarray(self._right.directions, dtype=np.float32).copy()

    def _start_reader(self):
        if self._reader is not None:
            return

        def _run():
            while not self._reader_stop.is_set():
                did = False
                if self._left is not None:
                    r = self._left.poll()
                    joy = self._left.last_joystick
                    with self._lock:
                        if r is not None:
                            self._left_rad = r
                        self._left_joy = joy
                    self._note_press("left", joy)
                    did = True
                if self._right is not None:
                    r = self._right.poll()
                    joy = self._right.last_joystick
                    with self._lock:
                        if r is not None:
                            self._right_rad = r
                        self._right_joy = joy
                    self._note_press("right", joy)
                    did = True
                if not did:
                    self._reader_stop.wait(0.02)   # jog-only: nothing to poll

        self._reader = threading.Thread(target=_run, daemon=True, name="LeaderRx")
        self._reader.start()

    def _note_press(self, which: str, joy: dict):
        if not joy or not joy.get("present"):
            return
        pc = int(joy.get("press_counter", 0))
        with self._lock:
            prev = self._last_press[which]
            if pc > prev:
                self._pending_record_edges += (pc - prev)
            self._last_press[which] = pc

    def close(self):
        self._reader_stop.set()
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        for dev in (self._left, self._right):
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass

    @property
    def status(self) -> dict:
        return {"left": self._left is not None, "right": self._right is not None}

    # ------------------------------------------------------------------
    # Leader↔arm swap (which physical leader drives which follower arm)
    # ------------------------------------------------------------------
    @property
    def swapped(self) -> bool:
        return self._swapped

    def set_swapped(self, value: bool) -> Optional[bool]:
        """Set the leader→arm routing. Refused (returns None) while engaged, because
        re-routing mid-teleop would make an arm suddenly chase a far-away leader and
        lunge. Otherwise returns the new state. Thread-safe (GUI calls this)."""
        with self._lock:
            if self._engage_qpos is not None:
                return None
            self._swapped = bool(value)
            return self._swapped

    def toggle_swap(self) -> Optional[bool]:
        """Flip the routing; returns the new state, or None if refused (engaged)."""
        with self._lock:
            if self._engage_qpos is not None:
                return None
            self._swapped = not self._swapped
            return self._swapped

    def _arm_source(self, follower_side: str):
        """(frac[7], dir[7]) of the leader that drives the follower's `follower_side`
        ('left'|'right') arm slice, honoring the swap toggle. `frac` is the leader's
        normalized position in [0,1] within its calibrated sweep (the leader calib maps
        to [0,1]); `dir` (<0) flips which follower end frac=0 maps to. Pure reader — call
        under self._lock."""
        use_left = (follower_side == "left") != self._swapped
        if use_left:
            return self._left_rad, self._left_dir
        return self._right_rad, self._right_dir

    @staticmethod
    def _range_map(frac, dr, lo_hi) -> np.ndarray:
        """Map leader frac[0..1] onto each follower joint's [min,max] (range-to-range):
        dir>=0 -> lo + f*(hi-lo); dir<0 -> hi - f*(hi-lo). So the leader's full sweep
        drives the follower across its whole travel, and dir flips the sense."""
        lo = lo_hi[:, 0]
        hi = lo_hi[:, 1]
        f = np.clip(np.asarray(frac[:7], dtype=np.float32), 0.0, 1.0)
        span = hi - lo
        return np.where(np.asarray(dr[:7]) >= 0, lo + f * span, hi - f * span).astype(np.float32)

    # ------------------------------------------------------------------
    # Engage / target
    # ------------------------------------------------------------------
    def engage(self, qpos_actual: np.ndarray):
        """Snapshot leader poses + follower pose so teleop starts jump-free."""
        q = np.asarray(qpos_actual, dtype=np.float32).copy()
        with self._lock:
            self._engage_qpos = q
            self._left_anchor = None if self._left_rad is None else self._left_rad.copy()
            self._right_anchor = None if self._right_rad is None else self._right_rad.copy()
            self._head = q[HEAD_INDICES].copy()
            self._lift = float(q[LIFT_INDEX])
            self._jog[:] = 0.0
        self._last_t = time.monotonic()

    def disengage(self):
        with self._lock:
            self._engage_qpos = None

    @property
    def engaged(self) -> bool:
        return self._engage_qpos is not None

    def leader_preview(self) -> np.ndarray:
        """17-vec of where each LEADER currently points, mapped into follower frame
        (`dir*(leader - zero)`) — computed ALWAYS, independent of engage, so the GUI
        can draw a leader↔follower diff bar for SAFE engagement (drive the bars green,
        then engage jump-free). Joints with no live leader reading / no zero yet, and
        head+lift (joystick-integrated, no absolute leader), are NaN."""
        out = np.full(_N, np.nan, dtype=np.float32)
        with self._lock:
            for lo, side in ((0, "left"), (7, "right")):
                frac, dr = self._arm_source(side)
                if frac is not None:
                    out[lo:lo + 7] = self._range_map(frac, dr, JOINT_LIMITS[lo:lo + 7])
        return out

    def target(self, qpos_actual: np.ndarray) -> Optional[np.ndarray]:
        """Compute the 17-DoF follower target, or None if not engaged."""
        now = time.monotonic()
        dt = 0.0 if self._last_t is None else min(now - self._last_t, 0.1)
        self._last_t = now
        with self._lock:
            if self._engage_qpos is None:
                return None
            eng = self._engage_qpos
            ljoy, rjoy = self._left_joy, self._right_joy
            if self._swapped:                 # left device now plays the right role
                ljoy, rjoy = rjoy, ljoy

            tgt = eng.copy()
            # RANGE-TO-RANGE: the leader's normalized position (frac 0..1) maps onto the
            # FOLLOWER joint's full calibrated range, so a small leader sweep drives the
            # arm across its whole travel (and the gripper fully open↔closed). _arm_source
            # honors the leader↔arm swap; dir flips the sense. Absolute, so no per-engage
            # anchor — the collector's engage ramp eases in from the current pose.
            for lo, side in ((0, "left"), (7, "right")):
                frac, dr = self._arm_source(side)
                if frac is not None:
                    tgt[lo:lo + 7] = self._range_map(frac, dr, JOINT_LIMITS[lo:lo + 7])

            # Head/lift integrate joystick deflection, with ANTI-WINDUP: the
            # integrator state itself is clamped to the joint limits, so holding
            # the stick past a limit doesn't accumulate an overshoot the operator
            # then has to unwind before head/lift will reverse.
            h0, h1 = HEAD_INDICES
            if ljoy.get("present"):
                self._head[0] += float(ljoy.get("x", 0.0)) * self.head_speed * dt
                self._head[1] += float(ljoy.get("y", 0.0)) * self.head_speed * dt
            if rjoy.get("present"):
                self._lift += float(rjoy.get("y", 0.0)) * self.lift_speed * dt
            self._head[0] = float(np.clip(self._head[0], JOINT_LIMITS[h0, 0], JOINT_LIMITS[h0, 1]))
            self._head[1] = float(np.clip(self._head[1], JOINT_LIMITS[h1, 0], JOINT_LIMITS[h1, 1]))
            self._lift = float(np.clip(self._lift, JOINT_LIMITS[LIFT_INDEX, 0], JOINT_LIMITS[LIFT_INDEX, 1]))
            tgt[h0] = self._head[0]
            tgt[h1] = self._head[1]
            tgt[LIFT_INDEX] = self._lift

            # Persistent keyboard/GUI jog trims on every joint.
            tgt = tgt + self._jog
        return clamp_positions(tgt)

    # ------------------------------------------------------------------
    # Keyboard / GUI jog + record edges
    # ------------------------------------------------------------------
    def jog(self, joint_idx: int, direction: float):
        """Nudge one joint by its step (direction in {-1,+1}). Works with or
        without leaders; for head/lift this is the no-joystick control path."""
        if not (0 <= joint_idx < _N):
            return
        if joint_idx in HEAD_INDICES:
            step = self.jog_step_head
        elif joint_idx == LIFT_INDEX:
            step = self.jog_step_lift
        else:
            step = self.jog_step_arm
        with self._lock:
            lo, hi = JOINT_LIMITS[joint_idx]
            span = float(hi - lo)   # bound the trim so it can't wind up unbounded
            self._jog[joint_idx] = float(np.clip(
                self._jog[joint_idx] + float(direction) * step, -span, span))

    def take_record_edges(self) -> int:
        """Number of joystick-button record-toggle presses since last call."""
        with self._lock:
            n = self._pending_record_edges
            self._pending_record_edges = 0
        return n

    # ------------------------------------------------------------------
    # Zero functions (collect_demo parity: Z = leader zero, M = mirror)
    # ------------------------------------------------------------------
    def leader_zero(self) -> str:
        """[Z] Not needed in range-to-range mode — the leader's normalized sweep already
        maps to the follower's full range, so there is no zero to capture."""
        return "range-to-range: zero not needed"

    def mirror(self, qpos_actual: Optional[np.ndarray]) -> str:
        """[M] Not needed in range-to-range mode — the mapping is absolute (leader sweep →
        follower range), so there is no offset to mirror."""
        return "range-to-range: mirror not needed"


__all__ = ["MinervaTeleop"]
