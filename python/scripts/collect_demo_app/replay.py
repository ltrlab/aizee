"""Live episode replay (on-robot playback) (from collect_demo.py)."""
from __future__ import annotations

import enum
import time
from pathlib import Path
from typing import Optional

import h5py
import numpy as np

from common.arm_constants import clamp_arm_positions

from .runtime import LOOP_HZ, NUM_JOINTS

# ---------------------------------------------------------------------------
# Live episode replay (on-robot playback inside the main loop)
# ---------------------------------------------------------------------------

def _load_episode_for_replay(path: Path) -> tuple[np.ndarray, float]:
    """Load an episode HDF5 for live replay.

    Returns `(qpos[T, NUM_JOINTS], hz)` — qpos is always 7-DOF in
    ARM_JOINTS order (swivel-first).  Older 6-DOF files with a sidecar
    `swivel` dataset are stitched into the unified shape on the way out.
    """
    with h5py.File(path, "r") as f:
        if "observations" in f and "qpos" in f["observations"]:
            obs = f["observations"]
            raw = obs["qcmd"][:] if "qcmd" in obs else obs["qpos"][:]
            sw  = obs["swivel"][:] if "swivel" in obs else None
            hz  = float(f.attrs.get("hz", 20.0))
        elif "qpos" in f:
            raw = f["qpos"][:]
            sw  = f["swivel"][:] if "swivel" in f else None
            hz  = float(f.attrs.get("hz", 20.0))
        else:
            raise ValueError(f"Unrecognised HDF5 format: {path}")

    raw = raw.astype(np.float32)
    if raw.ndim != 2:
        raise ValueError(f"qpos must be 2D, got shape {raw.shape}")

    # 7-DOF unified — return as-is.
    if raw.shape[1] == NUM_JOINTS:
        return raw, hz

    # Legacy 6-DOF gantry-only with sidecar swivel.
    if raw.shape[1] == NUM_JOINTS - 1:
        if sw is None:
            sw = np.zeros((raw.shape[0],), dtype=np.float32)
        sw_col = sw.astype(np.float32).reshape(-1, 1)
        n      = min(sw_col.shape[0], raw.shape[0])
        return np.hstack([sw_col[:n], raw[:n]]).astype(np.float32), hz

    raise ValueError(
        f"qpos has {raw.shape[1]} columns; expected {NUM_JOINTS} (unified) or "
        f"{NUM_JOINTS - 1} (legacy 6-DOF + sidecar swivel)"
    )


class _LiveReplay:
    """State machine for on-robot episode playback inside collect_demo.

    The main loop owns ZMQ; this class produces lists of motor-command
    dicts via step() / arm() / play() etc., which the caller sends.
    See episode_replay_live.py for the standalone reference.
    """

    class Phase(enum.Enum):
        READY    = "ready"
        ARMING   = "arming"
        PLAYING  = "playing"
        PAUSED   = "paused"
        DONE     = "done"
        SHUTDOWN = "shutdown"

    def __init__(
        self, *, kp, kd,
        max_delta: float, arm_limits, all_motor_ids,
    ):
        self._kp              = list(kp)
        self._kd              = list(kd)
        self._arm_limits      = arm_limits
        self._all_motor_ids   = list(all_motor_ids)

        # Live config (mutable from GUI)
        self.max_delta        = float(max_delta)
        self.speed            = 1.0
        self.loop_mode        = False
        self.goto_start       = True
        self.vel_ff_enabled   = False
        self.ramp_speed       = 1.5   # rad/s approach to start pose.  0.4
                                      # was too slow to traverse end→start
                                      # within a reasonable window when the
                                      # episode ended far from its beginning,
                                      # making re-arming look stuck.
        self._ramp_step       = self.ramp_speed / LOOP_HZ
        self._arm_max_lead    = 0.15  # rad cap on how far the command may
                                      # lead q_actual during ARMING.  At
                                      # kp=30 this puts peak PD torque at
                                      # ~4.5 N·m — enough to overcome gantry
                                      # gravity (was 0.05 / 1.5 N·m, which
                                      # could not move the arm against load
                                      # for re-arming from a distant pose).

        # Episode (set by load()).  qpos is always 7-DOF (swivel-first).
        self.ep_path:       Optional[Path]       = None
        self.ep_name:       str                  = ""
        self.ep_qpos:       Optional[np.ndarray] = None
        self.ep_velocities: Optional[np.ndarray] = None
        self.ep_hz:         float                = 0.0
        self.ep_frames:     int                  = 0

        # Runtime state.  current_target is the 7-DOF arm target — swivel
        # is current_target[0]; there's no separate swivel state anymore.
        self.live:              bool                  = False
        self.phase                                    = self.Phase.READY
        self.frame_idx:         int                   = 0
        self.last_frame_wall:   float                 = 0.0
        self.current_target:    Optional[np.ndarray]  = None
        self.error:             float                 = 0.0
        self.message:           str                   = ""

        # Shutdown state
        self._shutdown_target:      Optional[np.ndarray] = None
        self._shutdown_countdown:   float                = 0.0
        self._shutdown_zero_since:  float                = 0.0
        self._SHUTDOWN_TIMEOUT                           = 3.0

        # Arming-phase stall detection.  ARMING used to require every joint
        # within 0.03 rad of the start pose, which deadlocked when one joint
        # had enough stiction to sit ~0.04 rad off under low PD authority.
        self._arming_start_t:        float = 0.0
        self._ARM_DONE_THRESHOLD            = 0.05   # rad — promote to PLAYING below this
        self._ARM_STALL_ACCEPT              = 0.10   # rad — after stall timeout, accept up to this
        self._ARM_STALL_TIMEOUT             = 4.0    # s — start accepting residual after this

    # -- Episode loading ---------------------------------------------------
    def load(self, path: Path) -> Optional[str]:
        """Load an episode. Returns None on success, error string on failure."""
        try:
            qpos, hz = _load_episode_for_replay(path)
        except Exception as e:
            self.message = f"load error: {e}"
            return str(e)
        self.ep_path       = path
        self.ep_name       = path.name
        self.ep_qpos       = qpos
        self.ep_hz         = hz
        self.ep_frames     = len(qpos)
        self.ep_velocities = self._compute_vel_ff(qpos, hz)
        self.frame_idx     = 0
        self.phase         = self.Phase.READY
        self.current_target = qpos[0].copy() if self.ep_frames > 0 else None
        self.error         = 0.0
        self.message       = f"loaded {path.name}  {self.ep_frames}f @ {hz:.0f} Hz"
        return None

    @staticmethod
    def _compute_vel_ff(qpos: np.ndarray, hz: float) -> Optional[np.ndarray]:
        T = len(qpos)
        if T < 2 or hz <= 0:
            return None
        dt = 1.0 / hz
        dq = np.diff(qpos, axis=0) / dt                        # [T-1, J]
        dq = np.vstack([dq, np.zeros((1, qpos.shape[1]))])     # [T,   J]
        smoothed = dq.copy()
        for i in range(1, T - 1):
            smoothed[i] = (dq[i - 1] + dq[i] + dq[i + 1]) / 3.0
        if T > 1:
            smoothed[0] = (dq[0] + dq[1]) / 2.0
        return smoothed.astype(np.float32)

    # -- Mode transitions --------------------------------------------------
    def enter_live(self) -> bool:
        if self.ep_qpos is None or self.ep_frames == 0:
            return False
        self.live      = True
        self.phase     = self.Phase.READY
        self.frame_idx = 0
        return True

    def exit_live(self) -> bool:
        """Exit live mode. Rejected while motors are actively commanded."""
        if self.phase in (self.Phase.ARMING, self.Phase.PLAYING, self.Phase.SHUTDOWN):
            return False
        self.live  = False
        self.phase = self.Phase.READY
        return True

    # -- Transport ---------------------------------------------------------
    def arm(self, q_actual) -> list[dict]:
        if not self.live or self.ep_qpos is None or self.ep_frames == 0:
            return []
        if self.phase == self.Phase.SHUTDOWN:
            return []
        self.frame_idx = 0
        cmds: list[dict] = [{"type": "enable", "motor_ids": self._all_motor_ids}]
        if self.goto_start and q_actual is not None:
            # Seed the ramp's integrator from the current measured pose so
            # the arming step starts exactly where the arm is.
            self.current_target = q_actual.copy().astype(np.float32)
            self.phase = self.Phase.ARMING
            self._arming_start_t = time.time()
        else:
            self.phase           = self.Phase.PLAYING
            self.last_frame_wall = time.time()
        return cmds

    def play(self, q_actual) -> list[dict]:
        if not self.live or self.phase == self.Phase.SHUTDOWN:
            return []
        if self.phase in (self.Phase.READY, self.Phase.DONE):
            return self.arm(q_actual)
        if self.phase == self.Phase.PAUSED:
            self.phase           = self.Phase.PLAYING
            self.last_frame_wall = time.time()
        return []

    def pause(self) -> None:
        if self.live and self.phase == self.Phase.PLAYING:
            self.phase = self.Phase.PAUSED

    def toggle(self, q_actual) -> list[dict]:
        if not self.live:
            return []
        if self.phase == self.Phase.PLAYING:
            self.pause()
            return []
        return self.play(q_actual)

    def restart(self, q_actual) -> list[dict]:
        if not self.live or self.phase == self.Phase.SHUTDOWN:
            return []
        self.frame_idx = 0
        cmds: list[dict] = []
        # If we arrived from DONE/PAUSED, motors may have been left disabled
        # by a prior stop; re-enable defensively so ARMING actually moves.
        if self.phase != self.Phase.ARMING and self.phase != self.Phase.PLAYING:
            cmds.append({"type": "enable", "motor_ids": self._all_motor_ids})
        if self.goto_start and q_actual is not None:
            # Reset the ramp integrator from current measured pose so we
            # don't carry the previous run's end-frame target into ARMING
            # (which would make ref ≈ end_pose and queue a giant delta).
            self.current_target = q_actual.copy().astype(np.float32)
            self.phase = self.Phase.ARMING
            self._arming_start_t = time.time()
        else:
            self.phase           = self.Phase.PLAYING
            self.last_frame_wall = time.time()
        return cmds

    def stop(self, q_actual) -> None:
        """Abort: ramp to zero and disable. Phase returns to READY when done."""
        if not self.live or self.phase == self.Phase.SHUTDOWN:
            return
        if self.phase == self.Phase.READY:
            return
        self._shutdown_target = (q_actual.copy() if q_actual is not None
                                 else (self.current_target.copy()
                                       if self.current_target is not None
                                       else np.zeros(NUM_JOINTS, dtype=np.float32)))
        self._shutdown_countdown  = 1.0
        self._shutdown_zero_since = 0.0
        self.phase                = self.Phase.SHUTDOWN

    def set_speed(self, speed: float) -> None:
        self.speed = max(0.1, min(float(speed), 4.0))

    def set_opts(self, **opts) -> None:
        if "loop" in opts:
            self.loop_mode = bool(opts["loop"])
        if "goto_start" in opts:
            self.goto_start = bool(opts["goto_start"])
        if "max_delta" in opts:
            self.max_delta = max(0.01, float(opts["max_delta"]))
        if "vel_ff" in opts:
            self.vel_ff_enabled = bool(opts["vel_ff"])
        if "ramp_speed" in opts:
            self.ramp_speed = max(0.1, float(opts["ramp_speed"]))
            self._ramp_step = self.ramp_speed / LOOP_HZ

    # -- Command builders --------------------------------------------------
    def _safe_cmd(self, target: np.ndarray, ref: Optional[np.ndarray]) -> np.ndarray:
        r = ref if ref is not None else target
        q = r + np.clip(target - r, -self.max_delta, self.max_delta)
        if self._arm_limits:
            q = np.array(clamp_arm_positions(q.tolist(), self._arm_limits))
        return q

    def _arm_cmd(self, q_cmd: np.ndarray, vel_ff: Optional[list]) -> dict:
        return {
            "type":       "arm_joints",
            "positions":  q_cmd.tolist(),
            "velocities": vel_ff if vel_ff is not None else [0.0] * NUM_JOINTS,
            "kp":         self._kp,
            "kd":         self._kd,
            "torques":    [0.0] * NUM_JOINTS,
        }

    # -- Per-tick step -----------------------------------------------------
    def step(
        self, t0: float, q_actual: Optional[np.ndarray], period: float,
    ) -> list[dict]:
        if not self.live:
            return []

        # Tracking error telemetry (max-norm across all 7 joints)
        if q_actual is not None and self.current_target is not None:
            self.error = float(np.max(np.abs(q_actual - self.current_target)))

        cmds: list[dict] = []
        phase = self.phase

        if phase == self.Phase.READY:
            return cmds

        if phase == self.Phase.ARMING:
            tgt = self.ep_qpos[0]
            ref = (self.current_target if self.current_target is not None
                   else (q_actual if q_actual is not None else tgt))
            if q_actual is not None:
                lead = np.clip(ref - q_actual,
                               -self._arm_max_lead, self._arm_max_lead)
                ref  = q_actual + lead
            q_cmd = ref + np.clip(tgt - ref, -self._ramp_step, self._ramp_step)
            if self._arm_limits:
                q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), self._arm_limits))
            cmds.append(self._arm_cmd(q_cmd, None))
            self.current_target = q_cmd
            arm_ok = False
            if q_actual is not None:
                err = float(np.max(np.abs(q_actual - tgt)))
                arm_ok = err < self._ARM_DONE_THRESHOLD
                # Stall escape: if a joint has stiction/PD-authority limits
                # that prevent closing the last sliver, accept a small
                # residual after a few seconds rather than hanging forever.
                if (not arm_ok
                        and self._arming_start_t > 0
                        and t0 - self._arming_start_t > self._ARM_STALL_TIMEOUT
                        and err < self._ARM_STALL_ACCEPT):
                    self.message = f"armed with residual {err:.3f} rad"
                    arm_ok = True
            if arm_ok:
                self.phase           = self.Phase.PLAYING
                self.last_frame_wall = t0
            return cmds

        if phase == self.Phase.PLAYING:
            frame_period = 1.0 / max(self.ep_hz * self.speed, 0.1)
            if t0 - self.last_frame_wall >= frame_period and self.frame_idx < self.ep_frames:
                self.last_frame_wall = t0
                tgt    = self.ep_qpos[self.frame_idx]
                vel_ff = (self.ep_velocities[self.frame_idx].tolist()
                          if (self.ep_velocities is not None and self.vel_ff_enabled)
                          else None)
                cmds.append(self._arm_cmd(
                    self._safe_cmd(tgt, self.current_target), vel_ff))
                self.current_target = tgt
                self.frame_idx += 1
                if self.frame_idx >= self.ep_frames:
                    if self.loop_mode:
                        self.frame_idx = 0
                    else:
                        self.phase = self.Phase.DONE
            return cmds

        if phase in (self.Phase.PAUSED, self.Phase.DONE):
            if self.current_target is not None:
                cmds.append(self._arm_cmd(
                    self._safe_cmd(self.current_target, q_actual), None))
            return cmds

        if phase == self.Phase.SHUTDOWN:
            dt         = period
            max_change = 0.2 * dt   # 0.2 rad/s ramp to zero
            if self._shutdown_countdown > 0:
                self._shutdown_countdown -= dt
                if self._shutdown_target is not None:
                    cmds.append(self._arm_cmd(self._shutdown_target, None))
                return cmds
            if self._shutdown_target is None:
                self._shutdown_target = np.zeros(NUM_JOINTS, dtype=np.float32)
            new_tgt = self._shutdown_target.copy()
            for i in range(len(new_tgt)):
                new_tgt[i] = (0.0 if abs(new_tgt[i]) < max_change
                              else new_tgt[i] - np.sign(new_tgt[i]) * max_change)
            self._shutdown_target = new_tgt
            ramp_done = bool(np.all(np.abs(self._shutdown_target) < 0.01))
            if ramp_done and self._shutdown_zero_since == 0.0:
                self._shutdown_zero_since = t0
            actual_close = (q_actual is None or np.all(np.abs(q_actual) < 0.05))
            timed_out    = (self._shutdown_zero_since > 0
                            and t0 - self._shutdown_zero_since >= self._SHUTDOWN_TIMEOUT)
            if ramp_done and (actual_close or timed_out):
                cmds.append({"type": "disable", "motor_ids": self._all_motor_ids})
                self.phase               = self.Phase.READY
                self.frame_idx           = 0
                self._shutdown_target    = None
                self.current_target = (self.ep_qpos[0].copy()
                                       if self.ep_frames > 0 else None)
            else:
                ref   = q_actual if q_actual is not None else self._shutdown_target
                q_cmd = self._safe_cmd(self._shutdown_target, ref)
                cmds.append(self._arm_cmd(q_cmd, None))
            return cmds

        return cmds

    # -- Snapshot / status -------------------------------------------------
    def snapshot_fields(self) -> dict:
        pct = (100.0 * self.frame_idx / self.ep_frames) if self.ep_frames else 0.0
        dur = (self.ep_frames / self.ep_hz) if self.ep_hz > 0 else 0.0
        # 7-DOF target the GUI's joint panel plots against live actual.
        # Layout matches ARM_JOINTS (swivel-first).
        if self.current_target is not None:
            replay_target = [float(x) for x in self.current_target]
        else:
            replay_target = None
        return {
            "replay_live":        self.live,
            "replay_phase":       self.phase.value if self.live else None,
            "replay_frame":       self.frame_idx,
            "replay_frames":      self.ep_frames,
            "replay_pct":         pct,
            "replay_hz":          self.ep_hz,
            "replay_duration":    dur,
            "replay_speed":       self.speed,
            "replay_loop":        self.loop_mode,
            "replay_goto_start":  self.goto_start,
            "replay_vel_ff":      self.vel_ff_enabled,
            "replay_max_delta":   self.max_delta,
            "replay_error":       self.error,
            "replay_path":        str(self.ep_path) if self.ep_path else "",
            "replay_name":        self.ep_name,
            "replay_message":     self.message,
            "replay_target":      replay_target,
        }

    def status_line(self) -> tuple[str, str]:
        """Returns (status, hint) strings, or ('','') when live mode is off."""
        if not self.live:
            return "", ""
        p = self.phase
        if p == self.Phase.READY:
            return (f"[replay] ready — {self.ep_name}  {self.ep_frames}f",
                    "PLAY to arm+play · exit to return to teleop")
        if p == self.Phase.ARMING:
            return (f"[replay] arming — err {self.error:.3f} rad",
                    "STOP to abort")
        if p == self.Phase.PLAYING:
            pct = 100.0 * self.frame_idx / max(self.ep_frames, 1)
            lp  = "  LOOP" if self.loop_mode else ""
            return (f"[replay] PLAYING  {pct:.0f}%  {self.speed:.2f}x{lp}",
                    "PAUSE · STOP")
        if p == self.Phase.PAUSED:
            pct = 100.0 * self.frame_idx / max(self.ep_frames, 1)
            return (f"[replay] PAUSED  {pct:.0f}%", "PLAY · STOP")
        if p == self.Phase.DONE:
            return ("[replay] done", "PLAY to re-run · STOP to disable")
        if p == self.Phase.SHUTDOWN:
            if self._shutdown_countdown > 0:
                return (f"[replay] shutdown  hold {self._shutdown_countdown:.1f}s", "")
            return ("[replay] returning to zero", "")
        return "", ""
