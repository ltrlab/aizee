"""QuestLeader — WebXR / Meta Quest Pro Cartesian leader for AIZEE.

Implements the same duck-typed interface as `OpenRBLeader` and
`So101Leader` (connect / poll / close / JOINTS / AIZEE_JOINTS /
zero_offsets / directions / clamped_joints) so it slots into the
existing `_leader_reader` / `_lr_latest` pipeline in collect_demo.py
without touching the 30 Hz send path.

Control model: **clutched incremental Cartesian**.
  * Operator holds the right grip button to engage.
  * On the rising edge of grip:
      engage_ctrl  = current right-controller pose (WebXR local-floor frame)
      engage_ee    = FK(current_qpos) — the EE pose where the arm is *now*
      engage_yaw   = headset yaw at engage time (rotates frame so "forward in VR"
                     = "forward in robot base")
  * While grip is held:
      Δpose_xr      = current_ctrl  ⊖  engage_ctrl  (in WebXR frame)
      Δpose_robot   = R_yaw⁻¹ · R_xr→robot · Δpose_xr
      target_ee     = engage_ee  ⊕  Δpose_robot
      clamp target_ee position to the workspace box
      q_new[0:6]    = IK(target_ee, warm_start=last_q[0:6])
  * Gripper (7th joint) is driven by the trigger analog axis.
  * On grip release: the last commanded q is held; next engage rebases.

Frame conventions:
  WebXR local-floor:  +X right, +Y up, +Z toward viewer (so -Z forward).
  AIZEE base (URDF):  +X forward, +Y left, +Z up.

Pose source is `SharedState.latest_control` (written by the /ws/control
aiohttp handler).  Current qpos source is `SharedState.latest_telem`
(written by the telemetry bridge that subscribes to ZMQ:5556).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# Use absolute imports so this works both as `python.teleop.quest_leader`
# and as `teleop.quest_leader` (matches how collect_demo.py imports leaders).
try:
    from ik import load_aizee_arm, solve_ik
    from ik.kinematics import Kinematics, R_to_quat, quat_to_R
except ImportError:
    from python.ik import load_aizee_arm, solve_ik  # type: ignore
    from python.ik.kinematics import Kinematics, R_to_quat, quat_to_R  # type: ignore


# -----------------------------------------------------------------------------
# Frame helpers
# -----------------------------------------------------------------------------

# Fixed remap WebXR (Y-up, -Z forward) -> AIZEE base (Z-up, +X forward).
#   X_robot = -Z_xr,  Y_robot = -X_xr,  Z_robot = +Y_xr
_R_XR_TO_ROBOT: np.ndarray = np.array([
    [0.0, 0.0, -1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------

@dataclass
class QuestLeaderConfig:
    """Operator-facing knobs.  Loaded from config/quest_teleop.yaml."""
    # Operator clutch workspace — target EE position is clamped to this box
    # in robot base frame [m].  This is the SAFETY clamp; size it for
    # comfortable operator hand motion (~40-60 cm typical).
    workspace_min: tuple[float, float, float] = (-0.10, -0.30, 0.05)
    workspace_max: tuple[float, float, float] = (0.55, 0.30, 0.55)
    # Reachable hard limit — full IK reach AABB.  Informational only,
    # drawn as a thin wireframe in VR so the operator can see where the
    # arm physically cannot go regardless of clutch direction.  Computed
    # by `python -m ik.workspace --update-config`; falls back to None
    # (no overlay) if absent.
    reachable_min: Optional[tuple[float, float, float]] = None
    reachable_max: Optional[tuple[float, float, float]] = None
    # Stale-frame timeouts [s]
    stale_drop_clutch_s: float = 0.10        # no pose for this long -> release
    stale_telem_s: float = 0.50              # no telem for this long -> no engage
    # Gripper trigger -> rad mapping
    gripper_open_rad: float = 0.0
    gripper_closed_rad: float = 0.785
    # Velocity limit per joint [rad/s] applied to the leader-output stream
    max_joint_vel: float = 4.0
    # Cartesian velocity limit on the EE target [m/s].  Smooths out
    # tracking-noise jumps (esp. when the operator's hand is near the
    # Quest's FOV edge) before they reach the IK.  60 cm/s is plenty for
    # most teleop; tighten to ~30 cm/s for very fine work.
    cartesian_max_vel_m_s: float = 0.60
    # IK weighting / iteration knobs (passed straight to solve_ik)
    ik_pos_weight: float = 1.0
    ik_ori_weight: float = 0.3
    ik_damping: float = 5e-2
    ik_max_iter: int = 8
    # Per-joint damping weights in DLS order (swivel, gantry_base,
    # gantry_mid, gantry_end, wrist_pitch, wrist_roll).  weight>1 makes
    # that joint *harder* for the IK to move — use it to keep the gantry
    # joints out of rotation tasks they shouldn't be doing (a 3x weight
    # on gantry_base/mid/end makes the IK prefer the wrist joints for
    # any rotation that the wrists can handle alone).  Defaults to all
    # ones (original behaviour).
    ik_joint_weights: Optional[list] = None
    # Low-pass on the controller pose itself (kills hand tremor / sensor noise).
    # alpha=1.0 disables; lower = smoother but laggier.  0.3 is a good
    # default for hand tracking which is noisier than controllers.
    pose_lpf_alpha: float = 0.3

    @classmethod
    def load_yaml(cls, path: Path | str) -> "QuestLeaderConfig":
        import yaml
        data = yaml.safe_load(Path(path).read_text()) or {}
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})


# -----------------------------------------------------------------------------
# QuestLeader
# -----------------------------------------------------------------------------

class QuestLeader:
    """WebXR Cartesian leader — drop-in for OpenRBLeader/So101Leader."""

    # The serial leaders persist an encoder-offset "zero" (Z/M keys).  A
    # Cartesian VR leader has no such offset — the clutch re-anchors on
    # every engage — so Z/M are no-ops here.  collect_demo.py checks this
    # flag before running the (otherwise harmful) zeroing math.
    SUPPORTS_ZEROING = False

    # IK emits joint angles in URDF frame (right-handed, axes from
    # aizee.urdf).  Physical leader arms (SO-101 / OpenRB) emit in
    # MOTOR frame — their `directions` calibration was pre-tuned to
    # match the motor encoder direction so the arm followed correctly
    # before joint_align existed.  collect_demo reads this flag to
    # decide whether to fold _ALIGN_SIGNS into the mapped target.
    EMITS_URDF_FRAME = True

    # Public order — same as other leaders' AIZEE_JOINTS (this leader IS
    # already in the AIZEE-arm joint order, so JOINTS == AIZEE_JOINTS).
    JOINTS = [
        "swivel", "gantry_base", "gantry_mid", "gantry_end",
        "wrist_pitch", "wrist_roll", "gripper",
    ]
    AIZEE_JOINTS = list(JOINTS)

    def __init__(
        self,
        port: Optional[str] = None,        # ignored — for constructor compat
        baud: Optional[int] = None,        # ignored — for constructor compat
        calib: Optional[Path | str] = None,  # ignored — uses YAML config instead
        *,
        shared_state=None,                 # web.SharedState — required for poll()
        config: Optional[QuestLeaderConfig] = None,
        urdf_path: Optional[Path | str] = None,
        joint_limits_path: Optional[Path | str] = None,
    ) -> None:
        self.port = port or "webxr://"
        self.cfg = config or QuestLeaderConfig()
        self._state = shared_state         # caller wires this up in the factory
        self._kin: Kinematics = load_aizee_arm(urdf_path)
        # Tighten kin.lower/upper using the collision-sweep results so the
        # IK can never solve into a self-colliding pose.  Falls back silently
        # if the YAML isn't present yet (e.g. fresh repo without a sweep).
        if joint_limits_path is None:
            joint_limits_path = (
                Path(__file__).resolve().parents[2] / "config" / "joint_limits.yaml"
            )
        if Path(joint_limits_path).exists():
            try:
                self._kin.apply_limits_overlay(joint_limits_path)
            except Exception as exc:
                print(f"[quest] joint_limits overlay failed: {exc}", flush=True)
        # Last commanded full 7-vector (6 IK joints + gripper).  Defaults
        # to zeros; gets replaced by current qpos on first successful engage.
        # q_last is in URDF *control* frame (what the motor gets, modulo
        # signs the boundary handles).  All FK/IK below operate in URDF
        # *visual* frame = control + _visual_offsets, so the engage anchor
        # and IK target marker land where the visual mesh EE actually is
        # (the mesh has the same offsets added in scene.js).  The motor
        # gets q_visual - offsets, leaving its physical motion unchanged.
        self._q_last: np.ndarray = np.zeros(7, dtype=np.float64)
        self._visual_offsets: np.ndarray = np.zeros(7, dtype=np.float64)
        # Until the first real telem arrives, the commanded pose (which the
        # main URDF mirror now shows) sits at home (zeros) — looks "off" vs
        # the real arm.  We seed it from the actual arm on first telem so
        # the mirror starts matching reality; after that it's operator-driven.
        self._q_initialized: bool = False
        self._connected: bool = False
        self._clamped: list[bool] = [False] * 7
        self._poll_t_last: float = 0.0
        # Clutch state
        self._engaged: bool = False
        self._engage_ctrl_pos: Optional[np.ndarray] = None
        self._engage_ctrl_quat: Optional[np.ndarray] = None
        self._engage_R_ee: Optional[np.ndarray] = None
        self._engage_t_ee: Optional[np.ndarray] = None
        # Smoothing
        self._lpf_pos: Optional[np.ndarray] = None
        self._lpf_quat: Optional[np.ndarray] = None
        # Estop latch — driven by the B button.  Cleared by holding both
        # grips for 1 s; also clears on disconnect/close.
        self._estop: bool = False
        # E-stop latch needs a one-shot edge detect — track previous B state.
        self._prev_b: bool = False
        # Dual-grip-hold dwell tracking for clearing the e-stop.
        self._dual_grip_t0: Optional[float] = None
        # Exposed for the WebXR HUD via the telem mirror.
        self._engaged_public: bool = False
        # Latest IK targets in robot base frame — surfaced via hud_snapshot
        # for the in-headset marker overlay.  *_unclamped is the operator's
        # raw commanded pose; *_clamped is after the workspace box (so when
        # the two diverge you're past the wall).
        self._last_target_unclamped_t: Optional[np.ndarray] = None
        self._last_target_clamped_t:   Optional[np.ndarray] = None
        self._last_target_t_time: float = 0.0
        # Per-poll latency metrics (EMA-smoothed so the HUD doesn't flicker).
        self._last_ik_ms: float = 0.0
        self._last_pose_age_ms: float = 0.0
        self._ema_ik_ms: float = 0.0
        self._ema_pose_age_ms: float = 0.0
        # Anti-glitch counters.  _glitch_drops is the total reject count
        # (lifetime), surfaced in hud_snapshot so the operator can spot
        # flaky tracking.  _consec_glitches is the run length — when it
        # crosses a threshold we force-accept the next sample so the LPF
        # can't get permanently wedged behind a stale state.
        self._glitch_drops: int = 0
        self._consec_glitches: int = 0
        # Mutable copies of workspace bounds; can be edited at runtime by
        # quest_command actions without mutating the immutable cfg dataclass.
        self._workspace_min = np.asarray(self.cfg.workspace_min, dtype=np.float64).copy()
        self._workspace_max = np.asarray(self.cfg.workspace_max, dtype=np.float64).copy()

    # ---- duck-typed leader interface ----------------------------------

    def connect(self) -> bool:
        if self._state is None:
            raise RuntimeError(
                "QuestLeader needs a SharedState — pass one via the factory "
                "in leader.py (use kind='quest')."
            )
        self._connected = True
        self._estop = False
        return True

    def close(self) -> None:
        self._connected = False
        self._engaged = False

    def save_zero(self, zero_offsets) -> None:
        """No-op for duck-type compatibility.  The Cartesian leader has no
        persistent encoder zero — the clutch re-anchors on each engage —
        so there's nothing to save.  collect_demo.py guards on
        SUPPORTS_ZEROING and shouldn't reach here, but we implement it
        anyway so any other caller can't crash on a missing attribute."""
        return None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def clamped_joints(self) -> list[bool]:
        return list(self._clamped)

    @property
    def zero_offsets(self) -> np.ndarray:
        # QuestLeader emits desired joint angles directly — identity mapping.
        return np.zeros(7, dtype=np.float32)

    @property
    def directions(self) -> np.ndarray:
        return np.ones(7, dtype=np.float32)

    def set_visual_offsets(self, offsets) -> None:
        """Install the per-joint visual offsets (rad, length 7) used to align
        the URDF mesh with the real arm.  IK/FK here operate in `q_last +
        offsets` (visual frame); the motor receives q_last alone.  Called
        by collect_demo at install and on joint_align.json hot-reload."""
        arr = np.asarray(offsets, dtype=np.float64).reshape(-1)
        if arr.size < 7:
            return
        self._visual_offsets = arr[:7].copy()

    @property
    def last_joystick(self) -> Optional[dict]:
        """Mirror the OpenRB M5 Joystick2 snapshot from the left controller's
        thumbstick + A button.  Lets the existing drive/record-toggle path
        in collect_demo.py work unchanged."""
        if self._state is None or self._state.latest_control is None:
            return None
        left = self._state.latest_control.get("left") or {}
        stick = left.get("stick") or [0.0, 0.0]
        a_btn = bool(left.get("a", False))
        # Increment press_counter on rising edge — matches OpenRB semantics so
        # the existing edge-detector in collect_demo.py just works.
        if not hasattr(self, "_left_a_prev"):
            self._left_a_prev = False
            self._left_a_count = 0
        if a_btn and not self._left_a_prev:
            self._left_a_count += 1
        self._left_a_prev = a_btn
        return {
            "x": float(stick[0]),
            "y": float(stick[1]),
            "button": a_btn,
            "press_counter": int(self._left_a_count),
            "status": 0 if self._state.latest_control else 1,
            "present": True,
        }

    # ---- the hot path -------------------------------------------------

    def poll(self) -> Optional[np.ndarray]:
        """Called by the leader-reader thread at ~500 Hz; we sub-rate
        ourselves to ~60 Hz because the IK isn't free."""
        if not self._connected or self._state is None:
            return None
        now = time.time()
        if (now - self._poll_t_last) < (1.0 / 60.0):
            # Return the last command unchanged so the leader_reader's
            # velocity-EMA gets stable input and the cmd-thread keeps
            # ticking at its own rate.
            return self._q_last.astype(np.float32)
        self._poll_t_last = now

        # Drain any discrete operator commands queued by the in-VR UI before
        # we look at the pose / clutch state.  Commands can rewrite the
        # workspace and force a clutch re-anchor on this tick.
        self._drain_commands(now)

        frame = self._state.latest_control
        telem = self._state.latest_telem

        # Seed the commanded pose from the real arm on first valid telem so
        # the mirror starts matching reality instead of sitting at home.
        if not self._q_initialized:
            cur0 = self._current_qpos(telem, now)
            if cur0 is not None:
                self._q_last = cur0.astype(np.float64).copy()
                self._q_initialized = True

        # Stale-pose handling.
        if frame is None:
            return self._maybe_held_cmd()
        rx_ts = frame.get("_rx_ts", 0.0)
        # Pose age (always tracked — useful for HUD even when disengaged).
        self._last_pose_age_ms = (now - rx_ts) * 1000.0
        self._ema_pose_age_ms = 0.2 * self._last_pose_age_ms + 0.8 * self._ema_pose_age_ms
        if (now - rx_ts) > self.cfg.stale_drop_clutch_s:
            self._engaged = False
            return self._maybe_held_cmd()

        right = frame.get("right") or {}
        left  = frame.get("left")  or {}
        b_btn = bool(right.get("b", False))
        # E-stop edge: B button rising edge latches.
        if b_btn and not self._prev_b:
            self._estop = True
            print("[quest] E-STOP latched (right B). Hold both grips 1s to clear.",
                  flush=True)
        self._prev_b = b_btn
        # Dual-grip-hold clears the latch (requires 1 s of both grips down to
        # avoid accidental release during normal use).
        right_grip = bool(right.get("grip", False))
        left_grip  = bool(left.get("grip",  False))
        if right_grip and left_grip:
            if self._dual_grip_t0 is None:
                self._dual_grip_t0 = now
            elif (now - self._dual_grip_t0) >= 1.0 and self._estop:
                self._estop = False
                self._dual_grip_t0 = None
                print("[quest] E-STOP cleared by dual-grip hold.", flush=True)
        else:
            self._dual_grip_t0 = None
        if self._estop:
            self._engaged = False
            # Hold the current commanded q — do not introduce a step.  The
            # collect_demo.py disable() / E key path is still in the loop
            # and can fully cut motors.
            return self._q_last.astype(np.float32)

        # Current pose + LPF.
        rp = np.asarray(right.get("pos", [0, 0, 0]), dtype=np.float64)
        rq = np.asarray(right.get("quat", [0, 0, 0, 1]), dtype=np.float64)
        rq = rq / (np.linalg.norm(rq) + 1e-12)
        if self._lpf_pos is None:
            self._lpf_pos = rp.copy()
            self._lpf_quat = rq.copy()
            self._consec_glitches = 0
        else:
            # ---- Hemisphere normalize BEFORE any blend/diff -------------
            # Quaternions q and -q represent the same rotation, but a
            # linear blend between them is degenerate — averaging produces
            # a vector pointing in some near-random direction that
            # normalises to a meaningless rotation.  Quest controllers
            # often flip the sign of the reported quaternion between
            # consecutive frames (especially during fast wrist motion);
            # without this fix the LPF would briefly emit nonsense and the
            # IK would chase it as a sudden orientation jump.
            if float(np.dot(rq, self._lpf_quat)) < 0.0:
                rq = -rq

            # Outlier rejection: a single bad sample from hand-tracking can
            # produce a > 30 cm "jump" in one frame.  Drop it from the LPF
            # so the IK uses last-good state, BUT if we reject 3 frames in
            # a row, the "outlier" is the new reality (operator's hand
            # actually moved while we missed frames) — force-accept and
            # re-seed the LPF so we can't get wedged.
            dpos = float(np.linalg.norm(rp - self._lpf_pos))
            # Orientation outlier: angle between rq and the LPF state.
            # 2·acos(|dot|) is robust across hemispheres (we already
            # flipped above).  > ~30° in a single frame is a tracking
            # glitch — the same 3-frame grace as the position path lets
            # genuinely fast wrist motion eventually break through.
            cos_half = abs(float(np.dot(rq, self._lpf_quat)))
            cos_half = max(min(cos_half, 1.0), -1.0)
            dori = 2.0 * float(np.arccos(cos_half))
            if (dpos > 0.30 or dori > 0.50) and self._consec_glitches < 3:
                self._glitch_drops += 1
                self._consec_glitches += 1
            else:
                if self._consec_glitches >= 3:
                    # Recovery path: re-seed instead of LPF-merging so we
                    # snap forward cleanly rather than gradually catching up.
                    self._lpf_pos = rp.copy()
                    self._lpf_quat = rq.copy()
                else:
                    a = float(self.cfg.pose_lpf_alpha)
                    self._lpf_pos = a * rp + (1.0 - a) * self._lpf_pos
                    self._lpf_quat = a * rq + (1.0 - a) * self._lpf_quat
                    self._lpf_quat = self._lpf_quat / (np.linalg.norm(self._lpf_quat) + 1e-12)
                self._consec_glitches = 0

        grip = bool(right.get("grip", False))

        # Clutch FSM.
        if grip and not self._engaged:
            # Rising edge: anchor the clutch.  We anchor to the LAST
            # COMMANDED pose (self._q_last), NOT the real arm — so releasing
            # and re-gripping continues from where you left off rather than
            # snapping the commanded preview back to the (static, in IDLE)
            # real arm on every grip.  _q_last is seeded from the real arm on
            # connect and re-synced by the "Align to arm" button, so the
            # command never starts wildly off from reality.
            if not self._q_initialized:
                cur_qpos = self._current_qpos(telem, now)
                if cur_qpos is None:
                    return self._maybe_held_cmd()  # need a starting pose
                self._q_last = cur_qpos.astype(np.float64).copy()
                self._q_initialized = True
            # FK in visual frame so the engage anchor lines up with the
            # rendered URDF mesh EE (mesh = q_last + offsets in scene.js).
            R_ee, t_ee = self._kin.fk(self._q_last[:6] + self._visual_offsets[:6])
            self._engage_R_ee = R_ee
            self._engage_t_ee = t_ee
            self._engage_ctrl_pos = self._lpf_pos.copy()
            self._engage_ctrl_quat = self._lpf_quat.copy()
            self._engaged = True

        if not grip and self._engaged:
            self._engaged = False
            # Hold position — _q_last is already the last commanded q.
            # Clear the velocity-clamp memory so a future re-engage starts
            # fresh; otherwise the dt since last clamp could be huge.
            self._last_target_clamped_t = None
            self._last_target_unclamped_t = None

        # Trigger -> gripper.
        trig = float(right.get("trigger", 0.0))
        trig = max(0.0, min(1.0, trig))
        gripper_q = self.cfg.gripper_open_rad + trig * (
            self.cfg.gripper_closed_rad - self.cfg.gripper_open_rad
        )

        if self._engaged and self._engage_t_ee is not None:
            target_t_raw, target_quat = self._compute_target_pose()
            target_t = self._clamp_workspace(target_t_raw)
            # Cartesian velocity clamp on the EE target.  Catches edge-of-FOV
            # tracking jumps that survive the LPF — even a 5 cm/frame jump
            # at 60 Hz = 300 cm/s, well above any human reach speed, so
            # capping at 60 cm/s removes it without touching real motion.
            #
            # dt is CAPPED at 33 ms (1/30 s) so a long pause (browser
            # stutter, release-then-reengage, network blip) can't grant a
            # giant max_step on the first resumed tick.  Worst case: arm
            # smoothly catches up at the configured velocity over a few
            # frames rather than snapping forward all at once.
            if self._last_target_clamped_t is not None:
                t_dt = max(min(now - self._last_target_t_time, 1.0 / 30.0), 1e-3)
                t_max_step = float(self.cfg.cartesian_max_vel_m_s) * t_dt
                t_delta = target_t - self._last_target_clamped_t
                t_norm = float(np.linalg.norm(t_delta))
                if t_norm > t_max_step:
                    target_t = self._last_target_clamped_t + t_delta * (t_max_step / t_norm)
            self._last_target_unclamped_t = target_t_raw.copy()
            self._last_target_clamped_t   = target_t.copy()
            self._last_target_t_time = now
            _t_ik_start = time.perf_counter()
            # Seed IK in visual frame; solution comes out in visual frame
            # too.  The visual-frame seed matches what the mesh shows, so
            # the IK converges relative to the user's perceived pose.
            _vo6 = self._visual_offsets[:6]
            q_last_visual = self._q_last[:6] + _vo6
            _jw = (np.asarray(self.cfg.ik_joint_weights, dtype=np.float64)
                   if self.cfg.ik_joint_weights is not None else None)
            res = solve_ik(
                self._kin,
                q_init=q_last_visual,
                target_pos=target_t,
                target_quat=target_quat,
                pos_weight=self.cfg.ik_pos_weight,
                ori_weight=self.cfg.ik_ori_weight,
                damping=self.cfg.ik_damping,
                joint_weights=_jw,
                max_iter=self.cfg.ik_max_iter,
            )
            self._last_ik_ms = (time.perf_counter() - _t_ik_start) * 1000.0
            self._ema_ik_ms = 0.2 * self._last_ik_ms + 0.8 * self._ema_ik_ms
            q_arm_visual = res.q.astype(np.float64)
            # Per-joint velocity clamp on the leader's emitted stream (in
            # visual frame so the clamp tracks visible joint motion).
            dt = max(now - self._poll_t_last_for_vel(), 1e-3)
            self._poll_t_last_set_vel(now)
            max_step = self.cfg.max_joint_vel * dt
            q_arm_visual = np.minimum(
                np.maximum(q_arm_visual, q_last_visual - max_step),
                q_last_visual + max_step,
            )
            # Subtract offsets to get back to control frame for the motor.
            self._q_last[:6] = q_arm_visual - _vo6
            self._clamped[:6] = list(res.clamped)
        else:
            # Disengaged: hold the last arm command (no motion).
            self._clamped[:6] = [False] * 6

        # Gripper applies in both engaged and disengaged states — it's its
        # own DoF and operators expect it to track the trigger continuously.
        self._q_last[6] = gripper_q
        self._clamped[6] = False
        self._engaged_public = self._engaged

        return self._q_last.astype(np.float32)

    # ---- HUD snapshot (read by collect_demo.py to publish into telem) ----

    def hud_snapshot(self) -> dict:
        """Read-only state dict for the WebXR HUD.  Safe to call from any
        thread — all fields are scalar reads of attributes set in poll()."""
        out: dict = {
            "engaged":       bool(self._engaged_public),
            "estop":         bool(self._estop),
            "workspace_min": self._workspace_min.tolist(),
            "workspace_max": self._workspace_max.tolist(),
            "ik_ms":         float(self._ema_ik_ms),
            "pose_age_ms":   float(self._ema_pose_age_ms),
            "glitch_drops":  int(self._glitch_drops),
            # Commanded joint vector — what the leader is asking the arm
            # to reach.  Used by the browser ghost-URDF overlay: it stays
            # invisible while the real arm tracks the command, and floats
            # ahead in yellow when there's lag / saturation / collision.
            "qcmd":          self._q_last.astype(float).tolist(),
        }
        if self._state is not None and self._state.stats:
            out["control_hz"] = float(self._state.stats.get("control_rx_hz", 0.0))
        if self.cfg.reachable_min is not None and self.cfg.reachable_max is not None:
            out["reachable_min"] = list(self.cfg.reachable_min)
            out["reachable_max"] = list(self.cfg.reachable_max)
        # IK marker positions for the scene overlay — green = engage anchor,
        # yellow = clamped target, red line drawn when raw target diverges.
        if self._engage_t_ee is not None:
            out["engage_ee"] = self._engage_t_ee.astype(float).tolist()
        if self._last_target_clamped_t is not None:
            out["target_ee"] = self._last_target_clamped_t.astype(float).tolist()
        if self._last_target_unclamped_t is not None:
            out["target_ee_raw"] = self._last_target_unclamped_t.astype(float).tolist()
        return out

    # ---- discrete commands from the in-VR UI ---------------------------

    def _drain_commands(self, now: float) -> None:
        """Pop and apply every command in shared_state.pending_commands."""
        if self._state is None or not self._state.pending_commands:
            return
        # collections.deque popleft is atomic in CPython, so this is safe
        # without a lock for the single-reader case.
        while self._state.pending_commands:
            try:
                cmd = self._state.pending_commands.popleft()
            except IndexError:
                break
            try:
                self._apply_command(cmd, now)
            except Exception as exc:
                print(f"[quest] command {cmd!r} failed: {exc}", flush=True)

    def _apply_command(self, cmd: dict, now: float) -> None:
        name = cmd.get("cmd")
        if name == "realign":
            self._do_realign(now)
        elif name == "grow_workspace":
            self._scale_workspace(float(cmd.get("factor", 1.10)))
        elif name == "shrink_workspace":
            self._scale_workspace(float(cmd.get("factor", 0.90)))
        elif name == "center_workspace_on_ee":
            self._center_workspace_on_ee()
        elif name == "reset_workspace":
            self._workspace_min = np.asarray(self.cfg.workspace_min, dtype=np.float64).copy()
            self._workspace_max = np.asarray(self.cfg.workspace_max, dtype=np.float64).copy()
        elif name == "reset_sim":
            # Reset the COMMANDED pose to home (all joints zero) and drop the
            # clutch so the next grip re-anchors cleanly.  Use when the
            # kinematic sim has wandered into an awkward configuration.
            self._q_last = np.zeros(7, dtype=np.float64)
            self._engaged = False
            self._last_target_clamped_t = None
            self._last_target_unclamped_t = None
            print("[quest] sim reset to home", flush=True)
        elif name == "align_to_actual":
            # Snap the COMMANDED pose to the real arm's current position so
            # the kinematic sim matches reality — eliminates the jump when
            # motors are first enabled.  Reads actual qpos from telem.
            cur = self._current_qpos(self._state.latest_telem if self._state else None, now)
            if cur is not None:
                self._q_last = cur.astype(np.float64).copy()
                self._engaged = False
                self._last_target_clamped_t = None
                self._last_target_unclamped_t = None
                print("[quest] sim aligned to actual arm", flush=True)
            else:
                print("[quest] align_to_actual: no fresh telem", flush=True)
        else:
            print(f"[quest] unknown command: {name!r}", flush=True)

    def _do_realign(self, now: float) -> None:
        """Atomically re-anchor the clutch frame using the CURRENT controller
        pose + EE pose, regardless of engagement state.

        Engaged:  re-captures engage_{ctrl,ee,yaw} so the next tick continues
                  driving from delta=0, with no "unresponsive" window where
                  _engaged briefly drops to False.

        Disengaged:  resets the LPF so the next clutch engagement gets a
                     fresh pose buffer (avoids stale data from before realign).
        """
        # Always blow away the LPF so we don't carry smoothed-out lag.
        self._lpf_pos = None
        self._lpf_quat = None
        if not self._engaged or self._state is None:
            return
        frame = self._state.latest_control
        telem = self._state.latest_telem
        if frame is None:
            return
        cur_qpos = self._current_qpos(telem, now)
        if cur_qpos is None:
            return
        right = frame.get("right") or {}
        rp = np.asarray(right.get("pos", [0, 0, 0]), dtype=np.float64)
        rq = np.asarray(right.get("quat", [0, 0, 0, 1]), dtype=np.float64)
        rq = rq / (np.linalg.norm(rq) + 1e-12)

        # Rebuild ALL engage state in place; _engaged stays True so the IK
        # keeps tracking through this tick.
        self._lpf_pos = rp.copy()
        self._lpf_quat = rq.copy()
        self._q_last = cur_qpos.astype(np.float64).copy()
        # Visual-frame FK so re-engage anchor matches the rendered mesh EE.
        R_ee, t_ee = self._kin.fk(self._q_last[:6] + self._visual_offsets[:6])
        self._engage_R_ee = R_ee
        self._engage_t_ee = t_ee
        self._engage_ctrl_pos = rp.copy()
        self._engage_ctrl_quat = rq.copy()

    def _scale_workspace(self, factor: float) -> None:
        """Resize the workspace box by `factor` (e.g. 1.10 = 10% bigger)
        about its current center.  Clamped to [5 cm, 2 m] half-extents to
        keep things sane."""
        center = (self._workspace_min + self._workspace_max) * 0.5
        half = (self._workspace_max - self._workspace_min) * 0.5 * factor
        half = np.clip(half, 0.05, 2.0)
        self._workspace_min = center - half
        self._workspace_max = center + half

    def _center_workspace_on_ee(self) -> None:
        """Slide the workspace box so its center coincides with the current
        EE position.  Keeps size; just shifts.  Lets the operator re-frame
        the box without resizing it."""
        # FK in visual frame so the box centers on the mesh EE the user sees.
        try:
            q_vis = (self._q_last[:6].astype(np.float64)
                     + self._visual_offsets[:6])
            t_ee = self._kin.fk(q_vis)[1]
        except Exception:
            return
        size = (self._workspace_max - self._workspace_min) * 0.5
        self._workspace_min = t_ee - size
        self._workspace_max = t_ee + size

    # ---- helpers -------------------------------------------------------

    def _maybe_held_cmd(self) -> Optional[np.ndarray]:
        """Return the last command if we have one, else None (= no-op)."""
        if not np.any(self._q_last):
            return None
        return self._q_last.astype(np.float32)

    def _current_qpos(self, telem: Optional[dict], now: float) -> Optional[np.ndarray]:
        """Pull current 7-DoF qpos from the latest telem frame, if fresh."""
        if telem is None:
            return None
        ts = float(telem.get("ts", 0.0))
        if (now - ts) > self.cfg.stale_telem_s:
            return None
        q = telem.get("qpos")
        if q is None or len(q) < 7:
            return None
        return np.asarray(q[:7], dtype=np.float64)

    def _compute_target_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Compose Δpose (XR frame) onto engage_ee (robot frame) -> target EE.

        No head-yaw correction: the operator faces the robot in physical
        space, and XR forward IS robot forward.  This makes "push your
        hand forward" always move the arm forward in the robot's frame,
        independent of where the visual robot model is placed in the VR
        scene or which way the user happens to be looking when they
        engage the clutch.
        """
        # Δ in WebXR frame, axes-remapped to robot frame.
        dp_xr = self._lpf_pos - self._engage_ctrl_pos
        dq_xr = _quat_mul(self._lpf_quat, _quat_conj(self._engage_ctrl_quat))
        dp_robot = _R_XR_TO_ROBOT @ dp_xr
        R_dq_xr = quat_to_R(dq_xr)
        R_dq_robot = _R_XR_TO_ROBOT @ R_dq_xr @ _R_XR_TO_ROBOT.T
        target_t = self._engage_t_ee + dp_robot
        target_R = R_dq_robot @ self._engage_R_ee
        target_quat = R_to_quat(target_R)
        return target_t, target_quat

    def _clamp_workspace(self, t: np.ndarray) -> np.ndarray:
        # Use mutable copies so runtime actions (grow/shrink/center) can
        # adjust the clamp without rebuilding the leader.
        return np.minimum(np.maximum(t, self._workspace_min), self._workspace_max)

    # Velocity-clamp dt accounting — kept on a separate field from the
    # poll-rate gate so the two timings don't interfere.
    def _poll_t_last_for_vel(self) -> float:
        return getattr(self, "_vel_t_last", time.time() - 0.033)
    def _poll_t_last_set_vel(self, t: float) -> None:
        self._vel_t_last = t


# -----------------------------------------------------------------------------
# Factory hook — used by leader.py so collect_demo.py's
# get_leader_class("quest") returns a class that doesn't need the
# SharedState passed in the open() call (the factory binds it).
# -----------------------------------------------------------------------------

def make_quest_leader_class(
    shared_state,
    config: Optional[QuestLeaderConfig] = None,
    *,
    joint_limits_path: Optional[Path | str] = None,
):
    """Return a subclass with `shared_state`, `config`, and the joint-limit
    overlay pre-bound, so `cls(port, baud, calib=...)` works exactly like
    the other leaders."""
    captured_state = shared_state
    captured_cfg = config or QuestLeaderConfig()
    captured_limits = joint_limits_path

    class _BoundQuestLeader(QuestLeader):
        def __init__(self, port=None, baud=None, calib=None) -> None:
            super().__init__(
                port=port, baud=baud, calib=calib,
                shared_state=captured_state, config=captured_cfg,
                joint_limits_path=captured_limits,
            )

    _BoundQuestLeader.__name__ = "QuestLeader"
    return _BoundQuestLeader
