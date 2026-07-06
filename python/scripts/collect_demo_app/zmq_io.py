"""ZMQ send helpers, cmd-sender thread, and tracking-bundle math (from collect_demo.py)."""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import zmq

from common.arm_constants import clamp_arm_positions
from common.wire import pack_msg, unpack_msg

from . import alignment
from .alignment import _urdf_to_motor_arm_payload
from .runtime import NUM_JOINTS
from .telem import _qpos

# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def _drain(sock) -> Optional[dict]:
    latest = None
    while True:
        try:
            latest = unpack_msg(sock.recv(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


_cmd_sock_lock = threading.Lock()  # serialises cmd_sock.send across threads


def _send(sock, msg: dict) -> None:
    """Send one message.  Holds `_cmd_sock_lock` so direct sends from the
    main loop (enable/disable/emergency_stop/replay) don't race with the
    dedicated cmd-sender thread on the same PUSH socket.

    Boundary transform: `arm_joints` messages flow in URDF frame
    internally; convert to motor frame on the way out so callers don't
    need to remember to do it themselves.  `bundle` messages with an
    embedded arm_joints field get the same treatment."""
    if isinstance(msg, dict):
        if msg.get("type") == "arm_joints":
            msg = _urdf_to_motor_arm_payload(msg)
        elif msg.get("type") == "bundle" and isinstance(msg.get("arm_joints"), dict):
            msg = dict(msg)
            msg["arm_joints"] = _urdf_to_motor_arm_payload(msg["arm_joints"])
    try:
        with _cmd_sock_lock:
            sock.send(pack_msg(msg), zmq.NOBLOCK)
    except zmq.Again:
        pass


def _build_bundle(
    arm: Optional[dict] = None,
    drive: Optional[dict] = None,
) -> Optional[dict]:
    """Construct a Bundle message.  Returns None if both sub-payloads are None.

    Swivel is part of `arm` (joint 0) — there is no separate swivel field.
    """
    if arm is None and drive is None:
        return None
    msg: dict = {"type": "bundle"}
    if arm is not None:
        msg["arm_joints"] = arm
    if drive is not None:
        msg["drive"] = drive
    return msg


def _send_bundle(
    sock,
    arm: Optional[dict] = None,
    drive: Optional[dict] = None,
) -> None:
    """Send a Bundle once, synchronously.  For one-shots (live replay) and
    shutdown paths.  The 30 Hz teleop path goes through the dedicated
    cmd-sender thread instead — see `_start_cmd_sender`.

    `arm.positions` is expected in URDF frame; this function applies the
    inverse alignment transform on the way out so the Rust driver sees
    motor-frame values.  See _urdf_to_motor_arm_payload for the math."""
    if arm is not None:
        arm = _urdf_to_motor_arm_payload(arm)
    msg = _build_bundle(arm, drive)
    if msg is None:
        return
    try:
        with _cmd_sock_lock:
            sock.send(pack_msg(msg), zmq.NOBLOCK)
    except zmq.Again:
        pass


# ---------------------------------------------------------------------------
# Dedicated command sender thread
# ---------------------------------------------------------------------------
# Re-emits the latest queued Bundle at a higher rate than the 30 Hz main
# loop, so the controller's PD loop sees a fresh frame within ~10 ms even
# when the main loop is mid-tick.  Also pays the msgpack pack + ZMQ send
# cost off the main loop (used to be the dominant `motor` p99 spike).

_CMD_SEND_HZ = 100   # send cadence; 10 ms tick

# Lead-clamp is skipped when telem age exceeds this — see #3 in the
# main-loop lag analysis.  Otherwise a stale q_actual would pin the
# command behind the arm's actual progress and gate fast motion.
_LEAD_CLAMP_TELEM_FRESH_S = 0.10


def _start_cmd_sender(
    sock,
    lr_lock: threading.Lock,
    lr_latest: dict,
    lr_mapping: dict,
    telem_lock: threading.Lock,
    telem_cache: dict,
    hz: int = _CMD_SEND_HZ,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    """Dedicated cmd-sender thread.

    Two modes (set by main loop via the holder):

    * "static"   — re-emits a prebuilt Bundle dict every tick.  Used for
                   IDLE / HOLD / ENGAGING / SHUTDOWN, where the target is
                   either frozen or following a slow ramp the main loop
                   integrates at 30 Hz.

    * "tracking" — computes q_cmd live from `_lr_latest` at the cmd-thread
                   rate (100 Hz), so leader→motor latency is bounded by
                   ~10 ms instead of ~33 ms.  The rate limit is per-time
                   (`max_vel * dt`) rather than per-tick, and the lead-clamp
                   relaxes when telemetry is stale (see _LEAD_CLAMP_TELEM_FRESH_S).

    Holder shape (under the returned lock):
        {
          "mode":     "static" | "tracking" | None,
          "bundle":   {...}  | None,                   # used when mode=="static"
          "tracking": {kp, kd, max_vel, max_lead,
                       vel_ff, drive, arm_limits} | None,
        }
    """
    lock   = threading.Lock()
    # `last_q_cmd` is the current commanded arm pose — main loop reads it
    # for recording / display / shutdown-target seeding regardless of mode.
    holder: dict = {"mode": None, "bundle": None, "tracking": None,
                    "last_q_cmd": None}
    stop   = threading.Event()
    period = 1.0 / hz

    def _run() -> None:
        next_t = time.perf_counter() + period
        last_t: Optional[float] = None
        last_mode: Optional[str] = None
        last_q_cmd: Optional[np.ndarray] = None

        while not stop.is_set():
            now_pc = time.perf_counter()
            dt = (now_pc - last_t) if last_t is not None else period
            last_t = now_pc

            with lock:
                mode     = holder.get("mode")
                bundle   = holder.get("bundle")
                tracking = holder.get("tracking")

            # Reset the rate-limit integrator on any mode transition so the
            # first tick after entering "tracking" doesn't apply a step
            # accumulated from a prior tracking session.
            if mode != last_mode:
                last_q_cmd = None
                last_mode = mode

            msg: Optional[dict] = None

            if mode == "tracking" and tracking is not None:
                msg = _compute_tracking_bundle(
                    tracking, dt, last_q_cmd,
                    lr_lock, lr_latest, lr_mapping,
                    telem_lock, telem_cache,
                )
                if msg is not None:
                    # Pull the q_cmd back out of the bundle for the next
                    # tick's rate-limit integrator and for the main loop
                    # (recording / display).
                    arm = msg.get("arm_joints", {})
                    pos = arm.get("positions")
                    if pos is not None:
                        last_q_cmd = np.asarray(pos, dtype=np.float32)
                        with lock:
                            holder["last_q_cmd"] = last_q_cmd
            elif mode == "static" and bundle is not None:
                msg = bundle
                # Mirror the static bundle's arm positions into last_q_cmd
                # so callers that read `holder["last_q_cmd"]` see the same
                # value regardless of mode.
                arm = bundle.get("arm_joints") or {}
                pos = arm.get("positions")
                if pos is not None:
                    with lock:
                        holder["last_q_cmd"] = np.asarray(pos, dtype=np.float32)

            if msg is not None:
                # Boundary: arm_joints flows in URDF frame internally but the
                # Rust motor driver expects motor frame.  Apply the inverse
                # alignment transform on a SHALLOW COPY so holder['bundle']
                # and last_q_cmd stay in URDF frame for callers that read
                # them.  drive/other top-level fields pass through unchanged.
                arm = msg.get("arm_joints")
                if arm is not None:
                    msg = dict(msg)
                    msg["arm_joints"] = _urdf_to_motor_arm_payload(arm)
                try:
                    with _cmd_sock_lock:
                        sock.send(pack_msg(msg), zmq.NOBLOCK)
                except zmq.Again:
                    pass
                except Exception:
                    pass

            sleep_t = next_t - time.perf_counter()
            if sleep_t > 0:
                stop.wait(sleep_t)
            next_t += period
            if next_t < time.perf_counter():
                next_t = time.perf_counter() + period

    thread = threading.Thread(target=_run, daemon=True, name="CmdTx")
    thread.start()
    return stop, thread, lock, holder


def _compute_tracking_bundle(
    cfg: dict,
    dt: float,
    last_q_cmd: Optional[np.ndarray],
    lr_lock: threading.Lock,
    lr_latest: dict,
    lr_mapping: dict,
    telem_lock: threading.Lock,
    telem_cache: dict,
) -> Optional[dict]:
    """Build a TRACKING-mode arm_joints bundle from the latest leader sample.

    Returns None when the leader has no sample yet, no mapping is
    installed, or the leader pose is too stale to trust.
    """
    # Snapshot leader + mapping under one lock acquisition.
    with lr_lock:
        leader_rad     = lr_latest.get("rad")
        leader_vel     = lr_latest.get("vel")
        leader_t       = lr_latest.get("time", 0.0)
        zero_offsets   = lr_mapping.get("zero_offsets")
        directions     = lr_mapping.get("directions")
        for_aizee      = lr_mapping.get("for_aizee")
        emits_urdf     = lr_mapping.get("emits_urdf", False)

    if (leader_rad is None or zero_offsets is None
            or directions is None or for_aizee is None):
        return None

    # Map leader → AIZEE target.  Physical leaders' `directions` array
    # was pre-tuned to match motor-encoder direction (predating
    # joint_align), so the raw mapped vector is in MOTOR frame.  Fold in
    # _ALIGN_SIGNS to convert to URDF frame here; the cmd-thread boundary
    # transform will fold the same signs back out before sending, leaving
    # the motor command identical to the pre-joint_align behaviour while
    # downstream (recording, holder["last_q_cmd"]) sees URDF frame.
    # QuestLeader already emits URDF, so it skips this fold.
    mapped = directions * (leader_rad - zero_offsets)
    target = mapped[for_aizee]
    if not emits_urdf:
        target = target * alignment._ALIGN_SIGNS
    if leader_vel is not None:
        vel_ff_arr = (directions * leader_vel)[for_aizee]
        if not emits_urdf:
            vel_ff_arr = vel_ff_arr * alignment._ALIGN_SIGNS
    else:
        vel_ff_arr = None

    # Telemetry q_actual + age — used for the lead-clamp.
    with telem_lock:
        telem_msg = telem_cache.get("msg")
        telem_t   = telem_cache.get("time", 0.0)
    q_actual = _qpos(telem_msg) if telem_msg is not None else None
    telem_age = time.time() - telem_t if telem_t > 0 else 999.0

    # Reference for the rate limit.
    if last_q_cmd is not None:
        ref = last_q_cmd.copy()
        # Lead-clamp only when telemetry is fresh.  When telem is stale the
        # clamp would pin the command behind a known-stale q_actual; trust
        # the velocity rate-limit alone in that case.
        if q_actual is not None and telem_age < _LEAD_CLAMP_TELEM_FRESH_S:
            max_lead = float(cfg.get("max_lead", 0.6))
            ref = q_actual + np.clip(ref - q_actual, -max_lead, max_lead)
    elif q_actual is not None:
        ref = q_actual
    else:
        ref = target

    # Velocity rate-limit, per-time (was per-30Hz-tick in the old path).
    max_vel  = float(cfg.get("max_vel", 9.0))   # rad/s
    max_step = max_vel * max(dt, 1e-3)
    delta    = np.clip(target - ref, -max_step, max_step)
    q_cmd    = ref + delta

    arm_limits = cfg.get("arm_limits")
    clamped_mask: Optional[np.ndarray] = None
    if arm_limits:
        q_cmd_clamped = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
        clamped_mask  = q_cmd_clamped != q_cmd
        q_cmd         = q_cmd_clamped

    # Zero vel_ff on any joint pinned at its limit. Without this, the leader's
    # trigger velocity keeps driving firmware kd·(v_cmd − v_actual) torque
    # against the mechanical stop, and trigger chatter knocks the motor back
    # and forth around the limit.
    if cfg.get("vel_ff", True) and vel_ff_arr is not None:
        vel_arr = vel_ff_arr.copy()
        if clamped_mask is not None and clamped_mask.any():
            vel_arr[clamped_mask] = 0.0
        vel = vel_arr.tolist()
    else:
        vel = [0.0] * NUM_JOINTS

    bundle = {
        "type": "bundle",
        "arm_joints": {
            "positions":  q_cmd.tolist(),
            "velocities": vel,
            "kp":         cfg.get("kp"),
            "kd":         cfg.get("kd"),
            "torques":    [0.0] * NUM_JOINTS,
        },
    }
    drive = cfg.get("drive")
    if drive is not None:
        bundle["drive"] = drive
    return bundle
