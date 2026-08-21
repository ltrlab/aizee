#!/usr/bin/env python3
"""
collect_minerva.py — integrated teleop + data-collection app for Minerva.

The Minerva analog of AIZEE's `collect_demo.py --gui`: a 30 Hz capture loop that
drives the 17-DoF bimanual follower from two OpenRB-150 leader arms (+ joystick
head/lift, + keyboard/GUI jog) AND records v6 episodes (3 cameras + language
instruction) via save_minerva_episode.

Architecture (mirrors collect_demo_app): the main thread runs the loop; daemon
threads handle telemetry, cameras, image decode, leader polling, and the 100 Hz
command re-emit. A PySide6 GUI (optional, `--gui`) runs on its own worker
thread, exchanging a display snapshot + raw camera JPEGs + a key queue with the
loop through lock-guarded holders.

State machine:
    DISABLED -- E --> HOLD -- T --> TELEOP        (T toggles HOLD<->TELEOP)
       ^--------- H ------ (any)     recording (R) allowed only in TELEOP

Usage:
    python collect_minerva.py --gui --output-dir episodes/minerva \
        --instruction "pick up the red block"
    python collect_minerva.py --no-gui            # terminal status only
    python collect_minerva.py --dry-run           # never send motor commands
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import time
from enum import Enum
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import zmq

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))          # python/         -> common.*
sys.path.insert(0, str(_HERE))                 # python/scripts/ -> collect_*_app
sys.path.insert(0, str(_HERE.parent / "teleop"))  # python/teleop/ -> openrb_leader

from common.minerva_constants import (
    CAMERAS, KD, KP, MINERVA_JOINTS, NUM_MINERVA_JOINTS,
    apply_safety_limits, lead_cap_vector, max_delta_vector,
)
from collect_minerva_app import config as mcfg
from collect_minerva_app.follower import DualArmTransport
from collect_minerva_app.images import start_image_decoder, raw_jpeg
from collect_minerva_app.receivers import start_cam_receiver
from collect_minerva_app.recording import RecordingSession, start_async_save
from collect_minerva_app.teleop import MinervaTeleop
from collect_minerva_app.settings import CollectorSettings

LOOP_HZ = 30
REC_HZ = 20
CAM_STALE = 0.5


class State(Enum):
    DISABLED = "DISABLED"
    IDLE = "IDLE"          # enabled, zero-torque (backdrive)
    HOLD = "HOLD"
    ENGAGING = "ENGAGING"  # torque-capped ramp to the leader before live tracking
    TELEOP = "TELEOP"
    SHUTDOWN = "SHUTDOWN"  # ramp to zero, then disable


# Engage onset: on T, ramp the torque cap 0 -> full over this many seconds for a gentle
# start, then promote to live TELEOP. Time-based, so it ALWAYS completes quickly and never
# stalls waiting for the follower to arrive. Tracking speed itself is set by kp_scale (slider).
ENGAGE_RAMP_S = 0.3


def parse_args():
    p = argparse.ArgumentParser(description="Minerva teleop + data collection")
    p.add_argument("--config", default=None)
    p.add_argument("--gui", dest="gui", action="store_true", default=True)
    p.add_argument("--no-gui", dest="gui", action="store_false")
    p.add_argument("--output-dir", default="episodes/minerva")
    p.add_argument("--instruction", default="", help="task/language string for episodes")
    p.add_argument("--task-id", type=int, default=None)
    p.add_argument("--notes", default="")
    p.add_argument("--max-steps", type=int, default=10000)
    p.add_argument("--dry-run", action="store_true", help="never send motor commands")
    p.add_argument("--kp-scale", type=float, default=0.3,
                   help="scale on the arm KP (start low for first motion; raise once trusted)")
    # leader ports (auto-detected if omitted)
    p.add_argument("--left-port", default=None)
    p.add_argument("--right-port", default=None)
    p.add_argument("--left-calib", default=None)
    p.add_argument("--right-calib", default=None)
    # dual follower endpoints (Path A: one motor_control instance per arm).
    # Default to <host>:5555/5556 (left) and :5557/5558 (right); override per-arm.
    p.add_argument("--host", default=None, help="Jetson host for default arm endpoints")
    p.add_argument("--left-cmd", default=None)
    p.add_argument("--left-telem", default=None)
    p.add_argument("--right-cmd", default=None)
    p.add_argument("--right-telem", default=None)
    return p.parse_args()


def _read_decoded(dec_lock, dec_cache, now) -> Optional[tuple]:
    """Return (frames{name:rgb}, cam_ts{name:ts}) only if every camera has a
    FRESH decoded frame. Freshness is gated on the decoded frame's OWN host
    receive time (not the raw receive time), so a lagging or dead decoder
    correctly yields a drop instead of recording a stale frame."""
    frames, cam_ts = {}, {}
    for c in CAMERAS:
        with dec_lock:
            img = dec_cache.get(c)
            ts = dec_cache.get(f"{c}_ts")
            recv_t = dec_cache.get(f"{c}_recv_time")
        if img is None or recv_t is None or (now - recv_t) > CAM_STALE:
            return None
        frames[c] = img
        cam_ts[c] = ts
    return frames, cam_ts


def _repo_config(name: str) -> Path:
    """Absolute path to config/<name> (robust to the process cwd)."""
    return Path(__file__).resolve().parents[2] / "config" / name


def _resolve_leader_calib(side: str, override: Optional[str]) -> Optional[str]:
    """Prefer a PER-LEADER calib (config/openrb_left.json / openrb_right.json, created by
    `openrb_calibrate.py --output ...`); else fall back to OpenRBLeader's shared default.
    Two physical leaders have different encoder offsets, so each needs its own file — a
    single shared calib clamps whichever leader it doesn't match (joints 'stuck' until you
    rotate into the wrong window)."""
    if override:
        return override
    per = _repo_config(f"openrb_{side}.json")
    return str(per) if per.exists() else None


def _apply_follower_calibration(path: Path) -> int:
    """Update the shared JOINT_LIMITS in place from minerva_calibrate.py's output
    (config/minerva_calibration.json, {joints: {name: {min_rad, max_rad}}}) so the diff
    bars AND the safety clamp use each arm's REAL measured travel instead of the seeded
    placeholder limits. Returns the count of joints updated. Degenerate/uncaptured joints
    (max<=min) are skipped."""
    import common.minerva_constants as mc
    if not path.exists():
        return 0
    try:
        joints = json.loads(path.read_text()).get("joints", {})
    except Exception as e:   # noqa: BLE001
        print(f"[calib] follower calibration parse failed: {e}")
        return 0
    n = 0
    for name, jc in joints.items():
        if name not in MINERVA_JOINTS:
            continue
        # The wizard captures min_rad at the physical MIN and max_rad at the physical
        # MAX; the encoder's sign means those can come out inverted (min_rad > max_rad).
        # JOINT_LIMITS just wants [lower, upper], so SORT — don't discard (discarding left
        # those joints on the placeholder limits that clamped them).
        a, b = float(jc.get("min_rad", 0.0)), float(jc.get("max_rad", 0.0))
        lo, hi = min(a, b), max(a, b)
        if hi - lo < 1e-3:                 # joint wasn't actually moved during capture
            continue
        idx = MINERVA_JOINTS.index(name)
        mc.JOINT_LIMITS[idx, 0] = lo
        mc.JOINT_LIMITS[idx, 1] = hi
        n += 1
    return n


def main():
    args = parse_args()
    cfg = mcfg.load_config(args.config)
    ep = cfg["endpoints"]
    host = mcfg.resolve_jetson_host(args.host or "192.168.0.27")
    arms_ep = ep.get("arms", {})

    def _arm_ep(side: str, kind: str, port: int) -> str:
        override = getattr(args, f"{side}_{'cmd' if kind == 'command' else 'telem'}")
        if override:
            return override
        return arms_ep.get(side, {}).get(kind) or f"tcp://{host}:{port}"

    left_cmd = _arm_ep("left", "command", 5555)
    left_telem = _arm_ep("left", "telemetry", 5556)
    right_cmd = _arm_ep("right", "command", 5575)   # 5557-5560 = aizee-camera-relay
    right_telem = _arm_ep("right", "telemetry", 5576)
    cam_eps = mcfg.camera_endpoints(cfg)
    cam_sizes = mcfg.camera_sizes(cfg)
    safe = mcfg.safety(cfg)
    max_delta = max_delta_vector(
        arm=float(safe.get("max_delta_arm", 0.30)),
        gripper=float(safe.get("max_delta_gripper", 0.50)),
        head=float(safe.get("max_delta_head", 0.15)),
        lift=float(safe.get("max_delta_lift", 0.01)))
    kp_cmd = [float(k) * args.kp_scale for k in KP]   # scaled-down gains for first motion
    # Per-joint torque-based LEAD cap: bounds (command − actual) so PD torque never
    # exceeds each joint's nominal saturation. Used for both the engage ramp and live
    # tracking — tighter than the flat velocity guard on the high-kp joints.
    lead_cap = lead_cap_vector(kp_cmd, max_delta)
    # Live speed: the GUI slider writes gui_params["kp_scale"]; the loop rebuilds kp_cmd +
    # lead_cap when it changes (lead_cap auto-tightens as kp rises, so torque stays safe).
    gui_params = {"kp_scale": float(args.kp_scale)}
    cur_kp_scale = float(args.kp_scale)
    # Follower joint limits: override the seeded placeholder JOINT_LIMITS with the REAL
    # measured travel from minerva_calibrate.py (if present) so the diff bars and the
    # safety clamp use true ranges. Mutates the shared array before the GUI/loop read it.
    _nf = _apply_follower_calibration(_repo_config("minerva_calibration.json"))
    if _nf:
        print(f"[calib] follower limits: {_nf} joints from minerva_calibration.json")
    out_dir = args.output_dir
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    meta = {"language_instruction": args.instruction, "notes": args.notes,
            "task_id": args.task_id}

    print("=" * 64)
    print("Minerva collector — teleop + record  (dual-arm, Path A)")
    print(f"  left : cmd={left_cmd}  telem={left_telem}")
    print(f"  right: cmd={right_cmd}  telem={right_telem}")
    print(f"  cameras={cam_eps}")
    print(f"  output={out_dir}  gui={args.gui}  dry_run={args.dry_run}")
    print("=" * 64)

    ctx = zmq.Context()
    follower = DualArmTransport(
        ctx, left_cmd=left_cmd, left_telem=left_telem,
        right_cmd=right_cmd, right_telem=right_telem)

    cam_stop, cam_thread, cam_lock, cam_cache = start_cam_receiver(ctx, cam_eps)
    dec_stop, dec_thread, dec_lock, dec_cache = start_image_decoder(
        cam_lock, cam_cache, CAMERAS, cam_sizes, always_on=True)

    # Per-user prefs (leader-swap routing is persisted here; the GUI's Swap button
    # is the sole writer, this reads the startup value).
    collector_settings = CollectorSettings()
    teleop = MinervaTeleop(
        left_port=args.left_port, right_port=args.right_port,
        left_calib=_resolve_leader_calib("left", args.left_calib),
        right_calib=_resolve_leader_calib("right", args.right_calib),
        swap=bool(collector_settings.get("leader_swap")))
    teleop.connect(verbose=True)

    session = RecordingSession(CAMERAS)
    save_threads: list = []
    save_result: dict = {}
    save_lock = __import__("threading").Lock()

    # GUI wiring (holders + key queue), or terminal renderer.
    gui_queue: "queue.Queue[str]" = queue.Queue(maxsize=32)
    label_queue: "queue.Queue[str]" = queue.Queue(maxsize=64)
    qt = None
    if args.gui:
        try:
            from collect_minerva_gui import QtRenderer
            qt = QtRenderer(cmd_queue=gui_queue, meta=meta, teleop=teleop,
                            cameras=CAMERAS, output_dir=out_dir, label_queue=label_queue,
                            params=gui_params)
            qt.start()
        except Exception as e:
            print(f"[gui] failed to start ({e}); falling back to terminal")
            qt = None

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------
    state = State.DISABLED
    recording = False
    current_label = ""       # current action label for live segment marking
    q_target: Optional[np.ndarray] = None
    engage_t0: float = 0.0                   # monotonic start of the engage torque-onset ramp
    last_saved: Optional[str] = None
    last_rec = 0.0
    last_term_print = 0.0
    msg_text = ""            # transient status (zero fns, shutdown, ...) for GUI/terminal
    msg_until = 0.0
    period = 1.0 / LOOP_HZ

    def enable():
        # Apply gains → HOLD. Reached only from IDLE (idle-first safety rule): the
        # motors are already enabled/reporting from IDLE, so we've read state and
        # zeroed before any torque authority is applied.
        nonlocal state, q_target
        if not args.dry_run:
            follower.enable()      # idempotent — already enabled in IDLE
        q_target = None            # re-capture current pose for the hold target
        state = State.HOLD

    def disable():
        nonlocal state, q_target, recording
        if recording:
            finalize_recording("(disable)")
        teleop.disengage()
        # follower.disable() clears the re-emit holder BEFORE sending disable, so
        # the 100 Hz re-emitter can't push a stale arm command after it.
        if not args.dry_run:
            follower.disable()
        else:
            follower.clear_target()
        state = State.DISABLED
        q_target = None

    def start_recording():
        nonlocal recording, last_rec, session
        # Fresh buffers per take; the finished session is owned by its in-flight
        # save thread (which also snapshots its refs), so they can never collide.
        session = RecordingSession(CAMERAS)
        if current_label:
            session.set_label(current_label)   # open the first segment at frame 0
        recording = True
        last_rec = 0.0

    def finalize_recording(reason: str):
        nonlocal recording
        recording = False
        n = session.steps
        if n == 0 or args.dry_run:
            print(f"\n[rec] discarded {reason} (steps={n}, dry_run={args.dry_run})")
            return
        session.finalize_segments()   # close the open action segment at the last frame
        t = start_async_save(
            session, out_dir,
            language_instruction=meta.get("language_instruction", ""),
            task_id=meta.get("task_id"), notes=meta.get("notes", ""),
            result_holder=save_result, result_lock=save_lock)
        save_threads.append(t)
        print(f"\n[rec] saving {n} steps {reason} ...")

    def toggle_recording():
        if state != State.TELEOP:
            print("\n[rec] must be in TELEOP to record")
            return
        if recording:
            finalize_recording("(toggle)")
        else:
            start_recording()

    def _announce(text: str):
        nonlocal msg_text, msg_until
        msg_text, msg_until = text, time.monotonic() + 3.0
        print(f"\n{text}")

    def _save_ready_pose(q):
        if q is None:
            _announce("[P] no telemetry — ready pose not saved")
            return
        p = Path(__file__).resolve().parents[2] / "config" / "minerva_ready_pose.json"
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(
            {"joint_names": list(MINERVA_JOINTS), "positions": [float(x) for x in q]}, indent=2))
        _announce(f"[P] ready pose saved -> {p.name}")

    def handle_key(key: str):
        nonlocal state, q_target, engage_t0
        if key == "E":                         # enable gains (HOLD) — ONLY from IDLE
            if state == State.IDLE:
                enable()
            elif state == State.DISABLED:
                _announce("[E] Idle first (I): read actuator state + zero before applying gains")
            # HOLD/TELEOP: already under gains — no-op
        elif key == "I":                       # idle: enable at ZERO torque (backdrive)
            if not args.dry_run:               # the SAFE first step, always allowed
                follower.enable()
            state = State.IDLE
        elif key == "H":
            disable()
        elif key == "T":
            if state in (State.HOLD, State.IDLE):
                q = follower.qpos()               # cold-start guard: need telemetry first
                if q is not None:
                    teleop.engage(q)
                    engage_t0 = time.monotonic()  # start the gentle torque-onset ramp
                    state = State.ENGAGING
                else:
                    _announce("[T] no telemetry yet — Idle (I) first to read arm state")
            elif state in (State.ENGAGING, State.TELEOP):
                if recording:
                    finalize_recording("(disengage)")
                teleop.disengage()
                state = State.HOLD
        elif key == "R":
            toggle_recording()
        elif key == "Z":                       # leader zero (both leaders -> calib)
            _announce(f"[Z] {teleop.leader_zero()}")
        elif key == "M":                       # mirror leader to actual (both)
            _announce(f"[M] {teleop.mirror(follower.qpos())}")
        elif key == "K":                       # RobStride mechanical zero + SaveConfig
            if state != State.DISABLED:
                _announce("[K] disable first (H), then K")
            elif not args.dry_run:
                follower.mech_zero(save=True)
                _announce("[K] mech_zero sent to both arms — saved to flash")
        elif key == "P":                       # save ready pose
            _save_ready_pose(follower.qpos())
        elif key == "X":                       # soft shutdown: ramp to zero, disable
            if state in (State.IDLE, State.HOLD, State.TELEOP):
                if recording:
                    finalize_recording("(shutdown)")
                teleop.disengage()
                q = follower.qpos()
                q_target = q.copy() if q is not None else None
                state = State.SHUTDOWN
                _announce("[X] soft shutdown — ramping to zero")

    print("Controls (IDLE-FIRST): [I]dle=read+zero-torque  [E]nable-gains(only from Idle)  "
          "[T]eleop  [H]disable [R]ec | [Z]leader-zero [M]irror [K]mech-zero [P]ready [X]shutdown [Q]uit")

    try:
        while True:
            t0 = time.monotonic()

            # ---- inputs (GUI key queue; joystick record edges) ----
            key = None
            try:
                key = gui_queue.get_nowait()
            except queue.Empty:
                key = None
            if key == "Q":
                break
            if key:
                handle_key(key)
            if teleop.take_record_edges() > 0 and state == State.TELEOP:
                toggle_recording()

            # ---- action-label changes (live segment marking) ----
            try:
                while True:
                    current_label = label_queue.get_nowait()
                    if recording:
                        session.set_label(current_label)
            except queue.Empty:
                pass

            # ---- telemetry (RESILIENT: partial when one arm's bus drops) ----
            qpos_actual = follower.qpos_partial()   # 17-vec, NaN for a dropped arm; None only if BOTH down
            torque = follower.torques_partial()
            temp_c = follower.temps_partial()        # per-joint temperature (°C)
            qpos_both = follower.qpos()              # strict (both present) — required for recording
            present = follower.arm_ok()              # {left, right} bools
            # live speed (GUI slider) — rebuild gains + torque cap when it changes
            ks = float(gui_params.get("kp_scale", cur_kp_scale))
            if ks != cur_kp_scale:
                cur_kp_scale = ks
                kp_cmd = [float(k) * ks for k in KP]
                lead_cap = lead_cap_vector(kp_cmd, max_delta)
            # Control math must never see NaN; set_target skips any dropped arm, so
            # 0-filling the missing slots here is harmless (they're never commanded).
            q_ctrl = None if qpos_actual is None else np.nan_to_num(qpos_actual, nan=0.0)

            # ---- target ----
            if state == State.ENGAGING and q_ctrl is not None:
                # Gentle onset: ramp the torque cap 0 -> full over ENGAGE_RAMP_S, then promote
                # to live tracking. Time-based, so it always completes in ~0.3 s and never
                # stalls waiting for the follower (the old "nudge the leader to engage" bug).
                leader_tgt = teleop.target(q_ctrl)
                if leader_tgt is None:
                    leader_tgt = q_ctrl
                ramp = min(1.0, (t0 - engage_t0) / ENGAGE_RAMP_S)
                q_target, _ = apply_safety_limits(leader_tgt, q_ctrl, max_delta=lead_cap * ramp)
                if ramp >= 1.0:
                    state = State.TELEOP
                    _announce("[T] engaged")
            elif state == State.TELEOP and q_ctrl is not None:
                tgt = teleop.target(q_ctrl)
                if tgt is not None:
                    tgt, _ = apply_safety_limits(tgt, q_ctrl, max_delta=lead_cap)
                    q_target = tgt
            elif state == State.HOLD and q_target is None and q_ctrl is not None:
                q_target = q_ctrl.copy()
            elif state == State.SHUTDOWN and q_ctrl is not None:
                # ramp the target toward zero; disable once the arms are there.
                ramp = q_target if q_target is not None else q_ctrl.copy()
                ramp = ramp - np.sign(ramp) * np.minimum(np.abs(ramp), 0.02)
                ramp, _ = apply_safety_limits(ramp, q_ctrl, max_delta=max_delta)
                q_target = ramp
                if np.all(np.abs(q_ctrl[:14]) < 0.05):
                    disable()

            # ---- command (set_target skips whichever arm's bus is down) ----
            if not args.dry_run:
                if state in (State.HOLD, State.ENGAGING, State.TELEOP, State.SHUTDOWN) and q_target is not None:
                    follower.set_target(q_target, kp_cmd, KD)
                elif state == State.IDLE and q_ctrl is not None:
                    # zero-torque backdrive: command the measured pose with kp=kd=0
                    follower.set_target(q_ctrl, [0.0] * NUM_MINERVA_JOINTS,
                                        [0.0] * NUM_MINERVA_JOINTS)

            # ---- recording (subsampled; needs BOTH arms — never record half data) ----
            if recording and (t0 - last_rec) >= (1.0 / REC_HZ):
                last_rec = t0
                decoded = _read_decoded(dec_lock, dec_cache, time.time())
                if qpos_both is not None and decoded is not None:
                    frames, cam_ts = decoded
                    session.append(qpos_both, q_target, torque, frames,
                                   telem_ts=time.time(), cam_ts=cam_ts)
                    if session.steps >= args.max_steps:
                        finalize_recording("(max steps)")
                else:
                    session.dropped += 1

            # ---- async-save pickup ----
            with save_lock:
                if "path" in save_result:
                    last_saved = save_result.pop("path")
                    print(f"[rec] saved -> {last_saved} ({save_result.pop('steps', '?')} steps)")
                if "error" in save_result:
                    print(f"[rec] SAVE ERROR: {save_result.pop('error')}")

            # ---- render ----
            now = time.monotonic()
            cam_ages = {}
            for c in CAMERAS:
                with cam_lock:
                    ct = cam_cache.get(f"{c}_time", 0.0)
                cam_ages[c] = (time.time() - ct) if ct else 999.0
            snapshot = {
                "state": state.value, "recording": recording,
                "rec_steps": session.steps, "dropped": session.dropped,
                "qpos": None if qpos_actual is None else qpos_actual.tolist(),
                "target": None if q_target is None else q_target.tolist(),
                "leader": teleop.leader_preview().tolist(),   # leader-mapped pose (NaN where absent)
                "torque": None if torque is None else torque.tolist(),
                "temp": None if temp_c is None else temp_c.tolist(),
                "kp_scale": cur_kp_scale,
                "telem_age": max(follower.telem_age().values()), "cam_ages": cam_ages,
                "arm_ages": follower.telem_age(),
                "present": present,          # {left, right} — which arms are reporting
                "message": (msg_text if now < msg_until else ""),
                "leaders": teleop.status, "last_saved": last_saved,
                "language_instruction": meta.get("language_instruction", ""),
                "current_label": current_label,
                "seg_count": len(session.segments) + (1 if session._seg is not None else 0),
                "robot_ok": qpos_actual is not None,   # at least one arm present
                "both_ok": qpos_both is not None,       # both arms (recording-ready)
            }
            if qt is not None:
                # snapshot holder
                with qt.lock:
                    qt.holder["args"] = snapshot
                # raw JPEG per camera for Qt-side decode
                cam_frames = {}
                for c in CAMERAS:
                    with cam_lock:
                        msg = cam_cache.get(c)
                        ts = cam_cache.get(f"{c}_ts")
                    jb = raw_jpeg(msg)
                    if jb is not None:
                        cam_frames[c] = jb
                        cam_frames[f"{c}_ts"] = ts
                if cam_frames:
                    with qt.cam_lock:
                        qt.cam_holder.update(cam_frames)
                if qt.should_quit():
                    break
            elif now - last_term_print > 0.5:
                last_term_print = now
                led = teleop.status
                qs = "n/a" if qpos_actual is None else " ".join(f"{v:+.2f}" for v in qpos_actual[:6])
                print(f"\r[{state.value:<8}] rec={'Y' if recording else 'n'}({session.steps}) "
                      f"drop={session.dropped} L={'Y' if led['left'] else 'n'} "
                      f"R={'Y' if led['right'] else 'n'} arm0:[{qs}]   ", end="", flush=True)

            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nShutting down...")
        if recording:
            finalize_recording("(shutdown)")
        # Disable both arms (follower.disable clears each re-emit holder before
        # sending disable, so nothing trails it).
        try:
            if not args.dry_run:
                follower.disable()
        except Exception:
            pass
        # Wait for in-flight episode saves to finish writing (gzip can take
        # seconds) so the final episode is never truncated on exit.
        for t in save_threads:
            t.join(timeout=30)
        with save_lock:
            if "path" in save_result:
                print(f"[rec] saved -> {save_result['path']}")
            if "error" in save_result:
                print(f"[rec] SAVE ERROR: {save_result['error']}")
        teleop.close()
        follower.close()          # stops the re-emitter + both telemetry receivers
        for st in (dec_stop, cam_stop):
            st.set()
        # Join the receiver threads so their SUB sockets are CLOSED before
        # ctx.term() — otherwise term() blocks forever waiting on them.
        for th in (dec_thread, cam_thread):
            th.join(timeout=1.5)
        if qt is not None:
            qt.request_quit()
            qt.join(timeout=3.0)
        ctx.term()
        print("Done.")
    # PySide6's QApplication (created on a worker thread) segfaults during
    # interpreter teardown. Every resource is already released above, so exit
    # immediately and skip Python's teardown to guarantee a clean process exit.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
