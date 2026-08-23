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
import threading
import time
import urllib.request
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
    CAMERAS, KD, KP, SAT_TORQUE, MINERVA_JOINTS, NUM_MINERVA_JOINTS, GRIPPER_INDICES,
    apply_safety_limits, lead_cap_vector, max_delta_vector,
    GRIP_FF_GAIN_MA_PER_NM, GRIP_FF_DEADBAND_NM, GRIP_FF_CAP_MA, GRIP_FF_SIGN,
)
from control.minerva_gravity import MinervaGravityModel
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


class HeartbeatPoller:
    """Background poller for the Jetson heartbeat server (/api/status on :8088).

    Surfaces host metrics (CPU / mem / disk / WiFi) and the logic-UPS battery that
    aren't in the motor telemetry. Polled slowly on its own daemon thread with a
    short timeout, so it never blocks the control loop; host() returns the latest
    compact dict (or None when the server is unreachable / stale)."""

    def __init__(self, host: str, port: int = 8088, period: float = 2.0):
        self._url = f"http://{host}:{port}/api/status"
        self._period = period
        self._lock = threading.Lock()
        self._data = None
        self._t = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="HeartbeatPoll")

    def start(self) -> "HeartbeatPoller":
        self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self._url, timeout=1.5) as r:
                    d = json.loads(r.read().decode())
                with self._lock:
                    self._data, self._t = d, time.time()
            except Exception:
                pass   # server down / not deployed — host() reports None
            self._stop.wait(self._period)

    def host(self) -> Optional[dict]:
        with self._lock:
            d, t = self._data, self._t
        if not d or (time.time() - t) > 10.0:
            return None
        h = d.get("host") or {}
        ups = ((d.get("telemetry") or {}).get("ups")) or {}
        u_ok = not ups.get("stale")
        return {
            "cpu": h.get("cpu_percent"),
            "mem": (h.get("mem") or {}).get("percent"),
            "disk": (h.get("disk") or {}).get("percent"),
            "wifi": ((h.get("network") or {}).get("ap") or {}).get("connection"),
            "ups_pct": ups.get("percentage") if u_ok else None,
            "ups_v": ups.get("voltage") if u_ok else None,
        }

    def close(self) -> None:
        self._stop.set()


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
    p.add_argument("--grip-strength", type=float, default=1.5,
                   help="gripper KP multiplier, DECOUPLED from --kp-scale so the gripper grips "
                        "firmly regardless of arm speed; contact force bounded by the motor")
    # gripper force feedback (leader haptics) — off by default; enable + tune in the GUI.
    p.add_argument("--grip-ff", dest="grip_ff", action="store_true",
                   help="start with leader gripper force-feedback ON (default off)")
    p.add_argument("--grip-ff-gain", type=float, default=GRIP_FF_GAIN_MA_PER_NM,
                   help="leader mA per Nm of follower grasp torque")
    # gravity feedforward — off by default; enable + trim the scale live in the GUI.
    p.add_argument("--grav-comp", dest="grav_comp", action="store_true",
                   help="start with arm gravity feedforward ON (default off)")
    p.add_argument("--grav-scale", type=float, default=1.0,
                   help="global multiplier on the gravity feedforward (ramp up while validating)")
    p.add_argument("--grav-file", default="config/minerva_gravity.json",
                   help="gravity model from minerva_gravity_calibrate.py")
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
    heartbeat = HeartbeatPoller(host).start()   # Jetson host metrics + logic-UPS (background)
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
    # Camera endpoints in the config are tcp://localhost:PORT; rewrite the host to
    # the resolved Jetson (same as the arms) so we subscribe to the Jetson's camera
    # publishers, not the laptop's localhost (which is why no camera showed up).
    cam_eps = {name: url.replace("localhost", host).replace("127.0.0.1", host)
               for name, url in mcfg.camera_endpoints(cfg).items()}
    cam_sizes = mcfg.camera_sizes(cfg)
    safe = mcfg.safety(cfg)
    max_delta = max_delta_vector(
        arm=float(safe.get("max_delta_arm", 0.30)),
        gripper=float(safe.get("max_delta_gripper", 0.50)),
        head=float(safe.get("max_delta_head", 0.15)),
        lift=float(safe.get("max_delta_lift", 0.01)))
    # ---- persistent user preferences (Settings dialog) override the CLI defaults ----
    # A pref is None until the operator sets it in Settings, so CLI flags still work until
    # then. Arm gains are 6-vectors (j1..j6, applied to both arms) or None = constants.
    _settings = CollectorSettings()

    def _pref(key, fallback):
        v = _settings.get(key)
        return fallback if v is None else v

    def _splat_arm(arm6, base17):
        """Put a 6-vec (j1..j6) onto both arms of a 17-vec; keep gripper/head/lift."""
        out = [float(x) for x in base17]
        for lo in (0, 7):                       # left arm 0..5, right arm 7..12
            for k in range(6):
                out[lo + k] = float(arm6[k])
        return out

    # Live per-joint base gains (mutable — the loop re-splats these when the Settings
    # dialog bumps gui_params["gains_rev"]).
    arm_kp6 = list(_pref("arm_kp", [float(x) for x in KP[0:6]]))
    arm_kd6 = list(_pref("arm_kd", [float(x) for x in KD[0:6]]))
    arm_sat6 = list(_pref("arm_sat", [float(x) for x in SAT_TORQUE[0:6]]))
    base_kp = _splat_arm(arm_kp6, KP)
    base_kd = _splat_arm(arm_kd6, KD)
    base_sat = np.array(_splat_arm(arm_sat6, SAT_TORQUE), dtype=np.float32)

    # Arm gains scale with kp_scale (the Speed slider); the GRIPPER gets its own
    # strength (grip_strength), decoupled from arm speed so it can grip firmly.
    cur_grip_strength = float(_pref("grip_strength", args.grip_strength))
    cur_kp_scale = float(_pref("kp_scale", args.kp_scale))

    def _build_gains(ks: float, gs: float):
        """Return (kp_cmd[17], lead_cap[17]) from the LIVE base gains. Arms: kp=base_kp*ks,
        lead capped to base_sat/kp (torque-bounded, so a fast move can't spike torque).
        GRIPPER: kp=base_kp*gs, lead = FULL max_delta so it snaps to the trigger (contact
        force bounded by the motor's own torque saturation)."""
        kc = [float(k) * ks for k in base_kp]
        for gi in GRIPPER_INDICES:
            kc[gi] = float(base_kp[gi]) * gs
        lc = lead_cap_vector(kc, max_delta, sat=base_sat)
        for gi in GRIPPER_INDICES:
            lc[gi] = max_delta[gi]     # gripper tracks the trigger fast; motor caps the force
        return kc, lc

    kp_cmd, lead_cap = _build_gains(cur_kp_scale, cur_grip_strength)
    _ff_invert = _pref("grip_ff_invert", None)
    _ff_sign = (1 if _ff_invert else -1) if _ff_invert is not None else int(GRIP_FF_SIGN)
    # Live params: GUI sliders/dialog write here; the loop reads them each tick and
    # rebuilds gains when kp_scale / grip_strength / gains_rev change.
    gui_params = {
        "kp_scale": cur_kp_scale,
        "grip_strength": cur_grip_strength,           # gripper KP mult (GUI slider, live)
        # Gripper force-feedback (leader haptics): GUI toggles/invert/gain write here.
        "grip_ff": bool(_pref("grip_ff", args.grip_ff)),
        "grip_ff_gain": float(_pref("grip_ff_gain", args.grip_ff_gain)),
        "grip_ff_sign": _ff_sign,
        # Gravity feedforward (arm droop cancellation): GUI toggles grav_comp + trims
        # grav_scale live; grav_ok tells the GUI whether a calibration file was found.
        "grav_comp": bool(_pref("grav_comp", args.grav_comp)),
        "grav_scale": float(_pref("grav_scale", args.grav_scale)),
        "grav_ok": False,
        # Live arm tuning: the Settings dialog writes these 6-vecs then bumps gains_rev.
        "arm_kp": list(arm_kp6), "arm_kd": list(arm_kd6), "arm_sat": list(arm_sat6),
        "gains_rev": 0,
    }
    _cur_gains_rev = 0

    # Gravity feedforward model (from minerva_gravity_calibrate.py). Optional — if the
    # file is missing the collector runs exactly as before (grav_comp stays inert).
    grav_model = None
    try:
        _gpath = _repo_config(Path(args.grav_file).name) if not Path(args.grav_file).is_absolute() \
            else Path(args.grav_file)
        if _gpath and Path(_gpath).exists():
            grav_model = MinervaGravityModel.from_json(_gpath)
            gui_params["grav_ok"] = True
            print(f"[grav] loaded {len(grav_model.fits)} joint fits from {_gpath} "
                  f"(feedforward {'ON' if args.grav_comp else 'off'}, scale={args.grav_scale})")
        else:
            print(f"[grav] no gravity model at {args.grav_file} — run "
                  f"minerva_gravity_calibrate.py; feedforward disabled")
    except Exception as _exc:
        print(f"[grav] could not load gravity model ({_exc}); feedforward disabled")
    # Follower joint limits: override the seeded placeholder JOINT_LIMITS with the REAL
    # measured travel from minerva_calibrate.py (if present) so the diff bars and the
    # safety clamp use true ranges. Mutates the shared array before the GUI/loop read it.
    _nf = _apply_follower_calibration(_repo_config("minerva_calibration.json"))
    if _nf:
        print(f"[calib] follower limits: {_nf} joints from minerva_calibration.json")
    out_dir = args.output_dir
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    meta = {"language_instruction": args.instruction, "notes": args.notes,
            "task_id": args.task_id,
            # Camera stream endpoints (host-rewritten) + configured sizes, so the GUI
            # camera tiles can show the source port and target resolution.
            "cam_endpoints": cam_eps, "cam_sizes": cam_sizes}

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

    # Optional loop profiler (set AIZEE_PROFILE=1). Every ~2 s it prints where each
    # 30 Hz tick's wall time went — telemetry read, target (incl. leader poll read),
    # command send, snapshot+camera push — plus the actual loop rate and worst-case
    # period. Lets us see whether a stutter is the loop blocking (and where) vs. the
    # leader/telemetry arriving late. Zero cost when the env var is unset.
    _profile = bool(os.environ.get("AIZEE_PROFILE"))
    _prof = {"period": [], "tel": [], "tgt": [], "cmd": [], "snap": []}
    _prof_last = time.monotonic()
    _prev_t0 = None

    try:
        while True:
            t0 = time.monotonic()
            if _profile:
                if _prev_t0 is not None:
                    _prof["period"].append(t0 - _prev_t0)
                _prev_t0 = t0
                _tstamp = [t0]   # section boundary timestamps

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
            # live speed + grip strength (GUI sliders) — rebuild gains + torque cap on change
            ks = float(gui_params.get("kp_scale", cur_kp_scale))
            gs = float(gui_params.get("grip_strength", cur_grip_strength))
            rev = int(gui_params.get("gains_rev", _cur_gains_rev))
            if rev != _cur_gains_rev:
                # Settings dialog edited per-joint kp/kd/SAT — re-splat the base gains
                # (both arms) and rebuild the commanded gains + torque-bounded lead.
                _cur_gains_rev = rev
                base_kp = _splat_arm(gui_params.get("arm_kp", arm_kp6), KP)
                base_kd = _splat_arm(gui_params.get("arm_kd", arm_kd6), KD)
                base_sat = np.array(_splat_arm(gui_params.get("arm_sat", arm_sat6), SAT_TORQUE),
                                    dtype=np.float32)
                cur_kp_scale, cur_grip_strength = ks, gs
                kp_cmd, lead_cap = _build_gains(ks, gs)
            elif ks != cur_kp_scale or gs != cur_grip_strength:
                cur_kp_scale = ks
                cur_grip_strength = gs
                kp_cmd, lead_cap = _build_gains(ks, gs)
            # Control math must never see NaN; set_target skips any dropped arm, so
            # 0-filling the missing slots here is harmless (they're never commanded).
            q_ctrl = None if qpos_actual is None else np.nan_to_num(qpos_actual, nan=0.0)
            if _profile: _tstamp.append(time.monotonic())   # after telemetry read

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

            if _profile: _tstamp.append(time.monotonic())   # after target (incl. leader read)

            # ---- gravity feedforward (arm droop cancellation) ----
            # tau_ff from the identified per-joint gravity model, evaluated at the
            # MEASURED pose (the model zeros NaN/uncalibrated joints; set_target zeros
            # any dropped arm). Only when a model is loaded AND the GUI toggle is on;
            # grav_scale trims it live (ramp 0->1 while validating). None => plain PD.
            tau_ff = None
            grav_ma = 0.0
            if (grav_model is not None and gui_params.get("grav_comp")
                    and qpos_actual is not None
                    and state in (State.HOLD, State.ENGAGING, State.TELEOP, State.SHUTDOWN)):
                tau_ff = grav_model.gravity_torques(
                    qpos_actual, scale=float(gui_params.get("grav_scale", 1.0)))
                grav_ma = float(np.nanmax(np.abs(tau_ff))) if tau_ff is not None else 0.0

            # ---- command (set_target skips whichever arm's bus is down) ----
            if not args.dry_run:
                if state in (State.HOLD, State.ENGAGING, State.TELEOP, State.SHUTDOWN) and q_target is not None:
                    follower.set_target(q_target, kp_cmd, base_kd, tau17=tau_ff)
                elif state == State.IDLE and q_ctrl is not None:
                    # zero-torque backdrive: command the measured pose with kp=kd=0
                    follower.set_target(q_ctrl, [0.0] * NUM_MINERVA_JOINTS,
                                        [0.0] * NUM_MINERVA_JOINTS)

            # ---- leader gripper force feedback (haptics) ----
            # ONLY while actively teleoperating: render each follower gripper's
            # grasp torque back onto the leader gripper it drives, so the operator
            # feels the squeeze. Every other state (and the GUI toggle off) releases
            # the leader grippers to free backdrive. The OpenRB firmware watchdog is
            # the backstop that drops leader torque if this loop ever stalls.
            grip_ff_ma = {"left": 0, "right": 0}
            if (not args.dry_run and state == State.TELEOP
                    and gui_params.get("grip_ff") and torque is not None):
                grip_ff_ma = teleop.apply_gripper_ff(
                    torque,
                    gain=float(gui_params.get("grip_ff_gain", GRIP_FF_GAIN_MA_PER_NM)),
                    deadband=GRIP_FF_DEADBAND_NM,
                    cap=GRIP_FF_CAP_MA,
                    sign=int(gui_params.get("grip_ff_sign", GRIP_FF_SIGN)))
            else:
                teleop.release_gripper_ff()
            if _profile: _tstamp.append(time.monotonic())   # after command + FF

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
                "grip_ff_ma": grip_ff_ma,   # {left,right} applied leader current (mA)
                "grav_on": bool(grav_model is not None and gui_params.get("grav_comp")),
                "grav_peak": grav_ma,        # peak |gravity FF| applied this tick (Nm)
                "battery": follower.battery_voltage(),   # motor-pack V (min of arms)
                "estop": follower.estop(),               # True/False/None
                "jetson": host,                          # resolved Jetson address
                "host": heartbeat.host(),                # Jetson CPU/mem/disk/wifi + UPS (or None)
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

            if _profile and len(_tstamp) >= 4:
                _te = time.monotonic()
                _prof["tel"].append(_tstamp[1] - _tstamp[0])
                _prof["tgt"].append(_tstamp[2] - _tstamp[1])
                _prof["cmd"].append(_tstamp[3] - _tstamp[2])
                _prof["snap"].append(_te - _tstamp[3])
                if _te - _prof_last >= 2.0:
                    _prof_last = _te
                    def _mm(x):   # (mean_ms, max_ms)
                        return (sum(x) / len(x) * 1e3, max(x) * 1e3) if x else (0.0, 0.0)
                    pm = _prof["period"]
                    hz = (len(pm) / sum(pm)) if pm and sum(pm) > 0 else 0.0
                    pmax = max(pm) * 1e3 if pm else 0.0
                    tel, tgt, cmd, snap = (_mm(_prof[k]) for k in ("tel", "tgt", "cmd", "snap"))
                    print(f"\n[perf] {hz:4.1f}Hz  period_max={pmax:6.1f}ms | "
                          f"tel {tel[0]:4.1f}/{tel[1]:5.1f}  tgt {tgt[0]:4.1f}/{tgt[1]:6.1f}  "
                          f"cmd {cmd[0]:4.1f}/{cmd[1]:5.1f}  snap {snap[0]:4.1f}/{snap[1]:6.1f}  "
                          f"(mean/max ms)  telem_age={max(follower.telem_age().values())*1e3:.0f}ms", flush=True)
                    for _k in _prof:
                        _prof[_k].clear()

            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nShutting down...")
        heartbeat.close()
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
