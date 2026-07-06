#!/usr/bin/env python3
"""collect_demo.py — Motor control + ACT demo recorder for AIZEE arm.

Combines SO-101 teleoperation with demonstration recording.  Optionally
drives the arm via the SO-101 leader arm (--port).  Without --port you
still get full motor control for setup.

Usage:
    python collect_demo.py --port COM4
    python collect_demo.py --port /dev/ttyACM0 \\
        --cmd     tcp://192.168.0.27:5555 \\
        --telem   tcp://192.168.0.27:5556 \\
        --gripper-cam tcp://192.168.0.27:5563

Controls:
    E    enable arm motors (align to leader if --port given)
    I    idle — enable with zero torque (see actual positions)
    H    hold — freeze target at current actual position
    R    toggle recording (TRACKING only)
    X    soft shutdown — hold 1 s, return to zero, disable
    Z    zero — capture current SO-101 pose as zero reference
    M    mirror — set zero so current leader maps to current actual
    P    save current arm position as ready pose (config/ready_pose.json)
    K    mechanical zero — write Robstride hardware zero + SaveConfig to all
         arm joints (motors must be disabled; persists across power cycle)
    Q    quit  (Ctrl-C also works)
    WASD drive wheels (W=fwd S=back A=left D=right; wheels enable with arm)

Gamepad: A=enable  B=shutdown/cancel  Start=hold  Back=quit
         Left stick = drive (wheels enable with arm)

M5 Joystick2 (wired to OpenRB-150 leader, I2C 0x63 on D11/D12):
         Stick = drive (overrides WASD / xbox stick when deflected)
         Button = start/stop recording (no-op unless TRACKING or already recording)
"""

from __future__ import annotations

import argparse
import enum
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import zmq

try:
    import pygame
    _pygame_available = True
except ImportError:
    _pygame_available = False

try:
    import rerun as rr
    import rerun.blueprint as rrb
    _rerun_available = True
except ImportError:
    _rerun_available = False

try:
    import serial as _serial
    _pyserial_available = True
except ImportError:
    _pyserial_available = False

_so101_available = False
try:
    sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
    from so101_leader import (
        So101Leader, CALIB_PATH as _CALIB_PATH, find_so101_port, _probe_so101,
    )
    _so101_available = True
except ImportError:
    _CALIB_PATH = Path("so101_calibration.json")

# OpenRB-150 + Dynamixel XL330 leader (newer build). Same duck-typed interface
# as So101Leader, so the runtime code below is leader-kind agnostic once
# instantiated.
_leader_module_available = False
try:
    from leader import (
        find_any_leader, get_leader_class, default_calib_path,
        identify_port, LEADER_KINDS,
    )
    _leader_module_available = True
    # FF protocol constants (OpenRB only).  Imported once at top so the
    # FF-send block in the main loop stays cheap; getattr fallback keeps
    # this safe if an older openrb_leader.py is on the path.
    try:
        from openrb_leader import FF_MAX_CURRENT_MA, FF_DISABLE_SENTINEL
    except Exception:
        FF_MAX_CURRENT_MA   = 200
        FF_DISABLE_SENTINEL = -32768
except ImportError:
    LEADER_KINDS = ("so101",)
    FF_MAX_CURRENT_MA   = 200
    FF_DISABLE_SENTINEL = -32768

# Quest / WebXR leader (optional — kicks in only when --leader quest).
# Pulled in lazily because it imports IK + aiohttp + cryptography which the
# user may not have installed if they're sticking with the physical leader.
# Imported as a top-level module via the `python/teleop/` path entry that
# line 84 already added (same pattern as so101_leader / leader), not as a
# subpackage of `teleop` — `teleop/teleop.py` would shadow the namespace.
_quest_available = False
try:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from quest_leader import QuestLeader, QuestLeaderConfig, make_quest_leader_class
    from web import SharedState, start_server_in_thread
    _quest_available = True
except ImportError as _quest_imp_err:
    _quest_imp_err_msg = str(_quest_imp_err)

sys.path.insert(0, str(Path(__file__).parent))
from common.arm_constants import (
    ARM_JOINTS, POLICY_JOINTS, KP, KD,
    setup_keyboard, load_arm_limits, clamp_arm_positions,
)

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from collect_demo_app import alignment
from collect_demo_app.alignment import (
    _SAT_TORQUE, _load_joint_align, _maybe_reload_joint_align,
    _push_visual_offsets_to_leader,
)
from collect_demo_app.config import (
    _load_endpoints, _load_teleop_yaml, _resolve_rover_host,
    _split_tcp_endpoint,
)
from collect_demo_app.display import (
    _BG_RED, _CAM_STALE, _RST, _ansi_on, _start_display_thread,
    _start_rerun_thread,
)
from collect_demo_app.gamepad import (
    _apply_curve, _apply_deadzone, _init_joystick, _ramp_toward,
    _read_gamepad,
)
from collect_demo_app.images import _start_image_decoder
from collect_demo_app.profiler import _LoopProfiler
from collect_demo_app.receivers import (
    _start_cam_receiver, _start_estop_reader, _start_telem_receiver,
)
from collect_demo_app.recording import save_episode
from collect_demo_app.replay import _LiveReplay
from collect_demo_app.runtime import (
    LOOP_HZ, NUM_JOINTS, REC_HZ, _ALL_MOTORS, _BASE_MOTORS,
)
from collect_demo_app.telem import _qpos, _qpos_motor, _qtemp, _qtorque
from collect_demo_app.zmq_io import _build_bundle, _send, _start_cmd_sender


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    _load_joint_align()  # populate _ALIGN_OFFSETS / _ALIGN_SIGNS from joint_align.json
    _ep = _load_endpoints()
    ap  = argparse.ArgumentParser(
        description="SO-101 leader arm teleop + ACT demo recorder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",            default=None,
                    help="Leader-arm serial port (optional — enables leader tracking)")
    ap.add_argument("--baud",            type=int, default=1_000_000)
    ap.add_argument("--calib",           default=None,
                    help="Leader calibration JSON (defaults to per-leader-kind path)")
    ap.add_argument("--leader",          default="auto",
                    choices=("auto", *LEADER_KINDS),
                    help="Which physical leader arm to use at startup (auto = try "
                         "SO-101 then OpenRB-150).  Quest VR is added/removed at "
                         "runtime via the GUI — not selectable here.")
    # Quest server configuration knobs.  They take effect when the GUI
    # toggles Quest on; ignored otherwise.
    ap.add_argument("--quest-bind",      default="0.0.0.0", dest="quest_bind",
                    help="Bind address for the WebXR server when activated from the GUI")
    ap.add_argument("--quest-port",      type=int, default=8443, dest="quest_port",
                    help="HTTPS port for the WebXR server when activated from the GUI")
    ap.add_argument("--quest-config",    default=None, dest="quest_config",
                    help="Path to quest_teleop.yaml (defaults to config/quest_teleop.yaml)")
    ap.add_argument("--cmd",             default=_ep.get("command",       "tcp://192.168.0.27:5555"))
    ap.add_argument("--telem",           default=_ep.get("telemetry",     "tcp://192.168.0.27:5556"))
    ap.add_argument("--gripper-cam",     default="tcp://192.168.0.27:5563", dest="gripper_cam",
                    help="Gripper camera ZMQ endpoint (single ELP UVC stream)")
    ap.add_argument("--gripper-cam-ctrl", default="tcp://192.168.0.27:5573", dest="gripper_cam_ctrl",
                    help="Gripper camera control ZMQ REP endpoint (V4L2 sliders in GUI). "
                         "Empty string disables the camera-controls panel.")
    ap.add_argument("--scene-cam",       default="tcp://192.168.0.27:5564", dest="scene_cam",
                    help="Scene camera ZMQ endpoint (Intel RealSense RGB-D). "
                         "Empty string disables scene-cam subscribe / record / preview.")
    ap.add_argument("--ups",             default=_ep.get("ups_telemetry", "tcp://192.168.0.27:5562"),
                    help="UPS telemetry address (empty to disable)")
    ap.add_argument("--output-dir",      default="episodes",              dest="output_dir")
    ap.add_argument("--max-steps",       type=int, default=10000,           dest="max_steps",
                    help="Max steps per episode (default: 10000 = 30 s at 20 Hz)")
    ap.add_argument("--image-size",      default="768x1024",              dest="image_size",
                    help="Image size HxW (default: 768x1024 — matches gripper-cam capture)")
    ap.add_argument("--dry-run",         action="store_true",             dest="dry_run")
    ap.add_argument("--max-delta",       type=float, default=0.3,         dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.3)")
    ap.add_argument("--robstride-calib", default=None,                    dest="robstride_calib")
    ap.add_argument("--no-rerun",       action="store_true",             dest="no_rerun",
                    help="Disable Rerun live camera preview")
    ap.add_argument("--gui",            action="store_true",             dest="gui",
                    help="Launch PySide6 control panel (embeds Rerun web viewer)")
    ap.add_argument("--estop-port",    default=None,                    dest="estop_port",
                    help="Serial port for ESP32 e-stop receiver (e.g. /dev/estop-receiver, COM10)")
    ap.add_argument("--task-tag",      default="",                      dest="task_tag",
                    help="Task label written as episode attr (GUI can override live)")
    # Force-feedback on the OpenRB GELLO leader (only).  Off by default —
    # requires the dedicated 5V supply on the OpenRB rail; running on USB
    # power alone will brown out.  Currently scoped to the gripper trigger
    # so the operator feels follower-side gripper load (object squeeze /
    # back-drive from a surface).  Other joints stay passive (torque off).
    ap.add_argument("--ff-leader",     action="store_true",            dest="ff_leader",
                    help="Enable leader force feedback (OpenRB only). "
                         "Reflects follower gripper torque onto the trigger.")
    ap.add_argument("--ff-gripper-gain", type=float, default=350.0,    dest="ff_gripper_gain",
                    help="mA per N·m of follower gripper torque (signed). "
                         "Sign flips which way the trigger pushes; tune empirically "
                         "(default 350 → ~70 mA at typical 0.2 N·m grip load).")
    ap.add_argument("--ff-gripper-deadband", type=float, default=0.05, dest="ff_gripper_deadband",
                    help="N·m of follower gripper torque to ignore before "
                         "applying FF (cancels idle holding torque).")
    args = ap.parse_args()

    _ansi_on()

    # Resolve which network the rover is actually reachable on and repoint every
    # rover ZMQ endpoint accordingly: configured IP → USB-C ethernet → WiFi AP.
    _primary_host, _cmd_port = _split_tcp_endpoint(args.cmd)
    if _primary_host:
        _sel_host, _cands = _resolve_rover_host(_primary_host, _cmd_port)
        _tried = " → ".join(_cands)
        if _sel_host != _primary_host:
            print(f"[net] {_primary_host} unreachable; rover found at {_sel_host} "
                  f"(tried {_tried})", flush=True)
            for _name in ("cmd", "telem", "gripper_cam", "gripper_cam_ctrl",
                          "scene_cam", "ups"):
                _h, _p = _split_tcp_endpoint(getattr(args, _name, None))
                if _h == _primary_host and _p:
                    setattr(args, _name, f"tcp://{_sel_host}:{_p}")
        else:
            print(f"[net] rover reachable at {_sel_host} (priority: {_tried})",
                  flush=True)

    h_s, w_s = args.image_size.split("x")
    img_size  = (int(w_s), int(h_s))   # PIL: (width, height)

    # -------------------------------------------------------------------------
    # Leader arm (optional, hot-pluggable)
    #
    # Two leader kinds are supported, both exposing the same duck-typed
    # interface (poll/connect/JOINTS/AIZEE_JOINTS/zero_offsets/directions):
    #   - so101  : Feetech STS3215 over WaveShare USB-serial bus adapter.
    #   - openrb : Dynamixel XL330 servos behind an OpenRB-150 USB-CDC bridge.
    #
    # The leader is allowed to be absent at startup AND to appear later.  A
    # background watcher polls comports() at low frequency and only probes
    # ports when the set actually changes — no spammy probe loop.
    # -------------------------------------------------------------------------
    leader           = None
    _lr_lock         = threading.Lock()
    _lr_latest: dict = {
        "rad": None, "vel": None, "clamped": None, "time": 0.0,
        # M5 Joystick2 snapshot (populated by _leader_reader when an
        # OpenRB-150 leader is installed; left at neutral defaults
        # otherwise).  See OpenRBLeader.last_joystick for field semantics.
        "joy": {
            "x":             0.0,
            "y":             0.0,
            "button":        False,
            "press_counter": 0,
            "status":        1,    # JOY_STATUS_NOT_PRESENT
            "present":       False,
        },
    }
    # Shared leader-to-AIZEE mapping params.  Written by main loop (Z, M
    # keys; leader install / hot-plug); read by the cmd-sender thread when
    # it computes q_cmd live from leader at 100 Hz.  Held under _lr_lock.
    _lr_mapping: dict = {
        "zero_offsets": None,   # np.ndarray, leader-frame
        "directions":   None,   # np.ndarray, ±1 per leader joint
        "for_aizee":    None,   # list[int], leader-index → AIZEE-arm-index
        "emits_urdf":   False,  # True for QuestLeader; False for physical
                                # leaders whose `directions` was tuned to
                                # the motor-encoder direction pre-joint_align
    }
    _lr_stop         = threading.Event()
    zero_offsets     = None
    directions       = None
    _so101_for_aizee: list[int] = []
    _emits_urdf      = False    # set per-leader in _try_install_leader
    _arm_joint_set   = set(ARM_JOINTS)

    # Selected leader kind ("so101" / "openrb") and its class — set at install
    # time, used by the hot-plug watcher so a re-plug picks the same kind.
    _leader_kind:  Optional[str] = None if args.leader == "auto" else args.leader
    _leader_cls                  = None
    _leader_calib                = args.calib

    # Quest WebXR runtime state.  Lazily built when the GUI toggles Quest
    # on for the first time; persists for the rest of the session so the
    # /preview page stays accessible even after the operator disconnects
    # the VR leader.  Building costs ~1.5 s (cert + URDF + server) which
    # is why we don't do it at boot.
    _quest_state = None
    _quest_cfg   = None
    _quest_server_started = False
    if args.leader != "auto" and _leader_module_available:
        _leader_cls   = get_leader_class(args.leader)
        if _leader_calib is None:
            _leader_calib = str(default_calib_path(args.leader))
    # Back-compat fallback: if the leader module isn't importable for any
    # reason, fall through to the original SO-101-only code path.
    if _leader_cls is None and _so101_available:
        _leader_cls   = So101Leader
        _leader_kind  = _leader_kind or "so101"
        if _leader_calib is None:
            _leader_calib = str(_CALIB_PATH)

    # Atomic single-slot box read by the always-on reader thread.  Updating
    # the dict key is a single bytecode op, so the reader sees None or a
    # complete leader object — never a half-installed one.
    _leader_box: dict = {"leader": None}

    # Hot-plug install hand-off: watcher writes a dict here, main loop pops
    # it at the top of the loop and rebinds `leader`/`zero_offsets`/etc.
    _install_lock = threading.Lock()
    _install_pending: dict = {}

    def _try_install_leader(port: str, kind: Optional[str] = None) -> bool:
        """Connect to *port* and install as the active leader. Returns True on success.

        *kind* overrides the previously-selected leader kind (useful for the
        hot-plug watcher when --leader=auto).  When None, falls back to the
        currently-bound _leader_cls / _leader_kind / _leader_calib.
        """
        nonlocal _leader_cls, _leader_kind, _leader_calib
        if kind is not None and _leader_module_available:
            _leader_cls   = get_leader_class(kind)
            _leader_kind  = kind
            if args.calib is None:
                _leader_calib = str(default_calib_path(kind))
        if _leader_cls is None:
            print(f"No leader class available — cannot install {port}", flush=True)
            return False
        calib_path = _leader_calib if _leader_calib is not None else str(_CALIB_PATH)
        kind_name  = _leader_kind or "leader"
        try:
            ldr = _leader_cls(port, args.baud, calib=calib_path)
        except Exception as exc:
            print(f"{kind_name} init failed on {port}: {exc}", flush=True)
            return False
        try:
            ok = ldr.connect()
        except Exception as exc:
            print(f"{kind_name} connect raised on {port}: {exc}", flush=True)
            return False
        if not ok:
            return False
        for_aizee = [i for i, j in enumerate(ldr.AIZEE_JOINTS) if j in _arm_joint_set]
        # Caller decides whether to write to local rebinds or hand off to main loop.
        with _install_lock:
            _install_pending["data"] = {
                "leader":       ldr,
                "zero_offsets": ldr.zero_offsets,
                "directions":   ldr.directions,
                "for_aizee":    for_aizee,
                # See _apply_leader_to_urdf_frame in _compute_tracking_bundle
                # / the main loop for what this gates.  Physical leaders
                # default to False (their `directions` calibration was tuned
                # to match motor encoder direction, predating joint_align).
                "emits_urdf":   bool(getattr(ldr, "EMITS_URDF_FRAME", False)),
            }
        # Hand the leader the current visual-offset vector so its FK/IK
        # operate in mesh frame (QuestLeader only — others are no-ops).
        _push_visual_offsets_to_leader(ldr)
        _leader_box["leader"] = ldr
        return True

    # ------------------------------------------------------------------
    # Quest WebXR — runtime install/uninstall (driven by the GUI button).
    # The boot path no longer touches these; they're called by the dict
    # command handler when the operator clicks "Connect Quest VR".
    # ------------------------------------------------------------------

    def _ensure_quest_runtime() -> bool:
        """Lazily build SharedState + start the WebXR server.  Idempotent —
        safe to call multiple times; subsequent calls are no-ops."""
        nonlocal _quest_state, _quest_cfg, _quest_server_started
        if not _quest_available:
            print("[quest] support unavailable — install: "
                  "pip install aiohttp cryptography pyyaml", flush=True)
            return False
        if _quest_state is None:
            _quest_state = SharedState()
            _cfg_path = args.quest_config or str(
                Path(__file__).resolve().parents[2] / "config" / "quest_teleop.yaml"
            )
            if Path(_cfg_path).exists():
                _quest_cfg = QuestLeaderConfig.load_yaml(_cfg_path)
                print(f"[quest] loaded config from {_cfg_path}", flush=True)
            else:
                _quest_cfg = QuestLeaderConfig()
                print(f"[quest] no config at {_cfg_path} — using defaults", flush=True)
        if not _quest_server_started:
            start_server_in_thread(
                _quest_state, bind=args.quest_bind, port=args.quest_port,
            )
            _quest_server_started = True
        return True

    def _install_quest_leader() -> bool:
        """Install QuestLeader as the active leader.  Replaces any existing
        serial leader for the duration of the Quest session."""
        nonlocal _leader_cls, _leader_kind, _leader_calib
        if not _ensure_quest_runtime():
            return False
        _leader_cls   = make_quest_leader_class(_quest_state, _quest_cfg)
        _leader_kind  = "quest"
        _leader_calib = None
        return _try_install_leader("webxr://")

    def _uninstall_leader() -> None:
        """Drop the currently-installed leader (no auto-recovery).  The
        WebXR server stays running so /preview is still reachable; only
        the leader-poll path is detached."""
        nonlocal _leader_cls, _leader_kind
        _leader_box["leader"] = None
        with _lr_lock:
            _lr_latest["rad"]     = None
            _lr_latest["vel"]     = None
            _lr_latest["clamped"] = None
            _lr_mapping["zero_offsets"] = None
            _lr_mapping["directions"]   = None
            _lr_mapping["for_aizee"]    = None
            _lr_mapping["emits_urdf"]   = False
        _leader_kind = None

    def _leader_reader(stop: threading.Event) -> None:
        """Always-on reader thread; idles until a leader is installed in _leader_box."""
        prev_r: Optional[np.ndarray] = None
        prev_t: float = 0.0
        ema_v:  Optional[np.ndarray] = None
        # EMA constant for the velocity estimate. Differentiating quantized
        # 12-bit encoders at ~500 Hz produces ~13 mrad/s of LSB noise, so we
        # smooth before forwarding. alpha tuned for ~3 sample time-constant.
        _V_ALPHA = 0.4
        while not stop.is_set():
            ldr = _leader_box["leader"]
            if ldr is None:
                prev_r = None
                ema_v  = None
                time.sleep(0.02)
                continue
            try:
                r = ldr.poll()
            except Exception:
                r = None
            now = time.time()
            v: Optional[np.ndarray] = None
            if r is not None and prev_r is not None and (now - prev_t) > 1e-3:
                inst_v = (r - prev_r) / (now - prev_t)
                ema_v  = inst_v if ema_v is None else (
                    _V_ALPHA * inst_v + (1.0 - _V_ALPHA) * ema_v)
                v = ema_v
            # M5 Joystick2 snapshot — only OpenRBLeader exposes this; older
            # SO-101 leader has no `last_joystick`, so we degrade silently.
            joy_snapshot = getattr(ldr, "last_joystick", None)
            with _lr_lock:
                if r is not None:
                    _lr_latest["rad"]     = r
                    _lr_latest["vel"]     = v
                    _lr_latest["clamped"] = ldr.clamped_joints
                    _lr_latest["time"]    = now
                if joy_snapshot is not None:
                    _lr_latest["joy"] = joy_snapshot
            if r is not None:
                prev_r = r
                prev_t = now

    _lr_thread = threading.Thread(target=_leader_reader, args=(_lr_stop,), daemon=True)
    _lr_thread.start()

    leader_port = args.port
    if leader_port is None and _leader_module_available:
        _excl = [args.estop_port] if args.estop_port else []
        if args.leader == "auto":
            print("Searching for any leader arm...", flush=True)
            leader_port, detected_kind = find_any_leader(exclude=_excl, verbose=True)
        else:
            print(f"Searching for {args.leader} leader arm...", flush=True)
            leader_port, detected_kind = find_any_leader(
                exclude=_excl, verbose=True, prefer=args.leader,
            )
            if detected_kind != args.leader:
                # Honour the explicit --leader choice — don't silently use a different kind.
                leader_port, detected_kind = None, None
        if leader_port:
            print(f"{detected_kind} auto-detected on {leader_port}")
            _leader_kind = detected_kind
            _leader_cls  = get_leader_class(detected_kind)
            if args.calib is None:
                _leader_calib = str(default_calib_path(detected_kind))
        else:
            print("Leader not detected — continuing without leader tracking "
                  "(plug it in any time, or pass --port to force)")
    elif leader_port is None and _so101_available:
        # Fallback path when leader.py is missing — original SO-101-only code.
        _excl = [args.estop_port] if args.estop_port else []
        print("Searching for SO-101 leader arm...", flush=True)
        leader_port = find_so101_port(exclude=_excl, verbose=True)
        if leader_port:
            print(f"SO-101 auto-detected on {leader_port}")

    if leader_port is not None and _leader_cls is None:
        print("Leader-arm support not available (missing leader modules)")
        leader_port = None

    if leader_port is not None:
        if _try_install_leader(leader_port):
            print(f"{_leader_kind or 'leader'} connected on {leader_port}")
            # Drain the pending hand-off into local bindings immediately
            # (main loop hasn't started yet).
            with _install_lock:
                _p = _install_pending.pop("data", None)
            if _p is not None:
                leader           = _p["leader"]
                zero_offsets     = _p["zero_offsets"]
                directions       = _p["directions"]
                _so101_for_aizee = _p["for_aizee"]
                _emits_urdf      = _p.get("emits_urdf", False)
                # Publish mapping to the cmd-thread shared state so live
                # TRACKING-mode q_cmd computation can see it.
                with _lr_lock:
                    _lr_mapping["zero_offsets"] = zero_offsets
                    _lr_mapping["directions"]   = directions
                    _lr_mapping["for_aizee"]    = _so101_for_aizee
                    _lr_mapping["emits_urdf"]   = _emits_urdf
        else:
            print(f"{_leader_kind or 'leader'} connect failed on {leader_port} — "
                  "continuing; will retry when port reappears")

    # Background hot-plug watcher.  Runs whenever a leader is not currently
    # installed; only probes when the port set changes (cheap enumeration is
    # the trigger; expensive sync-read is gated).
    _hp_stop = threading.Event()

    def _leader_hotplug_watcher() -> None:
        if not _pyserial_available:
            return
        try:
            from serial.tools import list_ports
        except ImportError:
            return
        excl = {args.estop_port} if args.estop_port else set()
        # Which kinds the watcher will accept on hot-plug.
        if args.leader == "auto":
            kinds = list(LEADER_KINDS)
        else:
            kinds = [args.leader]
        try:
            prev = {p.device for p in list_ports.comports()}
        except Exception:
            prev = set()
        while not _hp_stop.is_set():
            if _hp_stop.wait(1.5):
                return
            if _leader_box["leader"] is not None:
                continue
            try:
                cur = {p.device for p in list_ports.comports()}
            except Exception:
                continue
            new_ports = (cur - prev) - excl
            prev = cur
            if not new_ports:
                continue
            for dev in sorted(new_ports):
                # Probe for each acceptable kind in the configured order.
                detected = None
                if _leader_module_available:
                    for k in kinds:
                        try:
                            from leader import probe_port
                            ok, _ = probe_port(dev, k)
                        except Exception:
                            ok = False
                        if ok:
                            detected = k
                            break
                else:
                    if _so101_available:
                        ok, _ = _probe_so101(dev)
                        if ok:
                            detected = "so101"
                if detected is None:
                    continue
                if _try_install_leader(dev, kind=detected):
                    print(f"{detected} hot-plugged on {dev}", flush=True)
                    return  # one-shot install; future unplug/replug not supported

    _hp_thread: Optional[threading.Thread] = None
    if _pyserial_available and (_leader_module_available or _so101_available):
        _hp_thread = threading.Thread(target=_leader_hotplug_watcher, daemon=True,
                                      name="LeaderHotPlug")
        _hp_thread.start()

    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)
    _yaml      = _load_teleop_yaml()
    # Arm gains are 7-DOF (swivel + 6 gantry).  Older configs that still have
    # split `gantry.kp` + `drive.swivel_kp` are migrated on the fly.
    _acfg: dict = _yaml.get("arm", {})
    if "kp" in _acfg and "kd" in _acfg:
        _kp: list = list(_acfg["kp"])
        _kd: list = list(_acfg["kd"])
    else:
        _gan = _yaml.get("gantry", {})
        _drv = _yaml.get("drive", {})
        _kp = [float(_drv.get("swivel_kp", KP[0]))] + list(_gan.get("kp", KP[1:]))
        _kd = [float(_drv.get("swivel_kd", KD[0]))] + list(_gan.get("kd", KD[1:]))
    if len(_kp) != NUM_JOINTS or len(_kd) != NUM_JOINTS:
        print(f"WARNING: arm gains length {len(_kp)}/{len(_kd)} != {NUM_JOINTS}; "
              f"falling back to record_replay defaults", flush=True)
        _kp = list(KP)
        _kd = list(KD)
    _dcfg        = _yaml.get("drive", {})
    _max_linear  = float(_dcfg.get("max_linear",  2.0))
    _max_angular = float(_dcfg.get("max_angular", 1.5))
    _drive_kp    = float(_dcfg.get("kp", 0.0))
    _drive_kd    = float(_dcfg.get("kd", 3.0))
    _gp_cfg      = _yaml.get("gamepad", {})

    # -------------------------------------------------------------------------
    # Live replay controller (on-robot playback from GUI Replay tab)
    # -------------------------------------------------------------------------
    live_replay = _LiveReplay(
        kp=_kp, kd=_kd,
        max_delta=args.max_delta,
        arm_limits=arm_limits,
        all_motor_ids=list(ARM_JOINTS),
    )

    # -------------------------------------------------------------------------
    # ZMQ sockets
    # -------------------------------------------------------------------------
    ctx = zmq.Context()

    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 4)
    cmd_sock.setsockopt(zmq.LINGER,  0)
    cmd_sock.connect(args.cmd)

    # Telemetry parsing runs in its own thread (json.loads of multi-motor
    # state was a 6-10 ms p99 spike when done in the main loop).  Started
    # before the cmd-sender thread because the cmd-sender consumes
    # _telem_cache for the lead-clamp in TRACKING mode.
    _telem_stop, _telem_thread, _telem_lock, _telem_cache = \
        _start_telem_receiver(ctx, args.telem)
    _telem_last_time = 0.0  # last cache["time"] consumed by main loop

    # Dedicated cmd-sender thread.  Re-emits the latest Bundle at 100 Hz
    # in "static" mode (HOLD/IDLE/ENGAGING/SHUTDOWN), or computes q_cmd
    # live from the leader thread's latest sample in "tracking" mode —
    # which lifts the 30 Hz aliasing cap on the leader→cmd path.
    _cmd_tx_stop, _cmd_tx_thread, _cmd_lock, _cmd_holder = \
        _start_cmd_sender(
            cmd_sock,
            _lr_lock, _lr_latest, _lr_mapping,
            _telem_lock, _telem_cache,
        )

    def _post_bundle(arm=None, drive=None) -> None:
        """Post a static (prebuilt) Bundle.  Used for IDLE/HOLD/ENGAGING/SHUTDOWN."""
        msg = _build_bundle(arm, drive)
        with _cmd_lock:
            _cmd_holder["mode"]     = "static" if msg is not None else None
            _cmd_holder["bundle"]   = msg
            _cmd_holder["tracking"] = None

    def _post_tracking(*, drive: dict, kp: list, kd: list,
                       max_vel: float, max_lead: float,
                       arm_limits, vel_ff: bool = True) -> None:
        """Switch the cmd thread into TRACKING mode.

        The cmd thread will read `_lr_latest` directly at 100 Hz and compute
        q_cmd live with rate limiting per real-time elapsed.  `drive` and
        the gains are still owned by the main loop (so WASD / gain tuning
        flow through the same channel).
        """
        cfg = {
            "kp":         list(kp),
            "kd":         list(kd),
            "max_vel":    float(max_vel),
            "max_lead":   float(max_lead),
            "vel_ff":     bool(vel_ff),
            "drive":      drive,
            "arm_limits": arm_limits,
        }
        with _cmd_lock:
            _cmd_holder["mode"]     = "tracking"
            _cmd_holder["bundle"]   = None
            _cmd_holder["tracking"] = cfg

    def _clear_bundle() -> None:
        """Stop the cmd thread from emitting anything."""
        with _cmd_lock:
            _cmd_holder["mode"]     = None
            _cmd_holder["bundle"]   = None
            _cmd_holder["tracking"] = None

    # Camera + UPS reception runs in a background thread so that
    # JSON-parsing large JPEG frames never delays motor commands.
    # Scene cam is optional (static-mount mode); in rover mode the publisher
    # never streams.  We always subscribe when an endpoint is configured, but
    # treat the scene cam as *present* only once a frame actually arrives
    # (`_scene_cam_seen`, latched below in the main loop).  That keeps rover
    # sessions from (a) showing a dead UI tile and (b) dropping every recorded
    # frame waiting for a scene image that never comes.
    _scene_cam_configured = bool(args.scene_cam)
    _scene_cam_seen       = False   # latched True on first fresh scene frame
    _cam_stop, _cam_thread, _cam_lock, _cam_cache = _start_cam_receiver(
        ctx, args.gripper_cam, args.ups or None,
        scene_ep=(args.scene_cam or None),
    )

    # Background image decoder (base64 + JPEG + resize off main loop).
    # Always-on when the GUI or Rerun viewer needs decoded frames live.
    _dec_always_on = args.gui or (not args.no_rerun and _rerun_available)
    _dec_stop, _dec_thread, _dec_lock, _dec_cache, _rec_flag = \
        _start_image_decoder(_cam_lock, _cam_cache, img_size, always_on=_dec_always_on)

    # -------------------------------------------------------------------------
    # Hardware e-stop (ESP32 serial)
    # -------------------------------------------------------------------------
    _estop_flag = threading.Event()   # set = e-stop active
    _estop_stop = threading.Event()
    _estop_thread: Optional[threading.Thread] = None
    if args.estop_port:
        _estop_thread = _start_estop_reader(args.estop_port, _estop_stop, _estop_flag)

    # -------------------------------------------------------------------------
    # Rerun live camera preview (terminal mode only — GUI uses native Qt
    # widgets for cameras + scalars, which avoids the WASM/gRPC/Chromium
    # pipeline that backs up unboundedly on weak CPUs).
    # -------------------------------------------------------------------------
    use_rerun = not args.no_rerun and not args.gui
    if use_rerun and not _rerun_available:
        print("WARNING: rerun not installed — live camera preview disabled")
        use_rerun = False
    if use_rerun:
        rr.init("aizee_collect")
        rr.spawn(memory_limit="1GiB")
        _joint_names = list(ARM_JOINTS)   # swivel + 6 gantry, in joint order
        rr.set_time("time", timestamp=time.time())
        for _jn in _joint_names:
            rr.log(f"joints/{_jn}", rr.Scalars(0.0))
            rr.log(f"leader/{_jn}", rr.Scalars(0.0))
        rr.send_blueprint(rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial2DView(name="Gripper", origin="cameras/gripper"),
                rrb.TimeSeriesView(
                    name="Joint Positions",
                    contents=[f"joints/{j}" for j in _joint_names]
                            + [f"leader/{j}" for j in _joint_names],
                ),
                row_shares=[2, 1],
            )
        ))

    get_key = setup_keyboard()

    # Seed q_actual from first telemetry packet
    q_actual: Optional[np.ndarray] = None
    for _ in range(40):
        with _telem_lock:
            telem = _telem_cache["msg"]
            tt    = _telem_cache["time"]
        if telem:
            _telem_last_time = tt
            q = _qpos(telem)
            if q is not None:
                q_actual = q
                break
        time.sleep(0.05)

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------
    class State(enum.Enum):
        READY    = "ready"
        IDLE     = "idle"
        TRACKING = "tracking"
        HOLD     = "hold"
        ENGAGING = "engaging"   # rate-limited approach to leader before TRACKING
        SHUTDOWN = "shutdown"
        ESTOP    = "estop"

    # Engagement parameters: when E is pressed from READY/IDLE, the arm ramps
    # toward the leader pose at a bounded rate (instead of snapping at full
    # PD authority).  Once the arm is within ENGAGE_DONE_THRESHOLD of the
    # leader on every joint, the state auto-promotes to TRACKING.
    ENGAGE_DELTA          = 0.015  # rad/tick (~0.45 rad/s @ 30 Hz) — slow ramp
    ENGAGE_WARN_THRESHOLD = 0.20   # rad — show warning toast above this gap
    ENGAGE_DONE_THRESHOLD = 0.04   # rad — promote to TRACKING below this gap

    # Per-joint cap on q_cmd lead vs. q_actual during ENGAGING.
    # Sized so each joint can demand exactly its rated motor torque
    # (kp · lead = sat_torque).  A flat 0.05 rad cap left high-kp joints
    # (gantry_base @ kp=200) requesting only 10 N·m — below the stiction +
    # gravity load needed to break the joint loose, leading to permanent
    # stuck-engaging states.  Sizing per-joint avoids that without giving
    # the controller windup margin: at saturation, more lead would not
    # increase delivered torque, just store position error to dump on the
    # joint when it finally breaks free.
    # Per-joint cap on q_cmd lead vs. q_actual during ENGAGING.  Swivel is
    # joint 0 — its sat_torque/kp falls out of the same per-joint formula
    # the gantry uses, so there's no separate `_engage_lead_sw` anymore.
    _engage_lead_arm = np.array(
        [_SAT_TORQUE[j] / float(_kp[i]) for i, j in enumerate(ARM_JOINTS)],
        dtype=np.float32,
    )

    teleop_state                          = State.READY
    held_target:     Optional[np.ndarray] = None
    engage_q_cmd:    Optional[np.ndarray] = None
    engage_warned:   bool                 = False
    shutdown_countdown: float             = 0.0
    shutdown_target: Optional[np.ndarray] = None
    shutdown_zero_since: float            = 0.0   # when ramp first hit zero
    _SHUTDOWN_TIMEOUT                     = 3.0   # force-disable after this many seconds at zero
    arm_torques:     Optional[np.ndarray] = None
    arm_temps:       Optional[np.ndarray] = None
    arm_states:      list                 = ["?"] * NUM_JOINTS
    last_telem_time: float             = time.time() if q_actual is not None else 0.0
    ups_data:        Optional[dict]    = None
    battery_voltage: Optional[float]   = None
    robot_ok = q_actual is not None
    estop_active = False
    prev_estop_hw = False

    # Drive state (wheels)
    drive_linear         = 0.0   # current smoothed linear (-1..+1)
    drive_angular        = 0.0   # current smoothed angular (-1..+1)
    drive_linear_target  = 0.0
    drive_angular_target = 0.0
    _drive_accel         = 50.0  # instant on key press
    _drive_decel         = 8.0   # smooth release
    _last_w_time         = 0.0   # WASD timeout tracking
    _last_s_time         = 0.0
    _last_a_time         = 0.0
    _last_d_time         = 0.0
    _wasd_timeout        = 0.15  # seconds — clear target if no repeat
    wheel_states:  Optional[dict] = None   # telemetry for wheel motors

    zero_msg       = ""
    zero_msg_until = 0.0
    save_msg       = ""
    save_msg_until = 0.0

    joystick           = _init_joystick() if _pygame_available else None
    prev_gp_a:   bool  = False
    prev_gp_b:   bool  = False
    prev_gp_start:bool = False

    # M5 Joystick2 (on the OpenRB-150 leader board, I2C addr 0x63 on D11/D12).
    # `prev_m5_press_counter` lets the 30 Hz main loop edge-detect button
    # presses captured at ~500 Hz by the leader-reader thread without
    # missing quick clicks.  `_m5_dz` is the analog-stick deadzone shared
    # with the curve / ramp code below.
    prev_m5_press_counter: int = 0
    _m5_dz                     = 0.08

    # Recording state.  qpos_buf / qcmd_buf / torque_buf are now 7-DOF
    # (swivel-first, matches ARM_JOINTS) — there's no separate swivel buffer.
    # scene_buf / scene_ts_buf are populated only when --scene-cam is set
    # AND a frame actually decoded — see the recording append below.
    recording      = False
    qpos_buf:     list = []
    qcmd_buf:     list = []
    torque_buf:   list = []
    gripper_buf:  list = []
    scene_buf:    list = []
    telem_ts_buf:   list = []
    gripper_ts_buf: list = []
    scene_ts_buf:   list = []
    dropped_frames = 0
    last_rec_time  = 0.0

    # Episode metadata (GUI can mutate task_tag / notes live via the holder)
    _meta: dict = {"task_tag": args.task_tag, "notes": ""}
    last_saved_path: Optional[Path] = None

    # Camera state
    last_gripper_time   = 0.0
    last_scene_time     = 0.0
    latest_gripper: Optional[dict] = None
    latest_scene:   Optional[dict] = None
    latest_telem_ts: Optional[float] = None
    latest_gripper_ts:  Optional[float] = None
    latest_scene_ts:    Optional[float] = None
    latest_q_cmd: Optional[np.ndarray] = None  # last commanded position sent to motors
    # Scene cam push to the GUI is rate-capped (separate from publisher
    # rate / recording rate). The publisher runs at 30 Hz and the
    # decoder thread keeps _dec_cache fresh at that rate for recording.
    # The GUI feed is dropped to ~6 Hz because the 3D model preview's
    # paint cost (hull projection + pointcloud + frustum + thumbnail)
    # dominates the QtRenderer thread; visually 6 Hz is plenty for a
    # fixed workspace cam and frees ~80% of the per-second paint budget
    # compared to 30 Hz.
    _SCENE_GUI_PERIOD = 0.18
    _last_scene_gui_push = 0.0

    # Leader force-feedback state (OpenRB gripper only).  Smoothed copy of
    # follower gripper torque; LPF coefficient is chosen so a noisy 0.05 N·m
    # ripple decays to <10% within ~5 ticks (~170 ms at 30 Hz) — not so
    # heavy that real grip events feel mushy.
    _ff_gripper_lpf:  float = 0.0
    _FF_LPF_ALPHA            = 0.30
    _ff_was_active           = False   # tracks last-tick FF state for clean release

    status = "[ ] ready — motors off"
    hint   = ("E=hold · I=idle · Q=quit" if leader is None
              else "E=track · I=idle · Z=zero · M=mirror · Q=quit")

    _nan = float("nan")
    # Display arrays now match q_actual shape directly (swivel = index 0).
    _init_actual = q_actual.copy() if q_actual is not None else None

    # -------------------------------------------------------------------------
    # Display: terminal renderer (default) or Qt GUI (--gui)
    # -------------------------------------------------------------------------
    gui_cmd_queue: queue.Queue = queue.Queue(maxsize=32)
    _qt_renderer = None
    _disp_thread = None
    _disp_stop: Optional[threading.Event] = None
    _disp_event: Optional[threading.Event] = None

    if args.gui:
        from collect_demo_gui import QtRenderer

        import os
        def _on_delete_last(path: Path) -> None:
            os.remove(path)

        _qt_renderer = QtRenderer(
            cmd_queue=gui_cmd_queue,
            meta=_meta,
            on_delete_last=_on_delete_last,
            output_dir=Path(args.output_dir),
            camera_ctrl_endpoint=(args.gripper_cam_ctrl or None),
        )
        _disp_lock   = _qt_renderer.lock
        _disp_holder = _qt_renderer.holder
        _disp_cams   = _qt_renderer.cam_holder
        _disp_stop   = _qt_renderer.stop_event
        _qt_renderer.start()
    else:
        _disp_stop, _disp_thread, _disp_lock, _disp_holder, _disp_event = \
            _start_display_thread()
        _disp_cams: Optional[dict] = None

    # Queue the initial frame (first=True is the default in holder)
    with _disp_lock:
        _disp_holder["args"] = dict(
            leader_rad=None, target=None, actual=_init_actual,
            status=status, hint=hint, robot_ok=robot_ok,
            leader_connected=(leader is not None),
            wheel_states=wheel_states, wheels_enabled=False,
        )
    if _disp_event is not None:
        _disp_event.set()

    # -------------------------------------------------------------------------
    # Background Rerun thread (avoids blocking main loop on rr.log IPC)
    # -------------------------------------------------------------------------
    _rr_stop:   Optional[threading.Event]  = None
    _rr_thread: Optional[threading.Thread] = None
    _rr_lock:   Optional[threading.Lock]   = None
    _rr_holder: Optional[dict]             = None
    _rr_event:  Optional[threading.Event]  = None
    if use_rerun:
        _rr_stop, _rr_thread, _rr_lock, _rr_holder, _rr_event = \
            _start_rerun_thread(_dec_lock, _dec_cache)

    frame_counter = 0
    period = 1.0 / LOOP_HZ

    _prof_log_path = Path(__file__).resolve().parent.parent.parent / "logs" / "loop_prof.log"
    _prof = _LoopProfiler(log_path=_prof_log_path)

    _save_thread:        Optional[threading.Thread] = None
    _save_result_holder: list                       = [None]

    def _start_async_save(out_dir, qb, gb, tb, gtb, dur, drop_note, tag="",
                          qcb=None, tqb=None, sb=None, stb=None,
                          task_tag="", notes=""):
        def _run():
            try:
                p, T = save_episode(
                    out_dir, qb, gb,
                    telem_ts_buf=tb, gripper_ts_buf=gtb,
                    qcmd_buf=qcb, torque_buf=tqb,
                    scene_buf=sb, scene_ts_buf=stb,
                    task_tag=task_tag, notes=notes,
                )
                _save_result_holder[0] = (p, f"[SAVED {p.name}  {T} steps  {dur:.1f}s{drop_note}]{tag}")
            except Exception as e:
                _save_result_holder[0] = (None, f"[SAVE ERROR: {e}]")
        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t

    def _finalize_recording(reason: str, t_now: float) -> None:
        """Stop recording and dispatch an async save (or dry-run / skip-empty).

        reason is a short free-text suffix shown in status + attached to the
        save-success message ("" = user R toggle; " (hw e-stop)" / " (e-stop)" /
        " (max steps)" for auto-stop paths).
        """
        nonlocal recording, save_msg, save_msg_until, _save_thread
        if not recording:
            return
        recording = False
        _rec_flag.clear()
        steps     = len(qpos_buf)
        dur       = steps / REC_HZ
        drop_note = f"  drop:{dropped_frames}" if dropped_frames else ""
        tag_txt   = reason if reason else ""

        if steps == 0:
            save_msg       = f"[STOPPED{tag_txt} — 0 steps, nothing saved]"
            save_msg_until = t_now + 5.0
            return

        if args.dry_run:
            save_msg       = f"[DRY RUN]{tag_txt} {steps} steps  {dur:.1f}s{drop_note}"
            save_msg_until = t_now + 5.0
            return

        save_msg               = f"[saving {steps} steps{tag_txt}...]"
        save_msg_until         = t_now + 120.0
        _save_result_holder[0] = None
        _save_thread = _start_async_save(
            args.output_dir, qpos_buf, gripper_buf,
            telem_ts_buf, gripper_ts_buf,
            dur, drop_note, tag=tag_txt, qcb=qcmd_buf, tqb=torque_buf,
            sb=scene_buf, stb=scene_ts_buf,
            task_tag=_meta["task_tag"], notes=_meta["notes"],
        )

    try:
        while True:
            t0 = time.time()
            _prof.begin()

            # -----------------------------------------------------------------
            # Hot-plug: install a leader handed off by the watcher thread
            # -----------------------------------------------------------------
            if _install_pending:
                with _install_lock:
                    _p = _install_pending.pop("data", None)
                if _p is not None:
                    leader           = _p["leader"]
                    zero_offsets     = _p["zero_offsets"]
                    directions       = _p["directions"]
                    _so101_for_aizee = _p["for_aizee"]
                    _emits_urdf      = _p.get("emits_urdf", False)
                    # Publish to the cmd-thread mapping state.
                    with _lr_lock:
                        _lr_mapping["zero_offsets"] = zero_offsets
                        _lr_mapping["directions"]   = directions
                        _lr_mapping["for_aizee"]    = _so101_for_aizee
                        _lr_mapping["emits_urdf"]   = _emits_urdf
                    print(f"[hot-plug] leader installed — {len(_so101_for_aizee)} arm joints mapped",
                          flush=True)

            # -----------------------------------------------------------------
            # Pick up completed background save
            # -----------------------------------------------------------------
            if _save_thread is not None and not _save_thread.is_alive():
                if _save_result_holder[0] is not None:
                    _saved_path, save_msg = _save_result_holder[0]
                    save_msg_until  = t0 + 5.0
                    last_saved_path = _saved_path
                    _save_result_holder[0] = None
                _save_thread = None

            # -----------------------------------------------------------------
            # Read cached camera data (populated by background thread)
            # -----------------------------------------------------------------
            with _cam_lock:
                if _cam_cache["gripper"] is not None:
                    latest_gripper    = _cam_cache["gripper"]
                    last_gripper_time = _cam_cache["gripper_time"]
                    latest_gripper_ts = _cam_cache["gripper_ts"]
                if _cam_cache.get("scene") is not None:
                    latest_scene    = _cam_cache["scene"]
                    last_scene_time = _cam_cache["scene_time"]
                    latest_scene_ts = _cam_cache.get("scene_ts")

            cam_age = (t0 - last_gripper_time) if last_gripper_time > 0 else 999.0
            # End-to-end frame age (publisher capture timestamp → host now).
            # Includes any clock skew between Jetson and host; we care about
            # *drift* over time, which is skew-invariant.
            if latest_gripper_ts is not None:
                _prof.gauge("gripper_age_ms", (t0 - latest_gripper_ts) * 1000.0)
            # Time since this loop last *received* a new cam frame (host-only,
            # no clock-skew component) — flags publisher gaps directly.
            _prof.gauge("gripper_recv_age_ms", cam_age * 1000.0)
            if _scene_cam_configured:
                scene_age = (t0 - last_scene_time) if last_scene_time > 0 else 999.0
                if latest_scene_ts is not None:
                    _prof.gauge("scene_age_ms", (t0 - latest_scene_ts) * 1000.0)
                _prof.gauge("scene_recv_age_ms", scene_age * 1000.0)
                # Latch presence the first time a fresh scene frame arrives —
                # distinguishes static mode (scene present) from rover mode.
                if not _scene_cam_seen and scene_age < _CAM_STALE:
                    _scene_cam_seen = True
            else:
                scene_age = 999.0

            # Wake the Rerun thread (~15 Hz, every other frame).  Camera
            # frames are pulled from the shared decoder cache by the
            # Rerun thread itself — no JPEG copy through the main loop.
            # Joint data is queued later, after telemetry + leader are read.
            if _rr_event is not None and (frame_counter % 2 == 0):
                with _rr_lock:
                    _rr_holder["time"] = t0
                _rr_event.set()

            # Push raw JPEG bytes to the GUI's native camera widget — only
            # when a new frame has actually arrived (last_*_time changed),
            # otherwise we'd re-feed the same JPEG every loop tick.
            if _disp_cams is not None:
                push_g = (latest_gripper is not None
                          and last_gripper_time > _disp_cams["gripper_ts"])
                if push_g:
                    gj_bytes = latest_gripper.get("color", {}).get("data_bytes")
                    if gj_bytes is not None:
                        with _disp_lock:
                            _disp_cams["gripper"]    = gj_bytes
                            _disp_cams["gripper_ts"] = last_gripper_time
                        # Mirror to the WebXR client (no-op when not in quest mode).
                        if _quest_state is not None:
                            _quest_state.latest_cam_jpeg = gj_bytes
                            _quest_state.latest_cam_seq += 1
                # Scene cam: rate-cap to _SCENE_GUI_PERIOD so the GUI
                # thread isn't slammed by 30 Hz × (paint + pointcloud).
                # Recording continues at REC_HZ through _dec_cache — this
                # throttle ONLY affects the 3D model preview + scene-tile
                # refresh rate.
                #
                # The GUI gets pre-projected camera-frame points from the
                # decoder thread (heavy numpy already done there), the
                # paired publisher ts, and the raw JPEG bytes for the
                # tile + thumbnail (Qt's native decoder is fast). No depth
                # bytes / intrinsics travel through here anymore.
                push_s = (latest_scene is not None
                          and last_scene_time > _disp_cams.get("scene_ts", 0.0)
                          and (t0 - _last_scene_gui_push) >= _SCENE_GUI_PERIOD)
                if push_s:
                    with _dec_lock:
                        sj_img       = _dec_cache.get("scene")
                        sj_ts_pub    = _dec_cache.get("scene_ts_pub")
                        s_pts_cam    = _dec_cache.get("scene_pts_cam")
                        s_pts_depth  = _dec_cache.get("scene_pts_depth")
                    sj_bytes = latest_scene.get("color", {}).get("data_bytes")
                    if sj_bytes is not None and sj_img is not None:
                        with _disp_lock:
                            _disp_cams["scene"]           = sj_bytes
                            _disp_cams["scene_ts"]        = (
                                sj_ts_pub if sj_ts_pub is not None
                                else last_scene_time)
                            _disp_cams["scene_pts_cam"]   = s_pts_cam
                            _disp_cams["scene_pts_depth"] = s_pts_depth
                        _last_scene_gui_push = t0

            _prof.tick("cam")

            # -----------------------------------------------------------------
            # Gamepad + drive axes
            # -----------------------------------------------------------------
            # When pygame handles WASD, drain terminal buffer discarding WASD
            # so held W doesn't starve command keys (E, Q, etc.)
            if _pygame_available:
                key = None
                while True:
                    _k = get_key()
                    if _k is None:
                        break
                    if _k not in ("W", "A", "S", "D"):
                        key = _k
            else:
                key = get_key()

            # GUI button presses flow through the same key dispatch as keyboard.
            # Drain one per frame so rapid clicks don't starve state transitions.
            try:
                key = gui_cmd_queue.get_nowait()
            except queue.Empty:
                pass

            # Dict commands (live-replay control protocol from GUI)
            if isinstance(key, dict):
                _cmd = key.get("cmd", "")
                if _cmd == "replay_on":
                    if recording:
                        _finalize_recording(" (replay)", t0)
                    _path = Path(key.get("path", ""))
                    err = live_replay.load(_path)
                    if err is None and live_replay.enter_live():
                        # Park teleop while replaying — motors off until user arms
                        teleop_state = State.READY
                        save_msg       = f"[replay loaded] {_path.name}"
                        save_msg_until = t0 + 3.0
                    else:
                        save_msg       = f"[replay load failed] {err or 'no episode'}"
                        save_msg_until = t0 + 5.0
                elif _cmd == "replay_off":
                    if not live_replay.exit_live():
                        save_msg       = "[replay] stop first before exiting live mode"
                        save_msg_until = t0 + 3.0
                elif _cmd == "replay_arm":
                    for _c in live_replay.arm(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_play":
                    for _c in live_replay.play(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_pause":
                    live_replay.pause()
                elif _cmd == "replay_toggle":
                    for _c in live_replay.toggle(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_restart":
                    for _c in live_replay.restart(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_stop":
                    live_replay.stop(q_actual)
                elif _cmd == "replay_speed":
                    live_replay.set_speed(float(key.get("speed", 1.0)))
                elif _cmd == "replay_opts":
                    live_replay.set_opts(**{k: v for k, v in key.items() if k != "cmd"})
                elif _cmd == "leader_quest_on":
                    # GUI-driven Quest VR enable.  Builds server lazily on
                    # first toggle; subsequent toggles reuse the same state.
                    if _install_quest_leader():
                        save_msg = f"[quest] connected on :{args.quest_port}"
                        save_msg_until = t0 + 4.0
                    else:
                        save_msg = "[quest] connect failed (see console)"
                        save_msg_until = t0 + 4.0
                elif _cmd == "leader_quest_off":
                    _uninstall_leader()
                    save_msg = "[quest] disconnected"
                    save_msg_until = t0 + 3.0
                elif _cmd == "quest_sim_reset":
                    if _quest_state is not None:
                        _quest_state.pending_commands.append({"cmd": "reset_sim"})
                        save_msg = "[quest] sim reset to home"
                        save_msg_until = t0 + 2.0
                elif _cmd == "quest_sim_align":
                    if _quest_state is not None:
                        _quest_state.pending_commands.append({"cmd": "align_to_actual"})
                        save_msg = "[quest] sim aligned to arm"
                        save_msg_until = t0 + 2.0
                key = None

            # Block teleop motor/recording keys while live replay owns the arm
            if live_replay.live and key in ("E", "I", "H", "X", "R", "Z", "M", "P",
                                            "PEDAL_A", "PEDAL_B", "PEDAL_C"):
                key = None

            # -----------------------------------------------------------------
            # M5 Joystick2 (on the OpenRB-150 leader, I2C 0x63 on D11/D12).
            # Read first so it takes precedence over the xbox gamepad and
            # the WASD path below — operator's hand-stick is the primary
            # drive when present.  Button-press edges are detected against
            # the leader thread's monotonic counter so quick clicks survive
            # the 30 Hz main-loop cadence.
            # -----------------------------------------------------------------
            with _lr_lock:
                m5 = dict(_lr_latest["joy"])
            _m5_active = False
            if m5["present"]:
                # Map axes to drive (matches xbox convention in _read_gamepad):
                #   Y → angular (forward/back, negated so stick-fwd = robot-fwd)
                #   X → linear  (turn)
                _m5_x = _apply_curve(_apply_deadzone(m5["x"], _m5_dz))
                _m5_y = _apply_curve(_apply_deadzone(m5["y"], _m5_dz))
                _m5_active = (abs(_m5_x) > 0.01 or abs(_m5_y) > 0.01)

                # Button-press edge → toggle recording, but only when it
                # would actually do something.  When idle / ready / engaging
                # (or while live replay owns the arm) we silently ignore the
                # click — matches user policy "do nothing if the arm is not
                # tracking".  The press counter is always consumed so a
                # silently-ignored edge can't fire later under different state.
                if m5["press_counter"] != prev_m5_press_counter:
                    can_toggle = (recording or teleop_state == State.TRACKING)
                    if can_toggle and not live_replay.live:
                        key = "R"
                    prev_m5_press_counter = m5["press_counter"]

            _stick_active = False
            if joystick is not None:
                gp = _read_gamepad(joystick, prev_gp_a, prev_gp_b, prev_gp_start,
                                   gp_cfg=_gp_cfg)
                prev_gp_a     = gp["raw_a"]
                prev_gp_b     = gp["raw_b"]
                prev_gp_start = gp["raw_start"]
                # Stick axes → drive targets (always apply, 0 when centered)
                _stick_active = (abs(gp["drive_linear"]) > 0.01
                                 or abs(gp["drive_angular"]) > 0.01)
                drive_linear_target  = gp["drive_linear"]
                drive_angular_target = gp["drive_angular"]
                if gp["enable"] and teleop_state in (State.READY, State.IDLE):
                    key = "E"
                if gp["hold"] and teleop_state in (State.TRACKING, State.HOLD,
                                                    State.IDLE, State.ENGAGING):
                    key = "H"
                if gp["shutdown"]:
                    key = "CANCEL_SHUTDOWN" if teleop_state == State.SHUTDOWN else "X"
                if gp["quit"]:
                    key = "Q"

            # M5 stick wins over xbox when deflected — operator's primary
            # input.  When the M5 is centred, fall back to whatever the
            # xbox stick set above (or to WASD below if neither stick is live).
            if _m5_active:
                drive_angular_target = -_m5_y
                drive_linear_target  =  _m5_x
                _stick_active        = True

            # -----------------------------------------------------------------
            # WASD drive input — pygame true key state (no repeat delay)
            # Matches teleop.py read_keyboard_pygame(): instant on/off.
            # -----------------------------------------------------------------
            if _pygame_available:
                # Pump events if no joystick did it already
                if joystick is None:
                    pygame.event.pump()
                _pkeys = pygame.key.get_pressed()
                # WORKAROUND: motor controller has linear/angular backwards
                # W/S → angular (forward/back), A/D → linear (turn)
                _kb_ang = 0.0
                _kb_lin = 0.0
                if _pkeys[pygame.K_w]:
                    _kb_ang = -1.0
                elif _pkeys[pygame.K_s]:
                    _kb_ang = 1.0
                if _pkeys[pygame.K_d]:
                    _kb_lin = 1.0
                elif _pkeys[pygame.K_a]:
                    _kb_lin = -1.0
                # Keyboard overrides only when stick is idle
                if not _stick_active:
                    drive_angular_target = _kb_ang
                    drive_linear_target  = _kb_lin
            else:
                # Fallback: terminal key with timeout (has OS repeat delay)
                if not _stick_active and key in ("W", "S", "A", "D"):
                    if key == "W":
                        drive_angular_target = -1.0
                        _last_w_time = t0
                    elif key == "S":
                        drive_angular_target = 1.0
                        _last_s_time = t0
                    elif key == "A":
                        drive_linear_target = -1.0
                        _last_a_time = t0
                    elif key == "D":
                        drive_linear_target = 1.0
                        _last_d_time = t0
                    key = None
                if not _stick_active:
                    if (t0 - _last_w_time > _wasd_timeout
                            and t0 - _last_s_time > _wasd_timeout):
                        drive_angular_target = 0.0
                    if (t0 - _last_a_time > _wasd_timeout
                            and t0 - _last_d_time > _wasd_timeout):
                        drive_linear_target = 0.0

            # Zero drive targets while live replay owns the rover
            if live_replay.live:
                drive_linear_target  = 0.0
                drive_angular_target = 0.0

            # Drive smoothing (fast accel, smooth decel)
            drive_linear  = _ramp_toward(drive_linear,  drive_linear_target,
                                         _drive_accel, _drive_decel, period)
            drive_angular = _ramp_toward(drive_angular, drive_angular_target,
                                         _drive_accel, _drive_decel, period)

            # -----------------------------------------------------------------
            # Foot pedal (3-button USB keyboard emitting A/B/C, captured by
            # the Qt GUI and sent through gui_cmd_queue as PEDAL_*).
            #   A → ENABLE     (delegate to "E" handler below)
            #   B → toggle: IDLE while active, RESUME while idle
            #   C → SHUTDOWN   (delegate to "X" handler below)
            # B is handled inline because the keyboard "I" handler only fires
            # from READY/IDLE, but the pedal must reach IDLE from TRACKING /
            # ENGAGING / HOLD too.
            # -----------------------------------------------------------------
            if key == "PEDAL_A":
                key = "E"
            elif key == "PEDAL_C":
                key = "X"
            elif key == "PEDAL_B":
                if teleop_state == State.IDLE:
                    key = "E"   # resume tracking
                elif teleop_state in (State.TRACKING, State.ENGAGING,
                                      State.HOLD):
                    if recording:
                        _finalize_recording(" (pedal idle)", t0)
                    _send(cmd_sock, {"type": "enable",
                                     "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                    ref = (q_actual.tolist() if q_actual is not None
                           else [0.0] * NUM_JOINTS)
                    _send(cmd_sock, {
                        "type": "arm_joints", "positions": ref,
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    })
                    teleop_state = State.IDLE
                    key = None
                else:
                    key = None  # READY / SHUTDOWN — pedal B is a no-op

            # -----------------------------------------------------------------
            # Keyboard (command keys)
            # -----------------------------------------------------------------
            if key == "Q":
                break

            elif key == "I":
                if teleop_state in (State.READY, State.IDLE):
                    _send(cmd_sock, {"type": "enable",
                                     "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                    ref = q_actual.tolist() if q_actual is not None else [0.0] * NUM_JOINTS
                    _send(cmd_sock, {
                        "type": "arm_joints", "positions": ref,
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    })
                    teleop_state = State.IDLE

            elif key == "E":
                if teleop_state in (State.READY, State.IDLE):
                    # Safety: if no actuator positions have been read since
                    # the app opened, briefly hold the arm idle (zero kp/kd/
                    # torque) so telemetry can populate q_actual.  Without
                    # this, engage_q_cmd / held_target fall back to zeros
                    # and the PD loop snaps the arm toward 0 on every joint.
                    if q_actual is None:
                        _send(cmd_sock, {"type": "enable",
                                         "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                        _send(cmd_sock, {
                            "type": "arm_joints",
                            "positions":  [0.0] * NUM_JOINTS,
                            "velocities": [0.0] * NUM_JOINTS,
                            "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                            "torques":    [0.0] * NUM_JOINTS,
                        })
                        _wait_deadline = time.time() + 1.5
                        while time.time() < _wait_deadline:
                            with _telem_lock:
                                _tm = _telem_cache["msg"]
                                _tt = _telem_cache["time"]
                            if _tm is not None and _tt > _telem_last_time:
                                _q = _qpos(_tm)
                                if _q is not None:
                                    q_actual         = _q
                                    _telem_last_time = _tt
                                    last_telem_time  = t0
                                    robot_ok         = True
                                    break
                            time.sleep(0.05)
                    _send(cmd_sock, {"type": "enable",
                                     "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                    if leader is not None:
                        # Soft engage — seed the integrator from the current
                        # arm pose so ENGAGING ramps slowly to the leader
                        # instead of snapping at full PD authority.
                        engage_q_cmd  = (q_actual.copy().astype(np.float32)
                                         if q_actual is not None else None)
                        engage_warned = False
                        teleop_state  = State.ENGAGING
                    else:
                        held_target  = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                        teleop_state = State.HOLD

            elif key == "H":
                if teleop_state in (State.TRACKING, State.ENGAGING):
                    held_target  = q_actual.copy() if q_actual is not None else held_target
                    teleop_state = State.HOLD
                elif teleop_state == State.HOLD:
                    teleop_state = State.TRACKING if leader is not None else State.IDLE
                elif teleop_state == State.IDLE:
                    held_target  = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                    teleop_state = State.HOLD

            elif key == "R":
                if not recording:
                    if teleop_state == State.TRACKING:
                        recording      = True
                        _rec_flag.set()   # start background image decoder
                        qpos_buf       = []
                        qcmd_buf       = []
                        torque_buf     = []
                        gripper_buf    = []
                        scene_buf      = []
                        telem_ts_buf   = []
                        gripper_ts_buf = []
                        scene_ts_buf   = []
                        dropped_frames = 0
                        last_rec_time  = 0.0
                    else:
                        save_msg       = "[record blocked] enable tracking first (E)"
                        save_msg_until = t0 + 2.0
                else:
                    _finalize_recording("", t0)

            elif key == "Z" and leader is not None:
                if not getattr(leader, "SUPPORTS_ZEROING", True):
                    # Cartesian VR leader self-zeroes on every clutch engage;
                    # there's no persistent offset to capture, and running
                    # the zeroing math would corrupt the IK mapping.
                    zero_msg       = "[Z] n/a — VR leader self-zeroes on grip"
                    zero_msg_until = t0 + 2.0
                else:
                    with _lr_lock:
                        _z = _lr_latest["rad"]
                    if _z is not None:
                        zero_offsets = _z.copy()
                        leader.save_zero(zero_offsets)
                        # Republish to the cmd-thread mapping so its TRACKING
                        # compute picks up the new zero immediately.
                        with _lr_lock:
                            _lr_mapping["zero_offsets"] = zero_offsets
                        zero_msg       = "[Z] zeroed — saved"
                        zero_msg_until = t0 + 2.0

            elif key == "M" and leader is not None:
                if not getattr(leader, "SUPPORTS_ZEROING", True):
                    zero_msg       = "[M] n/a — VR leader self-zeroes on grip"
                    zero_msg_until = t0 + 2.0
                    _m = None
                else:
                    with _lr_lock:
                        _m = _lr_latest["rad"]
                if _m is not None and q_actual is not None:
                    # Mirror: pin zero so leader maps to current actual.
                    # ARM_JOINTS now includes swivel as joint 0, so a single
                    # loop covers all 7 joints.
                    # q_actual is now URDF frame; for physical leaders the
                    # mapped target is `directions * (leader - zero) * sign`
                    # → solve for zero such that target == q_actual:
                    #   zero = leader - q_actual * sign * direction
                    new_offsets = zero_offsets.copy()
                    for ai, si in enumerate(_so101_for_aizee):
                        s = alignment._ALIGN_SIGNS[ai] if not _emits_urdf else 1.0
                        new_offsets[si] = _m[si] - directions[si] * s * q_actual[ai]
                    zero_offsets = new_offsets
                    leader.save_zero(zero_offsets)
                    with _lr_lock:
                        _lr_mapping["zero_offsets"] = zero_offsets
                    zero_msg       = "[M] mirrored — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "P":
                if q_actual is not None:
                    ready = {
                        "arm_joints": list(ARM_JOINTS),
                        "positions": q_actual.tolist(),
                    }
                    rp_path = Path(__file__).resolve().parent.parent.parent / "config" / "ready_pose.json"
                    rp_path.parent.mkdir(parents=True, exist_ok=True)
                    rp_path.write_text(json.dumps(ready, indent=2))
                    zero_msg       = f"[P] ready pose saved"
                    zero_msg_until = t0 + 3.0
                else:
                    zero_msg       = "[P] no telemetry — cannot save"
                    zero_msg_until = t0 + 2.0

            elif key == "K":
                # Hardware mechanical zero — sends Robstride CAN ZeroPos + SaveConfig
                # to every arm joint. Refused unless motors are fully disabled
                # (motor_control gates the same way), since a running motor may
                # produce a torque spike when zeroed.
                if teleop_state != State.READY:
                    zero_msg       = "[K] disable arm first (X), then press K"
                    zero_msg_until = t0 + 3.0
                else:
                    pre = (", ".join(f"{j}={q_actual[i]:+.3f}"
                                     for i, j in enumerate(ARM_JOINTS))
                           if q_actual is not None else "no telemetry")
                    _send(cmd_sock, {
                        "type": "mech_zero",
                        "motor_ids": list(ARM_JOINTS),
                        "save": True,
                    })
                    print(f"[K] mech_zero sent — pre: {pre}")
                    zero_msg       = "[K] mech zero sent — saved to flash"
                    zero_msg_until = t0 + 4.0

            elif key == "CANCEL_SHUTDOWN" and teleop_state == State.SHUTDOWN:
                teleop_state = State.HOLD
                held_target  = q_actual.copy() if q_actual is not None else held_target

            elif key == "X":
                if teleop_state in (State.TRACKING, State.HOLD, State.IDLE,
                                    State.ENGAGING):
                    shutdown_target    = (q_actual.copy() if q_actual is not None
                                          else held_target.copy() if held_target is not None
                                          else np.zeros(NUM_JOINTS))
                    shutdown_countdown  = 1.0
                    shutdown_zero_since = 0.0
                    teleop_state        = State.SHUTDOWN
                    if recording:
                        recording = False   # stop recording on shutdown
                        _rec_flag.clear()

            _prof.tick("input")

            # -----------------------------------------------------------------
            # Leader data
            # -----------------------------------------------------------------
            leader_rad:    Optional[np.ndarray] = None
            leader_vel:    Optional[np.ndarray] = None
            _clamped_live: Optional[list]       = None
            aizee_cmd:     Optional[np.ndarray] = None    # 7-DOF, swivel-first
            aizee_vel_ff:  Optional[np.ndarray] = None
            leader_age:    float                = 999.0

            if leader is not None:
                with _lr_lock:
                    leader_rad    = _lr_latest["rad"]
                    leader_vel    = _lr_latest["vel"]
                    _clamped_live = _lr_latest["clamped"]
                    _leader_t     = _lr_latest["time"]
                leader_age = t0 - _leader_t if _leader_t > 0 else 999.0
                if leader_rad is not None:
                    mapped = directions * (leader_rad - zero_offsets)
                    aizee_cmd = mapped[_so101_for_aizee]
                    # Convert motor-frame leader mapping → URDF frame for
                    # physical leaders; see _compute_tracking_bundle for
                    # the full explanation.  QuestLeader sets _emits_urdf
                    # so it skips this fold.
                    if not _emits_urdf:
                        aizee_cmd = aizee_cmd * alignment._ALIGN_SIGNS
                if leader_vel is not None:
                    # Velocity has the same sign-flip mapping as position
                    # (zero_offset cancels under differentiation).
                    aizee_vel_ff = (directions * leader_vel)[_so101_for_aizee]
                    if not _emits_urdf:
                        aizee_vel_ff = aizee_vel_ff * alignment._ALIGN_SIGNS

            # Determine target (7-DOF in ARM_JOINTS order; swivel is index 0).
            if live_replay.live:
                target = live_replay.current_target
            elif teleop_state == State.HOLD:
                target = held_target
            elif aizee_cmd is not None:
                target = aizee_cmd
            else:
                target = q_actual

            _prof.tick("leader")

            # -----------------------------------------------------------------
            # Hardware e-stop gate — skip ALL motor commands so watchdog
            # holds position (arm doesn't fall).
            # -----------------------------------------------------------------
            estop_hw_active = _estop_flag.is_set()
            if estop_hw_active and not prev_estop_hw:
                _finalize_recording(" (hw e-stop)", t0)
            prev_estop_hw = estop_hw_active

            # -----------------------------------------------------------------
            # Send motor commands
            # -----------------------------------------------------------------
            if estop_hw_active:
                pass  # watchdog holds position

            elif live_replay.live:
                # Live replay owns the arm — send whatever step() emits.
                # Clear the cmd-thread holder every tick so a stale bundle
                # left over from before replay_on can't keep getting
                # re-emitted at 100 Hz between our 30 Hz replay sends
                # (used to undermine ARMING with leftover kp=0 IDLE holds).
                _clear_bundle()
                for _c in live_replay.step(t0, q_actual, period):
                    _send(cmd_sock, _c)

            # Send drive command every tick (feeds watchdog, enables WASD/stick movement)
            elif teleop_state == State.READY:
                _clear_bundle()  # don't let cmd thread re-emit any prior teleop bundle

            elif teleop_state == State.SHUTDOWN:
                drive_zero = {"linear": 0.0, "angular": 0.0,
                              "kp": _drive_kp, "kd": _drive_kd}
                dt         = period
                max_change = 0.2 * dt   # 0.2 rad/s ramp
                if shutdown_countdown > 0:
                    shutdown_countdown -= dt
                    arm_payload = (
                        {"positions": shutdown_target.tolist(),
                         "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                         "torques": [0.0] * NUM_JOINTS}
                        if shutdown_target is not None else None
                    )
                    _post_bundle(arm=arm_payload, drive=drive_zero)
                else:
                    if shutdown_target is None:
                        shutdown_target = np.zeros(NUM_JOINTS)
                    ref     = q_actual if q_actual is not None else shutdown_target
                    new_tgt = shutdown_target.copy()
                    for i in range(len(new_tgt)):
                        new_tgt[i] = (0.0 if abs(new_tgt[i]) < max_change
                                      else new_tgt[i] - np.sign(new_tgt[i]) * max_change)
                    shutdown_target = new_tgt
                    ramp_done = bool(np.all(np.abs(shutdown_target) < 0.01))
                    if ramp_done and shutdown_zero_since == 0.0:
                        shutdown_zero_since = t0
                    actual_close = (q_actual is None or np.all(np.abs(q_actual) < 0.05))
                    timed_out    = (shutdown_zero_since > 0
                                    and t0 - shutdown_zero_since >= _SHUTDOWN_TIMEOUT)
                    if ramp_done and (actual_close or timed_out):
                        # Stop the cmd-sender thread from re-emitting the
                        # last shutdown bundle, then send disable.  We do
                        # NOT reuse the cmd thread for `disable` — it's a
                        # one-shot transition, not a periodic command.
                        _clear_bundle()
                        _send(cmd_sock, {"type": "disable",
                                         "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                        drive_linear = drive_angular = 0.0
                        drive_linear_target = drive_angular_target = 0.0
                        teleop_state = State.READY
                    else:
                        delta   = np.clip(shutdown_target - ref, -args.max_delta, args.max_delta)
                        q_cmd   = ref + delta
                        if arm_limits:
                            q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                        _post_bundle(
                            arm={"positions": q_cmd.tolist(),
                                 "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                                 "torques": [0.0] * NUM_JOINTS},
                            drive=drive_zero,
                        )

            elif teleop_state == State.IDLE:
                arm_payload: Optional[dict] = None
                if q_actual is not None:
                    arm_payload = {
                        "positions": q_actual.tolist(),
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    }
                _post_bundle(
                    arm=arm_payload,
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                )

            elif teleop_state == State.ENGAGING:
                # One-shot warning when engaging with a large gap to leader.
                if (not engage_warned and target is not None
                        and q_actual is not None):
                    gap = float(np.max(np.abs(target - q_actual)))
                    if gap > ENGAGE_WARN_THRESHOLD:
                        save_msg       = (f"[engaging] leader is {gap:.2f} rad "
                                          f"from arm — ramping slowly")
                        save_msg_until = t0 + 4.0
                    engage_warned = True

                arm_payload: Optional[dict] = None
                if target is not None:
                    # Integrate previous command (not q_actual) to keep the
                    # ramp smooth, then clamp the lead vs. q_actual so the
                    # commanded velocity is bounded by physical motion.
                    ref = (engage_q_cmd if engage_q_cmd is not None
                           else (q_actual if q_actual is not None else target))
                    if q_actual is not None:
                        lead = np.clip(ref - q_actual,
                                       -_engage_lead_arm, _engage_lead_arm)
                        ref  = q_actual + lead
                    delta = np.clip(target - ref, -ENGAGE_DELTA, ENGAGE_DELTA)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                    engage_q_cmd = q_cmd
                    latest_q_cmd = q_cmd.copy()
                    arm_payload = {
                        "positions": q_cmd.tolist(),
                        "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                        "torques": [0.0] * NUM_JOINTS,
                    }
                _post_bundle(
                    arm=arm_payload,
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                )
                # Promote to TRACKING once every joint is close (swivel is
                # joint 0 of the arm — no separate close-check anymore).
                arm_close = (q_actual is not None and target is not None
                             and np.max(np.abs(target - q_actual))
                                 < ENGAGE_DONE_THRESHOLD)
                if arm_close:
                    teleop_state  = State.TRACKING
                    engage_q_cmd  = None

            elif teleop_state == State.TRACKING:
                # Hand the leader→q_cmd computation off to the cmd-sender
                # thread.  It reads `_lr_latest` directly at 100 Hz and
                # computes q_cmd live with rate-limiting per real-time
                # elapsed — bounding leader→motor latency to ~10 ms instead
                # of the ~33 ms aliasing the 30 Hz main loop used to add.
                #
                # Quest gets tighter limits than the physical leaders: IK
                # near a singularity can throw a multi-radian jump in
                # leader_rad in a single sample, and the operator has no
                # haptic feedback to stop it.  Lower max_vel caps chase
                # speed; lower max_lead caps PD torque during the chase
                # (peak τ ≈ kp · max_lead = 30 · 0.20 = 6 N·m vs the
                # previous 18 N·m).  Engage mode's 0.45 rad/s smoothing
                # is what makes it feel safe — tracking now meets it
                # closer to half-way while staying responsive enough for
                # normal hand motion.
                if _leader_kind == "quest":
                    _tr_max_vel  = 3.0   # rad/s
                    _tr_max_lead = 0.20  # rad
                else:
                    _tr_max_vel  = args.max_delta * LOOP_HZ
                    _tr_max_lead = 2.0 * args.max_delta
                _post_tracking(
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                    kp=_kp, kd=_kd,
                    max_vel=_tr_max_vel,
                    max_lead=_tr_max_lead,
                    arm_limits=arm_limits,
                    vel_ff=True,
                )

            elif teleop_state == State.HOLD:
                # held_target is frozen — no benefit from running the rate
                # limit at 100 Hz, so this stays a static prebuilt bundle.
                arm_payload: Optional[dict] = None
                if target is not None:
                    if latest_q_cmd is not None:
                        ref = latest_q_cmd.copy()
                        if q_actual is not None:
                            _max_lead = 2.0 * args.max_delta
                            ref = q_actual + np.clip(ref - q_actual,
                                                     -_max_lead, _max_lead)
                    elif q_actual is not None:
                        ref = q_actual
                    else:
                        ref = target
                    delta = np.clip(target - ref, -args.max_delta, args.max_delta)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                    latest_q_cmd = q_cmd.copy()
                    arm_payload = {
                        "positions": q_cmd.tolist(),
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": _kp, "kd": _kd,
                        "torques": [0.0] * NUM_JOINTS,
                    }
                _post_bundle(
                    arm=arm_payload,
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                )

            _prof.tick("motor")

            # -----------------------------------------------------------------
            # Telemetry — pulled from background-thread cache; only consume
            # the message when its timestamp advances so we don't re-parse
            # the same payload tick after tick.
            # -----------------------------------------------------------------
            with _telem_lock:
                _t_msg  = _telem_cache["msg"]
                _t_time = _telem_cache["time"]
            telem = _t_msg if _t_time > _telem_last_time else None
            if telem is not None:
                _telem_last_time = _t_time
            q_new = _qpos(telem)
            if q_new is not None:
                q_actual        = q_new
                robot_ok        = True
                last_telem_time = t0
            # Mirror to WebXR every tick — NOT gated on fresh Jetson telem.
            # Without this, the browser never loads the URDF mirror when no
            # arm is connected (q_new stays None), even though the operator
            # can still drive QuestLeader and see useful HUD state.
            if _quest_state is not None:
                # Use the real arm pose if we have it; otherwise the leader's
                # last commanded q; otherwise a zeros vector so the URDF
                # mirror still loads at home pose.
                _q_for_telem = q_actual
                _hud_ldr = _leader_box.get("leader")
                if _q_for_telem is None and _hud_ldr is not None:
                    _hud_qcmd = getattr(_hud_ldr, "_q_last", None)
                    if _hud_qcmd is not None:
                        _q_for_telem = np.asarray(_hud_qcmd, dtype=np.float32)
                if _q_for_telem is None:
                    _q_for_telem = np.zeros(NUM_JOINTS, dtype=np.float32)
                _telem_out = {
                    "ts":   t0,
                    "qpos": [float(x) for x in _q_for_telem],   # URDF control frame
                    # Per-joint visual offsets [rad].  Added BY THE BROWSER
                    # when rendering the URDF mesh — collect_demo keeps them
                    # OUT of the control loop on purpose (see _ALIGN_OFFSETS
                    # comment).  Forwarded here so hot-reload from /preview
                    # propagates to the VR mirror without a page reload.
                    "align_offsets": [float(x) for x in alignment._ALIGN_OFFSETS],
                    # Camera freshness so the browser can hide the cam panel
                    # (avoiding a big black square) when no frames are flowing.
                    "cam_age": float(cam_age),
                }
                # Raw motor-frame snapshot for the /preview calibration UI.
                # Absent when we don't have arm telem (sim mode etc.); the
                # client falls back to displaying qpos directly in that case.
                _q_motor_now = _qpos_motor(telem) if telem is not None else None
                if _q_motor_now is not None:
                    _telem_out["qpos_motor"] = [float(x) for x in _q_motor_now]
                # Surface leader hud_snapshot (engaged / estop / workspace /
                # qcmd / fk_ee_actual) so the in-headset overlay can render
                # the workspace box, status pills, and the ghost URDF.
                _hud_get = getattr(_hud_ldr, "hud_snapshot", None)
                if _hud_get is not None:
                    try:
                        _hud = _hud_get()
                        # FK marker is computed from the COMMANDED q (what
                        # the main mirror shows) so the blue validation dot
                        # sits on the visible arm, not the static real one.
                        # Add _ALIGN_OFFSETS so the FK matches the visual
                        # mesh (which renders qcmd + offsets in scene.js).
                        _kin = getattr(_hud_ldr, "_kin", None)
                        _cmd_q = _hud.get("qcmd")
                        if _kin is not None and _cmd_q is not None:
                            try:
                                _q_vis = (np.asarray(_cmd_q[:6], dtype=np.float64)
                                          + alignment._ALIGN_OFFSETS[:6].astype(np.float64))
                                _fk_pos, _ = _kin.fk_pose(_q_vis)
                                _hud["fk_ee_actual"] = _fk_pos.tolist()
                            except Exception:
                                pass
                        _telem_out["leader"] = _hud
                        if "qcmd" in _hud:
                            _telem_out["qcmd"] = _hud["qcmd"]
                    except Exception:
                        pass
                _quest_state.latest_telem = _telem_out
                _quest_state.latest_telem_seq += 1
            if telem and "motors" in telem:
                # Swivel is the first joint of ARM_JOINTS, so torque/temp
                # arrays already include it at index 0 — no separate extract.
                tq = _qtorque(telem)
                if tq is not None:
                    arm_torques = tq

                    # Force-feedback to OpenRB leader (gripper trigger only).
                    # Reflects follower gripper torque so the operator feels
                    # objects in the gripper / back-drive when the gripper
                    # hits a surface.  All other servos stay torque-OFF
                    # (SENTINEL) — they're free to backdrive as before.
                    if (args.ff_leader
                            and leader is not None
                            and hasattr(leader, "set_ff_currents")
                            and not estop_hw_active):
                        raw_t = float(tq[6])  # gripper is ARM_JOINTS[6]
                        if not math.isnan(raw_t):
                            _ff_gripper_lpf = (
                                _FF_LPF_ALPHA * raw_t
                                + (1.0 - _FF_LPF_ALPHA) * _ff_gripper_lpf
                            )
                            # Soft-knee deadband: anything below the floor
                            # gets zeroed out; above it the response is
                            # linear from zero (no step at the boundary).
                            eff = _ff_gripper_lpf
                            db  = float(args.ff_gripper_deadband)
                            if abs(eff) <= db:
                                eff = 0.0
                            else:
                                eff = eff - math.copysign(db, eff)
                            cur_ma = int(round(args.ff_gripper_gain * eff))
                            cur_ma = max(-FF_MAX_CURRENT_MA,
                                          min( FF_MAX_CURRENT_MA, cur_ma))
                            leader.set_ff_currents(
                                [FF_DISABLE_SENTINEL] * 6 + [cur_ma]
                            )
                            _ff_was_active = True
                    elif _ff_was_active:
                        # We were sending currents and now the conditions
                        # no longer apply (FF flag toggled, leader gone,
                        # e-stop fired).  Send one DISABLE_ALL frame so
                        # the trigger goes free immediately instead of
                        # waiting the firmware's 1 s watchdog timeout.
                        if leader is not None and hasattr(leader, "set_ff_currents"):
                            try:
                                leader.set_ff_currents([FF_DISABLE_SENTINEL] * 7)
                            except Exception:
                                pass
                        _ff_gripper_lpf = 0.0
                        _ff_was_active  = False
                te = _qtemp(telem)
                if te is not None:
                    arm_temps = te
                _arm_st = [
                    str(telem["motors"].get(j, {}).get("state", "?"))
                    for j in ARM_JOINTS
                ]
                if any(s != "?" for s in _arm_st):
                    arm_states = _arm_st
                # Wheel motor telemetry
                _ws: dict = {}
                for wn in _BASE_MOTORS:
                    wm = telem["motors"].get(wn)
                    if wm is not None:
                        _ws[wn] = {
                            "state":       wm.get("state", "?"),
                            "velocity":    wm.get("velocity"),
                            "torque":      wm.get("torque"),
                            "temperature": wm.get("temperature"),
                        }
                if _ws:
                    wheel_states = _ws
            if telem:
                ts = telem.get("timestamp")
                if ts is not None:
                    latest_telem_ts = float(ts)
                bv = telem.get("battery_voltage")
                if bv is not None:
                    battery_voltage = float(bv)
                estop_from_telem = bool(telem.get("emergency_stop", False))
            else:
                estop_from_telem = False
            estop_active = estop_from_telem or estop_hw_active

            with _cam_lock:
                _ups_msg = _cam_cache["ups"]
            if _ups_msg is not None and "ups" in _ups_msg:
                ups_data = _ups_msg["ups"]

            _prof.tick("telem")

            # Queue joint positions + leader commands to Rerun (every frame).
            # ARM_JOINTS includes swivel as joint 0 — q_actual / aizee_cmd
            # are both 7-element so a single loop covers everything.
            if _rr_event is not None:
                _jd: Optional[dict] = None
                _ld: Optional[dict] = None
                if q_actual is not None:
                    _jd = {jn: float(q_actual[i]) for i, jn in enumerate(ARM_JOINTS)}
                if aizee_cmd is not None:
                    _ld = {jn: float(aizee_cmd[i]) for i, jn in enumerate(ARM_JOINTS)}
                if _jd or _ld:
                    with _rr_lock:
                        _rr_holder["time"]   = t0
                        _rr_holder["joints"] = _jd
                        _rr_holder["leader"] = _ld
                    _rr_event.set()

            # -----------------------------------------------------------------
            # E-Stop detection
            # -----------------------------------------------------------------
            if telem and telem.get("emergency_stop"):
                if teleop_state != State.ESTOP:
                    _finalize_recording(" (e-stop)", t0)
                    teleop_state = State.ESTOP
            elif teleop_state == State.ESTOP:
                # E-stop cleared — return to READY, user must re-enable
                teleop_state = State.READY

            # -----------------------------------------------------------------
            # Recording (sub-sampled to REC_HZ)
            # -----------------------------------------------------------------
            if recording and t0 - last_rec_time >= 1.0 / REC_HZ:
                last_rec_time = t0
                # Pull image + its publisher capture-time atomically from
                # the decoder cache. Using the paired ts (not
                # latest_*_ts) is what keeps the recorded HDF5 actually
                # synchronized — latest_*_ts could be from a frame that
                # arrived AFTER the decoded image was committed,
                # producing an off-by-one ts in the file.
                with _dec_lock:
                    gripper_img    = _dec_cache["gripper"]
                    gripper_ts_rec = _dec_cache.get("gripper_ts_pub")
                    scene_img      = _dec_cache.get("scene")
                    scene_ts_rec   = _dec_cache.get("scene_ts_pub")
                cams_ok   = cam_age < _CAM_STALE
                # When scene cam is enabled it must also be fresh — a stale
                # scene frame would desync from the gripper / qpos timeline.
                # When disabled, scene_img is None and we skip the scene
                # append entirely (episode stays format_version=4).
                scene_ok  = (not _scene_cam_seen) or (
                    scene_img is not None and scene_age < _CAM_STALE)
                # In TRACKING the cmd thread owns the live commanded pose;
                # pull it from the holder so the recording captures the
                # exact value sent on the wire.  Falls back to the main
                # loop's `latest_q_cmd` (still authoritative for HOLD/
                # ENGAGING/SHUTDOWN) when the cmd thread hasn't run yet.
                with _cmd_lock:
                    _qcmd_live = _cmd_holder.get("last_q_cmd")
                if _qcmd_live is not None:
                    rec_q_cmd = np.asarray(_qcmd_live, dtype=np.float32).copy()
                elif latest_q_cmd is not None:
                    rec_q_cmd = latest_q_cmd.copy()
                else:
                    rec_q_cmd = None

                if (q_actual is not None and gripper_img is not None and cams_ok
                        and scene_ok):
                    # qpos_buf/qcmd_buf/torque_buf are 7-DOF (swivel-first)
                    # because q_actual / latest_q_cmd / arm_torques are.
                    qpos_buf.append(q_actual.copy())
                    qcmd_buf.append(rec_q_cmd if rec_q_cmd is not None else q_actual.copy())
                    torque_buf.append(arm_torques.copy() if arm_torques is not None else np.zeros(NUM_JOINTS, dtype=np.float32))
                    gripper_buf.append(gripper_img)
                    telem_ts_buf.append(latest_telem_ts if latest_telem_ts is not None else _nan)
                    gripper_ts_buf.append(
                        gripper_ts_rec if gripper_ts_rec is not None else _nan)
                    if _scene_cam_seen and scene_img is not None:
                        scene_buf.append(scene_img)
                        scene_ts_buf.append(
                            scene_ts_rec if scene_ts_rec is not None else _nan)
                else:
                    dropped_frames += 1

                if len(qpos_buf) >= args.max_steps:
                    _finalize_recording(" (max steps)", t0)

            # -----------------------------------------------------------------
            # Status strings
            # -----------------------------------------------------------------
            if teleop_state == State.READY:
                status = "[ ] ready — motors off"
                if leader is not None:
                    hint = "E=track · I=idle · Z=zero · M=mirror · K=mech-zero · Q=quit"
                else:
                    hint = "E=hold · I=idle · K=mech-zero · Q=quit"

            elif teleop_state == State.IDLE:
                status = "[I] idle — zero torque (arm free)"
                if leader is not None:
                    hint = "E=track · H=hold · R=record · X=shutdown · Q=quit"
                else:
                    hint = "H=hold · R=record · X=shutdown · Q=quit"

            elif teleop_state == State.ENGAGING:
                gap_str = ""
                if q_actual is not None and target is not None:
                    _g = float(np.max(np.abs(target - q_actual)))
                    gap_str = f"  (gap {_g:.2f} rad)"
                status = f"[~] engaging — slow ramp to leader{gap_str}"
                hint   = "X=shutdown · Q=quit"

            elif teleop_state == State.TRACKING:
                if leader_rad is None or leader_age > 0.5:
                    status = "[!] tracking — NO LEADER DATA"
                else:
                    status = "[*] tracking leader"
                hint = "H=hold · R=record · X=shutdown · Z=zero · M=mirror · Q=quit"

            elif teleop_state == State.HOLD:
                status = "[H] HOLD — target frozen"
                if leader is not None:
                    hint = "H=resume tracking · R=record · X=shutdown · Q=quit"
                else:
                    hint = "H=resume · R=record · X=shutdown · Q=quit"

            elif teleop_state == State.SHUTDOWN:
                status = (f"[X] shutdown  hold {shutdown_countdown:.1f}s"
                          if shutdown_countdown > 0 else "[X] returning to zero")
                hint   = "B=cancel (gamepad) · Q=quit"

            elif teleop_state == State.ESTOP:
                status = f"{_BG_RED} !! EMERGENCY STOP !! {_RST}"
                hint   = "release e-stop to clear · Q=quit"

            # Live replay overrides teleop status (shown while live mode active)
            if live_replay.live:
                _rs, _rh = live_replay.status_line()
                if _rs:
                    status = _rs
                if _rh:
                    hint = _rh

            # Flash messages override
            if t0 < zero_msg_until:
                status = zero_msg
            if t0 < save_msg_until:
                hint = save_msg

            # -----------------------------------------------------------------
            # Render — queue raw values to display thread (render + draw
            # both run off the main loop to avoid GIL contention)
            # -----------------------------------------------------------------
            # Display arrays match q_actual / target / arm_torques shape
            # directly — swivel is element 0 of each, no concat needed.
            _da = shutdown_target if teleop_state == State.SHUTDOWN else target

            # Mapped leader in the robot frame (Z/M-corrected); 7-DOF and
            # already includes swivel as element 0.  None when tracking is
            # inactive or no leader sample has arrived.
            leader_mapped = aizee_cmd if aizee_cmd is not None else None

            _disp_snapshot = dict(
                leader_rad=leader_rad,
                leader_mapped=leader_mapped,
                target=_da,
                actual=q_actual,
                status=status, hint=hint,
                robot_ok=robot_ok,
                telem_age=(t0 - last_telem_time if robot_ok else 999.0),
                ups_data=ups_data,
                clamped=(_clamped_live if leader_rad is not None else None),
                torque=arm_torques,
                temp=arm_temps,
                motor_states=list(arm_states),
                battery_voltage=battery_voltage,
                leader_connected=(leader is not None),
                leader_age=leader_age,
                cam_age=cam_age,
                rec_steps=len(qpos_buf),
                recording=recording,
                dropped=dropped_frames,
                estop_active=estop_active,
                wheel_states=wheel_states,
                wheels_enabled=teleop_state in (
                    State.IDLE, State.TRACKING, State.HOLD,
                    State.ENGAGING, State.SHUTDOWN),
                drive_linear=drive_linear * _max_linear,
                drive_angular=drive_angular * _max_angular,
                # M5 Joystick2 snapshot for the GUI's diagnostic panel.
                # `m5` was already read above under _lr_lock; passing it
                # through unchanged so the GUI sees the same view as the
                # drive / record-toggle logic.  None when no leader
                # installed yet (so the panel shows "NO LEADER").
                joy=(m5 if leader is not None else None),
                state=teleop_state.value,
                # Quest VR state for the GUI's leader panel.
                quest_available=_quest_available,
                quest_kind=_leader_kind,                       # "quest" | "openrb" | "so101" | None
                quest_server_running=_quest_server_started,
                quest_port=args.quest_port,
                quest_bind=args.quest_bind,
                save_msg=(save_msg if t0 < save_msg_until else None),
                action_msg=(zero_msg if t0 < zero_msg_until else None),
                last_saved_path=last_saved_path,
                task_tag=_meta["task_tag"],
                **live_replay.snapshot_fields(),
            )
            # Lock held only for reference swap (~µs)
            with _disp_lock:
                _disp_holder["args"] = _disp_snapshot
            if _disp_event is not None:
                _disp_event.set()

            _prof.tick("display")
            _prof.end()

            frame_counter += 1
            # Pick up edits to joint_align.json (made via /preview Save)
            # roughly once per second so calibration loops without a
            # collect_demo restart.  Stat-only; cheap enough at 1 Hz.
            if (frame_counter % LOOP_HZ) == 0:
                if _maybe_reload_joint_align() and leader is not None:
                    _push_visual_offsets_to_leader(leader)
            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        _hp_stop.set()
        if _hp_thread is not None:
            _hp_thread.join(timeout=2.0)
        _lr_stop.set()
        _lr_thread.join(timeout=1.0)
        if leader is not None:
            leader.close()
        # Stop the cmd-sender thread first so it can't re-emit a bundle
        # AFTER our explicit drive-zero + disable below.
        _clear_bundle()
        _cmd_tx_stop.set()
        _cmd_tx_thread.join(timeout=1.0)
        # Disable all motors before closing (prevents motors staying enabled after quit)
        _send(cmd_sock, {"type": "drive", "linear": 0.0, "angular": 0.0,
                         "kp": 0.0, "kd": 3.0})
        _send(cmd_sock, {"type": "disable", "motor_ids": _ALL_MOTORS})
        time.sleep(0.1)  # let ZMQ flush the disable command
        cmd_sock.close()
        _telem_stop.set()
        _telem_thread.join(timeout=1.0)
        _rec_flag.clear()
        _dec_stop.set()
        _dec_thread.join(timeout=1.0)
        _cam_stop.set()
        _cam_thread.join(timeout=2.0)
        _estop_stop.set()
        if _estop_thread is not None:
            _estop_thread.join(timeout=1.0)
        if _qt_renderer is not None:
            _qt_renderer.request_quit()
            _qt_renderer.join(timeout=2.0)
        else:
            _disp_stop.set()
            _disp_thread.join(timeout=1.0)
        if _rr_stop is not None:
            _rr_stop.set()
            _rr_thread.join(timeout=1.0)
        ctx.term()
        print("\nDone.")


if __name__ == "__main__":
    main()
