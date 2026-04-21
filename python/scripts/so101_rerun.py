#!/usr/bin/env python3
"""so101_rerun.py — Rerun visualization companion to so101_teleop.py.

Subscribes to:
  Teleop state publisher  (so101_teleop.py  --teleop-pub tcp://*:5570)
  AIZEE motor telemetry   (Jetson           :5556)
  UPS power monitor       (Jetson           :5562)
  Gripper arm cameras     (Jetson relay     :5563 left  :5564 right)

Layout (three columns):
  | Gripper cameras L/R |  Joint positions · Torques · Temps · Battery  | Status + Controls |

Run alongside so101_teleop.py:
    # Terminal 1 — teleop
    python python/scripts/so101_teleop.py --port COM4

    # Terminal 2 — rerun viewer
    python python/scripts/so101_rerun.py

    # Save an MCAP recording at the same time:
    python python/scripts/so101_rerun.py --save logs/teleop_session.mcap

    # Custom host / endpoints:
    python python/scripts/so101_rerun.py --host 192.168.0.27
"""

from __future__ import annotations

import argparse
import base64
import json
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import zmq


# ---------------------------------------------------------------------------
# Joint ordering — must match _LEADER_JOINTS in so101_teleop.py
# ---------------------------------------------------------------------------
_JOINTS = [
    "swivel",
    "gantry_base",
    "gantry_mid",
    "gantry_end",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
]

# Per-series colors for the joint position graph
# Leader = cyan-blue, Target = amber, Actual = green
_COL_LEADER = [80,  180, 255]
_COL_TARGET = [255, 200, 60]
_COL_ACTUAL = [60,  220, 100]

# Torque saturation thresholds (Nm) — same as so101_teleop.py
_SAT_TORQUE = {
    "swivel":      6.0,
    "gantry_base": 12.0,
    "gantry_mid":  6.0,
    "gantry_end":  4.0,
    "wrist_pitch": 4.0,
    "wrist_roll":  2.0,
    "gripper":     2.0,
}
_TEMP_WARN = 65.0   # °C
_TEMP_CRIT = 80.0   # °C
_VBUS_WARN = 20.0   # V (actuator supply)
_VBUS_CRIT = 18.0   # V
_UPS_WARN  = 10.8   # V (Jetson UPS)
_UPS_CRIT  = 10.0   # V

_CONTROLS_MD = """\
## SO-101 Teleop Controls

| Key | Gamepad | Action |
|-----|---------|--------|
| **E** | A button | Enable motors · begin alignment |
| **I** | — | Idle — enable with zero torque |
| **H** | Start | Hold — freeze target at current actual |
| **X** | B button | Soft shutdown (hold 1 s → return to zero) |
| **Q** | Back | Quit |
| **Z** | — | Zero SO-101 (capture current as zero) |
| **M** | — | Mirror — map SO-101 pose to AIZEE actual |

**State machine**

```
ready  ──E──▶  aligning  ──auto──▶  tracking
  ▲               │  ◀──H──  hold  ──H──▶  aligning
  │               └──X──▶  shutdown  ──B──▶  hold
  └────────────────────────────done──────────┘
```

**Gamepad B during shutdown** — cancels back to hold.
"""


# ---------------------------------------------------------------------------
# Blueprint
# ---------------------------------------------------------------------------

def build_blueprint() -> rrb.Blueprint:
    # --- Left column: gripper camera pair ---
    cam_col = rrb.Vertical(
        rrb.Spatial2DView(name="Gripper Left",  origin="cameras/arm_cam_left"),
        rrb.Spatial2DView(name="Gripper Right", origin="cameras/arm_cam_right"),
        row_shares=[1, 1],
    )

    # --- Centre column: time-series data ---
    series_col = rrb.Vertical(
        rrb.TimeSeriesView(
            name="Joint Positions — leader · target · actual  (rad)",
            contents=["teleop/joints/**"],
        ),
        rrb.TimeSeriesView(
            name="Torque  (Nm)",
            contents=["motors/*/torque"],
        ),
        rrb.TimeSeriesView(
            name="Temperature  (°C)",
            contents=["motors/*/temperature"],
        ),
        rrb.Horizontal(
            rrb.TimeSeriesView(
                name="Jetson UPS  (V)",
                contents=["power/ups/voltage", "power/ups/percentage"],
            ),
            rrb.TimeSeriesView(
                name="Actuator VBUS  (V)",
                contents=["power/actuator/voltage"],
            ),
            column_shares=[1, 1],
        ),
        row_shares=[4, 2, 2, 1],
    )

    # --- Right column: status tables + controls legend ---
    info_col = rrb.Vertical(
        rrb.TextDocumentView(name="Motor Status",    origin="status/motors"),
        rrb.TextDocumentView(name="Teleop State",    origin="status/teleop"),
        rrb.TextDocumentView(name="Controls Legend", origin="status/controls"),
        row_shares=[3, 1, 2],
    )

    return rrb.Blueprint(
        rrb.Horizontal(
            cam_col,
            series_col,
            info_col,
            column_shares=[2, 5, 2],
        )
    )


# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def _drain_latest(sock: zmq.Socket) -> Optional[dict]:
    """Drain all queued messages; return the most recent, or None."""
    latest = None
    while True:
        try:
            latest = json.loads(sock.recv_string(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def _sub(ctx: zmq.Context, endpoint: str) -> zmq.Socket:
    s = ctx.socket(zmq.SUB)
    s.setsockopt(zmq.RCVHWM, 2)
    s.setsockopt(zmq.LINGER, 0)
    s.connect(endpoint)
    s.setsockopt_string(zmq.SUBSCRIBE, "")
    return s


# ---------------------------------------------------------------------------
# Message processors
# ---------------------------------------------------------------------------

def process_teleop(msg: dict) -> None:
    """Log leader / target / actual joint positions + teleop state."""
    ts     = msg.get("timestamp", time.time())
    state  = msg.get("state", "unknown")
    leader = msg.get("leader")   # list[7] | None  (null = missing)
    target = msg.get("target")
    actual = msg.get("actual")
    torque = msg.get("torque")
    temp   = msg.get("temp")

    rr.set_time("time", timestamp=ts)

    for i, joint in enumerate(_JOINTS):
        lv = leader[i] if (leader and i < len(leader) and leader[i] is not None) else None
        tv = target[i] if (target and i < len(target) and target[i] is not None) else None
        av = actual[i] if (actual and i < len(actual) and actual[i] is not None) else None

        if lv is not None:
            rr.log(f"teleop/joints/{joint}/leader", rr.Scalars(lv))
        if tv is not None:
            rr.log(f"teleop/joints/{joint}/target", rr.Scalars(tv))
        if av is not None:
            rr.log(f"teleop/joints/{joint}/actual", rr.Scalars(av))

    # Teleop state indicator
    state_emoji = {
        "ready":    "⬜ ready",
        "idle":     "🟦 idle — zero torque",
        "aligning": "🟡 aligning...",
        "tracking": "🟢 tracking",
        "hold":     "🟠 HOLD",
        "shutdown": "🔴 shutdown",
    }.get(state, f"? {state}")
    rr.log(
        "status/teleop",
        rr.TextDocument(
            f"## Teleop State\n\n**{state_emoji}**",
            media_type=rr.MediaType.MARKDOWN,
        ),
    )


def process_telemetry(msg: dict) -> None:
    """Log per-joint torques, temperatures, battery_voltage from arm telemetry."""
    ts     = msg.get("timestamp", time.time())
    motors = msg.get("motors", {})

    rr.set_time("time", timestamp=ts)

    rows = []
    header  = "| Joint | State | Pos (rad) | Torque (Nm) | Temp (°C) | Error |\n"
    header += "|---|---|---|---|---|---|\n"

    for joint in _JOINTS:
        m = motors.get(joint)
        if m is None:
            rows.append(f"| {joint} | — | — | — | — | — |")
            continue

        state = m.get("state", "—")
        pos   = m.get("position",    float("nan"))
        torq  = m.get("torque",      float("nan"))
        temp  = m.get("temperature", float("nan"))
        err   = m.get("error") or "—"

        # Torque saturation indicator
        sat = abs(torq) / _SAT_TORQUE.get(joint, 999.0)
        sat_flag = " 🔴" if sat >= 0.85 else (" 🟡" if sat >= 0.60 else "")

        # Temperature indicator
        temp_flag = " 🔴" if temp >= _TEMP_CRIT else (" 🟡" if temp >= _TEMP_WARN else "")

        rows.append(
            f"| {joint} | {state} | {pos:+.3f} | {torq:+.2f}{sat_flag} "
            f"| {temp:.1f}{temp_flag} | {err} |"
        )

        if np.isfinite(torq):
            rr.log(f"motors/{joint}/torque",      rr.Scalars(float(torq)))
        if np.isfinite(temp):
            rr.log(f"motors/{joint}/temperature", rr.Scalars(float(temp)))

    rr.log(
        "status/motors",
        rr.TextDocument(
            "## AIZEE Motor Status\n\n" + header + "\n".join(rows),
            media_type=rr.MediaType.MARKDOWN,
        ),
    )

    # VBUS = actuator battery supply voltage from motor telemetry
    bv = msg.get("battery_voltage")
    if bv is not None:
        bv = float(bv)
        bv_flag = " 🔴" if bv < _VBUS_CRIT else (" 🟡" if bv < _VBUS_WARN else "")
        rr.log("power/actuator/voltage", rr.Scalars(bv))


def process_ups(msg: dict) -> None:
    """Log Jetson UPS voltage, current, power, battery %."""
    ups = msg.get("ups", {})
    if not ups:
        return

    rr.set_time("time", timestamp=time.time())

    v   = float(ups.get("voltage",    0.0))
    c   = float(ups.get("current",    0.0))
    p   = float(ups.get("power",      0.0))
    pct = float(ups.get("percentage", 0.0))

    rr.log("power/ups/voltage",    rr.Scalars(v))
    rr.log("power/ups/current",    rr.Scalars(c))
    rr.log("power/ups/power",      rr.Scalars(p))
    rr.log("power/ups/percentage", rr.Scalars(pct))


def process_camera(msg: dict) -> None:
    """Decode and log a gripper camera JPEG frame."""
    camera_id = msg.get("camera_id", "")
    if camera_id not in ("arm_cam_left", "arm_cam_right"):
        return

    rr.set_time("time", timestamp=time.time())

    color_info = msg.get("color")
    if not color_info:
        return

    try:
        raw   = base64.b64decode(color_info["data"])
        bgr   = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rr.log(f"cameras/{camera_id}", rr.Image(rgb))
    except Exception:
        pass   # Silently drop decode errors to avoid log spam


# ---------------------------------------------------------------------------
# Static series style annotations
# ---------------------------------------------------------------------------

def _log_series_styles() -> None:
    """Set per-series line color + display name for the joint position graph."""
    for joint in _JOINTS:
        rr.log(
            f"teleop/joints/{joint}/leader",
            rr.SeriesLine(color=_COL_LEADER, name=f"{joint} leader"),
            static=True,
        )
        rr.log(
            f"teleop/joints/{joint}/target",
            rr.SeriesLine(color=_COL_TARGET, name=f"{joint} target"),
            static=True,
        )
        rr.log(
            f"teleop/joints/{joint}/actual",
            rr.SeriesLine(color=_COL_ACTUAL, name=f"{joint} actual"),
            static=True,
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="Rerun visualization companion for so101_teleop.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--host", default="192.168.0.27",
        help="Jetson host (used to build default endpoints)",
    )
    ap.add_argument(
        "--teleop", default="tcp://localhost:5570",
        help="Teleop state endpoint (so101_teleop.py --teleop-pub)",
    )
    ap.add_argument(
        "--telem", default=None,
        help="Motor telemetry endpoint (default tcp://<host>:5556)",
    )
    ap.add_argument(
        "--ups", default=None,
        help="UPS telemetry endpoint (default tcp://<host>:5562)",
    )
    ap.add_argument(
        "--cam-left", default=None, dest="cam_left",
        help="Left gripper camera endpoint (default tcp://<host>:5563)",
    )
    ap.add_argument(
        "--cam-right", default=None, dest="cam_right",
        help="Right gripper camera endpoint (default tcp://<host>:5564)",
    )
    ap.add_argument(
        "--save", default=None,
        help="Save MCAP recording to this path (e.g. logs/session.mcap)",
    )
    ap.add_argument(
        "--no-spawn", action="store_true",
        help="Don't auto-spawn the Rerun viewer (connect manually)",
    )
    args = ap.parse_args()

    host = args.host
    telem_ep     = args.telem    or f"tcp://{host}:5556"
    ups_ep       = args.ups      or f"tcp://{host}:5562"
    cam_left_ep  = args.cam_left or f"tcp://{host}:5563"
    cam_right_ep = args.cam_right or f"tcp://{host}:5564"

    # --- Rerun init ---
    rr.init("so101_teleop", spawn=not args.no_spawn)
    rr.send_blueprint(build_blueprint())

    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        rr.save(args.save)
        print(f"Recording to: {args.save}")

    # Static series color annotations
    _log_series_styles()

    # Static controls legend (never changes)
    rr.log(
        "status/controls",
        rr.TextDocument(_CONTROLS_MD, media_type=rr.MediaType.MARKDOWN),
        static=True,
    )

    # Placeholder motor status (shown before first telemetry arrives)
    rr.log(
        "status/motors",
        rr.TextDocument("*Waiting for motor telemetry…*", media_type=rr.MediaType.MARKDOWN),
        static=True,
    )
    rr.log(
        "status/teleop",
        rr.TextDocument("*Waiting for teleop state…*", media_type=rr.MediaType.MARKDOWN),
        static=True,
    )

    # --- ZMQ ---
    ctx = zmq.Context()

    endpoints = {
        "teleop":     args.teleop,
        "telem":      telem_ep,
        "ups":        ups_ep,
        "cam_left":   cam_left_ep,
        "cam_right":  cam_right_ep,
    }
    # Both cameras share the same handler
    handlers = {
        "teleop":    process_teleop,
        "telem":     process_telemetry,
        "ups":       process_ups,
        "cam_left":  process_camera,
        "cam_right": process_camera,
    }

    sockets: list[tuple[zmq.Socket, str]] = []
    for name, ep in endpoints.items():
        if not ep:
            continue
        try:
            s = _sub(ctx, ep)
            sockets.append((s, name))
            print(f"  {name:<12} → {ep}")
        except Exception as exc:
            print(f"  {name:<12} → {ep}  [WARN: {exc}]")

    if not sockets:
        print("No endpoints configured — nothing to display.")
        ctx.term()
        return

    poller = zmq.Poller()
    for sock, _ in sockets:
        poller.register(sock, zmq.POLLIN)

    print(f"\nso101_rerun: {len(sockets)} stream(s) connected.  Ctrl-C to quit.")

    running = True

    def _stop(sig, frame):  # noqa: ANN001
        nonlocal running
        running = False

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while running:
            ready = dict(poller.poll(timeout=200))

            for sock, name in sockets:
                if sock not in ready:
                    continue
                msg = _drain_latest(sock)
                if msg is None:
                    continue
                try:
                    handlers[name](msg)
                except Exception as exc:
                    # Don't let a malformed message kill the loop
                    pass

    finally:
        for sock, _ in sockets:
            sock.close()
        ctx.term()
        print("\nso101_rerun: done.")


if __name__ == "__main__":
    main()
