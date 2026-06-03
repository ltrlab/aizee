"""Visual IK simulator — exercise the full WebXR + QuestLeader pipeline
without the Jetson, the arm, or the leader-arm USB stick.

Closes the control loop locally:
  controller pose  --[/ws/control]-->  QuestLeader.poll()  -->  q_cmd
       ▲                                                          │
       │                                                          ▼
   [Quest browser]  <--[/ws/telem JSON]--  SharedState.latest_telem
                                              { qpos = q_cmd,
                                                leader = hud snapshot }

Useful before hardware bring-up: put on the Quest, hold the right grip,
move your hand, and watch the URDF mirror's arm follow.  Workspace box,
e-stop pill, IK target marker all live; the camera panel renders a small
synthetic test pattern so the operator can confirm the WebRTC track is
healthy.

Run with:
    python -m web.sim_visual_ik --port 8443
"""

from __future__ import annotations

import argparse
import gc
import io
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Allow `python -m web.sim_visual_ik` from inside python/.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "teleop"))

from quest_leader import (
    QuestLeader,
    QuestLeaderConfig,
    make_quest_leader_class,
)
from web.server import SharedState, start_server_in_thread

# Separate Kinematics instance for telemetry-side FK validation.  Same
# URDF as the QuestLeader's solver — if my FK is correct the marker we
# emit overlaps the URDF-rendered EE link in the browser scene.
from ik import load_aizee_arm as _load_aizee_arm


# -----------------------------------------------------------------------------
# Synthetic camera — keeps the cam panel non-blank in sim mode so the
# operator can confirm the WebRTC track is alive.
# -----------------------------------------------------------------------------

_FONT: Optional[ImageFont.FreeTypeFont] = None


def _font() -> ImageFont.FreeTypeFont:
    global _FONT
    if _FONT is None:
        try:
            _FONT = ImageFont.truetype("arial.ttf", 28)
        except Exception:
            _FONT = ImageFont.load_default()
    return _FONT


def _synth_cam_jpeg(t: float, qpos: np.ndarray, engaged: bool) -> bytes:
    w, h = 1024, 768
    img = Image.new("RGB", (w, h), (8, 12, 18))
    d = ImageDraw.Draw(img)
    # Diagonal gradient corner so motion is visible at a glance.
    band_x = int((t * 40) % w)
    d.rectangle([(band_x, 0), (band_x + 60, h)], fill=(40, 48, 64))
    # Crosshair at centre (where the gripper would point)
    d.line([(w // 2 - 40, h // 2), (w // 2 + 40, h // 2)], fill=(180, 200, 220), width=2)
    d.line([(w // 2, h // 2 - 40), (w // 2, h // 2 + 40)], fill=(180, 200, 220), width=2)
    # Header
    d.rectangle([(0, 0), (w, 56)], fill=(22, 27, 34))
    d.text((20, 12), "AIZEE  —  SIM MODE  —  no robot connected",
           fill=(230, 237, 243), font=_font())
    # Sticky pill if engaged
    if engaged:
        d.rectangle([(w - 180, 8), (w - 12, 48)], fill=(46, 160, 67))
        d.text((w - 168, 14), "CLUTCH", fill=(12, 15, 20), font=_font())
    # Joint angles strip at bottom
    d.rectangle([(0, h - 56), (w, h)], fill=(22, 27, 34))
    qstr = "  ".join(f"{v:+5.2f}" for v in qpos[:7])
    d.text((20, h - 46), f"qpos  {qstr}", fill=(180, 200, 220), font=_font())
    buf = io.BytesIO()
    img.save(buf, "JPEG", quality=80)
    return buf.getvalue()


# -----------------------------------------------------------------------------
# Sim loop
# -----------------------------------------------------------------------------

def run(bind: str, port: int, sim_hz: float, cam_hz: float) -> int:
    # Raise the gen-0/gen-1 GC thresholds so the small-object churn from
    # numpy + JSON pack/unpack doesn't trigger gen-2 collections every
    # ~17 s (default).  Major GC during a control tick causes a visible
    # browser-side stutter even though the IK itself is unaffected.
    # Numbers are deliberately generous — we run a single short-lived
    # process per session, so a slightly larger heap is fine.
    gc.set_threshold(50000, 50, 50)
    print(f"[sim] gc thresholds set to {gc.get_threshold()}")
    print("[sim] building shared state + QuestLeader")
    state = SharedState()
    cfg_path = Path(__file__).resolve().parents[2] / "config" / "quest_teleop.yaml"
    if cfg_path.exists():
        cfg = QuestLeaderConfig.load_yaml(cfg_path)
        print(f"[sim] loaded config from {cfg_path}")
    else:
        cfg = QuestLeaderConfig()
        print(f"[sim] no config at {cfg_path} — using dataclass defaults")
    LeaderCls = make_quest_leader_class(state, cfg)
    leader = LeaderCls()
    _fk = _load_aizee_arm()
    if not leader.connect():
        print("[sim] leader.connect() failed", flush=True)
        return 1

    print(f"[sim] starting WebXR server on https://{bind}:{port}")
    start_server_in_thread(state, bind=bind, port=port)
    # Give the server a moment to bind + print its URLs before the sim
    # banner so the operator's stdout reads top-to-bottom.
    time.sleep(1.0)

    # Kinematic-only "robot": q_actual just copies q_cmd from the leader.
    # The leader needs latest_telem before it'll engage (it warm-starts IK
    # from current qpos), so we seed q with a comfortable home posture.
    q: np.ndarray = np.zeros(7, dtype=np.float32)

    def _cam_pump() -> None:
        period = 1.0 / max(cam_hz, 1.0)
        t0 = time.time()
        while True:
            engaged = False
            try:
                engaged = leader.hud_snapshot().get("engaged", False)
            except Exception:
                pass
            state.latest_cam_jpeg = _synth_cam_jpeg(time.time() - t0, q, engaged)
            state.latest_cam_seq += 1
            time.sleep(period)

    threading.Thread(target=_cam_pump, name="sim-cam", daemon=True).start()

    print(f"[sim] running at {sim_hz:.0f} Hz; ctrl-c to exit")
    period = 1.0 / sim_hz
    next_tick = time.time() + period
    while True:
        try:
            # Publish current qpos first so QuestLeader.poll() (which reads
            # latest_telem) has the right warm-start on its very first call.
            telem_ts = time.time()
            hud = leader.hud_snapshot()
            # Compute the Python FK on the actual qpos so the browser can
            # render a marker at it and visually compare to the URDF's
            # natural EE link world position.  If the two diverge, the
            # Python kinematics chain disagrees with the URDF — either a
            # joint axis mismatch, wrong origin, or wrong EE link.
            try:
                _fk_pos, _ = _fk.fk_pose(q[:6].astype(float))
                hud["fk_ee_actual"] = _fk_pos.tolist()
            except Exception:
                pass
            state.latest_telem = {
                "ts":     telem_ts,
                "qpos":   q.astype(float).tolist(),
                # Surface the commanded q at the top level too so the
                # browser ghost-URDF mirror doesn't have to dig into `leader`.
                "qcmd":   hud.get("qcmd"),
                # Synthetic camera is always fresh — keeps the cam panel
                # visible in sim (collect_demo sends the real cam_age).
                "cam_age": 0.0,
                "leader": hud,
            }
            state.latest_telem_seq += 1
            # Step the leader; in clutched-engaged mode this runs the IK
            # against the current qpos snapshot we just published.
            q_new = leader.poll()
            if q_new is not None:
                # Kinematic echo — no dynamics, no settling.  Good enough
                # for visual IK confirmation; real arm has KD/KP damping.
                q = q_new.astype(np.float32)
            now = time.time()
            if next_tick > now:
                time.sleep(next_tick - now)
            next_tick += period
            if next_tick < now - 0.5:
                next_tick = now + period
        except KeyboardInterrupt:
            print("[sim] stopping")
            return 0


def _cli() -> None:
    ap = argparse.ArgumentParser(description="AIZEE visual IK sim — no hardware required")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--hz",   type=float, default=60.0, dest="sim_hz",
                    help="Closed-loop tick rate [Hz] (default 60)")
    ap.add_argument("--cam-hz", type=float, default=15.0, dest="cam_hz",
                    help="Synthetic camera frame rate [Hz] (default 15)")
    args = ap.parse_args()
    sys.exit(run(args.bind, args.port, args.sim_hz, args.cam_hz))


if __name__ == "__main__":
    _cli()
