"""Fake-Quest dev client — drives /ws/control with synthetic poses.

Lets you exercise the full QuestLeader pipeline (clutch, IK, workspace
clamp, e-stop) without a real headset.  Useful for:
  * Bring-up on a developer laptop with no robot connected
  * CI smoke test (just check IK output stays inside bounds)
  * Reproducing operator-reported bugs without dragging the Quest out

Usage:
    # Drive a small circle in front of the robot, clutch held throughout.
    python -m web.fake_quest --url wss://127.0.0.1:8443/ws/control --mode circle

    # Send still pose + clutched/unclutched grip transitions.
    python -m web.fake_quest --mode toggle

Frames are JSON, matching what `web/static/main.js` sends.  Quaternions are
identity by default (no rotation); orientation IK is exercised separately
by the test_quest_leader.py module test.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import ssl
import time

import aiohttp


def _frame(ts: float, pos, *, grip: bool, trigger: float = 0.0,
           b: bool = False, left_grip: bool = False,
           left_stick=(0.0, 0.0), left_a: bool = False) -> dict:
    return {
        "ts": ts,
        "head":  {"pos": [0.0, 1.6, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]},
        "right": {
            "pos": list(pos),
            "quat": [0.0, 0.0, 0.0, 1.0],
            "trigger": float(trigger),
            "grip": bool(grip),
            "a": False,
            "b": bool(b),
        },
        "left": {
            "pos": [-0.3, 1.4, -0.4],
            "quat": [0.0, 0.0, 0.0, 1.0],
            "stick": list(left_stick),
            "a": bool(left_a),
            "grip": bool(left_grip),
        },
    }


async def _drive(url: str, mode: str, duration: float, insecure: bool) -> None:
    ssl_ctx = None
    if url.startswith("wss://"):
        ssl_ctx = ssl.create_default_context()
        if insecure:
            ssl_ctx.check_hostname = False
            ssl_ctx.verify_mode = ssl.CERT_NONE
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(url, ssl=ssl_ctx, heartbeat=2.0) as ws:
            print(f"[fake-quest] connected to {url}; mode={mode}", flush=True)
            t0 = time.time()
            engage_pos = (0.10, 1.40, -0.55)
            n = 0
            period = 1.0 / 90.0
            next_tick = t0 + period
            while True:
                t = time.time() - t0
                if t > duration:
                    break
                if mode == "circle":
                    # 10 cm circle in XY plane (WebXR: X right, Y up).  Clutch
                    # held throughout so the IK keeps mapping deltas to motion.
                    r = 0.10
                    pos = (
                        engage_pos[0] + r * math.cos(2 * math.pi * 0.25 * t),
                        engage_pos[1] + r * math.sin(2 * math.pi * 0.25 * t),
                        engage_pos[2],
                    )
                    frame = _frame(time.time(), pos, grip=True, trigger=0.0)
                elif mode == "toggle":
                    # Alternate clutch every 2 s while the controller drifts forward.
                    grip = (int(t // 2.0) % 2 == 0)
                    pos = (engage_pos[0], engage_pos[1], engage_pos[2] - 0.05 * (t / duration))
                    frame = _frame(time.time(), pos, grip=grip,
                                   trigger=0.5 * (1 + math.sin(t)) / 2)
                elif mode == "estop":
                    # Press B once at t=2s; then dual-grip-hold from t=4..6s to clear.
                    pos = engage_pos
                    b = (1.95 < t < 2.05)
                    grip_r = (4.0 < t < 6.0)
                    grip_l = (4.0 < t < 6.0)
                    frame = _frame(time.time(), pos, grip=grip_r, b=b, left_grip=grip_l)
                else:
                    pos = engage_pos
                    frame = _frame(time.time(), pos, grip=False)
                await ws.send_json(frame)
                n += 1
                if n % 90 == 0:
                    print(f"[fake-quest] t={t:.1f}s  sent {n} frames", flush=True)
                # Deadline-based pacer.  asyncio.sleep on Windows can return
                # immediately if the requested delay is below the timer
                # resolution; we re-target each tick from the *absolute*
                # next deadline so we don't accumulate drift either way.
                now = time.time()
                if next_tick > now:
                    await asyncio.sleep(next_tick - now)
                next_tick += period
                if next_tick < now:
                    # Far behind — reset to "now" rather than spin trying to catch up.
                    next_tick = now + period
            print(f"[fake-quest] done; sent {n} frames over {duration:.1f}s", flush=True)


def _cli() -> None:
    ap = argparse.ArgumentParser(description="Synthetic Quest pose driver")
    ap.add_argument("--url", default="wss://127.0.0.1:8443/ws/control")
    ap.add_argument("--mode", choices=("circle", "toggle", "estop", "idle"),
                    default="circle")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--insecure", action="store_true", default=True,
                    help="Skip TLS hostname/cert verification (for self-signed)")
    args = ap.parse_args()
    asyncio.run(_drive(args.url, args.mode, args.duration, args.insecure))


if __name__ == "__main__":
    _cli()
