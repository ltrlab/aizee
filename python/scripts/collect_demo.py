#!/usr/bin/env python3
"""
collect_demo.py — ACT demonstration data collector for AIZEE arm

Subscribes to arm telemetry (:5556) and both wrist cameras (:5563/:5564).
Press R to start/stop recording. Press Q to quit.

Usage:
    python collect_demo.py
    python collect_demo.py --telem tcp://192.168.0.27:5556 \\
                            --cam-left tcp://192.168.0.27:5563 \\
                            --cam-right tcp://192.168.0.27:5564 \\
                            --output-dir episodes/

Operator workflow per demo:
  1. Teleop terminal: move arm to consistent start position
  2. Collect terminal: verify qpos display looks right, press R
  3. Teleop terminal: perform the pickup
  4. Collect terminal: press R → [SAVED episode_0042.hdf5 — 94 steps, 4.7s]
"""

import argparse
import base64
import io
import json
import os
import sys
import time
from pathlib import Path

import h5py
import numpy as np
import zmq
from PIL import Image


# ---------------------------------------------------------------------------
# Arm joint order (must match motor controller telemetry)
# ---------------------------------------------------------------------------
ARM_JOINTS = ["gantry_base", "gantry_mid", "gantry_end", "wrist_pitch", "wrist_roll", "gripper"]
NUM_JOINTS = 6


# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def drain_sub(sock):
    """Drain a ZMQ SUB socket, return the latest message or None."""
    latest = None
    while True:
        try:
            raw = sock.recv_string(zmq.NOBLOCK)
            latest = json.loads(raw)
        except zmq.Again:
            break
        except json.JSONDecodeError:
            break
    return latest


def drain_cam(sock):
    """Drain a camera ZMQ SUB socket, return the latest message or None."""
    latest = None
    while True:
        try:
            raw = sock.recv_string(zmq.NOBLOCK)
            latest = json.loads(raw)
        except zmq.Again:
            break
        except (json.JSONDecodeError, Exception):
            break
    return latest


# ---------------------------------------------------------------------------
# Image decoding
# ---------------------------------------------------------------------------

def decode_image(msg, target_size=(320, 240)):
    """Decode a camera message to a uint8 numpy array [H, W, 3].

    Uses PIL (not cv2) to avoid BGR/RGB confusion.
    target_size is (width, height).
    """
    color = msg.get("color", {})
    data_b64 = color.get("data")
    if data_b64 is None:
        return None
    jpeg_bytes = base64.b64decode(data_b64)
    img = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
    img = img.resize(target_size, Image.LANCZOS)
    return np.array(img, dtype=np.uint8)


# ---------------------------------------------------------------------------
# Telemetry parsing
# ---------------------------------------------------------------------------

def extract_qpos(telem):
    """Extract [6] float32 arm joint positions from telemetry dict.

    Returns None if any required joint is missing.
    """
    if telem is None or "motors" not in telem:
        return None
    motors = telem["motors"]
    qpos = []
    for joint in ARM_JOINTS:
        m = motors.get(joint)
        if m is None:
            return None
        qpos.append(float(m.get("position", 0.0)))
    return np.array(qpos, dtype=np.float32)


# ---------------------------------------------------------------------------
# HDF5 writer
# ---------------------------------------------------------------------------

def save_episode(output_dir, qpos_buf, left_buf, right_buf):
    """Save collected buffers to an HDF5 episode file.

    Returns the path of the saved file.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find next episode number
    existing = sorted(output_dir.glob("episode_*.hdf5"))
    if existing:
        last_num = int(existing[-1].stem.split("_")[1])
        ep_num = last_num + 1
    else:
        ep_num = 0

    path = output_dir / f"episode_{ep_num:04d}.hdf5"

    T = len(qpos_buf)
    qpos_arr = np.stack(qpos_buf, axis=0)        # [T, 6]
    left_arr = np.stack(left_buf, axis=0)         # [T, 240, 320, 3]
    right_arr = np.stack(right_buf, axis=0)       # [T, 240, 320, 3]

    # action[t] = qpos[t+1] (next absolute position), last step repeats
    actions = np.concatenate([qpos_arr[1:], qpos_arr[-1:]], axis=0)  # [T, 6]

    with h5py.File(path, "w") as f:
        f.attrs["hz"] = 20
        f.attrs["arm_joints"] = ",".join(ARM_JOINTS)

        obs = f.create_group("observations")
        obs.create_dataset(
            "qpos", data=qpos_arr,
            compression="gzip", compression_opts=4,
        )

        imgs = obs.create_group("images")
        imgs.create_dataset(
            "left", data=left_arr,
            compression="gzip", compression_opts=4,
            chunks=(1, 240, 320, 3),
        )
        imgs.create_dataset(
            "right", data=right_arr,
            compression="gzip", compression_opts=4,
            chunks=(1, 240, 320, 3),
        )

        f.create_dataset(
            "actions", data=actions,
            compression="gzip", compression_opts=4,
        )

    return path, T


# ---------------------------------------------------------------------------
# Non-blocking keyboard (cross-platform)
# ---------------------------------------------------------------------------

def setup_keyboard():
    """Return a function that reads a single key without blocking (or None)."""
    if sys.platform == "win32":
        import msvcrt

        def _get_key():
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                return ch.upper() if hasattr(ch, "upper") else None
            return None
    else:
        import select
        import tty
        import termios

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        def _get_key():
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                return ch.upper()
            return None

        # Register cleanup
        import atexit
        atexit.register(lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old_settings))

    return _get_key


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="ACT Demo Collector")
    parser.add_argument("--telem", default="tcp://192.168.0.27:5556",
                        help="Telemetry ZMQ endpoint")
    parser.add_argument("--cam-left", default="tcp://192.168.0.27:5563",
                        help="Left camera ZMQ endpoint")
    parser.add_argument("--cam-right", default="tcp://192.168.0.27:5564",
                        help="Right camera ZMQ endpoint")
    parser.add_argument("--output-dir", default="episodes",
                        help="Directory to save HDF5 episodes")
    parser.add_argument("--max-steps", type=int, default=600,
                        help="Maximum steps per episode (default: 600 = 30s at 20Hz)")
    parser.add_argument("--image-size", default="240x320",
                        help="Image size HxW (default: 240x320)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Log data but do not write HDF5 files")
    args = parser.parse_args()

    # Parse image size
    h_str, w_str = args.image_size.split("x")
    img_h, img_w = int(h_str), int(w_str)
    img_size = (img_w, img_h)  # PIL uses (width, height)

    # ZMQ setup
    ctx = zmq.Context()

    telem_sub = ctx.socket(zmq.SUB)
    telem_sub.setsockopt(zmq.LINGER, 0)
    telem_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sub.connect(args.telem)

    left_sub = ctx.socket(zmq.SUB)
    left_sub.setsockopt(zmq.LINGER, 0)
    left_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    left_sub.connect(args.cam_left)

    right_sub = ctx.socket(zmq.SUB)
    right_sub.setsockopt(zmq.LINGER, 0)
    right_sub.setsockopt_string(zmq.SUBSCRIBE, "")
    right_sub.connect(args.cam_right)

    print(f"Connecting to telemetry: {args.telem}")
    print(f"Connecting to left cam:  {args.cam_left}")
    print(f"Connecting to right cam: {args.cam_right}")
    print(f"Output dir: {args.output_dir}")
    print()
    print("Controls:")
    print("  R = toggle record start/stop")
    print("  Q = quit")
    print()
    if args.dry_run:
        print("[DRY RUN MODE — no files will be written]")
        print()

    # Keyboard
    get_key = setup_keyboard()

    # State
    recording = False
    qpos_buf = []
    left_buf = []
    right_buf = []

    last_telem_time = 0.0
    last_left_time = 0.0
    last_right_time = 0.0
    latest_telem = None
    latest_left = None
    latest_right = None

    STALE_THRESH = 0.200  # 200 ms
    tick = 1.0 / 20.0    # 20 Hz

    print("Waiting for data... (Ctrl+C to abort)")

    try:
        while True:
            t0 = time.monotonic()

            # --- Drain all sockets ---
            telem = drain_sub(telem_sub)
            if telem is not None:
                latest_telem = telem
                last_telem_time = t0

            left_msg = drain_cam(left_sub)
            if left_msg is not None:
                latest_left = left_msg
                last_left_time = t0

            right_msg = drain_cam(right_sub)
            if right_msg is not None:
                latest_right = right_msg
                last_right_time = t0

            # --- Keyboard ---
            key = get_key()
            if key == "R":
                if not recording:
                    recording = True
                    qpos_buf = []
                    left_buf = []
                    right_buf = []
                    print(f"\n[RECORDING STARTED]")
                else:
                    recording = False
                    steps = len(qpos_buf)
                    duration = steps / 20.0
                    if steps == 0:
                        print("\n[RECORDING STOPPED — 0 steps, nothing saved]")
                    elif args.dry_run:
                        print(f"\n[DRY RUN] Would save {steps} steps ({duration:.1f}s)")
                    else:
                        path, T = save_episode(
                            args.output_dir, qpos_buf, left_buf, right_buf
                        )
                        print(f"\n[SAVED {path.name} — {T} steps, {duration:.1f}s]")
            elif key == "Q":
                print("\nQuitting...")
                break

            # --- Check freshness ---
            telem_age = t0 - last_telem_time if last_telem_time > 0 else 999
            left_age = t0 - last_left_time if last_left_time > 0 else 999
            right_age = t0 - last_right_time if last_right_time > 0 else 999

            telem_ok = telem_age < STALE_THRESH and latest_telem is not None
            left_ok = left_age < STALE_THRESH and latest_left is not None
            right_ok = right_age < STALE_THRESH and latest_right is not None
            all_ok = telem_ok and left_ok and right_ok

            # --- Extract data ---
            qpos = extract_qpos(latest_telem) if latest_telem else None
            left_img = decode_image(latest_left, img_size) if latest_left else None
            right_img = decode_image(latest_right, img_size) if latest_right else None

            # --- Status display ---
            qpos_str = (
                " ".join(f"{v:+6.3f}" for v in qpos)
                if qpos is not None else "no data"
            )
            rec_str = f"REC {len(qpos_buf):4d}" if recording else "IDLE     "
            stale_flags = ""
            if not telem_ok:
                stale_flags += " [TELEM STALE]"
            if not left_ok:
                stale_flags += " [LEFT CAM STALE]"
            if not right_ok:
                stale_flags += " [RIGHT CAM STALE]"

            print(
                f"\r[{rec_str}] qpos: {qpos_str}{stale_flags}          ",
                end="", flush=True,
            )

            # --- Record if active ---
            if recording:
                if not all_ok:
                    # Warn but keep recording — operator can re-record
                    pass  # status already shows stale flags
                elif qpos is None or left_img is None or right_img is None:
                    pass  # decode failed
                else:
                    qpos_buf.append(qpos)
                    left_buf.append(left_img)
                    right_buf.append(right_img)

                    if len(qpos_buf) >= args.max_steps:
                        recording = False
                        steps = len(qpos_buf)
                        duration = steps / 20.0
                        if args.dry_run:
                            print(
                                f"\n[DRY RUN] Max steps reached: {steps} steps "
                                f"({duration:.1f}s)"
                            )
                        else:
                            path, T = save_episode(
                                args.output_dir, qpos_buf, left_buf, right_buf
                            )
                            print(
                                f"\n[SAVED {path.name} — {T} steps, {duration:.1f}s]"
                                f" (max steps reached)"
                            )

            # --- Sleep remainder ---
            elapsed = time.monotonic() - t0
            remaining = tick - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        telem_sub.close()
        left_sub.close()
        right_sub.close()
        ctx.term()
        print("Done.")


if __name__ == "__main__":
    main()
