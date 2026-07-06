#!/usr/bin/env python3
"""
mock_arm_publisher.py — Simulates the AIZEE arm hardware ZMQ streams.

Publishes exactly the same message format as the real hardware so you can
run collect_demo.py, train.py, and act_policy_node.py against this mock
without touching the real arm.

Sockets (binds, matching real hardware topology):
    :5556 PUB  — arm telemetry  (matches Rust motor_control)
    :5563 PUB  — gripper camera (matches gripper_camera_node.py)
    :5564 PUB  — scene camera   (matches camera_node.py / RealSense)

In --mode closed_loop only:
    :5555 PULL — receives arm_joints commands, applies simulated dynamics

Modes:
  sinusoidal   (default) — each joint oscillates at a fixed sine wave.
                           Use this to generate synthetic demos via collect_demo.py.
  closed_loop  —          joints respond to commands received on :5555.
                           Use this to test act_policy_node.py end-to-end.

Typical test workflows
----------------------
# 1. Collect synthetic demos (sinusoidal mode)
#    Terminal A:
        python python/scripts/mock_arm_publisher.py
#    Terminal B:
        python python/scripts/collect_demo.py \\
            --telem tcp://localhost:5556 \\
            --cam-left tcp://localhost:5563 \\
            --cam-right tcp://localhost:5564

# 2. Train on those demos
        python python/training/train.py --data-dir episodes/ --epochs 5 --batch-size 4

# 3. Test inference loop (closed-loop mode, dry-run — no real commands)
#    Terminal A:
        python python/scripts/mock_arm_publisher.py --mode closed_loop
#    Terminal B:
        python python/nodes/act_policy_node.py \\
            --checkpoint checkpoints/act_epoch_0005.pt \\
            --telem tcp://localhost:5556 \\
            --cam-left tcp://localhost:5563 \\
            --cam-right tcp://localhost:5564 \\
            --cmd tcp://localhost:5555 \\
            --dry-run   # remove this flag once you're confident

# 4. Full closed-loop without dry-run
#    Watch the mock's console — it prints what commands it receives and
#    how the simulated arm responds. Verify positions stay within range.
"""

import argparse
import base64
import io
import json
import math
import sys
import time

import numpy as np
import zmq
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------------------
# Joint configuration — canonical 7-DoF vocabulary (swivel first). The old
# local 6-joint copy made collect_demo's telemetry extractor reject every
# message (it requires all of ARM_JOINTS to be present).
# ---------------------------------------------------------------------------
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.arm_constants import ARM_JOINTS, KP, KD

NUM_JOINTS = len(ARM_JOINTS)

# Sinusoidal motion parameters: (amplitude_rad, frequency_hz, phase_offset_rad)
# Chosen to look like a plausible slow pick-and-place trajectory.
JOINT_SINUSOIDS = [
    (0.35, 0.06, 0.52),  # swivel       ±0.35 rad
    (0.30, 0.12, 0.0),   # gantry_base  ±0.30 rad
    (0.40, 0.10, 1.05),  # gantry_mid   ±0.40 rad
    (0.20, 0.15, 2.09),  # gantry_end   ±0.20 rad
    (0.15, 0.18, 3.14),  # wrist_pitch  ±0.15 rad
    (0.10, 0.22, 4.19),  # wrist_roll   ±0.10 rad
    (0.25, 0.08, 5.24),  # gripper      ±0.25 rad
]
assert len(JOINT_SINUSOIDS) == NUM_JOINTS

# Simulation dynamics: fraction of error closed per step (per-joint).
# Derived from the canonical kd/kp time constants. Capped at 0.8 to avoid
# overshoot in this simple Euler approximation.
_TAU = np.array([kd / kp for kp, kd in zip(KP, KD)], dtype=np.float32)
_SIM_DT = 1.0 / 30.0
_SIM_ALPHA = np.minimum(0.8, _SIM_DT / np.maximum(_TAU, 1e-3))  # per-joint convergence


class MockArmPublisher:

    def __init__(self, args):
        self.mode = args.mode
        self.hz = args.hz
        self.telem_port = args.telem_port
        self.gripper_cam_port = args.gripper_cam_port
        self.scene_cam_port = args.scene_cam_port
        self.cmd_port = args.cmd_port

        # Joint state
        self.q = np.zeros(NUM_JOINTS, dtype=np.float32)   # current simulated positions
        self.q_cmd = np.zeros(NUM_JOINTS, dtype=np.float32)  # last received command

        self.t_start = time.monotonic()
        self.frame_count = 0
        self.cmd_count = 0

        # ZMQ
        self.ctx = zmq.Context()

        self.telem_pub = self.ctx.socket(zmq.PUB)
        self.telem_pub.setsockopt(zmq.SNDHWM, 10)
        self.telem_pub.bind(f"tcp://*:{self.telem_port}")

        self.gripper_pub = self.ctx.socket(zmq.PUB)
        self.gripper_pub.setsockopt(zmq.SNDHWM, 4)
        self.gripper_pub.bind(f"tcp://*:{self.gripper_cam_port}")

        self.scene_pub = self.ctx.socket(zmq.PUB)
        self.scene_pub.setsockopt(zmq.SNDHWM, 4)
        self.scene_pub.bind(f"tcp://*:{self.scene_cam_port}")

        self.cmd_pull = None
        if self.mode == "closed_loop":
            self.cmd_pull = self.ctx.socket(zmq.PULL)
            self.cmd_pull.setsockopt(zmq.LINGER, 0)
            self.cmd_pull.bind(f"tcp://*:{self.cmd_port}")
            print(f"  Bound PULL (cmd receiver) on :{self.cmd_port}")

        print(f"  Bound PUB (telemetry)   on :{self.telem_port}")
        print(f"  Bound PUB (gripper cam) on :{self.gripper_cam_port}")
        print(f"  Bound PUB (scene cam)   on :{self.scene_cam_port}")

    # ------------------------------------------------------------------
    # Joint state
    # ------------------------------------------------------------------

    def _sinusoidal_q(self, t: float) -> np.ndarray:
        """Compute joint positions for sinusoidal mode at time t."""
        q = np.zeros(NUM_JOINTS, dtype=np.float32)
        for i, (amp, freq, phase) in enumerate(JOINT_SINUSOIDS):
            q[i] = amp * math.sin(2 * math.pi * freq * t + phase)
        return q

    def _step_dynamics(self):
        """Advance simulated joint positions toward last command."""
        self.q += _SIM_ALPHA * (self.q_cmd - self.q)

    # ------------------------------------------------------------------
    # Message builders
    # ------------------------------------------------------------------

    def _build_telemetry(self) -> dict:
        motors = {}
        for i, joint in enumerate(ARM_JOINTS):
            motors[joint] = {
                "position":    float(self.q[i]),
                "velocity":    0.0,
                "torque":      0.0,
                "temperature": 35.0,
                "error":       None,
                "state":       "running",
            }
        return {"timestamp": time.time(), "motors": motors}

    def _build_camera_msg(self, camera_id: str, side: str) -> dict:
        """Build a camera message with a PIL image encoded as base64 JPEG.

        The image shows the current joint positions so you can visually verify
        that the right data is flowing during dry-run testing.
        """
        t_elapsed = time.monotonic() - self.t_start

        # Background: hue slowly cycles with time, slightly different per camera
        hue_base = int(t_elapsed * 20) % 256
        offset = 80 if side == "scene" else 0
        bg_r = (hue_base + offset) % 256
        bg_g = (128 + hue_base // 2) % 256
        bg_b = (255 - hue_base + offset) % 256
        img = Image.new("RGB", (640, 480), color=(bg_r, bg_g, bg_b))

        draw = ImageDraw.Draw(img)

        # Header
        draw.rectangle([(0, 0), (640, 30)], fill=(30, 30, 30))
        draw.text((10, 8), f"MOCK {side.upper()} CAM   t={t_elapsed:.2f}s  "
                           f"frame={self.frame_count}", fill=(220, 220, 220))

        # Joint positions
        draw.rectangle([(0, 35), (300, 200)], fill=(20, 20, 20, 180))
        for i, (joint, pos) in enumerate(zip(ARM_JOINTS, self.q)):
            bar_w = int(abs(pos) / 0.5 * 80)  # visual bar
            bar_color = (100, 200, 100) if pos >= 0 else (200, 100, 100)
            draw.text(
                (10, 40 + i * 26),
                f"{joint:<14}: {pos:+.3f}",
                fill=(240, 240, 240),
            )
            draw.rectangle(
                [(160, 44 + i * 26), (160 + min(bar_w, 130), 56 + i * 26)],
                fill=bar_color,
            )

        # Mode label
        mode_color = (255, 180, 0) if self.mode == "closed_loop" else (100, 200, 255)
        draw.text((10, 215), f"MODE: {self.mode.upper()}", fill=mode_color)
        if self.mode == "closed_loop":
            draw.text((10, 240), f"cmds received: {self.cmd_count}", fill=(200, 200, 200))

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=70)
        jpeg_bytes = buf.getvalue()

        return {
            "camera_id": camera_id,
            "timestamp": time.time(),
            "frame_number": self.frame_count,
            "color": {
                "data":   base64.b64encode(jpeg_bytes).decode("ascii"),
                "format": "jpeg",
                "width":  640,
                "height": 480,
            },
        }

    # ------------------------------------------------------------------
    # Command draining (closed_loop mode)
    # ------------------------------------------------------------------

    def _drain_commands(self):
        """Drain :5555, apply latest arm_joints command to simulation."""
        if self.cmd_pull is None:
            return
        latest = None
        while True:
            try:
                raw = self.cmd_pull.recv_string(zmq.NOBLOCK)
                latest = json.loads(raw)
            except zmq.Again:
                break
            except json.JSONDecodeError:
                break

        if latest is not None and latest.get("type") == "arm_joints":
            positions = latest.get("positions", [])
            if len(positions) == NUM_JOINTS:
                self.q_cmd = np.array(positions, dtype=np.float32)
                self.cmd_count += 1
            elif len(positions) == NUM_JOINTS - 1:
                # legacy 6-DoF (no swivel) command — apply to joints 1..6
                self.q_cmd[1:] = np.array(positions, dtype=np.float32)
                self.cmd_count += 1

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        tick = 1.0 / self.hz
        print(f"\nRunning at {self.hz} Hz  (Ctrl+C to stop)")
        print("─" * 60)

        last_stats_time = time.monotonic()
        stats_interval = 5.0   # print summary every 5 s

        try:
            while True:
                t0 = time.monotonic()
                t_elapsed = t0 - self.t_start

                # Update joint state
                if self.mode == "sinusoidal":
                    self.q = self._sinusoidal_q(t_elapsed)
                elif self.mode == "closed_loop":
                    self._drain_commands()
                    self._step_dynamics()

                # Build and send messages
                telem = self._build_telemetry()
                self.telem_pub.send_string(json.dumps(telem), zmq.NOBLOCK)

                gripper_msg = self._build_camera_msg("gripper_cam", "gripper")
                self.gripper_pub.send_string(json.dumps(gripper_msg), zmq.NOBLOCK)

                scene_msg = self._build_camera_msg("scene_cam", "scene")
                self.scene_pub.send_string(json.dumps(scene_msg), zmq.NOBLOCK)

                self.frame_count += 1

                # Status line
                q_str = "  ".join(f"{j[:6]}:{v:+.2f}" for j, v in zip(ARM_JOINTS, self.q))
                print(f"\r[{t_elapsed:7.1f}s | frame {self.frame_count:5d}]  {q_str}   ",
                      end="", flush=True)

                # Periodic summary
                now = time.monotonic()
                if now - last_stats_time >= stats_interval:
                    print()
                    if self.mode == "closed_loop":
                        q_err = np.abs(self.q - self.q_cmd)
                        print(f"  Position error from command: "
                              f"max={q_err.max():.3f}  mean={q_err.mean():.3f} rad")
                        print(f"  Commands received so far: {self.cmd_count}")
                    print(f"  Telemetry frames published: {self.frame_count}")
                    last_stats_time = now

                # Sleep remainder
                elapsed = time.monotonic() - t0
                time.sleep(max(0, tick - elapsed))

        except KeyboardInterrupt:
            print(f"\n\nMock stopped after {self.frame_count} frames.")
            if self.mode == "closed_loop":
                print(f"Total arm_joints commands received: {self.cmd_count}")
        finally:
            self.telem_pub.close()
            self.gripper_pub.close()
            self.scene_pub.close()
            if self.cmd_pull:
                self.cmd_pull.close()
            self.ctx.term()


def main():
    parser = argparse.ArgumentParser(
        description="Mock AIZEE arm hardware publisher",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--mode", choices=["sinusoidal", "closed_loop"], default="sinusoidal",
        help="sinusoidal: fixed sine-wave motion (for collecting demos). "
             "closed_loop: responds to arm_joints commands on --cmd-port.",
    )
    parser.add_argument("--telem-port",       type=int, default=5556)
    parser.add_argument("--gripper-cam-port", type=int, default=5563)
    parser.add_argument("--scene-cam-port",   type=int, default=5564)
    parser.add_argument("--cmd-port",      type=int, default=5555,
                        help="PULL port for arm_joints commands (closed_loop only)")
    parser.add_argument("--hz", type=int, default=30,
                        help="Publish rate in Hz (default 30)")
    args = parser.parse_args()

    print("=" * 60)
    print(f"  AIZEE Mock Arm Publisher — {args.mode.upper()} mode")
    print("=" * 60)
    MockArmPublisher(args).run()


if __name__ == "__main__":
    main()
