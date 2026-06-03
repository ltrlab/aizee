"""Smoke test for QuestLeader — verifies the clutched IK pipeline end-to-end
without needing a real Quest headset or aiohttp server.

Run with:  python -m teleop.test_quest_leader   (from inside python/)
"""

from __future__ import annotations

import sys
import time

import numpy as np

from ik import load_aizee_arm
from ik.kinematics import R_to_quat
from teleop.quest_leader import QuestLeader, QuestLeaderConfig


class _FakeState:
    """Stand-in for web.SharedState."""
    def __init__(self):
        self.latest_control = None
        self.latest_telem = None


def _ctrl(pos=(0, 0, 0), quat=(0, 0, 0, 1), trigger=0.0, grip=False, b=False):
    return {
        "_rx_ts": time.time(),
        "ts": time.time(),
        "head": {"pos": [0, 1.6, 0], "quat": [0, 0, 0, 1]},
        "right": {
            "pos": list(pos),
            "quat": list(quat),
            "trigger": trigger,
            "grip": grip,
            "a": False,
            "b": b,
        },
        "left": {"pos": [0, 0, 0], "quat": [0, 0, 0, 1], "stick": [0, 0], "a": False, "grip": False},
    }


def main() -> int:
    print("[quest-test] building QuestLeader with synthetic state")
    state = _FakeState()
    kin = load_aizee_arm()
    ql = QuestLeader(shared_state=state, config=QuestLeaderConfig())
    assert ql.JOINTS == ql.AIZEE_JOINTS == [
        "swivel", "gantry_base", "gantry_mid", "gantry_end",
        "wrist_pitch", "wrist_roll", "gripper",
    ]
    assert np.allclose(ql.zero_offsets, 0.0)
    assert np.allclose(ql.directions, 1.0)
    assert ql.connect() is True

    # Telemetry: pretend the arm is at a reachable, non-zero pose.
    q_now = np.array([0.1, 0.3, -0.2, 0.4, 0.1, -0.2, 0.0], dtype=np.float32)
    state.latest_telem = {"ts": time.time(), "qpos": q_now.tolist()}
    pos_now, _ = kin.fk_pose(q_now[:6])
    print(f"[quest-test] simulated arm at qpos[0:3]={q_now[:3]}  EE pos={pos_now}")

    # Phase A: no controller frame -> poll returns None.
    out = ql.poll()
    assert out is None, "expected None with no control frame yet"

    # Phase B: controller appears, grip OFF, trigger 0.5.
    # poll() is rate-limited internally to 60 Hz; sleep so the gate opens.
    time.sleep(0.02)
    state.latest_control = _ctrl(pos=(0.1, 1.4, -0.3), trigger=0.5, grip=False)
    out = ql.poll()
    # With no engage yet, _q_last is zeros and gripper joint should follow trigger.
    assert out is not None
    print(f"[quest-test] no-clutch out={out}")
    assert abs(float(out[6]) - 0.5 * ql.cfg.gripper_closed_rad) < 1e-3, "gripper not tracking trigger"

    # Phase C: rising edge of grip — engage at the current pose.
    time.sleep(0.02)
    state.latest_control = _ctrl(pos=(0.1, 1.4, -0.3), trigger=0.5, grip=True)
    out_engage = ql.poll()
    assert out_engage is not None
    print(f"[quest-test] engage out[0:6]={out_engage[:6]}  expected~={q_now[:6]}")
    # On engage with zero delta, the IK target equals engage_ee_pose, so q should
    # stay (essentially) at the current qpos.  Velocity clamp may bound the
    # first step, so allow a sub-tolerance.
    assert np.allclose(out_engage[:6], q_now[:6], atol=0.05), \
        "engage with zero delta should leave joints near current qpos"

    # Phase D: while engaged, move controller +5 cm in -Z (forward in robot frame).
    # +X_robot = -Z_xr -> so moving controller from z=-0.30 to z=-0.35 should
    # push the EE forward in +X_robot by ~5 cm.
    time.sleep(0.02)
    state.latest_control = _ctrl(pos=(0.1, 1.4, -0.35), trigger=0.5, grip=True)
    out_move = ql.poll()
    assert out_move is not None
    pos_after, _ = kin.fk_pose(out_move[:6].astype(np.float64))
    forward_delta = float(pos_after[0] - pos_now[0])
    print(f"[quest-test] after +5 cm -Z_xr move: EE +X_robot delta = {forward_delta*1000:.2f} mm")
    # Velocity clamp is 4 rad/s * ~16 ms = ~64 mrad per joint; the EE delta
    # may be less than 5 cm on the first tick.  Just assert direction + nonzero.
    assert forward_delta > 0.001, f"expected forward EE motion, got {forward_delta}"

    # Phase E: release clutch — subsequent moves should NOT move the arm.
    time.sleep(0.02)
    state.latest_control = _ctrl(pos=(0.1, 1.4, -0.40), trigger=0.5, grip=False)
    q_before_release = out_move[:6].copy()
    out_release = ql.poll()
    pos_release, _ = kin.fk_pose(out_release[:6].astype(np.float64))
    pos_held, _ = kin.fk_pose(q_before_release.astype(np.float64))
    drift = float(np.linalg.norm(pos_release - pos_held))
    print(f"[quest-test] after release + further move: EE drift = {drift*1000:.4f} mm")
    assert drift < 1e-3, "EE moved after clutch release"

    # Phase F: E-stop edge — B rising should latch and hold.
    time.sleep(0.02)
    state.latest_control = _ctrl(pos=(0.1, 1.4, -0.40), trigger=1.0, grip=True, b=True)
    out_estop = ql.poll()
    # Even though grip+trigger says move + close gripper, E-stop must hold q_last.
    assert np.allclose(out_estop, ql._q_last.astype(np.float32)), "e-stop did not hold q"
    print(f"[quest-test] e-stop latch OK; q held at {out_estop}")

    print("[quest-test] OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
