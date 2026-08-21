#!/usr/bin/env python3
"""flip_leader_dir.py — flip the teleop direction (+1 <-> -1) of leader joints.

Use when a joint drives its arm the WRONG way during teleop (move the leader one way,
the arm goes the other). Flips the `direction` field in the per-leader calib; restart
the collector and re-Mirror (M) afterward so the zero re-seeds for the new sign.

    python python/scripts/flip_leader_dir.py left j3          # left arm, joint 3
    python python/scripts/flip_leader_dir.py right j2 j5      # several at once
    python python/scripts/flip_leader_dir.py left gripper
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

# Minerva jN / gripper  ->  OpenRB leader joint name (poll order, = arm joint order)
_JMAP = {"j1": "shoulder_pan", "j2": "shoulder_lift", "j3": "elbow_flex",
         "j4": "wrist_flex", "j5": "wrist_yaw", "j6": "wrist_roll", "gripper": "gripper"}


def main() -> None:
    if len(sys.argv) < 3 or sys.argv[1].lower() not in ("left", "right"):
        print(__doc__)
        sys.exit(1)
    side = sys.argv[1].lower()
    joints = [a.lower() for a in sys.argv[2:]]
    path = Path(__file__).resolve().parents[2] / "config" / f"openrb_{side}.json"
    if not path.exists():
        print(f"{path} not found — calibrate the {side} leader first.")
        sys.exit(1)
    data = json.loads(path.read_text())
    jd = data.get("joints", {})
    changed = 0
    for jn in joints:
        name = _JMAP.get(jn, jn)
        if name not in jd:
            print(f"  {jn} ({name}) not in {path.name} — skipped")
            continue
        old = jd[name].get("direction", 1)
        jd[name]["direction"] = -1 if old >= 0 else 1
        print(f"  {side} {jn} ({name}): direction {old:+d} -> {jd[name]['direction']:+d}")
        changed += 1
    if changed:
        path.write_text(json.dumps(data, indent=2))
        print(f"Saved {path.name}. Restart the collector, then press M to re-zero.")
    else:
        print("Nothing changed.")


if __name__ == "__main__":
    main()
