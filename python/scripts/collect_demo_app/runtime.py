"""Loop-rate constants, platform setup, and display-layout constants (from collect_demo.py)."""
from __future__ import annotations

import sys

from common.arm_constants import ARM_JOINTS

LOOP_HZ    = 30
REC_HZ     = 20
NUM_JOINTS = len(ARM_JOINTS)   # 7 (swivel + 6 gantry)

# Reduce GIL switch interval so background threads (camera JSON parsing,
# image decode, Rerun logging) yield to the main loop faster.  Default is
# 5 ms — a single large camera JSON parse can stall the main loop that long.
sys.setswitchinterval(0.001)   # 1 ms

# On Windows, time.sleep granularity defaults to ~15.6 ms, which inflates
# the 30 Hz period jitter (we've measured p99 leaking to 100+ ms).  Asking
# the multimedia timer for 1 ms resolution tightens the loop period to the
# OS scheduler floor.  No-op on non-Windows.
if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------

_W  = 76
_IW = _W - 2

# Display layout — leader joints in the same swivel-first order as ARM_JOINTS.
_LEADER_JOINTS = list(ARM_JOINTS)

_BASE_MOTORS = ["left_wheel", "right_wheel"]
# Swivel is now part of ARM_JOINTS (joint 0), so no separate entry here.
_ALL_MOTORS  = _BASE_MOTORS + list(ARM_JOINTS)
