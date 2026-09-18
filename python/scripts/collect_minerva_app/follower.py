"""Minerva's historical collector API backed by the shared robot session.

The shared worker owns every motor socket, named route, freshness check and
100 Hz keepalive. This compatibility boundary alone retains the old 17-slot
arrays with zero head/lift placeholders; shared observations mark those slots
inactive and unavailable. An unavailable arm remains NaN, never fabricated zero.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Dict, List, Optional

import numpy as np

from common.minerva_constants import KD, KP, MINERVA_JOINTS, NUM_MINERVA_JOINTS
from robot_platform.control import RobotSession, ThreadedRobotClient
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings

LEFT_NAMES: List[str] = list(MINERVA_JOINTS[:7])
RIGHT_NAMES: List[str] = list(MINERVA_JOINTS[7:14])


class DualArmTransport:
    """Two arm endpoints with shared routing, observation and command lifecycle."""

    def __init__(self, ctx, *, left_cmd: str, left_telem: str,
                 right_cmd: str, right_telem: str, reemit_hz: int = 100,
                 receive_only: bool = False) -> None:
        spec = load_robot("minerva")
        overrides = {"endpoints": {"arms": {
            "left": {"command": left_cmd, "telemetry": left_telem},
            "right": {"command": right_cmd, "telemetry": right_telem},
        }}}
        resolved = os.environ.get("ROBOT_PLATFORM_SETTINGS_JSON")
        if resolved:
            overrides["control"] = json.loads(resolved).get("control", {})
        self.session = RobotSession(spec, load_settings(spec, overrides=overrides))
        self.client = ThreadedRobotClient(self.session, context=ctx, reemit_hz=reemit_hz,
                                          receive_only=receive_only)
        self.client.start()

    def _merged(self, field: str, *, partial: bool = False) -> Optional[np.ndarray]:
        values = np.array(self.client.joint_field(field), dtype=np.float32)
        indices = {name: index for index, name in enumerate(self.session.joint_names)}
        present = []
        for endpoint in self.session.arms.values():
            slots = [indices[name] for name in endpoint.joints]
            valid = bool(np.all(np.isfinite(values[slots])))
            present.append(valid)
            if not valid:
                values[slots] = np.nan
        if not any(present) or not partial and not all(present):
            return None
        # Historical v6/v7 files and collector math use these fixed placeholders.
        # The shared Observation and validity masks still report them as missing.
        values[~np.asarray(self.session.active_mask)] = 0.0
        return values

    def qpos(self) -> Optional[np.ndarray]:
        return self._merged("position")

    def torques(self) -> Optional[np.ndarray]:
        return self._merged("torque")

    def qpos_partial(self) -> Optional[np.ndarray]:
        return self._merged("position", partial=True)

    def torques_partial(self) -> Optional[np.ndarray]:
        return self._merged("torque", partial=True)

    def temps_partial(self) -> Optional[np.ndarray]:
        return self._merged("temperature", partial=True)

    def observation(self):
        """Shared snapshot; .ready requires both arms' measured position and torque."""
        return self.client.snapshot()

    def recording_sample(self, *, require_torque: bool = False):
        """Atomic sample with both positions fresh; absent torque stays NaN.

        Torque-dependent consumers may opt into strict completeness. The legacy
        position/camera recorder keeps samples when torque alone is unavailable.
        """
        observation = self.client.snapshot()
        if not (observation.ready if require_torque else observation.position_ready):
            return None
        positions = np.array(observation.positions, dtype=np.float32)
        torques = np.array(observation.torques, dtype=np.float32)
        inactive = ~np.asarray(observation.active_mask)
        positions[inactive] = torques[inactive] = 0.0
        return positions, torques

    def battery_voltage(self) -> Optional[float]:
        values = []
        for arm in self.session.arms:
            message = self.client.telemetry(arm)
            value = message.get("battery_voltage") if message else None
            if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
                values.append(float(value))
        return min(values) if values else None

    def estop(self) -> Optional[bool]:
        values = []
        for arm in self.session.arms:
            message = self.client.telemetry(arm)
            if message and isinstance(message.get("emergency_stop"), bool):
                values.append(message["emergency_stop"])
        return any(values) if values else None

    def telem_age(self, now: Optional[float] = None) -> Dict[str, float]:
        # Keep the historical optional wall-clock argument at this boundary.
        adjustment = 0.0 if now is None else now - time.time()
        return {arm: (max(0.0, state.age_s + adjustment) if math.isfinite(state.age_s) else 999.0)
                for arm, state in self.client.snapshot().arm_freshness.items()}

    def arm_ok(self) -> Dict[str, bool]:
        return self.client.arm_ready("position")

    def _service(self, kind: str, **kwargs) -> None:
        self.client.submit({arm: {"type": kind, "motor_ids": list(endpoint.joints), **kwargs}
                            for arm, endpoint in self.session.arms.items()})

    def enable(self) -> None:
        self._service("enable")

    def disable(self) -> None:
        # Worker atomically forgets pending/held motion before queuing disable.
        self._service("disable")

    def mech_zero(self, save: bool = True) -> None:
        """Keep existing Rust mechanical-zero semantics; motors must be disabled."""
        self._service("mech_zero", save=save)

    def set_target(self, q17: np.ndarray, kp17=None, kd17=None, tau17=None) -> None:
        # Gains in the old constants include unrealized head/lift defaults. They
        # are metadata, so omit them explicitly at the legacy boundary. Position
        # or torque requests for inactive axes remain errors in the shared router.
        kp = list(KP if kp17 is None else kp17)
        kd = list(KD if kd17 is None else kd17)
        if len(kp) != NUM_MINERVA_JOINTS or len(kd) != NUM_MINERVA_JOINTS:
            raise ValueError("Minerva gain vectors require 17 values")
        for index, active in enumerate(self.session.active_mask):
            if not active:
                kp[index] = kd[index] = 0.0
        tau = [0.0] * NUM_MINERVA_JOINTS if tau17 is None else tau17
        self.client.set_target(q17, kp=kp, kd=kd, torques=tau,
                               allow_partial=True, require_torque=False)

    def clear_target(self) -> None:
        self.client.clear_target()

    def close(self) -> None:
        self.client.close()


__all__ = ["DualArmTransport", "LEFT_NAMES", "RIGHT_NAMES"]
