"""Gamepad helpers (from collect_demo.py)."""
from __future__ import annotations

import sys
from typing import Optional

try:
    import pygame
    _pygame_available = True
except ImportError:
    _pygame_available = False

# ---------------------------------------------------------------------------
# Gamepad helpers
# ---------------------------------------------------------------------------

def _init_joystick():
    if not _pygame_available:
        return None
    try:
        import os
        if sys.platform == "win32":
            os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "-10000,-10000")
        pygame.init()
        if sys.platform == "win32":
            pygame.display.set_mode((1, 1))
        pygame.joystick.init()
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            js.init()
            if "keyboard" in js.get_name().lower():
                continue
            if js.get_numaxes() >= 2:
                return js
    except Exception:
        pass
    return None


def _apply_deadzone(value: float, deadzone: float = 0.08) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _apply_curve(value: float, exponent: float = 1.5) -> float:
    sign = 1.0 if value >= 0 else -1.0
    return sign * (abs(value) ** exponent)


def _ramp_toward(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    diff = target - current
    rate = accel if abs(target) > abs(current) else decel
    max_change = rate * dt
    if abs(diff) <= max_change:
        return target
    return current + max_change if diff > 0 else current - max_change


def _read_gamepad(joystick, prev_a: bool, prev_b: bool, prev_start: bool,
                  gp_cfg: Optional[dict] = None) -> dict:
    _empty = {
        "enable": False, "shutdown": False, "hold": False, "quit": False,
        "raw_a": False, "raw_b": False, "raw_start": False,
        "drive_linear": 0.0, "drive_angular": 0.0,
    }
    try:
        pygame.event.pump()
        raw_a     = bool(joystick.get_button(0))
        raw_b     = bool(joystick.get_button(1))
        raw_back  = bool(joystick.get_button(6))
        raw_start = bool(joystick.get_button(7))

        # Left stick axes for driving
        drive_linear  = 0.0
        drive_angular = 0.0
        if gp_cfg is not None:
            axes   = gp_cfg.get("axes", {})
            invert = gp_cfg.get("axis_invert", {})
            dz     = 0.08

            raw_y = joystick.get_axis(axes.get("left_stick_y", 1))
            if invert.get("left_stick_y", False):
                raw_y = -raw_y
            raw_x = joystick.get_axis(axes.get("left_stick_x", 0))
            if invert.get("left_stick_x", False):
                raw_x = -raw_x

            # WORKAROUND: motor controller has linear/angular backwards
            # Y-axis → angular (forward/back), X-axis → linear (turn)
            # Negate angular so stick-forward = robot-forward
            drive_angular = -_apply_curve(_apply_deadzone(raw_y, dz))
            drive_linear  = _apply_curve(_apply_deadzone(raw_x, dz))

        return {
            "enable":    raw_a and not prev_a,
            "shutdown":  raw_b and not prev_b,
            "hold":      raw_start and not prev_start,
            "quit":      raw_back,
            "raw_a":     raw_a,
            "raw_b":     raw_b,
            "raw_start": raw_start,
            "drive_linear":  drive_linear,
            "drive_angular": drive_angular,
        }
    except Exception:
        return _empty
