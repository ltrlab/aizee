#!/usr/bin/env python3
"""
AIZEE Teleop — Real-time teleoperation interface

Terminal-based UI with Xbox controller and keyboard input.
Communicates with the Rust motor controller via ZeroMQ.

Usage:
    python teleop.py                          # default config
    python teleop.py --config path/to.yaml    # custom config
    python teleop.py --keyboard-only          # no gamepad
    python teleop.py --endpoint tcp://IP:5555 # override endpoint

KNOWN ISSUE:
    The motor controller (rust/motor_control) interprets linear/angular parameters
    backwards. This teleop includes workarounds in the input mapping and display
    to provide correct control feel and semantic display labels.
    TODO: Fix this properly in the Rust motor controller code.
"""

import argparse
import curses
import json
import logging
import os
import sys
import time
import yaml
import zmq

# Pygame for gamepad — optional
# On Linux (Jetson/SSH), use dummy video driver since there's no display.
# On Windows, SDL needs a real video driver for joystick input to work.
_pygame_available = False
try:
    if sys.platform != "win32":
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    _pygame_available = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

def setup_logging(log_level=logging.INFO):
    """Configure logging for teleop."""
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('teleop.log'),
            logging.StreamHandler(sys.stderr)
        ]
    )
    return logging.getLogger('aizee.teleop')


logger = logging.getLogger('aizee.teleop')


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

def load_config(path):
    """Load teleop YAML config, return dict."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Input state
# ---------------------------------------------------------------------------

class InputState:
    """Holds current input values and one-shot action flags."""

    def __init__(self):
        # Continuous axes (-1.0 .. +1.0, scaled to max later)
        self.linear = 0.0
        self.angular = 0.0

        # Swivel joint position (relative to home, in radians) - position controlled
        self.swivel_position = 0.0
        self.swivel_offset = 0.0
        self.swivel_initialized = False

        # Gantry joint positions (relative to home, in radians)
        self.gantry_base = 0.0
        self.gantry_mid = 0.0
        self.gantry_end = 0.0
        # Home offsets (absolute encoder positions when homed)
        self.gantry_base_offset = 0.0
        self.gantry_mid_offset = 0.0
        self.gantry_end_offset = 0.0
        self.gantry_base_rate = 0.0      # Right stick Y → gantry base velocity (-1..+1)
        self.gantry_base_rate_smooth = 0.0  # Smoothed version for output
        self.gantry_initialized = False  # Set true after reading from telemetry
        self.gantry_homed = False        # Set true after homing
        self.gantry_countdown = 0.0      # Countdown timer (seconds remaining)

        # Keyboard smoothing: target values and ramp rates
        self.linear_target = 0.0
        self.angular_target = 0.0
        self.keyboard_accel_rate = 50.0  # units/sec - instant response (~1 tick)
        self.keyboard_decel_rate = 8.0   # units/sec - smooth deceleration

        # Swivel position control flags (one-shot)
        self.swivel_inc = False
        self.swivel_dec = False

        # Track when each key was first pressed and last seen
        self.w_first_press_time = 0.0
        self.s_first_press_time = 0.0
        self.a_first_press_time = 0.0
        self.d_first_press_time = 0.0
        self.last_w_time = 0.0
        self.last_s_time = 0.0
        self.last_a_time = 0.0
        self.last_d_time = 0.0
        self.w_repeat_count = 0
        self.s_repeat_count = 0
        self.a_repeat_count = 0
        self.d_repeat_count = 0

        # Timeout settings
        self.key_timeout_short = 0.12    # 120ms - base timeout for release detection
        self.key_timeout_active = 0.08   # 80ms - fast release during active repeat
        self.tap_window = 0.2            # 200ms - quick tap detection window
        self.repeat_delay_end = 0.65     # 650ms - OS repeat should start by this time
        self.repeat_active_threshold = 2 # Need 2+ events to confirm repeat is active

        # One-shot flags — consumed after dispatch
        self.enable_all = False
        self.disable_all = False
        self.emergency_stop = False
        self.clear_estop = False
        self.clear_faults = False
        self.zero_positions = False
        self.quit = False

        # Timestamp set whenever gantry_initialized is forced to False (disable/estop/enable).
        # Auto-init is only allowed to fire when comms.last_telemetry_time is NEWER than
        # this value, preventing stale "running" telemetry (captured at the start of the
        # same tick) from triggering a false init on the exact tick the reset was issued.
        # Initialized to -1.0 so the first real telemetry timestamp (always > 0) passes
        # the guard on fresh startup before any disable/enable has been issued.
        self.gantry_init_reset_time = -1.0

        # Gamepad previous-frame button state for rising-edge detection.
        # Prevents multiple enable/disable commands from queuing when a button is held.
        self.prev_gamepad_enable = False
        self.prev_gamepad_disable = False

        # Safe shutdown state (X key)
        self.safe_shutdown = False
        self.shutdown_countdown = 0.0  # Seconds remaining in countdown
        self.shutdown_active = False   # True when moving to zero positions

        # Safe disable confirmation (E key)
        self.disable_confirm_pending = False
        self.disable_confirm_time = 0.0  # Time when warning was shown

        # Gantry control flags (one-shot)
        self.gantry_base_dec = False
        self.gantry_base_inc = False
        self.gantry_mid_dec = False
        self.gantry_mid_inc = False
        self.gantry_end_dec = False
        self.gantry_end_inc = False
        self.gantry_home = False  # Set current positions as home

        # Wrist and gripper joint positions (relative to home)
        self.wrist_pitch = 0.0
        self.wrist_roll = 0.0
        self.gripper = 0.0
        self.wrist_pitch_offset = 0.0
        self.wrist_roll_offset = 0.0
        self.gripper_offset = 0.0
        self.wrist_pitch_dec = False
        self.wrist_pitch_inc = False
        self.wrist_roll_dec = False
        self.wrist_roll_inc = False
        self.gripper_dec = False
        self.gripper_inc = False

        self.gamepad_connected = False

        # Gamepad raw state for display
        self.gamepad_axes = {}
        self.gamepad_buttons = {}


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def apply_deadzone(value, deadzone):
    """Zero out values inside deadzone, rescale the rest to full range."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def apply_curve(value, exponent):
    """Apply exponential response curve preserving sign."""
    sign = 1.0 if value >= 0 else -1.0
    return sign * (abs(value) ** exponent)


# ---------------------------------------------------------------------------
# Gamepad
# ---------------------------------------------------------------------------

def init_joystick():
    """Initialise pygame and return the first joystick, or None."""
    if not _pygame_available:
        return None
    pygame.init()
    pygame.joystick.init()
    # On Windows, SDL needs a display surface for full joystick input.
    # Create a tiny hidden window that won't interfere with curses.
    if sys.platform == "win32":
        os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "-10000,-10000")
        pygame.display.set_mode((1, 1))
    if pygame.joystick.get_count() == 0:
        return None
    # Pick first device that isn't a keyboard/non-gamepad
    chosen = 0
    for i in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(i)
        js.init()
        name = js.get_name().lower()
        if js.get_numaxes() >= 2 and "keychron" not in name and "keyboard" not in name:
            chosen = i
            break
    js = pygame.joystick.Joystick(chosen)
    js.init()
    return js


def read_gamepad(joystick, cfg, state):
    """Read gamepad axes/buttons into *state*. Call once per tick."""
    if joystick is None:
        state.gamepad_connected = False
        return

    # get() drains the event queue — needed on Windows for full input
    pygame.event.get()

    state.gamepad_connected = True
    gp = cfg["gamepad"]
    axes = gp["axes"]
    buttons = gp["buttons"]
    invert = gp.get("axis_invert", {})
    deadzone = cfg["control"]["deadzone"]

    # --- axes ---
    # WORKAROUND: Motor controller has linear/angular backwards
    # Swapping axes here so controls feel right to user
    # TODO: Fix in rust/motor_control instead
    raw_y = joystick.get_axis(axes["left_stick_y"])
    if invert.get("left_stick_y", False):
        raw_y = -raw_y
    raw_x = joystick.get_axis(axes["left_stick_x"])
    if invert.get("left_stick_x", False):
        raw_x = -raw_x
    raw_swivel = joystick.get_axis(axes["right_stick_x"])
    if invert.get("right_stick_x", False):
        raw_swivel = -raw_swivel
    raw_ry = joystick.get_axis(axes["right_stick_y"])
    if invert.get("right_stick_y", False):
        raw_ry = -raw_ry

    # Store raw values for display
    state.gamepad_axes = {
        "left_stick_x": raw_x,
        "left_stick_y": raw_y,
        "right_stick_x": raw_swivel,
        "right_stick_y": raw_ry,
    }

    # Y-axis (forward/back) mapped to angular (makes robot turn)
    # X-axis (left/right) mapped to linear (makes robot go forward/back)
    state.angular = apply_curve(
        apply_deadzone(raw_y, deadzone),
        cfg["drive"]["angular_exponent"],
    )
    state.linear = apply_curve(
        apply_deadzone(raw_x, deadzone),
        cfg["drive"]["linear_exponent"],
    )
    # Swivel is now position-controlled via Z/C keys (no gamepad velocity control)
    # Right stick Y → gantry base velocity
    state.gantry_base_rate = apply_curve(
        apply_deadzone(raw_ry, deadzone),
        cfg["drive"]["linear_exponent"],
    )

    # --- buttons (one-shot on press) ---
    # Store button states for display
    state.gamepad_buttons = {
        "a": joystick.get_button(buttons["a"]),
        "b": joystick.get_button(buttons["b"]),
        "back": joystick.get_button(buttons["back"]),
        "start": joystick.get_button(buttons["start"]),
    }

    # Rising-edge detection for enable/disable: only fire on the tick the button
    # transitions from not-pressed to pressed.  Without this, holding A for >50ms
    # queues multiple Enable commands that Rust processes sequentially, causing
    # repeated ~280ms disable→enable cycles per motor while the arm is live.
    enable_pressed  = state.gamepad_buttons["a"]
    disable_pressed = state.gamepad_buttons["b"]
    if enable_pressed and not state.prev_gamepad_enable:
        state.enable_all = True
    if disable_pressed and not state.prev_gamepad_disable:
        state.disable_all = True
    state.prev_gamepad_enable  = enable_pressed
    state.prev_gamepad_disable = disable_pressed

    if state.gamepad_buttons["back"]:
        state.emergency_stop = True
    if state.gamepad_buttons["start"]:
        state.clear_estop = True
        state.clear_faults = True


def read_keyboard_pygame(state):
    """Read keyboard state using pygame (true key state, not events).

    Only called when pygame is available. Provides instant response
    without timeout hacks since we can detect key up/down directly.
    """
    if not _pygame_available:
        return

    # Need to pump events for keyboard state to update
    pygame.event.pump()
    keys = pygame.key.get_pressed()

    # WORKAROUND: Motor controller has linear/angular backwards
    # W/S maps to angular (forward/back)
    if keys[pygame.K_w]:
        state.angular_target = 1.0
    elif keys[pygame.K_s]:
        state.angular_target = -1.0
    else:
        state.angular_target = 0.0

    # A/D maps to linear (turn)
    if keys[pygame.K_d]:
        state.linear_target = 1.0
    elif keys[pygame.K_a]:
        state.linear_target = -1.0
    else:
        state.linear_target = 0.0

    # Z/C for swivel position control - handled via events in main loop, not here


def read_keyboard(key, state, current_time, skip_movement_keys=False):
    """Process a single curses key code into *state*.

    Tracks first press time and repeat count for each key to determine
    if keyboard repeat is active.

    Args:
        skip_movement_keys: If True, ignore WASD (used when pygame handles movement)
    """
    # WORKAROUND: Motor controller has linear/angular backwards
    # Swapping keys here so controls feel right to user
    # TODO: Fix in rust/motor_control instead
    if not skip_movement_keys and (key == ord("w") or key == ord("W")):
        if state.w_repeat_count == 0:
            state.w_first_press_time = current_time
        state.last_w_time = current_time
        state.w_repeat_count += 1
        state.angular_target = 1.0
    elif not skip_movement_keys and (key == ord("s") or key == ord("S")):
        if state.s_repeat_count == 0:
            state.s_first_press_time = current_time
        state.last_s_time = current_time
        state.s_repeat_count += 1
        state.angular_target = -1.0
    elif not skip_movement_keys and (key == ord("a") or key == ord("A")):
        if state.a_repeat_count == 0:
            state.a_first_press_time = current_time
        state.last_a_time = current_time
        state.a_repeat_count += 1
        state.linear_target = -1.0
    elif not skip_movement_keys and (key == ord("d") or key == ord("D")):
        if state.d_repeat_count == 0:
            state.d_first_press_time = current_time
        state.last_d_time = current_time
        state.d_repeat_count += 1
        state.linear_target = 1.0
    elif key == ord("e") or key == ord("E"):
        state.enable_all = True
    elif key == ord("q") or key == ord("Q"):
        # Safe disable: requires confirmation
        if state.disable_confirm_pending:
            # Second press - confirm disable
            logger.info("Q pressed - confirming disable")
            state.disable_all = True
            state.disable_confirm_pending = False
        else:
            # First press - show warning
            logger.info("Q pressed - showing disable warning")
            state.disable_confirm_pending = True
            state.disable_confirm_time = current_time
    elif key == ord("x") or key == ord("X"):
        # Safe shutdown: countdown then move to zero
        if not state.shutdown_active and state.shutdown_countdown <= 0:
            state.safe_shutdown = True
            state.shutdown_countdown = 3.0  # 3 second countdown
    elif key == ord(" "):
        state.emergency_stop = True
    elif key == ord("r") or key == ord("R"):
        state.clear_estop = True
        state.clear_faults = True
    elif key == ord("z") or key == ord("Z"):
        # Z for swivel decrement
        state.swivel_dec = True
    elif key == ord("c") or key == ord("C"):
        # C for swivel increment
        state.swivel_inc = True
    elif key == ord("0"):
        # 0 key for zero positions (moved from Z)
        state.zero_positions = True
    elif key == ord("1"):
        state.gantry_base_dec = True
    elif key == ord("2"):
        state.gantry_base_inc = True
    elif key == ord("3"):
        state.gantry_mid_dec = True
    elif key == ord("4"):
        state.gantry_mid_inc = True
    elif key == ord("5"):
        state.gantry_end_dec = True
    elif key == ord("6"):
        state.gantry_end_inc = True
    elif key == ord("7"):
        state.wrist_pitch_dec = True
    elif key == ord("8"):
        state.wrist_pitch_inc = True
    elif key == ord("["):
        state.wrist_roll_dec = True
    elif key == ord("]"):
        state.wrist_roll_inc = True
    elif key == ord("-"):
        state.gripper_dec = True
    elif key == ord("="):
        state.gripper_inc = True
    elif key == ord("h") or key == ord("H"):
        state.gantry_home = True
    elif key == 27:  # Escape
        state.quit = True


def smooth_keyboard_inputs(state, dt):
    """Smoothly ramp keyboard inputs toward target values.

    Uses fast acceleration (instant response on key press) and
    slow deceleration (smooth release) for better control feel.

    Args:
        state: InputState with current and target values
        dt: Time delta since last update (seconds)
    """
    def ramp_toward(current, target, accel_rate, decel_rate, dt):
        """Ramp current value toward target with asymmetric rates."""
        diff = target - current

        # Choose rate based on whether we're accelerating or decelerating
        # Accelerating: target magnitude > current magnitude
        # Decelerating: target magnitude < current magnitude
        is_accelerating = abs(target) > abs(current)
        rate = accel_rate if is_accelerating else decel_rate

        max_change = rate * dt
        if abs(diff) <= max_change:
            return target
        return current + max_change if diff > 0 else current - max_change

    # Ramp each axis toward its target with asymmetric rates
    state.linear = ramp_toward(state.linear, state.linear_target,
                               state.keyboard_accel_rate, state.keyboard_decel_rate, dt)
    state.angular = ramp_toward(state.angular, state.angular_target,
                                state.keyboard_accel_rate, state.keyboard_decel_rate, dt)


# ---------------------------------------------------------------------------
# Comms
# ---------------------------------------------------------------------------

class Comms:
    """ZeroMQ command (PUSH) and telemetry (SUB) sockets."""

    def __init__(self, cmd_addr, telem_addr, ups_telem_addr=None):
        self.cmd_addr = cmd_addr
        self.telem_addr = telem_addr
        self.ups_telem_addr = ups_telem_addr
        self.ctx = zmq.Context()

        # CRITICAL: Set linger to 0 to prevent hanging on close
        self.cmd = self.ctx.socket(zmq.PUSH)
        self.cmd.setsockopt(zmq.LINGER, 0)
        self.cmd.setsockopt(zmq.SNDTIMEO, 1000)  # 1 second send timeout

        try:
            self.cmd.connect(cmd_addr)
            logger.info(f"Connected to command endpoint: {cmd_addr}")
        except zmq.ZMQError as e:
            logger.error(f"Failed to connect to command endpoint {cmd_addr}: {e}")
            raise

        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.setsockopt(zmq.RCVTIMEO, 50)

        try:
            self.sub.connect(telem_addr)
            logger.info(f"Connected to telemetry endpoint: {telem_addr}")
        except zmq.ZMQError as e:
            logger.error(f"Failed to connect to telemetry endpoint {telem_addr}: {e}")
            self.cmd.close()
            raise

        # Optional UPS telemetry subscriber
        self.ups_sub = None
        self.last_ups_telemetry = None
        self.last_ups_telemetry_time = 0.0
        if ups_telem_addr:
            try:
                self.ups_sub = self.ctx.socket(zmq.SUB)
                self.ups_sub.setsockopt(zmq.LINGER, 0)
                self.ups_sub.setsockopt_string(zmq.SUBSCRIBE, "")
                self.ups_sub.setsockopt(zmq.RCVTIMEO, 50)
                self.ups_sub.connect(ups_telem_addr)
                logger.info(f"Connected to UPS telemetry endpoint: {ups_telem_addr}")
            except zmq.ZMQError as e:
                logger.warning(f"Failed to connect to UPS telemetry endpoint {ups_telem_addr}: {e}")
                if self.ups_sub:
                    self.ups_sub.close()
                self.ups_sub = None

        self.commands_sent = 0
        self.last_telemetry = None
        self.last_telemetry_time = 0.0
        self.connected = True
        self.telemetry_stale_threshold = 0.5  # 500ms

        # Telemetry rate tracking
        self.telem_count = 0
        self.telem_rate_window_start = time.monotonic()
        self.telem_rate = 0.0

    # -- send helpers -------------------------------------------------------

    def send(self, msg):
        """Send command with error handling."""
        try:
            self.cmd.send_string(json.dumps(msg), zmq.NOBLOCK)
            self.commands_sent += 1
            return True
        except zmq.Again:
            logger.warning(f"Send timeout - command queue full")
            return False
        except zmq.ZMQError as e:
            logger.error(f"Failed to send command: {e}")
            self.connected = False
            return False

    def send_drive(self, linear, angular, kp=0.0, kd=3.0):
        self.send({
            "type": "drive",
            "linear": linear,
            "angular": angular,
            "kp": kp,
            "kd": kd
        })

    def send_swivel_position(self, position, kp=5.0, kd=0.5):
        self.send({
            "type": "swivel",
            "position": position,
            "kp": kp,
            "kd": kd
        })

    def send_enable(self, motor_ids):
        self.send({"type": "enable", "motor_ids": motor_ids})

    def send_disable(self, motor_ids):
        self.send({"type": "disable", "motor_ids": motor_ids})

    def send_emergency_stop(self):
        self.send({"type": "emergency_stop"})

    def send_clear_estop(self):
        self.send({"type": "clear_emergency_stop"})

    def send_clear_faults(self, motor_ids):
        self.send({"type": "clear_fault", "motor_ids": motor_ids})

    def send_zero_position(self, motor_ids):
        self.send({"type": "zero_position", "motor_ids": motor_ids})

    def send_arm_joints(self, positions, velocities=None, kp=None, kd=None):
        """Send arm/gantry joint command with position targets."""
        cmd = {
            "type": "arm_joints",
            "positions": positions,
            "velocities": velocities or [0.0] * len(positions),
        }
        if kp:
            cmd["kp"] = kp
        if kd:
            cmd["kd"] = kd
        self.send(cmd)

    # -- receive ------------------------------------------------------------

    def recv_latest_telemetry(self):
        """Drain SUB socket, keep only the newest message."""
        latest = None
        try:
            while True:
                try:
                    raw = self.sub.recv_string(zmq.NOBLOCK)
                    latest = json.loads(raw)
                    self.telem_count += 1
                except zmq.Again:
                    break
                except json.JSONDecodeError as e:
                    logger.error(f"Invalid telemetry JSON: {e}")
                    break
            if latest is not None:
                # Validate telemetry structure
                if not isinstance(latest, dict):
                    logger.warning(f"Invalid telemetry format: expected dict, got {type(latest)}")
                    return self.last_telemetry
                self.last_telemetry = latest
                self.last_telemetry_time = time.monotonic()

            # Update telemetry rate every second
            now = time.monotonic()
            elapsed = now - self.telem_rate_window_start
            if elapsed >= 1.0:
                self.telem_rate = self.telem_count / elapsed
                self.telem_count = 0
                self.telem_rate_window_start = now
        except zmq.ZMQError as e:
            logger.error(f"Telemetry receive error: {e}")
            self.connected = False
        return self.last_telemetry

    def recv_latest_ups_telemetry(self):
        """Drain UPS SUB socket, keep only the newest message."""
        if not self.ups_sub:
            return self.last_ups_telemetry

        latest = None
        try:
            while True:
                try:
                    raw = self.ups_sub.recv_string(zmq.NOBLOCK)
                    latest = json.loads(raw)
                except zmq.Again:
                    break
                except json.JSONDecodeError as e:
                    logger.error(f"Invalid UPS telemetry JSON: {e}")
                    break
            if latest is not None:
                if not isinstance(latest, dict):
                    logger.warning(f"Invalid UPS telemetry format: expected dict, got {type(latest)}")
                    return self.last_ups_telemetry
                self.last_ups_telemetry = latest
                self.last_ups_telemetry_time = time.monotonic()
        except zmq.ZMQError as e:
            logger.error(f"UPS telemetry receive error: {e}")
        return self.last_ups_telemetry

    def is_telemetry_stale(self):
        """Check if telemetry is stale (no recent updates)."""
        if self.last_telemetry_time == 0.0:
            return True  # Never received telemetry
        age = time.monotonic() - self.last_telemetry_time
        return age > self.telemetry_stale_threshold

    def get_telemetry_age(self):
        """Get age of last telemetry in seconds."""
        if self.last_telemetry_time == 0.0:
            return float('inf')
        return time.monotonic() - self.last_telemetry_time

    # -- cleanup ------------------------------------------------------------

    def close(self):
        """Close sockets and context without hanging."""
        logger.info("Closing ZeroMQ connections...")
        try:
            # Set linger to 0 again to ensure no blocking
            self.cmd.setsockopt(zmq.LINGER, 0)
            self.sub.setsockopt(zmq.LINGER, 0)
            if self.ups_sub:
                self.ups_sub.setsockopt(zmq.LINGER, 0)
        except zmq.ZMQError:
            pass  # Socket might already be closed

        try:
            self.cmd.close()
            logger.debug("Command socket closed")
        except zmq.ZMQError as e:
            logger.warning(f"Error closing command socket: {e}")

        try:
            self.sub.close()
            logger.debug("Telemetry socket closed")
        except zmq.ZMQError as e:
            logger.warning(f"Error closing telemetry socket: {e}")

        if self.ups_sub:
            try:
                self.ups_sub.close()
                logger.debug("UPS telemetry socket closed")
            except zmq.ZMQError as e:
                logger.warning(f"Error closing UPS telemetry socket: {e}")

        try:
            # Terminate context with no linger - should not block now
            self.ctx.term()
            logger.info("ZeroMQ context terminated")
        except zmq.ZMQError as e:
            logger.error(f"Error terminating ZeroMQ context: {e}")


class MultiModuleComms:
    """Multi-module ZeroMQ communication for distributed robot architecture."""

    def __init__(self, modules_config):
        self.ctx = zmq.Context()
        self.modules = {}
        self.commands_sent = 0
        self.telemetry_stale_threshold = 0.5  # 500ms

        for module_name, cfg in modules_config.items():
            logger.info(f"Initializing module '{module_name}'...")

            # Command socket with linger=0 to prevent hanging
            cmd_sock = self.ctx.socket(zmq.PUSH)
            cmd_sock.setsockopt(zmq.LINGER, 0)
            cmd_sock.setsockopt(zmq.SNDTIMEO, 1000)  # 1 second timeout

            try:
                cmd_sock.connect(cfg["command"])
                logger.info(f"  Command: {cfg['command']}")
            except zmq.ZMQError as e:
                logger.error(f"  Failed to connect to command endpoint: {e}")
                cmd_sock.close()
                raise

            # Telemetry socket with linger=0
            telem_sock = self.ctx.socket(zmq.SUB)
            telem_sock.setsockopt(zmq.LINGER, 0)
            telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            telem_sock.setsockopt(zmq.RCVTIMEO, 50)

            try:
                telem_sock.connect(cfg["telemetry"])
                logger.info(f"  Telemetry: {cfg['telemetry']}")
            except zmq.ZMQError as e:
                logger.error(f"  Failed to connect to telemetry endpoint: {e}")
                cmd_sock.close()
                telem_sock.close()
                raise

            self.modules[module_name] = {
                "cmd": cmd_sock,
                "telem": telem_sock,
                "motors": cfg["motors"],
                "last_telemetry": None,
                "last_telemetry_time": 0.0,
                "connected": True,
                "cmd_addr": cfg["command"],
                "telem_addr": cfg["telemetry"],
                "telem_count": 0,
                "telem_rate_window_start": time.monotonic(),
                "telem_rate": 0.0,
            }

    # -- send helpers -------------------------------------------------------

    def send_command(self, module_name, cmd_msg):
        """Send command to specific module with error handling."""
        if module_name not in self.modules:
            logger.warning(f"Module '{module_name}' not found")
            return False

        mod = self.modules[module_name]
        try:
            mod["cmd"].send_string(json.dumps(cmd_msg), zmq.NOBLOCK)
            self.commands_sent += 1
            return True
        except zmq.Again:
            logger.warning(f"Send timeout to module '{module_name}' - queue full")
            return False
        except zmq.ZMQError as e:
            logger.error(f"Failed to send command to '{module_name}': {e}")
            mod["connected"] = False
            return False

    def send_drive(self, linear, angular, kp=0.0, kd=3.0):
        """Send drive command to rover module with tunable MIT mode gains (wheels only)."""
        self.send_command("rover", {
            "type": "drive",
            "linear": linear,
            "angular": angular,
            "kp": kp,
            "kd": kd
        })

    def send_swivel_position(self, position, kp=5.0, kd=0.5):
        """Send swivel position command to rover module."""
        self.send_command("rover", {
            "type": "swivel",
            "position": position,
            "kp": kp,
            "kd": kd
        })

    def send_arm_joints(self, positions, velocities=None, kp=None, kd=None):
        """Send arm joint command to arm module."""
        cmd = {
            "type": "arm_joints",
            "positions": positions,
            "velocities": velocities or [0.0] * len(positions),
        }
        if kp:
            cmd["kp"] = kp
        if kd:
            cmd["kd"] = kd
        self.send_command("arm", cmd)

    def send_enable(self, motor_ids):
        """Enable motors across all relevant modules."""
        for module_name, mod in self.modules.items():
            motors_to_enable = [m for m in motor_ids if m in mod["motors"]]
            if motors_to_enable:
                self.send_command(module_name, {
                    "type": "enable",
                    "motor_ids": motors_to_enable
                })

    def send_disable(self, motor_ids):
        """Disable motors across all relevant modules."""
        for module_name, mod in self.modules.items():
            motors_to_disable = [m for m in motor_ids if m in mod["motors"]]
            if motors_to_disable:
                self.send_command(module_name, {
                    "type": "disable",
                    "motor_ids": motors_to_disable
                })

    def send_emergency_stop(self):
        """Send emergency stop to all modules."""
        for module_name in self.modules:
            self.send_command(module_name, {"type": "emergency_stop"})

    def send_clear_estop(self):
        """Clear emergency stop on all modules."""
        for module_name in self.modules:
            self.send_command(module_name, {"type": "clear_emergency_stop"})

    def send_clear_faults(self, motor_ids):
        """Clear faults for motors across all relevant modules."""
        for module_name, mod in self.modules.items():
            motors_to_clear = [m for m in motor_ids if m in mod["motors"]]
            if motors_to_clear:
                self.send_command(module_name, {
                    "type": "clear_fault",
                    "motor_ids": motors_to_clear
                })

    def send_zero_position(self, motor_ids):
        """Zero position for motors across all relevant modules."""
        for module_name, mod in self.modules.items():
            motors_to_zero = [m for m in motor_ids if m in mod["motors"]]
            if motors_to_zero:
                self.send_command(module_name, {
                    "type": "zero_position",
                    "motor_ids": motors_to_zero
                })

    # -- receive ------------------------------------------------------------

    def recv_latest_telemetry(self):
        """Drain telemetry from all modules with error handling."""
        for module_name, mod in self.modules.items():
            try:
                while True:
                    try:
                        raw = mod["telem"].recv_string(zmq.NOBLOCK)
                        telem = json.loads(raw)
                        mod["telem_count"] += 1
                        # Validate telemetry structure
                        if not isinstance(telem, dict):
                            logger.warning(f"Invalid telemetry from '{module_name}': expected dict")
                            break
                        mod["last_telemetry"] = telem
                        mod["last_telemetry_time"] = time.monotonic()
                    except zmq.Again:
                        break
                    except json.JSONDecodeError as e:
                        logger.error(f"Invalid JSON from '{module_name}': {e}")
                        break

                # Update telemetry rate every second
                now = time.monotonic()
                elapsed = now - mod["telem_rate_window_start"]
                if elapsed >= 1.0:
                    mod["telem_rate"] = mod["telem_count"] / elapsed
                    mod["telem_count"] = 0
                    mod["telem_rate_window_start"] = now
            except zmq.ZMQError as e:
                logger.error(f"Telemetry receive error from '{module_name}': {e}")
                mod["connected"] = False

    @property
    def last_telemetry(self):
        """Merged telemetry from all modules (cached to avoid rebuilding every call)."""
        merged = {"timestamp": time.time(), "motors": {}}
        for module_name, mod in self.modules.items():
            if mod["last_telemetry"] and "motors" in mod["last_telemetry"]:
                merged["motors"].update(mod["last_telemetry"]["motors"])
        return merged if merged["motors"] else None

    @property
    def last_telemetry_time(self):
        """Most recent telemetry timestamp across all modules."""
        times = [m["last_telemetry_time"] for m in self.modules.values()]
        return max(times) if times else 0.0

    def is_telemetry_stale(self):
        """Check if any module has stale telemetry."""
        for mod in self.modules.values():
            if mod["last_telemetry_time"] == 0.0:
                return True
            age = time.monotonic() - mod["last_telemetry_time"]
            if age > self.telemetry_stale_threshold:
                return True
        return False

    def get_telemetry_age(self):
        """Get age of oldest telemetry across all modules."""
        ages = []
        for mod in self.modules.values():
            if mod["last_telemetry_time"] == 0.0:
                return float('inf')
            ages.append(time.monotonic() - mod["last_telemetry_time"])
        return max(ages) if ages else float('inf')

    def get_module_status(self):
        """Get connection status for all modules."""
        status = {}
        for module_name, mod in self.modules.items():
            age = time.monotonic() - mod["last_telemetry_time"] if mod["last_telemetry_time"] > 0 else float('inf')
            status[module_name] = {
                "connected": mod["connected"],
                "telemetry_age": age,
                "stale": age > self.telemetry_stale_threshold or mod["last_telemetry_time"] == 0,
                "telem_rate": mod["telem_rate"],
            }
        return status

    # -- cleanup ------------------------------------------------------------

    def close(self):
        """Close all module sockets and context without hanging."""
        logger.info("Closing multi-module ZeroMQ connections...")
        for module_name, mod in self.modules.items():
            logger.debug(f"Closing module '{module_name}'...")
            try:
                # Set linger to 0 to prevent blocking
                mod["cmd"].setsockopt(zmq.LINGER, 0)
                mod["telem"].setsockopt(zmq.LINGER, 0)
            except zmq.ZMQError:
                pass

            try:
                mod["cmd"].close()
                logger.debug(f"  Command socket closed")
            except zmq.ZMQError as e:
                logger.warning(f"  Error closing command socket: {e}")

            try:
                mod["telem"].close()
                logger.debug(f"  Telemetry socket closed")
            except zmq.ZMQError as e:
                logger.warning(f"  Error closing telemetry socket: {e}")

        try:
            self.ctx.term()
            logger.info("ZeroMQ context terminated")
        except zmq.ZMQError as e:
            logger.error(f"Error terminating ZeroMQ context: {e}")


# ---------------------------------------------------------------------------
# Command dispatch
# ---------------------------------------------------------------------------

def dispatch_commands(state, comms, cfg, dt=0.05):
    """Send one-shot actions then drive command. Consumes flags."""
    all_motors = cfg["motors"]["all"]

    # --- one-shots ---------------------------------------------------------
    sent_estop = False

    if state.emergency_stop:
        comms.send_emergency_stop()
        state.emergency_stop = False
        sent_estop = True
        # Force re-read of actual positions on next enable.
        # gantry_init_reset_time prevents the auto-init block from firing on this
        # same tick with stale "running" telemetry from recv_latest_telemetry().
        state.gantry_initialized = False
        state.gantry_homed = False
        state.gantry_init_reset_time = time.monotonic()
        state.swivel_initialized = False

    if state.clear_estop:
        comms.send_clear_estop()
        state.clear_estop = False

    if state.clear_faults:
        comms.send_clear_faults(all_motors)
        state.clear_faults = False

    if state.enable_all:
        comms.send_enable(all_motors)
        state.enable_all = False
        # Reset gantry init so auto-init fires fresh with post-enable telemetry.
        # This clears any stale offsets set by the same-tick false init that occurs
        # on the disable tick (see gantry_init_reset_time for the guard mechanism).
        state.gantry_initialized = False
        state.gantry_homed = False
        state.gantry_init_reset_time = time.monotonic()
        state.swivel_initialized = False
        # Immediately follow with a zero-impedance arm command at the current actual
        # positions.  This overwrites any stale target cached in the Rust arm loop,
        # preventing motors from spiking to their pre-disable position when the arm
        # was physically moved while disabled.  Kp=0 / Kd=0 means zero torque
        # regardless of any position error, so the command is safe even if telemetry
        # is slightly stale.  Auto-init (below) will re-seed offsets and enable full
        # Kp/Kd control on the next loop iteration that sees "running" motor state.
        if (comms.last_telemetry and "motors" in comms.last_telemetry
                and "gantry" in cfg.get("motors", {})):
            _telem = comms.last_telemetry["motors"]
            _gjoints = cfg["motors"]["gantry"]
            _seed_pos = [(_telem.get(j) or {}).get("position", 0.0) for j in _gjoints]
            comms.send_arm_joints(
                _seed_pos,
                [0.0] * len(_gjoints),
                kp=[0.0] * len(_gjoints),
                kd=[0.0] * len(_gjoints),
            )

    if state.disable_all:
        logger.info(f"Disabling all motors: {all_motors}")
        comms.send_disable(all_motors)
        state.disable_all = False
        # Force re-read of actual positions on next enable so motors don't
        # snap back to stale targets if the arm was moved while disabled.
        # gantry_init_reset_time prevents the auto-init block from firing on this
        # same tick with stale "running" telemetry from recv_latest_telemetry().
        state.gantry_initialized = False
        state.gantry_homed = False
        state.gantry_init_reset_time = time.monotonic()
        state.swivel_initialized = False

    if state.zero_positions:
        comms.send_zero_position(all_motors)
        state.zero_positions = False

    # --- safe disable confirmation timeout (Q key) -------------------------
    if state.disable_confirm_pending:
        # Cancel if timeout (5 seconds) or if any other key pressed
        if (time.monotonic() - state.disable_confirm_time) > 5.0:
            state.disable_confirm_pending = False
            logger.info("Disable confirmation timeout - cancelled")

    # --- safe shutdown sequence (X key) ------------------------------------
    if state.shutdown_countdown > 0:
        state.shutdown_countdown -= dt
        if state.shutdown_countdown <= 0:
            # Countdown finished, start moving to zero
            state.shutdown_active = True
            logger.info("Safe shutdown: Moving gantry to zero positions...")

    if state.shutdown_active and state.gantry_initialized:
        # Move gantry slowly to zero positions
        shutdown_speed = 0.2  # rad/s - slow and safe
        max_change = shutdown_speed * dt

        # Move each joint toward zero
        for joint_name, current_pos in [
            ("base", state.gantry_base),
            ("mid", state.gantry_mid),
            ("end", state.gantry_end),
            ("wrist_pitch", state.wrist_pitch),
            ("wrist_roll", state.wrist_roll),
            ("gripper", state.gripper),
        ]:
            if abs(current_pos) > 0.01:  # Not at zero yet
                if abs(current_pos) < max_change:
                    # Close enough, snap to zero
                    if joint_name == "base":
                        state.gantry_base = 0.0
                    elif joint_name == "mid":
                        state.gantry_mid = 0.0
                    elif joint_name == "end":
                        state.gantry_end = 0.0
                    elif joint_name == "wrist_pitch":
                        state.wrist_pitch = 0.0
                    elif joint_name == "wrist_roll":
                        state.wrist_roll = 0.0
                    elif joint_name == "gripper":
                        state.gripper = 0.0
                else:
                    # Move toward zero
                    step = -max_change if current_pos > 0 else max_change
                    if joint_name == "base":
                        state.gantry_base += step
                    elif joint_name == "mid":
                        state.gantry_mid += step
                    elif joint_name == "end":
                        state.gantry_end += step
                    elif joint_name == "wrist_pitch":
                        state.wrist_pitch += step
                    elif joint_name == "wrist_roll":
                        state.wrist_roll += step
                    elif joint_name == "gripper":
                        state.gripper += step

        # Check if all at zero
        if (abs(state.gantry_base) < 0.01 and abs(state.gantry_mid) < 0.01 and
                abs(state.gantry_end) < 0.01 and abs(state.wrist_pitch) < 0.01 and
                abs(state.wrist_roll) < 0.01 and abs(state.gripper) < 0.01):
            # All at zero, disable motors
            logger.info("Safe shutdown: Gantry at zero, disabling motors...")
            comms.send_disable(cfg["motors"]["all"])
            state.shutdown_active = False
            state.safe_shutdown = False
            state.gantry_initialized = False
            state.gantry_homed = False

    # --- swivel position control (auto-initialize from telemetry) ----------
    if not state.swivel_initialized and comms.last_telemetry and "motors" in comms.last_telemetry:
        motors = comms.last_telemetry["motors"]
        if "swivel" in motors and motors["swivel"]:
            motor_state = motors["swivel"].get("state", "disabled")
            if motor_state == "running" or motor_state == "enabled":
                # Initialize swivel position from telemetry
                state.swivel_offset = motors["swivel"].get("position", 0.0)
                state.swivel_position = 0.0  # Relative position starts at 0
                state.swivel_initialized = True
                logger.info(f"Swivel auto-homed at encoder position: {state.swivel_offset:.3f}")

    # Handle swivel position adjustments (Z/C keys)
    swivel_increment = cfg.get("drive", {}).get("swivel_increment", 0.1)  # rad per key press
    if state.swivel_initialized:
        if state.swivel_dec:
            state.swivel_position -= swivel_increment
            state.swivel_dec = False
        if state.swivel_inc:
            state.swivel_position += swivel_increment
            state.swivel_inc = False

    # --- gantry control (incremental position adjustments) ----------------
    gantry_changed = False
    if "gantry" in cfg:
        # Auto-initialize: When motors are first enabled, read their actual positions
        # to prevent commanding 0,0,0 on startup.
        #
        # Guard: comms.last_telemetry_time > state.gantry_init_reset_time
        # recv_latest_telemetry() runs BEFORE dispatch_commands() each tick, so
        # last_telemetry_time is always older than any reset_time issued during
        # this tick.  This prevents stale "running" telemetry (captured at the
        # start of the tick that sent disable/enable/estop) from triggering a
        # false init on the same tick as the reset.
        if (not state.gantry_initialized
                and comms.last_telemetry
                and comms.last_telemetry_time > state.gantry_init_reset_time
                and "motors" in comms.last_telemetry):
            motors = comms.last_telemetry["motors"]
            gantry_motor_ids = cfg["motors"].get("gantry", [])

            # Check if any gantry motors are enabled and have valid telemetry
            has_any_enabled = False
            for mid in gantry_motor_ids:
                if mid in motors and motors[mid]:
                    motor_state = motors[mid].get("state", "disabled")
                    if motor_state == "running" or motor_state == "enabled":
                        has_any_enabled = True
                        break

            if has_any_enabled:
                # Initialize home offsets to current absolute positions
                # Relative positions start at 0.0
                if "gantry_base" in motors and motors["gantry_base"]:
                    state.gantry_base_offset = motors["gantry_base"].get("position", 0.0)
                    state.gantry_base = 0.0
                if "gantry_mid" in motors and motors["gantry_mid"]:
                    state.gantry_mid_offset = motors["gantry_mid"].get("position", 0.0)
                    state.gantry_mid = 0.0
                if "gantry_end" in motors and motors["gantry_end"]:
                    state.gantry_end_offset = motors["gantry_end"].get("position", 0.0)
                    state.gantry_end = 0.0
                if "wrist_pitch" in motors and motors["wrist_pitch"]:
                    state.wrist_pitch_offset = motors["wrist_pitch"].get("position", 0.0)
                    state.wrist_pitch = 0.0
                if "wrist_roll" in motors and motors["wrist_roll"]:
                    state.wrist_roll_offset = motors["wrist_roll"].get("position", 0.0)
                    state.wrist_roll = 0.0
                if "gripper" in motors and motors["gripper"]:
                    state.gripper_offset = motors["gripper"].get("position", 0.0)
                    state.gripper = 0.0
                # Reset smoothed joystick rate so a held stick doesn't cause
                # immediate motion on the first command tick after init.
                state.gantry_base_rate_smooth = 0.0
                state.gantry_initialized = True
                state.gantry_homed = True  # Auto-homed on first enable
                logger.info(f"Gantry auto-homed at encoder positions: base={state.gantry_base_offset:.3f}, mid={state.gantry_mid_offset:.3f}, end={state.gantry_end_offset:.3f}")
                logger.info(f"Relative positions: base={state.gantry_base:.3f}, mid={state.gantry_mid:.3f}, end={state.gantry_end:.3f}")

        # Handle home command - re-zero at current position
        if state.gantry_home:
            if comms.last_telemetry and "motors" in comms.last_telemetry:
                motors = comms.last_telemetry["motors"]
                # Check which gantry motors have valid telemetry
                gantry_motor_ids = cfg["motors"].get("gantry", [])
                has_any = False
                for mid in gantry_motor_ids:
                    if mid in motors and motors[mid] and motors[mid].get("state") != "disabled":
                        has_any = True
                        break

                if has_any:
                    # Re-home: update offsets to current absolute positions, reset relative to 0
                    if "gantry_base" in motors and motors["gantry_base"]:
                        abs_pos = motors["gantry_base"].get("position", state.gantry_base_offset)
                        # New offset = old offset + relative position
                        state.gantry_base_offset = state.gantry_base_offset + state.gantry_base
                        state.gantry_base = 0.0
                    if "gantry_mid" in motors and motors["gantry_mid"]:
                        abs_pos = motors["gantry_mid"].get("position", state.gantry_mid_offset)
                        state.gantry_mid_offset = state.gantry_mid_offset + state.gantry_mid
                        state.gantry_mid = 0.0
                    if "gantry_end" in motors and motors["gantry_end"]:
                        abs_pos = motors["gantry_end"].get("position", state.gantry_end_offset)
                        state.gantry_end_offset = state.gantry_end_offset + state.gantry_end
                        state.gantry_end = 0.0
                    if "wrist_pitch" in motors and motors["wrist_pitch"]:
                        state.wrist_pitch_offset = state.wrist_pitch_offset + state.wrist_pitch
                        state.wrist_pitch = 0.0
                    if "wrist_roll" in motors and motors["wrist_roll"]:
                        state.wrist_roll_offset = state.wrist_roll_offset + state.wrist_roll
                        state.wrist_roll = 0.0
                    if "gripper" in motors and motors["gripper"]:
                        state.gripper_offset = state.gripper_offset + state.gripper
                        state.gripper = 0.0
                    state.gantry_initialized = True
                    state.gantry_homed = True
                    # Reset smoothed rate when homing to avoid drift
                    state.gantry_base_rate_smooth = 0.0
                    logger.info(f"Gantry re-homed at encoder positions: base={state.gantry_base_offset:.3f}, mid={state.gantry_mid_offset:.3f}, end={state.gantry_end_offset:.3f}")
                    logger.info(f"Relative positions reset to: base={state.gantry_base:.3f}, mid={state.gantry_mid:.3f}, end={state.gantry_end:.3f}")
                else:
                    logger.warning("Cannot home gantry - no gantry motors enabled or no valid telemetry")
            state.gantry_home = False

        # Send arm_joints commands continuously to keep motors alive
        # Only send after initialization to prevent motors from moving to 0,0,0 on startup
        # Send absolute positions (relative + offset) to motors
        if state.gantry_initialized and hasattr(comms, 'send_arm_joints'):
            max_vel = cfg["gantry"]["max_velocity"]
            base_vel = state.gantry_base_rate_smooth * max_vel
            # Convert relative positions to absolute by adding home offsets
            abs_base = state.gantry_base + state.gantry_base_offset
            abs_mid = state.gantry_mid + state.gantry_mid_offset
            abs_end = state.gantry_end + state.gantry_end_offset
            abs_wrist_pitch = state.wrist_pitch + state.wrist_pitch_offset
            abs_wrist_roll = state.wrist_roll + state.wrist_roll_offset
            abs_gripper = state.gripper + state.gripper_offset
            positions = [abs_base, abs_mid, abs_end, abs_wrist_pitch, abs_wrist_roll, abs_gripper]
            arm_limits = cfg.get("arm_limits")
            if arm_limits:
                _joint_names = ["gantry_base", "gantry_mid", "gantry_end",
                                "wrist_pitch", "wrist_roll", "gripper"]
                for _i, _jname in enumerate(_joint_names):
                    if _jname in arm_limits:
                        _lo, _hi = arm_limits[_jname]
                        positions[_i] = max(_lo, min(_hi, positions[_i]))
            velocities = [base_vel, 0.0, 0.0, 0.0, 0.0, 0.0]
            comms.send_arm_joints(positions, velocities,
                                cfg["gantry"]["kp"],
                                cfg["gantry"]["kd"])

        # Allow gantry movement after initialization (auto-init or explicit homing)
        # Homing (H key) is optional - just confirms current position as reference
        if state.gantry_initialized:
            increment = cfg["gantry"]["increment"]
            max_vel = cfg["gantry"]["max_velocity"]

            # Right joystick Y → gantry base continuous velocity control
            # Smooth the rate with asymmetric ramp (fast accel, smooth decel)
            accel = 8.0  # rate/sec — how fast stick input takes effect
            decel = 4.0  # rate/sec — how smoothly it stops
            diff = state.gantry_base_rate - state.gantry_base_rate_smooth
            is_accel = abs(state.gantry_base_rate) > abs(state.gantry_base_rate_smooth)
            rate = accel if is_accel else decel
            max_change = rate * dt
            if abs(diff) <= max_change:
                state.gantry_base_rate_smooth = state.gantry_base_rate
            else:
                state.gantry_base_rate_smooth += max_change if diff > 0 else -max_change

            if abs(state.gantry_base_rate_smooth) > 0.01:
                state.gantry_base += state.gantry_base_rate_smooth * max_vel * dt
                gantry_changed = True

            if state.gantry_base_dec:
                state.gantry_base -= increment
                state.gantry_base_dec = False
                gantry_changed = True
            if state.gantry_base_inc:
                state.gantry_base += increment
                state.gantry_base_inc = False
                gantry_changed = True

            if state.gantry_mid_dec:
                state.gantry_mid -= increment
                state.gantry_mid_dec = False
                gantry_changed = True
            if state.gantry_mid_inc:
                state.gantry_mid += increment
                state.gantry_mid_inc = False
                gantry_changed = True

            if state.gantry_end_dec:
                state.gantry_end -= increment
                state.gantry_end_dec = False
                gantry_changed = True
            if state.gantry_end_inc:
                state.gantry_end += increment
                state.gantry_end_inc = False
                gantry_changed = True

            if state.wrist_pitch_dec:
                state.wrist_pitch -= increment
                state.wrist_pitch_dec = False
                gantry_changed = True
            if state.wrist_pitch_inc:
                state.wrist_pitch += increment
                state.wrist_pitch_inc = False
                gantry_changed = True

            if state.wrist_roll_dec:
                state.wrist_roll -= increment
                state.wrist_roll_dec = False
                gantry_changed = True
            if state.wrist_roll_inc:
                state.wrist_roll += increment
                state.wrist_roll_inc = False
                gantry_changed = True

            if state.gripper_dec:
                state.gripper -= increment
                state.gripper_dec = False
                gantry_changed = True
            if state.gripper_inc:
                state.gripper += increment
                state.gripper_inc = False
                gantry_changed = True

            # Clamp the target accumulator to physical limits so the user cannot
            # wind it up past a reachable position (which would create a dead zone
            # where key presses appear to do nothing until the accumulator unwinds).
            # With calibration: convert each absolute limit to relative space via
            #   rel_limit = abs_limit - offset
            # and clamp the relative accumulator directly.
            # Without calibration: fall back to ±π from home as a conservative guard.
            if gantry_changed:
                arm_limits = cfg.get("arm_limits")
                if arm_limits:
                    for _jname, _off_attr, _rel_attr in [
                        ("gantry_base",  "gantry_base_offset",  "gantry_base"),
                        ("gantry_mid",   "gantry_mid_offset",   "gantry_mid"),
                        ("gantry_end",   "gantry_end_offset",   "gantry_end"),
                        ("wrist_pitch",  "wrist_pitch_offset",  "wrist_pitch"),
                        ("wrist_roll",   "wrist_roll_offset",   "wrist_roll"),
                        ("gripper",      "gripper_offset",      "gripper"),
                    ]:
                        if _jname in arm_limits:
                            _lo, _hi = arm_limits[_jname]
                            _off = getattr(state, _off_attr)
                            _rel = getattr(state, _rel_attr)
                            setattr(state, _rel_attr,
                                    max(_lo - _off, min(_hi - _off, _rel)))
                else:
                    pos_limit = 3.14159
                    state.gantry_base = max(-pos_limit, min(pos_limit, state.gantry_base))
                    state.gantry_mid = max(-pos_limit, min(pos_limit, state.gantry_mid))
                    state.gantry_end = max(-pos_limit, min(pos_limit, state.gantry_end))
                    state.wrist_pitch = max(-pos_limit, min(pos_limit, state.wrist_pitch))
                    state.wrist_roll = max(-pos_limit, min(pos_limit, state.wrist_roll))
                    state.gripper = max(-pos_limit, min(pos_limit, state.gripper))
        else:
            # Consume gantry key presses but don't move (not initialized yet)
            state.gantry_base_dec = False
            state.gantry_base_inc = False
            state.gantry_mid_dec = False
            state.gantry_mid_inc = False
            state.gantry_end_dec = False
            state.gantry_end_inc = False
            state.wrist_pitch_dec = False
            state.wrist_pitch_inc = False
            state.wrist_roll_dec = False
            state.wrist_roll_inc = False
            state.gripper_dec = False
            state.gripper_inc = False

    # --- drive (always, to feed watchdog) ----------------------------------
    if not sent_estop:
        linear = state.linear * cfg["drive"]["max_linear"]
        angular = state.angular * cfg["drive"]["max_angular"]
        # Get tunable MIT mode gains from config
        kp = cfg["drive"].get("kp", 0.0)
        kd = cfg["drive"].get("kd", 3.0)
        comms.send_drive(linear, angular, kp, kd)

    # --- swivel position control (send absolute position) ------------------
    if state.swivel_initialized and not sent_estop:
        swivel_kp = cfg["drive"].get("swivel_kp", 5.0)
        swivel_kd = cfg["drive"].get("swivel_kd", 0.5)
        # Convert relative position to absolute
        abs_swivel = state.swivel_position + state.swivel_offset
        comms.send_swivel_position(abs_swivel, swivel_kp, swivel_kd)


# ---------------------------------------------------------------------------
# Terminal UI
# ---------------------------------------------------------------------------

def safe_addstr(stdscr, y, x, text, attr=0, clear_line=False):
    """addnstr wrapper that silently clips to window width.

    Args:
        stdscr: curses window
        y, x: coordinates
        text: string to display
        attr: curses attributes
        clear_line: if True, clear the line before writing (more efficient than erase())
    """
    h, w = stdscr.getmaxyx()
    if y < 0 or y >= h or x >= w:
        return
    try:
        if clear_line:
            stdscr.move(y, 0)
            stdscr.clrtoeol()
        max_len = w - x - 1
        if max_len > 0:
            stdscr.addnstr(y, x, text, max_len, attr)
    except curses.error:
        pass


def _format_motor_error(err_str):
    """Condense a Rust MotorError debug string to just the active flag names.

    Input:  'fault@Run: MotorError { undervoltage: false, overcurrent: true, ... } (attempts: 1)'
    Output: 'Run: overcurrent  (attempts: 1)'
    No-flags case: 'Run fault, no error flags (likely watchdog)  (attempts: 0)'
    """
    import re
    if not err_str:
        return ""
    flags = re.findall(r'(\w+): true', err_str)
    attempts_m = re.search(r'\(attempts:\s*(\d+)\)', err_str)
    attempts = f"  (attempts: {attempts_m.group(1)})" if attempts_m else ""
    mode_m = re.search(r'fault@(\w+):', err_str)
    mode = mode_m.group(1) if mode_m else ""
    if flags:
        prefix = f"{mode}: " if mode else ""
        return prefix + ", ".join(flags) + attempts
    if "fault" in err_str:
        cause = f"{mode} " if mode else ""
        return f"{cause}fault, no error flags (likely watchdog)" + attempts
    return err_str[:80]


def draw_ui(stdscr, state, comms, cfg, start_time, last_row_count=None):
    """Render the full terminal UI with optimized updates.

    Instead of erasing the entire screen, we clear only the rows we'll use.
    This reduces flickering and CPU usage.

    Returns:
        Number of rows used (for next frame's selective clear)
    """
    telem = comms.last_telemetry
    now   = time.monotonic()

    hz     = cfg["control"]["command_rate_hz"]
    W      = 62
    BAR    = "=" * W
    SEP    = "-" * W
    gp_tag = "GAMEPAD" if state.gamepad_connected else "KB"

    telem_stale = comms.is_telemetry_stale()

    row = 0

    # -----------------------------------------------------------------------
    # Header
    # -----------------------------------------------------------------------
    safe_addstr(stdscr, row, 0, BAR, curses.A_BOLD, clear_line=True); row += 1
    right_tag = f"[{gp_tag}]  {hz}Hz"
    safe_addstr(stdscr, row, 0,
        f"  AIZEE TELEOP{' ' * max(1, W - 14 - len(right_tag))}{right_tag}",
        curses.A_BOLD, clear_line=True); row += 1
    safe_addstr(stdscr, row, 0, BAR, curses.A_BOLD, clear_line=True); row += 1

    # -----------------------------------------------------------------------
    # Alerts (conditional, variable height)
    # -----------------------------------------------------------------------
    if state.disable_confirm_pending:
        remaining = 5.0 - (now - state.disable_confirm_time)
        if remaining > 0:
            safe_addstr(stdscr, row, 0,
                f"  ! MOTORS WILL POWER OFF — press Q again ({remaining:.1f}s)",
                curses.A_REVERSE | curses.A_BOLD, clear_line=True); row += 1
            safe_addstr(stdscr, row, 0, "  ! CHECK SAFETY BEFORE CONFIRMING!",
                curses.A_REVERSE | curses.A_BOLD, clear_line=True); row += 1
        else:
            state.disable_confirm_pending = False

    if state.shutdown_countdown > 0:
        safe_addstr(stdscr, row, 0,
            f"  SAFE SHUTDOWN in {state.shutdown_countdown:.1f}s — moving all joints to zero",
            curses.A_REVERSE | curses.A_BOLD, clear_line=True); row += 1

    if state.shutdown_active:
        safe_addstr(stdscr, row, 0, "  SAFE SHUTDOWN: moving to zero...",
            curses.A_REVERSE | curses.A_BOLD, clear_line=True); row += 1

    # -----------------------------------------------------------------------
    # System / Connection
    # -----------------------------------------------------------------------
    if hasattr(comms, 'modules'):
        module_status = comms.get_module_status()
        parts = []
        any_warn = False
        for mod_name, s in module_status.items():
            if s["stale"]:
                age_s = "NO DATA" if s["telemetry_age"] > 99 else f"{s['telemetry_age']*1000:.0f}ms"
                parts.append(f"{mod_name}:[WARN {age_s}]")
                any_warn = True
            else:
                parts.append(f"{mod_name}:[OK {s['telemetry_age']*1000:.0f}ms {s['telem_rate']:.0f}Hz]")
        safe_addstr(stdscr, row, 0, "  SYS   " + "   ".join(parts),
            curses.A_REVERSE if any_warn else 0, clear_line=True); row += 1
    else:
        rate = getattr(comms, 'telem_rate', 0.0)
        telem_age = comms.get_telemetry_age()
        if telem_stale:
            age_s = "NO DATA" if telem_age > 99 else f"{telem_age*1000:.0f}ms"
            safe_addstr(stdscr, row, 0, f"  SYS   [{age_s}]  OFFLINE",
                curses.A_REVERSE, clear_line=True)
        else:
            safe_addstr(stdscr, row, 0, f"  SYS   [OK {telem_age*1000:.0f}ms  {rate:.0f}Hz]",
                clear_line=True)
        row += 1

    uptime = now - start_time
    if comms.last_telemetry_time > 0:
        age_ms = (now - comms.last_telemetry_time) * 1000
        safe_addstr(stdscr, row, 0,
            f"  Uptime: {uptime:.0f}s   Cmds: {comms.commands_sent}   Telem: {age_ms:.0f}ms ago",
            clear_line=True)
    else:
        safe_addstr(stdscr, row, 0,
            f"  Uptime: {uptime:.0f}s   Cmds: {comms.commands_sent}   Telem: (none)",
            clear_line=True)
    row += 1

    # -----------------------------------------------------------------------
    # Drive
    # -----------------------------------------------------------------------
    safe_addstr(stdscr, row, 0, SEP, clear_line=True); row += 1

    # NOTE: Motor controller interprets linear/angular backwards; swap for display.
    lin_scaled = state.linear  * cfg["drive"]["max_linear"]
    ang_scaled = state.angular * cfg["drive"]["max_angular"]
    swv_pos = state.swivel_position if state.swivel_initialized else 0.0
    safe_addstr(stdscr, row, 0,
        f"  DRIVE   lin={ang_scaled:+7.3f} m/s   ang={lin_scaled:+7.3f} rad/s   swv={swv_pos:+6.3f}",
        clear_line=True); row += 1

    # -----------------------------------------------------------------------
    # Arm Joints — target vs actual vs error
    # -----------------------------------------------------------------------
    if "gantry" in cfg:
        safe_addstr(stdscr, row, 0, SEP, clear_line=True); row += 1

        motors = telem.get("motors", {}) if telem else {}
        arm_limits = cfg.get("arm_limits")
        homed_tag = "" if state.gantry_homed else "  [NOT HOMED — press H]"
        hdr_attr = curses.A_BOLD if state.gantry_homed else (curses.A_BOLD | curses.A_REVERSE)
        lim_hdr = "      lo        hi" if arm_limits else ""
        safe_addstr(stdscr, row, 0,
            f"  ARM JOINTS      target      actual       err{lim_hdr}{homed_tag}",
            hdr_attr, clear_line=True); row += 1

        for jname, target in [
            ("gantry_base",  state.gantry_base  + state.gantry_base_offset),
            ("gantry_mid",   state.gantry_mid   + state.gantry_mid_offset),
            ("gantry_end",   state.gantry_end   + state.gantry_end_offset),
            ("wrist_pitch",  state.wrist_pitch  + state.wrist_pitch_offset),
            ("wrist_roll",   state.wrist_roll   + state.wrist_roll_offset),
            ("gripper",      state.gripper      + state.gripper_offset),
        ]:
            m = motors.get(jname) if state.gantry_initialized else None
            if m is not None:
                actual = m.get("position", 0.0)
                err    = target - actual
                base_str = f"  {jname:<14}  {target:>+8.4f}   {actual:>+8.4f}   {err:>+7.4f}"
            else:
                base_str = f"  {jname:<14}  {target:>+8.4f}         --          --"
            if arm_limits and jname in arm_limits:
                lo, hi = arm_limits[jname]
                at_lo = target <= lo + 1e-4
                at_hi = target >= hi - 1e-4
                marker = "!" if (at_lo or at_hi) else " "
                lim_str = f"  {marker}{lo:>+8.3f} {hi:>+8.3f}"
            else:
                lim_str = ""
            safe_addstr(stdscr, row, 0, base_str + lim_str, clear_line=True)
            row += 1

    # -----------------------------------------------------------------------
    # Motor telemetry
    # -----------------------------------------------------------------------
    safe_addstr(stdscr, row, 0, SEP, clear_line=True); row += 1
    safe_addstr(stdscr, row, 0,
        f"  MOTORS           state      pos(rad)  vel(rad/s)    T",
        curses.A_BOLD, clear_line=True); row += 1

    if telem and "motors" in telem:
        for mid in cfg["motors"]["all"]:
            m = telem["motors"].get(mid)
            if m is None:
                safe_addstr(stdscr, row, 0, f"  {mid:<16}  (no data)", clear_line=True); row += 1
            else:
                st   = m.get("state", "?")
                pos  = m.get("position", 0.0)
                vel  = m.get("velocity", 0.0)
                temp = m.get("temperature", 0.0)
                err  = m.get("error")
                attr = (curses.color_pair(4) | curses.A_BOLD) if err else 0
                safe_addstr(stdscr, row, 0,
                    f"  {mid:<16}  {st:<10}  {pos:>+7.3f}   {vel:>+7.3f}   {temp:.0f}°C",
                    attr, clear_line=True); row += 1
                if err:
                    safe_addstr(stdscr, row, 0,
                        f"    \u2514\u2500 {_format_motor_error(err)}",
                        curses.color_pair(4) | curses.A_BOLD, clear_line=True); row += 1
    else:
        safe_addstr(stdscr, row, 0, "  (waiting for telemetry...)", clear_line=True); row += 1

    # -----------------------------------------------------------------------
    # Power (battery + UPS)
    # -----------------------------------------------------------------------
    safe_addstr(stdscr, row, 0, SEP, clear_line=True); row += 1

    if "battery" in cfg and telem and "battery_voltage" in telem:
        voltage = telem["battery_voltage"]
        bat_cfg = cfg["battery"]
        v_range = bat_cfg["voltage_full"] - bat_cfg["voltage_min"]
        percent = max(0, min(100, (voltage - bat_cfg["voltage_min"]) / v_range * 100))
        if voltage >= bat_cfg["voltage_nominal"]:
            bat_st, bat_attr = "OK",   curses.color_pair(1) | curses.A_BOLD
        elif voltage >= bat_cfg["voltage_warning"]:
            bat_st, bat_attr = "GOOD", curses.color_pair(2)
        elif voltage >= bat_cfg["voltage_critical"]:
            bat_st, bat_attr = "WARN", curses.color_pair(3) | curses.A_BOLD
        else:
            bat_st, bat_attr = "CRIT", curses.color_pair(4) | curses.A_BOLD | curses.A_REVERSE
        safe_addstr(stdscr, row, 0,
            f"  BAT  {voltage:.2f}V ({percent:.0f}%) [{bat_st}]"
            f"  ({bat_cfg['cells']}S {bat_cfg['cell_type'].upper()})",
            bat_attr, clear_line=True); row += 1
    elif "battery" in cfg:
        safe_addstr(stdscr, row, 0, "  BAT  DC  (actuator power off)", clear_line=True); row += 1

    ups_telem = comms.last_ups_telemetry if hasattr(comms, 'last_ups_telemetry') else None
    if ups_telem and "ups" in ups_telem:
        ud  = ups_telem["ups"]
        v, c, p, pct = (ud.get("voltage", 0.0), ud.get("current", 0.0),
                        ud.get("power", 0.0),   ud.get("percentage", 0.0))
        ups_cfg = cfg.get("ups", {})
        vn, vw  = ups_cfg.get("voltage_nominal", 11.7), ups_cfg.get("voltage_warning", 10.8)
        vs      = ups_cfg.get("voltage_shutdown", 10.0)
        if v >= vn:   ups_st, ups_attr = "OK",       curses.color_pair(1) | curses.A_BOLD
        elif v >= vw: ups_st, ups_attr = "WARN",     curses.color_pair(3) | curses.A_BOLD
        elif v >= vs: ups_st, ups_attr = "CRIT",     curses.color_pair(4) | curses.A_BOLD
        else:         ups_st, ups_attr = "SHUTDOWN",  curses.color_pair(4) | curses.A_BOLD | curses.A_REVERSE
        safe_addstr(stdscr, row, 0,
            f"  UPS  {v:.2f}V  {c:.2f}A  {p:.1f}W  ({pct:.0f}%) [{ups_st}]",
            ups_attr, clear_line=True); row += 1
    elif hasattr(comms, 'ups_sub') and comms.ups_sub:
        safe_addstr(stdscr, row, 0, "  UPS  (no data)", clear_line=True); row += 1

    # Gamepad axes (compact single line, only when connected)
    if state.gamepad_connected and state.gamepad_axes:
        ax = state.gamepad_axes
        safe_addstr(stdscr, row, 0,
            f"  PAD  LX={ax.get('left_stick_x', 0.0):+.2f}  LY={ax.get('left_stick_y', 0.0):+.2f}"
            f"  RX={ax.get('right_stick_x', 0.0):+.2f}  RY={ax.get('right_stick_y', 0.0):+.2f}",
            clear_line=True); row += 1

    # -----------------------------------------------------------------------
    # Key legend
    # -----------------------------------------------------------------------
    safe_addstr(stdscr, row, 0, BAR, curses.A_BOLD, clear_line=True); row += 1
    safe_addstr(stdscr, row, 0,
        "  E=enable  Q=disable  SPC=estop  R=clear  H=home  ESC=quit",
        clear_line=True); row += 1
    if "gantry" in cfg:
        safe_addstr(stdscr, row, 0,
            "  1/2=base  3/4=mid  5/6=end  7/8=pitch  [/]=roll  -/==grip",
            clear_line=True); row += 1
    safe_addstr(stdscr, row, 0, BAR, curses.A_BOLD, clear_line=True); row += 1

    # Clear any remaining lines from previous frame (if screen shrunk)
    if last_row_count is not None and last_row_count > row:
        h, w = stdscr.getmaxyx()
        for clear_row in range(row, min(last_row_count, h)):
            try:
                stdscr.move(clear_row, 0)
                stdscr.clrtoeol()
            except curses.error:
                break

    stdscr.noutrefresh()
    curses.doupdate()

    return row  # Return row count for next frame


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(stdscr):
    # --- args --------------------------------------------------------------
    parser = argparse.ArgumentParser(description="AIZEE Teleop")
    parser.add_argument(
        "--config", default=None,
        help="Path to teleop.yaml config file",
    )
    parser.add_argument(
        "--endpoint", default=None,
        help="Override command endpoint (e.g. tcp://localhost:5555)",
    )
    parser.add_argument(
        "--keyboard-only", action="store_true",
        help="Disable gamepad input",
    )
    parser.add_argument(
        "--log-level",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Set logging level (default: INFO)",
    )
    parser.add_argument(
        "--robstride-calib", default=None,
        help="Path to robstride_calibration.json (default: auto-discover)",
    )
    args = parser.parse_args()

    # --- logging setup -----------------------------------------------------
    log_level = getattr(logging, args.log_level)
    setup_logging(log_level)
    logger.info("="*60)
    logger.info("AIZEE Teleop Starting")
    logger.info("="*60)

    # --- config ------------------------------------------------------------
    config_path = args.config
    if config_path is None:
        # Search relative to this script, then CWD
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(here, "..", "..", "config", "teleop.yaml"),
            os.path.join("config", "teleop.yaml"),
        ]
        for c in candidates:
            if os.path.isfile(c):
                config_path = c
                break
    if config_path is None:
        logger.error("Cannot find config/teleop.yaml — use --config")
        raise FileNotFoundError("Cannot find config/teleop.yaml — use --config")

    logger.info(f"Loading config from: {config_path}")
    cfg = load_config(config_path)

    # --- arm calibration limits --------------------------------------------
    cfg["arm_limits"] = None
    if args.robstride_calib:
        calib_candidates = [args.robstride_calib]
    else:
        here = os.path.dirname(os.path.abspath(__file__))
        calib_candidates = [
            os.path.join(here, "..", "..", "config", "robstride_calibration.json"),
            os.path.join("config", "robstride_calibration.json"),
        ]
    for c in calib_candidates:
        if os.path.isfile(c):
            try:
                with open(c) as f:
                    _calib = json.load(f)
                cfg["arm_limits"] = {}
                for _j, _d in _calib.get("joints", {}).items():
                    _a, _b = float(_d["min_rad"]), float(_d["max_rad"])
                    cfg["arm_limits"][_j] = (min(_a, _b), max(_a, _b))
                logger.info(f"Arm limits loaded from {c} ({len(cfg['arm_limits'])} joints)")
            except Exception as _e:
                logger.warning(f"Could not load arm limits from {c}: {_e}")
            break
    if cfg["arm_limits"] is None:
        logger.info("No robstride_calibration.json found — arm limits not enforced")

    # --- curses setup ------------------------------------------------------
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    # Initialize colors for battery status
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)   # OK - green
    curses.init_pair(2, curses.COLOR_CYAN, -1)    # GOOD - cyan
    curses.init_pair(3, curses.COLOR_YELLOW, -1)  # WARN - yellow
    curses.init_pair(4, curses.COLOR_RED, -1)     # CRIT - red (bold)

    # --- gamepad -----------------------------------------------------------
    joystick = None
    if not args.keyboard_only:
        logger.info("Initializing gamepad...")
        joystick = init_joystick()
        if joystick:
            logger.info(f"Gamepad found: {joystick.get_name()}")
        else:
            logger.warning("No gamepad detected - keyboard only mode")
    else:
        logger.info("Keyboard-only mode")

    # --- comms -------------------------------------------------------------
    # Multi-module mode if 'modules' exists in config
    try:
        if "modules" in cfg:
            logger.info("Using multi-module configuration")
            comms = MultiModuleComms(cfg["modules"])
        else:
            # Legacy single-module mode
            logger.info("Using single-module configuration")
            cmd_addr = cfg["endpoints"]["command"]
            telem_addr = cfg["endpoints"]["telemetry"]
            ups_telem_addr = cfg["endpoints"].get("ups_telemetry", None)
            if args.endpoint:
                cmd_addr = args.endpoint
                # Derive telemetry from command port + 1
                parts = cmd_addr.rsplit(":", 1)
                telem_port = int(parts[1]) + 1
                telem_addr = f"{parts[0]}:{telem_port}"
            comms = Comms(cmd_addr, telem_addr, ups_telem_addr)
    except Exception as e:
        logger.error(f"Failed to initialize communications: {e}")
        raise

    # --- state -------------------------------------------------------------
    state = InputState()
    state.gamepad_connected = joystick is not None
    start_time = time.monotonic()
    tick = 1.0 / cfg["control"]["command_rate_hz"]

    logger.info(f"Starting main loop at {cfg['control']['command_rate_hz']} Hz")
    logger.info("Press ESC to exit")

    last_row_count = None  # Track rows from previous frame for optimization
    no_telem_logged = False  # Track if we've warned about missing telemetry
    no_telem_start = None    # When did telemetry loss start

    try:
        last_time = time.monotonic()
        while not state.quit:
            t0 = time.monotonic()
            dt = t0 - last_time
            last_time = t0

            # 1. Gamepad
            if joystick is not None:
                read_gamepad(joystick, cfg, state)

            # 2. Keyboard input
            if joystick is None or not state.gamepad_connected:
                # Use pygame keyboard state if available (true key state, no timing hacks)
                if _pygame_available:
                    read_keyboard_pygame(state)
                    # Still process curses keys for commands (E, Q, R, etc) but skip WASD
                    while True:
                        key = stdscr.getch()
                        if key == -1:
                            break
                        read_keyboard(key, state, t0, skip_movement_keys=True)
                    # Smooth inputs
                    smooth_keyboard_inputs(state, dt)
                else:
                    # Fallback to curses-only (with timing hacks)
                    # Process all keys in buffer
                    while True:
                        key = stdscr.getch()
                        if key == -1:
                            break
                        read_keyboard(key, state, t0)

                    # Smart timeout: Don't reset during repeat delay window (0.2-0.65s)
                    # Forward/back (W/S -> angular)
                    w_age = t0 - state.last_w_time
                    s_age = t0 - state.last_s_time
                    w_time_since_first = t0 - state.w_first_press_time if state.w_repeat_count > 0 else 999
                    s_time_since_first = t0 - state.s_first_press_time if state.s_repeat_count > 0 else 999

                    # Determine timeout
                    if state.w_repeat_count >= state.repeat_active_threshold or state.s_repeat_count >= state.repeat_active_threshold:
                        timeout_angular = state.key_timeout_active
                    else:
                        timeout_angular = state.key_timeout_short

                    # Check if we should reset
                    should_reset_angular = False
                    if w_age > timeout_angular and s_age > timeout_angular:
                        # Timeout expired - but check if we're in repeat delay window
                        w_in_repeat_window = (state.w_repeat_count > 0 and
                                             state.tap_window < w_time_since_first < state.repeat_delay_end)
                        s_in_repeat_window = (state.s_repeat_count > 0 and
                                             state.tap_window < s_time_since_first < state.repeat_delay_end)

                        # Only reset if NOT in repeat delay window
                        if not w_in_repeat_window and not s_in_repeat_window:
                            should_reset_angular = True

                    if should_reset_angular:
                        state.angular_target = 0.0
                        state.w_repeat_count = 0
                        state.s_repeat_count = 0

                    # Left/right (A/D -> linear)
                    a_age = t0 - state.last_a_time
                    d_age = t0 - state.last_d_time
                    a_time_since_first = t0 - state.a_first_press_time if state.a_repeat_count > 0 else 999
                    d_time_since_first = t0 - state.d_first_press_time if state.d_repeat_count > 0 else 999

                    if state.a_repeat_count >= state.repeat_active_threshold or state.d_repeat_count >= state.repeat_active_threshold:
                        timeout_linear = state.key_timeout_active
                    else:
                        timeout_linear = state.key_timeout_short

                    should_reset_linear = False
                    if a_age > timeout_linear and d_age > timeout_linear:
                        a_in_repeat_window = (state.a_repeat_count > 0 and
                                             state.tap_window < a_time_since_first < state.repeat_delay_end)
                        d_in_repeat_window = (state.d_repeat_count > 0 and
                                             state.tap_window < d_time_since_first < state.repeat_delay_end)

                        if not a_in_repeat_window and not d_in_repeat_window:
                            should_reset_linear = True

                    if should_reset_linear:
                        state.linear_target = 0.0
                        state.a_repeat_count = 0
                        state.d_repeat_count = 0

                    # Swivel is position-controlled via Z/C keys (handled separately)

                    # Smooth keyboard inputs toward targets
                    smooth_keyboard_inputs(state, dt)
            else:
                # Gamepad mode - still need to process keyboard for commands
                while True:
                    key = stdscr.getch()
                    if key == -1:
                        break
                    read_keyboard(key, state, t0)

            # 3. Telemetry
            comms.recv_latest_telemetry()
            comms.recv_latest_ups_telemetry()

            # Check for persistent telemetry issues
            if comms.last_telemetry_time == 0.0:
                if no_telem_start is None:
                    no_telem_start = time.monotonic()
                elif not no_telem_logged and (time.monotonic() - no_telem_start) > 5.0:
                    logger.warning("No telemetry received for 5+ seconds. Check motor controller connection.")
                    no_telem_logged = True
            else:
                # Reset warning state when telemetry is received
                if no_telem_logged:
                    logger.info("Telemetry connection restored")
                no_telem_start = None
                no_telem_logged = False

            # 4. Dispatch
            dispatch_commands(state, comms, cfg, dt)

            # 5. Draw (optimized with selective line clearing)
            last_row_count = draw_ui(stdscr, state, comms, cfg, start_time, last_row_count)

            # 6. Sleep remainder
            elapsed = time.monotonic() - t0
            remaining = tick - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Unexpected error in main loop: {e}", exc_info=True)
    finally:
        logger.info("Shutting down...")
        # Safety shutdown: zero velocity + disable all
        try:
            logger.info("Sending safety shutdown commands...")
            comms.send_drive(0.0, 0.0, 0.0)
            time.sleep(0.1)  # Give command time to send
            comms.send_disable(cfg["motors"]["all"])
            time.sleep(0.1)
            logger.info("Safety shutdown commands sent")
        except Exception as e:
            logger.warning(f"Error during safety shutdown: {e}")

        # Close communications (with fixed non-blocking cleanup)
        try:
            comms.close()
        except Exception as e:
            logger.error(f"Error closing communications: {e}")

        # Close gamepad
        if joystick is not None and _pygame_available:
            try:
                pygame.quit()
                logger.info("Pygame closed")
            except Exception as e:
                logger.warning(f"Error closing pygame: {e}")

        logger.info("Teleop shutdown complete")


if __name__ == "__main__":
    curses.wrapper(main)
