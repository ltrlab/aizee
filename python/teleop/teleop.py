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
        self.swivel = 0.0

        # One-shot flags — consumed after dispatch
        self.enable_all = False
        self.disable_all = False
        self.emergency_stop = False
        self.clear_estop = False
        self.clear_faults = False
        self.zero_positions = False
        self.quit = False

        self.gamepad_connected = False


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
    state.swivel = apply_curve(
        apply_deadzone(raw_swivel, deadzone),
        cfg["drive"]["angular_exponent"],  # Use same exponent as angular
    )

    # --- buttons (one-shot on press) ---
    if joystick.get_button(buttons["a"]):
        state.enable_all = True
    if joystick.get_button(buttons["b"]):
        state.disable_all = True
    if joystick.get_button(buttons["back"]):
        state.emergency_stop = True
    if joystick.get_button(buttons["start"]):
        state.clear_estop = True
        state.clear_faults = True


def read_keyboard(key, state):
    """Process a single curses key code into *state*.

    Keyboard is binary: pressed = full value. The caller resets
    linear/angular to 0 before draining keys, so only the *last*
    directional key in the drain wins (good enough at 20 Hz).
    """
    # WORKAROUND: Motor controller has linear/angular backwards
    # Swapping keys here so controls feel right to user
    # TODO: Fix in rust/motor_control instead
    if key == ord("w") or key == ord("W"):
        state.angular = 1.0   # W maps to angular (makes robot turn)
    elif key == ord("s") or key == ord("S"):
        state.angular = -1.0  # S maps to angular (makes robot turn)
    elif key == ord("a") or key == ord("A"):
        state.linear = -1.0   # A maps to linear (makes robot go back)
    elif key == ord("d") or key == ord("D"):
        state.linear = 1.0    # D maps to linear (makes robot go forward)
    elif key == ord("e") or key == ord("E"):
        state.enable_all = True
    elif key == ord("q") or key == ord("Q"):
        state.disable_all = True
    elif key == ord(" "):
        state.emergency_stop = True
    elif key == ord("r") or key == ord("R"):
        state.clear_estop = True
        state.clear_faults = True
    elif key == ord("z") or key == ord("Z"):
        state.zero_positions = True
    elif key == 27:  # Escape
        state.quit = True


# ---------------------------------------------------------------------------
# Comms
# ---------------------------------------------------------------------------

class Comms:
    """ZeroMQ command (PUSH) and telemetry (SUB) sockets."""

    def __init__(self, cmd_addr, telem_addr):
        self.cmd_addr = cmd_addr
        self.telem_addr = telem_addr
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

    def send_drive(self, linear, angular, swivel=0.0):
        # TODO: Motor controller interprets linear/angular parameters incorrectly
        # This needs to be fixed in rust/motor_control/src/main.rs
        # For now, sending parameters as-is with incorrect semantics
        self.send({"type": "drive", "linear": linear, "angular": angular, "swivel": swivel})

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

    def send_drive(self, linear, angular, swivel=0.0):
        """Send drive command to rover module."""
        # TODO: Motor controller interprets linear/angular parameters incorrectly
        # This needs to be fixed in rust/motor_control/src/main.rs
        # For now, sending parameters as-is with incorrect semantics
        self.send_command("rover", {
            "type": "drive",
            "linear": linear,
            "angular": angular,
            "swivel": swivel
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

def dispatch_commands(state, comms, cfg):
    """Send one-shot actions then drive command. Consumes flags."""
    all_motors = cfg["motors"]["all"]

    # --- one-shots ---------------------------------------------------------
    sent_estop = False

    if state.emergency_stop:
        comms.send_emergency_stop()
        state.emergency_stop = False
        sent_estop = True

    if state.clear_estop:
        comms.send_clear_estop()
        state.clear_estop = False

    if state.clear_faults:
        comms.send_clear_faults(all_motors)
        state.clear_faults = False

    if state.enable_all:
        comms.send_enable(all_motors)
        state.enable_all = False

    if state.disable_all:
        comms.send_disable(all_motors)
        state.disable_all = False

    if state.zero_positions:
        comms.send_zero_position(all_motors)
        state.zero_positions = False

    # --- drive (always, to feed watchdog) ----------------------------------
    if not sent_estop:
        linear = state.linear * cfg["drive"]["max_linear"]
        angular = state.angular * cfg["drive"]["max_angular"]
        swivel = state.swivel * cfg["drive"]["max_swivel"]
        comms.send_drive(linear, angular, swivel)


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


def draw_ui(stdscr, state, comms, cfg, start_time, last_row_count=None):
    """Render the full terminal UI with optimized updates.

    Instead of erasing the entire screen, we clear only the rows we'll use.
    This reduces flickering and CPU usage.

    Returns:
        Number of rows used (for next frame's selective clear)
    """
    # Don't use erase() - it's slow. We'll clear lines as we go.
    telem = comms.last_telemetry
    now = time.monotonic()

    hz = cfg["control"]["command_rate_hz"]
    gp_tag = "GAMEPAD" if state.gamepad_connected else "KB ONLY"
    bar = "=" * 60

    # Check telemetry health
    telem_stale = comms.is_telemetry_stale()
    telem_age = comms.get_telemetry_age()

    row = 0
    safe_addstr(stdscr, row, 0, bar, curses.A_BOLD, clear_line=True)
    row += 1
    safe_addstr(
        stdscr, row, 0,
        f"  AIZEE TELEOP{' ' * 28}[{gp_tag}] {hz}Hz",
        curses.A_BOLD, clear_line=True
    )
    row += 1
    safe_addstr(stdscr, row, 0, bar, curses.A_BOLD, clear_line=True)
    row += 1

    # Connection status (for multi-module systems)
    if hasattr(comms, 'modules'):
        module_status = comms.get_module_status()
        safe_addstr(stdscr, row, 0, "  CONNECTION STATUS:", curses.A_BOLD, clear_line=True)
        row += 1
        for module_name, status in module_status.items():
            conn_str = "OK" if status["connected"] and not status["stale"] else "WARN" if status["stale"] else "FAIL"
            age_str = f"{status['telemetry_age']*1000:.0f}ms" if status['telemetry_age'] < 999 else "NO DATA"
            rate_str = f"{status['telem_rate']:.1f}Hz" if status['telem_rate'] > 0 else "0Hz"
            attr = curses.A_NORMAL if conn_str == "OK" else curses.A_REVERSE
            safe_addstr(stdscr, row, 0, f"    {module_name:10s} [{conn_str}]  age={age_str:8s}  rate={rate_str}", attr, clear_line=True)
            row += 1
        row += 1
    else:
        # Single module status
        rate_str = f"{comms.telem_rate:.1f}Hz" if hasattr(comms, 'telem_rate') and comms.telem_rate > 0 else "0Hz"
        if telem_stale:
            age_str = f"{telem_age*1000:.0f}ms" if telem_age < 999 else "NO DATA"
            safe_addstr(stdscr, row, 0, f"  CONNECTION: [WARN] age={age_str:8s}  rate={rate_str}", curses.A_REVERSE, clear_line=True)
        else:
            safe_addstr(stdscr, row, 0, f"  CONNECTION: [OK] age={telem_age*1000:.0f}ms  rate={rate_str}", clear_line=True)
        row += 2

    # Drive values (scaled)
    # NOTE: Motor controller interprets linear/angular backwards
    # Swap values in display to show correct semantic meaning
    lin_scaled = state.linear * cfg["drive"]["max_linear"]
    ang_scaled = state.angular * cfg["drive"]["max_angular"]
    swv_scaled = state.swivel * cfg["drive"]["max_swivel"]

    # Display with swapped values so labels match actual robot behavior
    # state.angular → show as "linear" (angular command causes forward/back motion)
    # state.linear → show as "angular" (linear command causes turning)
    safe_addstr(
        stdscr, row, 0,
        f"  DRIVE:  linear={ang_scaled:+7.3f} rad/s   "
        f"angular={lin_scaled:+7.3f} rad/s   swivel={swv_scaled:+6.3f} rad/s",
        clear_line=True
    )
    row += 2

    # Motor telemetry
    safe_addstr(stdscr, row, 0, f"  --- MOTORS {'-' * 47}", curses.A_BOLD, clear_line=True)
    row += 1

    if telem and "motors" in telem:
        for mid in cfg["motors"]["all"]:
            m = telem["motors"].get(mid)
            if m is None:
                safe_addstr(stdscr, row, 0, f"  {mid:15s}  (no data)", clear_line=True)
            else:
                st = m.get("state", "?")
                pos = m.get("position", 0.0)
                vel = m.get("velocity", 0.0)
                temp = m.get("temperature", 0.0)
                err = m.get("error")
                line = (
                    f"  {mid:15s} {st:10s} "
                    f"pos={pos:+8.3f}  vel={vel:+7.3f}  T={temp:.0f}C"
                )
                if err:
                    line += f"  ERR:{err}"
                safe_addstr(stdscr, row, 0, line, clear_line=True)
            row += 1
    else:
        safe_addstr(stdscr, row, 0, "  (waiting for telemetry...)", clear_line=True)
        row += 1

    row += 1
    safe_addstr(stdscr, row, 0, f"  --- STATUS {'-' * 47}", curses.A_BOLD, clear_line=True)
    row += 1

    # Telemetry age
    if comms.last_telemetry_time > 0:
        age_ms = (now - comms.last_telemetry_time) * 1000
        safe_addstr(
            stdscr, row, 0,
            f"  Telemetry: {age_ms:.0f}ms ago"
            f"         Commands sent: {comms.commands_sent}",
            clear_line=True
        )
    else:
        safe_addstr(
            stdscr, row, 0,
            f"  Telemetry: (none)          Commands sent: {comms.commands_sent}",
            clear_line=True
        )
    row += 1

    uptime = now - start_time
    safe_addstr(stdscr, row, 0, f"  Uptime: {uptime:.0f}s", clear_line=True)
    row += 2

    # Key legend
    safe_addstr(
        stdscr, row, 0,
        "  WASD=drive  E=enable  Q=disable  SPACE=ESTOP  ESC=quit",
        clear_line=True
    )
    row += 1
    safe_addstr(
        stdscr, row, 0,
        "  R=clear faults  Z=zero positions",
        clear_line=True
    )
    row += 1
    safe_addstr(stdscr, row, 0, bar, curses.A_BOLD, clear_line=True)
    row += 1

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

    # --- curses setup ------------------------------------------------------
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

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
            if args.endpoint:
                cmd_addr = args.endpoint
                # Derive telemetry from command port + 1
                parts = cmd_addr.rsplit(":", 1)
                telem_port = int(parts[1]) + 1
                telem_addr = f"{parts[0]}:{telem_port}"
            comms = Comms(cmd_addr, telem_addr)
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
        while not state.quit:
            t0 = time.monotonic()

            # 1. Gamepad
            if joystick is not None:
                read_gamepad(joystick, cfg, state)

            # 2. Keyboard — drain all pending keys
            #    If no gamepad, reset axes so keyboard is binary per tick
            if joystick is None or not state.gamepad_connected:
                state.linear = 0.0
                state.angular = 0.0
                state.swivel = 0.0

            while True:
                key = stdscr.getch()
                if key == -1:
                    break
                read_keyboard(key, state)

            # 3. Telemetry
            comms.recv_latest_telemetry()

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
            dispatch_commands(state, comms, cfg)

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
