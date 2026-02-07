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
"""

import argparse
import curses
import json
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
    raw_linear = joystick.get_axis(axes["left_stick_y"])
    if invert.get("left_stick_y", False):
        raw_linear = -raw_linear
    raw_angular = joystick.get_axis(axes["left_stick_x"])
    if invert.get("left_stick_x", False):
        raw_angular = -raw_angular
    raw_swivel = joystick.get_axis(axes["right_stick_x"])
    if invert.get("right_stick_x", False):
        raw_swivel = -raw_swivel

    state.linear = apply_curve(
        apply_deadzone(raw_linear, deadzone),
        cfg["drive"]["linear_exponent"],
    )
    state.angular = apply_curve(
        apply_deadzone(raw_angular, deadzone),
        cfg["drive"]["angular_exponent"],
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
    if key == ord("w") or key == ord("W"):
        state.linear = 1.0
    elif key == ord("s") or key == ord("S"):
        state.linear = -1.0
    elif key == ord("a") or key == ord("A"):
        state.angular = 1.0
    elif key == ord("d") or key == ord("D"):
        state.angular = -1.0
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
        self.ctx = zmq.Context()

        self.cmd = self.ctx.socket(zmq.PUSH)
        self.cmd.connect(cmd_addr)

        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(telem_addr)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.setsockopt(zmq.RCVTIMEO, 50)

        self.commands_sent = 0
        self.last_telemetry = None
        self.last_telemetry_time = 0.0

    # -- send helpers -------------------------------------------------------

    def send(self, msg):
        self.cmd.send_string(json.dumps(msg))
        self.commands_sent += 1

    def send_drive(self, linear, angular, swivel=0.0):
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
        while True:
            try:
                raw = self.sub.recv_string(zmq.NOBLOCK)
                latest = json.loads(raw)
            except zmq.Again:
                break
        if latest is not None:
            self.last_telemetry = latest
            self.last_telemetry_time = time.monotonic()
        return self.last_telemetry

    # -- cleanup ------------------------------------------------------------

    def close(self):
        self.cmd.close()
        self.sub.close()
        self.ctx.term()


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

def safe_addstr(stdscr, y, x, text, attr=0):
    """addnstr wrapper that silently clips to window width."""
    h, w = stdscr.getmaxyx()
    if y < 0 or y >= h or x >= w:
        return
    max_len = w - x - 1
    if max_len <= 0:
        return
    try:
        stdscr.addnstr(y, x, text, max_len, attr)
    except curses.error:
        pass


def draw_ui(stdscr, state, comms, cfg, start_time):
    """Render the full terminal UI. Called once per tick."""
    stdscr.erase()
    telem = comms.last_telemetry
    now = time.monotonic()

    hz = cfg["control"]["command_rate_hz"]
    gp_tag = "GAMEPAD" if state.gamepad_connected else "KB ONLY"
    bar = "=" * 60

    row = 0
    safe_addstr(stdscr, row, 0, bar, curses.A_BOLD)
    row += 1
    safe_addstr(
        stdscr, row, 0,
        f"  AIZEE TELEOP{' ' * 28}[{gp_tag}] {hz}Hz",
        curses.A_BOLD,
    )
    row += 1
    safe_addstr(stdscr, row, 0, bar, curses.A_BOLD)
    row += 1

    # Drive values (scaled)
    lin_scaled = state.linear * cfg["drive"]["max_linear"]
    ang_scaled = state.angular * cfg["drive"]["max_angular"]
    swv_scaled = state.swivel * cfg["drive"]["max_swivel"]
    safe_addstr(
        stdscr, row, 0,
        f"  DRIVE:  linear={lin_scaled:+7.3f} rad/s   "
        f"angular={ang_scaled:+7.3f} rad/s   swivel={swv_scaled:+6.3f} rad/s",
    )
    row += 2

    # Motor telemetry
    safe_addstr(stdscr, row, 0, f"  --- MOTORS {'-' * 47}", curses.A_BOLD)
    row += 1

    if telem and "motors" in telem:
        for mid in cfg["motors"]["all"]:
            m = telem["motors"].get(mid)
            if m is None:
                safe_addstr(stdscr, row, 0, f"  {mid:15s}  (no data)")
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
                safe_addstr(stdscr, row, 0, line)
            row += 1
    else:
        safe_addstr(stdscr, row, 0, "  (waiting for telemetry...)")
        row += 1

    row += 1
    safe_addstr(stdscr, row, 0, f"  --- STATUS {'-' * 47}", curses.A_BOLD)
    row += 1

    # Telemetry age
    if comms.last_telemetry_time > 0:
        age_ms = (now - comms.last_telemetry_time) * 1000
        safe_addstr(
            stdscr, row, 0,
            f"  Telemetry: {age_ms:.0f}ms ago"
            f"         Commands sent: {comms.commands_sent}",
        )
    else:
        safe_addstr(
            stdscr, row, 0,
            f"  Telemetry: (none)          Commands sent: {comms.commands_sent}",
        )
    row += 1

    uptime = now - start_time
    safe_addstr(stdscr, row, 0, f"  Uptime: {uptime:.0f}s")
    row += 2

    # Key legend
    safe_addstr(
        stdscr, row, 0,
        "  WASD=drive  E=enable  Q=disable  SPACE=ESTOP  ESC=quit",
    )
    row += 1
    safe_addstr(
        stdscr, row, 0,
        "  R=clear faults  Z=zero positions",
    )
    row += 1
    safe_addstr(stdscr, row, 0, bar, curses.A_BOLD)

    stdscr.noutrefresh()
    curses.doupdate()


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
    args = parser.parse_args()

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
        raise FileNotFoundError("Cannot find config/teleop.yaml — use --config")

    cfg = load_config(config_path)

    # Endpoint override
    cmd_addr = cfg["endpoints"]["command"]
    telem_addr = cfg["endpoints"]["telemetry"]
    if args.endpoint:
        cmd_addr = args.endpoint
        # Derive telemetry from command port + 1
        parts = cmd_addr.rsplit(":", 1)
        telem_port = int(parts[1]) + 1
        telem_addr = f"{parts[0]}:{telem_port}"

    # --- curses setup ------------------------------------------------------
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(0)

    # --- gamepad -----------------------------------------------------------
    joystick = None
    if not args.keyboard_only:
        joystick = init_joystick()

    # --- comms -------------------------------------------------------------
    comms = Comms(cmd_addr, telem_addr)

    # --- state -------------------------------------------------------------
    state = InputState()
    state.gamepad_connected = joystick is not None
    start_time = time.monotonic()
    tick = 1.0 / cfg["control"]["command_rate_hz"]

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

            # 4. Dispatch
            dispatch_commands(state, comms, cfg)

            # 5. Draw
            draw_ui(stdscr, state, comms, cfg, start_time)

            # 6. Sleep remainder
            elapsed = time.monotonic() - t0
            remaining = tick - elapsed
            if remaining > 0:
                time.sleep(remaining)

    finally:
        # Safety shutdown: zero velocity + disable all
        try:
            comms.send_drive(0.0, 0.0)
            comms.send_disable(cfg["motors"]["all"])
        except Exception:
            pass
        comms.close()
        if joystick is not None and _pygame_available:
            pygame.quit()


if __name__ == "__main__":
    curses.wrapper(main)
