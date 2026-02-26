"""
AIZEE Tufty2040 Status Display — MicroPython firmware for Pimoroni Tufty2040.

Reads newline-delimited JSON from USB CDC (sys.stdin) at 2 Hz and renders a
live robot-health dashboard on the 320×240 IPS LCD.

Deploy:
    mpremote cp tufty2040/main.py :main.py

JSON packet format (from display_node.py on Jetson):
    {"mv":24.1,"up":11.4,"ub":73,"me":true,"ms":{...},"sv":{...},"t":1740000000.0}

Motor state chars: r=running, e=enabling, d=disabled, x=error, ?=unknown
Service state chars: a=active, f=failed, i=inactive, e=activating, ?=unknown
"""

import json
import select
import sys
import time

from picographics import DISPLAY_TUFTY_2040, PicoGraphics

# ---------------------------------------------------------------------------
# Hardware init
# ---------------------------------------------------------------------------

display = PicoGraphics(display=DISPLAY_TUFTY_2040)
WIDTH, HEIGHT = display.get_bounds()  # 320 × 240

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

BG        = display.create_pen(10,  10,  30)   # near-black background
TITLE_BG  = display.create_pen(0,   30,  80)   # dark blue title bar
GREEN     = display.create_pen(0,   200, 80)   # good / running
YELLOW    = display.create_pen(230, 200, 0)    # warning / enabling
RED       = display.create_pen(220, 40,  40)   # critical / error / disabled
GRAY      = display.create_pen(140, 140, 140)  # labels / unknown
WHITE     = display.create_pen(255, 255, 255)
BLACK     = display.create_pen(0,   0,   0)
DARK_GRAY = display.create_pen(50,  50,  50)   # disabled motor bg
DARK_BG   = display.create_pen(20,  20,  50)   # section separator bg

# ---------------------------------------------------------------------------
# Voltage thresholds (from hardware_jetson_rover.yaml)
# ---------------------------------------------------------------------------

UPS_NOMINAL  = 11.1   # V → green
UPS_WARNING  = 10.5   # V → yellow
UPS_CRITICAL = 9.9    # V → red
UPS_FULL     = 12.6   # V (100%)
UPS_MIN      = 9.0    # V (0%)

MOTOR_WARN   = 22.0   # V → yellow (24V system)
MOTOR_CRIT   = 20.0   # V → red

# ---------------------------------------------------------------------------
# Motor abbreviation / display order
# ---------------------------------------------------------------------------

MOTOR_IDS = ["lw", "rw", "sw", "gb", "gm", "ge", "wp", "wr", "gr"]
MOTOR_LABELS = {
    "lw": "lw", "rw": "rw", "sw": "sw",
    "gb": "gb", "gm": "gm", "ge": "ge",
    "wp": "wp", "wr": "wr", "gr": "gr",
}
BASE_MOTORS = {"lw", "rw", "sw"}

# Motor state char → (bg_pen, text_pen)
STATE_COLORS = {
    "r": (GREEN,     BLACK),
    "e": (YELLOW,    BLACK),
    "d": (DARK_GRAY, GRAY),
    "x": (RED,       WHITE),
    "?": (DARK_GRAY, GRAY),
}

# ---------------------------------------------------------------------------
# Service display layout
# ---------------------------------------------------------------------------

# Two rows of (abbrev, display_label) pairs
SERVICES_ROWS = [
    [("motors", "MOTOR"), ("lidar", "LIDAR"), ("ups", "UPS")],
    [("relay",  "RELAY"), ("disp",  "DISP")],
]

# Service state char → (bg_pen, text_pen, status_label)
SV_COLORS = {
    "a": (GREEN,     BLACK, "OK "),
    "f": (RED,       WHITE, "ERR"),
    "i": (YELLOW,    BLACK, "---"),
    "e": (YELLOW,    BLACK, "ACT"),
    "?": (DARK_GRAY, GRAY,  "???"),
}

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------

last_data: dict = {}
last_recv: float = 0.0
buf: str = ""

STALE_TIMEOUT = 5.0  # seconds — show "NO SIGNAL" if no packet received

# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------

def clear():
    display.set_pen(BG)
    display.clear()


def color_for_ups_voltage(v):
    if v is None:
        return GRAY
    if v >= UPS_NOMINAL:
        return GREEN
    if v >= UPS_WARNING:
        return YELLOW
    return RED


def color_for_motor_voltage(v):
    if v is None:
        return GRAY
    if v >= MOTOR_WARN:
        return GREEN
    if v >= MOTOR_CRIT:
        return YELLOW
    return RED


def draw_title_bar(signal_ok: bool, waiting: bool):
    """Dark-blue title bar across the full width."""
    display.set_pen(TITLE_BG)
    display.rectangle(0, 0, WIDTH, 20)

    display.set_pen(WHITE)
    display.set_font("bitmap8")
    display.text("AIZEE STATUS", 4, 6, scale=1)

    if waiting:
        display.set_pen(YELLOW)
        display.text("WAITING...", 200, 6, scale=1)
    elif signal_ok:
        display.set_pen(GREEN)
        display.text("OK", 272, 6, scale=1)
    else:
        display.set_pen(RED)
        display.text("NO SIGNAL", 208, 6, scale=1)


def draw_section_line(y: int):
    display.set_pen(DARK_BG)
    display.rectangle(0, y, WIDTH, 2)


def draw_battery_section(mv, up, ub):
    """
    Left half: Jetson UPS battery  |  Right half: Motor bus voltage
    y range: 22–78
    """
    # --- Section labels ---
    display.set_pen(GRAY)
    display.set_font("bitmap8")
    display.text("JETSON BATTERY", 4, 22, scale=1)
    display.text("MOTOR BATTERY", 166, 22, scale=1)

    # --- UPS voltage (large) ---
    up_color = color_for_ups_voltage(up)
    display.set_pen(up_color)
    if up is None:
        display.text("---", 4, 32, scale=2)
    else:
        display.text(f"{up:.1f}V", 4, 32, scale=2)

    # --- UPS percentage ---
    if ub is None:
        display.set_pen(GRAY)
        display.text("---%", 80, 32, scale=2)
    else:
        pct_color = GREEN if ub >= 50 else (YELLOW if ub >= 20 else RED)
        display.set_pen(pct_color)
        display.text(f"{ub:3d}%", 80, 32, scale=2)

    # --- UPS progress bar ---
    bar_x, bar_y, bar_w, bar_h = 4, 58, 152, 12
    display.set_pen(DARK_BG)
    display.rectangle(bar_x, bar_y, bar_w, bar_h)
    if ub is not None:
        fill_w = int(bar_w * max(0, min(ub, 100)) / 100)
        bar_color = GREEN if ub >= 50 else (YELLOW if ub >= 20 else RED)
        display.set_pen(bar_color)
        display.rectangle(bar_x, bar_y, fill_w, bar_h)
    display.set_pen(GRAY)
    display.rectangle(bar_x, bar_y, bar_w, bar_h)  # outline (overdraw trick — just border)
    # Draw inner bar area background again and refill to get a border
    display.set_pen(DARK_BG)
    display.rectangle(bar_x + 1, bar_y + 1, bar_w - 2, bar_h - 2)
    if ub is not None:
        fill_w = int((bar_w - 2) * max(0, min(ub, 100)) / 100)
        bar_color = GREEN if ub >= 50 else (YELLOW if ub >= 20 else RED)
        display.set_pen(bar_color)
        display.rectangle(bar_x + 1, bar_y + 1, fill_w, bar_h - 2)

    # --- Motor voltage (large, right half) ---
    mv_color = color_for_motor_voltage(mv)
    display.set_pen(mv_color)
    if mv is None:
        display.text("---", 166, 32, scale=2)
    else:
        display.text(f"{mv:.1f}V", 166, 32, scale=2)

    # --- Vertical divider ---
    display.set_pen(DARK_BG)
    display.rectangle(158, 20, 4, 60)


def draw_motors_enabled(me: bool, ms: dict):
    """
    Motor enable status pill + per-motor state boxes.
    y range: 80–160
    """
    draw_section_line(80)

    # --- Enable/Disable pill ---
    display.set_pen(GRAY)
    display.set_font("bitmap8")
    display.text("MOTORS:", 4, 84, scale=1)

    if me:
        display.set_pen(GREEN)
        display.rectangle(60, 82, 90, 14)
        display.set_pen(BLACK)
        display.text("ENABLED", 64, 86, scale=1)
    else:
        display.set_pen(RED)
        display.rectangle(60, 82, 90, 14)
        display.set_pen(WHITE)
        display.text("DISABLED", 64, 86, scale=1)

    draw_section_line(100)

    # --- Motor state boxes ---
    # Row 1 (BASE): lw, rw, sw   y=104
    # Row 2 (ARM1): gb, gm, ge   y=122
    # Row 3 (ARM2): wp, wr, gr   y=140

    display.set_pen(GRAY)
    display.text("BASE:", 4, 106, scale=1)
    display.text("ARM: ", 4, 124, scale=1)

    rows = [
        (["lw", "rw", "sw"], 102),
        (["gb", "gm", "ge"], 120),
        (["wp", "wr", "gr"], 138),
    ]

    for motor_row, y in rows:
        x = 40
        for mid in motor_row:
            state_char = ms.get(mid, "?") if ms else "?"
            bg_pen, txt_pen = STATE_COLORS.get(state_char, (DARK_GRAY, GRAY))

            # Box background
            display.set_pen(bg_pen)
            display.rectangle(x, y, 38, 14)

            # Label inside box
            display.set_pen(txt_pen)
            label = f"{mid}[{state_char.upper()}]"
            display.text(label, x + 2, y + 3, scale=1)

            x += 44


def draw_services_section(sv: dict):
    """
    Systemd service status grid — two rows of coloured boxes.
    y range: 154–200
    """
    draw_section_line(154)

    display.set_pen(GRAY)
    display.set_font("bitmap8")
    display.text("SERVICES:", 4, 158, scale=1)

    row_ys = [170, 188]
    for row_idx, row in enumerate(SERVICES_ROWS):
        y = row_ys[row_idx]
        x = 4
        for abbrev, label in row:
            state = sv.get(abbrev, "?") if sv else "?"
            bg_pen, txt_pen, status_text = SV_COLORS.get(state, SV_COLORS["?"])

            display.set_pen(bg_pen)
            display.rectangle(x, y, 100, 14)

            display.set_pen(txt_pen)
            display.text(label,       x + 3,  y + 3, scale=1)
            display.text(status_text, x + 65, y + 3, scale=1)

            x += 104


def draw_no_data():
    """Placeholder values shown when no packet has been received yet."""
    draw_battery_section(None, None, None)
    draw_motors_enabled(False, {})
    draw_services_section({})


def render(data: dict, stale: bool, waiting: bool):
    """Full-screen render from a data dict."""
    clear()
    draw_title_bar(signal_ok=not stale and not waiting, waiting=waiting)

    if waiting:
        draw_no_data()
    else:
        mv = data.get("mv")
        up = data.get("up")
        ub = data.get("ub")
        me = data.get("me", False)
        ms = data.get("ms", {})
        sv = data.get("sv", {})
        draw_battery_section(mv, up, ub)
        draw_motors_enabled(me, ms)
        draw_services_section(sv)

    display.update()


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

# Initial screen — waiting for data
clear()
draw_title_bar(signal_ok=False, waiting=True)
draw_no_data()
display.update()

while True:
    # --- Non-blocking read from USB CDC ---
    if select.select([sys.stdin], [], [], 0)[0]:
        chunk = sys.stdin.read(256)
        if chunk:
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    try:
                        data = json.loads(line)
                        last_data = data
                        last_recv = time.time()
                    except ValueError:
                        pass

    # --- Determine signal status ---
    now = time.time()
    if last_recv == 0:
        waiting = True
        stale = False
    else:
        waiting = False
        stale = (now - last_recv) > STALE_TIMEOUT

    # --- Render ---
    render(last_data, stale=stale, waiting=waiting)

    # ~10 Hz redraw loop — fast enough to catch new data promptly
    time.sleep(0.1)
