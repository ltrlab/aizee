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

import _thread
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
# Logo JPEG (22×22, composited over TITLE_BG on dev machine via PIL)
# ---------------------------------------------------------------------------

LOGO_JPEG = (
    b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00'
    b'\xff\xdb\x00C\x00\x05\x03\x04\x04\x04\x03\x05\x04\x04\x04\x05\x05\x05'
    b'\x06\x07\x0c\x08\x07\x07\x07\x07\x0f\x0b\x0b\t\x0c\x11\x0f\x12\x12\x11'
    b'\x0f\x11\x11\x13\x16\x1c\x17\x13\x14\x1a\x15\x11\x11\x18!\x18\x1a\x1d'
    b'\x1d\x1f\x1f\x1f\x13\x17"$"\x1e$\x1c\x1e\x1f\x1e\xff\xdb\x00C\x01\x05'
    b'\x05\x05\x07\x06\x07\x0e\x08\x08\x0e\x1e\x14\x11\x14\x1e\x1e\x1e\x1e'
    b'\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e'
    b'\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e'
    b'\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\x1e\xff\xc0\x00\x11\x08'
    b'\x00\x16\x00\x16\x03\x01"\x00\x02\x11\x01\x03\x11\x01\xff\xc4\x00\x1f'
    b'\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00'
    b'\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xc4\x00\xb5\x10\x00'
    b'\x02\x01\x03\x03\x02\x04\x03\x05\x05\x04\x04\x00\x00\x01}\x01\x02\x03'
    b'\x00\x04\x11\x05\x12!1A\x06\x13Qa\x07"q\x142\x81\x91\xa1\x08#B\xb1\xc1'
    b'\x15R\xd1\xf0$3br\x82\t\n\x16\x17\x18\x19\x1a%&\'()*456789:CDEFGHIJ'
    b'STUVWXYZcdefghijstuvwxyz\x83\x84\x85\x86\x87\x88\x89\x8a\x92\x93\x94'
    b'\x95\x96\x97\x98\x99\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3'
    b'\xb4\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xd2'
    b'\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9'
    b'\xea\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xff\xc4\x00\x1f\x01\x00'
    b'\x03\x01\x01\x01\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x01'
    b'\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xc4\x00\xb5\x11\x00\x02\x01'
    b'\x02\x04\x04\x03\x04\x07\x05\x04\x04\x00\x01\x02w\x00\x01\x02\x03\x11'
    b'\x04\x05!1\x06\x12AQ\x07aq\x13"2\x81\x08\x14B\x91\xa1\xb1\xc1\t#3R\xf0'
    b'\x15br\xd1\n\x16$4\xe1%\xf1\x17\x18\x19\x1a&\'()*56789:CDEFGHIJSTUVWXYZ'
    b'cdefghijstuvwxyz\x82\x83\x84\x85\x86\x87\x88\x89\x8a\x92\x93\x94\x95'
    b'\x96\x97\x98\x99\x9a\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xb2\xb3\xb4'
    b'\xb5\xb6\xb7\xb8\xb9\xba\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xd2\xd3'
    b'\xd4\xd5\xd6\xd7\xd8\xd9\xda\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xf2'
    b'\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xff\xda\x00\x0c\x03\x01\x00\x02\x11'
    b'\x03\x11\x00?\x00\xf9x\x02z\x02jQkrN\x05\xbc\xa4\xe38\xd8i\xf6^YWY\n'
    b'\xe3r\x9c\x16\xc6q\x9a\xb4\x92\xdb\xb9X\xe7\x9d\xe2\x01\x9f\x1e[g\x1f'
    b'w\x19>\x9dy\xe6\xbe\xf6\x9e\x1a\x12\x82\x93{\xff\x00\x9d\x8f\nq\xe5\x87'
    b'23H \xe0\xf0h\xa9\xaf\xe53^I!\n\t=\x9b>\xdd{\xfdh\xaeI\xa4\xa4\xd2"-'
    b'\xb5vAE\x14T\x94\x14QE\x00\x7f\xff\xd9'
)

_LOGO_PURPLE = display.create_pen(102, 0, 255)

def _draw_logo_fallback(x, y):
    display.set_pen(_LOGO_PURPLE)
    display.rectangle(x, y, 22, 22)
    display.set_pen(WHITE)
    display.set_font("bitmap8")
    display.text("ltr", x + 3, y + 7, scale=1)

try:
    import jpegdec as _jd
    _jpeg = _jd.JPEG(display)
    def draw_logo(x, y):
        try:
            _jpeg.open_RAM(LOGO_JPEG)
            _jpeg.decode(x, y, _jd.JPEG_SCALE_FULL)
        except Exception:
            _draw_logo_fallback(x, y)
except ImportError:
    draw_logo = _draw_logo_fallback

# ---------------------------------------------------------------------------
# Border animation constants
# ---------------------------------------------------------------------------

BORDER_W       = 10
WAVE_COUNT     = 3
WAVE_SPEED     = 0.07    # perimeter laps/s → 1 full lap ≈ 14 s
WAVE_HALFWIDTH = 0.12    # bell half-width as fraction of perimeter

# Colour rules — evaluated top-to-bottom, first True wins
BORDER_RULES = [
    ("yellow", lambda sv, mv, me, stale, waiting: waiting),
    ("red",    lambda sv, mv, me, stale, waiting: stale),
    ("red",    lambda sv, mv, me, stale, waiting: any(v == "f" for v in sv.values())),
    ("green",  lambda sv, mv, me, stale, waiting: me),
    ("blue",   lambda sv, mv, me, stale, waiting: True),
]
BORDER_RGBS = {
    "red":    (255,  60,  60),
    "green":  ( 60, 255, 120),
    "blue":   ( 80, 180, 255),
    "yellow": (255, 240,  40),
}

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
MOTOR_FULL   = 25.2   # V (100%) — 6S LiPo @ 4.2 V/cell
MOTOR_MIN    = 19.8   # V (0%)   — 6S LiPo @ 3.3 V/cell

# ---------------------------------------------------------------------------
# Motor abbreviation / display order
# ---------------------------------------------------------------------------

MOTOR_IDS = ["lw", "rw", "sw", "gb", "gm", "ge", "wp", "wr", "gr"]
BASE_MOTORS = {"lw", "rw", "sw"}

# Motor state char → (bg_pen, text_pen)
STATE_COLORS = {
    "r": (GREEN,     BLACK),
    "e": (YELLOW,    BLACK),
    "d": (YELLOW,    BLACK),   # disabled → yellow
    "x": (RED,       WHITE),
    "?": (RED,       WHITE),   # unknown/off → red
}

# ---------------------------------------------------------------------------
# Service display layout
# ---------------------------------------------------------------------------

# Two rows of (abbrev, display_label) pairs.  Tiles are 70 px wide (4 per
# row at the 320 px display width) so we can fit 7 services without
# pushing the Pi section off the bottom edge.  Labels are 4 chars max so
# they fit alongside the 3-char status text inside a 70 px tile.
SERVICES_ROWS = [
    [("motors", "MOTR"), ("lidar", "LIDR"), ("ups",  "UPS"),  ("relay", "RLAY")],
    [("disp",   "DISP"), ("armL",  "ARML"), ("armR", "ARMR")],
]

# Pi state char → (bg_pen, text_pen, status_label)
PI_COLORS = {
    "u": (GREEN,     BLACK, "UP"),
    "d": (RED,       WHITE, "DN"),
    "?": (DARK_GRAY, GRAY,  "??"),
}

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
# Border segment table (computed once at startup)
# ---------------------------------------------------------------------------

def _build_border_segs():
    """Return list of (x, y, w, h, frac) for all 56 border segments.

    Clockwise: Top (L→R) 16 segs, Right (T→B) 12 segs,
               Bottom (R→L) 16 segs, Left (B→T) 12 segs.
    frac ∈ [0,1) = clockwise position along perimeter.
    """
    segs = []
    W, H, B = WIDTH, HEIGHT, BORDER_W
    total = 56
    # Top: L→R
    for i in range(16):
        segs.append((i * 20, 0, 20, B, i / total))
    # Right: T→B
    for i in range(12):
        segs.append((W - B, i * 20, B, 20, (16 + i) / total))
    # Bottom: R→L
    for i in range(16):
        segs.append(((15 - i) * 20, H - B, 20, B, (28 + i) / total))
    # Left: B→T
    for i in range(12):
        segs.append((0, (11 - i) * 20, B, 20, (44 + i) / total))
    return segs

_BORDER_SEGS = _build_border_segs()

# Lock and shared state for dual-core border animation (Core 1)
_lock     = _thread.allocate_lock()
_bsv      = {}
_bmv      = None
_bme      = False
_bstale   = False
_bwaiting = True

# ---------------------------------------------------------------------------
# Section icons (8×8 px, drawn with primitives)
# ---------------------------------------------------------------------------

def icon_bolt(x, y, p):          # ⚡ battery/power
    display.set_pen(p)
    display.rectangle(x + 3, y,     3, 4)
    display.rectangle(x + 1, y + 3, 6, 2)
    display.rectangle(x,     y + 4, 3, 4)


def icon_motor(x, y, p):         # ◎ motors
    display.set_pen(p)
    display.circle(x + 4, y + 4, 4)
    display.set_pen(BG)
    display.circle(x + 4, y + 4, 2)


def icon_gear(x, y, p):          # ⚙ services
    display.set_pen(p)
    display.circle(x + 4, y + 4, 3)
    for dx, dy in [(3, 0), (3, 5), (0, 3), (5, 3)]:
        display.rectangle(x + dx, y + dy, 2, 2)


def icon_net(x, y, p):           # ◈ pies/network
    display.set_pen(p)
    for cx, cy in [(2, 2), (6, 2), (2, 6), (6, 6)]:
        display.circle(x + cx, y + cy, 1)

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


def draw_section_line(y):
    display.set_pen(DARK_BG)
    display.rectangle(10, y, 300, 2)


def draw_border(sv, mv, me, stale, waiting):
    rgb = BORDER_RGBS["blue"]
    for label, fn in BORDER_RULES:
        if fn(sv, mv, me, stale, waiting):
            rgb = BORDER_RGBS[label]
            break
    phase = (time.ticks_ms() * WAVE_SPEED / 1000) % 1.0
    br, bg_c, bb = rgb
    for x, y, w, h, frac in _BORDER_SEGS:
        intensity = 0.0
        for k in range(WAVE_COUNT):
            d = frac - (phase + k / WAVE_COUNT) % 1.0
            if d >  0.5: d -= 1.0
            elif d < -0.5: d += 1.0
            d = abs(d)
            if d < WAVE_HALFWIDTH:
                t = 1.0 - d / WAVE_HALFWIDTH
                intensity += t * t        # quadratic bell — no trig
        intensity = min(1.0, intensity / WAVE_COUNT)
        factor = 0.5 + 0.5 * intensity
        display.set_pen(display.create_pen(
            min(255, int(br * factor)),
            min(255, int(bg_c * factor)),
            min(255, int(bb * factor)),
        ))
        display.rectangle(x, y, w, h)


def draw_title_bar(signal_ok, waiting, ip=""):
    """Dark-blue title bar: y=10..36 (26 px), with logo and status."""
    display.set_pen(TITLE_BG)
    display.rectangle(10, 10, 300, 26)

    draw_logo(12, 12)   # 22×22 logo, vertically centred in 26 px bar

    display.set_pen(WHITE)
    display.set_font("bitmap8")
    display.text("AIZEE STATUS", 38, 19, scale=1)

    if ip:
        display.set_pen(GRAY)
        display.text(ip, 130, 19, scale=1)

    if waiting:
        display.set_pen(YELLOW)
        display.text("WAITING...", 220, 19, scale=1)
    elif signal_ok:
        display.set_pen(GREEN)
        display.text("OK", 282, 19, scale=1)
    else:
        display.set_pen(RED)
        display.text("NO SIGNAL", 232, 19, scale=1)

    draw_section_line(36)


def draw_battery_section(mv, up, ub, mp=None):
    """
    Left half: Jetson UPS battery  |  Right half: Motor bus voltage
    y range: 38–90 (52 px)
    """
    display.set_font("bitmap8")

    # --- Section labels with icons ---
    icon_bolt(14, 39, GRAY)
    display.set_pen(GRAY)
    display.text("JETSON BATTERY", 24, 40, scale=1)

    icon_bolt(165, 39, GRAY)
    display.set_pen(GRAY)
    display.text("MOTOR BATTERY", 175, 40, scale=1)

    # --- UPS voltage (large) ---
    up_color = color_for_ups_voltage(up)
    display.set_pen(up_color)
    if up is None:
        display.text("---", 14, 50, scale=2)
    else:
        display.text("{:.1f}V".format(up), 14, 50, scale=2)

    # --- UPS percentage ---
    if ub is None:
        display.set_pen(GRAY)
        display.text("---%", 90, 50, scale=2)
    else:
        pct_color = GREEN if ub >= 50 else (YELLOW if ub >= 20 else RED)
        display.set_pen(pct_color)
        display.text("{:3d}%".format(ub), 90, 50, scale=2)

    # --- UPS progress bar ---
    bar_x, bar_y, bar_w, bar_h = 14, 74, 143, 10
    display.set_pen(GRAY)
    display.rectangle(bar_x, bar_y, bar_w, bar_h)
    display.set_pen(DARK_BG)
    display.rectangle(bar_x + 1, bar_y + 1, bar_w - 2, bar_h - 2)
    if ub is not None:
        fill_w = int((bar_w - 2) * max(0, min(ub, 100)) / 100)
        bar_color = GREEN if ub >= 50 else (YELLOW if ub >= 20 else RED)
        display.set_pen(bar_color)
        display.rectangle(bar_x + 1, bar_y + 1, fill_w, bar_h - 2)

    # --- Vertical divider between halves ---
    display.set_pen(DARK_BG)
    display.rectangle(157, 36, 4, 54)

    # --- Motor voltage (large) ---
    mv_color = color_for_motor_voltage(mv)
    display.set_pen(mv_color if mv is not None else GRAY)
    if mv is None:
        display.text("DC", 167, 50, scale=2)
    else:
        display.text("{:.1f}V".format(mv), 167, 50, scale=2)
        if mp is not None:
            pct_color = GREEN if mp >= 50 else (YELLOW if mp >= 20 else RED)
            display.set_pen(pct_color)
            display.text("{:3d}%".format(mp), 237, 50, scale=2)

    # --- Motor battery bar ---
    bar_x, bar_y, bar_w, bar_h = 165, 74, 143, 10
    display.set_pen(GRAY)
    display.rectangle(bar_x, bar_y, bar_w, bar_h)
    display.set_pen(DARK_BG)
    display.rectangle(bar_x + 1, bar_y + 1, bar_w - 2, bar_h - 2)
    if mv is not None and mp is not None:
        fill_w = int((bar_w - 2) * max(0, min(mp, 100)) / 100)
        bar_color = GREEN if mp >= 50 else (YELLOW if mp >= 20 else RED)
        display.set_pen(bar_color)
        display.rectangle(bar_x + 1, bar_y + 1, fill_w, bar_h - 2)


def draw_motors_enabled(me, ms, mpos=None):
    """
    Motor enable status pill (y=92–108) + per-motor state boxes (y=110–156).
    mpos: dict of abbrev → position in radians (optional).
    Boxes are 50×14 px to fit motor label + position value.
    """
    draw_section_line(90)

    # --- Enable/Disable pill ---
    icon_motor(14, 93, GRAY)
    display.set_pen(GRAY)
    display.set_font("bitmap8")
    display.text("MOTORS:", 24, 95, scale=1)

    if me:
        display.set_pen(GREEN)
        display.rectangle(80, 93, 90, 14)
        display.set_pen(BLACK)
        display.text("ENABLED", 84, 97, scale=1)
    else:
        display.set_pen(RED)
        display.rectangle(80, 93, 90, 14)
        display.set_pen(WHITE)
        display.text("DISABLED", 84, 97, scale=1)

    draw_section_line(108)

    # --- Motor state boxes ---
    # Row 1 (BASE): lw, rw, sw  y=110
    # Row 2 (ARM):  gb, gm, ge  y=126
    # Row 3 (ARM):  wp, wr, gr  y=142
    # Box: 50×14 px, step 56 px — fits "xx+X.X" (6 chars × 8 px = 48 px)

    display.set_pen(GRAY)
    display.text("BASE:", 14, 114, scale=1)
    display.text("ARM: ", 14, 130, scale=1)

    rows = [
        (["lw", "rw", "sw"], 110),
        (["gb", "gm", "ge"], 126),
        (["wp", "wr", "gr"], 142),
    ]
    for motor_row, y in rows:
        x = 50
        for mid in motor_row:
            state_char = ms.get(mid, "?") if ms else "?"
            bg_pen, txt_pen = STATE_COLORS.get(state_char, (RED, WHITE))
            display.set_pen(bg_pen)
            display.rectangle(x, y, 50, 14)
            display.set_pen(txt_pen)
            pos = mpos.get(mid) if mpos else None
            if pos is not None:
                label = "{}{}".format(mid, "{:+.1f}".format(pos))
            else:
                label = "{}[{}]".format(mid, state_char.upper())
            display.text(label, x + 2, y + 3, scale=1)
            x += 56


def draw_services_section(sv):
    """
    Systemd service status grid — two rows of coloured boxes.
    y range: 158–204 (46 px)
    """
    draw_section_line(156)

    icon_gear(14, 159, GRAY)
    display.set_pen(GRAY)
    display.set_font("bitmap8")
    display.text("SERVICES:", 24, 160, scale=1)

    row_ys = [172, 190]
    # Tile geometry: 70 px wide, 74 px stride — fits 4 tiles + left margin
    # within the 320 px display width.  Status text is right-aligned
    # within the tile so 4-char labels and 3-char states never collide.
    for row_idx, row in enumerate(SERVICES_ROWS):
        y = row_ys[row_idx]
        x = 14
        for abbrev, label in row:
            state = sv.get(abbrev, "?") if sv else "?"
            bg_pen, txt_pen, status_text = SV_COLORS.get(state, SV_COLORS["?"])
            display.set_pen(bg_pen)
            display.rectangle(x, y, 70, 14)
            display.set_pen(txt_pen)
            display.text(label,       x + 3,  y + 3, scale=1)
            display.text(status_text, x + 45, y + 3, scale=1)
            x += 74


def draw_pi_section(pi):
    """
    RPi camera node reachability row.
    y range: 206–230 (24 px)
    """
    draw_section_line(204)

    icon_net(14, 207, GRAY)
    display.set_pen(GRAY)
    display.set_font("bitmap8")
    display.text("PIES:", 24, 208, scale=1)

    # 4 × 70 px boxes + 4 px gaps = 292 px starting at x=14
    x = 14
    for key, label in [("pi1", "P1"), ("pi2", "P2"), ("pi3", "P3"), ("pi4", "P4")]:
        state = pi.get(key, "?") if pi else "?"
        bg_pen, txt_pen, status = PI_COLORS.get(state, PI_COLORS["?"])
        display.set_pen(bg_pen)
        display.rectangle(x, 216, 70, 14)
        display.set_pen(txt_pen)
        display.text("{}:{}".format(label, status), x + 4, 219, scale=1)
        x += 74


def draw_no_data():
    """Placeholder values shown when no packet has been received yet."""
    draw_battery_section(None, None, None, None)
    draw_motors_enabled(False, {})
    draw_services_section({})
    draw_pi_section({})


def render_content(data, stale, waiting):
    """Redraw content area (no border/update — Core 1 handles those)."""
    global _bsv, _bmv, _bme, _bstale, _bwaiting
    sv  = data.get("sv", {}) if data else {}
    mv  = data.get("mv")     if data else None
    me  = data.get("me", False) if data else False
    ip  = data.get("ip", "") if data else ""
    mp  = data.get("mp")     if data else None
    up  = data.get("up")     if data else None
    ub  = data.get("ub")     if data else None
    ms   = data.get("ms",   {}) if data else {}
    mpos = data.get("mpos", {}) if data else {}
    pi   = data.get("pi",   {}) if data else {}
    _lock.acquire()
    try:
        _bsv, _bmv, _bme, _bstale, _bwaiting = sv, mv, me, stale, waiting
        clear()
        draw_title_bar(signal_ok=not stale and not waiting, waiting=waiting, ip=ip)
        if waiting:
            draw_no_data()
        else:
            draw_battery_section(mv, up, ub, mp)
            draw_motors_enabled(me, ms, mpos)
            draw_services_section(sv)
            draw_pi_section(pi)
    finally:
        _lock.release()


def _border_loop():
    """Border animation — runs on Core 1 at ~30 Hz."""
    while True:
        _lock.acquire()
        try:
            draw_border(_bsv, _bmv, _bme, _bstale, _bwaiting)
            display.update()
        finally:
            _lock.release()
        time.sleep_ms(33)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

# Initial screen — draw once before starting border thread
clear()
draw_title_bar(signal_ok=False, waiting=True, ip="")
draw_no_data()
draw_border({}, None, False, False, True)
display.update()

# Start border animation on Core 1
_thread.start_new_thread(_border_loop, ())

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

    # --- Render content (Core 1 handles border + display.update) ---
    render_content(last_data, stale=stale, waiting=waiting)

    time.sleep_ms(100)
