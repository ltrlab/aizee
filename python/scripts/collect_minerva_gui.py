"""collect_minerva_gui.py — PySide6 control panel for collect_minerva.py.

Mirrors AIZEE's collect_demo_gui architecture (`QtRenderer` on a worker thread,
lock-guarded display + camera holders, a key queue for buttons) but for
Minerva's bimanual, 3-camera collector, with direct Qt raw-JPEG painting (the
same approach AIZEE's `_LiveCameraPair` actually uses).

Contract with collect_minerva.py:
    qt.lock / qt.holder        — main loop writes holder["args"] = snapshot dict
    qt.cam_lock / qt.cam_holder — main loop writes {name: jpeg_bytes, name_ts: ts}
    qt.cmd_queue                — buttons post single-char keys ("E","H","T","R","Q")
    qt.teleop                   — jog buttons call teleop.jog(idx, ±1) directly
    qt.start()/should_quit()/request_quit()/join()

Layout:
    ┌───────────────────────────── status pills ─────────────────────────────┐
    │  left_wrist | right_wrist | head        ║  RECORD (big)                  │
    │        (3 live camera tiles)            ║  instruction: [__________]     │
    │                                         ║  [Enable][Teleop][Disable]     │
    ├─────────────────── 17-DoF joint bars (target vs actual) ────────────────┤
    │  shortcut legend                                    last saved: …        │
    └─────────────────────────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import h5py

from PySide6.QtCore import Qt, QTimer, QThread, QPointF, Signal
from PySide6.QtGui import (
    QColor, QFont, QFontMetrics, QImage, QKeySequence, QPainter, QPen, QPixmap,
    QPolygonF, QShortcut,
)
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFormLayout, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QProgressBar,
    QPushButton, QSizePolicy, QSlider, QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

from collect_minerva_app.settings import CollectorSettings, list_input_devices

# Controlled vocabulary of sub-action labels (editable — free text also allowed).
_ACTION_VOCAB = [
    "reach", "grasp", "lift", "move", "align", "insert", "place",
    "release", "open gripper", "close gripper", "retract", "push", "pull",
]

from common.minerva_constants import (
    IDX, JOINT_LIMITS, KP, KD, MINERVA_JOINTS, NUM_MINERVA_JOINTS, SAT_TORQUE)

# Joint display groups (label, index list).
_GROUPS = [
    ("Left arm", list(range(0, 6))),
    ("L grip", [6]),
    ("Right arm", list(range(7, 13))),
    ("R grip", [13]),
    ("Head", [14, 15]),
    ("Lift", [16]),
]
_SHORT = {0: "j1", 1: "j2", 2: "j3", 3: "j4", 4: "j5", 5: "j6", 6: "grip",
          7: "j1", 8: "j2", 9: "j3", 10: "j4", 11: "j5", 12: "j6", 13: "grip",
          14: "pan", 15: "tilt", 16: "lift"}

# Per-group trace colours (joint index -> colour) for the episode joint trace.
_GROUP_COLOR = {
    "Left arm": "#ffb300", "L grip": "#ff7043", "Right arm": "#26c6da",
    "R grip": "#4db6ac", "Head": "#ba68c8", "Lift": "#66bb6a",
}
_JOINT_COLOR: Dict[int, str] = {}
for _lbl, _idxs in _GROUPS:
    for _j in _idxs:
        _JOINT_COLOR[_j] = _GROUP_COLOR[_lbl]


def _fmt_qpos(q) -> str:
    """Compact 3-line numeric readout of a 17-DoF frame."""
    L = " ".join(f"{q[i]:+.2f}" for i in range(0, 6))
    R = " ".join(f"{q[i]:+.2f}" for i in range(7, 13))
    return (f"L {L}  grip{q[6]:+.2f}\n"
            f"R {R}  grip{q[13]:+.2f}\n"
            f"head {q[14]:+.2f}/{q[15]:+.2f}   lift {q[16]:+.3f}")


def _parse_seg_attr(raw) -> list:
    """Parse an HDF5 `segments` attr (JSON) into [{start,end,label}, ...]."""
    if raw is None:
        return []
    try:
        if isinstance(raw, bytes):
            raw = raw.decode("utf-8")
        segs = json.loads(raw)
        return [s for s in segs if isinstance(s, dict) and {"start", "end", "label"} <= set(s)]
    except Exception:
        return []


_BAND_COLORS = ["#33478a", "#3a6b35", "#7a4a1e", "#5a2d6b", "#155e63", "#6b2020"]


# ---------------------------------------------------------------------------
# Small widgets
# ---------------------------------------------------------------------------

class _Pill(QLabel):
    def __init__(self, text: str = ""):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setFixedHeight(26)
        self.setMinimumWidth(90)
        self._set("#444")

    def _set(self, color: str):
        self.setStyleSheet(
            f"background:{color};color:white;padding:2px 8px;font-weight:bold;")

    def update_pill(self, text: str, ok: Optional[bool] = None):
        self.setText(text)
        if ok is None:
            self._set("#555")
        else:
            self._set("#2e7d32" if ok else "#c62828")


# Status-dashboard palette (readable on the dark top bar).
_C_OK, _C_WARN, _C_HOT, _C_BAD, _C_IDLE, _C_INFO = (
    "#5cb85c", "#e0a800", "#e07b00", "#e05252", "#9aa0a6", "#4a90d9")
_STATE_COLORS = {
    "DISABLED": _C_IDLE, "IDLE": _C_INFO, "HOLD": _C_INFO,
    "ENGAGING": _C_WARN, "TELEOP": _C_OK, "SHUTDOWN": _C_HOT,
}


def _tempbar_color(t: float) -> str:
    return _C_OK if t < 45 else _C_WARN if t < 60 else _C_HOT if t < 72 else _C_BAD


class _StatBox(QFrame):
    """A CAPTION + big value tile for the top status dashboard. Value text and color
    update live via set(). Matches the heartbeat page's at-a-glance style."""

    def __init__(self, caption: str, width: int = 84):
        super().__init__()
        self.setMinimumWidth(width)
        self.setFixedHeight(42)
        self.setStyleSheet("QFrame { background:#2b2f34; border:1px solid #3a3f45; border-radius:0px; }")
        v = QVBoxLayout(self)
        v.setContentsMargins(9, 3, 9, 3)
        v.setSpacing(0)
        self._cap = QLabel(caption)
        self._cap.setAlignment(Qt.AlignCenter)
        self._cap.setStyleSheet("color:#868d95; font-size:9px; font-weight:700; border:none;")
        self._val = QLabel("—")
        self._val.setAlignment(Qt.AlignCenter)
        self._val.setStyleSheet("color:#e8e8e8; font-size:15px; font-weight:800; border:none;")
        v.addWidget(self._cap)
        v.addWidget(self._val)

    def set(self, value, color: Optional[str] = None):
        self._val.setText(str(value))
        self._val.setStyleSheet(
            f"color:{color or '#e8e8e8'}; font-size:15px; font-weight:800; border:none;")


# GUI panel layout order (left -> right). The canonical CAMERAS order is kept for
# recording/policy; this only arranges the on-screen panels: head/scene camera in the
# CENTER, and the two wrist views swapped so they match the operator's physical
# perspective facing the robot. Cameras not listed here trail in their given order.
_CAM_DISPLAY_ORDER = ("right_wrist", "head", "left_wrist")


def _cam_display_order(cameras):
    """Reorder a camera-name list for on-screen panels (see _CAM_DISPLAY_ORDER)."""
    known = [c for c in _CAM_DISPLAY_ORDER if c in cameras]
    return known + [c for c in cameras if c not in _CAM_DISPLAY_ORDER]


# Stylized control-button palette. role -> (bg, hover, pressed, fg).
_BTN_ROLES = {
    "neutral": ("#3a3f45", "#484e56", "#2c3036", "#e9e9e9"),
    "go":      ("#2e7d32", "#369a3c", "#1f5c24", "#ffffff"),  # Teleop
    "info":    ("#1565c0", "#1c76d6", "#0e4a92", "#ffffff"),  # Enable
    "warn":    ("#e07b00", "#f28c10", "#b56400", "#ffffff"),  # Disable
    "danger":  ("#c62828", "#d84343", "#9e1c1c", "#ffffff"),  # Shutdown / Quit
}

# Tab bar styling for the right-hand Controls / Recorded tabs.
_TAB_QSS = (
    "QTabWidget::pane { border:1px solid #3a3f45; border-radius:0px; top:-1px; }"
    " QTabBar::tab { background:#2b2f34; color:#b9c0c7; padding:9px 22px; margin-right:3px;"
    " border-top-left-radius:0px; border-top-right-radius:0px; font-size:13px; font-weight:700; }"
    " QTabBar::tab:selected { background:#3a3f45; color:#ffffff; }"
    " QTabBar::tab:hover { background:#343a41; }"
)


def _style_btn(btn, role="neutral", height=38, font=13):
    """Give a QPushButton a flat, rounded, hover-aware look (like the record button)."""
    bg, hover, press, fg = _BTN_ROLES.get(role, _BTN_ROLES["neutral"])
    btn.setMinimumHeight(height)
    btn.setCursor(Qt.PointingHandCursor)
    btn.setStyleSheet(
        f"QPushButton {{ background:{bg}; color:{fg}; border:none; border-radius:0px;"
        f" padding:6px 8px; font-size:{font}px; font-weight:600; }}"
        f" QPushButton:hover {{ background:{hover}; }}"
        f" QPushButton:pressed {{ background:{press}; }}"
        f" QPushButton:disabled {{ background:#2a2d31; color:#6a6a6a; }}")
    return btn


class _CameraPanel(QLabel):
    """Paints a camera's latest JPEG scaled to fit."""

    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.setMinimumSize(240, 180)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background:#111;color:#888;border:1px solid #333;")
        self.setText(f"{name}\n(no signal)")
        self.setToolTip(f"{name} camera")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._last_ts = None

    def set_jpeg(self, jpeg: bytes, ts=None):
        """Paint the JPEG. Returns the decoded source (w, h) on a *new* frame, or
        None when the frame is a repeat (same ts) or fails to decode — the tile
        uses that to drive its live resolution/FPS readout only on real frames."""
        if ts is not None and ts == self._last_ts:
            return None
        self._last_ts = ts
        img = QImage()
        if not img.loadFromData(jpeg, "JPEG"):
            return None
        pm = QPixmap.fromImage(img)
        self.setPixmap(pm.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        return img.width(), img.height()

    def set_rgb(self, arr) -> None:
        """Paint a decoded uint8 RGB frame [H,W,3] (recorded-episode playback)."""
        if arr is None or arr.ndim != 3:
            return
        h, w = arr.shape[:2]
        # .copy() so QImage owns the buffer (arr.tobytes() would be GC'd otherwise).
        img = QImage(arr.tobytes(), w, h, 3 * w, QImage.Format_RGB888).copy()
        pm = QPixmap.fromImage(img)
        self.setPixmap(pm.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


# Per-camera display identity for the live tiles: (title, subtitle, accent). Keyed by
# the canonical camera name — the accent tints the title chip + the divider under the
# feed, giving each viewport a recognisable signature the operator learns at a glance.
#
# NOTE the deliberate cross-labelling: the on-screen order is mirror-swapped
# (_CAM_DISPLAY_ORDER) so each feed sits on the operator's matching side while they face
# the robot. The label therefore names the OPERATOR'S side (which screen half it's on),
# not the hardware wrist producing the feed — so the `right_wrist` camera, shown on the
# operator's left, reads "LEFT WRIST", and vice-versa. Positions right, names match.
_CAM_META = {
    "right_wrist": ("LEFT WRIST",  "gripper", "#4ad8ff"),   # operator-left  → cyan
    "left_wrist":  ("RIGHT WRIST", "gripper", "#7ee787"),   # operator-right → green
    "head":        ("SCENE",       "overhead", "#ffcf4a"),  # centre         → amber
}
_CAM_META_DEFAULT = ("CAMERA", "stream", "#9aa0a6")


def _spaced_font(px: int, weight, spacing: int = 120) -> QFont:
    """A pixel-sized, letter-spaced font for the little uppercase HUD chips."""
    f = QFont()
    f.setPixelSize(px)
    f.setWeight(weight)
    f.setLetterSpacing(QFont.PercentageSpacing, spacing)
    return f


def _port_of(url: Optional[str]) -> Optional[str]:
    """'tcp://10.42.0.1:5563' -> '5563' (best effort; None if unparseable)."""
    if not url or ":" not in url:
        return None
    tail = url.rsplit(":", 1)[-1].strip()
    return tail if tail.isdigit() else None


class _CameraTile(QFrame):
    """A framed camera monitor: solid HUD chips OVER the live feed (a title plate top-
    left, a LIVE/STALE status lamp top-right) and a specs strip UNDER it (resolution ·
    measured FPS · source port · frame age). No gradients or translucency touch the
    video — the chips are opaque plates and the feed itself is never tinted."""

    def __init__(self, name: str, endpoint: Optional[str] = None,
                 cfg_size: Optional[tuple] = None):
        super().__init__()
        self.name = name
        title, subtitle, accent = _CAM_META.get(name, _CAM_META_DEFAULT)
        self.accent = accent
        self._port = _port_of(endpoint)
        self._cfg_wh = tuple(cfg_size) if cfg_size else None
        self._src_wh: Optional[tuple] = None
        self._fps: Optional[float] = None
        self._t_prev: Optional[float] = None
        self.setObjectName("camTile")   # ID selector so the frame border can't bleed to children
        self.setStyleSheet("#camTile { background:#0b0d10; border:1px solid #23282e; }")

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # -- feed + overlay chips (z-stacked in one grid cell) --
        stack = QWidget()
        grid = QGridLayout(stack)
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setSpacing(0)
        self.panel = _CameraPanel(name)
        self.panel.setStyleSheet("background:#000; color:#586069; border:none;")
        self.panel.setText(f"{title}\n(no signal)")
        grid.addWidget(self.panel, 0, 0)

        self.title_chip = QLabel(title)
        self.title_chip.setFont(_spaced_font(11, QFont.DemiBold, 140))
        # Role + true hardware camera on hover (the label is operator-facing, so the
        # feed behind "LEFT WRIST" is actually the right_wrist camera — see _CAM_META).
        self.title_chip.setToolTip(f"{subtitle} · {name}")
        self.title_chip.setStyleSheet(
            f"color:{accent}; background:#12151a; border:none;"
            " border-radius:0px; padding:3px 10px;")
        grid.addWidget(self.title_chip, 0, 0, Qt.AlignTop | Qt.AlignLeft)

        self.status_chip = QLabel("○ NO SIGNAL")
        self.status_chip.setFont(_spaced_font(10, QFont.Bold, 110))
        self.status_chip.setAlignment(Qt.AlignCenter)
        # Fixed width sized to the widest state so the pill never resizes (and its live
        # age moved to the footer) — the chip must not twitch the tile's width.
        _fm = QFontMetrics(self.status_chip.font())
        _w = max(_fm.horizontalAdvance(s)
                 for s in ("● LIVE", "● STALE", "● LOST", "○ NO SIGNAL"))
        self.status_chip.setFixedWidth(_w + 24)
        self.status_chip.setStyleSheet(
            "color:#9aa0a6; background:#12151a; border:none;"
            " border-radius:0px; padding:3px 10px;")
        grid.addWidget(self.status_chip, 0, 0, Qt.AlignTop | Qt.AlignRight)
        outer.addWidget(stack, 1)

        # -- specs strip under the feed (accent divider = the tile's signature) --
        self.footer = QLabel(self._footer_text())
        ff = QFont("Consolas")
        ff.setStyleHint(QFont.Monospace)
        ff.setPixelSize(11)
        self.footer.setFont(ff)
        # Ignored horizontal policy: the footer fills the tile but its text width never
        # feeds back into the tile's minimum, so a changing readout can't reflow the row.
        self.footer.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        self.footer.setStyleSheet(
            f"color:#8b939c; background:#12151a; border:none;"
            f" border-top:2px solid {accent}; padding:4px 11px;")
        outer.addWidget(self.footer, 0)

    # -- footer composition ------------------------------------------------
    def _footer_text(self, age_ms: Optional[int] = None) -> str:
        # Every numeric field is FIXED-WIDTH (right-justified, monospace) so the string's
        # pixel width never changes as digit counts do — that width churn is what made the
        # tiles jitter sideways frame-to-frame. Tight " · " separators keep it compact.
        wh = self._src_wh or self._cfg_wh
        res = f"{wh[0]}×{wh[1]}" if wh else "—×—"
        fps = f"{min(self._fps, 99):>2.0f} fps" if self._fps else "-- fps"
        port = f":{self._port}" if self._port else "tcp"
        age = f"{min(age_ms, 9999):>4d} ms" if age_ms is not None else "  -- ms"
        return f"{res} · {fps} · {port} · {age}"

    # -- live updates ------------------------------------------------------
    def set_jpeg(self, jpeg: bytes, ts=None) -> None:
        wh = self.panel.set_jpeg(jpeg, ts)
        if wh is None:
            return                       # repeat frame / decode fail — no new data
        self._src_wh = wh
        now = time.monotonic()
        if self._t_prev is not None:
            dt = now - self._t_prev
            if dt > 1e-4:
                inst = 1.0 / dt
                self._fps = inst if self._fps is None else 0.8 * self._fps + 0.2 * inst
        self._t_prev = now

    def set_rgb(self, arr) -> None:       # kept for parity; live tiles use set_jpeg
        self.panel.set_rgb(arr)

    def set_status(self, age: Optional[float]) -> None:
        """Drive the top-right lamp + the footer's age field from the collector's
        authoritative per-camera age (seconds); None = no telemetry for this cam."""
        if age is None or age >= 999.0:
            self._fps = None
            self._t_prev = None
            self.status_chip.setText("○ NO SIGNAL")
            self.status_chip.setStyleSheet(
                "color:#9aa0a6; background:#12151a; border:none;"
                " border-radius:0px; padding:3px 10px;")
            self.footer.setText(self._footer_text(None))
            return
        age_ms = int(age * 1000)
        if age < 0.5:
            lamp, col = "● LIVE", _C_OK
        elif age < 1.5:
            lamp, col = "● STALE", _C_WARN
        else:
            lamp, col = "● LOST", _C_BAD
        self.status_chip.setText(lamp)   # numeric age lives in the footer (fixed-width)
        self.status_chip.setStyleSheet(
            f"color:{col}; background:#12151a; border:none;"
            " border-radius:0px; padding:3px 10px;")
        self.footer.setText(self._footer_text(age_ms))


# Leader↔follower tracking-error buckets (radians) + palette — mirrors collect_demo's
# _TrackingStrip (_ERR_TIGHT/_ERR_LOOSE, COL_OK/WARN/CRIT).
_ERR_TIGHT = 0.05   # <= green: aligned, safe to engage
_ERR_LOOSE = 0.15   # <= yellow: mid; above = red (too far to engage safely)
_COL_OK, _COL_WARN, _COL_CRIT = "#2a8a3d", "#b88710", "#c42020"
_COL_LEADER, _COL_ACTUAL, _COL_TARGET = "#4ad8ff", "#33465a", "#ffd84a"


def _err_color(err: float) -> str:
    return _COL_OK if err < _ERR_TIGHT else (_COL_WARN if err < _ERR_LOOSE else _COL_CRIT)


class _DiffBar(QWidget):
    """Per-joint bar showing the leader↔follower DIFFERENCE. Dim fill = follower
    position within limits; a coloured band spans follower→leader (green when the
    gap < 0.05 rad, yellow < 0.15, red beyond); a cyan marker is the leader, a yellow
    marker the commanded target. This is the safe-engage gauge — drive every bar green,
    then engage and the follower won't snap."""

    def __init__(self, lo: float, hi: float):
        super().__init__()
        self.lo, self.hi = float(lo), float(hi)
        self.actual: Optional[float] = None
        self.leader: Optional[float] = None
        self.target: Optional[float] = None
        self.setMinimumHeight(16)
        self.setMinimumWidth(60)          # never let the splitter squish it invisible
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_values(self, actual, leader, target):
        self.actual, self.leader, self.target = actual, leader, target
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#161616"))
        a = self.actual
        if a is None or a != a:            # None / NaN (dropped arm) → blank track
            p.end()
            return
        led = self.leader
        tgt = self.target
        # Auto-scale the axis to always contain the joint limits AND the live markers,
        # so a leader that sweeps past the follower's range (or an arm sitting outside its
        # placeholder limits) is shown in-scale instead of pinned to an edge. The gap
        # COLOUR is derived from |leader-actual|, independent of the axis, so alignment
        # stays readable either way.
        vals = [self.lo, self.hi, float(a)]
        if led is not None and led == led:
            vals.append(float(led))
        if tgt is not None and tgt == tgt:
            vals.append(float(tgt))
        axis_lo, axis_hi = min(vals), max(vals)
        pad = max(axis_hi - axis_lo, 1e-6) * 0.05
        axis_lo -= pad
        axis_hi += pad
        rng = axis_hi - axis_lo

        def _X(v):
            return int((float(v) - axis_lo) / rng * (w - 1))

        # subtle band = the follower's valid limit range (context)
        bx0, bx1 = _X(self.lo), _X(self.hi)
        p.fillRect(bx0, 0, max(1, bx1 - bx0), h, QColor("#232b34"))
        ax = _X(a)
        if led is not None and led == led:
            lx = _X(led)
            col = _err_color(abs(float(led) - float(a)))
            x0, x1 = sorted((ax, lx))
            p.fillRect(x0, 2, max(2, x1 - x0), h - 4, QColor(col))   # the DIFFERENCE band
            p.setPen(QPen(QColor(_COL_LEADER), 2))
            p.drawLine(lx, 0, lx, h)                            # leader marker (cyan)
        if tgt is not None and tgt == tgt:
            p.setPen(QPen(QColor(_COL_TARGET), 1))
            p.drawLine(_X(tgt), 0, _X(tgt), h)                  # commanded-target marker
        p.setPen(QPen(QColor("#f0f0f0"), 2))
        p.drawLine(ax, 0, ax, h)                                # follower marker (white)
        p.end()


# ---- per-joint temperature + torque indicators ----
_TEMP_COOL, _TEMP_WARM, _TEMP_HOT = 45.0, 60.0, 72.0   # °C bucket edges


def _temp_color(t: float) -> str:
    return (_COL_OK if t < _TEMP_COOL else
            _COL_WARN if t < _TEMP_WARM else
            "#d05a1a" if t < _TEMP_HOT else _COL_CRIT)


# ---- per-joint readout cells: a split POS|GAP box + standalone temp / torque boxes,
# each FRAMED with a border whose colour tracks that value's state (gap alignment,
# temperature, torque-vs-limit). Widths are shared by the header + rows so they align.
_CW_NAME, _CW_BTN, _COL_SP = 30, 54, 10
_POS_W, _GAP_W, _TEMP_W, _TRQ_W = 48, 40, 40, 50   # inner label widths
_CELL_MONO = "font-family:Consolas,'DejaVu Sans Mono',monospace;font-size:11px;"
_CAP_QSS = "color:#7c848c;font-size:9px;font-weight:700;"
_CELL_DIM = "#5a6169"
_CELL_BG = "#15181c"
_BORDER_IDLE = "#3a3f45"


class _MonoCell(QFrame):
    """A single framed value (temperature or torque). Border + text colour track state;
    both are cached so we only re-style when the colour bucket actually changes."""

    def __init__(self, width: int, caption: str = "", header: bool = False, tip: str = ""):
        super().__init__()
        self.setObjectName("mcell")
        self._bc = self._fc = None
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)   # never compress/overlap
        h = QHBoxLayout(self)
        h.setContentsMargins(5, 1, 5, 1)
        h.setSpacing(0)
        self.lbl = QLabel(caption if header else "—")
        self.lbl.setFixedWidth(width)
        self.lbl.setAlignment(Qt.AlignCenter)
        if tip:
            self.setToolTip(tip)
        h.addWidget(self.lbl)
        if header:                      # transparent 1px border keeps the same footprint
            self.setStyleSheet("#mcell{border:1px solid transparent;background:transparent;}")
            self.lbl.setStyleSheet(_CAP_QSS)
        else:
            self.set_val("—", _BORDER_IDLE, _CELL_DIM)

    def set_val(self, text: str, border: str, fg: str):
        self.lbl.setText(text)
        if border != self._bc:
            self._bc = border
            self.setStyleSheet(f"#mcell{{border:1px solid {border};background:{_CELL_BG};}}")
        if fg != self._fc:
            self._fc = fg
            self.lbl.setStyleSheet(f"color:{fg};{_CELL_MONO}border:none;background:transparent;")


class _DualCell(QFrame):
    """POS | GAP in ONE split box with a divider. The whole box's border colour = the
    leader↔follower alignment (gap): green = aligned/safe to engage, red = far. The
    divider matches the border so the box reads as a single state-coloured unit."""

    def __init__(self, header: bool = False):
        super().__init__()
        self.setObjectName("dcell")
        self._bc = self._gfc = None
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)   # never compress/overlap
        h = QHBoxLayout(self)
        h.setContentsMargins(5, 1, 5, 1)
        h.setSpacing(0)
        self.pos = QLabel("POS" if header else "—")
        self.pos.setFixedWidth(_POS_W)
        self.pos.setAlignment(Qt.AlignCenter)
        self.gap = QLabel("GAP" if header else "—")
        self.gap.setFixedWidth(_GAP_W)
        self.gap.setAlignment(Qt.AlignCenter)
        h.addWidget(self.pos)
        h.addWidget(self.gap)
        if header:
            self.setStyleSheet("#dcell{border:1px solid transparent;background:transparent;}")
            self.pos.setStyleSheet(_CAP_QSS + "border:none;")
            self.gap.setStyleSheet(_CAP_QSS + "border-left:1px solid transparent;")
        else:
            self.setToolTip("POS = follower position (rad).  GAP = leader−follower gap (rad); "
                            "the box reddens as it grows — green = safe to engage.")
            self.pos.setStyleSheet(f"color:#e6e9ed;{_CELL_MONO}border:none;background:transparent;")
            self._paint(_BORDER_IDLE, _CELL_DIM)

    def _paint(self, border: str, gap_fg: str):
        if border == self._bc and gap_fg == self._gfc:
            return
        self._bc, self._gfc = border, gap_fg
        self.setStyleSheet(f"#dcell{{border:1px solid {border};background:{_CELL_BG};}}")
        self.gap.setStyleSheet(f"color:{gap_fg};{_CELL_MONO}background:transparent;"
                               f"border-left:1px solid {border};")

    def set(self, pos_txt: str, gap_txt: str, gap_color: str, have_gap: bool):
        self.pos.setText(pos_txt)
        self.gap.setText(gap_txt)
        self._paint(gap_color if have_gap else _BORDER_IDLE,
                    gap_color if have_gap else _CELL_DIM)


class _JointHeader(QWidget):
    """Column captions above a group of joint rows: LEADER↔FOLLOWER (the diff bar) · the
    POS|GAP box · °C · Nm. Built from the same cell widgets so the boxes line up."""

    def __init__(self):
        super().__init__()
        row = QHBoxLayout(self)
        row.setContentsMargins(2, 2, 2, 0)
        row.setSpacing(_COL_SP)
        namec = QLabel("")
        namec.setFixedWidth(_CW_NAME)
        row.addWidget(namec)
        barc = QLabel("LEADER ↔ FOLLOWER")
        barc.setStyleSheet(_CAP_QSS)
        barc.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        barc.setToolTip("Diff bar: white=follower, cyan=leader, band=gap. Drive green, then engage.")
        # Ignored width: this caption must be free to shrink/clip so it never forces the
        # header row wider than the joint rows (which would squeeze + overlap the cells).
        barc.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        barc.setMinimumWidth(0)
        row.addWidget(barc, 1)
        row.addWidget(_DualCell(header=True))
        row.addWidget(_MonoCell(_TEMP_W, "°C", header=True))
        row.addWidget(_MonoCell(_TRQ_W, "Nm", header=True))
        endc = QLabel("")
        endc.setFixedWidth(_CW_BTN)
        row.addWidget(endc)


class _JointRow(QWidget):
    """One joint as a table row: name · leader↔follower diff bar · POS|GAP box · temp box ·
    torque box · −/+ jog. Each box's border colour signals its state; only the numbers
    (and colour bucket) change per tick."""

    def __init__(self, idx: int, lo: float, hi: float, jog_cb):
        super().__init__()
        self.idx = idx
        self.sat = float(SAT_TORQUE[idx]) if idx < len(SAT_TORQUE) else 1.0
        row = QHBoxLayout(self)
        row.setContentsMargins(2, 1, 2, 1)
        row.setSpacing(_COL_SP)

        jname = MINERVA_JOINTS[idx] if idx < len(MINERVA_JOINTS) else str(idx)
        name = QLabel(_SHORT.get(idx, str(idx)))
        name.setFixedWidth(_CW_NAME)
        name.setStyleSheet("color:#cfd3d8;font-weight:600;")
        name.setToolTip(f"{jname}  (index {idx})")
        row.addWidget(name)

        self.bar = _DiffBar(lo, hi)
        self.bar.setToolTip(
            "Dim fill = follower position; cyan = leader; coloured band = leader−follower "
            "gap (green <0.05, yellow <0.15, red beyond). Drive it green, then engage.")
        row.addWidget(self.bar, 1)

        self.dual = _DualCell()
        self.temp_cell = _MonoCell(_TEMP_W, tip="Motor temperature (°C)")
        self.trq_cell = _MonoCell(_TRQ_W, tip="Motor torque (Nm), border reddens toward the limit")
        row.addWidget(self.dual)
        row.addWidget(self.temp_cell)
        row.addWidget(self.trq_cell)

        btns = QHBoxLayout()
        btns.setContentsMargins(0, 0, 0, 0)
        btns.setSpacing(4)
        for label, d in (("−", -1.0), ("+", 1.0)):
            b = QPushButton(label)
            b.setFixedSize(24, 20)
            b.setCursor(Qt.PointingHandCursor)
            b.setToolTip(f"Jog {jname} {'down' if d < 0 else 'up'}")
            b.setStyleSheet("QPushButton{background:#2b2f34;border:1px solid #3a3f45;"
                            "color:#cfd3d8;border-radius:0px;font-weight:700;}"
                            "QPushButton:hover{background:#3a3f45;}"
                            "QPushButton:pressed{background:#454b52;}")
            b.clicked.connect(lambda _=False, dd=d: jog_cb(self.idx, dd))
            btns.addWidget(b)
        bw = QWidget()
        bw.setFixedWidth(_CW_BTN)
        bw.setLayout(btns)
        row.addWidget(bw)

    def set(self, actual: Optional[float], leader: Optional[float], target: Optional[float],
            temp: Optional[float] = None, torque: Optional[float] = None):
        # `x != x` is True only for NaN — a dropped arm's joints arrive as NaN, and
        # head/lift have no leader; render those cleanly instead of crashing.
        self.bar.set_values(actual, leader, target)
        have_a = actual is not None and actual == actual
        pos_txt = f"{float(actual):+.2f}" if have_a else "—"
        if have_a and leader is not None and leader == leader:
            g = abs(float(leader) - float(actual))
            self.dual.set(pos_txt, f"{g:.2f}", _err_color(g), True)
        else:
            self.dual.set(pos_txt, "·", _CELL_DIM, False)   # head/lift/dropped: no leader gap
        if temp is None or temp != temp:
            self.temp_cell.set_val("—", _BORDER_IDLE, _CELL_DIM)
        else:
            c = _temp_color(float(temp))
            self.temp_cell.set_val(f"{float(temp):.0f}°", c, c)
        if torque is None or torque != torque:
            self.trq_cell.set_val("—", _BORDER_IDLE, _CELL_DIM)
        else:
            frac = min(1.0, abs(float(torque)) / max(self.sat, 1e-3))
            c = _COL_OK if frac < 0.5 else (_COL_WARN if frac < 0.8 else _COL_CRIT)
            self.trq_cell.set_val(f"{float(torque):+.2f}", c, c)


# ---------------------------------------------------------------------------
# Joint trace (per-episode qpos trajectories with a playback cursor)
# ---------------------------------------------------------------------------

class _JointTrace(QWidget):
    """Small time-series plot of all 17 joint trajectories across an episode,
    with a vertical cursor at the current frame. The trajectories are rendered
    ONCE to a background pixmap (on load / resize); per-frame repaint just
    redraws that pixmap + the cursor, so scrubbing/playback is cheap even for
    thousands of frames."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(90)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._qpos = None      # [T, J]
        self._T = 0
        self._t = 0
        self._range: List[tuple] = []
        self._bg: Optional[QPixmap] = None
        self._seglist: list = []

    def set_segments(self, segments) -> None:
        self._seglist = list(segments or [])
        self._render_bg()
        self.update()

    def set_data(self, qpos) -> None:
        self._qpos = qpos
        self._T = 0 if qpos is None else int(qpos.shape[0])
        self._t = 0
        if self._T:
            self._range = [(float(qpos[:, j].min()), float(qpos[:, j].max()))
                           for j in range(qpos.shape[1])]
        else:
            self._range = []
        self._render_bg()
        self.update()

    def set_frame(self, t: int) -> None:
        self._t = int(t)
        self.update()

    def resizeEvent(self, ev):
        self._render_bg()
        super().resizeEvent(ev)

    def _render_bg(self) -> None:
        w, h = self.width(), self.height()
        if self._qpos is None or self._T < 2 or w < 2 or h < 2:
            self._bg = None
            return
        pm = QPixmap(w, h)
        pm.fill(QColor("#0d0d0d"))
        p = QPainter(pm)
        # segment bands (drawn behind the trajectories)
        for i, s in enumerate(self._seglist):
            try:
                x0 = int(w * int(s["start"]) / max(self._T - 1, 1))
                x1 = int(w * min(int(s["end"]), self._T - 1) / max(self._T - 1, 1))
            except Exception:
                continue
            c = QColor(_BAND_COLORS[i % len(_BAND_COLORS)])
            c.setAlpha(90)
            p.fillRect(x0, 0, max(1, x1 - x0), h, c)
            p.setPen(QColor("#dddddd"))
            p.drawText(x0 + 2, 12, str(s.get("label", ""))[:16])
        T, J = self._T, self._qpos.shape[1]
        for j in range(J):
            lo, hi = self._range[j]
            span = max(hi - lo, 1e-6)
            p.setPen(QPen(QColor(_JOINT_COLOR.get(j, "#888")), 1))
            poly = QPolygonF()
            for t in range(T):
                x = w * t / (T - 1)
                y = h - 2 - (float(self._qpos[t, j]) - lo) / span * (h - 4)
                poly.append(QPointF(x, y))
            p.drawPolyline(poly)
        p.end()
        self._bg = pm

    def paintEvent(self, ev):
        p = QPainter(self)
        if self._bg is not None:
            p.drawPixmap(0, 0, self._bg)
        else:
            p.fillRect(self.rect(), QColor("#0d0d0d"))
        if self._T > 1:
            cx = int(self.width() * self._t / (self._T - 1))
            p.setPen(QPen(QColor("#ffffff"), 1))
            p.drawLine(cx, 0, cx, self.height())
        p.end()


# ---------------------------------------------------------------------------
# Episode viewer (recorded-episode browser + scrub playback)
# ---------------------------------------------------------------------------

class _EpisodeViewer(QWidget):
    """Browse recorded v6 episodes and scrub/play their 3-camera frames — the
    Minerva analog of collect_demo.py --gui's replay panel. Reads frames lazily
    from the open HDF5 on the GUI thread (one small frame per camera per tick)."""

    def __init__(self, output_dir, cameras):
        super().__init__()
        self.output_dir = Path(output_dir)
        self.cameras = list(cameras)
        self._h5 = None
        self._path: Optional[Path] = None
        self._T = 0
        self._t = 0
        self._qpos_all = None
        self._meta_cache: Dict[str, tuple] = {}
        self._known_saved: set = set()
        self._edit_segments: list = []
        self._mark_in_f: Optional[int] = None
        self._mark_out_f: Optional[int] = None

        col = QVBoxLayout(self)
        col.setContentsMargins(4, 4, 4, 4)

        hdr = QHBoxLayout()
        title = QLabel("Recorded episodes")
        title.setStyleSheet("color:#fc9;font-weight:bold;")
        hdr.addWidget(title)
        hdr.addStretch(1)
        self.del_btn = QPushButton("🗑 Delete")
        self.del_btn.setToolTip("Delete the selected episode (asks to confirm)")
        self.del_btn.clicked.connect(self._delete_current)
        hdr.addWidget(self.del_btn)
        refresh = QPushButton("⟳")
        refresh.setFixedWidth(30)
        refresh.setToolTip("Rescan the output folder for episodes")
        refresh.clicked.connect(self.refresh)
        hdr.addWidget(refresh)
        col.addLayout(hdr)

        self.list = QListWidget()
        self.list.setMaximumHeight(150)
        self.list.setToolTip("Recorded episodes — filename, frame count, instruction. "
                             "Select one to play it back.")
        self.list.currentItemChanged.connect(self._on_select)
        col.addWidget(self.list)

        pb = QHBoxLayout()
        self.pb_panels: Dict[str, _CameraPanel] = {}
        for c in _cam_display_order(self.cameras):
            p = _CameraPanel(c)
            p.setMinimumSize(110, 84)
            self.pb_panels[c] = p
            pb.addWidget(p)
        col.addLayout(pb, 1)

        tr = QHBoxLayout()
        self.play_btn = QPushButton("▶")
        self.play_btn.setFixedWidth(38)
        self.play_btn.setToolTip("Play / pause the selected episode")
        self.play_btn.clicked.connect(self._toggle_play)
        tr.addWidget(self.play_btn)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setEnabled(False)
        self.slider.setToolTip("Scrub through the episode's frames")
        self.slider.valueChanged.connect(self._render)
        tr.addWidget(self.slider, 1)
        self.frame_lbl = QLabel("–/–")
        self.frame_lbl.setFixedWidth(72)
        self.frame_lbl.setStyleSheet("color:#9cf;font-family:monospace;")
        tr.addWidget(self.frame_lbl)
        col.addLayout(tr)

        # joint trace (all 17 trajectories + playback cursor) + numeric readout
        self.trace = _JointTrace()
        self.trace.setToolTip("Joint trajectories across the episode; white line = current frame")
        col.addWidget(self.trace)
        self.qpos_lbl = QLabel("")
        self.qpos_lbl.setToolTip("Joint values at the current frame (radians; lift in metres)")
        self.qpos_lbl.setStyleSheet("color:#bbb;font-family:monospace;font-size:11px;")
        col.addWidget(self.qpos_lbl)

        self.info = QLabel("select an episode to play")
        self.info.setWordWrap(True)
        self.info.setStyleSheet("color:#8c8;")
        col.addWidget(self.info)

        # -- segment editor (post-hoc annotation) --
        seg_hdr = QHBoxLayout()
        stitle = QLabel("Action segments")
        stitle.setStyleSheet("color:#fc9;font-weight:bold;")
        seg_hdr.addWidget(stitle)
        seg_hdr.addStretch(1)
        self.seg_auto = QPushButton("Auto")
        self.seg_auto.setToolTip("Propose segment boundaries from gripper open/close + "
                                 "motion pauses; then label each span and Save")
        self.seg_auto.clicked.connect(self._auto_segment)
        seg_hdr.addWidget(self.seg_auto)
        self.seg_save = QPushButton("Save")
        self.seg_save.setToolTip("Write these segments back into the episode file (v7)")
        self.seg_save.clicked.connect(self._save_segments)
        seg_hdr.addWidget(self.seg_save)
        col.addLayout(seg_hdr)

        self.seg_list = QListWidget()
        self.seg_list.setMaximumHeight(84)
        self.seg_list.setToolTip("Action segments in this episode — [start–end] label. "
                                 "Select one to load its label for editing.")
        self.seg_list.currentRowChanged.connect(self._on_seg_selected)
        col.addWidget(self.seg_list)

        erow = QHBoxLayout()
        self.mark_in_btn = QPushButton("In")
        self.mark_in_btn.setToolTip("Set segment start = current frame")
        self.mark_in_btn.clicked.connect(self._mark_in)
        self.mark_out_btn = QPushButton("Out")
        self.mark_out_btn.setToolTip("Set segment end = current frame")
        self.mark_out_btn.clicked.connect(self._mark_out)
        self.seg_combo = QComboBox()
        self.seg_combo.setEditable(True)
        self.seg_combo.addItems([""] + _ACTION_VOCAB)
        self.seg_combo.setToolTip("Label for the new segment")
        self.seg_add_btn = QPushButton("Add")
        self.seg_add_btn.setToolTip("Add a segment [In..Out] with the label")
        self.seg_add_btn.clicked.connect(self._add_segment)
        self.seg_set_btn = QPushButton("Set")
        self.seg_set_btn.setToolTip("Apply the label to the selected segment (for auto-cut spans)")
        self.seg_set_btn.clicked.connect(self._relabel_selected)
        self.seg_del_btn = QPushButton("Del")
        self.seg_del_btn.setToolTip("Delete the selected segment")
        self.seg_del_btn.clicked.connect(self._del_segment)
        erow.addWidget(self.mark_in_btn)
        erow.addWidget(self.mark_out_btn)
        erow.addWidget(self.seg_combo, 1)
        erow.addWidget(self.seg_add_btn)
        erow.addWidget(self.seg_set_btn)
        erow.addWidget(self.seg_del_btn)
        col.addLayout(erow)

        self.mark_lbl = QLabel("in — / out —")
        self.mark_lbl.setStyleSheet("color:#9cf;font-family:monospace;font-size:11px;")
        col.addWidget(self.mark_lbl)

        self._timer = QTimer()
        self._timer.timeout.connect(self._advance)
        self.refresh()

    # -- list --
    def refresh(self):
        self.list.blockSignals(True)
        self.list.clear()
        eps = sorted(self.output_dir.glob("episode_*.hdf5")) if self.output_dir.exists() else []
        for p in eps:
            key = str(p)
            meta = self._meta_cache.get(key)
            if meta is None:
                try:
                    with h5py.File(p, "r") as f:
                        T = int(f["observations/qpos"].shape[0])
                        instr = str(f.attrs.get("language_instruction", ""))
                    meta = (T, instr)
                except Exception:
                    meta = (0, "")
                self._meta_cache[key] = meta
            T, instr = meta
            it = QListWidgetItem(f"{p.name}  ({T})  {instr}"[:56])
            it.setData(Qt.UserRole, key)
            self.list.addItem(it)
        self.list.blockSignals(False)

    def on_saved(self, last_saved: Optional[str]):
        """Called from apply_snapshot; refresh the list when a new episode lands."""
        if last_saved and last_saved not in self._known_saved:
            self._known_saved.add(last_saved)
            self._meta_cache.pop(last_saved, None)
            self.refresh()

    # -- load / render --
    def _on_select(self, cur, _prev=None):
        if cur is None:
            return
        self._load(Path(cur.data(Qt.UserRole)))

    def _load(self, path: Path):
        self._stop()
        self._close_h5()
        try:
            self._h5 = h5py.File(path, "r")
            self._T = int(self._h5["observations/qpos"].shape[0])
            self._path = path
            self._qpos_all = self._h5["observations/qpos"][:]   # [T,17], small
            self.trace.set_data(self._qpos_all)
            self._edit_segments = _parse_seg_attr(self._h5.attrs.get("segments"))
            self._mark_in_f = None
            self._mark_out_f = None
            self._refresh_segments()
            self._update_mark_lbl()
            instr = str(self._h5.attrs.get("language_instruction", ""))
            self.info.setText(f"{path.name} — {self._T} frames\n{instr or '(no instruction)'}")
            self.slider.setEnabled(self._T > 1)
            self.slider.setRange(0, max(0, self._T - 1))
            self.slider.setValue(0)
            self._render(0)
        except Exception as e:   # noqa: BLE001
            self.info.setText(f"load error: {e}")
            self._qpos_all = None
            self.trace.set_data(None)
            self.qpos_lbl.setText("")
            self._close_h5()

    def _avail_cams(self):
        if self._h5 is None:
            return []
        return [c for c in self.cameras if f"observations/images/{c}" in self._h5]

    def _render(self, t: int):
        if self._h5 is None or self._T == 0:
            return
        t = max(0, min(self._T - 1, int(t)))
        self._t = t
        for c in self._avail_cams():
            try:
                self.pb_panels[c].set_rgb(self._h5[f"observations/images/{c}"][t])
            except Exception:
                pass
        self.frame_lbl.setText(f"{t + 1}/{self._T}")
        self.trace.set_frame(t)
        if self._qpos_all is not None and t < len(self._qpos_all):
            self.qpos_lbl.setText(_fmt_qpos(self._qpos_all[t]))

    # -- transport --
    def _toggle_play(self):
        if self._timer.isActive():
            self._stop()
        elif self._h5 is not None and self._T > 1:
            self.play_btn.setText("⏸")
            self._timer.start(50)   # ~20 Hz playback

    def _stop(self):
        self._timer.stop()
        self.play_btn.setText("▶")

    def _advance(self):
        nt = self._t + 1
        if nt >= self._T:
            self._stop()
            return
        self.slider.setValue(nt)   # -> valueChanged -> _render

    # -- delete --
    def _delete_current(self):
        item = self.list.currentItem()
        if item is None:
            return
        path = Path(item.data(Qt.UserRole))
        resp = QMessageBox.question(
            self, "Delete episode",
            f"Delete {path.name}?\nThis permanently removes the file and cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if resp == QMessageBox.Yes:
            self._delete_path(path)

    def _delete_path(self, path):
        """Delete an episode file. The confirmation lives in _delete_current;
        this is the no-modal core (also the unit-test entry point)."""
        path = Path(path)
        if self._path is not None and Path(self._path) == path:
            self._stop()
            self._close_h5()
            self._path = None
            self._T = 0
            self._qpos_all = None
            self.trace.set_data(None)
            self.trace.set_segments([])
            self._edit_segments = []
            self.seg_list.clear()
            self._mark_in_f = None
            self._mark_out_f = None
            self._update_mark_lbl()
            self.qpos_lbl.setText("")
            self.info.setText("select an episode to play")
            self.slider.setEnabled(False)
            self.slider.setRange(0, 0)
            self.frame_lbl.setText("–/–")
            for c in self.cameras:
                self.pb_panels[c].clear()
                self.pb_panels[c].setText(f"{c}\n(no signal)")
        try:
            os.remove(path)
        except OSError as e:
            self.info.setText(f"delete failed: {e}")
            return
        self._meta_cache.pop(str(path), None)
        self.refresh()

    # -- segment editing (post-hoc annotation) --
    def _mark_in(self):
        if self._T:
            self._mark_in_f = self._t
            self._update_mark_lbl()

    def _mark_out(self):
        if self._T:
            self._mark_out_f = self._t
            self._update_mark_lbl()

    def _update_mark_lbl(self):
        i = "—" if self._mark_in_f is None else str(self._mark_in_f)
        o = "—" if self._mark_out_f is None else str(self._mark_out_f)
        self.mark_lbl.setText(f"in {i} / out {o}")

    def _add_segment(self):
        if self._mark_in_f is None or self._mark_out_f is None or self._T == 0:
            return
        a, b = sorted((self._mark_in_f, self._mark_out_f))
        end = min(b + 1, self._T)   # inclusive Out frame -> half-open [a, end)
        label = self.seg_combo.currentText().strip()
        if not label or end <= a:
            return
        self._edit_segments.append({"start": int(a), "end": int(end), "label": label})
        self._edit_segments.sort(key=lambda s: s["start"])
        self._refresh_segments()

    def _del_segment(self):
        row = self.seg_list.currentRow()
        if 0 <= row < len(self._edit_segments):
            del self._edit_segments[row]
            self._refresh_segments()

    def _refresh_segments(self):
        self.seg_list.clear()
        for s in self._edit_segments:
            self.seg_list.addItem(f"[{s['start']}–{s['end']}] {s['label'] or '?'}")
        self.trace.set_segments(self._edit_segments)

    def _auto_segment(self):
        if self._qpos_all is None or self._T < 3:
            return
        from collect_minerva_app.autosegment import (
            auto_segment_boundaries, segments_from_boundaries)
        bnds = auto_segment_boundaries(self._qpos_all)
        self._edit_segments = segments_from_boundaries(bnds, self._T, label="")
        self._refresh_segments()
        self.info.setText(f"auto-segmented into {len(self._edit_segments)} span(s) — "
                          "select each, label it, then Save")

    def _on_seg_selected(self, row: int):
        if 0 <= row < len(self._edit_segments):
            self.seg_combo.setCurrentText(str(self._edit_segments[row]["label"]))

    def _relabel_selected(self):
        row = self.seg_list.currentRow()
        lbl = self.seg_combo.currentText().strip()
        if 0 <= row < len(self._edit_segments) and lbl:
            self._edit_segments[row]["label"] = lbl
            self._refresh_segments()
            self.seg_list.setCurrentRow(row)

    def _save_segments(self):
        if self._path is None:
            return
        path = Path(self._path)
        self._close_h5()                      # release the read handle to write attrs
        try:
            with h5py.File(path, "a") as f:
                if self._edit_segments:
                    f.attrs["segments"] = json.dumps(self._edit_segments)
                    f.attrs["format_version"] = 7
                elif "segments" in f.attrs:
                    del f.attrs["segments"]
                    f.attrs["format_version"] = 6
            self.info.setText(f"{path.name} — saved {len(self._edit_segments)} segment(s)")
            self._meta_cache.pop(str(path), None)
        except Exception as e:   # noqa: BLE001
            self.info.setText(f"save failed: {e}")
        try:
            self._h5 = h5py.File(path, "r")   # reopen for continued playback
        except Exception:
            self._h5 = None

    # -- lifecycle --
    def _close_h5(self):
        if self._h5 is not None:
            try:
                self._h5.close()
            except Exception:
                pass
            self._h5 = None

    def close_viewer(self):
        self._stop()
        self._close_h5()


# ---------------------------------------------------------------------------
# Voice worker (captures + transcribes off the GUI thread)
# ---------------------------------------------------------------------------

class _VoiceWorker(QThread):
    done = Signal(str)
    failed = Signal(str)

    def __init__(self, stt, seconds: float = 5.0):
        super().__init__()
        self._stt = stt
        self._seconds = seconds

    def run(self):
        try:
            self.done.emit(self._stt.transcribe_once(self._seconds) or "")
        except Exception as e:   # noqa: BLE001
            self.failed.emit(str(e))


# ---------------------------------------------------------------------------
# Settings dialog
# ---------------------------------------------------------------------------

class _SettingsDialog(QDialog):
    """Tabbed per-user collector settings: teleop defaults, feedback, per-joint arm
    tuning, and voice. Reads the live params dict so it reflects the current state;
    values() is persisted to CollectorSettings and applied live by the main window."""

    _MODELS = ["tiny.en", "base.en", "small.en", "medium.en"]
    _JN = ["j1", "j2", "j3", "j4", "j5", "j6"]

    def __init__(self, settings: CollectorSettings, params: Optional[dict] = None, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.p = params if params is not None else {}
        self.setWindowTitle("Collector settings")
        self.setMinimumWidth(500)
        root = QVBoxLayout(self)
        tabs = QTabWidget()
        tabs.setStyleSheet(_TAB_QSS)
        tabs.addTab(self._teleop_tab(), "Teleop")
        tabs.addTab(self._feedback_tab(), "Feedback")
        tabs.addTab(self._tuning_tab(), "Arm tuning")
        tabs.addTab(self._voice_tab(), "Voice")
        root.addWidget(tabs)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        root.addWidget(btns)

    # -- helpers --
    def _pv(self, key, fallback):
        v = self.p.get(key)
        return fallback if v is None else v

    @staticmethod
    def _dspin(lo, hi, step, val, dec=2, suffix=""):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(dec)
        s.setValue(float(val))
        if suffix:
            s.setSuffix(suffix)
        return s

    # -- tabs --
    def _teleop_tab(self):
        w = QWidget()
        f = QFormLayout(w)
        self.speed_spin = self._dspin(0.10, 1.00, 0.05, self._pv("kp_scale", 0.3), 2, " x")
        self.speed_spin.setToolTip("Default arm tracking-gain scale (the Speed slider)")
        f.addRow("Speed (kp scale):", self.speed_spin)
        self.grip_spin = self._dspin(0.5, 6.0, 0.1, self._pv("grip_strength", 1.5), 1, " x")
        self.grip_spin.setToolTip("Default gripper KP multiplier (decoupled from arm speed)")
        f.addRow("Grip strength:", self.grip_spin)
        return w

    def _feedback_tab(self):
        w = QWidget()
        f = QFormLayout(w)
        grav_ok = bool(self.p.get("grav_ok", False))
        self.grav_chk = QCheckBox("Gravity feedforward on at start")
        self.grav_chk.setChecked(bool(self._pv("grav_comp", False)) and grav_ok)
        self.grav_chk.setEnabled(grav_ok)
        self.grav_chk.setToolTip("Start with arm gravity compensation enabled" if grav_ok
                                 else "No gravity model loaded — run minerva_gravity_calibrate.py")
        f.addRow(self.grav_chk)
        self.grav_scale_spin = self._dspin(0.0, 1.2, 0.05, self._pv("grav_scale", 1.0), 2, " x")
        self.grav_scale_spin.setToolTip("Global gravity feedforward trim")
        f.addRow("Gravity scale:", self.grav_scale_spin)
        self.ff_chk = QCheckBox("Gripper force-feedback on at start")
        self.ff_chk.setChecked(bool(self._pv("grip_ff", False)))
        f.addRow(self.ff_chk)
        self.ff_gain_spin = self._dspin(0, 600, 10, self._pv("grip_ff_gain", 200.0), 0, " mA/Nm")
        self.ff_gain_spin.setToolTip("Leader mA per Nm of follower grasp torque")
        f.addRow("Gripper FF gain:", self.ff_gain_spin)
        self.ff_invert_chk = QCheckBox("Invert gripper FF polarity")
        self.ff_invert_chk.setChecked(int(self._pv("grip_ff_sign", -1)) >= 0)
        f.addRow(self.ff_invert_chk)
        return w

    def _tuning_tab(self):
        w = QWidget()
        v = QVBoxLayout(w)
        note = QLabel("Per-joint PD gains (kp, kd) and torque headroom (SAT, which caps "
                      "tracking speed as SAT/kp). Applies to BOTH arms. Big changes are "
                      "best made while idle.")
        note.setWordWrap(True)
        note.setStyleSheet("color:#9aa0a6; font-size:11px;")
        v.addWidget(note)
        grid = QGridLayout()
        for c, h in enumerate(("joint", "kp", "kd", "SAT (Nm)")):
            lab = QLabel(h)
            lab.setStyleSheet("color:#868d95; font-weight:700;")
            grid.addWidget(lab, 0, c)
        kp0 = self._pv("arm_kp", [float(x) for x in KP[0:6]])
        kd0 = self._pv("arm_kd", [float(x) for x in KD[0:6]])
        sat0 = self._pv("arm_sat", [float(x) for x in SAT_TORQUE[0:6]])
        self.kp_spins, self.kd_spins, self.sat_spins = [], [], []
        for r, jn in enumerate(self._JN, start=1):
            grid.addWidget(QLabel(jn), r, 0)
            skp = self._dspin(0, 400, 5, kp0[r - 1], 0)
            skd = self._dspin(0, 60, 1, kd0[r - 1], 0)
            ssat = self._dspin(0.1, 40, 0.5, sat0[r - 1], 1)
            self.kp_spins.append(skp)
            self.kd_spins.append(skd)
            self.sat_spins.append(ssat)
            grid.addWidget(skp, r, 1)
            grid.addWidget(skd, r, 2)
            grid.addWidget(ssat, r, 3)
        v.addLayout(grid)
        reset = QPushButton("Reset to firmware defaults")
        reset.setToolTip("Restore kp/kd/SAT to the values baked into minerva_constants.py")
        reset.clicked.connect(self._reset_tuning)
        v.addWidget(reset)
        v.addStretch(1)
        return w

    def _reset_tuning(self):
        for i in range(6):
            self.kp_spins[i].setValue(float(KP[i]))
            self.kd_spins[i].setValue(float(KD[i]))
            self.sat_spins[i].setValue(float(SAT_TORQUE[i]))

    def _voice_tab(self):
        w = QWidget()
        f = QFormLayout(w)
        self.mic_combo = QComboBox()
        self.mic_combo.addItem("System default", None)
        for idx, name in list_input_devices():
            self.mic_combo.addItem(f"[{idx}] {name}", idx)
        pos = self.mic_combo.findData(self.settings.get("mic_device"))
        self.mic_combo.setCurrentIndex(pos if pos >= 0 else 0)
        self.mic_combo.setToolTip("Audio input device used for voice action-labeling")
        f.addRow("Microphone:", self.mic_combo)
        self.model_combo = QComboBox()
        self.model_combo.setEditable(True)
        self.model_combo.addItems(self._MODELS)
        self.model_combo.setCurrentText(str(self.settings.get("voice_model")))
        self.model_combo.setToolTip("Local whisper model — tiny=fastest, larger=more accurate")
        f.addRow("Voice model:", self.model_combo)
        self.secs_spin = self._dspin(1.0, 15.0, 0.5, float(self.settings.get("voice_seconds")), 1, " s")
        self.secs_spin.setToolTip("How long the mic listens per voice capture")
        f.addRow("Record length:", self.secs_spin)
        return w

    def values(self) -> dict:
        return {
            "mic_device": self.mic_combo.currentData(),
            "voice_model": self.model_combo.currentText().strip() or "base.en",
            "voice_seconds": float(self.secs_spin.value()),
            "kp_scale": float(self.speed_spin.value()),
            "grip_strength": float(self.grip_spin.value()),
            "grav_comp": bool(self.grav_chk.isChecked()),
            "grav_scale": float(self.grav_scale_spin.value()),
            "grip_ff": bool(self.ff_chk.isChecked()),
            "grip_ff_gain": float(self.ff_gain_spin.value()),
            "grip_ff_invert": bool(self.ff_invert_chk.isChecked()),
            "arm_kp": [float(s.value()) for s in self.kp_spins],
            "arm_kd": [float(s.value()) for s in self.kd_spins],
            "arm_sat": [float(s.value()) for s in self.sat_spins],
        }


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MinervaMainWindow(QMainWindow):
    def __init__(self, cmd_queue, meta, teleop, cameras, output_dir, label_queue=None,
                 params=None):
        super().__init__()
        self.cmd_queue = cmd_queue
        self.meta = meta
        self.teleop = teleop
        self.cameras = list(cameras)
        self.output_dir = output_dir
        self.label_queue = label_queue if label_queue is not None else queue.Queue()
        self.params = params if params is not None else {}
        self.settings = CollectorSettings()
        self._voice_seconds = float(self.settings.get("voice_seconds"))
        self.setWindowTitle("Minerva Demo Collector")
        self.resize(1280, 820)
        # Flat, square-cornered dark theme — no rounded corners anywhere.
        self.setStyleSheet("""
            QWidget { color: #dddddd; }
            QToolTip { color: #eee; background: #222; border: 1px solid #555; padding: 3px; }
            QPushButton { border: 1px solid #555; border-radius: 0px; background: #2b2b2b; padding: 4px 6px; }
            QPushButton:hover { background: #3a3a3a; }
            QPushButton:pressed { background: #454545; }
            QProgressBar { border: 1px solid #444; border-radius: 0px; background: #161616; }
            QProgressBar::chunk { background: #4a90d9; }
            QLineEdit { border: 1px solid #555; border-radius: 0px; background: #1e1e1e; padding: 3px; }
            QListWidget { border: 1px solid #444; border-radius: 0px; background: #141414; }
            QFrame { border-radius: 0px; }
            QSlider::groove:horizontal { height: 4px; background: #333; }
            QSlider::handle:horizontal { width: 12px; background: #4a90d9; border-radius: 0px; margin: -6px 0; }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # -- top status dashboard (row 1: robot · row 2: Jetson host) --
        self.stat = {}
        _row1 = [
            ("state",  "STATE",     116, "Robot state — DISABLED / IDLE / HOLD / ENGAGING / TELEOP / SHUTDOWN"),
            ("estop",  "E-STOP",    84,  "Emergency stop reported by the follower"),
            ("batt",   "MOTOR BAT", 92,  "Motor-pack voltage (lowest of the two arms)"),
            ("arms",   "ARMS",      94,  "Follower telemetry — both arms present?"),
            ("temp",   "MAX TEMP",  88,  "Hottest motor across both arms"),
            ("left",   "L LEADER",  78,  "Left OpenRB-150 leader connection"),
            ("right",  "R LEADER",  78,  "Right OpenRB-150 leader connection"),
            ("cams",   "CAMERAS",   78,  "Camera streams receiving fresh frames (< 0.5 s)"),
            ("telem",  "LINK",      78,  "Follower telemetry freshness (age)"),
        ]
        _row2 = [
            ("jetson", "JETSON",    132, "Resolved Jetson address + link status"),
            ("ups",    "UPS BAT",   104, "Logic-UPS battery (percentage / voltage)"),
            ("cpu",    "CPU",       72,  "Jetson CPU utilisation"),
            ("mem",    "MEM",       72,  "Jetson memory used"),
            ("disk",   "DISK",      72,  "Jetson root filesystem used"),
            ("wifi",   "WIFI",      124, "Jetson WiFi connection"),
        ]
        top = QHBoxLayout()
        top.setSpacing(8)
        tiles = QVBoxLayout()
        tiles.setSpacing(6)
        for row_defs in (_row1, _row2):
            bar = QHBoxLayout()
            bar.setSpacing(6)
            for key, cap, wdt, tip in row_defs:
                sb = _StatBox(cap, wdt)
                sb.setToolTip(tip)
                self.stat[key] = sb
                bar.addWidget(sb)
            bar.addStretch(1)
            tiles.addLayout(bar)
        top.addLayout(tiles, 1)

        # -- top-right corner: Settings + Fullscreen --
        corner = QVBoxLayout()
        corner.setSpacing(6)
        self.settings_btn = QPushButton("⚙  Settings")
        self.settings_btn.setToolTip("Microphone, voice model, and record length")
        self.settings_btn.clicked.connect(self._open_settings)
        self.settings_btn.setFixedWidth(132)
        _style_btn(self.settings_btn, "neutral", 34)
        corner.addWidget(self.settings_btn)
        self.fs_btn = QPushButton("⛶  Fullscreen")
        self.fs_btn.setToolTip("Toggle fullscreen")
        self.fs_btn.clicked.connect(self._toggle_fullscreen)
        self.fs_btn.setFixedWidth(132)
        _style_btn(self.fs_btn, "neutral", 34)
        corner.addWidget(self.fs_btn)
        top.addLayout(corner)
        root.addLayout(top)

        # -- body: LEFT [live cameras + joint bars]  |  RIGHT [controls + episode viewer] --
        # A draggable splitter lets the operator trade width between the viewing area and
        # the controls column (the divider persists for the session).
        body = QSplitter(Qt.Horizontal)
        body.setHandleWidth(6)
        body.setChildrenCollapsible(False)
        body.setStyleSheet(
            "QSplitter::handle{background:#2b2f34;border-left:1px solid #22262b;"
            "border-right:1px solid #22262b;}"
            "QSplitter::handle:hover{background:#3a90d9;}"
            "QSplitter::handle:pressed{background:#4aa3ec;}")

        left = QVBoxLayout()
        left.setSpacing(6)
        cam_row = QHBoxLayout()
        cam_row.setContentsMargins(0, 0, 0, 0)
        cam_row.setSpacing(6)                     # tight gutter between the three monitors
        cam_eps = (self.meta or {}).get("cam_endpoints", {})
        cam_sizes = (self.meta or {}).get("cam_sizes", {})
        self.cam_tiles: Dict[str, _CameraTile] = {}
        for c in _cam_display_order(self.cameras):
            tile = _CameraTile(c, cam_eps.get(c), cam_sizes.get(c))
            self.cam_tiles[c] = tile
            cam_row.addWidget(tile, 1)
        cam_host = QWidget()
        cam_host.setLayout(cam_row)
        left.addWidget(cam_host, 2)
        left.addWidget(self._build_joints(), 3)
        left.setContentsMargins(0, 0, 0, 0)
        left_host = QWidget()
        left_host.setLayout(left)
        # Floor above the two joint columns' content width (2×~412 + margins) so the
        # splitter can never squeeze the fixed cells into overlap when not fullscreen.
        left_host.setMinimumWidth(880)

        right = QVBoxLayout()
        right.setSpacing(6)
        # Controls and the recorded-episode viewer are TABS, so each gets the full
        # column height — no scroll box, nothing squeezed.
        self.viewer = _EpisodeViewer(self.output_dir, self.cameras)
        self.right_tabs = QTabWidget()
        self.right_tabs.setStyleSheet(_TAB_QSS)
        self.right_tabs.addTab(self._build_controls(), "Controls")
        self.right_tabs.addTab(self.viewer, "Recorded")
        right.addWidget(self.right_tabs, 1)
        right.setContentsMargins(0, 0, 0, 0)
        right_host = QWidget()
        right_host.setLayout(right)
        right_host.setMinimumWidth(430)

        body.addWidget(left_host)
        body.addWidget(right_host)
        body.setStretchFactor(0, 3)          # left grows faster on window resize
        body.setStretchFactor(1, 2)
        body.setSizes([780, 500])            # initial ~3:2 split (draggable thereafter)
        root.addWidget(body, 1)

        # -- footer --
        footer = QHBoxLayout()
        self.legend = QLabel("keys: I idle(first) · E enable · T teleop · R record · H disable · Q quit   |   "
                             "bar: white=arm cyan=leader, gap green<0.05 yellow<0.15 red=far — all green ⇒ safe to engage")
        self.legend.setToolTip("Keyboard shortcuts fire when no text field is focused")
        self.legend.setStyleSheet("color:#888;")
        footer.addWidget(self.legend)
        footer.addStretch(1)
        self.saved_lbl = QLabel("last saved: —")
        self.saved_lbl.setStyleSheet("color:#8c8;")
        footer.addWidget(self.saved_lbl)
        root.addLayout(footer)

        self._setup_voice()
        self._setup_shortcuts()

    # -- builders --
    def _build_controls(self) -> QWidget:
        w = QFrame()
        col = QVBoxLayout(w)
        col.setContentsMargins(10, 10, 10, 10)
        col.setSpacing(7)

        self.record_btn = QPushButton("● RECORD")
        self.record_btn.setCheckable(False)
        self.record_btn.setFixedHeight(58)
        self.record_btn.setCursor(Qt.PointingHandCursor)
        self.record_btn.setStyleSheet(self._record_qss(False))
        self.record_btn.setToolTip("Start / stop recording (R) — only while teleop is engaged")
        self.record_btn.clicked.connect(lambda: self._key("R"))
        col.addWidget(self.record_btn)

        self.rec_status = QLabel("not recording")
        self.rec_status.setAlignment(Qt.AlignCenter)
        self.rec_status.setStyleSheet("color:#ccc;")
        col.addWidget(self.rec_status)

        col.addWidget(QLabel("Task / language instruction:"))
        self.instr = QLineEdit(self.meta.get("language_instruction", ""))
        self.instr.setPlaceholderText("e.g. pick up the red block with the left arm")
        self.instr.setToolTip("Task / language string saved into every recorded episode (v6 attr)")
        self.instr.textChanged.connect(self._on_instr)
        col.addWidget(self.instr)

        # -- current action label (live per-phase segment marking) --
        col.addWidget(QLabel("Current action (segment label):"))
        arow = QHBoxLayout()
        self.action_combo = QComboBox()
        self.action_combo.setEditable(True)
        self.action_combo.addItems([""] + _ACTION_VOCAB)
        self.action_combo.setCurrentText("")
        self.action_combo.setToolTip(
            "Pick or type the action for the current phase; 'Set phase' starts a new "
            "labeled segment from the current frame while recording.")
        arow.addWidget(self.action_combo, 1)
        self.phase_btn = QPushButton("Set phase")
        self.phase_btn.setToolTip("Mark a new action segment starting at the current frame")
        self.phase_btn.clicked.connect(self._set_phase)
        _style_btn(self.phase_btn, "neutral", 32, 12)
        arow.addWidget(self.phase_btn)
        self.voice_btn = QPushButton("🎤")
        self.voice_btn.setFixedWidth(40)
        self.voice_btn.setToolTip("Speak the action label (voice-to-text)")
        _style_btn(self.voice_btn, "neutral", 32)
        arow.addWidget(self.voice_btn)
        col.addLayout(arow)

        self.seg_lbl = QLabel("")
        self.seg_lbl.setStyleSheet("color:#9c9;")
        col.addWidget(self.seg_lbl)

        # Primary state machine — color-coded by role.
        btns = QHBoxLayout(); btns.setSpacing(6)
        _tips = {"I": "Idle — enable at ZERO torque; read positions + zero. ALWAYS FIRST (I)",
                 "E": "Enable gains (HOLD) — only from Idle, after zeroing (E)",
                 "T": "Engage / disengage leader teleop (T)",
                 "H": "Disable the follower motors (H)"}
        for label, key, role in (("Idle", "I", "neutral"), ("Enable", "E", "info"),
                                 ("Teleop", "T", "go"), ("Disable", "H", "warn")):
            b = QPushButton(label)
            b.setToolTip(_tips[key])
            b.clicked.connect(lambda _=False, k=key: self._key(k))
            _style_btn(b, role, 44, 14)
            btns.addWidget(b)
        col.addLayout(btns)

        # Zeroing + soft shutdown (Ready removed — was unused).
        btns2 = QHBoxLayout(); btns2.setSpacing(6)
        _tips2 = {"Z": "Leader zero — capture each leader's current pose as its zero (Z)",
                  "M": "Mirror — set leader zero so it maps to the arm's actual pose (M)",
                  "K": "RobStride mechanical zero + SaveConfig to both arms; disable first (K)",
                  "X": "Soft shutdown — ramp both arms to zero, then disable (X)"}
        for label, key, role in (("Zero", "Z", "neutral"), ("Mirror", "M", "neutral"),
                                 ("MechZero", "K", "neutral"), ("Shutdown", "X", "danger")):
            b = QPushButton(label)
            b.setToolTip(_tips2[key])
            b.clicked.connect(lambda _=False, k=key: self._key(k))
            _style_btn(b, role, 38)
            btns2.addWidget(b)
        col.addLayout(btns2)

        # Leader↔arm routing (persists across sessions; disabled while teleop is live).
        self.swap_btn = QPushButton()
        self.swap_btn.setToolTip("Swap which leader drives which arm. Saved for future "
                                 "sessions. Disengage teleop (T) first — re-routing while "
                                 "engaged would make an arm lunge toward the other leader.")
        self.swap_btn.clicked.connect(self._toggle_swap)
        _style_btn(self.swap_btn, "neutral", 34)
        self._update_swap_btn()
        col.addWidget(self.swap_btn)

        # Live speed (tracking-gain scale) — writes params["kp_scale"], read by the loop.
        srow = QHBoxLayout()
        self.speed_lbl = QLabel()
        self.speed_lbl.setFixedWidth(92)
        self.speed_lbl.setToolTip("Teleop tracking-gain scale (kp). Higher = faster/stiffer; "
                                  "the torque cap auto-tightens so it stays safe.")
        srow.addWidget(self.speed_lbl)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(10, 100)      # 0.10× .. 1.00×
        self.speed_slider.setValue(int(float(self.params.get("kp_scale", 0.3)) * 100))
        self.speed_slider.setToolTip("Drag to change teleop tracking speed live")
        self.speed_slider.valueChanged.connect(self._on_speed)
        srow.addWidget(self.speed_slider, 1)
        col.addLayout(srow)
        self._on_speed(self.speed_slider.value())   # set the label

        # Grip strength (gripper KP multiplier) — decoupled from Speed, writes
        # params["grip_strength"]. Higher = firmer grasp + a stronger force-feedback
        # signal; SAT_TORQUE still caps the actual gripper torque so it can't crush.
        grow0 = QHBoxLayout()
        self.grip_kp_lbl = QLabel()
        self.grip_kp_lbl.setFixedWidth(92)
        self.grip_kp_lbl.setToolTip("How firmly the grippers squeeze, independent of arm speed. "
                                    "Higher grips harder (and gives more force-feedback to feel).")
        grow0.addWidget(self.grip_kp_lbl)
        self.grip_kp_slider = QSlider(Qt.Horizontal)
        self.grip_kp_slider.setRange(5, 60)      # 0.5× .. 6.0× of base gripper KP
        self.grip_kp_slider.setValue(int(float(self.params.get("grip_strength", 3.0)) * 10))
        self.grip_kp_slider.setToolTip("Drag to change gripper squeeze firmness live")
        self.grip_kp_slider.valueChanged.connect(self._on_grip_strength)
        grow0.addWidget(self.grip_kp_slider, 1)
        col.addLayout(grow0)
        self._on_grip_strength(self.grip_kp_slider.value())

        # -- Gripper force-feedback (leader haptics) --
        # Off by default. Enable only after teleop feels good; start at low gain
        # and use Invert if the feedback pushes WITH your squeeze (wrong polarity).
        ff_group = QGroupBox("Gripper force-feedback")
        ff_group.setToolTip("Feel the follower's grasp on the leader gripper. Renders the "
                            "follower gripper's torque as a resist current on the leader "
                            "gripper, only while teleop is engaged. Open-only: it can push "
                            "back against a squeeze but never pull your hand closed.")
        ffv = QVBoxLayout(ff_group)
        ffrow = QHBoxLayout()
        self.grip_ff_enable = QCheckBox("Enable")
        self.grip_ff_enable.setChecked(bool(self.params.get("grip_ff", False)))
        self.grip_ff_enable.setToolTip("Turn leader gripper force-feedback on/off")
        self.grip_ff_enable.toggled.connect(self._on_grip_ff_enable)
        ffrow.addWidget(self.grip_ff_enable)
        self.grip_ff_invert = QCheckBox("Invert")
        self.grip_ff_invert.setChecked(int(self.params.get("grip_ff_sign", -1)) >= 0)
        self.grip_ff_invert.setToolTip("Flip current polarity — use if feedback assists your "
                                       "squeeze instead of resisting it (hardware-specific)")
        self.grip_ff_invert.toggled.connect(self._on_grip_ff_invert)
        ffrow.addWidget(self.grip_ff_invert)
        ffv.addLayout(ffrow)
        grow = QHBoxLayout()
        self.grip_ff_lbl = QLabel()
        self.grip_ff_lbl.setFixedWidth(110)
        self.grip_ff_lbl.setToolTip("Feedback strength: leader mA per Nm of follower grasp torque")
        grow.addWidget(self.grip_ff_lbl)
        self.grip_ff_slider = QSlider(Qt.Horizontal)
        self.grip_ff_slider.setRange(0, 600)     # mA per Nm (soft gripper -> needs headroom)
        self.grip_ff_slider.setValue(int(float(self.params.get("grip_ff_gain", 200.0))))
        self.grip_ff_slider.setToolTip("Drag to change force-feedback strength live")
        self.grip_ff_slider.valueChanged.connect(self._on_grip_ff_gain)
        grow.addWidget(self.grip_ff_slider, 1)
        ffv.addLayout(grow)
        self._on_grip_ff_gain(self.grip_ff_slider.value())   # set the label
        self.grip_ff_read = QLabel("L: —   R: —")
        self.grip_ff_read.setStyleSheet("color:#9c9;")
        self.grip_ff_read.setToolTip("Applied leader gripper current (mA) — left / right")
        ffv.addWidget(self.grip_ff_read)
        col.addWidget(ff_group)

        # -- Gravity feedforward (arm droop cancellation) --
        # Off by default. Requires a model from minerva_gravity_calibrate.py; if none
        # loaded, the whole group is disabled. Ramp Scale up from 0 while watching the
        # arm the first time — the feedforward offloads gravity so the arms feel light.
        grav_ok = bool(self.params.get("grav_ok", False))
        grav_group = QGroupBox("Gravity feedforward")
        grav_group.setToolTip("Cancels arm gravity droop: feeds the identified per-joint "
                              "holding torque forward so the arms no longer feel 'slow up, "
                              "fast down'. Applied only while enabled + PD-controlled.")
        gv = QVBoxLayout(grav_group)
        grav_top = QHBoxLayout()
        self.grav_enable = QCheckBox("Enable")
        self.grav_enable.setChecked(bool(self.params.get("grav_comp", False)) and grav_ok)
        self.grav_enable.setEnabled(grav_ok)
        self.grav_enable.setToolTip("Turn arm gravity feedforward on/off" if grav_ok
                                    else "No gravity model loaded — run minerva_gravity_calibrate.py")
        self.grav_enable.toggled.connect(self._on_grav_enable)
        grav_top.addWidget(self.grav_enable)
        self.grav_read = QLabel("—")
        self.grav_read.setStyleSheet("color:#9c9;")
        self.grav_read.setToolTip("Peak gravity feedforward applied this tick (Nm)")
        grav_top.addWidget(self.grav_read)
        grav_top.addStretch(1)
        gv.addLayout(grav_top)
        srow = QHBoxLayout()
        self.grav_scale_lbl = QLabel()
        self.grav_scale_lbl.setFixedWidth(110)
        self.grav_scale_lbl.setToolTip("Global trim on the feedforward — ramp up from 0 while validating")
        srow.addWidget(self.grav_scale_lbl)
        self.grav_scale_slider = QSlider(Qt.Horizontal)
        self.grav_scale_slider.setRange(0, 120)     # percent (allow slight over-comp)
        self.grav_scale_slider.setValue(int(float(self.params.get("grav_scale", 1.0)) * 100))
        self.grav_scale_slider.setEnabled(grav_ok)
        self.grav_scale_slider.setToolTip("Drag to trim gravity feedforward live (0–120%)")
        self.grav_scale_slider.valueChanged.connect(self._on_grav_scale)
        srow.addWidget(self.grav_scale_slider, 1)
        gv.addLayout(srow)
        self._on_grav_scale(self.grav_scale_slider.value())   # set the label
        if not grav_ok:
            hint = QLabel("no model — run minerva_gravity_calibrate.py")
            hint.setStyleSheet("color:#c99; font-size:11px;")
            gv.addWidget(hint)
        col.addWidget(grav_group)

        col.addStretch(1)
        quit_btn = QPushButton("Quit")
        quit_btn.setToolTip("Quit the collector (Q) — asks for confirmation")
        quit_btn.clicked.connect(self._quit)
        _style_btn(quit_btn, "danger", 36)
        col.addWidget(quit_btn)
        return w

    @staticmethod
    def _record_qss(recording: bool) -> str:
        bg = "#1b5e20" if recording else "#b71c1c"
        return (f"QPushButton {{ font-size:19px; font-weight:bold; background:{bg};"
                " color:white; border:none; border-radius:0px; }"
                f" QPushButton:hover {{ background:{'#217026' if recording else '#c62828'}; }}")

    def _build_joints(self) -> QWidget:
        """Two balanced columns: left-side joints | right-side joints."""
        w = QFrame()
        w.setFrameShape(QFrame.StyledPanel)
        outer = QHBoxLayout(w)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(18)
        self.rows: Dict[int, _JointRow] = {}
        columns = [
            [("Left arm", list(range(0, 6))), ("L grip", [6]), ("Head", [14, 15])],
            [("Right arm", list(range(7, 13))), ("R grip", [13]), ("Lift", [16])],
        ]
        for groups in columns:
            colv = QVBoxLayout()
            colv.setSpacing(2)
            colv.addWidget(_JointHeader())    # column captions (POS · GAP · °C · Nm)
            for label, idxs in groups:
                hdr = QLabel(label)
                hdr.setStyleSheet("color:#fca85a;font-weight:bold;font-size:11px;"
                                  "letter-spacing:1px;padding-top:4px;")
                colv.addWidget(hdr)
                for j in idxs:
                    lo, hi = JOINT_LIMITS[j]
                    row = _JointRow(j, lo, hi, self._jog)
                    self.rows[j] = row
                    colv.addWidget(row)
            colv.addStretch(1)
            host = QWidget()
            host.setLayout(colv)
            outer.addWidget(host, 1)
        return w

    # -- actions --
    def _key(self, k: str):
        try:
            self.cmd_queue.put_nowait(k)
        except queue.Full:
            pass

    def _jog(self, idx: int, direction: float):
        try:
            self.teleop.jog(idx, direction)
        except Exception:
            pass

    # -- leader↔arm swap --
    def _swap_label(self) -> str:
        return ("⇄ Leaders: L→RIGHT  R→LEFT  (swapped)" if self.teleop.swapped
                else "⇄ Leaders: L→left  R→right  (normal)")

    def _update_swap_btn(self):
        self.swap_btn.setText(self._swap_label())

    def _toggle_swap(self, _=False):
        new = self.teleop.toggle_swap()      # None if refused (engaged)
        if new is None:
            QMessageBox.information(
                self, "Swap leaders",
                "Disengage teleop (T) before swapping — re-routing while engaged would "
                "make an arm lunge toward the other leader.")
            return
        self.settings.set("leader_swap", bool(new))
        self.settings.save()
        self._update_swap_btn()

    def _on_speed(self, v: int):
        ks = max(0.10, min(1.0, v / 100.0))
        self.params["kp_scale"] = ks          # shared with the control loop
        self.speed_lbl.setText(f"Speed {ks:.2f}×")

    def _on_grip_strength(self, v: int):
        gs = max(0.5, v / 10.0)
        self.params["grip_strength"] = gs     # gripper KP multiplier, shared with the loop
        self.grip_kp_lbl.setText(f"Grip {gs:.1f}×")

    # -- gripper force feedback (shared with the control loop via self.params) --
    def _on_grip_ff_enable(self, checked: bool):
        self.params["grip_ff"] = bool(checked)

    def _on_grip_ff_invert(self, checked: bool):
        # Invert checked -> +1, unchecked -> -1 (the constant default). Which
        # polarity actually opens the leader gripper is hardware-specific.
        self.params["grip_ff_sign"] = 1 if checked else -1

    def _on_grip_ff_gain(self, v: int):
        self.params["grip_ff_gain"] = float(v)
        self.grip_ff_lbl.setText(f"Gain {int(v)} mA/Nm")

    # -- gravity feedforward (shared with the control loop via self.params) --
    def _on_grav_enable(self, checked: bool):
        self.params["grav_comp"] = bool(checked)

    def _on_grav_scale(self, v: int):
        self.params["grav_scale"] = v / 100.0
        self.grav_scale_lbl.setText(f"Scale {int(v)}%")

    def _set_phase(self, _=False):
        lbl = self.action_combo.currentText().strip()
        if not lbl:
            return
        try:
            self.label_queue.put_nowait(lbl)
        except queue.Full:
            pass

    # -- voice action-labeling --
    def _setup_voice(self):
        self._voice_worker = None
        self._warm_ok = False
        self._warm_err = None
        self._warm_timer = None
        self._stt = None
        self.voice_btn.clicked.connect(self._start_voice)   # connected once
        self._reload_stt()

    def _reload_stt(self):
        """(Re)build the STT from current settings and (re)warm it — at startup
        and whenever the microphone / model settings change."""
        if self._warm_timer is not None:
            self._warm_timer.stop()
            self._warm_timer = None
        self._warm_ok = False
        self._warm_err = None
        try:
            from collect_minerva_app.speech import SpeechToText
            self._stt = SpeechToText(whisper_model=self.settings.get("voice_model"),
                                     device=self.settings.get("mic_device"))
        except Exception:
            self._stt = None
        if self._stt is None or not self._stt.available():
            self.voice_btn.setEnabled(False)
            self.voice_btn.setToolTip(
                "Voice-to-text unavailable — install a LOCAL backend:\n"
                "pip install faster-whisper sounddevice")
            return
        if os.environ.get("QT_QPA_PLATFORM") == "offscreen":
            self.voice_btn.setEnabled(True)
            self.voice_btn.setToolTip(f"Speak the action label (voice: {self._stt.name})")
            return
        # Warm the model in the background so the first mic press isn't a long
        # wait; enable the mic once it's ready.
        self.voice_btn.setEnabled(False)
        self.voice_btn.setToolTip(f"Loading local voice model ({self._stt.name})…")
        self.seg_lbl.setText("voice: loading model…")
        self._warm_thread = threading.Thread(target=self._warm_stt, daemon=True)
        self._warm_thread.start()
        self._warm_timer = QTimer()
        self._warm_timer.timeout.connect(self._check_warm)
        self._warm_timer.start(400)

    def _open_settings(self):
        dlg = _SettingsDialog(self.settings, self.params, self)
        if dlg.exec() == QDialog.Accepted:
            vals = dlg.values()
            self.settings.update(vals)
            self.settings.save()
            self._voice_seconds = float(self.settings.get("voice_seconds"))
            self._reload_stt()           # apply the new microphone / model live
            self._apply_settings_live(vals)

    def _apply_settings_live(self, v: dict):
        """Push dialog values into the live control loop. Runtime prefs go through the
        existing sliders/checkboxes (so widget + params stay in sync); arm gains write
        params directly and bump gains_rev, which the loop watches to re-splat the gains."""
        # teleop + feedback: drive the existing controls (their handlers update params)
        if hasattr(self, "speed_slider"):
            self.speed_slider.setValue(int(round(float(v["kp_scale"]) * 100)))
        if hasattr(self, "grip_kp_slider"):
            self.grip_kp_slider.setValue(int(round(float(v["grip_strength"]) * 10)))
        if hasattr(self, "grav_enable") and self.grav_enable.isEnabled():
            self.grav_enable.setChecked(bool(v["grav_comp"]))
        if hasattr(self, "grav_scale_slider"):
            self.grav_scale_slider.setValue(int(round(float(v["grav_scale"]) * 100)))
        if hasattr(self, "grip_ff_enable"):
            self.grip_ff_enable.setChecked(bool(v["grip_ff"]))
        if hasattr(self, "grip_ff_slider"):
            self.grip_ff_slider.setValue(int(round(float(v["grip_ff_gain"]))))
        if hasattr(self, "grip_ff_invert"):
            self.grip_ff_invert.setChecked(bool(v["grip_ff_invert"]))
        # arm tuning: hand the loop new per-joint gains + trigger a rebuild
        self.params["arm_kp"] = [float(x) for x in v["arm_kp"]]
        self.params["arm_kd"] = [float(x) for x in v["arm_kd"]]
        self.params["arm_sat"] = [float(x) for x in v["arm_sat"]]
        self.params["gains_rev"] = int(self.params.get("gains_rev", 0)) + 1

    def _warm_stt(self):
        try:
            self._stt.warmup()
            self._warm_ok = True
        except Exception as e:   # noqa: BLE001
            self._warm_err = str(e)

    def _check_warm(self):
        if self._warm_ok:
            self._warm_timer.stop()
            self.voice_btn.setEnabled(True)
            self.voice_btn.setToolTip(f"Speak the action label (voice: {self._stt.name})")
            self.seg_lbl.setText("voice: ready")
        elif self._warm_err:
            self._warm_timer.stop()
            self.voice_btn.setToolTip(f"Voice model failed to load: {self._warm_err}")
            self.seg_lbl.setText("voice: model load failed")

    def _start_voice(self):
        if self._stt is None or not self._stt.available():
            return
        if self._voice_worker is not None and self._voice_worker.isRunning():
            return
        self.voice_btn.setText("●")
        self.voice_btn.setEnabled(False)
        self.seg_lbl.setText(f"🔴 listening — speak the action now ({self._voice_seconds:.0f}s)…")
        self._voice_worker = _VoiceWorker(self._stt, seconds=self._voice_seconds)
        self._voice_worker.done.connect(self._on_voice_done)
        self._voice_worker.failed.connect(self._on_voice_error)
        self._voice_worker.start()

    def _reset_voice_btn(self):
        self.voice_btn.setText("🎤")
        self.voice_btn.setEnabled(True)

    def _on_voice_done(self, text: str):
        self._reset_voice_btn()
        text = (text or "").strip()
        if not text:
            self.seg_lbl.setText("voice: nothing heard — try again")
            return
        resp = QMessageBox.question(
            self, "Confirm action label",
            f"Heard:\n\n  “{text}”\n\nUse this as the current action?",
            QMessageBox.Yes | QMessageBox.Retry | QMessageBox.Cancel, QMessageBox.Yes)
        if resp == QMessageBox.Yes:
            self._apply_voice_label(text)
        elif resp == QMessageBox.Retry:
            self._start_voice()

    def _on_voice_error(self, msg: str):
        self._reset_voice_btn()
        self.seg_lbl.setText(f"voice error: {msg}")

    def _apply_voice_label(self, text: str):
        """Set the combo to `text` and mark it as the current phase (used by the
        voice confirm dialog; also the unit-test entry point)."""
        self.action_combo.setCurrentText(text)
        self._set_phase()

    # -- keyboard hotkeys (phase cycling + control keys) --
    def _setup_shortcuts(self):
        specs = [
            ("E", lambda: self._key("E")), ("I", lambda: self._key("I")),
            ("H", lambda: self._key("H")), ("T", lambda: self._key("T")),
            ("R", lambda: self._key("R")), ("Q", self._quit),
            # Zero functions + pose + soft shutdown (collect_demo parity).
            ("Z", lambda: self._key("Z")), ("M", lambda: self._key("M")),
            ("K", lambda: self._key("K")), ("P", lambda: self._key("P")),
            ("X", lambda: self._key("X")),
            ("]", lambda: self._cycle_phase(1)), ("[", lambda: self._cycle_phase(-1)),
        ]
        for key, fn in specs:
            QShortcut(QKeySequence(key), self).activated.connect(self._guarded(fn))
        for i in range(1, 10):
            QShortcut(QKeySequence(str(i)), self).activated.connect(
                self._guarded(lambda ii=i: self._pick_phase(ii - 1)))

    def _guarded(self, fn):
        """Wrap a shortcut so it is ignored while a text field is focused — don't
        hijack keystrokes during instruction / label entry."""
        def _w():
            if isinstance(QApplication.focusWidget(), QLineEdit):
                return
            fn()
        return _w

    def _cycle_phase(self, direction: int):
        vocab = _ACTION_VOCAB
        cur = self.action_combo.currentText().strip()
        try:
            idx = vocab.index(cur)
        except ValueError:
            idx = -1 if direction > 0 else 0
        idx = (idx + direction) % len(vocab)
        self.action_combo.setCurrentText(vocab[idx])
        self._set_phase()

    def _pick_phase(self, idx: int):
        if 0 <= idx < len(_ACTION_VOCAB):
            self.action_combo.setCurrentText(_ACTION_VOCAB[idx])
            self._set_phase()

    def _on_instr(self, text: str):
        self.meta["language_instruction"] = text

    def _quit(self):
        r = QMessageBox.question(
            self, "Quit collector",
            "Quit the Minerva collector?\n\nThe follower arms will be disabled and any "
            "in-progress recording finalized.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if r != QMessageBox.Yes:
            return
        self._key("Q")
        self.close()

    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
            self.fs_btn.setText("⛶  Fullscreen")
        else:
            self.showFullScreen()
            self.fs_btn.setText("⛶  Windowed")

    # -- snapshot updates (called on the GUI thread by QtRenderer) --
    def apply_snapshot(self, s: dict):
        st = s.get("state", "?")
        sb = self.stat
        sb["state"].set(st, _STATE_COLORS.get(st, _C_IDLE))
        # Leaders can't be re-routed while engaging/teleoping (would lunge) — grey out.
        if hasattr(self, "swap_btn"):
            self.swap_btn.setEnabled(st not in ("TELEOP", "ENGAGING"))

        estop = s.get("estop")
        if estop is None:
            sb["estop"].set("—", _C_IDLE)
        else:
            sb["estop"].set("STOP" if estop else "CLEAR", _C_BAD if estop else _C_OK)

        batt = s.get("battery")
        if batt is None:
            sb["batt"].set("—", _C_IDLE)
        else:
            bc = _C_OK if batt >= 23.0 else _C_WARN if batt >= 21.5 else _C_BAD
            sb["batt"].set(f"{batt:.1f}V", bc)

        present = s.get("present") or {}
        if s.get("both_ok"):
            sb["arms"].set("BOTH OK", _C_OK)
        elif s.get("robot_ok"):
            sb["arms"].set(("R DOWN" if present.get("left") else "L DOWN"), _C_BAD)
        else:
            sb["arms"].set("NO TELEM", _C_BAD)

        temp = s.get("temp")
        tvals = [t for t in (temp or []) if isinstance(t, (int, float)) and t == t]  # drop NaN
        if tvals:
            mx = max(tvals)
            sb["temp"].set(f"{mx:.0f}°C", _tempbar_color(mx))
        else:
            sb["temp"].set("—", _C_IDLE)

        led = s.get("leaders", {})
        sb["left"].set("✓" if led.get("left") else "✗", _C_OK if led.get("left") else _C_BAD)
        sb["right"].set("✓" if led.get("right") else "✗", _C_OK if led.get("right") else _C_BAD)

        ages = s.get("cam_ages", {})
        nfresh = sum(1 for a in ages.values() if a < 0.5)
        ntot = len(ages)
        cc = _C_OK if (ntot and nfresh == ntot) else (_C_WARN if nfresh else _C_BAD)
        sb["cams"].set(f"{nfresh}/{ntot}", cc)
        for c, tile in self.cam_tiles.items():   # per-monitor lamp + age readout
            tile.set_status(ages.get(c))

        age = s.get("telem_age")
        if age is None:
            sb["telem"].set("—", _C_IDLE)
        else:
            tc = _C_OK if age < 0.15 else _C_WARN if age < 0.5 else _C_BAD
            sb["telem"].set(f"{age * 1000:.0f}ms", tc)

        host = s.get("jetson")
        linked = age is not None and age < 0.5
        sb["jetson"].set(host or "—", _C_OK if linked else _C_BAD)

        # Jetson host metrics + logic-UPS (from the heartbeat server; None if down).
        hm = s.get("host")
        if not hm:
            for k in ("ups", "cpu", "mem", "disk", "wifi"):
                sb[k].set("—", _C_IDLE)
        else:
            up, uv = hm.get("ups_pct"), hm.get("ups_v")
            if up is None:
                sb["ups"].set("—", _C_IDLE)
            else:
                uc = _C_OK if up >= 50 else _C_WARN if up >= 20 else _C_BAD
                sb["ups"].set(f"{up:.0f}%  {uv:.1f}V" if uv is not None else f"{up:.0f}%", uc)
            for k in ("cpu", "mem", "disk"):   # higher = worse
                v = hm.get(k)
                if v is None:
                    sb[k].set("—", _C_IDLE)
                else:
                    c = _C_OK if v < 70 else _C_WARN if v < 90 else _C_BAD
                    sb[k].set(f"{v:.0f}%", c)
            wifi = hm.get("wifi")
            sb["wifi"].set(wifi or "—", _C_OK if wifi else _C_IDLE)

        rec = s.get("recording")
        steps = s.get("rec_steps", 0)
        dropped = s.get("dropped", 0)
        if rec:
            self.record_btn.setText("■ STOP")
            self.record_btn.setStyleSheet(self._record_qss(True))
            self.rec_status.setText(f"recording — {steps} steps (dropped {dropped})")
        else:
            self.record_btn.setText("● RECORD")
            self.record_btn.setStyleSheet(self._record_qss(False))
            self.rec_status.setText("not recording" if st == "TELEOP"
                                    else "engage teleop (T) to record")
        self.seg_lbl.setText(
            f"phase: {s.get('current_label') or '—'}    segments: {s.get('seg_count', 0)}")

        qpos = s.get("qpos")
        target = s.get("target")
        leader = s.get("leader")
        temp = s.get("temp")
        torque = s.get("torque")
        for j, row in self.rows.items():
            a = qpos[j] if qpos else None
            led = leader[j] if leader else None
            t = target[j] if target else None
            tc = temp[j] if temp else None
            tq = torque[j] if torque else None
            row.set(a, led, t, tc, tq)

        ff = s.get("grip_ff_ma") or {}
        if hasattr(self, "grip_ff_read"):
            self.grip_ff_read.setText(
                f"L: {int(ff.get('left', 0))} mA   R: {int(ff.get('right', 0))} mA")

        if hasattr(self, "grav_read"):
            if s.get("grav_on"):
                self.grav_read.setText(f"peak {s.get('grav_peak', 0.0):.1f} Nm")
                self.grav_read.setStyleSheet("color:#9c9;")
            else:
                self.grav_read.setText("off")
                self.grav_read.setStyleSheet("color:#888;")

        if s.get("last_saved"):
            self.saved_lbl.setText(f"last saved: {Path(s['last_saved']).name}")
        self.viewer.on_saved(s.get("last_saved"))

    def set_camera_frames(self, frames: Dict[str, bytes], ts: Optional[Dict] = None):
        for c, jpeg in frames.items():
            tile = self.cam_tiles.get(c)
            if tile is not None and jpeg is not None:
                tile.set_jpeg(jpeg, ts.get(c) if ts else None)

    def closeEvent(self, event):
        # Stop the warm-poll timer.
        t = getattr(self, "_warm_timer", None)
        if t is not None:
            try:
                t.stop()
            except Exception:
                pass
        # Wait (bounded) for an in-flight voice capture, so the QThread isn't
        # destroyed while running (that would abort the process).
        w = getattr(self, "_voice_worker", None)
        if w is not None:
            try:
                if w.isRunning():
                    w.wait(3000)
            except Exception:
                pass
        # Wait (bounded) for a model warm-load in progress so its C-extension
        # call isn't killed mid-flight.
        wt = getattr(self, "_warm_thread", None)
        if wt is not None and wt.is_alive():
            wt.join(timeout=5.0)
        try:
            self.viewer.close_viewer()
        except Exception:
            pass
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# QtRenderer — worker-thread QApplication + holders
# ---------------------------------------------------------------------------

class QtRenderer:
    def __init__(self, cmd_queue, meta, teleop, cameras, output_dir, label_queue=None,
                 params=None):
        self.cmd_queue = cmd_queue
        self.meta = meta
        self.teleop = teleop
        self.cameras = list(cameras)
        self.output_dir = output_dir
        self.label_queue = label_queue if label_queue is not None else queue.Queue()
        self.params = params if params is not None else {}
        self.lock = threading.Lock()
        self.holder: dict = {}
        self.cam_lock = threading.Lock()
        self.cam_holder: dict = {}
        self._stop = threading.Event()
        self._closed = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._app = None
        self._win: Optional[MinervaMainWindow] = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="QtRenderer")
        self._thread.start()

    def _run(self):
        try:
            app = QApplication.instance() or QApplication([])
            self._app = app
            win = MinervaMainWindow(self.cmd_queue, self.meta, self.teleop,
                                    self.cameras, self.output_dir, self.label_queue, self.params)
            self._win = win
            win.show()
            timer = QTimer()
            timer.timeout.connect(self._tick)
            timer.start(33)
            app.exec()
        except Exception as e:   # a setup crash must still signal the main loop
            import traceback
            print(f"[gui] renderer thread crashed: {e}")
            traceback.print_exc()
        finally:
            self._closed.set()   # always tell the main loop the GUI is gone

    def _tick(self):
        if self._stop.is_set():
            if self._app is not None:
                self._app.quit()
            return
        with self.lock:
            snap = self.holder.get("args")
        if snap and self._win is not None:
            self._win.apply_snapshot(snap)
        frames, tss = {}, {}
        with self.cam_lock:
            for c in self.cameras:
                jb = self.cam_holder.get(c)
                if jb is not None:
                    frames[c] = jb
                    tss[c] = self.cam_holder.get(f"{c}_ts")
        if frames and self._win is not None:
            self._win.set_camera_frames(frames, tss)

    def should_quit(self) -> bool:
        return self._closed.is_set()

    def request_quit(self):
        self._stop.set()
        app = self._app
        if app is not None:
            # Post quit onto the GUI thread's event loop so app.exec() returns
            # even if the tick timer isn't firing (thread-safe, queued).
            try:
                from PySide6.QtCore import QMetaObject
                QMetaObject.invokeMethod(app, "quit", Qt.QueuedConnection)
            except Exception:
                pass

    def join(self, timeout: Optional[float] = None):
        if self._thread is not None:
            self._thread.join(timeout)


__all__ = ["QtRenderer", "MinervaMainWindow"]
