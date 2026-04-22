"""Qt control panel for collect_demo.py.

Operator-facing GUI that wraps a trimmed-down Rerun camera embed (panels +
timeline hidden) with native Qt widgets for status, joint state, temperature
monitoring, recording controls, and live shortcut hints.

Layout (L2 — cameras dominant on the left, controls stacked on the right):

    ┌────────────────────────────────┬──────────────────┐
    │  Status pills + hot-temp banner                    │
    ├────────────────────────────────┼──────────────────┤
    │                                │  RECORD (big)    │
    │  Cameras (Rerun embed)         │  Task tag        │
    │                                │  Motion buttons  │
    │                                │  Teleop buttons  │
    ├────────────────────────────────┤  Last save       │
    │  Joint bars + temp badges      │                  │
    ├────────────────────────────────┴──────────────────┤
    │  Shortcut legend                                  │
    └───────────────────────────────────────────────────┘

The QApplication runs on a worker thread; the main collect_demo loop pushes
display snapshots via _disp_holder (same pattern as the ANSI renderer).
GUI buttons post synthetic key-strings into a queue the main loop drains
alongside keyboard/gamepad input.
"""

from __future__ import annotations

import math
from collections import deque
import queue
import threading
import time
from pathlib import Path
from typing import Optional, Callable

import numpy as np

from PySide6.QtCore import Qt, QTimer, QUrl, QPoint, QThread, Signal
from PySide6.QtGui import (
    QAction, QColor, QDesktopServices, QFont, QImage, QKeySequence,
    QPainter, QPainterPath, QPen, QPixmap, QShortcut,
)
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
    QMainWindow, QMenu, QMessageBox, QProgressBar, QPushButton, QSizePolicy,
    QScrollArea, QSlider, QSplitter, QStackedWidget, QVBoxLayout, QWidget,
)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JOINT_NAMES = ["swivel", "gantry_base", "gantry_mid", "gantry_end",
               "wrist_pitch", "wrist_roll", "gripper"]

# Approximate software limits per joint (radians).  Used only for the visual
# bar range — doesn't affect commands.  Matches AIZEE_DEFAULTS in so101_leader.
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "swivel":       (-3.14, 3.14),
    "gantry_base":  (-3.14, 3.14),
    "gantry_mid":   (-3.14, 3.14),
    "gantry_end":   (-3.14, 3.14),
    "wrist_pitch":  (-3.14, 3.14),
    "wrist_roll":   (-3.14, 3.14),
    "gripper":      ( 0.00, 1.57),
}

# Temperature thresholds (°C).
TEMP_WARN     = 50.0
TEMP_HOT      = 65.0
TEMP_CRITICAL = 80.0

# Per-joint saturation torque (N·m) — used to color torque values by load
# fraction.  Mirrors _SAT_TORQUE in collect_demo.py.
_SAT_TORQUE: dict[str, float] = {
    "swivel":      12.0,
    "gantry_base": 24.0,
    "gantry_mid":  12.0,
    "gantry_end":   5.0,
    "wrist_pitch":  5.0,
    "wrist_roll":   0.5,
    "gripper":      0.5,
}
TORQUE_WARN_FRAC = 0.60
TORQUE_HOT_FRAC  = 0.85

# Palette.
COL_BG         = "#1a1a1a"
COL_PANEL      = "#232323"
COL_PANEL_ALT  = "#2a2a2a"
COL_BORDER     = "#3a3a3a"
COL_TEXT       = "#e8e8e8"
COL_MUTED      = "#888"
COL_ACCENT     = "#4a9eff"
COL_OK         = "#2a8a3d"
COL_WARN       = "#b88710"
COL_HOT        = "#d05a1a"
COL_CRIT       = "#c42020"


# ---------------------------------------------------------------------------
# Reusable styled helpers
# ---------------------------------------------------------------------------

def _hline() -> QFrame:
    f = QFrame(); f.setFrameShape(QFrame.HLine); f.setFrameShadow(QFrame.Plain)
    f.setStyleSheet(f"color: {COL_BORDER};")
    return f


def _temp_colors(temp: Optional[float]) -> tuple[str, str]:
    """Return (background, foreground) for a given temperature in °C."""
    if temp is None or math.isnan(temp):
        return ("#333", COL_MUTED)
    if temp >= TEMP_CRITICAL:
        return (COL_CRIT, "white")
    if temp >= TEMP_HOT:
        return (COL_HOT, "white")
    if temp >= TEMP_WARN:
        return (COL_WARN, "white")
    return (COL_OK, "white")


def _torque_color(joint: str, torque: Optional[float]) -> str:
    """Color for a torque value based on |torque| / saturation for that joint."""
    if torque is None or math.isnan(torque):
        return COL_MUTED
    sat = _SAT_TORQUE.get(joint, 1.0)
    ratio = abs(torque) / sat if sat > 0 else 0.0
    if ratio >= TORQUE_HOT_FRAC:  return COL_CRIT
    if ratio >= TORQUE_WARN_FRAC: return COL_WARN
    return COL_OK


def _state_colors(state: str, estop: bool) -> tuple[str, str]:
    if estop:
        return (COL_CRIT, "white")
    return {
        "ready":    ("#444", "#bbb"),
        "idle":     (COL_WARN, "white"),
        "tracking": (COL_OK, "white"),
        "engaging": ("#b07a2a", "white"),  # amber — slow ramp before tracking
        "hold":     ("#2a6ab0", "white"),
        "shutdown": (COL_CRIT, "white"),
        "estop":    (COL_CRIT, "white"),
    }.get(state, ("#444", "#bbb"))


# ---------------------------------------------------------------------------
# Status pill widget
# ---------------------------------------------------------------------------

class _Pill(QLabel):
    """Rounded badge with a label + dynamic color."""

    def __init__(self, text: str = "—", bg: str = "#333", fg: str = "#ccc",
                 min_w: int = 80, font_family: Optional[str] = None) -> None:
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(min_w)
        self._font_family = font_family
        self._set(bg, fg)

    def set_pill(self, text: str, bg: str, fg: str = "white") -> None:
        self.setText(text)
        self._set(bg, fg)

    def _set(self, bg: str, fg: str) -> None:
        family = (f"font-family: '{self._font_family}'; "
                  if self._font_family else "")
        self.setStyleSheet(
            f"background: {bg}; color: {fg}; "
            f"padding: 6px 14px; border-radius: 12px; "
            f"font-weight: 600; font-size: 10pt; {family}"
        )


# ---------------------------------------------------------------------------
# Joint position bar (custom-painted)
# ---------------------------------------------------------------------------

class _PositionBar(QWidget):
    """Horizontal bar showing actual fill + target (▾ yellow) + leader (◆ cyan).

    When the leader diverges from the commanded target (e.g. in HOLD/IDLE
    while the operator is moving the leader freely), a dotted connector is
    drawn between the two markers so the operator sees the mismatch at a
    glance.  An error shade between actual-fill-edge and target highlights
    live tracking error intensity-by-width.
    """

    COL_LEADER = "#4ad8ff"   # cyan — raw leader input
    COL_TARGET = "#ffd84a"   # yellow — commanded target

    def __init__(self, limits: tuple[float, float]) -> None:
        super().__init__()
        self._lo, self._hi = limits
        self._actual: Optional[float] = None
        self._target: Optional[float] = None
        self._leader: Optional[float] = None
        self.setMinimumHeight(22)
        self.setMinimumWidth(160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

    def set_values(
        self,
        actual: Optional[float],
        target: Optional[float],
        leader: Optional[float] = None,
    ) -> None:
        self._actual = actual
        self._target = target
        self._leader = leader
        self.update()

    def _to_x(self, val: float, w: int, span: float) -> int:
        v = max(self._lo, min(self._hi, val))
        return int(w * (v - self._lo) / span)

    def paintEvent(self, _ev) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, False)
        w, h = self.width(), self.height()
        span = self._hi - self._lo
        if span <= 0 or w <= 0:
            return

        # Background track.
        p.fillRect(0, 0, w, h, QColor("#1e1e1e"))

        # Zero tick.
        zero_x = int(w * (0.0 - self._lo) / span)
        p.setPen(QPen(QColor("#555"), 1))
        p.drawLine(zero_x, 2, zero_x, h - 2)

        a_x = t_x = l_x = None
        if self._actual is not None and not math.isnan(self._actual):
            a_x = self._to_x(self._actual, w, span)
        if self._target is not None and not math.isnan(self._target):
            t_x = self._to_x(self._target, w, span)
        if self._leader is not None and not math.isnan(self._leader):
            l_x = self._to_x(self._leader, w, span)

        # Actual fill (from zero).
        if a_x is not None:
            rect_x = min(zero_x, a_x)
            rect_w = max(1, abs(a_x - zero_x))
            p.fillRect(rect_x, 4, rect_w, h - 8, QColor(COL_OK))

        # Tracking error shade — thin strip between actual and target.
        if a_x is not None and t_x is not None and abs(a_x - t_x) > 1:
            span_px = abs((self._target or 0.0) - (self._actual or 0.0))
            if span_px >= 0.15:
                err_col = QColor(COL_CRIT); err_col.setAlpha(150)
            elif span_px >= 0.05:
                err_col = QColor(COL_HOT); err_col.setAlpha(140)
            else:
                err_col = QColor(COL_WARN); err_col.setAlpha(110)
            ex = min(a_x, t_x); ew = abs(a_x - t_x)
            p.fillRect(ex, h - 4, ew, 3, err_col)

        # Target marker (yellow triangle at top).
        if t_x is not None:
            p.setPen(QPen(QColor(self.COL_TARGET), 2))
            p.drawLine(t_x, 1, t_x, h - 1)
            tri = [QPoint(t_x - 4, 0), QPoint(t_x + 4, 0), QPoint(t_x, 5)]
            p.setBrush(QColor(self.COL_TARGET))
            p.drawPolygon(tri)

        # Leader marker (cyan diamond at bottom).
        if l_x is not None:
            p.setPen(QPen(QColor(self.COL_LEADER), 2))
            p.drawLine(l_x, 1, l_x, h - 1)
            y_b = h - 1
            diamond = [
                QPoint(l_x,     y_b - 6),
                QPoint(l_x + 4, y_b - 2),
                QPoint(l_x,     y_b),
                QPoint(l_x - 4, y_b - 2),
            ]
            p.setBrush(QColor(self.COL_LEADER))
            p.drawPolygon(diamond)

        # Border.
        p.setPen(QPen(QColor(COL_BORDER), 1))
        p.drawRect(0, 0, w - 1, h - 1)


# ---------------------------------------------------------------------------
# Temperature badge with pulse animation above critical
# ---------------------------------------------------------------------------

class _TempBadge(QLabel):
    """Colored pill showing joint temperature; pulses when critical."""

    def __init__(self) -> None:
        super().__init__("--")
        self.setAlignment(Qt.AlignCenter)
        self.setFixedWidth(62)
        self._temp: Optional[float] = None
        self._pulse_on = False
        self._pulse_timer = QTimer(self)
        self._pulse_timer.setInterval(450)
        self._pulse_timer.timeout.connect(self._toggle_pulse)
        self._apply(None)

    def set_temp(self, temp: Optional[float]) -> None:
        self._temp = temp
        if temp is None or math.isnan(temp):
            self.setText("--")
            self._stop_pulse()
            self._apply(None)
            return
        self.setText(f"{temp:.0f}°C")
        if temp >= TEMP_CRITICAL:
            if not self._pulse_timer.isActive():
                self._pulse_on = False
                self._pulse_timer.start()
        else:
            self._stop_pulse()
        self._apply(temp)

    def _stop_pulse(self) -> None:
        self._pulse_timer.stop()
        self._pulse_on = False

    def _toggle_pulse(self) -> None:
        self._pulse_on = not self._pulse_on
        self._apply(self._temp)

    def _apply(self, temp: Optional[float]) -> None:
        bg, fg = _temp_colors(temp)
        if self._pulse_on and temp is not None and temp >= TEMP_CRITICAL:
            bg = "#ff4040"
        self.setStyleSheet(
            f"background: {bg}; color: {fg}; "
            f"padding: 3px 6px; border-radius: 8px; "
            f"font-weight: 700; font-size: 9pt;"
        )


# ---------------------------------------------------------------------------
# One joint row: name | bar | target | actual | torque | temp
# ---------------------------------------------------------------------------

class _JointRow(QWidget):

    def __init__(self, name: str) -> None:
        super().__init__()
        lo, hi = JOINT_LIMITS.get(name, (-1.57, 1.57))

        row = QHBoxLayout(self)
        row.setContentsMargins(6, 3, 6, 3)
        row.setSpacing(10)

        self.lbl_name = QLabel(name)
        self.lbl_name.setFixedWidth(110)
        self.lbl_name.setStyleSheet(
            f"color: {COL_TEXT}; font-weight: 600; font-size: 10pt;")

        self.bar = _PositionBar((lo, hi))

        self.lbl_target = QLabel("--")
        self.lbl_target.setFixedWidth(58)
        self.lbl_target.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_target.setStyleSheet("color: #ffd84a; font-family: Consolas, monospace;")

        self.lbl_actual = QLabel("--")
        self.lbl_actual.setFixedWidth(58)
        self.lbl_actual.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_actual.setStyleSheet(f"color: {COL_TEXT}; font-family: Consolas, monospace;")

        self.lbl_torque = QLabel("--")
        self.lbl_torque.setFixedWidth(58)
        self.lbl_torque.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_torque.setStyleSheet(f"color: {COL_MUTED}; font-family: Consolas, monospace;")

        self.temp = _TempBadge()

        row.addWidget(self.lbl_name)
        row.addWidget(self.bar, 1)
        row.addWidget(self.lbl_target)
        row.addWidget(self.lbl_actual)
        row.addWidget(self.lbl_torque)
        row.addWidget(self.temp)

    def update_values(self, actual: Optional[float], target: Optional[float],
                      torque: Optional[float], temp: Optional[float],
                      leader: Optional[float] = None) -> None:
        self.bar.set_values(actual, target, leader)
        self.lbl_actual.setText(
            f"{actual:+.3f}" if actual is not None and not math.isnan(actual) else "--")
        self.lbl_target.setText(
            f"{target:+.3f}" if target is not None and not math.isnan(target) else "--")
        if torque is not None and not math.isnan(torque):
            self.lbl_torque.setText(f"{torque:+.2f}")
            if abs(torque) > 3.0:
                self.lbl_torque.setStyleSheet(
                    f"color: #ff8844; font-weight: 700; font-family: Consolas, monospace;")
            else:
                self.lbl_torque.setStyleSheet(
                    f"color: {COL_MUTED}; font-family: Consolas, monospace;")
        else:
            self.lbl_torque.setText("--")
            self.lbl_torque.setStyleSheet(f"color: {COL_MUTED}; font-family: Consolas, monospace;")
        self.temp.set_temp(temp)


# ---------------------------------------------------------------------------
# Tracking strip: 7 cells showing per-joint leader→actual error at a glance
# ---------------------------------------------------------------------------

# Short names displayed in the strip — keep to 3–4 chars each.
_JOINT_SHORT = {
    "swivel":      "swv",
    "gantry_base": "base",
    "gantry_mid":  "mid",
    "gantry_end":  "end",
    "wrist_pitch": "pit",
    "wrist_roll":  "roll",
    "gripper":     "grip",
}

# Error bucketing in radians (absolute delta between leader and actual).
_ERR_TIGHT   = 0.05   # green
_ERR_LOOSE   = 0.15   # yellow
# above _ERR_LOOSE is red


class _TrackingCell(QFrame):
    """Single joint cell in the tracking strip."""

    def __init__(self, joint: str) -> None:
        super().__init__()
        self.setFixedHeight(44)
        self.setMinimumWidth(70)
        self._joint = joint
        self._err: Optional[float] = None
        self._has_leader = False

        v = QVBoxLayout(self)
        v.setContentsMargins(4, 3, 4, 3)
        v.setSpacing(1)
        self.lbl_name = QLabel(_JOINT_SHORT.get(joint, joint[:4]))
        self.lbl_name.setAlignment(Qt.AlignCenter)
        self.lbl_name.setStyleSheet(
            f"color: {COL_TEXT}; font-weight: 700; font-size: 9pt;")

        self.bar = _ErrorFill()

        self.lbl_val = QLabel("—")
        self.lbl_val.setAlignment(Qt.AlignCenter)
        self.lbl_val.setStyleSheet(
            f"color: {COL_MUTED}; font-family: Consolas, monospace; font-size: 8pt;")

        v.addWidget(self.lbl_name)
        v.addWidget(self.bar, 1)
        v.addWidget(self.lbl_val)
        self._apply_border()

    def set_err(self, err: Optional[float], has_leader: bool) -> None:
        self._err = err
        self._has_leader = has_leader
        if not has_leader or err is None or math.isnan(err):
            self.lbl_val.setText("—")
            self.lbl_val.setStyleSheet(
                f"color: {COL_MUTED}; font-family: Consolas, monospace; font-size: 8pt;")
            self.bar.set_err(None)
            self._apply_border(neutral=True)
            return
        self.lbl_val.setText(f"Δ{err:+.2f}")
        self.bar.set_err(err)
        self._apply_border(neutral=False)

    def _apply_border(self, neutral: bool = True) -> None:
        if neutral or self._err is None:
            border = COL_BORDER
        else:
            mag = abs(self._err)
            if mag >= _ERR_LOOSE:  border = COL_CRIT
            elif mag >= _ERR_TIGHT: border = COL_WARN
            else:                    border = COL_OK
        self.setStyleSheet(
            f"_TrackingCell {{ background: {COL_PANEL_ALT}; "
            f"border: 1px solid {border}; border-radius: 5px; }}")


class _ErrorFill(QWidget):
    """Tiny horizontal bar filled green→yellow→red by |error|."""

    def __init__(self) -> None:
        super().__init__()
        self._err: Optional[float] = None
        self.setMinimumHeight(10)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

    def set_err(self, err: Optional[float]) -> None:
        self._err = err
        self.update()

    def paintEvent(self, _ev) -> None:
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#1c1c1c"))
        if self._err is None or math.isnan(self._err):
            return
        mag = abs(self._err)
        # Fill proportional to mag, clamped at _ERR_LOOSE * 2.
        full = min(1.0, mag / (_ERR_LOOSE * 2.0))
        fw = max(1, int(w * full))
        if mag >= _ERR_LOOSE:  col = QColor(COL_CRIT)
        elif mag >= _ERR_TIGHT: col = QColor(COL_WARN)
        else:                    col = QColor(COL_OK)
        p.fillRect(0, 0, fw, h, col)


class _TrackingStrip(QFrame):
    """Row of per-joint cells showing leader→actual tracking error."""

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_TrackingStrip {{ background: transparent; border: none; }}")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(2)

        title = QLabel("LEADER → ROBOT  tracking error (◆ cyan = leader, ▾ yellow = target)")
        title.setStyleSheet(
            f"color: {COL_MUTED}; font-size: 9pt; font-weight: 700; "
            f"letter-spacing: 1px; padding: 0 2px;")
        outer.addWidget(title)

        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(4)
        self._cells: dict[str, _TrackingCell] = {}
        for name in JOINT_NAMES:
            c = _TrackingCell(name)
            self._cells[name] = c
            row.addWidget(c, 1)
        outer.addLayout(row)

    def apply(
        self,
        actual: Optional[np.ndarray],
        leader: Optional[np.ndarray],
    ) -> None:
        has_leader = leader is not None
        for i, name in enumerate(JOINT_NAMES):
            if (actual is not None and leader is not None
                    and i < len(actual) and i < len(leader)):
                a = float(actual[i]); l = float(leader[i])
                err = l - a if not (math.isnan(a) or math.isnan(l)) else float("nan")
            else:
                err = float("nan")
            self._cells[name].set_err(err, has_leader)


# ---------------------------------------------------------------------------
# Joint panel with column header + hot-temp banner
# ---------------------------------------------------------------------------

class _JointPanel(QFrame):

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_JointPanel {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(2)

        # Tracking strip — per-joint leader→actual error at a glance.
        self.tracking = _TrackingStrip()
        root.addWidget(self.tracking)

        # Hot-temp banner (T3): shows only when some joint is at/past the HOT
        # threshold.  Prominent, red, and carries the worst joint's temp.
        self.hot_banner = QLabel("")
        self.hot_banner.setAlignment(Qt.AlignCenter)
        self.hot_banner.setStyleSheet(
            f"background: {COL_CRIT}; color: white; font-weight: 800; "
            f"font-size: 11pt; border-radius: 6px; padding: 6px;")
        self.hot_banner.hide()
        root.addWidget(self.hot_banner)

        # Column header.  Slot widths MUST mirror _JointRow exactly:
        # name=110, bar=stretch, target=58, actual=58, torque=58, temp=62.
        header = QHBoxLayout()
        header.setContentsMargins(6, 2, 6, 2)
        header.setSpacing(10)

        def _h_fixed(txt: str, w: int, align=Qt.AlignRight) -> QLabel:
            lbl = QLabel(txt); lbl.setFixedWidth(w)
            lbl.setAlignment(align | Qt.AlignVCenter)
            lbl.setStyleSheet(f"color: {COL_MUTED}; font-size: 9pt; font-weight: 600;")
            return lbl

        def _h_flex(txt: str) -> QLabel:
            lbl = QLabel(txt)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            lbl.setStyleSheet(f"color: {COL_MUTED}; font-size: 9pt; font-weight: 600;")
            return lbl

        header.addWidget(_h_fixed("joint", 110, Qt.AlignLeft))
        header.addWidget(_h_flex("position  (leader ◆  target ▾  actual ▬)"), 1)
        header.addWidget(_h_fixed("target", 58))
        header.addWidget(_h_fixed("actual", 58))
        header.addWidget(_h_fixed("torque", 58))
        header.addWidget(_h_fixed("temp",   62, Qt.AlignCenter))
        root.addLayout(header)

        # Joint rows.
        self._rows: dict[str, _JointRow] = {}
        for i, name in enumerate(JOINT_NAMES):
            row = _JointRow(name)
            if i % 2 == 0:
                row.setStyleSheet(f"_JointRow {{ background: {COL_PANEL_ALT}; "
                                  f"border-radius: 4px; }}")
            self._rows[name] = row
            root.addWidget(row)

        root.addStretch(0)

    def apply(self, target: Optional[np.ndarray], actual: Optional[np.ndarray],
              torque: Optional[np.ndarray], temp: Optional[np.ndarray],
              leader: Optional[np.ndarray] = None) -> None:
        hottest = (-1.0, "")
        for i, name in enumerate(JOINT_NAMES):
            a = float(actual[i]) if actual is not None and i < len(actual) else float("nan")
            t = float(target[i]) if target is not None and i < len(target) else float("nan")
            q = float(torque[i]) if torque is not None and i < len(torque) else float("nan")
            c = float(temp[i])   if temp   is not None and i < len(temp)   else float("nan")
            l = float(leader[i]) if leader is not None and i < len(leader) else float("nan")
            self._rows[name].update_values(a, t, q, c, leader=l)
            if not math.isnan(c) and c > hottest[0]:
                hottest = (c, name)

        self.tracking.apply(actual=actual, leader=leader)

        if hottest[0] >= TEMP_HOT:
            tag = "CRITICAL" if hottest[0] >= TEMP_CRITICAL else "HOT"
            self.hot_banner.setText(
                f"⚠  {tag}: {hottest[1]} at {hottest[0]:.0f}°C — pause recording to cool")
            bg = COL_CRIT if hottest[0] >= TEMP_CRITICAL else COL_HOT
            self.hot_banner.setStyleSheet(
                f"background: {bg}; color: white; font-weight: 800; "
                f"font-size: 11pt; border-radius: 6px; padding: 6px;")
            self.hot_banner.show()
        else:
            self.hot_banner.hide()


# ---------------------------------------------------------------------------
# Joint position time-series chart — used in two places:
#   - Replay tab while live mode is driving the arm (rolling window).
#   - Collect tab (rolling window when idle, accumulate-from-record-start
#     while recording, so each take's full trajectory is visible).
# ---------------------------------------------------------------------------

class _ChartCanvas(QWidget):
    """Paint surface for _LiveTimeSeriesPanel.  Delegates back to the parent
    panel so all chart state lives in one place."""

    def __init__(self, panel: "_LiveTimeSeriesPanel") -> None:
        super().__init__(panel)
        self._panel = panel
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(180)

    def paintEvent(self, _ev) -> None:  # noqa: N802 (Qt)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        self._panel._paint_canvas(p, self.rect())
        p.end()


class _LiveTimeSeriesPanel(QFrame):

    WINDOW_SEC      = 10.0          # rolling window default
    TARGET_COLOR    = QColor("#ffc83c")   # amber — matches existing target text
    ACTUAL_COLOR    = QColor("#5ab4ff")   # blue — matches Rerun qpos color
    REC_COLOR       = QColor("#ff5050")   # recording indicator
    GRID_COLOR      = QColor("#2a2a2a")
    AXIS_COLOR      = QColor("#3a3a3a")
    NAME_W          = 132           # left gutter for joint status tile
    VAL_W           = 84            # right gutter for current values
    ROW_PAD         = 6
    MIN_RANGE       = 0.05          # rad — minimum y-axis span so flat lines aren't flat-flat
    MAX_SAMPLES     = 12000         # hard cap on accumulate-mode buffer
    BANNER_H        = 22            # height reserved for the HOT/CRITICAL banner

    def __init__(self, *, title: str = "TARGET vs ACTUAL — last 10s") -> None:
        super().__init__()
        self.setObjectName("liveTSPanel")
        self.setStyleSheet(
            f"QFrame#liveTSPanel {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(260)

        # Tracking strip (leader → actual error) sits above the chart canvas.
        self.tracking = _TrackingStrip()
        self._canvas  = _ChartCanvas(self)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        outer.addWidget(self.tracking, 0)
        outer.addWidget(self._canvas,  1)

        # Bound state lives on the panel; canvas reads via parent reference.
        self._times:   deque[float]      = deque()
        self._targets: deque[np.ndarray] = deque()
        self._actuals: deque[np.ndarray] = deque()
        # Per-joint y-range cache so the axis doesn't twitch every frame.
        self._yrange_cache: list[tuple[float, float]] = [(0.0, 0.0)] * len(JOINT_NAMES)
        self._idle_title    = title
        self._accumulating  = False
        self._record_start: Optional[float] = None
        # Latest per-joint status (no history kept — always paint current).
        self._temps:    list[float] = [float("nan")] * len(JOINT_NAMES)
        self._states:   list[str]   = ["?"] * len(JOINT_NAMES)
        self._torques:  list[float] = [float("nan")] * len(JOINT_NAMES)
        # Slow pulse used for critical-temp row wash and LED.
        self._pulse_phase = 0
        self._pulse_timer = QTimer(self)
        self._pulse_timer.setInterval(450)
        self._pulse_timer.timeout.connect(self._tick_pulse)
        self._has_critical = False

        # Static-episode mode: shows a fully-loaded episode with a moving
        # playback cursor instead of a streaming rolling window.
        self._static_mode    = False
        self._cursor_frame   = 0
        self._episode_T      = 0
        self._episode_hz     = 20.0
        self._episode_label  = ""

    # ---- Mode --------------------------------------------------------------

    def set_accumulate(self, accumulate: bool) -> None:
        """Switch between rolling-window and accumulate-from-now modes.

        Entering accumulate mode resets the buffer so the chart shows only
        the new recording's data.  Leaving it trims to the rolling window.
        """
        if accumulate == self._accumulating:
            return
        self._accumulating = accumulate
        if accumulate:
            self._times.clear()
            self._targets.clear()
            self._actuals.clear()
            self._yrange_cache = [(0.0, 0.0)] * len(JOINT_NAMES)
            self._record_start = time.monotonic()
        else:
            self._record_start = None
            self._trim_to_window()
        self.update()

    def is_accumulating(self) -> bool:
        return self._accumulating

    # ---- Data ingestion ---------------------------------------------------

    def add_sample(self, target, actual, temp=None, states=None,
                   leader=None, torque=None) -> None:
        # Tracking strip is independent of chart history — update every call.
        if actual is not None or leader is not None:
            self.tracking.apply(
                actual=np.asarray(actual, dtype=float) if actual is not None else None,
                leader=np.asarray(leader, dtype=float) if leader is not None else None,
            )

        if target is None and actual is None:
            # Still accept a status-only update so tile colors stay live.
            if temp is not None or states is not None or torque is not None:
                self._update_status(temp, states, torque)
                self._canvas.update()
            return
        n = len(JOINT_NAMES)

        def _to_arr(x):
            if x is None:
                return np.full(n, np.nan)
            arr = np.asarray(x, dtype=float).ravel()
            if arr.size < n:
                out = np.full(n, np.nan); out[: arr.size] = arr; return out
            return arr[:n]

        now = time.monotonic()
        self._times.append(now)
        self._targets.append(_to_arr(target))
        self._actuals.append(_to_arr(actual))
        self._update_status(temp, states, torque)

        if self._accumulating:
            # Hard cap so painter stays responsive on multi-minute takes.
            while len(self._times) > self.MAX_SAMPLES:
                self._times.popleft()
                self._targets.popleft()
                self._actuals.popleft()
        else:
            self._trim_to_window()

        self._canvas.update()

    def _update_status(self, temp, states, torque=None) -> None:
        n = len(JOINT_NAMES)
        if temp is not None:
            t = np.asarray(temp, dtype=float).ravel()
            for i in range(min(n, t.size)):
                self._temps[i] = float(t[i])
        if states is not None:
            for i in range(min(n, len(states))):
                s = states[i]
                self._states[i] = str(s) if s is not None else "?"
        if torque is not None:
            tq = np.asarray(torque, dtype=float).ravel()
            for i in range(min(n, tq.size)):
                self._torques[i] = float(tq[i])
        # Drive critical-pulse timer based on latest temps.
        crit = any(
            (not math.isnan(c)) and c >= TEMP_CRITICAL for c in self._temps
        )
        if crit and not self._pulse_timer.isActive():
            self._pulse_timer.start()
        elif not crit and self._pulse_timer.isActive():
            self._pulse_timer.stop()
            self._pulse_phase = 0
        self._has_critical = crit

    def _tick_pulse(self) -> None:
        self._pulse_phase = (self._pulse_phase + 1) % 2
        self._canvas.update()

    def _trim_to_window(self) -> None:
        if not self._times:
            return
        cutoff = self._times[-1] - self.WINDOW_SEC
        while self._times and self._times[0] < cutoff:
            self._times.popleft()
            self._targets.popleft()
            self._actuals.popleft()

    def clear(self) -> None:
        self._times.clear()
        self._targets.clear()
        self._actuals.clear()
        self._yrange_cache = [(0.0, 0.0)] * len(JOINT_NAMES)
        self.tracking.apply(actual=None, leader=None)
        self._static_mode   = False
        self._cursor_frame  = 0
        self._episode_T     = 0
        self._episode_label = ""
        self.tracking.show()
        self._canvas.update()

    # ---- Static episode mode ----------------------------------------------

    def set_episode(self, qpos, actions, hz: float, label: str = "") -> None:
        """Pre-load an entire recorded episode for static display.

        qpos: [T, 7] actual positions per frame
        actions: [T, 7] target positions per frame (or None to mirror qpos)
        Cursor starts at frame 0 — drive it with set_cursor_frame().
        """
        self._times.clear()
        self._targets.clear()
        self._actuals.clear()
        self._yrange_cache = [(0.0, 0.0)] * len(JOINT_NAMES)

        n = len(JOINT_NAMES)
        T = int(qpos.shape[0]) if qpos is not None else 0
        if T == 0:
            self._static_mode = False
            self._canvas.update()
            return

        def _row(arr, i):
            if arr is None or i >= arr.shape[0]:
                return np.full(n, np.nan)
            r = np.asarray(arr[i], dtype=float).ravel()
            if r.size < n:
                out = np.full(n, np.nan); out[: r.size] = r; return out
            return r[:n]

        dt = 1.0 / max(float(hz), 1e-3)
        for i in range(T):
            self._times.append(i * dt)
            self._actuals.append(_row(qpos, i))
            self._targets.append(_row(actions if actions is not None else qpos, i))

        self._static_mode    = True
        self._cursor_frame   = 0
        self._episode_T      = T
        self._episode_hz     = float(hz)
        self._episode_label  = label
        # No live leader/temp/state during pure playback.
        self.tracking.hide()
        self._temps  = [float("nan")] * n
        self._states = ["?"] * n
        if self._pulse_timer.isActive():
            self._pulse_timer.stop()
        self._has_critical = False
        self._canvas.update()

    def set_cursor_frame(self, idx: int) -> None:
        if not self._static_mode:
            return
        idx = max(0, min(int(idx), max(0, self._episode_T - 1)))
        if idx == self._cursor_frame:
            return
        self._cursor_frame = idx
        self._canvas.update()

    # ---- Painting ---------------------------------------------------------

    def _paint_canvas(self, p: QPainter, rect) -> None:
        # Header line — title (mode-dependent).
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        header_h = 18
        if self._static_mode:
            dur = self._episode_T / max(self._episode_hz, 1e-3)
            t_cur = self._cursor_frame / max(self._episode_hz, 1e-3)
            label = f"{self._episode_label}  ·  " if self._episode_label else ""
            title = (f"EPISODE: {label}"
                     f"{t_cur:.1f}/{dur:.1f}s  ·  frame {self._cursor_frame}/"
                     f"{max(0, self._episode_T - 1)}  @  {self._episode_hz:.0f}Hz")
            p.setPen(QColor("#ffb84a"))
        elif self._accumulating:
            elapsed = (self._times[-1] - self._record_start) if (
                self._times and self._record_start is not None) else 0.0
            title = f"● RECORDING — {elapsed:.1f}s captured"
            p.setPen(self.REC_COLOR)
        else:
            title = self._idle_title
            p.setPen(QColor(COL_MUTED))
        p.drawText(rect.left(), rect.top() + 12, title)

        # Legend chips on the right.
        chip_y = rect.top() + 4
        chip_h = 12
        legend_x = rect.right() - 180
        for label, color in (("target", self.TARGET_COLOR),
                             ("actual", self.ACTUAL_COLOR)):
            p.fillRect(legend_x, chip_y + 2, 16, 8, color)
            p.setPen(QColor(COL_TEXT))
            p.setFont(QFont("Segoe UI", 8))
            p.drawText(legend_x + 22, chip_y + chip_h, label)
            legend_x += 80

        # Optional banner: HOT/CRITICAL summary across all joints.  Skipped
        # in static mode (no live telemetry).
        body_top = rect.top() + header_h + 4
        if not self._static_mode:
            banner_text, banner_bg = self._banner_state()
            if banner_text is not None:
                br_y = body_top
                p.fillRect(rect.left(), br_y, rect.width(), self.BANNER_H,
                           QColor(banner_bg))
                p.setPen(QColor("white"))
                p.setFont(QFont("Segoe UI", 9, QFont.Bold))
                p.drawText(rect.left() + 10, br_y + 15, banner_text)
                body_top += self.BANNER_H + 4

        body = rect.adjusted(0, body_top - rect.top(), 0, 0)
        rows = len(JOINT_NAMES)
        if rows == 0 or body.height() <= 0:
            return

        row_h = body.height() / rows

        for i, name in enumerate(JOINT_NAMES):
            row_top    = body.top() + i * row_h
            row_rect_y = int(row_top)
            row_rect_h = max(1, int(row_h) - 2)

            # Alternating row tint.
            if i % 2 == 0:
                p.fillRect(body.left(), row_rect_y,
                           body.width(), row_rect_h, QColor(COL_PANEL_ALT))

            # Heat wash — overlays the whole row when this joint is hot.
            # Skipped in static mode (no live temperature).
            if not self._static_mode:
                self._row_heat_wash(p, i, body.left(), row_rect_y,
                                    body.width(), row_rect_h)

            # Left gutter: full status tile in live mode, minimal joint
            # name in static mode (no live state/temp data).
            if self._static_mode:
                self._draw_static_name(p, name,
                                       body.left() + 4, row_rect_y + 2,
                                       self.NAME_W - 8, row_rect_h - 4)
            else:
                self._draw_status_tile(p, i, name,
                                       body.left() + 4, row_rect_y + 2,
                                       self.NAME_W - 8, row_rect_h - 4)

            # Plot rect (between name gutter and value gutter).
            plot_x  = body.left() + self.NAME_W
            plot_w  = body.width() - self.NAME_W - self.VAL_W
            plot_y  = row_rect_y + self.ROW_PAD
            plot_h  = row_rect_h - 2 * self.ROW_PAD
            if plot_w < 10 or plot_h < 4:
                continue
            self._draw_joint_strip(p, i, plot_x, plot_y, plot_w, plot_h)

            # Current values on the right.  In static mode these come from
            # the cursor frame; otherwise the latest streamed sample.
            if self._targets:
                if self._static_mode:
                    sample_idx = max(0, min(self._cursor_frame, len(self._targets) - 1))
                else:
                    sample_idx = -1
                tgt = self._targets[sample_idx][i]
                act = self._actuals[sample_idx][i]
                vx = body.right() - self.VAL_W + 4
                vy = row_rect_y + int(row_h * 0.6)
                p.setFont(QFont("Consolas", 8))
                p.setPen(self.TARGET_COLOR)
                p.drawText(vx, vy - 6,
                           "—" if math.isnan(tgt) else f"{tgt:+.3f}")
                p.setPen(self.ACTUAL_COLOR)
                p.drawText(vx, vy + 6,
                           "—" if math.isnan(act) else f"{act:+.3f}")

        # Static-mode playback cursor: a single vertical line spanning all
        # rows at the current frame's x position.
        if self._static_mode and self._episode_T > 0:
            plot_x = body.left() + self.NAME_W
            plot_w = body.width() - self.NAME_W - self.VAL_W
            if plot_w > 4:
                t_cur  = self._cursor_frame / max(self._episode_hz, 1e-3)
                dur    = self._episode_T / max(self._episode_hz, 1e-3)
                cx     = plot_x + int(plot_w * (t_cur / max(dur, 1e-6)))
                p.setPen(QPen(QColor("#ffffff"), 2))
                p.drawLine(cx, body.top(), cx, body.bottom())

    # ---- Status tile + heat wash + banner ---------------------------------

    @staticmethod
    def _state_color(state: str) -> QColor:
        s = (state or "?").lower()
        if s in ("running", "enabled"):
            return QColor(COL_OK)
        if s == "enabling":
            return QColor(COL_WARN)
        if s == "error":
            return QColor(COL_CRIT)
        if s == "disabled":
            return QColor("#555")
        return QColor(COL_MUTED)

    def _draw_status_tile(self, p: QPainter, joint_idx: int, name: str,
                          x: int, y: int, w: int, h: int) -> None:
        if w < 20 or h < 16:
            return
        state = self._states[joint_idx]
        temp  = self._temps[joint_idx]
        tq    = self._torques[joint_idx]
        jname = JOINT_NAMES[joint_idx]
        scol  = self._state_color(state)

        # Tile background — dark, with a colored left rail = state.
        p.fillRect(x, y, w, h, QColor("#1c1c1c"))
        p.fillRect(x, y, 4, h, scol)

        # State LED (filled circle) at top-left of the tile.
        led_d = 8
        led_x = x + 10
        led_y = y + 5
        led_color = scol
        # Error pulses; critical temp also pulses regardless of state.
        if (state or "").lower() == "error" or self._has_critical:
            if self._pulse_phase == 1:
                led_color = QColor("#ff5050")
        p.setBrush(led_color)
        p.setPen(QPen(QColor("#000"), 1))
        p.drawEllipse(led_x, led_y, led_d, led_d)

        # Joint name to the right of the LED.
        p.setPen(QColor(COL_TEXT))
        p.setFont(QFont("Segoe UI", 9, QFont.DemiBold))
        p.drawText(led_x + led_d + 6, led_y + led_d, name)

        # Pre-compute torque + temp readouts (both shown in either layout).
        has_temp = not math.isnan(temp)
        has_tq   = not math.isnan(tq)
        temp_col = QColor(_temp_colors(temp)[0]) if has_temp else QColor(COL_MUTED)
        tq_col   = QColor(_torque_color(jname, tq if has_tq else None))
        temp_txt = f"{temp:.0f}°" if has_temp else "—"
        tq_txt   = f"τ{tq:+.1f}" if has_tq else "τ—"

        # Compact layout for short rows: skip the bar entirely so the temp
        # graphic can't collide with the LED/name above.
        if h < 30:
            p.setFont(QFont("Consolas", 7, QFont.Bold))
            text_y = y + h - 4
            p.setPen(tq_col)
            p.drawText(x + 10, text_y, tq_txt)
            p.setPen(temp_col)
            p.drawText(x + w - 34, text_y, temp_txt)
            return

        # Full layout: torque numeric on the left, temp bar in the middle,
        # temp numeric on the right.  State is conveyed entirely by the LED
        # color (and pulse on error/critical-temp), so the old mnemonic
        # text is dropped to make room for the torque value.
        bar_h = 10
        bot_y = y + h - bar_h - 4
        p.setFont(QFont("Consolas", 7, QFont.Bold))
        p.setPen(tq_col)
        p.drawText(x + 10, bot_y + 9, tq_txt)

        # Thermal bar — full width is tinted by the temperature *zone* so
        # the whole bar lights up yellow/orange/red the moment a threshold
        # is crossed.  A vertical needle marks the exact reading.
        bar_x = x + 50
        bar_w = max(0, w - 50 - 38)   # leaves room for °C text
        bar_y = bot_y - 1
        if bar_w > 8:
            zone_color = QColor(_temp_colors(temp if has_temp else None)[0])
            # Background: faded zone color so the bar always reads as the
            # current threshold, with a darker base showing it's a gauge.
            bg = QColor(zone_color); bg.setAlpha(70)
            p.fillRect(bar_x, bar_y, bar_w, bar_h, QColor("#0e0e0e"))
            p.fillRect(bar_x, bar_y, bar_w, bar_h, bg)
            p.setPen(QPen(QColor("#444"), 1))
            p.drawRect(bar_x, bar_y, bar_w - 1, bar_h - 1)

            if has_temp:
                # Tick marks at WARN/HOT thresholds for context.
                p.setPen(QPen(QColor("#666"), 1))
                lo, hi = 25.0, TEMP_CRITICAL
                for thr in (TEMP_WARN, TEMP_HOT):
                    tx = bar_x + int((bar_w - 2) * (thr - lo) / (hi - lo)) + 1
                    p.drawLine(tx, bar_y + 2, tx, bar_y + bar_h - 3)

                # Solid fill in the zone color up to current temp.
                frac = max(0.0, min(1.0, (temp - lo) / (hi - lo)))
                fill_w = max(2, int((bar_w - 2) * frac))
                p.fillRect(bar_x + 1, bar_y + 1, fill_w, bar_h - 2, zone_color)

                # Needle on top so it reads above both fills.
                nx = bar_x + fill_w
                p.setPen(QPen(QColor("white"), 2))
                p.drawLine(nx, bar_y - 1, nx, bar_y + bar_h)

        # Numeric °C readout to the right of the bar.
        p.setFont(QFont("Consolas", 8, QFont.Bold))
        p.setPen(temp_col)
        p.drawText(x + w - 34, bot_y + 9, temp_txt)

    def _draw_static_name(self, p: QPainter, name: str,
                          x: int, y: int, w: int, h: int) -> None:
        if w < 20 or h < 14:
            return
        p.fillRect(x, y, w, h, QColor("#1c1c1c"))
        p.fillRect(x, y, 4, h, QColor("#ffb84a"))
        p.setPen(QColor(COL_TEXT))
        p.setFont(QFont("Segoe UI", 9, QFont.DemiBold))
        p.drawText(x + 12, y + h // 2 + 4, name)

    def _row_heat_wash(self, p: QPainter, joint_idx: int,
                       x: int, y: int, w: int, h: int) -> None:
        temp = self._temps[joint_idx]
        if math.isnan(temp) or temp < TEMP_HOT:
            return
        if temp >= TEMP_CRITICAL:
            base = QColor(COL_CRIT)
            base.setAlpha(60 if self._pulse_phase == 0 else 95)
        else:
            base = QColor(COL_HOT)
            base.setAlpha(40)
        p.fillRect(x, y, w, h, base)

    def _banner_state(self) -> tuple[Optional[str], str]:
        worst_idx = -1
        worst_val = -1.0
        for i, c in enumerate(self._temps):
            if not math.isnan(c) and c > worst_val:
                worst_val = c
                worst_idx = i
        # Error-state takes precedence over temp warnings.
        err_joints = [
            JOINT_NAMES[i] for i, s in enumerate(self._states)
            if (s or "").lower() == "error"
        ]
        if err_joints:
            return (
                f"⚠  ACTUATOR ERROR: {', '.join(err_joints)} — "
                f"check teleop status / re-enable",
                COL_CRIT,
            )
        if worst_idx >= 0 and worst_val >= TEMP_CRITICAL:
            return (
                f"⚠  CRITICAL: {JOINT_NAMES[worst_idx]} at "
                f"{worst_val:.0f}°C — stop and let it cool",
                COL_CRIT,
            )
        if worst_idx >= 0 and worst_val >= TEMP_HOT:
            return (
                f"⚠  HOT: {JOINT_NAMES[worst_idx]} at "
                f"{worst_val:.0f}°C — pause recording to cool",
                COL_HOT,
            )
        return (None, "")

    def _draw_joint_strip(self, p: QPainter, joint_idx: int,
                          x: int, y: int, w: int, h: int) -> None:
        # Background + frame.
        p.fillRect(x, y, w, h, QColor("#141414"))
        p.setPen(QPen(self.GRID_COLOR, 1))
        p.drawRect(x, y, w - 1, h - 1)

        if not self._times:
            p.setPen(QColor(COL_MUTED))
            p.setFont(QFont("Segoe UI", 8))
            p.drawText(x + 6, y + h // 2 + 3, "(no data)")
            return

        # Determine y-range for this joint over the visible window.
        tgt_col = np.array([t[joint_idx] for t in self._targets])
        act_col = np.array([a[joint_idx] for a in self._actuals])
        all_vals = np.concatenate([tgt_col[~np.isnan(tgt_col)],
                                   act_col[~np.isnan(act_col)]])
        if all_vals.size == 0:
            ymin, ymax = -0.5, 0.5
        else:
            ymin = float(all_vals.min())
            ymax = float(all_vals.max())
            if ymax - ymin < self.MIN_RANGE:
                mid = 0.5 * (ymax + ymin)
                ymin, ymax = mid - self.MIN_RANGE / 2, mid + self.MIN_RANGE / 2

        # Smooth y-range with previous frame's range so the axis doesn't jitter.
        prev = self._yrange_cache[joint_idx]
        if prev[0] != prev[1]:
            ymin = 0.7 * prev[0] + 0.3 * ymin
            ymax = 0.7 * prev[1] + 0.3 * ymax
        self._yrange_cache[joint_idx] = (ymin, ymax)

        # Mid-line.
        mid_y = y + int(h * 0.5)
        p.setPen(QPen(self.GRID_COLOR, 1, Qt.DotLine))
        p.drawLine(x + 1, mid_y, x + w - 2, mid_y)

        # Map (t, val) → (px, py). In static mode the x-axis spans the
        # entire episode; in accumulate mode it spans from record_start to
        # now; otherwise it's a rolling window ending at the latest sample.
        t_arr = np.fromiter(self._times, dtype=float)
        t_now = t_arr[-1]
        if self._static_mode and self._episode_T > 0:
            t_lo  = 0.0
            t_span = max(self._episode_T / max(self._episode_hz, 1e-3), 0.5)
        elif self._accumulating and self._record_start is not None:
            t_lo  = self._record_start
            t_span = max(t_now - t_lo, 0.5)
        else:
            t_lo  = t_now - self.WINDOW_SEC
            t_span = self.WINDOW_SEC

        def _sx(t: float) -> float:
            return x + (t - t_lo) / t_span * (w - 2) + 1

        def _sy(v: float) -> float:
            if math.isnan(v) or ymax == ymin:
                return y + h * 0.5
            return y + h - 2 - ((v - ymin) / (ymax - ymin)) * (h - 4)

        # Build paths.
        def _path_from(values: np.ndarray) -> QPainterPath:
            path = QPainterPath()
            started = False
            for ti, vi in zip(t_arr, values):
                if math.isnan(vi):
                    started = False
                    continue
                px, py = _sx(ti), _sy(float(vi))
                if not started:
                    path.moveTo(px, py); started = True
                else:
                    path.lineTo(px, py)
            return path

        # Draw target first so actual sits on top.
        p.setPen(QPen(self.TARGET_COLOR, 1.5))
        p.drawPath(_path_from(tgt_col))
        p.setPen(QPen(self.ACTUAL_COLOR, 1.5))
        p.drawPath(_path_from(act_col))


# ---------------------------------------------------------------------------
# Big animated RECORD button (P1)
# ---------------------------------------------------------------------------

class _RecordButton(QPushButton):

    def __init__(self) -> None:
        super().__init__("● RECORD   (R / F2)")
        self.setMinimumHeight(88)
        self.setCursor(Qt.PointingHandCursor)
        self._recording = False
        self._steps = 0
        self._pulse_phase = 0
        self._pulse_timer = QTimer(self)
        self._pulse_timer.setInterval(60)
        self._pulse_timer.timeout.connect(self._tick_pulse)
        self._apply(False, 0.0)

    def set_state(self, recording: bool, steps: int, dropped: int = 0,
                  enabled: bool = True) -> None:
        self._steps = steps
        if recording and not self._pulse_timer.isActive():
            self._pulse_timer.start()
        elif not recording and self._pulse_timer.isActive():
            self._pulse_timer.stop()
            self._pulse_phase = 0
            self._apply(False, 0.0)
        self._recording = recording
        # Only block input while idle — once recording, the user must always
        # be able to stop, even if state has shifted away from TRACKING.
        self.setEnabled(enabled or recording)
        if recording:
            dur = steps / 20.0
            drops = f"  ·  dropped {dropped}" if dropped else ""
            self.setText(f"●  REC   {dur:5.1f}s   ·   {steps} steps{drops}")
        elif not enabled:
            self.setText("● RECORD   (enable tracking — E)")
        else:
            self.setText("● RECORD   (R / F2)")

    def _tick_pulse(self) -> None:
        self._pulse_phase = (self._pulse_phase + 1) % 40
        t = (math.sin(self._pulse_phase / 40 * 2 * math.pi) + 1) * 0.5
        self._apply(True, t)

    def _apply(self, recording: bool, pulse_t: float) -> None:
        if recording:
            r = int(180 + pulse_t * 60)
            bg     = f"rgb({r},22,22)"
            border = f"rgb({min(255, r + 30)},120,120)"
            self.setStyleSheet(f"""
                QPushButton {{
                    background: {bg}; color: white;
                    font-weight: 900; font-size: 16pt;
                    border: 3px solid {border}; border-radius: 10px;
                    padding: 10px;
                    letter-spacing: 1px;
                }}
                QPushButton:hover {{ background: rgb(220,30,30); }}
            """)
        else:
            self.setStyleSheet(f"""
                QPushButton {{
                    background: #3a3a3a; color: #ddd;
                    font-weight: 900; font-size: 15pt;
                    border: 2px solid {COL_BORDER}; border-radius: 10px;
                    padding: 10px;
                    letter-spacing: 1px;
                }}
                QPushButton:hover {{ background: #4a4a4a; color: white;
                    border-color: #aa4040; }}
            """)


# ---------------------------------------------------------------------------
# Task tag combo: recent tags dropdown + editable (P2)
# ---------------------------------------------------------------------------

class _TaskTagCombo(QComboBox):

    def __init__(self, current_tag: str, output_dir: Path) -> None:
        super().__init__()
        self.setEditable(True)
        self.setMinimumHeight(32)
        self.setInsertPolicy(QComboBox.NoInsert)
        self.lineEdit().setPlaceholderText("e.g. pick_cube_red")
        self.setStyleSheet(f"""
            QComboBox {{
                background: {COL_PANEL_ALT}; color: {COL_TEXT};
                border: 1px solid {COL_BORDER}; border-radius: 4px;
                padding: 4px 8px; font-size: 11pt;
            }}
            QComboBox:focus {{ border: 1px solid {COL_ACCENT}; }}
            QComboBox QAbstractItemView {{
                background: {COL_PANEL}; color: {COL_TEXT};
                selection-background-color: {COL_ACCENT};
            }}
        """)
        recents = _scan_recent_tags(output_dir)
        for t in recents:
            self.addItem(t)
        if current_tag:
            self.setEditText(current_tag)

    def add_recent(self, tag: str) -> None:
        if not tag:
            return
        # Move to top / insert
        idx = self.findText(tag)
        if idx >= 0:
            self.removeItem(idx)
        self.insertItem(0, tag)


def _scan_recent_tags(output_dir: Path, limit: int = 25) -> list[str]:
    """Best-effort scan of episode_*.h5 for unique task_tag attrs."""
    try:
        import h5py
    except ImportError:
        return []
    if not output_dir.exists():
        return []
    seen: list[str] = []
    try:
        files = sorted(output_dir.glob("episode_*.h5"),
                       key=lambda p: p.stat().st_mtime, reverse=True)
    except OSError:
        return []
    for p in files[:80]:
        try:
            with h5py.File(p, "r") as f:
                tag = f.attrs.get("task_tag", "")
                if isinstance(tag, bytes):
                    tag = tag.decode("utf-8", "replace")
                tag = str(tag).strip()
                if tag and tag not in seen:
                    seen.append(tag)
                    if len(seen) >= limit:
                        break
        except Exception:
            continue
    return seen


# ---------------------------------------------------------------------------
# Episode list — scans output_dir for episode_*.h5, shows rich rows
# ---------------------------------------------------------------------------

class _EpisodeMeta:
    """Plain data carrier for one episode's metadata."""
    __slots__ = ("path", "name", "task_tag", "hz", "steps", "duration",
                 "collected_at", "error")

    def __init__(self, path: Path) -> None:
        self.path = path
        self.name = path.name
        self.task_tag = ""
        self.hz = 0.0
        self.steps = 0
        self.duration = 0.0
        self.collected_at = ""
        self.error: Optional[str] = None


class _EpisodeLoader(QThread):
    """Scans an output dir for episode_*.h5 and loads metadata in the
    background so the GUI stays responsive even with hundreds of saves."""

    loaded = Signal(object)     # emits one _EpisodeMeta at a time
    finished_scan = Signal(int) # total count when done

    def __init__(self, output_dir: Path) -> None:
        super().__init__()
        self._output_dir = output_dir
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        try:
            import h5py
        except ImportError:
            self.finished_scan.emit(0)
            return
        if not self._output_dir.exists():
            self.finished_scan.emit(0)
            return
        try:
            files = sorted(
                self._output_dir.glob("episode_*.hdf5"),
                key=lambda p: p.stat().st_mtime, reverse=True)
        except OSError:
            self.finished_scan.emit(0)
            return
        count = 0
        for p in files:
            if self._stop:
                break
            meta = _read_episode_meta(p, h5py)
            self.loaded.emit(meta)
            count += 1
        self.finished_scan.emit(count)


def _read_episode_meta(p: Path, h5py_mod) -> _EpisodeMeta:
    m = _EpisodeMeta(p)
    try:
        with h5py_mod.File(p, "r") as f:
            m.hz = float(f.attrs.get("hz", 20.0))
            tag = f.attrs.get("task_tag", "")
            if isinstance(tag, bytes):
                tag = tag.decode("utf-8", "replace")
            m.task_tag = str(tag).strip()
            ca = f.attrs.get("collected_at", "")
            if isinstance(ca, bytes):
                ca = ca.decode("utf-8", "replace")
            m.collected_at = str(ca).strip()
            if "observations/qpos" in f:
                m.steps = int(f["observations/qpos"].shape[0])
            elif "qpos" in f:
                m.steps = int(f["qpos"].shape[0])
            if m.hz > 0:
                m.duration = m.steps / m.hz
    except Exception as e:
        m.error = str(e)
    return m


class _EpisodeList(QFrame):
    """Scrollable list of saved episodes with context-menu actions."""

    episodeReplayRequested = Signal(object)   # Path
    episodeOpenInFolder    = Signal(object)   # Path
    episodeDeleted         = Signal(object)   # Path

    def __init__(self, output_dir: Path) -> None:
        super().__init__()
        self._output_dir = output_dir
        self.setStyleSheet(
            f"_EpisodeList {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(3)

        head_row = QHBoxLayout()
        head_row.setContentsMargins(0, 0, 0, 0)
        self._header = QLabel("EPISODES")
        self._header.setStyleSheet(
            f"color: {COL_MUTED}; font-weight: 700; font-size: 9pt; "
            f"letter-spacing: 2px;")
        head_row.addWidget(self._header)
        head_row.addStretch(1)
        self._count_lbl = QLabel("—")
        self._count_lbl.setStyleSheet(
            f"color: {COL_MUTED}; font-size: 9pt;")
        head_row.addWidget(self._count_lbl)
        v.addLayout(head_row)

        self._list = QListWidget()
        self._list.setFrameShape(QFrame.NoFrame)
        self._list.setSelectionMode(QListWidget.SingleSelection)
        self._list.setContextMenuPolicy(Qt.CustomContextMenu)
        self._list.customContextMenuRequested.connect(self._on_context_menu)
        self._list.itemDoubleClicked.connect(self._on_double_click)
        self._list.setStyleSheet(f"""
            QListWidget {{
                background: {COL_BG}; color: {COL_TEXT};
                border: 1px solid {COL_BORDER}; border-radius: 4px;
                font-size: 9pt; padding: 2px;
            }}
            QListWidget::item {{ padding: 4px 6px; border-bottom: 1px solid #222; }}
            QListWidget::item:selected {{
                background: {COL_ACCENT}; color: white;
            }}
            QListWidget::item:hover {{ background: #2a2a2a; }}
        """)
        v.addWidget(self._list, 1)

        self._loader: Optional[_EpisodeLoader] = None
        self.refresh()

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def refresh(self) -> None:
        if self._loader is not None and self._loader.isRunning():
            self._loader.stop()
            self._loader.wait(200)
        self._list.clear()
        self._count_lbl.setText("scanning…")
        self._loader = _EpisodeLoader(self._output_dir)
        self._loader.loaded.connect(self._on_meta_loaded)
        self._loader.finished_scan.connect(self._on_scan_done)
        self._loader.start()

    def add_or_update(self, path: Path) -> None:
        """Called when a new episode is saved at runtime."""
        try:
            import h5py
            meta = _read_episode_meta(path, h5py)
        except ImportError:
            meta = _EpisodeMeta(path)
        # Remove any existing row for the same path.
        for i in range(self._list.count()):
            it = self._list.item(i)
            if it.data(Qt.UserRole) and Path(it.data(Qt.UserRole)) == path:
                self._list.takeItem(i)
                break
        self._insert_item(meta, at_top=True)
        self._update_count()

    def selected_path(self) -> Optional[Path]:
        it = self._list.currentItem()
        if it is None:
            return None
        data = it.data(Qt.UserRole)
        return Path(data) if data else None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _on_meta_loaded(self, meta: _EpisodeMeta) -> None:
        self._insert_item(meta, at_top=False)
        self._update_count()

    def _on_scan_done(self, count: int) -> None:
        self._update_count()

    def _update_count(self) -> None:
        n = self._list.count()
        self._count_lbl.setText(f"{n} saved" if n else "none yet")

    def _insert_item(self, meta: _EpisodeMeta, at_top: bool) -> None:
        if meta.error:
            text = f"{meta.name}\n  ⚠ {meta.error}"
            color = COL_CRIT
        else:
            tag = meta.task_tag or "—"
            when = _human_time(meta.collected_at) if meta.collected_at else ""
            line1 = f"{meta.name}   ·   {tag}"
            line2 = f"  {meta.duration:4.1f}s  ·  {meta.steps} steps"
            if when:
                line2 += f"   ·   {when}"
            text = f"{line1}\n{line2}"
            color = COL_TEXT
        item = QListWidgetItem(text)
        item.setData(Qt.UserRole, str(meta.path))
        item.setForeground(QColor(color))
        if at_top:
            self._list.insertItem(0, item)
        else:
            self._list.addItem(item)

    def _on_double_click(self, item: QListWidgetItem) -> None:
        data = item.data(Qt.UserRole)
        if data:
            self.episodeReplayRequested.emit(Path(data))

    def _on_context_menu(self, pos) -> None:
        item = self._list.itemAt(pos)
        if item is None:
            return
        data = item.data(Qt.UserRole)
        if not data:
            return
        p = Path(data)
        menu = QMenu(self)
        menu.setStyleSheet(f"""
            QMenu {{ background: {COL_PANEL}; color: {COL_TEXT};
                     border: 1px solid {COL_BORDER}; }}
            QMenu::item {{ padding: 6px 20px; }}
            QMenu::item:selected {{ background: {COL_ACCENT}; color: white; }}
        """)
        act_replay = QAction("▶  Replay", self)
        act_open   = QAction("📁 Open folder", self)
        act_delete = QAction("🗑  Delete…", self)
        act_replay.triggered.connect(lambda: self.episodeReplayRequested.emit(p))
        act_open.triggered.connect(lambda: self.episodeOpenInFolder.emit(p))
        act_delete.triggered.connect(lambda: self._confirm_delete(p))
        menu.addAction(act_replay)
        menu.addAction(act_open)
        menu.addSeparator()
        menu.addAction(act_delete)
        menu.exec(self._list.mapToGlobal(pos))

    def _confirm_delete(self, p: Path) -> None:
        reply = QMessageBox.question(
            self, "Delete episode",
            f"Delete {p.name}?\n\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        try:
            p.unlink()
        except OSError as e:
            QMessageBox.warning(self, "Delete failed", str(e))
            return
        for i in range(self._list.count()):
            it = self._list.item(i)
            if it.data(Qt.UserRole) == str(p):
                self._list.takeItem(i)
                break
        self._update_count()
        self.episodeDeleted.emit(p)


def _human_time(iso_ts: str) -> str:
    """Turn 'YYYY-MM-DDTHH:MM:SS' into a compact display string."""
    if not iso_ts:
        return ""
    # Try to find the date/time boundary.
    if "T" in iso_ts:
        date_part, time_part = iso_ts.split("T", 1)
        time_part = time_part[:5]  # drop seconds
        try:
            today = time.strftime("%Y-%m-%d")
            if date_part == today:
                return time_part
        except Exception:
            pass
        # fall through: short form "MM-DD HH:MM"
        try:
            return f"{date_part[5:]} {time_part}"
        except IndexError:
            return iso_ts
    return iso_ts


# ---------------------------------------------------------------------------
# Save toast (P5) — floating notification with Undo
# ---------------------------------------------------------------------------

class _SaveToast(QFrame):

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setObjectName("saveToast")
        self.setStyleSheet(f"""
            #saveToast {{
                background: #1e4a1e; border: 2px solid #4abf4a;
                border-radius: 10px;
            }}
            QLabel {{ color: white; font-size: 11pt; background: transparent;
                padding: 0 6px; }}
            QPushButton {{
                background: #666; color: white; font-weight: 700;
                border: 1px solid #888; border-radius: 4px;
                padding: 4px 14px;
            }}
            QPushButton:hover {{ background: #888; }}
        """)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(10)
        self.lbl = QLabel("✓ Saved")
        self.btn_undo = QPushButton("Undo")
        self.btn_undo.setCursor(Qt.PointingHandCursor)
        self.btn_undo.clicked.connect(self._on_undo)
        lay.addWidget(self.lbl, 1)
        lay.addWidget(self.btn_undo)
        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self.hide)
        self._path: Optional[Path] = None
        self._on_undo_cb: Optional[Callable[[Path], None]] = None
        self.setFixedHeight(50)
        self.hide()

    def show_save(self, path: Path, on_undo: Callable[[Path], None]) -> None:
        self._path = path
        self._on_undo_cb = on_undo
        self.lbl.setText(f"✓ Saved  {path.name}")
        self.adjustSize()
        self._reposition()
        self.show()
        self.raise_()
        self._hide_timer.start(8000)

    def _on_undo(self) -> None:
        if self._on_undo_cb and self._path:
            try:
                self._on_undo_cb(self._path)
                self.lbl.setText(f"Removed {self._path.name}")
                self.setStyleSheet(self.styleSheet().replace("#1e4a1e", "#4a1e1e")
                                                    .replace("#4abf4a", "#bf4a4a"))
                self.btn_undo.hide()
                self._hide_timer.start(2500)
            except OSError as e:
                QMessageBox.warning(self, "Undo failed", str(e))
                self.hide()

    def _reposition(self) -> None:
        if self.parent() is None:
            return
        pw = self.parent().width()
        ph = self.parent().height()
        self.adjustSize()
        w = self.width()
        h = self.height()
        self.move(max(12, pw - w - 24), max(12, ph - h - 44))


# ---------------------------------------------------------------------------
# Action toast — top-center confirmation for Z / M / P (zero / mirror / save)
# ---------------------------------------------------------------------------

class _ActionToast(QFrame):
    """Big floating banner at top-center for momentary action confirmations.

    Used for [Z] zero leader, [M] mirror, [P] save ready pose.  Distinct
    from the bottom-right save toast: this one is bigger, accent-colored,
    and dismisses faster (the action is instantaneous so the operator just
    needs a moment of "yes, that worked").
    """

    # Per-action color + icon, keyed by the prefix written into zero_msg
    # in collect_demo.py ("[Z] zeroed — saved", "[M] mirrored — saved", ...).
    _STYLES = {
        "[Z]": ("#2a4a8a", "#6aa0ff", "◎"),  # blue — zero
        "[M]": ("#2a6ab0", "#6ac0ff", "⇄"),  # cyan — mirror
        "[P]": ("#1e6a2a", "#6abf6a", "✓"),  # green — save
    }
    _DEFAULT_STYLE = ("#2a2a2a", COL_ACCENT, "ℹ")

    def __init__(self, parent: QWidget) -> None:
        super().__init__(parent)
        self.setObjectName("actionToast")
        lay = QHBoxLayout(self)
        lay.setContentsMargins(20, 10, 20, 10)
        lay.setSpacing(14)
        self.lbl_icon = QLabel("")
        self.lbl_icon.setStyleSheet(
            "color: white; font-size: 24pt; background: transparent;")
        self.lbl_text = QLabel("")
        self.lbl_text.setStyleSheet(
            "color: white; font-size: 14pt; font-weight: 700; "
            "background: transparent; letter-spacing: 1px;")
        lay.addWidget(self.lbl_icon)
        lay.addWidget(self.lbl_text)
        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.timeout.connect(self.hide)
        self._last_msg: Optional[str] = None
        self.hide()
        self._apply_style(*self._DEFAULT_STYLE)

    def show_msg(self, msg: str) -> None:
        bg, border, icon = self._DEFAULT_STYLE
        for prefix, style in self._STYLES.items():
            if msg.startswith(prefix):
                bg, border, icon = style
                msg = msg[len(prefix):].strip(" —-:")
                break
        self._apply_style(bg, border, icon)
        self.lbl_text.setText(msg.strip())
        self.adjustSize()
        self._reposition()
        self.show()
        self.raise_()
        self._hide_timer.start(1800)

    def _apply_style(self, bg: str, border: str, icon: str) -> None:
        self.setStyleSheet(f"""
            #actionToast {{
                background: {bg}; border: 2px solid {border};
                border-radius: 14px;
            }}
        """)
        self.lbl_icon.setText(icon)

    def _reposition(self) -> None:
        if self.parent() is None:
            return
        pw = self.parent().width()
        self.adjustSize()
        w = self.width()
        # Top-center, just below the status bar + health strip area.
        self.move(max(12, (pw - w) // 2), 100)


# ---------------------------------------------------------------------------
# Event log — scrolling, timestamped record of Z/M/P/save/delete events
# ---------------------------------------------------------------------------

class _EventLog(QFrame):
    """Compact scrolling log shown in the right column.

    Mirrors the action-toast events (Z / M / P) and save/delete activity
    into a persistent, timestamped list so the operator has a running
    record of what happened this session without having to rely on toasts
    that disappear.
    """

    _MAX_ENTRIES = 60

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_EventLog {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(3)

        header = QLabel("EVENT LOG")
        header.setStyleSheet(
            f"color: {COL_MUTED}; font-weight: 700; font-size: 9pt; "
            f"letter-spacing: 2px;")
        v.addWidget(header)

        self._list = QListWidget()
        self._list.setFrameShape(QFrame.NoFrame)
        self._list.setSelectionMode(QListWidget.NoSelection)
        self._list.setFocusPolicy(Qt.NoFocus)
        self._list.setStyleSheet(f"""
            QListWidget {{
                background: {COL_BG}; color: {COL_TEXT};
                border: 1px solid {COL_BORDER}; border-radius: 4px;
                font-family: Consolas, monospace; font-size: 9pt;
                padding: 2px;
            }}
            QListWidget::item {{ padding: 1px 2px; }}
        """)
        v.addWidget(self._list, 1)

    def add_event(self, icon: str, text: str, color: str = COL_TEXT) -> None:
        ts = time.strftime("%H:%M:%S")
        item = QListWidgetItem(f"{ts}  {icon}  {text}")
        item.setForeground(QColor(color))
        self._list.addItem(item)
        # Trim head if needed.
        while self._list.count() > self._MAX_ENTRIES:
            self._list.takeItem(0)
        self._list.scrollToBottom()


# ---------------------------------------------------------------------------
# Startup health check strip (P4)
# ---------------------------------------------------------------------------

class _HealthStrip(QFrame):

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_HealthStrip {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        self._dismissed = False
        self._all_ok_time: Optional[float] = None

        row = QHBoxLayout(self)
        row.setContentsMargins(10, 6, 10, 6)
        row.setSpacing(14)

        def _chk(label: str) -> _Pill:
            p = _Pill(label, bg="#333", fg=COL_MUTED, min_w=120)
            row.addWidget(p)
            return p

        self._title = QLabel("Startup checks:")
        self._title.setStyleSheet(f"color: {COL_MUTED}; font-weight: 700;")
        row.addWidget(self._title)

        self._c_robot  = _chk("robot  ?")
        self._c_cams   = _chk("cameras ?")
        self._c_leader = _chk("leader  ?")
        self._c_estop  = _chk("e-stop  ?")
        row.addStretch(1)

        self._btn_dismiss = QPushButton("✕")
        self._btn_dismiss.setFixedSize(24, 24)
        self._btn_dismiss.setCursor(Qt.PointingHandCursor)
        self._btn_dismiss.setStyleSheet(
            f"QPushButton {{ background: transparent; color: {COL_MUTED}; "
            f"border: none; font-size: 14pt; }} "
            f"QPushButton:hover {{ color: white; }}")
        self._btn_dismiss.clicked.connect(self._dismiss)
        row.addWidget(self._btn_dismiss)

    def apply(self, snap: dict) -> None:
        if self._dismissed:
            return

        now = time.time()

        # Robot telemetry.
        robot_ok  = bool(snap.get("robot_ok", False))
        telem_age = float(snap.get("telem_age", 999.0))
        robot_good = robot_ok and telem_age < 1.5
        self._c_robot.set_pill(
            f"robot {'OK' if robot_good else 'WAIT'}",
            COL_OK if robot_good else "#333",
            "white" if robot_good else COL_MUTED,
        )

        # Cameras.
        lage = float(snap.get("cam_left_age",  999.0))
        rage = float(snap.get("cam_right_age", 999.0))
        cams_good = lage < 0.5 and rage < 0.5
        self._c_cams.set_pill(
            f"cams  {'OK' if cams_good else 'WAIT'}",
            COL_OK if cams_good else "#333",
            "white" if cams_good else COL_MUTED,
        )

        # Leader arm — hot-pluggable, so just WAIT until present, then OK.
        leader_good = bool(snap.get("leader_connected", False))
        self._c_leader.set_pill(
            f"leader {'OK' if leader_good else 'WAIT'}",
            COL_OK if leader_good else "#333",
            "white" if leader_good else COL_MUTED,
        )

        # E-stop: shown as "SAFE" when not tripped and we've seen telemetry.
        estop_active = bool(snap.get("estop_active", False))
        estop_good   = robot_good and not estop_active
        self._c_estop.set_pill(
            "e-stop SAFE" if estop_good else ("e-stop ACTIVE" if estop_active else "e-stop ?"),
            COL_OK if estop_good else (COL_CRIT if estop_active else "#333"),
            "white" if (estop_good or estop_active) else COL_MUTED,
        )

        all_ok = robot_good and cams_good and leader_good and estop_good
        if all_ok and self._all_ok_time is None:
            self._all_ok_time = now
        if all_ok and self._all_ok_time and now - self._all_ok_time > 2.5:
            self._dismiss()

    def _dismiss(self) -> None:
        self._dismissed = True
        self.hide()


# ---------------------------------------------------------------------------
# Shortcut legend footer (P3)
# ---------------------------------------------------------------------------

class _ShortcutLegend(QLabel):

    def __init__(self) -> None:
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setTextFormat(Qt.RichText)
        self.setStyleSheet(
            f"background: {COL_PANEL}; color: {COL_MUTED}; "
            f"border-top: 1px solid {COL_BORDER}; "
            f"font-size: 10pt; padding: 6px;")
        self._render("ready")

    def set_state(self, state: str, recording: bool) -> None:
        self._render(state, recording)

    def _render(self, state: str, recording: bool = False) -> None:
        def kb(k, d):
            return (f"<span style='color:{COL_ACCENT}; font-weight:700; "
                    f"background:#222; padding:1px 6px; border-radius:3px;'>{k}</span>"
                    f" <span style='color:#aaa;'>{d}</span>")
        parts: list[str] = []
        parts.append(kb("E", "enable"))
        parts.append(kb("I", "idle"))
        parts.append(kb("H", "hold"))
        parts.append(kb("F", "wheels"))
        parts.append(kb("X", "shutdown"))
        if state == "shutdown":
            parts.append(kb("Esc", "cancel"))
        parts.append(kb("R / F2", "stop rec" if recording else "record"))
        parts.append(kb("Z", "zero"))
        parts.append(kb("M", "mirror"))
        parts.append(kb("P", "save ready"))
        parts.append(kb("WASD", "drive"))
        parts.append(kb("F11", "fullscreen"))
        parts.append(kb("Q", "quit"))
        self.setText("   ·   ".join(parts))


# ---------------------------------------------------------------------------
# Button group helper
# ---------------------------------------------------------------------------

def _group_title(txt: str) -> QLabel:
    lbl = QLabel(txt)
    lbl.setStyleSheet(
        f"color: {COL_MUTED}; font-weight: 700; font-size: 9pt; "
        f"letter-spacing: 2px; padding-top: 4px;")
    return lbl


def _make_button(label: str, tooltip: str, accent: str = "") -> QPushButton:
    b = QPushButton(label)
    b.setMinimumHeight(38)
    b.setCursor(Qt.PointingHandCursor)
    b.setToolTip(tooltip)
    accent_border = accent or COL_BORDER
    accent_hover  = accent or "#555"
    b.setStyleSheet(f"""
        QPushButton {{
            background: {COL_PANEL_ALT}; color: {COL_TEXT};
            border: 1px solid {accent_border}; border-radius: 6px;
            font-weight: 600; font-size: 10pt; padding: 6px 8px;
        }}
        QPushButton:hover {{ background: #333; border-color: {accent_hover}; }}
        QPushButton:pressed {{ background: #2a2a2a; }}
    """)
    return b


# ---------------------------------------------------------------------------
# Mode switcher — large pill buttons at top: Collect | Replay | (Inference)
# ---------------------------------------------------------------------------

MODE_COLLECT   = "collect"
MODE_REPLAY    = "replay"
MODE_INFERENCE = "inference"


class _ModeSwitch(QFrame):

    modeChanged = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_ModeSwitch {{ background: transparent; border: none; }}")
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 4, 0, 4)
        row.setSpacing(8)

        self._btns: dict[str, QPushButton] = {}
        specs = [
            (MODE_COLLECT,   "◉  COLLECT",    "Record teleop demonstrations",    True,  COL_ACCENT),
            (MODE_REPLAY,    "▶  REPLAY",     "Play back saved episodes",        True,  "#ffb84a"),
            (MODE_INFERENCE, "✦  INFERENCE",  "Run policy (coming soon)",        False, "#888"),
        ]
        for key, label, tip, enabled, accent in specs:
            b = QPushButton(label)
            b.setCheckable(True)
            b.setEnabled(enabled)
            b.setCursor(Qt.PointingHandCursor if enabled else Qt.ArrowCursor)
            b.setToolTip(tip)
            b.setMinimumHeight(40)
            b.setStyleSheet(f"""
                QPushButton {{
                    background: {COL_PANEL}; color: {COL_MUTED};
                    border: 2px solid {COL_BORDER}; border-radius: 10px;
                    padding: 6px 24px; font-weight: 700; font-size: 11pt;
                    letter-spacing: 2px;
                }}
                QPushButton:hover:!checked:enabled {{
                    background: #2a2a2a; color: {COL_TEXT};
                    border-color: {accent};
                }}
                QPushButton:checked {{
                    background: {accent}; color: white;
                    border-color: {accent};
                }}
                QPushButton:disabled {{
                    color: #555; border-color: #333;
                }}
            """)
            b.clicked.connect(lambda _=False, k=key: self._select(k))
            self._btns[key] = b
            row.addWidget(b)
        row.addStretch(1)

        self._current = MODE_COLLECT
        self._btns[MODE_COLLECT].setChecked(True)

    def _select(self, key: str) -> None:
        if key == self._current:
            self._btns[key].setChecked(True)
            return
        if not self._btns[key].isEnabled():
            self._btns[self._current].setChecked(True)
            return
        self._btns[self._current].setChecked(False)
        self._btns[key].setChecked(True)
        self._current = key
        self.modeChanged.emit(key)

    def set_mode(self, key: str) -> None:
        self._select(key)

    def current(self) -> str:
        return self._current


# ---------------------------------------------------------------------------
# Playback engine — loads episode HDF5, ticks frames on a QTimer
# ---------------------------------------------------------------------------

class _PlaybackEngine(QWidget):
    """Owns the loaded episode and a QTimer that advances frames
    wall-clock-paced.  Emits frameChanged(idx) every step.

    Subclassing QWidget (not QObject) just so we can parent a QTimer
    to it cheaply without an extra thread boundary."""

    frameChanged = Signal(int)
    episodeLoaded = Signal(object)  # emits dict or None on failure
    playStateChanged = Signal(bool)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.hide()
        self._ep: Optional[dict] = None
        self._path: Optional[Path] = None
        self._idx = 0
        self._speed = 1.0
        self._playing = False
        self._timer = QTimer(self)
        self._timer.setInterval(16)  # ~60 Hz ticker; actual frame advance paced by hz*speed
        self._timer.timeout.connect(self._tick)
        self._last_tick_wall: float = 0.0
        self._accum: float = 0.0

    # --- API -----------------------------------------------------------

    def load(self, path: Path) -> bool:
        try:
            import h5py
        except ImportError:
            return False
        try:
            with h5py.File(path, "r") as f:
                qpos    = f["observations/qpos"][:]
                actions = f["actions"][:] if "actions" in f else None
                left    = (f["observations/images/left"][:]
                           if "observations/images/left" in f else None)
                right   = (f["observations/images/right"][:]
                           if "observations/images/right" in f else None)
                hz      = float(f.attrs.get("hz", 20.0))
                tag     = f.attrs.get("task_tag", "")
                if isinstance(tag, bytes):
                    tag = tag.decode("utf-8", "replace")
                ca = f.attrs.get("collected_at", "")
                if isinstance(ca, bytes):
                    ca = ca.decode("utf-8", "replace")
        except Exception as e:
            self._ep = None
            self.episodeLoaded.emit({"error": str(e), "path": path})
            return False

        self._ep = {
            "qpos": qpos, "actions": actions,
            "left": left, "right": right,
            "hz": hz, "T": int(qpos.shape[0]),
            "task_tag": str(tag).strip(),
            "collected_at": str(ca).strip(),
        }
        self._path = path
        self._idx = 0
        self._accum = 0.0
        self.pause()
        self.episodeLoaded.emit(self._ep)
        self.frameChanged.emit(0)
        return True

    def episode(self) -> Optional[dict]:
        return self._ep

    def path(self) -> Optional[Path]:
        return self._path

    def frame(self) -> int:
        return self._idx

    def total_frames(self) -> int:
        return int(self._ep["T"]) if self._ep else 0

    def play(self) -> None:
        if self._ep is None: return
        if self._idx >= self._ep["T"] - 1:
            self._idx = 0
            self.frameChanged.emit(0)
        self._playing = True
        self._last_tick_wall = time.monotonic()
        self._accum = 0.0
        self._timer.start()
        self.playStateChanged.emit(True)

    def pause(self) -> None:
        if self._playing:
            self._playing = False
            self._timer.stop()
            self.playStateChanged.emit(False)
        else:
            self._timer.stop()

    def toggle_play(self) -> None:
        if self._playing: self.pause()
        else:             self.play()

    def is_playing(self) -> bool:
        return self._playing

    def seek(self, idx: int) -> None:
        if self._ep is None: return
        self._idx = max(0, min(int(idx), self._ep["T"] - 1))
        self._accum = 0.0
        self.frameChanged.emit(self._idx)

    def step(self, delta: int) -> None:
        if self._ep is None: return
        self.seek(self._idx + delta)

    def set_speed(self, speed: float) -> None:
        self._speed = max(0.05, float(speed))

    def speed(self) -> float:
        return self._speed

    # --- Tick ----------------------------------------------------------

    def _tick(self) -> None:
        if not self._playing or self._ep is None:
            return
        now = time.monotonic()
        dt = now - self._last_tick_wall
        self._last_tick_wall = now
        self._accum += dt * self._speed
        frame_period = 1.0 / max(1.0, self._ep["hz"])
        advanced = False
        while self._accum >= frame_period and self._idx < self._ep["T"] - 1:
            self._idx += 1
            self._accum -= frame_period
            advanced = True
        if advanced:
            self.frameChanged.emit(self._idx)
        if self._idx >= self._ep["T"] - 1:
            self.pause()


# ---------------------------------------------------------------------------
# Playback cameras — two QLabels that display RGB numpy frames
# ---------------------------------------------------------------------------

class _PlaybackCameraPair(QFrame):

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_PlaybackCameraPair {{ background: #0a0a0a; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        row = QHBoxLayout(self)
        row.setContentsMargins(6, 6, 6, 6)
        row.setSpacing(6)
        self._left  = self._make_cam("Left")
        self._right = self._make_cam("Right")
        row.addWidget(self._left_wrap, 1)
        row.addWidget(self._right_wrap, 1)

    def _make_cam(self, label: str) -> QLabel:
        wrap = QFrame()
        wrap.setStyleSheet(
            f"QFrame {{ background: black; border: 1px solid {COL_BORDER}; "
            f"border-radius: 4px; }}")
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0); v.setSpacing(0)
        hdr = QLabel(f"  {label}")
        hdr.setStyleSheet(
            f"color: {COL_MUTED}; font-size: 9pt; font-weight: 700; "
            f"padding: 3px; background: rgba(0,0,0,0.5);")
        v.addWidget(hdr)
        cam = QLabel()
        cam.setAlignment(Qt.AlignCenter)
        # QSizePolicy.Ignored so the label's sizeHint (which tracks the
        # pixmap size) doesn't feed back into the splitter and push the
        # whole replay column wider every frame.
        cam.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        cam.setMinimumSize(1, 1)
        cam.setStyleSheet("background: black; color: #444;")
        cam.setText("(no frame loaded)")
        v.addWidget(cam, 1)
        if label == "Left":
            self._left_wrap = wrap
        else:
            self._right_wrap = wrap
        return cam

    def set_frames(self, left: Optional[np.ndarray],
                   right: Optional[np.ndarray]) -> None:
        self._render_to(self._left, left)
        self._render_to(self._right, right)

    def clear(self) -> None:
        self._left.clear();  self._left.setText("(no frame loaded)")
        self._right.clear(); self._right.setText("(no frame loaded)")

    def _render_to(self, lbl: QLabel, frame: Optional[np.ndarray]) -> None:
        if frame is None:
            return
        # Expect uint8 RGB H×W×3 (collect_demo writes RGB).
        h, w = frame.shape[:2]
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)
        img = QImage(frame.tobytes(), w, h, 3 * w, QImage.Format_RGB888)
        pm = QPixmap.fromImage(img)
        target = lbl.size()
        if target.width() > 4 and target.height() > 4:
            pm = pm.scaled(target, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        lbl.setPixmap(pm)


class _LiveCameraPair(QFrame):
    """Native Qt live camera preview — two QLabels fed raw JPEG bytes.

    Replaces the embedded Rerun WASM viewer in GUI mode.  Decoding happens
    in Qt's native C++ JPEG decoder, so there's no gRPC stream, no
    WASM viewer, and no Chromium compositor to back up on a slow CPU.
    Frames are timestamped — older arrivals are dropped, so a paint that
    falls behind by one tick simply skips ahead instead of accumulating.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_LiveCameraPair {{ background: #0a0a0a; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        row = QHBoxLayout(self)
        row.setContentsMargins(6, 6, 6, 6)
        row.setSpacing(6)
        self._left  = self._make_cam("Left")
        self._right = self._make_cam("Right")
        row.addWidget(self._left_wrap,  1)
        row.addWidget(self._right_wrap, 1)
        self._left_ts:  float = 0.0
        self._right_ts: float = 0.0

    def _make_cam(self, label: str) -> QLabel:
        wrap = QFrame()
        wrap.setStyleSheet(
            f"QFrame {{ background: black; border: 1px solid {COL_BORDER}; "
            f"border-radius: 4px; }}")
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0); v.setSpacing(0)
        hdr = QLabel(f"  {label}")
        hdr.setStyleSheet(
            f"color: {COL_MUTED}; font-size: 9pt; font-weight: 700; "
            f"padding: 3px; background: rgba(0,0,0,0.5);")
        v.addWidget(hdr)
        cam = QLabel()
        cam.setAlignment(Qt.AlignCenter)
        cam.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        cam.setMinimumSize(1, 1)
        cam.setStyleSheet("background: black; color: #444;")
        cam.setText("(waiting for camera…)")
        v.addWidget(cam, 1)
        if label == "Left":
            self._left_wrap = wrap
        else:
            self._right_wrap = wrap
        return cam

    def set_frames(self,
                   left_jpeg:  Optional[bytes], left_ts:  float,
                   right_jpeg: Optional[bytes], right_ts: float) -> None:
        if left_jpeg is not None and left_ts > self._left_ts:
            self._left_ts = left_ts
            # Left camera is mounted upside-down; mirror vertically.
            self._render_jpeg(self._left, left_jpeg, flip_v=True)
        if right_jpeg is not None and right_ts > self._right_ts:
            self._right_ts = right_ts
            self._render_jpeg(self._right, right_jpeg, flip_v=False)

    def _render_jpeg(self, lbl: QLabel, jpeg: bytes, flip_v: bool) -> None:
        img = QImage()
        if not img.loadFromData(jpeg, "JPEG"):
            return
        if flip_v:
            img = img.mirrored(False, True)
        pm = QPixmap.fromImage(img)
        target = lbl.size()
        if target.width() > 4 and target.height() > 4:
            pm = pm.scaled(target, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        lbl.setPixmap(pm)


# ---------------------------------------------------------------------------
# Replay transport — play/pause/scrub/speed/step controls
# ---------------------------------------------------------------------------

class _ReplayTransport(QFrame):

    playToggle = Signal()
    seekTo     = Signal(int)
    stepBy     = Signal(int)
    speedSet   = Signal(float)
    openFile   = Signal()   # "Open file…" button

    # Live replay (on-robot playback) signals.
    liveToggled          = Signal(bool)   # True = enter live, False = exit
    liveCommand          = Signal(str)    # "arm" / "play" / "pause" / "restart" / "stop"
    liveLoopChanged      = Signal(bool)
    liveGotoStartChanged = Signal(bool)
    liveVelFfChanged     = Signal(bool)
    liveMaxDeltaChanged  = Signal(float)

    _SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0]

    def __init__(self) -> None:
        super().__init__()
        self.setStyleSheet(
            f"QFrame {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 8px; }}")
        v = QVBoxLayout(self)
        v.setContentsMargins(14, 14, 14, 14)
        v.setSpacing(10)

        # ------- LIVE ON ROBOT ----------------------------------------
        self._live_on        = False
        self._live_ep_loaded = False
        self._live_box = QFrame()
        self._live_box.setObjectName("liveBox")
        self._apply_live_box_style()
        lv = QVBoxLayout(self._live_box)
        lv.setContentsMargins(12, 10, 12, 12)
        lv.setSpacing(8)

        hdr = QHBoxLayout(); hdr.setSpacing(8)
        self._live_title = QLabel("◉  LIVE ON ROBOT")
        self._live_title.setStyleSheet(
            f"color: {COL_MUTED}; font-weight: 900; font-size: 10pt; "
            f"letter-spacing: 1px;")
        hdr.addWidget(self._live_title)
        hdr.addStretch(1)
        self.lbl_live_phase = _Pill("OFF", "#2a2a2a", COL_MUTED, min_w=78)
        hdr.addWidget(self.lbl_live_phase)
        lv.addLayout(hdr)

        # Warning banner — hidden when live is on (replaced by progress).
        self.lbl_live_warn = QLabel(
            "⚠  Turning this on lets the episode drive the physical arm. "
            "Clear the workspace first.")
        self.lbl_live_warn.setWordWrap(True)
        self.lbl_live_warn.setStyleSheet(
            f"QLabel {{ color: #e0c070; background: rgba(184,135,16,0.10); "
            f"border: 1px solid rgba(184,135,16,0.35); border-radius: 4px; "
            f"padding: 6px 8px; font-size: 9pt; }}")
        lv.addWidget(self.lbl_live_warn)

        self.btn_live = QPushButton("▶  GO LIVE")
        self.btn_live.setCheckable(True)
        self.btn_live.setMinimumHeight(40)
        self.btn_live.setCursor(Qt.PointingHandCursor)
        self.btn_live.setEnabled(False)
        self._apply_live_button_style()
        self.btn_live.clicked.connect(self._on_live_clicked)
        lv.addWidget(self.btn_live)

        # Live transport grid — disabled until live mode is on.
        lrow = QGridLayout(); lrow.setSpacing(6)
        def _lb(label, tip, cmd, accent=""):
            b = _make_button(label, tip, accent)
            b.setProperty("_live_cmd", cmd)
            b.clicked.connect(
                lambda _=False, bb=b: self.liveCommand.emit(bb.property("_live_cmd")))
            return b
        self.btn_live_arm     = _lb("▶ ARM+PLAY", "Enable + ramp to start + play", "arm", COL_OK)
        self.btn_live_pause   = _lb("❚❚ PAUSE",   "Pause, hold position",            "pause", COL_WARN)
        self.btn_live_restart = _lb("⏮ RESTART",  "Back to frame 0 and re-arm",      "restart", COL_ACCENT)
        self.btn_live_stop    = _lb("⏹ STOP",    "Abort: ramp to zero + disable",    "stop", COL_CRIT)
        lrow.addWidget(self.btn_live_arm,     0, 0)
        lrow.addWidget(self.btn_live_pause,   0, 1)
        lrow.addWidget(self.btn_live_restart, 1, 0)
        lrow.addWidget(self.btn_live_stop,    1, 1)
        self._live_cmd_buttons = [
            self.btn_live_arm, self.btn_live_pause,
            self.btn_live_restart, self.btn_live_stop,
        ]
        for b in self._live_cmd_buttons:
            b.setEnabled(False)
        lv.addLayout(lrow)

        # Progress section — bar + one-line readout. Hidden until live is on.
        self._live_progress_box = QFrame()
        self._live_progress_box.setStyleSheet(
            "QFrame { background: rgba(0,0,0,0.25); border: none; "
            "border-radius: 4px; }")
        pgv = QVBoxLayout(self._live_progress_box)
        pgv.setContentsMargins(8, 6, 8, 6); pgv.setSpacing(4)

        self.live_progress_bar = QProgressBar()
        self.live_progress_bar.setMinimum(0); self.live_progress_bar.setMaximum(1000)
        self.live_progress_bar.setValue(0)
        self.live_progress_bar.setTextVisible(False)
        self.live_progress_bar.setFixedHeight(5)
        self.live_progress_bar.setStyleSheet(f"""
            QProgressBar {{
                background: #1a1a1a; border: none; border-radius: 2px;
            }}
            QProgressBar::chunk {{
                background: {COL_ACCENT}; border-radius: 2px;
            }}
        """)
        pgv.addWidget(self.live_progress_bar)

        readout_row = QHBoxLayout(); readout_row.setSpacing(10)
        self.lbl_live_frame = QLabel("frame —")
        self.lbl_live_frame.setStyleSheet(
            f"color: {COL_TEXT}; font-family: Consolas, monospace; "
            f"font-size: 9pt; font-weight: 600;")
        self.lbl_live_err = QLabel("")
        self.lbl_live_err.setStyleSheet(
            f"color: {COL_MUTED}; font-family: Consolas, monospace; "
            f"font-size: 9pt; font-weight: 700;")
        self.lbl_live_err.setAlignment(Qt.AlignRight)
        readout_row.addWidget(self.lbl_live_frame)
        readout_row.addStretch(1)
        readout_row.addWidget(self.lbl_live_err)
        pgv.addLayout(readout_row)

        self.lbl_live_time = QLabel("")
        self.lbl_live_time.setStyleSheet(
            f"color: {COL_MUTED}; font-family: Consolas, monospace; font-size: 9pt;")
        pgv.addWidget(self.lbl_live_time)

        self._live_progress_box.setVisible(False)
        lv.addWidget(self._live_progress_box)

        # Live options (loop / goto-start / vel FF).
        opt_row = QHBoxLayout(); opt_row.setSpacing(14)
        def _cb(txt, tip, signal, checked=False):
            c = QCheckBox(txt)
            c.setToolTip(tip)
            c.setChecked(checked)
            c.setCursor(Qt.PointingHandCursor)
            c.setStyleSheet(f"""
                QCheckBox {{ color: {COL_TEXT}; font-size: 9pt; spacing: 6px; }}
                QCheckBox::indicator {{
                    width: 14px; height: 14px; border-radius: 3px;
                    background: #1a1a1a; border: 1px solid {COL_BORDER};
                }}
                QCheckBox::indicator:hover {{ border-color: {COL_ACCENT}; }}
                QCheckBox::indicator:checked {{
                    background: {COL_ACCENT}; border-color: {COL_ACCENT};
                    image: none;
                }}
            """)
            c.toggled.connect(signal.emit)
            return c
        self.cb_live_loop = _cb("loop", "Repeat episode indefinitely",
                                self.liveLoopChanged, False)
        self.cb_live_goto = _cb("go to start",
                                "Slow ramp to episode start before playing",
                                self.liveGotoStartChanged, True)
        self.cb_live_vff  = _cb("vel FF",
                                "Feed recorded joint velocities to controller "
                                "(off by default — recorded velocity noise can "
                                "make playback shaky)",
                                self.liveVelFfChanged, False)
        opt_row.addWidget(self.cb_live_loop)
        opt_row.addWidget(self.cb_live_goto)
        opt_row.addWidget(self.cb_live_vff)
        opt_row.addStretch(1)
        lv.addLayout(opt_row)

        v.addWidget(self._live_box)

        # Episode metadata strip.
        self.lbl_name = QLabel("(no episode loaded)")
        self.lbl_name.setStyleSheet(
            f"color: {COL_TEXT}; font-weight: 800; font-size: 12pt;")
        self.lbl_name.setWordWrap(True)
        v.addWidget(self.lbl_name)
        self.lbl_meta = QLabel("")
        self.lbl_meta.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")
        self.lbl_meta.setWordWrap(True)
        v.addWidget(self.lbl_meta)

        v.addWidget(_hline())

        # Big play/pause.
        self.btn_play = QPushButton("▶  PLAY   (Space)")
        self.btn_play.setMinimumHeight(64)
        self.btn_play.setCursor(Qt.PointingHandCursor)
        self._apply_play_style(False)
        self.btn_play.clicked.connect(self.playToggle.emit)
        v.addWidget(self.btn_play)

        # Scrub slider + counter.
        row_counter = QHBoxLayout()
        self.lbl_frame = QLabel("0 / 0")
        self.lbl_frame.setStyleSheet(
            f"color: {COL_TEXT}; font-family: Consolas, monospace; "
            f"font-size: 10pt;")
        self.lbl_time = QLabel("0.0s / 0.0s")
        self.lbl_time.setStyleSheet(
            f"color: {COL_MUTED}; font-family: Consolas, monospace; "
            f"font-size: 10pt;")
        row_counter.addWidget(self.lbl_frame)
        row_counter.addStretch(1)
        row_counter.addWidget(self.lbl_time)
        v.addLayout(row_counter)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0); self.slider.setMaximum(0)
        self.slider.setTracking(True)
        self.slider.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                background: #1e1e1e; height: 8px; border-radius: 4px;
            }}
            QSlider::handle:horizontal {{
                background: {COL_ACCENT}; width: 16px; margin: -5px 0;
                border-radius: 8px;
            }}
            QSlider::sub-page:horizontal {{
                background: {COL_ACCENT}; border-radius: 4px;
            }}
        """)
        self.slider.sliderMoved.connect(self.seekTo.emit)
        v.addWidget(self.slider)

        # Step buttons.
        step_row = QHBoxLayout()
        step_row.setSpacing(4)
        def _sb(label: str, tip: str, delta: int) -> QPushButton:
            b = _make_button(label, tip)
            b.clicked.connect(lambda _=False, d=delta: self.stepBy.emit(d))
            return b
        step_row.addWidget(_sb("◂◂", "Jump back 10 frames (,)",   -10))
        step_row.addWidget(_sb("◂",  "Step back 1 frame (J)",      -1))
        step_row.addWidget(_sb("▸",  "Step forward 1 frame (L)",    1))
        step_row.addWidget(_sb("▸▸", "Jump forward 10 frames (.)",  10))
        v.addLayout(step_row)

        v.addWidget(_hline())

        # Speed selector.
        v.addWidget(_group_title("SPEED"))
        speed_row = QHBoxLayout()
        speed_row.setSpacing(4)
        self._speed_btns: list[QPushButton] = []
        for s in self._SPEEDS:
            b = QPushButton(f"{s}x" if s != 1.0 else "1x")
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setMinimumHeight(32)
            b.setStyleSheet(f"""
                QPushButton {{
                    background: {COL_PANEL_ALT}; color: {COL_TEXT};
                    border: 1px solid {COL_BORDER}; border-radius: 4px;
                    font-weight: 700; font-size: 10pt;
                }}
                QPushButton:hover {{ background: #333; }}
                QPushButton:checked {{
                    background: {COL_ACCENT}; color: white;
                    border-color: {COL_ACCENT};
                }}
            """)
            b.clicked.connect(lambda _=False, ss=s: self._on_speed(ss))
            self._speed_btns.append(b)
            speed_row.addWidget(b)
        v.addLayout(speed_row)
        # Default 1.0x
        self._speed_btns[2].setChecked(True)

        v.addWidget(_hline())

        # Open a different file from disk.
        self.btn_open = _make_button(
            "📁 Open episode file…",
            "Pick a different HDF5 to replay")
        self.btn_open.clicked.connect(self.openFile.emit)
        v.addWidget(self.btn_open)

        v.addStretch(1)

    # ---- API used by _MainWindow -------------------------------------

    def set_episode(self, ep: dict, path: Path) -> None:
        self.lbl_name.setText(path.name)
        meta_parts = []
        if ep.get("task_tag"):
            meta_parts.append(f"task: {ep['task_tag']}")
        meta_parts.append(f"{ep['T']} frames @ {ep['hz']:.0f} Hz")
        dur = ep['T'] / max(1.0, ep['hz'])
        meta_parts.append(f"{dur:.1f}s total")
        if ep.get("collected_at"):
            meta_parts.append(_human_time(ep["collected_at"]))
        self.lbl_meta.setText("   ·   ".join(meta_parts))
        self.slider.setMaximum(max(0, ep["T"] - 1))
        self.slider.setValue(0)
        self.lbl_frame.setText(f"0 / {ep['T']}")
        self.lbl_time.setText(f"0.0s / {dur:.1f}s")

    def clear_episode(self) -> None:
        self.lbl_name.setText("(no episode loaded)")
        self.lbl_meta.setText("")
        self.slider.setMaximum(0)
        self.slider.setValue(0)
        self.lbl_frame.setText("0 / 0")
        self.lbl_time.setText("0.0s / 0.0s")

    def set_frame(self, idx: int, total: int, hz: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(idx)
        self.slider.blockSignals(False)
        self.lbl_frame.setText(f"{idx} / {total}")
        self.lbl_time.setText(
            f"{idx/max(1.0,hz):.1f}s / {total/max(1.0,hz):.1f}s")

    def set_play_state(self, playing: bool) -> None:
        self._apply_play_style(playing)

    def _apply_play_style(self, playing: bool) -> None:
        if playing:
            self.btn_play.setText("❚❚  PAUSE   (Space)")
            bg = "#b88710"; border = "#ffd84a"
        else:
            self.btn_play.setText("▶  PLAY   (Space)")
            bg = COL_OK; border = "#4abf4a"
        self.btn_play.setStyleSheet(f"""
            QPushButton {{
                background: {bg}; color: white;
                font-weight: 900; font-size: 14pt;
                border: 2px solid {border}; border-radius: 8px;
                padding: 8px;
            }}
            QPushButton:hover {{ background: {border}; color: black; }}
        """)

    def _on_speed(self, speed: float) -> None:
        for b, s in zip(self._speed_btns, self._SPEEDS):
            b.setChecked(abs(s - speed) < 1e-6)
        self.speedSet.emit(speed)

    # ---- Live replay --------------------------------------------------

    def _apply_live_box_style(self) -> None:
        # Scoped to objectName so nested QFrames don't inherit the border.
        on = getattr(self, "_live_on", False)
        border = COL_CRIT if on else COL_BORDER
        self._live_box.setStyleSheet(
            f"QFrame#liveBox {{ background: {COL_PANEL_ALT}; "
            f"border: 1px solid {border}; border-radius: 8px; }}")

    def _apply_live_title_style(self) -> None:
        on = getattr(self, "_live_on", False)
        color = COL_CRIT if on else COL_MUTED
        self._live_title.setStyleSheet(
            f"color: {color}; font-weight: 900; font-size: 10pt; "
            f"letter-spacing: 1px;")

    def _relabel_button(self, btn: QPushButton, label: str, cmd: str,
                        tip: str, accent: str) -> None:
        btn.setText(label)
        btn.setToolTip(tip)
        btn.setProperty("_live_cmd", cmd)
        accent_border = accent or COL_BORDER
        accent_hover  = accent or "#555"
        btn.setStyleSheet(f"""
            QPushButton {{
                background: {COL_PANEL_ALT}; color: {COL_TEXT};
                border: 1px solid {accent_border}; border-radius: 6px;
                font-weight: 600; font-size: 10pt; padding: 6px 8px;
            }}
            QPushButton:hover {{ background: #333; border-color: {accent_hover}; }}
            QPushButton:pressed {{ background: #2a2a2a; }}
            QPushButton:disabled {{
                background: #242424; color: {COL_MUTED};
                border-color: {COL_BORDER};
            }}
        """)

    def _apply_transport_phase(self, live: bool, phase: Optional[str]) -> None:
        """Retarget ARM+PLAY and PAUSE buttons based on live phase.

        - Paused:   ARM+PLAY becomes a dimmed "restart" hint; PAUSE → RESUME.
        - Playing:  PAUSE holds; ARM+PLAY disabled (already running).
        - Arming:   both non-primary actions kept live for abort/restart flows.
        - Ready/done/shutdown: defaults — ARM+PLAY arms, PAUSE inert.
        """
        p = (phase or "").lower() if live else ""

        if p == "paused":
            self._relabel_button(
                self.btn_live_pause, "▶ RESUME",
                "play", "Resume playback from current frame", COL_OK)
            self._relabel_button(
                self.btn_live_arm, "⏮ ARM+PLAY",
                "arm", "Re-arm from frame 0 (restarts episode)", COL_WARN)
            self.btn_live_pause.setEnabled(True)
            self.btn_live_arm.setEnabled(True)
        elif p == "playing":
            self._relabel_button(
                self.btn_live_pause, "❚❚ PAUSE",
                "pause", "Pause, hold position", COL_WARN)
            self._relabel_button(
                self.btn_live_arm, "▶ ARM+PLAY",
                "arm", "Enable + ramp to start + play", COL_OK)
            self.btn_live_pause.setEnabled(True)
            self.btn_live_arm.setEnabled(False)   # already playing
        elif p == "arming":
            self._relabel_button(
                self.btn_live_pause, "❚❚ PAUSE",
                "pause", "Pause once arming completes", COL_WARN)
            self._relabel_button(
                self.btn_live_arm, "▶ ARM+PLAY",
                "arm", "Arming in progress…", COL_OK)
            self.btn_live_pause.setEnabled(True)
            self.btn_live_arm.setEnabled(False)
        else:   # ready / done / shutdown / off
            self._relabel_button(
                self.btn_live_pause, "❚❚ PAUSE",
                "pause", "Pause, hold position", COL_WARN)
            self._relabel_button(
                self.btn_live_arm, "▶ ARM+PLAY",
                "arm", "Enable + ramp to start + play", COL_OK)
            self.btn_live_pause.setEnabled(live)
            self.btn_live_arm.setEnabled(live)

    def _apply_live_button_style(self) -> None:
        on = self.btn_live.isChecked()
        if not self.btn_live.isEnabled():
            bg, border, fg = "#2a2a2a", "#444", COL_MUTED
            txt = "▶  GO LIVE"
        elif on:
            bg, border, fg = COL_CRIT, "#ff8a8a", "white"
            txt = "⏏  EXIT LIVE"
        else:
            bg, border, fg = COL_OK, "#4abf4a", "white"
            txt = "▶  GO LIVE"
        self.btn_live.setText(txt)
        self.btn_live.setStyleSheet(f"""
            QPushButton {{
                background: {bg}; color: {fg};
                font-weight: 900; font-size: 11pt;
                border: 2px solid {border}; border-radius: 6px;
                padding: 6px;
            }}
            QPushButton:hover {{ background: {border}; color: black; }}
            QPushButton:disabled {{
                background: #2a2a2a; color: {COL_MUTED}; border-color: #444;
            }}
        """)

    def _on_live_clicked(self) -> None:
        # Reflect the button state immediately; main window confirms via snapshot.
        self.liveToggled.emit(self.btn_live.isChecked())

    def set_live_episode_available(self, available: bool) -> None:
        self._live_ep_loaded = available
        self.btn_live.setEnabled(available or self._live_on)
        self._apply_live_button_style()

    def set_live_state(
        self, *, live: bool, phase: Optional[str],
        frame: int, frames: int, hz: float, error: float, speed: float,
    ) -> None:
        """Reflect collect_demo's live-replay snapshot into the panel."""
        self._live_on = live
        self.btn_live.blockSignals(True)
        self.btn_live.setChecked(live)
        self.btn_live.blockSignals(False)
        self.btn_live.setEnabled(self._live_ep_loaded or live)
        self._apply_live_button_style()
        self._apply_live_box_style()
        self._apply_live_title_style()

        if live and phase:
            bg, fg = _live_phase_colors(phase)
            self.lbl_live_phase.set_pill(phase.upper(), bg, fg)
        else:
            self.lbl_live_phase.set_pill("READY" if self._live_ep_loaded else "OFF",
                                         "#2a2a2a", COL_MUTED)

        # Transport buttons: RESTART and STOP always follow live; ARM+PLAY and
        # PAUSE are retargeted per phase (PAUSE ↔ RESUME, etc.).
        self.btn_live_restart.setEnabled(live)
        self.btn_live_stop.setEnabled(live)
        self._apply_transport_phase(live, phase)

        # Local scrub/preview controls are confusing while the robot is moving.
        self._set_local_preview_enabled(not live)

        # Swap warning ↔ progress block based on live state.
        self.lbl_live_warn.setVisible(not live)
        self._live_progress_box.setVisible(live)

        if live and frames > 0 and hz > 0:
            pct = frame / frames
            t   = frame / hz
            dur = frames / hz
            self.live_progress_bar.setValue(int(pct * 1000))
            self.lbl_live_frame.setText(f"frame {frame}/{frames}")
            self.lbl_live_time.setText(
                f"{t:.1f}s / {dur:.1f}s  ·  {pct*100:.0f}%  ·  {speed:.2f}x")
        elif live:
            self.live_progress_bar.setValue(0)
            self.lbl_live_frame.setText("frame —")
            self.lbl_live_time.setText(f"{speed:.2f}x")

        if live:
            err_color = (COL_OK if error < 0.05
                         else COL_WARN if error < 0.20
                         else COL_CRIT)
            self.lbl_live_err.setText(f"err {error:.3f} rad")
            self.lbl_live_err.setStyleSheet(
                f"color: {err_color}; font-family: Consolas, monospace; "
                f"font-size: 9pt; font-weight: 700;")

    def set_live_options(
        self, loop: bool, goto_start: bool, vel_ff: bool,
    ) -> None:
        for cb, val in (
            (self.cb_live_loop, loop),
            (self.cb_live_goto, goto_start),
            (self.cb_live_vff,  vel_ff),
        ):
            cb.blockSignals(True)
            cb.setChecked(val)
            cb.blockSignals(False)

    def _set_local_preview_enabled(self, enabled: bool) -> None:
        self.btn_play.setEnabled(enabled)
        self.slider.setEnabled(enabled)
        # Step buttons live in step_row; iterate via objectName isn't set —
        # iterate children of self that are QPushButton with arrow text.
        for child in self.findChildren(QPushButton):
            if child in self._live_cmd_buttons or child is self.btn_live:
                continue
            if child.text() in ("◂◂", "◂", "▸", "▸▸"):
                child.setEnabled(enabled)


def _live_phase_colors(phase: str) -> tuple[str, str]:
    p = phase.lower()
    if p == "playing":  return COL_OK,    "white"
    if p == "arming":   return "#b88710", "white"
    if p == "paused":   return "#666",    "white"
    if p == "done":     return "#3a6dbf", "white"
    if p == "shutdown": return COL_CRIT,  "white"
    return "#444", "#bbb"  # ready or unknown


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class _MainWindow(QMainWindow):

    def __init__(
        self,
        cmd_queue: queue.Queue,
        meta: dict,
        on_delete_last: Callable[[Path], None],
        output_dir: Path,
    ) -> None:
        super().__init__()
        self._cmd_queue      = cmd_queue
        self._meta           = meta
        self._on_delete_last = on_delete_last
        self._output_dir     = output_dir
        self._last_saved_path: Optional[Path] = None
        self._last_toasted_path: Optional[Path] = None
        self._last_snapshot: Optional[dict] = None
        self._last_action_msg: Optional[str] = None
        self._current_mode   = MODE_COLLECT

        self.setWindowTitle("AIZEE Demo Collector")
        self.resize(1760, 1000)
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background: {COL_BG}; color: {COL_TEXT}; }}
            QLabel {{ color: {COL_TEXT}; }}
            QToolTip {{ background: #222; color: {COL_TEXT}; border: 1px solid {COL_BORDER}; }}
        """)

        # Playback engine — owns QTimer, drives replay mode.
        self._playback = _PlaybackEngine(self)
        self._playback.frameChanged.connect(self._on_replay_frame)
        self._playback.episodeLoaded.connect(self._on_replay_episode_loaded)
        self._playback.playStateChanged.connect(self._on_replay_play_state)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 0)
        root.setSpacing(6)

        self._mode_switch = _ModeSwitch()
        self._mode_switch.modeChanged.connect(self._on_mode_changed)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        top_row.addWidget(self._mode_switch)
        top_row.addStretch(1)
        self._build_status_bar(top_row)
        root.addLayout(top_row)

        self._health = _HealthStrip()
        root.addWidget(self._health)

        main_row = QHBoxLayout()
        main_row.setSpacing(10)

        self._center_stack = QStackedWidget()
        self._center_stack.addWidget(self._build_collect_center())  # 0
        self._center_stack.addWidget(self._build_replay_center())   # 1
        main_row.addWidget(self._center_stack, 1)

        main_row.addWidget(self._build_right_column(), 0)

        root.addLayout(main_row, 1)

        self._legend = _ShortcutLegend()
        root.addWidget(self._legend)

        self.setCentralWidget(central)

        # Floating toasts.
        self._toast = _SaveToast(self)
        self._action_toast = _ActionToast(self)

        self._install_shortcuts()
        self._apply_mode_visibility()

    # ------------------------------------------------------------------
    # Status bar
    # ------------------------------------------------------------------

    def _build_status_bar(self, row: QHBoxLayout) -> None:
        """Append status pills + fullscreen button to *row* (called from __init__).

        The pills sit on the top-right of the window, in line with the
        fullscreen button and the mode switch (which is on the top-left).
        """
        row.setSpacing(8)

        self.lbl_state   = _Pill("READY",     "#444",    "#bbb", min_w=110)
        self.lbl_robot   = _Pill("robot --",  "#333",    COL_MUTED)
        self.lbl_battery = _Pill("bus --",    "#333",    COL_MUTED)
        self.lbl_estop   = _Pill("e-stop ?",  "#333",    COL_MUTED)
        # Fixed width + monospaced font so the changing latency digits don't
        # nudge adjacent pills as values fluctuate.
        self.lbl_cams    = _Pill("cams --",   "#333",    COL_MUTED,
                                 font_family="Consolas")
        self.lbl_cams.setFixedWidth(200)
        self.lbl_leader  = _Pill("leader --", "#333",    COL_MUTED)

        for w in (self.lbl_state, self.lbl_robot, self.lbl_battery,
                  self.lbl_estop, self.lbl_cams, self.lbl_leader):
            row.addWidget(w)

        self.btn_fullscreen = QPushButton("⛶ Fullscreen")
        self.btn_fullscreen.setCursor(Qt.PointingHandCursor)
        self.btn_fullscreen.setToolTip("Toggle fullscreen (F11)")
        self.btn_fullscreen.setStyleSheet(f"""
            QPushButton {{
                background: {COL_PANEL_ALT}; color: {COL_TEXT};
                border: 1px solid {COL_BORDER}; border-radius: 12px;
                padding: 6px 14px; font-weight: 600; font-size: 10pt;
            }}
            QPushButton:hover {{ background: #333; border-color: {COL_ACCENT}; }}
        """)
        self.btn_fullscreen.clicked.connect(self._toggle_fullscreen)
        row.addWidget(self.btn_fullscreen)

    # ------------------------------------------------------------------
    # Center pages (Collect / Replay)
    # ------------------------------------------------------------------

    def _build_collect_center(self) -> QWidget:
        split = QSplitter(Qt.Vertical)
        split.setHandleWidth(6)

        # Live camera preview is a native Qt widget (was an embedded Rerun
        # WASM viewer) — see _LiveCameraPair for the rationale.  Lives in
        # a host frame so it can be reparented into the replay center while
        # live-replay mode is on.
        self._collect_cam_host = QFrame()
        self._collect_cam_host.setMinimumHeight(280)
        _ch = QVBoxLayout(self._collect_cam_host)
        _ch.setContentsMargins(0, 0, 0, 0); _ch.setSpacing(0)
        self._cam_pair = _LiveCameraPair()
        self._cam_pair.setMinimumHeight(280)
        _ch.addWidget(self._cam_pair)
        split.addWidget(self._collect_cam_host)

        # Bottom area: time-series chart in place of the joint table.
        # Rolls while idle, accumulates while recording so each take's full
        # trajectory is captured.  _joints_collect is kept as an orphan
        # widget (never added to any layout) so apply_snapshot's existing
        # call still has a target — it's effectively a no-op sink.
        self._collect_chart  = _LiveTimeSeriesPanel(
            title="TARGET vs ACTUAL — last 10s")
        self._joints_collect = _JointPanel()
        split.addWidget(self._collect_chart)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 4)
        split.setSizes([520, 380])
        return split

    def _build_replay_center(self) -> QWidget:
        split = QSplitter(Qt.Vertical)
        split.setHandleWidth(6)

        # Camera area swaps between recorded HDF5 frames (page 0) and the
        # live preview (page 1, populated by reparenting self._cam_pair).
        self._replay_cams = _PlaybackCameraPair()
        self._replay_live_cam_host = QFrame()
        self._replay_live_cam_host.setStyleSheet(
            f"QFrame {{ background: #0a0a0a; border: 1px solid {COL_BORDER}; "
            f"border-radius: 6px; }}")
        _lh = QVBoxLayout(self._replay_live_cam_host)
        _lh.setContentsMargins(0, 0, 0, 0); _lh.setSpacing(0)
        self._replay_cam_stack = QStackedWidget()
        self._replay_cam_stack.addWidget(self._replay_cams)            # 0
        self._replay_cam_stack.addWidget(self._replay_live_cam_host)   # 1
        split.addWidget(self._replay_cam_stack)

        # Bottom area swaps between the per-joint table (page 0) and a live
        # time-series chart (page 1) — chart is shown only while the episode
        # is driving the physical arm.
        # Page 0: static episode timeline (full recording with playback
        # cursor).  Page 1: live time-series while live mode drives the arm.
        # _joints_replay is kept as an orphan so existing apply() calls in
        # _render_replay_frame remain valid no-ops.
        self._joints_replay  = _JointPanel()
        self._episode_chart  = _LiveTimeSeriesPanel(
            title="EPISODE TIMELINE — load an episode to view")
        self._live_chart     = _LiveTimeSeriesPanel()
        self._replay_bottom_stack = QStackedWidget()
        self._replay_bottom_stack.addWidget(self._episode_chart)   # 0
        self._replay_bottom_stack.addWidget(self._live_chart)      # 1
        split.addWidget(self._replay_bottom_stack)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)
        split.setSizes([560, 340])
        return split

    def _set_replay_cam_source_live(self, live: bool) -> None:
        """Swap the replay camera area between recorded frames and live preview."""
        if live:
            self._replay_live_cam_host.layout().addWidget(self._cam_pair)
            self._replay_cam_stack.setCurrentIndex(1)
        else:
            self._collect_cam_host.layout().addWidget(self._cam_pair)
            self._replay_cam_stack.setCurrentIndex(0)

    # ------------------------------------------------------------------
    # Right column (mode-specific controls + episode list + event log)
    # ------------------------------------------------------------------

    def _build_right_column(self) -> QWidget:
        wrap = QWidget()
        wrap.setFixedWidth(440)
        wrap.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        outer = QVBoxLayout(wrap)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        splitter = QSplitter(Qt.Vertical)
        splitter.setHandleWidth(6)

        self._controls_stack = QStackedWidget()
        self._controls_stack.addWidget(self._build_collect_controls())  # 0
        self._controls_stack.addWidget(self._build_replay_controls())   # 1
        splitter.addWidget(self._controls_stack)

        self._episode_list = _EpisodeList(self._output_dir)
        self._episode_list.episodeReplayRequested.connect(self._on_episode_replay)
        self._episode_list.episodeOpenInFolder.connect(self._on_episode_open_folder)
        self._episode_list.episodeDeleted.connect(self._on_episode_deleted)
        splitter.addWidget(self._episode_list)

        self._event_log = _EventLog()
        splitter.addWidget(self._event_log)

        splitter.setStretchFactor(0, 5)
        splitter.setStretchFactor(1, 3)
        splitter.setStretchFactor(2, 2)
        splitter.setSizes([500, 260, 200])

        outer.addWidget(splitter, 1)
        return wrap

    def _build_collect_controls(self) -> QWidget:
        col = QFrame()
        col.setStyleSheet(
            f"QFrame {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 8px; }}")
        v = QVBoxLayout(col)
        v.setContentsMargins(14, 14, 14, 14)
        v.setSpacing(10)

        self.btn_record = _RecordButton()
        self.btn_record.clicked.connect(lambda: self._send_key("R"))
        v.addWidget(self.btn_record)

        self.lbl_rec_status = QLabel("idle")
        self.lbl_rec_status.setAlignment(Qt.AlignCenter)
        self.lbl_rec_status.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")
        v.addWidget(self.lbl_rec_status)

        v.addWidget(_group_title("TASK TAG"))
        self.edit_tag = _TaskTagCombo(self._meta.get("task_tag", ""), self._output_dir)
        self.edit_tag.editTextChanged.connect(self._on_tag_changed)
        v.addWidget(self.edit_tag)

        v.addWidget(_hline())

        v.addWidget(_group_title("MOTION"))
        mg = QGridLayout(); mg.setSpacing(6)
        def _btn(r, c, key, label, tip, accent=""):
            b = _make_button(label, tip, accent)
            b.clicked.connect(lambda _=False, k=key: self._send_key(k))
            mg.addWidget(b, r, c)
            return b
        _btn(0, 0, "E", "Enable (E)",      "Start tracking / hold", COL_OK)
        _btn(0, 1, "I", "Idle (I)",        "Zero-torque — feel by hand", COL_WARN)
        _btn(0, 2, "H", "Hold (H)",        "Freeze target at current pose")
        _btn(1, 0, "X", "Shutdown (X)",    "Ramp to zero + disable", COL_CRIT)
        _btn(1, 1, "CANCEL_SHUTDOWN", "Cancel (Esc)", "Abort shutdown ramp")
        v.addLayout(mg)

        v.addWidget(_hline())

        v.addWidget(_group_title("TELEOP"))
        tg = QGridLayout(); tg.setSpacing(6)
        def _tbtn(r, c, key, label, tip):
            b = _make_button(label, tip)
            b.clicked.connect(lambda _=False, k=key: self._send_key(k))
            tg.addWidget(b, r, c)
            return b
        _tbtn(0, 0, "Z", "Zero Leader (Z)", "Set current leader pose as zero")
        _tbtn(0, 1, "M", "Mirror (M)",      "Align leader zero to robot")
        _tbtn(0, 2, "P", "Save Ready (P)",  "Write ready_pose.json")
        v.addLayout(tg)

        v.addStretch(1)
        return col

    def _build_replay_controls(self) -> QWidget:
        self._replay_transport = _ReplayTransport()
        self._replay_transport.playToggle.connect(self._playback.toggle_play)
        self._replay_transport.seekTo.connect(self._playback.seek)
        self._replay_transport.stepBy.connect(self._playback.step)
        self._replay_transport.speedSet.connect(self._on_replay_speed)
        self._replay_transport.openFile.connect(self._on_replay_open_file)

        # Live-replay wiring (dict commands into the main-loop cmd_queue).
        self._live_active = False
        self._replay_transport.liveToggled.connect(self._on_live_toggled)
        self._replay_transport.liveCommand.connect(self._on_live_command)
        self._replay_transport.liveLoopChanged.connect(
            lambda v: self._send_cmd({"cmd": "replay_opts", "loop": bool(v)}))
        self._replay_transport.liveGotoStartChanged.connect(
            lambda v: self._send_cmd({"cmd": "replay_opts", "goto_start": bool(v)}))
        self._replay_transport.liveVelFfChanged.connect(
            lambda v: self._send_cmd({"cmd": "replay_opts", "vel_ff": bool(v)}))
        self._replay_transport.liveMaxDeltaChanged.connect(
            lambda v: self._send_cmd({"cmd": "replay_opts", "max_delta": float(v)}))

        # Wrap in a scroll area so the live panel can never push other
        # transport controls out of the visible splitter region.
        scroll = QScrollArea()
        scroll.setWidget(self._replay_transport)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet(
            f"QScrollArea {{ background: {COL_BG}; border: none; }}"
            f"QScrollBar:vertical {{ background: {COL_BG}; width: 10px; margin: 0; }}"
            f"QScrollBar::handle:vertical {{ background: {COL_BORDER}; "
            f"border-radius: 5px; min-height: 30px; }}"
            f"QScrollBar::handle:vertical:hover {{ background: #555; }}"
            f"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}"
        )
        return scroll

    # ------------------------------------------------------------------
    # Live replay handlers
    # ------------------------------------------------------------------

    def _on_replay_speed(self, speed: float) -> None:
        self._playback.set_speed(speed)
        if self._live_active:
            self._send_cmd({"cmd": "replay_speed", "speed": float(speed)})

    def _on_live_toggled(self, on: bool) -> None:
        if on:
            path = self._playback.path()
            if path is None:
                self._event_log.add_event("⚠", "load an episode first", COL_WARN)
                # Roll the toggle back; snapshot will reaffirm.
                self._replay_transport.set_live_state(
                    live=False, phase=None, frame=0, frames=0, hz=0,
                    error=0.0, speed=1.0)
                return
            self._send_cmd({"cmd": "replay_on", "path": str(path)})
            self._event_log.add_event("◉", f"live → {path.name}", COL_CRIT)
        else:
            self._send_cmd({"cmd": "replay_off"})
            self._event_log.add_event("⏏", "exit live", "#bbb")

    def _on_live_command(self, cmd: str) -> None:
        mapping = {
            "arm":     "replay_arm",
            "play":    "replay_play",
            "pause":   "replay_pause",
            "restart": "replay_restart",
            "stop":    "replay_stop",
        }
        wire = mapping.get(cmd)
        if wire:
            self._send_cmd({"cmd": wire})

    def _send_cmd(self, payload: dict) -> None:
        try:
            self._cmd_queue.put_nowait(payload)
        except queue.Full:
            pass

    # ------------------------------------------------------------------
    # Mode switching
    # ------------------------------------------------------------------

    def _on_mode_changed(self, mode: str) -> None:
        self._current_mode = mode
        self._apply_mode_visibility()
        # Pause playback when leaving replay.
        if mode != MODE_REPLAY and self._playback.is_playing():
            self._playback.pause()
        # Auto-load the most recent episode the first time the operator
        # enters replay mode so the view isn't empty.
        if mode == MODE_REPLAY and self._playback.episode() is None:
            p = self._episode_list.selected_path() or self._first_episode_path()
            if p is not None:
                self._playback.load(p)

    def _apply_mode_visibility(self) -> None:
        idx = 1 if self._current_mode == MODE_REPLAY else 0
        self._center_stack.setCurrentIndex(idx)
        self._controls_stack.setCurrentIndex(idx)
        # Keep the live Rerun view parented to whichever center is visible
        # so the operator never sees an empty camera panel mid-live-replay.
        if getattr(self, "_live_active", False) and self._current_mode == MODE_REPLAY:
            self._set_replay_cam_source_live(True)
        else:
            self._set_replay_cam_source_live(False)

    def _first_episode_path(self) -> Optional[Path]:
        try:
            paths = sorted(self._output_dir.glob("episode_*.hdf5"))
            return paths[-1] if paths else None
        except OSError:
            return None

    # ------------------------------------------------------------------
    # Episode list handlers
    # ------------------------------------------------------------------

    def _on_episode_replay(self, path: Path) -> None:
        if self._current_mode != MODE_REPLAY:
            self._mode_switch.set_mode(MODE_REPLAY)
        if self._playback.path() != path:
            self._playback.load(path)
        else:
            self._playback.play()

    def _on_episode_open_folder(self, path: Path) -> None:
        QDesktopServices.openUrl(QUrl.fromLocalFile(str(path.parent)))

    def _on_episode_deleted(self, path: Path) -> None:
        if self._last_saved_path == path:
            self._last_saved_path = None
        if self._playback.path() == path:
            self._playback.pause()
            self._replay_transport.clear_episode()
            self._replay_cams.clear()
            self._episode_chart.clear()
        self._event_log.add_event("🗑", f"deleted {path.name}", COL_HOT)

    # ------------------------------------------------------------------
    # Replay engine handlers
    # ------------------------------------------------------------------

    def _on_replay_open_file(self) -> None:
        fn, _ = QFileDialog.getOpenFileName(
            self, "Open episode", str(self._output_dir),
            "HDF5 files (*.hdf5 *.h5);;All files (*)",
        )
        if fn:
            self._playback.load(Path(fn))

    def _on_replay_episode_loaded(self, payload: object) -> None:
        if not isinstance(payload, dict) or "qpos" not in payload:
            err  = payload.get("error") if isinstance(payload, dict) else "load failed"
            path = payload.get("path")  if isinstance(payload, dict) else None
            name = Path(path).name if path else "episode"
            QMessageBox.warning(self, "Could not load episode",
                                f"Failed to load {name}:\n{err}")
            self._replay_transport.clear_episode()
            self._replay_transport.set_live_episode_available(False)
            self._episode_chart.clear()
            self._event_log.add_event("⚠", f"load failed: {name}", COL_CRIT)
            return
        path = self._playback.path()
        if path is not None:
            self._replay_transport.set_episode(payload, path)
            self._replay_transport.set_live_episode_available(True)
            self._event_log.add_event("▶", f"loaded {path.name}", "#ffb84a")
        # Pre-load the entire episode into the timeline chart so the user
        # can see the full trajectory and the cursor's position in it.
        self._episode_chart.set_episode(
            qpos=payload["qpos"],
            actions=payload.get("actions"),
            hz=float(payload.get("hz", 20.0)),
            label=path.name if path is not None else "",
        )
        self._render_replay_frame(0)

    def _on_replay_frame(self, idx: int) -> None:
        self._render_replay_frame(idx)

    def _on_replay_play_state(self, playing: bool) -> None:
        self._replay_transport.set_play_state(playing)

    def _render_replay_frame(self, idx: int) -> None:
        ep = self._playback.episode()
        if ep is None:
            return
        total = ep["T"]
        hz    = ep["hz"]
        self._replay_transport.set_frame(idx, total, hz)

        left  = ep["left"][idx]  if ep["left"]  is not None else None
        right = ep["right"][idx] if ep["right"] is not None else None
        self._replay_cams.set_frames(left, right)

        qpos    = ep["qpos"][idx]
        actions = ep["actions"][idx] if ep["actions"] is not None else None
        # Replay target: swivel passes through from qpos; arm joints use
        # the recorded action.  Falls back to qpos if layout differs.
        if actions is not None and len(qpos) == 7 and len(actions) == 6:
            target = np.concatenate([[qpos[0]], actions])
        elif actions is not None and len(qpos) == len(actions):
            target = actions
        else:
            target = qpos
        self._joints_replay.apply(
            target=target, actual=qpos,
            torque=None, temp=None, leader=None,
        )
        self._episode_chart.set_cursor_frame(idx)

    # ------------------------------------------------------------------
    # Shortcuts
    # ------------------------------------------------------------------

    def _install_shortcuts(self) -> None:
        bindings = [
            ("F2",  "R"),  ("R", "R"),
            ("Q",   "Q"),  ("Esc", "CANCEL_SHUTDOWN"),
            ("E",   "E"),  ("I",   "I"),  ("H", "H"),
            ("F",   "F"),  ("X",   "X"),
            ("Z",   "Z"),  ("M",   "M"),  ("P", "P"),
            ("W",   "W"),  ("A",   "A"),  ("S", "S"),  ("D", "D"),
        ]
        for seq, key_str in bindings:
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(lambda k=key_str: self._on_shortcut(k))

        for seq in ("F11",):
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(self._toggle_fullscreen)

        # Replay hotkeys — gated by mode inside _on_replay_shortcut.
        replay_bindings = [
            ("Space", lambda: self._playback.toggle_play()),
            ("K",     lambda: self._playback.toggle_play()),
            ("J",     lambda: self._playback.step(-1)),
            ("L",     lambda: self._playback.step(+1)),
            (",",     lambda: self._playback.step(-10)),
            (".",     lambda: self._playback.step(+10)),
            ("Home",  lambda: self._playback.seek(0)),
            ("End",   lambda: self._playback.seek(self._playback.total_frames() - 1)),
        ]
        for seq, cb in replay_bindings:
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(lambda cb=cb: self._on_replay_shortcut(cb))

    def _on_shortcut(self, key: str) -> None:
        fw = QApplication.focusWidget()
        if isinstance(fw, QLineEdit):
            return
        # Collect/teleop/motion keys are only meaningful while collecting.
        # Q always works so the operator can quit from any mode.
        if self._current_mode != MODE_COLLECT and key != "Q":
            return
        self._send_key(key)

    def _on_replay_shortcut(self, cb) -> None:
        fw = QApplication.focusWidget()
        if isinstance(fw, QLineEdit):
            return
        if self._current_mode != MODE_REPLAY:
            return
        cb()

    def _toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
            self.btn_fullscreen.setText("⛶ Fullscreen")
        else:
            self.showFullScreen()
            self.btn_fullscreen.setText("⛶ Exit Fullscreen")

    # ------------------------------------------------------------------
    # Input → main loop
    # ------------------------------------------------------------------

    def _send_key(self, key: str) -> None:
        try:
            self._cmd_queue.put_nowait(key)
        except queue.Full:
            pass
        if key == "Q":
            QApplication.instance().quit()

    def _on_tag_changed(self, text: str) -> None:
        self._meta["task_tag"] = text

    # ------------------------------------------------------------------
    # Snapshot → widgets (Collect-mode live telemetry)
    # ------------------------------------------------------------------

    def set_camera_frames(self,
                          left:  Optional[bytes], left_ts:  float,
                          right: Optional[bytes], right_ts: float) -> None:
        self._cam_pair.set_frames(left, left_ts, right, right_ts)

    def apply_snapshot(self, s: dict) -> None:
        self._last_snapshot = s

        # Live replay snapshot — process first so the top state pill reflects it.
        live  = bool(s.get("replay_live", False))
        phase = s.get("replay_phase")
        was_live = getattr(self, "_live_active", False)
        self._live_active = live
        self._replay_transport.set_live_state(
            live=live, phase=phase,
            frame=int(s.get("replay_frame", 0)),
            frames=int(s.get("replay_frames", 0)),
            hz=float(s.get("replay_hz", 0.0)),
            error=float(s.get("replay_error", 0.0)),
            speed=float(s.get("replay_speed", 1.0)),
        )
        if live and not was_live and self._playback.is_playing():
            # Avoid two playback streams (local preview + robot) running together.
            self._playback.pause()
        if live != was_live:
            self._set_replay_cam_source_live(live)
            tag = "live ON" if live else "live OFF"
            self._event_log.add_event("◉", tag,
                                       COL_CRIT if live else "#bbb")

        state = s.get("state", "ready")
        estop = bool(s.get("estop_active", False))
        if live and phase:
            # Override teleop state with the replay phase while live mode is on.
            bg, fg = _live_phase_colors(phase)
            self.lbl_state.set_pill(f"REPLAY {phase.upper()}", bg, fg)
        else:
            bg, fg = _state_colors(state, estop)
            self.lbl_state.set_pill(state.upper(), bg, fg)

        robot_ok  = bool(s.get("robot_ok", False))
        telem_age = float(s.get("telem_age", 999.0))
        if robot_ok and telem_age < 2.0:
            self.lbl_robot.set_pill("robot OK", COL_OK)
        elif robot_ok:
            self.lbl_robot.set_pill(f"robot {telem_age:.0f}s stale", COL_WARN)
        else:
            self.lbl_robot.set_pill("robot offline", "#444", COL_MUTED)

        bv = s.get("battery_voltage")
        if bv is None:
            self.lbl_battery.set_pill("bus --", "#333", COL_MUTED)
        else:
            bv = float(bv)
            if bv < 18.0:
                self.lbl_battery.set_pill(f"bus {bv:.1f}V", COL_CRIT)
            elif bv < 20.0:
                self.lbl_battery.set_pill(f"bus {bv:.1f}V", COL_WARN)
            else:
                self.lbl_battery.set_pill(f"bus {bv:.1f}V", COL_OK)

        if estop:
            self.lbl_estop.set_pill("E-STOP", COL_CRIT)
        else:
            self.lbl_estop.set_pill("SAFE", COL_OK)

        lage = float(s.get("cam_left_age",  999.0))
        rage = float(s.get("cam_right_age", 999.0))
        if lage < 0.5 and rage < 0.5:
            self.lbl_cams.set_pill(
                f"cams OK  L{lage*1000:3.0f} R{rage*1000:3.0f}ms", COL_OK)
        elif lage > 2.0 and rage > 2.0:
            self.lbl_cams.set_pill("cams STALE", COL_CRIT)
        else:
            side = "L" if lage > 0.5 else "R"
            self.lbl_cams.set_pill(f"cam {side} slow", COL_WARN)

        if s.get("leader_connected"):
            age = float(s.get("leader_age", 999.0))
            if age < 1.0:
                self.lbl_leader.set_pill("leader OK", COL_OK)
            else:
                self.lbl_leader.set_pill(f"leader {age:.0f}s", COL_WARN)
        else:
            self.lbl_leader.set_pill("no leader", "#444", COL_MUTED)

        # Collect-mode joint panel.  (Replay mode's panel is normally driven
        # by _render_replay_frame from _PlaybackEngine ticks; while live
        # replay is on, the snapshot below takes over.)
        self._joints_collect.apply(
            target=s.get("target"),
            actual=s.get("actual"),
            torque=s.get("torque"),
            temp=s.get("temp"),
            leader=s.get("leader_mapped"),
        )

        rec   = bool(s.get("recording", False))
        steps = int(s.get("rec_steps", 0))
        drop  = int(s.get("dropped", 0))

        # Collect-mode time-series chart.  Switch to accumulate mode on the
        # leading edge of recording so each take's full trajectory is
        # captured; switch back to a rolling 10s window when idle.
        if rec != self._collect_chart.is_accumulating():
            self._collect_chart.set_accumulate(rec)
        self._collect_chart.add_sample(
            s.get("target"), s.get("actual"),
            temp=s.get("temp"), states=s.get("motor_states"),
            leader=s.get("leader_mapped"), torque=s.get("torque"),
        )

        # While live replay is driving the robot, swap the replay-mode joint
        # table for the time-series chart and feed it samples.  When live
        # disengages, swap back and clear the rolling buffer.
        if live:
            rt = s.get("replay_target")
            target_arr = np.asarray(rt, dtype=float) if rt is not None else None
            self._live_chart.add_sample(
                target_arr, s.get("actual"),
                temp=s.get("temp"), states=s.get("motor_states"),
                leader=s.get("leader_mapped"), torque=s.get("torque"),
            )
            if self._replay_bottom_stack.currentIndex() != 1:
                self._replay_bottom_stack.setCurrentIndex(1)
        else:
            if self._replay_bottom_stack.currentIndex() != 0:
                self._replay_bottom_stack.setCurrentIndex(0)
                self._live_chart.clear()
        rec_enabled = (s.get("state") == "tracking")
        self.btn_record.set_state(rec, steps, drop, enabled=rec_enabled)
        if rec:
            dur = steps / 20.0
            parts = [f"● recording · {dur:.1f}s · {steps} steps"]
            if drop: parts.append(f"dropped {drop}")
            self.lbl_rec_status.setText("   ·   ".join(parts))
            self.lbl_rec_status.setStyleSheet(
                f"color: #ff9090; font-size: 10pt; font-weight: 700;")
        elif not rec_enabled:
            self.lbl_rec_status.setText("enable tracking (E) to record")
            self.lbl_rec_status.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")
        else:
            self.lbl_rec_status.setText("idle")
            self.lbl_rec_status.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")

        # Action-confirmation toast + event log entry, edge-triggered.
        amsg = s.get("action_msg")
        if amsg and amsg != self._last_action_msg:
            self._action_toast.show_msg(str(amsg))
            icon, color, text = self._action_event_style(str(amsg))
            self._event_log.add_event(icon, text, color)
        self._last_action_msg = amsg

        msg = s.get("save_msg")
        if msg:
            self.lbl_rec_status.setText(str(msg))
            self.lbl_rec_status.setStyleSheet(
                f"color: {COL_TEXT}; font-size: 10pt; font-weight: 600;")

        # New saved episode: toast + event log + list row.
        lsp = s.get("last_saved_path")
        if lsp is not None:
            p = Path(lsp)
            if p != self._last_saved_path:
                self._last_saved_path = p
                tag = self._meta.get("task_tag", "")
                if tag:
                    self.edit_tag.add_recent(tag)
                if p != self._last_toasted_path:
                    self._last_toasted_path = p
                    self._toast.show_save(p, self._on_toast_undo)
                self._event_log.add_event("💾", f"saved {p.name}", COL_OK)
                self._episode_list.add_or_update(p)

        self._health.apply(s)
        self._legend.set_state(state, rec)

    @staticmethod
    def _action_event_style(msg: str) -> tuple[str, str, str]:
        """Map an action_msg prefix to (icon, color, text) for the event log."""
        for prefix, (icon, color) in (
            ("[Z]", ("◎", "#6aa0ff")),
            ("[M]", ("⇄", "#6ac0ff")),
            ("[P]", ("✓", "#6abf6a")),
        ):
            if msg.startswith(prefix):
                return (icon, color, msg[len(prefix):].strip(" —-:"))
        return ("ℹ", COL_TEXT, msg)

    def _on_toast_undo(self, path: Path) -> None:
        self._on_delete_last(path)
        if self._last_saved_path == path:
            self._last_saved_path = None
        self._event_log.add_event("↩", f"undid {path.name}", COL_WARN)
        # Remove from episode list row if still shown.
        try:
            lst = self._episode_list._list
            for i in range(lst.count()):
                it = lst.item(i)
                if it.data(Qt.UserRole) == str(path):
                    lst.takeItem(i)
                    break
            self._episode_list._update_count()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        self._toast._reposition()
        self._action_toast._reposition()


# ---------------------------------------------------------------------------
# QtRenderer facade (unchanged interface for collect_demo.py)
# ---------------------------------------------------------------------------

class QtRenderer:

    def __init__(
        self,
        cmd_queue: queue.Queue,
        meta: dict,
        on_delete_last: Callable[[Path], None],
        output_dir: Path,
    ) -> None:
        self._cmd_queue       = cmd_queue
        self._meta            = meta
        self._on_delete_last  = on_delete_last
        self._output_dir      = output_dir
        self._lock            = threading.Lock()
        self._holder: dict    = {"args": None}
        # Separate camera holder so the main loop can push raw JPEG bytes
        # at camera-publisher cadence without rebuilding the full snapshot
        # dict.  QtRenderer._tick reads it under the same lock and forwards
        # to _LiveCameraPair, which drops stale frames by timestamp.
        self._cam_holder: dict = {"left": None, "left_ts": 0.0,
                                  "right": None, "right_ts": 0.0}
        self._stop            = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._window: Optional[_MainWindow]      = None

    @property
    def holder(self) -> dict:
        return self._holder

    @property
    def cam_holder(self) -> dict:
        return self._cam_holder

    @property
    def lock(self) -> threading.Lock:
        return self._lock

    @property
    def stop_event(self) -> threading.Event:
        return self._stop

    def push(self, snapshot: dict) -> None:
        with self._lock:
            self._holder["args"] = snapshot

    def start(self) -> threading.Thread:
        self._thread = threading.Thread(target=self._run, daemon=True, name="QtRenderer")
        self._thread.start()
        return self._thread

    def request_quit(self) -> None:
        self._stop.set()
        app = QApplication.instance()
        if app is not None:
            app.quit()

    def join(self, timeout: float = 2.0) -> None:
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _run(self) -> None:
        app = QApplication.instance() or QApplication([])
        # Dark Fusion palette for a more polished default look.
        try:
            from PySide6.QtWidgets import QStyleFactory
            app.setStyle(QStyleFactory.create("Fusion"))
        except Exception:
            pass
        app.setFont(QFont("Segoe UI", 10))

        self._window = _MainWindow(
            cmd_queue=self._cmd_queue,
            meta=self._meta,
            on_delete_last=self._on_delete_last,
            output_dir=self._output_dir,
        )
        self._window.show()

        timer = QTimer()
        timer.setInterval(33)  # ~30 Hz
        timer.timeout.connect(self._tick)
        timer.start()

        stop_timer = QTimer()
        stop_timer.setInterval(200)
        stop_timer.timeout.connect(lambda: app.quit() if self._stop.is_set() else None)
        stop_timer.start()

        app.exec()

        try:
            self._cmd_queue.put_nowait("Q")
        except queue.Full:
            pass

    def _tick(self) -> None:
        with self._lock:
            snap   = self._holder["args"]
            l_jpg  = self._cam_holder["left"]
            l_ts   = self._cam_holder["left_ts"]
            r_jpg  = self._cam_holder["right"]
            r_ts   = self._cam_holder["right_ts"]
        if self._window is not None:
            if snap is not None:
                self._window.apply_snapshot(snap)
            if l_jpg is not None or r_jpg is not None:
                self._window.set_camera_frames(l_jpg, l_ts, r_jpg, r_ts)
