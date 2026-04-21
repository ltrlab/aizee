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
import queue
import threading
import time
from pathlib import Path
from typing import Optional, Callable

import numpy as np

from PySide6.QtCore import Qt, QTimer, QUrl, QPoint
from PySide6.QtGui import (
    QColor, QFont, QKeySequence, QPainter, QPen, QShortcut,
)
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton,
    QSizePolicy, QSplitter, QVBoxLayout, QWidget,
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


def _state_colors(state: str, estop: bool) -> tuple[str, str]:
    if estop:
        return (COL_CRIT, "white")
    return {
        "ready":    ("#444", "#bbb"),
        "idle":     (COL_WARN, "white"),
        "tracking": (COL_OK, "white"),
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
                 min_w: int = 80) -> None:
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(min_w)
        self._set(bg, fg)

    def set_pill(self, text: str, bg: str, fg: str = "white") -> None:
        self.setText(text)
        self._set(bg, fg)

    def _set(self, bg: str, fg: str) -> None:
        self.setStyleSheet(
            f"background: {bg}; color: {fg}; "
            f"padding: 6px 14px; border-radius: 12px; "
            f"font-weight: 600; font-size: 10pt;"
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

    def set_state(self, recording: bool, steps: int, dropped: int = 0) -> None:
        self._steps = steps
        if recording and not self._pulse_timer.isActive():
            self._pulse_timer.start()
        elif not recording and self._pulse_timer.isActive():
            self._pulse_timer.stop()
            self._pulse_phase = 0
            self._apply(False, 0.0)
        self._recording = recording
        if recording:
            dur = steps / 20.0
            drops = f"  ·  dropped {dropped}" if dropped else ""
            self.setText(f"●  REC   {dur:5.1f}s   ·   {steps} steps{drops}")
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
# Startup health check strip (P4)
# ---------------------------------------------------------------------------

class _HealthStrip(QFrame):

    def __init__(self, leader_connected: bool) -> None:
        super().__init__()
        self.setStyleSheet(
            f"_HealthStrip {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 6px; }}")
        self._leader_expected = leader_connected
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

        # Leader arm (pass/skip depending on whether user plugged it in).
        leader_good = bool(snap.get("leader_connected", False)) or not self._leader_expected
        leader_label = "leader OK" if snap.get("leader_connected") else (
            "leader  —  skip" if not self._leader_expected else "leader  MISSING")
        self._c_leader.set_pill(
            leader_label,
            COL_OK if snap.get("leader_connected") else (
                "#333" if not self._leader_expected else COL_WARN),
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
# Main window
# ---------------------------------------------------------------------------

class _MainWindow(QMainWindow):

    def __init__(
        self,
        rerun_url: Optional[str],
        cmd_queue: queue.Queue,
        meta: dict,
        on_delete_last: Callable[[Path], None],
        output_dir: Path,
        leader_connected: bool,
    ) -> None:
        super().__init__()
        self._cmd_queue      = cmd_queue
        self._meta           = meta
        self._on_delete_last = on_delete_last
        self._output_dir     = output_dir
        self._last_saved_path: Optional[Path] = None
        self._last_toasted_path: Optional[Path] = None
        self._last_snapshot: Optional[dict] = None

        self.setWindowTitle("AIZEE Demo Collector")
        self.resize(1700, 960)
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background: {COL_BG}; color: {COL_TEXT}; }}
            QLabel {{ color: {COL_TEXT}; }}
            QToolTip {{ background: #222; color: {COL_TEXT}; border: 1px solid {COL_BORDER}; }}
        """)

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 0)
        root.setSpacing(8)

        root.addLayout(self._build_status_bar())
        self._health = _HealthStrip(leader_connected)
        root.addWidget(self._health)

        main_row = QHBoxLayout()
        main_row.setSpacing(10)

        # Left: cameras on top, joints below — vertical splitter so the user
        # can trade space between them.
        left_split = QSplitter(Qt.Vertical)
        left_split.setHandleWidth(6)
        if rerun_url:
            self._web = QWebEngineView()
            self._web.setUrl(QUrl(rerun_url))
            self._web.setMinimumHeight(280)
            left_split.addWidget(self._web)
        else:
            ph = QLabel("(Rerun disabled — run without --no-rerun to see cameras)")
            ph.setAlignment(Qt.AlignCenter)
            ph.setStyleSheet(f"color: {COL_MUTED}; font-size: 13pt;")
            left_split.addWidget(ph)

        self._joints = _JointPanel()
        left_split.addWidget(self._joints)
        left_split.setStretchFactor(0, 3)
        left_split.setStretchFactor(1, 2)
        left_split.setSizes([560, 340])

        main_row.addWidget(left_split, 1)

        # Right: narrow control column.
        main_row.addWidget(self._build_control_column(), 0)

        root.addLayout(main_row, 1)

        self._legend = _ShortcutLegend()
        root.addWidget(self._legend)

        self.setCentralWidget(central)

        # Floating save toast (positioned in resizeEvent).
        self._toast = _SaveToast(self)

        # Hotkeys.
        self._install_shortcuts()

    # ------------------------------------------------------------------
    # Layout sub-builders
    # ------------------------------------------------------------------

    def _build_status_bar(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(8)

        self.lbl_state   = _Pill("READY",     "#444",    "#bbb", min_w=110)
        self.lbl_robot   = _Pill("robot --",  "#333",    COL_MUTED)
        self.lbl_battery = _Pill("bus --",    "#333",    COL_MUTED)
        self.lbl_estop   = _Pill("e-stop ?",  "#333",    COL_MUTED)
        self.lbl_cams    = _Pill("cams --",   "#333",    COL_MUTED, min_w=140)
        self.lbl_leader  = _Pill("leader --", "#333",    COL_MUTED)
        self.lbl_wheels  = _Pill("wheels OFF","#333",    COL_MUTED)

        for w in (self.lbl_state, self.lbl_robot, self.lbl_battery,
                  self.lbl_estop, self.lbl_cams, self.lbl_leader, self.lbl_wheels):
            row.addWidget(w)
        row.addStretch(1)

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
        return row

    def _build_control_column(self) -> QWidget:
        col = QFrame()
        col.setStyleSheet(
            f"QFrame {{ background: {COL_PANEL}; "
            f"border: 1px solid {COL_BORDER}; border-radius: 8px; }}")
        col.setFixedWidth(440)
        col.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

        v = QVBoxLayout(col)
        v.setContentsMargins(14, 14, 14, 14)
        v.setSpacing(10)

        # --- RECORD area ------------------------------------------------
        self.btn_record = _RecordButton()
        self.btn_record.clicked.connect(lambda: self._send_key("R"))
        v.addWidget(self.btn_record)

        self.lbl_rec_status = QLabel("idle")
        self.lbl_rec_status.setAlignment(Qt.AlignCenter)
        self.lbl_rec_status.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")
        v.addWidget(self.lbl_rec_status)

        # --- Task tag ---------------------------------------------------
        v.addWidget(_group_title("TASK TAG"))
        self.edit_tag = _TaskTagCombo(self._meta.get("task_tag", ""), self._output_dir)
        self.edit_tag.editTextChanged.connect(self._on_tag_changed)
        v.addWidget(self.edit_tag)

        v.addWidget(_hline())

        # --- Motion controls --------------------------------------------
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
        _btn(1, 0, "F", "Wheels (F)",      "Toggle drive motors")
        _btn(1, 1, "X", "Shutdown (X)",    "Ramp to zero + disable", COL_CRIT)
        _btn(1, 2, "CANCEL_SHUTDOWN", "Cancel (Esc)", "Abort shutdown ramp")
        v.addLayout(mg)

        v.addWidget(_hline())

        # --- Teleop calibration -----------------------------------------
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

        v.addWidget(_hline())

        # --- Last save panel --------------------------------------------
        v.addWidget(_group_title("LAST SAVE"))
        last_row = QHBoxLayout()
        self.lbl_last_save = QLabel("No saves this session")
        self.lbl_last_save.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")
        self.lbl_last_save.setWordWrap(True)
        last_row.addWidget(self.lbl_last_save, 1)
        self.btn_delete_last = _make_button("Delete", "Permanently delete the last saved episode")
        self.btn_delete_last.setMinimumWidth(80)
        self.btn_delete_last.setEnabled(False)
        self.btn_delete_last.clicked.connect(self._on_delete_clicked)
        last_row.addWidget(self.btn_delete_last)
        v.addLayout(last_row)

        v.addStretch(1)
        return col

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

        # Fullscreen toggle — F11 (standard) and F.  Local to this window so
        # it doesn't fight the operator's other apps.
        for seq in ("F11",):
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(self._toggle_fullscreen)

    def _on_shortcut(self, key: str) -> None:
        fw = QApplication.focusWidget()
        if isinstance(fw, QLineEdit):
            return
        self._send_key(key)

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

    def _on_delete_clicked(self) -> None:
        p = self._last_saved_path
        if p is None:
            return
        reply = QMessageBox.question(
            self, "Delete episode",
            f"Delete {p.name}?\n\nThis cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            try:
                self._on_delete_last(p)
                self.lbl_last_save.setText(f"Deleted {p.name}")
                self.btn_delete_last.setEnabled(False)
                self._last_saved_path = None
            except OSError as e:
                QMessageBox.warning(self, "Delete failed", str(e))

    # ------------------------------------------------------------------
    # Snapshot → widgets
    # ------------------------------------------------------------------

    def apply_snapshot(self, s: dict) -> None:
        self._last_snapshot = s

        state = s.get("state", "ready")
        estop = bool(s.get("estop_active", False))
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
                f"cams OK  L{lage*1000:.0f} R{rage*1000:.0f}ms", COL_OK)
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

        if bool(s.get("wheels_enabled")):
            self.lbl_wheels.set_pill("wheels ON", COL_OK)
        else:
            self.lbl_wheels.set_pill("wheels OFF", "#444", COL_MUTED)

        # Joint panel.
        self._joints.apply(
            target=s.get("target"),
            actual=s.get("actual"),
            torque=s.get("torque"),
            temp=s.get("temp"),
            leader=s.get("leader_mapped"),
        )

        # Record button.
        rec   = bool(s.get("recording", False))
        steps = int(s.get("rec_steps", 0))
        drop  = int(s.get("dropped", 0))
        self.btn_record.set_state(rec, steps, drop)
        if rec:
            dur = steps / 20.0
            parts = [f"● recording · {dur:.1f}s · {steps} steps"]
            if drop: parts.append(f"dropped {drop}")
            self.lbl_rec_status.setText("   ·   ".join(parts))
            self.lbl_rec_status.setStyleSheet(
                f"color: #ff9090; font-size: 10pt; font-weight: 700;")
        else:
            self.lbl_rec_status.setText("idle")
            self.lbl_rec_status.setStyleSheet(f"color: {COL_MUTED}; font-size: 10pt;")

        # Transient status message from the main loop (e.g. "saved to ...").
        msg = s.get("save_msg")
        if msg:
            self.lbl_rec_status.setText(str(msg))
            self.lbl_rec_status.setStyleSheet(
                f"color: {COL_TEXT}; font-size: 10pt; font-weight: 600;")

        # Last-save panel + save toast.
        lsp = s.get("last_saved_path")
        if lsp is not None:
            p = Path(lsp)
            if p != self._last_saved_path:
                self._last_saved_path = p
                self.btn_delete_last.setEnabled(True)
                self.lbl_last_save.setText(f"Saved  {p.name}")
                self.lbl_last_save.setStyleSheet(
                    f"color: {COL_TEXT}; font-size: 10pt;")
                # Track the episode's tag in recent tags.
                tag = self._meta.get("task_tag", "")
                if tag:
                    self.edit_tag.add_recent(tag)
                # Fire toast once per new path.
                if p != self._last_toasted_path:
                    self._last_toasted_path = p
                    self._toast.show_save(p, self._on_toast_undo)

        # Health strip + shortcut legend state.
        self._health.apply(s)
        self._legend.set_state(state, rec)

    def _on_toast_undo(self, path: Path) -> None:
        self._on_delete_last(path)
        if self._last_saved_path == path:
            self._last_saved_path = None
            self.btn_delete_last.setEnabled(False)
            self.lbl_last_save.setText(f"Undid {path.name}")

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def resizeEvent(self, ev) -> None:
        super().resizeEvent(ev)
        self._toast._reposition()


# ---------------------------------------------------------------------------
# QtRenderer facade (unchanged interface for collect_demo.py)
# ---------------------------------------------------------------------------

class QtRenderer:

    def __init__(
        self,
        rerun_url: Optional[str],
        cmd_queue: queue.Queue,
        meta: dict,
        on_delete_last: Callable[[Path], None],
        output_dir: Path,
        leader_connected: bool,
    ) -> None:
        self._rerun_url       = rerun_url
        self._cmd_queue       = cmd_queue
        self._meta            = meta
        self._on_delete_last  = on_delete_last
        self._output_dir      = output_dir
        self._leader_connected = leader_connected
        self._lock            = threading.Lock()
        self._holder: dict    = {"args": None}
        self._stop            = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._window: Optional[_MainWindow]      = None

    @property
    def holder(self) -> dict:
        return self._holder

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
            rerun_url=self._rerun_url,
            cmd_queue=self._cmd_queue,
            meta=self._meta,
            on_delete_last=self._on_delete_last,
            output_dir=self._output_dir,
            leader_connected=self._leader_connected,
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
            snap = self._holder["args"]
        if snap is not None and self._window is not None:
            self._window.apply_snapshot(snap)
