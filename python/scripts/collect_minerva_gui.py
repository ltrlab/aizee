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
from pathlib import Path
from typing import Dict, List, Optional

import h5py

from PySide6.QtCore import Qt, QTimer, QThread, QPointF, Signal
from PySide6.QtGui import (
    QColor, QFont, QImage, QKeySequence, QPainter, QPen, QPixmap, QPolygonF, QShortcut,
)
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout,
    QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QSizePolicy, QSlider,
    QVBoxLayout, QWidget,
)

from collect_minerva_app.settings import CollectorSettings, list_input_devices

# Controlled vocabulary of sub-action labels (editable — free text also allowed).
_ACTION_VOCAB = [
    "reach", "grasp", "lift", "move", "align", "insert", "place",
    "release", "open gripper", "close gripper", "retract", "push", "pull",
]

from common.minerva_constants import IDX, JOINT_LIMITS, MINERVA_JOINTS, NUM_MINERVA_JOINTS, SAT_TORQUE

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
        if ts is not None and ts == self._last_ts:
            return
        self._last_ts = ts
        img = QImage()
        if not img.loadFromData(jpeg, "JPEG"):
            return
        pm = QPixmap.fromImage(img)
        self.setPixmap(pm.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def set_rgb(self, arr) -> None:
        """Paint a decoded uint8 RGB frame [H,W,3] (recorded-episode playback)."""
        if arr is None or arr.ndim != 3:
            return
        h, w = arr.shape[:2]
        # .copy() so QImage owns the buffer (arr.tobytes() would be GC'd otherwise).
        img = QImage(arr.tobytes(), w, h, 3 * w, QImage.Format_RGB888).copy()
        pm = QPixmap.fromImage(img)
        self.setPixmap(pm.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


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


class _Thermo(QWidget):
    """Tiny thermometer: mercury fill height + colour track temperature (20..90 °C)."""

    def __init__(self):
        super().__init__()
        self.temp: Optional[float] = None
        self.setFixedSize(48, 16)
        self.setToolTip("Motor temperature (green <45° · yellow <60° · orange <72° · red hot)")

    def set_temp(self, t):
        self.temp = t
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#161616"))
        t = self.temp
        if t is None or t != t:
            p.end()
            return
        t = float(t)
        frac = min(1.0, max(0.0, (t - 20.0) / 70.0))
        col = QColor(_temp_color(t))
        p.setPen(QPen(QColor("#666"), 1))
        p.drawRect(3, 1, 6, h - 5)                       # tube outline
        p.setPen(Qt.NoPen)
        p.setBrush(col)
        fill = int(frac * (h - 7))
        p.drawRect(4, (h - 4) - fill, 4, fill)           # mercury column
        p.drawEllipse(1, h - 6, 10, 5)                   # bulb
        p.setPen(col)
        f = QFont(); f.setPointSize(7); p.setFont(f)
        p.drawText(14, h - 4, f"{t:.0f}°")
        p.end()


class _TorqueBox(QLabel):
    """Torque readout; border + text coloured by |torque| as a fraction of the joint's
    nominal saturation (green <0.5 · yellow <0.8 · red near the limit)."""

    def __init__(self, sat: float):
        super().__init__("--")
        self.sat = max(float(sat), 1e-3)
        self.setFixedWidth(52)
        self.setAlignment(Qt.AlignCenter)
        self.setToolTip("Motor torque (Nm); border reddens as it nears the actuator limit")
        self._paint("#555", "#888")

    def _paint(self, border, fg):
        self.setStyleSheet(f"border:1px solid {border};color:{fg};"
                           "font-family:monospace;font-size:10px;padding:1px;border-radius:0px;")

    def set_torque(self, tq):
        if tq is None or tq != tq:
            self.setText("--")
            self._paint("#555", "#888")
            return
        frac = min(1.0, abs(float(tq)) / self.sat)
        col = _COL_OK if frac < 0.5 else (_COL_WARN if frac < 0.8 else _COL_CRIT)
        self.setText(f"{float(tq):+.2f}")
        self._paint(col, col)


class _JointRow(QWidget):
    """One joint: name, the leader↔follower diff bar, temperature (thermometer) + torque
    indicators, a numeric readout, and -/+ jog buttons that call teleop.jog(idx, ±1)."""

    def __init__(self, idx: int, lo: float, hi: float, jog_cb):
        super().__init__()
        self.idx = idx
        self.lo, self.hi = float(lo), float(hi)
        row = QHBoxLayout(self)
        row.setContentsMargins(2, 1, 2, 1)
        row.setSpacing(4)

        jname = MINERVA_JOINTS[idx] if idx < len(MINERVA_JOINTS) else str(idx)
        name = QLabel(_SHORT.get(idx, str(idx)))
        name.setFixedWidth(34)
        name.setStyleSheet("color:#ccc;")
        name.setToolTip(f"{jname}  (index {idx})")
        row.addWidget(name)

        self.bar = _DiffBar(lo, hi)
        self.bar.setToolTip(
            "Dim fill = follower position; cyan = leader; coloured band = leader−follower "
            "gap (green <0.05, yellow <0.15, red beyond). Drive it green, then engage.")
        row.addWidget(self.bar, 1)

        self.thermo = _Thermo()
        row.addWidget(self.thermo)
        self.torque_box = _TorqueBox(SAT_TORQUE[idx] if idx < len(SAT_TORQUE) else 1.0)
        row.addWidget(self.torque_box)

        self.val = QLabel("--")
        self.val.setFixedWidth(118)
        self.val.setStyleSheet("color:#9cf;font-family:monospace;")
        row.addWidget(self.val)

        for label, d in (("−", -1.0), ("+", 1.0)):
            b = QPushButton(label)
            b.setFixedWidth(24)
            b.setToolTip(f"Jog {jname} {'down' if d < 0 else 'up'}")
            b.clicked.connect(lambda _=False, dd=d: jog_cb(self.idx, dd))
            row.addWidget(b)

    def set(self, actual: Optional[float], leader: Optional[float], target: Optional[float],
            temp: Optional[float] = None, torque: Optional[float] = None):
        # `x != x` is True only for NaN — a dropped arm's joints arrive as NaN, and
        # head/lift have no leader; render those cleanly instead of crashing.
        self.bar.set_values(actual, leader, target)
        self.thermo.set_temp(temp)
        self.torque_box.set_torque(torque)
        if actual is None or actual != actual:
            self.val.setText("--")
            return
        s = f"{actual:+.2f}"
        if leader is not None and leader == leader:
            s += f" L{leader:+.2f} Δ{abs(float(leader) - float(actual)):.2f}"
        elif target is not None and target == target:
            s += f" →{target:+.2f}"
        self.val.setText(s)


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
        for c in self.cameras:
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
    """Edit per-user collector settings (microphone, voice model, record length)."""

    _MODELS = ["tiny.en", "base.en", "small.en", "medium.en"]

    def __init__(self, settings: CollectorSettings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.setWindowTitle("Collector settings")
        self.setMinimumWidth(420)
        form = QFormLayout(self)

        self.mic_combo = QComboBox()
        self.mic_combo.addItem("System default", None)
        for idx, name in list_input_devices():
            self.mic_combo.addItem(f"[{idx}] {name}", idx)
        pos = self.mic_combo.findData(settings.get("mic_device"))
        self.mic_combo.setCurrentIndex(pos if pos >= 0 else 0)
        self.mic_combo.setToolTip("Audio input device used for voice action-labeling")
        form.addRow("Microphone:", self.mic_combo)

        self.model_combo = QComboBox()
        self.model_combo.setEditable(True)
        self.model_combo.addItems(self._MODELS)
        self.model_combo.setCurrentText(str(settings.get("voice_model")))
        self.model_combo.setToolTip("Local whisper model — tiny=fastest, larger=more accurate")
        form.addRow("Voice model:", self.model_combo)

        self.secs_spin = QDoubleSpinBox()
        self.secs_spin.setRange(1.0, 15.0)
        self.secs_spin.setSingleStep(0.5)
        self.secs_spin.setSuffix(" s")
        self.secs_spin.setValue(float(settings.get("voice_seconds")))
        self.secs_spin.setToolTip("How long the mic listens per voice capture")
        form.addRow("Record length:", self.secs_spin)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        form.addRow(btns)

    def values(self) -> dict:
        return {
            "mic_device": self.mic_combo.currentData(),
            "voice_model": self.model_combo.currentText().strip() or "base.en",
            "voice_seconds": float(self.secs_spin.value()),
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

        # -- status pills --
        pills = QHBoxLayout()
        self.pill_state = _Pill("STATE")
        self.pill_state.setToolTip("Robot state — DISABLED / HOLD / TELEOP")
        self.pill_robot = _Pill("robot")
        self.pill_robot.setToolTip("Telemetry link to the follower")
        self.pill_left = _Pill("L leader")
        self.pill_left.setToolTip("Left OpenRB-150 leader arm connection")
        self.pill_right = _Pill("R leader")
        self.pill_right.setToolTip("Right OpenRB-150 leader arm connection")
        self.pill_cams = _Pill("cams")
        self.pill_cams.setToolTip("Camera streams receiving fresh frames (age < 0.5 s)")
        for p in (self.pill_state, self.pill_robot, self.pill_left, self.pill_right, self.pill_cams):
            pills.addWidget(p)
        pills.addStretch(1)
        root.addLayout(pills)

        # -- body: LEFT [live cameras + joint bars]  |  RIGHT [controls + episode viewer] --
        body = QHBoxLayout()
        body.setSpacing(8)

        left = QVBoxLayout()
        left.setSpacing(6)
        cam_row = QHBoxLayout()
        self.cam_panels: Dict[str, _CameraPanel] = {}
        for c in self.cameras:
            panel = _CameraPanel(c)
            self.cam_panels[c] = panel
            cam_row.addWidget(panel, 1)
        cam_host = QWidget()
        cam_host.setLayout(cam_row)
        left.addWidget(cam_host, 2)
        left.addWidget(self._build_joints(), 3)
        left_host = QWidget()
        left_host.setLayout(left)

        right = QVBoxLayout()
        right.setSpacing(6)
        right.addWidget(self._build_controls())
        self.viewer = _EpisodeViewer(self.output_dir, self.cameras)
        right.addWidget(self.viewer, 1)
        right_host = QWidget()
        right_host.setLayout(right)
        right_host.setMinimumWidth(440)

        body.addWidget(left_host, 3)
        body.addWidget(right_host, 2)
        root.addLayout(body, 1)

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
        w.setFrameShape(QFrame.StyledPanel)
        col = QVBoxLayout(w)

        self.record_btn = QPushButton("● RECORD")
        self.record_btn.setCheckable(False)
        self.record_btn.setFixedHeight(64)
        self.record_btn.setStyleSheet(
            "font-size:20px;font-weight:bold;background:#b71c1c;color:white;border:1px solid #7a1010;")
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
        arow.addWidget(self.phase_btn)
        self.voice_btn = QPushButton("🎤")
        self.voice_btn.setFixedWidth(38)
        self.voice_btn.setToolTip("Speak the action label (voice-to-text)")
        arow.addWidget(self.voice_btn)
        col.addLayout(arow)

        self.seg_lbl = QLabel("")
        self.seg_lbl.setStyleSheet("color:#9c9;")
        col.addWidget(self.seg_lbl)

        btns = QHBoxLayout()
        _tips = {"I": "Idle — enable at ZERO torque; read positions + zero. ALWAYS FIRST (I)",
                 "E": "Enable gains (HOLD) — only from Idle, after zeroing (E)",
                 "T": "Engage / disengage leader teleop (T)",
                 "H": "Disable the follower motors (H)"}
        for label, key in (("Idle", "I"), ("Enable", "E"), ("Teleop", "T"), ("Disable", "H")):
            b = QPushButton(label)
            b.setToolTip(_tips[key])
            b.clicked.connect(lambda _=False, k=key: self._key(k))
            btns.addWidget(b)
        col.addLayout(btns)

        # Zeroing + ready pose + soft shutdown (collect_demo parity).
        btns2 = QHBoxLayout()
        _tips2 = {"Z": "Leader zero — capture each leader's current pose as its zero (Z)",
                  "M": "Mirror — set leader zero so it maps to the arm's actual pose (M)",
                  "K": "RobStride mechanical zero + SaveConfig to both arms; disable first (K)",
                  "P": "Save the current pose as the ready pose (P)",
                  "X": "Soft shutdown — ramp both arms to zero, then disable (X)"}
        for label, key in (("Zero", "Z"), ("Mirror", "M"), ("MechZero", "K"),
                           ("Ready", "P"), ("Shutdown", "X")):
            b = QPushButton(label)
            b.setToolTip(_tips2[key])
            b.clicked.connect(lambda _=False, k=key: self._key(k))
            btns2.addWidget(b)
        col.addLayout(btns2)

        # Leader↔arm routing (persists across sessions; disabled while teleop is live).
        self.swap_btn = QPushButton()
        self.swap_btn.setToolTip("Swap which leader drives which arm. Saved for future "
                                 "sessions. Disengage teleop (T) first — re-routing while "
                                 "engaged would make an arm lunge toward the other leader.")
        self.swap_btn.clicked.connect(self._toggle_swap)
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

        self.settings_btn = QPushButton("⚙ Settings")
        self.settings_btn.setToolTip("Microphone, voice model, and record length")
        self.settings_btn.clicked.connect(self._open_settings)
        col.addWidget(self.settings_btn)

        quit_btn = QPushButton("Quit")
        quit_btn.setToolTip("Quit the collector (Q)")
        quit_btn.clicked.connect(self._quit)
        col.addWidget(quit_btn)
        col.addStretch(1)
        return w

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
            for label, idxs in groups:
                hdr = QLabel(label)
                hdr.setStyleSheet("color:#fc9;font-weight:bold;")
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
        dlg = _SettingsDialog(self.settings, self)
        if dlg.exec() == QDialog.Accepted:
            self.settings.update(dlg.values())
            self.settings.save()
            self._voice_seconds = float(self.settings.get("voice_seconds"))
            self._reload_stt()   # apply the new microphone / model live

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
        self._key("Q")
        self.close()

    # -- snapshot updates (called on the GUI thread by QtRenderer) --
    def apply_snapshot(self, s: dict):
        st = s.get("state", "?")
        self.pill_state.update_pill(st, ok=(st != "DISABLED"))
        # Leaders can't be re-routed while engaging/teleoping (would lunge) — grey out.
        if hasattr(self, "swap_btn"):
            self.swap_btn.setEnabled(st not in ("TELEOP", "ENGAGING"))
        present = s.get("present") or {}
        if s.get("both_ok"):
            self.pill_robot.update_pill("arms ok", ok=True)
        elif s.get("robot_ok"):          # exactly one arm reporting
            down = "R" if present.get("left") else "L"
            self.pill_robot.update_pill(f"{down} arm DOWN", ok=False)
        else:
            self.pill_robot.update_pill("no telem", ok=False)
        led = s.get("leaders", {})
        self.pill_left.update_pill("L leader" + (" ✓" if led.get("left") else " ✗"),
                                   ok=led.get("left"))
        self.pill_right.update_pill("R leader" + (" ✓" if led.get("right") else " ✗"),
                                    ok=led.get("right"))
        ages = s.get("cam_ages", {})
        cams_ok = all(a < 0.5 for a in ages.values()) if ages else False
        self.pill_cams.update_pill(f"cams {sum(1 for a in ages.values() if a < 0.5)}/{len(ages)}",
                                   ok=cams_ok)

        rec = s.get("recording")
        steps = s.get("rec_steps", 0)
        dropped = s.get("dropped", 0)
        if rec:
            self.record_btn.setText("■ STOP")
            self.record_btn.setStyleSheet(
                "font-size:20px;font-weight:bold;background:#1b5e20;color:white;border:1px solid #0d3d12;")
            self.rec_status.setText(f"recording — {steps} steps (dropped {dropped})")
        else:
            self.record_btn.setText("● RECORD")
            self.record_btn.setStyleSheet(
                "font-size:20px;font-weight:bold;background:#b71c1c;color:white;border:1px solid #7a1010;")
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

        if s.get("last_saved"):
            self.saved_lbl.setText(f"last saved: {Path(s['last_saved']).name}")
        self.viewer.on_saved(s.get("last_saved"))

    def set_camera_frames(self, frames: Dict[str, bytes], ts: Optional[Dict] = None):
        for c, jpeg in frames.items():
            panel = self.cam_panels.get(c)
            if panel is not None and jpeg is not None:
                panel.set_jpeg(jpeg, ts.get(c) if ts else None)

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
