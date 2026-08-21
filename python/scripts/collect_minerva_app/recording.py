"""recording.py — recording buffers + async episode save for the collector.

Accumulates per-frame qpos/qcmd/torque + one decoded frame per camera (with
paired publisher timestamps) into plain lists at REC_HZ, then finalizes by
dispatching save_minerva_episode (the v6 writer) on a daemon thread so the
30 Hz main loop never blocks on gzip.
"""

from __future__ import annotations

import threading
from typing import Dict, Optional, Sequence

import numpy as np

from collect_demo_app.minerva_recording import save_minerva_episode


class RecordingSession:
    """Frame buffers for one episode. Reset at RECORD start, drained at save."""

    def __init__(self, cameras: Sequence[str]):
        self.cameras = list(cameras)
        self._use_torque = False
        self.reset()

    def reset(self) -> None:
        self.qpos: list = []
        self.qcmd: list = []
        self.torque: list = []
        self.cam: Dict[str, list] = {c: [] for c in self.cameras}
        self.cam_ts: Dict[str, list] = {c: [] for c in self.cameras}
        self.telem_ts: list = []
        self.dropped = 0
        self._use_torque = False
        # Temporal action labels: closed segments + the currently-open one.
        self.segments: list = []
        self._seg: Optional[dict] = None

    def append(
        self,
        qpos: np.ndarray,
        qcmd: Optional[np.ndarray],
        torque: Optional[np.ndarray],
        frames: Dict[str, np.ndarray],
        telem_ts: Optional[float],
        cam_ts: Dict[str, Optional[float]],
    ) -> None:
        if self.steps == 0:
            self._use_torque = torque is not None
        self.qpos.append(np.asarray(qpos, dtype=np.float32))
        self.qcmd.append(np.asarray(qcmd if qcmd is not None else qpos, dtype=np.float32))
        if self._use_torque:
            self.torque.append(
                np.asarray(torque if torque is not None else np.zeros_like(qpos), dtype=np.float32))
        for c in self.cameras:
            self.cam[c].append(frames[c])
            self.cam_ts[c].append(cam_ts.get(c))
        self.telem_ts.append(telem_ts if telem_ts is not None else 0.0)

    @property
    def steps(self) -> int:
        return len(self.qpos)

    # -- temporal action labels (half-open [start, end) frame ranges) --
    def set_label(self, label: str) -> None:
        """Close the open segment at the current frame and open a new one for
        `label`. Called when the operator marks a new phase mid-recording."""
        label = str(label or "").strip()
        now = self.steps
        if self._seg is not None:
            self._seg["end"] = now
            if self._seg["end"] > self._seg["start"] and self._seg["label"]:
                self.segments.append(self._seg)
        self._seg = {"start": now, "label": label} if label else None

    def finalize_segments(self) -> None:
        """Close the open segment at the final frame — call once before save."""
        if self._seg is not None:
            self._seg["end"] = self.steps
            if self._seg["end"] > self._seg["start"] and self._seg["label"]:
                self.segments.append(self._seg)
            self._seg = None


def start_async_save(
    session: RecordingSession,
    output_dir: str,
    *,
    language_instruction: str,
    task_id: Optional[int],
    notes: str,
    result_holder: dict,
    result_lock: threading.Lock,
) -> threading.Thread:
    """Save `session` on a daemon thread. The buffer references are SNAPSHOTTED
    here on the calling thread, so a later RecordingSession.reset() (which
    rebinds these attributes to new lists) can never make the save serialize the
    next take's buffers. Writes result_holder["path"]/"steps" (or "error")."""
    qpos, cam = session.qpos, session.cam
    qcmd, torque = session.qcmd, session.torque
    telem_ts, cam_ts = session.telem_ts, session.cam_ts
    segments = list(session.segments)

    def _run() -> None:
        try:
            path, n = save_minerva_episode(
                output_dir,
                qpos,
                cam,
                qcmd_buf=qcmd or None,
                torque_buf=torque or None,
                telem_ts_buf=telem_ts or None,
                camera_ts_bufs=cam_ts,
                language_instruction=language_instruction,
                task_id=task_id,
                notes=notes,
                segments=segments or None,
            )
            with result_lock:
                result_holder["path"] = str(path)
                result_holder["steps"] = int(n)
                result_holder.pop("error", None)
        except Exception as e:  # noqa: BLE001 — surface any writer failure to the UI
            with result_lock:
                result_holder["error"] = f"{type(e).__name__}: {e}"

    t = threading.Thread(target=_run, daemon=True, name="Save")
    t.start()
    return t


__all__ = ["RecordingSession", "start_async_save"]
