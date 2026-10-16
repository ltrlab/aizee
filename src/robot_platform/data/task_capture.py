"""Incremental, bounded, atomic task episodes shared by simulation and real collectors.

Files intentionally use task_episode_*.hdf5 rather than the legacy episode glob:
readers must understand action_valid and source clocks before promoting them into
a training dataset. This is raw capture, not an automatic dataset promotion.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import threading
import uuid

import h5py
import numpy as np

CAMERAS = ("left_wrist", "right_wrist", "head")
AUTHORITY = {"human", "policy", "correction", "scripted_fixture"}


def _json(value):
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))


class TaskEpisodeWriter:
    """Nonblocking append, bounded memory and no published file until successful finish.

    q/qcmd are paired receiver motor coordinates in joint_names order. Actions use
    next-sample qcmd, like the existing Minerva collector, with explicit terminal
    and authority-transition masks. No torque or clock is invented when absent.
    Camera timestamps must be publisher/render timestamps in the declared clock;
    The acquisition stage of each publisher stamp must be declared (for example,
    exposure or host read-complete); clock uncertainty belongs in task_state.
    A later collector receive timestamp must not replace a publisher stamp.
    """

    def __init__(self, directory, joint_names, metadata, *, hz=20., max_pending=8, camera_roles=None):
        self.names = tuple(joint_names)
        required = {f"{side}_arm_j{j}" for side in ("left", "right") for j in range(1, 7)}
        required.update({"left_gripper", "right_gripper"})
        if len(self.names) != 14 or set(self.names) != required:
            raise ValueError("task capture requires exactly 14 named bimanual arm/gripper axes")
        if not math.isfinite(hz) or hz <= 0 or not isinstance(max_pending, int) or max_pending < 1:
            raise ValueError("capture rate and pending-frame bound must be positive")
        self.metadata = copy.deepcopy(metadata)
        roles = camera_roles if camera_roles is not None else self.metadata.get("camera_roles", CAMERAS)
        if isinstance(roles, str) or not isinstance(roles, (list, tuple)) or not roles or \
                len(set(roles)) != len(roles) or not set(roles).issubset(CAMERAS):
            raise ValueError("camera_roles must be a nonempty distinct subset of supported camera roles")
        if "camera_roles" in self.metadata and tuple(self.metadata["camera_roles"]) != tuple(roles):
            raise ValueError("camera_roles argument conflicts with metadata roster")
        self.camera_roles = tuple(roles)
        self.metadata["camera_roles"] = list(self.camera_roles)
        if self.metadata.get("data_source") not in {"simulation", "real"}:
            raise ValueError("metadata must identify data_source as simulation or real")
        if self.metadata["data_source"] == "simulation":
            if self.metadata.get("evidence") != "synthetic" or not self.metadata.get("run_id"):
                raise ValueError("simulation requires synthetic evidence and a scene run_id")
        self.metadata.setdefault("clock_domains", {
            "source_time": "receiver monotonic seconds",
            "scene_time": "simulation seconds" if self.metadata["data_source"] == "simulation" else "absent",
            "camera_times": "simulation seconds" if self.metadata["data_source"] == "simulation" else "device acquisition seconds; declared in camera provenance",
        })
        self.max_source_gap_s = float(self.metadata.setdefault("max_source_gap_s", .25))
        if not math.isfinite(self.max_source_gap_s) or self.max_source_gap_s <= 0:
            raise ValueError("max_source_gap_s must be positive and finite")
        _json(self.metadata)
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        identity = uuid.uuid4().hex
        self.path = directory / f"task_episode_{identity}.hdf5"
        self.partial_path = directory / f"task_episode_{identity}.partial.hdf5"
        self.hz = float(hz)
        self._queue = queue.Queue(maxsize=max_pending)
        self._lock = threading.Lock()
        self._error = None
        self._abort = None
        self._closed = False
        self._frames = 0
        self._accepted = 0
        self._complete = False
        self._last_time = None
        self._last_camera_times = None
        self._shapes = None
        self._last_scene_time = None
        self._thread = threading.Thread(target=self._run, daemon=True, name="task-episode-writer")
        self._thread.start()

    def append(self, *, q, qcmd, source_time, scene_time, images, camera_times,
               task_state, authority="human", torques=None):
        try:
            self._append(q=q, qcmd=qcmd, source_time=source_time, scene_time=scene_time,
                         images=images, camera_times=camera_times, task_state=task_state,
                         authority=authority, torques=torques)
        except Exception as exc:
            if not self._complete:
                self.abort(str(exc))
            raise

    def _append(self, *, q, qcmd, source_time, scene_time, images, camera_times,
                task_state, authority="human", torques=None):
        if self._closed:
            raise RuntimeError("episode is closed")
        if self._error:
            raise RuntimeError(self._error)
        if authority not in AUTHORITY:
            raise ValueError("authority must be human, policy or correction")
        if authority == "scripted_fixture" and (self.metadata["data_source"] != "simulation" or
                                                self.metadata.get("collection_mode") != "scripted_fixture"):
            raise ValueError("scripted_fixture authority requires explicitly labelled simulation fixture metadata")
        vectors = {}
        for key, value in (("qpos", q), ("qcmd", qcmd)):
            array = np.asarray(value, dtype=np.float32)
            if array.shape != (14,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{key} requires 14 finite motor coordinates")
            vectors[key] = array.copy()
        if torques is None:
            vectors["torques"] = np.full(14, np.nan, dtype=np.float32)
        else:
            array = np.asarray(torques, dtype=np.float32)
            if array.shape != (14,) or np.any(np.isinf(array)):
                raise ValueError("torques requires 14 numbers with NaN for unavailable readings")
            vectors["torques"] = array.copy()
        vectors["torque_valid"] = np.isfinite(vectors["torques"])
        if not isinstance(source_time, (int, float)) or not math.isfinite(source_time):
            raise ValueError("source_time must be finite seconds from the paired receiver snapshot")
        if self._last_time is not None and source_time <= self._last_time:
            raise ValueError("source_time must increase strictly; duplicate samples are not new observations")
        if self._last_time is not None and source_time-self._last_time > self.max_source_gap_s + 1e-8:
            raise ValueError("source sample gap exceeds capture bound; episode quarantined")
        if self.metadata["data_source"] == "simulation":
            if scene_time is None or not math.isfinite(scene_time):
                raise ValueError("simulation requires finite scene_time")
            if self._last_scene_time is not None and scene_time <= self._last_scene_time:
                raise ValueError("scene_time must increase strictly")
        elif scene_time is not None:
            raise ValueError("real capture must leave scene_time absent (None)")
        if set(images) != set(self.camera_roles) or set(camera_times) != set(self.camera_roles):
            raise ValueError("images and timestamps must match the declared camera_roles exactly")
        copied, stamps = {}, {}
        for name in self.camera_roles:
            array = np.asarray(images[name])
            if array.dtype != np.uint8 or array.ndim != 3 or array.shape[-1] != 3 or min(array.shape[:2]) <= 0:
                raise ValueError(f"{name} must be uint8 HxWx3 RGB")
            if array.nbytes > 1280*960*3:
                raise ValueError("camera frame exceeds 1280x960 capture memory bound")
            stamp = float(camera_times[name])
            if not math.isfinite(stamp):
                raise ValueError("camera timestamps must be finite")
            if self._last_camera_times is not None and stamp <= self._last_camera_times[name]:
                raise ValueError(f"{name} acquisition timestamp must increase; stale frames cannot be recorded")
            copied[name], stamps[name] = array.copy(), stamp
        shapes = {name: value.shape for name, value in copied.items()}
        if self._shapes is not None and shapes != self._shapes:
            raise ValueError("camera dimensions cannot change during an episode")
        task = _json(task_state)
        if len(task.encode()) > 256*1024:
            raise ValueError("per-frame task metadata exceeds bound")
        packet = dict(vectors, images=copied, camera_times=stamps, source_time=float(source_time),
                      scene_time=float(scene_time) if scene_time is not None else np.nan,
                      task_state=task, authority=authority)
        try:
            self._queue.put_nowait(packet)
        except queue.Full as exc:
            self._error = "capture queue overflow; episode quarantined instead of dropping frames"
            self._abort = self._error
            raise RuntimeError(self._error) from exc
        self._accepted += 1
        self._last_time, self._last_scene_time = source_time, scene_time
        self._last_camera_times, self._shapes = stamps, shapes

    def status(self):
        with self._lock:
            return {"active": not self._closed and self._error is None,
                    "state": "complete" if self._complete else ("failed" if self._error else ("closed" if self._closed else "recording")),
                    "frames": self._frames, "directory": str(self.path.parent),
                    "accepted_frames": self._accepted, "written_frames": self._frames,
                    "pending_frames": self._queue.qsize(), "complete": self._complete,
                    "path": str(self.path if self._complete else self.partial_path),
                    "error": self._error, "data_source": self.metadata["data_source"]}

    def finish(self):
        if self._complete:
            return self.path
        if not self._closed:
            self._closed = True
            if self._thread.is_alive():
                try:
                    self._queue.put(None, timeout=10)
                except queue.Full:
                    self._error = "writer did not drain in time; partial episode retained"
                    self._abort = self._error
        self._thread.join(timeout=30)
        if self._thread.is_alive():
            self.abort("writer shutdown timed out; partial episode retained")
            raise RuntimeError(self._error)
        if self._error:
            raise RuntimeError(self._error)
        if not self._complete:
            raise RuntimeError("episode was not finalized")
        return self.path

    def abort(self, reason):
        with self._lock:
            if self._complete:
                return
            self._abort = str(reason)
            self._error = self._abort
            self._closed = True
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        # The owner can signal failure without waiting for disk or compression.

    def _run(self):
        try:
            with h5py.File(self.partial_path, "x") as file:
                file.attrs.update(format_version=8, task_capture_schema=1, complete=False,
                                  hz=self.hz, minerva_joints=",".join(self.names), camera_roles=_json(list(self.camera_roles)), action_space="absolute",
                                  action_semantics="next paired receiver qcmd; action_valid excludes terminal and authority transitions",
                                  action_units="receiver motor radians; see joint map for model conversion",
                                  requires_action_valid_mask=True, data_source=self.metadata["data_source"],
                                  authority_scope="both_arms", metadata_json=_json(self.metadata),
                                  language_instruction=str(self.metadata.get("language_instruction", "")),
                                  collected_at=datetime.now(timezone.utc).isoformat())
                datasets = {}
                previous_authority = None
                eligible = self.metadata.get("collection_mode") != "scripted_fixture"
                while True:
                    packet = self._queue.get()
                    if self._abort:
                        raise RuntimeError(self._abort)
                    if packet is None:
                        break
                    values = {f"observations/{key}": packet[key] for key in ("qpos", "qcmd", "torques", "torque_valid")}
                    values.update({f"observations/images/{key}": value for key, value in packet["images"].items()})
                    values.update({f"timestamps/camera_{key}": np.float64(value) for key, value in packet["camera_times"].items()})
                    values.update({"timestamps/telem": np.float64(packet["source_time"]),
                                   "timestamps/scene": np.float64(packet["scene_time"]),
                                   "timestamps/scene_valid": np.bool_(math.isfinite(packet["scene_time"])),
                                   "actions": packet["qcmd"], "action_valid": np.bool_(False),
                                   "authority": packet["authority"], "task_state": packet["task_state"]})
                    if not datasets:
                        for name, value in values.items():
                            if isinstance(value, str):
                                datasets[name] = file.create_dataset(name, shape=(0,), maxshape=(None,),
                                                                     dtype=h5py.string_dtype(), chunks=(64,))
                            else:
                                array = np.asarray(value)
                                shape = array.shape
                                datasets[name] = file.create_dataset(name, shape=(0, *shape), maxshape=(None, *shape),
                                                                     dtype=array.dtype, chunks=(1, *shape),
                                                                     compression="gzip", compression_opts=1)
                    index = self._frames
                    for name, value in values.items():
                        datasets[name].resize(index+1, axis=0)
                        datasets[name][index] = value
                    if index:
                        datasets["actions"][index-1] = packet["qcmd"]
                        datasets["action_valid"][index-1] = previous_authority == packet["authority"]
                    previous_authority = packet["authority"]
                    state = json.loads(packet["task_state"])
                    eligible = eligible and bool(state.get("training_eligible", True))
                    self._frames += 1
                    if self._frames % 20 == 0:
                        file.flush()
                if self._frames < 2:
                    raise ValueError("episode needs at least two complete synchronized frames")
                file.attrs.update(complete=True, frame_count=self._frames,
                                  capture_eligible=eligible, training_promoted=False,
                                  sim2real_validated=False,
                                  final_task_state=packet["task_state"] if packet else datasets["task_state"][-1])
                file.flush()
            # Flush file content before making it discoverable under its final name.
            with self.partial_path.open("r+b") as stream:
                os.fsync(stream.fileno())
            # Abort and the publication commit share one fence. A stop/disconnect
            # arriving during flush/fsync must not publish an aborted episode.
            with self._lock:
                if self._abort:
                    raise RuntimeError(self._abort)
                os.replace(self.partial_path, self.path)
                self._complete = True
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            try:
                with h5py.File(self.partial_path, "a") as file:
                    file.attrs.update(complete=False, error=self._error)
            except Exception:
                pass
        finally:
            self._closed = True
