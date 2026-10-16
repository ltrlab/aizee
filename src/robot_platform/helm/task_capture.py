"""Read-only task capture beside Helm's existing whole-robot receiver.

Only this worker opens camera SUB sockets, decodes images and closes episode files.
The control callback publishes one immutable, paired receiver row. No motor or camera
control socket exists here. Same-host publisher clocks are an explicit profile claim.
"""
from collections import deque
from copy import deepcopy
from io import BytesIO
import math
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlparse

from robot_platform.transport.wire import unpack_camera
from robot_platform.data.capture_process import STATUS_TIMEOUT_S

CAMERAS = ('left_wrist', 'right_wrist', 'head')


def validate_config(value):
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('Task capture needs a schema 1 camera profile')
    roles = value.get('camera_roles')
    if (not isinstance(roles, list) or len(roles) not in (2, 3)
            or any(not isinstance(role, str) or role not in CAMERAS for role in roles)
            or len(set(roles)) != len(roles)):
        raise ValueError('Task capture requires an explicit list of two or three distinct supported camera_roles')
    cameras = value.get('cameras')
    if not isinstance(cameras, dict) or set(cameras) != set(roles):
        raise ValueError('Camera configuration must match the declared camera_roles exactly')
    result = deepcopy(value)
    for name, camera in cameras.items():
        if not isinstance(camera, dict):
            raise ValueError(f'{name}: camera endpoint, identity and clock are required')
        endpoint = urlparse(camera.get('endpoint', ''))
        if (endpoint.scheme != 'tcp' or endpoint.hostname not in ('127.0.0.1', 'localhost')
                or endpoint.port is None or endpoint.path or endpoint.username):
            raise ValueError(f'{name}: live capture runs on the Jetson and requires a loopback camera endpoint')
        if not isinstance(camera.get('camera_id'), str) or not camera['camera_id']:
            raise ValueError(f'{name}: explicit publisher camera_id required')
        if camera.get('timestamp_clock') != 'local_unix':
            raise ValueError(f'{name}: verify publisher timestamps use this Jetson clock before capture')
        if camera.get('timestamp_semantics') != 'host_read_complete':
            raise ValueError(f'{name}: declare the existing publisher host_read_complete timestamp semantics')
    for field, default, high in (('hz', 20., 50.), ('max_camera_age_s', .2, .5),
                                 ('max_receiver_age_s', .03, .03), ('max_camera_skew_s', .02, .02)):
        v = value.get(field, default)
        if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or not 0 < v <= high:
            raise ValueError(f'{field} must be finite and in (0, {high}]')
        result[field] = float(v)
    return result


class CameraCache:
    """Bounded decoded camera history; duplicate publisher stamps never renew freshness."""
    def __init__(self, config):
        self.config = validate_config(config)
        self.roles = tuple(self.config['camera_roles'])
        self.frames = {name: deque(maxlen=8) for name in self.roles}
        self.errors = {}
        self.sequence = 0

    def ingest(self, name, parts, *, received_at, wall_time, mapped_at=None):
        import numpy as np
        from PIL import Image
        try:
            msg = unpack_camera(parts)
            expected = self.config['cameras'][name]
            if msg.get('camera_id') != expected['camera_id']:
                raise ValueError('publisher camera_id does not match the configured role')
            stamp = msg.get('timestamp')
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError('publisher timestamp is missing or invalid')
            age = wall_time - stamp
            if not 0 <= age <= self.config['max_camera_age_s']:
                raise ValueError('publisher timestamp is stale or in the future')
            previous = self.frames[name][-1] if self.frames[name] else None
            if previous and stamp == previous['publisher_time']:
                return False
            if previous and stamp < previous['publisher_time']:
                raise ValueError('publisher clock moved backwards; restart capture after checking its clock')
            color = msg.get('color') or {}
            raw = color.get('data_bytes')
            if color.get('format') != 'jpeg' or not isinstance(raw, bytes) or not 0 < len(raw) <= 16 * 1024 * 1024:
                raise ValueError('camera needs a bounded JPEG payload')
            with Image.open(BytesIO(raw)) as image:
                if image.size != (color.get('width'), color.get('height')):
                    raise ValueError('JPEG dimensions disagree with the camera header')
                if image.width * image.height > 4096 * 4096:
                    raise ValueError('camera image exceeds the capture size bound')
                rgb = np.asarray(image.convert('RGB')).copy()
            rgb.setflags(write=False)
            self.sequence += 1
            frame = dict(image=rgb, publisher_time=float(stamp), sampled_at=received_at-age,
                         received_at=received_at, mapping_uncertainty_s=max(0., (mapped_at or received_at)-received_at),
                         receive_id=self.sequence, publisher_frame_number=msg.get('frame_number'),
                         timestamp_semantics=expected['timestamp_semantics'], exposure_time_known=False)
            self.frames[name].append(frame)
            self.errors.pop(name, None)
            return True
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.errors[name] = str(exc)
            return False

    def select(self, cutoff):
        """Newest coherent set already received by cutoff, within the existing age/skew limits.

        Publishers need not run at the same rate. The most recent wrist frame can
        be too new for the head even when a valid wrist frame is in bounded history.
        Never wait, synthesize a frame or move a timestamp to make a set admissible.
        """
        candidates = {}
        for name in self.roles:
            if name in self.errors:
                raise ValueError(f'{name}: {self.errors[name]}')
            candidates[name] = sorted((f for f in self.frames[name]
                if f['received_at'] <= cutoff and 0 <= cutoff-f['sampled_at'] <= self.config['max_camera_age_s']),
                key=lambda f: f['sampled_at'], reverse=True)
            if not candidates[name]:
                raise ValueError(f'{name}: no fresh camera frame available at the observation cutoff')
        # Descending lower edges maximize the oldest timestamp in the chosen set;
        # each role then takes its newest frame inside that window. At most 24
        # bounded windows are examined for the supported three-camera roster.
        lower_edges = sorted({f['sampled_at'] for rows in candidates.values() for f in rows}, reverse=True)
        for lower in lower_edges:
            chosen = {name: next((f for f in rows
                if 0 <= f['sampled_at']-lower <= self.config['max_camera_skew_s']), None)
                for name, rows in candidates.items()}
            if all(chosen.values()):
                return chosen
        raise ValueError('camera publisher timestamp skew exceeds the capture profile')

    def status(self, now):
        result = {}
        for name in self.roles:
            row = self.frames[name][-1] if self.frames[name] else None
            age = now-row['sampled_at'] if row else None
            result[name] = dict(fresh=bool(row and name not in self.errors and 0 <= age <= self.config['max_camera_age_s']),
                                age_s=age, error=self.errors.get(name),
                                publisher_time=row['publisher_time'] if row else None,
                                receive_id=row['receive_id'] if row else None)
        return result


def paired_receiver_sample(result, rows, names, config, *, dropped=0, started_at, received_at):
    """Bounded scalar snapshot shared by the thread worker and its process adapter."""
    try:
        if not rows or started_at is None:
            return
        if (type(started_at) not in (int, float) or type(received_at) not in (int, float)
                or not math.isfinite(started_at) or not math.isfinite(received_at) or received_at < started_at):
            raise ValueError('Receiver exchange timestamps are invalid')
        status = result.get('status') or {}
        feedback = result.get('feedback') or []
        if (dropped or len(feedback) != len(names) or status.get('set_aside')
                or tuple(status.get('joint_names', ())) != names
                or any(f.get('fault') is True for f in feedback)):
            raise ValueError('Receiver capture lost rows or a required motor is unavailable')
        now_ms = status['now_ms']
        stamps = [f['sampled_at_ms'] for f in feedback]
        if (not math.isfinite(now_ms) or any(not math.isfinite(stamp)
                or not 0 <= now_ms-stamp <= config['max_receiver_age_s']*1000 for stamp in stamps)):
            raise ValueError('Required motor feedback is stale')
        row = rows[-1]
        if not math.isfinite(row[0]) or not 0 <= now_ms-row[0] <= config['max_receiver_age_s']*1000:
            raise ValueError('Receiver recorder row is stale or in the future')
        q, qcmd = tuple(row[1+4*i] for i in range(14)), tuple(row[4+4*i] for i in range(14))
        if not all(math.isfinite(v) for v in q+qcmd):
            raise ValueError('Receiver positions or emitted commands are not finite')
        sampled_ms = min(row[0], *stamps)
        cutoff = started_at-max(0., now_ms-sampled_ms)/1000
        return dict(q=q, qcmd=qcmd, source_time=cutoff, received_at=received_at,
                            receiver_ms=row[0], receiver_feedback_ms=stamps,
                            clock_uncertainty_s=max(0., received_at-started_at), error=None,
                            command_valid=result.get('hardware_enabled') is True
                                and status.get('state') in ('holding', 'executing'),
                            hardware_enabled=result.get('hardware_enabled'), receiver_state=status.get('state'))
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        return dict(error=str(exc), received_at=received_at)


class LiveTaskCapture:
    def __init__(self, directory, names, config, *, metadata=None, clock=time.perf_counter,
                 wall_clock=time.time, writer_factory=None, context_factory=None):
        self.config = validate_config(config)
        self.names = tuple(names)
        self.camera_roles = tuple(self.config['camera_roles'])
        expected = {f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)} | {'left_gripper', 'right_gripper'}
        if len(self.names) != 14 or set(self.names) != expected:
            raise ValueError('Live task capture needs the complete 14-axis Minerva roster')
        self.directory, self.metadata = Path(directory), dict(metadata or {}, data_source='real', camera_roles=list(self.camera_roles))
        self.clock, self.wall_clock = clock, wall_clock
        self.writer_factory, self.context_factory = writer_factory, context_factory
        self._sample = None
        self._writer, self._abort_reason = None, None
        self._last_progress, self._finalizing = self.clock(), False
        self._commands = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._request = 0
        self._state = dict(available=False, capabilities={'source': 'real', 'contacts': 'unobserved',
                           'task_success': 'operator_review_required'}, task={'phase': 'camera preflight'},
                           recording={'state': 'idle', 'active': False}, cameras={}, camera_roles=list(self.camera_roles),
                           capture_ready=False, camera_valid_until=None, error=None,
                           failed=False, last_command=None)
        self._thread = threading.Thread(target=self._run, daemon=True, name='helm-task-capture')
        self._thread.start()

    def receive(self, result, rows, *, dropped=0, started_at, received_at):
        """Snapshot paired receiver values; never decode images, perform I/O or send commands."""
        sample = paired_receiver_sample(result, rows, self.names, self.config, dropped=dropped,
                                        started_at=started_at, received_at=received_at)
        if sample is not None:
            self.receive_sample(sample)

    def receive_sample(self, sample):
        self._sample = sample
        writer = getattr(self, '_writer', None)
        if writer is not None and (sample.get('error') or not sample.get('command_valid')):
            self.abort(sample.get('error') or 'Receiver no longer has active emitted command authority')

    def abort(self, reason):
        """Fence publication immediately, including while finish waits on disk, without joining."""
        with self._lock:
            self._abort_reason = self._abort_reason or str(reason)
            writer = self._writer
            self._state.update(error=self._abort_reason, failed=True, capture_ready=False)
        self._stop.set()
        if writer is not None:
            writer.abort(self._abort_reason)

    def command(self, action, payload=None):
        if action not in ('start_record', 'stop_record') or payload:
            raise ValueError('Live capture supports start_record and stop_record; physical scenes are reset by the operator')
        with self._lock:
            if self._stop.is_set() or self._state['failed']:
                raise ValueError(self._state['error'] or 'Task capture is closed')
            self._request += 1
            request = self._request
            try:
                self._commands.put_nowait((request, action))
            except queue.Full as exc:
                raise ValueError('Task command queue is full') from exc
            self._state['last_command'] = dict(id=request, action=action, state='pending')
        return {'accepted': True, 'id': request}

    def health(self):
        """Small control-loop status; never copy camera/UI diagnostics or touch worker I/O."""
        sample = self._sample
        age = self.clock()-sample['source_time'] if sample and not sample.get('error') else None
        with self._lock:
            recording = self._state['recording']
            progress = getattr(self, '_last_progress', None)
            finalizing = getattr(self, '_finalizing', False)
            stalled = bool(progress is not None and not finalizing
                           and self.clock()-progress > STATUS_TIMEOUT_S)
            camera_deadline = self._state.get('camera_valid_until')
            camera_expired = 'camera_valid_until' in self._state and (camera_deadline is None
                              or self.clock() > camera_deadline)
            return dict(available=self._state['available'] and not stalled,
                        error='Capture camera worker stalled' if stalled else self._state['error'],
                        failed=self._state['failed'] or stalled, source_age_s=age, camera_valid_until=camera_deadline,
                        capture_ready=bool(self._state['capture_ready'] and not camera_expired
                            and not stalled and not finalizing and age is not None
                            and 0 <= age <= self.config['max_receiver_age_s']),
                        recording={key: recording.get(key) for key in ('active', 'error', 'state', 'complete')})

    def snapshot(self):
        with self._lock:
            state = dict(self._state)
        result = deepcopy(state)
        result.update({key: value for key, value in self.health().items() if key != 'recording'})
        return result

    def _run(self):
        import zmq
        context = None
        sockets, writer = {}, None
        last_sources = None
        next_frame = 0.
        try:
            factory = self.writer_factory
            if factory is None:
                from robot_platform.data.task_capture import TaskEpisodeWriter
                factory = TaskEpisodeWriter
            context = (self.context_factory or zmq.Context)()
            poller, cache = zmq.Poller(), CameraCache(self.config)
            for name, camera in self.config['cameras'].items():
                socket = context.socket(zmq.SUB)
                socket.setsockopt(zmq.LINGER, 0)
                socket.setsockopt(zmq.RCVHWM, 2)
                socket.setsockopt(zmq.MAXMSGSIZE, 16*1024*1024)
                socket.setsockopt(zmq.SUBSCRIBE, b'')
                socket.connect(camera['endpoint'])
                poller.register(socket, zmq.POLLIN)
                sockets[name] = socket
            while not self._stop.is_set():
                events = dict(poller.poll(10))
                for name, socket in sockets.items():
                    if socket not in events:
                        continue
                    for _ in range(8):
                        try:
                            parts = socket.recv_multipart(zmq.NOBLOCK)
                        except zmq.Again:
                            break
                        received = self.clock()
                        wall = self.wall_clock()
                        cache.ingest(name, parts, received_at=received, wall_time=wall, mapped_at=self.clock())
                now, sample = self.clock(), self._sample
                try:
                    request, action = self._commands.get_nowait()
                except queue.Empty:
                    request = action = None
                if action is not None:
                    error = None
                    try:
                        if action == 'start_record':
                            if writer is not None:
                                raise ValueError('An episode is already recording')
                            self._ready(sample, now, require_commands=True)
                            cache.select(sample['source_time'])
                            writer = factory(self.directory, self.names,
                                dict(self.metadata, capture_profile=self.config,
                                     clock='host_monotonic_seconds', camera_clock='mapped_same_host_unix',
                                     camera_selection='newest coherent received history within configured age and skew bounds',
                                     clock_domains={'source_time': 'conservative host mapping of paired receiver row',
                                                    'scene_time': 'absent',
                                                    'camera_times': 'mapped host read-complete stamps; sensor exposure time unknown'},
                                     action_semantics='receiver_emitted_position_at_sample',
                                     task_success='operator_review_required'), hz=self.config['hz'])
                            with self._lock:
                                self._writer = writer
                                cancelled = self._abort_reason
                            if cancelled:
                                writer.abort(cancelled)
                                raise RuntimeError(cancelled)
                            next_frame, last_sources = 0., None
                        else:
                            if writer is None:
                                raise ValueError('No episode is recording')
                            with self._lock:
                                self._finalizing = True
                            try:
                                writer.finish()
                            finally:
                                with self._lock:
                                    self._finalizing = False
                            with self._lock:
                                self._state['recording'] = writer.status()
                                self._writer = None
                            writer = None
                    except Exception as exc:
                        error = str(exc)
                    with self._lock:
                        self._state.update(error=error, last_command=dict(id=request, action=action,
                                           state='failed' if error else 'complete', error=error))
                if writer is not None and now >= next_frame:
                    self._ready(sample, now, require_commands=True)
                    frames = cache.select(sample['source_time'])
                    sources = (sample['source_time'], *(frames[n]['publisher_time'] for n in self.camera_roles))
                    # Wait for a new acquisition from every camera; never relabel a repeated frame as new.
                    if last_sources is None or all(a > b for a, b in zip(sources, last_sources)):
                        provenance = {n: {k: v for k, v in f.items() if k != 'image'} for n, f in frames.items()}
                        writer.append(q=sample['q'], qcmd=sample['qcmd'], source_time=sample['source_time'], scene_time=None,
                                      images={n: f['image'] for n, f in frames.items()},
                                      camera_times={n: f['sampled_at'] for n, f in frames.items()},
                                      task_state={'capture': {k: v for k, v in sample.items() if k not in ('q', 'qcmd')},
                                                  'cameras': provenance, 'success': 'operator_review_required'},
                                      authority='human')
                        last_sources = sources
                    next_frame = now+1/self.config['hz']
                with self._lock:
                    self._last_progress = self.clock()
                    self._state['available'] = True
                    self._state['cameras'] = cache.status(now)
                    for camera in self._state['cameras'].values():
                        camera['valid_until'] = (now+self.config['max_camera_age_s']-camera['age_s']
                                                  if camera['age_s'] is not None else None)
                    fresh = all(row['fresh'] for row in self._state['cameras'].values())
                    try:
                        self._ready(sample, now, require_commands=True)
                        selected = cache.select(sample['source_time'])
                        self._state['camera_valid_until'] = min(frame['sampled_at']
                            + self.config['max_camera_age_s'] for frame in selected.values())
                        self._state['capture_ready'] = True
                    except ValueError:
                        self._state['capture_ready'] = False
                        self._state['camera_valid_until'] = None
                    self._state['task'] = {'phase': 'recording' if writer else
                                           'ready for recording' if self._state['capture_ready'] else
                                           'waiting for fresh paired receiver and camera timing' if fresh else 'waiting for cameras'}
                    if writer is not None:
                        self._state['recording'] = writer.status()
                        if self._state['recording'].get('error'):
                            raise RuntimeError(self._state['recording']['error'])
        except Exception as exc:
            if writer is not None:
                writer.abort(str(exc))
            with self._lock:
                self._state.update(error=str(exc), failed=True)
                if writer is not None:
                    self._state['recording'] = writer.status()
        finally:
            if writer is not None and writer.status().get('active'):
                writer.abort('Session ended before explicit recording completion')
            for socket in sockets.values():
                socket.close(linger=0)
            if context is not None:
                context.term()
            with self._lock:
                self._state['available'] = False
                self._state['capture_ready'] = False
                if writer is not None:
                    self._state['recording'] = writer.status()

    def _ready(self, sample, now, *, require_commands=False):
        if sample is None or sample.get('error'):
            raise ValueError((sample or {}).get('error') or 'Waiting for complete receiver feedback')
        if not 0 <= now-sample['source_time'] <= self.config['max_receiver_age_s']:
            raise ValueError('Receiver sample expired before capture')
        if require_commands and not sample.get('command_valid'):
            raise ValueError('Receiver has no active emitted command authority')

    def close(self):
        # A requested stop_record is the sole successful completion path. Closing
        # the owning session must win an in-progress finish before its rename.
        with self._lock:
            unfinished = self._writer is not None or not self._commands.empty()
        if unfinished:
            self.abort('Capture session ended before explicit recording completion')
        self._stop.set()
        self._thread.join(timeout=5.)
        if self._thread.is_alive():
            raise RuntimeError('Task capture worker did not close; episode completion is unconfirmed')
