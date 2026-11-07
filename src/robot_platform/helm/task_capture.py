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
from robot_platform.data.roster import ARM_AXES, HEAD_AXES, HEAD_JOINTS

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
    size = value.get('recording_image_size')
    if size is not None and (not isinstance(size, list) or len(size) != 2
            or any(type(v) is not int for v in size)
            or not 0 < size[0] <= 1280 or not 0 < size[1] <= 960):
        raise ValueError('recording_image_size must be [width, height] within 1280x960, or null')
    result['recording_image_size'] = list(size) if size is not None else None
    result['head'] = validate_head(value.get('head'))
    return result


def validate_head(value):
    """The head's pose feed (neck.head_pose.ENDPOINT): every episode frame records the neck's two axes and the head
    camera's pose from it, so a capture without it is refused."""
    if not isinstance(value, dict):
        raise ValueError('Task capture records the head: configure its pose feed (head.endpoint, head.motors)')
    endpoint = urlparse(value.get('endpoint', ''))
    if (endpoint.scheme != 'tcp' or endpoint.hostname not in ('127.0.0.1', 'localhost')
            or endpoint.port is None or endpoint.path or endpoint.username):
        raise ValueError('head: the pose feed is local to the Jetson and needs a loopback endpoint')
    motors = value.get('motors')
    if list(motors or ()) != list(HEAD_AXES):
        raise ValueError(f'head: motors must be {list(HEAD_AXES)}, in the order the head service reports them')
    age = value.get('max_age_s', .1)
    if isinstance(age, bool) or not isinstance(age, (float, int)) or not math.isfinite(age) or not 0 < age <= .1:
        raise ValueError('head.max_age_s must be finite and in (0, 0.1]')
    return {'endpoint': value['endpoint'], 'motors': list(motors), 'max_age_s': float(age)}


class HeadCache:
    """The head's recent samples from its pose feed, on the capture's clock. Each sample is one head service row: the
    two servos' position, velocity, effort and command, the neck's CAD angles and the camera's pose. Like a camera
    frame, a sample is admitted only once received and never restamped."""

    def __init__(self, config):
        self.config = config
        self.samples = deque(maxlen=64)
        self.error = None

    def ingest(self, message, *, received_at, wall_time):
        import json
        try:
            data = json.loads(message)
            if data.get('motors') != self.config['motors']:
                raise ValueError('the head pose feed reports other motors than the capture profile')
            joints = list(data.get('joints') or ())
            if len(joints) != len(HEAD_JOINTS) or set(joints) != set(HEAD_JOINTS):
                raise ValueError(f'the head pose feed needs the {" and ".join(HEAD_JOINTS)} angles')
            samples, raw = data.get('samples') or [], data.get('raw') or []
            if len(samples) != len(raw):
                raise ValueError('the head pose feed rows and their motor readings disagree')
            width = 4*len(self.config['motors'])
            for sample, row in zip(samples, raw):
                if len(sample) != 1+len(joints)+7 or len(row) != width:
                    raise ValueError('a head pose feed row has the wrong width')
                values = [float(v) for v in (*sample, *row)]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('a head pose feed row is not finite')
                stamp = values[0]
                age = wall_time-stamp
                if not -.005 <= age <= 1.:
                    raise ValueError('a head pose sample is stale or in the future')
                if self.samples and stamp <= self.samples[-1]['publisher_time']:
                    continue  # Rows already seen never renew freshness.
                angles = dict(zip(joints, sample[1:1+len(joints)]))
                n = len(self.config['motors'])
                self.samples.append(dict(
                    publisher_time=stamp, sampled_at=received_at-max(age, 0.), received_at=received_at,
                    q=[row[4*i] for i in range(n)], qvel=[row[4*i+1] for i in range(n)],
                    effort=[row[4*i+2] for i in range(n)], qcmd=[row[4*i+3] for i in range(n)],
                    angles=[angles[j] for j in HEAD_JOINTS], camera_pose=list(sample[1+len(joints):])))
            self.error = None
            return True
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self.error = str(exc)
            return False

    def select(self, cutoff):
        """The newest head sample received by cutoff and within the head's age bound."""
        if self.error:
            raise ValueError(f'head: {self.error}')
        for sample in reversed(self.samples):
            if sample['received_at'] <= cutoff and sample['sampled_at'] <= cutoff:
                if cutoff-sample['sampled_at'] > self.config['max_age_s']:
                    break
                return sample
        raise ValueError('head: no fresh head pose sample at the observation cutoff (is a live head running?)')

    def status(self, now):
        row = self.samples[-1] if self.samples else None
        age = now-row['sampled_at'] if row else None
        return dict(fresh=bool(row and not self.error and 0 <= age <= self.config['max_age_s']), age_s=age,
                    error=self.error, publisher_time=row['publisher_time'] if row else None)


def recording_images(frames, size):
    """Transform only an admitted coherent set; preserve cached RGB and its clocks.

    This is spatial downsampling, not lossless image preservation. The episode
    stores the transformed RGB losslessly; no stored-image intrinsics are known.
    """
    import numpy as np
    import PIL
    from PIL import Image
    images, provenance = {}, {}
    for name, frame in frames.items():
        source = frame['image']
        height, width = source.shape[:2]
        target = tuple(size) if size is not None else (width, height)
        if frame.get('decoded_from') is not None:
            # CameraCache decoded this JPEG at the recording size, so there is nothing left to transform.
            if target != (width, height):
                raise ValueError(f'{name}: frame was decoded for a different recording size')
            images[name] = source
            provenance[name] = {k: v for k, v in frame.items() if k not in ('image', 'decoded_from')}
            provenance[name]['image_transform'] = dict(
                operation='jpeg_dct_scale', source_shape=list(frame['decoded_from']), stored_shape=list(source.shape),
                algorithm='Pillow.Image.draft', pillow_version=PIL.__version__, color_order='RGB',
                stored_image_intrinsics=None, factor=[frame['decoded_from'][1]//width, frame['decoded_from'][0]//height])
            continue
        if target[0] > width or target[1] > height or target[0]*height != target[1]*width:
            raise ValueError(f'{name}: recording_image_size must downsample without changing aspect ratio')
        changed = target != (width, height)
        integer_reduce = width == target[0]*2 and height == target[1]*2
        if changed:
            image = Image.fromarray(source)
            # Exact integer reduction avoids the general BOX filter overhead.
            # Its rounding differs from BOX, so record the actual algorithm.
            image = (image.reduce(2) if integer_reduce else
                     image.resize(target, getattr(Image, 'Resampling', Image).BOX))
            images[name] = np.asarray(image).copy()
        else:
            images[name] = source
        provenance[name] = {k: v for k, v in frame.items() if k != 'image'}
        provenance[name]['image_transform'] = dict(
            operation='integer_reduce' if integer_reduce else 'resize' if changed else 'identity',
            source_shape=list(source.shape), stored_shape=list(images[name].shape),
            algorithm='Pillow.Image.reduce' if integer_reduce else 'Pillow.BOX' if changed else 'identity',
            pillow_version=PIL.__version__, color_order='RGB', stored_image_intrinsics=None)
        if integer_reduce:
            provenance[name]['image_transform']['factor'] = [2, 2]
    return images, provenance


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
            rotation = msg.get('image_rotation_deg')
            if rotation is not None and (type(rotation) is not int or rotation not in (0, 180)):
                raise ValueError('publisher image_rotation_deg must be 0 or 180 when supplied')
            color = msg.get('color') or {}
            raw = color.get('data_bytes')
            if color.get('format') != 'jpeg' or not isinstance(raw, bytes) or not 0 < len(raw) <= 16 * 1024 * 1024:
                raise ValueError('camera needs a bounded JPEG payload')
            size = self.config['recording_image_size']
            with Image.open(BytesIO(raw)) as image:
                if image.size != (color.get('width'), color.get('height')):
                    raise ValueError('JPEG dimensions disagree with the camera header')
                if image.width * image.height > 4096 * 4096:
                    raise ValueError('camera image exceeds the capture size bound')
                source_shape = [image.height, image.width, 3]
                # Recording keeps only the configured size, so a JPEG of exactly twice that size is decoded straight
                # to it (the decoder's DCT scaling): a quarter of the decoding, and no resize left between pairing a
                # receiver sample and admitting the frame. Decoding full frames and reducing the recorded triplet
                # inside the 30 ms receiver bound failed every live recording within seconds (2026-10-19).
                if size is not None and image.size == (2*size[0], 2*size[1]):
                    image.draft('RGB', tuple(size))
                rgb = np.asarray(image.convert('RGB')).copy()
            rgb.setflags(write=False)
            self.sequence += 1
            frame = dict(image=rgb, publisher_time=float(stamp), sampled_at=received_at-age,
                         received_at=received_at, mapping_uncertainty_s=max(0., (mapped_at or received_at)-received_at),
                         receive_id=self.sequence, publisher_frame_number=msg.get('frame_number'),
                         image_rotation_deg=rotation,
                         timestamp_semantics=expected['timestamp_semantics'], exposure_time_known=False)
            if list(rgb.shape) != source_shape:
                frame['decoded_from'] = source_shape  # The JPEG's own shape, before the decoder scaled it.
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
                                image_rotation_deg=row['image_rotation_deg'] if row else None,
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
        if (type(now_ms) not in (int, float) or not math.isfinite(now_ms)
                or now_ms < 0 or now_ms != math.floor(now_ms)):
            raise ValueError('Receiver status clock must be a finite nonnegative integer millisecond')
        # Rust status floors elapsed milliseconds; its later recorder sample retains
        # fractions. Admit only the same half-open bucket, and use its upper bound
        # for every age/cutoff calculation rather than adding grace to the timeout.
        status_upper_ms = now_ms+1
        max_age_ms = config['max_receiver_age_s']*1000
        stamps = [f['sampled_at_ms'] for f in feedback]
        if any(type(stamp) not in (int, float) or not math.isfinite(stamp)
               or not 0 <= stamp <= now_ms or status_upper_ms-stamp > max_age_ms for stamp in stamps):
            raise ValueError('Required motor feedback is stale or in the future')
        row = rows[-1]
        if (type(row[0]) not in (int, float) or not math.isfinite(row[0])
                or not 0 <= row[0] < status_upper_ms or status_upper_ms-row[0] > max_age_ms):
            raise ValueError('Receiver recorder row is stale or in the future')
        # The receiver's recorder row: its ms, then per joint the measured position, velocity and effort (N m) and the
        # emitted command (control.servo_sweep.recorder_columns).
        n = len(names)
        q, qcmd = tuple(row[1+4*i] for i in range(n)), tuple(row[4+4*i] for i in range(n))
        qvel, effort = tuple(row[2+4*i] for i in range(n)), tuple(row[3+4*i] for i in range(n))
        if not all(math.isfinite(v) for v in q+qcmd):
            raise ValueError('Receiver positions or emitted commands are not finite')
        if not all(math.isfinite(v) for v in qvel+effort):
            raise ValueError('Receiver velocities or efforts are not finite')
        sampled_ms = min(row[0], *stamps)
        cutoff = started_at-(status_upper_ms-sampled_ms)/1000
        rpc_round_trip_s = received_at-started_at
        return dict(q=q, qcmd=qcmd, qvel=qvel, effort=effort, source_time=cutoff, received_at=received_at,
                            receiver_ms=row[0], receiver_feedback_ms=stamps,
                            receiver_status_ms=now_ms, receiver_status_upper_ms=status_upper_ms,
                            status_clock_quantization_s=.001, rpc_round_trip_s=rpc_round_trip_s,
                            clock_uncertainty_s=rpc_round_trip_s+.001, error=None,
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
        if len(self.names) != len(ARM_AXES) or set(self.names) != set(ARM_AXES):
            raise ValueError('Live task capture needs the complete 14-axis receiver roster of both arms')
        # Episodes record the whole robot: the arms' receiver joints, then the head's servos from its pose feed.
        self.roster = self.names + tuple(self.config['head']['motors'])
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
                           recording={'state': 'idle', 'active': False}, cameras={}, head={},
                           camera_roles=list(self.camera_roles),
                           capture_ready=False, camera_valid_until=None, error=None,
                           failed=False, last_command=None)
        self._thread = threading.Thread(target=self._run, daemon=True, name='helm-task-capture')
        self._thread.start()

    def receive(self, result, rows, *, dropped=0, started_at, received_at, label=None):
        """Snapshot paired receiver values; never decode images, perform I/O or send commands. `label`: who drives
        and the lifecycle state, for the frames this sample becomes (helm.teleop._capture_label)."""
        sample = paired_receiver_sample(result, rows, self.names, self.config, dropped=dropped,
                                        started_at=started_at, received_at=received_at)
        if sample is not None:
            if label:
                sample.update(authority=label.get('authority', 'human'), lifecycle=label.get('lifecycle'))
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
        opened_at = None  # When the episode now recording finished opening, until its first frame.
        try:
            factory = self.writer_factory
            if factory is None:
                from robot_platform.data.task_capture import TaskEpisodeWriter
                factory = TaskEpisodeWriter
            context = (self.context_factory or zmq.Context)()
            poller, cache, head = zmq.Poller(), CameraCache(self.config), HeadCache(self.config['head'])
            head_socket = context.socket(zmq.SUB)
            head_socket.setsockopt(zmq.LINGER, 0)
            head_socket.setsockopt(zmq.RCVHWM, 16)
            head_socket.setsockopt(zmq.MAXMSGSIZE, 1024*1024)
            head_socket.setsockopt(zmq.SUBSCRIBE, b'')
            head_socket.connect(self.config['head']['endpoint'])
            poller.register(head_socket, zmq.POLLIN)
            sockets['head_pose'] = head_socket
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
                        if socket is head_socket:
                            head.ingest(parts[-1], received_at=received, wall_time=wall)
                        else:
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
                            head.select(sample['source_time'])
                            writer = factory(self.directory, self.roster,
                                dict(self.metadata, capture_profile=self.config,
                                     clock='host_monotonic_seconds', camera_clock='mapped_same_host_unix',
                                     camera_selection='newest coherent received history within configured age and skew bounds',
                                     clock_domains={'source_time': 'conservative host mapping of paired receiver row',
                                                    'scene_time': 'absent',
                                                    'camera_times': 'mapped host read-complete stamps; sensor exposure time unknown',
                                                    'head_time': 'head service row time mapped from the Jetson wall clock'},
                                     head_selection='newest head pose feed row received by the observation cutoff, within head.max_age_s',
                                     action_semantics='receiver_emitted_position_at_sample',
                                     task_success='operator_review_required'), hz=self.config['hz'])
                            with self._lock:
                                self._writer = writer
                                cancelled = self._abort_reason
                            if cancelled:
                                writer.abort(cancelled)
                                raise RuntimeError(cancelled)
                            next_frame, last_sources = 0., None
                            # Opening the episode (an HDF5 file on the Jetson's SD card) can outlast the receiver
                            # sample read before it: the first frame then failed as expired, and so did every
                            # first recording (2026-10-19). The first frame waits for a sample from after the open.
                            opened_at = self.clock()
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
                if writer is not None and now >= next_frame and not (
                        opened_at is not None and sample is not None and not sample.get('error')
                        and sample['source_time'] < opened_at
                        and now - opened_at <= self.config['max_receiver_age_s']):
                    # Waiting for that sample never extends a sample's freshness: none arriving within the
                    # receiver age bound of the open fails here like any stalled receiver.
                    opened_at = None
                    self._ready(sample, now, require_commands=True)
                    frames = cache.select(sample['source_time'])
                    head_sample = head.select(sample['source_time'])
                    sources = (sample['source_time'], *(frames[n]['publisher_time'] for n in self.camera_roles))
                    # Wait for a new acquisition from every camera; never relabel a repeated frame as new. Until
                    # one comes, look again on the next poll (10 ms at most, sooner when a camera publishes), not a
                    # whole period later: the slowest camera (the head, ~15 fps) then paces a steady rate instead
                    # of gaps of one and two periods (demo 1, 2026-10-21).
                    if last_sources is None or all(a > b for a, b in zip(sources, last_sources)):
                        self._append_frame(writer, sample, frames, head_sample)
                        last_sources = sources
                        # The next tick on a grid from the first frame, not a period after this admission: the few
                        # milliseconds spent noticing each head frame never add up into a skipped one. A grid left
                        # more than a period behind restarts from now.
                        period = 1/self.config['hz']
                        next_frame = next_frame+period if now-next_frame < period else now+period
                with self._lock:
                    self._last_progress = self.clock()
                    self._state['available'] = True
                    self._state['cameras'] = cache.status(now)
                    for camera in self._state['cameras'].values():
                        camera['valid_until'] = (now+self.config['max_camera_age_s']-camera['age_s']
                                                  if camera['age_s'] is not None else None)
                    self._state['head'] = head.status(now)
                    fresh = all(row['fresh'] for row in self._state['cameras'].values())
                    try:
                        self._ready(sample, now, require_commands=True)
                        selected = cache.select(sample['source_time'])
                        head_selected = head.select(sample['source_time'])
                        self._state['camera_valid_until'] = min(
                            [frame['sampled_at'] + self.config['max_camera_age_s'] for frame in selected.values()]
                            + [head_selected['sampled_at'] + self.config['head']['max_age_s']])
                        self._state['capture_ready'] = True
                    except ValueError:
                        self._state['capture_ready'] = False
                        self._state['camera_valid_until'] = None
                    self._state['task'] = {'phase': 'recording' if writer else
                                           'ready for recording' if self._state['capture_ready'] else
                                           'waiting for the head pose (a live head)' if fresh and not self._state['head']['fresh'] else
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

    def _append_frame(self, writer, sample, frames, head):
        images, provenance = recording_images(frames, self.config['recording_image_size'])
        # Resizing may take time or overlap an authority-loss notification. Keep
        # the original paired sample and acquisitions; never restamp or replace q.
        now = self.clock()
        self._ready(sample, now, require_commands=True)
        if self._abort_reason:
            raise ValueError(self._abort_reason)
        for name, frame in frames.items():
            if not 0 <= now-frame['sampled_at'] <= self.config['max_camera_age_s']:
                raise ValueError(f'{name}: camera frame expired during recording preprocessing')
        if sample['source_time']-head['sampled_at'] > self.config['head']['max_age_s']:
            raise ValueError('head: the head pose sample is older than its bound at the observation cutoff')
        # Who drove the frame: a policy session's frames are the policy's, or a correction while the operator's
        # leaders drive. The lifecycle state lets the export leave out holds and alignments (data.lerobot_export).
        labels = ('q', 'qcmd', 'qvel', 'effort', 'authority', 'lifecycle')
        writer.append(q=tuple(sample['q'])+tuple(head['q']), qcmd=tuple(sample['qcmd'])+tuple(head['qcmd']),
                      torques=tuple(sample['effort'])+tuple(head['effort']),
                      velocities=tuple(sample['qvel'])+tuple(head['qvel']),
                      head=dict(time=head['sampled_at'], angles=head['angles'], camera_pose=head['camera_pose']),
                      source_time=sample['source_time'], scene_time=None,
                      images=images, camera_times={n: f['sampled_at'] for n, f in frames.items()},
                      task_state={'capture_admission_at': now, 'lifecycle': sample.get('lifecycle'),
                                  'capture': {k: v for k, v in sample.items() if k not in labels},
                                  'head': {'publisher_time': head['publisher_time'], 'received_at': head['received_at'],
                                           'age_at_cutoff_s': sample['source_time']-head['sampled_at']},
                                  'cameras': provenance, 'success': 'operator_review_required'},
                      authority=sample.get('authority', 'human'))

    def _ready(self, sample, now, *, require_commands=False):
        if sample is None or sample.get('error'):
            raise ValueError((sample or {}).get('error') or 'Waiting for complete receiver feedback')
        if not 0 <= now-sample['source_time'] <= self.config['max_receiver_age_s']:
            raise ValueError('Receiver sample expired before capture')
        if require_commands and not sample.get('command_valid'):
            raise ValueError('Receiver has no active emitted command authority')

    def close(self, *, timeout=5.):
        # A requested stop_record is the sole successful completion path. Closing
        # the owning session must win an in-progress finish before its rename.
        deadline = time.monotonic()+timeout
        with self._lock:
            unfinished = self._writer is not None or not self._commands.empty()
        if unfinished:
            self.abort('Capture session ended before explicit recording completion')
        self._stop.set()
        self._thread.join(timeout=max(0., deadline-time.monotonic()))
        if self._thread.is_alive():
            raise TimeoutError('Task capture worker did not close; partial episode integrity is unconfirmed')
        # The camera thread may have ended on error while its daemon writer is
        # still closing HDF5. Keep the reference and spend only the remaining
        # teardown budget; this path runs in the capture child, never a tick.
        with self._lock:
            writer = self._writer
        if writer is not None:
            writer.wait_closed(timeout=max(0., deadline-time.monotonic()))
            with self._lock:
                self._state['recording'] = writer.status()
                self._writer = None
