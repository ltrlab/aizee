"""Live cameras and episode writing in an owned interpreter, never the leader loop.

Only bounded scalar receiver snapshots cross the shared process mailbox. Camera
sockets, JPEG decoding and HDF5 stay in the child. No motor sockets are opened.
"""
from __future__ import annotations
import math
from pathlib import Path
import sys
import threading
import time

from robot_platform.data.capture_process import ProcessCapture, STATUS_TIMEOUT_S, _read, _write
from .task_capture import paired_receiver_sample, validate_config


class LiveTaskCaptureProcess(ProcessCapture):
    def __init__(self, directory, names, config, *, metadata=None):
        self.config, self.names = validate_config(config), tuple(names)
        expected = {f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)} | {'left_gripper', 'right_gripper'}
        if len(self.names) != 14 or set(self.names) != expected:
            raise ValueError('Live task capture needs the complete 14-axis Minerva roster')
        self.camera_roles = tuple(self.config['camera_roles'])
        self._record_requested = False
        super().__init__(dict(directory=str(Path(directory)), names=list(self.names), config=self.config,
                              metadata=dict(metadata or {})), worker_module=__name__,
                         source_timeout_s=self.config['max_receiver_age_s'], source_clock=time.perf_counter)
        with self._lock:
            self._state = dict(self._state, camera_roles=list(self.camera_roles))

    def health(self):
        result = super().health()
        sample = self._latest
        deadline = self._state.get('camera_valid_until')
        result['capture_ready'] = bool(result['capture_ready'] and sample and sample.get('command_valid')
                                       and deadline is not None and time.perf_counter() <= deadline)
        return result

    def snapshot(self):
        result = super().snapshot()
        now = time.perf_counter()
        for camera in result.get('cameras', {}).values():
            if not isinstance(camera, dict):
                continue
            deadline = camera.get('valid_until')
            if deadline is None or now > deadline:
                camera['fresh'] = False
            if deadline is not None:
                camera['age_s'] = now-(deadline-self.config['max_camera_age_s'])
        return result

    def receive(self, result, rows, *, dropped=0, started_at, received_at):
        sample = paired_receiver_sample(result, rows, self.names, self.config, dropped=dropped,
                                        started_at=started_at, received_at=received_at)
        if sample is None:
            return
        with self._lock:
            last = self._state.get('last_command') or {}
            recording = self._state.get('recording') or {}
            if ((last.get('action') == 'stop_record' and last.get('state') == 'complete'
                    and recording.get('complete')) or
                    (last.get('action') == 'start_record' and last.get('state') == 'failed')):
                self._record_requested = False
            recording_expected = self._record_requested or recording.get('active')
        # Latch a transient loss before the latest-only mailbox can overwrite it.
        if recording_expected and (sample.get('error') or not sample.get('command_valid')):
            self.abort(sample.get('error') or 'Receiver no longer has active emitted command authority')
            return
        self.submit(sample, source_time=sample.get('source_time'))

    def command(self, action, payload=None):
        if action not in ('start_record', 'stop_record') or payload:
            raise ValueError('Live capture supports start_record and stop_record only')
        result = super().command(action)
        if action == 'start_record':
            self._record_requested = True
        return result


def _sample(value):
    """Validate scalar JSON before allowing the child to treat it as a receiver snapshot."""
    if not isinstance(value, dict):
        raise ValueError('capture receiver sample must be an object')
    if value.get('error') is not None:
        if not isinstance(value['error'], str):
            raise ValueError('capture sample error must be text')
        return value
    for key in ('source_time', 'received_at', 'receiver_ms', 'clock_uncertainty_s'):
        if type(value.get(key)) not in (int, float) or not math.isfinite(value[key]):
            raise ValueError(f'capture sample {key} must be finite')
    for key in ('q', 'qcmd', 'receiver_feedback_ms'):
        row = value.get(key)
        if not isinstance(row, (list, tuple)) or len(row) != 14 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in row):
            raise ValueError(f'capture sample {key} requires 14 finite values')
    if type(value.get('command_valid')) is not bool:
        raise ValueError('capture sample command_valid must be boolean')
    if value['command_valid'] and (value.get('hardware_enabled') is not True
                                  or value.get('receiver_state') not in ('holding', 'executing')):
        raise ValueError('capture sample command validity disagrees with receiver authority')
    return value


def _worker(*, factory=None, input_stream=None, output_stream=None):
    """Injectable worker host. Production uses only the fixed LiveTaskCapture implementation."""
    if factory is None:
        # Warm import costs in this interpreter before camera readiness can be reported.
        import h5py  # noqa: F401
        import numpy  # noqa: F401
        from PIL import Image
        Image.init()
        from .task_capture import LiveTaskCapture
        factory = LiveTaskCapture
    source, sink = input_stream or sys.stdin.buffer, output_stream or sys.stdout.buffer
    view, publisher, watchdog = None, None, None
    stopped = threading.Event()
    last_parent = [time.monotonic()]
    ids = {}

    def publish():
        known_capabilities = None
        try:
            while not stopped.is_set():
                state = view.snapshot()
                capabilities = state.get('capabilities')
                if capabilities == known_capabilities:
                    state.pop('capabilities', None)
                else:
                    known_capabilities = capabilities
                command = state.get('last_command')
                if command and command.get('id') in ids:
                    command['id'] = ids[command['id']]
                _write(sink, state)
                stopped.wait(.1)
        except (OSError, ValueError, TypeError):
            view.abort('Parent status pipe closed')
            stopped.set()

    def watch_parent():
        while not stopped.wait(.1):
            if time.monotonic()-last_parent[0] > STATUS_TIMEOUT_S:
                view.abort('Parent capture heartbeat stalled')
                stopped.set()

    try:
        first = _read(source)
        config = first.get('config')
        if not isinstance(config, dict):
            raise ValueError('capture worker configuration must be an object')
        view = factory(**config)
        last_parent[0] = time.monotonic()
        publisher = threading.Thread(target=publish, daemon=True, name='live-capture-status')
        watchdog = threading.Thread(target=watch_parent, daemon=True, name='live-capture-parent')
        publisher.start()
        watchdog.start()
        while not stopped.is_set():
            packet = _read(source)
            last_parent[0] = time.monotonic()
            if packet.get('close'):
                break
            if packet.get('abort'):
                if not isinstance(packet['abort'], str):
                    raise ValueError('capture abort reason must be text')
                view.abort(packet['abort'])
                break
            sample = packet.get('sample')
            if sample is not None:
                view.receive_sample(_sample(sample))
            commands = packet.get('commands', [])
            if not isinstance(commands, list) or len(commands) > 16:
                raise ValueError('capture commands exceed the bounded queue')
            for command in commands:
                if not isinstance(command, (list, tuple)) or len(command) != 3:
                    raise ValueError('invalid capture command')
                identity, action, payload = command
                if not isinstance(identity, str) or len(identity) > 64 or not isinstance(payload, dict):
                    raise ValueError('invalid capture command identity or payload')
                accepted = view.command(action, payload)
                ids[accepted['id']] = identity
                if len(ids) > 32:
                    del ids[next(iter(ids))]
    except (EOFError, OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
        if view is not None:
            view.abort(f'Parent connection ended: {exc}')
    finally:
        if view is not None:
            # close cancels in-flight finish through the writer publication fence.
            view.close()
            stopped.set()
            if publisher is not None:
                publisher.join(timeout=1)
            if watchdog is not None:
                watchdog.join(timeout=1)
            try:
                _write(sink, view.snapshot())
            except (OSError, ValueError, TypeError):
                pass


if __name__ == '__main__':
    _worker()
