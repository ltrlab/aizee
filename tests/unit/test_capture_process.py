"""Bounded IPC, parent loss and authority cancellation; no motor or leader devices."""
from copy import deepcopy
from io import BytesIO
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time

import pytest

from robot_platform.data import capture_process as ipc
from robot_platform.helm.task_capture import LiveTaskCapture, paired_receiver_sample, validate_config
from robot_platform.helm.task_capture_process import LiveTaskCaptureProcess, _sample, _worker

NAMES = tuple(f'{side}_arm_j{k}' for side in ('left', 'right') for k in range(1, 7)) + ('left_gripper', 'right_gripper')


def config():
    return validate_config(dict(schema=1, camera_roles=['right_wrist', 'head'], cameras={
        role: dict(endpoint=f'tcp://127.0.0.1:{5900+i}', camera_id=role,
                   timestamp_clock='local_unix', timestamp_semantics='host_read_complete')
        for i, role in enumerate(('right_wrist', 'head'))}))


def receiver(state='executing', enabled=True):
    row = [100.]
    for i in range(14):
        row += [float(i), 0., 0., float(i)+.2]
    return dict(status=dict(now_ms=102., joint_names=list(NAMES), state=state), hardware_enabled=enabled,
                feedback=[dict(sampled_at_ms=100., fault=False) for _ in NAMES]), [row]


def wait_for(predicate, timeout=5):
    until = time.monotonic()+timeout
    while time.monotonic() < until:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError('condition did not become true')


class Chunks(BytesIO):
    def write(self, value):
        return super().write(value[:3])
    def read(self, count=-1):
        return super().read(min(count, 2))


def test_framed_json_handles_partial_reads_and_writes_without_pickle():
    pipe = Chunks()
    expected = {'sample': {'q': [1., 2.], 'source_time': 5.}, 'commands': []}
    ipc._write(pipe, expected)
    pipe.seek(0)
    assert ipc._read(pipe) == expected


@pytest.mark.parametrize('value', [b'[]', b'{"bad":NaN}', b'{', b'null'])
def test_invalid_scalar_or_non_object_json_is_rejected(value):
    with pytest.raises(ValueError):
        ipc._read(BytesIO(struct.pack('<I', len(value))+value))


def test_packet_limit_is_checked_before_payload_read():
    with pytest.raises(ValueError, match='size'):
        ipc._read(BytesIO(struct.pack('<I', ipc.MAX_PACKET+1)))


@pytest.fixture
def process_worker(monkeypatch, tmp_path):
    # Real child process/pipes. The fake worker owns no devices or scene models.
    script = tmp_path/'worker.py'
    script.write_text(r"""
import sys, time
from robot_platform.data.capture_process import _read, _write
p = _read(sys.stdin.buffer)['config']
mode = p.get('test_mode')
if mode == 'stderr':
    sys.stderr.write('diagnostic fixture\n' + 'x'*100000); sys.stderr.flush()
if mode != 'startup_stall':
    _write(sys.stdout.buffer, dict(available=mode != 'initializing', capture_ready=True, failed=False, error=None,
                                  recording=dict(active=False), task={}, capabilities={}))
while True:
    value = _read(sys.stdin.buffer)
    if value.get('close') or value.get('abort'): break
    if mode in ('stall', 'startup_stall'): continue
    commands = value.get('commands') or []
    status = dict(available=mode != 'initializing',capture_ready=True,failed=False,error=None,recording=dict(active=False),task={})
    if value.get('sample') is not None: status['task']['sample'] = value['sample']
    if commands:
        i,a,_ = commands[-1]; status['last_command'] = dict(id=i,action=a,state='complete')
    _write(sys.stdout.buffer,status)
""", encoding='utf-8')
    popen = subprocess.Popen
    monkeypatch.setattr(ipc.subprocess, 'Popen', lambda command, **kwargs:
                        popen([sys.executable, str(script)], **kwargs))
    children = []
    def create(mode='echo', **kwargs):
        child = ipc.ProcessCapture(dict(directory=str(tmp_path/'evidence'), test_mode=mode),
                                   worker_module='unused_fixture', **kwargs)
        children.append(child)
        return child
    yield create
    for child in children:
        child.close()


def test_process_latest_snapshot_command_ack_and_actual_source_age(process_worker):
    now = [5.]
    child = process_worker(source_timeout_s=.03, source_clock=lambda: now[0])
    wait_for(lambda: child.health()['available'])
    assert not child.health()['capture_ready']
    child.submit({'q': [1., 2.], 'clock': 4.99}, source_time=4.99)
    wait_for(lambda: child.snapshot()['task'].get('sample'))
    assert child.health()['source_age_s'] == pytest.approx(.01)
    assert child.health()['capture_ready']
    now[0] = 5.04
    assert not child.health()['capture_ready']
    assert child.health()['source_age_s'] == pytest.approx(.05)
    command = child.command('start_record')
    wait_for(lambda: (child.snapshot().get('last_command') or {}).get('state') == 'complete')
    assert child.snapshot()['last_command']['id'] == command['id']


@pytest.mark.parametrize('mode', ['stall', 'startup_stall', 'initializing'])
def test_status_and_startup_stalls_latch_failure_and_abort_child(process_worker, mode):
    child = process_worker(mode, startup_timeout_s=.15)
    wait_for(lambda: child.health()['failed'])
    assert not child.health()['capture_ready']
    wait_for(lambda: child._process.poll() is not None)
    assert child.snapshot()['failed'] and child.snapshot()['error']


def test_child_exit_is_a_latched_failure(process_worker):
    child = process_worker()
    wait_for(lambda: child.health()['available'])
    child._process.terminate()
    wait_for(lambda: child.health()['failed'])
    assert not child.health()['available']
    assert not child.snapshot()['capture_ready']


def test_stderr_is_saved_but_bounded_and_child_can_continue(process_worker):
    child = process_worker('stderr')
    wait_for(lambda: child.health()['available'])
    wait_for(lambda: child.stderr_path.exists() and child.stderr_path.stat().st_size >= ipc.STDERR_LIMIT)
    content = child.stderr_path.read_bytes()
    assert content.startswith(b'diagnostic fixture')
    assert len(content) < ipc.STDERR_LIMIT+100
    assert b'truncated' in content


@pytest.mark.parametrize('state,enabled,valid', [('holding', True, True), ('executing', True, True),
    ('disarmed', False, False), ('idle', True, False), ('stopping', True, False), ('faulted', True, False),
    ('holding', False, False), ('holding', 1, False)])
def test_receiver_command_validity_is_distinct_from_fresh_observation(state, enabled, valid):
    response, rows = receiver(state, enabled)
    sample = paired_receiver_sample(response, rows, NAMES, config(), started_at=10., received_at=10.004)
    assert sample['source_time'] == pytest.approx(9.998)
    assert sample['clock_uncertainty_s'] == pytest.approx(.004)
    assert sample['qcmd'][3] == pytest.approx(3.2)
    assert sample['command_valid'] is valid
    view = LiveTaskCapture.__new__(LiveTaskCapture)
    view.config = config()
    view._ready(sample, 10.)  # Fresh stopped observations remain diagnostic evidence.
    if not valid:
        with pytest.raises(ValueError, match='command authority'):
            view._ready(sample, 10., require_commands=True)


def test_process_latches_short_authority_loss_before_latest_mailbox_replacement(monkeypatch):
    child = LiveTaskCaptureProcess.__new__(LiveTaskCaptureProcess)
    child.names, child.config = NAMES, config()
    child._lock = threading.Lock()
    child._state = {'recording': {'active': False}, 'last_command': {'state': 'pending', 'action': 'start_record'}}
    child._record_requested = True
    received, aborted = [], []
    monkeypatch.setattr(child, 'submit', lambda *a, **k: received.append(a))
    monkeypatch.setattr(child, 'abort', aborted.append)
    response, rows = receiver('disarmed', False)
    child.receive(response, rows, started_at=10., received_at=10.004)
    assert aborted and not received


def test_child_rejects_forged_command_validity():
    response, rows = receiver('disarmed', False)
    value = paired_receiver_sample(response, rows, NAMES, config(), started_at=10., received_at=10.004)
    value['command_valid'] = True
    with pytest.raises(ValueError, match='disagrees'):
        _sample(value)


def test_parent_eof_aborts_real_writer_during_fsync_before_publication(monkeypatch, tmp_path):
    import numpy as np
    from robot_platform.data.task_capture import TaskEpisodeWriter
    # The same worker EOF path runs with an injectable live worker and a real writer.
    view = LiveTaskCapture.__new__(LiveTaskCapture)
    view._lock, view._stop = threading.Lock(), threading.Event()
    view._abort_reason, view._commands = None, __import__('queue').Queue()
    view._state = dict(available=True, capture_ready=False, failed=False, error=None,
                       recording={'active': True}, capabilities={}, task={}, last_command=None)
    writer = TaskEpisodeWriter(tmp_path, NAMES, {'data_source':'real','camera_roles':['right_wrist','head']})
    view._writer = writer
    frames = {role: np.zeros((6,8,3), dtype=np.uint8) for role in ('right_wrist','head')}
    for stamp in (10.,10.05):
        writer.append(q=[0.]*14,qcmd=[.1]*14,source_time=stamp,scene_time=None,images=frames,
                      camera_times={role:stamp for role in frames},task_state={},authority='human')
    in_fsync, resume_fsync = threading.Event(), threading.Event()
    real_fsync = os.fsync
    def blocked_fsync(fd):
        in_fsync.set()
        assert resume_fsync.wait(5)
        return real_fsync(fd)
    monkeypatch.setattr(os, 'fsync', blocked_fsync)
    finish_errors=[]
    def finish():
        try: writer.finish()
        except RuntimeError as exc: finish_errors.append(str(exc))
    view._thread = threading.Thread(target=finish,daemon=True)
    view.snapshot = lambda: dict(view._state, recording=writer.status())
    view._thread.start()
    assert in_fsync.wait(5)
    source=BytesIO(); ipc._write(source, {'config': {}}); source.seek(0)
    # EOF follows config exactly as it does when the owning process dies.
    host=threading.Thread(target=_worker,kwargs={'factory':lambda **k:view,
                         'input_stream':source,'output_stream':BytesIO()},daemon=True)
    host.start()
    wait_for(lambda: view._abort_reason)
    resume_fsync.set()
    host.join(timeout=5)
    assert not host.is_alive()
    assert not writer.path.exists()
    assert writer.partial_path.exists()
    assert not writer.status()['complete']
    assert writer.status()['error'] and finish_errors


def test_real_live_process_starts_without_fabricated_camera_readiness_and_exits_on_eof(tmp_path):
    child = LiveTaskCaptureProcess(tmp_path, NAMES, config())
    try:
        wait_for(lambda: child.health()['available'], timeout=10)
        assert not child.health()['capture_ready']
        assert child.snapshot()['camera_roles'] == ['right_wrist', 'head']
        child._process.stdin.close()  # Exact EOF signal from a disappeared parent.
        wait_for(lambda: child._process.poll() is not None)
        wait_for(lambda: child.health()['failed'])
        assert not list(tmp_path.glob('*.hdf5'))
    finally:
        child.close()


def test_submit_health_and_bounded_commands_never_wait_for_blocked_pipe(process_worker, monkeypatch):
    child = process_worker()
    wait_for(lambda: child.health()['available'])
    original = ipc._write
    entered, release, completed = threading.Event(), threading.Event(), threading.Event()
    def blocked(stream, packet):
        if packet.get('sample') is not None:
            entered.set()
            assert release.wait(5)
        return original(stream, packet)
    monkeypatch.setattr(ipc, '_write', blocked)
    child.submit({'q': [0.]}, source_time=1.)
    assert entered.wait(5)
    errors = []
    def producer():
        try:
            for value in range(1000):
                child.submit({'q': [float(value)]}, source_time=float(value))
                child.health()
            for _ in range(ipc.MAX_COMMANDS):
                child.command('reset')
            with pytest.raises(RuntimeError, match='queue is full'):
                child.command('reset')
        except BaseException as exc:
            errors.append(exc)
        completed.set()
    producer_thread=threading.Thread(target=producer,daemon=True)
    producer_thread.start()
    try:
        assert completed.wait(1), 'Control callback waited for blocked child I/O'
        assert not errors
        assert child._latest['q'] == [999.]
    finally:
        release.set()
        producer_thread.join(timeout=2)


def test_live_decode_worker_stall_cannot_hide_behind_fresh_receiver_samples():
    view = LiveTaskCapture.__new__(LiveTaskCapture)
    view._lock, view.config = threading.Lock(), config()
    view.clock = lambda: 10.
    view._sample = {'source_time': 9.999}
    view._last_progress, view._finalizing = 9., False
    view._state = dict(available=True, capture_ready=True, failed=False, error=None,
                       recording={'active':True})
    health = view.health()
    assert health['failed'] and not health['capture_ready']
    assert 'worker stalled' in health['error']


def test_live_camera_readiness_expires_before_worker_stall_timeout():
    view = LiveTaskCapture.__new__(LiveTaskCapture)
    view._lock, view.config = threading.Lock(), config()
    now = [10.]
    view.clock = lambda: now[0]
    view._sample = {'source_time': 9.999}
    view._last_progress, view._finalizing = 9.9, False
    view._state = dict(available=True, capture_ready=True, camera_valid_until=10.001,
                       failed=False, error=None, recording={'active':False})
    assert view.health()['capture_ready']
    now[0] = 10.01
    health = view.health()
    assert not health['capture_ready'] and not health['failed']
