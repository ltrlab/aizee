"""Control progress, guarded recording-fault holds and receiver-first cleanup."""
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.helm import teleop
from robot_platform.helm.recording import RecordingError, RecordingWriter
from robot_platform.teleop.lifecycle import TeleopLifecycle

ROOT = Path(__file__).resolve().parents[2]


def core(tmp_path):
    result = teleop.TeleopCore({
        'teleop_profile': str(ROOT/'config/devices/minerva-teleop-direct-tuning.json'),
        'asset_id': 'minerva-production-01', 'route_edits': {}}, clock=lambda: 10.)
    result.phase = 'running'
    return result


def sample():
    # One joint's recorder row: time, measured position, velocity, effort, commanded position, then the drive's
    # commanded velocity, effort, kp and kd.
    return dict(recent=[[10., .1, .2, .3, .4, .5, .6, 30., 2.]])


class BlockedSink:
    def __init__(self, path, started, release):
        self.path, self.started, self.release = path, started, release
        self.chunks, self.closed = [], False
    def write(self, data):
        if self.path.name == 'direct-latency.jsonl':
            self.started.set()
            assert self.release.wait(3.), 'test writer was not released'
        self.chunks.append(data)
        return len(data)
    def flush(self): pass
    def close(self): self.closed = True


def test_real_core_cycles_continue_while_latency_disk_write_is_blocked(monkeypatch, tmp_path):
    started, release, completed = threading.Event(), threading.Event(), threading.Event()
    sinks, failure = {}, []
    def opener(path, binary):
        sink = BlockedSink(path, started, release)
        sinks[path.name] = sink
        return sink
    monkeypatch.setattr(teleop, 'RecordingWriter', lambda directory: RecordingWriter(directory, opener=opener))
    instance = core(tmp_path)
    receiver = SimpleNamespace(directory=tmp_path, transport=SimpleNamespace(on_result=None))
    instance._start_recording(receiver, {'joints': [{'name': 'joint'}]})
    instance._pending_latency = {'index': -1}
    instance._flush_latency()
    assert started.wait(2.)
    sequence = []
    instance._tick = lambda key, at: (sequence.append(key), receiver.transport.on_result(sample()))
    def cycles():
        try:
            for index in range(100):
                instance._pending_latency = {'index': index}
                instance.tick(None, 10.)
        except Exception as exc:
            failure.append(exc)
        finally:
            completed.set()
    producer = threading.Thread(target=cycles)
    producer.start()
    try:
        assert completed.wait(1.), 'control cycles waited for recording I/O'
        assert not release.is_set() and not failure and len(sequence) == 100
    finally:
        release.set()
        producer.join(2.)
        instance.close()
    assert [json.loads(row)['index'] for row in sinks['direct-latency.jsonl'].chunks] == list(range(-1, 100))
    data = b''.join(sinks['samples.f32'].chunks)
    assert np.frombuffer(data, dtype=np.float32).reshape(-1, 9).shape == (100, 9)
    assert all(sink.closed for sink in sinks.values())
    assert instance.phase == 'stopped' and instance.fault is None


class LifecycleReceiver:
    """In-memory receiver boundary; use the real lifecycle and core dispatch.

    Native lease renewal and fourteen-axis authority are covered separately by
    test_helm_capture_failure. This fixture makes no hardware/timing claim.
    """
    def __init__(self, state, callback, order):
        self.callback, self.order = callback, order
        self.now, self.position, self.records = 10., .02, []
        self.blocked = None
        joint = dict(name='joint', lower=-2., upper=2., hold_kp=10., hold_kd=1.,
                     max_velocity=.5, align_speed=.1, shutdown_speed=.05,
                     max_lead=.2, tolerance=.005, settled_velocity=.01, zero_position=0.)
        self.lifecycle = TeleopLifecycle([joint], settle_time_s=0.)
        self.lifecycle.action('idle', self.feedback(), now=self.now)
        self.lifecycle.action('enable', self.feedback(), now=self.now)
        if state == 'TELEOP':
            self.lifecycle.action('engage', self.feedback(), target=self.feedback(), now=self.now)
            self.now += .05
            self.lifecycle.step(self.feedback(), target=self.feedback(), now=self.now)
        assert self.lifecycle.state == state
        self.enabled = True

    def feedback(self):
        return [dict(name='joint', position=self.position, velocity=0.,
                     sampled_at=self.now-.001, sequence=round(self.now*1000))]

    def cycle(self, *, action, target, started_at):
        self.now = started_at
        rows = self.feedback()
        proposal = (self.lifecycle.action(action, rows, target=target, now=self.now) if action
                    else self.lifecycle.step(rows, target=target, now=self.now))
        self.enabled = self.lifecycle.state not in ('DISABLED', 'IDLE')
        # Shutdown feedback tracks the bounded proposal, without sleeping or I/O.
        if self.lifecycle.state == 'SHUTDOWN':
            self.position = proposal['positions'][0]
        record = dict(action=action, target=target, proposal=proposal, feedback=rows,
                      started_at=started_at, completed_at=started_at+.001,
                      accepted=True, status={'state': 'holding' if self.enabled else 'disarmed'})
        self.records.append(record)
        self.callback(sample())  # Includes the response to H / shutdown disable.
        return record

    def disable(self):
        self.lifecycle.action('disable')
        self.enabled = False
        self.callback(sample())
        self.order.append('disabled')


@pytest.mark.parametrize('failure_at', ['callback', 'event', 'latency', 'flush', 'worker'])
@pytest.mark.parametrize('initial_state', ['HOLD', 'TELEOP'])
@pytest.mark.parametrize('ending', ['H', 'X', 'stop'])
def test_recording_failure_holds_until_operator_stop_and_preserves_cleanup(
        tmp_path, failure_at, initial_state, ending):
    instance = core(tmp_path)
    receiver = SimpleNamespace(directory=tmp_path, transport=SimpleNamespace(on_result=None))
    instance._start_recording(receiver, {'joints': [{'name': 'joint'}]})
    original = instance._recorder
    order = []
    session = instance.session = LifecycleReceiver(initial_state, receiver.transport.on_result, order)
    instance._leader_target = lambda *_: pytest.fail('failed session polled the leaders')
    original_close = original.close

    def drain(**kwargs):
        assert not session.enabled and order[-2:] == ['disabled', 'receiver closed']
        order.append('drain')
        original_close(**kwargs)
    original.close = drain
    instance._closers.append(lambda: order.append('receiver closed'))

    class BrokenStream:
        def write(self, data): raise RecordingError('incomplete injected recording')
        def flush(self): raise RecordingError('incomplete injected recording')
    class BrokenWriter:
        def check(self): raise RecordingError('incomplete injected recording')
        def close(self, **kwargs):
            original.close(**kwargs)
            self.check()  # A failed RecordingWriter also reports its error on close.

    try:
        if failure_at == 'callback':
            instance._samples = BrokenStream()
            receiver.transport.on_result(sample())
        elif failure_at == 'event':
            instance._events = BrokenStream()
            instance._event('test')
        elif failure_at == 'latency':
            instance._latency = BrokenStream()
            instance._pending_latency = {'index': 1}
            instance._flush_latency()
        elif failure_at == 'flush':
            instance._samples = BrokenStream()
            instance._note()
        else:
            instance._recorder = BrokenWriter()

        instance.tick('T', session.now+.01)  # Queued Engage must lose to the fault latch.
        assert instance.recording_fault == 'incomplete injected recording'
        assert instance.fault is None and instance.phase == 'running'
        assert session.lifecycle.state == 'HOLD' and session.enabled
        assert session.records[-1]['action'] == ('disengage' if initial_state == 'TELEOP' else None)
        assert instance._recording_hold_confirmed
        assert instance._samples is instance._events is instance._latency is None
        held = session.lifecycle.target
        for index in range(30):
            session.position = .02 + .001*(index % 5)  # Feedback drift must not recapture the hold.
            instance.tick(('I', 'E', 'T')[index % 3], session.now+.01)
            assert instance.fault is None and session.lifecycle.state == 'HOLD'
            assert session.enabled and session.lifecycle.target == held
            assert session.records[-1]['action'] is None and session.records[-1]['target'] is None
        assert len(session.records) == instance.sequence == 31
        assert all(row['accepted'] and row['proposal']['operation'] == 'move' for row in session.records)
        assert original._worker.is_alive() and not order  # No powered-tick drain/disable.
        with pytest.raises(ValueError, match='Recording failed'):
            instance.task_action('start_record')

        # Repeated writer errors in a receiver callback must not interrupt H or
        # controlled shutdown, nor escape later receiver-first resource cleanup.
        instance._samples = BrokenStream()
        if ending == 'stop':
            assert instance.land()
        instance.tick(ending if ending != 'stop' else None, session.now+.01)
        assert session.records[-1]['action'] == ('disable' if ending == 'H' else 'shutdown')
        assert session.lifecycle.state == ('DISABLED' if ending == 'H' else 'SHUTDOWN')
        for _ in range(100):
            if session.lifecycle.state == 'DISABLED':
                break
            instance.tick(None, session.now+.01)
            assert instance.fault is None
        assert session.lifecycle.state == 'DISABLED' and not session.enabled
        for key in ('I', 'E', 'T'):
            instance.tick(key, session.now+.01)
            assert session.lifecycle.state == 'DISABLED' and not session.enabled
            assert session.records[-1]['action'] is None
        assert instance.recording_fault == 'incomplete injected recording'
    finally:
        instance.close()
    assert order == ['disabled', 'receiver closed', 'drain']
    assert instance.phase == 'fault' and 'incomplete injected' in instance.message
    assert not original._worker.is_alive()
    receiver.transport.on_result(sample())  # Late callbacks cannot restart recording.


def test_drain_only_after_receiver_cleanup_even_when_disable_failed(tmp_path):
    instance = core(tmp_path)
    order = []
    def disable():
        order.append('disable attempted')
        raise RuntimeError('transport lost')
    def fault(reason): order.append('fault stop attempted')
    class Recorder:
        def close(self, *, timeout):
            assert order == ['disable attempted', 'fault stop attempted', 'receiver closed']
            assert timeout == 5.
            order.append('drain')
            raise RecordingError('Recording drain timed out; evidence incomplete')
    instance.session = SimpleNamespace(disable=disable, fault=fault)
    instance._closers.append(lambda: order.append('receiver closed'))
    instance._recorder = Recorder()
    instance.fault = 'original motor fault'
    instance.close()
    assert order[-1] == 'drain'
    assert instance.phase == 'fault' and instance.fault == 'original motor fault'
    assert 'evidence incomplete' in instance.message


def test_recording_drains_final_cycle_samples_and_closed_event(tmp_path):
    instance = core(tmp_path)
    receiver = SimpleNamespace(directory=tmp_path, transport=SimpleNamespace(on_result=None))
    instance._start_recording(receiver, {'joints': [{'name': 'joint'}]})
    receiver.transport.on_result(sample())
    instance._pending_latency = {'index': 17}
    instance.session = SimpleNamespace(disable=lambda: receiver.transport.on_result(sample()))
    instance.close()
    assert json.loads((tmp_path/'direct-latency.jsonl').read_text()) == {'index': 17}
    events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert events[0]['kind'] == 'session' and events[-1]['kind'] == 'closed'
    assert np.fromfile(tmp_path/'samples.f32', dtype=np.float32).reshape(-1, 9).shape == (2, 9)
    receiver.transport.on_result(sample())  # Detached recording never accepts late callbacks.
    instance.close()  # Idempotent cleanup.


def test_receiver_cleanup_failure_never_claims_confirmed_disable(tmp_path):
    instance = core(tmp_path)
    instance.live_arm = 'left'
    instance.session = SimpleNamespace(disable=lambda: None)
    def failed_close():
        raise RuntimeError('CAN link cleanup failed')
    instance._closers.append(failed_close)
    instance.close()
    assert instance.phase == 'fault'
    assert 'could not be confirmed' in instance.fault
    assert 'CAN link cleanup failed' in instance.message
    assert 'CAN link is down' not in instance.message


def test_failed_disable_and_receiver_teardown_never_wait_for_recording_drain(tmp_path):
    instance = core(tmp_path)
    instance.live_arm, instance.fault = 'left', 'primary motor fault'
    order = []

    def disable():
        order.append('disable attempted')
        raise RuntimeError('disable transport lost')

    def fault(reason):
        order.append('fault stop attempted')

    def close_receiver():
        order.append('receiver close attempted')
        raise RuntimeError('receiver teardown unconfirmed')

    class Recorder:
        def close(self, *, timeout):
            assert order == ['disable attempted', 'fault stop attempted', 'receiver close attempted']
            assert timeout == 0.
            order.append('nonblocking recorder close')
            raise RecordingError('Recording drain timed out after 0s; evidence incomplete')

    instance.session = SimpleNamespace(disable=disable, fault=fault)
    instance._closers.append(close_receiver)
    instance._recorder = Recorder()
    instance.close()
    assert order[-1] == 'nonblocking recorder close'
    assert instance.phase == 'fault' and instance.fault == 'primary motor fault'
    assert 'receiver teardown unconfirmed' in instance.message
    assert 'evidence incomplete' in instance.message
    assert 'CAN link is down' not in instance.message
