"""Control progress, error stops and cleanup ordering with a blocked recording disk."""
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.helm import teleop
from robot_platform.helm.recording import RecordingError, RecordingWriter

ROOT = Path(__file__).resolve().parents[2]


def core(tmp_path):
    result = teleop.TeleopCore({
        'teleop_profile': str(ROOT/'config/devices/minerva-teleop-direct-tuning.json'),
        'asset_id': 'minerva-production-01', 'route_edits': {}}, clock=lambda: 10.)
    result.phase = 'running'
    return result


def sample():
    return dict(recent=[[10., .1, .2, .3, .4]])


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
    assert np.frombuffer(data, dtype=np.float32).reshape(-1, 5).shape == (100, 5)
    assert all(sink.closed for sink in sinks.values())
    assert instance.phase == 'stopped' and instance.fault is None


@pytest.mark.parametrize('failure_at', ['callback', 'event', 'latency', 'flush', 'worker'])
def test_recording_failure_latches_and_never_interrupts_disable_callbacks(tmp_path, failure_at):
    instance = core(tmp_path)
    receiver = SimpleNamespace(directory=tmp_path, transport=SimpleNamespace(on_result=None))
    instance._start_recording(receiver, {'joints': [{'name': 'joint'}]})
    original = instance._recorder
    class BrokenStream:
        def write(self, data): raise RecordingError('incomplete injected recording')
        def flush(self): raise RecordingError('incomplete injected recording')
    class BrokenWriter:
        def check(self): raise RecordingError('incomplete injected recording')
        def close(self, **kwargs): original.close(**kwargs)
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
    instance._tick = lambda *args: pytest.fail('motion submitted after recording failure')
    with pytest.raises(RecordingError, match='incomplete injected'):
        instance.tick(None, 10.)
    # The same callback may execute while processing the disable response. It
    # must return normally even with an already failed/overflowed writer.
    instance._samples = BrokenStream()
    receiver.transport.on_result(sample())
    order = []
    def disable():
        receiver.transport.on_result(sample())
        order.append('disabled')
    instance.session = SimpleNamespace(disable=disable)
    instance._closers.append(lambda: order.append('receiver closed'))
    instance.close()
    assert order == ['disabled', 'receiver closed']
    assert instance.phase == 'fault' and 'incomplete injected' in instance.message
    assert not original._worker.is_alive()


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
    assert np.fromfile(tmp_path/'samples.f32', dtype=np.float32).reshape(-1, 5).shape == (2, 5)
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
