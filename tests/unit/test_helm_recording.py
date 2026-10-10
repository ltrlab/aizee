"""Recording stalls/errors are deterministic and never enter the producer's I/O path."""
import threading
import time

import pytest

from robot_platform.helm.recording import RecordingError, RecordingWriter


class Sink:
    def __init__(self, name, journal, *, started=None, release=None, fail=None, short=False):
        self.name, self.journal = name, journal
        self.started, self.release, self.fail, self.short = started, release, fail, short
        self.chunks, self.closed = [], False

    def record(self, operation, data=None):
        self.journal.append((self.name, operation, data, threading.get_ident()))
        if operation == self.fail:
            raise OSError('injected disk failure')

    def write(self, data):
        self.record('write', data)
        if self.started is not None:
            self.started.set()
            assert self.release.wait(5.), 'test did not release blocked writer'
        self.chunks.append(data)
        return len(data) - 1 if self.short else len(data)

    def flush(self):
        self.record('flush')

    def close(self):
        self.record('close')
        self.closed = True


def wait_until(predicate, timeout=2.):
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, 'writer did not reach expected state'
        time.sleep(.002)


def blocked_writer(tmp_path, **bounds):
    journal, sinks = [], {}
    started, release = threading.Event(), threading.Event()

    def opener(path, binary):
        journal.append((path.name, 'open', binary, threading.get_ident()))
        sink = Sink(path.name, journal, started=started if path.name == 'samples.f32' else None,
                    release=release)
        sinks[path.name] = sink
        return sink

    writer = RecordingWriter(tmp_path, opener=opener, **bounds)
    stream = writer.stream('samples.f32', binary=True)
    return writer, stream, started, release, journal, sinks


def test_blocked_disk_does_not_block_producer_and_fifo_snapshots_mutable_chunks(tmp_path):
    writer, samples, started, release, journal, sinks = blocked_writer(tmp_path)
    producer_done, failure = threading.Event(), []
    mutable, backing = bytearray(b'original'), bytearray(b'view')
    view = memoryview(backing)
    main_thread = threading.get_ident()
    samples.write(b'first')
    assert started.wait(2.)

    def produce():
        try:
            events = writer.stream('events.jsonl')
            assert events.write('event\n') == 6
            samples.write(mutable)
            samples.write(view)
            events.flush()
            samples.flush()
            latency = writer.stream('direct-latency.jsonl')
            latency.write('latency\n')
        except Exception as exc:
            failure.append(exc)
        finally:
            producer_done.set()

    producer = threading.Thread(target=produce)
    producer.start()
    try:
        assert producer_done.wait(.5), 'producer waited for disk I/O'
        assert not failure and not release.is_set()
        assert writer.snapshot()['pending_chunks'] > 1
        mutable[:] = b'overwrit'
        backing[:] = b'xxxx'
    finally:
        release.set()
        producer.join(2.)
        writer.close(timeout=2.)
    assert sinks['samples.f32'].chunks == [b'first', b'original', b'view']
    operations = [(name, operation, data) for name, operation, data, _ in journal]
    assert operations[:11] == [
        ('samples.f32', 'open', True), ('samples.f32', 'write', b'first'),
        ('events.jsonl', 'open', False), ('events.jsonl', 'write', 'event\n'),
        ('samples.f32', 'write', b'original'), ('samples.f32', 'write', b'view'),
        ('events.jsonl', 'flush', None), ('samples.f32', 'flush', None),
        ('direct-latency.jsonl', 'open', False), ('direct-latency.jsonl', 'write', 'latency\n'),
        ('samples.f32', 'close', None)]
    assert len({thread for _, _, _, thread in journal}) == 1
    assert journal[0][3] not in (main_thread, producer.ident)
    assert all(sink.closed for sink in sinks.values())
    assert writer.snapshot()['pending_bytes'] == writer.snapshot()['pending_chunks'] == 0
    assert writer.snapshot()['closed'] and not writer._worker.is_alive()


@pytest.mark.parametrize('bound', ['bytes', 'chunks'])
def test_inflight_io_counts_toward_bounds_and_overflow_is_sticky_without_dropping_accepted_data(tmp_path, bound):
    writer, samples, started, release, _, sinks = blocked_writer(
        tmp_path, max_bytes=8 if bound == 'bytes' else 100, max_chunks=2 if bound == 'chunks' else 20)
    samples.write(b'first!')
    assert started.wait(2.)
    try:
        if bound == 'chunks':
            samples.flush()
        with pytest.raises(RecordingError, match='queue overflow'):
            samples.write(b'bad')
        with pytest.raises(RecordingError, match='queue overflow'):
            writer.check()
        with pytest.raises(RecordingError, match='queue overflow'):
            samples.write(b'')
        snapshot = writer.snapshot()
        assert snapshot['pending_bytes'] == 6
        assert snapshot['peak_chunks'] <= (2 if bound == 'chunks' else 20)
    finally:
        release.set()
        with pytest.raises(RecordingError, match='queue overflow'):
            writer.close(timeout=2.)
    assert sinks['samples.f32'].chunks == [b'first!']
    assert sinks['samples.f32'].closed


@pytest.mark.parametrize('operation', ['open', 'write', 'flush', 'close', 'short_write'])
def test_async_io_failure_is_reported_by_check_and_future_writes_and_close(tmp_path, operation):
    journal, sinks = [], []

    def opener(path, binary):
        if operation == 'open':
            raise OSError('injected open failure')
        sink = Sink(path.name, journal, fail=operation, short=operation == 'short_write')
        sinks.append(sink)
        return sink

    writer = RecordingWriter(tmp_path, opener=opener)
    stream = writer.stream('events.jsonl')
    try:
        if operation != 'open':
            stream.write('data\n')
            if operation in ('flush', 'close'):
                getattr(stream, operation)()
        wait_until(lambda: writer.snapshot()['error'] is not None)
        for action in (writer.check, lambda: stream.write('later')):
            with pytest.raises(RecordingError, match='Recording .* failed'):
                action()
    finally:
        with pytest.raises(RecordingError, match='Recording .* failed'):
            writer.close(timeout=2.)
    assert writer.snapshot()['closed'] and not writer._worker.is_alive()
    if operation not in ('open', 'close'):
        assert sinks[0].closed


def test_close_timeout_is_bounded_and_reports_incomplete_recording(tmp_path):
    writer, stream, started, release, _, sinks = blocked_writer(tmp_path)
    stream.write(b'pending')
    assert started.wait(2.)
    before = time.monotonic()
    try:
        with pytest.raises(RecordingError, match='drain timed out.*still pending'):
            writer.close(timeout=.02)
        assert time.monotonic() - before < .5
        assert not writer.snapshot()['closed'] and not sinks['samples.f32'].closed
        with pytest.raises(RecordingError, match='drain timed out'):
            stream.flush()
    finally:
        release.set()
        with pytest.raises(RecordingError, match='drain timed out'):
            writer.close(timeout=2.)
    assert sinks['samples.f32'].chunks == [b'pending'] and sinks['samples.f32'].closed


def test_real_stream_contents_append_flush_close_and_empty_files_are_preserved(tmp_path):
    (tmp_path/'events.jsonl').write_text('before\n', encoding='utf-8')
    writer = RecordingWriter(tmp_path)
    samples = writer.stream('samples.f32', binary=True)
    events = writer.stream('events.jsonl')
    writer.stream('empty.jsonl')
    samples.write(b'\x00\xff\x01')
    events.write('non-ASCII: \u00e9\U0001f916\n')
    events.flush()
    events.close()
    events.close()  # Producer close is idempotent and never joins.
    with pytest.raises(RecordingError, match='stream is closed'):
        events.write('after close')
    writer.close(timeout=2.)
    writer.close(timeout=2.)
    assert (tmp_path/'samples.f32').read_bytes() == b'\x00\xff\x01'
    assert (tmp_path/'events.jsonl').read_text(encoding='utf-8') == 'before\nnon-ASCII: \u00e9\U0001f916\n'
    assert (tmp_path/'empty.jsonl').read_bytes() == b''
    assert events.closed and samples.closed
    with pytest.raises(RecordingError, match='closing'):
        samples.write(b'after close')


def test_large_chunk_rejected_before_copy_and_bad_input_does_not_poison_writer(tmp_path):
    writer = RecordingWriter(tmp_path, max_bytes=4)
    stream = writer.stream('samples.f32', binary=True)
    with pytest.raises(TypeError):
        stream.write('not bytes')
    with pytest.raises(ValueError, match='basename'):
        writer.stream('../outside')
    writer.check()
    with pytest.raises(RecordingError, match='chunk needs 5 bytes'):
        stream.write(bytearray(b'12345'))
    with pytest.raises(RecordingError, match='chunk needs 5 bytes'):
        writer.close(timeout=2.)
