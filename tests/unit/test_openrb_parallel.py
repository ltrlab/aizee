"""Independent buses overlap, but partial/stale pairs never reach a consumer."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import threading

import pytest

from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped, OpenRBInputs


def pair(barrier, *, failure=None, stale=False):
    group = OpenRBInputs({'leaders': {'left': {'usb_serial': 'L'}, 'right': {'usb_serial': 'R'}}}, clock=lambda: 10.)
    base = LeaderSample((0.,) * 7, (0,) * 7, 1, 9.995, 10., (0, 0, 1, 1), 1, 2)
    class Reader:
        def __init__(self, name):
            self.name, self.closed, self.finished = name, False, False
        def poll(self):
            barrier.wait(timeout=2.)  # Sequential polling cannot pass this test.
            self.finished = True
            if self.name == failure:
                raise ValueError('incomplete board read')
            return replace(base, sampled_at=9.8) if stale and self.name == 'left' else base
        def close(self):
            self.closed = True
    readers = {name: Reader(name) for name in group.pins}
    group.readers = readers
    group._pool = ThreadPoolExecutor(max_workers=2)
    return group, readers


def test_two_boards_read_concurrently_without_changing_timestamps():
    group, readers = pair(threading.Barrier(2))
    try:
        result = group.poll()
        assert set(result) == {'left', 'right'}
        assert all(s.sampled_at == 9.995 and s.sequence == 1 for s in result.values())
    finally:
        group.close()
    assert all(r.finished and r.closed for r in readers.values())


def test_partial_pair_closes_both_boards_after_reads_complete():
    group, readers = pair(threading.Barrier(2), failure='left')
    with pytest.raises(ValueError):
        group.poll()
    assert all(r.finished and r.closed for r in readers.values())
    assert group.readers == {} and group._pool is None
    with pytest.raises(RuntimeError):
        group.poll()


def test_late_or_skewed_complete_pair_is_skipped_without_closing_either_board():
    group, readers = pair(threading.Barrier(2), stale=True)
    try:
        with pytest.raises(LeaderSampleSkipped, match='age or skew'):
            group.poll()
        assert all(r.finished and not r.closed for r in readers.values())
        assert set(group.readers) == {'left', 'right'} and group._pool is not None
    finally:
        group.close()


def test_skipped_board_read_never_masks_a_hard_failure_on_the_other_board():
    group = OpenRBInputs({'leaders': {'left': {'usb_serial': 'L'}, 'right': {'usb_serial': 'R'}}}, clock=lambda: 10.)
    class Reader:
        closed = False
        def __init__(self, error): self.error = error
        def poll(self): raise self.error
        def close(self): self.closed = True
    readers = dict(left=Reader(LeaderSampleSkipped('late')), right=Reader(ValueError('bad CRC')))
    group.readers, group._pool = dict(readers), ThreadPoolExecutor(max_workers=2)
    with pytest.raises(ValueError, match='bad CRC'):
        group.poll()
    assert all(r.closed for r in readers.values()) and group.readers == {}
