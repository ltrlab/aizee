from dataclasses import replace

import pytest

from robot_platform.teleop.leader_bridge import CachedLeaderSampleStale, LeaderSnapshotClient, LatestLeaderInputs, sample_reply
from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped, OpenRBInputs


PROFILE = dict(leaders={"left": dict(usb_serial="L"), "right": dict(usb_serial="R")})


class Inputs:
    pins = dict(left="L", right="R")
    sequence = 0

    def poll(self):
        self.sequence += 1
        return {name: LeaderSample((0.,) * 7, (0,) * 7, self.sequence, 1000., 1000.001,
                                   (0, 0, 1, 1), 100, 103) for name in self.pins}


def test_bridge_uses_age_plus_round_trip_not_unrelated_clock_values():
    board = Inputs()
    clock = [10.]
    def exchange(request):
        reply = sample_reply(request, board, "boot", clock=lambda: 1000.005)
        clock[0] += .012
        return reply
    client = LeaderSnapshotClient(exchange, PROFILE, clock=lambda: clock[0])
    sample = client.poll()["left"]
    assert sample.sampled_at == pytest.approx(9.995)
    assert sample.received_at - sample.sampled_at == pytest.approx(.017)
    assert client.poll()["left"].sequence == 2


@pytest.mark.parametrize("failure", ["skew", "late"])
def test_bridge_skips_late_or_skewed_samples_without_latching(failure):
    board, clock = Inputs(), [10.]
    corrupt = [False]
    def exchange(request):
        reply = sample_reply(request, board, "boot", clock=lambda: 1000.005)
        if corrupt[0]:
            if failure == "skew": reply["leaders"]["left"]["age_s"] += .025
            else: clock[0] += .06
        return reply
    client = LeaderSnapshotClient(exchange, PROFILE, clock=lambda: clock[0])
    first = client.poll()["left"].sequence
    corrupt[0] = True
    with pytest.raises(LeaderSampleSkipped): client.poll()
    assert client.fault is None and client.previous["left"].sequence == first
    corrupt[0] = False
    assert client.poll()["left"].sequence > first


@pytest.mark.parametrize("failure", ["nonce", "epoch", "serial", "missing", "sequence", "nan", "negative_age"])
def test_bridge_latches_malformed_replayed_or_changed_sources(failure):
    board, clock = Inputs(), [10.]
    corrupt = [False]
    def exchange(request):
        reply = sample_reply(request, board, "boot", clock=lambda: 1000.005)
        if corrupt[0]:
            if failure == "nonce": reply["nonce"] = "wrong"
            elif failure == "epoch": reply["epoch"] = "reboot"
            elif failure == "serial": reply["leaders"]["left"]["usb_serial"] = "R"
            elif failure == "missing": del reply["leaders"]["left"]
            elif failure == "sequence": reply["leaders"]["left"]["sequence"] = 1
            elif failure == "nan": reply["leaders"]["left"]["age_s"] = float("nan")
            elif failure == "negative_age": reply["leaders"]["left"]["age_s"] = -.001
        return reply
    client = LeaderSnapshotClient(exchange, PROFILE, clock=lambda: clock[0])
    client.poll()
    corrupt[0] = True
    with pytest.raises(ValueError): client.poll()
    corrupt[0] = False
    with pytest.raises(RuntimeError, match="faulted"): client.poll()


def test_read_only_bridge_does_not_accept_robot_or_setup_commands():
    for request in (dict(type="arm", nonce="a" * 32), dict(type="sample", nonce="a" * 32, center=1)):
        with pytest.raises(ValueError): sample_reply(request, Inputs(), "boot")


def test_multi_input_failure_closes_every_reader_and_never_reuses_other_arm():
    from concurrent.futures import ThreadPoolExecutor
    inputs = OpenRBInputs(PROFILE, clock=lambda: 10.)
    class Reader:
        closed = False
        def poll(self): raise TimeoutError("missing servo")
        def close(self): self.closed = True
    a, b = Reader(), Reader()
    inputs.readers = dict(left=a, right=b)
    inputs._pool = ThreadPoolExecutor(max_workers=2)
    with pytest.raises(TimeoutError): inputs.poll()
    assert a.closed and b.closed and not inputs.readers


def test_buffering_preserves_source_age_and_never_turns_cache_reads_into_new_samples():
    source = Inputs()
    source.max_age_s = .05
    clock = [1000.01]
    buffered = LatestLeaderInputs(source, clock=lambda: clock[0])
    buffered._samples = source.poll()
    first = buffered.poll()["left"]
    clock[0] = 1000.04
    second = buffered.poll()["left"]
    assert second.sequence == first.sequence and second.sampled_at == first.sampled_at
    clock[0] = 1000.051
    with pytest.raises(CachedLeaderSampleStale, match="stale"): buffered.poll()
    assert not buffered._done.is_set() and buffered._failure is None
    buffered._samples = {name: replace(sample, sampled_at=1000.05, received_at=1000.051)
                         for name, sample in buffered._samples.items()}
    assert buffered.poll()['left'].sampled_at == 1000.05
    buffered._failure = "servo missing"
    clock[0] = 1000.01
    with pytest.raises(RuntimeError, match="servo missing"): buffered.poll()


@pytest.mark.parametrize('stamp', [1001., float('nan'), float('inf')])
def test_cache_future_and_invalid_timestamps_are_not_transient_staleness(stamp):
    source = Inputs()
    source.max_age_s = .05
    buffered = LatestLeaderInputs(source, clock=lambda: 1000.01)
    buffered._samples = {name: replace(sample, sampled_at=stamp) for name, sample in source.poll().items()}
    with pytest.raises(ValueError) as error:
        buffered.poll()
    assert not isinstance(error.value, CachedLeaderSampleStale)
