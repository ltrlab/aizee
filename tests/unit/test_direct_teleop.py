"""Immediate references preserve acceleration, freshness and trace meaning."""
from dataclasses import replace

import pytest

from robot_platform.teleop.absolute_mapping import AbsoluteLeaderMapper
from robot_platform.teleop.latency import summarize
from robot_platform.teleop.leader_bridge import LeaderSnapshotClient, sample_reply
from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped
from robot_platform.teleop.lifecycle import TeleopLifecycle


def test_zero_interpolation_keeps_acceleration_and_velocity_feedforward():
    joint = dict(name='q', lower=-2., upper=2., hold_kp=10., hold_kd=1., max_velocity=1.,
                 align_speed=.1, shutdown_speed=.1, max_lead=.2, tolerance=.01,
                 settled_velocity=.05, zero_position=0., interpolation_s=0., direct_velocity=True,
                 approach=dict(lower=-2., upper=2., max_velocity=1.25, max_tracking_error=.3,
                               stop_acceleration=2., reaction_s=.04))
    life = TeleopLifecycle([joint], settle_time_s=0.)
    def rows(t, q=0.):
        return [dict(name='q', position=q, velocity=0., sequence=round(t*1000)+1, sampled_at=t)]
    life.action('idle', rows(0.), now=0.)
    life.action('enable', rows(0.), now=0.)
    life.action('engage', rows(0.), target=rows(0.), now=0.)
    life.step(rows(.05), target=rows(.05), now=.05)
    assert life.state == 'TELEOP'
    first = life.step(rows(.06), target=rows(.06, 1.), now=.06)
    assert first['positions'][0] == pytest.approx(2 * .01**2)
    assert first['joints'][0]['velocity'] == pytest.approx(.02)
    assert 'acceleration' in first['limits']['q']
    hold = life.step(rows(.07), target=None, now=.07)
    assert hold['positions'] == first['positions'] and hold['joints'][0]['velocity'] == 0


def test_mapper_repeated_samples_hold_without_new_age_and_changed_duplicates_fault():
    mapper = AbsoluteLeaderMapper([dict(name='q', lower=-2., upper=2.)],
        [dict(actuator='q', leader='left', channel=0, direction=1, raw_min=-1., raw_max=1.,
              follower_min=-2., follower_max=2.)])
    sample = LeaderSample((0.,)*7, (0,)*7, 1, 10., 10., (0,0,0,0), 1, 2)
    assert mapper.map({'left': sample}, now=10., allow_repeated=True)
    assert mapper.map({'left': sample}, now=10.01, allow_repeated=True) is None
    assert mapper.fault is None and mapper.previous['left'].sampled_at == 10.
    with pytest.raises(ValueError, match='without a new sequence'):
        mapper.map({'left': replace(sample, sampled_at=10.01)}, now=10.01, allow_repeated=True)


def test_remote_duplicate_preserves_original_age_and_still_validates_payload():
    sample = LeaderSample((0.,)*7, (0,)*7, 1, 100., 100., (0,0,0,0), 1, 2)
    class Source:
        pins = {'left': 'L'}
        def poll(self): return {'left': sample}
    local, remote = [10.], [100.005]
    def exchange(request):
        return sample_reply(request, Source(), 'boot', clock=lambda: remote[0])
    client = LeaderSnapshotClient(exchange, {'leaders': {'left': {'usb_serial':'L'}}},
                                  clock=lambda: local[0], allow_repeated=True)
    original = client.poll()['left']
    local[0] += .01
    remote[0] += .01
    assert client.poll()['left'] == original
    local[0] += .06
    remote[0] += .06
    with pytest.raises(LeaderSampleSkipped): client.poll()
    assert client.fault is None


def test_trace_does_not_confuse_acknowledgement_with_application():
    record = dict(operation='move', accepted=True, cycle_ms=2., rpc_ms=1., rpc_count=1,
                  source_sequence=[10], source_age_ms=5., receiver_accepted_ms=100, request_sequence=8,
                  receiver_applied=dict(sequence=7, accepted_at_ms=90, applied_at_ms=93))
    previous = dict(record, receiver_accepted_ms=90, request_sequence=7, receiver_applied=None)
    report = summarize([previous, record], .01)
    assert report['receiver_apply_ms']['p99'] == 3.
    assert report['processing_deadline_misses'] == 0
    assert report['maximum_rpc_count'] == 1 and report['unique_source_samples'] == 1


def test_live_timing_includes_previous_trace_write_and_ui_and_counts_late_completion(monkeypatch):
    import io
    import json
    from robot_platform.helm.teleop import TeleopCore, run_loop
    from tests.unit.test_helm_teleop import FakeCore

    now, ticks = [10.], [0]
    class SlowTrace(io.StringIO):
        def write(self, value):
            now[0] += .003
            return super().write(value)
    trace = SlowTrace()
    class Core(FakeCore):
        collision_mode = 'disabled'
        _record = TeleopCore._record
        _flush_latency = TeleopCore._flush_latency
        finish_cycle = TeleopCore.finish_cycle
        clock = staticmethod(lambda: now[0])
        def __init__(self):
            super().__init__()
            self._latency, self._pending_latency, self._recorded = trace, None, 0
            self._input_timing, self.cycles, self._loop_timings = {}, [], []
            self._loop_max_ms = self._loop_misses = self._loop_skipped = 0
            self.slowest_ms = self.overruns = 0
        def tick(self, key, started):
            self._flush_latency()
            now[0] += .002
            ticks[0] += 1
            self._record(dict(started_at=started, completed_at=now[0], accepted=True))
        def view(self):
            now[0] += .003
            return super().view()
        def guide(self):
            now[0] += .001
            return None
        def close(self):
            self._flush_latency()
            super().close()
    class Pipe:
        def poll(self): return ticks[0] >= 2
        def recv(self): return ('stop',)
        def send(self, value): pass
        def close(self): pass
    def late_wakeup(duration):
        now[0] += duration + .004
    monkeypatch.setattr('robot_platform.helm.teleop.time.sleep', late_wakeup)
    core = Core()
    run_loop(Pipe(), Pipe(), core)
    records = [json.loads(line) for line in trace.getvalue().splitlines()]
    assert core.fault is None and len(records) == 2
    assert records[0]['full_loop_ms'] == pytest.approx(6.)
    assert records[1]['full_loop_ms'] == pytest.approx(9.)
    assert records[1]['start_lateness_ms'] == pytest.approx(4.)
    assert records[1]['scheduled_deadline_miss'] is True
    assert records[1]['skipped_slots'] == 1
