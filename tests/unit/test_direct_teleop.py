"""Immediate references preserve acceleration, freshness and trace meaning."""
from dataclasses import replace

import pytest

from robot_platform.teleop.absolute_mapping import AbsoluteLeaderMapper
from robot_platform.teleop.latency import summarize
from robot_platform.teleop.leader_bridge import LeaderSnapshotClient, sample_reply
from robot_platform.teleop.openrb import LeaderSample, LeaderSampleSkipped
from robot_platform.teleop.lifecycle import TeleopLifecycle


def test_direct_cache_owns_feedback_without_copying_recorder_history():
    from types import SimpleNamespace
    from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
    class RecorderOnly:
        def __deepcopy__(self, memo):
            raise AssertionError('Recorder history entered the control cache')
    raw = dict(position=.2, velocity=.1, effort=0., sequence=1, sampled_at_ms=90, fault=False)
    reply = dict(status=dict(now_ms=100))
    snapshot = dict(reply=reply, status=reply['status'], feedback=[raw], recent=RecorderOnly())
    session = DirectLifecycleSession.__new__(DirectLifecycleSession)
    session.controller = SimpleNamespace(joint_names=('q',))
    session.snapshot, session.clock, session._samples = lambda: snapshot, lambda: 10.002, None
    _, rows, feedback = session._cache(reply, 10.)
    assert rows[0]['sampled_at'] == pytest.approx(9.99)
    assert session._offer_received == 10.002
    raw['position'] = .3
    assert feedback[0]['position'] == .2
    assert session._samples[1][0]['position'] == .2
    with pytest.raises(ValueError, match='without a new sequence'):
        session._cache(reply, 10.01)


def test_direct_cache_rejects_mismatched_reply_before_reading_feedback():
    from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
    session = DirectLifecycleSession.__new__(DirectLifecycleSession)
    session.snapshot = lambda: dict(reply={'status': {'now_ms': 99}}, status={'now_ms': 99})
    with pytest.raises(ValueError, match='same receiver reply'):
        session._cache({'status': {'now_ms': 100}}, 10.)


def direct_joint():
    return dict(name='q', lower=-2., upper=2., hold_kp=10., hold_kd=1., max_velocity=1.,
                 align_speed=.1, shutdown_speed=.1, max_lead=.2, tolerance=.01,
                 settled_velocity=.05, zero_position=0., interpolation_s=0., direct_velocity=True,
                 approach=dict(lower=-2., upper=2., max_velocity=1.25, max_tracking_error=.3,
                               stop_acceleration=2., reaction_s=.04))


def direct_rows(t, q=0.):
    return [dict(name='q', position=q, velocity=0., sequence=round(t*1000)+1, sampled_at=t)]


def following_direct(**options):
    life = TeleopLifecycle([direct_joint()], settle_time_s=0., **options)
    life.action('idle', direct_rows(0.), now=0.)
    life.action('enable', direct_rows(0.), now=0.)
    life.action('engage', direct_rows(0.), target=direct_rows(0.), now=0.)
    life.step(direct_rows(.05), target=direct_rows(.05), now=.05)
    assert life.state == 'TELEOP'
    return life


def test_zero_interpolation_keeps_acceleration_and_velocity_feedforward():
    life = following_direct()
    def rows(t, q=0.):
        return direct_rows(t, q)
    first = life.step(rows(.06), target=rows(.06, 1.), now=.06)
    assert first['positions'][0] == pytest.approx(2 * .01**2)
    assert first['joints'][0]['velocity'] == pytest.approx(.02)
    assert 'acceleration' in first['limits']['q']
    hold = life.step(rows(.07), target=None, now=.07)
    assert hold['positions'] == first['positions'] and hold['joints'][0]['velocity'] == 0


def test_fresh_repeats_continue_bounded_motion_without_refreshing_source_or_loss_budget():
    life = following_direct(allow_repeated_source=True)
    source = direct_rows(.06, 1.)
    first = life.step(direct_rows(.06), target=source, now=.06)
    repeat = life.step(direct_rows(.07, first['positions'][0]), target=source, now=.07)
    assert repeat['positions'][0] > first['positions'][0]
    assert repeat['joints'][0]['velocity'] == pytest.approx(.04)
    assert life._source == source and life._misses == 1
    second = life.step(direct_rows(.08, repeat['positions'][0]), target=source, now=.08)
    assert second['joints'][0]['velocity'] == pytest.approx(.06)
    held = life.step(direct_rows(.09, second['positions'][0]), target=source, now=.09)
    assert held['state'] == 'HOLD' and held['joints'][0]['velocity'] == 0
    resumed_source = direct_rows(.10, 1.)
    assert life.step(direct_rows(.10, second['positions'][0]), target=resumed_source, now=.10)['state'] == 'HOLD'


def test_repeated_and_missing_samples_share_the_same_loss_budget():
    life = following_direct(allow_repeated_source=True)
    source = direct_rows(.06, .5)
    life.step(direct_rows(.06), target=source, now=.06)
    life.step(direct_rows(.07), target=source, now=.07)
    life.step(direct_rows(.08), target=None, now=.08)
    assert life.step(direct_rows(.09), target=source, now=.09)['state'] == 'HOLD'


def test_repeated_stationary_goal_never_invents_position_or_velocity():
    life = following_direct(allow_repeated_source=True)
    proposal = life.step(direct_rows(.06), target=direct_rows(.05), now=.06)
    assert proposal['positions'] == [0.] and proposal['joints'][0]['velocity'] == 0.


@pytest.mark.parametrize('invalid', ['position', 'timestamp', 'stale', 'backwards'])
def test_repeat_opt_in_still_rejects_mutated_stale_or_reordered_source(invalid):
    life = following_direct(allow_repeated_source=True)
    source = direct_rows(.06, .5)
    life.step(direct_rows(.06), target=source, now=.06)
    now = .07
    if invalid == 'position': source[0]['position'] = .6
    elif invalid == 'timestamp': source[0]['sampled_at'] = .061
    elif invalid == 'backwards': source[0]['sequence'] -= 1
    else: now = .111
    with pytest.raises(ValueError):
        life.step(direct_rows(now), target=source, now=now)
    assert life.state == 'FAULT'


def test_valid_repeat_uses_the_same_approach_lead_bounds_as_a_new_goal():
    # Measured motion passed the previous target during a reversal. A normal
    # duplicate must re-evaluate the bounded goal rather than enter static HOLD.
    life = following_direct(allow_repeated_source=True)
    life.target = (-.17,)
    life._last_measured = (-.35,)
    life._sent_velocities = (.1,)
    source = direct_rows(.05, .5)
    life._source = source
    repeated = life.step(direct_rows(.06, -.38), target=source, now=.06)
    assert repeated['state'] == 'TELEOP'
    assert abs(repeated['positions'][0] - (-.38)) <= direct_joint()['max_lead'] + 1e-12
    assert abs(repeated['joints'][0]['velocity']) <= direct_joint()['max_velocity']
    assert 'tracking_lead' in repeated['limits']['q']


@pytest.mark.parametrize('direction', [-1., 1.])
def test_constructed_direct_lead_endpoint_survives_next_missing_sample(direction):
    joint = direct_joint()
    joint['max_lead'] = .027731747305049924
    life = TeleopLifecycle([joint], allow_repeated_source=True)
    measured = direction * 1.399970531463623
    life.state, life.target = 'TELEOP', (direction * 1.3704579049057413,)
    life._last_time, life._last_measured = .05, (measured,)
    life._source = direct_rows(.05)
    command = life.step(direct_rows(.06, measured), target=direct_rows(.05), now=.06)
    assert abs(command['positions'][0]-measured) <= joint['max_lead']
    held = life.step(direct_rows(.07, measured), target=None, now=.07)
    assert held['state'] == 'TELEOP' and held['positions'] == command['positions']
    assert held['joints'][0]['velocity'] == 0.


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
