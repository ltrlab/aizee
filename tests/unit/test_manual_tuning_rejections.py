"""Focused manual-tuning rejection and shutdown contracts; no hardware/builds."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from robot_platform.control.guarded import GuardRejected
from robot_platform.helm.teleop import TeleopCore
from robot_platform.teleop.direct_lifecycle import DirectLifecycleSession
from robot_platform.teleop.lifecycle import TeleopLifecycle


def joint():
    return dict(
        name='q',
        lower=-2.0,
        upper=2.0,
        hold_kp=10.0,
        hold_kd=1.0,
        max_velocity=1.0,
        align_speed=0.1,
        shutdown_speed=0.1,
        max_lead=0.2,
        tolerance=0.005,
        settled_velocity=0.05,
        zero_position=0.0,
        interpolation_s=0.0,
        direct_velocity=True,
        position_only=True,
        approach=dict(
            lower=-2.0, upper=2.0, max_velocity=1.25, max_tracking_error=0.3, stop_acceleration=2.0, reaction_s=0.04
        ),
    )


def rows(now, position=0.5):
    return [dict(name='q', position=position, velocity=0.0, sequence=round(now * 1000) + 1, sampled_at=now)]


def settings(speed=0.001, kp=10.0):
    return [dict(motor='q', kp=kp, kd=1.0, speed=speed, follow=True, travel_rad=None)]


def held():
    life = TeleopLifecycle([joint()])
    life.configure_manual(settings())
    life.action('idle', rows(0.0), now=0.0)
    life.action('enable', rows(0.0), now=0.0)
    return life


def test_shutdown_does_not_inherit_slow_following_cap_or_clipped_velocity_history():
    life = held()
    assert life.joints[0]['max_velocity'] == 0.001
    assert life.joints[0]['shutdown_speed'] == 0.1
    life.action('shutdown', rows(0.01), now=0.01)
    position, speeds = 0.5, []
    for count in range(2, 52):
        proposal = life.step(rows(count * 0.01, position), now=count * 0.01)
        position = proposal['positions'][0]
        speeds.append(abs(proposal['reference_velocities'][0]))
        assert proposal['joints'][0]['velocity'] == 0.0
    assert max(speeds) == pytest.approx(0.1)
    assert position < 0.46
    assert life.state == 'SHUTDOWN'


class Client:
    identity = {'model': 'test'}
    group = 'test'
    joint_names = ('q',)

    def __init__(
        self,
        clock,
        rejection='invalid_state',
        state='executing',
        reason='ok',
        owned=True,
        elapsed=0.0,
        reject_hold=False,
    ):
        self.clock, self.rejection, self.receiver_state, self.reason = clock, rejection, state, reason
        self.owned, self.elapsed, self.reject_hold = owned, elapsed, reject_hold
        self.exchanges, self.calls, self.stopped = 0, [], False

    def direct_update(self, joints, offered, *, valid_for_ms, ramp_efforts=None, update_hold_gains=False):
        self.exchanges += 1
        self.calls.append(
            dict(joints=deepcopy(joints), offer=offered, lifetime=valid_for_ms, update_hold_gains=update_hold_gains)
        )
        status = dict(
            state=self.receiver_state,
            reason=self.reason,
            now_ms=round(self.clock[0] * 1000),
            lease={'last_sequence': self.exchanges},
        )
        if update_hold_gains:
            self.clock[0] += self.elapsed
            raise GuardRejected(self.rejection, status)
        if self.reject_hold:
            raise GuardRejected('offer_expired', status)
        return dict(status=status)

    def stop(self):
        self.exchanges += 1
        self.stopped = True


def session(**options):
    life, clock = held(), [0.01]
    client = Client(clock, **options)
    config = dict(joints=[joint()], identity=client.identity, group=client.group, direct={'command_ms': 30})
    session = DirectLifecycleSession(
        client, config, life, lambda: None, clock=lambda: clock[0], period_s=0.01, cycle_budget_s=0.01
    )
    session.owned = True
    offer = dict(offer={'id': 123, 'issued_at_ms': 0, 'expires_at_ms': 30}, status={'now_ms': 0})
    session._cached = offer, rows(0.01), [dict(position=0.5, velocity=0.0)]
    session._cache = lambda reply, started: None
    return session, client, life, offer


def test_settling_race_rejects_edit_and_preserves_pose_gains_offer_and_lifetime():
    adapter, client, life, offer = session()
    record = adapter.cycle(started_at=0.01, tuning=settings(kp=20.0))
    assert record['accepted'] and record['tuning_rejected']
    assert record['rpc_count'] == 2
    assert [call['update_hold_gains'] for call in client.calls] == [True, False]
    assert all(call['offer'] is offer and call['lifetime'] == 30 for call in client.calls)
    assert [call['joints'][0]['kp'] for call in client.calls] == [20.0, 10.0]
    assert [call['joints'][0]['position'] for call in client.calls] == [0.5, 0.5]
    assert life.joints[0]['hold_kp'] == 10.0 and life.state == 'HOLD' and not client.stopped


@pytest.mark.parametrize(
    'options',
    [dict(rejection='invalid_command'), dict(state='stopping'), dict(reason='command_expired'), dict(owned=False)],
)
def test_other_rejections_fault_without_fallback_or_gain_commit(options):
    adapter, client, life, _ = session(**options)
    with pytest.raises(GuardRejected):
        adapter.cycle(started_at=0.01, tuning=settings(kp=20.0))
    assert len(client.calls) == 1 and client.stopped and life.state == 'FAULT'
    assert life.joints[0]['hold_kp'] == 10.0


def test_settling_fallback_does_not_extend_cycle_deadline():
    adapter, client, life, _ = session(elapsed=0.011)
    with pytest.raises(TimeoutError):
        adapter.cycle(started_at=0.01, tuning=settings(kp=20.0))
    assert len(client.calls) == 1 and client.stopped and life.state == 'FAULT'


def test_original_offer_expiry_during_fallback_still_faults_without_retry():
    adapter, client, life, _ = session(reject_hold=True)
    with pytest.raises(GuardRejected, match='offer_expired'):
        adapter.cycle(started_at=0.01, tuning=settings(kp=20.0))
    assert len(client.calls) == 2 and client.stopped and life.state == 'FAULT'


def core_for_queue(velocity):
    core = TeleopCore.__new__(TeleopCore)
    core.phase, core.fault, core.recording_fault = 'running', None, None
    core._tuning_pending = core._tuning_confirm = core._tuning_result = None
    core._manual_state = dict(session_id='session', revision=1, joints=settings())
    core._manual_rows = [
        dict(
            settings()[0],
            bounds={'kp': [0.0, 50.0], 'kd': [0.0, 10.0], 'speed': [0.001, 1.0], 'travel_rad': [0.001, 4.0]},
        )
    ]
    core.session = SimpleNamespace(lifecycle=held())
    core.harness = SimpleNamespace(last={'feedback': [{'velocity': velocity}]})
    core.derived = {'config': {'settled_velocity': 0.15}}
    return core


@pytest.mark.parametrize('velocity', [0.15001, -0.2, float('nan'), None])
def test_queue_unsettled_feedback_rejection_does_not_change_held_settings(velocity):
    core = core_for_queue(velocity)
    with pytest.raises(ValueError, match='not settled'):
        core.queue_tuning(dict(core._manual_state, joints=settings(kp=20.0)))
    assert core._tuning_pending is None and core._manual_state['revision'] == 1
    assert core.session.lifecycle.state == 'HOLD' and core.session.lifecycle.joints[0]['hold_kp'] == 10.0


def test_queue_settled_boundary_matches_native_inclusive_check():
    core = core_for_queue(-0.15)
    core.queue_tuning(dict(core._manual_state, joints=settings(kp=20.0)))
    assert core._tuning_pending['joints'][0]['kp'] == 20.0
    assert core.session.lifecycle.joints[0]['hold_kp'] == 10.0


def test_core_rejected_edit_produces_user_error_without_ack_or_fault():
    core = core_for_queue(0.0)
    core.queue_tuning(dict(core._manual_state, joints=settings(kp=20.0)))
    core._land = core._hold = core._powered = False
    core._last_start, core.live_arm, core._shown_block = None, 'left', None
    core.sequence, core.largest_gap_ms = 0, 0.0
    core._leader_target = lambda _: (None, None)
    core._record = lambda record: None
    core.session.cycle = lambda **kwargs: dict(accepted=True, tuning_rejected='still settling', request_sequence=2)
    core._tick(None, 0.01)
    assert core._tuning_result == ('still settling', None)
    assert core._tuning_confirm is None and core._tuning_pending is None
    assert core._manual_state['revision'] == 1 and core.fault is None and core.session.lifecycle.state == 'HOLD'
