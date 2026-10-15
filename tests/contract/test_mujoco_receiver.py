"""The real Rust receiver owns commands/expiry while MuJoCo produces feedback."""
from pathlib import Path

import pytest

pytest.importorskip('mujoco')

from hypothesis import given, settings, strategies as st

from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.simulation.arm_plant import PlantReceiver
from robot_platform.simulation.mujoco_plant import JointBinding, MujocoPlant

FIXTURES = Path(__file__).resolve().parents[1] / 'fixtures/physics'


def config():
    return {'group': 'synthetic',
            'identity': {'model': 'synthetic-slider', 'calibration': 'synthetic', 'collision_policy': 'fixture'},
            'lease_ms': 100, 'feedback_timeout_ms': 20, 'settled_velocity': .001,
            'joints': [{'name': 'motor', 'unit': 'radians', 'lower': -3., 'upper': 3.,
                        'max_velocity': 2., 'max_effort': 5., 'max_kp': 20., 'max_kd': 5.,
                        'stop_acceleration': 2., 'hold_kp': 10., 'hold_kd': 1.,
                        'max_tracking_error': .5, 'max_temperature': 70.}]}


def test_receiver_refuses_a_mismatched_motor_binding_before_starting_harness(tmp_path):
    plant = MujocoPlant(FIXTURES / 'slider.xml', [JointBinding('wrong_motor', 'slide', 5.)])
    with pytest.raises(ValueError, match='joint order'):
        PlantReceiver(config(), plant, clock=lambda: 100., directory=tmp_path)


def test_receiver_commands_drive_mujoco_and_feedback_returns_to_rust(tmp_path):
    cfg = config()
    now = [100.]
    plant = MujocoPlant(FIXTURES / 'slider.xml', [JointBinding('motor', 'slide', 5.)], position_step=0.)
    receiver = PlantReceiver(cfg, plant, clock=lambda: now[0], directory=tmp_path, service_latency=False)
    client = GuardedControllerClient(receiver.exchange, group=cfg['group'], identity=cfg['identity'],
                                     joint_names=('motor',))
    try:
        client.acquire()
        client.arm()
        for _ in range(15):
            client.move([{'position': .05, 'velocity': .2, 'effort': 0., 'kp': 10., 'kd': 1.}])
            now[0] += .005
            receiver.advance()
        assert plant.q[0] > 0.
        assert receiver.last['feedback'][0]['position'] > 0.
        # Stop receiving feedback without changing the simulated mechanics.
        plant.quiet.add('motor')
        now[0] += .04
        receiver.advance()
        assert receiver.last['status']['state'] == 'faulted'
        assert receiver.last['status']['reason'] == 'feedback_stale'
    finally:
        receiver.close()


@pytest.mark.parametrize('service_latency', [True, False])
@pytest.mark.parametrize('every', [3, 5, 7])
def test_a_request_never_steps_the_receiver_back_when_the_clock_runs_on_while_it_ticks(tmp_path, service_latency,
                                                                                      every):
    # A real clock moves while the plant ticks. Helm's simulated Minerva faulted clock_regression at its first
    # request (2026-10-14): a hiccup during the ticks (its window starting) put the request past the next scheduled
    # tick, which then stepped the receiver back. Here every few readings jump 6 ms (both modes faulted before).
    cfg = config()
    reads = [0]
    now = [100.]

    def clock():
        reads[0] += 1
        now[0] += .006 if reads[0] % every == 0 else .0005
        return now[0]
    plant = MujocoPlant(FIXTURES / 'slider.xml', [JointBinding('motor', 'slide', 5.)], position_step=0.)
    receiver = PlantReceiver(cfg, plant, clock=clock, directory=tmp_path, service_latency=service_latency)
    client = GuardedControllerClient(receiver.exchange, group=cfg['group'], identity=cfg['identity'],
                                     joint_names=('motor',))
    try:
        try:
            client.acquire()
            client.arm()
            for _ in range(100):
                client.renew()
                client.move([{'position': .05, 'velocity': 0., 'effort': 0., 'kp': 10., 'kd': 1.}])
        except Exception as exc:
            raise AssertionError(receiver.last['status']) from exc
        assert receiver.last['status']['state'] == 'executing', receiver.last['status']
    finally:
        receiver.close()


@settings(max_examples=12, deadline=None)
@given(gaps=st.lists(st.integers(min_value=1, max_value=80), min_size=1, max_size=8))
def test_generated_silent_intervals_cannot_renew_receiver_authority(gaps):
    # A fresh receiver per example. Hypothesis stores/shrinks the failing gap list.
    import tempfile
    with tempfile.TemporaryDirectory(prefix='mujoco-lease-') as directory:
        cfg = config()
        now = [100.]
        plant = MujocoPlant(FIXTURES / 'slider.xml', [JointBinding('motor', 'slide', 5.)], position_step=0.)
        receiver = PlantReceiver(cfg, plant, clock=lambda: now[0], directory=directory, service_latency=False)
        client = GuardedControllerClient(receiver.exchange, group=cfg['group'], identity=cfg['identity'],
                                         joint_names=('motor',))
        try:
            client.acquire()
            client.arm()
            client.move([{'position': 0., 'velocity': 0., 'effort': 0., 'kp': 10., 'kd': 1.}])
            assert receiver.last['status']['state'] == 'executing'
            elapsed = 0
            for gap in gaps:
                elapsed += gap
                now[0] = 100. + elapsed / 1000.
                receiver.advance()
                if elapsed >= 103:  # covers float clock rounding around the 100 ms tick
                    assert receiver.last['status']['state'] in {'stopping', 'stopped'}
            now[0] += .15
            receiver.advance()
            assert receiver.last['status']['state'] in {'stopping', 'stopped'}
            # Lease and command expire together; the receiver explicitly prioritizes lease expiry.
            assert receiver.last['status']['reason'] == 'lease_expired'
            assert receiver.last['hardware_enabled']  # expiry stops and holds; it does not go limp
        finally:
            receiver.close()
