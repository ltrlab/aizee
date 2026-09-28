"""Helm's servo sweep on one arm, with the actual Rust core standing in for the live service.

Minerva's calibration, saved limits and keep-out model; the receiver the live
sweep opens is the native harness (ideal following, one row per request), so the
one-arm roster, the raised ceilings, the enable, every planned move with its
collision check, the guards' plumbing, closing, evidence and saving all run for
real without motors or CAN.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.control.guarded_config import arm_subset, derive_receiver
from robot_platform.control.joint_calibration import load_plan, validate_preview_transmissions
from robot_platform.helm.session import HelmSession
from robot_platform.helm.tune import TuneCore, TuneSession, load_servo, servo_path
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.simulation.native_collision import library_path

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
LIVE = {'reference_tolerance_rad': .1, 'period_us': 2500, 'maximum_lateness_us': 10000}
SHORT = {'defaults': {'amplitudes': [.1, .3], 'kp_ladder': [1., .5], 'damping': [1., .6], 'speeds': [.4, .8],
                      'decelerations': [4.], 'hold_s': .2, 'rest_s': .1}}

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class Receiver:
    """The live receiver's interface (exchange, last, transport hook, close) over the actual Rust core."""

    def __init__(self, config, reference, clock):
        roster = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
        initial = [min(joint['upper'], max(joint['lower'], roster[joint['name']] + .03)) for joint in config['joints']]
        self.harness = NativeControlHarness(config, initial)
        self.harness.step(0)
        self.clock, self.epoch, self.closed = clock, clock(), False
        self.transport = SimpleNamespace(on_result=None)
        self.config = config

    @property
    def last(self):
        return self.harness.last

    def exchange(self, request):
        now_ms = max(self.harness.now_ms, int((self.clock() - self.epoch) * 1000))
        result = self.harness.step(now_ms, request)
        if self.transport.on_result:
            self.transport.on_result(result)
        return result['reply']

    def close(self):
        self.closed = True
        self.harness.close()


@pytest.fixture
def setup(tmp_path):
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(live=LIVE, sweep=dict(profile['sweep'], defaults=SHORT['defaults']))  # The file's ceilings.
    profile_path = tmp_path / 'teleop.json'
    profile_path.write_text(json.dumps(profile))
    session = TuneSession(helm, profile_path)
    session.configure(arm='left', joints=['left_arm_j6'])  # Small range: the reserve clamps it.
    yield helm, session, tmp_path
    helm.close()


def test_a_live_sweep_runs_one_arm_to_the_end_disables_it_and_leaves_evidence(setup):
    helm, session, tmp_path = setup
    clock = Clock()
    receivers = []

    def open_receiver(config, device, reference):
        receivers.append(Receiver(config, reference, clock))
        return receivers[-1]
    core = TuneCore(session._bundle(), open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    receiver = receivers[0]
    names = [joint['name'] for joint in receiver.config['joints']]
    assert all(name.startswith('left_') for name in names)  # The right arm is not in this receiver.
    joints = {joint['name']: joint for joint in receiver.config['joints']}
    assert joints['left_arm_j6']['max_kp'] >= 16.  # Ceilings raised for the sweep.
    for _ in range(20000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if core.stage == 'finished':
            break
    assert core.stage == 'finished' and core.engine.task == 'finished', core.message
    # Paths are checked in the teleop profile's collision scope, as teleop checks them (not every CAD interface).
    assert core.engine.checker.scope == session.profile['receiver']['collision']['scope'] == 'inter_branch'
    assert receiver.last['status']['state'] == 'disarmed' and not receiver.last['hardware_enabled']
    result = core.engine.summary()['left_arm_j6']
    assert result['status'] == 'done' and result['recommendation'] is not None
    assert result['verification']['passed']
    assert result['verified']['up'] > 0 and result['verified']['down'] > 0
    core.close()
    samples = np.fromfile(core.evidence / 'samples.f32', dtype=np.float32)
    columns = json.loads((core.evidence / 'columns.json').read_text())['columns']
    assert samples.size and samples.size % len(columns) == 0
    saved = json.loads((core.evidence / 'results.json').read_text())
    assert saved['task'] == 'finished' and 'left_arm_j6' in saved['joints']
    assert receiver.closed


def test_saved_results_become_the_servo_calibration_the_receiver_uses(setup):
    helm, session, tmp_path = setup
    session.path = tmp_path / 'servo.json'
    motor = next(m for m in helm.reference['motors'] if m['name'] == 'left_arm_j5')
    recommendation = {'joint': 'left_arm_j5', 'hold_kp': 6.5, 'hold_kd': .8, 'gains_met': True,
                      'max_velocity': 1.2, 'stop_acceleration': 8., 'max_tracking_error': .06, 'notes': ['n']}
    session.results = {'left_arm_j5': {'recommendation': recommendation, 'verification': {'passed': True},
                                       'torque': {'friction': .1}, 'verified': {'up': .5, 'down': .5}},
                       'left_arm_j6': {'recommendation': dict(recommendation, gains_met=False),
                                       'verification': {'passed': True}}}
    with pytest.raises(ValueError, match='left_arm_j6: no recommended settings'):
        session.save(['left_arm_j6'])
    document = session.save(['left_arm_j5'])
    entry = document['joints']['left_arm_j5']
    assert (entry['uid'], entry['firmware'], entry['hold_kp'], entry['max_tracking_error']) == (
        motor['uid'], motor['firmware'], 6.5, .06)
    assert load_servo(session.path) == document
    plan, _paths, device, reference, model, pose, bindings = load_plan(helm.profile_path)
    transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), plan['steps'], reference,
                                                   model)
    reference, steps, transmissions = arm_subset(reference, plan['steps'], transmissions, 'left')
    backend = SimpleNamespace(policy_revision='p', runtime_model=SimpleNamespace(revision='m', world_revision='w'))
    derived = derive_receiver(reference=reference, steps=steps, directions=helm.mapping, transmissions=transmissions,
                              model=model, limits=helm.saved_limits, profile=session.profile, backend=backend,
                              servo=document)
    joint = next(j for j in derived['config']['joints'] if j['name'] == 'left_arm_j5')
    assert (joint['hold_kp'], joint['hold_kd'], joint['max_velocity'], joint['max_tracking_error']) == (
        6.5, .8, 1.2 * 1.25, .06)  # The receiver enforces 25% over the saved (commanded) speed.
    assert servo_path(helm.profile_path, helm.asset_id).name == 'minerva-production-01-servo.json'


def test_a_live_sweep_needs_a_confirmation_the_jetson_and_the_tether(setup):
    helm, session, _ = setup
    with pytest.raises(ValueError, match="Confirm the sweep: the left arm's motors move on their own"):
        session.start()
    with pytest.raises(ValueError, match="runs its loop on Minerva's Jetson"):
        session.configure(arm='left', target='pc')
    session.configure(arm='left', mode='virtual', target='pc')
    assert session.snapshot()['receiver_choice'] == {'mode': 'virtual', 'arm': 'left'}
