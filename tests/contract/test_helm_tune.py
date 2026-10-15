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

    def __init__(self, config, reference, clock, start=None):
        roster = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
        initial = [min(joint['upper'], max(joint['lower'], roster[joint['name']] + .03)) if start is None
                   else start[joint['name']] for joint in config['joints']]
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


def test_an_arm_returns_from_where_it_rests_to_its_reference_one_joint_at_a_time(setup):
    # The right arm rested with its shoulder swivel 1.22 rad and its wrist 0.58 rad off (2026-09-30): from that
    # confirmed pose it returns along one line, checked first, then it is disabled.
    helm, session, tmp_path = setup
    clock = Clock()
    pose = {m['name']: 0. for m in helm.reference['motors'] if m['name'].startswith('right_')}
    pose.update(right_arm_j1=1.22, right_arm_j6=.58)
    session.receiver, session.joints, session.mode = {'mode': 'live', 'arm': 'right'}, None, 'return'
    session._start_pose = pose
    receivers = []

    def open_receiver(config, device, reference):
        receivers.append(Receiver(config, reference, clock, start=pose))
        return receivers[-1]
    core = TuneCore(session._bundle(), open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    tasks = []
    for _ in range(20000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if not tasks or tasks[-1] != core.engine.task:
            tasks.append(core.engine.task)
        if core.stage == 'finished':
            break
    assert core.stage == 'finished' and core.engine.task == 'finished', core.message
    assert core.message == 'Returned: the arm is at its reference and disabled.'
    assert 'back to the reference' in tasks and core.engine.checker is not None
    receiver = receivers[0]
    assert all(abs(sample['position']) < .02 for sample in receiver.last['feedback'])
    assert receiver.last['status']['state'] == 'disarmed' and not receiver.last['hardware_enabled']
    assert core.evidence.name.startswith('return-') and not core.engine.summary()
    core.close()


def test_closing_a_sweep_lowers_the_arm_before_the_receiver_closes(setup):
    # Closing the receiver cuts torque: a session stopped with a joint out lowers it to the reference first.
    helm, session, tmp_path = setup
    clock = Clock()
    receivers = []

    def open_receiver(config, device, reference):
        receivers.append(Receiver(config, reference, clock))
        return receivers[-1]
    core = TuneCore(session._bundle(), open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    receiver = receivers[0]
    index = [joint['name'] for joint in receiver.config['joints']].index('left_arm_j6')
    home = core.engine.home['left_arm_j6']
    for _ in range(20000):
        core.tick(None, clock())
        clock.now += core.period_s
        if abs(receiver.last['feedback'][index]['position'] - home) > .15:
            break
    assert core.stage == 'sweeping'
    assert core.land() and core.stage == 'lowering'
    for _ in range(2000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if core.stage != 'lowering':
            break
    assert core.stage == 'disabled' and 'lowered to the reference and disabled' in core.message, core.message
    assert abs(receiver.last['feedback'][index]['position'] - home) < .01
    assert receiver.last['status']['state'] == 'disarmed' and not receiver.last['hardware_enabled']
    assert not core.land()  # Nothing left to lower.
    core.close()


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


def test_a_sweep_leaves_an_unplugged_gripper_out_and_saves_nothing(setup):
    # The left wrist harness (2026-10-07): the gripper unplugged, the rest of the arm swept for dropouts.
    helm, session, tmp_path = setup
    with pytest.raises(ValueError, match='only the left gripper'):
        session.configure(arm='left', joints=['left_arm_j6'], absent=['left_arm_j5'])
    with pytest.raises(ValueError, match='only the left gripper'):
        session.configure(arm='left', verify=True, absent=['left_gripper'])
    session.configure(arm='left', joints=['left_arm_j6'], absent=['left_gripper'])
    assert session.snapshot()['absent_choice'] == ['left_gripper']
    bundle = session._bundle()
    session._swept_without = bundle['absent']  # As start() records it.
    clock = Clock()
    receivers = []

    def open_receiver(config, device, reference):
        assert [m['name'] for m in reference['motors']] == [j['name'] for j in config['joints']]
        receivers.append(Receiver(config, reference, clock))
        return receivers[-1]
    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    names = [joint['name'] for joint in receivers[0].config['joints']]
    assert names == [f'left_arm_j{i}' for i in range(1, 7)]  # Neither scanned nor driven.
    assert 'WITHOUT LEFT GRIPPER' in core.message and core.view()['absent'] == ['left_gripper']
    assert core.compiler.fixed and not set(core.compiler.fixed) & set(core.derived['joint_map'])
    for _ in range(20000):
        core.tick(None, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if core.stage == 'finished':
            break
    assert core.stage == 'finished', core.message
    session.results = core.engine.summary()
    core.close()
    with pytest.raises(ValueError, match='left out left_gripper to check the wiring'):
        session.save(['left_arm_j6'])
    # A return or Minerva Check's sweep always covers every drive.
    session.configure(arm='left', verify=True)
    assert 'absent' not in session._bundle()
    with pytest.raises(ValueError, match='Only a tuning sweep'):
        TuneCore(dict(session._bundle(), absent=['left_gripper']), open_receiver=open_receiver,
                 clock=clock, evidence_root=tmp_path / 'evidence').build()


def test_a_late_cycle_is_logged_with_where_its_time_went(setup):
    # Left J3's sweep let the receiver's 80 ms verdict lapse at 1.2 rad/s (2026-10-09) with nothing to show why.
    import gc
    helm, session, tmp_path = setup
    clock = Clock()
    core = TuneCore(session._bundle(), open_receiver=lambda config, device, reference: Receiver(config, reference, clock),
                    clock=clock, evidence_root=tmp_path / 'evidence')
    core.build()
    core.note_cycle(100., 100.010, 100.012)  # On time.
    gc.collect()  # A full collection inside the next cycle.
    core.note_cycle(100.1, 100.105, 100.106)  # Started 100 ms after the last one.
    assert len(core.slow) == 1
    entry = core.slow[0]
    assert (entry['gap_ms'], entry['tick_ms'], entry['view_ms'], entry['gc_generation']) == (100., 5., 1., 2)
    assert entry['gc_ms'] >= 0 and core.view(results=False)['slow_cycles'] == 1  # Frozen: a full pass is quick.
    core.close()
    logged = [json.loads(line) for line in (core.evidence / 'slow-cycles.jsonl').read_text().splitlines()]
    assert logged == [entry] and core._gc_callback not in gc.callbacks


def test_a_live_sweep_needs_a_confirmation_the_jetson_and_the_tether(setup):
    helm, session, _ = setup
    with pytest.raises(ValueError, match="Confirm the sweep: the left arm's motors move on their own"):
        session.start()
    with pytest.raises(ValueError, match="runs its loop on Minerva's Jetson"):
        session.configure(arm='left', target='pc')
    session.configure(arm='left', mode='virtual', target='pc')
    assert session.snapshot()['receiver_choice'] == {'mode': 'virtual', 'arm': 'left'}


def test_a_saved_gravity_slice_feeds_its_arms_pitch_chain_forward(setup):
    helm, session, tmp_path = setup
    session.path = tmp_path / 'servo.json'
    recommendation = {'joint': 'left_arm_j2', 'hold_kp': 220., 'hold_kd': 15., 'gains_met': True,
                      'max_velocity': 1.5, 'stop_acceleration': 16., 'max_tracking_error': .08, 'notes': []}
    session.results = {'left_arm_j2': {
        'recommendation': recommendation, 'verification': {'passed': True}, 'verified': {'up': .5, 'down': .5},
        'torque': {'friction': .6, 'span': [-.4, .4], 'gravity_fit': {'A': 17., 'C': .05}}}}
    document = session.save(['left_arm_j2'])
    assert document['gravity']['left_arm_j2']['A'] == 17. and document['gravity']['left_arm_j2']['span'] == [-.4, .4]
    plan, _paths, device, reference, model, pose, bindings = load_plan(helm.profile_path)
    transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), plan['steps'], reference,
                                                   model)
    reference, steps, transmissions = arm_subset(reference, plan['steps'], transmissions, 'left')
    backend = SimpleNamespace(policy_revision='p', runtime_model=SimpleNamespace(revision='m', world_revision='w'))
    derived = derive_receiver(reference=reference, steps=steps, directions=helm.mapping, transmissions=transmissions,
                              model=model, limits=helm.saved_limits, profile=session.profile, backend=backend,
                              servo=document)
    lifecycle = {row['name']: row for row in derived['lifecycle']}
    # The new shoulder slice joins the saved elbow and wrist slices: the shoulder carries its upper arm and, at
    # their own angles, the links beyond it (the elbow's motor turns the other way about their common axis).
    a3, a4 = document['gravity']['left_arm_j3']['A'], document['gravity']['left_arm_j4']['A']
    assert [term[0] for term in lifecycle['left_arm_j2']['gravity']] == pytest.approx([17. - a3, a3 - a4, a4])
    assert [term[1] for term in lifecycle['left_arm_j2']['gravity']] == [
        {'left_arm_j2': 1}, {'left_arm_j2': 1, 'left_arm_j3': -1},
        {'left_arm_j2': 1, 'left_arm_j3': -1, 'left_arm_j4': 1}]
    assert not lifecycle['left_arm_j1']['gravity'] and not lifecycle['left_arm_j5']['gravity']
    report = {row['motor']: row for row in derived['report']}
    assert report['left_arm_j2']['gravity_ff'] and not report['left_arm_j1']['gravity_ff']


def test_a_silent_helm_stops_a_sweep_after_lowering_the_arm():
    """helm.edge: behind a dropped tether no end of stream comes, so a sweep that hears nothing from Helm for
    `silence_s` stops as if Helm had closed it: the raised arm is lowered and the sweep ends disabled."""
    import time
    from robot_platform.helm.edge import Inbox
    from robot_platform.helm.tune import run_tune_loop

    class Sweep:
        period_s, fault, stage = .01, None, 'sweeping'
        clock = staticmethod(time.perf_counter)

        def __init__(self):
            self.lowering, self.closed = 0, False

        def view(self, results=True):
            return {'stage': self.stage}

        def build(self):
            pass

        def land(self):
            self.lowering = 3  # Cycles to lower the arm, then the sweep disables it.
            return True

        def tick(self, key, started):
            if self.lowering:
                self.lowering -= 1
                self.stage = 'disabled' if not self.lowering else self.stage

        def note_cycle(self, *times):
            pass

        def close(self):
            self.closed = True

        def fail(self, reason):
            self.fault = reason

    core, commands, sent = Sweep(), Inbox(), []
    commands.put(['beat'])
    started = time.monotonic()
    run_tune_loop(commands, SimpleNamespace(send=sent.append, close=lambda: None), core, silence_s=.2)
    assert .2 <= time.monotonic() - started < 2.
    assert core.stage == 'disabled' and core.closed and core.fault is None
