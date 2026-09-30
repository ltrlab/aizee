"""Live sessions rehearsed against a simulated arm, with the actual Rust receiver.

The receiver reads its configuration from JSON, as the live one does, and ticks every
2.5 ms against an ArmPlant: inertia, gravity (10% off the model teleop feeds forward),
friction with stiction, the drives' speed noise and noisy 16-bit positions. These catch
the receiver refusing or faulting on realistic feedback, which the harness's ideal
joints never show: a joint settling a hair past a command at the edge of its range
(2026-09-29), a speed held down by the lead, a pose past the model's range.
"""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.control.gravity import gravity_effort, gravity_model
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.helm.tune import TuneCore, TuneSession
from robot_platform.simulation.arm_plant import ArmPlant, PlantReceiver
from robot_platform.simulation.control_harness import default_binary
from robot_platform.simulation.native_collision import library_path
from robot_platform.teleop.leader_ranges import KIND, SCHEMA, UNITS, pins
from robot_platform.teleop.openrb import LeaderSample

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
LIVE = {'reference_tolerance_rad': .1, 'period_us': 2500, 'maximum_lateness_us': 10000}
# Inertia (kg m2), friction and static friction (Nm) per joint: rough, not measured.
SPECS = {'j1': (.3, .33, .45), 'j2': (.25, .67, .9), 'j3': (.08, .41, .55), 'j4': (.01, .19, .25),
         'j5': (.004, .15, .2), 'j6': (.003, .11, .15), 'gripper': (.002, .1, .15)}
CENTERS = {'left': [4.67, 3.07, 1.2, 2.2, 3.2, .4, 5.1], 'right': [1.69, 4.76, 2.1, .9, 5.9, 3.3, 6.2]}
SPANS = [2., 2., 2., 2., 2., 2., .8]
HANG = .3

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


def plant_receiver(helm, servo, clock, directory, disturbance=None):
    """An open_receiver for the right arm's simulated joints, with gravity 10% over the fed model."""
    truth = {motor: dict(fit, A=fit['A'] * 1.1) for motor, fit in ((servo or {}).get('gravity') or {}).items()}
    terms = gravity_model(helm.model, helm.mapping, [m for m in helm.reference['motors']
                                                     if m['name'].startswith('right')], truth)
    opened = []

    def open_receiver(config, device, reference):
        names = [joint['name'] for joint in config['joints']]
        spec = {name: SPECS[name.split('_')[-1]] for name in names}
        plant = ArmPlant([dict(name=n, inertia=spec[n][0], friction=spec[n][1], static=spec[n][2]) for n in names],
                         gravity={n: (lambda pose, t=terms[n]: gravity_effort(t, pose)) for n in names if n in terms},
                         position_noise=.0006, disturbance=disturbance)
        opened.append(PlantReceiver(config, plant, clock=clock, directory=directory))
        return opened[-1]
    return open_receiver, opened


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    yield session
    session.close()


def test_a_sweep_runs_to_the_edges_of_a_joints_range_on_a_simulated_arm(helm, tmp_path):
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(live=LIVE, sweep=dict(profile['sweep'], defaults={
        'amplitudes': [.5, 2.], 'kp_ladder': [1.], 'damping': [1., .5], 'speeds': [.8, 2.], 'decelerations': [16.],
        'hold_s': .2, 'rest_s': .1}))
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile))
    session = TuneSession(helm, path)
    session.configure(arm='right', joints=['right_arm_j6'])
    clock = Clock()
    bundle = session._bundle()
    open_receiver, opened = plant_receiver(helm, bundle.get('servo'), clock, tmp_path / 'receiver')
    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    try:
        core.build()
        for _ in range(20000):
            core.tick(None, clock())
            clock.now += core.period_s
            assert core.fault is None, core.fault
            if core.stage == 'finished':
                break
        result = core.engine.summary()['right_arm_j6']
        assert core.engine.task == 'finished' and result['status'] == 'done' and result['verification']['passed']
        usable = next(row['usable'] for row in core.derived['report'] if row['motor'] == 'right_arm_j6')
        # To both edges of its usable range, where it settles a hair either side of its command.
        assert result['verified']['up'] == pytest.approx(usable[1], abs=1e-6)
        assert result['verified']['down'] == pytest.approx(-usable[0], abs=1e-6)
        assert opened[0].last['status']['state'] == 'disarmed'
    finally:
        core.close()


def test_a_trip_with_the_arm_raised_holds_it_and_lowers_it_to_the_reference(helm, tmp_path):
    # Right J2, 2026-09-29: raised near horizontal, a trip cut its torque and it fell. Here a push drives
    # J2 past its command while it is raised: the receiver brakes it to a hold, and the sweep takes it
    # back there and lowers it to the reference before disabling; it never falls.
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(live=LIVE, sweep=dict(profile['sweep'], defaults={
        'amplitudes': [.6], 'kp_ladder': [1.], 'damping': [1.], 'speeds': [.4], 'decelerations': [16.],
        'hold_s': .2, 'rest_s': .1}))
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile))
    session = TuneSession(helm, path)
    session.configure(arm='right', joints=['right_arm_j2'])
    clock = Clock()
    bundle = session._bundle()
    push = {}

    def disturbance(plant):
        q = plant.q[plant.names.index('right_arm_j2')]
        if 'at' not in push and q > .45:
            push['at'] = plant.t
        return {'right_arm_j2': 80.} if 'at' in push and plant.t - push['at'] < .15 else {}
    open_receiver, opened = plant_receiver(helm, bundle.get('servo'), clock, tmp_path / 'receiver', disturbance)
    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    try:
        core.build()
        states, fastest = set(), 0.
        for _ in range(20000):
            core.tick(None, clock())
            clock.now += core.period_s
            assert core.fault is None, core.fault
            plant = opened[0].plant
            states.add(opened[0].last['status']['state'])
            if 'at' in push and plant.t > push['at'] + .6:  # After the push and the braking.
                fastest = max(fastest, abs(plant.v[plant.names.index('right_arm_j2')]))
            if core.stage in ('finished', 'disabled', 'held'):
                break
        assert 'at' in push and 'stopping' in states  # Braked to a hold (re-armed the cycle it stopped).
        assert core.stage == 'disabled' and 'lowered to the reference and disabled' in core.message, core.message
        plant = opened[0].plant
        assert abs(plant.q[plant.names.index('right_arm_j2')]) < .05  # Hanging at the reference.
        assert fastest < .5  # Lowered at its characterisation speed; never fell.
        assert opened[0].last['status']['state'] == 'disarmed'
        result = core.engine.summary()['right_arm_j2']
        assert result['status'] == 'stopped'
        assert any('lowering the arm' in event['text'] for event in result['events']), result['events']
    finally:
        core.close()


def test_a_drive_that_stops_answering_with_the_arm_raised_holds_it_then_lowers_it(helm, tmp_path):
    # The left gripper stopped answering for over 30 ms with the left arm raised, and cutting every joint's torque
    # dropped the arm (2026-09-30). Here the right gripper goes quiet for 0.1 s with J2 raised: the receiver holds
    # the arm on the drives that answer; once the gripper answers again, the sweep lowers the arm and disables it.
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(live=LIVE, sweep=dict(profile['sweep'], defaults={
        'amplitudes': [.6], 'kp_ladder': [1.], 'damping': [1.], 'speeds': [.4], 'decelerations': [16.],
        'hold_s': .2, 'rest_s': .1}))
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile))
    session = TuneSession(helm, path)
    session.configure(arm='right', joints=['right_arm_j2'])
    clock = Clock()
    bundle = session._bundle()
    quiet = {}

    def silence(plant):  # Rides the disturbance hook, which runs every substep: pushes nothing.
        q = plant.q[plant.names.index('right_arm_j2')]
        if 'at' not in quiet and q > .45:
            quiet['at'] = plant.t
        plant.quiet = {'right_gripper'} if 'at' in quiet and plant.t - quiet['at'] < .1 else set()
        return {}
    open_receiver, opened = plant_receiver(helm, bundle.get('servo'), clock, tmp_path / 'receiver', silence)
    core = TuneCore(bundle, open_receiver=open_receiver, clock=clock, evidence_root=tmp_path / 'evidence')
    try:
        core.build()
        reasons, fastest = set(), 0.
        for _ in range(20000):
            core.tick(None, clock())
            clock.now += core.period_s
            assert core.fault is None, core.fault
            plant = opened[0].plant
            reasons.add(opened[0].last['status']['reason'])
            fastest = max(fastest, abs(plant.v[plant.names.index('right_arm_j2')]))
            if core.stage in ('finished', 'disabled', 'held'):
                break
        assert 'at' in quiet and 'feedback_stale' in reasons
        assert core.stage == 'disabled' and 'lowered to the reference and disabled' in core.message, core.message
        plant = opened[0].plant
        assert abs(plant.q[plant.names.index('right_arm_j2')]) < .05  # Lowered to hanging.
        assert fastest < .5  # Held, never fell.
    finally:
        core.close()


class Leaders:
    class inputs:
        max_age_s, max_skew_s = .05, .02

    def __init__(self, clock):
        self.clock, self.sequence, self.closed = clock, 0, False
        self.offsets = {name: [HANG] * 7 for name in CENTERS}

    def poll(self):
        self.sequence += 1
        now = self.clock()
        return {name: LeaderSample(tuple((c + d) % (2 * math.pi) for c, d in zip(CENTERS[name], self.offsets[name])),
                                   (0,) * 7, self.sequence, now - .005, now - .002, (0, 0, 1, 1), 1, 2)
                for name in CENTERS}

    def close(self):
        self.closed = True


def test_teleop_sweeps_every_right_arm_joint_to_its_ends_at_speed_on_a_simulated_arm(helm, tmp_path):
    leader_profile = json.loads((DEVICES / 'gello-dual-viewer.json').read_text(encoding='utf-8'))
    ranges = {'schema': SCHEMA, 'kind': KIND, 'profile': leader_profile['id'], 'pins': pins(leader_profile),
              'units': UNITS, 'runtime_approved': False, 'leaders': {
                  name: {joint['name']: {'center_rad': center, 'span_rad': span}
                         for joint, center, span in zip(item['joints'], CENTERS[name], SPANS)}
                  for name, item in leader_profile['leaders'].items()},
              'reference': {'captured_at': 'fixture', 'leaders': {
                  name: {joint['name']: (center + HANG) % (2 * math.pi)
                         for joint, center in zip(item['joints'], CENTERS[name])}
                  for name, item in leader_profile['leaders'].items()}}}
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(leader_profile=str(DEVICES / 'gello-dual-viewer.json'), live=LIVE)
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile))
    bundle = TeleopSession(helm, path, runs=tmp_path / 'runs',
                           leaders=SimpleNamespace(saved=ranges, missing=lambda routes: []))._bundle()
    bundle = dict(bundle, receiver={'mode': 'live', 'arm': 'right', 'motor_models': {}})
    clock = Clock()
    open_receiver, opened = plant_receiver(helm, bundle.get('servo'), clock, tmp_path / 'receiver')
    leaders = Leaders(clock)
    core = TeleopCore(bundle, open_leaders=lambda _profile: leaders, open_receiver=open_receiver, clock=clock)

    def tick(key=None):
        core.tick(key, clock())
        clock.now += core.period_s
        assert core.fault is None, core.fault
    try:
        core.build()
        tick('I')
        tick('E')
        for _ in range(10):
            tick()
        tick('T')
        for _ in range(400):
            tick()
            if core.session.lifecycle.state == 'TELEOP':
                break
        assert core.session.lifecycle.state == 'TELEOP'
        names = [joint['name'] for joint in core.derived['config']['joints']]
        reached = {name: [0., 0.] for name in names}
        start = clock()
        while clock() - start < 12.:
            t = clock() - start
            for k in range(6):  # Each channel past both ends of its span, at up to 1.6 rad/s of leader.
                leaders.offsets['right'][k] = HANG + (1.05 * math.sin(1.5 * t + .7 * k) - HANG) * min(1., t / 2.)
            tick()
            for name, sample in zip(names, opened[0].last['feedback']):
                reached[name] = [min(reached[name][0], sample['position']), max(reached[name][1], sample['position'])]
        usable = {row['motor']: row['usable'] for row in core.derived['report']}
        # The wrist tilt (J6) crosses its whole range and holds at its ends; the wrist swivel (J5) goes past the
        # model's +-pi.
        assert reached['right_arm_j6'][0] == pytest.approx(usable['right_arm_j6'][0], abs=.01)
        assert reached['right_arm_j6'][1] == pytest.approx(usable['right_arm_j6'][1], abs=.01)
        assert reached['right_arm_j5'][0] < -math.pi
        # J2 lifts the arm to its end, gravity fed forward.
        assert reached['right_arm_j2'][1] == pytest.approx(usable['right_arm_j2'][1], abs=.01)
    finally:
        core.close()
