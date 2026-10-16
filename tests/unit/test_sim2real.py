"""The sim-to-real replay runs the simulator's own plant on a recording's commands, and picks its windows honestly.

These check the machinery only. How well the simulator matches the arm is measured on real recordings, never here.
"""
import json
import math

import numpy as np
import pytest

pytest.importorskip('scipy')

from robot_platform.characterization import sim2real
from robot_platform.characterization.response_fit import TICK_S, Joint
from robot_platform.control.gravity import gravity_effort
from robot_platform.simulation.arm_plant import ArmPlant

TERMS = [(6.5, {'a': 1., 'b': 1.}), (2.2, {'a': 1.})]  # A pitch chain: b's measured position couples into a.
SPEC = {'inertia': .3, 'friction': .4, 'static': .55, 'viscous': .2}


def recording(n=4000, *, kp=None, gap_at=None):
    t = np.arange(1, n + 1) * TICK_S
    rng = np.random.default_rng(5)
    q_star = .6 * np.sin(1.3 * t) + .2 * np.sign(np.sin(.7 * t))  # Ramps and steps.
    v_star = np.gradient(q_star, TICK_S) * (np.sin(.2 * t) > 0)  # Velocity fed forward half the time.
    ff = 5. * np.sin(q_star)
    gains = np.where(np.sin(.11 * t) > 0, 60., 25.) if kp is None else kp
    q = q_star + .01 * np.sin(5 * t) + rng.normal(0, 1e-4, n)
    if gap_at is not None:
        t[gap_at:] += .05  # Twenty recorder rows lost.
    commands = {'command_velocity': v_star, 'command_effort': ff, 'kp': gains, 'kd': np.sqrt(gains) * .5}
    a = Joint('a', t, q, np.zeros(n), np.zeros(n), q_star, commands)
    b = Joint('b', t, .3 * np.sin(.9 * t), np.zeros(n), np.zeros(n), np.zeros(n), None)
    return {'joints': {'a': a, 'b': b}}


def plant_positions(replay, start, n, delay=0):
    """ArmPlant itself, stepped tick by tick with the recorded commands and b's measured position."""
    joint, context = replay.joint, replay.context['b']
    pose = {'b': 0.}
    plant = ArmPlant([dict(name='a', q=replay.q0[start], **SPEC)],
                     gravity={'a': lambda p: gravity_effort(TERMS, {'a': p['a'], 'b': pose['b']})})
    plant.v[0] = replay.v0[start]
    c = joint.commands
    out = []
    for k in range(n):
        row = start + k - delay
        pose['b'] = context[start + k]
        plant.step([{'position': joint.command[row], 'velocity': c['command_velocity'][row],
                     'effort': c['command_effort'][row], 'kp': c['kp'][row], 'kd': c['kd'][row]}], TICK_S)
        out.append(plant.q[0])
    return np.array(out)


@pytest.mark.parametrize('delay', [0, 2])
def test_the_replay_is_the_simulators_plant(delay):
    replay = sim2real.Replay(recording(), 'a', TERMS)
    starts, n = np.array([600, 1500, 2700]), 400
    simulated = replay.simulate(starts, n, SPEC, delay)
    for i, start in enumerate(starts):
        np.testing.assert_allclose(simulated[i], plant_positions(replay, start, n, delay), rtol=0, atol=1e-12)


def test_a_fractional_delay_takes_commands_between_whole_ticks():
    replay = sim2real.Replay(recording(), 'a', TERMS)
    rows = np.arange(900, 1100)
    for x in replay.commands:
        np.testing.assert_allclose(replay._late(x, rows, .5), (x[rows] + x[rows - 1]) / 2, rtol=0, atol=1e-12)
        np.testing.assert_array_equal(replay._late(x, rows, 2.), x[rows - 2])


def test_windows_leave_out_disabled_drives_recorder_gaps_and_the_time_before_a_fault():
    n = 8000
    kp = np.full(n, 40.)
    kp[2000:2400] = 0.  # Disabled for a second.
    replay = sim2real.Replay(recording(n, kp=kp, gap_at=5000), 'a', TERMS, faults_ms=[18.5 * 1000])
    starts, fit, length = replay.windows()
    ends = starts + length
    assert len(starts) and length == round(sim2real.WINDOW_S / TICK_S)
    assert not np.any((starts < 2400) & (ends >= 2000))  # Never across the disabled rows 2000-2399.
    assert not np.any((starts < 5000) & (ends >= 5000))  # Never across the gap.
    assert np.all(replay.t[ends] < 18.5 - sim2real.WINDOW_FAULT_S)  # Nothing near or after the fault.
    blocks = np.floor((replay.t[starts] - replay.t[0]) / sim2real.BLOCK_S).astype(int)
    assert np.array_equal(fit, blocks % 2 == 0) and fit.any() and (~fit).any()


def test_a_recording_without_commands_is_refused():
    session = recording()
    session['joints']['a'].commands = None
    with pytest.raises(ValueError, match='before release 5852fe2'):
        sim2real.Replay(session, 'a', TERMS)


def test_fault_events_are_placed_on_the_receiver_clock(tmp_path):
    host0 = 1000.
    with (tmp_path / 'events.jsonl').open('w') as stream:
        stream.write(json.dumps({'t': host0 + 12.25, 'kind': 'state', 'lifecycle': 'TELEOP'}) + '\n')
        stream.write(json.dumps({'t': host0 + 30.5, 'kind': 'state', 'lifecycle': 'FAULT'}) + '\n')
    with (tmp_path / 'direct-latency.jsonl').open('w') as stream:
        for k in range(200):  # The receiver clock started 4 s after host0; a request takes 2 ms.
            started = host0 + 5 + k * .01
            stream.write(json.dumps({'started_at': started, 'completed_at': started + .002,
                                     'receiver_accepted_ms': round((started + .001 - host0 - 4) * 1000)}) + '\n')
    assert sim2real.fault_times_ms(tmp_path) == [pytest.approx(26500., abs=1.)]


def test_a_fault_that_cannot_be_placed_leaves_out_every_window(tmp_path):
    (tmp_path / 'events.jsonl').write_text(json.dumps({'t': 5., 'kind': 'state', 'lifecycle': 'FAULT'}) + '\n')
    (tmp_path / 'direct-latency.jsonl').write_text(json.dumps({'started_at': 4., 'completed_at': 4.1,
                                                               'receiver_accepted_ms': None}) + '\n')
    faults = sim2real.fault_times_ms(tmp_path)
    assert faults == [-math.inf]
    assert not len(sim2real.Replay(recording(), 'a', TERMS, faults).windows()[0])


def test_the_simulator_is_built_from_helms_files(tmp_path):
    from pathlib import Path
    from robot_platform.helm.session import HelmSession
    root = Path(__file__).resolve().parents[2]
    devices = root / 'config/devices'
    helm = HelmSession(devices / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=devices / 'minerva-production-01-joint-limits.json',
                       seed=root / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    helm.close()
    built = sim2real.simulator(devices / 'minerva-joint-calibration.json', tmp_path / 'runs')
    assert len(built['specs']) == 14 and built['specs']['right_arm_j2']['inertia'] == pytest.approx(1.3167, abs=1e-3)
    assert {f'{side}_arm_j{k}' for side in ('left', 'right') for k in (2, 3, 4)} <= set(built['terms'])
    assert built['nominal']['left_arm_j1'] == ['inertia', 'friction', 'static']
    assert len(built['identity']['servo_sha256']) == 64
    assert math.isfinite(built['specs']['left_gripper']['inertia'])
