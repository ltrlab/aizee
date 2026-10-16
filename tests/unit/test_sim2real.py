"""The sim-to-real replay runs the simulator's own plant on a recording's commands, and picks its windows honestly.

These check the machinery only. How well the simulator matches the arm is measured on real recordings, never here.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('scipy')

from robot_platform.characterization import sim2real
from robot_platform.characterization.response_fit import TICK_S, Joint
from robot_platform.control.gravity import gravity_effort
from robot_platform.simulation.arm_plant import ArmPlant
from robot_platform.simulation.minerva_sim import ACTUATORS_KIND, FITTED, calibrated_actuators

TERMS = [(6.5, {'a': 1., 'b': 1.}), (2.2, {'a': 1.})]  # A pitch chain: b's measured position couples into a.
SPEC = {'inertia': .3, 'friction': .4, 'static': .55, 'viscous': .2}
ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'


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


def plant_positions(replay, start, n, delay=0, *, late_s=0.):
    """ArmPlant itself, stepped tick by tick with the recorded commands and b's measured position. With `late_s`
    the plant's drive applies them that late, from the tick before `start` on."""
    joint, context = replay.joint, replay.context['b']
    pose = {'b': 0.}
    plant = ArmPlant([dict(name='a', q=replay.q0[start], **SPEC, **({'delay': late_s} if late_s else {}))],
                     gravity={'a': lambda p: gravity_effort(TERMS, {'a': p['a'], 'b': pose['b']})})
    plant.v[0] = replay.v0[start]
    c = joint.commands

    def command(row):
        return [{'position': joint.command[row], 'velocity': c['command_velocity'][row],
                 'effort': c['command_effort'][row], 'kp': c['kp'][row], 'kd': c['kd'][row]}]
    if late_s:
        plant.previous = command(start - 1)
    out = []
    for k in range(n):
        pose['b'] = context[start + k]
        plant.step(command(start + k - delay), TICK_S)
        out.append(plant.q[0])
    return np.array(out)


@pytest.mark.parametrize('delay', [0, 2])
def test_the_replay_is_the_simulators_plant(delay):
    replay = sim2real.Replay(recording(), 'a', TERMS)
    starts, n = np.array([600, 1500, 2700]), 400
    simulated = replay.simulate(starts, n, SPEC, delay)
    for i, start in enumerate(starts):
        np.testing.assert_allclose(simulated[i], plant_positions(replay, start, n, delay), rtol=0, atol=1e-12)


@pytest.mark.parametrize('delay', [.37, .9])
def test_a_drive_that_applies_commands_late_in_the_simulator_is_the_replays_delay(delay):
    """The simulator's plant takes each tick's command as the receiver gives it and applies it `delay` ticks late; the
    replay takes the recorded commands that much older. They agree to rounding (they interpolate in another order)."""
    replay = sim2real.Replay(recording(), 'a', TERMS)
    starts, n = np.array([600, 1500, 2700]), 400
    simulated = replay.simulate(starts, n, SPEC, delay)
    for i, start in enumerate(starts):
        late = plant_positions(replay, start, n, late_s=delay * TICK_S)
        np.testing.assert_allclose(simulated[i], late, rtol=0, atol=1e-12)
        assert np.max(np.abs(late - plant_positions(replay, start, n))) > 1e-4  # The delay matters here.


def test_a_drive_just_enabled_has_only_this_ticks_command_and_a_delay_is_at_most_a_tick():
    commands = [{'position': .5, 'velocity': 0., 'effort': 0., 'kp': 40., 'kd': 2.}]
    plain = ArmPlant([dict(name='a', **SPEC)])
    late = ArmPlant([dict(name='a', delay=.5 * TICK_S, **SPEC)])
    for plant in (plain, late):
        plant.step(None, TICK_S)  # Limp: there is no older command to apply.
        plant.step(commands, TICK_S)
    assert (late.q, late.v) == (plain.q, plain.v) and late.q[0] > 0
    too_late = ArmPlant([dict(name='a', delay=2 * TICK_S, **SPEC)])
    too_late.step(commands, TICK_S)
    with pytest.raises(ValueError, match='longer than one step'):
        too_late.step(commands, TICK_S)


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


def seeded_runs(tmp_path):
    """A Helm runs folder holding the seed's confirmed directions (which mirror right J3 and J4: HELM_SIM.md)."""
    from robot_platform.helm.session import HelmSession
    HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                limits=DEVICES / 'minerva-production-01-joint-limits.json',
                seed=ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json').close()
    return tmp_path / 'runs'


def test_the_current_simulator_is_replayed_with_its_gravity_scale_and_delay(monkeypatch):
    """With an actuator file the simulator scales a joint's gravity and delays its drive, and the comparison replays
    exactly that. Without one, as before: unscaled and undelayed."""
    session = dict(recording(8000), provenance={'session': 'synthetic'})
    monkeypatch.setattr(sim2real, 'load_session', lambda path: session)
    monkeypatch.setattr(sim2real, 'fault_times_ms', lambda path: [])
    replay = sim2real.Replay(session, 'a', TERMS)
    starts, fit, n = replay.windows()
    held = starts[~fit]
    assert len(held)

    def expected(params, delay=0.):
        return sim2real.score(replay.simulate(held, n, params, delay) - replay.measured(held, n),
                              replay.moving(held, n))
    plain = {'specs': {'a': dict(name='a', **SPEC)}, 'terms': {'a': TERMS}, 'nominal': {}, 'identity': {}}
    entry = sim2real.compare(['synthetic'], plain, ['a'], calibrate_joints=False)['joints']['a']
    assert entry['current_params'] == SPEC and entry['current_simulator'] == expected(dict(SPEC, gravity_scale=1.))
    late = dict(plain, specs={'a': dict(name='a', delay=.4 * TICK_S, **SPEC)}, gravity_scale={'a': 1.1})
    entry = sim2real.compare(['synthetic'], late, ['a'], calibrate_joints=False)['joints']['a']
    delay = .4 * TICK_S / TICK_S  # As the plant takes it: ArmPlant.step divides by its step.
    assert entry['current_params'] == dict(SPEC, gravity_scale=1.1, delay_ticks=delay)
    assert entry['current_simulator'] == expected(dict(SPEC, gravity_scale=1.1), delay)
    assert entry['current_simulator'] != expected(dict(SPEC, gravity_scale=1.))


def test_the_simulators_actuator_file_is_written_from_a_saved_report(tmp_path, monkeypatch):
    """--write-sim-params: each converged arm-joint fit, bound to its motor's UID and firmware and to the report,
    recordings and servo calibration it came from, and read back by the simulator as written. It is never
    overwritten, and a report made against another servo calibration is refused."""
    runs, profile = seeded_runs(tmp_path), DEVICES / 'minerva-joint-calibration.json'
    built = sim2real.simulator(profile, runs)
    scores = {part: {'windows': 3, 'rms_mrad_p50': 1., 'max_mrad_p50': 2., 'max_mrad_p95': 3.}
              for part in ('all', 'moving', 'holding')}
    params = {'inertia': .3, 'friction': .5, 'static': .6, 'viscous': 3.3, 'gravity_scale': 1.07, 'delay_ticks': .25,
              'delay_ms': .625}
    fitted = {'params': params, 'fit_windows': 5, 'fit_rms_mrad': 1.2, 'converged': True, 'at_bounds': [],
              'evaluations': 9, 'holdout': scores}

    def joint(calibrated):
        return {'current_params': {'inertia': 1.3, 'friction': .9, 'static': 1.2, 'viscous': 0.}, 'nominal': [],
                'windows': {'fit': 5, 'holdout': 3}, 'follows_command': scores, 'current_simulator': scores,
                'calibrated': calibrated}
    report = {'schema': 1, 'kind': 'sim2real_comparison', 'method': {'window_s': 1.},
              'recordings': [{'session': 'live-x', 'samples_sha256': '5' * 64, 'rows': 10, 'span_s': 1., 'gaps': 0,
                              'command_block': True, 'faults_receiver_ms': []}],
              'simulator': dict(built['identity'], nominal=built['nominal']),
              'joints': {'left_arm_j2': joint(fitted), 'left_arm_j3': joint(dict(fitted, converged=False)),
                         'left_gripper': joint({'unavailable': 'grippers grasp objects the simulator does not have'})}}
    report_path, out = tmp_path / 'report.json', tmp_path / 'simulation' / 'actuators.json'
    report_path.write_text(json.dumps(report), encoding='utf-8')
    monkeypatch.setattr('sys.argv', ['sim2real', '--report', str(report_path), '--runs', str(runs),
                                     '--profile', str(profile), '--write-sim-params', str(out)])
    assert sim2real.main() == 0
    document = json.loads(out.read_text(encoding='utf-8'))
    assert (document['schema'], document['kind'], document['asset_id']) == (1, ACTUATORS_KIND, 'minerva-production-01')
    assert list(document['motors']) == ['left_arm_j2']
    assert document['left_out'] == {'left_arm_j3': 'its fit did not converge',
                                    'left_gripper': 'grippers grasp objects the simulator does not have'}
    entry, bound = document['motors']['left_arm_j2'], built['servo']['joints']['left_arm_j2']
    assert (entry['uid'], entry['firmware']) == (bound['uid'], bound['firmware'])
    assert {key: entry[key] for key in params} == params
    assert entry['gravity_terms'] == [[coefficient, angle] for coefficient, angle in built['terms']['left_arm_j2']]
    assert entry['fit'] == {'fit_windows': 5, 'fit_rms_mrad': 1.2, 'converged': True, 'evaluations': 9}
    assert entry['held_out']['calibrated'] == scores and entry['at_bounds'] == []
    assert document['source']['report'] == {'path': str(report_path),
                                            'sha256': hashlib.sha256(report_path.read_bytes()).hexdigest()}
    assert document['source']['recordings'] == report['recordings']
    assert document['source']['servo'] == {'path': built['identity']['servo'],
                                           'sha256': built['identity']['servo_sha256']}
    calibrated, record = calibrated_actuators(out, built['servo'], list(built['specs']), built['terms'])
    assert record['used'] == ['left_arm_j2']
    assert calibrated['left_arm_j2'] == dict({key: params[key] for key in FITTED}, gravity_scale=1.07,
                                             delay=.25 * TICK_S)
    with pytest.raises(FileExistsError):
        sim2real.main()
    other = dict(report, simulator=dict(report['simulator'], servo_sha256='0' * 64))
    with pytest.raises(ValueError, match='another servo calibration'):
        sim2real.sim_params(other, report_path, built)
