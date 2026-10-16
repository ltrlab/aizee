"""The response fit recovers a known joint and drive from a synthetic recording, and refuses what it cannot know.

The recording comes from an independent simulation (finer steps, its own friction and delay handling) with the
real recorders' noise: 16-bit positions, the drives' noisy speed reading (which their own damping term uses),
and quantized effort. Tolerances were set before the fit was run.
"""
import json
import math

import numpy as np
import pytest

pytest.importorskip('scipy')

from robot_platform.characterization import response_fit as rf

TRUE = dict(inertia=.32, viscous=.4, friction=.46, gravity_sin=7.2, gravity_cos=.3, gravity_offset=.06)
DELAY_TICKS = 2  # 5 ms from the receiver's command to the drive's loop.
POSITION_STEP = 8 * math.pi / 65535
EFFORT_STEP = .00366
KP, KD = 215., 9.95


def profile(rng, seconds, steps=True):
    """Commanded position and velocity at every tick: ramps of several speeds, small steps and holds."""
    n = int(seconds / rf.TICK_S)
    q_star, v_star = np.zeros(n), np.zeros(n)
    q, i = 0., 0
    while i < n:
        kind = rng.choice(['ramp', 'step', 'hold'], p=[.6, .2, .2] if steps else [.75, 0., .25])
        if kind == 'ramp':
            target, speed = rng.uniform(-1.4, 1.4), rng.uniform(.2, 2.)
            steps = max(1, int(abs(target - q) / speed / rf.TICK_S))
            path = np.linspace(q, target, steps + 1)[1:]
            velocity = np.full(steps, math.copysign(speed, target - q))
        elif kind == 'step':
            target = float(np.clip(q + rng.choice([-1, 1]) * rng.uniform(.03, .12), -1.4, 1.4))
            steps = int(rng.uniform(.3, .8) / rf.TICK_S)
            path, velocity = np.full(steps, target), np.zeros(steps)
        else:
            steps = int(rng.uniform(.2, 1.) / rf.TICK_S)
            path, velocity = np.full(steps, q), np.zeros(steps)
        take = min(len(path), n - i)
        q_star[i:i + take], v_star[i:i + take] = path[:take], velocity[:take]
        q, i = float(path[take - 1]), i + take
    return q_star, v_star


def synthetic(directory, *, seconds=240., seed=3, command_block=True, scale=1., params=TRUE, noise=1., steps=True,
              effort_sample='end', ladder=False, effort_lag_s=0.):
    """effort_sample: the drive reports its torque at each tick's end, or averaged over the tick. ladder: the
    stiffness changes every 4 s, as a sweep's does (damping scaled as sqrt(stiffness)). effort_lag_s: the
    drive reports its torque through a first-order lag."""
    rng = np.random.default_rng(seed)
    q_star, v_star = profile(rng, seconds, steps)
    n = len(q_star)
    kp = np.full(n, KP)
    if ladder:
        kp = np.repeat(rng.choice([215., 150., 100., 70.], size=n // 1600 + 1), 1600)[:n]
    kd = KD * np.sqrt(kp / KP)
    v_star = np.where(rng.random() < 2, v_star, 0.)  # Velocity fed forward.
    ff = .9 * (params['gravity_sin'] * np.sin(q_star) + params['gravity_offset'])  # Gravity fed forward, 10% short.
    q, v = 0., 0.
    rows = []
    sub = 10
    h = rf.TICK_S / sub
    v_read = 0.
    lagged = 0.
    alpha = rf.TICK_S / (effort_lag_s + rf.TICK_S)
    for k in range(n):
        j = max(0, k - DELAY_TICKS)
        tau, total = 0., 0.
        for _ in range(sub):
            tau = float(np.clip(scale * (kp[j] * (q_star[j] - q) + kd[j] * (v_star[j] - v_read) + ff[j]),
                                -60., 60.))
            g = params['gravity_sin'] * math.sin(q) + params['gravity_cos'] * math.cos(q) + params['gravity_offset']
            net = tau - g - params['viscous'] * v
            if abs(v) < 1e-5 and abs(net) <= params['friction']:
                v = 0.
            else:
                v += (net - params['friction'] * math.copysign(1., v if abs(v) >= 1e-5 else net)) / params['inertia'] * h
            q += v * h
            total += tau / sub
        v_read = v + noise * rng.normal(0., .08)
        q_read = round((q + noise * rng.normal(0., .0002)) / POSITION_STEP) * POSITION_STEP
        reported = tau if effort_sample == 'end' else total
        lagged += alpha * (reported - lagged)
        reported = lagged
        effort = round((reported + noise * rng.normal(0., .05)) / EFFORT_STEP) * EFFORT_STEP
        row = [(k + 1) * rf.TICK_S * 1000, q_read, v_read, effort, q_star[k]]
        if command_block:
            row += [v_star[k], ff[k], kp[k], kd[k]]
        rows.append(row)
    columns = ['t_ms'] + [f'joint.{f}' for f in ('position', 'velocity', 'effort', 'command')]
    if command_block:
        columns += [f'joint.{f}' for f in rf.COMMAND_FIELDS]
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'columns.json').write_text(json.dumps({'dtype': 'float32', 'columns': columns}))
    np.asarray(rows, dtype=np.float32).tofile(directory / 'samples.f32')
    return directory


@pytest.fixture(scope='module')
def recording(tmp_path_factory):
    return synthetic(tmp_path_factory.mktemp('response') / 'session')


@pytest.fixture(scope='module')
def report(recording):
    return rf.fit_session(recording)['joints']['joint']


def test_the_mechanics_fit_recovers_the_joint_within_the_declared_tolerances(report):
    p = report['mechanics']['params']
    assert p['inertia'] == pytest.approx(TRUE['inertia'], rel=.05)
    assert p['friction'] == pytest.approx(TRUE['friction'], rel=.1)
    assert p['gravity_sin'] == pytest.approx(TRUE['gravity_sin'], rel=.02)
    assert p['gravity_cos'] == pytest.approx(TRUE['gravity_cos'], abs=.05)
    assert p['gravity_offset'] == pytest.approx(TRUE['gravity_offset'], abs=.05)
    # Constant gains: the viscous term carries the end-of-tick sampling, kp x T/2 less, and says so.
    assert p['viscous'] == pytest.approx(TRUE['viscous'] - KP * rf.TICK_S / 2, abs=.1)
    assert report['mechanics']['flags'] == [
        "viscous includes the drive's effort-sampling term (gains constant or not recorded)"]
    # Held out, it explains the torque down to about the effort's noise.
    assert report['mechanics']['holdout']['r2'] > .99


def test_a_drive_reporting_its_tick_average_torque_gives_the_viscous_term(tmp_path):
    p = rf.fit_session(synthetic(tmp_path / 'average', seconds=160., effort_sample='average'))['joints']['joint']
    assert p['mechanics']['params']['viscous'] == pytest.approx(TRUE['viscous'], abs=.1)


def test_a_stiffness_ladder_separates_the_effort_sampling_from_viscous_friction(tmp_path):
    joint = rf.fit_session(synthetic(tmp_path / 'ladder', seconds=200., ladder=True))['joints']['joint']
    mechanics = joint['mechanics']
    assert mechanics['params']['viscous'] == pytest.approx(TRUE['viscous'], abs=.1)
    assert mechanics['effort_sampling_tick'] == pytest.approx(.5, abs=.15)  # Sampled at each tick's end.
    assert mechanics['params']['inertia'] == pytest.approx(TRUE['inertia'], rel=.05) and not mechanics['flags']


def test_the_drive_fit_finds_the_command_delay_and_scale(report):
    drive = report['drive']
    assert drive['delay_ticks'] == pytest.approx(DELAY_TICKS, abs=.5)
    assert drive['scale'] == pytest.approx(1., rel=.02)


def test_the_closed_loop_replay_reproduces_the_held_out_motion(report):
    replay = report['replay_closed_loop']
    assert replay['model']['windows'] > 20
    # Several times closer to the measured motion than a joint that follows its command exactly.
    assert replay['model']['max_mrad_p50'] * 3 < replay['baseline_follows_command']['max_mrad_p50']
    opened = report['replay_open_loop']
    assert opened['model']['max_mrad_p50'] < opened['baseline_coast']['max_mrad_p50']


def test_the_closed_loop_replay_tells_a_wrong_inertia_apart(recording, report):
    session = rf.load_session(recording)
    joint = session['joints']['joint']
    s = rf.smoothed(joint)
    fit = rf.blocks(joint.t, s['index'])
    holdout = np.zeros(len(joint.t), dtype=bool)
    holdout[s['index'][~fit]] = True
    wrong = {'params': dict(report['mechanics']['params'], inertia=3 * report['mechanics']['params']['inertia'])}
    worse = rf.replay_closed(joint, s, wrong, report['drive'], holdout)
    assert worse['model']['max_mrad_p50'] > 1.5 * report['replay_closed_loop']['model']['max_mrad_p50']


def test_the_closed_loop_fit_recovers_the_joint_from_positions_when_the_effort_lags(tmp_path):
    # The effort reported through a 20 ms lag: the effort fit's inertia and damping are off, the positions are not.
    joint = rf.fit_session(synthetic(tmp_path / 'lag', seconds=200., effort_lag_s=.02))['joints']['joint']
    fitted = joint['closed_loop_fit']
    p = fitted['params']
    assert fitted['converged']
    assert p['inertia'] == pytest.approx(TRUE['inertia'], rel=.05)
    assert p['viscous'] == pytest.approx(TRUE['viscous'], abs=.15)
    assert p['friction'] == pytest.approx(TRUE['friction'], rel=.1)
    assert fitted['delay_ticks'] == pytest.approx(DELAY_TICKS, abs=.5)
    held_out = fitted['replay_closed_loop']
    assert held_out['model']['max_mrad_p50'] * 3 < held_out['baseline_follows_command']['max_mrad_p50']
    assert held_out['model']['max_mrad_p50'] <= joint['replay_closed_loop']['model']['max_mrad_p50']


def test_a_recording_without_commands_fits_the_mechanics_only(tmp_path):
    joint = rf.fit_session(synthetic(tmp_path / 'old', seconds=120., command_block=False))['joints']['joint']
    assert joint['mechanics']['params']['inertia'] == pytest.approx(TRUE['inertia'], rel=.05)
    assert 'unavailable' in joint['drive'] and 'unavailable' in joint['replay_closed_loop']
    assert 'unavailable' in joint['closed_loop_fit']
    assert 'model' in joint['replay_open_loop']


def test_a_drive_scale_error_shows_up_in_the_drive_fit(tmp_path):
    joint = rf.fit_session(synthetic(tmp_path / 'scaled', seconds=120., scale=.9))['joints']['joint']
    assert joint['drive']['scale'] == pytest.approx(.9, rel=.02)


def test_the_loader_refuses_partial_rows_and_marks_padded_command_blocks(tmp_path, recording):
    broken = tmp_path / 'broken'
    broken.mkdir()
    (broken / 'columns.json').write_text((recording / 'columns.json').read_text())
    (broken / 'samples.f32').write_bytes((recording / 'samples.f32').read_bytes()[:-4])
    with pytest.raises(ValueError, match='whole rows'):
        rf.load_session(broken)
    padded = tmp_path / 'padded'
    padded.mkdir()
    (padded / 'columns.json').write_text((recording / 'columns.json').read_text())
    data = np.fromfile(recording / 'samples.f32', dtype=np.float32).reshape(-1, 9)
    data[:, 5:] = np.nan  # An older receiver's rows, padded by servo_sweep.full_rows.
    data.tofile(padded / 'samples.f32')
    assert rf.load_session(padded)['joints']['joint'].commands is None
