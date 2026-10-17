"""Trial records, outcomes from scene events, the shared sim/real report, the seeded grid and the CLI."""
import json

import pytest

from robot_platform.evaluation import report as reports, stats, trials
from robot_platform.evaluation.__main__ import main
from robot_platform.evaluation.grid import DEFAULT_X, DEFAULT_Y, GridSpec, cube_poses
from robot_platform.simulation.task_scene import CUBE_START


def event(kind, at, **values):
    return dict(kind=kind, scene_time=at, **values)


SUCCESS = [event('reset', 0.), event('pickup', 1.98), event('handoff', 6.03), event('bucket_settled', 10.8, task_success=True)]


def test_outcome_is_read_from_scene_events():
    assert trials.outcome_from_events(SUCCESS) == (
        {'pickup': 1.98, 'handoff': 6.03, 'bucket': 10.8}, 'success', True, None)
    stages, outcome, eligible, detail = trials.outcome_from_events(
        [event('reset', 0.), event('pickup', 2.), event('dropped_outside_bucket', 7.97)])
    assert (stages['pickup'], stages['handoff'], outcome, eligible) == (2., None, 'dropped_cube', True)
    # Penetration outranks a latched success, as the scene revokes it.
    stages, outcome, eligible, detail = trials.outcome_from_events(SUCCESS + [
        event('forbidden_robot_environment_penetration', 11., geoms=['pad', 'task/table'], depth_m=.003)])
    assert outcome == 'forbidden_penetration' and not eligible and 'task/table 3.0 mm' in detail
    # Settled in the bucket without a handoff: not a placement.
    stages, outcome, *_ = trials.outcome_from_events(
        [event('pickup', 2.), event('bucket_settled', 5., task_success=False)])
    assert outcome == 'incomplete_sequence' and stages['bucket'] == 5.
    assert not trials.reached(stages)['bucket']
    assert trials.outcome_from_events([event('reset', 0.)], timed_out=True)[1] == 'timeout'
    assert trials.outcome_from_events([event('reset', 0.)], aborted='receiver stopping')[1:4:2] == (
        'source_gap_or_abort', 'receiver stopping')
    assert trials.outcome_from_events([event('reset', 0.)])[1] is None


def record(index, outcome, pickup=None, handoff=None, bucket=None, eligible=True, source='simulation'):
    return {'trial': index, 'source': source, 'outcome': outcome, 'eligible': eligible,
            'stages': {'pickup': pickup, 'handoff': handoff, 'bucket': bucket}, 'cube_start': {'x': .18, 'y': -.34}}


def test_trial_validation_refuses_inconsistent_records():
    trials.validate(record(0, 'success', 1., 2., 3.))
    for bad in (record(0, 'success', 1., None, 3.),  # A success reaches every stage.
                record(0, 'success', 3., 2., 4.),  # Stages in order.
                record(0, 'forbidden_penetration', eligible=True),
                record(0, 'timeout', eligible=False),  # Only penetration is ineligible.
                record(0, 'crashed'), record(0, 'timeout', source='lab'), dict(record(0, 'timeout'), trial=-1)):
        with pytest.raises(ValueError):
            trials.validate(bad)


SET = [record(0, 'success', 2., 6., 10.), record(1, 'success', 2.5, 7., 12.), record(2, 'success', 1.8, 6.5, 11.),
       record(3, 'dropped_cube', 2.), record(4, 'timeout'), record(5, 'forbidden_penetration', 2., eligible=False),
       record(6, 'invalid_start')]


def test_report_metrics():
    built = reports.build_report(SET, {'policy': 'example'})
    assert built['trials'] == {'counted': 6, 'excluded_invalid_start': 1, 'logged': 7, 'eligible': 5}
    assert built['success']['count'] == 3 and built['success']['rate'] == .5
    assert built['success']['wilson95'] == list(stats.wilson_interval(3, 6))
    # (3 + 3 + 3 + 1 + 0 + 1) / 3 / 6 stages
    assert built['stage_completion']['score'] == pytest.approx(11 / 18)
    assert built['stage_completion']['stages']['pickup']['count'] == 5
    assert built['time_to_complete_s']['median'] == 11. and built['time_to_complete_s']['p90'] == pytest.approx(11.8)
    assert built['outcomes'] == {'success': 3, 'dropped_cube': 1, 'forbidden_penetration': 1,
                                 'incomplete_sequence': 0, 'timeout': 1, 'source_gap_or_abort': 0}
    assert built['excluded'] == [{'trial': 6, 'outcome': 'invalid_start', 'detail': None}]
    assert built['provenance'] == {'sim2real_validated': False, 'training_promoted': False}
    text = reports.markdown(built)
    assert '3/6 = 50.0%' in text and 'Wilson 95%' in text and '| timeout | 1 |' in text
    with pytest.raises(ValueError):
        reports.build_report([record(0, 'timeout'), record(0, 'timeout')])


def test_hand_logged_real_trials_produce_the_same_report(tmp_path):
    rows = ['trial,cube_x,cube_y,cube_yaw,pickup_s,handoff_s,bucket_s,outcome,eligible,duration_s,detail,notes',
            '0,0.18,-0.34,0,2.0,6.0,10.0,success,,10.4,,',
            '1,0.12,-0.40,0,2.5,7.0,12.0,success,true,12.3,,',
            '2,0.25,-0.29,0,1.8,6.5,11.0,success,,11.2,,',
            '3,0.18,-0.40,0,2.0,,,dropped_cube,,8.0,slipped during handoff,',
            '4,0.12,-0.28,0,,,,timeout,,30,,',
            '5,0.25,-0.40,0,2.0,,,forbidden_penetration,,4.0,wrist hit table,operator stop',
            '6,0.18,-0.28,0,,,,invalid_start,,0,cube knocked before go,']
    path = tmp_path / 'real.csv'
    path.write_text('\n'.join(rows) + '\n')
    logged = trials.load(path)
    for item in logged:
        item.update(source='real', policy='example')
    real = reports.build_report(logged, {'policy': 'example'})
    simulated = reports.build_report(SET, {'policy': 'example'})
    for key in ('trials', 'success', 'stage_completion', 'outcomes'):
        assert real[key] == simulated[key]
    assert real['source'] == 'real' and simulated['source'] == 'simulation'
    assert set(real) == set(simulated)
    assert real['trials_detail'][5]['eligible'] is False  # Unlogged eligibility follows the outcome.
    path.write_text(rows[0] + '\n0,0.1,-0.3,0,abc,,,timeout,,,,\n')
    with pytest.raises(ValueError, match='line 2'):
        trials.load(path)


def test_grid_is_fixed_by_its_seed_and_stays_in_the_reachable_area():
    poses = cube_poses(GridSpec())
    assert len(poses) == 10 and [p['index'] for p in poses] == list(range(10))
    assert poses[0]['cell'] == 'nominal' and (poses[0]['x'], poses[0]['y']) == CUBE_START[:2]
    assert cube_poses(GridSpec()) == poses
    assert cube_poses(GridSpec(seed=1)) != poses and cube_poses(GridSpec(seed=1))[0] == poses[0]
    for pose in poses[1:]:
        assert DEFAULT_X[0] <= pose['x'] <= DEFAULT_X[1] and DEFAULT_Y[0] <= pose['y'] <= DEFAULT_Y[1]
        assert pose['yaw'] == 0.
    # The centre cell's centre is the scene's own cube start; jitter keeps each pose in its cell.
    centre = poses[5]
    assert centre['cell'] == [1, 1] and abs(centre['x'] - CUBE_START[0]) <= .5 * (16 / 300) / 2 + 1e-12
    unjittered = cube_poses(GridSpec(jitter=0., nominal=False))
    assert unjittered[4]['x'] == pytest.approx(CUBE_START[0]) and unjittered[4]['y'] == pytest.approx(CUBE_START[1])
    assert all(abs(p['yaw']) <= .3 for p in cube_poses(GridSpec(yaw=.3)))
    with pytest.raises(ValueError):
        GridSpec(nx=0)


def test_cli_compare_power_grid_and_report(tmp_path, capsys):
    assert main(['compare', '3/10', '9/10']) == 0
    assert 'DIFFER' in capsys.readouterr().out
    assert main(['compare', '5/10', '6/10']) == 0
    out = capsys.readouterr().out
    assert 'no detectable difference' in out and f'{stats.trials_per_arm(.5, .6)} trials per arm' in out
    assert main(['power', '0.5', '0.6']) == 0
    assert capsys.readouterr().out.startswith('388 trials per arm')
    assert main(['grid', '--template']) == 0
    sheet = capsys.readouterr().out.splitlines()
    assert sheet[0].split(',') == list(trials.CSV_COLUMNS) and len(sheet) == 11
    assert sheet[1].startswith('0,0.1800,-0.3400,0.0000,')
    path = tmp_path / 'sheet.csv'
    path.write_text('\n'.join([sheet[0], sheet[1].replace(',,,,,,,,', ',,,,timeout,,30,,')]) + '\n')
    assert main(['report', str(path), '--policy', 'example', '--grid', '3x3', '--out', str(tmp_path / 'out')]) == 0
    saved = json.loads((tmp_path / 'out/report.json').read_text())
    assert saved['source'] == 'real' and saved['outcomes']['timeout'] == 1
    assert saved['protocol']['grid']['seed'] == 0 and len(saved['protocol']['cube_poses']) == 10
    assert (tmp_path / 'out/report.md').read_text().startswith('# Minerva evaluation: example')
