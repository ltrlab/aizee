"""Hand-swept limits: held extremes, glitch rejection, confirmation and binding."""
import json

import pytest

from robot_platform.helm.limits import LimitRecorder, load_limits, validate_limits

MOTORS = [dict(name=name, bus='canL', id=4 + i, uid=f'uid-{i}', firmware='0.1', raw_reference_rad=0.)
          for i, name in enumerate(('j1', 'j2'))]
REFERENCE = dict(id='ref-1', asset_id='bot', model_revision='cad-1', motors=MOTORS)
BINDING = dict(asset_id='bot', reference=REFERENCE, reference_sha256='sha-ref')


def sweep(recorder, values, *, other=0., stream='s', start=1):
    for sequence, q in enumerate(values, start):
        recorder.observe({'j1': q, 'j2': other}, sequence=sequence, stream=stream)


def test_an_end_counts_only_when_two_consecutive_readings_agree():
    recorder = LimitRecorder(MOTORS)
    sweep(recorder, [0., .5, 1.2, .6, 0., -.8, -.8, -.2, 1.0, 1.0, .1])
    row = recorder.rows()[0]
    assert row['max_rad'] == pytest.approx(1.0)   # 1.2 was a single reading
    assert row['min_rad'] == pytest.approx(-.8)
    assert row['current_rad'] == pytest.approx(.1) and row['samples'] == 11


def test_a_single_glitch_cannot_widen_a_limit():
    recorder = LimitRecorder(MOTORS)
    sweep(recorder, [0., .3, .3, 1.4, .3, -.3, -.3])
    row = recorder.rows()[0]
    assert row['max_rad'] == pytest.approx(.3) and row['min_rad'] == pytest.approx(-.3)
    assert row['jumps'] == 0


def test_sudden_jumps_are_counted_and_never_paired():
    recorder = LimitRecorder(MOTORS, jump_rad=1.)
    sweep(recorder, [0., 0., 2.5, 2.5, 0., 0.])
    row = recorder.rows()[0]
    assert row['jumps'] == 2 and row['max_rad'] == pytest.approx(2.5)
    assert recorder.rows()[1]['jumps'] == 0


def test_readings_never_pair_across_reader_streams():
    recorder = LimitRecorder(MOTORS)
    sweep(recorder, [0., .9], stream='a')
    sweep(recorder, [.2, .2], stream='b')
    assert recorder.rows()[0]['max_rad'] == pytest.approx(.2)


def test_a_reading_that_jumps_across_a_reconnect_discards_that_joints_sweep():
    recorder = LimitRecorder(MOTORS, jump_rad=1.)
    sweep(recorder, [0., -.5, -.5, .5, .5], other=.2, stream='a')
    recorder.confirm('j1')
    # After a power cycle j1 reads a turn away; j2 is where it was.
    sweep(recorder, [.5 - 6.2832, .5 - 6.2832], other=.2, stream='b')
    j1, j2 = recorder.rows()
    assert j1['discarded'] == 1 and j1['confirmed'] is None and j1['samples'] == 2
    assert j1['min_rad'] == pytest.approx(.5 - 6.2832) == j1['max_rad']
    assert j2['discarded'] == 0 and j2['samples'] == 7
    recorder.reset('j1')
    assert recorder.rows()[0]['discarded'] == 1


def test_scans_must_be_complete_finite_and_move_forward():
    recorder = LimitRecorder(MOTORS)
    with pytest.raises(ValueError, match='complete'):
        recorder.observe({'j1': 0.}, sequence=1, stream='s')
    with pytest.raises(ValueError, match='Nonfinite'):
        recorder.observe({'j1': float('nan'), 'j2': 0.}, sequence=1, stream='s')
    recorder.observe({'j1': 0., 'j2': 0.}, sequence=2, stream='s')
    with pytest.raises(ValueError, match='sequence'):
        recorder.observe({'j1': 0., 'j2': 0.}, sequence=2, stream='s')


def test_confirmation_needs_both_ends_a_real_span_and_the_reference_inside():
    recorder = LimitRecorder(MOTORS)
    with pytest.raises(ValueError, match='No held readings'):
        recorder.confirm('j1')
    sweep(recorder, [.02, .03, .03])
    with pytest.raises(ValueError, match='only'):
        recorder.confirm('j1')
    recorder.reset('j1')
    sweep(recorder, [.5, .5, 1.2, 1.2], start=10)
    with pytest.raises(ValueError, match='reference'):
        recorder.confirm('j1')
    sweep(recorder, [0., 0., -1., -1.], start=20)
    entry = recorder.confirm('j1', at='t')
    assert entry == dict(min_rad=-1., max_rad=1.2, span_rad=pytest.approx(2.2), samples=8, jumps=0, confirmed_at='t')


def test_growing_past_a_confirmation_is_flagged_and_reset_clears_it():
    recorder = LimitRecorder(MOTORS)
    sweep(recorder, [0., -.5, -.5, .5, .5])
    recorder.confirm('j1')
    sweep(recorder, [.8, .8], start=10)
    row = recorder.rows()[0]
    assert row['confirmed']['max_rad'] == pytest.approx(.5) and row['changed_since_confirmed']
    recorder.reset('j1')
    row = recorder.rows()[0]
    assert row['confirmed'] is None and row['samples'] == 0 and row['max_rad'] is None


def test_document_merges_confirmed_joints_over_valid_saved_limits():
    recorder = LimitRecorder(MOTORS)
    sweep(recorder, [0., -.5, -.5, .5, .5])
    recorder.confirm('j1', at='t')
    first = recorder.document(**BINDING, session='a', at='now')
    assert validate_limits(first, **BINDING) is first
    assert first['joints']['j1'] | {} == dict(bus='canL', id=4, uid='uid-0', firmware='0.1', min_rad=-.5,
        max_rad=.5, span_rad=1., samples=5, jumps=0, confirmed_at='t', session='a')
    assert first['runtime_approved'] is False and first['units'] == 'motor_rad_from_mechanical_zero'
    later = LimitRecorder(MOTORS)
    later.observe({'j1': 0., 'j2': 0.}, sequence=1, stream='x')
    later.observe({'j1': 0., 'j2': -.4}, sequence=2, stream='x')
    later.observe({'j1': 0., 'j2': -.4}, sequence=3, stream='x')
    later.observe({'j1': 0., 'j2': .6}, sequence=4, stream='x')
    later.observe({'j1': 0., 'j2': .6}, sequence=5, stream='x')
    later.confirm('j2')
    merged = later.document(**BINDING, existing=first, session='b')
    assert set(merged['joints']) == {'j1', 'j2'} and merged['joints']['j1']['session'] == 'a'
    with pytest.raises(ValueError, match='Confirm'):
        LimitRecorder(MOTORS).document(**BINDING)


@pytest.mark.parametrize('change', ['reference', 'reference_sha256', 'asset_id', 'units', 'uid', 'range', 'approved'])
def test_saved_limits_only_apply_to_the_same_zero_and_motor_identity(tmp_path, change):
    recorder = LimitRecorder(MOTORS)
    sweep(recorder, [0., -.5, -.5, .5, .5])
    recorder.confirm('j1')
    document = recorder.document(**BINDING)
    if change in ('reference', 'reference_sha256', 'asset_id', 'units'):
        document[change] = 'other'
    elif change == 'uid':
        document['joints']['j1']['uid'] = 'replaced-motor'
    elif change == 'range':
        document['joints']['j1']['max_rad'] = -1.
    else:
        document['runtime_approved'] = True
    path = tmp_path / 'limits.json'
    path.write_text(json.dumps(document))
    loaded, reason = load_limits(path, **BINDING)
    assert loaded is None and reason
    assert load_limits(tmp_path / 'absent.json', **BINDING) == (None, None)
