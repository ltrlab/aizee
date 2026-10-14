"""Manual operator presets are bounded data, not measured servo calibration."""
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from robot_platform.helm.manual_tuning import ManualTuning, enabled, install_initial, receiver_rows, validate, values


def derived_fixture():
    names = ('left_arm_j1', 'right_arm_j1')
    return {
        'config': {'joints': [dict(name=name, lower=-2., upper=2., hold_kp=10., hold_kd=1.,
            max_kp=20., max_kd=2., max_velocity=1., max_effort=5., max_tracking_error=.2,
            stop_acceleration=2.) for name in names]},
        'lifecycle': [dict(name=name, lower=-2., upper=2., max_velocity=cap, hold_kp=10., hold_kd=1., max_lead=.12,
                          friction_nm=friction, interpolation_s=0.)
                      for name, cap, friction in zip(names, (.8, .4), (0., .5))],
    }


@pytest.fixture
def preset_factory(tmp_path, monkeypatch):
    from robot_platform.control import commissioning, guarded_config
    monkeypatch.setattr(guarded_config, 'derive_receiver', lambda **kwargs: derived_fixture())
    monkeypatch.setattr(commissioning, 'motor_models', lambda *_: {
        'left_arm_j1': 'ROBSTRIDE03', 'right_arm_j1': 'ROBSTRIDE00'})
    helm = SimpleNamespace(asset_id='fixture-robot', profile_path=tmp_path/'config/devices/calibration.json',
        reference={'id': 'reference-1', 'motors': [
            dict(name=name, uid=f'fixture-uid-{i}', firmware='1.0.0')
            for i, name in enumerate(('left_arm_j1', 'right_arm_j1'))]},
        bindings={'reference': 'reference-digest', 'directions': 'direction-digest'},
        steps=[], mapping={}, transmissions=[], model=None, saved_limits={}, device={}, labels={})
    profile = {'id': 'manual-fixture', 'manual_tuning': True,
               'receiver': {'collision_mode': 'disabled', 'loop_hz': 100, 'direct': {'position_only': True}}}
    servo = {'schema': 1, 'kind': 'servo_calibration', 'joints': {'measured': {'hold_kp': 42.}}}
    servo_path = helm.profile_path.parent / f'{helm.asset_id}-servo.json'
    servo_path.parent.mkdir(parents=True)
    servo_path.write_text(json.dumps(servo), encoding='utf-8')
    return SimpleNamespace(make=lambda: ManualTuning(helm, profile, tmp_path/'runs', servo),
                           helm=helm, profile=profile, servo=servo, servo_path=servo_path)


def test_manual_mode_is_explicit_and_reports_wire_gain_bounds_without_weakening_other_limits():
    assert enabled({'manual_tuning': True})
    assert all(not enabled({'manual_tuning': value}) for value in (False, 1, 'true', None))
    derived = derived_fixture()
    before = deepcopy(derived)
    rows = receiver_rows(derived, {'left_arm_j1': 'ROBSTRIDE03', 'right_arm_j1': 'ROBSTRIDE00'})
    assert rows[0]['bounds'] == {'kp': [0., 5000.], 'kd': [0., 100.],
                                 'speed': [.001, .8], 'travel_rad': [.001, 4.]}
    assert rows[1]['bounds']['kp'] == [0., 500.]
    assert rows[1]['bounds']['kd'] == [0., 5.]
    assert rows[0]['follow'] is True and rows[0]['travel_rad'] == .03
    assert rows[1]['follow'] is False and rows[1]['travel_rad'] is None
    for actual, original in zip(derived['config']['joints'], before['config']['joints']):
        assert {k: v for k, v in actual.items() if k not in ('max_kp', 'max_kd')} == {
            k: v for k, v in original.items() if k not in ('max_kp', 'max_kd')}
    assert derived['lifecycle'] == before['lifecycle']
    with pytest.raises(ValueError, match='known motor model'):
        receiver_rows(derived_fixture(), {})


@pytest.mark.parametrize('case', ['missing', 'extra', 'reverse', 'duplicate', 'unknown_field', 'missing_field',
    'bool_gain', 'nan_gain', 'infinite_damping', 'negative_gain', 'gain_cap', 'damping_cap', 'zero_speed',
    'speed_cap', 'bool_speed', 'zero_travel', 'travel_cap', 'string_travel', 'nonbool_follow'])
def test_validation_rejects_bad_roster_schema_and_limits_atomically(preset_factory, case):
    tuning = preset_factory.make()
    candidate = deepcopy(tuning.active)
    row = candidate[-1]
    if case == 'missing': candidate.pop()
    elif case == 'extra': candidate.append(deepcopy(row))
    elif case == 'reverse': candidate.reverse()
    elif case == 'duplicate': row['motor'] = candidate[0]['motor']
    elif case == 'unknown_field': row['max_effort'] = 100.
    elif case == 'missing_field': row.pop('travel_rad')
    else:
        key, value = {'bool_gain': ('kp', True), 'nan_gain': ('kp', float('nan')),
            'infinite_damping': ('kd', float('inf')), 'negative_gain': ('kp', -.1),
            'gain_cap': ('kp', 500.01), 'damping_cap': ('kd', 5.01), 'zero_speed': ('speed', 0.),
            'speed_cap': ('speed', .401), 'bool_speed': ('speed', True), 'zero_travel': ('travel_rad', 0.),
            'travel_cap': ('travel_rad', 4.01), 'string_travel': ('travel_rad', '.01'),
            'nonbool_follow': ('follow', 1)}[case]
        row[key] = value
    before = tuning.bundle()
    with pytest.raises(ValueError):
        tuning.accept(candidate)
    assert tuning.bundle() == before


def test_complete_boundary_values_and_initial_gains_have_owned_data(preset_factory):
    tuning = preset_factory.make()
    candidate = deepcopy(tuning.active)
    candidate[0].update(kp=0, kd=100, speed=.001, travel_rad=4.)
    candidate[1].update(kp=500, kd=0, speed=.4, travel_rad=None, follow=True)
    checked = validate(candidate, tuning.rows)
    assert type(checked[0]['kp']) is float
    assert checked[1]['travel_rad'] is None
    derived = derived_fixture()
    initial_limits = deepcopy(derived['config']['joints'])
    install_initial(derived, checked)
    for index, (joint, lifecycle) in enumerate(zip(derived['config']['joints'], derived['lifecycle'])):
        assert joint['hold_kp'] == lifecycle['hold_kp'] == checked[index]['kp']
        assert joint['hold_kd'] == lifecycle['hold_kd'] == checked[index]['kd']
        assert joint['max_velocity'] == initial_limits[index]['max_velocity']
        assert joint['max_effort'] == initial_limits[index]['max_effort']
    checked[0]['kp'] = 9.
    assert candidate[0]['kp'] == 0


def test_revision_and_session_tokens_reject_stale_edits_and_bundles_do_not_alias(preset_factory):
    tuning = preset_factory.make()
    initial = tuning.bundle()
    tuning.check_revision(initial['session_id'], initial['revision'])
    for session, revision in [('wrong-session', 0), (initial['session_id'], True),
                              (initial['session_id'], '0'), (initial['session_id'], -1)]:
        with pytest.raises(ValueError, match='changed or the session restarted'):
            tuning.check_revision(session, revision)
    candidate = deepcopy(initial['joints'])
    candidate[0]['kp'] = 22.
    accepted = tuning.accept(candidate)
    assert accepted['revision'] == 1
    with pytest.raises(ValueError): tuning.check_revision(initial['session_id'], 0)
    candidate[0]['kp'] = 99.
    accepted['joints'][0]['kp'] = 88.
    assert tuning.active[0]['kp'] == 22.
    tuning.new_session()
    assert tuning.revision == 1
    with pytest.raises(ValueError): tuning.check_revision(initial['session_id'], 1)
    tuning.check_revision(tuning.session_id, 1)


def test_save_reload_is_manual_and_never_overwrites_measured_servo(preset_factory):
    tuning = preset_factory.make()
    servo_before = preset_factory.servo_path.read_bytes()
    assert tuning.snapshot(can_apply=True, reason='')['dirty'] is True
    candidate = deepcopy(tuning.active)
    candidate[0].update(kp=15., kd=2., speed=.08, travel_rad=.12)
    tuning.accept(candidate)
    assert not tuning.path.exists()  # Apply is separate from deliberately saving.
    tuning.save()
    document = json.loads(tuning.path.read_text(encoding='utf-8'))
    assert document['kind'] == 'manual_teleop_tuning'
    assert document['measured_calibration'] is False
    assert 'operator-selected' in document['provenance']
    assert tuning.path.name == 'manual-fixture-manual.json'
    assert not tuning.path.with_suffix('.json.tmp').exists()
    assert preset_factory.servo_path.read_bytes() == servo_before
    assert tuning.snapshot(can_apply=True, reason='')['dirty'] is False
    reloaded = preset_factory.make()
    assert reloaded.active == candidate
    assert reloaded.session_id != tuning.session_id and reloaded.revision == 0
    candidate[0]['kp'] = 16.
    reloaded.accept(candidate)
    assert reloaded.snapshot(can_apply=False, reason='Hold first')['dirty'] is True
    assert preset_factory.make().active[0]['kp'] == 15.


@pytest.mark.parametrize('field', ['asset', 'profile', 'control', 'calibration', 'profile_sha256',
                                  'servo_sha256', 'motor_uid', 'motor_firmware'])
def test_saved_presets_require_exact_current_binding(preset_factory, field):
    tuning = preset_factory.make()
    tuning.save()
    document = json.loads(tuning.path.read_text(encoding='utf-8'))
    if field.startswith('motor_'):
        document['binding']['motors'][0][field.removeprefix('motor_')] = 'different'
    else:
        document['binding'][field] = 'different'
    tuning.path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError, match='another profile, calibration or motor'):
        preset_factory.make()


@pytest.mark.parametrize('field,value', [('schema', 2), ('schema', True), ('kind', 'servo_calibration'),
                                        ('measured_calibration', True), ('measured_calibration', 'false')])
def test_saved_preset_schema_and_provenance_are_strict(preset_factory, field, value):
    tuning = preset_factory.make()
    tuning.save()
    document = json.loads(tuning.path.read_text(encoding='utf-8'))
    document[field] = value
    tuning.path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError): preset_factory.make()


def test_saved_preset_values_are_revalidated(preset_factory):
    tuning = preset_factory.make()
    tuning.save()
    document = json.loads(tuning.path.read_text(encoding='utf-8'))
    document['joints'][-1]['kd'] = 6.
    tuning.path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError, match='kd must be between'):
        preset_factory.make()


def test_failed_save_preserves_previous_preset_and_in_memory_saved_state(preset_factory, monkeypatch):
    from pathlib import Path
    tuning = preset_factory.make()
    tuning.save()
    before_file, before_saved = tuning.path.read_bytes(), deepcopy(tuning.saved)
    candidate = deepcopy(tuning.active)
    candidate[0]['kp'] = 20.
    tuning.accept(candidate)

    def fail_replace(_source, _destination):
        raise OSError('fixture: filesystem replacement failed')
    monkeypatch.setattr(Path, 'replace', fail_replace)
    with pytest.raises(OSError, match='replacement failed'):
        tuning.save()
    assert tuning.path.read_bytes() == before_file
    assert tuning.saved == before_saved
    assert tuning.snapshot(can_apply=True, reason='')['dirty'] is True


@pytest.mark.parametrize('document', [[], None, 'not a preset'])
def test_nonobject_saved_document_is_a_validation_error(preset_factory, document):
    tuning = preset_factory.make()
    tuning.path.parent.mkdir(parents=True)
    tuning.path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError): preset_factory.make()


def test_rows_carry_what_sets_each_joints_top_following_speed(preset_factory):
    """The panel estimates top speed from each joint's lead, friction and how late the drive sees a command."""
    tuning = preset_factory.make()
    snapshot = tuning.snapshot(can_apply=False, reason='')
    assert [row['following'] for row in snapshot['rows']] == [
        dict(max_lead=.12, friction_nm=0., command_lag_s=.01, gripper=False, velocity_feedforward=False),
        dict(max_lead=.12, friction_nm=.5, command_lag_s=.01, gripper=False, velocity_feedforward=False)]
    assert snapshot['control'] == tuning.binding['control'] == 'direct-position-only'
    assert all('following' not in row for row in values(tuning.rows))  # Display only: never saved or applied.


def test_velocity_feedforward_is_the_presets_bound_control_law(preset_factory, monkeypatch):
    """Presets made under one law are not loaded under the other: the same Kd damps differently."""
    from robot_platform.control import guarded_config
    derived = derived_fixture()
    for joint in derived['lifecycle']:
        joint['velocity_feedforward'] = True
    monkeypatch.setattr(guarded_config, 'derive_receiver', lambda **kwargs: deepcopy(derived))
    tuning = preset_factory.make()
    snapshot = tuning.snapshot(can_apply=False, reason='')
    assert snapshot['control'] == tuning.binding['control'] == 'direct-velocity-feedforward'
    assert all(row['following']['velocity_feedforward'] for row in snapshot['rows'])
    tuning.save()
    monkeypatch.setattr(guarded_config, 'derive_receiver', lambda **kwargs: derived_fixture())
    with pytest.raises(ValueError, match='belongs to another profile'):
        preset_factory.make()  # Position-only again: the feed-forward preset is not taken for it.
