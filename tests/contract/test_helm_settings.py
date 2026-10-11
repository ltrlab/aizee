"""Operator settings on Minerva's direct tuning profile, through the real lifecycle and native receiver.

Settings only tighten what the profile and calibration derive (speed ceilings, freshness, receiver timing),
persist bound to the profile, calibration and motors, and take effect from the next session. No motors.
"""
from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from robot_platform.helm.manual_tuning import ManualTuning
from robot_platform.helm.session import HelmSession
from robot_platform.helm.settings import OperatorSettings, resolve_profile
from robot_platform.helm.teleop import TeleopCore
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders, Receiver, run, DEVICES, SEED  # noqa: F401

EDITABLE_RECEIVER = {'receiver.lease_ms', 'receiver.feedback_timeout_ms', 'receiver.settled_velocity',
                     'receiver.direct.command_ms', 'receiver.direct.velocity_timeout_ms'}


@pytest.fixture
def direct(bundle, tmp_path):  # noqa: F811
    data, path = bundle
    profile = json.loads((DEVICES / 'minerva-teleop-direct-tuning.json').read_text(encoding='utf-8'))
    profile['leader_profile'] = str(DEVICES / 'gello-dual-viewer.json')
    path.write_text(json.dumps(profile))
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'other',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    try:
        manual = ManualTuning(helm, profile, tmp_path / 'manual', data['servo'])
        settings = OperatorSettings(helm, profile, tmp_path / 'settings', data['servo'], manual)
        yield SimpleNamespace(data=data, profile=profile, helm=helm, manual=manual, settings=settings,
                              runs=tmp_path / 'settings')
    finally:
        helm.close()


def fields(snapshot):
    return {field['id']: field for section in snapshot['sections'] for field in section['fields']}


def apply(settings, manual, values):
    return settings.update('apply', {'session_id': settings.session_id, 'revision': settings.revision,
                                     'values': values}, manual=manual)


def test_the_catalog_shows_everything_and_only_tightening_values_are_editable(direct):
    snapshot = direct.settings.snapshot(can_apply=True, manual=direct.manual)
    assert [s['id'] for s in snapshot['sections']] == ['motion', 'source', 'receiver', 'guards', 'profile',
                                                      'binding', 'runtime']
    catalog = fields(snapshot)
    editable = {key for key, field in catalog.items() if field['editable']}
    assert editable >= EDITABLE_RECEIVER and 'source.target_max_age_s' in editable
    assert all(key.split('.')[-1] in ('max_velocity', 'align_speed', 'shutdown_speed')
               for key in editable if key.startswith('joint.'))
    assert not editable & {key for key in catalog if key.split('.')[0] in ('guard', 'profile', 'binding',
                                                                             'runtime', 'manual')}
    # Every editable value can only go down (or a settling time up): its upper bound is its default.
    for key in editable - {'source.settle_time_s'}:
        assert catalog[key]['bounds'][1] == catalog[key]['default'], key
    # Manual gains are listed, and point to their own Hold -> Apply -> Engage workflow.
    assert catalog['manual.left_arm_j1.kp']['workflow']['tab'] == 'teleop'
    # A running session leaves the catalog readable but nothing editable.
    assert not any(f['editable'] for f in fields(direct.settings.snapshot(can_apply=False)).values())


def test_applied_settings_persist_bound_to_the_profile_and_only_tighten(direct):
    settings, manual = direct.settings, direct.manual
    ceiling = fields(settings.snapshot(can_apply=True))['joint.left_arm_j1.max_velocity']['value']
    applied = apply(settings, manual, {'joint.left_arm_j1.max_velocity': ceiling / 2,
                                       'source.target_max_age_s': .04, 'receiver.lease_ms': 120})
    assert applied['revision'] == 1
    document = json.loads(settings.path.read_text(encoding='utf-8'))
    assert document['measured_calibration'] is False and document['binding'] == settings.binding
    # A new Helm session reads them back, bound to the same profile, calibration and motors.
    again = OperatorSettings(direct.helm, direct.profile, direct.runs, direct.data['servo'], manual)
    assert again.revision == 1 and again.bundle() == settings.bundle()
    for values, match in [({'joint.left_arm_j1.max_velocity': ceiling * 2}, 'between'),
                          ({'source.target_max_age_s': 1.}, 'between'),
                          ({'receiver.lease_ms': 10_000}, 'between'),
                          ({'guard.left_arm_j1.max_velocity': .1}, 'read-only'),
                          ({'receiver.loop_hz': 50}, 'read-only')]:
        with pytest.raises(ValueError, match=match):
            apply(settings, manual, values)
    with pytest.raises(ValueError, match='reload'):
        settings.update('apply', {'session_id': settings.session_id, 'revision': 0,
                                  'values': {'source.target_max_age_s': .03}}, manual=manual)
    with pytest.raises(ValueError, match='Stop'):
        settings.update('reset', {'session_id': settings.session_id, 'revision': 1}, running=True)
    reset = settings.update('reset', {'session_id': settings.session_id, 'revision': 1}, manual=manual)
    assert reset['revision'] == 2 and reset['joints'] == settings.defaults['joints']
    # A changed profile or calibration does not inherit them.
    other = deepcopy(direct.profile)
    other['receiver']['lease_ms'] -= 1
    with pytest.raises(ValueError):
        OperatorSettings(direct.helm, dict(other, id=direct.profile['id']), direct.runs, direct.data['servo'], manual)


def test_speeds_stay_consistent_with_each_ceiling(direct):
    settings, manual = direct.settings, direct.manual
    catalog = fields(settings.snapshot(can_apply=True))
    floor = catalog['joint.left_arm_j1.max_velocity']['bounds'][0]
    # The ceiling bounds the return speed, which must reach the reference within its budget.
    assert floor == catalog['joint.left_arm_j1.shutdown_speed']['bounds'][0] > .001
    with pytest.raises(ValueError, match='between'):
        apply(settings, manual, {'joint.left_arm_j1.max_velocity': floor / 2})
    # Alignment and return speeds above a lowered ceiling are named, not reported as a generic interval error.
    align = catalog['joint.left_arm_j1.align_speed']['value']
    with pytest.raises(ValueError, match='alignment and return speeds'):
        apply(settings, manual, {'joint.left_arm_j1.max_velocity': align / 2})
    # A manual following speed above a lowered ceiling must come down first.
    original = deepcopy(manual.bundle()['joints'])
    fast = deepcopy(original)
    next(r for r in fast if r['motor'] == 'left_arm_j1')['speed'] = align * 2
    manual.accept(fast)
    with pytest.raises(ValueError, match='manual following speed'):
        apply(settings, manual, {'joint.left_arm_j1.max_velocity': align})
    manual.accept(original)
    apply(settings, manual, {'joint.left_arm_j1.max_velocity': align})
    with pytest.raises(ValueError, match='ceiling'):
        settings.validate_manual(fast)  # Nor can a manual edit exceed an applied ceiling.


def test_a_session_started_with_settings_derives_and_follows_within_them(direct):
    settings, manual, data = direct.settings, direct.manual, direct.data
    catalog = fields(settings.snapshot(can_apply=True))
    ceiling = catalog['joint.left_arm_j1.align_speed']['value']  # At the alignment speed: above the return speed.
    apply(settings, manual, {'joint.left_arm_j1.max_velocity': ceiling, 'joint.left_arm_j1.align_speed': ceiling / 2,
                             'source.target_max_age_s': .04, 'receiver.lease_ms': 120})
    clock = Clock()
    leaders = Leaders(clock)
    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'left'}, manual_tuning=manual.bundle(),
                           operator_settings=settings.bundle()),
                      clock=clock, open_leaders=lambda _: leaders,
                      open_receiver=lambda config, device, reference: Receiver(config, reference, clock),
                      open_backend=lambda _: pytest.fail('No collision backend'))
    try:
        core.build()
        assert core.profile == resolve_profile(core.base_profile, settings.bundle())
        assert core.derived['config']['lease_ms'] == 120
        lifecycle = core.session.lifecycle
        # The operator's ceilings are the lifecycle's base rows; manual tuning follows within them.
        row = next(j for j in lifecycle._manual_base if j['name'] == 'left_arm_j1')
        assert row['max_velocity'] == ceiling and row['align_speed'] == ceiling / 2
        following = next(j for j in lifecycle.joints if j['name'] == 'left_arm_j1')
        assert following['max_velocity'] <= ceiling
        assert lifecycle.target_max_age_s == .04
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock)
        # The model's joint telemetry comes from the receiver's feedback, in model units, with its age.
        packet = core.view()['telemetry']
        assert packet['stream_id'].startswith('teleop:') and packet['model'] == core.model.profile.id
        rows = {r['motor']: r for r in packet['rows']}
        assert rows['left_arm_j1']['position_unit'] == 'rad' and rows['left_arm_j1']['source'] == 'measured'
        assert rows['left_arm_j1']['age_seconds'] is not None and rows['left_arm_j1']['position'] is not None
        assert not any(motor.startswith('right_') for motor in rows)
    finally:
        core.close()
