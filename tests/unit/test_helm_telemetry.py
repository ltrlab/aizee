"""The model's joint telemetry: measured values keep their identity, units and age; nothing is invented."""
import math

import pytest

from robot_platform.helm.telemetry import (STALE_AFTER_S, age_packet, empty_packet, native_packet,
                                            select_telemetry, stopped_packet)

JOINT_TYPES = {'left_arm_j1': {'type': 'revolute'}, 'left_slide': {'type': 'prismatic'}}
DERIVED = {'config': {'joints': [{'name': 'left_arm_j1', 'lower': -1., 'upper': 2., 'max_effort': 60.,
                                  'max_temperature': 70.}, {'name': 'left_gripper'}]},
           'joint_map': {'left_arm_j1': {'actuator': 'left_arm_j1', 'scale': -1., 'offset': .5},
                         'left_slide': {'actuator': 'left_gripper', 'scale': .01, 'offset': 0.}},
           'report': [{'motor': 'left_arm_j1', 'direction_verified': True, 'home': .1},
                      {'motor': 'left_gripper', 'direction_verified': False}]}


def receiver_result(now_ms=1000, sampled_ms=990, fault=False):
    return {'status': {'now_ms': now_ms, 'epoch': 'epoch-1', 'joint_names': ['left_arm_j1', 'left_gripper']},
            'feedback': [{'position': .2, 'velocity': .3, 'effort': -1.5, 'temperature': 31., 'fault': fault,
                          'sampled_at_ms': sampled_ms},
                         {'position': 2., 'velocity': -1., 'effort': .4, 'temperature': 28., 'fault': False,
                          'sampled_at_ms': sampled_ms}]}


def native(result, **kwargs):
    return native_packet(result, DERIVED, model='minerva', urdf_sha256='abc', joint_types=JOINT_TYPES,
                         labels={'left_arm_j1': 'Left J1'}, **kwargs)


def test_receiver_feedback_maps_into_model_units_with_its_own_age():
    packet = native(receiver_result())
    rows = {row['joint']: row for row in packet['rows']}
    arm, slide = rows['left_arm_j1'], rows['left_slide']
    # Model position is scale x motor + offset; velocity scales too; effort and temperature pass through.
    assert arm['position'] == pytest.approx(-1. * .2 + .5) and arm['position_unit'] == 'rad'
    assert arm['velocity'] == pytest.approx(-.3) and arm['velocity_unit'] == 'rad/s'
    assert arm['effort'] == -1.5 and arm['effort_unit'] == 'Nm' and arm['effort_label'] == 'Drive torque'
    assert arm['temperature_c'] == 31. and arm['label'] == 'Left J1' and arm['mapping_verified'] is True
    assert slide['position'] == pytest.approx(.02) and slide['position_unit'] == 'm'
    assert slide['mapping_verified'] is False
    # Age from the receiver's own clock: now_ms - sampled_at_ms.
    assert arm['age_seconds'] == pytest.approx(.01) and not arm['stale'] and not packet['stale']
    assert packet['stream_id'] == 'receiver:epoch-1' and packet['sequence'] == 1000
    assert native(receiver_result(), source='simulated')['rows'][0]['effort_label'] == 'Simulated drive torque'


def test_receiver_interval_reference_and_limits_map_into_model_units_for_gauges():
    rows = {row['joint']: row for row in native(receiver_result())['rows']}
    arm, slide = rows['left_arm_j1'], rows['left_slide']
    # A negative scale swaps the ends: -1 x [-1, 2] + .5 is [-1.5, 1.5], low end first.
    assert arm['range'] == pytest.approx([-1.5, 1.5]) and arm['reference'] == pytest.approx(.4)
    assert arm['effort_limit'] == 60. and arm['temperature_limit_c'] == 70.
    # Missing config stays missing: no range, reference or limits are made up.
    assert slide['range'] is None and slide['reference'] is None
    assert slide['effort_limit'] is None and slide['temperature_limit_c'] is None


def test_a_faulted_old_or_missing_reading_is_stale_and_nothing_is_filled_in():
    assert native(receiver_result(fault=True))['rows'][0]['stale'] is True
    old = native(receiver_result(now_ms=2000, sampled_ms=1000))
    assert all(row['stale'] for row in old['rows']) and old['stale']
    missing = native({'status': {'now_ms': 5}, 'feedback': []})
    assert all(row['position'] is None and row['effort'] is None and row['stale'] for row in missing['rows'])
    nan = receiver_result()
    nan['feedback'][0]['effort'] = math.nan
    assert native(nan)['rows'][0]['effort'] is None and native(nan)['rows'][0]['effort_unit'] is None
    with pytest.raises(ValueError):
        native(receiver_result(), source='guessed')


def test_the_stopped_reader_supplies_positions_only():
    mapping = {'left_arm_j1': {'motor': 'left_arm_j1', 'sign': -1, 'offset_rad': .5, 'direction_verified': True}}
    transmissions = [{'motor': 'left_gripper', 'joint': 'left_slide', 'metres_per_motor_rad': .01,
                      'joint_reference_m': 0., 'motor_reference_rad': 0., 'direction_verified': True}]
    frame = {'motors': [{'name': 'left_arm_j1', 'position_rad': .2}, {'name': 'left_gripper', 'position_rad': 2.}],
             'roundtrip_seconds': .05, 'request_id': 'r1', 'sequence': 7}
    packet = stopped_packet(frame, mapping, transmissions, model='minerva', urdf_sha256='abc',
                            joint_types=JOINT_TYPES, cache_age_s=.1, reading=True)
    rows = {row['joint']: row for row in packet['rows']}
    assert rows['left_arm_j1']['position'] == pytest.approx(.3)
    assert rows['left_slide']['position'] == pytest.approx(.02)
    assert all(row['velocity'] is None and row['effort'] is None and row['temperature_c'] is None
               for row in packet['rows'])
    assert rows['left_arm_j1']['age_seconds'] == pytest.approx(.15) and not packet['stale']
    # Not reading (disconnected or faulted): the last positions are shown stale.
    assert stopped_packet(frame, mapping, transmissions, model='minerva', urdf_sha256='abc',
                          joint_types=JOINT_TYPES, cache_age_s=.1, reading=False)['stale']
    assert stopped_packet(None, mapping, [], model='m', urdf_sha256='u', joint_types=JOINT_TYPES)['stale']
    # The swept range and reference map like positions; the stopped reader has no load limits to give.
    gauged = stopped_packet(frame, mapping, transmissions, model='minerva', urdf_sha256='abc', joint_types=JOINT_TYPES,
                            reading=True, ranges={'left_arm_j1': (-1., 2.), 'left_gripper': (0., 4.)},
                            references={'left_arm_j1': 0., 'left_gripper': 0.})
    rows = {row['joint']: row for row in gauged['rows']}
    assert rows['left_arm_j1']['range'] == pytest.approx([-1.5, 1.5]) and rows['left_arm_j1']['reference'] == .5
    assert rows['left_slide']['range'] == pytest.approx([0., .04]) and rows['left_slide']['reference'] == 0.
    assert all(row['effort_limit'] is None and row['temperature_limit_c'] is None for row in gauged['rows'])
    assert all(row['range'] is None and row['reference'] is None for row in packet['rows'])


def test_ageing_a_cached_packet_never_makes_it_fresher():
    packet = native(receiver_result())
    aged = age_packet(packet, STALE_AFTER_S)
    assert aged['rows'][0]['age_seconds'] == pytest.approx(.01 + STALE_AFTER_S) and aged['stale']
    assert packet['rows'][0]['age_seconds'] == pytest.approx(.01)  # The original is untouched.
    assert age_packet(packet, None)['stale'] and age_packet(None, 0.)['rows'] == []


def test_the_view_shows_one_source_and_an_active_session_without_data_clears_it():
    baseline = dict(empty_packet(model='minerva', urdf_sha256='abc', stream_id='stopped:1'), stale=False)
    session = native(receiver_result())
    assert select_telemetry({'telemetry': baseline})['stream_id'] == 'stopped:1'
    assert select_telemetry({'telemetry': baseline, 'teleop': {'phase': 'stopped', 'telemetry': session}}
                            )['stream_id'] == 'stopped:1'
    assert select_telemetry({'telemetry': baseline, 'teleop': {'phase': 'running', 'telemetry': session}}
                            )['stream_id'] == 'receiver:epoch-1'
    starting = select_telemetry({'telemetry': baseline, 'tune': {'phase': 'starting'}})
    assert starting['rows'] == [] and starting['stale'] and starting['stream_id'] == 'tune:unavailable'
    assert starting['model'] == 'minerva'
