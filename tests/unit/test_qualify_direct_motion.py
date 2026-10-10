"""Moving, quantized source contracts are distinct from stationary timing checks."""
import importlib.util
from pathlib import Path

import pytest

from robot_platform.teleop.lifecycle import TeleopLifecycle


@pytest.fixture(scope='module')
def qualification():
    path=Path(__file__).resolve().parents[2]/'scripts/qualify_direct_motion.py'
    spec=importlib.util.spec_from_file_location('qualify_direct_motion',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('case_index', range(8))
def test_quantized_moving_position_references_preserve_planner_and_remove_feedforward_pulses(qualification,case_index):
    case=qualification.cases()[case_index]
    reference,old=qualification.run_case(TeleopLifecycle,case,position_only=False)
    result,new=qualification.run_case(TeleopLifecycle,case,position_only=True)
    assert qualification.acceptance(result)==[]
    assert reference['faults']==[]
    assert len(old)==len(new) and result['moving_duplicates']>0
    assert [r['source_sequence'] for r in new]==[r['source_sequence'] for r in old]
    assert [r['source_sampled_at'] for r in new]==[r['source_sampled_at'] for r in old]
    assert [r['position'] for r in new]==pytest.approx([r['position'] for r in old],abs=1e-12,rel=0)
    assert [r['reference_velocity'] for r in new]==pytest.approx([r['reference_velocity'] for r in old],abs=1e-12,rel=0)
    assert any(abs(r['reference_velocity'])>.01 for r in new)
    assert all(r['commanded_velocity']==0 and r['ramp_effort']==0 for r in new)
    assert all(r['commanded_effort']==pytest.approx(r['gravity_effort'],abs=1e-12) for r in new)
    # Include the previously unaddressed case: a duplicate arriving after the
    # old quantized target was already reached while the leader keeps moving.
    reached=[r for r in new if r['source_moving'] and r['repeated_source'] and r['goal_already_reached']]
    if case['speed']==.08:
        assert reached
    assert all(abs(r['delta_non_gravity_command_term'])<=1e-12 for r in reached)


def test_gate_detects_old_feedforward_pulses_even_when_motion_completes(qualification):
    result,trace=qualification.run_case(TeleopLifecycle,qualification.cases()[0],position_only=False)
    assert result['faults']==[] and result['final_state']=='DISABLED'
    assert result['reached_goal_repeat_zero_velocity_drops']>0
    assert result['repeat_non_gravity_step_abs']['max']>1.
    failed=qualification.acceptance(result)
    assert 'nonzero velocity feedforward' in failed
    assert 'repeat-linked command torque pulse' in failed
    assert trace[-1]['segment']=='final_hold'


def test_position_only_qualification_preserves_source_and_feedback_fail_closed_behavior(qualification):
    results=qualification.freshness_checks(TeleopLifecycle)
    assert set(results)=={'stale_source','stale_feedback','changed_duplicate','changed_timestamp',
                          'backwards_sequence','missing_requires_reengage'}
    assert all(row['passed'] for row in results.values()),results


def test_qualification_inputs_are_reproducible_and_explicitly_synthetic(qualification):
    assert qualification.cases(17)==qualification.cases(17)
    assert qualification.cases(17)!=qualification.cases(18)
    joint=qualification.synthetic_joint(position_only=True)
    assert joint['position_only'] is True and joint['direct_velocity'] is True
    assert qualification.SOURCE_HZ==90 and qualification.DT==.01
    assert qualification.QUANTUM>0 and qualification.SOURCE_DELAY>0
