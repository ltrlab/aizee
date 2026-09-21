from dataclasses import replace

import pytest

from robot_platform.characterization.limit_probe import (
    ContactDetector, ProbeConfig, ProbeSample, synthetic_validation,
)


def config(**changes):
    return replace(ProbeConfig(1, 0, .4, 1.5, .2, 5, .001, .005, .005, .002,
                               4, .5, .2, .04, .05, 50), **changes)


def sample(i, **changes):
    t = i * .02
    return replace(ProbeSample(t, t, i, "run1", .05, 0, .6, .06 + .02*t, 25, True), **changes)


def test_synthetic_decisions_both_directions_and_negative_controls():
    report = synthetic_validation()
    assert report["passed"], report
    assert report["hardware_commands_sent"] == 0
    assert not report["physical_model_validated"]
    assert all(not result["accepted_limit"] for result in report["cases"].values())
    assert report["cases"]["upper_stop"]["position_rad"] == pytest.approx(.05)
    assert report["cases"]["lower_stop"]["position_rad"] == pytest.approx(-.05)


@pytest.mark.parametrize("change,reason", [
    ({"sequence": 0}, "feedback_not_advancing"),
    ({"producer_id": "run2"}, "producer_changed"),
    ({"source_time_s": -.1}, "stale_or_future_feedback"),
    ({"source_time_s": .03}, "stale_or_future_feedback"),
    ({"time_s": .2, "source_time_s": .2}, "feedback_gap"),
    ({"time_s": 0, "source_time_s": .001}, "stale_or_future_feedback"),
    ({"position_rad": float("nan")}, "invalid_numeric_sample"),
    ({"torque_nm": 1.5}, "torque_cap"),
    ({"torque_nm": -1.5}, "torque_cap"),
    ({"velocity_rad_s": -.2}, "velocity_cap"),
    ({"temperature_c": 50}, "temperature_cap"),
    ({"collision_clear": None}, "collision_or_unknown_scene"),
    ({"collision_clear": 1}, "collision_or_unknown_scene"),
    ({"fault": True}, "motor_fault_or_unknown"),
    ({"target_rad": -.5}, "command_reversed"),
    ({"target_rad": .55}, "travel_cap"),
])
def test_faults_abort_before_contact_and_latch(change, reason):
    detector = ContactDetector(config())
    detector.observe(sample(0))
    result = detector.observe(sample(1, **change))
    assert (result.state, result.reason) == ("aborted", reason)
    assert detector.observe(sample(100)).reason == reason


@pytest.mark.parametrize("change", [
    {"torque_nm": .2}, {"torque_nm": -.6}, {"target_rad": .05},
    {"target_rad": .1}, {"velocity_rad_s": .01},
])
def test_stationary_target_or_inconsistent_evidence_never_establishes_contact(change):
    detector = ContactDetector(config())
    for i in range(40):
        assert detector.observe(sample(i, **change)).state == "probing"


def test_torque_spike_does_not_fill_a_sustained_window():
    detector = ContactDetector(config())
    for i in range(40):
        assert detector.observe(sample(i, torque_nm=.6 if i == 20 else .1)).state == "probing"


def test_short_window_cannot_be_replaced_with_many_samples():
    detector = ContactDetector(config())
    for i in range(100):
        t = i * .001
        assert detector.observe(sample(i, time_s=t, source_time_s=t)).state == "probing"


def test_duration_cap_and_explicit_no_message_abort():
    detector = ContactDetector(config(max_duration_s=.3))
    for i in range(20):
        result = detector.observe(sample(i, torque_nm=.1))
    assert result.reason == "duration_cap"
    assert ContactDetector(config()).abort("no_feedback").reason == "no_feedback"


@pytest.mark.parametrize("change", [{"direction": 0}, {"min_samples": 2},
    {"window_s": float("nan")}, {"max_travel_rad": -1}, {"torque_abort_nm": .3},
    {"baseline_torque_nm": 1.2}, {"window_s": 5}, {"max_velocity_rad_s": .001}])
def test_invalid_thresholds_rejected(change):
    with pytest.raises(ValueError):
        config(**change)
