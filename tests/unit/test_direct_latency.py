"""Timing reports distinguish scheduling, processing and correlated application."""
import pytest

from robot_platform.teleop.latency import next_scheduled_slot, summarize, trace_record


def move(sequence=2, accepted_at=100, epoch='boot-a', **values):
    return dict(operation='move', accepted=True, request_sequence=sequence,
                receiver_accepted_ms=accepted_at, receiver_epoch=epoch, **values)


def applied(sequence=2, accepted_at=100, applied_at=103):
    return dict(sequence=sequence, accepted_at_ms=accepted_at, applied_at_ms=applied_at)


def test_late_start_can_miss_a_deadline_with_fast_processing():
    record = move(cycle_ms=4., scheduled_at=10., started_at=10.007,
                  completed_at=10.011, start_lateness_ms=7.)
    result = summarize([record], .01)
    assert result['processing_deadline_misses'] == 0
    assert result['scheduled_deadline_misses'] == 1
    assert result['scheduled_cycles'] == 1
    assert result['start_lateness_ms']['p99'] == 7.


def test_skipped_slots_do_not_shift_schedule_or_count_as_completed_cycles():
    due, skipped = next_scheduled_slot(0., .027, .01)
    assert due == pytest.approx(.03)
    assert skipped == 2
    result = summarize([move(cycle_ms=1., scheduled_at=0., completed_at=.027,
                             skipped_slots=skipped)], .01)
    assert result['cycles'] == 1
    assert result['skipped_slots'] == 2
    assert result['scheduled_deadline_misses'] == 1
    next_due, skipped = next_scheduled_slot(due, .031, .01)
    assert next_due == pytest.approx(.04) and skipped == 0


def test_application_identity_survives_sequence_reuse_and_receiver_restart():
    first = applied(2, 100, 102)
    second = applied(2, 200, 205)
    rebooted = applied(2, 200, 203)
    records = [move(2, 100, receiver_applied=first),
               move(2, 200, receiver_applied=second),
               move(2, 200, epoch='boot-b', receiver_applied=rebooted),
               dict(operation='status', receiver_epoch='boot-b', receiver_applied=rebooted),
               dict(operation='status', receiver_epoch='boot-a',
                    receiver_applied=applied(999, 300, 999))]
    result = summarize(records, .01)
    assert result['cycles'] == result['accepted'] == 3
    assert result['receiver_applied_targets'] == 3
    assert result['receiver_unobserved_targets'] == 0
    assert result['receiver_apply_ms']['count'] == 3
    assert result['receiver_apply_ms']['max'] == 5


def test_final_status_closes_deferred_application_without_adding_a_cycle():
    record = move(receiver_applied=None)
    result = summarize([record], .01)
    assert result['receiver_apply_ms'] is None
    assert result['receiver_unobserved_targets'] == 1
    final = dict(operation='status', receiver_epoch='boot-a', receiver_applied=applied())
    result = summarize([record, final], .01)
    assert result['cycles'] == result['accepted'] == 1
    assert result['receiver_unobserved_targets'] == 0
    assert result['receiver_apply_ms']['p99'] == 3


def test_application_from_an_earlier_request_is_not_evidence_for_this_one():
    record = move(accepted_at=200, receiver_applied=applied(2, 100, 103))
    result = summarize([record], .01)
    assert result['receiver_apply_ms'] is None
    assert result['receiver_applied_targets'] == 0
    assert result['receiver_unobserved_targets'] == 1


def test_full_loop_and_processing_metrics_remain_separate():
    records = [move(cycle_ms=2., rpc_ms=1., full_loop_ms=12.,
                    scheduled_at=10., completed_at=10.002, loop_completed_at=10.012),
               move(3, 110, cycle_ms=2., rpc_ms=1., full_loop_ms=3.,
                    scheduled_at=20., completed_at=20.012, scheduled_deadline_miss=False)]
    result = summarize(records, .01)
    assert result['cycle_ms']['p99'] == 2.
    assert result['rpc_ms']['p99'] == 1.
    assert result['full_loop_ms']['p99'] == 12.
    assert result['processing_deadline_misses'] == 0
    assert result['scheduled_deadline_misses'] == 1
    assert result['scheduled_cycles'] == 2


def test_trace_record_keeps_existing_fields_and_optional_schedule_metadata():
    record = dict(started_at=1., completed_at=1.002, accepted=True,
                  proposal=dict(operation='move', positions=[.1]),
                  status=dict(epoch='boot-a'), request_sequence=2,
                  receiver_accepted_ms=100, receiver_applied=applied(),
                  scheduled_at=1., full_loop_ms=4., loop_completed_at=1.004,
                  scheduled_deadline_miss=False, skipped_slots=2)
    trace = trace_record(record)
    assert trace['cycle_ms'] == pytest.approx(2.)
    assert trace['commanded'] == [.1]
    assert trace['source_sequence'] == [] and trace['source_age_ms'] is None
    assert trace['receiver_epoch'] == 'boot-a'
    assert trace['scheduled_at'] == 1. and trace['full_loop_ms'] == 4.
    assert trace['skipped_slots'] == 2 and trace['scheduled_deadline_miss'] is False
