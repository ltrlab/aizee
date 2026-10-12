"""Small, clock-explicit direct teleop traces and distribution reports."""
import math


def distribution(values):
    values = sorted(float(v) for v in values if v is not None and math.isfinite(v))
    if not values:
        return None
    return {**{f'p{q}': values[max(0, math.ceil(len(values)*q/100)-1)] for q in (50, 95, 99)},
            'max': values[-1], 'count': len(values)}


def next_scheduled_slot(scheduled_at, completed_at, period_s):
    """Advance after completion, preserving the executed cycle's original deadline."""
    next_at = scheduled_at + period_s
    skipped = int((completed_at-next_at)/period_s)+1 if completed_at >= next_at else 0
    return next_at + skipped*period_s, skipped


def trace_record(record):
    proposal, status = record.get('proposal') or {}, record.get('status') or {}
    target, feedback = record.get('target') or [], record.get('feedback') or []
    started, completed = record.get('started_at'), record.get('completed_at')
    return dict(cycle_ms=None if started is None or completed is None else (completed-started)*1000,
                rpc_ms=record.get('rpc_ms'), rpc_count=record.get('rpc_count'),
                operation=proposal.get('operation'), request_sequence=record.get('request_sequence'),
                source_sequence=[r['sequence'] for r in target],
                source_age_ms=(completed-min(r['sampled_at'] for r in target))*1000 if target and completed is not None else None,
                desired=[r['position'] for r in target], commanded=proposal.get('positions'),
                commanded_velocity=[r['velocity'] for r in proposal.get('joints') or []],
                commanded_effort=[r['effort'] for r in proposal.get('joints') or []],
                measured=[r['position'] for r in feedback], limits=proposal.get('limits'),
                measured_velocity=[r.get('velocity') for r in feedback],
                measured_effort=[r.get('effort') for r in feedback],
                measured_temperature=[r.get('temperature') for r in feedback],
                feedback_sequence=[r.get('sequence') for r in feedback],
                feedback_age_ms=record.get('feedback_age_ms'),
                lifecycle_before=record.get('lifecycle_before'), lifecycle_after=record.get('lifecycle_after'),
                receiver_accepted_ms=record.get('receiver_accepted_ms'),
                receiver_applied=record.get('receiver_applied'),
                receiver_epoch=status.get('epoch'),
                command_expires_at_ms=status.get('command_expires_at_ms'),
                receiver_state=status.get('state'), receiver_reason=status.get('reason'),
                started_at=started, completed_at=completed,
                scheduled_at=record.get('scheduled_at'),
                loop_completed_at=record.get('loop_completed_at'),
                full_loop_ms=record.get('full_loop_ms'),
                start_lateness_ms=record.get('start_lateness_ms'),
                scheduled_deadline_miss=record.get('scheduled_deadline_miss'),
                skipped_slots=record.get('skipped_slots', 0),
                accepted=record.get('accepted', False), failure=record.get('failure'))


def summarize(records, period_s):
    following = [r for r in records if r.get('operation') == 'move']
    # Sequences restart when ownership is acquired again. Receiver boot, sequence
    # and acceptance time together identify the command whose application we saw.
    accepted = {(r.get('receiver_epoch'), r['request_sequence'], r['receiver_accepted_ms'])
                for r in following if r.get('accepted') and r.get('request_sequence') is not None
                and r.get('receiver_accepted_ms') is not None}
    applied = {}
    for record in records:
        target = record.get('receiver_applied')
        if target:
            key = (record.get('receiver_epoch'), target['sequence'], target['accepted_at_ms'])
            if key in accepted and target['applied_at_ms'] >= target['accepted_at_ms']:
                applied.setdefault(key, target)
    scheduled = []
    for record in following:
        missed = record.get('scheduled_deadline_miss')
        if missed is None:
            due = record.get('scheduled_at')
            completed = record.get('loop_completed_at')
            if completed is None:
                completed = record.get('completed_at')
            if due is not None and completed is not None:
                missed = completed > due + period_s
        if missed is not None:
            scheduled.append(bool(missed))
    unique = {tuple(r['source_sequence']) for r in following if r.get('source_sequence')}
    return dict(cycles=len(following), accepted=sum(r['accepted'] for r in following),
                unique_source_samples=len(unique),
                cycle_ms=distribution(r.get('cycle_ms') for r in following),
                full_loop_ms=distribution(r.get('full_loop_ms') for r in following),
                start_lateness_ms=distribution(r.get('start_lateness_ms') for r in following),
                rpc_ms=distribution(r.get('rpc_ms') for r in following),
                poll_ms=distribution(r.get('poll_ms') for r in following),
                map_ms=distribution(r.get('map_ms') for r in following),
                source_age_ms=distribution(r.get('source_age_ms') for r in following),
                receiver_apply_ms=distribution(r['applied_at_ms']-r['accepted_at_ms'] for r in applied.values()),
                receiver_applied_targets=len(applied),
                receiver_unobserved_targets=len(accepted-applied.keys()),
                processing_deadline_misses=sum((r.get('cycle_ms') or 0) >= period_s*1000 for r in following),
                scheduled_cycles=len(scheduled), scheduled_deadline_misses=sum(scheduled),
                skipped_slots=sum(r.get('skipped_slots', 0) for r in following),
                maximum_rpc_count=max((r.get('rpc_count') or 0 for r in following), default=0),
                failures=[r['failure'] for r in records if r.get('failure')])
