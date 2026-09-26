"""Hand-swept leader ranges: continuous unwrap, agreed ends and the saved, bound file."""
import json
import math

import pytest

from robot_platform.teleop.leader_ranges import (MAX_SPAN_RAD, LeaderRangeRecorder, load_ranges, range_table,
                                                 reference_table, validate_ranges, with_reference, wrap)
from robot_platform.teleop.openrb import LeaderSample

PROFILE = {'id': 'fixture-leaders', 'leaders': {'left': {'usb_serial': 'A1', 'joints': [
    {'name': 'channel_1', 'label': 'Shoulder swivel'}, {'name': 'channel_2', 'label': 'Gripper trigger'}]}}}
KEY, TRIGGER = 'left.channel_1', 'left.channel_2'


class Feed:
    """Pairs every 20 ms; channel 2 stays still unless told otherwise."""
    def __init__(self, recorder):
        self.recorder, self.now, self.sequence = recorder, 100., 0

    def read(self, first, second=1.):
        self.sequence += 1
        self.now += .02
        sample = LeaderSample((first % (2 * math.pi), second), (0, 0), self.sequence, self.now, self.now,
                              (0, 0, 1, 1), 1, 2)
        self.recorder.observe({'left': sample}, now=self.now)

    def hold(self, value, seconds=.4):
        for _ in range(round(seconds / .02)):
            self.read(value)

    def move(self, start, end, step=.05):
        count = max(1, math.ceil(abs(end - start) / step))
        for i in range(1, count + 1):
            self.read(start + (end - start) * i / count)


def stats(recorder, key=KEY):
    return next(row for row in recorder.rows() if row['key'] == key)


def test_a_sweep_across_the_encoder_seam_records_the_true_arc():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.hold(5.5)
    feed.move(5.5, 2 * math.pi + .9)  # Readings pass 2pi and restart near 0.
    feed.hold(2 * math.pi + .9)
    row = stats(recorder)
    assert row['span_rad'] == pytest.approx(2 * math.pi + .9 - 5.5)
    assert row['problem'] is None and row['fraction'] == pytest.approx(1.)
    entry = recorder.confirm(KEY)
    assert entry['span_rad'] == pytest.approx(row['span_rad'])
    # At runtime a reading r is center + wrap(r - center): both ends land on their side.
    center = entry['center_rad']
    ends = sorted(center + wrap(q - center) for q in (5.5, .9))
    assert ends == pytest.approx([center - entry['span_rad'] / 2, center + entry['span_rad'] / 2])


def test_sweeping_through_an_end_counts_without_pausing():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.read(0.)
    with pytest.raises(ValueError, match='both ends'):
        recorder.confirm(KEY)
    feed.move(0., 1.)
    feed.move(1., -1.)  # Straight through both ends, 0.05 rad per reading.
    feed.move(-1., 0.)
    row = stats(recorder)
    # Three readings in a row reach within one step of each turnaround.
    assert (row['low'], row['high']) == pytest.approx((-.95, .95))
    assert row['problem'] is None


def test_a_glitch_shorter_than_three_readings_cannot_widen_a_range():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.hold(0., .1)
    feed.move(0., .5)
    feed.hold(.5, .1)
    for glitch in ([1.2], [1.2, 1.3]):  # Under the 1 rad jump limit, so they are observed.
        for value in glitch:
            feed.read(value)
        feed.hold(.5, .1)
    row = stats(recorder)
    assert row['seen_high'] == pytest.approx(1.3) and row['high'] == pytest.approx(.5)
    assert row['jumps'] == 0


def test_a_glitch_is_ignored_but_a_jump_that_persists_discards_the_sweep():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.hold(0.)
    feed.move(0., .8)
    feed.hold(.8)
    recorder.confirm(KEY)
    feed.read(3.)  # One bad reading.
    feed.hold(.8, .1)
    row = stats(recorder)
    assert row['jumps'] == 1 and row['span_rad'] == pytest.approx(.8) and row['confirmed']
    for _ in range(3):
        feed.read(3.)
    row = stats(recorder)
    assert row['discarded'] == 1 and row['span_rad'] is None and row['confirmed'] is None
    assert row['current'] == pytest.approx(3.)


def test_a_reconnect_continues_a_small_move_and_discards_a_large_one():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.hold(0.)
    feed.move(0., .8)
    feed.hold(.8)
    recorder.restart()
    feed.read(.5)
    assert stats(recorder)['span_rad'] == pytest.approx(.8) and stats(recorder)['discarded'] == 0
    recorder.restart()
    feed.read(2.)
    assert stats(recorder)['discarded'] == 1 and stats(recorder)['span_rad'] is None


def test_a_pair_without_new_data_cannot_fake_a_hold():
    recorder = LeaderRangeRecorder(PROFILE)
    sample = LeaderSample((.3, 1.), (0, 0), 7, 100., 100., (0, 0, 1, 1), 1, 2)
    for step in range(30):
        recorder.observe({'left': sample}, now=100. + step * .02)
    row = stats(recorder)
    assert row['samples'] == 1 and row['low'] is None
    with pytest.raises(ValueError, match='complete leader roster'):
        recorder.observe({}, now=101.)


def test_a_nearly_full_turn_is_refused():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.hold(0.)
    feed.move(0., MAX_SPAN_RAD + .1)
    feed.hold(MAX_SPAN_RAD + .1)
    assert 'nearly a full turn' in stats(recorder)['problem']


def test_saved_ranges_merge_and_bind_to_the_profile_and_usb_serials(tmp_path):
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.hold(0.)
    feed.move(0., .8)
    feed.hold(.8)
    recorder.confirm(KEY)
    existing = {'leaders': {'left': {'channel_2': {'channel': 2, 'label': 'Gripper trigger',
                                                   'center_rad': 1.2, 'span_rad': .6}}}}
    document = validate_ranges(recorder.document(PROFILE, existing=existing, session='run-1'), PROFILE)
    assert document['pins'] == {'left': 'A1'} and document['runtime_approved'] is False
    assert document['leaders']['left']['channel_1']['center_rad'] == pytest.approx(.4)
    assert document['leaders']['left']['channel_1']['session'] == 'run-1'
    assert range_table(document, PROFILE) == {'left': [pytest.approx((.4, .8)), (1.2, .6)]}
    assert range_table(None, PROFILE) == {'left': [None, None]}

    path = tmp_path / 'ranges.json'
    assert load_ranges(path, PROFILE) == (None, None)
    path.write_text(json.dumps(document))
    assert load_ranges(path, PROFILE)[0] == document
    moved = {**PROFILE, 'leaders': {'left': {**PROFILE['leaders']['left'], 'usb_serial': 'B2'}}}
    assert 'USB serials differ' in load_ranges(path, moved)[1]
    path.write_text(json.dumps({**document, 'kind': 'other'}))
    assert 'different kind' in load_ranges(path, PROFILE)[1]
    broken = json.loads(json.dumps(document))
    broken['leaders']['left']['channel_1']['span_rad'] = 0.
    with pytest.raises(ValueError, match='is invalid'):
        validate_ranges(broken, PROFILE)


def test_a_saved_range_is_shown_in_the_current_readings_turn():
    saved = {TRIGGER: (6.1, .6)}
    recorder = LeaderRangeRecorder(PROFILE, saved=saved)
    feed = Feed(recorder)
    feed.read(0., .05)  # The trigger reads just past the seam: 6.33 in the saved range's turn.
    row = stats(recorder, TRIGGER)
    assert (row['saved']['low'], row['saved']['high']) == pytest.approx((-.483, .117), abs=1e-3)
    assert row['fraction'] == pytest.approx((.05 + .483) / .6, abs=1e-3)


def test_the_leader_zero_covers_every_channel_and_survives_later_saves():
    recorder = LeaderRangeRecorder(PROFILE)
    feed = Feed(recorder)
    feed.read(.4, 1.2)
    assert recorder.readings() == {KEY: pytest.approx(.4), TRIGGER: pytest.approx(1.2)}
    first = with_reference(None, PROFILE, {'left': {'channel_1': .4, 'channel_2': 1.2 + 2 * math.pi}},
                           session='run-1', at='now')
    assert first['leaders'] == {} and first['reference']['captured_at'] == 'now'
    assert reference_table(first, PROFILE) == {'left': [pytest.approx(.4), pytest.approx(1.2)]}
    assert reference_table(None, PROFILE) is None
    feed.hold(.4)
    feed.move(.4, 1.4)
    feed.hold(1.4)
    recorder.confirm(KEY)
    second = recorder.document(PROFILE, existing=first, session='run-2')  # Saving ranges keeps the zero.
    assert second['reference'] == first['reference'] and 'channel_1' in second['leaders']['left']
    third = with_reference(second, PROFILE, {'left': {'channel_1': 1., 'channel_2': 1.}}, at='later')
    assert third['leaders'] == second['leaders'] and reference_table(third, PROFILE) == {'left': [1., 1.]}
    partial = {**first, 'reference': {'leaders': {'left': {'channel_1': .4}}}}
    with pytest.raises(ValueError, match='every leader channel'):
        validate_ranges(partial, PROFILE)
    with pytest.raises(ValueError, match='Confirm at least one'):
        LeaderRangeRecorder(PROFILE).document(PROFILE)
