import assert from 'node:assert/strict';
import {ReplayModel, formatSeconds, groupEpisodes, reviewCounts, sessionDate, shownBy, SPEEDS, verdictOf} from '../../src/robot_platform/helm/static/replay.js';

// Uneven gaps, as recording waits for a new frame from every camera: playback must follow the recorded times.
const times = [0, .05, .16, .2, .3];
const model = new ReplayModel();
model.load(times);
assert.equal(model.length, 5);
assert.equal(model.duration, .3);
for (const [t, index] of [[-1, 0], [0, 0], [.049, 0], [.05, 1], [.1, 1], [.16, 2], [.25, 3], [.3, 4], [9, 4]]) {
  assert.equal(model.indexAt(t), index, `indexAt(${t})`);
}

// Real time at speed 1, then twice as fast from the moment the speed changes.
assert.equal(model.play(1000), true);
assert.equal(model.tick(1000 + 100), true);
assert.equal(model.index, 1);
assert.equal(model.tick(1000 + 120), false);  // Still frame 1: nothing new to show.
assert.equal(model.tick(1000 + 160), true);
assert.equal(model.index, 2);
model.setSpeed(2, 1160);
model.tick(1160 + 50);  // .16 s + 2 x 50 ms = .26 s
assert.equal(model.index, 3);
assert.equal(model.tick(1160 + 1000), true);  // Past the end: the last frame, and playback stops.
assert.equal(model.index, 4);
assert.equal(model.playing, false);
assert.equal(model.atEnd, true);

// Play at the end starts over; seeking while playing restarts the clock there.
model.setSpeed(1, 0);
model.play(5000);
assert.equal(model.index, 0);
model.seek(3, 5010);
model.tick(5010 + 50);  // .2 s + 50 ms = .25 s
assert.equal(model.index, 3);

// Stepping pauses and clamps; seeks clamp and tolerate junk.
model.step(1);
assert.equal(model.playing, false);
assert.equal(model.index, 4);
model.step(1);
assert.equal(model.index, 4);
for (const [target, index] of [[-5, 0], [99, 4], [NaN, 0], ['2', 2], [1.6, 2]]) assert.equal(model.seek(target, 0), index);
assert.equal(model.seekTime(.17, 0), 2);
assert.equal(model.time, .16);

// Toggle, invalid speeds, and an empty or one-frame episode.
assert.equal(model.toggle(0), true);
assert.equal(model.toggle(0), false);
assert.throws(() => model.setSpeed(0, 0), RangeError);
assert.throws(() => model.setSpeed(-1, 0), RangeError);
assert.throws(() => model.setSpeed(Infinity, 0), RangeError);
const empty = new ReplayModel();
assert.equal(empty.play(0), false);
assert.equal(empty.tick(1000), false);
assert.equal(empty.time, 0);
assert.equal(empty.duration, 0);
empty.load([3]);
assert.equal(empty.play(0), false);
assert.deepEqual(SPEEDS, [0.25, 0.5, 1, 2, 4]);

// Labels.
assert.equal(formatSeconds(4.25), '4.3 s');
assert.equal(formatSeconds(64.45), '1:04.5');
assert.equal(formatSeconds(NaN), '—');
assert.equal(sessionDate('live-20261021T004507Z').toISOString(), '2026-10-21T00:45:07.000Z');
assert.equal(sessionDate('tune-20261021T004507Z'), null);

// Sessions newest first; episodes numbered in recording order, listed newest first.
const groups = groupEpisodes([
  {id: 'live-20261020T235949Z/a', session: 'live-20261020T235949Z', modified: 5},
  {id: 'live-20261021T004507Z/b', session: 'live-20261021T004507Z', modified: 30},
  {id: 'live-20261021T004507Z/c', session: 'live-20261021T004507Z', modified: 10},
  {id: 'live-20261021T004507Z/d', session: 'live-20261021T004507Z', modified: 20},
]);
assert.deepEqual(groups.map(group => group.session), ['live-20261021T004507Z', 'live-20261020T235949Z']);
assert.deepEqual(groups[0].episodes.map(episode => [episode.id.slice(-1), episode.number]), [['b', 3], ['d', 2], ['c', 1]]);
assert.deepEqual(groupEpisodes(undefined), []);
console.log('replay model checks passed');

// Training review: each episode's verdict, the counts and the list filter.
const reviewed = [
  {id: 'a', review: {verdict: 'keep', note: 'clean'}},
  {id: 'b', review: {verdict: 'discard'}},
  {id: 'c', review: {verdict: null}},  // Cleared: back to review.
  {id: 'd'},
  {id: 'e', review: {verdict: 'maybe'}},  // Not a verdict this page knows: still to review.
];
assert.deepEqual(reviewed.map(verdictOf), ['keep', 'discard', null, null, null]);
assert.deepEqual(reviewCounts(reviewed), {keep: 1, discard: 1, unreviewed: 3});
assert.deepEqual(reviewCounts([]), {keep: 0, discard: 0, unreviewed: 0});
const ids = filter => reviewed.filter(episode => shownBy(filter, episode)).map(episode => episode.id);
assert.deepEqual(ids('all'), ['a', 'b', 'c', 'd', 'e']);
assert.deepEqual(ids(undefined), ['a', 'b', 'c', 'd', 'e']);
assert.deepEqual(ids('keep'), ['a']);
assert.deepEqual(ids('discard'), ['b']);
assert.deepEqual(ids('unreviewed'), ['c', 'd', 'e']);
console.log('replay review checks passed');

