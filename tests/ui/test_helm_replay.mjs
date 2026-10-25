import assert from 'node:assert/strict';
import {ReplayModel, formatSeconds, groupEpisodes, replayTelemetry, reviewCounts, sessionDate, shownBy, SPEEDS, verdictOf} from '../../src/robot_platform/helm/static/replay.js';

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


// Replay on Minerva: what it would drive, its playback position and the next step, and Teleop's card.
const live = await import('../../src/robot_platform/helm/static/replay.js');
assert.equal(live.receiverLabel(null), 'No teleop profile');
assert.equal(live.receiverLabel({receiver_choice: {mode: 'live', arm: 'both'}, target: 'jetson'}), 'LIVE · both arms · motors move');
assert.equal(live.receiverLabel({receiver_choice: {mode: 'live', arm: 'left'}}), 'LIVE · left arm · motors move');
assert.equal(live.receiverLabel({receiver_choice: {mode: 'virtual'}, target: 'jetson'}, 'robot'),
  "Virtual receiver · loop on Minerva's Jetson · no motors move");
assert.equal(live.receiverLabel({target: 'pc'}, 'simulation'), 'Simulated Minerva · loop on this PC · no motors move');

const playing = {episode: 's/e', time_s: 1.5, duration_s: 6, frame: 22, frames: 91, speed: .5, finished: false};
assert.deepEqual(live.replayProgress(playing), {fraction: .25, text: '1.5 s / 6.0 s · frame 23 of 91 · 0.5×'});
assert.equal(live.replayProgress({...playing, time_s: 6, frame: 90, finished: true}).text, '6.0 s / 6.0 s · frame 91 of 91 · 0.5× · finished');
assert.deepEqual(live.replayProgress({episode: 's/e', speed: 1, time_s: null, duration_s: null}),
  {fraction: 0, text: '1× · reading the episode…'});  // Starting: the loop has not read it yet.
assert.deepEqual(live.replayProgress(null), {fraction: 0, text: ''});

assert.match(live.replayNote('DISABLED', {...playing, time_s: 0}), /^Idle \(I\)/);
assert.match(live.replayNote('IDLE', playing), /^Enable \(E\)/);
assert.match(live.replayNote('HOLD', {...playing, time_s: 0}), /first frame at the alignment speed/);
assert.match(live.replayNote('HOLD', playing), /^Paused/);
assert.match(live.replayNote('ENGAGING', playing), /^Aligning/);
assert.match(live.replayNote('TELEOP', playing), /^Replaying/);
assert.match(live.replayNote('HOLD', {...playing, finished: true}), /^Finished: .*from the start/);
assert.equal(live.replayNote('TELEOP', null), '');

// Teleop's card: one recorded frame per camera at the playback frame, each camera loading only its latest.
const node = tag => ({tag, children: [], dataset: {}, style: {}, attributes: {}, hidden: false, textContent: '',
  append(...children) { this.children.push(...children); },
  replaceChildren(...children) { this.children = children; },
  setAttribute(name, value) { this.attributes[name] = value; }});
const doc = {createElement: node};
const elements = Object.fromEntries(['card', 'title', 'time', 'track', 'fill', 'frames', 'note'].map(key => [key, node('div')]));
const card = new live.LiveReplayCard({elements, doc});
card.render(null);
assert.equal(elements.card.hidden, true);
card.render(playing, {state: 'TELEOP', label: 'Episode 3 · Oct 21'});
assert.equal(elements.card.hidden, false);
assert.equal(elements.title.textContent, 'Episode 3 · Oct 21');
assert.equal(elements.fill.style.width, '25.0%');
assert.equal(elements.track.attributes['aria-valuenow'], '25');
assert.match(elements.note.textContent, /^Replaying/);
const images = () => elements.frames.children.map(figure => figure.children[0]);
assert.deepEqual(images().map(img => img.src), ['head', 'right_wrist', 'left_wrist'].map(role => live.frameUrl('s/e', 22, role)));
card.render({...playing, frame: 23, time_s: 1.55}, {state: 'TELEOP'});
assert.equal(images()[0].src, live.frameUrl('s/e', 22, 'head'));  // Still loading frame 22: 23 waits.
images()[0].onload();
assert.equal(images()[0].src, live.frameUrl('s/e', 23, 'head'));
assert.equal(elements.title.textContent, 's/e');  // No label: the episode's id.
card.render({...playing, episode: 's/other', frame: 0});
assert.equal(images()[0].src, live.frameUrl('s/other', 0, 'head'));  // Another episode: fresh cameras.
// The model at a replayed frame: recorded positions through the stopped reader's mapping, as the viewer's telemetry.
const mapping = {left_arm_j1: {joint: 'left_arm_j1', scale: 1, offset: 0, unit: 'rad'},
  right_arm_j1: {joint: 'right_arm_j1', scale: -1, offset: .1, unit: 'rad'},
  left_gripper: {joint: 'left_gripper_finger_a_slide', scale: .00921, offset: -.03, unit: 'm'}};
const view = {episode: 's/e', index: 1, mapping, summary: {joints: ['left_arm_j1', 'right_arm_j1', 'left_gripper', 'extra']},
  series: {qpos: [[0, 0, 0, 0], [.5, .25, 2, 9]]}};
const pose = replayTelemetry(view, {model: 'minerva', urdf_sha256: 'abc'}, 7);
assert.deepEqual([pose.model, pose.urdf_sha256, pose.stream_id, pose.sequence, pose.stale], ['minerva', 'abc', 'replay:s/e', 7, false]);
assert.deepEqual(pose.rows.map(row => [row.motor, row.joint, row.position_unit, row.source]),
  [['left_arm_j1', 'left_arm_j1', 'rad', 'replay'], ['right_arm_j1', 'right_arm_j1', 'rad', 'replay'],
   ['left_gripper', 'left_gripper_finger_a_slide', 'm', 'replay']]);  // Unmapped columns are left out.
assert.ok(Math.abs(pose.rows[1].position - (-.25 + .1)) < 1e-12 && Math.abs(pose.rows[2].position - (2 * .00921 - .03)) < 1e-12);
assert.equal(replayTelemetry({...view, mapping: null}, {model: 'minerva', urdf_sha256: 'abc'}, 8), null);
assert.equal(replayTelemetry(view, null, 9), null);  // No live model identity: nothing to pose.
console.log('live replay checks passed');
