import assert from 'node:assert/strict';

import { RemoteCamera } from '../../src/robot_platform/helm/static/xr_remote.js';
import { pageLink } from '../../src/robot_platform/helm/static/xr_status.js';

// The remote headset's camera and pose channel (xr_remote.js), with the browser's parts replaced: what it asks of the
// peer, that poses leave by the channel only while it is open, that a frame is handed over once, and that a lost peer
// is replaced.

class Peer {
  constructor() {
    this.iceGatheringState = 'complete';
    this.connectionState = 'new';
    this.localDescription = { sdp: 'offer sdp', type: 'offer' };
    this.closed = false;
    this.channels = [];
    this.transceivers = [];
  }

  addTransceiver(kind, options) {
    this.transceivers.push([kind, options]);
  }

  createDataChannel(label, options) {
    const channel = {
      label,
      options,
      readyState: 'connecting',
      sent: [],
      send(text) {
        this.sent.push(text);
      },
    };
    this.channels.push(channel);
    return channel;
  }

  async createOffer() {
    return this.localDescription;
  }

  async setLocalDescription() {}

  async setRemoteDescription(answer) {
    this.answer = answer;
  }

  close() {
    this.closed = true;
  }
}

function camera(options = {}) {
  const made = {
    peers: [],
    frames: [],
    statuses: [],
    offers: [],
    timers: [],
    element: {
      readyState: 4,
      count: 0,
      getVideoPlaybackQuality() {
        return { totalVideoFrames: this.count };
      },
    },
    now: 0,
  };
  const remote = new RemoteCamera({
    offer: async (offer) => {
      made.offers.push(offer);
      return { sdp: 'answer sdp', type: 'answer' };
    },
    onFrame: (bitmap, metadata, depth) => made.frames.push({ bitmap, metadata, depth }),
    onStatus: (status) => made.statuses.push(status),
    peer: () => {
      const peer = new Peer();
      made.peers.push(peer);
      return peer;
    },
    video: () => made.element,
    grab: async () => 'a bitmap',
    now: () => made.now,
    every: () => 1,
    stopEvery: () => {},
    after: (fn, ms) => {
      made.timers.push({ fn, ms });
      return made.timers.length;
    },
    stopAfter: () => {},
    ...options,
  });
  return [remote, made];
}

const settle = () => new Promise((done) => setImmediate(done));

// It asks for video only and an unordered channel that never resends, and hands Helm its offer.
{
  const [remote, made] = camera();
  remote.start();
  await settle();
  const [peer] = made.peers;
  assert.deepEqual(peer.transceivers, [['video', { direction: 'recvonly' }]]);
  assert.equal(peer.channels[0].label, 'poses');
  assert.deepEqual(peer.channels[0].options, { ordered: false, maxRetransmits: 0 });
  assert.deepEqual(made.offers, [{ sdp: 'offer sdp', type: 'offer' }]);
  assert.equal(peer.answer.sdp, 'answer sdp');
  remote.stop();
  assert.equal(peer.closed, true);
}

// A pose goes on the channel only while it is open; otherwise the page's socket carries it.
{
  const [remote, made] = camera();
  remote.start();
  await settle();
  const [channel] = made.peers[0].channels;
  assert.equal(remote.sendPose('early'), false);
  channel.readyState = 'open';
  channel.onopen();
  assert.equal(remote.sendPose('pose'), true);
  assert.deepEqual(channel.sent, ['pose']);
  channel.readyState = 'closed';
  channel.onclose();
  assert.equal(remote.sendPose('late'), false);
  channel.onmessage({ data: '{"type":"status","tracking":true}' });
  assert.deepEqual(made.statuses, [{ type: 'status', tracking: true }]);
  channel.onmessage({ data: 'not json' });
  assert.equal(made.statuses.length, 1);
  assert.equal(remote.message, 'unreadable status');
  remote.stop();
}

// A decoded frame is handed over once, flat, and counts as fresh until a second has passed without another.
{
  const [remote, made] = camera();
  remote.start();
  await settle();
  assert.equal(remote.stats.fresh, false);
  assert.equal(remote.stats.message, 'connecting…');
  await remote.check(); // No frame decoded yet.
  assert.equal(made.frames.length, 0);
  made.element.count = 1;
  await remote.check();
  await remote.check(); // The same frame.
  assert.deepEqual(made.frames, [{ bitmap: 'a bitmap', metadata: { role: 'head' }, depth: null }]);
  assert.equal(remote.stats.fresh, true);
  made.now = 1500;
  assert.equal(remote.stats.fresh, false);
  assert.equal(remote.stats.message, 'last frame 2 s ago');
  remote.setRole('head_depth'); // The 3D switch changes nothing here.
  assert.equal(made.peers.length, 1);
  remote.stop();
}

// A video not yet playing gives no frame, and one that cannot be read says so.
{
  const [remote, made] = camera({
    grab: async () => {
      throw new Error('bad');
    },
  });
  remote.start();
  await settle();
  made.element.readyState = 1;
  made.element.count = 1;
  await remote.check();
  assert.equal(made.frames.length, 0);
  made.element.readyState = 4;
  await remote.check();
  assert.equal(remote.message, 'undecodable frame (bad)');
  assert.equal(remote.grabbing, false);
  remote.stop();
}

// A failed peer or a refused offer is replaced after a wait, until stopped.
{
  const [remote, made] = camera();
  remote.start();
  await settle();
  made.peers[0].connectionState = 'failed';
  made.peers[0].onconnectionstatechange();
  assert.equal(made.peers[0].closed, true);
  assert.equal(remote.message, 'reconnecting…');
  assert.equal(made.timers.at(-1).ms, 3000);
  made.timers.at(-1).fn();
  await settle();
  assert.equal(made.peers.length, 2);
  remote.stop();

  const [refused, seen] = camera({
    offer: async () => {
      throw new Error('Helm answered 500');
    },
  });
  refused.start();
  await settle();
  assert.equal(refused.message, 'no link: Helm answered 500');
  assert.equal(seen.peers[0].closed, true);
  refused.stop();
  seen.timers.length = 0;
}

// The link's numbers, from the browser's own counters.
{
  const [remote, made] = camera();
  remote.start();
  await settle();
  const report = (rows) => new Map(rows.map((row) => [row.id, row]));
  made.peers[0].getStats = async () =>
    report([
      {
        id: 'a',
        type: 'inbound-rtp',
        kind: 'video',
        bytesReceived: 1000,
        packetsLost: 3,
        decoderImplementation: 'hardware',
      },
      { id: 'b', type: 'candidate-pair', nominated: true, currentRoundTripTime: 0.06 },
    ]);
  await remote.read();
  made.now = 1000;
  made.peers[0].getStats = async () =>
    report([
      {
        id: 'a',
        type: 'inbound-rtp',
        kind: 'video',
        bytesReceived: 126000,
        packetsLost: 5,
        decoderImplementation: 'hardware',
      },
    ]);
  await remote.read();
  assert.deepEqual(remote.link, { lost: 5, decoder: 'hardware', rtt_ms: 60, bitrate_bps: 1e6 });
  remote.stop();
}

// Each page names its link and leads to the other, and a login returns to the one it left.
{
  assert.equal(pageLink(true).page, '/xr?remote');
  assert.equal(pageLink(true).other, '/xr');
  assert.equal(pageLink(false).page, '/xr');
  assert.equal(pageLink(false).other, '/xr?remote');
  assert.match(pageLink(true).name, /^Remote/);
  assert.match(pageLink(false).name, /^Ordinary/);
}

// Every second read goes on the channel for Minerva's log: what arrived, what decoded, what reached the screen, and by
// which path. Nothing is sent while the channel is shut, and a long message does not overrun the worker's bound.
{
  const [remote, made] = camera({ visibility: () => 'visible' });
  remote.start();
  await settle();
  const [peer] = made.peers;
  const [channel] = peer.channels;
  peer.getStats = async () =>
    new Map([
      [
        'a',
        {
          id: 'a',
          type: 'inbound-rtp',
          kind: 'video',
          bytesReceived: 1000,
          packetsReceived: 9,
          packetsLost: 3,
          framesReceived: 7,
          framesDecoded: 6,
          framesDropped: 1,
          keyFramesDecoded: 1,
          freezeCount: 2,
          nackCount: 4,
          pliCount: 5,
          decoderImplementation: 'hardware',
        },
      ],
      [
        'b',
        {
          id: 'b',
          type: 'candidate-pair',
          nominated: true,
          currentRoundTripTime: 0.06,
          localCandidateId: 'l',
          remoteCandidateId: 'r',
        },
      ],
      ['l', { id: 'l', type: 'local-candidate', candidateType: 'prflx' }],
      ['r', { id: 'r', type: 'remote-candidate', candidateType: 'host', address: '198.51.100.9', port: 5000 }],
    ]);
  await remote.read();
  await remote.read();
  assert.deepEqual(channel.sent, []); // Shut.
  channel.readyState = 'open';
  channel.onopen();
  made.element.count = 1;
  await remote.check();
  remote.message = 'x'.repeat(5000);
  await remote.read();
  assert.deepEqual(channel.sent, []); // Every second read.
  await remote.read();
  assert.equal(channel.sent.length, 1);
  assert.ok(channel.sent[0].startsWith('{"type":"link"')); // webrtc_worker.PAGE_REPORT: for the log, not for Helm.
  assert.ok(channel.sent[0].length < 1024); // webrtc_worker.MAX_POSE.
  const sent = JSON.parse(channel.sent[0]);
  assert.equal(sent.shown, 1);
  assert.equal(sent.video_frames, 1);
  assert.equal(sent.ready, 4);
  assert.equal(sent.page, 'visible');
  assert.equal(sent.bytes, 1000);
  assert.equal(sent.decoded, 6);
  assert.equal(sent.freezes, 2);
  assert.equal(sent.rtt_ms, 60);
  assert.equal(sent.path, 'prflx to host 198.51.100.9:5000');
  remote.stop();
}

console.log('remote headset link: ok');
