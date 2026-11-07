"""What a link to a headset carries, measured from the headset's own browser. Run it on a machine at Minerva's end (it
needs aiortc and aiohttp, and nothing of Helm's; it never touches the robot), open its page on the Quest, press Start:

    python webrtc_link_probe.py --host <its Tailscale address> --port 8771 --results ~/webrtc-probe

About two minutes of stages, one after the other:
  poses, idle    60 poses a second of a pose's usual size, over a WebSocket, then over an unordered WebRTC channel
  ladder         fixed rates of H.264 over WebRTC, 0.25 to 8 Mbit/s, each with poses on the channel
  adaptive       the video on rate control (helm/webrtc_rate.py), poses on the channel, then on the WebSocket

A fixed rate is a tiny real H.264 stream padded with filler the decoder skips, so this asks almost nothing of the CPU and
the rate is exact. What came through at each rate shows a cap: above it, more sent brings no more received. A pose gap
over 100 ms is what would let an arm go (teleop/vr_arms.py), one over 250 ms what stops the head counting as tracked.
"""

import argparse
import asyncio
import fractions
import json
import logging
from pathlib import Path
import statistics
import sys
import time

from aiohttp import web
from aiortc import RTCSessionDescription, VideoStreamTrack
import av

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'robot_platform' / 'helm'))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from webrtc_rate import RateControl  # noqa: E402
from webrtc_video import LinkEncoder, follow_loss, limit_resends, peer_connection, prefer_h264, use_encoder  # noqa: E402

FPS = 10
WIDTH, HEIGHT = 640, 480
PAD_BPS = 100_000  # What the real picture gets; the filler makes up the stage's rate.
GAP_LIMITS_S = {'over_100_ms': 0.1, 'over_250_ms': 0.25}

PAGE = r"""<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Link probe</title>
<style>body{font:18px system-ui;margin:16px;background:#101418;color:#eef}button{font-size:24px;padding:12px 28px}
table{border-collapse:collapse;margin-top:12px;font-size:15px}td,th{border:1px solid #456;padding:3px 7px;text-align:right}
th:first-child,td:first-child{text-align:left}video{width:320px;display:block;margin:8px 0;background:#000}.bad{color:#f88}</style>
<h2>Link probe</h2><button id=start>Start</button> <span id=state>about two minutes; keep this page in front</span>
<video id=video autoplay muted playsinline></video><div id=path></div><table id=table></table>
<script type=module>
const $ = id => document.getElementById(id), sleep = ms => new Promise(done => setTimeout(done, ms));
const LADDER = [250, 500, 1000, 2000, 4000, 8000].map(kbit => kbit * 1000);
const POSE = JSON.stringify({type: 'pose', q: [0, 0, 0, 1], hands: {left: {p: [0, 0, 0], q: [0, 0, 0, 1], grip: 1, trigger: 0},
  right: {p: [0, 0, 0], q: [0, 0, 0, 1], grip: 1, trigger: 0}}, pad: 'x'.repeat(170)});
const pc = new RTCPeerConnection({iceServers: []});
pc.addTransceiver('video', {direction: 'recvonly'});
const channel = pc.createDataChannel('poses', {ordered: false, maxRetransmits: 0});
pc.ontrack = event => { $('video').srcObject = event.streams[0] || new MediaStream([event.track]); };
const results = {agent: navigator.userAgent, started: new Date().toISOString(), stages: []};
let pings = {sent: 0, back: [], outstanding: new Map()};
channel.onmessage = event => {
  const at = pings.outstanding.get(event.data);
  if (at !== undefined) { pings.back.push(performance.now() - at); pings.outstanding.delete(event.data); }
};
async function post(path, body) {
  const response = await fetch(path, {method: 'POST', body: JSON.stringify(body), headers: {'Content-Type': 'application/json'}});
  if (!response.ok) throw new Error(`${path}: ${response.status}`);
  return response.json();
}
async function stats() {
  const report = await pc.getStats(), out = {};
  for (const item of report.values()) {
    if (item.type === 'inbound-rtp' && item.kind === 'video') Object.assign(out, {bytes: item.bytesReceived, packets: item.packetsReceived,
      lost: item.packetsLost, decoded: item.framesDecoded, dropped: item.framesDropped, freezes: item.freezeCount,
      frozen_s: item.totalFreezesDuration, nacks: item.nackCount, decoder: item.decoderImplementation, power: item.powerEfficientDecoder});
    if (item.type === 'candidate-pair' && item.nominated && item.state === 'succeeded') {
      out.rtt_ms = item.currentRoundTripTime * 1000;
      const local = report.get(item.localCandidateId), remote = report.get(item.remoteCandidateId);
      out.path = `${local?.candidateType} ${local?.address || ''} to ${remote?.candidateType} ${remote?.address || ''}:${remote?.port || ''}`;
    }
  }
  return out;
}
const round = (value, places = 0) => value == null || Number.isNaN(value) ? '' : Number(value.toFixed(places));
function row(stage) {
  const table = $('table');
  if (!table.rows.length) table.innerHTML = '<tr><th>stage<th>sent Mbit/s<th>got Mbit/s<th>frames/s<th>frozen s<th>lost<th>RTT ms<th>poses got<th>gaps &gt;100 ms<th>worst gap ms';
  const cell = value => `<td>${value}`;
  table.insertRow().innerHTML = cell(stage.name) + cell(round(stage.sent_mbit, 2)) + cell(round(stage.got_mbit, 2)) + cell(round(stage.frames_s, 1))
    + cell(round(stage.frozen_s, 1)) + cell(stage.lost ?? '') + cell(round(stage.rtt_ms)) + cell(stage.poses ? `${stage.poses.received}/${stage.poses.sent}` : '')
    + cell(stage.poses?.over_100_ms ?? '') + cell(round(stage.poses?.worst_ms));
}
async function stage(name, {rate = 0, adaptive = false, poses = null, seconds}) {
  $('state').textContent = name;
  await post('/stage', {name, rate, adaptive, poses});
  const before = await stats(), startedAt = performance.now();
  pings = {sent: 0, back: [], outstanding: new Map()};
  let socket = null, sent = 0, sequence = 0;
  if (poses === 'socket') {
    socket = new WebSocket(`ws://${location.host}/poses`);
    await new Promise(done => { socket.onopen = done; socket.onerror = done; setTimeout(done, 3000); });
  }
  const send = () => {
    sequence += 1;
    const message = `{"seq":${sequence},` + POSE.slice(1);
    if (poses === 'channel' && channel.readyState === 'open') { channel.send(message); sent += 1; }
    if (poses === 'socket' && socket.readyState === 1) { socket.send(message); sent += 1; }
  };
  const posing = poses ? setInterval(send, 1000 / 60) : null;
  const pinging = setInterval(() => {
    if (channel.readyState !== 'open') return;
    const id = `ping ${++pings.sent}`;
    pings.outstanding.set(id, performance.now());
    channel.send(id);
  }, 200);
  await sleep(seconds * 1000);
  clearInterval(posing); clearInterval(pinging);
  socket?.close();
  await sleep(600);  // What is on its way comes in.
  const after = await stats(), elapsed = (performance.now() - startedAt) / 1000;
  const server = await post('/stage/end', {sent});
  const got = {name, seconds: elapsed, sent_mbit: server.sent_bytes * 8 / elapsed / 1e6, target_mbit: server.target_bps / 1e6,
    got_mbit: ((after.bytes || 0) - (before.bytes || 0)) * 8 / elapsed / 1e6, frames_s: ((after.decoded || 0) - (before.decoded || 0)) / elapsed,
    frozen_s: (after.frozen_s || 0) - (before.frozen_s || 0), lost: (after.lost || 0) - (before.lost || 0), nacks: (after.nacks || 0) - (before.nacks || 0),
    rtt_ms: after.rtt_ms, decoder: after.decoder, path: after.path, poses: server.poses, pings_sent: pings.sent, pings_back: pings.back.length,
    ping_median_ms: [...pings.back].sort((a, b) => a - b)[pings.back.length >> 1]};
  results.stages.push(got);
  row(got);
  $('path').textContent = `${after.path || ''} · decoder ${after.decoder || '?'}`;
  await post('/result', results);
}
$('start').onclick = async () => {
  $('start').disabled = true;
  try {
    $('state').textContent = 'connecting';
    await pc.setLocalDescription(await pc.createOffer());
    await Promise.race([new Promise(done => { pc.onicegatheringstatechange = () => pc.iceGatheringState === 'complete' && done(); }), sleep(2000)]);
    await pc.setRemoteDescription(await post('/offer', pc.localDescription));
    await Promise.race([new Promise(done => { channel.onopen = done; }), sleep(15000)]);
    if (channel.readyState !== 'open') throw new Error(`WebRTC did not connect (${pc.iceConnectionState})`);
    await stage('poses idle, socket', {poses: 'socket', seconds: 15});
    await stage('poses idle, channel', {poses: 'channel', seconds: 15});
    for (const rate of LADDER) await stage(`${rate / 1e6} Mbit/s`, {rate, poses: 'channel', seconds: 8});
    await stage('adaptive, channel', {adaptive: true, poses: 'channel', seconds: 30});
    await stage('adaptive, socket', {adaptive: true, poses: 'socket', seconds: 15});
    await post('/stage', {name: 'done', rate: 0});
    $('state').textContent = 'done: results are saved';
  } catch (error) {
    $('state').textContent = `stopped: ${error.message}`;
    $('state').className = 'bad';
    results.error = error.message;
    post('/result', results).catch(() => {});
  }
};
</script>"""


class Picture(VideoStreamTrack):
    """A moving bar, `FPS` frames a second, none while the stage sends no video."""

    kind = 'video'

    def __init__(self, probe):
        super().__init__()
        self.probe = probe
        self.count = 0
        self.at = None

    async def recv(self):
        while not self.probe.sending:
            await asyncio.sleep(0.05)
            self.at = None
        now = time.monotonic()
        self.at = now if self.at is None else self.at + 1 / FPS
        await asyncio.sleep(max(0.0, self.at - now))
        self.count += 1
        frame = av.VideoFrame(WIDTH, HEIGHT, 'yuv420p')
        at = self.count * 8 % WIDTH
        row = bytes(235 if at <= x < at + 40 else 40 for x in range(WIDTH))
        frame.planes[0].update(row * HEIGHT)
        for plane in frame.planes[1:]:
            plane.update(bytes([128]) * plane.buffer_size)
        frame.pts = int(now * 90_000)
        frame.time_base = fractions.Fraction(1, 90_000)
        return frame


def gaps(arrivals, sequences, sent):
    """The poses that came, and the waits between them."""
    waits = [b - a for a, b in zip(arrivals, arrivals[1:])]
    out = {
        'sent': sent,
        'received': len(arrivals),
        'out_of_order': sum(b < a for a, b in zip(sequences, sequences[1:])),
        'median_ms': statistics.median(waits) * 1000 if waits else None,
        'worst_ms': max(waits) * 1000 if waits else None,
    }
    out.update({key: sum(wait > limit for wait in waits) for key, limit in GAP_LIMITS_S.items()})
    return out


class Probe:
    def __init__(self, results):
        self.results = Path(results).expanduser()
        self.results.mkdir(parents=True, exist_ok=True)
        self.name = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        self.rate = RateControl()
        self.fixed = None
        self.sending = False
        self.encoder = None
        self.peers, self.tasks = set(), set()
        self.arrivals, self.sequences = [], []
        self.sender = None
        self.sent_at_start = 0

    def make_encoder(self):
        self.encoder = LinkEncoder(self.rate, FPS, pad=PAD_BPS)
        return self.encoder

    def pose(self, text):
        try:
            sequence = json.loads(text)['seq']
        except (ValueError, KeyError, TypeError):
            return
        self.arrivals.append(time.monotonic())
        self.sequences.append(sequence)

    async def sent_bytes(self):
        if self.sender is None:
            return 0
        report = await self.sender.getStats()
        return sum(item.bytesSent for item in report.values() if item.type == 'outbound-rtp')

    async def offer(self, request):
        data = await request.json()
        self.name = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())  # Each run its own results file.
        pc = peer_connection()
        self.peers.add(pc)

        @pc.on('datachannel')
        def on_channel(channel):
            @channel.on('message')
            def on_message(message):
                if isinstance(message, str) and message.startswith('ping '):
                    channel.send(message)
                elif isinstance(message, str):
                    self.pose(message)

        @pc.on('connectionstatechange')
        async def on_state():
            logging.info('connection %s', pc.connectionState)
            if pc.connectionState in ('failed', 'closed'):
                await pc.close()
                self.peers.discard(pc)

        self.sender = pc.addTrack(Picture(self))
        limit_resends(self.sender, self.rate)
        prefer_h264(next(t for t in pc.getTransceivers() if t.sender == self.sender))  # Before the offer is matched.
        await pc.setRemoteDescription(RTCSessionDescription(sdp=data['sdp'], type=data['type']))
        await pc.setLocalDescription(await pc.createAnswer())
        self.tasks.add(asyncio.create_task(self.follow(self.sender)))
        return web.json_response({'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type})

    async def follow(self, sender):
        await follow_loss(sender, self)

    def loss(self, fraction):
        """A receiver report: the rate control's, in the adaptive stages."""
        if self.fixed is None:
            self.rate.loss(fraction)

    async def stage(self, request):
        data = await request.json()
        rate, adaptive = data.get('rate', 0), data.get('adaptive', False)
        logging.info('stage %s', data.get('name'))
        if adaptive:
            self.fixed, self.rate.minimum, self.rate.maximum = None, 120_000, 8_000_000
            if self.rate.allowed > 500_000:
                self.rate.allowed = 500_000.0
            self.rate.estimated = None
        elif rate:
            self.fixed = rate
            self.rate.minimum = self.rate.maximum = rate
            self.rate.allowed, self.rate.estimated = float(rate), None
        self.sending = adaptive or rate > 0
        self.arrivals, self.sequences = [], []
        self.sent_at_start = await self.sent_bytes()
        return web.json_response({'ok': True})

    async def stage_end(self, request):
        data = await request.json()
        return web.json_response(
            {
                'poses': gaps(self.arrivals, self.sequences, data.get('sent', 0)),
                'sent_bytes': await self.sent_bytes() - self.sent_at_start,
                'target_bps': self.rate.target if self.sending else 0,
            }
        )

    async def poses(self, request):
        socket = web.WebSocketResponse(max_msg_size=4096, compress=False)
        await socket.prepare(request)
        async for message in socket:
            if message.type == web.WSMsgType.TEXT:
                self.pose(message.data)
        return socket

    async def result(self, request):
        (self.results / f'{self.name}.json').write_text(await request.text(), encoding='utf-8')
        return web.json_response({'ok': True})

    async def page(self, _request):
        return web.Response(text=PAGE, content_type='text/html', headers={'Cache-Control': 'no-store'})

    def app(self):
        app = web.Application()
        app.router.add_get('/', self.page)
        app.router.add_post('/offer', self.offer)
        app.router.add_post('/stage', self.stage)
        app.router.add_post('/stage/end', self.stage_end)
        app.router.add_post('/result', self.result)
        app.router.add_get('/poses', self.poses)
        return app


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8771)
    parser.add_argument('--results', default='~/webrtc-probe')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    probe = Probe(args.results)
    use_encoder(probe.make_encoder)
    web.run_app(probe.app(), host=args.host, port=args.port, print=lambda text: logging.info(text))


if __name__ == '__main__':
    main()
