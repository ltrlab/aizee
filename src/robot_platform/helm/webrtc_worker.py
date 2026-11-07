"""The remote headset's WebRTC peer, in a process of its own (webrtc_link starts it): libx264 and aiortc's Python must not
share Helm's interpreter or the arms' CPUs. Helm writes framed messages to its stdin (`write_message`) and reads one JSON
message a line from its stdout; it ends when its stdin does.

    to it:    {'kind': 'offer', 'id': n, 'sdp': ..., 'type': 'offer'}
              {'kind': 'frame', 'sequence': n}, with the JPEG after it
              {'kind': 'status', 'text': ...}   Helm's word to the page, over the pose channel
              {'kind': 'close'}                 the peer ends
    from it:  {'kind': 'answer', 'id': n, 'sdp': ..., 'type': 'answer'}, or {'kind': 'error', 'id': n, 'error': ...}
              {'kind': 'pose', 'text': ...}     a message on the pose channel
              {'kind': 'state', 'state': ...}   the peer's connection, and 'closed' when it has gone
              {'kind': 'stats', ...}            once a second while connected
A message on the pose channel that starts with PAGE_REPORT is the page's own look at the link: it goes to the log only.

One viewer at a time: an offer ends the last one. Read-only towards the robot: its only way in is the pose channel, whose
messages go to Helm unread except for their size, and Helm checks them as it does the socket's.
"""

import asyncio
import fractions
import json
import logging
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import time

MAX_HEADER = 256 * 1024  # An offer's SDP.
MAX_PAYLOAD = 2 * 1024 * 1024
MAX_POSE = 1024  # As Helm's socket (server.xr_socket).
FPS = 10  # The camera relay's rate (camera_video_worker): x264 counts its bits by it.
STATS_S = 1.0
PARTITION = '/etc/minerva/cpu-partition.json'
EVIDENCE = 'HELM_WEBRTC_EVIDENCE'  # Helm on Minerva: the folder for this worker's log of the link, if any.
PAGE_REPORT = '{"type":"link"'


def write_message(output, header, payload=b''):
    """One framed message: two big-endian sizes, the JSON header and the payload."""
    encoded = json.dumps(header, allow_nan=False, separators=(',', ':')).encode()
    if len(encoded) > MAX_HEADER or len(payload) > MAX_PAYLOAD:
        raise ValueError('A message to the WebRTC worker exceeds its bound')
    return output.write(struct.pack('!II', len(encoded), len(payload)) + encoded + payload)


def read_messages(stream):
    """Each framed message from a blocking `stream` as (header, payload), to its end."""
    while True:
        sizes = stream.read(8)
        if len(sizes) < 8:
            return
        header_size, payload_size = struct.unpack('!II', sizes)
        if not 1 <= header_size <= MAX_HEADER or payload_size > MAX_PAYLOAD:
            raise ValueError('A message to the WebRTC worker exceeds its bound')
        header, payload = stream.read(header_size), stream.read(payload_size)
        if len(header) < header_size or len(payload) < payload_size:
            return
        yield json.loads(header), payload


def candidates(sdp):
    """What of an SDP tells how the two ends could reach each other."""
    return [line for line in sdp.splitlines() if line.startswith('a=candidate')]


def page_report(text):
    """The page's look at the link as a record for the log, or None if `text` is anything else: a pose, for Helm."""
    if not text.startswith(PAGE_REPORT):
        return None
    try:
        return {'kind': 'page', **json.loads(text)}
    except ValueError:
        return {'kind': 'page', 'unreadable': text[:80]}


class LinkLog:
    """What the link did, a JSON line a record, in `folder` (none: nothing): a test from another network can only be read
    afterwards. On a thread of its own, as a stalled microSD must not hold up the poses."""

    def __init__(self, folder=None, clock=time.time):
        self.clock, self.records, self.on = clock, queue.SimpleQueue(), bool(folder)
        if folder:
            self.path = Path(folder) / time.strftime('remote-link-%Y%m%dT%H%M%SZ.jsonl', time.gmtime(clock()))
            threading.Thread(target=self._write, daemon=True).start()

    def __call__(self, record):
        if self.on:
            self.records.put({'at': round(self.clock(), 3), **record})

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, 'a', encoding='utf-8') as out:
                while True:
                    out.write(json.dumps(self.records.get(), separators=(',', ':')) + '\n')
                    out.flush()
        except OSError:
            logging.exception('The link log')
            self.on = False


def background_cpus(path=PARTITION):
    """Onto the robot's background CPUs where it has a CPU split (as camera_video_worker), and nicer than Helm."""
    try:
        with open(path, encoding='utf-8') as source:
            os.sched_setaffinity(0, set(json.load(source)['background']))
    except (OSError, ValueError, KeyError, TypeError, AttributeError):
        pass
    try:
        os.nice(5)
    except (OSError, AttributeError):
        pass


class Worker:
    def __init__(self, output=None, log=None):
        from aiortc import MediaStreamTrack
        import av
        from .webrtc_rate import RateControl
        from .webrtc_video import LinkEncoder, follow_loss, limit_resends, peer_connection, prefer_h264, use_encoder

        self.limit_resends = limit_resends
        self.av, self.RateControl, self.LinkEncoder = av, RateControl, LinkEncoder
        self.follow_loss, self.peer_connection, self.prefer_h264 = follow_loss, peer_connection, prefer_h264
        self.output = output or sys.stdout.buffer
        self.log = log or LinkLog()
        self.pc = self.channel = self.sender = self.track = self.rate = self.encoder = None
        self.tasks = set()
        self.frames_in = self.poses_in = 0
        use_encoder(self.make_encoder)

        class Newest(MediaStreamTrack):
            """The newest JPEG as a frame: one that comes while the encoder is busy replaces the one waiting."""

            kind = 'video'

            def __init__(track):
                super().__init__()
                track.jpeg, track.ready = None, asyncio.Event()
                track.decoder = av.CodecContext.create('mjpeg', 'r')
                track.time_base = fractions.Fraction(1, 90_000)

            def put(track, jpeg):
                track.jpeg = jpeg
                track.ready.set()

            def decode(track, jpeg):
                frames = track.decoder.decode(av.Packet(jpeg))
                return frames[-1] if frames else None

            async def recv(track):
                while True:
                    await track.ready.wait()
                    track.ready.clear()
                    jpeg, track.jpeg = track.jpeg, None
                    if jpeg is None:
                        continue
                    frame = await asyncio.get_running_loop().run_in_executor(None, track.decode, jpeg)
                    if frame is not None:
                        frame.pts, frame.time_base = int(time.monotonic() * 90_000), track.time_base
                        return frame

        self.Newest = Newest

    def make_encoder(self):
        self.encoder = self.LinkEncoder(self.rate, FPS)
        return self.encoder

    def say(self, message):
        if message.get('kind') != 'pose':
            self.log(dict(message, sdp=candidates(message['sdp'])) if 'sdp' in message else message)
        try:
            self.output.write(json.dumps(message, allow_nan=False, separators=(',', ':')).encode() + b'\n')
            self.output.flush()
        except OSError:  # Helm has gone: its end of stdin says so too, and this process follows.
            pass

    async def close_peer(self):
        pc, self.pc, self.channel, self.sender, self.track = self.pc, None, None, None, None
        for task in self.tasks:
            task.cancel()
        self.tasks = set()
        if pc is not None:
            await pc.close()
            self.say({'kind': 'state', 'state': 'closed'})

    async def offer(self, message):
        from aiortc import RTCSessionDescription

        await self.close_peer()
        self.rate = self.RateControl()
        pc = self.pc = self.peer_connection()
        self.track = self.Newest()
        self.sender = pc.addTrack(self.track)
        self.resends = self.limit_resends(self.sender, self.rate)
        self.prefer_h264(next(t for t in pc.getTransceivers() if t.sender == self.sender))

        @pc.on('datachannel')
        def on_channel(channel):
            if channel.label != 'poses':
                return
            self.channel = channel

            @channel.on('message')
            def on_message(text):
                if not isinstance(text, str) or len(text) > MAX_POSE:
                    return
                report = page_report(text)
                if report is not None:
                    self.log(report)
                    return
                self.poses_in += 1
                self.say({'kind': 'pose', 'text': text})

        @pc.on('connectionstatechange')
        async def on_state():
            if pc is not self.pc:
                return
            self.say({'kind': 'state', 'state': pc.connectionState})
            if pc.connectionState in ('failed', 'closed'):
                await self.close_peer()

        await pc.setRemoteDescription(RTCSessionDescription(sdp=message['sdp'], type=message['type']))
        await pc.setLocalDescription(await pc.createAnswer())
        self.tasks.add(asyncio.create_task(self.follow_loss(self.sender, self.rate)))
        self.tasks.add(asyncio.create_task(self.stats(pc, self.sender)))
        return {'sdp': pc.localDescription.sdp, 'type': pc.localDescription.type}

    async def stats(self, pc, sender):
        while True:
            await asyncio.sleep(STATS_S)
            sent, packets, lost, rtt = None, None, None, None
            for item in (await sender.getStats()).values():
                if item.type == 'outbound-rtp':
                    sent, packets = item.bytesSent, item.packetsSent
                elif item.type == 'remote-inbound-rtp':
                    lost, rtt = item.fractionLost / 256, item.roundTripTime
            self.say(
                {
                    'kind': 'stats',
                    'state': pc.connectionState,
                    'bytes_sent': sent,
                    'packets_sent': packets,
                    'resends': self.resends.resent,
                    'resends_refused': self.resends.refused,
                    'target_bps': self.rate.target,
                    'estimate_bps': self.rate.estimated,
                    'fraction_lost': lost,
                    'rtt_s': rtt,
                    'frames_in': self.frames_in,
                    'poses_in': self.poses_in,
                }
            )

    async def handle(self, header, payload):
        kind = header.get('kind')
        if kind == 'frame':
            self.frames_in += 1
            if self.track is not None:
                self.track.put(payload)
        elif kind == 'status':
            if self.channel is not None and self.channel.readyState == 'open':
                self.channel.send(header['text'])
        elif kind == 'close':
            await self.close_peer()
        elif kind == 'offer':
            self.log({'kind': 'offer', 'sdp': candidates(header.get('sdp', ''))})
            try:
                self.say({'kind': 'answer', 'id': header['id'], **await self.offer(header)})
            except Exception as exc:  # noqa: BLE001 - an offer that cannot be answered ends the peer, not the worker.
                logging.exception('offer')
                self.say({'kind': 'error', 'id': header['id'], 'error': str(exc)[:300]})
                await self.close_peer()

    async def run(self, stream):
        loop, queue = asyncio.get_running_loop(), asyncio.Queue()

        def read():
            try:
                for message in read_messages(stream):
                    loop.call_soon_threadsafe(queue.put_nowait, message)
            finally:
                loop.call_soon_threadsafe(queue.put_nowait, None)

        threading.Thread(target=read, daemon=True).start()
        while (message := await queue.get()) is not None:
            await self.handle(*message)
        await self.close_peer()


def main():
    logging.basicConfig(level=logging.WARNING, stream=sys.stderr)
    background_cpus()
    asyncio.run(Worker(log=LinkLog(os.environ.get(EVIDENCE))).run(sys.stdin.buffer))


if __name__ == '__main__':
    main()
