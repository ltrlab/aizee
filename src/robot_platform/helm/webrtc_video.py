"""H.264 for a remote viewer over WebRTC, on a link's rate (webrtc_rate): aiortc's encoder is built for a good network, so
it stays above 500 kbit/s, ignores loss, and assumes 30 frames a second. Needs aiortc and PyAV, and nothing of Helm's:
scripts/webrtc_link_probe.py runs it on a machine without Helm. The Orin Nano has no hardware encoder, so this is libx264
on the CPU, which is why it runs in a process of its own (webrtc_worker).
"""
import asyncio
import fractions
import time

import aiortc.rtcrtpsender
from aiortc.codecs.h264 import H264Encoder
from aiortc.rtcconfiguration import RTCConfiguration
from aiortc import RTCPeerConnection, RTCRtpSender
import av

try:
    from .webrtc_rate import KEYFRAME_EVERY_S, RateControl, ResendBudget
except ImportError:  # Beside the probe, outside the package.
    from webrtc_rate import KEYFRAME_EVERY_S, RateControl, ResendBudget

# The most one picture may take, in bits: without a limit x264's first picture of a busy scene is 148 kB (measured), over
# 2 s of a 600 kbit/s link.
FRAME_LIMIT_BITS = 160_000
RETUNE_SHARE = .05  # A rate this far from the encoder's is worth telling it.
LOSS_POLL_S = .5
# Most of one packet, in a frame's filler NAL (type 12): the probe's exact rate.
FILLER_START = b'\x00\x00\x00\x01\x0c'
# aiortc reports a receiver's fraction lost as RTP does, out of 256.
FRACTION_UNIT = 256


class LinkEncoder(H264Encoder):
    """libx264 at `rate`'s target, for a source of `fps` frames a second. `pad`: encode at most `pad` bits a second and
    fill up to the target with filler the decoder skips, so a probe sends an exact rate at almost no CPU."""

    def __init__(self, rate, fps, preset='superfast', pad=None):
        super().__init__()
        self.rate, self.fps, self.preset, self.pad = rate, fps, preset, pad
        self.keyframe_at = -KEYFRAME_EVERY_S

    @property
    def target_bitrate(self):
        return self.rate.target

    @target_bitrate.setter
    def target_bitrate(self, bitrate):  # The sender hands over each REMB here.
        self.rate.estimate(bitrate)

    def _encoding(self, target):
        return target if self.pad is None else min(target, self.pad)

    def _open(self, frame, target):
        """Opened at its top rate, then steered down: x264 follows a rate changed mid-stream, with no whole new picture,
        only below the rate it was opened at (measured: opened at 300 kbit/s it stays there, asked for 150 or 1000;
        opened at 1.5 Mbit/s it gives 152 and 706)."""
        top = self.pad or self.rate.maximum
        codec = av.CodecContext.create('libx264', 'w')
        codec.width, codec.height, codec.pix_fmt = frame.width, frame.height, 'yuv420p'
        codec.bit_rate = top
        # x264's rate control counts frames at this rate, whatever their timestamps say.
        codec.framerate = fractions.Fraction(self.fps)
        codec.time_base = fractions.Fraction(1, 90_000)
        codec.options = {'level': '31', 'tune': 'zerolatency', 'preset': self.preset, 'forced-idr': '1',
                         'threads': '1', 'maxrate': str(top), 'bufsize': str(FRAME_LIMIT_BITS)}
        codec.profile = 'Baseline'
        codec.open()
        codec.bit_rate = self._encoding(target)
        self.codec = codec

    def _encode_frame(self, frame, force_keyframe):
        target = self.rate.target
        if self.codec is not None and (frame.width, frame.height) != (self.codec.width, self.codec.height):
            self.codec = None
        if self.codec is None:
            self._open(frame, target)
        elif abs(self._encoding(target) - self.codec.bit_rate) > RETUNE_SHARE * self.codec.bit_rate:
            self.codec.bit_rate = self._encoding(target)
        now = time.monotonic()
        if force_keyframe and now - self.keyframe_at >= KEYFRAME_EVERY_S:
            self.keyframe_at = now
        else:
            force_keyframe = False
        frame.pict_type = av.video.frame.PictureType.I if force_keyframe else av.video.frame.PictureType.NONE
        data = b''.join(bytes(packet) for packet in self.codec.encode(frame))
        if self.pad is not None:
            missing = int(target / 8 / self.fps) - len(data) - len(FILLER_START) - 1
            if missing > 0:
                data += FILLER_START + b'\xff' * missing + b'\x80'
        if data:
            yield from self._split_bitstream(data)


def limit_resends(sender, rate):
    """`sender` answers a receiver's requests for lost packets only within a budget (webrtc_rate.ResendBudget), returned
    with its counts. aiortc has no setting for it, so its method is wrapped (pinned in requirements/helm-webrtc.lock)."""
    budget, resend = ResendBudget(rate, time.monotonic), sender._retransmit

    async def limited(sequence_number):
        if budget.allow(sequence_number):
            await resend(sequence_number)
    sender._retransmit = limited
    return budget


def use_encoder(encoder):
    """The next video sender this process starts encodes with `encoder()`: aiortc has no way to choose one, so its
    lookup is replaced (pinned in requirements/helm-robot.lock)."""
    original = aiortc.rtcrtpsender.get_encoder

    def get_encoder(codec):
        return encoder() if codec.mimeType.lower() == 'video/h264' else original(codec)
    aiortc.rtcrtpsender.get_encoder = get_encoder


def peer_connection():
    """A peer with no STUN or TURN server: a viewer reaches this machine directly, on its LAN or over a VPN's route, so
    nothing here asks an outside server for an address."""
    return RTCPeerConnection(RTCConfiguration(iceServers=[]))


def prefer_h264(transceiver):
    """H.264 only: a Quest decodes it in hardware."""
    capabilities = RTCRtpSender.getCapabilities('video').codecs
    transceiver.setCodecPreferences([codec for codec in capabilities if codec.mimeType.lower() in
                                     ('video/h264', 'video/rtx')])


async def follow_loss(sender, rate, period=LOSS_POLL_S):
    """Feed `sender`'s receiver reports to `rate`, each once, until cancelled."""
    seen = None
    while True:
        await asyncio.sleep(period)
        report = await sender.getStats()
        for stats in report.values():
            if stats.type == 'remote-inbound-rtp' and stats.packetsReceived is not None and stats.timestamp != seen:
                seen = stats.timestamp
                rate.loss(min(1., stats.fractionLost / FRACTION_UNIT))


__all__ = ['LinkEncoder', 'RateControl', 'follow_loss', 'limit_resends', 'peer_connection', 'prefer_h264', 'use_encoder']
