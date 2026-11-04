"""The remote headset's real worker and a real peer, where aiortc is installed: not in the gate, whose environment leaves
it out as Helm's own does (requirements/helm-webrtc.in). Run it from an environment with aiortc:
    python -m pytest tests/unit/test_helm_webrtc_peer.py
"""
import asyncio
import time

import pytest

from robot_platform.helm.webrtc_link import WebRTCLink

pytest.importorskip('aiortc')
pytest.importorskip('av')

JPEG_SIDE = 64


class Video:
    """Helm's camera relay: real JPEGs of a plain square."""

    def __init__(self):
        import io
        from PIL import Image
        buffer = io.BytesIO()
        Image.new('RGB', (JPEG_SIDE, JPEG_SIDE), (200, 60, 30)).save(buffer, 'JPEG')
        self.jpeg, self.sequence, self.viewers = buffer.getvalue(), 0, set()

    def subscribe(self, depth=False):
        token = object()
        self.viewers.add(token)
        return token

    def unsubscribe(self, token):
        self.viewers.discard(token)

    def snapshot(self):
        self.sequence += 1
        return {'type': 'status'}, {'head': ({'role': 'head', 'sequence': self.sequence}, self.jpeg)}


async def until(condition, timeout):
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, 'Timed out'
        await asyncio.sleep(.02)


def test_real_peer_answers_an_offer():
    from aiortc import RTCPeerConnection, RTCSessionDescription

    async def run():
        video, poses, frames = Video(), [], []
        link = WebRTCLink(video, lambda text, owner: poses.append(text))
        browser = RTCPeerConnection()
        browser.addTransceiver('video', direction='recvonly')

        @browser.on('track')
        def on_track(track):
            async def read():
                while True:
                    frames.append(await track.recv())
            asyncio.ensure_future(read())
        channel = browser.createDataChannel('poses', ordered=False, maxRetransmits=0)
        await browser.setLocalDescription(await browser.createOffer())
        answer = await link.offer(browser.localDescription.sdp, browser.localDescription.type, None)
        assert 'H264' in answer['sdp'] and 'VP8' not in answer['sdp']
        await browser.setRemoteDescription(RTCSessionDescription(sdp=answer['sdp'], type=answer['type']))
        await until(lambda: channel.readyState == 'open', 15.)
        channel.send('{"type":"pose"}')
        await until(lambda: poses, 5.)
        assert poses == ['{"type":"pose"}']
        await until(lambda: frames, 10.)  # H.264 all the way, decoded at the far end.
        assert (frames[0].width, frames[0].height) == (JPEG_SIDE, JPEG_SIDE)
        await browser.close()
        await link.close()
    asyncio.run(run())
