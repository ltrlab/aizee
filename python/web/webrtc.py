"""WebRTC video bridge for the WebXR client.

Replaces the JPEG-over-WebSocket camera path with a real WebRTC peer
connection.  The Quest browser opens an RTCPeerConnection, receives
VP8/H.264-encoded video via DTLS-SRTP, hardware-decodes it, and feeds it
to a `<video>` element backing a THREE.VideoTexture.

Wins over the previous WS+JPEG path:
  * Hardware decode in the Quest browser (lower CPU + battery)
  * Jitter buffer absorbs LAN micro-stalls instead of stalling THREE
  * SRTP runs over UDP, no head-of-line blocking under packet loss

Source is the same `SharedState.latest_cam_jpeg` / `latest_cam_seq` that
collect_demo.py already writes — the bridge just re-encodes JPEG frames
into the codec aiortc negotiated with the browser.

Signaling: one POST /api/webrtc/offer per session (SDP exchange in JSON).
ICE: host candidates only (LAN-direct); no STUN/TURN.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

import numpy as np

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from av import VideoFrame
    _AIORTC_AVAILABLE = True
except ImportError:
    _AIORTC_AVAILABLE = False


_log = logging.getLogger("aizee.webrtc")
_log.setLevel(logging.INFO)


# -----------------------------------------------------------------------------
# Source track
# -----------------------------------------------------------------------------

class JpegCameraTrack(VideoStreamTrack):
    """A VideoStreamTrack that re-encodes the SharedState's JPEG stream.

    aiortc handles VP8/H.264 encoding internally; we just decode JPEG to
    RGB and hand it `VideoFrame.from_ndarray` at the camera's native rate.
    pts pacing is delegated to the base class's `next_timestamp()` — it
    sleeps to the next 1/30s tick AND returns monotonic pts.  Earlier
    versions of this file rolled their own deadline math, which produced
    pts=0 after a fall-behind rebase and stalled the browser decoder.
    """

    kind = "video"

    def __init__(self, shared_state) -> None:
        super().__init__()
        self._state = shared_state
        self._last_seq = -1
        self._last_arr: Optional[np.ndarray] = None

    async def recv(self) -> "VideoFrame":
        # Sleeps to the next frame deadline AND advances pts monotonically.
        pts, time_base = await self.next_timestamp()

        # Pull the freshest JPEG; if no new frame, re-encode the last one
        # (keeps the codec stream going so the browser jitter buffer doesn't
        # drain to empty during a brief camera stall).
        jpeg = self._state.latest_cam_jpeg
        seq = self._state.latest_cam_seq
        if jpeg is not None and seq != self._last_seq and _CV2_AVAILABLE:
            arr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if arr is not None:
                # cv2 returns BGR; the codec wants RGB.
                self._last_arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
                self._last_seq = seq

        if self._last_arr is None:
            # No JPEG ever — black placeholder so the negotiation still
            # produces a steady stream and the browser shows *something*.
            self._last_arr = np.zeros((768, 1024, 3), dtype=np.uint8)

        frame = VideoFrame.from_ndarray(self._last_arr, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame


# -----------------------------------------------------------------------------
# Signaling endpoint
# -----------------------------------------------------------------------------

class WebRTCBridge:
    """Owns the active RTCPeerConnections and the SDP offer handler.

    Construct once per server; pass `state` so each new peer connection
    gets its own JpegCameraTrack pointed at the same shared frame source.
    """

    def __init__(self, shared_state) -> None:
        if not _AIORTC_AVAILABLE:
            raise RuntimeError(
                "aiortc is required for the WebRTC camera path — "
                "install via: pip install aiortc av"
            )
        if not _CV2_AVAILABLE:
            raise RuntimeError(
                "opencv-python is required for JPEG decode in the WebRTC track"
            )
        self._state = shared_state
        self._pcs: set[RTCPeerConnection] = set()

    async def handle_offer(self, payload: dict) -> dict:
        """Process a browser's SDP offer and return the answer SDP."""
        offer = RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
        pc = RTCPeerConnection()
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def _on_state_change() -> None:
            _log.info("[webrtc] connection state: %s", pc.connectionState)
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self._pcs.discard(pc)

        @pc.on("iceconnectionstatechange")
        async def _on_ice_state() -> None:
            _log.info("[webrtc] ICE state: %s", pc.iceConnectionState)

        # Outbound video track — re-encode from the SharedState JPEG source.
        track = JpegCameraTrack(self._state)
        pc.addTrack(track)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        # aiortc's setLocalDescription blocks until ICE gathering completes
        # (non-trickle), so localDescription.sdp already has candidates.
        return {
            "sdp":  pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }

    async def close_all(self) -> None:
        coros = [pc.close() for pc in self._pcs]
        await asyncio.gather(*coros, return_exceptions=True)
        self._pcs.clear()

    @property
    def active_peer_count(self) -> int:
        return len(self._pcs)
