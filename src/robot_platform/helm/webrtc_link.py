"""A remote headset's WebRTC link, from Helm's side: the head camera's newest frames to the worker (webrtc_worker, a
process of its own), and the pose channel's messages back. Only asked for by a page opened with ?remote: the local path,
and Helm's own views, stay on their WebSockets. Asyncio, on Helm's loop.
"""

import asyncio
import importlib.util
import json
import logging
import os
from pathlib import Path
import subprocess
import sys

from .webrtc_worker import write_message

log = logging.getLogger(__name__)

ROLE = 'head'
FRAME_POLL_S = 0.02
ANSWER_TIMEOUT_S = 10.0
BACKLOG_BYTES = 256 * 1024  # More than this waiting for the worker: it is behind, and this frame is not sent.
SOURCE = Path(__file__).resolve().parents[2]
DEPS = (
    'HELM_WEBRTC_DEPS'  # Helm on Minerva: the worker's own dependencies (requirements/helm-webrtc.lock), if installed.
)
COMPLETE = '.complete'  # As ops.helm_service: the folder finished installing.


class WebRTCLink:
    """`camera_video`: Helm's camera relay (camera_video.CameraVideo). `on_pose`: called with each pose channel message's
    text and this link's owner. Starting the worker is deferred to the first offer: most Helms never see one."""

    def __init__(self, camera_video, on_pose, *, command=None, offload=None):
        self.camera_video, self.on_pose = camera_video, on_pose
        self.command = command or (sys.executable, '-m', 'robot_platform.helm.webrtc_worker')
        self.offload = offload or (lambda function: asyncio.get_running_loop().run_in_executor(None, function))
        self.process = self.reader = self.publisher = self.viewer = None
        self.owner = None
        self.state = 'none'
        self.stats = None
        self.next_id = 0
        self.answers = {}

    def available(self):
        """Whether this machine can run the worker: looked up, not imported, as they would stall Helm's loop."""
        deps = os.environ.get(DEPS)
        if deps:
            return (Path(deps) / COMPLETE).is_file()
        return all(importlib.util.find_spec(name) is not None for name in ('aiortc', 'av'))

    async def _start_worker(self):
        deps = os.environ.get(DEPS)
        paths = [deps, str(SOURCE)] if deps else [path for path in sys.path if path]
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(paths), PYTHONUNBUFFERED='1')
        self.process = await asyncio.create_subprocess_exec(
            *self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env, limit=1024 * 1024
        )
        self.reader = asyncio.create_task(self._read(self.process))

    async def _read(self, process):
        while line := await process.stdout.readline():
            try:
                message = json.loads(line)
                kind = message.get('kind')
            except (ValueError, AttributeError):
                continue
            if kind == 'pose':
                try:
                    self.on_pose(message.get('text'), self.owner)
                except Exception:  # noqa: BLE001 - one bad pose must not end the link's reader.
                    log.exception('A remote pose')
            elif kind in ('answer', 'error'):
                waiter = self.answers.pop(message.get('id'), None)
                if waiter is not None and not waiter.done():
                    waiter.set_result(message)
            elif kind == 'state':
                self.state = message.get('state')
                if self.state in ('closed', 'failed'):
                    await self._stop_frames()
            elif kind == 'stats':
                self.stats = message
        for waiter in self.answers.values():
            if not waiter.done():
                waiter.set_result({'kind': 'error', 'error': 'The WebRTC worker stopped'})
        self.answers.clear()
        self.state, self.process = 'none', None
        await self._stop_frames()

    async def _send(self, header, payload=b''):
        process = self.process
        if process is None or process.stdin.is_closing():
            raise ConnectionError('The WebRTC worker is not running')
        write_message(process.stdin, header, payload)
        await process.stdin.drain()

    async def offer(self, sdp, kind, owner):
        """A page's offer: its answer as {'sdp', 'type'}, or ValueError. It replaces any peer before it."""
        if self.camera_video is None:
            raise ValueError('Live camera video is not configured')
        if not self.available():
            raise ValueError('This Helm has no WebRTC (aiortc): the headset stays on its ordinary link')
        if self.process is None:
            await self._start_worker()
        self.owner, self.state = owner, 'new'  # Before the worker can speak of the new peer.
        self.next_id += 1
        waiter = self.answers[self.next_id] = asyncio.get_running_loop().create_future()
        await self._send({'kind': 'offer', 'id': self.next_id, 'sdp': sdp, 'type': kind})
        try:
            answer = await asyncio.wait_for(waiter, ANSWER_TIMEOUT_S)
        except asyncio.TimeoutError:
            self.answers.pop(self.next_id, None)
            raise ValueError('The WebRTC worker did not answer') from None
        if answer['kind'] == 'error':
            raise ValueError(answer['error'])
        if self.publisher is None:
            self.viewer = await self.offload(self.camera_video.subscribe)
            self.publisher = asyncio.create_task(self._publish())
        return {'sdp': answer['sdp'], 'type': answer['type']}

    async def _publish(self):
        sent = None
        try:
            while True:
                _status, frames = self.camera_video.snapshot()
                frame = frames.get(ROLE)
                if frame is not None and frame[0]['sequence'] != sent and self._caught_up():
                    await self._send({'kind': 'frame', 'sequence': frame[0]['sequence']}, frame[1])
                    sent = frame[0]['sequence']
                await asyncio.sleep(FRAME_POLL_S)
        except ConnectionError:
            pass

    def _caught_up(self):
        process = self.process
        return process is not None and process.stdin.transport.get_write_buffer_size() < BACKLOG_BYTES

    def status(self, reply):
        """Helm's word to the page, over the pose channel: dropped if the worker is behind, as the next is due soon."""
        if self.process is not None and self._caught_up():
            write_message(self.process.stdin, {'kind': 'status', 'text': json.dumps(reply, allow_nan=False)})

    async def _stop_frames(self):
        publisher, viewer, self.publisher, self.viewer = self.publisher, self.viewer, None, None
        if publisher is not None:
            publisher.cancel()
            await asyncio.gather(publisher, return_exceptions=True)
        if viewer is not None:
            await self.offload(lambda: self.camera_video.unsubscribe(viewer))

    async def close(self):
        """Helm is going down: the worker ends with its stdin."""
        await self._stop_frames()
        process, reader = self.process, self.reader
        if process is not None:
            process.stdin.close()
            try:
                await asyncio.wait_for(process.wait(), 3.0)
            except asyncio.TimeoutError:
                process.kill()
        if reader is not None:
            await asyncio.gather(reader, return_exceptions=True)
