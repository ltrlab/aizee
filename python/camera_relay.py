#!/usr/bin/env python3
"""
AIZEE Camera Relay

Runs on the Jetson Orin Nano.  Subscribes to the 4 RealSense camera ZMQ
streams that arrive over the PoE Ethernet subnet (10.42.0.x) and
re-publishes each one on the *same port* bound to all interfaces
(including the Jetson WiFi at 192.168.0.27).

This lets dev-machine clients subscribe to tcp://192.168.0.27:5557-5560
without needing any direct route to the Pi PoE subnet.

Architecture:
    PI-1 (10.42.0.11:5557) ──┐
    PI-2 (10.42.0.12:5558) ──┤  camera_relay.py  ──► *:5557-5560 (WiFi)
    PI-3 (10.42.0.13:5559) ──┤
    PI-4 (10.42.0.14:5560) ──┘

Usage (manual):
    python camera_relay.py

Service:
    systemctl start aizee-camera-relay
"""

import logging
import signal
import sys
import threading
import time

import zmq

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
)
logger = logging.getLogger(__name__)

# Each entry: (upstream SUB endpoint, downstream PUB bind port)
RELAY_MAP = [
    ("tcp://10.42.0.11:5557", 5557),   # cam_front
    ("tcp://10.42.0.12:5558", 5558),   # cam_rear
    ("tcp://10.42.0.13:5559", 5559),   # cam_left
    ("tcp://10.42.0.14:5560", 5560),   # cam_right
]

_stop_event = threading.Event()


def relay_thread(ctx: zmq.Context, upstream: str, pub_port: int) -> None:
    """Subscribe to one Pi camera stream and re-publish on pub_port."""
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, 2)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(upstream)
    sub.subscribe(b"")

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 2)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(f"tcp://*:{pub_port}")

    logger.info(f"Relay: {upstream} → *:{pub_port}")

    last_log = time.time()
    count = 0

    while not _stop_event.is_set():
        try:
            if sub.poll(timeout=200):          # 200 ms wait
                # Camera frames are multipart (header + JPEG + optional
                # depth) — relay every frame so we don't truncate.
                frames = sub.recv_multipart(zmq.NOBLOCK)
                pub.send_multipart(frames, zmq.NOBLOCK)
                count += 1
        except zmq.Again:
            pass
        except zmq.ZMQError as e:
            if not _stop_event.is_set():
                logger.error(f"ZMQ error on {upstream}: {e}")
            break

        now = time.time()
        if now - last_log >= 10.0:
            logger.info(f"  *:{pub_port} relayed {count} frames in last 10 s")
            count = 0
            last_log = now

    sub.close()
    pub.close()


def main() -> None:
    ctx = zmq.Context()

    threads = []
    for upstream, port in RELAY_MAP:
        t = threading.Thread(
            target=relay_thread,
            args=(ctx, upstream, port),
            daemon=True,
            name=f"relay-{port}",
        )
        t.start()
        threads.append(t)

    def _shutdown(signum, frame):
        logger.info("Shutting down relay…")
        _stop_event.set()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    logger.info("Camera relay running.  Ctrl-C or SIGTERM to stop.")
    _stop_event.wait()

    ctx.term()
    for t in threads:
        t.join(timeout=2.0)
    logger.info("Relay stopped.")


if __name__ == "__main__":
    main()
