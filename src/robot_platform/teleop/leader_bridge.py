"""Read-only leader bridge for a local/SSH-forwarded loopback connection.

The edge requests a fresh sample with a random challenge. Source age plus the
entire round trip conservatively bounds age without comparing PC and Jetson
clocks. This is freshness, not authentication; use a pinned SSH tunnel between
machines. No arm, motion, torque, centering or automatic reconnect messages exist.
"""
import math
import re
import secrets
import threading
import time
import uuid

from .openrb import LeaderSample, LeaderSampleSkipped, OpenRBInputs
from robot_platform.transport.wire import pack_msg, unpack_msg


class CachedLeaderSampleStale(LeaderSampleSkipped):
    """No fresh cached frame yet; the independent USB sampler may still run.

    A read-only display can wait for a new sample. Control consumers must still
    stop on this failed sample and require a new explicit engagement.
    """


class LatestLeaderInputs:
    """Poll USB independently; expose one age-preserving snapshot, never a queue."""
    def __init__(self, inputs, *, rate_hz=100., clock=time.perf_counter):
        if not math.isfinite(rate_hz) or not 25 <= rate_hz <= 100:
            raise ValueError("Leader polling rate must be 25-100 Hz")
        self.inputs, self.pins, self.clock = inputs, inputs.pins, clock
        self.period = 1 / rate_hz
        self._lock, self._done = threading.Lock(), threading.Event()
        self._samples, self._failure, self._thread = None, None, None
        self.skipped, self.last_skip = 0, None

    def connect(self):
        self.inputs.connect()
        try:
            self._samples = self.inputs.poll()
            self._thread = threading.Thread(target=self._run, name="leader-usb-input", daemon=True)
            self._thread.start()
        except BaseException:
            self.inputs.close()
            raise

    def _run(self):
        due = self.clock() + self.period
        try:
            while not self._done.wait(max(0., due - self.clock())):
                try:
                    samples = self.inputs.poll()
                except LeaderSampleSkipped as exc:
                    # Publish nothing; the previous pair ages out for consumers.
                    with self._lock:
                        self.skipped += 1
                        self.last_skip = str(exc)
                else:
                    with self._lock:
                        self._samples = samples
                due += self.period
                if due < self.clock():
                    due = self.clock() + self.period
        except Exception as exc:
            with self._lock:
                self._failure = str(exc) or type(exc).__name__
            self._done.set()

    def poll(self):
        with self._lock:
            if self._failure:
                raise RuntimeError(self._failure)
            if self._samples is None or self._done.is_set():
                raise RuntimeError("Leader sampler is not running")
            samples = dict(self._samples)
        now = self.clock()
        ages = [now - s.sampled_at for s in samples.values()]
        if any(not math.isfinite(age) or age < 0 for age in ages):
            raise ValueError("Cached leader source has an invalid or future timestamp")
        if any(age >= self.inputs.max_age_s for age in ages):
            raise CachedLeaderSampleStale("Cached leader source is stale")
        return samples

    def close(self):
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=1.)
        self.inputs.close()


def sample_reply(request, inputs, epoch, *, clock=time.perf_counter):
    if (set(request) != {"type", "nonce"} or request["type"] != "sample"
            or not isinstance(request["nonce"], str) or not re.fullmatch(r"[0-9a-f]{32}", request["nonce"])):
        raise ValueError("Only a fresh input sample request is supported")
    samples = inputs.poll()
    now = clock()
    return dict(protocol_version=1, epoch=epoch, nonce=request["nonce"], leaders={
        name: dict(usb_serial=inputs.pins[name], raw_positions=list(s.raw_positions),
                   sequence=s.sequence, age_s=now - s.sampled_at,
                   joystick=list(s.joystick), board_sample_ms=s.board_sample_ms,
                   board_reply_ms=s.board_reply_ms)
        for name, s in samples.items()})


class LeaderSnapshotClient:
    def __init__(self, exchange, profile, *, clock=time.perf_counter):
        # Reuse the device-profile validator; this does not open local ports.
        checked = OpenRBInputs(profile, clock=clock)
        self.pins, self.max_age_s, self.max_skew_s = checked.pins, checked.max_age_s, checked.max_skew_s
        self.exchange, self.clock = exchange, clock
        self.epoch, self.previous, self.fault = None, {}, None

    def poll(self):
        if self.fault:
            raise RuntimeError("Leader bridge is faulted; explicit new session required")
        try:
            nonce = secrets.token_hex(16)
            started = self.clock()
            reply = self.exchange(dict(type="sample", nonce=nonce))
            received = self.clock()
            elapsed = received - started
            if (reply.get("protocol_version") != 1 or reply.get("nonce") != nonce
                    or not isinstance(reply.get("epoch"), str) or not reply["epoch"]
                    or (self.epoch is not None and reply["epoch"] != self.epoch)
                    or set(reply["leaders"]) != set(self.pins)
                    or not math.isfinite(elapsed) or elapsed < 0):
                raise ValueError("Leader bridge version, challenge, epoch or roster changed")
            samples = {}
            for name, row in reply["leaders"].items():
                age = row["age_s"] + elapsed
                raw, sequence = row["raw_positions"], row["sequence"]
                if (row["usb_serial"] != self.pins[name] or len(raw) != 7
                        or any(type(q) is not int or not -(1 << 31) <= q < (1 << 31) for q in raw)
                        or type(sequence) is not int or not 0 < sequence < (1 << 32)
                        or not math.isfinite(age) or row["age_s"] < 0
                        or any(type(row[k]) is not int or not 0 <= row[k] < (1 << 32)
                               for k in ("board_sample_ms", "board_reply_ms"))
                        or len(row["joystick"]) != 4):
                    raise ValueError("Missing or invalid remote leader sample")
                if age >= self.max_age_s:
                    raise LeaderSampleSkipped("Remote leader sample exceeded its source-age limit")
                previous = self.previous.get(name)
                if previous and (not 0 < (sequence - previous.sequence) & 0xffffffff < 0x80000000
                                 or (row["board_reply_ms"] - previous.board_reply_ms) & 0xffffffff >= 0x80000000):
                    raise ValueError("Remote leader sample repeated, reordered or restarted")
                samples[name] = LeaderSample(tuple(q * 2 * math.pi / 4096 for q in raw), tuple(raw),
                    sequence, received - age, received, tuple(row["joystick"]),
                    row["board_sample_ms"], row["board_reply_ms"])
            stamps = [s.sampled_at for s in samples.values()]
            if max(stamps) - min(stamps) > self.max_skew_s:
                raise LeaderSampleSkipped("Remote leader pair exceeds skew budget")
            self.epoch, self.previous = reply["epoch"], samples
            return samples
        except LeaderSampleSkipped:
            # Timing only: nothing is returned or remembered; the caller stops.
            raise
        except Exception as exc:
            self.fault = str(exc) or type(exc).__name__
            raise


class LeaderZmqInput(LeaderSnapshotClient):
    def __init__(self, endpoint, profile, *, timeout_ms=25, clock=time.perf_counter):
        import zmq
        if not endpoint.startswith("tcp://127.0.0.1:") or not 1 <= timeout_ms <= 50:
            raise ValueError("Leader bridge requires loopback/SSH and a 1-50 ms timeout")
        super().__init__(self._exchange, profile, clock=clock)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        for option, value in ((zmq.LINGER, 0), (zmq.SNDHWM, 1), (zmq.RCVHWM, 1),
                              (zmq.SNDTIMEO, timeout_ms), (zmq.RCVTIMEO, timeout_ms),
                              (zmq.IMMEDIATE, 1), (zmq.MAXMSGSIZE, 16384)):
            self.socket.setsockopt(option, value)
        self.socket.connect(endpoint)

    def _exchange(self, request):
        try:
            self.socket.send(pack_msg(request))
            return unpack_msg(self.socket.recv())
        except Exception:
            self.close()
            raise

    def close(self):
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None
        self.context.term()


def main():
    import argparse
    import json
    from pathlib import Path
    import zmq
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:8792")
    parser.add_argument("--seconds", type=float, default=300.)
    parser.add_argument("--rate", type=float, default=100., help="USB pair sampling rate, 25-100 Hz (default: 100)")
    args = parser.parse_args()
    if not re.fullmatch(r"tcp://127\.0\.0\.1:[0-9]{4,5}", args.endpoint) or not 0 < args.seconds <= 3600:
        parser.error("A loopback endpoint and 0-3600 second duration are required")
    inputs = LatestLeaderInputs(OpenRBInputs(json.loads(args.profile.read_text())), rate_hz=args.rate)
    context, epoch = zmq.Context(), str(uuid.uuid4())
    server = context.socket(zmq.REP)
    server.setsockopt(zmq.LINGER, 0)
    server.setsockopt(zmq.SNDHWM, 1)
    server.setsockopt(zmq.RCVHWM, 1)
    server.setsockopt(zmq.SNDTIMEO, 50)
    server.setsockopt(zmq.MAXMSGSIZE, 16384)
    failure = None
    try:
        inputs.connect()
        server.bind(args.endpoint)
        print(json.dumps(dict(endpoint=args.endpoint, epoch=epoch, input_only=True)), flush=True)
        deadline = time.perf_counter() + args.seconds
        while time.perf_counter() < deadline:
            if not server.poll(50):
                continue
            try:
                request = unpack_msg(server.recv())
                if failure:
                    raise RuntimeError(failure)
                reply = sample_reply(request, inputs, epoch)
            except Exception as exc:
                failure = str(exc) or type(exc).__name__
                inputs.close()
                reply = dict(error=failure)
            server.send(pack_msg(reply))
    finally:
        inputs.close()
        server.close(linger=0)
        context.term()


if __name__ == "__main__":
    main()
