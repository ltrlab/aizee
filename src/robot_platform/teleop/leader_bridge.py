"""Leader bridge for a local/SSH-forwarded loopback connection.

The edge requests a fresh sample with a random challenge. Source age plus the
entire round trip conservatively bounds age without comparing PC and Jetson
clocks. This is freshness, not authentication; use a pinned SSH tunnel between
machines. No arm, motion, centering or automatic reconnect messages exist. The one
command a request may carry is gripper force feedback (teleop.grip_feedback). Per leader it is
a wall (where the trigger meets the object the robot's gripper holds), which the sampler
renders as a spring from its own trigger readings, or a bounded steady push. The sampler
sends it to that leader's trigger servo only, in the direction the leader profile says
opens it, and turns it off when requests stop carrying it or stop arriving.
"""
import math
import re
import secrets
import threading
import time
import uuid

from .openrb import LeaderSample, LeaderSampleSkipped, OpenRBInputs
from robot_platform.transport.wire import pack_msg, unpack_msg


# A control loop's sampler polls the leaders as fast as they answer (both boards in parallel take about 6 ms), so
# a 50 Hz loop reads a sample at most one poll old, not up to 10 ms as at 100 Hz.
SAMPLER_HZ = 200.
# Force feedback: a strength older than GRIP_STALE_S (the loop stopped asking) turns the trigger off; an active
# trigger is refreshed at least every GRIP_REFRESH_S, well inside the firmware's 200 ms zeroing watchdog.
GRIP_STALE_S = .1
GRIP_REFRESH_S = .02
# A wall pushes nothing at its reading and k mA per radian squeezed past it, up to its cap, from the sampler's own
# latest trigger reading (a poll old, not the robot loop's round trip). A new wall's push rises over WALL_ONSET_S,
# so a trigger already squeezed past it is not kicked.
WALL_KEYS = frozenset({'wall', 'k', 'cap'})
WALL_ONSET_S = .06


class CachedLeaderSampleStale(LeaderSampleSkipped):
    """No fresh cached frame yet; the independent USB sampler may still run.

    A read-only display can wait for a new sample. Control consumers must still
    stop on this failed sample and require a new explicit engagement.
    """


def next_poll(due, period, now):
    """When the sampler polls next: a period after the last due time, or at once after a read that overran it.
    Starting a whole period after an overrun idled the USB boards: a 5.8 ms pair read at 200 Hz sampled at
    90 Hz instead of the boards' 172 Hz, and leader samples reached the robot 20 ms old (2026-10-12)."""
    due += period
    return due if due >= now else now


class LatestLeaderInputs:
    """Poll USB independently; expose one age-preserving snapshot, never a queue."""
    def __init__(self, inputs, *, rate_hz=100., clock=time.perf_counter):
        if not math.isfinite(rate_hz) or not 25 <= rate_hz <= 250:
            raise ValueError("Leader polling rate must be 25-250 Hz")
        self.inputs, self.pins, self.clock = inputs, inputs.pins, clock
        self.period = 1 / rate_hz
        self._lock, self._done = threading.Lock(), threading.Event()
        self._samples, self._failure, self._thread = None, None, None
        self.skipped, self.last_skip = 0, None
        # Force feedback: the leaders whose trigger the profile names, the loop's latest strengths, and what each
        # trigger was last sent (None: off). Only the sampler thread writes to the boards.
        self.triggers = dict(getattr(inputs, 'triggers', None) or {})
        self._grip = self._grip_at = None
        self._sent = {name: None for name in self.triggers}
        self._sent_at = {name: -math.inf for name in self.triggers}
        self._wall_since = {name: None for name in self.triggers}  # When each trigger's current wall appeared.

    @property
    def grip(self):
        with self._lock:
            return None if self._grip is None else dict(self._grip)

    @grip.setter
    def grip(self, value):
        self.set_grip(value)

    def set_grip(self, grip):
        """The loop's force feedback {leader: wall or mA} (grip_value), or None for none. A leader without a trigger
        in the profile is ignored; a push over the firmware cap is capped when sent."""
        if grip is not None:
            checked = {name: grip_value(value) for name, value in grip.items()} if isinstance(grip, dict) else None
            if checked is None or any(value is None for value in checked.values()):
                raise ValueError("Force feedback is a wall or finite mA per leader, not negative")
            grip = {name: value for name, value in checked.items() if name in self.triggers}
        with self._lock:
            self._grip, self._grip_at = grip, self.clock()

    def _wall_push(self, wall, sample, trigger, age):
        """A wall's push now (mA): k per radian the trigger's latest reading is squeezed past it, capped, rising
        over WALL_ONSET_S from when it appeared."""
        from .grip_feedback import past_wall
        channel, opens = trigger
        # Never an exception here: it would stop the sampler, and with it teleop. No usable reading: no push.
        positions = getattr(sample, 'positions_rad', None)
        if not isinstance(positions, (tuple, list)) or not 0 < channel <= len(positions):
            return 0
        reading = positions[channel - 1]
        if type(reading) not in (int, float) or not math.isfinite(reading):
            return 0
        push = min(wall['cap'], wall['k'] * past_wall(wall['wall'], reading, opens))
        return int(round(push * min(1., max(0., age) / WALL_ONSET_S)))

    def _feedback_frames(self, now):
        """The FF_CURRENT frames due now: an active trigger refreshed or changed, a stopped one turned off once."""
        from .grip_feedback import FF_DISABLE, SERVOS, ff_frame, trigger_currents
        with self._lock:
            grip, at, samples = self._grip, self._grip_at, self._samples
        fresh = grip is not None and at is not None and now - at <= GRIP_STALE_S
        frames = {}
        for name, trigger in self.triggers.items():
            value = grip.get(name, 0.) if fresh else None
            if isinstance(value, dict):
                if self._wall_since[name] is None:
                    self._wall_since[name] = now
                wanted = self._wall_push(value, (samples or {}).get(name), trigger, now - self._wall_since[name])
            else:
                self._wall_since[name] = None
                wanted = None if value is None else int(round(value))
            sent = self._sent[name]
            if wanted is None:
                if sent is not None:
                    frames[name] = ff_frame([FF_DISABLE] * SERVOS)
                    self._sent[name] = None
            elif sent is None or abs(wanted - sent) >= 2 or now - self._sent_at[name] >= GRIP_REFRESH_S:
                frames[name] = ff_frame(trigger_currents(wanted, trigger))
                self._sent[name], self._sent_at[name] = wanted, now
        return frames

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
                    frames = self._feedback_frames(self.clock()) if self.triggers else None
                    samples = self.inputs.poll(frames) if frames else self.inputs.poll()
                except LeaderSampleSkipped as exc:
                    # Publish nothing; the previous pair ages out for consumers.
                    with self._lock:
                        self.skipped += 1
                        self.last_skip = str(exc)
                else:
                    with self._lock:
                        self._samples = samples
                due = next_poll(due, self.period, self.clock())
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


def grip_value(value):
    """One leader's force feedback, or None when malformed. It is either:
    - a steady push in mA, 0-1000; or
    - a wall {'wall': trigger reading (rad), 'k': mA per rad squeezed past it, 0-1e6, 'cap': mA, 0-1000}."""
    if type(value) in (int, float):
        return float(value) if math.isfinite(value) and 0 <= value <= 1000 else None
    if not isinstance(value, dict) or set(value) != WALL_KEYS or any(
            type(value[key]) not in (int, float) or not math.isfinite(value[key]) for key in WALL_KEYS):
        return None
    if not abs(value['wall']) <= 1000 or not 0 <= value['k'] <= 1e6 or not 0 <= value['cap'] <= 1000:
        return None
    return {key: float(value[key]) for key in WALL_KEYS}


def grip_request(value):
    """A request's force feedback {leader: wall or mA}, or None (none asked for, or malformed: feedback off, never
    a fault)."""
    if not isinstance(value, dict) or not value or len(value) > 8:
        return None
    checked = {name: grip_value(item) for name, item in value.items()}
    if any(not isinstance(name, str) or item is None for name, item in checked.items()):
        return None
    return checked


def sample_reply(request, inputs, epoch, *, clock=time.perf_counter):
    if (not {"type", "nonce"} <= set(request) <= {"type", "nonce", "grip"} or request["type"] != "sample"
            or not isinstance(request["nonce"], str) or not re.fullmatch(r"[0-9a-f]{32}", request["nonce"])):
        raise ValueError("Only a fresh input sample request is supported")
    set_grip = getattr(inputs, "set_grip", None)
    if set_grip is not None:
        # Every request says whether force feedback is wanted; one without it turns the triggers off.
        set_grip(grip_request(request.get("grip")))
    samples = inputs.poll()
    now = clock()
    return dict(protocol_version=1, epoch=epoch, nonce=request["nonce"], leaders={
        name: dict(usb_serial=inputs.pins[name], raw_positions=list(s.raw_positions),
                   sequence=s.sequence, age_s=now - s.sampled_at,
                   joystick=list(s.joystick), board_sample_ms=s.board_sample_ms,
                   board_reply_ms=s.board_reply_ms)
        for name, s in samples.items()})


class LeaderSnapshotClient:
    def __init__(self, exchange, profile, *, clock=time.perf_counter, allow_repeated=False):
        # Reuse the device-profile validator; this does not open local ports.
        checked = OpenRBInputs(profile, clock=clock)
        self.pins, self.max_age_s, self.max_skew_s = checked.pins, checked.max_age_s, checked.max_skew_s
        self.exchange, self.clock = exchange, clock
        self.epoch, self.previous, self.fault = None, {}, None
        self.allow_repeated = allow_repeated
        self.grip = None  # The loop's force-feedback strengths {leader: mA}, sent with each request; None: off.

    def poll(self):
        if self.fault:
            raise RuntimeError("Leader bridge is faulted; explicit new session required")
        try:
            nonce = secrets.token_hex(16)
            started = self.clock()
            request = dict(type="sample", nonce=nonce)
            if self.grip is not None:
                request["grip"] = dict(self.grip)
            reply = self.exchange(request)
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
                if self.allow_repeated and previous and sequence == previous.sequence:
                    if (tuple(raw) != previous.raw_positions or tuple(row['joystick']) != previous.joystick
                            or row['board_sample_ms'] != previous.board_sample_ms
                            or row['board_reply_ms'] != previous.board_reply_ms):
                        raise ValueError('Remote leader changed without a new sequence')
                    if received - previous.sampled_at >= self.max_age_s:
                        raise LeaderSampleSkipped('Repeated remote sample expired')
                    samples[name] = previous  # Preserve the first conservative age bound.
                    continue
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
