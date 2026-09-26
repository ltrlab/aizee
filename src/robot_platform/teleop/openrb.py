"""Read-only OpenRB input with explicit source freshness; no robot joint mapping.

Uses IDENT and FRESH_POLL only. Legacy polls have no source validity and are not
accepted for guarded teleoperation. Setup, centering and force feedback remain
separate tools. A failed read raises; callers must latch a stop, not reuse data.
A skipped sample was complete and CRC-checked, so the USB stream stays in step;
the input owner may keep sampling, but control consumers still get no data.
"""
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor, wait
import math
import struct
import time


SERVO_COUNT = 7
FRAME_SIZE = 50
IDENTITY = b"AIZEE-OPENRB-LEADER"


class LeaderSampleSkipped(ValueError):
    """A complete frame missed its age/pairing bound or reported a partial servo read.

    The frame was fully consumed and CRC-checked, so the device stays synchronized.
    Nothing from it is published; consumers see only the older sample age out.
    """


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 255 if crc & 128 else (crc << 1) & 255
    return crc


@dataclass(frozen=True)
class LeaderSample:
    positions_rad: tuple[float, ...]
    raw_positions: tuple[int, ...]
    sequence: int
    sampled_at: float
    received_at: float
    joystick: tuple[int, int, int, int]
    board_sample_ms: int
    board_reply_ms: int


def decode_frame(frame, *, requested_at, received_at, previous=None, max_age_s=.05):
    if (len(frame) != FRAME_SIZE or frame[:2] != bytes((0xA6, SERVO_COUNT))
            or crc8(frame[:-1]) != frame[-1]):
        raise ValueError("Invalid OpenRB freshness frame/CRC; current firmware is required")
    positions = struct.unpack_from("<7i", frame, 2)
    joystick = struct.unpack_from("<hhBB", frame, 30)
    sequence, sample_ms, reply_ms, valid = struct.unpack_from("<IIIB", frame, 36)
    age = ((reply_ms - sample_ms) & 0xffffffff) / 1000 + received_at - requested_at
    if (not all(math.isfinite(v) for v in (requested_at, received_at, max_age_s))
            or received_at < requested_at or max_age_s <= 0 or sequence == 0):
        raise ValueError("OpenRB servo data is incomplete, faulted or stale")
    if age >= max_age_s:
        raise LeaderSampleSkipped("OpenRB sample exceeded its source-age limit")
    if valid != 1:
        raise LeaderSampleSkipped("OpenRB board reported an incomplete servo read")
    if previous is not None:
        sequence_delta = (sequence - previous.sequence) & 0xffffffff
        time_delta = (reply_ms - previous.board_reply_ms) & 0xffffffff
        if not 0 < sequence_delta < 0x80000000 or time_delta >= 0x80000000:
            raise ValueError("OpenRB source repeated, reordered or restarted")
    return LeaderSample(tuple(value * (2 * math.pi / 4096) for value in positions), positions,
                        sequence, received_at - age, received_at, joystick, sample_ms, reply_ms)


class OpenRBInput:
    """Explicit USB identity, bounded read/write, no automatic reconnect."""
    def __init__(self, port, *, usb_serial, max_age_s=.05, clock=time.perf_counter):
        if not usb_serial or not math.isfinite(max_age_s) or not 0 < max_age_s <= .1:
            raise ValueError("Pinned USB serial and source-age limit in (0, .1] required")
        self.port, self.usb_serial, self.max_age_s, self.clock = port, usb_serial, max_age_s, clock
        self.serial = None
        self.previous = None

    def connect(self):
        import serial
        from serial.tools import list_ports
        if self.serial is not None:
            raise RuntimeError("Already connected")
        found = [port for port in list_ports.comports()
                 if port.device == self.port and port.serial_number == self.usb_serial
                 and port.vid == 0x2F5D and port.pid == 0x2202]
        if len(found) != 1:
            raise ValueError("Pinned OpenRB USB device is absent or ambiguous")
        # Only an explicitly pinned USB CDC device is opened; never Bluetooth.
        connection = serial.Serial(self.port, 1_000_000, timeout=.01, write_timeout=.01)
        self.serial = connection
        try:
            connection.reset_input_buffer()
            connection.write(b"\x50")
            deadline = self.clock() + .2
            reply = bytearray()
            while self.clock() < deadline and len(reply) <= len(IDENTITY) + 2:
                part = connection.read(1)
                reply.extend(part)
                if part == b"\n":
                    break
            if bytes(reply).strip() != IDENTITY:
                raise ValueError("OpenRB firmware identity mismatch")
            self.previous = None
        except BaseException:
            self.close()
            raise

    def poll(self):
        if self.serial is None:
            raise RuntimeError("OpenRB input is not connected")
        try:
            self.serial.reset_input_buffer()
            started = self.clock()
            self.serial.write(b"\xa6")
            raw = bytearray()
            while len(raw) < FRAME_SIZE and self.clock() - started < self.max_age_s:
                raw.extend(self.serial.read(FRAME_SIZE - len(raw)))
            sample = decode_frame(bytes(raw), requested_at=started, received_at=self.clock(),
                                  previous=self.previous, max_age_s=self.max_age_s)
            self.previous = sample
            return sample
        except LeaderSampleSkipped:
            raise
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None


class OpenRBInputs:
    """Profile-named inputs resolved by USB serial, independent of robot joints."""
    def __init__(self, profile, *, clock=time.perf_counter):
        self.clock = clock
        self.pins = {name: item["usb_serial"] for name, item in profile["leaders"].items()}
        self.max_age_s = profile.get("max_age_s", .05)
        self.max_skew_s = profile.get("max_skew_s", .02)
        if (not self.pins or len(self.pins) > 8 or len(set(self.pins.values())) != len(self.pins)
                or any(not name or not serial for name, serial in self.pins.items())
                or not math.isfinite(self.max_age_s) or not 0 < self.max_age_s <= .1
                or not math.isfinite(self.max_skew_s) or not 0 < self.max_skew_s <= self.max_age_s):
            raise ValueError("Unique pinned leader identities and bounded freshness/skew required")
        self.readers = {}
        self._pool = None

    def connect(self):
        from serial.tools import list_ports
        if self.readers:
            raise RuntimeError("Inputs already connected")
        ports = list(list_ports.comports())
        try:
            for name, serial in self.pins.items():
                matches = [p for p in ports if p.serial_number == serial
                           and (p.vid, p.pid) == (0x2F5D, 0x2202)]
                if len(matches) != 1:
                    raise ValueError(f"Leader {name} is missing or ambiguous")
                reader = OpenRBInput(matches[0].device, usb_serial=serial,
                                     max_age_s=self.max_age_s, clock=self.clock)
                self.readers[name] = reader
                reader.connect()
            # Each board has its own USB/Dynamixel bus. Read them concurrently
            # so the second arm does not wait for the first arm's bus round trip.
            self._pool = ThreadPoolExecutor(max_workers=len(self.readers), thread_name_prefix="openrb-read")
        except BaseException:
            self.close()
            raise

    def poll(self):
        if set(self.readers) != set(self.pins) or self._pool is None:
            raise RuntimeError("All leader inputs must be explicitly connected")
        try:
            pending = {name: self._pool.submit(reader.poll) for name, reader in self.readers.items()}
            # Drain every bounded read before closing either device after a
            # failure. No partial pair can escape as a successful sample.
            wait(pending.values())
            errors = [future.exception() for future in pending.values() if future.exception()]
            hard = [error for error in errors if not isinstance(error, LeaderSampleSkipped)]
            if hard or errors:
                raise (hard or errors)[0]
            samples = {name: future.result() for name, future in pending.items()}
            now = self.clock()
            stamps = [s.sampled_at for s in samples.values()]
            if any(not math.isfinite(now - stamp) or now < stamp for stamp in stamps):
                raise ValueError("Leader pair has an invalid or future timestamp")
            if (any(now - stamp >= self.max_age_s for stamp in stamps)
                    or max(stamps) - min(stamps) > self.max_skew_s):
                raise LeaderSampleSkipped("Leader pair age or skew exceeded the input profile")
            return samples
        except LeaderSampleSkipped:
            raise
        except BaseException:
            self.close()
            raise

    def close(self):
        pool, self._pool = self._pool, None
        if pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)
        readers, self.readers = self.readers, {}
        try:
            for reader in readers.values():
                reader.close()
        finally:
            self.readers = {}


def main():
    """Read-only discovery/capture, without importing any robot controller."""
    import argparse
    from dataclasses import asdict
    import datetime
    import json
    from pathlib import Path
    from serial.tools import list_ports
    parser = argparse.ArgumentParser(description="Inspect OpenRB inputs; never sends torque or centering commands")
    parser.add_argument("--usb-serial", action="append", default=[])
    parser.add_argument("--seconds", type=float, default=60.)
    parser.add_argument("--rate", type=float, default=25.)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    ports = [p for p in list_ports.comports() if (p.vid, p.pid) == (0x2F5D, 0x2202)]
    devices = [dict(port=p.device, usb_serial=p.serial_number, location=p.location) for p in ports]
    if not args.usb_serial:
        print(json.dumps(devices, indent=2))
        return
    if (len(set(args.usb_serial)) != len(args.usb_serial) or args.output is None or args.output.exists()
            or not 0 < args.seconds <= 300 or not 1 <= args.rate <= 100):
        parser.error("Use unique USB serials, a new output file, 0-300 seconds and 1-100 Hz")
    selected = [p for p in ports if p.serial_number in args.usb_serial]
    if len(selected) != len(args.usb_serial):
        parser.error("Every pinned USB serial must be present exactly once")
    inputs = OpenRBInputs(dict(leaders={p.serial_number: dict(usb_serial=p.serial_number) for p in selected}))
    report = dict(schema_version=1, utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  devices=[d for d in devices if d["usb_serial"] in inputs.pins], rate_hz=args.rate,
                  hardware_motion=False, samples=[], error=None)
    try:
        inputs.connect()
        started = due = time.perf_counter()
        while time.perf_counter() - started < args.seconds:
            time.sleep(max(0., due - time.perf_counter()))
            cycle_start = time.perf_counter()
            samples = {name: asdict(sample) for name, sample in inputs.poll().items()}
            completed = time.perf_counter()
            report["samples"].append(dict(started_at=cycle_start, scheduled_at=due,
                                          completed_at=completed, leaders=samples))
            due += 1 / args.rate
            # Do not issue catch-up polls after a host scheduling stall.
            if due < completed:
                due = completed + 1 / args.rate
    except (Exception, KeyboardInterrupt) as exc:
        report["error"] = str(exc) or type(exc).__name__
    finally:
        inputs.close()
    rows = report["samples"]
    if rows:
        report["summary"] = dict(cycles=len(rows),
            max_source_age_ms=max((r["completed_at"] - s["sampled_at"]) * 1000
                                  for r in rows for s in r["leaders"].values()),
            max_pair_skew_ms=max((max(s["sampled_at"] for s in r["leaders"].values())
                                 - min(s["sampled_at"] for s in r["leaders"].values())) * 1000 for r in rows),
            scheduled_deadline_misses=sum(r["completed_at"] > r["scheduled_at"] + 1 / args.rate for r in rows),
            raw_range_by_board={name: [max(r["leaders"][name]["raw_positions"][i] for r in rows)
                                       - min(r["leaders"][name]["raw_positions"][i] for r in rows)
                                       for i in range(7)] for name in inputs.pins})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as f:
        json.dump(report, f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps(dict(output=str(args.output), summary=report.get("summary"), error=report["error"]), indent=2))
    if report["error"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
