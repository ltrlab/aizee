"""Does a leader board keep answering USB across port closes? Polls, closes, reopens; logs every step.

A board that stops answering USB setup requests hangs a port open (and leaves the port held, so later opens
say "Access is denied") while it answers polls throughout (2026-10-14, LEADER_USB_LOCKUP.md). This talks to the
port directly and never closes it after a failed read, so a hang shows the step that blocked. Run it under a
time limit, e.g. `timeout 60 python -u scripts/probe_leader_reopen.py left`; a board that hangs it needs a power
cycle (supply off as well as USB).

Usage: python scripts/probe_leader_reopen.py {left,right} [--seconds 20] [--cycles 3]
"""
import argparse
import json
from pathlib import Path
import time

import serial
from serial.tools import list_ports

from robot_platform.teleop.openrb import FRAME_SIZE, IDENTITY, LeaderSampleSkipped, decode_frame

ROOT = Path(__file__).resolve().parents[1]
T0 = time.perf_counter()


def log(text):
    print(f'{time.perf_counter() - T0:7.2f} s  {text}', flush=True)


def open_port(device):
    log('open ...')
    port = serial.Serial(device, 1_000_000, timeout=.01, write_timeout=.01)
    log('open ok')
    port.reset_input_buffer()
    port.write(b'\x50')
    reply, deadline = bytearray(), time.perf_counter() + .2
    while time.perf_counter() < deadline and len(reply) <= len(IDENTITY) + 2:
        part = port.read(1)
        reply.extend(part)
        if part == b'\n':
            break
    log(f'identity {"ok" if bytes(reply).strip() == IDENTITY else f"MISMATCH {bytes(reply)!r}"}')
    return port


def read_for(port, seconds, label):
    good = skipped = failed = 0
    slowest, previous = 0., None
    end, second = time.perf_counter() + seconds, time.perf_counter() + 1.
    while time.perf_counter() < end:
        started = time.perf_counter()
        try:
            port.reset_input_buffer()
            port.write(b'\xa6')
            raw = bytearray()
            while len(raw) < FRAME_SIZE and time.perf_counter() - started < .05:
                raw.extend(port.read(FRAME_SIZE - len(raw)))
            previous = decode_frame(bytes(raw), requested_at=started, received_at=time.perf_counter(),
                                    previous=previous, max_age_s=.05)
            good += 1
        except LeaderSampleSkipped:
            skipped += 1
        except Exception as exc:
            failed += 1
            if failed <= 3:
                log(f'{label}: read error {type(exc).__name__}: {exc}')
        slowest = max(slowest, time.perf_counter() - started)
        if time.perf_counter() >= second:
            log(f'{label}: {good} samples, {skipped} skipped, {failed} failed, slowest {1000 * slowest:.1f} ms')
            good = skipped = failed = 0
            slowest = 0.
            second += 1.
        time.sleep(max(0., .01 - (time.perf_counter() - started)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('side', choices=('left', 'right'))
    parser.add_argument('--profile', default=str(ROOT / 'config/devices/gello-dual-viewer.json'))
    parser.add_argument('--seconds', type=float, default=20., help='continuous polling before the first close')
    parser.add_argument('--cycles', type=int, default=3, help='close/reopen cycles')
    args = parser.parse_args()
    serial_number = json.loads(Path(args.profile).read_text(encoding='utf-8'))['leaders'][args.side]['usb_serial']
    devices = [p.device for p in list_ports.comports() if p.serial_number == serial_number]
    if len(devices) != 1:
        raise SystemExit(f'The {args.side} leader ({serial_number}) is not connected: Windows does not list it')
    log(f'{args.side} leader on {devices[0]}')
    port = open_port(devices[0])
    read_for(port, args.seconds, 'continuous')
    for cycle in range(1, args.cycles + 1):
        log(f'close {cycle} ...')
        port.close()
        log(f'close {cycle} ok; waiting 2 s')
        time.sleep(2)
        port = open_port(devices[0])
        read_for(port, 2, f'after reopen {cycle}')
    port.close()
    log('done: every open answered')


if __name__ == '__main__':
    main()
