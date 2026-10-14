"""OpenRB leader boards (firmware 2+): read their INFO, or set their servos' Return Delay Time.

  info                          each board's firmware, each servo's firmware, Return Delay Time and torque,
                                and the board's continuous-read count, failures and read time; from firmware 3
                                also why the board last reset (power-on, brown-out, watchdog, ...)
  return-delay VALUE            show what would change (2 us per unit; the XL330 factory value is 250)
  return-delay VALUE --confirm  write it: one EEPROM write per torque-off servo that differs, read back

Boards are opened by the pinned USB serials in the leader profile (identity checked first). Return Delay
Time is the pause before a servo answers; 0 shortens each seven-servo read by about 3.5 ms. The firmware
refuses it while force feedback is on and skips a torque-on servo.

Usage: python scripts/openrb_leader_setup.py [--profile config/devices/gello-dual-viewer.json] info
"""
import argparse
import json
from pathlib import Path
import struct
import time

ROOT = Path(__file__).resolve().parents[1]
INFO, RETURN_DELAY, RESET_CAUSE, SERVOS = 0x49, 0x44, 0x57, 7
# PM->RCAUSE bits (SAMD21): a brown-out means the board's supply dipped; the watchdog, that its loop stopped.
CAUSES = {0x01: 'power-on', 0x02: 'brown-out (1.2 V core)', 0x04: 'brown-out (3.3 V)', 0x10: 'reset pin',
          0x20: 'watchdog (its loop stopped)', 0x40: 'software (bootloader entry or upload)'}


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 255 if crc & 128 else (crc << 1) & 255
    return crc


def return_delay_request(value):
    if type(value) is not int or not 0 <= value <= 254:
        raise ValueError('Return Delay Time is 0-254 (2 us per unit)')
    return bytes((RETURN_DELAY, value, crc8((RETURN_DELAY, value))))


def _checked(frame, header, length):
    if len(frame) != length or frame[0] != header or crc8(frame[:-1]) != frame[-1]:
        raise ValueError(f'Bad or missing reply to 0x{header:02X}; firmware 2 is required')


def unknown(value):
    return None if value == 0xFF else value


def decode_info(frame):
    _checked(frame, INFO, 3 + 4 * SERVOS + 16 + 1)
    if frame[2] != SERVOS:
        raise ValueError('INFO servo count mismatch')
    servos = [dict(id=frame[3 + 4 * i], firmware=unknown(frame[4 + 4 * i]), return_delay=unknown(frame[5 + 4 * i]),
                   torque=unknown(frame[6 + 4 * i])) for i in range(SERVOS)]
    reads, failed, last_us, max_us = struct.unpack_from('<4I', frame, 3 + 4 * SERVOS)
    return dict(firmware=frame[1], servos=servos, reads=reads, failed_reads=failed, last_read_us=last_us,
                max_read_us=max_us)


def decode_reset_cause(frame):
    _checked(frame, RESET_CAUSE, 3)
    return [name for bit, name in CAUSES.items() if frame[1] & bit] or [f'unknown 0x{frame[1]:02X}']


def decode_return_delay(frame):
    _checked(frame, RETURN_DELAY, 2 + 3 * SERVOS + 1)
    return [dict(id=frame[2 + 3 * i], before=unknown(frame[3 + 3 * i]), after=unknown(frame[4 + 3 * i]))
            for i in range(SERVOS)]


def exchange(port, request, length, timeout=3.):
    port.reset_input_buffer()
    port.write(request)
    reply, deadline = bytearray(), time.monotonic() + timeout
    while len(reply) < length and time.monotonic() < deadline:
        reply.extend(port.read(length - len(reply)))
    return bytes(reply)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--profile', default=str(ROOT / 'config/devices/gello-dual-viewer.json'))
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('info')
    setting = commands.add_parser('return-delay')
    setting.add_argument('value', type=int)
    setting.add_argument('--confirm', action='store_true', help='write it (otherwise only show what would change)')
    args = parser.parse_args()
    from robot_platform.teleop.openrb import OpenRBInputs
    profile = json.loads(Path(args.profile).read_text(encoding='utf-8'))
    inputs = OpenRBInputs(profile)
    inputs.connect()  # Pinned USB serials and the firmware identity, for every board.
    try:
        for name, reader in inputs.readers.items():
            info = decode_info(exchange(reader.serial, bytes((INFO,)), 3 + 4 * SERVOS + 16 + 1))
            rate = f"{info['reads']} reads, {info['failed_reads']} failed, last {info['last_read_us']} us, " \
                   f"max {info['max_read_us']} us"
            print(f"{name} ({reader.port}): firmware {info['firmware']}; {rate}")
            if info['firmware'] >= 3:
                causes = decode_reset_cause(exchange(reader.serial, bytes((RESET_CAUSE,)), 3))
                print(f"  last reset: {', '.join(causes)}")
            for servo in info['servos']:
                print(f"  servo {servo['id']}: firmware {servo['firmware']}, return delay {servo['return_delay']}, "
                      f"torque {servo['torque']}")
            if args.command != 'return-delay':
                continue
            request = return_delay_request(args.value)
            changing = [s['id'] for s in info['servos'] if s['return_delay'] != args.value]
            if not args.confirm:
                print(f"  would set return delay {args.value} on servos {changing or 'none'} (add --confirm)")
                continue
            for row in decode_return_delay(exchange(reader.serial, request, 2 + 3 * SERVOS + 1)):
                print(f"  servo {row['id']}: return delay {row['before']} -> {row['after']}")
    finally:
        inputs.close()


if __name__ == '__main__':
    main()
