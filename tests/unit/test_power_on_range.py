"""Setting the drives' power-on range: one fixed write and a save per drive, every drive read first."""
import math
import struct

import pytest

from robot_platform.control.mechanical_zero import StoppedMaintenance, maintenance_frame
from robot_platform.control.power_on_range import power_on_exchange, set_power_on_range

MOTORS = [{'name': f'joint{i}', 'id': i + 4, 'bus': 'canTest', 'uid': '0102030405060708', 'firmware': '0.2.3.34'}
          for i in range(3)]


class Drives:
    """zero_sta in RAM and flash per drive; a write sets RAM, a save copies it to flash."""

    def __init__(self, zero_sta=0, q=-.11):
        self.ram = {m['id']: zero_sta for m in MOTORS}
        self.flash = dict(self.ram)
        self.q = {m['id']: q for m in MOTORS}
        self.events = []
        self.ignore_write, self.remap = set(), False

    def __call__(self, bus, frame, kind, parameter):
        arb, data = frame
        assert frame == maintenance_frame(arb & 255, 0xAA, kind, parameter)  # The socket boundary's allowlist.
        motor = arb & 255
        self.events.append((motor, kind, parameter))
        if kind == 'read':
            value = {0x7005: 0, 0x7019: self.q[motor], 0x7029: self.ram[motor]}[parameter]
            payload = bytes([value, 0, 0, 0]) if parameter in (0x7005, 0x7029) else struct.pack('<f', value)
            return (17 << 24) | (motor << 8) | 0xAA, struct.pack('<H', parameter) + bytes(2) + payload
        if kind == 'power_on_range' and motor not in self.ignore_write:
            self.ram[motor] = 1
            if self.remap and self.q[motor] > math.pi:
                self.q[motor] -= 2 * math.pi
        if kind == 'save':
            self.flash[motor] = self.ram[motor]
        return (2 << 24) | (motor << 8) | 0xAA, bytes([128, 0, 128, 0, 128, 0, 0, 250])


def run(drives, **options):
    result = {}
    set_power_on_range(StoppedMaintenance(power_on_exchange(drives), sleep=lambda _: None), MOTORS, result,
                       sleep=lambda _: None, **options)
    return result


def test_every_drive_is_read_before_any_write_and_each_gets_one_write_and_one_save():
    drives = Drives()
    result = run(drives)
    assert drives.flash == {4: 1, 5: 1, 6: 1} and result['written'] == ['joint0', 'joint1', 'joint2']
    first_write = next(i for i, event in enumerate(drives.events) if event[1] == 'power_on_range')
    assert {(motor, parameter) for motor, kind, parameter in drives.events[:first_write]} >= {
        (m['id'], p) for m in MOTORS for p in (0x7029, 0x7019)}
    assert [e[:2] for e in drives.events if e[1] in ('power_on_range', 'save')] == [
        (4, 'power_on_range'), (4, 'save'), (5, 'power_on_range'), (5, 'save'), (6, 'power_on_range'), (6, 'save')]
    assert {kind for _, kind, _ in drives.events} <= {'read', 'power_on_range', 'save', 'stop'}
    assert all(row['zero_sta_after'] == 1 for row in result['motors']) and not result['power_cycle_verified']
    assert set(result['final_stops']) == {m['name'] for m in MOTORS}


def test_the_frame_is_the_fixed_zero_sta_write():
    assert maintenance_frame(4, 0xAA, 'power_on_range') == (
        (18 << 24) | (0xAA << 8) | 4, bytes([0x29, 0x70, 0, 0, 1, 0, 0, 0]))


def test_a_drive_already_set_is_left_alone():
    drives = Drives(zero_sta=1)
    result = run(drives)
    assert result['written'] == [] and all(row['already_set'] for row in result['motors'])
    assert not any(kind in ('power_on_range', 'save') for _, kind, _ in drives.events)


def test_a_write_that_does_not_read_back_is_never_saved_and_stops_the_rest():
    drives = Drives()
    drives.ignore_write = {5}
    with pytest.raises(RuntimeError, match='joint1: zero_sta reads 0 after the write; nothing was saved'):
        run(drives)
    assert drives.flash == {4: 1, 5: 0, 6: 0}
    assert (6, 'power_on_range', None) not in drives.events
    assert [e[0] for e in drives.events if e[1] == 'stop'] == [4, 5, 6]  # Every drive still confirmed stopped.


def test_a_drive_may_re_express_its_angle_by_a_whole_turn_but_nothing_else():
    drives = Drives(q=2 * math.pi - .11)
    drives.remap = True
    result = run(drives)
    assert [round(row['position_after_rad'], 4) for row in result['motors']] == [-.11] * 3
    drives = Drives()
    drives.q[5] = .3

    def moved(bus, frame, kind, parameter):
        reply = drives(bus, frame, kind, parameter)
        if kind == 'save' and frame[0] & 255 == 5:
            drives.q[5] = .35  # Someone moved the arm.
        return reply
    with pytest.raises(RuntimeError, match=r'joint1: its position changed by \+0\.0500 rad'):
        set_power_on_range(StoppedMaintenance(power_on_exchange(moved), sleep=lambda _: None), MOTORS, {},
                           sleep=lambda _: None)


def test_only_stopped_reads_the_write_and_the_save_get_through():
    exchange = power_on_exchange(lambda *args: None)
    for kind in ('zero', 'enable', 'control'):
        with pytest.raises(ValueError):
            exchange('canTest', (0, bytes(8)), kind, None)
    with pytest.raises(ValueError, match='not allowlisted'):
        exchange('canTest', (0, bytes(8)), 'read', 0x7028)
