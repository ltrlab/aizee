"""Stopped-reader preflight names every silent motor; identity changes stay fatal."""
import pytest

from robot_platform.control.joint_capture import MotorSilent, live_preflight, live_scan

MOTORS = [dict(name=f'm{i}', bus='canL', id=4 + i, uid=f'uid{i}', firmware='1.0') for i in range(4)]


class Reader:
    def __init__(self, silent=(), silent_after_identity=(), wrong_uid=None):
        self.silent, self.late, self.wrong_uid = set(silent), set(silent_after_identity), wrong_uid
        self.calls = []

    def request(self, motor, kind, parameter=None):
        self.calls.append((motor['name'], kind))
        if motor['name'] in self.silent or (kind != 'identity' and motor['name'] in self.late):
            raise TimeoutError('timed out')
        if kind == 'identity':
            return 'other' if motor['name'] == self.wrong_uid else motor['uid']
        if kind == 'version':
            return motor['firmware']
        if kind == 'read':
            return 0
        return {'mode': 0}


def test_every_silent_motor_is_named_not_just_the_first():
    result = {}
    reader = Reader(silent=('m1', 'm3'))
    with pytest.raises(RuntimeError, match='Not answering: m1, m3'):
        live_preflight(reader, MOTORS, result)
    assert result == {'error_code': 'motors_unresponsive', 'unresponsive': ['m1', 'm3']}
    assert [name for name, kind in reader.calls if kind == 'identity'] == ['m0', 'm1', 'm2', 'm3']


def test_a_motor_that_drops_after_identity_is_also_named():
    result = {}
    with pytest.raises(RuntimeError, match='Not answering: m2'):
        live_preflight(Reader(silent_after_identity=('m2',)), MOTORS, result)
    assert result['unresponsive'] == ['m2'] and 'identity_verified' not in result


def test_a_different_motor_identity_is_fatal_not_a_dropout():
    result = {}
    with pytest.raises(RuntimeError, match='UID mismatch: m2'):
        live_preflight(Reader(wrong_uid='m2'), MOTORS, result)
    assert 'error_code' not in result


def test_complete_preflight_verifies_identity():
    result = {}
    live_preflight(Reader(), MOTORS, result)
    assert result == {'identity_verified': True}


def test_the_scan_reads_each_drives_power_on_range_and_tolerates_firmware_without_it():
    class Ranges(Reader):
        def request(self, motor, kind, parameter=None):
            if parameter == 0x7029:
                if motor['name'] == 'm2':
                    raise TimeoutError('no such parameter')
                return 1
            return super().request(motor, kind, parameter)
    rows = live_scan(Ranges(), MOTORS)
    assert [row['zero_sta'] for row in rows] == [1, 1, None, 1]
    assert [row['position_rad'] for row in rows] == [0] * 4


def test_a_silent_motor_during_a_scan_is_identified():
    with pytest.raises(MotorSilent, match='Missing stopped position: m1') as error:
        live_scan(Reader(silent=('m1',)), MOTORS)
    assert error.value.motor == 'm1' and isinstance(error.value, TimeoutError)
