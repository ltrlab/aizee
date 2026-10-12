"""The leader setup script's frames match firmware 2's INFO and RETURN_DELAY layouts."""
import importlib.util
from pathlib import Path
import struct

import pytest

spec = importlib.util.spec_from_file_location(
    'openrb_leader_setup', Path(__file__).resolve().parents[2] / 'scripts/openrb_leader_setup.py')
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


def sealed(body):
    return bytes(body) + bytes((setup.crc8(body),))


def test_info_reports_each_servo_and_the_read_statistics():
    servos = [value for i in range(7) for value in (i + 1, 46, 250 if i else 0xFF, 0)]
    frame = sealed(bytes((0x49, 2, 7, *servos)) + struct.pack('<4I', 12000, 3, 1310, 4870))
    info = setup.decode_info(frame)
    assert info['firmware'] == 2 and info['reads'] == 12000 and info['failed_reads'] == 3
    assert info['last_read_us'] == 1310 and info['max_read_us'] == 4870
    assert info['servos'][0] == dict(id=1, firmware=46, return_delay=None, torque=0)  # 0xFF: did not answer.
    assert info['servos'][6] == dict(id=7, firmware=46, return_delay=250, torque=0)


def test_return_delay_request_and_reply():
    assert setup.return_delay_request(0) == bytes((0x44, 0, setup.crc8((0x44, 0))))
    for bad in (-1, 255, 1.5, True):
        with pytest.raises(ValueError):
            setup.return_delay_request(bad)
    rows = setup.decode_return_delay(sealed(bytes((0x44, 7, *[v for i in range(7) for v in (i + 1, 250, 0)]))))
    assert rows[3] == dict(id=4, before=250, after=0)


@pytest.mark.parametrize('frame', [b'', bytes(48), sealed(bytes((0x49, 2, 6)) + bytes(44))])
def test_a_missing_or_foreign_reply_is_refused(frame):
    with pytest.raises(ValueError):
        setup.decode_info(frame)
