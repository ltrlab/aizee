"""Cameras dropping off USB from the kernel's uevents (helm.camera_watch), and when a late cycle's trip pauses the
arms rather than lowering them (helm.teleop)."""
from collections import deque
import socket

from robot_platform.helm.camera_watch import MAX_MESSAGES, CameraWatch, camera_removed, parse
from robot_platform.helm.teleop import CAMERA_PAUSE_S, PAUSE_ATTEMPTS, TeleopCore

LEFT = '/devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.2/1-2.2:1.0'
RIGHT = '/devices/platform/bus@0/3610000.usb/usb1/1-2/1-2.3/1-2.3.2/1-2.3.2:1.1'


def uevent(action, devpath, **fields):
    lines = [f'{action}@{devpath}', f'ACTION={action}', f'DEVPATH={devpath}', *(f'{k}={v}' for k, v in fields.items())]
    return '\0'.join(lines).encode() + b'\0'


def camera(action, devpath, product='5a3/9230/100', interface='14/1/0'):
    return uevent(action, devpath, SUBSYSTEM='usb', DEVTYPE='usb_interface', PRODUCT=product, INTERFACE=interface)


class Socket:
    def __init__(self, messages):
        self.messages = list(messages)

    def recv(self, size):
        if not self.messages:
            raise BlockingIOError
        return self.messages.pop(0)


def test_a_camera_interface_removal_names_the_camera():
    assert camera_removed(parse(camera('remove', LEFT))) == 'the camera at USB 1-2.2 (05a3:9230)'
    assert camera_removed(parse(camera('remove', RIGHT, '32e4/9230/100', '14/2/0'))) == \
        'the camera at USB 1-2.3.2 (32e4:9230)'


def test_other_uevents_are_not_camera_drops():
    assert camera_removed(parse(camera('add', LEFT))) is None
    # A CAN adapter's interface is vendor specific (class 255), not video.
    can = LEFT.replace('1-2.2', '1-2.1')
    assert camera_removed(parse(camera('remove', can, '1d50/606f/0', '255/255/255'))) is None
    assert camera_removed(parse(uevent('remove', '/devices/virtual/video4linux/video0', SUBSYSTEM='video4linux'))) is None
    assert parse(b'libudev\0\xfe\xed') is None


def test_a_poll_reads_what_waits_and_names_the_newest_camera():
    watch = CameraWatch(Socket([camera('remove', LEFT), camera('remove', RIGHT, '32e4/9230/100'),
                                camera('add', LEFT)]))
    assert watch.poll() == 'the camera at USB 1-2.3.2 (32e4:9230)'
    assert watch.poll() is None


def test_a_burst_longer_than_one_poll_is_read_on_over_the_next():
    sock = Socket([camera('add', LEFT)] * MAX_MESSAGES + [camera('remove', LEFT)])
    watch = CameraWatch(sock)
    assert watch.poll() is None and len(sock.messages) == 1
    assert watch.poll() == 'the camera at USB 1-2.2 (05a3:9230)'


def test_without_netlink_there_is_no_watch(monkeypatch):
    monkeypatch.delattr(socket, 'AF_NETLINK', raising=False)
    assert CameraWatch.open() is None


def test_a_late_cycle_pauses_only_soon_after_a_camera_dropped_and_a_few_times_a_minute():
    core = object.__new__(TeleopCore)  # Without __init__: no camera has dropped.
    assert not core._camera_pause_due(100.)
    core._camera_dropped_at, core._pauses = 95., deque(maxlen=PAUSE_ATTEMPTS)
    assert core._camera_pause_due(100.)
    assert not core._camera_pause_due(95. + CAMERA_PAUSE_S + .1)
    core._pauses.extend([60., 80., 99.])
    assert not core._camera_pause_due(100.)  # PAUSE_ATTEMPTS in the last minute: lowered as any other fault.
    core._camera_dropped_at = 118.
    assert core._camera_pause_due(121.)  # The first of them is over a minute old.
