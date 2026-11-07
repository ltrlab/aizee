"""Cameras dropping off USB, as the kernel announces it (its uevents; Linux only). A camera that drops and
re-enumerates sets off a burst of hotplug work on every CPU. On 2026-11-01 one such burst (the left wrist camera's
cable was loose) stalled a live cycle long enough for the receiver's command to expire, and the session lowered the
arms and ended. The live loop now pauses the arms when a camera goes (helm.teleop) instead.

The loop polls once a cycle, through a non-blocking socket: no thread shares the loop's interpreter."""

import select
import socket

NETLINK_KOBJECT_UEVENT = 15
KERNEL_GROUP = 1  # The kernel's own uevents, not udev's.
VIDEO = '14/'  # USB interface class 14 (video): the UVC wrist cameras and the D435 alike.
MAX_MESSAGES = 256  # Read in one poll at most: a reconnect burst is a few dozen.


def parse(message):
    """A kernel uevent (b'action@devpath\\0KEY=value\\0...') as a dict of its fields, or None."""
    head, *rest = message.split(b'\0')
    if b'@' not in head:
        return None
    fields = {}
    for item in rest:
        key, sep, value = item.partition(b'=')
        if sep:
            fields[key.decode(errors='replace')] = value.decode(errors='replace')
    return fields


def camera_removed(fields):
    """The camera a removal names ('the camera at USB 1-2.2 (05a3:9230)'), or None for any other event."""
    if (
        fields.get('ACTION') != 'remove'
        or fields.get('SUBSYSTEM') != 'usb'
        or fields.get('DEVTYPE') != 'usb_interface'
        or not fields.get('INTERFACE', '').startswith(VIDEO)
    ):
        return None
    port = fields.get('DEVPATH', '').rsplit('/', 1)[-1].split(':', 1)[0] or 'an unknown port'
    vendor, _, rest = fields.get('PRODUCT', '').partition('/')
    try:
        ids = f'{int(vendor, 16):04x}:{int(rest.partition("/")[0], 16):04x}'
    except ValueError:
        return f'the camera at USB {port}'
    return f'the camera at USB {port} ({ids})'


class CameraWatch:
    """The kernel's uevents, read without blocking: poll() names the newest camera to drop off USB since the last
    poll, or returns None."""

    def __init__(self, sock, ready=None):
        self.sock = sock
        self.ready = ready  # select.poll on the socket: an idle cycle costs one syscall, and no exception.

    @classmethod
    def open(cls):
        """A watch, or None where the kernel's uevents cannot be read (not Linux)."""
        family = getattr(socket, 'AF_NETLINK', None)
        if family is None:
            return None
        try:
            sock = socket.socket(family, socket.SOCK_DGRAM, NETLINK_KOBJECT_UEVENT)
        except OSError:
            return None
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)  # A reconnect burst stays queued.
            sock.bind((0, KERNEL_GROUP))
            sock.setblocking(False)
            ready = select.poll()
            ready.register(sock, select.POLLIN)
        except OSError:
            sock.close()
            return None
        return cls(sock, ready)

    def poll(self):
        if self.ready is not None and not self.ready.poll(0):
            return None
        dropped = None
        for _ in range(MAX_MESSAGES):
            try:
                message = self.sock.recv(1 << 16)
            except BlockingIOError:
                break
            except OSError:  # ENOBUFS: the kernel dropped uevents in a burst. Read on next cycle.
                break
            fields = parse(message)
            dropped = (camera_removed(fields) if fields else None) or dropped
        return dropped

    def close(self):
        self.sock.close()
