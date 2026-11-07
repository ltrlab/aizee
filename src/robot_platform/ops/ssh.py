"""Pinned SSH to a robot computer: host key fingerprint and machine identity checked.

The device profile names the address and machine id; the calibration profile's
connection block names the user, key file (relative to that profile) and host
key fingerprint. Nothing here runs a command with elevated privileges.
"""

import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import threading

DEFAULT_PROFILE = Path(__file__).resolve().parents[3] / 'config/devices/minerva-joint-calibration.json'
TETHER = '10.42.0.1'  # USB Ethernet tether; the control path never uses Wi-Fi.


def key_file(connection, directory):
    """Explicit per-process key override for isolated worktrees; never discover or copy keys."""
    override = os.environ.get('AIZEE_SSH_KEY_FILE')
    if override is not None:
        path = Path(override).expanduser().resolve()
        if not override.strip() or not path.is_file():
            raise FileNotFoundError('AIZEE_SSH_KEY_FILE must name an existing private key file')
        return path
    return (Path(directory) / connection['key_file']).resolve()


def robot_identity(profile_path=DEFAULT_PROFILE):
    """(device, connection, profile directory) from a calibration profile."""
    profile_path = Path(profile_path).resolve()
    plan = json.loads(profile_path.read_text(encoding='utf-8'))
    device = json.loads((profile_path.parent / plan['device']).read_text(encoding='utf-8'))
    return device, plan['connection'], profile_path.parent


def pinned(profile_path=DEFAULT_PROFILE, *, address=TETHER, timeout=8.0, key=None):
    """An SSH client to the robot computer whose host key is the profile's (`key`: a private key file instead of the
    profile's). connect() also checks the machine id; a key restricted to one command (the GELLO bridge's relay)
    cannot, so its caller checks it another way (helm.bridge's welcome)."""
    import paramiko

    device, connection, directory = robot_identity(profile_path)
    expected = connection['host_key_sha256']

    class PinnedHost(paramiko.MissingHostKeyPolicy):
        def missing_host_key(self, client, hostname, key):
            actual = 'SHA256:' + base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
            if actual != expected:
                raise paramiko.SSHException('SSH host fingerprint mismatch')
            client.get_host_keys().add(hostname, key.get_name(), key)

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(PinnedHost())
    client.connect(
        address or device['address'],
        username=connection['username'],
        key_filename=str(Path(key).expanduser().resolve() if key else key_file(connection, directory)),
        look_for_keys=False,
        allow_agent=False,
        timeout=timeout,
        auth_timeout=timeout,
        banner_timeout=timeout,
    )
    return client


def connect(profile_path=DEFAULT_PROFILE, *, address=TETHER, timeout=8.0):
    """An SSH client to the robot computer, refusing any other host or machine."""
    device, _connection, _directory = robot_identity(profile_path)
    client = pinned(profile_path, address=address, timeout=timeout)
    expired = threading.Event()

    def identity_expired():
        expired.set()
        client.close()  # Also interrupts subsystem negotiation before SFTP exposes its channel.

    watchdog = threading.Timer(timeout, identity_expired)
    watchdog.daemon = True
    watchdog.start()
    try:
        # Bound identity verification too: TCP/auth timeouts do not cover a
        # stalled SFTP channel open or machine-id read on an established link.
        client.get_transport().channel_timeout = timeout
        with client.open_sftp() as sftp:
            sftp.get_channel().settimeout(timeout)
            with sftp.open('/etc/machine-id', 'rb') as handle:
                if handle.read(256).strip().decode() != device['machine_id']:
                    raise RuntimeError('Wrong robot computer: machine id differs from the device profile')
    except BaseException as exc:
        client.close()
        if expired.is_set():
            raise TimeoutError('SSH robot identity verification timed out') from exc
        raise
    finally:
        watchdog.cancel()
        watchdog.join()  # No late timer may close an accepted client after return.
    if expired.is_set():
        raise TimeoutError('SSH robot identity verification timed out')
    return client


def open_command(client, command, *, timeout=3.0):
    """Bound exec-request acknowledgment as well as channel-open/I/O timeouts.

    Paramiko's Channel._wait_for_event does not use its socket timeout. Closing
    this dedicated client interrupts a stalled acknowledgment before it returns
    a channel. Only camera helpers use this wrapper; control transports are unchanged.
    """
    expired = threading.Event()

    def expire():
        expired.set()
        client.close()

    watchdog = threading.Timer(timeout, expire)
    watchdog.daemon = True
    watchdog.start()
    try:
        result = client.exec_command(command, timeout=timeout)
    except BaseException as exc:
        if expired.is_set():
            raise TimeoutError('SSH command startup timed out') from exc
        raise
    finally:
        watchdog.cancel()
        watchdog.join()
    if expired.is_set():
        raise TimeoutError('SSH command startup timed out')
    return result


def run(client, command, *, timeout=60.0, stdin=None):
    """(exit status, stdout, stderr) of one remote command."""
    channel_in, out, err = client.exec_command(command, timeout=timeout)
    if stdin is not None:
        channel_in.write(stdin)
    channel_in.channel.shutdown_write()
    stdout, stderr = out.read().decode(errors='replace'), err.read().decode(errors='replace')
    return out.channel.recv_exit_status(), stdout, stderr


def route_source(address):
    """The local address this PC would use to reach `address`, or None (no packet is sent)."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((address, 9))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()
