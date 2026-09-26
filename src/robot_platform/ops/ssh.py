"""Pinned SSH to a robot computer: host key fingerprint and machine identity checked.

The device profile names the address and machine id; the calibration profile's
connection block names the user, key file (relative to that profile) and host
key fingerprint. Nothing here runs a command with elevated privileges.
"""
import base64
import hashlib
import json
from pathlib import Path
import socket

DEFAULT_PROFILE = Path(__file__).resolve().parents[3] / 'config/devices/minerva-joint-calibration.json'
TETHER = '10.42.0.1'  # USB Ethernet tether; the control path never uses Wi-Fi.


def robot_identity(profile_path=DEFAULT_PROFILE):
    """(device, connection, profile directory) from a calibration profile."""
    profile_path = Path(profile_path).resolve()
    plan = json.loads(profile_path.read_text(encoding='utf-8'))
    device = json.loads((profile_path.parent / plan['device']).read_text(encoding='utf-8'))
    return device, plan['connection'], profile_path.parent


def connect(profile_path=DEFAULT_PROFILE, *, address=TETHER, timeout=8.):
    """An SSH client to the robot computer, refusing any other host or machine."""
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
    client.connect(address or device['address'], username=connection['username'],
                   key_filename=str((directory / connection['key_file']).resolve()),
                   look_for_keys=False, allow_agent=False, timeout=timeout, auth_timeout=timeout,
                   banner_timeout=timeout)
    try:
        with client.open_sftp() as sftp, sftp.open('/etc/machine-id', 'rb') as handle:
            if handle.read().strip().decode() != device['machine_id']:
                raise RuntimeError('Wrong robot computer: machine id differs from the device profile')
    except BaseException:
        client.close()
        raise
    return client


def run(client, command, *, timeout=60., stdin=None):
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
