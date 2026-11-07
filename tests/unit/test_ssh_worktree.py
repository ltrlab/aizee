"""A worktree can use an explicitly named credential without changing its profile or pins."""

import pytest
from robot_platform.ops.ssh import key_file


def test_profile_key_remains_relative_to_its_own_directory(tmp_path, monkeypatch):
    monkeypatch.delenv('AIZEE_SSH_KEY_FILE', raising=False)
    assert key_file({'key_file': '../keys/robot'}, tmp_path / 'profile') == tmp_path / 'keys/robot'


def test_explicit_worktree_key_override_does_not_change_profile(tmp_path, monkeypatch):
    selected = tmp_path / 'robot-key'
    selected.write_text('test credential placeholder')
    connection = {'key_file': '../keys/robot', 'host_key_sha256': 'pinned-host'}
    monkeypatch.setenv('AIZEE_SSH_KEY_FILE', str(selected))
    assert key_file(connection, tmp_path / 'profile') == selected
    assert connection == {'key_file': '../keys/robot', 'host_key_sha256': 'pinned-host'}


@pytest.mark.parametrize('value', ['', 'missing-key'])
def test_bad_override_cannot_silently_fall_back(value, tmp_path, monkeypatch):
    monkeypatch.setenv('AIZEE_SSH_KEY_FILE', value)
    with pytest.raises(FileNotFoundError, match='AIZEE_SSH_KEY_FILE'):
        key_file({'key_file': 'other-key'}, tmp_path)


@pytest.mark.parametrize('failure', [None, 'open', 'read', 'identity'])
def test_identity_sftp_open_and_read_have_timeout_and_failure_closes_client(monkeypatch, tmp_path, failure):
    from contextlib import nullcontext
    from types import SimpleNamespace
    import paramiko
    from robot_platform.ops import ssh

    calls = []
    transport = SimpleNamespace(channel_timeout=None)
    channel = SimpleNamespace(settimeout=lambda value: calls.append(('timeout', value)))

    class Handle:
        def read(self, count):
            assert calls[-1] == ('timeout', 3.0) and count == 256
            if failure == 'read':
                raise TimeoutError('machine-id read timed out')
            return b'other-machine' if failure == 'identity' else b'expected-machine\n'

    sftp = SimpleNamespace(get_channel=lambda: channel, open=lambda *a: nullcontext(Handle()))

    class Client:
        closed = False

        def set_missing_host_key_policy(self, policy):
            pass

        def connect(self, *a, **kw):
            assert kw['timeout'] == kw['auth_timeout'] == kw['banner_timeout'] == 3.0
            assert kw['look_for_keys'] is kw['allow_agent'] is False

        def get_transport(self):
            return transport

        def open_sftp(self):
            assert transport.channel_timeout == 3.0
            if failure == 'open':
                raise TimeoutError('SFTP open timed out')
            return nullcontext(sftp)

        def close(self):
            self.closed = True

    client = Client()
    monkeypatch.setattr(paramiko, 'SSHClient', lambda: client)
    monkeypatch.setattr(
        ssh,
        'robot_identity',
        lambda _: (
            {'machine_id': 'expected-machine'},
            {'host_key_sha256': 'pinned', 'username': 'operator', 'key_file': 'key'},
            tmp_path,
        ),
    )
    monkeypatch.delenv('AIZEE_SSH_KEY_FILE', raising=False)
    if failure:
        with pytest.raises((TimeoutError, RuntimeError)):
            ssh.connect(tmp_path / 'profile.json', timeout=3.0)
        assert client.closed
    else:
        assert ssh.connect(tmp_path / 'profile.json', timeout=3.0) is client
        assert not client.closed


def test_stalled_sftp_subsystem_negotiation_is_interrupted_by_identity_watchdog(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import threading
    import time
    import paramiko
    from robot_platform.ops import ssh

    closed = threading.Event()
    transport = SimpleNamespace(channel_timeout=None)

    class Client:
        def set_missing_host_key_policy(self, policy):
            pass

        def connect(self, *a, **kw):
            pass

        def get_transport(self):
            return transport

        def open_sftp(self):
            assert closed.wait(1), 'Watchdog did not close stalled subsystem negotiation'
            raise OSError('Transport closed during SFTP handshake')

        def close(self):
            closed.set()

    monkeypatch.setattr(paramiko, 'SSHClient', Client)
    monkeypatch.setattr(
        ssh,
        'robot_identity',
        lambda _: (
            {'machine_id': 'fixture'},
            {'host_key_sha256': 'pin', 'username': 'operator', 'key_file': 'key'},
            tmp_path,
        ),
    )
    monkeypatch.delenv('AIZEE_SSH_KEY_FILE', raising=False)
    before = time.monotonic()
    with pytest.raises(TimeoutError, match='identity verification timed out'):
        ssh.connect(tmp_path / 'profile', timeout=0.03)
    assert closed.is_set() and time.monotonic() - before < 0.5


def test_camera_exec_startup_ack_is_bounded(monkeypatch):
    import threading
    from robot_platform.ops.ssh import open_command

    closed = threading.Event()

    class Client:
        def exec_command(self, *a, **k):
            assert closed.wait(1), 'exec acknowledgment watchdog did not run'
            raise OSError('transport closed before exec acknowledgment')

        def close(self):
            closed.set()

    with pytest.raises(TimeoutError, match='command startup timed out'):
        open_command(Client(), 'python3 -', timeout=0.03)
    assert closed.is_set()
