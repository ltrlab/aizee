"""A worktree can use an explicitly named credential without changing its profile or pins."""
import pytest
from robot_platform.ops.ssh import key_file


def test_profile_key_remains_relative_to_its_own_directory(tmp_path, monkeypatch):
    monkeypatch.delenv('AIZEE_SSH_KEY_FILE', raising=False)
    assert key_file({'key_file': '../keys/robot'}, tmp_path/'profile') == tmp_path/'keys/robot'


def test_explicit_worktree_key_override_does_not_change_profile(tmp_path, monkeypatch):
    selected = tmp_path/'robot-key'
    selected.write_text('test credential placeholder')
    connection = {'key_file': '../keys/robot', 'host_key_sha256': 'pinned-host'}
    monkeypatch.setenv('AIZEE_SSH_KEY_FILE', str(selected))
    assert key_file(connection, tmp_path/'profile') == selected
    assert connection == {'key_file': '../keys/robot', 'host_key_sha256': 'pinned-host'}


@pytest.mark.parametrize('value', ['', 'missing-key'])
def test_bad_override_cannot_silently_fall_back(value, tmp_path, monkeypatch):
    monkeypatch.setenv('AIZEE_SSH_KEY_FILE', value)
    with pytest.raises(FileNotFoundError, match='AIZEE_SSH_KEY_FILE'):
        key_file({'key_file': 'other-key'}, tmp_path)
