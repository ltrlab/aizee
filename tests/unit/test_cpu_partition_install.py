"""ops.cpu_partition on a host directory: the cameras are confined and checked before the partition file the
loop reads appears, and the file goes first when the split is removed."""
import os
import sys
from types import SimpleNamespace

import pytest

from robot_platform.control.scheduling import cpu_partition
from robot_platform.ops import cpu_partition as installer
from robot_platform.ops.cpu_partition import DROP_IN, SERVICES, Host, cpu_list, drop_in, install, remove, status


class System:
    """systemctl over a host directory: a restart starts a new process placed as the unit's drop-in says,
    as systemd does, unless `honour` is off."""

    def __init__(self, root, honour=True):
        self.root, self.honour, self.calls, self.pids, self.next = root, honour, [], {}, 1000
        for unit in SERVICES:
            self.start(unit)

    def start(self, unit):
        self.next += 1
        pid, drop = self.next, self.root / 'etc/systemd/system' / (unit + '.d') / DROP_IN
        placed = '0-3' if self.honour and drop.exists() else '0-5'
        for tid in (pid, pid + 500):
            task = self.root / 'proc' / str(pid) / 'task' / str(tid)
            task.mkdir(parents=True)
            (task / 'status').write_text(f'Name:\tpython3\nCpus_allowed_list:\t{placed}\n', encoding='utf-8')
        (self.root / 'proc' / str(pid) / 'cmdline').write_bytes(b'/usr/bin/python3\0/home/ltr/camera_node.py\0')
        self.pids[unit] = pid

    def __call__(self, *args):
        self.calls.append(args)
        if args[0] == 'show':
            return 'loaded\n' if args[2] == 'LoadState' else f'{self.pids[args[-1]]}\n'
        if args[0] == 'try-restart':
            # Cameras restart before a partition appears (install) and after it is gone (remove).
            assert not (self.root / 'etc/minerva/cpu-partition.json').exists()
            self.start(args[1])
        return ''


@pytest.fixture
def jetson(tmp_path):
    (tmp_path / 'sys/devices/system/cpu').mkdir(parents=True)
    (tmp_path / 'sys/devices/system/cpu/online').write_text('0-5\n', encoding='utf-8')
    system = System(tmp_path)
    return SimpleNamespace(root=tmp_path, system=system, host=Host(tmp_path, systemctl=system))


def session(root, pid=77, cpus='4-5'):
    task = root / 'proc' / str(pid) / 'task' / str(pid)
    task.mkdir(parents=True)
    (task / 'status').write_text(f'Cpus_allowed_list:\t{cpus}\n', encoding='utf-8')
    (root / 'proc' / str(pid) / 'cmdline').write_bytes(b'python3\0-u\0-m\0robot_platform.helm.edge\0')


def test_install_confines_the_cameras_then_writes_the_partition(jetson):
    report = install(jetson.host)
    for unit in SERVICES:
        assert (jetson.root / 'etc/systemd/system' / (unit + '.d') / DROP_IN).read_text(encoding='utf-8') == (
            '# robot_platform.ops.cpu_partition: this camera keeps off the control CPUs.\n'
            '[Service]\nCPUAffinity=\nCPUAffinity=0 1 2 3\n')
    assert cpu_partition(jetson.root / 'etc/minerva/cpu-partition.json') == ({4, 5}, {0, 1, 2, 3})
    restarts = [args for args in jetson.system.calls if args[0] in ('daemon-reload', 'try-restart')]
    assert restarts == [('daemon-reload',)] + [('try-restart', unit) for unit in SERVICES]
    assert report == {'partition': {'control': [4, 5], 'background': [0, 1, 2, 3]}, 'sessions': {},
                      'cameras': {unit: {'drop_in': True, 'pid': jetson.system.pids[unit], 'cpus': [0, 1, 2, 3]}
                                  for unit in SERVICES}}


def test_install_refuses_during_a_session_offline_cpus_or_a_missing_camera(jetson):
    session(jetson.root)
    with pytest.raises(RuntimeError, match='Helm session is running'):
        install(jetson.host)
    (jetson.root / 'proc/77/cmdline').write_bytes(b'python3\0other\0')
    (jetson.root / 'sys/devices/system/cpu/online').write_text('0-3\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='nvpmodel'):
        install(jetson.host)
    (jetson.root / 'sys/devices/system/cpu/online').write_text('0-5\n', encoding='utf-8')
    host = Host(jetson.root, systemctl=lambda *args: 'not-found\n' if args[2] == 'LoadState' else '')
    with pytest.raises(RuntimeError, match='is not installed'):
        install(host)
    assert not (jetson.root / 'etc').exists()
    assert [args[0] for args in jetson.system.calls] == []


def test_a_camera_left_on_every_cpu_leaves_the_loop_unpinned(tmp_path):
    (tmp_path / 'sys/devices/system/cpu').mkdir(parents=True)
    (tmp_path / 'sys/devices/system/cpu/online').write_text('0-5\n', encoding='utf-8')
    host = Host(tmp_path, systemctl=System(tmp_path, honour=False))
    with pytest.raises(RuntimeError, match='still run outside'):
        install(host)
    assert not (tmp_path / 'etc/minerva/cpu-partition.json').exists()


def test_remove_unpins_the_loop_before_the_cameras_return_everywhere(jetson):
    install(jetson.host)
    session(jetson.root)
    with pytest.raises(RuntimeError, match='Helm session is running'):
        remove(jetson.host)
    (jetson.root / 'proc/77/cmdline').write_bytes(b'python3\0other\0')
    report = remove(jetson.host)
    assert not (jetson.root / 'etc/minerva/cpu-partition.json').exists()
    assert not any((jetson.root / 'etc/systemd/system').iterdir())
    assert report['partition'] is None and all(camera == {'drop_in': False, 'pid': camera['pid'],
                                                          'cpus': [0, 1, 2, 3, 4, 5]}
                                               for camera in report['cameras'].values())


def test_status_reports_where_the_cameras_and_a_session_run(jetson):
    install(jetson.host)
    session(jetson.root, pid=77, cpus='4-5')
    assert status(jetson.host)['sessions'] == {77: [4, 5]}


def test_only_root_changes_the_units(monkeypatch, capsys):
    monkeypatch.setattr(os, 'geteuid', lambda: 1000, raising=False)
    assert installer.main(['install']) == 2 and installer.main(['remove']) == 2
    assert 'run them with sudo' in capsys.readouterr().err
    # As root, only without bytecode: root-owned caches in the release would block pruning it.
    monkeypatch.setattr(os, 'geteuid', lambda: 0, raising=False)
    monkeypatch.setattr(sys, 'flags', SimpleNamespace(dont_write_bytecode=False))
    assert installer.main(['install']) == 2 and 'python3 -B' in capsys.readouterr().err


def test_cpu_lists_read_as_the_kernel_writes_them():
    assert cpu_list('0-3,5\n') == {0, 1, 2, 3, 5} and cpu_list('4') == {4}
    assert drop_in((0, 1)).endswith('CPUAffinity=\nCPUAffinity=0 1\n')
