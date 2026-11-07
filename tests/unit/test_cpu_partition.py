"""The Jetson's CPU split: the loop and its receiver keep to the control CPUs only where the host installed a
partition (tests/unit/test_cpu_partition_install.py), and live task capture moves back off them."""

import json
import os
import sys
from types import SimpleNamespace

import pytest

from robot_platform.control import scheduling
from robot_platform.control.scheduling import (
    cpu_partition,
    head_cpus,
    prefer_background,
    prefer_background_thread,
    prefer_head,
    prefer_responsive,
    thread_placement,
)

ALL = set(range(6))
OTHER = 0  # os.SCHED_OTHER on Linux.


@pytest.fixture
def linux(tmp_path, monkeypatch):
    """A Linux host whose /proc lives in tmp_path; the affinity calls act on a table the test reads back."""
    monkeypatch.setattr(sys, 'platform', 'linux')
    monkeypatch.setattr(scheduling, 'PROC', tmp_path / 'proc')
    monkeypatch.setattr(scheduling, 'PARTITION', str(tmp_path / 'cpu-partition.json'))
    host = SimpleNamespace(cpus={}, started={}, refuse=None, partition=tmp_path / 'cpu-partition.json')

    def thread(pid, tid):
        (tmp_path / 'proc' / str(pid) / 'task' / str(tid)).mkdir(parents=True)
        host.cpus[tid] = set(ALL)

    def setaffinity(tid, cpus):
        if host.refuse:
            raise host.refuse
        if tid not in host.cpus:
            raise ProcessLookupError(tid)
        host.cpus[tid] = set(cpus)
        for pid, new in host.started.pop(tid, ()):  # A thread this one starts as it moves keeps the old CPUs.
            thread(pid, new)

    monkeypatch.setattr(os, 'sched_getaffinity', lambda tid: set(host.cpus[tid]), raising=False)
    monkeypatch.setattr(os, 'sched_setaffinity', setaffinity, raising=False)
    monkeypatch.setattr(os, 'SCHED_OTHER', OTHER, raising=False)
    monkeypatch.setattr(os, 'sched_getscheduler', lambda tid: OTHER, raising=False)
    monkeypatch.setattr(os, 'sched_getparam', lambda tid: SimpleNamespace(sched_priority=0), raising=False)
    host.thread = thread
    return host


def split(path, control=(4, 5), background=(0, 1, 2, 3), head=None):
    path.write_text(
        json.dumps(
            {'control': list(control), 'background': list(background)} | ({} if head is None else {'head': list(head)})
        ),
        encoding='utf-8',
    )


def test_without_an_installed_partition_nothing_moves(linux):
    for tid in (100, 101):
        linux.thread(100, tid)
    assert prefer_responsive(100) == [] and prefer_background(100) == []
    assert linux.cpus == {100: ALL, 101: ALL}


def test_a_control_process_keeps_every_thread_to_the_control_cpus(linux):
    split(linux.partition)
    for tid in (100, 101, 102):
        linux.thread(100, tid)
    linux.started[101] = [(100, 103)]  # Started by a thread not yet moved: the next pass catches it.
    assert prefer_responsive(100) == ['control_cpus']
    assert linux.cpus == {tid: {4, 5} for tid in (100, 101, 102, 103)}
    assert prefer_background(100) == ['background_cpus']
    assert linux.cpus == {tid: {0, 1, 2, 3} for tid in (100, 101, 102, 103)}


def test_a_process_moves_itself_by_default(linux):
    split(linux.partition)
    linux.thread(os.getpid(), os.getpid())
    assert prefer_responsive() == ['control_cpus'] and linux.cpus == {os.getpid(): {4, 5}}


def test_a_refused_move_reports_no_hint(linux):
    split(linux.partition)
    linux.thread(100, 100)
    linux.refuse = PermissionError('another user')
    assert prefer_responsive(100) == [] and prefer_background(100) == []
    assert prefer_responsive(4242) == []  # No such process.


def test_a_thread_of_a_control_process_moves_alone_onto_the_background_cpus(linux):
    # A live head's cycles beside the arms' (helm.head_teleop.HeadLoop): the calling thread (0) moves, its process
    # stays on the control CPUs.
    linux.cpus[0] = set(ALL)
    assert prefer_background_thread() == [] and linux.cpus[0] == ALL  # No partition installed: nothing moves.
    split(linux.partition)
    for tid in (100, 101):
        linux.thread(100, tid)
    assert prefer_responsive(100) == ['control_cpus']
    assert prefer_background_thread() == ['background_cpus']
    assert linux.cpus == {0: {0, 1, 2, 3}, 100: {4, 5}, 101: {4, 5}}
    linux.refuse = PermissionError('refused')
    assert prefer_background_thread() == []


def test_the_heads_receiver_keeps_to_its_own_cpus(linux):
    # 2026-11-03: beside the cameras on 0-3 its 10 ms ticks started up to 15 ms late; one late tick stopped the head.
    for tid in (300, 301):
        linux.thread(300, tid)
    assert prefer_head(300) == []  # No partition: nothing moves.
    split(linux.partition)  # Installed before the head kept a CPU: the background, as before.
    assert prefer_head(300) == ['background_cpus'] and linux.cpus == {300: {0, 1, 2, 3}, 301: {0, 1, 2, 3}}
    split(linux.partition, background=(0, 1, 2), head=(3,))
    linux.started[300] = [(300, 302)]  # Started as it moves: the next pass catches it.
    assert prefer_head(300) == ['head_cpus'] and linux.cpus == {300: {3}, 301: {3}, 302: {3}}
    assert thread_placement(300) == {
        tid: {'cpus': [3], 'policy': 'SCHED_OTHER', 'priority': 0} for tid in (300, 301, 302)
    }
    assert prefer_background(300) == ['background_cpus'] and linux.cpus[300] == {0, 1, 2}
    assert head_cpus(linux.partition) == {3} and thread_placement(4242) == {}
    linux.refuse = PermissionError('refused')
    assert prefer_head(300) == []


@pytest.mark.parametrize(
    'content',
    [
        {'control': [4, 5], 'background': [0, 1, 2], 'head': [2]},
        {'control': [4, 5], 'background': [0, 1, 2], 'head': [5]},
        {'control': [4, 5], 'background': [0, 1, 2], 'head': []},
        {'control': [4, 5], 'background': [0, 1, 2], 'head': '3'},
    ],
)
def test_a_head_cpu_shared_or_malformed_is_an_installation_error(tmp_path, content):
    path = tmp_path / 'cpu-partition.json'
    path.write_text(json.dumps(content), encoding='utf-8')
    assert cpu_partition(path) == ({4, 5}, {0, 1, 2})
    with pytest.raises(ValueError, match='apart from control and background'):
        head_cpus(path)


@pytest.mark.parametrize(
    'content',
    [
        {'control': [4, 5], 'background': [3, 4]},
        {'control': [], 'background': [0, 1]},
        {'control': [4, 5]},
        {'control': ['4'], 'background': [0]},
        {'control': [-1], 'background': [0]},
        {'control': [True], 'background': [0]},
        [[4, 5], [0, 1, 2, 3]],
    ],
)
def test_a_malformed_partition_is_an_installation_error(tmp_path, content):
    path = tmp_path / 'cpu-partition.json'
    path.write_text(json.dumps(content), encoding='utf-8')
    with pytest.raises(ValueError, match='disjoint, non-empty lists'):
        cpu_partition(path)


def test_the_robots_loop_reports_the_cpus_it_keeps_to(monkeypatch):
    """helm.edge.main applies the hints before its first thread; serve records them in the session's view."""
    from robot_platform.helm import edge, teleop, tune

    ran = []
    monkeypatch.setattr(
        edge,
        'prepare',
        lambda start, root, leaders=True: (
            start['bundle'],
            SimpleNamespace(address=('127.0.0.1', 5)) if leaders else None,
        ),
    )
    monkeypatch.setattr(teleop, 'run_loop', lambda commands, views, core, silence_s=None: ran.append(core))
    monkeypatch.setattr(tune, 'run_tune_loop', lambda commands, views, core, silence_s=None: ran.append(core))

    class Core:
        def __init__(self, bundle, open_leaders=None):
            self.scheduling = {}

    sent = []
    views = SimpleNamespace(send=sent.append, close=lambda: None)
    assert edge.serve({'bundle': {}}, None, views, core_factory=Core, scheduling=['control_cpus']) == 0
    assert edge.serve({'bundle': {'kind': 'tune'}}, None, views, core_factory=Core, scheduling=['control_cpus']) == 0
    assert edge.serve({'bundle': {}}, None, views, core_factory=Core) == 0
    assert [core.scheduling for core in ran] == [{'loop': ['control_cpus']}, {'loop': ['control_cpus']}, {}]
    assert sent == [['link', 5], ['link', 0], ['link', 5]]


def test_live_task_capture_moves_off_the_control_cpus_before_it_works(monkeypatch):
    from robot_platform.helm import task_capture_process

    calls = []
    monkeypatch.setattr(scheduling, 'prefer_background', lambda: calls.append('background') or [])
    monkeypatch.setattr(task_capture_process, '_worker', lambda: calls.append('worker'))
    task_capture_process.main()
    assert calls == ['background', 'worker']
