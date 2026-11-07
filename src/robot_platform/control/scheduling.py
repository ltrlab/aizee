"""Best-effort OS scheduling hints for fixed-rate control loops.

Windows may treat a windowless process as background work: EcoQoS on the
efficiency cores and coalesced timers. The guarded receiver's collision verdict
lasts two 25 Hz periods, so one such stall between cycles stops the receiver.
The loop, its receiver process and the leader link server opt out explicitly,
and a process applying the hints to itself also asks for 1 ms timer resolution.
Nothing here needs elevation; a failed hint leaves the operating-system default
in place.

On Minerva's Jetson the camera services shared all six cores with the loop and
its receiver, and their load stretched the loop's cycles until a command
expired (2026-10-17). A host with a CPU partition installed (PARTITION, written
by ops.cpu_partition with the camera services' CPUAffinity) keeps every thread
of a control process on the partition's control CPUs, and prefer_background
moves camera-side work, such as live task capture, onto the others;
prefer_background_thread moves one thread of a control process there (a live
head's cycles beside the arms', whose receiver otherwise started its ticks up to
9 ms late and let an arm command expire, 2026-10-31). Without the file nothing
moves: control CPUs only help where the cameras keep off them.

A partition may also keep CPUs for the head's receiver alone (head_cpus), and
prefer_head moves it there: on the background CPUs beside the cameras its 10 ms
ticks started up to 15 ms late, and one late tick stopped the head mid-session
(2026-11-03). It runs there at normal priority. Real-time (SCHED_FIFO) would
need the robot user's services to hold an RLIMIT_RTPRIO, which Minerva's
systemd-user PAM session drops; on a CPU of its own, under a load filling the
background CPUs, the receiver missed no deadline in 30 s.
"""

import json
import os
from pathlib import Path
import sys

HIGH_PRIORITY_CLASS = 0x80
PROCESS_SET_INFORMATION = 0x0200
PROCESS_POWER_THROTTLING = 4  # PROCESS_INFORMATION_CLASS.ProcessPowerThrottling
THROTTLE_EXECUTION_SPEED, THROTTLE_IGNORE_TIMER_RESOLUTION = 0x1, 0x4
PARTITION = '/etc/minerva/cpu-partition.json'
PROC = Path('/proc')


def _split(path):
    """The partition file at `path` (default: PARTITION) and what it holds, None where there is none."""
    path = Path(path or PARTITION)
    try:
        return path, json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return path, None


def _cpu_numbers(cpus):
    return isinstance(cpus, list) and bool(cpus) and all(type(cpu) is int and cpu >= 0 for cpu in cpus)


def cpu_partition(path=None):
    """The host's installed CPU split as (control, background) sets, or None where there is none."""
    path, split = _split(path)
    if split is None:
        return None
    control, background = (split.get(key) if isinstance(split, dict) else None for key in ('control', 'background'))
    if not all(_cpu_numbers(cpus) for cpus in (control, background)) or set(control) & set(background):
        raise ValueError(f'{path}: control and background must be disjoint, non-empty lists of CPU numbers')
    return frozenset(control), frozenset(background)


def head_cpus(path=None):
    """The CPUs the installed partition keeps for the head's receiver alone, or None where it keeps none (no
    partition, or one installed before 2026-11-03)."""
    path, split = _split(path)
    partition = cpu_partition(path)
    if partition is None or split.get('head') is None:
        return None
    head = split['head']
    if not _cpu_numbers(head) or set(head) & (partition[0] | partition[1]):
        raise ValueError(f'{path}: head must be a non-empty list of CPU numbers apart from control and background')
    return frozenset(head)


def pin_threads(cpus, pid=None):
    """Move every thread of process `pid` (default: this one) onto `cpus`; False if the system refused.

    Threads started later inherit their creator's CPUs, and so do child processes such as a receiver."""
    tasks = PROC / str(pid or os.getpid()) / 'task'
    for _ in range(5):  # A thread started during a pass by one not yet moved is caught by the next.
        moved = False
        try:
            threads = [int(task.name) for task in tasks.iterdir()]
        except FileNotFoundError:
            return False
        for thread in threads:
            try:
                if os.sched_getaffinity(thread) != cpus:
                    os.sched_setaffinity(thread, cpus)
                    moved = True
            except ProcessLookupError:
                continue  # That thread has ended.
            except OSError:
                return False
        if not moved:
            return True
    return False


def prefer_background(pid=None):
    """Keep work that must not delay a control loop (camera decoding, episode writing) off its CPUs."""
    partition = cpu_partition() if sys.platform.startswith('linux') else None
    return ['background_cpus'] if partition and pin_threads(partition[1], pid) else []


def prefer_background_thread():
    """Move the calling thread alone onto the background CPUs: work inside a control process that must not delay its
    loop (a live head's cycles beside the arms', helm.head_teleop). Threads it starts later go there too."""
    partition = cpu_partition() if sys.platform.startswith('linux') else None
    if not partition:
        return []
    try:
        os.sched_setaffinity(0, partition[1])  # 0: the calling thread, not its process.
    except OSError:
        return []
    return ['background_cpus']


def prefer_head(pid):
    """The head's receiver (neck.head.HeadReceiver), process `pid`: every thread onto the partition's head CPUs, or
    where it keeps none, onto the background CPUs as before. Returns the hints that took."""
    partition = cpu_partition() if sys.platform.startswith('linux') else None
    if not partition:
        return []
    head = head_cpus()
    if head is None:
        return ['background_cpus'] if pin_threads(partition[1], pid) else []
    return ['head_cpus'] if pin_threads(head, pid) else []


def thread_placement(pid):
    """Where each thread of process `pid` runs: {thread: {'cpus', 'policy', 'priority'}}, for the evidence beside
    a receiver; empty where it cannot be read (not Linux, or the process has ended)."""
    if not sys.platform.startswith('linux'):
        return {}
    policies = {
        getattr(os, name): name
        for name in ('SCHED_OTHER', 'SCHED_FIFO', 'SCHED_RR', 'SCHED_BATCH', 'SCHED_IDLE')
        if hasattr(os, name)
    }
    threads = {}
    try:
        tasks = [int(task.name) for task in (PROC / str(pid) / 'task').iterdir()]
    except FileNotFoundError:
        return {}
    for thread in tasks:
        try:
            threads[thread] = {
                'cpus': sorted(os.sched_getaffinity(thread)),
                'policy': policies.get(os.sched_getscheduler(thread), 'unknown'),
                'priority': os.sched_getparam(thread).sched_priority,
            }
        except OSError:
            continue  # That thread has ended.
    return threads


def prefer_responsive(pid=None):
    """Apply the hints to process `pid` (default: this one); return those that took."""
    if sys.platform.startswith('linux'):
        partition = cpu_partition()
        return ['control_cpus'] if partition and pin_threads(partition[0], pid) else []
    if sys.platform != 'win32':
        return []
    import ctypes
    from ctypes import wintypes

    class ThrottlingState(ctypes.Structure):
        _fields_ = [('Version', wintypes.ULONG), ('ControlMask', wintypes.ULONG), ('StateMask', wintypes.ULONG)]

    kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel32.GetCurrentProcess.restype = wintypes.HANDLE
    kernel32.OpenProcess.restype = wintypes.HANDLE
    kernel32.OpenProcess.argtypes = (wintypes.DWORD, wintypes.BOOL, wintypes.DWORD)
    kernel32.SetPriorityClass.argtypes = (wintypes.HANDLE, wintypes.DWORD)
    kernel32.SetProcessInformation.argtypes = (wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD)
    kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
    own = pid is None or pid == os.getpid()
    handle = kernel32.GetCurrentProcess() if own else kernel32.OpenProcess(PROCESS_SET_INFORMATION, False, pid)
    if not handle:
        return []
    applied = []
    try:
        # StateMask 0 = never throttle. Windows 10 lacks the timer flag; retry without it.
        for control in (THROTTLE_EXECUTION_SPEED | THROTTLE_IGNORE_TIMER_RESOLUTION, THROTTLE_EXECUTION_SPEED):
            state = ThrottlingState(1, control, 0)
            if kernel32.SetProcessInformation(
                handle, PROCESS_POWER_THROTTLING, ctypes.byref(state), ctypes.sizeof(state)
            ):
                applied.append('no_power_throttling')
                break
        if kernel32.SetPriorityClass(handle, HIGH_PRIORITY_CLASS):
            applied.append('high_priority')
        # Timed waits otherwise round up to the ~15.6 ms system tick: the 100 Hz leader
        # sampler ran at 30-60 Hz and served samples up to ~30 ms old (p99 29 ms; 21 ms
        # with this). The resolution is per process and ends with it.
        if own and ctypes.WinDLL('winmm').timeBeginPeriod(1) == 0:
            applied.append('timer_1ms')
    finally:
        if not own:
            kernel32.CloseHandle(handle)
    return applied
