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
moves camera-side work, such as live task capture, onto the others. Without the
file nothing moves: control CPUs only help where the cameras keep off them.
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


def cpu_partition(path=None):
    """The host's installed CPU split as (control, background) sets, or None where there is none."""
    path = Path(path or PARTITION)
    try:
        split = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return None
    control, background = (split.get(key) if isinstance(split, dict) else None for key in ('control', 'background'))
    if not all(isinstance(cpus, list) and cpus and all(type(cpu) is int and cpu >= 0 for cpu in cpus)
               for cpus in (control, background)) or set(control) & set(background):
        raise ValueError(f'{path}: control and background must be disjoint, non-empty lists of CPU numbers')
    return frozenset(control), frozenset(background)


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
        _fields_ = [('Version', wintypes.ULONG), ('ControlMask', wintypes.ULONG),
                    ('StateMask', wintypes.ULONG)]

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
            if kernel32.SetProcessInformation(handle, PROCESS_POWER_THROTTLING, ctypes.byref(state),
                                              ctypes.sizeof(state)):
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
