"""Best-effort OS scheduling hints for fixed-rate control loops.

Windows may treat a windowless process as background work: EcoQoS on the
efficiency cores and coalesced timers. The guarded receiver's collision verdict
lasts two 25 Hz periods, so one such stall between cycles stops the receiver.
The loop, its receiver process and the leader link server opt out explicitly,
and a process applying the hints to itself also asks for 1 ms timer resolution.
Nothing here needs elevation; a failed hint leaves the operating-system default
in place.
"""
import os
import sys

HIGH_PRIORITY_CLASS = 0x80
PROCESS_SET_INFORMATION = 0x0200
PROCESS_POWER_THROTTLING = 4  # PROCESS_INFORMATION_CLASS.ProcessPowerThrottling
THROTTLE_EXECUTION_SPEED, THROTTLE_IGNORE_TIMER_RESOLUTION = 0x1, 0x4


def prefer_responsive(pid=None):
    """Apply the hints to process `pid` (default: this one); return those that took."""
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
