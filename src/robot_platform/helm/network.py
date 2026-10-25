"""Minerva's network as Helm shows it, and its access point switch (robot_platform.ops.network runs them).

The network service writes its state (STATE) for anyone to read; switching the access point goes through the
service's program under sudo, which allows exactly `ap on` and `ap off` to the robot's user. Helm on a PC has no
such state: the section then says the service is not installed here.
"""
import json
from pathlib import Path
import subprocess

STATE = Path('/run/minerva-net/state.json')
PROGRAM = '/usr/local/sbin/minerva-net'
STALE_S = 10.  # The service writes every second: older than this, it is not running.


def path_of(peer, *, run=subprocess.run, interfaces=Path('/sys/class/net')):
    """How this computer reaches `peer`: 'wifi', 'wired' or 'local' (by the interface its route leaves on), or
    None where that cannot be told."""
    if not peer:
        return None
    try:
        out = run(['ip', '-o', 'route', 'get', peer], capture_output=True, text=True, timeout=2.).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return None
    if 'dev' not in out or out.index('dev') + 1 >= len(out):
        return None
    device = out[out.index('dev') + 1]
    if device == 'lo':
        return 'local'
    return 'wifi' if (interfaces / device / 'wireless').exists() or (interfaces / device / 'phy80211').exists() \
        else 'wired'


class NetworkView:
    """The network service's state for Helm, and the access point switch."""

    def __init__(self, state=STATE, *, run=subprocess.run, clock=None):
        import time
        self.state, self.run, self.clock = Path(state), run, clock or time.time

    def snapshot(self):
        try:
            state = json.loads(self.state.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return {'installed': False}
        updated = state.get('updated_at')
        age = self.clock() - updated if type(updated) in (int, float) else None
        return dict(state, installed=True, age_s=None if age is None else round(age, 1),
                    running=age is not None and age <= STALE_S)

    def addresses(self):
        """Minerva's addresses, the best path first, for the bridge to try next time: the wired ones, the
        tether/access point address, then Wi-Fi."""
        state = self.snapshot()
        if not state.get('installed'):
            return []
        ordered = [a.split('/')[0] for a in (state.get('ethernet') or {}).get('addresses') or []]
        ordered.append(state.get('local'))
        ordered += [a.split('/')[0] for a in (state.get('wifi') or {}).get('addresses') or []]
        return [a for a in dict.fromkeys(ordered) if a]

    def set_ap(self, on, timeout=60.):
        """Switch the access point and wait for it (the service's program, through sudo): the state it reached."""
        result = self.run(['sudo', '-n', PROGRAM, 'ap', 'on' if on else 'off'], capture_output=True, text=True,
                          timeout=timeout)
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()[-300:]
            raise ValueError(f"Could not switch the access point {'on' if on else 'off'}: "
                             f"{detail or f'exit {result.returncode}'}")
        return self.snapshot()
