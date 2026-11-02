"""Minerva's network as Helm shows it, and its access point switch (robot_platform.ops.network runs them).

The network service writes its state (STATE) for anyone to read; switching the access point goes through the
service's program under sudo, which allows exactly `ap on` and `ap off` to the robot's user. Helm on a PC has no
such state: the section then says the service is not installed here.

The headset opens Helm's /xr page on the network (the Quest on the same Wi-Fi as Minerva, at Minerva's wired address,
or its Wi-Fi client's when unplugged) or on Minerva's access point (the Quest on `minerva`, at Minerva's local
address, which answers in every mode). `links` gives both, which is live, and whether Helm's certificate names each
address: a browser refuses a self-signed certificate for an address it does not name.
"""
import json
from pathlib import Path
import re
import subprocess

STATE = Path('/run/minerva-net/state.json')
PROGRAM = '/usr/local/sbin/minerva-net'
STALE_S = 10.  # The service writes every second: older than this, it is not running.
ALT_NAME = re.compile(r'(?:IP Address|DNS):\s*([^,\s]+)')


def alt_names(text):
    """The addresses and names in OpenSSL's text for a certificate's subjectAltName (`openssl x509 -noout -ext
    subjectAltName`), in order."""
    return ALT_NAME.findall(text or '')


def certificate_names(path, *, run=subprocess.run):
    """The addresses and names the certificate at `path` names, or None where they cannot be read."""
    try:
        result = run(['openssl', 'x509', '-in', str(path), '-noout', '-ext', 'subjectAltName'], capture_output=True,
                     text=True, timeout=5.)
    except (OSError, subprocess.SubprocessError):
        return None
    return alt_names(result.stdout) if result.returncode == 0 else None


def network_address(state):
    """Minerva's address on the network it is plugged into or joined (wired first), or None (a tether, the access
    point only, or nothing)."""
    ethernet, wifi = state.get('ethernet') or {}, state.get('wifi') or {}
    if state.get('mode') == 'lan' and ethernet.get('addresses'):
        return ethernet['addresses'][0].split('/')[0]
    if wifi.get('mode') == 'client' and wifi.get('addresses'):
        return wifi['addresses'][0].split('/')[0]
    return None


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

    def __init__(self, state=STATE, *, run=subprocess.run, clock=None, certificate=None, port=None):
        """`certificate`: Helm's HTTPS certificate, `port` its HTTPS port: for the headset's links."""
        import time
        self.state, self.run, self.clock = Path(state), run, clock or time.time
        self.certificate, self.port = certificate, port
        self._names = None  # The certificate's names, read once.

    def links(self, state=None):
        """The headset's /xr links: [{mode: 'network' or 'access_point', address, url, live, certified}], `live`
        when Minerva has that address now (the access point's while it is on), `certified` whether Helm's
        certificate names it (None: no certificate, or it could not be read). [] without the service or a port.
        `state`: a snapshot just taken, to read the service's state once."""
        state = self.snapshot() if state is None else state
        if not state.get('installed') or not self.port:
            return []
        if self.certificate is not None and self._names is None:
            self._names = certificate_names(self.certificate, run=self.run) or []
        ap = state.get('ap') or {}
        links = []
        for mode, address, live in (('network', network_address(state), True),
                                    ('access_point', state.get('local'), bool(ap.get('on')))):
            if address:
                links.append({'mode': mode, 'address': address, 'url': f'https://{address}:{self.port}/xr',
                              'live': live, 'certified': (address in self._names) if self._names else None})
        return links

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
