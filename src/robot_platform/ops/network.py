"""Minerva's network: the Ethernet port finds its own mode, and a Wi-Fi access point can be switched on.

On the Jetson, once, from the active release (it copies itself to /usr/local/lib/minerva-net, asks for the access
point's password on the terminal, and starts the system service minerva-net):
    cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.network install
`remove` undoes it; `status` needs no sudo; `ap on` and `ap off` switch the access point (Helm runs them through
sudo, which allows exactly those two to the robot's user).

The modes:
- Ethernet plugged into a network: its DHCP server answers within DHCP_TIMEOUT_S and Minerva takes the address
  (wired LAN). Minerva's Wi-Fi client then steps aside: one path on the network at a time, so aizee-rover.local and
  the bridge find the wired address, and no address answers on two interfaces of one subnet.
- Ethernet plugged into a PC's USB Ethernet adapter: nothing answers, so the port takes 10.42.0.1/24 and hands the
  PC an address (tether). Should more than TETHER_CLIENTS_MAX devices take one, the cable is on a network after
  all (its DHCP server was slow to answer): the port stops serving and asks for an address again.
- Ethernet unplugged: the Wi-Fi client comes back.
- The access point `minerva` (5 GHz, WPA2; its clients get 10.42.1.x) takes the Wi-Fi radio while it is on, and
  stays on across restarts until switched off.
Minerva also always answers at 10.42.0.1 (a dummy interface), so a PC on the tether and a headset on the access
point both reach it there. State for Helm: /run/minerva-net/state.json.

Standard library only: the installed copy runs without a release (python3 -I).
"""
import argparse
import getpass
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

VERSION = 1
UNIT = 'minerva-net.service'
UNITS = PurePosixPath('/etc/systemd/system')
LIB = PurePosixPath('/usr/local/lib/minerva-net')
PROGRAM = PurePosixPath('/usr/local/sbin/minerva-net')
SUDOERS = PurePosixPath('/etc/sudoers.d/minerva-net')
CONNECTIONS = PurePosixPath('/etc/NetworkManager/system-connections')
NM_CONF = PurePosixPath('/etc/NetworkManager/conf.d/90-minerva-net.conf')
STATE = '/run/minerva-net/state.json'
SETTINGS = '/var/lib/minerva-net/settings.json'
USER = 'ltr'  # The robot's user: Helm runs as it.

ETHERNET, WIFI, LOCAL_DEVICE = 'enP8p1s0', 'wlP1p1s0', 'minerva0'
LOCAL_ADDRESS = '10.42.0.1'
TETHER_NETWORK = '10.42.0.1/24'
AP_NETWORK = '10.42.1.1/24'
AP_SSID = 'minerva'
AP_BAND, AP_CHANNEL = 'a', 36  # 5 GHz, below the radar (DFS) channels in the US domain.
LAN, TETHER, AP, LOCAL = 'minerva-lan', 'minerva-tether', 'minerva-ap', 'minerva-local'
OLD_WIRED, OLD_AP = 'Wired connection 1', 'aizee-ap'  # The fixed tether and AIZEE's access point it replaces.
OLD_DHCP = PurePosixPath('/etc/dnsmasq.d/aizee-poe.conf')  # The fixed tether's DHCP server (the system dnsmasq).
DHCP_TIMEOUT_S = 8
TETHER_CLIENTS_MAX = 4
LAN_ATTEMPTS = 3  # A port that stopped serving as a tether asks this many times, then waits for a replug.
RETRY_S = 30.  # A connection that failed to come up (the access point, the 10.42.0.1 dummy) is tried again after this.
LEASES = PurePosixPath(f'/var/lib/NetworkManager/dnsmasq-{ETHERNET}.leases')
PERIOD_S = 1.


def log(event, **fields):
    print(json.dumps({'event': event, **fields}, separators=(',', ':')), flush=True)


# -- the NetworkManager profiles ---------------------------------------------------------------------------------

def _uuid(name):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f'minerva-net:{name}'))


def _keyfile_value(text):
    """A string value as NetworkManager's keyfiles (GKeyFile) store it."""
    escaped = text.replace('\\', '\\\\').replace('\t', '\\t')
    if escaped.startswith(' '):
        escaped = '\\s' + escaped[1:]
    if escaped.endswith(' '):
        escaped = escaped[:-1] + '\\s'
    return escaped


def check_psk(psk):
    if not isinstance(psk, str) or not 8 <= len(psk) <= 63 or any(not ' ' <= c <= '~' for c in psk):
        raise ValueError('The access point password is 8 to 63 printable ASCII characters')
    return psk


def profiles(psk=None):
    """The keyfiles this service drives ({file name: text}); the access point's only given its password."""
    ipv6 = '\n[ipv6]\nmethod=disabled\n'

    def head(name, kind, device, autoconnect=False):
        return (f'[connection]\nid={name}\nuuid={_uuid(name)}\ntype={kind}\ninterface-name={device}\n'
                f"autoconnect={'true' if autoconnect else 'false'}\n")
    files = {
        f'{LAN}.nmconnection': head(LAN, 'ethernet', ETHERNET) + '\n[ethernet]\n\n[ipv4]\nmethod=auto\n'
                                f'dhcp-timeout={DHCP_TIMEOUT_S}\nmay-fail=false\n' + ipv6,
        f'{TETHER}.nmconnection': head(TETHER, 'ethernet', ETHERNET) + '\n[ethernet]\n\n[ipv4]\nmethod=shared\n'
                                   f'address1={TETHER_NETWORK}\n' + ipv6,
        f'{LOCAL}.nmconnection': head(LOCAL, 'dummy', LOCAL_DEVICE, autoconnect=True) + '\n[dummy]\n\n[ipv4]\n'
                                  f'method=manual\naddress1={LOCAL_ADDRESS}/32\n' + ipv6,
    }
    if psk is not None:
        files[f'{AP}.nmconnection'] = (
            head(AP, 'wifi', WIFI) + f'\n[wifi]\nmode=ap\nssid={AP_SSID}\nband={AP_BAND}\nchannel={AP_CHANNEL}\n'
            '\n[wifi-security]\nkey-mgmt=wpa-psk\nproto=rsn;\npairwise=ccmp;\ngroup=ccmp;\n'
            f'psk={_keyfile_value(check_psk(psk))}\n\n[ipv4]\nmethod=shared\naddress1={AP_NETWORK}\n' + ipv6)
    return files


NM_CONF_TEXT = f"""# minerva-net (robot_platform.ops.network) drives the Ethernet port itself: no automatic default profile on it.
[main]
no-auto-default=interface-name:{ETHERNET}
"""

SUDOERS_TEXT = f"""# Helm (the robot's user) switches Minerva's access point; nothing else runs as root this way.
{USER} ALL=(root) NOPASSWD: {PROGRAM} ap on, {PROGRAM} ap off
"""


# -- NetworkManager and the port, as the service sees them ------------------------------------------------------

def _run(command, timeout=60.):
    return subprocess.run(command, capture_output=True, text=True, timeout=timeout)


def _read(path):
    return Path(path).read_text(encoding='utf-8')


def _split_terse(line):
    """nmcli -t fields: ':' separates them and '\\:' is a colon inside one."""
    fields, current, escaped = [], '', False
    for char in line:
        if escaped:
            current, escaped = current + char, False
        elif char == '\\':
            escaped = True
        elif char == ':':
            fields.append(current)
            current = ''
        else:
            current += char
    return fields + [current]


class Manager:
    """NetworkManager through nmcli, the port's carrier and the tether's leases (each injectable for tests)."""

    def __init__(self, run=_run, read=_read):
        self.run, self.read = run, read

    def carrier(self):
        try:
            return self.read(f'/sys/class/net/{ETHERNET}/carrier').strip() == '1'
        except OSError:  # The port is down (no carrier file to read).
            return False

    def active(self):
        """{device: active connection id}."""
        result = self.run(['nmcli', '-t', '-f', 'NAME,DEVICE', 'connection', 'show', '--active'])
        active = {}
        for line in result.stdout.splitlines():
            fields = _split_terse(line)
            if len(fields) == 2 and fields[1]:
                active[fields[1]] = fields[0]
        return active

    def addresses(self, device):
        result = self.run(['ip', '-4', '-o', 'addr', 'show', 'dev', device])
        return [fields[3] for fields in (line.split() for line in result.stdout.splitlines()) if len(fields) > 3]

    def up(self, connection, wait):
        return self.run(['nmcli', '--wait', str(int(wait)), 'connection', 'up', connection],
                        timeout=wait + 15).returncode == 0

    def down(self, connection):
        return self.run(['nmcli', 'connection', 'down', connection]).returncode == 0

    def wifi_client(self, on):
        """The Wi-Fi client back (NetworkManager joins a known network when one is in range, without waiting
        here), or stepping aside until asked back."""
        self.run(['nmcli', 'device', 'set', WIFI, 'autoconnect', 'yes'] if on else
                 ['nmcli', 'device', 'disconnect', WIFI])

    def leases(self):
        """The hardware addresses holding a lease from the tether's DHCP server."""
        try:
            text = self.read(str(LEASES))
        except OSError:
            return set()
        return {line.split()[1].lower() for line in text.splitlines() if len(line.split()) >= 3}


class Network:
    """The service: each step reads the port, NetworkManager and the wanted access point, and acts on changes."""

    def __init__(self, manager, *, settings=SETTINGS, state=STATE, clock=time.time):
        self.nm, self.settings, self.state, self.clock = manager, Path(settings), Path(state), clock
        self.mode = 'starting'
        self.carrier = None
        self.served = set()  # The tether's leases when it started: clients before that are not counted.
        self.lan_attempts = 0
        self.retry_at = {}  # A connection that failed to come up: not before this time again.

    def bring_up(self, connection, wait):
        """Activate `connection`, unless it failed less than RETRY_S ago: an access point the radio refuses must not
        churn NetworkManager every step."""
        if self.clock() < self.retry_at.get(connection, 0.):
            return False
        ok = self.nm.up(connection, wait)
        if ok:
            self.retry_at.pop(connection, None)
        else:
            self.retry_at[connection] = self.clock() + RETRY_S
        return ok

    def ap_wanted(self):
        try:
            return json.loads(self.settings.read_text(encoding='utf-8')).get('ap') is True
        except (OSError, ValueError, AttributeError):
            return False

    def probe(self, ap, *, tether=True):
        """The port just got a cable (or lost its address): a network if its DHCP server answers, else a PC."""
        self.mode = 'probing'
        self.write(self.nm.active(), ap)
        if self.nm.up(LAN, DHCP_TIMEOUT_S + 10):
            self.mode, self.lan_attempts = 'lan', 0
            if not ap:
                self.nm.wifi_client(False)
            log('lan', addresses=self.nm.addresses(ETHERNET))
            return
        self.nm.down(LAN)
        if tether and self.nm.up(TETHER, 20):
            self.mode, self.served = 'tether', self.nm.leases()
            log('tether', address=TETHER_NETWORK)
            return
        self.mode = 'no_address'
        log('no_address', tether=tether)

    def step(self):
        ap = self.ap_wanted()
        active = self.nm.active()
        if active.get(LOCAL_DEVICE) != LOCAL:
            self.bring_up(LOCAL, 10)
        carrier = self.nm.carrier()
        if carrier and self.carrier is not True:
            log('plugged')
            self.probe(ap)
        elif not carrier and self.carrier is not False:
            if self.carrier is not None:
                log('unplugged')
            if active.get(ETHERNET) in (LAN, TETHER):
                self.nm.down(active[ETHERNET])
            self.mode = 'unplugged'
            if not ap:
                self.nm.wifi_client(True)
        elif carrier and self.mode in ('lan', 'tether') and active.get(ETHERNET) not in (LAN, TETHER):
            log('lost_address', mode=self.mode)  # Its lease ran out, or NetworkManager let it go.
            self.probe(ap)
        elif (carrier and self.mode == 'tether'
              and len(self.nm.leases() - self.served) > TETHER_CLIENTS_MAX):
            # A tether serves a PC or two: this many clients means a network whose DHCP server was slow.
            log('tether_is_a_network', clients=len(self.nm.leases() - self.served))
            self.nm.down(TETHER)
            self.lan_attempts = LAN_ATTEMPTS
            self.mode = 'no_address'
        if carrier and self.mode == 'no_address' and self.lan_attempts > 0:
            self.lan_attempts -= 1
            self.probe(ap, tether=False)
        self.carrier = carrier
        active = self.nm.active()
        on_ap = active.get(WIFI) == AP
        if not ap:
            self.retry_at.pop(AP, None)  # Wanted again later, it is tried at once.
        if ap and not on_ap:
            if self.clock() >= self.retry_at.get(AP, 0.):
                log('ap_on', ok=self.bring_up(AP, 30))
        elif not ap and on_ap:
            self.nm.down(AP)
            log('ap_off')
            if self.mode != 'lan':
                self.nm.wifi_client(True)
        elif not ap and self.mode == 'lan' and active.get(WIFI):
            self.nm.wifi_client(False)  # One path on the network: the wired one.
        self.write(self.nm.active(), ap)

    def snapshot(self, active, ap):
        wifi = active.get(WIFI)
        return {'version': VERSION, 'mode': self.mode, 'updated_at': self.clock(), 'local': LOCAL_ADDRESS,
                'ethernet': {'carrier': self.carrier, 'connection': active.get(ETHERNET),
                             'addresses': self.nm.addresses(ETHERNET)},
                'wifi': {'connection': wifi, 'mode': 'ap' if wifi == AP else ('client' if wifi else 'off'),
                         'addresses': self.nm.addresses(WIFI)},
                'ap': {'wanted': ap, 'on': wifi == AP, 'ssid': AP_SSID, 'address': AP_NETWORK.split('/')[0],
                       'failed': ap and wifi != AP and AP in self.retry_at}}

    def write(self, active, ap):
        _write_json(self.state, self.snapshot(active, ap))


def _write_json(path, value, mode=0o644):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    partial.write_text(json.dumps(value, indent=1), encoding='utf-8')
    os.chmod(partial, mode)
    os.replace(partial, path)


def serve(network, *, sleep=time.sleep, stop=lambda: False):
    while not stop():
        try:
            network.step()
        except (OSError, subprocess.SubprocessError) as exc:  # NetworkManager restarting: try again next step.
            log('step_failed', error=str(exc))
        sleep(PERIOD_S)


def set_ap(on, *, settings=SETTINGS, state=STATE, wait_s=45., clock=time.monotonic, sleep=time.sleep):
    """Want the access point on or off, and wait for the service to get there: the state it reached."""
    _write_json(settings, {'ap': bool(on)})
    deadline = clock() + wait_s
    while True:
        try:
            current = json.loads(Path(state).read_text(encoding='utf-8'))
        except (OSError, ValueError):
            current = None
        if current is not None and current['ap']['on'] == bool(on):
            return current
        if clock() > deadline:
            raise RuntimeError(f"The access point did not switch {'on' if on else 'off'} in {wait_s:g} s: "
                               f'journalctl -u {UNIT} -n 30')
        sleep(.5)


# -- installing --------------------------------------------------------------------------------------------------

class Host:
    """The robot computer as install and remove change it (a fake root in tests)."""

    def __init__(self, root='/', run=_run):
        self.root, self.run = Path(root), run

    def path(self, posix):
        return self.root / str(posix).lstrip('/')

    def systemctl(self, *args):
        return self.run(['systemctl', *args])

    def nmcli(self, *args):
        return self.run(['nmcli', *args])

    def enabled(self, unit):
        return self.run(['systemctl', 'is-enabled', unit]).stdout.strip() == 'enabled'

    def autoconnect(self, connection):
        """A connection profile's autoconnect ('yes', 'no'), or None when there is no such profile."""
        result = self.nmcli('-g', 'connection.autoconnect', 'connection', 'show', connection)
        return result.stdout.strip() if result.returncode == 0 else None


README = """Minerva's network service (robot_platform.ops.network), installed from a release by
    cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.network install
These files are root's copies: a root service must not run files the robot's user can change. Install again to
update them; `sudo minerva-net remove` restores what before.json recorded. State: /run/minerva-net/state.json.
"""


def unit_text():
    return f"""[Unit]
Description=Minerva network: the Ethernet port finds its mode (network by DHCP, or a tether at 10.42.0.1), and the access point
Documentation=file://{LIB}/README
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=simple
ExecStart=/usr/bin/python3 -I -B {LIB}/network.py run
Restart=always
RestartSec=3
RuntimeDirectory=minerva-net
RuntimeDirectoryMode=0755
RuntimeDirectoryPreserve=yes
StateDirectory=minerva-net
# Off the control CPUs (4-5) the teleop loop and receiver keep to (robot_platform.ops.cpu_partition).
CPUAffinity=0 1 2 3
Nice=10
ProtectSystem=full
ProtectHome=read-only
PrivateTmp=yes
NoNewPrivileges=yes

[Install]
WantedBy=multi-user.target
"""


def _write(path, data, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    partial.write_bytes(data)
    os.chmod(partial, mode)
    if path.exists() and not os.access(path, os.W_OK):
        os.chmod(path, 0o600)  # A read-only file (the sudoers rule) refuses replacing where renames honour it.
    os.replace(partial, path)


def install(host=None, source=None, *, psk=None, ask=getpass.getpass, start=True):
    """Install the service, its profiles and Helm's access point switch; ask for the access point's password the
    first time (or when `psk` is given). What it changes outside its own files is recorded for remove."""
    host = host or Host()
    source = Path(source or __file__)
    ap_file = host.path(CONNECTIONS / f'{AP}.nmconnection')
    if psk is None and not ap_file.exists():
        psk = ask(f"Password for the '{AP_SSID}' Wi-Fi access point (8-63 characters): ")
        check_psk(psk)
        if ask('The same password again: ') != psk:
            raise ValueError('The two passwords differ')
    keyfiles = profiles(psk)  # Checks the password before anything changes.
    lib = host.path(LIB)
    before = lib / 'before.json'
    if not before.exists():
        old_dhcp = host.path(OLD_DHCP)
        record = {'old_wired_autoconnect': host.autoconnect(OLD_WIRED), 'old_ap_autoconnect': host.autoconnect(OLD_AP),
                  'dnsmasq_enabled': host.enabled('dnsmasq.service'),
                  'old_dhcp': old_dhcp.read_text(encoding='utf-8') if old_dhcp.exists() else None}
        _write(before, json.dumps(record, indent=1).encode(), mode=0o600)
    _write(lib / 'network.py', source.read_bytes())
    _write(lib / 'README', README.encode())
    _write(host.path(PROGRAM), f'#!/bin/sh\nexec /usr/bin/python3 -I -B {LIB}/network.py "$@"\n'.encode(),
           mode=0o755)
    for name, text in keyfiles.items():
        _write(host.path(CONNECTIONS / name), text.encode(), mode=0o600)  # NetworkManager reads root-only keyfiles.
    _write(host.path(NM_CONF), NM_CONF_TEXT.encode())
    host.nmcli('connection', 'reload')
    for old in (OLD_WIRED, OLD_AP):
        if host.autoconnect(old) is not None:
            host.nmcli('connection', 'modify', old, 'connection.autoconnect', 'no')
    if host.path(OLD_DHCP).exists():
        host.path(OLD_DHCP).unlink()  # Its text is in before.json.
    host.systemctl('disable', '--now', 'dnsmasq.service')
    with tempfile.TemporaryDirectory() as scratch:
        check = Path(scratch) / 'minerva-net'
        check.write_text(SUDOERS_TEXT, encoding='utf-8')
        result = host.run(['visudo', '-cf', str(check)])
        if result.returncode != 0:
            raise RuntimeError(f'visudo refused the sudoers rule: {result.stdout}{result.stderr}')
    _write(host.path(SUDOERS), SUDOERS_TEXT.encode(), mode=0o440)
    _write(host.path(UNITS / UNIT), unit_text().encode())
    host.systemctl('daemon-reload')
    host.systemctl('enable', UNIT)
    if start:
        print('Starting minerva-net. If the Ethernet cable is on a network, Minerva\'s Wi-Fi steps aside and this '
              'session may end: reconnect at aizee-rover.local, its wired address, or 10.42.0.1.', flush=True)
        host.systemctl('restart', UNIT)
    return {'installed': True, 'started': start}


def remove(host=None):
    host = host or Host()
    if host.enabled(UNIT):
        host.systemctl('disable', '--now', UNIT)
    lib = host.path(LIB)
    try:
        record = json.loads((lib / 'before.json').read_text(encoding='utf-8'))
    except (OSError, ValueError):
        record = {}
    for path in [host.path(CONNECTIONS / name) for name in (*profiles(), f'{AP}.nmconnection')] + [
            host.path(NM_CONF), host.path(SUDOERS), host.path(PROGRAM), host.path(UNITS / UNIT)]:
        if path.exists():
            if not os.access(path, os.W_OK):
                os.chmod(path, 0o600)  # The read-only sudoers rule, where deleting honours that.
            path.unlink()
    host.nmcli('connection', 'reload')
    for old, key in ((OLD_WIRED, 'old_wired_autoconnect'), (OLD_AP, 'old_ap_autoconnect')):
        if record.get(key) is not None and host.autoconnect(old) is not None:
            host.nmcli('connection', 'modify', old, 'connection.autoconnect', record[key])
    if record.get('old_dhcp') is not None:
        _write(host.path(OLD_DHCP), record['old_dhcp'].encode())
    if record.get('dnsmasq_enabled'):
        host.systemctl('enable', '--now', 'dnsmasq.service')
    host.systemctl('daemon-reload')
    host.nmcli('device', 'set', WIFI, 'autoconnect', 'yes')
    shutil.rmtree(lib, ignore_errors=True)
    return {'removed': True, 'restored': sorted(k for k, v in record.items() if v is not None)}


def status(state=STATE):
    try:
        return json.loads(Path(state).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None


def main(argv=None):
    parser = argparse.ArgumentParser(prog='minerva-net', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('run', 'install', 'remove', 'status', 'ap'))
    parser.add_argument('switch', nargs='?', choices=('on', 'off'), help='ap: on or off')
    parser.add_argument('--new-password', action='store_true', help="install: ask for the access point's password again")
    args = parser.parse_args(argv)
    root = getattr(os, 'geteuid', lambda: -1)() == 0
    try:
        if args.action == 'run':
            signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
            log('started', version=VERSION)
            serve(Network(Manager()))
            return 0
        if args.action == 'status':
            report = status()
        elif args.action == 'ap':
            if args.switch is None:
                parser.error('ap needs on or off')
            if not root:
                print('Switching the access point needs sudo.', file=sys.stderr)
                return 2
            report = set_ap(args.switch == 'on')
        else:
            if not root:
                print(f'{args.action} changes NetworkManager and systemd: run it with sudo.', file=sys.stderr)
                return 2
            if args.action == 'install' and not sys.flags.dont_write_bytecode:
                print('Run as sudo python3 -B -m robot_platform.ops.network, so no root-owned bytecode is left in '
                      'the release.', file=sys.stderr)
                return 2
            if args.action == 'install':
                psk = None
                if args.new_password:
                    psk = getpass.getpass(f"New password for the '{AP_SSID}' access point (8-63 characters): ")
                    check_psk(psk)
                    if getpass.getpass('The same password again: ') != psk:
                        raise ValueError('The two passwords differ')
                report = install(psk=psk)
            else:
                report = remove()
    except (RuntimeError, ValueError, OSError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
