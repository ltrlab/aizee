"""Minerva's network service (robot_platform.ops.network) on a bench: what the Ethernet cable is plugged into decides
the mode (a network's DHCP answers: wired LAN, the Wi-Fi client steps aside; a PC: the tether at 10.42.0.1; nothing:
the Wi-Fi client back), a tether with many clients stops serving, and the access point takes and returns the radio.
Installing writes root's copy, the profiles, Helm's sudo rule and the unit, and records what it changes for remove."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.ops import network
from robot_platform.ops.network import AP, ETHERNET, LAN, LOCAL, LOCAL_DEVICE, TETHER, WIFI

ADDRESSES = {LAN: '192.168.8.43/24', TETHER: '10.42.0.1/24', 'birdnest': '192.168.8.180/24', AP: '10.42.1.1/24',
             LOCAL: '10.42.0.1/32'}


class Bench:
    """NetworkManager on a bench: `cable` is None (unplugged), 'network' (a DHCP server answers) or 'pc'."""

    def __init__(self):
        self.cable = None
        self.connections = {WIFI: 'birdnest'}
        self.lease_set = set()
        self.calls = []
        self.ap_refused = False

    def carrier(self):
        return self.cable is not None

    def active(self):
        return dict(self.connections)

    def addresses(self, device):
        connection = self.connections.get(device)
        return [ADDRESSES[connection]] if connection in ADDRESSES else []

    def up(self, connection, wait):
        self.calls.append(('up', connection))
        if connection == AP and self.ap_refused:
            return False
        if connection == LAN and self.cable != 'network':
            return False
        if connection in (LAN, TETHER) and self.cable is None:
            return False
        device = {LAN: ETHERNET, TETHER: ETHERNET, AP: WIFI, LOCAL: LOCAL_DEVICE}[connection]
        self.connections[device] = connection
        return True

    def down(self, connection):
        self.calls.append(('down', connection))
        for device, active in list(self.connections.items()):
            if active == connection:
                del self.connections[device]
        return True

    def wifi_client(self, on):
        self.calls.append(('wifi', on))
        if on:
            self.connections.setdefault(WIFI, 'birdnest')
        elif self.connections.get(WIFI) != AP:
            self.connections.pop(WIFI, None)

    def leases(self):
        return set(self.lease_set)


@pytest.fixture
def bench(tmp_path):
    nm = Bench()
    clock = SimpleNamespace(now=1000.)
    service = network.Network(nm, settings=tmp_path / 'settings.json', state=tmp_path / 'state.json',
                              clock=lambda: clock.now)

    def state():
        return json.loads((tmp_path / 'state.json').read_text(encoding='utf-8'))

    def want_ap(on):
        (tmp_path / 'settings.json').write_text(json.dumps({'ap': on}), encoding='utf-8')
    return SimpleNamespace(nm=nm, service=service, state=state, want_ap=want_ap, clock=clock)


def test_on_a_network_the_port_takes_its_address_and_the_wifi_steps_aside(bench):
    bench.service.step()  # Unplugged at start: the Wi-Fi client stays, and Minerva answers at 10.42.0.1.
    assert bench.state()['mode'] == 'unplugged' and bench.nm.connections[LOCAL_DEVICE] == LOCAL
    bench.nm.cable = 'network'
    bench.service.step()
    state = bench.state()
    assert state['mode'] == 'lan' and bench.nm.connections[ETHERNET] == LAN
    assert state['ethernet']['addresses'] == ['192.168.8.43/24'] and state['wifi']['mode'] == 'off'
    assert state['local'] == '10.42.0.1'
    bench.nm.cable = None  # Unplugged: the Wi-Fi client comes back.
    bench.service.step()
    assert bench.state()['mode'] == 'unplugged' and bench.nm.connections[WIFI] == 'birdnest'
    assert ETHERNET not in bench.nm.connections


def test_on_a_pc_the_port_is_the_tether_at_10_42_0_1(bench):
    bench.nm.cable = 'pc'
    bench.service.step()
    state = bench.state()
    assert state['mode'] == 'tether' and bench.nm.connections[ETHERNET] == TETHER
    assert state['ethernet']['addresses'] == ['10.42.0.1/24']
    assert bench.nm.connections[WIFI] == 'birdnest'  # The Wi-Fi client keeps Minerva on its network.
    assert bench.nm.calls[:2] == [('up', LOCAL), ('up', LAN)]  # A network is asked first.


def test_a_tether_with_many_clients_is_a_network_and_stops_serving(bench):
    bench.nm.cable = 'pc'
    bench.nm.lease_set = {'aa:00:00:00:00:01'}  # A lease from before this tether started is not counted.
    bench.service.step()
    bench.nm.lease_set |= {f'aa:00:00:00:00:{n:02x}' for n in range(2, 2 + network.TETHER_CLIENTS_MAX)}
    bench.service.step()
    assert bench.state()['mode'] == 'tether'  # TETHER_CLIENTS_MAX new clients: still a tether.
    bench.nm.lease_set.add('aa:00:00:00:00:ff')
    bench.service.step()
    assert ETHERNET not in bench.nm.connections and bench.state()['mode'] == 'no_address'
    for _ in range(network.LAN_ATTEMPTS + 2):
        bench.service.step()
    assert [c for c in bench.nm.calls if c == ('up', TETHER)] == [('up', TETHER)]  # Never served again.
    assert len([c for c in bench.nm.calls if c == ('up', LAN)]) == 2 + network.LAN_ATTEMPTS - 1
    bench.nm.cable = None
    bench.service.step()
    bench.nm.cable = 'pc'  # A replug starts over.
    bench.service.step()
    assert bench.state()['mode'] == 'tether'


def test_a_network_whose_dhcp_answers_late_is_found_on_a_retry(bench):
    bench.nm.cable = 'pc'
    bench.service.step()
    bench.nm.lease_set = {f'bb:00:00:00:00:{n:02x}' for n in range(network.TETHER_CLIENTS_MAX + 1)}
    bench.nm.cable = 'network'  # Its DHCP server is back.
    bench.service.step()
    assert bench.state()['mode'] == 'lan' and bench.nm.connections[ETHERNET] == LAN


def test_a_lost_address_with_the_cable_in_is_asked_for_again(bench):
    bench.nm.cable = 'network'
    bench.service.step()
    del bench.nm.connections[ETHERNET]  # NetworkManager let the lease go.
    bench.service.step()
    assert bench.state()['mode'] == 'lan' and bench.nm.connections[ETHERNET] == LAN


def test_the_access_point_takes_the_radio_and_gives_it_back(bench):
    bench.want_ap(True)
    bench.service.step()
    state = bench.state()
    assert bench.nm.connections[WIFI] == AP and state['ap'] == {'wanted': True, 'on': True, 'ssid': 'minerva',
                                                                 'address': '10.42.1.1', 'failed': False}
    assert state['wifi']['mode'] == 'ap'
    bench.nm.cable = 'network'  # On a network with the access point on: it keeps the radio.
    bench.service.step()
    assert bench.nm.connections[WIFI] == AP and bench.state()['mode'] == 'lan'
    bench.want_ap(False)
    bench.service.step()
    assert WIFI not in bench.nm.connections  # Wired on a network: the Wi-Fi client stays aside.
    bench.nm.cable = None
    bench.service.step()
    assert bench.nm.connections[WIFI] == 'birdnest'
    bench.want_ap(True)
    bench.service.step()
    bench.want_ap(False)
    bench.service.step()
    assert bench.nm.connections[WIFI] == 'birdnest'  # Unplugged: switching it off brings the client back.


def test_profiles_and_password(monkeypatch):
    files = network.profiles()
    assert set(files) == {f'{LAN}.nmconnection', f'{TETHER}.nmconnection', f'{LOCAL}.nmconnection'}
    assert 'method=auto' in files[f'{LAN}.nmconnection'] and 'dhcp-timeout=8' in files[f'{LAN}.nmconnection']
    assert 'method=shared' in files[f'{TETHER}.nmconnection'] and 'address1=10.42.0.1/24' in files[f'{TETHER}.nmconnection']
    assert 'type=dummy' in files[f'{LOCAL}.nmconnection'] and 'autoconnect=true' in files[f'{LOCAL}.nmconnection']
    assert all('autoconnect=false' in text for name, text in files.items() if LOCAL not in name)
    assert network.profiles()[f'{LAN}.nmconnection'] == files[f'{LAN}.nmconnection']  # Stable ids across installs.
    ap = network.profiles(' pass word\\ ')[f'{AP}.nmconnection']
    assert 'psk=\\spass word\\\\\\s\n' in ap and 'mode=ap' in ap and 'ssid=minerva' in ap
    assert 'band=a' in ap and 'channel=36' in ap and 'address1=10.42.1.1/24' in ap
    for bad in ('short', 'x' * 64, 'naïve password', 'line\nbreak'):
        with pytest.raises(ValueError, match='8 to 63'):
            network.profiles(bad)


class FakeRun:
    def __init__(self):
        self.commands = []
        self.autoconnect = {network.OLD_WIRED: 'yes', network.OLD_AP: 'yes'}
        self.enabled = {'dnsmasq.service': True}

    def __call__(self, command, timeout=60.):
        self.commands.append(command)
        ok = SimpleNamespace(returncode=0, stdout='', stderr='')
        if command[:3] == ['nmcli', '-g', 'connection.autoconnect']:
            value = self.autoconnect.get(command[-1])  # nmcli exits 10 for a profile that does not exist.
            return SimpleNamespace(returncode=0 if value else 10, stdout=f'{value}\n' if value else '', stderr='')
        if command[:3] == ['nmcli', 'connection', 'modify']:
            self.autoconnect[command[3]] = command[-1]
        if command[:2] == ['systemctl', 'is-enabled']:
            return SimpleNamespace(returncode=0, stdout='enabled\n' if self.enabled.get(command[2]) else 'disabled\n',
                                   stderr='')
        if command[:2] == ['systemctl', 'disable']:
            self.enabled[command[-1]] = False
        if command[:2] == ['systemctl', 'enable']:
            self.enabled[command[-1]] = True
        return ok


def test_install_and_remove_on_a_fake_root(tmp_path):
    run = FakeRun()
    host = network.Host(tmp_path, run=run)
    old_dhcp = host.path(network.OLD_DHCP)
    old_dhcp.parent.mkdir(parents=True)
    old_dhcp.write_text('interface=enP8p1s0\ndhcp-range=10.42.0.10,10.42.0.20,12h\n', encoding='utf-8')
    asked = iter(['minerva-pass', 'minerva-pass'])
    report = network.install(host, ask=lambda prompt: next(asked), start=False)
    assert report == {'installed': True, 'started': False}
    lib = host.path(network.LIB)
    assert (lib / 'network.py').read_bytes() == Path(network.__file__).read_bytes()
    assert 'network.py "$@"' in host.path(network.PROGRAM).read_text(encoding='utf-8')
    keyfiles = host.path(network.CONNECTIONS)
    assert sorted(p.name for p in keyfiles.iterdir()) == sorted([f'{AP}.nmconnection', f'{LAN}.nmconnection',
                                                                 f'{LOCAL}.nmconnection', f'{TETHER}.nmconnection'])
    assert 'psk=minerva-pass' in (keyfiles / f'{AP}.nmconnection').read_text(encoding='utf-8')
    if os.name == 'posix':
        assert (keyfiles / f'{AP}.nmconnection').stat().st_mode & 0o777 == 0o600
        assert host.path(network.SUDOERS).stat().st_mode & 0o777 == 0o440
    assert host.path(network.SUDOERS).read_text(encoding='utf-8') == network.SUDOERS_TEXT
    assert 'no-auto-default=interface-name:enP8p1s0' in host.path(network.NM_CONF).read_text(encoding='utf-8')
    assert not old_dhcp.exists() and run.enabled['dnsmasq.service'] is False
    assert run.autoconnect == {network.OLD_WIRED: 'no', network.OLD_AP: 'no'}
    assert any(c[:2] == ['visudo', '-cf'] for c in run.commands)
    assert 'ProtectSystem=full' in host.path(network.UNITS / network.UNIT).read_text(encoding='utf-8')
    before = json.loads((lib / 'before.json').read_text(encoding='utf-8'))
    assert before['old_wired_autoconnect'] == 'yes' and before['dnsmasq_enabled'] is True
    # Installing again keeps the access point's password and the first record of what was there before.
    network.install(host, ask=lambda prompt: pytest.fail('asked again'), start=False)
    assert json.loads((lib / 'before.json').read_text(encoding='utf-8')) == before
    report = network.remove(host)
    assert not lib.exists() and not keyfiles.joinpath(f'{AP}.nmconnection').exists()
    assert not host.path(network.SUDOERS).exists() and not host.path(network.PROGRAM).exists()
    assert old_dhcp.read_text(encoding='utf-8').startswith('interface=enP8p1s0')
    assert run.autoconnect == {network.OLD_WIRED: 'yes', network.OLD_AP: 'yes'} and run.enabled['dnsmasq.service']
    assert 'old_dhcp' in report['restored']


def test_install_refuses_a_bad_or_mistyped_password_before_changing_anything(tmp_path):
    host = network.Host(tmp_path, run=FakeRun())
    for answers, message in ((['short', 'short'], '8 to 63'), (['minerva-pass', 'minerva-pas'], 'differ')):
        asked = iter(answers)
        with pytest.raises(ValueError, match=message):
            network.install(host, ask=lambda prompt: next(asked), start=False)
        assert not host.path(network.LIB).exists()


def test_switching_the_access_point_waits_for_the_service(tmp_path):
    settings, state = tmp_path / 'settings.json', tmp_path / 'state.json'
    clock = SimpleNamespace(now=0.)

    def sleep(seconds):
        clock.now += seconds
        if clock.now >= 2.:  # The service gets there two seconds later.
            state.write_text(json.dumps({'ap': {'on': True}}), encoding='utf-8')
    reached = network.set_ap(True, settings=settings, state=state, clock=lambda: clock.now, sleep=sleep)
    assert reached['ap']['on'] is True and json.loads(settings.read_text(encoding='utf-8')) == {'ap': True}
    with pytest.raises(RuntimeError, match='did not switch off'):
        network.set_ap(False, settings=settings, state=state, wait_s=3., clock=lambda: clock.now, sleep=sleep)


def test_the_sudo_rule_allows_only_switching_the_access_point():
    rules = [line for line in network.SUDOERS_TEXT.splitlines() if line and not line.startswith('#')]
    assert rules == [f'ltr ALL=(root) NOPASSWD: {network.PROGRAM} ap on, {network.PROGRAM} ap off']


def test_an_access_point_the_radio_refuses_is_retried_every_30_s_not_every_step(bench):
    bench.nm.ap_refused = True
    bench.want_ap(True)
    bench.service.step()
    for _ in range(5):
        bench.clock.now += 1.
        bench.service.step()
    assert bench.nm.calls.count(('up', AP)) == 1 and bench.state()['ap']['failed'] is True
    assert bench.nm.connections[WIFI] == 'birdnest'  # The client keeps the radio meanwhile.
    bench.clock.now += network.RETRY_S
    bench.nm.ap_refused = False
    bench.service.step()
    assert bench.nm.calls.count(('up', AP)) == 2 and bench.state()['ap'] == dict(
        bench.state()['ap'], on=True, failed=False)
    bench.want_ap(False)
    bench.service.step()
    bench.nm.ap_refused = True
    bench.want_ap(True)  # Wanted again: tried at once, its old failure forgotten.
    bench.service.step()
    assert bench.nm.calls.count(('up', AP)) == 3


def test_a_state_file_windows_will_not_replace_yet_is_written_once_it_will(tmp_path, monkeypatch):
    # Under the gate's workers a virus scanner still had the last state.json open: os.replace was refused for up to
    # 82 ms (2026-11-01). The write waits that out; a refusal that lasts is still an error.
    replace, refusals = os.replace, iter([True, True, False])

    def busy(source, target):
        if next(refusals, False):
            raise PermissionError(5, 'Access is denied')
        replace(source, target)
    monkeypatch.setattr(network.os, 'replace', busy)
    network._write_json(tmp_path / 'state.json', {'mode': 'tether'})
    assert json.loads((tmp_path / 'state.json').read_text(encoding='utf-8')) == {'mode': 'tether'}

    def refused(source, target):
        raise PermissionError(5, 'Access is denied')
    monkeypatch.setattr(network.os, 'replace', refused)
    with pytest.raises(PermissionError):
        network._replace(tmp_path / 'state.json.partial', tmp_path / 'state.json', patience_s=.05)
