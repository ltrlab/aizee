# Minerva's network

Minerva's Ethernet port finds its own mode when a cable is plugged in, Minerva always answers at `10.42.0.1`, and a
Wi-Fi access point can be switched on from Helm. This is the system service `minerva-net`
(`robot_platform.ops.network`, installed in step 6 of [INSTALL_JETSON.md](../getting-started/INSTALL_JETSON.md#6-network)).
It watches the Ethernet port and drives NetworkManager.

## The modes

| The cable goes to | Minerva | Its Wi-Fi |
| --- | --- | --- |
| A network (its DHCP answers within 8 s) | The network's address (wired) | Steps aside: one path on the network |
| A PC's USB Ethernet adapter (no DHCP) | `10.42.0.1/24`, handing the PC `10.42.0.x` | Stays on its network |
| Nothing | (Wi-Fi only) | Back on its network |

- **Minerva is always at `10.42.0.1`** for anything plugged into it or joined to its access point. A dummy interface
  (`minerva0`) holds that address.
- **The access point `minerva`** runs on 5 GHz channel 36 (below the radar channels in the US domain) with WPA2. Its
  devices get `10.42.1.x` and reach Minerva at `10.42.0.1` through it.
  - It takes the Wi-Fi radio while it is on, so Minerva leaves its Wi-Fi network meanwhile.
  - It stays on across restarts until switched off.
  - Switch it from Helm: **Settings, Network**. Helm refuses while a session, sweep or Check holds the motors. Helm
    runs `ap on` and `ap off` through a sudo rule that allows exactly those two.
  - The password is typed on the robot at install (`--new-password` changes it).
- **A cable that is really a network:** if more than four devices take a tether address, the cable is on a network whose
  DHCP server was slow to answer. The port then stops serving addresses and asks for one three times, then waits for a
  replug. Serving addresses on someone's network would hand out dead ones; the old fixed tether did exactly that when
  its cable moved to a network.
- **State:** `/run/minerva-net/state.json`, readable by anyone. Helm shows it as the header's network pill and in
  Settings, Network. `minerva-net status` prints it.

Leader teleop is allowed over Wi-Fi, with a warning (late samples make the arms hold). That is the leader link between
the operator station and the robot; Helm's own page can be opened over any of these paths.

## The GELLO bridge

The bridge on the operator station finds Minerva by itself (`--robot auto`, the installer's default). It tries, in
order:

1. `10.42.0.1` (the tether or the access point);
2. the addresses Helm reported last time, wired first;
3. `aizee-rover.local` (mDNS; the name is the Jetson's host name);
4. the device profile's address.

The first that answers and proves to be Minerva wins: its host key is pinned, and the machine id is checked in Helm's
welcome. Name resolution is IPv4 only, because the bridge key refuses an IPv6 link-local address.

- **The relay** reports the address the bridge reached (`SSH_CONNECTION`). A session's leader link binds there.
- **Helm's GELLO pill** says when that path is Wi-Fi ("GELLO · Wi-Fi"), and so does a leader session's start message.
- **The bridge key's `from=`** is `10.42.0.0/16` (tether and access point) plus the network of the device profile's
  address. `authorize-bridge` replaces an earlier line of its own for the same key and name, keeping a dated copy.

## Installing

```
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.network install
```

1. It asks for the access point's password on the terminal, twice.
2. It writes root-owned copies (`/usr/local/lib/minerva-net`, `/usr/local/sbin/minerva-net`).
3. It writes the NetworkManager keyfiles `minerva-lan`, `minerva-tether`, `minerva-ap` and `minerva-local`.
4. It turns off autoconnect on the old `Wired connection 1` and `aizee-ap` profiles, and disables the old tether's
   dnsmasq (its config is kept).
5. It installs Helm's sudo rule (`ap on` and `ap off` only), then starts the service.

What it changes outside its own files is recorded in `before.json`; `sudo minerva-net remove` restores it. The
interface names it manages (`enP8p1s0`, `wlP1p1s0`) are constants in `ops/network.py`. On a network cable, Minerva's
Wi-Fi steps aside when the service starts, so a session over Wi-Fi ends: reconnect at `aizee-rover.local`, the wired
address, or `10.42.0.1`. The AIZEE access point `aizee` at `192.168.50.1` is the older robot's; Minerva's is `minerva`.

## What has been tried on the robot

The service was installed once, with the cable on a home network. Network mode (`lan`) was reached within a second,
Minerva's Wi-Fi stepped aside, the default route went over the cable, `minerva0` held `10.42.0.1`, and
`aizee-rover.local` resolved over IPv4. The bridge lost its Wi-Fi path and reached Minerva at `aizee-rover.local`
three seconds later, its route leaving on the cable, so Helm counted it wired.

**Not yet tried on the robot:** the tether with the cable on a PC's adapter, the access point, and leader teleop over
each path.

Before the install, with the cable on a network but the port still fixed at `10.42.0.1`, mDNS gave out `10.42.0.1`,
which nothing on that network can reach.

## Tests

`tests/unit/test_minerva_network.py` (the modes, the slow-DHCP rule, the access point, install and remove, the sudo
rule), `tests/unit/test_helm_network.py` (the route's kind, the state, the switch in Helm),
`tests/unit/test_gello_bridge.py` (the relay's address, the candidates and the loop),
`tests/unit/test_helm_on_robot.py` (the link binds where the bridge reached Minerva, and Wi-Fi is said) and
`tests/ui/test_helm_network.mjs` (the pills and Settings, Network).
