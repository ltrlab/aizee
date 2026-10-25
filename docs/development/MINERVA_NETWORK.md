# Minerva's network

- **Branch and worktree:** `claude/minerva-network` in `C:\Users\ltr\Workspace\aizee-network`, from `claude/neck-feetech`
  (`19fe00c`, what Minerva ran).
- **The user's request (2026-10-25):** "Can you make how we connect to it more dynamic and intuitive? I want it to
  detect if it's on a network when we plug it into one, if it's a usb adapter I want it to setup the local address
  for 10.42.0.1 and I also want an AP mode that we can toggle and connect with that same IP."
- **The user's choices:**
  - The access point is a new network, `minerva`, with a password typed on the robot.
  - Leader teleop is allowed over Wi-Fi, with a warning.
  - The bridge key is also accepted from the home network (192.168.8.0/24).

## The modes

`robot_platform.ops.network` runs as the system service `minerva-net` on the Jetson. It watches the Ethernet port
and drives NetworkManager.

| The cable goes to | Minerva | Its Wi-Fi |
|---|---|---|
| A network (its DHCP answers within 8 s) | The network's address (wired) | Steps aside: one path on the network |
| A PC's USB Ethernet adapter (no DHCP) | 10.42.0.1/24, handing the PC 10.42.0.x | Stays on its network |
| Nothing | (Wi-Fi only) | Back on its network |

- **Minerva is always at 10.42.0.1** for anything plugged into it or joined to its access point. A dummy
  interface (`minerva0`) holds that address.
- **The access point `minerva`** runs on 5 GHz channel 36 with WPA2. Its devices get 10.42.1.x, and reach Minerva
  at 10.42.0.1 through it.
  - It takes the Wi-Fi radio while it is on, so Minerva leaves its Wi-Fi network meanwhile.
  - It stays on across restarts until switched off.
  - Switch it from Helm: **Settings → Network**. Helm refuses while a session, sweep or Check holds the motors.
- **A cable that is really a network:** if more than four devices take a tether address, the cable is on a network
  whose DHCP server was slow to answer. The port then stops serving addresses and asks for one three times, then
  waits for a replug. Serving addresses on someone's network would hand out dead ones. The old fixed tether did
  exactly that when its cable moved to the network on 2026-10-25.
- **State:** `/run/minerva-net/state.json`, readable by anyone. Helm shows it as the header's network pill and in
  Settings → Network. `minerva-net status` prints it.

## The GELLO bridge

The bridge on the operator PC finds Minerva by itself (`--robot auto`, the installer's default). It tries, in order:

1. 10.42.0.1 (the tether or the access point);
2. the addresses Helm reported last time, wired first;
3. `aizee-rover.local` (mDNS);
4. the profile's address.

The first that answers and proves to be Minerva wins: its host key is pinned, and the machine id is checked in
Helm's welcome.

- **The relay** reports the address the bridge reached (`SSH_CONNECTION`). A session's leader link binds there.
- **Helm's GELLO pill** says when that path is Wi-Fi ("GELLO · Wi-Fi"), and so does a leader session's start
  message: late samples make the arms hold.
- **The bridge key's `from=`** is `10.42.0.0/16` (tether and access point) plus the network of the profile's address
  (`192.168.8.0/24`). `authorize-bridge` replaces an earlier line of its own for the same key and name, keeping a
  dated copy.

## Installing (the user's sudo)

```
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.network install
```

1. It asks for the access point's password on the terminal, twice.
2. It writes root-owned copies (`/usr/local/lib/minerva-net`, `/usr/local/sbin/minerva-net`).
3. It writes the NetworkManager keyfiles: `minerva-lan`, `minerva-tether`, `minerva-ap` and `minerva-local`.
4. It turns off autoconnect on the old `Wired connection 1` and `aizee-ap` profiles, and disables the old tether's
   dnsmasq (its config is kept).
5. It installs Helm's sudo rule (`ap on` and `ap off` only), then starts the service.

- What it changes outside its own files is recorded in `before.json`; `sudo minerva-net remove` restores it.
- `--new-password` changes the access point's password.
- On a network cable, Minerva's Wi-Fi steps aside when the service starts. A session over Wi-Fi then ends:
  reconnect at `aizee-rover.local`, the wired address, or 10.42.0.1.

## Tests

- `tests/unit/test_minerva_network.py`: the modes, the slow-DHCP rule, the access point, install and remove, and the
  sudo rule.
- `tests/unit/test_helm_network.py`: the route's kind, the state, and the switch in Helm.
- `tests/unit/test_gello_bridge.py`: the relay's address, the candidates and the loop.
- `tests/unit/test_helm_on_robot.py`: the link binds where the bridge reached Minerva, and Wi-Fi is said.
- `tests/ui/test_helm_network.mjs`: the pills and Settings → Network.

**Not yet tried on the robot.** The first install is the user's sudo.
