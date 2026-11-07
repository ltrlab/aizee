# Install the operator station

The operator station is the Windows PC the GELLO leader arms plug into. Helm itself runs on the Jetson and is reached
from any browser; the leaders cannot move there, so a small program on this PC, the **GELLO bridge**, serves them to
Helm. The station also holds the tether adapter, the foot pedals and, usually, the browser you operate from.

Do this after the [Jetson install](INSTALL_JETSON.md) has Helm running. The bridge opens every connection itself and
takes no inbound connection except the robot's signed leader-link requests, which arrive on a UDP socket opened for each
session. How it fits together is in [HELM_ON_JETSON.md](../architecture/HELM_ON_JETSON.md).

You need Windows 11, Python 3.12 (the bridge's hash-locked environment is for 3.12 exactly; the installer takes
`%LOCALAPPDATA%\Minerva\Python312\python.exe` if it exists, else `python` on the path, and stops on any other
version), and the Windows OpenSSH Client feature (`ssh-keygen`).

## 1. Power and the tether adapter

`scripts/setup_operator_station.ps1`, run once in PowerShell **as Administrator** with the tether plugged in:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/setup_operator_station.ps1 -AuthorizeKey "ssh-ed25519 AAAA... name"
```

It:

- turns off sleep and hibernation on mains power and USB selective suspend, so the bridge and the leaders stay up;
- turns off power saving on the tether adapter: Energy-Efficient Ethernet, Green Ethernet, Idle Power Saving,
  Selective Suspend, Miscellaneous Transfer Settings, and Device Manager's "Allow the computer to turn off this
  device". It treats a network interface on `10.42.0.x`, or any USB Ethernet adapter, as the tether, and restarts it.
  A power-saving tether delays leader replies past the robot's 4 ms deadline. Keep a monitor off the adapter's USB hub
  for the same reason;
- installs OpenSSH Server, starts it at boot, allows keys only, allows port 22 from the local subnet and from
  Tailscale's range (100.64.0.0/10) only, and adds the given public key. This lets a build PC elsewhere deploy to the
  robot through this PC over the tether; it is not needed if you deploy from this PC itself. `-AuthorizeKey` is
  required anyway, so pass a key you control.

It prints the SSH host key fingerprint, the user, the addresses and the adapters it changed. If it warns that it found
no tether adapter, plug the tether in and run it again.

Verify: `Get-Service sshd` is Running, `powercfg /q SCHEME_CURRENT SUB_SLEEP` shows the AC standby and hibernate
timeouts at 0, and the adapter's Advanced tab shows the power-saving items disabled.

## 2. The bridge

On the build PC, from a clean, committed checkout, package the bridge. The package holds committed files only (the
package refuses an uncommitted tree), each hashed in a manifest the installer checks:

```powershell
python -m robot_platform.ops.helm_robot bridge
```

This writes `build/gello-bridge/gello-bridge-<commit>.zip`. Copy it to the station and unpack
`scripts/install_gello_bridge.ps1` from it (or use the same commit's checkout). Then, as the operator and **not** as
administrator:

```powershell
powershell -ExecutionPolicy Bypass -File install_gello_bridge.ps1 -Package gello-bridge-<commit>.zip
```

The installer:

1. checks every file against the manifest and unpacks to `%LOCALAPPDATA%\Minerva\gello-bridge-<commit>`;
2. builds a per-lock environment `bridge-env-<lock id>` from `requirements/gello-bridge.lock` with hashes required,
   and runs `pip check`;
3. makes the bridge's SSH key once, `%LOCALAPPDATA%\Minerva\keys\gello_bridge_ed25519`, readable only by you;
4. writes `Run-GelloBridge.ps1`, which restarts the bridge five seconds after it exits;
5. registers the task `Minerva-GELLO-Bridge`: no window, from startup, run as you (S4U), and started again within a
   minute if it is not running. A reinstall ends any older bridge and moves the task to the new package. The log is
   `%LOCALAPPDATA%\Minerva\logs\gello-bridge.log`.

`-Robot auto` is the default: the bridge tries `10.42.0.1` (the tether or the access point), then the addresses Helm
last reported, then `aizee-rover.local`, then the profile's address, and the first that answers and proves to be the
robot wins ([MINERVA_NETWORK.md](../architecture/MINERVA_NETWORK.md#the-gello-bridge)). `-Robot <address>` fixes one.

The installer prints the bridge's public key. The key does nothing yet: authorize it on the Jetson, for Helm's relay
only (a forced command), from the tether, the access point and the network of the profile's address only:

```bash
cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service authorize-bridge --name <station-name> --key "<public key line>"
```

Verify: `Get-ScheduledTask Minerva-GELLO-Bridge` reports Running, the log's last lines say it connected to Helm, and
Helm's **GELLO** pill in the header turns on. Reinstalling with a newer package after each release that changes the
bridge or the leader link keeps the two ends in step.

## 3. The leaders

Each leader is a Robotis OpenRB-150 with seven Dynamixel XL330 servos, on USB. See [OPENRB_LEADER.md](../robots/OPENRB_LEADER.md)
for the wire protocol and [the firmware README](../../firmware/openrb_leader/README.md) for building it.

- **Firmware.** The boards must run firmware 2 or later (the installed boards run 3). Build and flash from a checkout
  with PlatformIO; the vendored board support makes this self-contained:

  ```bash
  pip install --user platformio
  pio run -e openrb150 -t upload --upload-port COM<n> -d firmware/openrb_leader
  ```

  `python scripts/openrb_leader_setup.py info` then prints each board's firmware, each servo's firmware and Return
  Delay Time, read statistics and, from firmware 3, why the board last reset. It opens the boards by the USB serials in
  the leader profile and checks them first.
- **Servo ids** are 1 to 7 per board, set one servo at a time with `python/scripts/openrb_setup_arm.py`.
- **Return Delay Time 0** shortens each seven-servo read by about 3.5 ms. It lives in the servos and survives a
  reflash: `python scripts/openrb_leader_setup.py return-delay 0` shows what would change, and `--confirm` writes it.
  The firmware refuses it while force feedback is on.
- **Power.** Force feedback on more than one or two servos needs the 5 V supply on the OpenRB rail. Unplugging USB
  does not reset a board that has that supply on: power it off too.
- **USB identity.** Helm tells left from right by USB serial number, not by COM port. The serials are pinned in the
  leader profile `config/devices/gello-dual-viewer.json` (`leaders.left.usb_serial`, `leaders.right.usb_serial`), which
  every teleop profile names, and in the bench identity file `config/devices/minerva-leaders-01.json`. The boards
  enumerate as USB `2F5D:2202`. Read your boards' serials from Device Manager or `python -m serial.tools.list_ports -v`,
  move only the left leader to confirm which is which, and put the serials in both files
  ([NEW_UNIT_PROFILE.md](NEW_UNIT_PROFILE.md)).
- **Ranges and zero.** Minerva's leaders are calibrated in Helm's **Leaders** tab (sweep each channel's travel, then
  set the leader zero), saved as `gello-dual-leader-ranges.json` (the repository's copy is `config/devices/gello-dual-leader-ranges.json`; on the
  Jetson Helm keeps its own in `~/minerva/helm/operator/`). The
  AIZEE-era `openrb_calibrate.py` is not part of Minerva's calibration. Order: [CALIBRATION.md](../operator/CALIBRATION.md).

Close any leader viewer before a Helm session: only one program holds a leader's USB port.

## 4. Pedals

The three-key USB pedal sends the letters A (left), B (middle) and C (right) as keyboard keys, so it needs no driver
and no setup here beyond being plugged into the PC whose browser has the Helm page focused. They act on the page that
has focus; they are not a hardware stop. What each pedal does is in [HELM_PEDALS](../history/features/HELM_PEDALS.md)
and the [Helm manual](../operator/HELM_MANUAL.md); [SAFETY.md](../operator/SAFETY.md) says what stops what.

## 5. The headset (Meta Quest Pro)

The Quest steers the head through Helm's `/xr` page and needs to trust Helm's self-signed certificate once.

1. Join the headset to Minerva's Wi-Fi (the access point `minerva` from the network service, or a network both are on).
2. In the Quest's browser open `https://<robot-address>:8760/xr`. The certificate names the addresses the Jetson had
   when it was made; if the wired address changed since, run `helm_service certificate` on the Jetson first.
3. Accept the certificate (compare its SHA-256 with `helm_service status`), log in, then start head tracking while
   looking straight ahead and level.

More on the headset, including slow links, is in [REMOTE_HEADSET.md](../operator/REMOTE_HEADSET.md) and
[NECK_FEETECH.md](../robots/minerva/NECK_FEETECH.md).

## 6. The browser

Any current browser on the tether or the Wi-Fi works: open `https://10.42.0.1:8760/`, accept the certificate once,
and log in with Helm's password. One device at a time is in control; any logged-in device can Disable (H) or Shut down
(X).

## Not covered here

- The firewall prompt Windows may show for the bridge's Python the first time a session starts: the leader link is a
  UDP socket the robot sends its requests to. Allow it for the network the tether is on. How the prompt looks on a
  fresh install has not been recorded.
- Using the station over Tailscale or another remote route. The deploy relay works over it; teleop over Wi-Fi is
  allowed, and Helm's GELLO pill and the session start say so, because late samples make the arms hold.
