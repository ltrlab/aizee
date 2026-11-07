# Install the Jetson

From a freshly flashed Jetson to Helm running on Minerva. Do the steps in order; each ends with a command that shows
it worked. Nothing here moves a motor. Afterwards continue with the [operator station](INSTALL_OPERATOR_STATION.md),
the [unit's profile](NEW_UNIT_PROFILE.md) and [calibration](../operator/CALIBRATION.md).

Two machines take part:

- **The build PC** builds a verified release and pushes it to the robot. It must be a Windows x64 PC: the Rust
  cross-build helper (`scripts/build_jetson.ps1`) is PowerShell. Set it up as [DEVELOPMENT.md](../DEVELOPMENT.md)
  says (`python scripts/dev.py setup`, Git LFS). Commands below that start with `python` run in `.venv-dev`.
- **The Jetson** runs everything else. The code assumes the Orin Nano developer kit, with JetPack 6 (Ubuntu 22.04,
  Python 3.10, glibc 2.35, kernel 5.15), and an account named `ltr`: the release path `~/minerva`, the systemd units
  and the sudoers rules all name that user, and none of them reads it from a setting.

The repository holds one unit's identity (address, machine id, host key, CAN adapter serials, calibration). Steps 2
and 3 name where to replace it; [NEW_UNIT_PROFILE.md](NEW_UNIT_PROFILE.md) says how.

## What lives where

| Piece | Installed by | Unit or location |
| --- | --- | --- |
| The release (Python platform, Rust receivers, models, profiles) | `robot-platform deploy` | `~/minerva/releases/<id>`, `~/minerva/current` |
| Helm | `ops.helm_service install` (generated) | user unit `minerva-helm.service` |
| Network modes and access point | `ops.network install` (generated) | system unit `minerva-net.service` |
| UPS monitor | `ops.ups install` (generated) | system unit `minerva-ups.service` |
| CPU partition | `ops.cpu_partition install` (generated) | `/etc/minerva/cpu-partition.json` and a `CPUAffinity` drop-in on each camera unit |
| Wrist cameras | by hand, from `config/systemd/` | `minerva-left-wrist.service`, `minerva-right-wrist.service` |
| Head camera (RealSense D435) | `scripts/deploy_scene_cam.sh`, from `config/systemd/` and `config/udev/` | `aizee-scene-cam.service` (the old name stays: it is Minerva's head camera) |
| CAN naming, helpers and sudoers | `scripts/setup_minerva_system.sh` | `/etc/udev/rules.d/80-minerva-can.rules`, `/usr/local/bin/aizee-can-up` and two more helpers, `/etc/sudoers.d/aizee-can` |
| Idle-state unit for the control CPUs | by hand, text in [JETSON_CPU_SPLIT.md](../architecture/JETSON_CPU_SPLIT.md#deep-idle-on-the-control-cpus) | `minerva-control-cpus.service` |

The four units `minerva-helm`, `minerva-net`, `minerva-ups` and the CPU drop-ins are written by the `ops/*.py` modules
themselves; they are not files in `config/systemd/`. That folder holds the camera units and AIZEE's own, and
[the AIZEE guide](../robots/aizee/README.md) sorts which `aizee-*` names Minerva uses.

## 1. The Jetson itself

1. Flash JetPack 6 (L4T R36) and create the user `ltr` during first boot.
2. Install the packages the platform needs, as `ltr` with sudo:

   ```bash
   sudo apt-get update
   sudo apt-get install -y can-utils i2c-tools v4l-utils python3-pip python3-dev python3-opencv python3-smbus \
       build-essential pkg-config libzmq3-dev libudev-dev curl git
   sudo usermod -aG i2c,dialout,video ltr
   python3 -m pip install --user -r requirements_jetson.txt scipy h5py
   ```

   `requirements_jetson.txt` is in the repository root, so copy it to the Jetson first (or run the pip line from a
   checkout). The release's smoke test imports `numpy`, `scipy`, `msgpack`, `zmq`, `h5py` and `PIL` from the system
   Python; the file lists all but `scipy` and `h5py`, so name them as above. Keep numpy below 2: JetPack's
   `python3-opencv` is built against 1.x.

   The `i2c` group is for the UPS monitor's reads, `dialout` for the neck servo adapter, `video` for the cameras. Log
   out and in again for the groups to apply.
3. The head camera also needs `pyrealsense2`. Try `python3 -m pip install --user pyrealsense2`; if there is no wheel,
   build the RSUSB backend with `scripts/build_librealsense_rsusb.sh`.

Verify:

```bash
ssh ltr@<jetson-ip> 'uname -m; python3 --version; id -nG; python3 -c "import numpy, scipy, msgpack, zmq, h5py, PIL; print(numpy.__version__)"'
```

Expect `aarch64`, Python 3.10, `i2c` in the groups, and a numpy 1.x version.

## 2. Reach the Jetson with a pinned key

Helm, the deploy and Minerva Check log in over SSH and check two things before they run anything: the host key's
SHA-256 fingerprint and `/etc/machine-id`. Both come from the repository's profile
(`config/devices/minerva-joint-calibration.json`, `connection.host_key_sha256`, and `config/devices/minerva-production-01.json`,
`machine_id`), so a different Jetson is refused until the profile names it.

1. Make a key pair on the build PC and put the public half in the Jetson's `~/.ssh/authorized_keys`.
2. Point the tools at the private key. The profile's `connection.key_file` is a relative path nobody else has, so
   override it for the process:

   ```powershell
   $env:AIZEE_SSH_KEY_FILE = "<path to the private key>"
   ```

   `AIZEE_SSH_KEY_FILE` must name an existing file or the tools stop. The `scripts/*.sh` helpers use their own
   `SSH_KEY` (searched under a few default paths) and `AIZEE_TARGET` (`user@host`, tried in turn against `10.42.0.1`,
   `192.168.50.1` and `192.168.0.27` when unset); set both when the Jetson is elsewhere.
3. Read the Jetson's identity and write it into the unit's profile ([NEW_UNIT_PROFILE.md](NEW_UNIT_PROFILE.md)):

   ```bash
   ssh-keyscan -t ed25519 <jetson-ip> | ssh-keygen -lf -
   ssh ltr@<jetson-ip> cat /etc/machine-id
   ```

Until step 6 installs the network service, the Jetson is not yet at `10.42.0.1`. Use its address on whatever network
it is on, or the JetPack USB device-mode address `192.168.55.1`, and pass it as `--address` to every command below.

Verify: `ssh -i $env:AIZEE_SSH_KEY_FILE ltr@<jetson-ip> true` exits 0.

## 3. CAN adapters, helpers and sudoers

`scripts/setup_minerva_system.sh` installs, with sudo on the Jetson:

- the helpers `aizee-can-up`, `aizee-reset-usb-can` and `aizee-can-reset` in `/usr/local/bin`;
- `config/udev/80-minerva-can.rules`, which names the two USB CAN adapters `canL` and `canR` by their USB serial
  numbers and turns USB autosuspend off for them;
- `/etc/sudoers.d/aizee-can`, which lets `ltr` bring `canL` and `canR` up and down without a password;
- a stop and disable of the retired `aizee-motor-control-rover` service, which would hold a bus.

It starts no motor service. Two things are specific to the unit and need editing first:

- **The adapter serials** are in the udev rule and again in the script's autosuspend loop. Replace both pairs with the
  new adapters' serials (`udevadm info -a -n /dev/<adapter>`, or `lsusb -v` and the `iSerial` line), and the same
  serials in `config/devices/minerva-production-01.json` under `buses`. Which adapter is left and which is right is
  fixed by unplugging one and seeing which stays.
- **The Jetson's sudo password** is read from `scripts/.jetson_password`, a git-ignored file you create with that one
  line in it. The script refuses to run without it. It also sends the password to `sudo -S` over SSH; delete the file
  afterwards, or run the same commands by hand.

```bash
export SSH_KEY=<private key>
scripts/setup_minerva_system.sh ltr@<jetson-ip>
```

Verify, with both adapters plugged in:

```bash
ssh ltr@<jetson-ip> 'ip -br link show canL canR; sudo -n -l /usr/local/bin/aizee-can-up'
```

Both interfaces exist (down is right), and `sudo -n -l` prints the command instead of asking for a password.

## 4. Build and deploy the first release

Cross-build the AArch64 binaries once, then deploy:

```powershell
git lfs pull
./scripts/build_jetson.ps1 -Bootstrap                        # first time only: downloads pinned tools
./scripts/build_jetson.ps1 -Offline -Package robot_control, robot_collision
python -m robot_platform deploy --address <jetson-ip>
```

`git lfs pull` brings the CAD meshes. The build checks the collision model against them and records their digests, and
it refuses to build without them. The meshes themselves never go into the release.

`robot-platform deploy` (see `--help`): builds the release, uploads it to `~/minerva/incoming`, unpacks and verifies
it against its own manifest, switches `~/minerva/current` to it atomically, runs the robot-side smoke test, and prunes
to the last three releases (`--keep N`). It uses user space only: no sudo, no services, no compiling on the Jetson.
When Helm's service is installed it then installs Helm's dependencies and meshes and restarts Helm, unless a session or
Minerva Check is running; `--no-helm` skips that. `--build-only DIR` writes the archive and stops.

The smoke test opens no device and no CAN interface. It imports the required modules, loads the calibration and
collision model, times a keep-out query and runs the receiver harness disarmed.

Verify:

```bash
ssh ltr@<jetson-ip> 'readlink ~/minerva/current; tail -n 2 ~/minerva/deploy.log'
```

**Rollback.** If the smoke test fails, the deploy switches back to the previous release by itself and says so. To go
back by hand, switch the link the way the deploy does, then restart Helm when no session runs:

```bash
ssh ltr@<jetson-ip> 'cd ~/minerva && ln -sfn releases/<previous id> current.new && mv -Tf current.new current'
python -m robot_platform.ops.helm_robot --address <jetson-ip> remote restart
```

Deploying an older checkout also works: a release whose content is already on the robot is switched to, not
uploaded again. No script wraps the manual switch.

## 5. Cameras

The camera units run from a checkout at `~/aizee` (their unit files name `~/aizee/python/nodes/...` and
`~/aizee/config/hardware_*.yaml` as absolute paths), not from the release. Ports: left wrist 5563, head 5564, right wrist 5565
([HARDWARE.md](../robots/minerva/HARDWARE.md#network-ports)).

- **Head camera.** `scripts/deploy_scene_cam.sh ltr@<jetson-ip>` copies the shared Python package and the camera node
  into `~/aizee`, installs the udev rule and `aizee-scene-cam.service`, and runs a stream self-test. Like step 3 it
  reads `scripts/.jetson_password`.
- **Wrist cameras.** No script installs these. Copy `python/nodes/gripper_camera_node.py` and the two
  `config/hardware_minerva_{left,right}_wrist.yaml` files into `~/aizee`, then:

  ```bash
  sudo cp config/systemd/minerva-left-wrist.service config/systemd/minerva-right-wrist.service /etc/systemd/system/
  sudo systemctl daemon-reload && sudo systemctl enable --now minerva-left-wrist minerva-right-wrist
  ```

  Each node finds its camera by USB id, so `/dev/videoN` renumbering does not matter. The ids are in the two YAML
  files and, for Helm, in `src/robot_platform/helm/cameras.py`.

Verify:

```bash
ssh ltr@<jetson-ip> 'systemctl is-active minerva-left-wrist minerva-right-wrist aizee-scene-cam; ls -l /dev/aizee_scene_cam'
```

Helm's camera cards (Settings) show the three streams once Helm runs.

## 6. Network

`minerva-net` makes the Ethernet port find its own mode (a network, or a PC's USB Ethernet adapter at 10.42.0.1),
keeps 10.42.0.1 on a dummy interface, and can run the access point `minerva`. The modes are in
[MINERVA_NETWORK.md](../architecture/MINERVA_NETWORK.md). Install it on the Jetson with sudo:

```bash
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.network install
```

It asks twice for the access point's password and writes root-owned copies under `/usr/local`, the NetworkManager
profiles, the sudoers rule Helm needs to switch the access point, and the service. It also turns off autoconnect on
the old `Wired connection 1` and `aizee-ap` profiles and disables the old tether's dnsmasq. `sudo minerva-net remove`
restores what `before.json` recorded. Its interface names (`enP8p1s0` Ethernet, `wlP1p1s0` Wi-Fi) are constants in
`ops/network.py`: on a Jetson whose ports have other names, check `ip -br link` first.

On a network cable the Wi-Fi client steps aside when the service starts, so a session over Wi-Fi ends: reconnect on
the wired address.

Verify: `ssh ltr@<jetson-ip> 'cd ~/minerva/current/src && python3 -B -m robot_platform.ops.network status'` (no sudo)
shows the mode, and `ping 10.42.0.1` answers from a PC on the tether.

## 7. UPS monitor

Minerva's UPS is a Waveshare UPS Power Module (C) read over I2C bus 7. The service publishes its state to Helm and
shuts the Jetson down cleanly before the battery runs out ([UPS_MONITOR.md](../robots/minerva/UPS_MONITOR.md)).

```bash
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.ups install
```

It copies itself and `config/devices/minerva-ups.json` to `/usr/local/lib/minerva-ups/` (a root service must not run
files its user can edit), stops and disables AIZEE's `aizee-ups-monitor`, and waits for a fresh reading. Editing the
profile later needs another install.

Verify: `python3 -B -m robot_platform.ops.ups read` prints one reading (no sudo), and `status` shows the service
active.

## 8. CPU partition

The arms' loop and receiver keep to CPUs 4-5, the head's receiver to CPU 3, and the cameras, Helm and everything else
to 0-2 (the reason and the evidence are in [JETSON_CPU_SPLIT.md](../architecture/JETSON_CPU_SPLIT.md)). Stop any
Helm session first: the cameras restart.

```bash
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.cpu_partition install
```

It gives the three camera units a `CPUAffinity` drop-in, checks every camera thread now runs on the background CPUs,
and only then writes `/etc/minerva/cpu-partition.json`. Install the camera units (step 5) first; it covers
`aizee-scene-cam`, `minerva-left-wrist` and `minerva-right-wrist`. It refuses when CPUs 3-5 are offline, which a
low power mode can cause.

Also install the `minerva-control-cpus` unit from the CPU-split page, which keeps CPUs 4-5 out of the deep idle state
whose 5 ms wake delayed the receiver's ticks. It is not in the repository as a file.

Verify: `python3 -B -m robot_platform.ops.cpu_partition status` lists each camera's threads on the background CPUs and
the partition in place.

## 9. Helm

Helm runs on the Jetson as the user service `minerva-helm` (`helm --on-robot`): HTTPS on port 8760 for the operator's
devices, plain HTTP on `127.0.0.1:8762` for the robot's own tools and SSH tunnels, a password login, and one device in
control at a time ([HELM_ON_JETSON.md](../architecture/HELM_ON_JETSON.md)). It runs the teleop profile
`config/devices/minerva-teleop-direct-tuning.json` (100 Hz, collision checking off) with
`config/devices/minerva-task-capture.json`.

1. **Dependencies and meshes**, from the build PC. The Jetson never downloads anything; the build PC fetches the
   wheels in `requirements/helm-robot.lock` for the Jetson's Python with their hashes checked and uploads them, and
   converts the CAD meshes for the 3D view:

   ```powershell
   python -m robot_platform.ops.helm_robot --address <jetson-ip> deps
   python -m robot_platform.ops.helm_robot --address <jetson-ip> meshes
   ```

   Both skip what is already there. `deps` also installs the remote headset worker's lock
   (`requirements/helm-webrtc.lock`); Helm runs without it and says so if that part fails.
2. **Folders, Helm's key and the certificate**, on the Jetson, from the active release:

   ```bash
   cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service prepare
   PYTHONPATH=src python3 -m robot_platform.ops.helm_service authorize-local
   ```

   `prepare` makes `~/minerva/helm/{keys,tls,deps,operator,runs,logs}` and `~/minerva/meshes`, an SSH key Helm uses to
   reach this same computer, and a self-signed certificate (825 days, not a CA) naming the tether, loopback, the
   profile's address, every address the network service reports and the host name. `authorize-local` adds one line
   to `~/.ssh/authorized_keys` that lets only that key in, only from loopback, with no terminal or forwarding; a dated
   copy of the file is kept. If the Jetson's wired address changes later, `helm_service certificate` issues the
   certificate again and each device accepts it once more.
3. **Lingering and the password**, once, by the person who owns the robot:

   ```bash
   sudo loginctl enable-linger ltr
   cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.helm.access set-password ~/minerva/helm/access.json
   ```

   Without lingering the unit stops when the last login ends. The password is stored as a scrypt hash and is the
   operator's to choose; Helm will not start without it (the unit exits 78 and does not restart until fixed).
4. **Install and start the unit:**

   ```bash
   PYTHONPATH=src python3 -m robot_platform.ops.helm_service install
   ```

   It writes `~/.config/systemd/user/minerva-helm.service` (CPUs 0-3, nice 5), enables and restarts it, and waits
   until Helm listens or says what is missing. `restart` after a deploy, `status` for the state, `remove` to take
   the unit and the keys it added away.

Verify:

```bash
PYTHONPATH=src python3 -m robot_platform.ops.helm_service status
curl -k -sI https://127.0.0.1:8760/ | head -n 1
systemctl --user is-active minerva-helm
```

`status` should show `listening: true`, the password and key present, dependencies installed, no meshes missing and
`missing: []`. The `curl` answers with a redirect to the login page. From a browser on the tether open
`https://10.42.0.1:8760/`, accept the certificate once (compare its SHA-256 with `status`'s), and log in.

Helm stays down for the few seconds of a deploy and logs every device out when it restarts. Logs are in
`~/minerva/helm/logs/helm.log`.

## 10. Connect the operator station, then check

The GELLO leaders live on a PC, not on the Jetson. [Install the operator station](INSTALL_OPERATOR_STATION.md), then
authorize its bridge key on the Jetson:

```bash
cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service authorize-bridge --name <pc-name> --key "<public key line>"
```

Finally run the go/no-go from the build PC. It checks the PC's calibration, the tether, the release and the Jetson's
health, then reads every motor while it stays disabled:

```powershell
python -m robot_platform check --address <jetson-ip>
```

On a new robot it will report the missing calibration files, which is expected: the order for making them is in
[CALIBRATION.md](../operator/CALIBRATION.md). The same check is the first tab in Helm.

## Remaining manual items

- `minerva-control-cpus.service` and the `aizee-heartbeat` CPU drop-in (an optional AIZEE-era dashboard on port
  8088) are documented, not shipped as files.
- `ops/deploy.py`, `ops/network.py` and the units fix the account name `ltr` and the path `~/minerva`; `ops/release.py`
  builds for the one calibration profile named in it. A second unit shares the code but needs these constants looked at.
