# Fresh Jetson Orin Nano Setup

From a blank SD card to a fully validated robot brain. The interactive version
of this checklist lives on the robot itself once the heartbeat service is up:
**http://\<jetson\>:8088/setup**.

## 0. What you need

- Jetson Orin Nano + SD card (or NVMe), JetPack **6.x**
- The USB-CAN adapter (gs_usb, 1 Mbps) — it has a physical power switch
- This repo checked out on the dev machine, git-bash available
- The `aizee_rover_id` SSH keypair (`ssh-keys/` in the Workspace); a fresh key
  also works — the bootstrap installs whatever `SSH_KEY` points at

## 1. Flash JetPack

Flash with SD Card Formatter + Balena Etcher (SD image) or NVIDIA SDK Manager.
During first-boot **oem-config**:

- user: `ltr` (everything — services, paths, sudoers — assumes it)
- connect WiFi or ethernet: the bootstrap needs internet on the device
- hostname: anything; the bootstrap can set `aizee-jetson` for you

USB-C to your PC gives a fallback network path at `192.168.55.1`
(JetPack device mode) before any of our networking is configured.

## 2. Bootstrap (one command from the dev machine)

```bash
./scripts/bootstrap_jetson.sh ltr@192.168.55.1 -- --ap-pass '<wifi-psk>' --hostname aizee-jetson
```

This installs your SSH key (one password prompt), syncs `rust/ config/
scripts/ python/ firmware/` to `~/aizee`, and runs `scripts/setup_jetson.sh`
on the device, which does — idempotently:

| step | what |
|---|---|
| apt | `can-utils i2c-tools v4l-utils python3-pip python3-dev python3-opencv python3-smbus build-essential pkg-config libzmq3-dev curl git` |
| groups | adds `ltr` to `i2c dialout video` |
| rust | rustup (minimal) if `cargo` is missing |
| python | `pip install --user -r requirements_jetson.txt`; tries `pyrealsense2` (warn-only) |
| CAN | `/usr/local/bin/aizee-reset-usb-can` + `/etc/sudoers.d/aizee-can` |
| udev | all of `config/udev/` (lidar rules only with `--with-lidar`) |
| systemd | installs **all** `aizee-*` units, enables boot + device-bound services |
| build | `cargo build --release` for motor_control (several minutes the first time) |
| network | WiFi AP `aizee` @ `192.168.50.1` (`--ap-pass`), optional USB-C share @ `10.42.0.1` (`--usb-eth <iface>`) |

Re-run any time: `./scripts/bootstrap_jetson.sh` (auto-detects the reachable
address). For code-only refreshes use `./scripts/deploy_jetson_rover.sh`.

> **Re-flashed device?** The SSH host key changed — run
> `ssh-keygen -R <address>` first.

## 3. Wire the hardware

- **CAN**: adapter powered ON, motors chained, 30 V pack on. Every motor listed
  in `config/hardware_jetson_rover.yaml` must be on the bus or `motor_control`
  wedges during init. Wheels physically absent → set `motors.wheels: []`
  (keep the key — the Rust config parser requires it).
- **USB**: gripper cam (ELP), scene cam (RealSense, optional — its absence just
  means "rover mode" v4 episodes), Tufty display (optional), e-stop receiver
  (optional). Order doesn't matter; udev symlinks + starts services on plug-in.
- **UPS**: INA219 on i2c bus 7, addr 0x41.

## 4. Validate

Open **http://192.168.50.1:8088/setup** (AP) or `http://10.42.0.1:8088/setup`
(USB-C share) and hit *Run all checks* — python deps, rust build, services,
CAN state, device symlinks, live motor/UPS/camera telemetry, network. Every
failing check carries its own fix hint.

Headless equivalent over SSH (exit code 1 on failure):

```bash
ssh ltr@10.42.0.1 'python3 ~/aizee/python/tools/setup_checks.py'
```

Old-school one-shot from the dev machine: `./scripts/check_rover_status.sh`.

## 5. Teleop smoke test (dev machine)

```bash
python python/scripts/collect_demo.py --gui
```

`E` engage → arm mirrors the leader → `R` record a short episode → `Q` quit.
Episode lands in `data/episodes/`. If the arm doesn't engage, check the
heartbeat dashboard first: E-stop tile, motor table, then service logs.

## Known sharp edges

- **USB-CAN adapter drops off the bus** occasionally (gone from `lsusb`) —
  power-cycle it with its physical switch; software can't recover it.
- **pyrealsense2 has no reliable aarch64 wheel** — if pip fails the scene cam
  stays off until you build it: `scripts/build_librealsense_rsusb.sh`.
- **Group membership** (i2c/dialout/video) applies to *new* logins; systemd
  services are unaffected, but interactive debugging needs a re-login after
  first setup.
- Shell scripts synced from Windows carry CRLF; `setup_jetson.sh` strips them
  for everything it installs, and `.gitattributes` now forces LF at checkout.
