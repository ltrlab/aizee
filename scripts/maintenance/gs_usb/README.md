# Minerva RH-02 host-driver investigation

This directory preserves a reproducible **candidate** for the production
Jetson's Linux `gs_usb` module. It is separate from adapter firmware and robot
control. The candidate has compiled and passed source review. Installation and
a clean reboot were verified on 2026-09-25 UTC. Module loading and detailed CAN
error reporting were verified after reconnection on 2026-09-26 UTC. After the
operator reseated the missing left gripper, a 60-second stopped test completed
211 full scans with no echo anomalies, CAN errors, drops or reconnects. See the
[short-pass record](../../../docs/devices/MINERVA_CAN_DRIVER_SHORT_PASS_20260926.json).
This bounded pass does not establish endurance reliability or control-loop
performance. Both CAN links were left down.

See [the backport record](../../../docs/devices/MINERVA_CAN_DRIVER_BACKPORT_20260925.md)
and [field research](../../../docs/devices/MINERVA_CAN_FIELD_RESEARCH_20260925.md).
Original Linux SPDX/copyright notices are retained; the patched driver is
GPL-2.0-only. Patch/source hashes and primary-source URLs are in
`source-manifest.json`.

## Reproduce the source and build

On a Linux build host with Python, `patch`, make, GCC and matching kernel headers:

```sh
python3 prepare.py --output /absolute/path/to/new-driver-source
make -C /lib/modules/5.15.185-tegra/build M=/absolute/path/to/new-driver-source -j2 modules
```

`--source /path/to/gs_usb.v5.15.185.c` avoids a download. Preparation verifies
the baseline, every patch and the final source hash. It refuses existing output
directories. Building does not load or install the result. Binary metadata can
differ with build paths; the device recipe below accepts only the already
reviewed/staged binary, not arbitrary rebuilds.

The five separate patches add transfer cleanup/length checks, anomaly logging,
balanced receive-request lifetime, advertised CAN bus-error reporting, and
bounded USB-header tracing. Both detailed bus-error reporting and header tracing
remain off unless explicitly enabled. The original driver's combined
listen-only/loopback behavior remains unsuitable for a connected-arm isolation
test; do not assume netlink mode flags guarantee physical silence.

## Install the pinned candidate on Minerva

The bundle is staged at
`/home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/`.
The module is in the sibling `candidate-full/` directory. Keep motor power off
and adapter USB disconnected. Controller services must be stopped and disabled.
Leave both adapters unplugged through installation and reboot; reconnecting
them after a resistance measurement will make the installer refuse maintenance.

```sh
python3 /home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/maintain_minerva.py check
sudo python3 /home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/maintain_minerva.py install
```

Installation verifies device/kernel identities and hashes, saves an additional
original copy, writes only `updates/aizee/gs_usb.ko`, rebuilds module lookup and
checks the selected file. It preserves the original module and does not change
services, motor settings, or the driver already loaded in memory. It restores
the prior disk selection if installation validation fails.

Then perform a clean reboot with `sudo reboot`. Do not hot-unload the old driver:
the investigated receive-request lifetime defect makes that a poor transition.
After SSH reconnects, verify the selected module and stopped controller services.
Reconnect the adapters, then verify that the module loads and exposes its new
`diag_frames` parameter before running a bounded stopped-only test. A module
that has not loaded while all adapters are unplugged is expected.

### Windows password-authenticated retry

Windows OpenSSH can reject the workspace SSH key when its ACL includes the
shared sandbox group (`UNPROTECTED PRIVATE KEY FILE`). If password login works,
use this command from PowerShell; it skips public-key authentication without
changing the key file or disabling server identity verification:

```powershell
ssh -t -o PubkeyAuthentication=no ltr@192.168.8.180 "sudo python3 /home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/maintain_minerva.py install && sudo reboot"
```

Enter the Jetson login and sudo passwords only in that terminal. Keep motor
power off and both adapter USB plugs disconnected. An `Unplug adapter USB`
preflight failure happens before installation, and `&&` prevents reboot after
that failure. Do not remove the disconnect check or hot-unload the driver to
bypass it.

## Diagnostics and rollback

Once the candidate is loaded and both CAN interfaces are down:

```sh
sudo python3 /home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/maintain_minerva.py trace-on
# Run one bounded stopped-only capture, then leave CAN down.
sudo python3 /home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/maintain_minerva.py trace-off
```

Tracing is capped at 16,384 header events per interface/open. It adds overhead
and is not a throughput benchmark. No persistent modprobe option or sudo grant
is installed. BERR remains an independently selected, temporary capture mode.

For rollback, unplug adapter USB again, keep motor power off, and run:

```sh
sudo python3 /home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/maintain_minerva.py rollback
sudo reboot
```

Rollback removes only the hash-verified override and selects the preserved
original. It refuses an unknown replacement. Records are retained in
`installation-records/` on the pinned Jetson. Sudo authentication happens on the
Jetson; passwords should never be supplied in chat.
