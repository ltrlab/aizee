# Minerva CAN driver backport — 2026-09-25 UTC

## Status and purpose

A focused diagnostic backport is built for the production Jetson's
`5.15.185-tegra` kernel. It addresses concrete deficiencies found in the installed
driver and adds bounded diagnostics to identify the remaining transport fault.
The full candidate compiled successfully as an ordinary user and passed an
independent code review with no actionable findings. The operator installed it
at **03:39:23 UTC** and rebooted. A read-only check at **03:40:46 UTC** verified
the new boot, exact candidate selection/hash, preserved original and backup,
and inactive/disabled controllers. See the
[installation and boot evidence](MINERVA_CAN_DRIVER_INSTALL_20260925.json).

**Latest bounded result, 2026-09-26 00:05 UTC:** after the operator reseated the
left gripper connections, all 14 motors passed the pinned checks and **211 full
stopped scans completed in 60.124 seconds**. Each bus transmitted and received
2,982 packets with zero echo anomalies, CAN error frames, kernel warnings or
counter drops, including cleanup. BERR was active during the test; its original
OFF state and both links DOWN were restored. See the
[short-pass evidence](MINERVA_CAN_DRIVER_SHORT_PASS_20260926.json). This is a
clean bounded comparison, not proof of the original fault's cause, endurance
reliability, or control-loop performance. The stopped reader deliberately spaces
requests and takes about 284 ms per full scan.

**Earlier, 2026-09-26 00:01 UTC:** after adapter reconnection, the patched module loaded
successfully with tracing off. Detailed CAN error reporting enabled on both
interfaces. The stopped diagnostic aborted before its scan loop: left IDs 4–9
verified, but left gripper ID 10 did not answer its identity queries. Right was
not queried. No echo anomalies, CAN error frames or counter drops occurred in
that brief exchange; this does **not** establish that the original transport
fault is fixed. Both modes were restored and links left down. See the
[bounded retest evidence](MINERVA_CAN_DRIVER_RETEST_20260926.json). This earlier
missing-gripper check did not enter the scan loop.

See the [installed binary audit](MINERVA_CAN_DRIVER_AUDIT_20260924.md) and
[production log](MINERVA_PRODUCTION_01.md) for the preceding evidence and the
current deployment state.

## Source match and provenance

The baseline is the official stable Linux
[v5.15.185 gs_usb.c](https://github.com/gregkh/linux/blob/v5.15.185/drivers/net/can/usb/gs_usb.c).
It was compiled as an external module using the Jetson's installed kernel build
directory and Ubuntu GCC 11.4.0. Its complete disassembly matches the installed
module: **1,410 instruction records and 201 relocation records are identical**.
This establishes a matching executable implementation. It does not recover the
original build command or establish identical module metadata.

| Artifact | SHA-256 |
| --- | --- |
| Installed original module | `8ff2f6fadea5b591a20976fa7114b2823e4c10aa048a979b347f9356acc0105e` |
| Official baseline source | `d1ee028fd35e29f1c120577f62d1c3d670731b3b5104f4107278d061a7e5ac06` |
| Rebuilt unmodified baseline module | `50fdc2a62f14a92fe581bcc613f887bbcdee02a7c7886130fa423e61fa5190a7` |
| Full candidate source, patches 1–5 | `b5ce09c3435ae69cb833dce655a8c1efd9df6e42da206d0e0d17fd671efb7a32` |
| Full candidate module, patches 1–5 | `e3153247afcc9c2468c353015fc01f5821fc6bef430c1e6d2d79eae210269323` |

Local artifacts are under
`simulation_runs/gs-usb-backport-20260925/`. The source manifest records downloaded
source and patch hashes; `baseline-comparison.json` records the binary comparison;
`patch-check.json` records clean sequential patch application and an exact match
to the generated full candidate source. Build logs, module artifacts, and build
receipts are kept in each compiled variant's directory.

The original GPL-2.0-only SPDX line and copyright notices are retained. Upstream
patch files, including authorship and sign-off information, are preserved beside
the adaptations. The optional bus-error mapping uses the protocol defined by the
pinned `candleLight_fw` revision
`85e5b3352db7f051aa879eeb71555dfce664c44c`; it is a local protocol adaptation, not
represented as an upstream Linux backport.

## Patch sequence

Each patch applies after the preceding numbered patch. Separate source variants
allow the effect of later additions to be reviewed independently.

| Patch | Resulting variant | Behavior |
| --- | --- | --- |
| `0001` | `fixes/` | Failed USB TX cleanup; validate received header and payload lengths |
| `0002` | `candidate/` | Rate-limited anomaly logs with USB status, lengths, echo IDs, CAN IDs, and active slots |
| `0003` | `candidate-lifecycle/` | Re-anchor resubmitted RX URBs, unanchor failed submissions, reject absent channel pointers |
| `0004` | `candidate-berr/` | Expose bus-error reporting only when firmware advertises it; count received bus-error frames |
| `0005` | `candidate-full/` | Optional bounded USB header trace; disabled by default |

### Transfer cleanup and length checks

Patch 1 adapts upstream
[516a0cd](https://github.com/torvalds/linux/commit/516a0cd1c03fa266bb67dd87940a209fd4e53ce7),
[6fe9f32](https://github.com/torvalds/linux/commit/6fe9f3279f7d), and
[395d988](https://github.com/torvalds/linux/commit/395d988f93861101ec89d0dd9e3b876ae9392a5b)
to the existing Classical-CAN-only driver. It uses that driver's 12-byte header
and existing CAN length rules. RX data copying is limited to bytes actually
received. A short header is rejected before channel, echo ID, or CAN ID access.

Unlike current-mainline driver code, this version allocates coherent USB TX
buffers. Its callback therefore continues to call `usb_free_coherent()` exactly
once on both successful and failed completion. The backport does not replace
that lifetime model with `URB_FREE_BUFFER` or copy an incompatible contemporary
driver into the old kernel.

### RX transfer lifetime

Patch 3 adapts the paired upstream lifecycle fixes
[7352e1d](https://github.com/torvalds/linux/commit/7352e1d5932a0e777e39fa4b619801191f57e603)
and [79a6d1b](https://github.com/torvalds/linux/commit/79a6d1bfe1148bc921b8d7f3371a7fbce44e30f7).
The USB core removes an URB from its anchor before calling the completion
handler. Resubmission must add it back so interface close can cancel outstanding
RX work. A failed resubmission must remove it again so cleanup does not wait
forever on a request that was never submitted. Failure logging uses the parent
USB device, which exists even if a short header prevented selecting a CAN device.

**Unproven lead:** orphaned RX work across repeated interface closes could
contribute to stale data or buffer reuse. This fits some observations, but no
capture yet establishes it as the root cause. Both lifecycle fixes are included
together; applying only the first introduces the cleanup failure corrected by
the second.

### Bus-error reporting

The pinned firmware advertises `GS_CAN_FEATURE_BERR_REPORTING` at bit 12 and uses
that bit in the requested start flags to enable detailed bus errors. Patch 4
maps this to Linux `CAN_CTRLMODE_BERR_REPORTING` only for advertising devices. It
is **off by default** and must be explicitly requested with `berr-reporting on`
while configuring a stopped CAN interface. Applications still need a CAN error
subscription to receive the error frames.

This improves visibility into ACK, bit, form, stuff, and CRC faults; it does not
change termination, bit timing, motor parameters, or command semantics. With a
noisy bus it can increase receive traffic, so comparison trials should record
whether it was enabled.

### Bounded USB trace

Patch 5 adds the root-writable module parameter `diag_frames`:

- Default `0`: no normal-frame trace.
- On each CAN interface open, the driver sets that channel's event budget to
  `min(diag_frames, 16384)`.
- TX submission, TX completion, and RX completion each consume an event.
- Records include completion status, transfer length, echo ID, CAN ID, channel,
  DLC, RX flags, and current active-slot count.
- RX records require a validated complete header. The trace never reads payload
  bytes or the baseline driver's uninitialized TX flag fields.
- An atomic budget prevents parallel callbacks from exceeding the allowance.
  The events can be interleaved across CPUs; log order alone is not proof of
  device wire order.

Set the parameter before the next bounded diagnostic session. Reset it to `0`
afterward. Enabling it during an already open session does not replenish the
event budget. The cap can be exhausted before a late failure, and console/journal
logging can affect timing; a trace run is not a performance qualification.
Anomaly logging remains rate-limited independently of this optional trace.

## Build and compatibility checks

Example ordinary-user build, with the variant already staged on the Jetson:

```sh
make -C /lib/modules/5.15.185-tegra/build M=/absolute/path/to/candidate-full modules
```

The installed build tree resolves beneath
`/usr/src/linux-headers-5.15.185-tegra-ubuntu22.04_aarch64/3rdparty/canonical/linux-jammy/kernel-source`.
It has kernel configuration and `Module.symvers`; the build does not install a
module or require root.

Kernel ABI checks are necessary in addition to compilation. The old custom
module carries `5.15.185` vermagic and only twelve recorded CAN symbol CRCs,
whereas a normal rebuild carries `5.15.185-tegra` and records core USB/kernel
symbols too. The runtime and installed-header configurations match except for
three compiler/toolchain settings: compiler version text, GCC version number,
and GCC plugin support. The rebuilt `module_layout` CRC `0x24d702c7` matches
stock CAN and NVMe modules. All **49 candidate symbol CRCs match the build
tree's Module.symvers**, and all **21 shared CRCs match the inspected stock
modules**. Evidence is in `kernel-abi.json` and `staging-check.json`; these static
checks do not substitute for verifying the module actually loads after reboot.

Do not force-load the module, remove version checks, or strip CRC records to
make an incompatible build appear acceptable. Compilation and static ABI checks
do not prove that a module will load or behave correctly on hardware.

## Installation and rollback plan

The reproducible recipe and reviewed installer/rollback tool are in
[scripts/maintenance/gs_usb](../../scripts/maintenance/gs_usb/README.md).
Source reproduction on the Jetson verified the final source hash. **18 offline
installer tests pass**, including identity/hash rejection, failed-install
restoration, rollback without staging files, and USB/DFU guards.

The bundle is staged at
`/home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/`.
The unprivileged check correctly refuses installation while adapters are still
USB-connected. Normal SSH access cannot replace a kernel module: activation
requires the operator to authenticate sudo on the Jetson, using the concrete
commands in the recipe. No password should be sent in chat.

Installation follows these steps:

1. Finish the stopped-reader session, verify both CAN interfaces are down, and
   ensure the robot controller cannot start automatically.
2. Preserve the exact original module outside any overwrite target and verify
   its SHA-256 against the baseline above. Preserve source, candidate hash,
   module metadata, ABI checks, and installation log.
3. Install only the verified candidate with ordinary kernel version checks.
   Resolve module selection with `depmod` and confirm which file `modprobe`
   would select.
4. **Reboot to activate the candidate; do not hot-unload the old driver.** Its
   missing RX anchor handling makes an unload after transport activity a poor
   diagnostic boundary. Reboot also clears old adapter/driver session state.
5. Verify the booted kernel, selected module hash, adapter serial assignments,
   and both stopped interfaces before a short stopped-motor comparison.

Rollback restores the preserved original module selection, reruns `depmod`, and
reboots. Keep a local login path available if network startup or module loading
fails. The original module remains a diagnostic baseline with the deficiencies
documented in the audit; restoring it is not a transport qualification.

Record installation and every subsequent hardware test in the production log.
No powered test or teleoperation approval follows solely from this backport.
