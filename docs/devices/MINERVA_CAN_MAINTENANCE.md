# Minerva RH-02 USB-CAN maintenance

## Current state

**Latest state, 2026-09-26 00:05:48 UTC (September 25, 20:05 EDT):** after the
operator reseated the left gripper connections, all **14 motor identities and
stopped-state checks passed**. The patched-driver diagnostic completed **211
full scans in 60.124 seconds** with no rejected scans, retries, or reconnects.
Each bus recorded **2,982 TX / 2,982 RX**, including identity preflight.

There were **zero outgoing-command copies, CAN error frames, kernel echo/USB
warnings, or counter errors/drops**, during the run and after cleanup. Detailed
CAN bus-error reporting was active on both interfaces; cleanup restored its
original OFF setting and verified both links down. All motors remained disabled;
no motor enables, parameter/zero writes, or 30-minute test were performed. See the
[bounded short-pass evidence](MINERVA_CAN_DRIVER_SHORT_PASS_20260926.json).

This is a **clean, bounded stopped-reader pass**, not proof of root cause,
endurance, or 25 Hz control readiness. The diagnostic deliberately spaces
requests by 10 ms and reads the full 14-motor roster at approximately **3.5 Hz**.
Median complete scan was 284.3613 ms; p99 285.9727 ms; maximum 292.0312 ms. The
gripper's availability returned after reseating, but this observation does not
identify which connection or component caused its preceding missing replies.

**First patched-driver check, 2026-09-26 00:01 UTC:** the patched
`gs_usb` driver is loaded, with `diag_frames=0`, on the verified post-install
boot. Both adapters retain their pinned serials and firmware at left `1-2.4` /
right `1-2.1`, with USB `power/control=on`. All three controller services remain
stopped and disabled, with no leftover capture process.

The first stopped diagnostic with the new driver had a **60-second cap**, but
stopped during identity preflight before entering its scan loop. Left CAN IDs
**4–9 passed** their identity/status/firmware/run-mode checks (24 replies).
**Left gripper ID 10 did not answer either of the two existing bounded identity
queries.** Left recorded 26 TX / 24 RX; right recorded 0 TX / 0 RX and was
untested in that attempt. Detailed bus-error reporting successfully enabled on both interfaces,
and cleanup restored its original OFF setting and brought both links down.

No unused-echo/USB warnings, outgoing-command copies, CAN error frames, counter
errors, or drops were observed, including cleanup. This short preflight is
**not a full transport pass and does not prove the echo fault is fixed**. There
were no motor enables, parameter/zero writes, or automatic reconnects. This
prompted the operator's power-off check and reseating of the left gripper's
power/CAN connections before the successful bounded diagnostic above. See the
[driver retest evidence](MINERVA_CAN_DRIVER_RETEST_20260926.json).

Earlier in this investigation, the left short USB cable was restored.
Primary-source [field research](MINERVA_CAN_FIELD_RESEARCH_20260925.md) and
[firmware review](MINERVA_CAN_FIRMWARE_RESEARCH_20260925.md) found relevant
upstream precedents, with the known firmware fixes already installed. Before
unplugging, both adapters had runtime autosuspend disabled and zero suspended
time since their reconnect. Shared USB controller/electrical behavior remains
unproven.

The operator initially measured 60 ohms on both buses, then clarified that USB
power remained connected. Those initial readings remain **provisional** in the
history. The operator has now repeated the measurement with **all power off and
both adapter USB plugs removed**, again reporting **60 ohms on each bus**. This
is consistent with the expected aggregate termination resistance; it does not
verify terminator placement, ground reference, or signal quality. No termination
switch change is justified by the resistance readings alone.

A [targeted driver backport](MINERVA_CAN_DRIVER_BACKPORT_20260925.md) now builds
on the Jetson and has passed independent review. The unmodified baseline matches
all 1,410 instructions and 201 relocations in the existing module. The candidate
adds transfer checks/cleanup, receive-request lifetime fixes, optional detailed
CAN errors and bounded USB metadata tracing. All 49 candidate symbol CRCs match
the build tree, and 21 shared symbols match stock modules. Source reproduction
and 18 offline installer tests passed. The successful install receipt is dated
**2026-09-25 03:39:23.485497 UTC**. Post-reboot lookup selects
`/lib/modules/5.15.185-tegra/updates/aizee/gs_usb.ko`, SHA-256
`e3153247afcc9c2468c353015fc01f5821fc6bef430c1e6d2d79eae210269323`.
The original driver and additional backup retain SHA-256
`8ff2f6fadea5b591a20976fa7114b2823e4c10aa048a979b347f9356acc0105e`.
**Installation, reboot, module loading, and the bounded stopped-reader pass are
verified; endurance and 25 Hz powered-operation qualification remain pending.** See the
[installation and reboot evidence](MINERVA_CAN_DRIVER_INSTALL_20260925.json),
also retained in `simulation_runs/gs-usb-backport-20260925/post-install-boot.json`.

The operator's first attempted installation stopped at preflight because
both USB interfaces, `1-2.4:1.0` and `1-2.1:1.0`, were attached. **No driver file
was installed, module unloaded, or reboot performed by that earlier attempt.**
The subsequent successful installation did not hot-unload the old module. The installer
requires disconnected adapters and stopped, disabled controller services, and
preserves the original driver for rollback. See the
[maintenance recipe](../../scripts/maintenance/gs_usb/README.md).

Windows OpenSSH rejected the private key because its ACL included
`CodexSandboxUsers`; the operator successfully used password authentication and
sudo instead. The first installation stopped on the adapter-disconnect guard, not an
SSH or sudo failure. The reported unpowered resistance measurement and the later
attached-interface preflight are separate observations.

**2026-09-24 23:02 EDT:** the receive-provenance diagnostic completed
**211 scans / 60.13 seconds**, but recorded five left and three right outgoing
commands as RX. All eight lacked local-origin/same-socket-confirmation flags;
normal application loopback does not explain them. Cleanup freed five left and
three right outstanding transmissions (TX-drop counters). Detailed bus-error
reporting was rejected by the installed driver (`Operation not supported`), so
the zero error-frame count cannot clear electrical CAN faults. Original modes
and CAN-down state were verified, with no reconnects or motor writes.
See [provenance evidence](MINERVA_CAN_PROVENANCE_20260924.json).

The [original-driver audit](MINERVA_CAN_DRIVER_AUDIT_20260924.md) confirmed three
missing upstream fixes: failed USB transmit cleanup and received header/payload
length validation. They are concrete deficiencies, not a proven root cause.
This prompted the backport with USB completion/length/echo diagnostics,
matching kernel build inputs and a preserved original module for rollback.
The driver override is now installed; adapter firmware and motor settings were
unchanged by this installation. The viewer stays stopped pending transport
validation.

**2026-09-24 22:51 EDT:** removing the short USB cable from left did
not resolve the failure: **94 complete scans / 27.09 seconds**, then the first
missing reply (`left_arm_j5`). Ten outgoing commands appeared as left RX;
ten left TX drops were recorded only when the link was brought down. Right
had no anomaly in this short comparison; no kernel warning was logged. Both
links are down. See [cable-removal evidence](MINERVA_CAN_LEFT_CABLE_REMOVED_20260924.json).

The operator clarified that short USB cables had remained after bypassing the
external hub. Left's cable is now removed; right's remains. USB locations also
changed to left `1-2.4`, right `1-2.1`, so this was not a cable-only comparison.
Earlier references to "direct" USB established external-hub removal, not the
absence of those short cables. Left termination remains operator-reported OFF.
Detailed CAN bus-error reporting was not enabled in the preceding comparisons;
zero error counters alone do not clear the CAN wiring/termination. This prompted
the receive-message flags / CAN-error diagnostic capped at 60 seconds described
above, and an audit of the exact installed driver. No firmware or
driver has been changed in response to this cable-removal result.

**2026-09-24 22:37 EDT:** after the operator changed left termination
(reported "Should be off now", not independently verified), the three-minute
dual check completed **637 scans / 180.09 seconds**, with no rejected scans or
reconnections. It is **not a clean transport pass**: four outgoing command
frames appeared as left receive data; right logged four unused-echo-ID warnings;
both TX-drop counters increased by four at cleanup. The setting may have helped
this finite trial, but one run does not establish causation or a fix. Both CAN
links are down and the viewer remains stopped. See
[termination comparison](MINERVA_CAN_TERMINATION_SHORT_20260924.json).
This prompted isolating the left USB data cable while keeping the recorded ports,
termination setting and CAN wiring unchanged. Further USB-level tracing may be
needed to locate the remaining transmit-echo accounting fault. No extra firmware
or driver changes, and no full 30-minute test, were performed.

**22:31 EDT, before the termination change:** left still lost replies on right's previously
tested USB port, including with right CAN down. The left-only check failed
after **119 scans / 17.26 seconds**, again with ten unexpected outgoing frames
and ten TX drops. Both CAN links and all controller services are now down.
Current ports: left `1-2.1`, right `1-2.2`. Both firmware updates and normal
identities are verified, but the transport fault remains. No full 30-minute
test was run in this update session. This prompted the cable check suggestion;
the operator instead changed termination first. See
[left-only evidence](MINERVA_CAN_LEFT_ONLY_SHORT_20260924.json).

**2026-09-24 22:26 EDT:** both adapters now boot normally with the same
`RH02_fw` candidate and their original serials. Left was backed up twice and
verified by full 128 KiB readback; all bytes outside the programmed pages match
its original backup. All 14 motor identities passed, but the requested short
dual test **failed after 498 scans / 141.34 seconds** on left. Ten outgoing
frames appeared as left receive traffic, followed by ten TX drops; no kernel
warnings or CAN error/bus-off counters accompanied this trial. Both CAN links
were brought down. The operator explicitly declined another 30-minute test;
this test had a three-minute cap and stopped on its first failed scan.
See [left update evidence](MINERVA_CAN_LEFT_UPDATE_20260924.json) and
[short dual test evidence](MINERVA_CAN_DUAL_SHORT_20260924.json).

The matched firmware did not resolve the problem. This prompted a USB-port
comparison while retaining the adapter cables and CAN wiring; serial-based arm
names remained unchanged. The results below distinguish the original host port
from the left adapter/cable/arm path without proving a defective component.

**22:29 EDT topology update:** the operator moved the plugs, rather than directly
swapping them. Verified current paths: left `1-2.1`, right `1-2.2`. Left is now
on right's previously tested port; right is on a third port. The second bounded
comparison failed on **left after 127 scans / 36.45 seconds**, again with ten
unexpected outgoing frames received and ten TX drops. Right stayed clean. No
cable or CAN wiring change was requested. This points away from the original
left USB port, but does not distinguish the left adapter, its cable, CAN branch
or per-device driver behavior. See [port-change evidence](MINERVA_CAN_PORT_CHANGE_SHORT_20260924.json).

**2026-09-24 22:12 EDT:** with both adapters connected directly to the Jetson,
the right-only stopped-reader test completed **30 minutes / 12,731 scans** with
zero rejected scans, reconnections, unexpected outgoing frames received as input,
or kernel warnings. The same firmware/workload through the external hub had
failed after 420 scans / 59.87 seconds. Keep the current direct connections:
`canL` at `1-2.4`, `canR` at `1-2.1`; serial-based naming remains unchanged.
Left firmware is unchanged. See the [direct USB evidence](MINERVA_DIRECT_USB_20260924.json)
and [right firmware update evidence](MINERVA_CAN_RIGHT_UPDATE_20260924.json).

**22:15 EDT dual-arm follow-up:** the full viewer still fails. Left lost replies
after 210 complete scans / 69.83 seconds; its receive log contains ten outgoing
command frames and cleanup recorded ten TX drops. Right also logged unused-echo-ID
warnings during dual-reader startup. One bounded automatic recovery succeeded,
then the session was explicitly closed; both CAN links are down. The second
attempt already contained three unexpected outgoing frames, so its clean exit
does not establish clean transport. See [dual-arm evidence](MINERVA_DIRECT_USB_DUAL_20260924.json).
The right-only pass must not be generalized to the full setup. This prompted
the left backup/update and short retest recorded above. The adapter firmware
is still a candidate, not a proven cure.

The stopped-position feed has repeatedly lost replies while the Jetson reports
`gs_usb: Unexpected unused echo id`. Cycling the CAN links restores replies
temporarily. The direct-port comparison implicates the external USB path but
does not isolate the hub, cable, power or driver interaction. It qualifies this
right-only stopped workload, not dual-arm powered control at 25 Hz. See the
[device log](MINERVA_PRODUCTION_01.md) for captured trials.

Both adapters are identified by the operator's
[RH-02 listing](https://www.amazon.com/dp/B0CRB8KXWL). The
[manufacturer](https://jhoinrch.net/index.php?a=index&aid=124&c=View&m=home)
describes a CANable-derived device shipped with Candlelight and supporting DFU
updates. The [Zephyr board documentation](https://docs.zephyrproject.org/latest/boards/jhoinrch/rh02/doc/index.html)
describes the STM32F072, 24 MHz crystal, BOOT switch and separate termination
switch. Do not substitute RH-02 Plus/Pro/Elite or CANable 2.x firmware: the
product family includes different MCUs. Confirm actual board and flash geometry
before writing; the published RH-02 flash-size information is inconsistent.

| Interface | USB serial | Reported configuration before any update |
|---|---|---|
| `canL` | `004D002C4730511520303650` | `fw_49fb001_2026-02-16` |
| `canR` | `0021002F4D53530920303937` | Empty; revision unknown |

The complete [inventory and build manifest](MINERVA_CAN_ADAPTERS_20260924.json)
contains USB paths, power state, CAN counters and artifact hashes. Do not edit
the hardware/reference profiles merely to record an adapter model: those files
are hashed inputs to the commissioning records.

## Prepared candidate

- Source: [candleLight_fw](https://github.com/candle-usb/candleLight_fw/tree/85e5b3352db7f051aa879eeb71555dfce664c44c),
  clean revision `85e5b3352db7f051aa879eeb71555dfce664c44c`.
- Board target: `RH02_fw`, Release (`-O3`, link-time optimization).
- Build host: pinned `minerva-production-01`; Arm GCC 10.3.1, CMake 3.22.1.
- Firmware version string: `fw_85e5b33_2026-09-17`.
- Binary: 23,740 bytes, flash load address `0x08000000`; 4,200 bytes static RAM
  plus 2,048-byte reserved stack. Payload fits below 64 KiB; this does not by
  itself verify board compatibility.
- `RH02_fw.bin` SHA-256:
  `1509d9ad16d87f949a74a57909b9a9c1d109bb38cacbe324f5014445c6cf400f`.
- Local artifacts: `simulation_runs/rh02-firmware-85e5b3352db7/`.
- Remote stage: `/home/ltr/aizee/device-maintenance/rh02-firmware-85e5b3352db7/`.

Build again from the staged clean checkout (these commands do not flash):

```sh
cd /home/ltr/aizee/device-maintenance/rh02-firmware-85e5b3352db7
git -C source rev-parse HEAD
git -C source diff --exit-code HEAD
cmake -S source -B build -DCMAKE_TOOLCHAIN_FILE="$PWD/source/cmake/arm-none-eabi-gcc.cmake" -DCMAKE_BUILD_TYPE=Release
cmake --build build --target RH02_fw --parallel 2
arm-none-eabi-size build/RH02_fw
sha256sum build/RH02_fw.bin
```

The ELF/map and source bundle are retained for inspection and rebuilding.
The upstream DFU suffix is metadata/checksum, not a cryptographic signature.
Successful compilation does not qualify this firmware for powered operation.

## Controlled update and validation

1. Keep the stopped reader exited, controller services stopped and both CAN
   links down. Work on the **right** adapter first; preserve the left firmware
   for comparison. Label by its arm/serial, not enumeration order.
2. Unplug the right adapter from USB, select its labelled **BOOT** mode, then
   plug it back into the same Jetson USB port. Leave the separate **120 ohm**
   termination switch unchanged. Zephyr documents BOOT down at plug-in; if the
   unit differs or the markings are unclear, identify it before proceeding.
3. Enumerate DFU, bind the selected device to its physical USB path and confirm
   the MCU/flash geometry. The ROM bootloader can report a different USB serial
   from the application; a missing `canR` in boot mode is expected. Check that
   the left adapter is unchanged. No wildcard multi-device flash.
4. Read the existing flash using the discovered geometry, save two matching
   reads with hashes, and retain the original descriptors and rollback steps.
   If read protection prevents backup, stop for review; do not unprotect or
   mass-erase it as a diagnostic shortcut.
5. Only after compatibility and backup are established, write the selected
   candidate and read it back for verification. Return BOOT to normal and
   reconnect. Verify USB identity, firmware string, `canR` naming, bit timing,
   and all seven pinned motor identities/firmware in stopped state. A changed
   adapter identity must be resolved before any reader/control launch.
6. Run a sustained stopped-position test. Record completed/rejected scans,
   latency distribution, kernel echo warnings, USB errors and CAN counters.
   Respect the operator's current limit: comparisons are capped at three
   minutes and stop on the first failed scan, without automatic reconnection.
   Longer endurance qualification remains pending; the operator has explicitly
   declined another full 30-minute run. Then test deliberate reader
   start/stop and USB reconnect recovery. This checks the calibration workload;
   it does not establish the 25 Hz control-loop throughput.
7. If the failure persists, retain the logs and compare one variable at a time:
   direct Jetson USB versus the cascaded hub, another known-good adapter, and
   the installed `gs_usb` driver. Reassess firmware from evidence rather than
   repeatedly restarting the feed. Update the left only after a useful result
   on the right. Qualify the full dual-arm traffic separately before powered
   teleoperation.

Motor firmware and mechanical zero are separate from USB-CAN firmware and must
not be rewritten during this procedure.

## Right adapter backup and rollback

The original firmware was read twice from the ROM bootloader's advertised
128 KiB flash (64 pages of 2 KiB). Both reads have SHA-256
`255c18b8b3fc2d06bb2d9a87b011f701fd56f6ea253ed5f2a0f0e1e56a184998`.
Local copies are in `simulation_runs/rh02-right-original-20260925T004842Z/`;
remote copies are under the prepared stage's `right-original-20260925T004842Z/`.
These are the original adapter contents, not a downloaded substitute.

DFU identity: `0483:df11`, path `1-2.3.4`, serial `FFFFFFFEFFFF`, alternate 0.
That bootloader serial is not a unique identity: bind by physical path and
confirm the unchanged left runtime serial before every operation. The
advertised geometry and firmware vector table are consistent with this target;
the chip's printed package marking has not been inspected.

For a deliberate rollback, first stop all readers/controllers, set CAN down,
put only the right adapter in BOOT mode, verify the path and peer identity,
and check the original file's exact length and SHA-256 above. The bounded
restore command for its latest recorded port `1-2.1` is (verify the physical
path again before using it):

```sh
cd /home/ltr/aizee/device-maintenance/rh02-firmware-85e5b3352db7
dfu-util -d ,0483:df11 -p 1-2.1 -S ,FFFFFFFEFFFF -a 0 -s 0x08000000 -D right-original-20260925T004842Z/original-a.bin
```

Read all 131,072 bytes back to a new file using the same selectors, `-s
0x08000000:131072 -U <new-file>`, and compare with the preserved hash before
returning BOOT to normal. Do not use mass-erase, unprotect, or the option-byte
alternate setting. A rollback has **not** been performed.

## Left adapter backup and rollback

The left original firmware was read twice on 2026-09-25 at 02:21 UTC. Both
131,072-byte copies match, SHA-256:
`d3eed13ca4e83f2ea68ce0cc4b1dcdcde72481a03e78654c82b3c05ac34c5f66`.
Local copies: `simulation_runs/rh02-left-original-20260925T022103Z/`.
Remote copies: the prepared stage's `left-original-20260925T022103Z/`.
The left runtime serial is `004D002C4730511520303650`; its DFU device was at
`1-2.4`, alternate 0, with the same non-unique ROM serial `FFFFFFFEFFFF`.
Bind by physical path and verify the right peer serial/configuration before
every operation. The right runtime serial is `0021002F4D53530920303937`.

For deliberate rollback, first stop readers/controllers, verify CAN down, put
only left in BOOT, verify its current path and the backup length/hash above:

```sh
cd /home/ltr/aizee/device-maintenance/rh02-firmware-85e5b3352db7
dfu-util -d ,0483:df11 -p 1-2.4 -S ,FFFFFFFEFFFF -a 0 -s 0x08000000 -D left-original-20260925T022103Z/original-a.bin
```

Read all 131,072 bytes back into a new file with `-s 0x08000000:131072 -U
<new-file>` and compare against the original hash before returning BOOT to
normal. Do not use the right backup for the left adapter. No rollback,
mass-erase, unprotect or option-byte write was performed.
