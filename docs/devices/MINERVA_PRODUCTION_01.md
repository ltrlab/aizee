# Minerva production computer 01

Asset ID: **minerva-production-01**. This is Minerva's existing robot computer,
hostname **aizee-rover**, separate from [jetson-dev-01](JETSON_DEV_01.md).

## Identity verified 2026-09-24

| Identifier | Observed value |
|---|---|
| SSH address | `ltr@192.168.8.180` (Wi-Fi, DHCP) |
| Board | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| Module serial | `1421623029971` |
| Wi-Fi permanent MAC | `b4:8c:9d:34:fe:75` (`wlP1p1s0`) |
| Ethernet permanent MAC | `48:b0:2d:ec:0e:94` (`enP8p1s0`) |
| SSH ED25519 fingerprint | `SHA256:D/et9EIhRMVdrf8sA0+W4SzLtvehbpxY6GwG3FU0qqk` |
| OS machine ID | `661607618a944c90a32de217d0c92476` |
| Root filesystem UUID | `8eb7d664-9a83-46e9-a041-770ce919b4e1` |
| OS | Ubuntu 22.04.5 LTS, AArch64 |
| L4T / kernel | R36 revision 5.0 / `5.15.185-tegra` |
| Existing Rust toolchain | `rustc 1.93.0` |
| Deployed source directory | `/home/ltr/aizee` (no Git database) |

The SSH host key matches the previously pinned rover identity at `192.168.0.27`
and `10.42.0.1`. Do not identify this machine by hostname/IP alone. The other
Jetson has module serial `1421225029686`, a different SSH key and different MACs.

## CAN adapters

Both are USB `1d50:606f`, using the `gs_usb` driver. Following their verified
firmware updates and normal-mode reconnection, both report `RH02 gs_usb`,
manufacturer `Jhoinrch`, configuration `fw_85e5b33_2026-09-17`.
The operator identified the adapters as **Jhoinrch RH-02** (classic CAN), Amazon
ASIN `B0CRB8KXWL`, on 2026-09-24. See the [adapter inventory and candidate firmware
manifest](MINERVA_CAN_ADAPTERS_20260924.json) and [maintenance procedure](MINERVA_CAN_MAINTENANCE.md).
**Arm assignments confirmed 2026-09-24:** the user unplugged the left adapter;
only serial `0021002F4D53530920303937` remained. Interface numbers are enumeration
order and are not evidence of arm identity.

| Arm / stable name | Adapter serial | Current verification |
|---|---|---|
| Left / `canL` | `004D002C4730511520303650` | Seven identities verified; stopped readings only |
| Right / `canR` | `0021002F4D53530920303937` | Seven identities verified; stopped readings only |

At the operator's direct Jetson reconnection on 2026-09-24, left moved to USB path
`1-2.4` and right to `1-2.1`. The former external hub paths were `1-2.3.2` and
`1-2.3.4`. Serial-based naming followed correctly without a udev change. USB
paths record topology; the serials remain the arm identity.

At **2026-09-25 02:29 UTC**, after a further operator port change, the then-current
paths are **left `1-2.1`, right `1-2.2`**. This was not a direct swap: left moved
to right's previously tested port, while right moved to a third port. Both
serial-based names and firmware strings were verified before the comparison.

At **2026-09-25 02:51 UTC**, paths returned to **left `1-2.4`, right `1-2.1`**.
The operator clarified that short USB cables had remained after external-hub
removal; left's cable has now been removed, while right's remains. Prior
"direct" connection descriptions establish hub removal only. Both serials and
firmware identities remain unchanged. Left termination is still operator-
reported OFF, not independently measured.

**Latest maintenance state, 2026-09-25 UTC:** left's short USB cable has been
restored. The operator repeated the resistance check with all power off and both
adapter USB plugs removed, reporting **60 ohms on each CAN bus**. This supports
the expected aggregate termination load but does not establish terminator
placement, ground reference, or signal integrity. The later attempted driver
installation detected attached USB interfaces `1-2.4:1.0` and `1-2.1:1.0` and
stopped at preflight, before mutation. The candidate remains staged; no driver
installation or reboot occurred in that attempt. Details are in the latest log
entry below.

Initial inventory before unplugging or changing rules:

| Initial interface | USB path | Adapter serial | State / traffic |
|---|---|---|---|
| `can0` | `1-2.3.2` | `004D002C4730511520303650` | Down/stopped; RX 0, TX 0 |
| `can1` | `1-2.3.4` | `0021002F4D53530920303937` | Down/stopped; RX 0, TX 0 |
| `can2` | Internal `c310000.mttcan` | Not a USB adapter | Down/stopped; RX 0, TX 0 |

The old udev rules matched USB paths `1-2.4.2:1.0` and `1-2.4.4:1.0`, which no
longer matched the physical connections. Replaced them with unique serial
matches in both `/etc/udev/rules.d/80-minerva-can.rules` and the deployed source
copy. Names now follow the adapters across USB ports. Reconfirm wiring and
update the serial mapping if either adapter is replaced or moved to another arm.

## Service state and integration gaps

- `aizee-minerva-left` and `aizee-minerva-right` are **stopped and disabled for
  boot** during bring-up. Both retain their previous failed status and have
  `MainPID=0`. Their earlier pre-start failures came from missing `canL`/`canR`.
  Both adapters are connected and both aliases are verified. Neither motor
  process was started to test the fix.
- The legacy rover motor service is inactive and disabled.
- No motor command/telemetry listeners were present on 5555/5556 or 5575/5576.
- The old ESP-NOW/serial stop design was **retired by user request**. Its service
  is uninstalled (`LoadState=not-found`, `MainPID=0`), with backups below. No
  replacement physical stop or programmatic electrical cutoff is available.
- The user confirmed that the arms can fall safely on their test stand when
  torque is removed. Software stop remains best-effort CAN disable.
- The `vcan` module is now loaded and `vcanGuardL` / `vcanGuardR` are up for
  synthetic tests. No persistent boot setup was installed for these interfaces.
- The guarded controller is staged in maintenance directories below. It has run
  virtual-CAN, observation and stopped-motor diagnostics. A separate finite tool
  ran watchdog and right-gripper micro-jog trials. No new boot service is enabled;
  general physical movement through the guarded network service remains blocked.
- The deployed root `Cargo.toml` is absent, so this is still an older source
  layout. A future update must include the complete shared workspace/package,
  or verified cross-built artifacts, rather than copying only the new crate.

All 14 motors now respond, with user-confirmed model labels and captured MCU UID /
firmware. Following the user's left RS00 and RS02 firmware updates, latest
command-loss outcomes are 14 passed, 0 failed, 0 deferred on current firmware.
Only left j4/j5 currently retain the trial's volatile watchdog setting;
the other twelve read zero and need configuration before further enable.
Both gripper micro-jogs failed to produce meaningful motion. Motor-to-CAD mapping,
loaded stopping and production operation remain unqualified. See the latest log
entry, [original evidence](MINERVA_MOTOR_COMMISSIONING_20260924.json),
[RS00 retest](MINERVA_RS00_FIRMWARE_RETEST_20260924.json) and
[latest RS02 retest](MINERVA_RS02_FIRMWARE_RETEST_20260924.json).

## Baseline fingerprints

| File under `/home/ltr/aizee` | SHA-256 |
|---|---|
| `rust/target/release/motor_control` | `458baf0a1d31d9d05d19ad190b79cdd0b4e1237316f8747e529cd928c4bd570b` |
| `rust/motor_control/src/main.rs` | `3e5e4cc9b569576a5961d5b84f61def2037288dc873323091ff4c23d58db8e83` |
| `rust/motor_control/src/robstride.rs` | `5f66a7a2c478894d66e49cfe3ef8fd2459da4f8ae3c90861cfa23f27bd4ddfd8` |
| `rust/motor_control/src/motor.rs` | `613dd441c3abf9352be4443e51c005640e2f98f6594348f9413171d5b7040516` |

## Change log

### 2026-09-24 — read-only access and inventory

1. User supplied hostname/IP and reported powered motors and connected adapters.
2. Verified the existing rover SSH host key before authentication. Used the existing
   rover SSH key without changing it or host key records. Windows OpenSSH rejected
   its local ACL; Paramiko connected with the same pinned host identity.
3. Collected module/MAC/OS identity, adapter serials, CAN state, installed service
   definitions and recent service logs. Recorded baseline binary/source hashes.
4. Identified missing CAN aliases, both failed motor services, single-arm e-stop
   routing, missing receiver device and the older deployed bridge implementation.
5. Requested physical adapter-to-arm mapping and the power-cutoff/support details.
6. No remote files were changed. No services were installed/restarted, no CAN
   interface was changed, no serial device was opened and no motor command was sent.

Raw local evidence is in the ignored
`simulation_runs/minerva-production-20260924/inventory.json` and `services.json`.
This document preserves the relevant identity and findings independently of
those temporary reports. Future deployments must log their source/binary hashes,
changed files, service actions and validation results here.

### 2026-09-24 — confirmed arm mapping and installed stable names

1. User unplugged the left adapter. At 16:14:49 UTC, the sole remaining USB CAN
   adapter was `can1`, serial `0021002F4D53530920303937`, establishing right/left
   assignments above.
2. Rechecked the pinned SSH identity, module serial and machine ID. Required
   exactly the right adapter, a down interface and no running arm service before
   making changes.
3. Ran `systemctl disable --now aizee-minerva-left.service
   aizee-minerva-right.service` to keep both controllers stopped during bring-up.
4. Backed up and atomically replaced the installed and source udev rules,
   preserving file permissions and ownership. Reloaded udev rules and issued an
   add event only for `/sys/class/net/can1`; no USB reset or bus activation.
5. Verified the resulting `canR` serial, DOWN/STOPPED state, RX 0 / TX 0, and both
   motor services disabled with `MainPID=0`. No motor commands were sent.
6. Left-adapter reconnect verification was pending at this stage (completed
   below). The e-stop integration and guarded production controller gaps above
   remain open.

Remote maintenance directory:
`/home/ltr/aizee/device-maintenance/can-naming-20260924T162206Z/`.
It contains `manifest.json`, the installer, staged rules, and backups
`installed-rules.before` / `source-rules.before`. The directory timestamp comes
from the development machine; the Jetson reported completion at 16:22:00 UTC.

| Changed file | Previous SHA-256 | Installed SHA-256 |
|---|---|---|
| `/etc/udev/rules.d/80-minerva-can.rules` | `3a04234453a5d7e7a2517cfda1489874545712e49c4f7506e0890d28f12dd5aa` | `294157ec1a0904637bb8c31811599e293fa1ef151a4560c34a7bfc3a304f4703` |
| `/home/ltr/aizee/config/udev/80-minerva-can.rules` | `483730b7893a9f7d88db2ae791bd5ceb6d8269b515904b605e5cb2b2fd2ecacf` | `294157ec1a0904637bb8c31811599e293fa1ef151a4560c34a7bfc3a304f4703` |

Local evidence: ignored
`simulation_runs/minerva-production-20260924/can-naming-20260924T162206Z.json`.
If reverting the naming change, restore the corresponding rule backups and
reload udev; keep motor services disabled until the outstanding bring-up checks
are complete.

### 2026-09-24 — verified both adapters after left reconnection

At 16:25:24 UTC, after the user reconnected the left adapter, a read-only SSH
check verified:

- `canL`: serial `004D002C4730511520303650`, USB path `1-2.3.2`.
- `canR`: serial `0021002F4D53530920303937`, USB path `1-2.3.4`.
- Both interfaces DOWN/STOPPED, RX 0 / TX 0. The left alias appeared through
  the installed udev rule without a manual rename.
- Both arm services disabled with `MainPID=0`; their previous failed status
  remains. No controller was started or motor command sent.
- Module serial, machine ID and installed udev-rule hash match the records above.
- `/dev/estop-receiver` remains absent; motor communication and powered operation
  have not been tested.

Local evidence: ignored
`simulation_runs/minerva-production-20260924/read_reconnected.json`.

### 2026-09-24 — retired stop bridge and staged guarded commissioning

User instruction: remove the ESP-NOW/serial stop design; a replacement will be
designed later. There is no programmatic power cutoff. The arms may fall safely
on their test stand if torque is removed.

Maintenance directory:
`/home/ltr/aizee/device-maintenance/guarded-20260924T164504Z/`.

1. Verified module/machine identity, both physical CAN interfaces down with TX 0,
   and all legacy motor services with `MainPID=0`.
2. Stopped and disabled `aizee-estop-bridge`; backed up and removed its installed
   service and udev rule, deployed source copies, and deployed `bridge.py`.
   Reloaded systemd/udev without triggering devices. The service is now absent.
   `maintenance.json` contains exact original paths, file hashes and backups
   named `retired-0-*`, `retired-2-*`, `retired-3-*`, `retired-5-*`, `retired-8-*`.
   Maintenance manifest SHA-256:
   `1bb8a0e2c30a4c165b4a5bee60faad4e94926100ca84e27559bde8add379cb40`.
3. Loaded `vcan` and created/up'd only `vcanGuardL` and `vcanGuardR`. No physical
   link state was changed. These virtual interfaces last until removed/rebooted;
   no persistent boot configuration or new systemd service was installed.
4. Staged the cross-built guarded service, shared Python package files and
   observation/parameter-probe profiles in the maintenance directory. Left the
   existing legacy motor binaries and their disabled services intact.
5. Ran the virtual-CAN fault matrix and 1,000-cycle 25 Hz command soak. The final
   build applies accepted setpoints at the scheduled 400 Hz tick; network requests
   cannot accelerate CAN control writes. Results below are synthetic.
6. Ran that final binary for five seconds on `canL`/`canR` in **receive-only**
   observation mode: RX 0 / TX 0, no disable transmission attempted, 1,982 ticks,
   zero missed tick deadlines, maximum lateness 536 us and maximum work 27 us.
7. Final inspection: both adapters retain their verified serials and names,
   remain DOWN/STOPPED with RX 0 / TX 0; legacy motor services remain disabled
   with no motor processes. No guarded listener remains on port 5590 after tests.
   The staged parameter-read profile validates without opening any sockets.

| Staged artifact | SHA-256 |
|---|---|
| `guarded_controller` | `6e09e60b4b35a815dfd6a2d3effc87142b0c9ca93bf70572922f8ec9300205ae` |
| `observe.json` | `e8c3ad0533a832c7986c1415811da3c345c06b0075705f32c1077263e0370739` |
| `parameter-probe.json` | `923df1f0eead891dc791afb14b9530bec70cc88364b78a5436ace02d1b012d70` |
| `guarded_vcan_bench.py` | `52c83d7157bbd5d266235bb9e64f7260fd1675eb3f36d72d115b6e06249b391b` |

Final raw benchmark: `bench-scheduled-1000/result.json` within the maintenance
directory. [Repository evidence](../architecture/GUARDED_COMMISSIONING_RESULTS.json)
preserves the report, artifact identities and physical-interface state.

| Final synthetic test | Result |
|---|---|
| Command cycles at 25 Hz | 1,000 / 1,000 completed |
| Renewal + acknowledged command cost | p50 7.43 ms, p99 7.75 ms, max 7.90 ms |
| Scheduled 40 ms misses / nominal faults | 0 / 0 |
| 400 Hz receiver tick misses during nominal phase | 0 |
| Nominal maximum receiver lateness / work | 434 us / 315 us |
| Startup gates, expiry, replay, stale feedback, invalid batch, stall, legacy-frame rejection, shutdown, restart epoch | Passed |

The injected 150 ms process stall deliberately produced one overrun and latched
a fault; those counts are recorded separately from the nominal run. These
tests do not include camera load, collision computation, physical CAN arbitration
or real actuator dynamics. They qualify preparation for the first supervised
parameter-read trial, not loaded/dual-arm movement. Follow
[powered commissioning](../POWERED_COMMISSIONING.md). No powered trial was run.

### 2026-09-24 — physical diagnostics, watchdog behavior and gripper trials

[Repository motor evidence](MINERVA_MOTOR_COMMISSIONING_20260924.json) preserves
motor identities, parameters, watchdog TX/RX traces, jog summaries, artifact
hashes, final service/link state and hashes/locations of unabridged captures.
The older entries above describe their historical state; physical CAN traffic
and finite enable trials were performed in this increment.

1. At 17:15 UTC, the first parameter trial received 33/42 replies: all seven
   right motors and left IDs 4..7 responded; left IDs 8..10 did not. The user
   reseated connections and power-cycled. No enable was sent in this read trial.
2. User confirmed the installed labels on each arm: IDs 4..10 are RS03, RS04,
   RS03, RS02, RS02, RS00, RS00. The device manifest now records that confirmation.
3. Corrected guarded commissioning wire ranges from manufacturer manuals: RS00
   uses ±14 Nm, ±33 rad/s, Kp maximum 500; RS03 velocity is ±20 rad/s. These are
   protocol scales, not allowed operating limits. The disabled legacy controller
   still contains older scales and was not used for these experiments.
4. Staged the diagnostic receiver and shared Python tools in
   `/home/ltr/aizee/device-maintenance/diagnostics-20260924T172419Z/`.
   Ran its 100-cycle synthetic 25 Hz check: all 15 checks passed, zero nominal
   deadline misses/faults. Renewal/command cost p50 7.193 ms, p99/max 7.301 ms;
   receiver maximum lateness/work 134/344 us. Raw report: `bench-100/result.json`.
5. At 17:25 UTC, all 14 motors completed stopped diagnostics: 182/182 replies,
   no fault bits, MCU UID/firmware captured. All initial watchdog settings were 0.
6. Between 17:33 and 17:38 UTC, ran finite nominal-zero-gain watchdog trials.
   Each identified motor received a volatile `0x7028 = 600` write and readback,
   neutral-frame check, enable, 150 ms command silence, status probe and final
   stop. No gain/zero/firmware/flash setting was saved. Results are below.
7. The user confirmed the right gripper empty and midway open. At 17:44 and
   17:49 UTC, tried +0.025 motor rad out-and-back jogs with Kp 1 then Kp 2;
   Kd 0.05, zero feedforward. Both first re-passed the watchdog test. Each used
   0.6 s out / 0.2 s dwell / 0.6 s return / 0.2 s dwell, monitored limits of
   0.08 Nm reported effort, 0.2 rad/s speed, and 0.04 rad relative displacement.
   Both failed the movement criterion: max travel 0.0003835 rad (one encoder
   count), final position unchanged at 2.461885 rad, 225 samples each. Peak
   reported effort was 0.05533 then 0.07285 Nm; peak reported speed 0.08812 then
   0.09114 rad/s. Both confirmed final Reset. No further force increase was run.
   These software thresholds are not hardware torque clamps; stored motor
   torque/current limits were unchanged. No useful jaw travel is established.
8. At 17:50 UTC, final stopped diagnostics again received 182/182 replies, all
   motors Reset/fault-free. Twelve watchdogs read 600; deferred left j5/gripper
   remained 0. Values are RAM-only and must be rechecked after power cycling.
9. At 17:55 UTC, final read-only inspection verified the same host and adapter
   identities, both CAN interfaces DOWN/STOPPED with no RX/TX errors, all legacy
   motor services disabled with MainPID 0, retired bridge absent, port 5590
   closed. No permanent service was enabled and no legacy binary was replaced.

| Joint | CAN ID | MCU UID (raw byte order) | Firmware | Command-loss test |
|---|---|---|---|---|
| left_arm_j1 | 4 | `3c56300214713704` | 0.3.1.30 | Passed |
| left_arm_j2 | 5 | `a5493bc000203018` | 0.4.1.32 | Passed |
| left_arm_j3 | 6 | `797a30020c323817` | 0.3.1.42 | Passed |
| left_arm_j4 | 7 | `a5283b8f2450b00e` | 0.2.3.24 | Failed: remained Run |
| left_arm_j5 | 8 | `3e4330021c40b614` | 0.2.3.24 | Deferred |
| left_arm_j6 | 9 | `2f3d30031064b401` | 0.0.3.22 | Failed: remained Run |
| left_gripper | 10 | `1057300214713506` | 0.0.3.22 | Deferred |
| right_arm_j1 | 4 | `4e8230021471b10a` | 0.3.1.30 | Passed |
| right_arm_j2 | 5 | `4fa930020c34380b` | 0.4.1.24 | Passed |
| right_arm_j3 | 6 | `379b93c79c90b014` | 0.3.1.37 | Passed |
| right_arm_j4 | 7 | `3d93302110703018` | 0.2.3.34 | Passed |
| right_arm_j5 | 8 | `0965302110703016` | 0.2.3.34 | Passed |
| right_arm_j6 | 9 | `961430020c343701` | 0.0.3.32 | Passed |
| right_gripper | 10 | `632f93c79c90b00e` | 0.0.3.32 | Passed |

Ten passed, two failed, two deferred. Both failures accepted/read back 600 but
remained Run after the silence interval; both acknowledged the subsequent stop.
The matching older firmware on deferred motors is a reason to investigate, not
proof of the root cause or proof an update will fix it. No firmware was flashed.
A passing test establishes Reset after the tested ~150 ms gap, not exact watchdog
latency, loaded stopping distance or independent power isolation.

All paths below are relative to `diagnostics-20260924T172419Z`:

| Artifact | SHA-256 |
|---|---|
| `guarded_controller` | `108b7b3ea81acb8db450037b5b2f660a84d1ec449209d4d53cc895c8b8f3ed7c` |
| `motor_diagnostics.json` | `a1398f358099fb8f61deda7e06e52fd15b6218d207cc153145d1155c6afad239` |
| `watchdog-preparation/motor_watchdog_trial` | `5f7bf9951ca30c15c0f60e8906e06ab6b2961f311b7120d7c67b619aa487bec6` |
| `jog-preparation/motor_watchdog_trial` (Kp 1) | `5894ea0f9aad03a13597598304dae239482eb6187774b18a255de63b54bb5c5d` |
| `jog-kp2-preparation/motor_watchdog_trial` (Kp 2) | `78f4031d3f2e2a381683536ed2dbce98896de9fd82df0b9792bcd5da0b223b2f` |
| Both jog profiles, `right-gripper-jog.json` | `370227403e5498a475d140dee9757f218fbbb8a5eb670c8694bcd42611a53511` |

Each experiment has a timestamped subdirectory with raw stdout/stderr/result;
the evidence JSON lists exact paths. Previous artifacts were preserved, including
the Kp 1 binary. The diagnostic receiver is an earlier deliberately pinned build;
the latest local cross-build is not installed as a production service.

Local checks after this increment: 30 Rust tests, Clippy with warnings denied,
432 Python unit/contract tests (34.35 s), and three successful AArch64 ELF checks.
The first sandboxed Python suite was blocked by Windows temporary-directory ACLs;
the complete offline suite passed outside that sandbox. Physical motion remains
unqualified pending the gates in [powered commissioning](../POWERED_COMMISSIONING.md).

Read-only follow-up located manufacturer release V26.05.19 with RS00 0.0.3.32 and
RS02 0.2.3.34 packages matching the corresponding tested right-arm versions.
Candidate asset hashes and update prerequisites are recorded in the commissioning
guide. No firmware download, flash, reset-to-defaults or calibration write was
performed in this follow-up.

### 2026-09-24 — verified user updates to left j6 and gripper

The user reported updating left j6 and gripper firmware. This increment checked
that work; the agent did not flash firmware or save motor configuration.
[Retest evidence](MINERVA_RS00_FIRMWARE_RETEST_20260924.json) preserves before/after
parameters, original identities, complete watchdog traces, hashes and final state.

1. At 18:35 UTC, verified the pinned SSH host, module serial, machine ID, adapter
   serials, stopped/disabled legacy services and closed control port. Both CAN
   interfaces started down.
2. Stopped diagnostics received 182/182 replies from all 14 motors. Left j6
   (CAN 9, UID `2f3d30031064b401`) and gripper (CAN 10, UID `1057300214713506`)
   now report **0.0.3.32**, previously 0.0.3.22. All other motor versions, UIDs
   and CAN routes matched the earlier capture. Every motor reported Reset with
   no fault bits. All watchdog registers initially read 0 after maintenance.
3. Generated two new single-motor profiles pinned to these UIDs/new firmware;
   kept the old profiles and evidence intact. Reused the previously tested
   nominal-zero-gain binary, SHA-256
   `5f7bf9951ca30c15c0f60e8906e06ab6b2961f311b7120d7c67b619aa487bec6`.
4. At 18:37:28 UTC, both tests passed: explicit volatile timeout 600 with readback,
   neutral-frame check, nominal-zero-gain enable, command silence, autonomous
   Reset observed, final stop acknowledged. Measured wait: j6 150 ms, gripper
   151 ms. No jog, nonzero gain target, zero-setting command or flash save was
   issued. The previously observed j6 failure is resolved under this test; the
   gripper was previously deferred and now has its first passing result.
5. At 18:38 UTC, final diagnostics again received 182/182 replies: all 14 Reset,
   no fault bits. Only left j6/gripper read timeout 600; the other twelve read 0.
   Both interfaces were returned down, services remain disabled with MainPID 0,
   and port 5590 is closed. No persistent service or controller code was changed.

Latest per-motor outcomes across commissioning: **12 passed, 1 failed (left j4),
1 deferred (left j5)**. Left j4/j5 remain RS02 0.2.3.24. Historical passes do not
mean watchdogs are presently enabled: configure/read back and verify protection
before subsequent motor enable, including after any reset or power cycle.
These tests establish Reset after the observed silence, not exact watchdog
latency or loaded stopping performance.

The configured left-gripper limits remained 2 Nm, 5 rad/s, 10 A; left j6 remained
14 Nm, 33 rad/s, 16 A. These stored values were read, not changed. Left j6's
reported position changed from 6.166641 rad in the earlier capture to near zero.
Physical repositioning, reboot and calibration effects were not distinguished;
preservation of mechanical zero is not established. Motor-to-CAD mapping remains
unverified and previous pose targets must not be replayed.

Remote directory, under
`/home/ltr/aizee/device-maintenance/diagnostics-20260924T172419Z/`:

- Initial readback: `stopped-motors-20260924T183545Z/result.json`.
- Trial: `watchdog-preparation/rs00-after-update-20260924T183728Z/result.json`.
- Final readback: `stopped-motors-20260924T183805Z/result.json`.
- New profiles: `watchdog-preparation/post-rs00-update-profiles/`.

Profile SHA-256:

| File | SHA-256 |
|---|---|
| `left_arm_j6.json` | `a8831a9a359fe7bb614e46c810cccffd72be35882330b532f0003cbce55d5b4f` |
| `left_gripper.json` | `202cd05ac57b9efc39d998888490b123d1d9324c1d34e58022d04722dafcde8d` |
| Remote `manifest.json` | `8e8d36e252593c3df3c7a91a073c74efb0f5a509a9dc1e8ab12fc71b49064b64` |

Only diagnostic profiles, trial artifacts and documentation were added in this
increment. Production code was unchanged; validation consisted of the bounded
physical trials, readback checks and local evidence consistency checks.

### 2026-09-24 — verified user updates to left j4 and j5

The user reported updating the left RS02 motors. This follow-up verified the new
versions and behavior; the agent did not flash firmware or save configuration.
[RS02 retest evidence](MINERVA_RS02_FIRMWARE_RETEST_20260924.json) preserves raw
watchdog traces, parameter snapshots, hashes, final CAN/service state and the
combined per-motor outcomes from all three commissioning increments.

1. At 18:50 UTC, pinned-host stopped diagnostics verified module/machine and
   adapter identities, disabled legacy services and no existing control listener.
   Received 182/182 replies: all 14 motors in Reset, no fault bits. Left j4
   (CAN 7, UID `a5283b8f2450b00e`) and j5 (CAN 8, UID `3e4330021c40b614`) now
   report **0.2.3.34**, previously 0.2.3.24. All original UIDs, CAN routes and
   other motor versions matched the previous capture. All watchdogs initially
   read 0 after this maintenance interval, including the two RS00 motors.
2. Generated two fresh profiles pinned to the new firmware and original UIDs;
   preserved prior profiles/reports. Reused the tested nominal-zero-gain binary,
   SHA-256 `5f7bf9951ca30c15c0f60e8906e06ab6b2961f311b7120d7c67b619aa487bec6`.
3. At 18:51:32 UTC, both tests passed: volatile `0x7028 = 600` write/readback,
   neutral-frame check, nominal-zero-gain enable, command silence, Reset observed
   before the final stop, and final stop acknowledged. Wait durations were
   150 ms for j4 and 151 ms for j5. No jog, nonzero gain target, motor-zero write
   or flash save was issued. The old j4 failure is resolved under this test;
   previously deferred j5 now has its first passing result.
4. At 18:51:56 UTC, final diagnostics again received 182/182 replies, all Reset
   with no fault bits. Only left j4/j5 read timeout 600; the other twelve read 0.
   Both CAN interfaces returned down, all legacy motor services remain disabled
   with MainPID 0, the retired bridge remains absent and port 5590 is closed.

**All 14 motors now have passing individual watchdog results on their installed
firmware.** There are no remaining failed/deferred results for current firmware.
This aggregates the original trials and both firmware retests; only j4/j5 were
enabled in this increment. The earlier failed outcomes remain in historical
records. No simultaneous loaded-arm test, exact timeout latency measurement,
physical travel calibration or full 25 Hz collision-loop test was performed.

Stored limits were preserved in readback: both RS02 torque limits 17 Nm and
current limits 27 A; j4 speed limit 20 rad/s and j5 2 rad/s. Their reported
positions are now near zero (previously roughly -0.009 and -0.053 rad). We have
not distinguished repositioning/reboot effects from calibration changes. Motor-
to-CAD mapping remains unverified; old pose targets must not be replayed.

Watchdog settings are volatile. All 14 historical passes do not mean all 14 are
currently protected. The guarded production startup path still needs mandatory
per-motor watchdog configuration and verification before enable; the finite
commissioning tool performs that explicitly today.

Remote paths, under
`/home/ltr/aizee/device-maintenance/diagnostics-20260924T172419Z/`:

- Initial readback: `stopped-motors-20260924T185018Z/result.json`.
- Trial: `watchdog-preparation/rs02-after-update-20260924T185132Z/result.json`.
- Final readback: `stopped-motors-20260924T185156Z/result.json`.
- Profiles: `watchdog-preparation/post-rs02-update-profiles/`.

| Profile artifact | SHA-256 |
|---|---|
| `left_arm_j4.json` | `03a547e4bc0a646c0b3f4fce33655545c6f64970a87b454bcfba9a9ba5b013e6` |
| `left_arm_j5.json` | `73cc05c2433a4451fd240f09e5dcbbf160e950c8e5207a9c78cb5b97cfc8b4c7` |
| Remote `manifest.json` | `3ae2133175acaeacf67127a04046ba94f45ef413a5a2ee3c00ec29982aca9f7c` |

Production code was unchanged. Validation for this increment was the bounded
physical retest, full stopped readback and local evidence consistency checks.
The next motion experiment remains right-gripper breakaway characterization,
with bounded travel and speed, after fresh watchdog setup/verification.

### 2026-09-24 - mandatory watchdog startup and right-gripper breakaway

Implemented the robot-independent watchdog gate in `robot_control`. Physical
adapter enable now requires a fresh UID/firmware-pinned proof of stopped mode,
volatile timeout 600 readback and neutral-frame behavior. Zero-torque service
startup verifies this before opening its command socket. Disarmed audits do not
rewrite settings. Failures invalidate proof, and enable consumes it; disable or
fault requires a fresh service process. The legacy services remain disabled.

[Complete evidence](MINERVA_WATCHDOG_GATE_GRIPPER_20260924.json) includes artifact
manifests, raw frames, both successful and failed outcomes, the stopped telemetry
sample and final device state. Existing commissioning reports are preserved.

1. **19:11 UTC:** stopped diagnostics returned all 182 replies, all 14 motors
   Reset/fault-free with unchanged UIDs and installed versions. Right gripper
   position parameter was 2.462806 rad. Its watchdog initially read zero.
2. **19:12 UTC:** staged the cross-built binaries in a new maintenance directory.
   The service passed 16 synthetic vcan checks and 100 cycles at 25 Hz: median
   7.397 ms, p99/max 7.620 ms, zero 40 ms misses or nominal faults. Injected faults
   were isolated from this nominal timing measurement.
3. **19:14:15 UTC:** on physical canR, wrong UID startup was rejected before the
   command socket opened. Correct UID/firmware startup completed; ten sampled
   audit statuses remained disarmed and watchdog-ready. No enable was requested.
   The two-second service completed with zero scheduling overruns/deadline misses.
4. **19:14:35 UTC:** positive right-gripper breakaway trial passed. Feedforward
   rose at 0.125 Nm/s with Kp 0 / Kd 0.05. The first 0.003 rad criterion was met
   at **+0.200375 Nm command**, reported effort +0.178805 Nm. Maximum travel was
   0.003068 rad; peak reported effort 0.208286 Nm and speed 0.147539 rad/s.
   Drive stopped on first qualifying movement; final stop was acknowledged.
5. **19:15:02 UTC:** fresh stopped diagnostics preceded reverse-profile creation.
   **19:15:44 UTC:** the negative ramp detected motion at **-0.206 Nm command**,
   reported effort -0.209140 Nm. It **failed** the post-stop speed envelope.
   Travel peaked at 0.003068 rad. After drive removal, the encoder reversed:
   q=2.461118 rad at 1857 ms, q=2.462268 rad at 1863 ms, and q=2.463035 rad at
   1865 ms in the trial clock. The corresponding latter speed readings were
   +0.229114 and +0.254292 rad/s. Both were Reset replies with nominal-zero
   reported effort. Acknowledged disable did not establish immediate rest.
6. **19:17:05 UTC:** final diagnostics again returned all 182 replies, all motors
   Reset with no fault bits. A 100-sample right-gripper stopped capture over
   2.485 seconds showed position parameter span 0.000307 rad, peak reported
   parameter velocity 0.097065 rad/s and peak wire velocity 0.087114 rad/s.
   No powered trial was repeated after the reverse failure.
7. Hardened reporting to include final-stop feedback in trial maxima and pass
   criteria. A deterministic regression covers motion followed by post-stop
   overspeed. **38 Rust tests, portable Clippy with warnings denied and 432 Python
   tests passed** (Python 37.05 s). Cross-build produced three AArch64 ELF64 tools.
   At 19:21 UTC, staged the reporting fix separately and validated Linux launch/
   configuration without opening CAN. The guarded service binary is unchanged;
   the new finite-trial binary has not repeated the hardware movement trials.

Both ramps first repeated the watchdog behavior experiment: 600 readback,
nominal-zero-gain enable, 150 ms silence, autonomous Reset, then fresh readback
before motion enable. Experiment caps remained 0.25 Nm command, 0.35 Nm reported
force/effort threshold, 0.2 rad/s speed, 0.02 motor rad travel, 20 ms command gap.
The 0.003 rad criterion measures motor motion, not calibrated jaw displacement.
No torque/current/speed parameter limits, motor zeros or flash settings changed.

These first-motion values support testing around roughly 0.20 Nm at this pose;
they are not yet a precise static-friction estimate or repeatability evidence.
The encoder rebound is consistent with mechanical recoil/backlash, while the
stopped sample demonstrates velocity-estimate variation. The contributions are
not yet separated. Next work is to investigate this stop transient and design a
bounded damped stop, then demonstrate repeated travel/return in both directions.
Keep the existing bounds rather than increasing them to pass. Full arm movement,
CAD mapping, loaded stopping and collision/camera loop qualification remain open.

Final state: watchdog 600 on **left j4, left j5 and right gripper**, zero on the
other eleven motors. Both physical CAN interfaces down; legacy units disabled,
MainPID 0; retired stop bridge absent; port 5590 closed. No reboot, service enable,
firmware flash, persistent motor setting or power-cutoff change was performed.

Artifact directories:

- Hardware trials: `/home/ltr/aizee/device-maintenance/watchdog-gate-20260924T191255Z/`.
  Reports: `startup-gate-20260924T191415Z/result.json`,
  `breakaway-plus-20260924T191435Z/result.json`,
  `breakaway-minus-20260924T191544Z/result.json`.
- Final stopped report, under the existing diagnostic stage:
  `stopped-motors-20260924T191705Z/result.json`.
- Latest artifacts: `/home/ltr/aizee/device-maintenance/watchdog-gate-20260924T192109Z/`.

| Artifact | SHA-256 |
|---|---|
| Guarded service, both stages | `1e941f1aa1a0f85b665e12254e4f158278bf8141cc9a97e450309ef2ad8c0e11` |
| Finite trial tool used for physical experiments | `321ac5335a5420abcf181e27814f33e96705550d286a9aa3cd81d4ce506bf48a` |
| Latest finite trial tool, with final-stop reporting | `76bc44591612fd08c87213778b6125cee2eb7ac6854d5a12d5f51f455dbfc8b9` |
| AArch64 harness | `bf09b1edd2d7f3a92b22205f3433edcadca74bcb7590d94d24e0ca35c5e50d41` |

The earlier negative report's per-ramp maximum is 0.229114 rad/s; its full raw
trace also contains the 0.254292 rad/s final cleanup reading. The new evidence
records that derived peak separately and preserves original report bytes. The
new binary makes this distinction explicit without rewriting historical results.

### 2026-09-24 - staged damped stop, four passing gripper trials

The user confirmed that the mechanism looked physically good and offered visual
feedback. Continued the attended right-gripper commissioning on the pinned host
and adapters. [Damped-stop evidence](MINERVA_DAMPED_GRIPPER_20260924.json) retains
all four raw trials, profile/source/binary hashes and final diagnostics. The
previous immediate-stop failure remains unchanged in its historical report.

Added an explicit `breakaway.stop = "damped"` option to the shared finite trial
tool and `--damped-stop` to the shared Python profile generator. Historical
profiles default to immediate disable. No robot name is embedded in the stop
algorithm; identity, bus, motor and protocol ranges still come from the profile.
The staged sequence is:

1. Taper feedforward to zero over 100 ms with Kp 0 and Kd 0.2 Nm*s/rad.
2. Keep zero feedforward and that damping for 200 ms.
3. Taper damping to zero over 200 ms.
4. Observe nominal-zero-gain behavior for 150 ms, then disable and collect
   twenty stopped samples plus the final stop reply.

Both the damping and zero-gain phases require at least 100 continuous ms and ten
samples within a 0.0008 rad position span and 0.1 rad/s reported speed. The
existing 0.02 rad travel, 0.2 rad/s speed and 0.35 Nm reported-effort limits remain
active throughout. Predicted drive-plus-damping effort is also checked against
0.25 Nm. A failed bound, reply, timing or settling check takes the immediate
stop path. Phase durations are finite; no extension or stronger retry is used.
The whole experiment remains limited to four seconds, with a bounded 2,048-event
trace budget for the additional stop samples.

At 19:38:31 UTC, fresh diagnostics received all 182 replies: all 14 motors Reset,
no fault bits, unchanged UID/firmware roster. Right-gripper position parameter
was 2.464149 rad. Each subsequent trial used a freshly measured start position
and repeated the pinned watchdog behavior test before motion enable.

| UTC start | Direction / trial | Command at 0.003 rad threshold (Nm) | Maximum travel (rad) | Peak reported speed (rad/s) | Peak reported effort (Nm) | Result |
|---|---|---:|---:|---:|---:|---|
| 19:39:48 | Negative 1 | -0.210375 | 0.003068 | 0.112291 | 0.214694 | Pass |
| 19:41:54 | Positive 1 | +0.180125 | 0.003452 | 0.099199 | 0.204868 | Pass |
| 19:42:28 | Negative 2 | -0.174750 | 0.003068 | 0.096178 | 0.193332 | Pass |
| 19:42:59 | Positive 2 | +0.178250 | 0.003068 | 0.093156 | 0.198459 | Pass |

The active stop phases completed in 656-662 ms; all four trials passed zero-gain
settling and post-disable checks. Peak reported speed in these trials stayed
below 0.113 rad/s, versus 0.254292 rad/s in the earlier failed immediate-stop
reverse trial. This is a comparison of the recorded experiments, not a guarantee
across loads, poses or a statistically established reduction.

Net encoder changes after release were -0.000767, +0.002685, -0.001534 and
+0.002301 rad respectively. Some of the loaded displacement relaxed as torque
fell. The measurements therefore establish repeatable small motor-space tests
at this pose, not independently confirmed jaw displacement, exact breakaway
torque or jaw force. A visual-observation question was sent after the first
trial; no later observation is assumed in this evidence. The next step is to
confirm jaw travel against encoder travel and design a useful bounded return
cycle before approaching travel stops or operating an arm.

At 19:44:06 UTC, final diagnostics received all 182 replies, all 14 motors Reset
and fault-free. Final right-gripper position parameter was 2.466948 rad. Stored
run-mode, torque/current/speed limits, UIDs and firmware matched the initial
capture. Watchdog settings remained 600 on left j4/j5 and right gripper, zero on
the other eleven. Both CAN interfaces returned down, legacy services disabled
with MainPID 0, retired bridge absent, port 5590 closed. No firmware/flash save,
zeroing, persistent limit change, arm motion or production service enable occurred.

Validation: **40 Rust tests, Clippy with warnings denied, and 432 Python tests
passed** (Python 24.91 s). Added deterministic checks for the full damping/neutral
sequence and reply loss, unexpected Reset, excess speed/travel, failure to settle
and delayed commands. Three AArch64 ELF64 binaries cross-built; the finite tool
was staged and used for all four physical trials. The Linux network controller
was not re-benchmarked in this increment; its previous vcan evidence is retained.

Remote stage:
`/home/ltr/aizee/device-maintenance/gripper-damped-20260924T193921Z/`.

- Trial binary SHA-256:
  `70a0130f82df0999df02e0f3b933e9a02daa2f27139e16ef0a697cad9928c21b`.
- Trial reports: `damped-minus-1-20260924T193948Z/result.json`,
  `damped-plus-1-20260924T194154Z/result.json`,
  `damped-minus-2-20260924T194228Z/result.json`,
  `damped-plus-2-20260924T194259Z/result.json`.
- Final stopped report under the existing diagnostic stage:
  `stopped-motors-20260924T194406Z/result.json`.

### 2026-09-24 - manual jaw measurements and offline transmission fit

Continued on the same pinned module, machine and CAN adapter identities. At
20:14:44 UTC, stopped diagnostics received all 182 replies, all 14 motors Reset
and fault-free; right-gripper position was 2.466986 rad. The operator then
reported a 48 mm inner jaw gap measured with calipers and manually moved the
jaws through three closing and three opening positions. Each measurement was
paired with a subsequent stopped encoder capture before requesting the next move.

The seven samples and failed first fit are in the
[calibration record](calibration/minerva-right-gripper-20260924/README.md).
[Full evidence](MINERVA_GRIPPER_TRANSMISSION_20260924.json) preserves all eight
diagnostic captures, raw replies, identities, hashes and operator measurements.
The user did not supply caliper accuracy; the recorded +/-0.25 mm manual-placement
allowance is explicitly provisional.

Added `robot_platform.control.transmission_calibration`, a shared offline tool
that binds measured aperture to one motor route/UID/firmware, requires both
approach directions and adequate span, checks affine/held-out error and
directional separation, and refuses extrapolation. It preserves existing output
files and never connects to hardware or installs settings.

Opening/closing slopes were 18.4196/18.5622 mm per motor radian. The common fit
had 1.1716 mm maximum residual and 1.5765 mm maximum leave-one-out prediction
error; directional separation reached 1.0645 mm. These exceed the unchanged
1 mm criteria, so **no calibration candidate was produced**. This establishes
manual jaw travel with corresponding encoder changes, not powered jaw accuracy
or a separation of backlash, compliance and measurement error. The next check
is to remeasure the released position without adjusting it.

At 20:32:49 UTC all 182 replies were again received, all 14 motors Reset and
fault-free. Right-gripper position was 2.360068 rad at the reported 45 mm opening.
All eight captures retained the UID/firmware roster and run-mode, torque/current/
speed-limit and watchdog values. Both physical CAN links returned down after
every capture; legacy service MainPIDs remained zero and port 5590 closed.
No motor enable, parameter write, firmware/flash action or runtime calibration
installation occurred in this increment. The arms stayed under manual test-stand
conditions; only the operator repositioned the gripper jaws.

Validation: **452 Python unit/contract tests passed** (21.19 s), including 20
new calibration tests. Rust code and deployed binaries were unchanged. The
previous 40-test Rust and AArch64 evidence remains applicable to those binaries.

#### Follow-up: use the existing measurements as a coarse estimate

The operator remeasured the released final gap as 45.5 mm, then explicitly
requested no more measurements, citing finger flex and caliper variation.
Stopped diagnostics at 20:35:02 UTC found 2.360221 rad, effectively unchanged
from the nominal 45 mm report. This does not isolate physical backlash from
caliper placement or flex.

Preserved the failed 1 mm assessment and all original measurements. A separate
[coarse fit](calibration/minerva-right-gripper-20260924/coarse/fit.json) uses the
45.5 mm rereading in place of nominal 45 mm, a provisional +/-1 mm per-sample
allowance and a 2 mm fit criterion. Result: aperture in mm approximately equals
**18.4104 * motor radians + 2.5535**, with 1.1688 mm maximum residual, 1.5728 mm
leave-one-out error and 1.0396 mm directional separation. The interpolation
range is 1.282715-2.020061 rad (estimated gap 26.17-39.74 mm), covered from both
directions. These error allowances are not independently validated accuracy.

This is an offline candidate for approximate unloaded geometry; live approval,
CAD zero, force/contact behavior and mechanical limits remain unverified. No
motion bounds or device readiness flags were relaxed. Additional evidence is in
[the coarse calibration update](MINERVA_GRIPPER_COARSE_20260924.json).

Final capture at **20:37:57 UTC** again received all 182 replies, all 14 motors
Reset and fault-free. Right-gripper position was 2.360260 rad, outside the fit's
shared interpolation interval. Identity/firmware and run-mode, torque/current/
speed limits and watchdog values matched the initial capture. Both physical CAN
links returned down; legacy MainPIDs were zero and port 5590 closed. No enable,
parameter write, firmware/flash operation or runtime calibration installation
occurred. Further caliper collection was stopped at the user's request.

## 2026-09-24 — shared teleop and reference preparation

Staged the shared Python supervisor, Minerva CAD assets and ARM64 Rust binaries
at `/home/ltr/aizee/device-maintenance/teleop-vcan-20260924T212423Z`. The archive
and uploaded files were hash-verified. System Python lacked collision packages;
`venv` creation failed because ensurepip was unavailable. No global packages or
services were changed. Installed isolated dependencies into that stage's
`python-packages`: numpy 2.2.6, scipy 1.15.3, trimesh 4.12.2, python-fcl 0.7.0.11,
msgpack 1.2.2, pyzmq 27.2.0 and Cython 3.3.0. The unused partial venv is not used.
An initially stale ABI-1 collision library was rejected; uploaded a freshly
cross-built ABI-3 library before tests.

The independently scheduled 400 Hz receiver and full inter-arm keep-out compiler
accepted **1,000/1,000 synthetic vcan cycles at 25 Hz**, median 14.595 ms,
p99 14.733 ms, maximum 17.227 ms, zero scheduled misses. Producer silence caused
the receiver's collision-expiry stop. Receiver: 16,075 ticks, no overruns or
deadline misses; maximum work 453 us, lateness 386 us. The initial fixture's
missing-feedback startup failure remains recorded; the successful fixture
explicitly reset that startup fault once before the session. No runtime fault
was automatically reset. Physical CAN links remained down; legacy services
remained stopped. This test uses simulated motor responses and excludes cameras.

Both PC OpenRB leaders were flashed with the source-freshness protocol and
checksum-verified. Left USB serial `86AE0E865157375037202020FF110E12` (COM7), right
`D5C58E435157375037202020FF0F2A0C` (COM4). The user moved only the left leader;
COM7 changed while COM4 remained within encoder noise. Device profiles use serial
identity, not COM number. Neither board reports a joystick. Direct USB input
completed 1,500 cycles without protocol errors but had one scheduled miss.
Read-only PC-to-Jetson input tests failed the 25 ms timeout after one and twelve
cycles. Native SSH comparison failed before sampling due to existing key ACLs;
key contents/permissions were not changed. Input transport remains unqualified.

The user manually placed both arms at the viewer's all-zero reference and
requested persistent hardware mechanical zero plus pre-enable reference checks.
Stopped capture at **21:45:38 UTC** found 13 fault-free Reset motors; left
gripper ID 10 did not respond. No zero, flash, enable or parameter writes occurred.
Physical CAN links returned down. The missing gripper connection and physical
meaning of the gripper zero were requested from the user. Complete the full
identity check before zeroing; old absolute gripper-fit offsets must not be used
after a zero change. The new finite maintenance tool verifies stopped readback;
persistence still requires a power-cycle check. The new Rust adapter also blocks
physical enable without fresh, single-use reference-pose verification.

Source hashes, staging details, failed timing tests and the measured result are
in [teleop evidence](MINERVA_TELEOP_20260924.json). See
[launch/readiness details](../GUARDED_TELEOP.md) and
[mechanical reference procedure](../MECHANICAL_REFERENCE.md). This remains an
isolated commissioning stage, not an enabled teleoperation service.

## 2026-09-24 — hardware mechanical zero, power-cycle check pending

The user confirmed fully closed inner jaw faces as gripper zero and restored the
left gripper connection. Fresh diagnostics found all 14 expected UID/firmware
pairs in fault-free Reset. The arms remained at the operator-aligned viewer zero
pose. Staged the disabled-only maintenance tool and pinned roster under
`/home/ltr/aizee/device-maintenance/mechanical-zero-20260924T220526Z`.

Preserved each interrupted attempt and its source hashes. The first preflight
rejected a left-j4 velocity-estimate spike before any zero writes. A bounded
continuous quiet window retained the same position/velocity limits. A later
parameter-read timeout occurred after left-j1 acknowledged zero. Maintenance
reads were paced and resumed without repeating that write. Type-22 save replies
varied by firmware: some returned the exact pinned UID, five left-side motors
returned no reply. Missing replies remain explicitly unverified; saves were not
repeated. Read-only diagnostics allow one bounded retry; zero and save never do.
Write intent and acknowledgements are checkpointed to preserve partial evidence.

The final verification is in the `final-readback-v6` subdirectory. Across all
attempts there were **14 unique zero transmissions and 14 unique save attempts**,
exactly one per motor, and **zero enable/control frames**. Every motor passed
repeated near-zero readings, maximum absolute deviation **0.00029690034 rad**.
All 14 final stops were fault-free Reset, legacy MainPIDs were zero, port 5590
was closed, and physical canL/canR were down. No deployed services were enabled.

Save responses are absent for left j3, j4, j5, j6 and gripper. Persistence is
pending a physical power cycle with the pose unchanged. A response or current
zero reading does not substitute for that test. The user was asked to cycle
motor power while leaving the Jetson on if separately supplied.

The new [reference record](../../config/devices/minerva-reference-20260924.json)
pins the geometry and motor roster; runtime approval, pose uncertainty and arm
directions remain unqualified. The earlier gripper fit's absolute encoder offset
is now invalid and must not be applied live. Raw measurements remain unchanged.
See [action/evidence summary](MINERVA_MECHANICAL_ZERO_20260924.json), including
hashes and all partial attempts. This does not complete physical teleop readiness.

### Power-cycle persistence verified

The user then reported a motor power cycle with no change in physical pose.
Stopped capture **2026-09-24 22:20:37 UTC** returned the complete 182 replies:
all 14 original UIDs/firmware, all fault-free Reset, and every raw motor position
within **0.00029690034 rad** of zero. No angle wrapping was applied. This includes
the five left-side motors without save responses. The record now marks one
power cycle verified, while runtime approval, motor-to-CAD direction and loaded
stopping remain unqualified. No zero/save/enable writes were used for this check.
Physical CAN links returned down, legacy services stayed stopped, and the
guarded receiver port closed. Maintenance regression suite: 517 Python tests
passed, including 15 mechanical-zero cases.

Prepared the viewer at left_arm_j1 +0.15 rad in Top view for the first manual
motor-to-CAD direction check. This viewer action is simulated only. The user is
asked to move the disabled physical left shoulder swivel in that direction;
no direction has yet been recorded from hardware.

### Manual shoulder-swivel directions recorded

After the operator moved the disabled left shoulder counterclockwise viewed
from above, stopped capture **22:26:18 UTC** measured left J1 at
**+0.405311823 rad**, a **+0.405397044 rad** change from the power-cycle baseline.
The largest change in any other motor was **0.007274012 rad** (left J4).
This establishes the observed motor-to-CAD sign **+1** for left J1.

The operator then returned the left swivel toward reference and moved the right
swivel clockwise, matching the positive right CAD axis. Capture **22:33:12 UTC**
measured right J1 at **-0.407911062 rad**, a **-0.408081505 rad** change from the
power-cycle baseline: motor-to-CAD sign **-1**. Left J1 returned to
**+0.018024312 rad** (about 1.0 degrees from zero). Right J2 shifted
**0.016234631 rad** incidentally; no other sign is inferred from this movement.

Both captures returned the complete pinned 14-motor UID/firmware roster in
fault-free Reset. They used stop/identity/parameter reads, with no enable or
parameter writes. CAN links returned down, all legacy service MainPIDs remained
zero, and port 5590 was closed. The original mechanical-zero reference and
power-cycle baseline are unchanged. The +0.15 rad viewer demonstrations were
direction cues, not measured physical angles; no scale ratio is inferred.
See [direction evidence](MINERVA_ARM_DIRECTIONS_20260924.json) for capture hashes,
remote paths and all raw before/after positions. Runtime approval remains false.

Prepared left J2 at +0.15 rad in the viewer's three-quarter view and asked the
operator to return right J1 to reference, then gently lift the left arm forward
at the shoulder hinge while supporting it. This next direction is still pending.

### Calibration moved into a local live workflow

The operator requested direct scripts and a live physical-to-CAD comparison
instead of chat-mediated joint captures. Added the shared stopped-only reader,
profile-driven terminal calibration session, and loopback live viewer feed.
The Minerva legacy calibration entry point now forwards to this workflow; it
does not enable the old controller. The two recorded J1 signs seed the session;
the remaining signs and all display ratios/offsets remain candidates.

The new finite reader verified all 14 pinned identities/firmware with six quiet
position samples each and restored CAN down. The initial live run measured
approximately 142 ms per complete 14-motor scan. A Windows file-replacement race
stopped the display producer; the viewer showed stale data. Added bounded retry
for that reader/writer race. A subsequent stopped capture confirmed CAN had
returned down and all 14 motors remained accessible in Reset. Raw failed and
successful trials are retained in the content-hashed calibration stage and local
`simulation_runs/calibration-live-smoke*-20260924` directories. No enable, zero,
save, firmware or parameter writes were performed.

See [launch and correction commands](../JOINT_CALIBRATION.md). Live preview edits
are saved separately from the mechanical-zero record and cannot approve runtime
teleoperation. Gripper CAD transmission mapping remains pending.

The bounded file-replacement retry still encountered Windows sharing failures;
the final live feed appends JSONL frames and the viewer reads the last complete
line. A sustained 5 ms CAN request spacing also encountered lost replies, with
`gs_usb` unused-echo warnings in the device kernel log. Failed trials closed both
CAN interfaces. The calibration reader now spaces transactions by 10 ms, discards
incomplete scans, and stops after three consecutive failures. Each frame is bound
to a unique request and its full round-trip age; old queued frames cannot be
presented as fresh. No CAN-driver or adapter firmware changes were made.

At **23:04:02–23:05:27 UTC**, the paced reader completed **292 full scans over
85.5 seconds**, with zero rejected CAN scans. The client discarded the first
late response and displayed the remaining 291 frames (median scan 273.35 ms,
maximum 277.97 ms). Explicit `q` exit returned a successful
cleanup report with both physical CAN links down. Full Python suite: 541 passed
before the final feed changes; final calibration/viewer regression suite: 56
passed. See [live preview evidence](MINERVA_LIVE_CALIBRATION_20260924.json) for
timings, source-stage and raw-frame paths. This finite calibration display test
does not qualify CAN transport for powered control.

### Live viewer freeze investigation and recovery (2026-09-24)

The operator reported that moving the disabled arms no longer changed the
browser. The reader from **23:08:33–23:11:30 UTC** had stopped after three
incomplete scans (`Missing stopped position: left_arm_j1`). Both CAN links were
down; the terminal was waiting for input and the browser retained its last pose.
This identifies the missing first reply on that bus, not a diagnosed J1 motor
fault. Local report: `calibration_runs/minerva-production-01-directions/`
`live-cleanup-446414e6.json`; remote report under
`joint-calibration-1ba8e1d744c0a853/captures/20260924T230833.836476Z.json`.

The kernel logged `gs_usb: Unexpected unused echo id` on canR at 19:09:06 EDT
and canL at 19:09:37/39 EDT. Captures also contain outgoing stop frames received
as CAN input before reply loss. These observations warrant USB adapter/driver
investigation; they do not prove a specific firmware defect. Kernel remains
`5.15.185-tegra`; both USB devices identify as `canable gs_usb`, VID/PID
`1d50:606f`, bcdDevice `0.00`. No kernel, driver, adapter firmware, motor firmware,
zero, flash-save, enable or motor parameter changes were made.

Implemented explicit waiting/reading/reconnecting/stopped feed states, a visible
viewport badge, frame/age display, no-store telemetry responses, a browser
freshness watchdog and stream-aware sequence handling. Large mesh compression
now runs off the HTTP event loop. Calibration mode disables collision requests
before model loading and keeps its pose controls locked. Added bounded
stopped-preview reconnection after verified CAN-down cleanup; all identity and
Reset checks run again. Network/identity/protocol failures are not automatically
retried. Capture reports now include link statistics before cleanup and frame
counts even on failure.

A subsequent run at **23:31:06–23:31:55 UTC** delivered 101 complete CAN scans
with no rejected CAN scans, then ended on the two-second client heartbeat. The
viewer correctly published FEED STOPPED. Increased this read-only heartbeat to
five seconds (SSH read timeout eight seconds), retaining the strict 750 ms pose
freshness limit. The passing 85.5-second trial above is therefore not evidence
of sustained reliability. This remaining transport issue must be resolved and
qualified before powered teleoperation.

Relevant upstream context, not a confirmed diagnosis: the Linux
[gs_usb transmit-cleanup fix](https://github.com/torvalds/linux/commit/516a0cd1c03fa266bb67dd87940a209fd4e53ce7)
addresses queue exhaustion after USB transmit failures. Our observed warnings
are unused echo IDs, so applying that patch or guessing adapter firmware has
not been justified by these captures.

The revised preview reproduced reply loss on **canR** after **447 complete
scans** at **23:35:38 UTC**, again first reporting J1 when a whole bus stopped
replying. Both links went down successfully, a fresh worker revalidated the full
hardware roster, and new poses reached the browser automatically. Recovery was
therefore exercised against an actual fault, not just a mocked timeout. The
viewport now distinguishes stale/stopped/reconnecting data and shows the number
of reconnections. All 59 focused calibration/viewer tests passed. See the
[rolling recovery observation](MINERVA_LIVE_FEED_RECOVERY_20260924.json) for raw
report paths, identities, link counters and timing; it is not a clean sustained
transport pass. Adapter make/model/board revision and firmware were requested
from the operator to guide the remaining diagnosis.

### RH-02 identification and firmware preparation (2026-09-24 local)

The operator supplied the adapter listing. Both installed devices are consistent
with the manufacturer's RH-02/Candlelight product. Read-only sysfs inventory on
the pinned host found left configuration `fw_49fb001_2026-02-16`; right has an
empty configuration string, so its revision remains unknown. Both have USB power
control `on` and runtime status `active`: autosuspend was already disabled during
the failures. Both CAN links remained down throughout this preparation.

Upstream candleLight now has an explicit `RH02_fw` target for the STM32F072 and
24 MHz crystal. Built unmodified revision
`85e5b3352db7f051aa879eeb71555dfce664c44c` in Release mode on this Jetson with
its existing Arm GCC 10.3.1 and CMake 3.22.1. The Windows GCC 7.2.1 attempt failed
at linking; that attempt produced no usable candidate. No tools were installed
and no adapter firmware, motor settings, zero references or services changed.

The successful 23,740-byte binary, ELF, map, DFU image, full source bundle and
checksums are retained locally under `simulation_runs/rh02-firmware-85e5b3352db7`
and remotely under `/home/ltr/aizee/device-maintenance/rh02-firmware-85e5b3352db7`.
This is **a candidate, not a verified fix**. The reported left revision already
contains the older January USB-buffer fixes; newer upstream changes include
USB-send error handling and cleanup of queued transfers during CAN shutdown.
These changes justify a controlled comparison but do not establish the cause of
the observed running-stream failures.

Pending: enter the right adapter's ROM bootloader physically, confirm its memory
geometry and identity, preserve a verified backup, then evaluate the candidate
on one adapter before changing the other. No flash operation or bootloader
transition has been performed. The viewer remains stopped and powered
teleoperation remains unapproved.

### Right RH-02 update (2026-09-25 00:48–00:50 UTC)

The operator placed the right adapter in BOOT mode. It enumerated as
`0483:df11` at its existing path `1-2.3.4`, with ROM serial `FFFFFFFEFFFF` and
128 KiB internal flash (64 x 2 KiB pages). Left serial/configuration remained
unchanged. All three controller services had `MainPID=0`; canL was down and
canR was absent in DFU as expected.

Read the entire flash twice and verified byte-for-byte equality, SHA-256
`255c18b8b3fc2d06bb2d9a87b011f701fd56f6ea253ed5f2a0f0e1e56a184998`.
Copies and command logs are retained on both machines. Programmed only the
right adapter with the pinned `RH02_fw` candidate (23,740 bytes), then read
all 128 KiB back. Firmware bytes matched exactly; the remainder of the final
erased page was `0xff`, and every byte beyond the 24,576-byte programmed page
range matched the original backup. No mass erase or option-byte change occurred.
The raw-bin DFU suffix warning was expected; dfu-util exited successfully and
independent byte comparison verified the flash contents.

See [update evidence](MINERVA_CAN_RIGHT_UPDATE_20260924.json) and the
[backup/rollback procedure](MINERVA_CAN_MAINTENANCE.md). Requested that the
operator return BOOT to normal and reconnect right USB. Normal enumeration,
stopped motor roster and sustained traffic are pending; no claim that the feed
fault is fixed. Left adapter, motor firmware, parameters and zeros unchanged.

The operator returned BOOT to normal and reconnected. The adapter retained
serial `0021002F4D53530920303937` and name `canR`; descriptors now read
`Jhoinrch`, `RH02 gs_usb`, `fw_85e5b33_2026-09-17`. At 00:52:27 UTC all seven
right motors passed the finite stopped identity/firmware/position capture,
with successful CAN-down cleanup.

The subsequent right-only, no-reconnection soak failed at **00:54:54 UTC** after
**420 scans / 59.87 seconds**: `Missing stopped position: right_arm_j4`.
Median scan 141.10 ms; maximum 141.73 ms. Ten outgoing parameter-request frames
were delivered back as CAN receive traffic before transmission stopped. After
cleanup there were 10 TX drops and RX exceeded TX by 20 packets; CAN error,
passive and bus-off counters stayed zero. Unlike earlier trials, no kernel
warning was found in the recovered test-window journal. The first journal
collection used an unsupported ISO timestamp and failed; it was rerun with a
supported UTC timestamp, and the audit records that correction.

Raw evidence is local `simulation_runs/rh02-right-soak-20260925T005353Z/` and
remote `joint-calibration-3a3c1bf32a472dae/captures/`
`rh02-right-soak-20260925T005353Z/`. This rules out declaring the firmware update
a cure. Both CAN links were down afterward. Requested a right-adapter direct
Jetson USB connection to isolate the external hub before changing the driver.

### Direct Jetson USB comparison (2026-09-25 01:42–02:12 UTC)

The operator connected both adapters directly to the Jetson. Serial-based names
followed them correctly: left `004D002C4730511520303650` moved from `1-2.3.2` to
`1-2.4`, right `0021002F4D53530920303937` from `1-2.3.4` to `1-2.1`. The external
second-level hub was bypassed; the Jetson's onboard USB hub remains in the path.
No udev, kernel, firmware, motor parameter or zero changes accompanied this test.

The unchanged right-only stopped-reader workload completed **1,800.010 seconds /
12,731 complete seven-motor scans**, with no automatic retries, rejected scans,
unexpected outgoing frames received as input, or kernel warnings. Median scan
141.108 ms, p99 141.649 ms, maximum 143.100 ms. All 178,262 raw transmitted and
178,262 received events were retained. CAN cleanup succeeded; left CAN stayed
down throughout. The reader source and test-worker hashes match the failed
external-hub comparison. See [direct USB evidence](MINERVA_DIRECT_USB_20260924.json)
for counters, identities, raw artifact paths and hashes.

This result implicates the external USB path, without proving a component-level
cause. Retain the direct wiring. It establishes stopped-reader stability for
this right-only workload, not 25 Hz powered operation or full dual-arm endurance.

Separately fixed healthy expiry of the calibration reader: each remote worker
still stops after 15 minutes and verifies CAN-down cleanup. The client then
repeats complete host/adapter/motor identity checks and starts a fresh reader.
Healthy renewal does not reset or consume the three-fault retry budget; unknown
cleanup still stops the feed. Explicit `q` ends the session. Calibration/viewer
regression checks: **67 passed**. This code change was made while the unchanged
comparison workload ran and was deployed only when restoring the live viewer.

### Direct USB dual-arm follow-up (2026-09-25 02:13–02:15 UTC)

Restored the live calibration viewer with the new finite-reader renewal code.
The first dual-arm attempt failed after **210 complete scans / 69.83 seconds**,
with three rejected scans: `Missing stopped position: left_arm_j1`. Left's
receive log contained ten outgoing command frames, RX exceeded TX by 20 packets, and
cleanup added ten TX drops. CAN error/passive/bus-off counters remained zero.
The kernel also logged `Unexpected unused echo id 0` on **canR** at startup of
both attempts (02:13:16 and 02:14:29 UTC), although right did not lose its replies.

The reader performed one bounded automatic recovery after verified CAN-down
cleanup and identity checks. The second attempt delivered 177 complete scans
over 52.51 seconds before explicit `q` shutdown; three unexpected outgoing
frames were already in that capture, so successful shutdown is not a clean
transport pass. Both CAN links were independently verified down, with all three
controller services at `MainPID=0`. The viewer was closed. Motor enables,
parameters, firmware and mechanical zeros were unchanged.

See [complete dual trial evidence](MINERVA_DIRECT_USB_DUAL_20260924.json), including
raw frames, both cleanup reports and successfully collected kernel logs. The
right-only 30-minute pass is valid for that workload; it does not establish
dual-arm reliability. Next comparison is the same verified firmware on the left
RH-02, after its own full verified backup and physical BOOT selection. If dual
traffic still fails, isolate single/dual USB traffic and the installed driver
before further firmware changes. No powered trial is qualified by these results.

### Left RH-02 backup and update (2026-09-25 02:21 UTC)

The operator put left USB in BOOT mode and reconnected to its direct port.
Read-only preflight confirmed STM32 DFU `0483:df11` at `1-2.4`, ROM serial
`FFFFFFFEFFFF`, with 128 KiB internal flash (64 x 2 KiB pages). `canL` was absent;
right serial `0021002F4D53530920303937` at `1-2.1` still reported
`fw_85e5b33_2026-09-17`, with CAN down. All three controller services had
`MainPID=0`. The ROM serial is shared, so target selection used the physical
port and verified peer identity.

Read all 131,072 bytes twice and verified exact equality, SHA-256
`d3eed13ca4e83f2ea68ce0cc4b1dcdcde72481a03e78654c82b3c05ac34c5f66`.
The first upload cleared a DFU error state before reading; both uploads then
completed successfully. Originals and complete command logs are preserved on
Windows and the Jetson.

Revalidated the backups, candidate hash, device/peer identity and inactive
controllers immediately before writing. Applied only the 23,740-byte pinned
`RH02_fw` payload and read all 128 KiB back. Firmware bytes match exactly;
final-page padding is `0xff`, and every byte beyond the 24,576-byte programmed
page range matches the original. Full readback SHA-256:
`b21f183c601cd1144c575573468e1eceed72a3f0e88b4bbc5a89b1dfb3f9ae65`.
The raw-bin DFU suffix warning was expected; the command returned success and
independent readback comparison passed. No mass erase or option-byte change.

See [left update evidence](MINERVA_CAN_LEFT_UPDATE_20260924.json) and the
[rollback procedure](MINERVA_CAN_MAINTENANCE.md). Normal-mode enumeration and
stopped dual-arm testing remain pending the physical BOOT switch. Right adapter
firmware, motor firmware, parameters and mechanical zeros were unchanged. This
update does not by itself establish a fix for the transport failure.

### Both updated: short dual test (2026-09-25 02:24–02:26 UTC)

The operator returned left BOOT to normal and specifically requested no full
30-minute test. Both runtime serials, direct ports and the new configuration
`fw_85e5b33_2026-09-17` were verified. The stopped reader verified all 14 pinned
motor UIDs, firmware revisions and Reset mode, then ran with a **180-second
cap**, no automatic retries and immediate exit on a failed scan.

It completed **498 scans**, then failed at **141.343 seconds** with
`Missing stopped position: left_arm_j4`. The named joint is the first missing
transaction, not evidence of a defective J4. Left again delivered ten outgoing
stop/read frames as received data and accumulated ten TX drops on cleanup.
No RX/TX error counts, CAN error/passive/bus-off counters or kernel warnings
were observed. Right had no unexpected received commands or error/drop deltas.
Median complete scan 282.339 ms; p99 282.964 ms; maximum 283.135 ms. Both CAN
links were verified down at exit. No motors were enabled or re-zeroed.

See [short dual test evidence](MINERVA_CAN_DUAL_SHORT_20260924.json) for the full
report, counters, USB identities and hashes of the raw frames/kernel log. The
short retest establishes that matching adapter firmware still does not fix the
failure; it is not an endurance qualification. The viewer remains stopped.
Requested swapping only the two USB plugs at the Jetson while retaining each
adapter's cable and CAN wiring, to distinguish host-port behavior from the
left adapter/cable/arm path. No further firmware or driver change was made.

### USB port change comparison (2026-09-25 02:29–02:30 UTC)

The operator moved the plugs, explicitly noting that this was not necessarily a
swap. Inventory found left serial `004D002C4730511520303650` at `1-2.1` (right's
previously tested port) and right serial `0021002F4D53530920303937` at `1-2.2`
(a third port). Names and matched firmware remained correct. The same unchanged
worker, 10 ms request spacing and 180-second cap were used.

The reader failed after **127 complete dual-arm scans / 36.453 seconds** with
`Missing stopped position: left_arm_j6`. Left again had ten outgoing frames
received as input and ten TX drops after cleanup; no CAN error counters or
kernel warnings. Right had no unexpected received commands or error/drop deltas.
Both CAN links went down and no automatic reconnection occurred. Median scan
282.298 ms; maximum 282.915 ms. See [port-change evidence](MINERVA_CAN_PORT_CHANGE_SHORT_20260924.json).

The failure stayed with left on a different host port. This reduces suspicion
of its original port, while leaving adapter/cable/arm and per-device driver
behavior unresolved. A final short left-only comparison was started to determine
whether the failure requires concurrent dual-adapter traffic. No additional
physical rewiring or firmware change accompanied that comparison.

### Left-only comparison (2026-09-25 02:31 UTC)

Kept right CAN down and ran the same stopped-read traffic on left alone at its
new `1-2.1` port, with the same 180-second cap and no retries. It failed after
**119 complete seven-motor scans / 17.255 seconds** with
`Missing stopped position: left_arm_j1`. Ten outgoing parameter requests were
received as CAN input; cleanup recorded ten more left TX drops. No CAN
error/passive/bus-off counters or kernel warnings. Median complete scan
141.104 ms; maximum 141.741 ms. See
[left-only evidence](MINERVA_CAN_LEFT_ONLY_SHORT_20260924.json).

A fresh read-only host check confirmed both physical CAN links down and all
three controller services at `MainPID=0`. The viewer was not restarted. The
failure does not require concurrent dual-arm traffic or the original left USB
port. Its cause remains unresolved among the left adapter/cable/CAN path and
per-device host/firmware behavior. Next replace only the left USB data cable,
keeping its current USB port and CAN wiring unchanged. No additional firmware,
driver, motor parameter or mechanical-zero writes were made. All comparisons
stopped at the first failure, and no full 30-minute test was run in this session.

### Left termination change (2026-09-25 02:34–02:37 UTC)

The operator changed the left adapter's resistance/termination switch and asked
for a retest, reporting its new state as **"Should be off now."** Record this as
operator-reported OFF, not an independently measured electrical state. No USB
cable replacement was reported. Ports stayed left `1-2.1`, right `1-2.2`; both
firmware strings, serials and all 14 motor identities were rechecked.

The same bounded dual stopped-reader workload completed **637 full scans /
180.086 seconds**, zero rejected scans and zero reconnections. Median scan
282.236 ms; p99 282.761 ms; maximum 283.019 ms. Both CAN links went down at the
normal duration limit. No motor enables, firmware/parameter/zero writes or
30-minute endurance test were performed.

Despite complete readings, transport was **not clean**. Four outgoing parameter
requests appeared as receive traffic on left, each immediately after the same
request was transmitted and before its valid motor response. Right logged four
`Unexpected unused echo id` warnings during the run. Each bus's TX-drop counter
increased by four at cleanup; RX/TX error and CAN error/passive/bus-off counters
did not increase. The two sets of four events are not established as paired
causal events. The improved finite outcome does not prove that the switch change
fixed the underlying problem. See [termination evidence](MINERVA_CAN_TERMINATION_SHORT_20260924.json).

The viewer remains stopped. Preserve the recorded switch/port state while
isolating the left USB cable next; USB-level tracing is a subsequent option to
locate the remaining transmit-echo accounting fault. All records and raw backup /
readback artifacts remain on Windows and the identified Jetson.

### Left short USB cable removed (2026-09-25 02:51 UTC)

The operator removed left's short USB cable; right's remains. USB paths also
changed from left `1-2.1` / right `1-2.2` to left `1-2.4` / right `1-2.1`, so
this comparison cannot isolate the cable as its only changed variable. Both
adapters still enumerate as full-speed USB (12 Mbps); a USB 3.0 cable label
does not change the adapter's operating speed.

Using the unchanged stopped-reader workload and 180-second maximum, all 14
motor identities passed. **94 scans completed in 27.094 seconds**, then
`left_arm_j5` failed to reply. Ten outgoing stop/read frames appeared as left
received traffic. TX-drop counts remained unchanged during scanning and rose
by ten only on left link-down cleanup. Right had no observed anomalies during
this short run. No kernel warnings or CAN error-state counter changes were
recorded. Both links went down, with zero automatic reconnects. No motor enables,
zero/parameter writes, firmware changes or long endurance run were performed.
See [cable-removal evidence](MINERVA_CAN_LEFT_CABLE_REMOVED_20260924.json).

The removed left cable is not necessary for this failure. The receive copies
in preceding trials matched that same adapter's immediately preceding command,
including during left-only traffic. The reader uses one socket per bus and
does not request its own messages. However, the old capture method discarded
message-provenance flags, and detailed CAN bus-error reporting was off. Earlier
zero error counters therefore do not exclude intermittent electrical faults.

Read-only disassembly of the installed `gs_usb.ko` confirmed three missing
upstream fixes: failed USB transmit cleanup and received header/payload length
validation. The binary hash, offsets, source references and limitations are in
the [driver audit](MINERVA_CAN_DRIVER_AUDIT_20260924.md). These establish driver
deficiencies, not proof that any one caused this observed failure. No driver was
replaced. The next bounded diagnostic adds receive flags and detailed CAN error
observation while keeping the original stopped-only command allowlist.

### Receive-provenance diagnostic (2026-09-25 03:01–03:02 UTC)

An isolated worker wrapped the original maintenance sockets with `recvmsg`,
preserving transmitted bytes and the unchanged stopped-only allowlist. It also
logged discarded drain frames and subscribed a separate socket to CAN error
frames only. Nine offline checks covered byte preservation, flags, filters,
initialization failures and mode restoration before deployment. This was capped
at 60 seconds, with no automatic reconnects and exit on the first failed scan.

Both attempts to enable `berr-reporting` with the interfaces down returned
`RTNETLINK answers: Operation not supported`. The original mode was retained
and verified at cleanup. Error sockets received no frames, but detailed physical-
layer reporting therefore remains unavailable; this is not electrical validation.

All 14 identities/Reset checks passed; **211 scans completed / 60.135 seconds**,
zero rejected scans, no kernel warnings. Nevertheless, **five left and three
right commands arrived as received frames**, all with `message_flags=0`:
neither `MSG_DONTROUTE` nor `MSG_CONFIRM` was set. Each command socket explicitly
reported default `receive_own_messages=0`. This excludes ordinary local-loopback
delivery as the explanation for these particular copies; it does not by itself
distinguish adapter firmware, driver misclassification, or actual wire reception.
At cleanup, TX drops rose by exactly five on left and three on right; during-run
drop/error counters stayed unchanged. See [provenance evidence](MINERVA_CAN_PROVENANCE_20260924.json).

Both interfaces were verified down, the original BERR settings restored, and no
motor enables/zero/parameter writes or driver/firmware updates were performed.
The extra logging affects timing, so this run is diagnostic evidence rather than
a throughput benchmark. The viewer remains stopped. Next prepare a kernel-
compatible backport of the audited fixes, with USB completion/length/echo
instrumentation and an original-module rollback; the root cause remains unproven.

### Adapter/driver/USB/CAN investigation (2026-09-25 UTC)

The operator restored the short USB cable on left; right's cable remains.
Read-only inventory at 03:10 UTC again pinned left serial at `1-2.4` and right
serial at `1-2.1`, both firmware `85e5b33`, 12 Mbps. Both adapters had
`power/control=on`, active state and zero runtime suspended time. Both share
the onboard `1-2` hub. Its historical suspended time was not linked to a failure.
The 150 mA descriptor is not an actual current measurement; Jetson rail readings
do not measure voltage at either USB connector. Raw inventory and a copy of the
original driver are in `simulation_runs/rh02-leads-20260925T031025Z/`.

Primary-source research covered RH-02, CANable/UCAN, STM32F072 firmware queue
bugs, ten-slot host-driver exhaustion, suspend recovery and multiple-adapter
reports. The known firmware fixes are already in the installed candidate; no new
reproducible firmware defect or resolved exact RH-02 match was established.
See [field research](MINERVA_CAN_FIELD_RESEARCH_20260925.md) and
[firmware audit](MINERVA_CAN_FIRMWARE_RESEARCH_20260925.md). No adapter was flashed.

The operator expects right termination only at its adapter and suspects an
internal resistor in left's older RS00 gripper. Initial readings were **60 ohms
on both buses**, but the operator then clarified **only the motor supply was
off; adapter USB remained powered**. These are not accepted unpowered resistance
measurements. Requested a repeat with both USB connections removed, CAN wires
and termination switches unchanged. No switch change was prescribed from these
provisional readings.

The official stable Linux 5.15.185 driver, rebuilt with the existing Jetson
headers, matched all 1,410 instructions and 201 relocation records in the installed
module. A targeted five-patch candidate adds the audited transfer fixes, balanced
receive-request lifetime, advertised BERR support and opt-in bounded USB metadata
logging. Baseline and candidate builds ran as ordinary `ltr`; no root install or
module load occurred. Full candidate SHA-256:
`e3153247afcc9c2468c353015fc01f5821fc6bef430c1e6d2d79eae210269323`.

Compatibility checks found the stock CAN/NVMe module-layout CRC matches the new
build, all 49 candidate symbol CRCs match its build tree, and all 21 symbols shared
with inspected stock modules agree. Runtime/header configs differ in compiler
version fields and GCC plugin support; ordinary version checks are retained.
The original custom module's incomplete CRC metadata was not copied or stripped
into the candidate. Compilation/static checks do not prove runtime compatibility.
See [backport evidence](MINERVA_CAN_DRIVER_BACKPORT_20260925.md).

The reproducible patch bundle and guarded installer/rollback are tracked under
`scripts/maintenance/gs_usb/` and staged at
`/home/ltr/aizee/device-maintenance/gs-usb-backport-20260925/maintenance/`.
Independent review found two installer issues, both corrected before use:
rollback no longer depends on the staged candidate, and unbound/DFU USB devices
cannot bypass the disconnect check. All **18 offline tests passed**. The
unprivileged device check correctly refused while adapter USB was still connected;
source reproduction and ABI comparisons completed independently.

All three controller services remain stopped and disabled. Normal SSH provides
no passwordless module-install privilege, so the operator was given a concrete
sudo install/reboot command after build, review and tests. The installer writes
only a hash-pinned module override, preserves the original, records the change,
and never hot-unloads or reboots itself. Activation/hardware validation is pending;
no new CAN traffic, motor enables, mechanical-zero/parameter writes, firmware
updates or long endurance tests were run in this investigation phase.

### Unpowered resistance repeat and install preflight refusal (2026-09-25 UTC)

Following clarification that the first resistance readings were taken with
adapter USB still powered, the operator repeated the measurements with **all
power off and both adapter USB connections unplugged**. Both buses still read
**60 ohms**, according to the operator. These repeat measurements supersede the
initial readings for evaluating aggregate DC termination resistance; the earlier
powered-reading history remains above. They do not locate the two terminators,
verify the ground-reference path, or demonstrate dynamic signal quality. No
termination switch or CAN wiring change was made on the basis of these readings.

The operator then attempted the prepared sudo driver installation through
Windows OpenSSH. OpenSSH refused the private key because its ACL included
`CodexSandboxUsers`; password login and sudo nevertheless succeeded. The
installer rejected the attached interfaces **`1-2.4:1.0` and `1-2.1:1.0`** during
its preflight disconnect check. This is a later connectivity observation, not a
contradiction of the operator's reported unplugged measurement conditions.

The refusal occurred before the installer's mutation phase: it did not install
the candidate override, run the installation's `depmod`, unload a module, or
reboot the Jetson. The candidate remains staged and hardware validation remains
pending. The original driver and motor configuration were not changed by this
attempt. Keep adapter USB disconnected for the guarded install and clean reboot;
the old driver must not be hot-unloaded. See the
[maintenance recipe](../../scripts/maintenance/gs_usb/README.md).

### Driver override installed and clean reboot verified (2026-09-25 03:39–03:40 UTC)

After the earlier disconnect-guard refusal, the operator successfully ran the
guarded installer with the adapters unplugged. Receipt
`20260925T033923.485497Z-install.json` records installation at **03:39:23.485497
UTC**. The installer selected the hash-pinned override and preserved the
original module; it did not hot-unload a driver or reboot the machine itself.
The operator then performed the clean reboot.

A read-only check at **03:40:46.108645 UTC** reverified the pinned machine ID,
Jetson module serial and kernel `5.15.185-tegra`. Boot ID changed from
`fad104c4-6ec4-4ee6-b371-8f87bbaa79b9` to
`b5206e44-4643-461e-a81a-f8121c8e24a3`, confirming a new boot. Module lookup now
selects `/lib/modules/5.15.185-tegra/updates/aizee/gs_usb.ko`, SHA-256
`e3153247afcc9c2468c353015fc01f5821fc6bef430c1e6d2d79eae210269323`.
The receipt's resolved `/usr/lib/modules/...` path refers to the same selected
override. The original module and preserved backup both retain SHA-256
`8ff2f6fadea5b591a20976fa7114b2823e4c10aa048a979b347f9356acc0105e`.

All three controller services were **inactive, disabled, and `MainPID=0`**. No
guarded receiver, USB-CAN adapters, or `canL`/`canR` links were present. Because
the adapters remained unplugged, **`gs_usb` was not loaded**. Successful disk
installation and reboot therefore do not yet establish module-loading success,
working CAN transport, or hardware qualification.

The [installation and reboot evidence](MINERVA_CAN_DRIVER_INSTALL_20260925.json)
includes the installation receipt; a copy is retained in
`simulation_runs/gs-usb-backport-20260925/post-install-boot.json`. No CAN traffic,
motor enables, motor parameter/zero writes, or adapter firmware changes were
performed during this verification. Awaiting the operator's USB and motor-power
reconnection for module verification and a **60-second maximum stopped-reader
diagnostic**; no long endurance run is planned.

### Patched driver loaded; left gripper identity missing (2026-09-26 00:01 UTC)

At approximately **00:01 UTC / September 25, 20:01 EDT**, read-only inventory
confirmed the same post-install boot ID,
`b5206e44-4643-461e-a81a-f8121c8e24a3`, and the selected driver SHA-256
`e3153247afcc9c2468c353015fc01f5821fc6bef430c1e6d2d79eae210269323`.
The patched `gs_usb` module was now **loaded**, exposing `diag_frames=0`.
Left adapter serial `004D002C4730511520303650` was at `1-2.4`; right serial
`0021002F4D53530920303937` was at `1-2.1`. Both retained firmware
`fw_85e5b33_2026-09-17` and USB power policy `on` (runtime autosuspend disabled).
All three controller services remained stopped and disabled, and no leftover
capture process was found.

The stopped diagnostic in `simulation_runs/rh02-provenance-20260926T000101Z/`
had a **60-second maximum**, but exited during identity preflight before any
scan-loop iteration. Left IDs **4 through 9** passed UID, status, firmware, and
run-mode checks, producing 24 replies. Left gripper **ID 10 returned no reply to
either of the two existing bounded identity queries**. This identifies the
missing transaction; it does not establish whether the cause is its power,
wiring, motor, or transport. Left counted **26 TX / 24 RX**; right counted
**0 TX / 0 RX** and its motors were not tested.

Unlike the original driver, the patched driver accepted detailed CAN bus-error
reporting on both interfaces. There were no recorded CAN error frames,
unused-echo/USB warnings, outgoing commands misclassified as received data,
counter errors, or drops, including link-down cleanup. Cleanup restored the
original bus-error-reporting OFF setting and verified both CAN links down.
There were no motor enables, parameter/mechanical-zero writes, or automatic
reconnections.

See [driver retest evidence](MINERVA_CAN_DRIVER_RETEST_20260926.json). Module
loading and BERR support are verified, but this brief preflight **does not prove
the echo fault is fixed or qualify the full transport**. The right arm and the
dual scan loop remain untested with the patched driver. Requested that the
operator switch motor power off, check/reseat the left gripper's power and CAN
connections, then restore power for a further stopped diagnostic. No new
firmware write or long endurance run was performed.

### Clean bounded stopped-reader pass after gripper reseating (2026-09-26 00:04–00:05 UTC)

After the operator reseated the left gripper's connections, the next patched-
driver diagnostic verified **all 14 motor identities and stopped states**. The
earlier gripper availability failure did not recur. This demonstrates restored
availability after reseating; it does not identify the precise failing
connection or component.

Run `simulation_runs/rh02-provenance-20260926T000448Z/` started at
**00:04:47.429423 UTC** and finished at **00:05:48.441075 UTC**. The capped scan
loop completed **211 full 14-motor scans in 60.124249875 seconds**, with zero
rejected scans, retries, or reconnections. Complete-scan latency was median
**284.3613 ms**, p99 **285.9727 ms**, maximum **292.0312 ms**. Each bus's packet
counters increased by **2,982 TX and 2,982 RX**, including identity preflight.

The independent report/log check found **zero outgoing-command copies, CAN
error frames, kernel echo/USB warnings, or counter errors/drops**. Error/drop
and CAN-state counter deltas stayed zero both during scanning and through
link-down cleanup; the kernel log contained only `-- No entries --`. Detailed
CAN bus-error reporting was active on both interfaces and successfully restored
to its original OFF setting afterward. Both links were verified down. All
motors remained disabled; no motor enable, parameter/mechanical-zero write,
adapter flash, or 30-minute test was performed.

See [bounded short-pass evidence](MINERVA_CAN_DRIVER_SHORT_PASS_20260926.json).
This is the first clean bounded dual-arm stopped-reader pass recorded after
the driver installation. It is **not root-cause proof, endurance qualification,
or a 25 Hz control benchmark**. This diagnostic intentionally spaces requests
by 10 ms, yielding approximately **3.5 Hz for the complete 14-motor roster**.
The driver update and gripper reseating are separate changes in this history;
the successful run does not establish which change explains every earlier
failure mode.

### Live calibration resumed (2026-09-26 UTC)

At the operator's request, resumed the stopped-only calibration session at
http://127.0.0.1:8766/ after the clean bounded driver comparison. Verified the
model renders and receives advancing LIVE frames with zero reconnects. An
initial browser warning cleared on reload; the live reader continued throughout.
Motors remain disabled; collision protection is inactive in this calibration
preview. Existing two J1 direction signs and candidate offsets were preserved;
no mappings, mechanical zeros or motor parameters were changed. Gripper slide
mapping remains pending. The interactive session remains running for hand
movement, with its existing cleanup/renewal policy.
See [live session evidence](MINERVA_LIVE_CALIBRATION_20260926.json).

### Live direction corrections (2026-09-26 UTC)

The operator reported that only right-arm J3 and J4 move backward in the live
model. Applied `flip right_arm_j3` and `flip right_arm_j4` through the active
calibration terminal. Both candidate signs are now -1 with zero offsets;
confirmed saved state, fresh feed mapping and visible browser rows agree.
Post-correction visual confirmation remains pending. Hardware zeros and motor
parameters were unchanged; no runtime approval was granted. The feed remains
active. See [direction edit evidence](MINERVA_LIVE_DIRECTION_EDITS_20260926.json).

### Confirmed arm directions and live finger preview (2026-09-26 UTC)

The operator confirmed the corrected right J3/J4 directions and reported all
other arm directions correct. All twelve live direction confirmations are saved;
reference uncertainty, angular scale, physical limits and runtime approval remain
separate. Both physical grippers were reported fully closed at their zero.
A left-only manual opening changed its raw reading from -0.000115 to +2.486928 rad
while right stayed near zero, establishing the left opening direction.

Added generic profile-based rotary-to-prismatic display mappings. Each CAD master
slider is zero when closed and drives its own URDF mimic finger. The old right
total-gap slope supplies an approximate 9.205 mm/rad per-jaw scale; left magnitude
is borrowed and unverified. The old absolute intercept and interpolation range
remain invalid after mechanical zero and were not reused. No hardware zero,
motor parameter or live controller config was changed.

Preserved the old profile and session, verified all non-plan bindings and rotary
edits unchanged, and recorded an explicit session migration. The preceding live
reader closed cleanly after 2,218 frames with zero rejected scans and CAN down.
Restarted the stopped-only preview; all 14 rows now update, left shows the manual
opening and right shows closed. There were zero reconnects at validation.
The shared mapping tests passed (85); JavaScript syntax and independent review
also passed. See [preview evidence](MINERVA_GRIPPER_PREVIEW_20260926.json) and
[left direction observation](MINERVA_GRIPPER_DIRECTION_20260926.json). The viewer
remains active for operator verification; mappings are display-only.

### GELLO input viewer and virtual teleop (2026-09-26 UTC)

The operator accepted the finger preview and requested dual GELLO operation,
followed by Quest Pro/WebXR. Added a robot-independent read-only leader schematic
with profile-defined links/axes, raw encoder values, display reference capture,
direction flips, and separate saved display settings. USB serial identity remains
pinned; neither servo zeros nor follower motor settings are changed.

On the operator PC, polling the two boards in parallel raised throughput from
45.65 Hz to 99.58 Hz at a requested 100 Hz. The 10-second parallel test returned
996 complete pairs, no protocol errors, maximum source age 15.23 ms, and maximum
pair skew 1.93 ms. Four cycles exceeded their 10 ms schedule. The browser displays
roughly 53–56 updates/s independently of the initial 25 Hz follower target.
An expired cached snapshot now freezes the read-only viewer and permits display
recovery from genuinely fresh samples; USB/protocol errors remain latched and
teleop clients still stop on stale input without automatic rearming.

The real stationary leaders also drove the shared mapper/collision compiler and
virtual Rust receiver for 250/250 accepted cycles at 25 Hz: median 6.90 ms,
p99 10.99 ms, maximum 13.45 ms, no scheduled/cycle deadline misses, and the
silent collision-producer expiry stop passed. This used synthetic follower
coordinates and bounds; the leaders varied by at most one tick. It qualifies
neither moving-leader trajectories, PC-to-Jetson transport nor physical stopping.

The operator explicitly requested retaining the existing GUI Idle, Enable,
Engage/Align, and slow-to-zero Shutdown flow. Hold-to-run is deferred, and no
replacement Start/Stop workflow is being introduced. The shared input and
collision paths will integrate behind existing controls. Actual flow differences
and migration gaps are in the [GUI integration map](../architecture/TELEOP_GUI_INTEGRATION.md).

Validation: 110 focused Python tests passed; JavaScript syntax and live browser
reference-guide/direction checks passed. The workbench remains at port 8767;
the stopped Minerva preview remains at port 8766. No physical motor enable,
CAN configuration, Jetson deployment, firmware change, or runtime approval was
performed in this increment. See [compact evidence](MINERVA_GELLO_VIEWER_20260926.json)
and [operator instructions](../GELLO_TELEOP.md).

### GELLO hanging layout correction (2026-09-26 UTC)

The operator supplied a photo showing overhead-mounted GELLO leaders arranged
like Minerva and confirmed both remained in that hanging pose. Captured fresh
raw readings through the existing single-owner input endpoint and saved them as
software display references. Corrected the profile to hanging upper/forearm
links, mirrored shoulder/elbow/wrist brackets, an overhead fixture and Minerva's
rotary-axis arrangement. USB side identities and prior direction signs were
preserved. Dimensions remain schematic; encoder directions still need a manual
movement check.

The generic viewer now accepts link displacement vectors, fixed base/joint
rotations and fixture geometry. Old positive-Z profiles remain supported.
Restarted only the leader viewer and verified the front view and live zeroed
readings (within one encoder tick). Forty-nine focused viewer tests, JavaScript
syntax and seven compound-transform checks passed. No motor zero, servo setting,
follower command or Jetson configuration changed. See
[layout/reference evidence](MINERVA_GELLO_HANGING_LAYOUT_20260926.json).

### Hand-swept joint limits in Helm (2026-09-26 02:49–03:00 UTC)

The operator swept all 14 disabled motors by hand in [Helm](../HELM.md) and saved
[`config/devices/minerva-production-01-joint-limits.json`](../../config/devices/minerva-production-01-joint-limits.json),
SHA-256 `a8e35d360b2d8a91d1016fa8eb554c09eb9424345fdfb6d528f4d4312bd05545`, saved
03:00:04 UTC. It validates against reference `minerva-mechanical-reference-20260924`
and every motor's bus, CAN id, UID and firmware. Values are raw motor radians; an
end counted only when two consecutive stopped scans agreed. Stopped reads only:
no enable, zero, save or parameter write. No direction edits were made.

| Joint | Left min / max (rad) | Right min / max (rad) |
|---|---|---|
| J1 | −1.557 / +0.842 | −0.829 / +1.611 |
| J2 | −1.600 / +1.573 | −1.563 / +1.615 |
| J3 | −2.051 / +2.133 | −2.152 / +2.024 |
| J4 | −1.122 / +1.282 | −1.494 / +1.827 |
| J5 | −3.274 / +3.267 | −3.413 / +3.238 |
| J6 | −1.163 / +1.066 | −0.935 / +0.973 |
| Gripper | −0.021 / +5.220 | −0.002 / +5.229 |

Through the recorded signs, J1–J3 fall within about 0.05 rad of the CAD viewing
ranges; J1's strongly asymmetric range matches CAD on both arms, independently
supporting left +1 / right −1. J4 and J6 are narrower than CAD. Left J4 spans
2.40 rad against right 3.32 rad because the operator stopped short of its lower
travel: wiring at left J4 sparked there. Its saved minimum is therefore a
deliberate, conservative limit, not the mechanical stop; do not re-sweep into
that range until the harness is repaired. Both J5 wrists travel about 6.6 rad,
beyond the model's ±π. Both grippers
open about 5.22 rad from closed; with the unverified display scale that is about
48 mm per jaw, consistent with the 47.4 mm CAD finger travel. None of this is
approved for motion (`runtime_approved: false`).

Electrical faults persisted during the session (`calibration_runs/helm/20260926T024938/`):
left J5, J6 and gripper (IDs 8–10) were silent together in two episodes (at
connect, and from 02:55:46 UTC), and the left gripper alone dropped out in five
more (02:56:50–02:59:33 UTC). All 14 were silent 02:50:11–02:51:52 UTC during the
operator's power cycle. Helm resumed each time the motors answered, with CAN-down
cleanup verified after every dropout. Right J3 once reported fault bit 16
(undervoltage) in its Reset reply at 02:56:58 UTC, which cleared by the next
connection. The sparking at left J4 is the likely common cause: a damaged harness
past J4 would drop J5, J6 and the gripper, and a short can dip the shared supply.
Repair the left J4 harness (and check the gripper connector and the supply)
before any powered motion.

### 2026-09-26 — harness repaired, temporary stop switch, operator link measured

The operator reported the left J4 harness repaired (no further sparking) and a
foot-operated power switch in place as the temporary emergency stop. No motor
command was sent in the work below.

Operator link, PC to Jetson (60 s at 100 Hz, UDP echo, no CAN):

| Path | Round trip p50 / p99 / max | Jetson arrival gap p99 / max | Lost |
|---|---|---|---|
| Wi-Fi (`wlP1p1s0`, 2.4 GHz channel 6, power save on) | ~20 / 457–844 / 785–1078 ms | 36 / 165 ms | 0 |
| Wired USB Ethernet tether (`enP8p1s0` 10.42.0.1) | 0.5 / 2.9 / 21.9 ms | 11.8 / 20.6 ms | 0 |

Wi-Fi is unsuitable for the control path; the tether is. The arms' control loop
will run on the Jetson, with leader samples requested over the tether by the
authenticated UDP leader link. End to end with the real leaders (25 Hz, 60 s):
1497 of 1500 cycles fresh, 3 skipped (never more than 2 in a row), sample age
p50 18.6 / p99 36.6 / max 50 ms, no faults. The Jetson's Wi-Fi power saving was
left unchanged.

### 2026-09-26 — first release deploys and Minerva Check

The guarded runtime now reaches the Jetson as verified releases under
`~/minerva/releases/` (user space, no sudo). Release `20260926T203521Z-2c39fb2`
is current: 104 files, 2.3 MB. Its smoke test loads the collision model in
0.8–0.9 s, with keep-out query p50 1.5 ms.

The first Minerva Check (20:32 UTC) found two problems:

- **Health probe crash.** An unreadable thermal zone crashed the Jetson health
  probe, so the motor stage was skipped. Fixed.
- **Late leader replies.** With the real leaders, 121 of 125 link cycles were
  fresh. Some PC replies missed the robot's 20 ms timeout because the PC end ran
  as a thread of a normally scheduled process. It now runs in its own process
  that opts out of Windows throttling.

The second check (20:35 UTC) passed every Operator PC and Link and Jetson check:

- **Leader link.** 124 of 125 cycles fresh from the real leaders. Sample age
  p50 21.5 / p99 39.8 ms (limit 50 ms), never more than one cycle skipped in a
  row, and the PC side skipped nothing.
- **Jetson.** 37.6 GB disk free, 6.7 GB memory available, hottest zone
  44.2 °C, 25W power mode with the performance governor, clock within 0.1 s of
  the PC.
- **CAN.** The helpers and serial pinning are installed. Both adapters were on
  their pinned serials and down, and no motor service was running.

The stopped scan then found **no motor answering on either bus**. Both CAN
controllers went error-passive (no node acknowledged a frame), so motor power
was off or the foot switch was pressed. CAN links were confirmed down afterwards,
and no motor was enabled.

Evidence: `calibration_runs/minerva-check/20260926T203544Z.json` on the PC and
`~/minerva/evidence/check-motors-20260926T203544Z.json` on the Jetson.

### 2026-09-26 — teleop loop on the Jetson (virtual receiver)

Helm's teleop loop now runs on the Jetson from the deployed release. It uses the
virtual receiver (the native harness), so no motor was commanded. The real
leaders are served from the PC over the authenticated leader link on the tether.

**Session run.** Start-up took 2.3 s. The lifecycle went Idle, Enable, then
Engage (TELEOP after about 1.3 s of alignment), followed for 30 s, then Disable,
and the session stopped cleanly.

**Cycle times, same session with the loop on each computer:**

| Loop on | Cycle p50 / p99 / max | Overruns | Largest gap between cycles | Late leader targets |
|---|---|---|---|---|
| Jetson | 6.5 / 14.4 / 26.5 ms | 0 | 40.5 ms | 10 |
| PC | 4.7 / 5.8 / 8.6 ms | 0 | 41.0 ms | 18 |

The Jetson loop keeps the 40 ms period with no measurable jitter. Its longest
cycles are waits for a late leader reply (timeout 20 ms). Late targets (sample
age plus margin over 50 ms) were isolated single cycles, and the lifecycle
stayed in TELEOP.

**Clock offset.** The Jetson's clock runs about 0.1–0.2 s ahead of the PC. That
made the 3D view mark every robot pose stale until views were stamped on arrival
at the PC.


### 2026-09-26 — live one-arm teleop built, not yet run on motors

The user chose advisory collision checks for single-arm tests first. The other
arm stays disabled, and a tighter bound for two-arm operation will be built
separately. What was added:

- **Receiver.** A new `teleop` service mode. It drives exactly one CAN bus (one
  arm) and requires a collision policy, a lease of at most 250 ms and a
  feedback timeout of at most 100 ms. Before opening its socket it passes the
  same watchdog and reference gates as the zero-torque trial. Its enable is
  single-use. It refuses to run without `--allow-powered-teleop
  --confirm-reference-pose`.
- **Live session start.** A stopped pre-scan requires every motor of the arm to
  be still and within 0.1 rad of its reference, then the arm's CAN link comes
  up and the service starts. The service dies with the loop's process;
  closing it disables the arm and takes the link down.
- **Grippers.** The receiver range is now the hand-swept physical travel: left
  −0.021 to 5.220 rad, right −0.002 to 5.229 rad. It was previously clipped at
  the CAD closed limit, 0.055 rad, below which a resting closed gripper would
  have faulted the receiver. CAD values are clamped wherever they meet the
  model.
- **Helm.** A receiver choice, Live · left arm or right arm, available only
  with the loop on the Jetson. Starting a live session asks for confirmation,
  and so does stopping while the arm is powered.

Release `20260926T224400Z-fd35f95` is deployed. Minerva Check passed every
Operator PC and Link and Jetson check, including the deployed receiver
validating both arms' teleop configurations and 125 of 125 fresh leader
samples. The stopped scan again found no motor answering, with both buses
error-passive (motor power off or foot switch pressed). No motor was enabled,
and no live session has been started.

### 2026-09-26 — first live start attempts fault; gripper limits and connection choice

With motor power on, Minerva Check was GO at 23:19 UTC. The operator then
started four live sessions on the right arm (23:21–23:23 UTC). Three passed
the stopped pre-scan and started the receiver, which faulted in about 0.1 s,
disarmed. The fourth failed the receiver's startup watchdog check. No motor was
enabled. The receiver logs show clean shutdowns: `disable_operation_succeeded`,
no disable transmit needed.

Cause of the faults: the receiver faults on any reading outside its joint
limits, even disarmed. Both grippers rest closed on their hand-swept minimum:
the right at −0.00165 rad against a −0.00176 limit, the left at −0.0202 against
−0.0206. A 16-bit feedback step is about 0.0004 rad, so one step past the end
is a limit violation. The new Minerva Check item shows it on the 23:19 scan:
without the fix it fails both grippers; with it, both arms pass.

The watchdog failure was not reproduced. The startup audit fails if a motor
answers late, leaves Reset, reports a fault, or moves faster than 0.2 rad/s. It
keeps auditing until the first enable, so the arm must hang still from Start
until Idle.

Changes:
- **Gripper limits.** The receiver's gripper limits extend 0.05 rad past each
  swept end (about 0.5 mm of jaw travel). Gripper targets stay within the
  swept travel.
- **Diagnostics.** The pre-scan names any joint at or next to a receiver limit.
  A receiver fault names the joints whose feedback explains it. The receiver's
  last status is saved as `live-<bus>-last.json` in the session's evidence
  folder.
- **Minerva Check.** A new item, "Live start", runs the live pre-scan's pose
  and limit checks on both arms.
- **Connection choice.** Helm's Connect reader went over Wi-Fi (the device
  profile's address) with no way to choose. Helm now has **Via Tether / Wi-Fi**,
  defaulting to the tether, for Connect, Minerva Check and teleop. Live teleop
  is refused on Wi-Fi.

### 2026-09-27 — live start: speed noise, not motion

The next two live starts on the right arm failed the receiver's startup check,
with nothing enabled. The receiver now names the motor, the check and the value
behind a failed read. The second attempt (01:50 UTC) said
`right_arm_j5: moving at +0.211 rad/s at the zero-gain feedback request (limit
0.2 rad/s)`.

J5 had not moved. Its position was −0.0669 rad, against −0.0667 rad three hours
earlier, and the operator confirmed it was still. The receiver's last status
showed every right-arm motor at rest reporting speeds of −0.06 to +0.01 rad/s
with constant positions. The drives' speed readings are noisy, and a single
reading over 0.2 rad/s failed the watchdog audit and the reference gate.

Both gates now count a motor as moving only after three readings in a row over
0.2 rad/s. A real motion persists, so it still fails within a few readings.
Positions, modes, fault bits, temperatures and reply timing are checked as
before.

With that fixed, the 01:56 UTC start reached Idle: the motors came on at zero
torque, with a 4 ms typical and 20 ms worst loop cycle. Pressing Enable was then
refused (`invalid_state`), and the session stopped with the arm disabled. It had
been hanging in Idle, so it did not drop. The controller refused Arm when any
joint's latest speed reading exceeded `settled_velocity` (0.15 rad/s): the same
noise. The lifecycle had the same weakness: it finished alignment and shutdown
only when every speed reading was under 0.05 rad/s, and the drives read up to
0.06 at rest. Now Arm and Reset are refused only for three fast samples in a
row, and the lifecycle settles on the change in measured position between
cycles. Release `20260927T020538Z-bb0a052` carries both.

### 2026-09-27 — first powered teleop session (right arm)

The session ran at 02:08 UTC on release `20260927T020538Z-bb0a052`. It went
through Start, Idle (motors on, zero torque) and Enable (hold), and the arm held
its pose with 0.000 rad tracking error on every joint. The loop cycle was 13 ms
typical and at most 34 ms, with no overruns. The advisory collision check read
clear. No receiver fault occurred while the arm was powered.

- **Engage** aligned for about a second, then fell back to hold ("leader input
  lost"): 8% of cycles had no fresh leader sample. The PC's leader sampler,
  nominally 100 Hz, woke on Windows' 15.6 ms timer tick and served samples up
  to about 30 ms old (p99 29 ms measured). Over the tether and with the loop's
  cycle margin, that often passed the 50 ms limit. The responsive processes now
  request 1 ms timer resolution; served age is p99 21 ms measured.
- **Shutdown**: J1 settled 0.016 rad and J4 0.007 rad from the reference, but
  J5 stayed 0.063 rad away. At 2.4 Nm/rad its hold torque there (0.15 Nm) did
  not overcome friction, so the shutdown never settled. Disable (H) ended the
  session with the arm hanging. J5 and J6 now hold at the collector's full
  gains, 8 and 3 Nm/rad.

### 2026-09-27 — first teleop following (right arm)

The session ran at 02:28 UTC on release `20260927T022032Z-e0d9571`, after Minerva
Check was GO (leader sample age p99 21 ms). It went through Start, Idle and
Enable, then Engage reached TELEOP in about a second. The arm followed the right
leader with 0.02-0.03 rad tracking errors and a 13 ms loop.

About 18 s later, on a quick leader motion of about 0.5 rad at J4, the receiver
faulted (`limit_violation`) and disabled the arm: J4 read +1.534 rad/s against
its 0.8 rad/s cap. The operator saw no fast motion. J4's position had moved
about 0.008 rad since the previous cycle, and its servo (hold Kp 15, Kd 4,
position-only commands) cannot drive it faster than about 0.3 rad/s. J4 was
also 0.088 rad behind its command, past its 0.08 rad tracking bound, because a
command could lead by the whole bound.

Release `20260927T024355Z-ee3cca2` changes three things:
- A joint faults on overspeed only after three samples in a row over its cap,
  or on one beyond three times it.
- A stop brakes from at most the cap, never from a spike.
- A command leads its measured position by at most 60% of the tracking bound.

### 2026-09-27 — teleop at the first-motion gains; full gains chosen

The session ran at 02:49 UTC on release `20260927T024355Z-ee3cca2`: Start,
Idle, Enable, Engage and TELEOP, with no receiver fault. Shutdown returned the
arm to its reference and disabled it by itself (the position-based settling
fix). Two holds came from leader-link stalls, each three requests in a row with
no reply within 20 ms, to be investigated.

At the collector's 0.3 first-motion kp scale the arm followed very slowly (J1
about 0.11 rad/s). J2 and J3 did not rise when the leader did, and J6 did not
move. Commands are position-only while collision envelopes are in force: the
receiver rejects any velocity or effort feed-forward. So a joint's drive is
hold Kp times its lead: 3.2 Nm at J2 and 1.9 Nm at J3 at the lead limit, below
the arm's gravity torque, and 0.14 Nm at J6, below its friction.

The operator chose the collector's full tuned gains: J1-J4 hold at 150, 220,
215 and 50 Nm/rad with their tuned damping, and J6 at 8 Nm/rad (release
`20260927T025732Z-a5cce96`, not yet run). The operator's direction: these
values should be calculated once from the arm itself, then checked by the live
joint sweep in Minerva Check, rather than tuned in teleop sessions.

### 2026-09-27 — servo sweep and Minerva Check's live joint sweep built, not yet run on motors

Following the operator's direction, the servo settings are now measured once on
the arm by a live sweep, reviewed and saved, then checked by a shorter live
sweep in Minerva Check. The operator chose a progressive range, and sweeping
stiffness, damping ratio and speeds; the first sweep may take as long as good
data needs.

- The receiver now returns its most recent ticks with each status: position,
  velocity, effort and command per joint, up to 256 rows. The sweep keeps every
  tick as evidence on the Jetson (`~/minerva/evidence/sweep-<time>`).
- Helm's Tune tab sweeps one arm, joint by joint, in four phases:
  1. slow ramps widening toward the usable range, for gravity and friction;
  2. steps and ramps over a stiffness ladder × damping ratios;
  3. speeds, then braking;
  4. a pass over the verified range at the chosen settings.

  Guards stop a task when:
  - a joint lags past 75% of its tracking bound;
  - it pushes over half its effort limit;
  - it passes 55 °C;
  - the collision check fails. Every planned path is also checked against the
    collision model beforehand.

  The `sweep` block of the teleop profile raises the wrist ceilings to
  16 Nm/rad (J5, J6) and the gripper's to 5 for the sweep only.
- Accepted joints are saved to `config/devices/minerva-production-01-servo.json`,
  bound to each motor's UID and firmware. The receiver takes those joints'
  gains, speed, braking and tracking bound from it.
- Minerva Check adds "Servo calibration" (a warning while joints are not swept,
  a failure if a motor changed). Its stage 4 is the live joint sweep per arm:
  the same engine in verify mode. A joint fails there on any guard event, on
  lag over half its bound, or on lag or friction well up on its saved sweep.
  GO now needs both arms' live sweeps; until then the verdict is INCOMPLETE,
  and reports saved before this load as OUTDATED.

Tested in simulation and on the virtual receiver through Helm, including a full
J6 sweep. Built in commits `32c51e9` and `55baca4` and deployed the same night.
It has not run on the motors. The first run is attended from the Tune tab (the
right arm suggested), with the foot switch in hand; then Minerva Check and its
live sweeps.

### 2026-09-27 — first live servo sweeps (right J6)

- **15:41 UTC, not started.** The tether was down (the PC's adapter showed
  "media disconnected"). The operator rebooted the Jetson and it came back.
- **15:47 UTC, stopped at the reference check** before enabling: the right
  gripper was 0.13 rad open (tolerance 0.1).
- **15:48 UTC, release `20260927T171842Z-64f5e91`, J6 only.** Clean: no guard
  events, no faults, about 5 min.
  - Measured: friction 0.10 Nm, gravity up to 0.16 Nm (worst at −0.46 rad),
    verified range +0.67/−0.63 rad.
  - Every stiffness from 1.9 to 16 Nm/rad stepped cleanly (under 4% overshoot
    at 5.6 and above). Holds at 8/11.2/16 sagged 0.016/0.013/0.012 rad.
  - Braking from 0.2 rad/s was clean up to 32 rad/s².
  - The recommender took 0.35 rad/s from the one direction that followed it;
    the other lagged 0.042 rad, over half the bound. Not saved. Fixed in
    `64f5e91`: a speed counts only when every direction follows it.
  - Position-only commands limit J6 to about 0.2 rad/s within half its bound:
    with a zero velocity target the drive's damping resists motion, so lag ≈
    (friction + gravity + kd·speed)/kp.
- **20:06 UTC, release `64f5e91`, J6 with the wider search** (stiffness to
  24, damping to 0.25×). The receiver faulted `limit_violation` during the hold
  tests: J6 read 0.97 rad/s against its 0.8 cap. The operator saw no buzz, and
  the samples show why:
  - Going out to the hold point at its profile gains (8/1), J6's command sat
    at the lead cap (0.6 × 0.08 = 0.048 rad) the whole way. That is 0.38 Nm of
    push, about what friction, gravity and damping take there, so J6 crawled
    about 0.13 rad behind its plan.
  - When the plan arrived, the hold test switched to 24 Nm/rad, and J6 lunged
    toward the hold point.
  - Fixed in `463573e`: the plan waits for a pinned joint; stiffness changes
    only once it is at rest, ramped over 0.3 s; speed tests stop at 80% of the
    cap; steps are at most 0.03 rad.

The operator judged the bounds too tight for performance. The sweep may now
raise a light distal joint's tracking bound (which sets its push through the
lead cap) and speed cap toward per-joint ceilings, while at least half its
swept travel stays usable (the stopping reserve grows with both):
- J5 and J6: 0.15 rad and 2.5 rad/s;
- J4: 0.12 rad and 1.5 rad/s;
- the gripper: bound 0.15 rad.

At the 50% floor, J6 (1.91 rad swept, 61% usable today) gets bound 0.15 at
0.8 rad/s; a faster cap would leave under half usable. J1–J3 keep their bounds
until gravity and velocity feed-forward.

Also found: teleop commands a joint at up to its receiver cap, which is also
its overspeed threshold, so a joint that briefly overshoots while following
a fast leader move at the cap can fault. Open: command at about 80% of the
cap.
