# Jetson development device 01

Asset ID: **jetson-dev-01**. Role: development, native builds and simulation.
This is the newly provisioned Jetson, distinct from Minerva's existing robot
computer. Current hostname is **minerva**; the asset ID is an inventory label,
not a hostname change. Never select a deployment target by hostname alone.

## Identity observed 2026-09-20 EDT

| Identifier | Value |
|---|---|
| Board | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| Module/device-tree serial | `1421225029686` |
| Wi-Fi permanent MAC | `a8:e2:91:d9:9b:39` (`wlP1p1s0`) |
| Ethernet permanent MAC | `3c:6d:66:1e:be:d3` (`enP8p1s0`) |
| Observed SSH address | `ltr@192.168.8.118` (Wi-Fi, DHCP) |
| SSH ED25519 host fingerprint | `SHA256:TKFnMn4zf9eKCL7iiOF8QVjCebjoAssMmU6uzCTOpVE` |
| OS machine ID | `5dbfb12414a3456d9014d88183e338b1` |
| Root filesystem UUID | `752d27c0-5370-43a2-9666-6f6c7e937dce` |
| SD-card CID | `0353445344313238852a34d503014b00` |
| Ubuntu | 22.04.5 LTS, ARM64 |
| L4T / kernel | R36 revision 4.4 / `5.15.148-tegra` |
| Observed power mode | 25 W (`nvpmodel` mode 1), not changed during setup |

Raw inventory: [jetson-dev-01.json](jetson-dev-01.json). Match the serial and
pinned SSH host key when reconnecting; DHCP may change the IP. Permanent MACs
were read with `ethtool -P` and match the current addresses. USB gadget/bridge
MACs are not primary device identifiers. Machine ID, filesystem UUID and SSH
keys may change after reimaging; SD CID follows the removable card. Record such
changes with the physical module serial rather than silently replacing identity.

## Development layout

- Root: `/home/ltr/robot-platform-dev`
- Environment: `/home/ltr/robot-platform-dev/.venv` (Python 3.10.12)
- Rust: user-local `/home/ltr/.cargo` and `/home/ltr/.rustup`, version 1.98.1
- Current source: `current` symlink to `checkouts/checkout-20260921T003916Z`
- Source provenance: `SOURCE_SNAPSHOT.json` inside that directory
- Subsequent source overlays: `updates/*/installed-manifest.json` and corresponding
  prior-file backups; apply these after the baseline snapshot when identifying source
- Activate: `source ~/robot-platform-dev/activate.sh`
- Mirrored device records: `~/robot-platform-dev/device-inventory.json` and
  `~/robot-platform-dev/device-log.md`
- Installation logs: `python-install.log`, `rust-build.log` in the root above
- Validation output: `current/simulation_runs/jetson-bringup-20260920/`

The source directory is a verified working-tree snapshot, including uncommitted
work from branch `overhaul`. It has no Git database. Do not run `git pull` there;
transfer a newly identified snapshot or intentionally establish a Git checkout.

## Change log

Keep this log when adding software, changing networking/power modes, replacing
source snapshots, deploying services or running hardware experiments. Add a
dated entry describing the action, result and evidence. Do not store credentials.
Mirror updates to the device. Failed or superseded trials remain in the record.

### 2026-09-20 EDT — initial access and development setup

1. User flashed the SD card and completed enough setup for SSH at
   `192.168.8.118`. Discovered hostname `minerva`, user `ltr`, Ubuntu 22.04.5,
   L4T R36.4.4, about 7.4 GiB RAM and a 116 GiB root filesystem with 92 GiB free.
2. First SSH connection pinned the new ED25519 host key. Initial project-key
   authentication failed. User installed the existing project's public key.
3. On the Windows workstation, OpenSSH rejected the original workspace private
   key ACL. Created an owner-only copy at
   `C:\Users\ltr\.ssh\aizee_jetson_dev`; authentication then succeeded.
   Private key contents were not included in the repository snapshot or logs.
4. Read-only preflight found Git, rsync, C/C++ build tools, pkg-config,
   python3-dev and python3-venv already installed. Unattended sudo is unavailable.
   No `aizee*` or `minerva*` systemd unit files were present.
5. Created the isolated development root and Python virtual environment.
   Upgraded pip/setuptools/wheel inside that environment; system Python was
   not changed. Installed official Rust 1.98.1 with the minimal profile and
   `--no-modify-path`; activation explicitly sources its environment.
6. Transferred 1,385 non-ignored source/CAD files (509,949,872 bytes before
   compression), excluding Git internals, credentials and ignored caches.
   Archive: `checkout-20260921T003916Z.tar.gz`, 185,275,275 bytes.
   SHA-256: `8a3538b6579006877115cd5afb8e68afeef96899ceb5a9c9b3550617309f9f0b`.
   Verified the archive and every extracted source-file hash on the Jetson.
7. Installed the editable shared Python package with
   `[simulation,dev,viewer,collision]`. ARM64 wheels were available, including
   `python-fcl==0.7.0.11`; no system FCL installation or sudo was required.
8. Started `cargo build --release --locked --workspace -j 2`. Build completion
   and validation results will be appended below.
9. Collected the identity inventory at `2026-09-21T00:46:23Z` (20:46 EDT on
   September 20). Ethernet is disconnected; Wi-Fi is the active connection.
   Read the existing 25 W mode without changing it. No motor services were
   installed, enabled or run, and no physical actuator commands were sent.

### 2026-09-20 EDT — native build and validation completed

10. Native release build of all four workspace crates succeeded in 7m 45s.
    Existing unused/dead-code warnings remain in the motor controller. Built
    `motor_control`, `lidar_control` and `librobot_collision.so`; binary sizes
    and SHA-256 values are in the [validation report](jetson-dev-01-validation.json).
11. `cargo test --release --locked -p comms -p robot_collision -j 2` passed:
    6 communications tests plus 3 collision tests. The existing opt-in
    `test_pub_sub` test remained ignored (1 ignored); no claim that it ran.
12. Completed a 10-second synthetic Minerva simulation, exit code 0. Its log
    includes a missing-Git-metadata diagnostic because this source is a snapshot
    without `.git`; source identity is independently preserved by
    `SOURCE_SNAPSHOT.json` and the validation report's archive checksum.
13. All 53 selected Python tests passed: model composition, viewer, runtime
    collision guard, in-process session and transport contracts. Native collision
    tests used the library built on this host. No physical CAN tests were run.
14. Completed 1,000-pose Minerva 1.2 collision benchmark with 100 independent
    reference comparisons and 100 source-contact audits. Zero reference
    disagreements, missed source contacts or extra source contacts in those
    samples. Full-result timings: median **54.07 ms**, p95 **63.23 ms**,
    p99 **72.62 ms**, maximum **127.91 ms**. All 1,000 pose checks exceeded
    40 ms. Compact results: median 53.77 ms / p99 72.48 ms.
15. Validation completed `2026-09-21T00:56:46Z` (20:56 EDT September 20).
    Power mode remained 25 W; the sampled CPU thermal zone was 43.94 C at
    validation start and 43.81 C at completion. No fixed-clock setting was
    applied. Benchmarking followed the build/tests and is not qualification
    under simultaneous camera and controller load.
16. Preserved raw logs, Python package versions, simulator traces and benchmark
    poses on the device and under local ignored
    `simulation_runs/jetson-bootstrap-20260920/`. The compact validation report
    and identity inventory are in `docs/devices/`; this log is mirrored onto
    the device as `device-log.md`.

### 2026-09-20 EDT — collision hierarchy update and benchmark

17. Reconnected with the pinned SSH key and verified module serial
    `1421225029686` before updating the development checkout. Applied a 14-file
    source overlay for generic branch grouping, hierarchical native filtering,
    CAD transform reuse, viewer scopes, tests and benchmarks. Overlay SHA-256:
    `1d6ed756a94aa27b708461748cb25adfefaf21620086eff7afa75761f171f8d2`.
    Every installed file was hash-verified. Old files and their hashes are in
    `~/robot-platform-dev/updates/collision-hierarchy-20260921/`; the original
    snapshot manifest describes the baseline, and `installed-manifest.json`
    describes this overlay. No CAD assets or credentials were transferred.
18. Built and tested `robot_collision` natively with the existing Rust toolchain,
    release mode, locked/offline dependencies and two jobs. Five Rust tests
    passed. Library ABI is now 2; `librobot_collision.so` SHA-256 is
    `9efc22fcb963561f358451d71b0b0ee64599eb92010f84941479872fbf3691b6`.
    All 92 selected Python runtime, viewer, collision and session tests passed.
19. Ran the same seeded 1,000 Minerva poses as the baseline. Whole-robot contact
    median/p99/max: **48.74 / 67.18 / 76.72 ms**, all 1,000 over 40 ms.
    Inter-arm contact median/p99/max: **1.96 / 6.66 / 70.24 ms**, one over 40 ms.
    Each run had 100 independent reference comparisons plus 100 original CAD
    contact audits, with no sampled discrepancies. Inter-arm-only results are
    explicitly partial coverage, never accepted as whole-robot clearance.
20. Ran 100 inter-arm queries with a 5 mm clearance threshold. Median/p99/max:
    **1.99 / 41.76 / 85.25 ms**, two over 40 ms; all 100 matched the independent
    FCL reference. The tail was in CAD distance queries. Positive-clearance
    checks remain a distinct optimization requirement for motion supervision.
21. Validation ran `2026-09-21T01:28:19Z` through `01:31:53Z` (21:28–21:31 EDT
    September 20). Power mode remained 25 W, sampled CPU temperature
    42.72 → 43.44 C. No clock, OS package, networking or motor-service changes.
    No physical actuator commands were sent. This was an offline benchmark,
    without simultaneous camera/controller load.
22. Raw reports/logs are in
    `current/simulation_runs/collision-hierarchy-20260921/`, copied locally under
    `simulation_runs/collision-hierarchy/`. The permanent
    [hierarchy report](../architecture/COLLISION_HIERARCHY_RESULTS.json) records
    report hashes, model/grouping identity, stage costs and deadline misses.
    Updated this device log, viewer guide, implementation status and 25 Hz plan;
    mirrored those documents and the summary report onto the device.

### 2026-09-20 EDT — conservative keep-out and motion replay

23. Verified module serial `1421225029686` and the pinned SSH identity before
    each source update. Applied the 13-file `updates/keepout-20260921` overlay,
    SHA-256 `58a2ca8f0338105dc99a8c3ab0f032df31d7e2149fc736949b85e34a07c8d80a`.
    It adds explicit coarse boundary decisions, profile-driven finger omissions,
    viewer envelopes, policy-bound motion verdicts and an offline replay script.
    The four Minerva finger links are omitted from every coarse pair, including
    finger-to-arm. CAD diagnostics still include them. Wrist/gripper bodies and
    same-arm/body interfaces were not exempted.
24. Built native ABI 3 with locked/offline Rust dependencies and two jobs.
    `librobot_collision.so` SHA-256:
    `0c62bd934c5db72373b78c0553f4a43e6efa58f457ef4bf4173ff1cefebf4657`.
    Six Rust and initially 97 selected Python tests passed. No packages, OS
    settings, credentials, networking, clock settings or motor services changed.
25. First inter-arm keep-out replay at 25 Hz missed 18 scheduled deadlines out
    of 1,000 (17 computations exceeded 40 ms). The median/p99/max computation
    was 37.73 / 40.45 / 43.94 ms. Pose checks were already fast; repeated fixed
    geometry calculations in motion bounds were removed next. These original
    results are preserved, not overwritten.
26. Applied a second two-file overlay `updates/keepout-cache-20260921`, SHA-256
    `5112b8305fe7c0b10eb38b9ce7f4a81e555c94185f483f743234305b6b8c5011`.
    Cached fixed shape radii and joint ancestry; dynamic prismatic extension and
    motion remain per-query. Added a regression for changing reach. All **98
    selected Python tests and six Rust tests passed**. Both overlays preserve
    old files, old hashes, installed hashes and the update archive. The baseline
    snapshot plus these and the hierarchy overlay identify the deployed source.
27. Replayed the same 1,000 inter-arm poses at 5 mm clearance and 25 Hz. Pose
    median/p99/max: **1.91 / 2.34 / 3.91 ms**. Three-short-path motion cycle:
    **26.15 / 34.75 / 36.67 ms**, **zero scheduled deadline misses**; 851 accepted
    and 149 rejected cycles, up to nine queries each. One hundred independent
    CAD audits found zero missed nonexcluded proximity pairs, 39 additional
    conservative pair restrictions and 34 deliberately omitted finger pairs.
    Counts are pair occurrences across poses, not unique link pairs.
28. Whole-robot keep-out pose median/p99/max: **2.69 / 3.22 / 4.11 ms**.
    Whole-robot replay rejected all 1,000 cycles because of unapproved assembly
    interfaces. Its short rejection latency is not a successful usable motion
    supervisor. No physical actuator commands were sent; these synthetic short
    paths are not calibrated braking trajectories or camera/controller load.
29. Initial validation ran `2026-09-21T02:07:14Z`–`02:10:57Z`; cached validation
    ran `02:12:13Z`–`02:15:40Z` (22:12–22:15 EDT September 20). Power mode remained
    25 W; sampled CPU temperature was 42.59 → 43.69 C for the final run.
    Reports/logs are in `current/simulation_runs/keepout-20260921/` and
    `current/simulation_runs/keepout-cache-20260921/`, copied locally beneath
    `simulation_runs/keepout/`. The generic development `validate_jetson.py`
    runner was replaced with the keep-out runner; prior reports remain intact.
    The final runner is also retained as `validate_keepout_cache.py`.
30. Updated and mirrored this log, the viewer guide, implementation status,
    25 Hz plan and [permanent keep-out evidence](../architecture/COLLISION_KEEPOUT_RESULTS.json).
    The desktop viewer correctly shows red arm boundaries in a crossing pose,
    clear inter-arm reference pose and four finger exclusions. Seven viewer
    JavaScript tests passed on the desktop. No additional Onshape calls occurred.

**Readiness:** native builds and scoped conservative keep-out checks work. A
short inter-arm replay completed 1,000 cycles at 25 Hz without deadline misses.
Whole-robot accepted motion, full telemetry/lease integration, longer load
qualification and physical stopping validation remain outstanding; follow the
[25 Hz plan](../architecture/COLLISION_25HZ_HARDWARE_PLAN.md).
