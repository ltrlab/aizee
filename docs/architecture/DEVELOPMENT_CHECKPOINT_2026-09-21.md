# Development checkpoint — September 21, 2026

This checkpoint records the accumulated work since `5011480` on `overhaul`.
The shared Python package/session migration and initial cleanup were committed
earlier; this change adds model tools, CAD evidence, collision policies and
development-device validation to that foundation.

## Implemented

- A robot-independent mechanical profile and browser viewer: catalog or external
  URDFs, joint jogging, coupled sliders, pose import/export, geometry overlays,
  flat CAD shading and profile-aligned Front/Top views. WebXR preview mounts the
  same viewer; its old separate preview page is removed. Three.js is vendored
  with version/source/license records for offline use.
- Minerva CAD source history and audits: the native 1.0 export, intended
  `onshape-to-robot` 1.1 export with repaired shoulder, and simplified **URDF 1.2
  No Holes** collision export. The active composed URDF retains 1.1 visuals and
  uses 1.2 collision shapes. Raw models, assets, source hashes and composition
  validation are retained. No further Onshape retrieval is needed for this work.
- Shared CAD collision/path checks, per-arm limit sweeps, provisional interface
  reports and viewer candidate ranges. Torque/stall probe classification and
  the known-load calibration plan are offline foundations; no powered limit
  discovery was performed.
- Fourth root-workspace crate `robot_collision`: persistent Parry f64 shapes,
  branch/link/part filtering and C ABI 3. Python supports CAD-confirmed queries
  and a separate conservative keep-out policy. Robot branches and omissions are
  profile data, not hard-coded robot names.
- Minerva's four sliding finger links are explicitly omitted from every coarse
  pair, including finger-to-arm pairs. Wrist/gripper bodies remain checked;
  CAD diagnostic modes still include fingers. Policy identity, scope and
  exclusions accompany verdicts and appear in the viewer.
- Opt-in session collision observation/simulation enforcement: fresh complete
  state, proposed and stopping paths, conservative displacement bounds,
  query/time budgets and expiring command-bound verdicts. Fixed radii/ancestry
  are cached; dynamic extension remains per-query. Hardware enforcement is
  unavailable.
- Native and ARM64 collision build support, seeded CAD/keep-out benchmarks,
  retained timing/audit summaries and an identified Orin Nano development host.
  Its device log records serial, permanent MAC addresses, pinned SSH identity,
  source overlays, tests, temperatures and configuration changes.

## Validation and measured scope

Desktop checks for this checkpoint:

| Check | Result |
|---|---|
| Default offline Python suite | 413 passed |
| Rust collision tests and release build | 6 passed; build succeeded |
| Viewer JavaScript scheduling tests | 7 passed; syntax check passed |

The previous development-Jetson run passed 98 selected Python and six Rust
tests. Its **1,000-cycle synthetic inter-arm replay paced at 25 Hz** had zero
scheduled deadline misses: median **26.15 ms**, p99 **34.75 ms**, maximum
**36.67 ms**; 851 cycles accepted and 149 rejected. A 100-pose CAD audit found
zero missed nonexcluded proximities, 39 additional conservative restrictions
and 34 intentionally omitted finger-pair occurrences. These are sampled
results, not exhaustive geometric or real-time guarantees.

Whole-robot replay rejects all sampled commands because unapproved same-arm/body
interfaces remain. The short inter-arm run omits full telemetry/transport,
camera/controller load and calibrated braking; it does not qualify hardware
enforcement or the longer 25 Hz acceptance test.

## Repository reproducibility

CAD meshes use Git LFS, including the simplified 1.2 export. Hashed URDF and
part metadata bytes are preserved across Windows/Linux checkout conversions.
The generated repository inventory includes the collision crate and current
models, tools, tests and documentation.

Credentials (`.env`, private keys), local environments, build outputs and raw
`simulation_runs/` evidence remain ignored. Versioned benchmark summaries retain
source/report hashes; raw reports remain on the development host and local disk.
This checkpoint is a local source commit and does not publish or deploy code.

## Continue from here

1. Review same-arm/body contact policy and retain useful accepted motions.
2. Integrate full-session replay, receiver-side command leases and fault tests;
   run the longer loaded 25 Hz qualification on the intended deployment host.
3. Verify motor-to-CAD coordinates, units, feedback timing and stopping response
   before physical enforcement. Use calibration masses for paired static trials.
4. Establish credible mass/inertia and measured actuator dynamics before
   claiming a physically validated simulator; then work toward 50/100 Hz.

## Detailed records

- [Repository map](REPOSITORY_MAP.md) and [inventory](REPOSITORY_INVENTORY.md)
- [Viewer guide](../MODEL_VIEWER.md) and [Minerva CAD history](../MINERVA_CAD.md)
- [Collision implementation](COLLISION_RUNTIME_IMPLEMENTATION.md)
- [Keep-out evidence](COLLISION_KEEPOUT_RESULTS.json) and [25 Hz plan](COLLISION_25HZ_HARDWARE_PLAN.md)
- [Known-load calibration](KNOWN_LOAD_CALIBRATION.md)
- [Jetson identity and maintenance log](../devices/JETSON_DEV_01.md)
