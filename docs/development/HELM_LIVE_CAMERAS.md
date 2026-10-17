# Helm live cameras

Validated 2026-10-16 (artifacts extend into 2026-10-17 UTC), source
`7fe997fb7615f1305c5967ebf487509ae9ecf82f`.

## Operator behavior and boundaries

Settings → Cameras and Teleop display left wrist, right wrist and head before a
motion session starts. Pause/resume and focus controls adjust the preview;
Camera/Model switches release the inactive renderer. Live frames preserve local
exposure/gain drafts. Settings reads and Apply remain explicit separate actions.
Simulation does not open physical camera previews.

The read-only preview uses one persistent pinned-SSH helper and a bounded latest
JPEG cache, capped at 10 frames/s per role. Each visible UI owns one WebSocket
subscription and at most one decode per role. Extra frames are dropped. Camera
source age plus echoed SSH and browser heartbeat transit bounds prevents delayed
queued frames from appearing fresh; stale pixels clear at one second. Frame
status does not refresh old pixels. EOF, heartbeat expiry, bounded sends and
shutdown cleanup stop unused helpers. Preview transport and decoding are separate
from motor control; this separation is not a hardware timing qualification.

## Validation and deployment

- **91 focused tests passed**, zero failures/skips:
  `build/live-camera-validation/integrated.xml` in the integration worktree.
- **10 offline browser checks passed**:
  `build/live-camera-ui/report.json`. These cover synthetic frames, delayed
  delivery, decode bounds, stale clearing, reconnect limits, visibility cleanup,
  draft retention and absence of mutation requests.
- **Actual terminal, 30 seconds:** left wrist 8.80 frames/s, right wrist 8.17,
  head 7.503; maximum conservative received-frame age below 0.559 s. All 61 HTTP state
  polls completed without error; their recorded zero latency reflects timer
  resolution and is not a precise latency measurement. No camera helper remained
  after closing the smoke-test subscription.
- **Actual browser:** all three feeds advanced in Settings and Teleop at 800×480
  and 1024×600. Pause/resume, Camera/Model ownership and local gain draft retention
  passed, with no POST/mutation requests or browser errors.

Original-workspace evidence is retained under `build/minerva-terminal-20261016/`:
`live-smoke-result.json`, `activate-live-console-result.json`,
`live-console-after-restart.json`, and
`live-helm-browser-2026-10-17T00-17-58-401Z.json` (plus screenshots).
The terminal Helm task was restarted and persisted against
`%LOCALAPPDATA%/Minerva/helm-live-7fe997fb76`; the old release was preserved.
Its launch includes `--no-live`. Teleop/tuning were stopped and the motor feed
was disconnected. No motor command was sent by these preview checks.

## Separate full offline gate

`scripts/dev.py check --offline` ran on the clean frozen source above. ARM64
release binaries built offline in this worktree; native checks passed **132 Rust
tests**. Python completed **987 passed, 5 failed, zero skipped/errors** in
539.46 seconds. The failures are the previously observed five parameter cases
(`callback`, `event`, `latency`, `flush`, `worker`) of
`test_recording_failure_latches_and_never_interrupts_disable_callbacks` in
`tests/unit/test_teleop_recording.py`. They were left unchanged. The required full
gate therefore remains failed; focused camera success does not override it.

Evidence: `build/dev-check/20261017T002135Z-0d0c1ca8/{report.json,output.log,tests.xml}`.
The gate exits before writing its automatic `source_after` on test failure.
A separate post-gate `source-after.json`, recorded before this documentation edit,
confirms clean commit identity and the unchanged source content SHA-256
`94e50186d808d508706f398619b8f54b421505f8916cd65d74a6d7ff535112e7`.
Toolchain and ARM64 build evidence are in `build/offline-gate-prerequisites/`.

This validates read-only camera viewing. Physical motion, robot control-loop
latency with preview/capture active, and task-data qualification remain outside
this evidence.
