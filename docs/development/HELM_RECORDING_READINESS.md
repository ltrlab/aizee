# Helm recording readiness

Acceptance dated 2026-10-17 UTC (2026-10-16 local). Verified runtime source:
`4c5237074b506d6c1abc4a8157b129b14cae5971`, branch `codex/recording-readiness`.
This is the recording handoff before the requested pedal A/B/C feature. Pedal
behavior is upcoming and is not covered by this release's evidence.

## Deployed state

Jetson `current` is `20261017T014715Z-4c52370`; all 145 release files matched its
manifest. Disarmed release smoke passed. Activation preserved every prior release
and did not prune. The terminal installation is
`%LOCALAPPDATA%/Minerva/helm-recording-4c5237074b`; its prior installation and
launch settings were backed up. The final Helm task was Running, PID 6328, with
teleop/tuning stopped, the motor feed disconnected and `--no-live` enabled.

The stopped scan found all 14 motors in fault-free Reset with expected identity,
firmware and positions inside saved ranges. Both CAN links were down afterward.
Minerva Check remains **INCOMPLETE**: left wrist J6 was +0.24 rad from reference
(the live-start tolerance is 0.1 rad), and attended sweeps remain pending.
The user confirmed that the bucket opening is visible and the hanging pose is
clear, but a full sweep would hit the table. No sweep or new physical motion was
performed for this handoff; that table conflict must not be ignored.

## Implemented recording behavior

The production capture process selects a coherent three-camera set with one
paired native receiver sample. Recording uses left wrist, right wrist and head;
older two-camera episodes retain their original roster. Previews stay 640x480.
`minerva-task-capture.json` sets `recording_image_size` to `[320,240]`.

For exact 2x RGB downsampling, `Image.reduce(2)` records
`operation: integer_reduce`, `algorithm: Pillow.Image.reduce` and `factor: [2,2]`.
Other aspect-preserving downsample ratios retain BOX; absent/null size preserves
the source. Each frame records source/stored shapes, Pillow version, RGB order
and null stored intrinsics. Source arrays, original acquisition clocks and the
paired q/qcmd sample are preserved. Freshness is rechecked after transformation:
receiver age <=30 ms, camera age <=200 ms, camera skew <=20 ms. The capture target
is capped at 20 Hz; this is not a guarantee of 20 admitted frames/s. Queue and
control bounds were not enlarged to obtain acceptance.

Normal stop drains and finalizes the writer before publishing a final HDF5 path.
Abort retains a partial with its reason; process teardown waits boundedly for
the HDF5 owner to finish, and an unclean timeout fails. Recording failure uses
the existing guarded hold behavior; it does not redefine motor enable/disable
or return-to-reference controls. Follow [the task workflow](HELM_TASK_WORKFLOW.md).
Head depth is not recorded. Exposure time, camera extrinsics and the head mount
remain uncalibrated; no new camera intrinsics are invented by resizing.

## Accepted evidence

The clean frozen full offline gate passed **1049 Python / 132 Rust tests**, zero
skips/xfails, with the same source content hash before and after:
`078f363856fc9e0b917202339bbc57f8b5e672afa96660b6c863ef8ba8b0b38d`.
Gate report in the `helm-live-cameras` worktree:
`build/dev-check/20261017T014826Z-196f9736/report.json`.

The actual Jetson v10 proof exercised the production capture child and writer
with three real camera streams, the actual ARM64 native Loopback receiver and
fixed virtual positions for all 14 axes. The private network namespace contained
only loopback at entry and exit. It had no CAN, USB or physical motor access.
Byte-exact camera relay and bounded deferred pixel audit preserved source evidence.

| Result | Observed evidence |
|---|---|
| Completed recording | 144 frames, 10.097231 s source span, 14.162 admitted frames/s |
| Deliberate interruption | 6 readable partial frames, explicit interruption reason, no final publication |
| Image integrity | All 450 stored images equal independent 2x2 RGB sums with `(sum + 2) // 4` rounding |
| Receiver and actions | Raw native q/qcmd pairing, ordered 14 axes, timestamps and action-valid masks verified |
| Admission limits | Receiver maximum 23.632 ms; camera maximum 140.162 ms; skew maximum 19.941 ms |
| Writer completion | 144 accepted / 144 written / 0 pending; observed peak queue 2; matching stop acknowledgement before virtual disable |
| Cleanup | Both exact owned worker PIDs exited 0; namespace exited 0; relay clean; no surviving process-group members |

These files declare `qualification_mixed`, stationary native virtual state and
training ineligibility. Their camera images show the real scene, but their joint
state does not describe the physical robot. They are **not real robot task
demonstrations**, training data or a moving-arm timing qualification. The proof
includes bounded diagnostics and therefore does not claim uninstrumented
production timing. The ordinary direct-control GC guard and all freshness,
command, lease and storage bounds were retained.

## Evidence locations and identity

Artifacts below are retained in the original workspace under
`build/recording-readiness-20261016/`; they are not committed datasets.

- `jetson-proof-20261017T014911Z-ddc9ad61/`: proof, namespace/case reports,
  native/camera audits, completed and partial HDF5 files, validation and diagnostics.
- `v10-independent-acceptance.json`: independent read-only validation matched the
  remote result; original downloaded evidence was preserved.
- `proof-4c52370-arm64-v10.tar.gz`: immutable reviewed proof bundle, SHA-256
  `e9faea1f4bcfce594c46cd6009c0d7d07c911f349cd69bac2f950f11ab91d01e`.
  All 225 proof inputs and 145 original release files were independently checked.
- `release/20261017T014715Z-4c52370.tar.gz`: production archive, SHA-256
  `532d0a9d3a2c5b8f5b99ee6dac61208a27b2efb34525cfc94f53055219c721c0`.
- `activate-20261017T014715Z-4c52370-20261017T020022Z-48a3f2a7.json`:
  accepted gate/proof/source identity, activation, disarmed smoke and preserved releases.
- `activate-console-4c5237074b-result.json`, `stopped-check-final-result.json` and
  `final-console-state.json`: terminal activation, exact deployed manifest,
  stopped scan/start-pose warning and final stopped/disconnected state.
- `local-smoke-v10/`: 171 complete frames over 10.016 s plus 6 interrupted frames
  with synthetic local cameras. All 39 helper regressions passed before packaging.

## Earlier failures and remaining boundaries

Prior failures remain evidence, including the five obsolete recording-test
expectations in the [live-camera gate record](HELM_LIVE_CAMERAS.md), HDF5 abort
teardown, full-resolution writer queue overflow, and the v8/v9 timing failures.
The teardown fix retains bounded HDF5 ownership through worker exit. Measurement
led to the explicit 320x240 dataset profile. V9 completed its long recording but
failed the next first append at a renewed receiver age of 30.030 ms; it remains
failed. Measured exact integer reduction replaced BOX only for the 2x case;
the 30 ms threshold was unchanged. V10 passed both cases on the new frozen source.

The next requested work is pedal A/B/C control and its separate validation.
Before physical collection, resolve the start-pose warning and workspace/table
clearance using the existing attended operating process. Pending sweeps are not
authorization to execute them in an obstructed workspace. Moving-arm capture,
physical cube transfer, contact truth and task success remain unvalidated.
Training promotion, inference and policy takeover/correction require their own
later evidence. No new physical motion follows implicitly from this document.
