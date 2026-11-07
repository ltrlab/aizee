# Live task capture: a failed capture reads as unavailable at once
- **Base commit:** `1580f1e81f8cfe2bf0dfed4c1269ee1e1de74ad2` (`codex/integration-20261016`).
- **Owned files:**
  - `src/robot_platform/helm/task_capture.py` (`LiveTaskCapture.health`)
  - `tests/unit/test_helm_task_capture.py`

## Problem

`tests/unit/test_helm_task_capture.py::test_socket_worker_aborts_stalled_receiver_without_completed_episode` failed intermittently: `assert not capture.snapshot()['available']` right after the snapshot reported `failed`. On the base commit it failed **14 of 60** separate runs, measured while the laptop also ran a training job and another worktree's gate.

**Cause.** When the capture worker fails, it publishes `failed=True` under its lock first. It clears its own `available` and `capture_ready` flags only at the end of its teardown, after aborting the writer, closing its camera sockets and terminating its ZMQ context. `LiveTaskCapture.health()`, and so `snapshot()`, passed the raw `available` flag through, cleared only for a stall. A read inside that window saw a capture that had failed and was still "available".

After `abort()` the same window opens, because `abort()` sets `failed` but leaves `available` set. A loop iteration already under way can also set `capture_ready` again before the worker sees its stop.

The process wrapper already reports the opposite. `ProcessCapture.health()` in `data/capture_process.py` gives `available` and `capture_ready` only while not failed.

## Fix

`LiveTaskCapture.health()` computes `failed` once (`failed` or stalled) and derives `available` and `capture_ready` from it, as `ProcessCapture.health()` does. `snapshot()` takes those fields from `health()`, so every reader sees a failure and its unavailability together. The worker's own flags and teardown order are unchanged. No assertion was relaxed.

## Reproducer and evidence

- **Deterministic reproducer.** `test_a_failed_capture_never_reads_as_available_or_ready` in `tests/unit/test_helm_task_capture.py` gives the worker a ZMQ context whose `term()` waits for release. That holds the worker inside its teardown after it has published the failure. It checks `snapshot()` and `health()` there, for both a stalled receiver and an `abort()`.
  - Base commit: both cases fail on every run, with `failed` true and `available` true.
  - With the fix: both pass.
- **Original test:** 60 separate runs with the fix, under the same load: **60 of 60 passed**.
- **Gate:** `python scripts/dev.py check --offline`, passed on `4cfa588`: 1120 Python tests, no skips, and 132 Rust tests, with the source unchanged. Report: `build/dev-check/20261018T045935Z-d5ba6971/report.json` in this worktree.

## Remaining uncertainty

None known for this window. The worker still clears its raw flags late. Code that reads `_state` directly, rather than `health()` or `snapshot()`, would still see the old order. Nothing in the repository does.
