# Feature task: Helm Settings workspace and model telemetry

- Branch and worktree: `codex/helm-settings-telemetry`, `C:/Users/ltr/.codex/worktrees/dev-baseline/aizee`
- Base commit (full SHA): `30c4835c2a6bc46ad94a7fc5b23ffad528a2626c` (`codex/teleop-direct-mirror`, itself
  from integration `8b4f2c4a9543699fd86d2054dbf658be920b72bb`)
- Owner: Claude Code, finishing the staged work Codex began on 2026-10-10
- Problem: the operator asked, on 2026-10-10, for every setting and speed parameter in one intuitive Settings
  menu, for controls and telemetry that are consistent, and for indicators beside the 3D model showing each
  joint's direction, torque, temperature and position.
- Hypothesis: not a fix; a feature. Settings stay outside the control loop and may only tighten what the
  profile and calibration derive.

## What it does

- **Settings tab** (Ctrl+, opens it): search, categories, and every field with its current value, default,
  bounds, unit and when it applies.
  - Manual motor tuning (Kp, Kd, speed, travel; Hold → Apply → Engage) moved here from Teleop. Teleop keeps a
    shortcut to it.
  - Editable, for the next session only: per-joint following ceiling, alignment and return speeds; source
    freshness and settling; five receiver timing values. Each can only go down from its derived value (a
    settling time only up). A following ceiling's floor is the return speed that still reaches the reference
    within its 40 s budget. Alignment and return speeds above a lowered ceiling are refused by name.
  - Read-only, with the workflow that changes them: motor limits and braking guards, the complete base profile,
    hardware and calibration bindings, fixed runtime guards.
  - Saved per profile under `runs/<asset>-teleop/<profile>-settings.json`, bound to the profile, calibration,
    servo calibration, limits, directions and motor identities; a change to any of them refuses the file.
    Profiles without direct teleop show a read-only catalog.
- **Model telemetry**: `state.telemetry` carries one source: a running teleop session or servo sweep (the
  receiver's feedback, measured live or simulated virtual), else the stopped reader (positions only). Values
  keep their units, age and stream identity, are aged on the PC, and are never filled in. The 3D view shows a
  card per joint (position, drive torque, temperature, motion) and a direction arrow from measured velocity
  where the joint's mapping is verified; stale readings hide the arrows.
- **Session controls bar**: Hold / Return & disable / Disable now, on every tab while a session or sweep runs.

## Owned files

`helm/settings.py`, `helm/telemetry.py`, `helm/static/{settings.js,helm.js,helm.css,index.html}`,
`viewer/static/{joint-overlay.js,joint-telemetry.js,viewer.js,viewer.css}`, settings/telemetry wiring in
`helm/{server.py,session.py,teleop.py,tune.py}`, `scripts/dev.py` (gate list), and the tests below.

## Shared interfaces held fixed

- No Rust, receiver, guarded/direct lifecycle transition, rate limit, watchdog or collision change.
- `TeleopCore.build` consumes `operator_settings` when present: the receiver is derived from the resolved
  profile and the lifecycle takes the validated rows. Every value is within the derived one, so this only
  tightens. Without the key, behaviour is unchanged. The teleop owner should review this seam.
- `teleop.load_profile` was split into `load_profile` + `validate_profile` (same checks).
- Telemetry is added to the teleop/tune views, which the loop publishes at most every 100 ms within its
  measured view budget (`run_loop.publish_in_budget`).

## Acceptance

- `python scripts/dev.py check` (required gate, now including the tests below).
- `node --test tests/ui/test_viewer_telemetry.mjs` (the overlay's display contract; Node is not in the gate).
- Tests: `tests/contract/test_helm_settings.py` (catalog, persistence/binding, tightening only, consistent
  speeds, a real core built with settings follows within them and reports telemetry),
  `tests/unit/test_helm_settings_api.py` (HTTP, token, read-only catalog), `tests/unit/test_helm_telemetry.py`.

## Results

- Required gate passed on the uncommitted source (content SHA256 `e3d99bf0…`, unchanged before/after the run):
  **645 Python tests, zero failures/skips/xfails, and 122 Rust tests**. Evidence:
  `build/dev-check/20261011T024926Z-1f0bcd14/report.json`. Only this brief was added after the gate.
- `node --test tests/ui/test_viewer_telemetry.mjs`: passed.
- No-hardware preview (a link that refuses Connect, a session that refuses Start, a throwaway runs
  folder): the Settings tab loaded 9 categories and 525 fields, 53 of them editable, plus the 14 manual
  tuning rows. Search worked, and an edit through the page applied as revision 1 and was saved for the
  next Start. Labels, effect badges and the model label read correctly. No motors were contacted.

## Remaining uncertainty

- The 3D overlay was not seen rendering: the browser pane was hidden during this work. Its logic is covered by
  the Node test, and the page's DOM was checked in a no-hardware preview.
- Jetson loop timing with telemetry in the view is not measured. The view is admission-controlled, but the
  direct teleop qualification should be rerun before a live session with this branch.
- Torque and temperature on the real drives are shown as reported; their accuracy is not validated here.
- Settings apply from the next Start. They are not pushed into a running session.
