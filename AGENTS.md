# Working on Aizee / Minerva

Read `docs/DEVELOPMENT_STATUS.md` and `docs/DEVELOPMENT.md` before changing code.
Both Codex and Claude Code use these instructions. Detailed history stays in the
device and architecture documents; current work and blockers go in the status file.

## Isolation and ownership

- Use one feature branch and worktree per task, based on the current integration
  commit. Do not edit another task's worktree or share editable Python installs,
  native build outputs, ports or recording directories.
- Before editing, inspect `git status` and the task's file ownership and acceptance
  criteria. Preserve existing user changes. Do not automatically merge or deploy.
- One owner changes the Python/Rust motion contract together: approach, collision
  envelope, guarded lifecycle and receiver checks. Coordinate a contract change
  before other branches depend on it. Performance work preserves these semantics.
- Keep changes bounded to one hypothesis. After two failed attempts, capture new
  evidence and revise the hypothesis before making more code changes.

## Verification

- `python scripts/dev.py setup` creates this checkout's `.venv-dev` from the
  hash-locked development requirements. `python scripts/dev.py check` builds and
  tests Rust, then runs required control contracts with zero skips/xfails.
- A small change that leaves the motion contract, Rust and the locks untouched may
  be deployed (with the deployment owner's approval) once `python scripts/dev.py
  quick` passes; it runs the gate's tests that touch what changed since the last
  passing check (name the tests when a widely imported module changed). The full
  `check` still passes before merging, and before any motion-contract, Rust or
  lock change is deployed (user decision, 2026-10-29).
- A passing offline gate is not hardware qualification. Follow deterministic
  tests, Linux vcan, Jetson timing, then supervised hardware validation in order.
  Do not start physical services, deploy, or command motors without authorization.
- Reproduce the failure before changing behavior. Preserve the failing input,
  expected outcome, source/model identity and seed. Do not relax timing, stopping,
  speed, freshness or collision assertions merely to make a test pass.
- Keep robot ownership, watchdogs, freshness and fault recovery explicit in the
  receiver. Physics libraries must run behind that boundary.
- Report what was tested, skipped or unverified, and link the generated report.
  Never replace a measured physical parameter with a plausible invented value.
- The repository is public. Run `ltr-code install --worktree` in each new worktree
  (`docs/DEVELOPMENT.md`, "Commit checks"). Never commit with `--no-verify` to get
  past a finding: remove the secret, or allow a real non-secret with a reason.

## Branch handoff

Include base commit, owned files, hypothesis, reproducer, acceptance command,
results, remaining uncertainty and integration dependencies. Use the task template
in `docs/development/TASK_TEMPLATE.md`. One integrator merges and reruns the gate
after each branch. Hardware deployment has one human-designated owner.
