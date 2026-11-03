# Jetson CPU split: the arms' loop on its own cores

- **Branch and worktree:** `claude/jetson-core-split` in `C:\Users\ltr\Workspace\aizee-helm-pedals`. It is stacked on `claude/helm-pedals-null-task` ([Helm terminal fixes](HELM_TERMINAL_FIXES.md)), because the terminal's package needs both.
- **Base commit:** `1580f1e81f8cfe2bf0dfed4c1269ee1e1de74ad2` (`codex/integration-20261016`, the source of the terminal's and the Jetson's current releases), via `2726a66`.
- **Owner:** Claude, 2026-10-17, at the user's request:
  - "run teleop without the camera services, check how it's effected and if it is the cameras, come up with a solution to optimally run them on the jetson";
  - "update the terminal with the proper package, implement the core split".

## Problem and evidence

The 14:17Z live session on 2026-10-17 faulted on an expired command. A command expires about 35 ms after its leader sample, and the sample is already about 12-13 ms old when the loop accepts it. That leaves about 20 ms, so a loop stall of 10 ms or more expires the command.

The Jetson has six Cortex-A78AE cores and no CPU isolation. The three camera services use about one core between them:
- left wrist about 44% of a core;
- right wrist about 37%;
- head about 21%.

The wrist nodes decode and re-encode JPEG. They ran on every core, beside the teleop loop's process (about 23% of a core) and its receiver.

Three holds on 2026-10-17, both arms Enabled, never Engaged, on the receiver configuration the laptop used on 2026-10-15. Evidence is on the Jetson in `~/minerva/evidence/<session>`:

| Condition | Session | Hold | Loop misses | Cycles over 10 ms | Cycle p50 / p99 / max | Smallest expiry margin | Receiver misses / max lateness | Fault |
|---|---|---|---|---|---|---|---|---|
| Cameras sharing all cores | `live-20261017T211457Z` | 77 s | 60 | 24 | 5.6 / 9.2 / 14.0 ms | 2 ms | 18 / 6.5 ms | no |
| Cameras paused (SIGSTOP) | `live-20261017T213238Z` | 28 s | 1 | 1 | 4.8 / 5.0 / 15.2 ms | 8 ms | 0 / 0.4 ms | yes, one 15 ms whole-process stall |
| Cameras on CPUs 0-3; loop and receiver on 4-5 | `live-20261017T213826Z` | 104 s | 0 | 0 | 5.1 / 6.5 / 8.1 ms | 7 ms | 7 / 3.5 ms | no |

The partition in the third row was applied by hand, with `sched_setaffinity` as user `ltr`, and removed afterwards. This branch makes it permanent.

## Change

**`control/scheduling.py`**
- `/etc/minerva/cpu-partition.json` holds `{"control": [4, 5], "background": [0, 1, 2, 3]}`.
- On Linux, where that file exists, `prefer_responsive` moves every thread of a process onto the control CPUs.
- `prefer_background` moves every thread onto the background CPUs.
- Without the file, as on PCs and CI, nothing moves. Control CPUs only help where the cameras keep off them.
- A malformed file is an installation error and raises.

**`helm/edge.py`**
- The robot-side loop applies `prefer_responsive` first, before its session starts any thread.
- Its threads and its child processes inherit the control CPUs. The children are the receiver (`guarded_controller`) or the control harness.
- The session view reports the hints as `scheduling.loop` (`["control_cpus"]`).

**`helm/task_capture_process.py`**
- Live task capture's worker is spawned by the loop. It moves itself onto the background CPUs before it decodes or writes anything.

**`ops/cpu_partition.py`** (new)
- The one-time installer for the cameras' half, run with sudo:
  - gives the three camera services (`aizee-scene-cam`, `minerva-left-wrist`, `minerva-right-wrist`) a `CPUAffinity=0 1 2 3` drop-in (`/etc/systemd/system/<unit>.d/50-minerva-cpu-partition.conf`);
  - restarts the running ones and checks every camera thread now runs on 0-3;
  - only then writes the partition file, so the loop never keeps to 4-5 while the cameras can still use them.
- `remove` deletes the file first, then the drop-ins.
- Both refuse while a Helm session runs on the robot, because a restart interrupts the camera's stream.
- `install` refuses if CPUs 4-5 are offline (power mode).
- `status` needs no sudo.

**Tests**
- `tests/unit/test_cpu_partition.py` and `tests/unit/test_cpu_partition_install.py`, both added to the gate's required list.
- The existing scheduling tests in `tests/unit/test_helm_teleop.py` now pass whether or not a host has a partition.

**Held fixed**
- The motion contract: receiver, lifecycle, collision and expiry.
- The receiver binary.
- The session protocol: `serve` only gains an optional keyword.
- Nice levels and real-time priorities are unchanged.

## Install on the robot (once, by the user)

On the Minerva terminal, run the desktop shortcut **Minerva - Split Jetson CPUs** and enter the Jetson's sudo password. Or, in a shell on the Jetson:

```bash
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.cpu_partition install
```

Stop any Helm session first; the cameras restart.
- **Check without sudo:** `cd ~/minerva/current/src && python3 -m robot_platform.ops.cpu_partition status`
- **Undo:** run the same command with `remove` in place of `install`.

## Acceptance

- **Gate:** `python scripts/dev.py check --offline`, zero skips or xfails (see Results and deployment).
- **On the robot, after install:**
  - `status` shows each camera's threads on `[0, 1, 2, 3]` and the partition in place.
  - A Start, Idle, Enable hold (no Engage) reports `scheduling.loop == ["control_cpus"]`, with every thread of the loop's process and of the receiver on `[4, 5]`.
  - Loop timing matches the partitioned hold above: no missed cycles and the slowest under 10 ms.

## Results and deployment

- **Gate:** passed on `ce086c5` with 1138 Python tests, no skips, and all 132 Rust tests; the source was identical before and after the run. Report: `build/dev-check/20261017T224217Z-b009724d/report.json` in this worktree.
  - The run included the gate's short MuJoCo-rendering tests, on the GPU, while a training run held it.
- **Deployed 2026-10-18 at about 00:05Z,** with the user's approval:
  - The Jetson's `current` is `20261017T224256Z-ce086c5`. It was staged, verified and smoke-tested, then switched with both CAN links down and no receiver running. The previous release, `20261017T151236Z-1580f1e`, is kept.
  - The Minerva terminal runs `helm-recording-ce086c58a5`, built from the same commit. It replaces the hot patches and the restored mesh described in [HELM_TERMINAL_FIXES.md](HELM_TERMINAL_FIXES.md). The previous folder and launchers are kept in `helm-recording-activation-20261018T000513Z`.
  - The packaging and activation commands are in this worktree's `build/terminal-deploy-20261017/`. They are Codex's process, with this worktree as the source.
- **Installed 2026-10-18 at about 03:19Z** by the user, with `sudo python3 -B -m robot_platform.ops.cpu_partition install`, run from the laptop through the terminal.
  - `status`, run independently: the partition file is in place, and each camera unit has its drop-in. systemd reports `CPUAffinity=0-3`.
  - Every camera thread is allowed on CPUs 0-3 only: 25 head threads and 9 per wrist. No unit has restarted since.
- **Acceptance hold:** `live-20261018T032718Z`, with the user's approval: Start, Idle, Enable, 146 s in Hold, Shutdown, Stop; no Engage, no fault.
  - During the session, all 6 threads of the loop process and all 3 of its `guarded_controller` were on CPUs 4-5. The session view reported `scheduling.loop == ["control_cpus"]`.
  - Loop timing in Hold, with the earlier holds for comparison:

| Hold | Duration | Missed cycles | Cycles over 10 ms | Cycle p50 / p99 / max | Smallest expiry margin | Receiver misses / max lateness |
|---|---|---|---|---|---|---|
| Cameras sharing all cores (`live-20261017T211457Z`) | 77 s | 60 | 24 | 5.6 / 9.2 / 14.0 ms | 2 ms | 18 / 6.5 ms |
| Split by hand (`live-20261017T213826Z`) | 104 s | 0 | 0 | 5.1 / 6.5 / 8.1 ms | 7 ms | 7 / 3.5 ms |
| **Split installed** (`live-20261018T032718Z`) | 146 s | 1 | 0 | 5.7 / 7.2 / 9.9 ms | 5 ms | 7 / 3.5 ms |

  - **Leader samples:** p99 age 16.2 ms, against 13.8 ms in the hand-split hold. The leaders are read on the terminal, so the Jetson's CPU split doesn't reach them.
  - **Helm's own counters:** they include the session start and the Idle and Enable transitions, so they reached a 13.1 ms maximum with 15 overruns, all before Hold.

## Remaining uncertainty

- **The 15 ms stall:** it happened with the cameras paused, so camera load doesn't explain it. 104 s partitioned had none, but bounding its rate needs longer holds.
- **The receiver's own deadlines:** partitioned, it still missed 7 (3.5 ms late at most), against 18 (6.5 ms) shared and none with the cameras paused. It now shares two cores with the loop's Python process. The next step would be real-time priority for the receiver (SCHED_FIFO), which needs a root-set rtprio limit. It was not tried.
- **Live task capture** runs on CPUs 0-3 with this change, but it has not been run that way on the robot.
- **Not done:**
  - the wrist nodes' `passthrough_mjpg` option, which would cut their decode and re-encode load;
  - confining `aizee-heartbeat` (about 6% of a core, left free as in the measured hold).

## Work started over SSH, and deep idle (2026-11-01)

- **SSH sessions take every CPU.** sshd and every session it starts are allowed CPUs 0-5. That includes Helm's
  robot paths, which go over its loopback SSH, and the GELLO bridge's relay, its key's forced command. A head map
  capture launched that way ran on CPUs 4-5 about a fifth of its time. In `live-20261101T020612Z` the loop's slow
  cycles rose fourfold once it started, keeping time with its 15 fps frames, until an arm command expired.
- **Pinning the ssh service would not help.** A login moves its session into
  `user.slice/user-1000.slice/session-N.scope`, whose cpuset differs from `system.slice/ssh.service`'s, and this
  kernel (5.15) resets a task's CPUs to the new cpuset's (0-5) on that move. So what SSH starts pins itself:
  - Helm's robot helpers, since d3ec1db: `helm.calibration.BACKGROUND` runs before each helper's `remote_main`
    (background CPUs, nice 10);
  - the bridge's relay, since 27674a8: `prefer_background_thread` before it joins Helm's hub.
  Both were verified on Minerva.
- **A still's SSH logout stalled the arms (2026-11-02, `live-20261102T022748Z`, the first live Calibrate still).**
  The loop's cycles were on time, but one receiver call took 13 ms with its feedback 19 ms old, and an arm command
  expired 1.5 s into Teleop (`command_expired`, then the arms lowered and disabled). The still's loopback SSH session
  closed in the same 20 ms (sshd's PAM close, systemd stopping its scope, logind removing it). It was the only SSH
  login or logout while the arms were engaged, and the night's only stall without a camera dropping off USB; the
  helper itself had run on CPUs 0-3. The services doing that work were allowed CPUs 4-5: the system slices'
  `AllowedCPUs=0-3` was not in force (an undo had been run in its place). So since then Helm on Minerva runs its
  robot helpers (wrist and head stills, map captures) as its own children (`helm.calibration.run_helper`): no login,
  logout or session scope during teleop, and the child inherits Helm's CPUs 0-3 before `BACKGROUND` lowers it.
- **The old heartbeat dashboard** (`aizee-heartbeat.service`, about 6 % of a core) is a system service, so a drop-in
  holds it to 0-3. The user runs, once:

  ```
  ssh -t ltr@192.168.8.159 "sudo mkdir -p /etc/systemd/system/aizee-heartbeat.service.d && printf '[Service]\nCPUAffinity=\nCPUAffinity=0 1 2 3\n' | sudo tee /etc/systemd/system/aizee-heartbeat.service.d/50-minerva-cpu-partition.conf >/dev/null && sudo systemctl daemon-reload && sudo systemctl restart aizee-heartbeat && systemctl show aizee-heartbeat -p CPUAffinity"
  ```

- **Deep idle on the control CPUs.** cpuidle let CPUs 4-5 enter `c7`, whose wake takes 5 ms, and the receiver's ticks
  started 4-11 ms late. It was switched off by hand through sysfs on 2026-10-31, which lasts only until a reboot.
  This unit switches it off at every boot (staged in `~/minerva/setup/` by Claude):

  ```
[Unit]
Description=Minerva: keep the arms' loop's CPUs 4-5 out of deep idle (a c7 wake takes 5 ms)

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'for c in 4 5; do d=/sys/devices/system/cpu/cpu$${c}/cpuidle/state1; grep -qx c7 $${d}/name && echo 1 > $${d}/disable; done'

[Install]
WantedBy=multi-user.target
  ```

  The user installs it once:

  ```
  ssh -t ltr@192.168.8.159 "sudo install -m 644 /home/ltr/minerva/setup/minerva-control-cpus.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now minerva-control-cpus.service && cat /sys/devices/system/cpu/cpu4/cpuidle/state1/disable /sys/devices/system/cpu/cpu5/cpuidle/state1/disable"
  ```

## The head's receiver on a CPU of its own, real-time (2026-11-03)

- **Branch:** `claude/head-realtime` in `C:\Users\ltr\Workspace\aizee-vr`, from `76cb69b` (the release Minerva ran, `20261103T020147Z-76cb69b`). Owner: Claude, at the user's request ("apply those 3 fix options").
- **Evidence:** the first VR arms session lost the head at 02:19:32Z (`live-head-20261103T021739Z/fault-20261103T021932Z.json`): one 10 ms tick of `head_controller` took 15.2 ms of work, past its `maximum_lateness_us` (15 ms), so the service sent itself an emergency stop and cut the head's torque. No bus failure and no USB event. Across every `live-head-*` session, the receiver's worst work and lateness were 2-9 ms before `ccf9148` (10-31 15:41Z) moved it onto the background CPUs beside the cameras, and 11-19 ms since; overruns before this release on 11-02 02:33Z (18.4 ms) and 11-03 00:19Z (18.6 ms). Late wake-ups grew too, so the stalls are CPU contention, not the serial bus.
- **Change, three parts:**
  1. **A CPU of its own:** the partition keeps CPU 3 for the head's receiver (`"head": [3]`); the cameras, Helm (`helm_service run` moves itself onto the background CPUs) and every background helper keep to 0-2. `neck.head.HeadReceiver` moves the spawned `head_controller` there (`control.scheduling.prefer_head`) and writes where each of its threads runs to `scheduling.json` beside its evidence.
  2. **Real-time there:** `prefer_head` runs every thread of the receiver at `SCHED_FIFO` priority 20 (`REALTIME_PRIORITY`), only on CPUs kept for it, so it can starve nothing else. Unprivileged, that needs the process's `RLIMIT_RTPRIO`: `install` gives the robot user's service manager (`user@UID.service`, which runs Helm and so every session) a `LimitRTPRIO=20` drop-in, `50-minerva-realtime.conf`, and restarts it, with Helm. Without the limit the receiver keeps its CPU at normal priority; with an older partition (no head CPU) nothing changes.
  3. **A late tick is counted, never a stop** (`rust/control_core/src/head_service.rs`, head only): through a late tick each Feetech servo holds the last goal it was given (they keep no watchdog), every goal inside the limits, and a stall long enough for the command or lease to expire still brakes the head to a hold with torque kept (`Controller::tick`). The arms' receiver keeps its emergency stop. The invalid-frame stop is unchanged.
- **Install (once, by the user, no Helm session running):** the same command as above, from the new release. It moves an older split to this one, restarts the cameras and the user's service manager (Helm restarts with it), and reports `partition.head == [3]` and `realtime.limit == 20`.
- **Not moved:** the small `minerva-net` and `minerva-ups` system services keep `CPUAffinity=0 1 2 3`, and system services are still allowed on every CPU (the `AllowedCPUs` apply was not run); the receiver's real-time priority keeps ahead of them.
