# Jetson CPU split: the arms' loop on its own cores

The Jetson has six Cortex-A78AE cores and no CPU isolation. The split gives the arms' control loop and receiver CPUs 4
and 5, the head's receiver CPU 3, and everything else (the cameras, Helm, background helpers) CPUs 0 to 2. It is
installed once with `ops.cpu_partition` ([INSTALL_JETSON.md](../getting-started/INSTALL_JETSON.md#8-cpu-partition));
this page says why and how it works.

## Why

A live session faulted on an expired command. A command expires about 35 ms after its leader sample, and the sample is
already about 12-13 ms old when the loop accepts it. That leaves about 20 ms, so a loop stall of 10 ms or more expires
the command.

The three camera services use about one core between them: left wrist about 44 % of a core, right wrist about 37 %,
head about 21 %. The wrist nodes decode and re-encode JPEG. Before the split they ran on every core, beside the teleop
loop's process (about 23 % of a core) and its receiver.

Three holds with both arms Enabled and never Engaged, on the same receiver configuration:

| Condition | Hold | Loop misses | Cycles over 10 ms | Cycle p50 / p99 / max | Smallest expiry margin | Receiver misses / max lateness | Fault |
|---|---|---|---|---|---|---|---|
| Cameras sharing all cores | 77 s | 60 | 24 | 5.6 / 9.2 / 14.0 ms | 2 ms | 18 / 6.5 ms | no |
| Cameras paused (SIGSTOP) | 28 s | 1 | 1 | 4.8 / 5.0 / 15.2 ms | 8 ms | 0 / 0.4 ms | yes, one 15 ms whole-process stall |
| Cameras on CPUs 0-3; loop and receiver on 4-5 (by hand) | 104 s | 0 | 0 | 5.1 / 6.5 / 8.1 ms | 7 ms | 7 / 3.5 ms | no |

The installed split, in a 146 s hold (Start, Idle, Enable, Hold, Shutdown, Stop; no Engage, no fault), gave 1 missed
cycle, none over 10 ms, p50 / p99 / max 5.7 / 7.2 / 9.9 ms, smallest expiry margin 5 ms, receiver misses 7 with 3.5 ms
maximum lateness. All 6 threads of the loop process and all 3 of its `guarded_controller` ran on CPUs 4-5.

## How it works

**`control/scheduling.py`**
- `/etc/minerva/cpu-partition.json` holds the partition: `control` (4, 5), `head` (3) and `background` (0, 1, 2).
- On Linux, where that file exists, `prefer_responsive` moves every thread of a process onto the control CPUs,
  `prefer_background` onto the background CPUs, and `prefer_head` moves the head's receiver onto its own CPU.
- Without the file, as on PCs and CI, nothing moves. Control CPUs only help where the cameras keep off them. A
  malformed file is an installation error and raises.

**`helm/edge.py`** the robot-side loop applies `prefer_responsive` first, before its session starts any thread. Its
threads and its child processes inherit the control CPUs; the children are the receiver (`guarded_controller`) or the
control harness. The session view reports it as `scheduling.loop` (`["control_cpus"]`).

**`helm/task_capture_process.py`** live task capture's worker is spawned by the loop and moves itself onto the
background CPUs before it decodes or writes anything.

**`ops/cpu_partition.py`** the one-time installer for the cameras' half, run with sudo:
- gives the three camera services (`aizee-scene-cam`, `minerva-left-wrist`, `minerva-right-wrist`) a `CPUAffinity`
  drop-in (`/etc/systemd/system/<unit>.d/50-minerva-cpu-partition.conf`);
- restarts the running ones and checks every camera thread now runs on the background CPUs;
- only then writes the partition file, so the loop never keeps to 4-5 while the cameras can still use them;
- `remove` deletes the file first, then the drop-ins; both refuse while a Helm session runs on the robot, because a
  restart interrupts the camera's stream; `install` refuses if the partition's CPUs are not all online (a power mode
  can take some offline); installing again moves an older split to the current one. `status` needs no sudo.

Helm's service moves itself onto the background CPUs when it starts (`helm_service run`), so a deploy, which restarts
it, applies a changed split.

## The head's receiver on a CPU of its own

Beside the cameras on CPUs 0-3, the head receiver's 10 ms ticks started up to 15 ms late, and one late tick stopped the
head mid-session: a tick took 15.2 ms of work, past its `maximum_lateness_us` (15 ms), so the service cut the head's
torque. Across every recorded head session, the receiver's worst work and lateness were 2-9 ms before it shared the
cameras' CPUs and 11-19 ms after. The stalls were CPU contention, not the serial bus.

Two changes:
1. **A CPU of its own:** the partition keeps CPU 3 for the head's receiver (`"head": [3]`); the cameras, Helm and every
   background helper keep to 0-2. `neck.head.HeadReceiver` moves the spawned `head_controller` there and writes where
   each of its threads runs to `scheduling.json` beside its evidence.
2. **A late tick is counted, never a stop** (`rust/control_core/src/head_service.rs`, head only): through a late tick
   each Feetech servo holds the last goal it was given (they keep no watchdog), every goal inside the limits, and a
   stall long enough for the command or lease to expire still brakes the head to a hold with torque kept
   (`Controller::tick`). The arms' receiver keeps its emergency stop. The invalid-frame stop is unchanged.

**Real-time priority was tried and taken out.** The first version ran the receiver at `SCHED_FIFO` 20 and had `install`
give the robot user's service manager (`user@<uid>.service`, which runs Helm) a `LimitRTPRIO=20` drop-in, then restart
it. The restart failed to attach the old manager's cgroup, Helm was down about 5 s, and the manager came up with a
real-time limit of 0: a probe printed 20 as root, 20 with `User=`, and 0 with `PAMName=systemd-user`, so the manager's
PAM session drops it. Rehearsed instead on the Jetson (virtual bus, the profile's 10 ms ticks and 15 ms limit, four busy
loops, 30 s each): on CPU 3 with the load on 0-2, 0 deadline misses and 8.0 ms worst lateness; on 0-2 beside the load, 3
misses and 11.9 ms; on CPU 3 with the load on 0-3 too, 11 misses and 14.3 ms. `install` and `remove` delete the stale
drop-in and never restart the manager. Granting real-time would need a dedicated `/etc/pam.d/systemd-user` ending in
`pam_limits` with an `rtprio` entry for the robot user.

## Work started over SSH

sshd and every session it starts are allowed CPUs 0-5, including Helm's robot paths (loopback SSH) and the GELLO
bridge's relay (its key's forced command). A head map capture launched that way ran on CPUs 4-5 about a fifth of its
time, and the loop's slow cycles rose fourfold once it started.

Pinning the ssh service would not help: a login moves its session into `user.slice/user-<uid>.slice/session-N.scope`,
whose cpuset differs from `system.slice/ssh.service`'s, and this kernel (5.15) resets a task's CPUs to the new cpuset's
on that move. So what SSH starts pins itself:
- Helm's robot helpers: `helm.calibration.BACKGROUND` runs before each helper's `remote_main` (background CPUs,
  nice 10);
- the bridge's relay: `prefer_background_thread` before it joins Helm's hub.

A still's SSH logout also stalled the arms once: the loop's cycles were on time, but one receiver call took 13 ms with
its feedback 19 ms old, and an arm command expired 1.5 s into Teleop. The still's loopback SSH session closed in the
same 20 ms (sshd's PAM close, systemd stopping its scope, logind removing it). So Helm on Minerva runs its robot helpers
(wrist and head stills, map captures) as its own children (`helm.calibration.run_helper`): no login, logout or session
scope during teleop, and the child inherits Helm's CPUs before `BACKGROUND` lowers it.

## Services outside the split

- **The old heartbeat dashboard** (`aizee-heartbeat.service`, about 6 % of a core, an AIZEE-era status page on port
  8088) is a system service, so a drop-in holds it to 0-3. Run once on the Jetson with sudo:

  ```
  ssh -t ltr@<robot-address> "sudo mkdir -p /etc/systemd/system/aizee-heartbeat.service.d && printf '[Service]\nCPUAffinity=\nCPUAffinity=0 1 2 3\n' | sudo tee /etc/systemd/system/aizee-heartbeat.service.d/50-minerva-cpu-partition.conf >/dev/null && sudo systemctl daemon-reload && sudo systemctl restart aizee-heartbeat && systemctl show aizee-heartbeat -p CPUAffinity"
  ```

- **`minerva-net` and `minerva-ups`** keep `CPUAffinity=0 1 2 3`; system services are otherwise still allowed on every
  CPU, so they and work started over SSH can still run on CPU 3. A cgroup-wide `AllowedCPUs` was not applied.

## Deep idle on the control CPUs

cpuidle let CPUs 4-5 enter `c7`, whose wake takes 5 ms, and the receiver's ticks started 4-11 ms late. Switching it
off through sysfs lasts only until a reboot. This unit does it at every boot. It is not a file in the repository: write
it to `~/minerva/setup/minerva-control-cpus.service` on the Jetson.

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

Install it once with sudo:

```
sudo install -m 644 ~/minerva/setup/minerva-control-cpus.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now minerva-control-cpus.service && cat /sys/devices/system/cpu/cpu4/cpuidle/state1/disable /sys/devices/system/cpu/cpu5/cpuidle/state1/disable
```

Both files should print 1.

## Install, check, undo

```bash
cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.cpu_partition install
python3 -B -m robot_platform.ops.cpu_partition status     # no sudo
sudo python3 -B -m robot_platform.ops.cpu_partition remove
```

Stop any Helm session first; the cameras restart. `-B` keeps a root run from leaving root-owned bytecode in the
release, which the deploy, running as the robot's user, could then not prune. After an install, `status` shows each
camera's threads on the background CPUs and the partition in place, and a Start, Idle, Enable hold reports
`scheduling.loop == ["control_cpus"]`.

## Remaining uncertainty

- The 15 ms stall seen with the cameras paused is not explained by camera load; the partitioned hold had none, but
  bounding its rate needs longer holds.
- The arms' receiver still missed 7 deadlines (3.5 ms late at most) in the partitioned holds, because it shares two
  cores with the loop's Python process. Real-time priority for it was not tried for the arms.
- Live task capture runs on the background CPUs with this change, but has not been run that way on the robot.
- The wrist nodes' `passthrough_mjpg` option would cut their decode and re-encode load; it is not used.
- Tests: `tests/unit/test_cpu_partition.py` and `tests/unit/test_cpu_partition_install.py` are in the gate.
