# Troubleshooting

What Helm says, what it means, and what to do. Banner and message texts are copied from the code. Safety first: if an arm is
raised and a message says to support it, support it before you press anything. See [Safety](SAFETY.md).

## Helm will not open or will not let you act

| You see | Meaning and fix |
|---|---|
| The page does not load | Is `minerva-helm` running? `systemctl --user status minerva-helm` on the Jetson. Exit code 78 means a missing password, key or dependency and the unit will not restart until fixed: run `python3 -m robot_platform.ops.helm_service status` from `~/minerva/current` (with `PYTHONPATH=src`); it lists what is missing. Install steps: [Install the Jetson](../getting-started/INSTALL_JETSON.md). |
| Certificate warning | Expected: the certificate is self-signed. Accept it once per device and compare its SHA-256 fingerprint with `helm_service status`. |
| Log in page again | A session lasts 12 hours from its last use, and a Helm restart (a deploy) ends every session. Log in again. Wrong passwords back off up to a minute. |
| `Log in to Helm first` | Same. |
| `View only: <device> has control. Take control first.` | Another device is in control. Press **Take control** (twice to confirm). H and X still work from this device. |
| `No GELLO` pill | The bridge on the operator station is not connected: check the station is on, the bridge task runs (it retries by itself), and the tether or network cable. Replay, policies and Check do not need it. |
| `GELLO · Wi-Fi` | Leader teleop runs, but expect holds when samples arrive late. Use a cable. |

## The feed pill and banners

| Message | What to do |
|---|---|
| `Not connected` | Press **Connect** (the reader is off by default at start). |
| `Connecting to <address>: checking host, CAN adapters, services and all motor identities.` | Wait. If it fails, read the banner. |
| `Waiting for motors` / `Not answering: <names>. Check its plug and power; power-cycling is fine. Retrying in N s (try K). Sweep progress is kept.` | Those motors do not answer. Check their power, plugs and the foot switch. Power-cycling while it waits is fine. If every motor is silent, motor power is off or the foot switch is pressed. |
| `Connection failed` | Read the banner: a different motor identity, unconfirmed CAN cleanup or a robot problem. These are not retried. Run Check. |
| `Live feed stale` | The reader is running but frames are old: network trouble or a loaded Jetson. Reconnect. |
| `Reconnecting...` / `Renewing the 15-minute stopped reader; last pose held.` | Normal every 15 minutes. |
| `Motors in use: <owner>` | A session, sweep or Check has the motors. Wait or stop it. |
| `Saved limits were not loaded: <reason>` | A limits file no longer matches the reference or the motors (a new mechanical zero, a replaced motor). Sweep again. |
| `Took over a calibration lock left by exited process N.` | Informational: a previous tool exited without releasing its lock. |
| `<file>: the release brought a new copy; the one saved on Minerva before is kept at <path>` | A deploy carried a changed calibration file and your saved copy moved to `operator/replaced/`. Compare, and re-apply what you need. |
| Battery banners | See [UPS](#ups-and-power). |

## Check results

Run **Check** and read the first failure; later items are skipped because of it.

| Check says | Do |
|---|---|
| `Missing the CAN helpers, ... Run scripts/setup_minerva_system.sh once (it needs sudo).` | The Jetson's system setup was not done. |
| `<name> is missing: check its USB-CAN adapter cable` | An adapter dropped off USB. See [CAN adapter drop-outs](#can-adapter-drop-outs). |
| `<name> is adapter X, expected Y` | The wrong adapter is on that port; adapters are pinned by serial. |
| `<name> is already up: another program owns it (Helm Limits connected?)` | Disconnect, stop teleop and sweeps, stop any other CAN tool. |
| `Running (or unreadable): <services>. Stop them before operating.` / `A guarded receiver is already listening` | Another motor owner runs. Stop it. |
| `No motor answered ... Motor power is off or the foot switch (e-stop) is pressed.` | Power and foot switch. |
| `Every motor on <bus> is silent (...): check that arm's power and CAN cable.` | That arm's supply or CAN cable. |
| `A latched fault clears only with a power cycle.` | Power-cycle the motors. |
| `CAN links were not confirmed down afterwards: check them before anything else.` | Do not start anything. Check the interfaces (`ip link show canL canR`) and bring them down. |
| A joint a whole turn off | The drives' power-on range: see [mechanical reference](MECHANICAL_REFERENCE.md#why-an-apparent-zero-may-change). |
| `Release matches this PC` fails | The release on the robot differs from the files Helm runs from: redeploy ([below](#deploy-and-roll-back)). |
| Live sweep failed | Read the joint's card: a guard event, lag over 55 % of the bound, or friction up on the swept value. Re-sweep in Tune. |

## Reading a trip

When the receiver stops itself, the Teleop message reads like:

`LIVE LEFT ARM FAULT · Receiver stopping: limit_violation (Left J4 moving at X rad/s for three readings, over the Y rad/s allowed there) · <detail>`

then either "The receiver braked the arm to a hold; lowering to the reference, then disabling...", or "PAUSED ... taking it
back there...", or "The receiver stopped itself ... Stop the session and start it again". The first word after `Receiver` is
the state (`stopping`, `stopped`, `faulted`), the second the receiver's **reason**, the parenthesis its own account of the
joint and reading. The reasons you will see:

| Reason | Meaning |
|---|---|
| `limit_violation` | A joint broke a check. The parenthesis names it: `at ... below its lower limit`, `above its upper limit`, `moving at ... rad/s, over three times the ... allowed there` (`speed`), `... for three readings, over the ... allowed there` (`sustained_speed`), `torque ... over its ... Nm limit` (`effort`), `at ... C, over its ... C limit` (`temperature`) |
| `tracking_error` | `<joint> ... rad from its command, over its ... bound`: the joint could not follow, or is blocked or caught |
| `collision_violation` / `collision_expired` | Collision box (not used in the production profile) |
| `command_expired` | No fresh command reached the receiver in its window (30 ms in production): the loop stalled. See [late cycles](#late-cycles-and-command_expired) |
| `lease_expired` | The loop's ownership lapsed (150 ms in production): the loop stopped or stalled |
| `feedback_stale` | `<joint> silent for N ms (limit M ms)`: one drive stopped answering; the receiver holds the arm on the others. `every drive silent` cuts torque |
| `feedback_invalid` | `<joint> sent invalid feedback` or `feedback went backwards`: torque is cut |
| `hardware_fault` | `<joint> reports a drive fault`, or an adapter failure: torque is cut. A drive fault clears only with a power cycle |
| `emergency_stop` | The receiver service's own emergency stop (a late service cycle or a malformed frame): torque is cut |
| `wrong_owner`, `wrong_lease` | A consequence: the receiver already stopped and revoked the lease. Read the first reason |

The receiver's last status is saved as `live-<bus>-last.json` in the session folder, with `events.jsonl` and `samples.f32`
(every tick at 400 Hz: time, then each joint's position, velocity, effort, command). Read the first reason in `events.jsonl`,
not the last.

| Banner | Meaning |
|---|---|
| `LIVE BOTH ARMS PAUSED ... The receiver braked the arm to a hold; taking it back there...` | A camera dropped off USB, or (in a release with trip hold) a joint was too fast. The arms hold; **T** follows again. Three within a minute are lowered. |
| `... dropped off USB. The arms hold: Engage (T) follows again once it is back.` | Reseat the camera cable. See [Cameras](#cameras). |
| `waiting to re-arm it (feedback_stale)` / `If it stays held, support the arm, then press H to disable it (it goes limp).` | The receiver holds the arm but cannot re-arm yet. Support the arm and press H if it persists. |
| `... stopped answering and is set aside. ` | A gripper went quiet; the arm is lowered without it. |
| `<fault> · The arms hold: Engage (T) follows again without the head; stop the session to start the head again.` | The head faulted (a servo alarm, lost feedback, its process). The arms hold. |
| `Leader input lost; holding measured pose until Engage` | The leader samples stopped for 80 ms. Check the bridge, the leaders' USB and the link, then **T**. |
| `Disable could not be confirmed: ... Use the foot switch; the session remains faulted.` | The software Disable did not confirm. Use the foot switch. |
| `Lowering stopped: ... Stop the session and start it again.` | The lowering failed. Support the arm, **H**. |
| `RECORDING FAILED · <reason> · ...` | A camera, the disk or the writer failed. The arms hold, following is locked. H disables; X or Stop session returns to the reference. Restart the session. |
| `<name> left Run mode while driven (it rebooted, or its own watchdog disabled it)` | That motor lost power or its watchdog fired. Support the arm, press H, power-cycle. |
| `Helm went silent for 5 s` (in the evidence) | Helm stopped or restarted during a session; the loop lowered and disabled the arm. |
| `Minerva's UPS battery is critical ...` | The UPS is shutting the Jetson down. Plug in the charger. |

### Waiting for motors

See the feed pill table. The reader retries every few seconds with a full identity check each time. It stops and tells you if
a motor's identity differs or CAN cleanup was not confirmed.

### Late cycles and `command_expired`

The loop must send a command every 10 ms. A stall of tens of milliseconds expires the command and the receiver stops the arm.
Known causes in the record: a camera dropping off USB (a hotplug burst on every CPU), a login or logout on the Jetson during
teleop (Helm's own helpers avoid it), heavy work on the control CPUs (4-5), deep CPU idle states (c7, switched off by the
CPU-partition unit), a loose camera cable. Check `journalctl -k` for `usb ... disconnect`, the camera services, and that
nothing else is using the control CPUs ([CPU split](../architecture/JETSON_CPU_SPLIT.md)). The loop writes any cycle that
started more than 40 ms after the last to `slow-cycles.jsonl` in a sweep's folder.

## CAN adapter drop-outs

The two USB-CAN adapters are pinned by serial and named `canL` and `canR`. When one drops:

1. Check the cable and USB port first (`USB-CAN adapter cable`, in Check). A loose cable is the commonest cause.
2. If it is wedged but still enumerated, `aizee-can-reset canL` (or `canR`) issues a USB port reset and brings the interface
   back up. It touches only the named adapter. If the adapter has left the USB bus entirely, it exits with "dropped off the bus
   and needs a physical re-plug".
3. Run Check. Never reconnect with an interface left up by something else.

A patched `gs_usb` driver is installed on the unit for the adapters' bus errors; its record is in
`scripts/maintenance/gs_usb/README.md`. A motor that reports a drive fault or "latched faults" needs a power cycle.

## Cameras

Helm's cameras: left wrist (port 5563), right wrist (5565), head (5564), all on the Jetson's loopback. Their services are
`minerva-left-wrist`, `minerva-right-wrist` and `aizee-scene-cam` (the head's D435; the `aizee-` name is Minerva's).

- A camera shows "Waiting for frame" or stale: `systemctl status minerva-left-wrist` etc. on the Jetson; reseat its USB cable.
  A camera that drops pauses the arms during a session.
- Recording needs all three fresh and the head's pose feed live; `3/3 cameras fresh` in the task card is per session, not a
  property of the service.
- On Wi-Fi the camera view sends fewer frames rather than closing; on a slow link prefer the cable.
- Exposure and gain are in Settings > Cameras and last until the camera service restarts.

## The headset

- The page will not load or says to log in: open `https://<robot-address>:8760/xr`, accept the certificate, log in. A Helm
  restart ends the login.
- The head does not move: the session must be engaged (T) and the head included (a head fault shows in the Head card). Press
  **Start head tracking** after looking straight ahead; **Zero headset** re-centres.
- The picture is nearly still on a hotspot or the internet: use `/xr?remote` ([remote headset](REMOTE_HEADSET.md)). If the
  button beside Start says "Use the remote link" you are on the ordinary one.
- A second headset is refused while one is connected.

## Deploy and roll back

Releases are built on a development machine and installed with `robot-platform deploy` (`python -m robot_platform deploy`): it uploads, verifies on the robot, switches `~/minerva/current` atomically, runs the smoke test, prunes old releases
(the newest three are kept, always including the current and previous), and restarts Helm unless a session or Check runs.

- **The smoke test fails:** the deploy switches back to the previous release by itself and says so
  ("rolled back to <id>").
- **Roll back by hand:** on the Jetson, point `current` at a kept release and restart Helm when no session runs:
  `cd ~/minerva && ln -sfn releases/<id> current.new && mv -Tf current.new current`, then
  `python3 -m robot_platform.ops.helm_service restart` from `~/minerva/current` with `PYTHONPATH=src`.
- **Helm was not updated:** the deploy prints "Helm on Minerva was not updated: ...". When no session runs:
  `python -m robot_platform.ops.helm_robot update`.
- What each deploy did is one line in `~/minerva/deploy.log`.

## Service and logs

```
systemctl --user status minerva-helm        # state
systemctl --user restart minerva-helm       # only with no session running; a deploy refuses otherwise
journalctl --user -u minerva-helm -n 100    # recent output
tail -n 100 ~/minerva/helm/logs/helm.log    # what Helm prints
```

| Where | What |
|---|---|
| `~/minerva/helm/logs/helm.log` | Helm's output |
| `~/minerva/evidence/live-<time>/` | A live session: events, ticks, the receiver's last status, episodes, `capture-worker-*.stderr.log` |
| `~/minerva/evidence/sweep-<time>/`, `check-sweep-<time>/` | Tune and Check sweeps |
| `~/minerva/evidence/remote-link-<UTC>.jsonl` | A remote-headset worker's run |
| `~/minerva/helm/runs/` | Helm's sessions and calibration evidence |
| `~/minerva/helm/operator/` (and `replaced/`) | Saved limits, servo calibration, leader ranges, head files, the arms' reference |
| `~/minerva/helm/calibration/<run>/` | Camera calibration stills and solves |
| `/var/log/minerva-ups/events.jsonl` | UPS transitions; it survives a power cut that the journal does not |
| `~/minerva/deploy.log` | One line per deployment |

Other system services: `minerva-net` (`journalctl -u minerva-net`, the access point and tether), `minerva-ups`
(`systemctl status minerva-ups`), and the CPU-partition unit.

## UPS and power

- **Battery pill** `—` with "no UPS monitor running": `minerva-ups` is not installed.
- **"Minerva is running on its UPS battery: about N%, roughly M before it shuts itself down at 10%."** Plug in the charger.
- **"Minerva's UPS battery is critical ...: Helm ends any session, lowering the arms first, and the Jetson powers off within N s."**
  Plug in the charger now to cancel. Starts that move the arms are refused until it is cancelled.
- **"Minerva went offline while running on its UPS battery"**: it may have shut down; charge it, then power the Jetson on.
