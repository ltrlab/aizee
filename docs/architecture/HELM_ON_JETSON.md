# Helm on the Jetson, the GELLO arms on the operator station

Helm, the operator console, runs **on Minerva's Jetson** and is served to any device with a browser. The GELLO leader
arms cannot move there, so they stay on the operator station (a Windows PC), which serves them to Helm through the
GELLO bridge. The leader link's latency is unchanged from when Helm ran on that PC.

This page is how the arrangement works. To install it, follow [INSTALL_JETSON.md](../getting-started/INSTALL_JETSON.md)
and [INSTALL_OPERATOR_STATION.md](../getting-started/INSTALL_OPERATOR_STATION.md). To use it, see the
[Helm manual](../operator/HELM_MANUAL.md).

Earlier, Helm ran on a Windows PC wired to the Jetson, with the leaders, the pedals and the touchscreen on it. That
"terminal" is retired and Helm no longer runs on a PC for Minerva (`helm --sim` still does, for simulation:
[SIMULATE.md](../getting-started/SIMULATE.md)).

## The pieces

```
Browser (any device)  --HTTPS 8760-->  Helm (user service minerva-helm, helm --on-robot)
                                          |  pinned SSH to 127.0.0.1: live view, sessions, Tune, Check,
                                          |  cameras, episodes, UPS
                                          v
                                    loop process (CPUs 4-5)  <--- UDP leader link, signed ---  GELLO bridge (operator station)
                                          |                                                      |  200 Hz USB sampler, leaders
                                    guarded_controller (arms), head_controller (neck)
```

- **Helm** is the user service `minerva-helm` (`helm --on-robot`), started by `ops/helm_service.py run` from the
  active release `~/minerva/current`. It serves the page beyond loopback, so it needs a login.
- **Every loop runs in its own process** on the Jetson's control CPUs. On a PC, the "This PC" choice ran a virtual
  teleop or sweep loop inside Helm's process. On Minerva, teleop, Tune and the command line refuse it and the page
  hides the choice.
- **The release is activated** by Helm as by the loop it starts: the release's verified mesh digests and native paths
  let the model load without the CAD files.

## Robot paths

Every robot path Helm takes is still pinned SSH, now to `127.0.0.1`: the live view, sessions, Tune, Check, cameras,
episodes and the UPS. Each checks the robot's host key and machine id.

- Helm uses its own key, which `authorized_keys` accepts only from `127.0.0.1` and `::1`, with no forwarding and no
  terminal.
- The robot connection is named `robot`, and it counts as wired, like the tether (`helm.session.WIRED`).
- Camera stills and map captures are the exception: each runs as Helm's own child process
  (`helm.calibration.run_helper`), because a still's SSH logout once stalled the arms' receiver during teleop
  ([JETSON_CPU_SPLIT.md](JETSON_CPU_SPLIT.md)).

## The leader arms and the bridge

The GELLO bridge (`teleop.gello_bridge`) runs on the operator station as a task with no window, from startup.

- **Connection.** The bridge holds one pinned SSH session to the Jetson. Its key may run only Helm's relay, as a forced
  command, and only from the tether and access point network (`10.42.0.0/16`) and the network of the device profile's
  address (`authorize-bridge` writes the `from=`). The relay (`helm.bridge relay`) joins Helm's `BridgeHub` on
  loopback port 8761 with a token that only the robot's user can read. Helm's welcome names the robot's machine id,
  and the bridge checks it. The station opens every connection; its only inbound traffic is the leader link's UDP
  requests. How it finds the robot is in [MINERVA_NETWORK.md](MINERVA_NETWORK.md#the-gello-bridge).
- **The leader link is the same code, process and path as before.** For each session the bridge runs
  `LeaderLinkProcess`: the 200 Hz USB sampler and the signed UDP answers over the tether, in its own responsive process.
  It gets the session's link key through the encrypted SSH session. The robot's loop still asks for the latest pair each
  cycle, on CPUs 4-5, so the leader-to-motor latency is unchanged.
- **The Leaders tab** gets the samples streamed with their age on the station. Helm restamps them on arrival, so the
  station's clock never meets the Jetson's.
- **Force feedback** stays in the station's link process ([GRIPPER_FORCE_FEEDBACK.md](../robots/minerva/GRIPPER_FORCE_FEEDBACK.md)).
- **The browser hop** carries only page polling, button presses, pedal keys and the camera relay, all off the control
  path. The pedals are keyboard keys in the operator's browser.

## Access (`helm.access`)

- **Login:** a password, stored as a scrypt hash and set by the operator. Each browser gets a session cookie (HttpOnly,
  SameSite=Strict, Secure over HTTPS) that lasts 12 hours from its last use.
- **Control:** one device at a time is in control and may start, move or change anything. Any logged-in device can take
  control, and can always Disable (H) or Shut down (X). Repeated wrong passwords from one address back off, doubling up
  to a minute.
- **HTTPS:** the certificate is self-signed and made on the robot (`helm_service prepare`). It is valid 825 days and is
  not a CA. Its names are the tether, loopback, the profile's address, the addresses the network service reports and the
  host names, and its use is serverAuth, so phones and tablets accept it once trusted. `helm_service status` shows its
  SHA-256 fingerprint to compare with the browser's. `helm_service certificate` issues it again when it does not name
  an address Minerva has now.
- **Restarts:** sessions live in Helm's memory, so a Helm restart (a deploy) logs every device out.
- **Listeners:** HTTPS on port 8760 for the operator's devices, and plain HTTP on `127.0.0.1:8762` for the robot's own
  tools and SSH tunnels (a browser behind a tunnel cannot pass the self-signed certificate). The login applies on both.

## Operator files (`helm.operator_files`)

Helm saves the joint limits (Limits tab), the servo calibration (Tune tab), the leader ranges (Leaders tab) and the
arms' reference beside the profiles. On Minerva that is inside the verified release, and a changed release file stops
every session from starting. So on the robot Helm reads and saves them in `~/minerva/helm/operator/`.

- Each start seeds missing files from the release.
- A deploy that changes one of these files takes effect: an operator copy never saved over is updated; an operator
  save is moved to `operator/replaced/`, never lost, and the header pill says so.

## Meshes and dependencies

- **Meshes:** the release carries no CAD. The build PC converts each mesh into the 3D view's payload, gzips it and
  uploads it once to `~/minerva/meshes/<digest>.mesh1.gz` (`ops.helm_robot meshes`). The Jetson needs neither the CAD
  nor trimesh. Helm serves each mesh by its release-verified digest, gzip-encoded, with an ETag, so a browser
  revalidates what it holds. The store holds the active model's meshes and the 73 more of the
  [head model](../robots/minerva/HEAD_NECK_MODEL.md) that the 3D view draws.
- **Dependencies:** `requirements/helm-robot.lock` adds aiohttp and its dependencies for the Jetson's CPython 3.10 on
  aarch64. The build PC downloads the wheels with their hashes checked (`ops.helm_robot deps`) and the Jetson installs
  them from those files only, into `~/minerva/helm/deps/<lock digest>`. The system Python and the robot's other
  packages are untouched. A second lock, `requirements/helm-webrtc.lock`, carries the remote headset worker's
  dependencies on that process's path only; Helm runs without it.
- **The release** carries Helm's page, the 3D viewer and the locks, never meshes. `robot-platform deploy` installs the
  new release's dependencies and meshes and then restarts Helm on it, but never while a session or Check runs, so Helm
  and the loops it starts stay on one release.

## Resources

- The service runs on CPUs 0-3 at nice 5, off the control CPUs and below the cameras; it then keeps to the partition's
  background CPUs ([JETSON_CPU_SPLIT.md](JETSON_CPU_SPLIT.md)).
- A rehearsal on the Jetson with synthetic leaders streaming at 50 Hz measured Helm at about 1.5 % of a core idle and
  6 % with the page open, and 65-130 MB of memory. Helm's CPU with the cameras and a recording together has not been
  measured; measure it before a recorded session.
- The camera relay needs about 1.5 MB/s per viewer. Over 2.4 GHz Wi-Fi, which gave 0.8-1.2 MB/s with pauses up to
  359 ms, a slow link gets fewer frames rather than a closed stream, and the state keeps about one frame queued so the
  page's state does not wait behind video.
- The loop sends the pose alone (`guide` messages: the arms' guide and the neck's angles, about 0.1 ms to build) about
  ten times a second in any cycle with 0.5 ms left, never past a deadline, so the 3D views follow the arms even when no
  full view fits the cycle's budget. The Teleop tab's joint rows still update only when a full view fits.

## The header

On Minerva the header shows the GELLO pill and View only / Take control (or In control) in the place of the choice of
route and the mode pill the PC version had. A release that replaced saved operator files says so in a banner. It fits a
1280 px screen.

## Open questions

- A real leader link through the bridge for a long session: the rehearsal used synthetic leaders, and live virtual
  sessions over the tether have run at 100 Hz with no skipped requests (one 20-50 ms gap in 256 s), with the station
  answering within 0.4 ms.
- Each device accepting the self-signed certificate once (a browser, a tablet, the headset).
- Several viewers at once on the camera relay, and its bandwidth to Wi-Fi devices.
- The station's tether adapter must keep its power saving off ([INSTALL_OPERATOR_STATION.md](../getting-started/INSTALL_OPERATOR_STATION.md)).

## Tests

`tests/unit/test_gello_bridge.py` (the link through the hub and relay with a real `LeaderLinkInput`, restamped leaders,
a wrong machine or token refused, a silent Helm ending the connection), `tests/unit/test_helm_on_robot.py` (access and
backoff, the gated server, operator files seeded, kept, updated and set aside, the mesh store's gzip and ETag),
`tests/unit/test_helm_service.py` (prepare, the narrow key lines, the unit, a restart refused under a session, `run`'s
options, mesh upload, dependencies, the bridge package, the locks) and `tests/unit/test_ops_release.py` (a built release
has the page, the viewer, the locks and the buses' hardware profiles, and no meshes).
