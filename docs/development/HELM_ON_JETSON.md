# Feature task: Helm on Minerva's Jetson, the GELLO arms on the operator PC (Option A)

- **Branch and worktree:** `claude/helm-on-jetson` in `C:\Users\ltr\Workspace\aizee-helm-pedals`.
- **Base commit:** `e83e224`, the policy-mode record on `claude/live-replay`.
- **Owner:** Claude (the 2026-10-22 session), at the user's request.
  - The user asked: "Can we have the service run on the jetson so we can control it with any device connected?"
  - They then corrected the plan: "why would the gello arms need to be plugged into the jetson, that is not
    feasible. how can we make it so helm can read the gello arms from its web app with the same latency we
    achieved?"
  - They chose this option: "yes, build option A, and the 3080 machine is online at 192.168.8.225".
- **Problem:**
  - Helm ran on the Minerva terminal: a Windows PC wired to the Jetson, with the leader arms, the pedals and the
    touchscreen.
  - On 2026-10-23 the terminal's RAM failed Windows Memory Diagnostic, after bit flips and a blue screen. It is off
    the robot, and the RTX 3080 PC (Windows 11 Pro) takes its place.
  - The user also wants Helm reachable from any device. The GELLO leader arms must stay on a PC, and the leader
    link's latency must not change.
- **Design:**
  - **Helm runs on the Jetson** (`helm --on-robot`) as the user service `minerva-helm`, from the active release
    `~/minerva/current` (`ops.helm_service`). It serves the page beyond loopback, so it needs a login.
    - **Robot paths are unchanged.** Every robot path Helm takes is still pinned SSH, now to `127.0.0.1`: the live
      view, sessions, Tune, Check, cameras, episodes and the UPS. Each one checks the robot's host key and machine
      id as before.
      - Helm uses its own key, which `authorized_keys` accepts only from `127.0.0.1`/`::1`, with no forwarding and
        no terminal.
      - The robot connection is `robot`, and it counts as wired, like the tether (`helm.session.WIRED`).
    - **It activates the release it runs from**, like the loop it starts. The release's verified mesh digests and
      native paths let the model load without the CAD files.
  - **The leader arms stay on the operator PC.** Its GELLO bridge (`teleop.gello_bridge`) runs as a task without a window, from startup.
    - **The bridge connection:**
      - The bridge holds one pinned SSH session over the tether.
      - Its key may run only Helm's relay, as a forced command, and only from the tether network `10.42.0.0/24`.
      - The relay (`helm.bridge relay`) joins Helm's `BridgeHub` on loopback port 8761 with a token that only the
        robot's user can read.
      - Helm's welcome names the robot's machine id, and the bridge checks it.
      - The PC opens every connection and accepts nothing inbound.
    - **The leader link is the same code, process and path as before:**
      - For each session, the bridge runs `LeaderLinkProcess`: the 200 Hz USB sampler and the signed UDP answers
        over the tether, in its own responsive process.
      - It gets the session's link key through the encrypted SSH session, as when Helm ran on the PC.
      - The robot's loop still asks for the latest pair each cycle, on CPUs 4–5. So the leader-to-motor latency is
        unchanged.
    - **The Leaders tab** gets the samples streamed with their age on the PC. Helm restamps them on arrival, so the
      PC's clock never meets the Jetson's.
    - **Force feedback** stays in the PC's link process, as before.
  - **The browser hop changes.** Browser → Helm used to be loopback; it is now the tether or Wi-Fi. That hop
    carries only page polling, button presses, pedal keys and the camera relay, all off the control path. The
    pedals are keyboard keys in the operator PC's browser, as before.
  - **Access** (`helm.access`):
    - **Login:** a password, stored as a scrypt hash and set by the operator. Each browser gets a session cookie
      (HttpOnly, SameSite=Strict, Secure over HTTPS) that lasts 12 hours from its last use.
    - **Control:**
      - One device at a time is in control and may start, move or change anything. Any logged-in device can take
        control.
      - Any logged-in device can always Disable (H) or Shut down (X).
      - Repeated wrong passwords from one address back off, doubling up to a minute.
    - **HTTPS:** the certificate is self-signed and made on the robot. It is valid 825 days and is not a CA. Its
      names are the tether, Wi-Fi and host names, and its use is serverAuth, so phones and tablets accept it once
      trusted. `helm_service status` shows its SHA-256 fingerprint to compare with the browser's.
    - **Restarts:** sessions live in Helm's memory, so a Helm restart (a deploy) logs every device out.
  - **Operator files** (`helm.operator_files`):
    - **The problem:** Helm saved the joint limits, servo calibration and leader ranges beside the profiles. On
      Minerva that is inside the verified release, and a changed release file stops every session from starting.
    - **On the robot,** Helm reads and saves those files in `~/minerva/helm/operator/`.
    - **Each start** seeds missing files from the release.
    - **A deploy that changes one of these files** takes effect, as a console deploy did on the terminal:
      - an operator copy never saved over is updated;
      - an operator save is moved to `operator/replaced/`, never lost, and the header pill says so.
  - **Meshes:**
    - The release still carries no CAD. The build PC converts each mesh into the 3D view's payload, gzips it and
      uploads it once to `~/minerva/meshes/<digest>.mesh1.gz`. The Jetson needs neither the CAD nor trimesh.
    - Helm serves each mesh by its release-verified digest, gzip-encoded, with an ETag. A browser revalidates what
      it already holds instead of downloading it again.
    - The store holds 158 distinct meshes: 142 MB of STL as 35.5 MB gzip, converted in 12 s.
    - The 3D view draws the head model (`helm.display_model`, see [HEAD_NECK_MODEL](HEAD_NECK_MODEL.md)). Its 73
      meshes that the active model lacks, 108 MB of STL, go to the same store.
  - **Dependencies:** `requirements/helm-robot.lock` adds aiohttp and its dependencies for the Jetson's CPython 3.10
    on aarch64, at the development lock's versions plus async-timeout. That is 11 wheels, 2.9 MB.
    - The build PC downloads them with their hashes checked (`ops.helm_robot deps`).
    - The Jetson installs them from those files only, into `~/minerva/helm/deps/<lock digest>`.
    - The system Python, numpy and the robot's other packages are untouched.
  - **The release** now carries Helm's page, the 3D viewer (29 files, 1.7 MB) and the lock, never meshes.
    `robot-platform deploy` installs the new release's dependencies and meshes, then restarts Helm on it. It does
    not restart Helm while a session or Check runs, so Helm and the loops it starts stay on one release.
  - **CPUs:** the service runs on CPUs 0–3 at nice 5, off the control CPUs 4–5 and below the cameras.
  - **Loops stay out of Helm.** On the operator PC, the "This PC" choice ran a virtual teleop or sweep loop inside
    Helm's process. On Minerva, teleop, Tune and the command line refuse it, and the page hides the choice. Every loop
    runs in its own process on CPUs 4–5.
  - **Listeners:**
    - HTTPS on port 8760 for the operator's devices.
    - Plain HTTP on `127.0.0.1:8762`, for the robot's own tools and SSH tunnels. A browser behind a tunnel cannot pass
      the self-signed certificate.
    - The login applies on both.
  - **The header still fits 1280 px.** On Minerva the GELLO pill and View only · Take control (or In control) take
    the place of the Via choice and the mode pill. A release that replaced saved files says so in the banner.
- **Install** (each write to `authorized_keys` and each deploy with the user's approval):
  0. The operator station (the RTX 3080 PC, `soliloquy`), once, by the user, as Administrator:
     `scripts/setup_operator_station.ps1 -AuthorizeKey "<the build PC's public key>"`.
     - It sets up OpenSSH Server, keys only, from the LAN, the tether and Tailscale only.
     - It turns off sleep on mains power and USB selective suspend.
     - It turns off the tether adapter's power saving.
     - Deploys reach Minerva through the station, over the tether.
  1. From the build PC:
     - `python -m robot_platform.ops.helm_robot --address <robot> deps`
     - `python -m robot_platform.ops.helm_robot --address <robot> meshes`
     - deploy a release that has this branch.
  2. On the Jetson:
     - `cd ~/minerva/current && PYTHONPATH=src python3 -m robot_platform.ops.helm_service prepare`
     - then `authorize-local`.
  3. The user, once: `sudo loginctl enable-linger ltr`, and Helm's password with
     `PYTHONPATH=src python3 -m robot_platform.helm.access set-password ~/minerva/helm/access.json`.
  4. On the Jetson: `helm_service install`. Helm then serves `https://10.42.0.1:8760/` on the tether and the Wi-Fi
     address beside it.
  5. On the 3080:
     - build the package with `python -m robot_platform.ops.helm_robot bridge`;
     - run `scripts/install_gello_bridge.ps1 -Package gello-bridge-<commit>.zip` as the operator;
     - authorize the printed key on the Jetson with
       `helm_service authorize-bridge --name operator-3080 --key "<line>"`.
- **Shared interfaces held fixed:**
  - the motion contract;
  - the receiver;
  - the leader link protocol and timing;
  - the recording format;
  - the release verification.
  The release gains files; its format and checks do not change.
- **Acceptance:**
  1. `python scripts/dev.py check --offline` passes with zero skips.
  2. On the Jetson, without installing anything: Helm from a temporary folder on a spare port answers the login,
     the page, the viewer and every mesh. Measure its CPU and memory.
  3. With the user's approval: install the service and keys, then use Helm from the laptop and a phone or tablet.
     Check the live view, Check, cameras and Replay.
  4. With the 3080 set up and the user present:
     - a bridge session;
     - the Leaders tab;
     - a teleop session in the virtual receiver, then live;
     - leader-link timing compared with the terminal's.
- **Results:**
  - **Tests:**
    - `tests/unit/test_gello_bridge.py` (4): the link through the hub and relay with a real `LeaderLinkInput`;
      restamped leaders and their mutual exclusion; a wrong machine or token refused; a silent Helm ending the
      connection.
    - `tests/unit/test_helm_on_robot.py` (9):
      - access, backoff and stop requests;
      - the gated server;
      - the robot connection and Check's tether item;
      - teleop needing the bridge;
      - operator files seeded, kept, updated and set aside;
      - Helm reading and saving them in their folder;
      - the mesh store's gzip, ETag and 304.
    - `tests/unit/test_helm_service.py` (8):
      - prepare's folders, key and certificate;
      - the narrow key lines, idempotence, refusal and removal;
      - the unit, and a restart refused under a session;
      - `run`'s options, or what is missing;
      - mesh upload verified on the far side;
      - dependencies installed once from uploaded wheels;
      - the bridge package;
      - the locks pinned to the development versions.
    - `tests/unit/test_ops_release.py`: a built release has the page, the viewer, the lock and the buses' hardware
      profiles, and no meshes.
  - **Gate:** `scripts/dev.py check --offline` passed on `a354600`: 1295 tests, none skipped
    (`build/dev-check/20261023T042859Z-e9f28e1e/report.json`). The first run, on `5a4a4a9`, failed one test:
    Check's servo-path stub took two arguments, and Check now passes the operator folder too.
  - **Bridge installer:** `scripts/install_gello_bridge.ps1` was run on this laptop in a sandbox folder, without
    its task. It checked the package, built the hash-locked environment (pip check clean), made the key and
    restricted it, and wrote the runner. `Set-Acl` had failed for an ordinary user, so the key now goes through
    `icacls`.
  - **Rehearsed on the Jetson, 2026-10-23, nothing installed:**
    - **Setup:**
      - The release (`fe4ad93`), the wheels and the meshes went under `/tmp/helm-rehearsal`, with `HOME` pointed
        there. `/dev/shm` does not work for this: logind removes a user's files there when their last session ends
        (`RemoveIPC`).
      - Without lingering, the same logout also stops the user's services, so the install needs
        `enable-linger`.
      - It all ran as the robot's user, and nothing in `~/minerva`, `authorized_keys` or systemd changed. Everything
        was removed afterwards.
    - **`prepare` on the robot:**
      - It made the key, and a certificate with the right names: 10.42.0.1, 127.0.0.1 and 192.168.8.180, plus
        `aizee-rover` and `aizee-rover.local`. The certificate is not a CA, is for serverAuth, and is valid 825 days.
      - The 11 wheels installed with their hashes checked, and aiohttp 3.14.4 imports.
      - `status` found all 158 meshes against the release's asset lock and listed nothing missing once a rehearsal
        password was set.
    - **Helm itself:**
      - It activated the verified release and served over HTTPS and HTTP: the login, the page, and the 3D view with
        the whole model from the mesh store.
      - All 220 meshes (41.4 MB gzip) were served locally in 0.66 s, using 0.46 s of Helm's CPU, and every
        revalidation was a 304.
      - **CPU:** idle, 1.5 % of a core. With the page open and synthetic leaders streaming through the bridge at
        50 Hz, 6.0 %; the relay used 0.3 %.
      - **Memory:** 65–130 MB.
    - **The bridge over real SSH** (laptop → relay → hub):
      - It connected, and the machine id matched.
      - The Leaders tab streamed the synthetic arms at "Live · 7 ms".
      - When Helm restarted, the bridge reconnected about 4 s later by itself.
    - **Release verification:** page files patched in place stopped Helm at start ("changed …"). That is the
      protection the operator folder exists for.
    - **Found by the rehearsal and fixed:**
      - Releases lacked `config/hardware_minerva_{left,right}.yaml`, which the direct tuning reads, so Helm would not
        start.
      - A heredoc broke `helm.js` with a raw newline. `test_browser_scripts` catches it, but it had not been run
        after that edit.
      - Hidden pills showed as empty boxes.
      - The header measured 1633 px at a width of 1280 px, pushing the UPS pill and Connect off-screen.
      - The "This PC" loop placement on Minerva.
      - No way in for a tunnel's browser, hence the loopback listener.
- **Deployed 2026-10-23 ~19:30Z,** with the user's approval ("let's deploy to Minerva from the 3080 machine"):
  - **The route:** through the operator station (`soliloquy`, reached over Tailscale) and the LAN to the Jetson's
    Wi-Fi, because the tether was not connected yet. The release was built on the laptop. The station only relays the
    SSH connection, so no robot key is stored on it.
  - **The release:** `20261023T192755Z-ec18fdd`, replacing `20261022T235916Z-f486fb3`. The smoke test passed:
    collision model 972 ms, keep-out p50 1.55 ms, receiver harness disarmed. Nothing was pruned.
  - **Helm:**
    - Dependencies `78fd3dfc446b`, 158 meshes (35 MB), and Helm's key and certificate (SHA-256
      `66:AF:47:0B:…:55:CB:00`).
    - Helm's loopback-only line in `authorized_keys`, with a dated backup of the file kept.
    - `minerva-helm` is enabled and stops at start (exit 78) until the operator sets the password; lingering is
      still off.
  - **The operator state salvaged from the terminal:** the directions, the Motor tuning preset and the routes went
    into `~/minerva/helm/runs`, matching by hash. Loaded as Helm loads them, the right arm's J1, J3 and J4 are −1,
    the verified mapping.
  - **The station:**
    - Python 3.12.10 per user, signature checked.
    - The GELLO bridge (`87b3478`) as the logon task, its key made. It retries until the tether is up and its key is
      authorized.
    - **Installer fixes found on the station:**
      - The redirected log came out as PowerShell error records in UTF-16. The bridge now writes its own UTF-8
        `--log`.
      - Stopping the task left the bridge running.
      - The installer's first filter for old bridges matched the installer itself.
      - A reinstall over a running bridge now replaces it.
- **Later on 2026-10-23,** after the user enabled lingering and set the password:
  - **Helm up:** Helm started and answered from soliloquy (302 to the login).
  - **Camera drops:**
    - Cause: in the camera view over Minerva's 2.4 GHz Wi-Fi, cameras dropped every couple of seconds. The relay needs
      1.47 MB/s; the Wi-Fi gave 0.8–1.2 MB/s, with pauses up to 359 ms.
    - Fix, `f4a7a69`: a slow link now gets fewer frames, not a closed stream. The gate passed: 1296 tests,
      `20261023T205643Z-c9c6f949`.
  - **Teleop headroom** (measured on the Jetson):
    - CPUs 4–5, the loop's, sat at 1–2 %; CPUs 0–3 at 28–35 % (cameras 107 % of a core in all, Helm about 6 %).
    - The relay's SSH transport cost 3 % of a core per 1.5 MB/s, and sshd stays on CPUs 0–3.
  - **Keys, with the user's OK:**
    - The bridge key is authorized as `minerva-gello-bridge:soliloquy`.
    - The terminal's key is removed; dated copies of `authorized_keys` are kept.
  - **Tether:** plugged into soliloquy (Realtek USB GbE "Ethernet 2", 10.42.0.18). Windows' inbox driver has no EEE
    or Green settings; only "allow the computer to turn off this device" remains, still on.
  - **Bridge task:**
    - It had died at 16:34 with 0xC000013A, in the desktop session, and Task Scheduler did not restart it.
    - Since `ec3a5a0` it runs as S4U (session 0, no window) from startup, with a one-minute watchdog trigger.
    - It connected over the tether ("connected to Helm on aizee-rover").
  - **Deployed `ec3a5a0` over the tether** through soliloquy: `20261023T222131Z-ec3a5a0`. The smoke test passed (949
    ms, 1.54 ms p50). Helm restarted, and the bridge reconnected within 1 s.
- **Deployed `863a47b`** (`20261024T013534Z-863a47b`, gate `20261024T012143Z-6726427e`, 1297 tests):
  - **Model flicker:** in a virtual session the 3D model jumped between the session's pose and the resting arms'.
    The loop's status-only views had cleared the session's pose; now they keep it.
  - **A still model, 2026-10-31:** in the user's 12-minute live two-arm session with the head
    (`live-20261031T190356Z`), Helm's 3D view and the headset's model stood still. The arms' tick took 5.6 ms of the
    10 ms cycle (median), leaving 4.4 ms; a full view, the only one carrying the pose, fitted 30 times in 69,212
    cycles of TELEOP, and its admission cost only ever rises. The pose went stale and both views stopped following
    it. Since then the loop also sends the pose alone (`guide` messages: the arms' guide and the neck's angles, about
    0.1 ms to build) about ten times a second in any cycle with 0.5 ms left (`POSE_INTERVAL_S`, `POSE_COST_S`), never
    past a deadline. Full views keep their budget: the Teleop tab's joint rows still update only when one fits.
  - **Wi-Fi state:** the camera stream keeps about one frame queued (64 kB in Helm's buffer, a 32 kB kernel send
    buffer), so on Wi-Fi the page's state no longer waits behind video and the pedals don't lock as stale.
  - **The user's virtual sessions over the tether** (leader-link reports): 20 s and 256 s, 25,657 requests at
    100 Hz with none skipped and one 20–50 ms gap. soliloquy answered within 0.4 ms.
- **Remaining uncertainty:**
  - A real leader link through the bridge: the rehearsal streamed synthetic leaders, but no session's link ran,
    because the laptop is not on the tether.
  - The Jetson's CPU with Helm, the cameras and a recording together. Measure it before a recorded session.
  - Each device accepting the self-signed certificate once (Edge on the 3080, an iPad). Compare its SHA-256 with
    `helm_service status`.
  - Camera relay bandwidth to Wi-Fi devices, and several viewers at once.
  - The 3080's tether adapter: its power saving must be off, as on the terminal (`tether-usb-adapter-latency`).
  - Windows firewall prompts for the bridge's Python on the 3080.
