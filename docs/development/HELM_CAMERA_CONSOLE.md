# Minerva camera and touch console

Verified 2026-10-16 on the Windows minerva-terminal console, with the Jetson on
its Ethernet link. This is camera/UI qualification, not permission to bypass the
physical-motion acceptance ladder or evidence that the full learning loop works.

## Operator workflow

Open **Minerva - Helm** on the terminal desktop. Enable **Touch mode** in the
header or Settings → Display & touch. The preference is local to that browser.
Open **Settings → Cameras** to read wrist exposure mode, exposure time, gain and
brightness. Each Apply waits for camera readback; no motion command is sent.
Use **Refresh settings & views** to get new snapshots of all three cameras.
Snapshots are labeled with their receive time; this page is not a live video feed.

Runtime changes last until the camera service restarts. The right-wrist startup
preset is manual exposure 150 (15 ms) and gain 30. This improved the dark resting
view, but lighting/framing at the task pose still needs attended inspection.
The head service does not expose exposure adjustment. Its 180-degree raster
correction is applied at the publisher to RGB and native depth, with explicit
orientation metadata; camera extrinsics remain uncalibrated.

Camera writes are rejected during teleop/capture. A write with an uncertain
outcome blocks new sessions and further writes, even after a settings refresh.
Restart the affected camera service, verify its settings, then restart Helm to
recover. This prevents a delayed legacy camera request from changing a recording.

## Installed console

The terminal runs packaged source `ad71afe7c6b2c1afa788e3e55a6ad25caa64a5df`
from `%LOCALAPPDATA%/Minerva/helm-console-ad71afe`, using its own locked Helm
environment. The manually started `Minerva-Helm-Console` Windows task keeps it
alive after setup SSH disconnects. It has no boot/logon trigger. The desktop
shortcut starts that task if needed and opens the loopback-only port 8760 page.
Launch includes `--no-live`: the stopped motor reader does not start automatically.
Teleop remains stopped with its initial virtual receiver selection.

The terminal has its own protected Jetson key. The laptop key was not copied.
After the user's one-time setup, Jetson sudoers permits only these commands
without a password:

```
/usr/bin/systemctl restart aizee-scene-cam.service
/usr/bin/systemctl restart minerva-left-wrist.service
/usr/bin/systemctl restart minerva-right-wrist.service
```

The prior head publisher and camera configs are preserved on the Jetson under
`/home/ltr/minerva/camera-update-20261016T231944Z`. The existing motor release
remains `20261016T181317Z-6b47de8`.

## Verification

- Integrated focused suite: 170 passed (camera API, source orientation, recording,
  shared wire format, SSH boundaries, Helm application and browser modules).
- Touch-only browser tests: 800×480,1024×600,390×844,1280×720, preference persistence,
  blocked storage, and preservation of existing disabled/hidden controls passed.
- Actual terminal Helm server tested through pinned SSH in headless Edge at
  1024×600: all three 640×480 snapshots, exposure menu/range, gain 30 → 31 → 30 with
  confirmed readback, no horizontal overflow, no browser errors, teleop stopped.
- All three real streams delivered frames: left wrist ~56 Hz, right wrist ~52 Hz,
  head ~15 Hz during a six-second sample. These rates are observed delivery rates,
  not synchronized exposure timestamps or sustained recording qualification.
- Head image visually verified upright; all three narrow restart permissions
  verified. No motor commands were issued.

Evidence is in the original workspace's ignored
`build/minerva-terminal-20261016/`: `helm-browser-verification.json`,
`camera-permissions-verified.json`, `camera-deployment.json`,
`robot-ready-20261016T233159Z/`, and the terminal installation reports.
The full control gate/hosted CI was not rerun for this camera/UI package; the
earlier integration candidate's known recording-test expectation failures remain
separate work. Supervised robot motion and a complete block-transfer episode
remain unverified.
