# Feature task: Teleop and Replay layout, Disable confirmation, recording indicator

- **Branch and worktree:** `claude/teleop-layout` in `C:\Users\ltr\Workspace\aizee-camera-calib`.
- **Base commit:** `b7f76cd` (`claude/helm-on-jetson`, the deployed release `783e450` plus its record).
- **Owner:** Claude (the 2026-10-24 session), at the user's requests while setting up the wrist calibration test:
  - "That area of the teleop with the wrist cal and task setup is very small vertically, hard to see. [...] The test
    pedals button could go into only the settings, plus the Receiver options could be constrained to one line instead
    of wrapping. Also can have it so when the disable button is pressed, it popups with a Are you sure warning"
  - "the background for the cameras there should be fixed, it should be white in day mode and then go dark in night
    mode" and "the camera / model button is over some of the other UI elements"
  - "When a recording is happening [...] I need something I can see from where I am operating."
  - "in the teleop view, have it so the camera views are shown all at once on a left side bar stacked vertically with
    the model preview in the center"
  - Replay: the dark background in light mode, "a preview of the model in the same way [...] The camera views should
    extend into the space the joint position timeline is, but the model view should sit above this joint timeline"
  - "that REC banner should be the same font as the theme"
- **Scope:** Helm's page only (static files), the episode endpoint's model mapping, and the 3D viewer's pose message.
  Nothing changes a receiver, lifecycle or motion rule.

## A hazard fixed: the calibration capture key

`783e450` captured a calibration still on **C**, but C is also pedal C (Next stage): in a session it engages from the
first Hold and shuts down from a later Hold or Teleop, so one press would both capture and move the arm. Capture is
**P** now; the user was told not to use C on `783e450`.

## What changed

- **The Teleop column scrolls as one** (measured at 1920x1080: the task and calibration cards had **18 px**). The pedal
  console stays pinned at the top and the status line at the bottom. While a session runs, the setup that only applies
  at the next start (title, instructions, loop target, receiver choice) is hidden, as the touch layout already did;
  the loop-target row hides whenever it has one choice (Helm on Minerva). The status grid is one row plus Control on
  its own line, and the grip bars sit side by side. During a session the header is 296 px instead of 582 px and the
  cards start with ~360 px visible.
- **Receiver choice on one line:** Virtual · Enforced | Live: Left · Right · Both (titles keep the long form).
- **Test pedals is in Settings → Pedals**, with counts per pedal; the test ends when you leave the page.
- **Disable asks first** while the arms are under torque (a session past Idle, or a sweep): the Teleop and session-bar
  Disable buttons and the H key open "Disable the arms now?"; H again or its button disables, Esc cancels, T or X
  still hold or shut down. Disabled or Idle arms are disabled at once (nothing to drop).
- **Recording indicator:** while recording, a red frame round the whole window (pulsing slowly), a large "● REC mm:ss"
  badge over the view in the look's own font, and the tab title "● REC mm:ss · Helm"; amber while it starts or saves,
  a green "Saved" for 5 s, a red "Recording failed" for 10 s. The time counts from when the page saw it start.
- **Teleop on a wide screen (≥ 1100 px):** the three cameras stacked in a left column, the 3D model beside them and the
  controls on the right; the camera stream and the model run together. Narrower, the Cameras / Model switch remains.
- **The Cameras / Model bar** no longer covers the 3D view's own banner: in Model view it has its own strip.
- **Day and night** (RetroUI): the camera pane and Replay's stage are white by day and dark at night (they were always
  dark); the timeline has a day palette and redraws when the theme changes.
- **Replay:** the episode's cameras stacked in a full-height left column; the 3D model in the centre, above the
  playback controls and the joint timeline, posed at the playhead from the recorded joints (the episode now carries the
  stopped reader's motor → joint mapping, and the viewer takes a pose from Helm by message, holding it over its polled
  guide until Helm releases it when you leave Replay).

## Verification

- Page module tests (node): pedals (tester only on its page; recording modes), replay (pose packet), calibration card,
  capture failure.
- Browser tests (Edge, Playwright): pedals (tester in Settings; H asks, H again disables, X shuts down), live cameras,
  theme and look: all passed.
- Screenshots of both looks, day and night, at 1920x1080 and 1366x768, from a local Helm with a stand-in session and a
  recorded episode (`calibration_card_preview.py` and the `*_shots.mjs` scripts in the session scratchpad).
- The gate before merging.
