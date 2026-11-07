# Feature task: Teleop and Replay layout, Disable confirmation, recording indicator
- **Base commit:** `b7f76cd` (`claude/helm-on-jetson`, the deployed release `783e450` plus its record).
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
  - Then, a follow-up: "can you tweak both camera views so they are wider? they should be bigger", "remove the pause
    preview button and make the images bigger with that space too" and "the order for the cameras in both teleop and
    replay should be from top to bottom, head right left"
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

## Follow-up: bigger cameras, head first

- **The stacked column is as wide as three 4:3 pictures filling its height** (at most 36 % of the width, so the model
  keeps the rest), and each picture takes its whole tile with its caption over its bottom edge. At 1920x1080 a picture
  is **436x327** (was about 321x241); at 1366x768, 298x223. Replay's tiles: 456x324.
- **Stacked Teleop has no Pause preview button and no heading:** the cameras are part of the view, so there is nothing
  to pause, and each caption says its own camera's state. Retry still appears over the column, top left, once the
  automatic retries run out. Narrower than 1100 px, the Cameras / Model switch and Pause remain.
- **The session-controls bar** starts right of the camera column (Teleop stacked and Replay) instead of covering its
  bottom picture.
- **Camera order, top to bottom: head, right wrist, left wrist**, everywhere in Teleop and Replay: the stacked column,
  the narrow screen's thumbnails, Replay's pictures and its Cameras line, and Teleop's card while an episode replays.

## Verification

- Page module tests (node): pedals (tester only on its page; recording modes), replay (pose packet), calibration card,
  capture failure.
- Browser tests (Edge, Playwright): pedals (tester in Settings; H asks, H again disables, X shuts down), live cameras,
  theme and look: all passed.
- Screenshots of both looks, day and night, at 1920x1080 and 1366x768, from a local Helm with a stand-in session and a
  recorded episode (`calibration_card_preview.py` and the `*_shots.mjs` scripts in the session scratchpad).
- Gate: 1322 tests passed, none skipped.
- The follow-up: the same page and browser tests (the replay test now expects head first); screenshots of both looks
  at both sizes with stand-in camera frames; Retry checked visible and on top over the stacked column.

## Deploy

- **2026-10-25 ~01:47Z, with the user's OK ("deploy when the gate passes"):** Minerva runs `20261025T014629Z-bfc2ae1`
  (`claude/helm-on-jetson` fast-forwarded to this branch: `ee48eb9` the P key, `cd82dad` the screens, `bfc2ae1` this
  record). Helm restarted and listens; no errors since its start; soliloquy's bridge is unchanged.
- **2026-10-25 ~02:37Z, the follow-up, with the user's OK ("no session running, deploy when the gate passes"):**
  Minerva runs `20261025T023633Z-282d261` (`fbebb72` bigger cameras, `6525e6c` head first, `b43f47d` their record,
  `282d261` a test fix). The first gate on them failed one test the change does not touch:
  `test_helm_policy_mode.py::test_a_policy_session_carries_the_policy_and_on_the_robot_the_cameras_not_a_capture`
  raised KeyError: 'policy', a race in the test (its stand-in loop's first view can replace the starting one before it
  reads it; 1 failure in 20 runs under CPU load). `282d261` reads it with `get` (0 in 30), and the gate then passed
  1322 tests. Helm restarted, listens, and its log is clean; soliloquy's bridge is unchanged.
- Not yet seen on the operator's screen, nor Replay with a real recorded episode.
