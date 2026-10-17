# Helm pedal controls

The operator's three-key USB pedal sends A (left), B (middle) and C (right).
These controls use the existing teleop lifecycle and recording worker. They do
not start a session, change motion limits, or provide an independent hardware
emergency stop. Physical operation still requires the existing robot checks.

## Operator sequence

Open **Teleop** and start the session on the touch screen. Wait for the receiver
to be ready before pressing C. Release the pedal between presses.

| Pedal | Behavior |
|---|---|
| C, first press | Idle: the existing powered, zero-torque state |
| C, second press | Enable: hold the measured pose |
| C, third press | Engage: align to the leaders, then follow in Teleop |
| C while following or pedal-paused | Controlled shutdown to the saved reference, then disable |
| A | Start an episode; press again to stop and finalize it |
| B while engaging or following | Pause in Hold; motors remain enabled |
| B while pedal-paused | Align to the leaders again, then resume Teleop |

C cannot skip a transition while engagement or shutdown is in progress. B does
not resume a fault or a hold caused by lost leader input. Follow the displayed
next action and actual state; a request is not proof that a transition completed.
Recording needs the task set up, a healthy capture worker and fresh required
cameras. Stop recording with A and wait for **Complete** before shutdown to save
a completed episode. An interrupted recording follows the existing partial-file
rules. Stopping recording does not stop the arms. Pausing the arms does not stop
an active recording; its held segment retains the reported lifecycle state.

The existing H/Disable and X/Shutdown controls remain available. Disable removes
torque where the arms are; shutdown returns toward the saved reference. The
table and bucket are not part of the current inter-arm collision model, so the
whole movement and return path must be clear. A keyboard pedal depends on the
focused Helm page and its connection; it is not a hardware stop circuit.

## Touch display and pedal check

The status bar reports the confirmed lifecycle and the next available pedal
action. In touch mode, a border surrounds the whole interface:

| State | Border |
|---|---|
| Off or Idle | None |
| Enabled/Hold, including pedal pause | Blue |
| Engaging | Flashing green |
| Teleop | Solid green |
| Shutdown | Flashing yellow |
| Fault or reset-required error | Red |

Text remains visible alongside color. Reduced-motion preferences suppress the
flashing. Lost connection must not leave an apparently healthy green indication.

With the session stopped, **Test pedals** provides a local input check. Press A,
B and C and verify the indicated key. This mode sends no robot commands and ends
when session startup begins. Pedal commands are scoped to Teleop and ignored
while entering text; browser shortcuts, key repeat and held-key duplicates do
not advance the sequence.

## Validation status

Focused backend validation passed 80 tests, including full pedal sequences with
the actual fourteen-axis native virtual receiver in direct and enforced-motion
modes. Policy, HTTP and command-ordering checks also cover expired offers,
duplicate presses, recording acknowledgements and H/X precedence over concurrent
pedal, Stop/EOF and pending leader-zero requests. Motion and recording offers use
separate revisions; an undispatched recording edge cannot delay a pause or
shutdown edge. These are virtual receiver checks, not physical operation.

The real Chromium fixture passed at 800x480 and 1024x600 using the shipped Helm
modules and a mock API. It checks the stopped pedal tester, actual-state display,
late/stale responses, all border states, reduced motion, and the existing live
Start/Stop confirmations. Evidence is in `build/pedal-ui/`; backend focused
reports are in `build/pedals-backend/`. Required regressions are part of the full
offline gate. Full release acceptance and deployment are pending at this source
checkpoint. The prior recording release and its completed/interrupted camera
proof are documented in [recording readiness](HELM_RECORDING_READINESS.md).
