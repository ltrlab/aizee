# Dual GELLO setup

The GELLO leaders (two OpenRB boards, seven encoder channels each) stay on the operator station. In production they
reach the robot through the **GELLO bridge** (`teleop/gello_bridge.py`), which Helm on the Jetson drives: the bridge runs
the 200 Hz USB sampler and the signed UDP leader link toward the robot's loop, and streams samples to Helm's Leaders
tab. See [HELM_ON_JETSON.md](HELM_ON_JETSON.md) and [TELEOP_LATENCY.md](TELEOP_LATENCY.md). This page covers the
stand-alone viewer used to check the leaders and the offline rehearsal tools.

The shared leader viewer reads both OpenRB boards and shows their seven encoder
channels, a six-joint arm schematic, and a gripper trigger. The USB serials,
joint axes, link lengths, labels, and display calibration are profile data.
There are no follower motor commands in this viewer.

## Start the viewer

From the repository root on the operator PC:

```powershell
.\scripts\view_leaders.ps1
```

Open <http://127.0.0.1:8767/>. The stopped Minerva viewer at port 8766 can remain
open independently. Close other OpenRB monitors before starting: a single
process owns the two serial ports.

The default requests **100 Hz paired USB sampling**, with the boards read in
parallel, and targets roughly **60 Hz browser updates**. This is independent of
the follower's initial **25 Hz collision/control loop**. Fresh source samples
still have a 50 ms age limit. `-Rate 50` selects a lower input rate if needed;
raising a rate never changes the freshness limit.

A 10-second parallel read returned 996 pairs with no protocol errors, maximum
source age 15.23 ms and maximum left/right skew 1.93 ms. Four cycles missed their
10 ms completion schedule. This is a short measured result, not deterministic
100 Hz qualification. The former sequential reader achieved about 46 pairs/s
when asked for 100 Hz; concurrent reads removed that bottleneck. See the
[input evidence](../devices/MINERVA_GELLO_VIEWER_20260926.json).

Equivalent shared command for another machine or leader profile:

```sh
python -m pip install -e ".[leaders]"
python -m robot_platform leaders --profile config/devices/gello-dual-viewer.json --calibration calibration_runs/gello-dual/display.json --port 8767
```

Stop the viewer with Ctrl+C in its terminal. A USB/protocol failure requires an
explicit restart; it does not reconnect in the background. A temporarily stale
cached reading freezes the read-only display until fresh source data arrives.
Teleop consumers still stop on stale input and require explicit new registration.

## Verify positions and directions

1. Compare each leader's numbered joints with the diagram. The seven channels
   are the six arm encoders followed by the gripper trigger.
2. Turn on **Reference guide**, put the physical leader in the hanging shape
   shown, then capture its display reference and turn the guide off. This sets
   software offsets only; live readings continue in the table while the guide
   shows its neutral pose.
3. Move one joint and check its displayed direction. Use that row's **Flip**
   control if necessary. Raw ticks and raw angles stay unchanged.
4. Save the display calibration. It is bound to this exact profile and both USB
   identities. Keep the saved file in `calibration_runs/`; it is local setup data.

The dual-leader profile now follows the photographed overhead mounting: hanging
upper arms and forearms, inward elbow offsets, outward wrist brackets, and the
Minerva-style sequence of rotary axes. Its dimensions are illustrative. The
operator-confirmed hanging pose was captured as a display reference on September
26 UTC; encoder directions still need movement verification. It shows encoder
movement and relative direction, not measured end-effector location. Change the
profile axes/dimensions when the exact leader design is available.
Saving a display calibration does not approve a motor mapping, change
ROBSTRIDE zeros, or enable a follower. Angles are not wrapped or clamped: an
unexpected encoder turn remains visible.

The generic renderer supports per-leader `base_rpy` and `mount`, per-joint
`origin_rpy` and `link_xyz`, and optional world-space `fixture.rods`. Rotations use
URDF roll/pitch/yaw convention. A joint's `offset_xyz` is relative to the previous
link's end; `link_xyz` gives its displacement after joint rotation. Older profiles
without `link_xyz` retain their `[0, 0, length_m]` behavior. The mounting layout
and reference description belong to the profile, not the renderer.

The profile's `front_direction` controls the default, **Fit**, **Front**, and
**Top** camera orientation. The hanging setup now faces from `[0, 1, 0]`, a
180-degree turn around the vertical axis from the earlier view. This is a camera
change: joint axes, USB sides, and all saved display references remain unchanged.
The saved calibration's profile hash was migrated after backing up both original
files; no reference was recaptured. See the
[camera migration evidence](../devices/MINERVA_GELLO_CAMERA_20260926.json).

## Collision-checked virtual teleop

The viewer's input snapshot endpoint shares its existing USB reader. A separate
test consumes those samples through the same sequence/epoch/age checks used by
the leader bridge, then runs relative registration, target slew limits, the
Minerva keep-out compiler, and the native virtual receiver.

```powershell
.venv-dev\Scripts\python.exe scripts/benchmark_leader_teleop.py --model urdf/minerva/model.json --routes config/devices/minerva-gello-shadow.json --leader-url http://127.0.0.1:8767 --output simulation_runs/gello-shadow-new.json --cycles 250
```

This is a short software test with ideal virtual motors. Its illustrative
channel-to-CAD routes are separate from display offsets and are explicitly
`simulation_only`. The test has no physical CAN backend. It does not qualify
network transport, loaded stopping, or actual motor gains. Stop on a timing or
collision rejection and inspect the report; do not widen deadlines to force a pass.

The first actual dual-leader run accepted **250/250 cycles** at 25 Hz: median
6.90 ms, p99 10.99 ms, maximum 13.45 ms, with no cycle or scheduled deadline
misses. The virtual receiver's silent-producer collision expiry stop passed.
The leaders were effectively stationary (0–1 tick variation), so this verifies
input integration and stationary collision timing, not a moving-arm trial.
Maximum source age was 42.66 ms at intake and 48.39 ms after the complete cycle.

## From the viewer to the robot

Done: the leaders are routed to the motors through the verified motor-to-CAD map; the Idle, Enable, Engage/Align, Hold,
Shutdown lifecycle (keys I, E, T, H, X) runs live on both arms through the guarded receiver (see
[GUARDED_TELEOP.md](GUARDED_TELEOP.md)); and the SSH input experiment that exceeded its 25 ms deadline was replaced by
the UDP leader link plus the bridge, so the PC-to-Jetson hop no longer rides an SSH tunnel. Every alignment and
shutdown step uses fresh measured feedback, and a fault stops the session rather than launching an unchecked zero
return.

Minerva's collision policy in the guarded profile covers coarse **inter-arm** keep-out, with finger links excluded;
it does not establish same-arm, environment, payload or finger protection, and the production direct profile runs with
collision checking off ([COLLISION_RUNTIME_IMPLEMENTATION.md](COLLISION_RUNTIME_IMPLEMENTATION.md)).

## Other inputs

The Quest Pro headset is built: it steers the head today through Helm's `/xr` page (`helm/xr.py`, `helm/head_teleop.py`,
`helm/static/xr*.js`), and its controllers can drive the arms by grab and drag (`teleop/vr_arms.py`). Both are
described in [HEADSET_3D_PLAN.md](HEADSET_3D_PLAN.md) and [VR_ARM_TELEOP_PLAN.md](VR_ARM_TELEOP_PLAN.md), with their
status. The legacy WebXR launcher under `python/web/` is not a shortcut around the guarded receiver.
