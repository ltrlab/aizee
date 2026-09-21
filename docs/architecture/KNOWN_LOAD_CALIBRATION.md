# Calibration with known weights

Proposed 2026-09-20. This extends the Minerva hardware plan with a static load
reference using the user's calibration weights. No powered routine is enabled
by this document. Weight values, accuracy and attachment geometry are pending.

## Principle

Repeat measurements with and without an added mass at the same measured robot
configuration. The difference largely cancels the arm's own gravitational load,
fixed fixture weight and constant reporting bias. It does not cancel changes
in friction, deflection, temperature, pose or contact with external supports.

For a revolute joint, the added gravity moment about its axis is:

```
delta_tau_gravity_j = axis_j dot ((payload_com - joint_origin_j) cross (delta_mass * gravity))
```

All vectors are in the same frame. The added holding torque opposes this moment;
its reported sign depends on the verified actuator-to-joint mapping. For a
simple perpendicular lever, the magnitude is `delta_mass * g * lever_length`.
The full chain calculation is the Jacobian-transpose wrench relationship from
[Modern Robotics, section 5.2](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-2-statics-of-open-chains/).

With multiple added masses or different locations, subtract their complete
gravity moments rather than assuming one unchanged center of mass. Include the
holder in the baseline. A rigid fixture has a known payload center of mass; a
stationary freely suspended mass applies force at its suspension point. Model
the actual attachment instead of applying the rigid-fixture assumption to both.

This first comparison needs accurate joint axes, coordinates, gravity direction
and attachment geometry, but does not need individual arm-link masses. Diverse
poses are essential: a joint axis parallel to gravity has no gravity moment to
identify. Off-axis fixtures help some wrists; a permanently vertical axis needs
a separately measured nonvertical force or other suitable test arrangement.

Commercial payload wizards also use multiple sufficiently different poses and
require correct mounting and no external pulling; this is a relevant procedure
reference, not evidence that Minerva has equivalent sensors or accuracy.
[Universal Robots payload estimation](https://www.universal-robots.com/manuals/EN/HTML/SW10_10/Content/prod-usr-man/software/PolyScopeX/polyx-application/polyx-TCP_Payload_CenterofGravity.htm).

## Initial experiment

1. Record weight values and uncertainty, fixture geometry/mass, payload position,
   base orientation, robot/model revision, device firmware, raw torque encoding,
   joint calibration, gains, temperature and supply voltage. Verify coordinates
   and the supported stop procedure before the robot moves to any test pose.
2. Choose collision-free poses whose predicted load moments span useful values
   for the target joints, inside the reviewed load/travel envelope. Evaluate all
   upstream joints as well as the wrist. Choose exact masses and poses after
   inventory and installed-device checks; no universal payload is assumed.
3. Keep one arm parked. At each selected pose, collect baseline, several known
   loads and a repeated baseline. Keep fixture and controller settings fixed.
   Secure and change weights with the robot supported, then remove unintended
   support forces before measuring; any catcher must be clear during the hold.
4. Approach each pose slowly from both directions and repeat trials. Detect
   settled windows using fresh position/velocity feedback rather than only a
   fixed sleep. Store raw CAN, reported effort/current if available, measured
   positions/velocities, command terms, sample ages and faults. Use measured
   configurations, rejecting or accounting for load-induced pose differences.
5. Compare per-joint changes in reported effort with the calculated load
   moments. Fit effective scale/sign and residuals; report directional spread,
   drift, nonlinearity and uncertainty. Static friction can change with load,
   so a slope alone is not an isolated motor torque constant or gearbox
   efficiency measurement. Paired subtraction also cannot identify an absolute
   sensor offset independently of the arm's gravity and static friction.
6. Reserve complete poses and at least one load level for validation when the
   available weights permit. Keep whole repetitions separate between fitting
   and validation; correlated samples from the same hold are not independent
   trials. Repeat after warm-up and on the other arm independently.

Known loads can help discriminate candidate torque decoders from raw frames;
they do not authorize changing a live codec or its trip limits without the
installed-firmware and controller checks in the hardware plan.

## What the evidence can establish

| Measurement | Supported conclusion |
|---|---|
| Repeatable load increments across poses/masses | Effective static reported-effort calibration and load sensitivity within the tested envelope |
| Opposite approaches and repeated baselines | Bounds on static hysteresis, drift and repeatability; not complete friction separation |
| Known fixture moment with stated uncertainty | Independent static external-load reference; not dynamic output-torque validation |
| Calibrated readings over additional unloaded poses | Gravity compensation from identifiable parameter combinations; not necessarily unique masses/centers for every link |
| Later bounded motion with known loads | Separate response, inertia/friction and stopping-distance experiments |

Static tests do not determine response delay, braking, full inertial tensors,
joint hard stops, fingertip gripping force or every joint zero. Do not promote
unobservable parameters. Assign/measure CAD mass properties where possible and
use later dynamic tests for parameters absent from static equilibrium.

The software deliverable should be a robot-independent known-load trial format,
offline pose/moment planner, recorder, fitter and held-out validation report.
Keep proposed calibration artifacts separate from active control settings.
Only then add supervised pose execution through the shared command authority.
This work supplies calibration evidence for the
[25 Hz hardware plan](COLLISION_25HZ_HARDWARE_PLAN.md); it does not replace the
receiver lease, contact-policy or stop-response work.
