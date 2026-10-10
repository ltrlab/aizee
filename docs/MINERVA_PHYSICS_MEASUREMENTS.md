# Minerva physical model: measurement and validation checklist

Status: measurements not yet available, confirmed 2026-10-10. The current URDF is
a kinematic/CAD view with 17 placeholder masses. This checklist defines the inputs
for `codex/minerva-physical-model`; it does not authorize powered robot trials.

## Prepare the record

- [ ] Freeze CAD assembly/version, URDF and mesh hashes, robot serial/build, payload,
  tool/gripper configuration and calibration revision. Record all units explicitly.
- [ ] Photograph the measured configuration and identify each rigid link assembly.
  Include motors/gears that move with that link, fasteners, cable carriers, tooling
  and payload. Assign each component to exactly one link; avoid double-counting.
- [ ] Record instrument ID, resolution, zero/tare procedure, repeat measurements,
  uncertainty and date/operator. Preserve raw readings alongside derived values.
- [ ] Use kg, m, s, rad and kg m^2 in exported data. Convert mm dimensions and
  CAD inertia units explicitly. For fixed mass, a length conversion changes inertia
  by the square of the conversion factor; density-based geometric scaling also
  changes mass. Never rescale the mesh while leaving COM/inertia inconsistent.

## Kinematics before dynamics

- [ ] Confirm the root mounting pose and gravity direction on the actual assembly.
  Document right-handed link frames and each joint origin relative to its parent.
- [ ] Check joint-axis sign, encoder direction, offset, transmission ratio and
  units individually. Keep encoder/mechanical coordinates separate from URDF frames.
- [ ] Compare measured end-effector/reference-point positions at home and several
  separated joint poses against forward kinematics. Use independent measurements,
  not a second rendering of the same transforms. Record residuals and tolerances.
- [ ] Measure mechanical travel and distinguish hard stops, software operational
  limits and collision-limited ranges. Record velocity and effort limits from
  actuator evidence, with continuous/peak duration distinctions where relevant.
- [ ] Measure both jaw displacements versus motor angle in both directions over
  their useful stroke. Fit m/rad (or a nonlinear mapping when required), offset,
  backlash and jaw coupling; do not map motor radians directly to slide metres.

## Mass, centre of mass and inertia

- [ ] Weigh each rigid assembly or its separately accounted components, using a
  tared scale and at least three repeats. Compare summed link/payload mass with an
  independent complete-assembly measurement. Explain any missing mass.
- [ ] In CAD, assign actual materials/densities, include the as-built hardware,
  and compare CAD mass with measured mass. Printed parts need measured mass or
  validated effective density; uniform solid plastic density can be misleading.
- [ ] Locate each assembly COM in its **link frame**, from corrected CAD plus
  independent balance/suspension checks where practicable. Record method and
  uncertainty for all three coordinates. A good total weight does not validate COM.
- [ ] Export the full symmetric inertia tensor about the COM in a documented
  orientation. URDF inertial origin is the COM pose relative to the link frame;
  its tensor is expressed in that inertial frame. Rotate tensors with `R I R^T`;
  use the parallel-axis theorem only when changing the reference point.
  [ROS physical properties guide](https://github.com/ros2/ros2_documentation/blob/rolling/source/Capabilities/Simulation/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.rst)
- [ ] Verify exporter product-of-inertia sign conventions against the URDF tensor
  convention. Check positive eigenvalues and principal-moment triangle inequalities.
  Passing those tests establishes physical plausibility, not measurement accuracy.
- [ ] Avoid fitting inertias from render meshes with arbitrary uniform density.
  Include distributed and concentrated components separately when CAD lacks them.

## Contact geometry and actuator behavior

- [ ] Retain detailed visual meshes, but build a reviewed contact model with
  primitives or convex parts. Compare coverage at grippers, wrists, cables and
  self-contact regions. The current keep-out finger exclusions do not automatically
  apply to physics contacts. Test the intended collision pairs explicitly.
- [ ] Measure friction/backlash and damping using a reviewed protocol and repeated
  traces in both directions. Keep stiction, viscous loss and contact friction distinct.
- [ ] Identify torque/current scale, effort saturation, response delay, rate limits,
  feedback quantization/noise and dropped-frame behavior. Keep motor internal MIT
  behavior distinct from the host's reference interpolation and receiver watchdog.
- [ ] Represent reflected rotor/gear inertia and transmissions explicitly. Avoid
  counting gear/motor effects both in a fitted link inertia and in armature terms.
- [ ] For mimic jaws or other coupling, verify the MJCF constraint/transmission
  against measured motion and force directions, including at the limits.

## Acceptance evidence, one arm first

- [ ] Run the URDF audit, review the exported model visually and compare frame
  transforms numerically. Do not enable mass/inertia auto-repair to pass validation.
- [ ] Compare predicted static holding torques with independent measured poses
  across the range; fit on one subset and evaluate on a held-out subset.
- [ ] Compare transient trajectories, steady errors and stopping behavior across
  payloads and directions using recorded inputs. Set acceptable residuals **before**
  fitting. Preserve sensor noise and timing distributions rather than smoothing
  away the failure being investigated.
- [ ] Repeat at smaller physics timesteps to check numerical convergence. Report
  model error separately from discretization and host scheduling error.
- [ ] Record parameter uncertainty and test plausible bounds. Do not tune until
  one nominal trace matches while ignoring the remaining poses or fault scenarios.
- [ ] Publish the source/model/parameter identity, raw measurements, fit/holdout
  split, residuals and unsupported behaviors. Only then extend to two arms/contact.

Each link record should include `link`, `assembly_revision`, `mass_kg`,
`mass_uncertainty_kg`, `com_xyz_m`, `com_uncertainty_m`, `inertial_rpy_rad`,
`inertia_kg_m2` (full tensor), method, instrument, operator/date and raw evidence
paths. Unknown values remain explicitly unknown; zero and exporter defaults are
not substitutes for measurements.

MuJoCo describes model frames and inertials in its
[modeling guide](https://mujoco.readthedocs.io/en/stable/modeling.html) and
[inertial reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-inertial).
These checklists apply that engine contract to Minerva; the measurement tolerances
must be set from the robot's intended tasks and available instruments.
