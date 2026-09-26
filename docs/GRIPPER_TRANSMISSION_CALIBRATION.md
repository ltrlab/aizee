# Gripper transmission calibration

**Reference changed on 2026-09-24:** both grippers were hardware-zeroed with the
inner jaw faces fully closed, and all motor zeros passed one power cycle. The
earlier Minerva fit's absolute encoder offset is invalid in this new reference.
Keep its raw measurements as historical evidence; do not install the old fit
for live control. See [mechanical reference](MECHANICAL_REFERENCE.md).

The shared offline tool pairs a stopped motor encoder reading with a measured
**total gap between the inner jaw faces**. It fits
`aperture_m = scale_m_per_rad * motor_position_rad + offset_m`.
Positive and negative transmission signs are both supported; there are no
Minerva motor names, pinion dimensions or endpoints in the algorithm.

## Collect paired measurements

1. Use the [stopped diagnostic procedure](POWERED_COMMISSIONING.md#repeatable-stopped-diagnostics).
   Confirm the intended motor, disabled drive, supported mechanism and clear gap.
2. If the mechanism can be moved gently by hand, approach several comfortable
   openings from the closing direction, then from the opening direction. Do not
   force a resistant transmission. A powered alternative needs its own bounded
   motion trial; this tool sends no commands.
3. Use the same inner jaw surfaces each time. Measure without squeezing the jaws,
   leave them stationary, and capture fresh diagnostics before changing position.
   Record the measured value, instrument, uncertainty and approach direction.
4. Keep at least three distinct encoder positions in each direction. Default
   criteria require a 10 mm measured span in each direction, a 0.1 rad encoder
   span, and at least 10 mm of shared fitted span. Overlap the two sets generously.
   The sampled range is not a measurement of the mechanical end stops.

The direction refers to increasing/decreasing **aperture**, independently of the
motor's sign. An initial reading with an unknown approach can be preserved, but
does not count toward either directional set. A stopped status does not prove a
stationary measurement: `--pairing-confirmed` records that the jaw measurement
and encoder capture describe the same unchanged pose.

## Record and fit offline

Example for an existing diagnostic envelope and an operator measurement:

```powershell
./.venv-platform/Scripts/python.exe -m robot_platform.control.transmission_calibration sample `
  --diagnostics stopped-40mm.json --motor right_gripper --aperture-mm 40 `
  --approach closing --instrument calipers --uncertainty-mm 0.25 `
  --pairing-confirmed --output closing-40mm.sample.json
```

`--uncertainty-mm` describes the whole manual measurement, including placement
and repeatability; it is not simply the instrument's display resolution. The
example 0.25 mm is a provisional allowance, not a verified caliper specification.
Omitting instrument/uncertainty preserves a pending sample and blocks a candidate.
Plain diagnostic-client JSON also needs `--captured-at` with a timezone-aware
capture timestamp; the remote diagnostic envelope already supplies one.

```powershell
$samples = (Get-ChildItem -LiteralPath simulation_runs/gripper-calibration -Filter *.sample.json).FullName
./.venv-platform/Scripts/python.exe -m robot_platform.control.transmission_calibration fit `
  @samples --output simulation_runs/gripper-calibration/fit.json
```

The fit command returns exit code 1 with a written report if there are missing
measurements or failed criteria. Output files are never overwritten. It checks:

- Complete, fault-free Reset diagnostics, no enable or parameter-write requests,
  and reported motor speed at most 0.1 rad/s at capture.
- One motor route/MCU UID/firmware across unique diagnostic captures. Each sample
  retains selected-motor raw replies, source SHA-256 and capture time. The hash
  identifies source bytes; it is not a hardware signature.
- Measurement uncertainty at most half the fit tolerance and one tenth of the
  minimum span; explicit instrument and stationary pairing.
- Common-line residuals and leave-one-out prediction error, plus disagreement
  between separate opening/closing fits. The default maximum error is 1 mm.
- Consistent transmission sign and overlapping ranges. The candidate's evaluation
  function refuses extrapolation beyond the shared motor-angle interval.

The span/tolerance options describe measurement quality, not motor operating
limits. Inspect residuals and repeat at new intermediate gaps before runtime use.
Directional disagreement can include backlash, hand-loading, compliance and
measurement error; these tests do not distinguish their causes automatically.

## What a passing candidate establishes

It estimates **unloaded, stopped total aperture** within the sampled overlap.
It does not install settings or approve powered motion. The intercept is an
affine fit coefficient; it does not establish an accessible zero-gap pose.
Encoder zeroing, transmission maintenance or changed jaw geometry invalidate
the mapping and require new measurements.

For symmetric opposed jaws, a total aperture change corresponds to half that
change per jaw. Mapping to a URDF slider still needs the measured reference pose
and the CAD axis sign/zero. Jaw force also requires a verified torque model and
friction/load measurements. Preserve these distinctions when adding the candidate
to the shared robot configuration.

Minerva's measurement evidence and current commissioning state are recorded in
the [production device log](devices/MINERVA_PRODUCTION_01.md).
