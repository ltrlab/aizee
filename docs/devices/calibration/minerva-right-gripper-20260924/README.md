# Right-gripper caliper measurements, 2026-09-24

Device: `minerva-production-01` / `aizee-rover`; right gripper on `canR`, ID 10,
MCU UID `632f93c79c90b00e`, firmware `0.0.3.32`.

The operator manually repositioned the disabled gripper's jaws and reported caliper
measurements. Each report was followed by stopped diagnostics before requesting
the next move. No enable or parameter-write commands were sent. Capture times
identify the subsequent encoder capture, not the exact manual measurement time.

| Approach | Total inner gap (mm) | Encoder (rad) | Capture UTC |
|---|---:|---:|---|
| Unknown, initial pose | 48 | 2.466986418 | 20:18:38 |
| Closing | 40 | 2.020061016 | 20:22:16 |
| Closing | 30 | 1.441942215 | 20:24:18 |
| Closing | 20 | 0.944625378 | 20:26:11 |
| Opening | 25 | 1.282714844 | 20:30:34 |
| Opening | 35 | 1.738767624 | 20:31:33 |
| Opening, instructed to release before capture | 45 | 2.360068083 | 20:32:49 |

The 0.25 mm uncertainty allowance in the samples is **provisional** manual
placement uncertainty, not a verified instrument specification. The operator
identified calipers but did not provide model, resolution or accuracy.

## First fit: rejected

[`fit.json`](fit.json) preserves the original seven-point analysis:

- Common slope: 18.2528 mm/rad; offset: 2.7585 mm.
- Opening slope: 18.4196 mm/rad; closing slope: 18.5622 mm/rad.
- Maximum residual: 1.1716 mm; maximum leave-one-out error: 1.5765 mm.
- Maximum separation between directional fits over their overlap: 1.0645 mm.

The fit exceeds the unchanged 1 mm criteria and produces **no calibration
candidate**. Directional separation is not independently measured gear backlash;
hand loading, relaxation and caliper placement can also contribute. Preserve
these readings and repeat measurements after release before changing the model.

The original full 14-motor diagnostics, source hashes, operator measurement
records and device final state are in
[`MINERVA_GRIPPER_TRANSMISSION_20260924.json`](../../MINERVA_GRIPPER_TRANSMISSION_20260924.json).
Selected-motor raw replies are also embedded in each `.sample.json` file. Fit
commands and interpretation are described in the
[shared calibration guide](../../../GRIPPER_TRANSMISSION_CALIBRATION.md).

## Approximate conversion using the existing measurements

The user remeasured the released final gap as **45.5 mm**, then confirmed that
finger flex and caliper variation contribute to the scatter and requested using
the existing measurements without further collection. At 20:35:02 UTC the encoder
read 2.360221386 rad, only 0.000153303 rad from the nominal 45 mm capture. That
small encoder difference does not explain the whole 0.5 mm reported gap change;
the records do not separate placement error from finger flex or relaxation.

[`coarse/fit.json`](coarse/fit.json) is a separate approximate assessment. It uses
the more precise 45.5 mm remeasurement instead of the nominal 45 mm entry, retaining
the other six points. Original samples and the failed fit are unchanged. Revised
sample copies explicitly allow +/-1 mm for manual placement/flex, with a **2 mm
fit-error criterion**. These are provisional engineering allowances, not verified
measurement uncertainty or motor-control limits.

```text
total aperture (mm) ~= 18.4104 * motor position (rad) + 2.5535
```

- Maximum residual: **1.1688 mm**; leave-one-out error: **1.5728 mm**.
- Directional separation: **1.0396 mm** over the sampled overlap.
- Interpolation interval: **1.282715 to 2.020061 rad**, approximately **26.17 to
  39.74 mm**. The narrower range preserves coverage from both approach directions.
- `runtime_approved` remains false. This estimates unloaded aperture for planning;
  no force/contact accuracy, CAD zero or mechanical end stops are established.

The sample selection, reassessment, additional raw captures and final device state
are preserved in
[`MINERVA_GRIPPER_COARSE_20260924.json`](../../MINERVA_GRIPPER_COARSE_20260924.json).
At 20:37:57 UTC all 14 motors remained Reset and fault-free, with both CAN links
down and unchanged motor identities, firmware and settings. The final encoder
position was 2.360260 rad; it is above the candidate's shared interpolation range.
No motor movement was commanded, no motion limits changed, and no live
calibration was installed.
