# Mechanical zero and startup reference

ROBSTRIDE mechanical zero defines the current physical position as the motor's
coordinate origin. It must be an explicit maintenance action with the drive
disabled and the robot placed at a documented reference pose. It is not a move
to zero and is not magnetic encoder calibration.

`robot_platform.control.mechanical_zero` verifies the complete pinned motor
UID/firmware roster, fault-free Reset replies, operation-control mode, and
stationary position before any zero write. It rechecks each pose, sends type 6
with payload `01 00 00 00 00 00 00 00`, and verifies raw position near zero.
Optional `--save` sends the current type-22 payload `01 02 03 04 05 06 07 08`;
this saves motor parameters, not only an application-side offset. There is no
enable, motion, fault clear or arbitrary parameter write. Zero/save writes are
never retried. An ambiguous zero result stops further writes. A save timeout is
recorded as unacknowledged, followed by fresh stopped position checks; it never
counts as persistence evidence. All roster motors receive a final stop attempt.
Evidence output must be a new file. Write intent and responses are journaled
before proceeding, so partial actions can be inspected without repeating writes.

Maintenance transactions are spaced by at least 5 ms. A stopped diagnostic may
be repeated once after a timeout; zero/save cannot. The position/velocity quiet
window must contain six consecutive acceptable samples; outliers reset the
window without widening its limits. The observation count is bounded.

Some installed firmware returns a matching UID after save, while other versions
give no response. The parser records a pinned UID response separately from a
type-2 Reset reply and then obtains fresh stopped position readings. Neither
response form proves persistence. Explicit `--resume-evidence` rechecks the
entire original identity/reference roster and skips all previous zero/save
writes. An unacknowledged zero cannot be resumed automatically; an attempted
save is skipped and remains pending the power-cycle test.

The caller must use the identified host/CAN adapters, keep other controllers
stopped, and restore physical CAN links down after the finite action. The CLI
also checks machine/module/USB identities, conflicting service PIDs and receiver
port, takes a process lock, and requires `--confirm-reference-pose`.

```sh
python -m robot_platform.control.mechanical_zero \
  --device config/devices/minerva-production-01.json \
  --roster /path/to/reviewed-reference-roster.json \
  --output /path/to/new-evidence.json --confirm-reference-pose --save
```

Do not turn this command into a startup script. After a successful write, keep
the robot at the same pose, power-cycle it and obtain fresh stopped readings to
verify persistence. A save reply alone does not prove persistence. Derive a new
calibration revision and invalidate old pose targets and offsets after zeroing.
The current startup reference gate requires independent pose confirmation and
fresh encoder agreement before physical enable. Its proof cannot be reused
after disable, faults or a new process. See [guarded teleoperation](../architecture/GUARDED_TELEOP.md).

## Minerva's reference revisions

Use the [live calibration session](JOINT_CALIBRATION.md) (or Helm's Limits tab) to check directions by moving disabled joints and
watching the CAD model follow. The reference pose is both arms matched to the viewer's six-joint zero pose; **gripper zero means
the inner jaw faces fully closed**. The records below are the revisions this unit has had; the evidence is in
[`docs/devices/`](../devices/).

| Revision | What was zeroed | Result |
|---|---|---|
| `minerva-reference-20260924` ([record](../../config/devices/minerva-reference-20260924.json), [summary](../devices/MINERVA_MECHANICAL_ZERO_20260924.json)) | All 14 motors: one zero command and one save attempt each, no enable or control frames | Readback within 0.000297 rad of zero, which is encoder precision at a manually aligned pose, not CAD calibration accuracy. Save replies were missing for left j3/j4/j5/j6/gripper; the operator's power cycle then passed: all 14 within 0.000297 rad with unchanged identities |
| `minerva-mechanical-reference-20261009` ([record](../../config/devices/minerva-reference-20261009.json), [summary](../devices/MINERVA_MECHANICAL_ZERO_20261009.json)) | Left J4 and J5 only, after a wrist harness rework | Both read within 0.0003 rad of zero after a power cycle. The other twelve keep the 2026-09-24 zero. The revision names the earlier one in `direction_evidence_from`, because direction checks are signs, which a re-zero does not move; limits and servo calibration were re-bound unchanged |
| `minerva-mechanical-reference-20261101T192305Z` ([summary](../devices/MINERVA_MECHANICAL_ZERO_20261101.json)) | Right J4 and J5, through Helm's Joint zero, after the right wrist structure was remounted in a different position | Verify read them at +0.00005 and -0.00010 rad after the power cycle |

Some firmware returns a matching UID after a save and some gives no response; neither form proves persistence, which only the
power-cycle readback does. Direction evidence is recorded separately in
[arm direction evidence](../devices/MINERVA_ARM_DIRECTIONS_20260924.json): the sign converts a raw motor-angle change to the CAD
joint's direction. Left J1 is +1 and right J1 -1 (the right CAD axis points down at the reference pose, so positive CAD J1 turns
clockwise viewed from above). Those observations establish signs, not transmission ratios or alignment accuracy.

## Joint zero in Helm

Later re-zeroes of arm joints run through Helm's **Joint zero** (Limits tab;
[Helm](HELM_MANUAL.md#limits)), so the operator can redo them without a release:
the same stopped maintenance (`ops.probe zero_joints`) at the operator-confirmed
zero pose, then a power cycle and Helm's Verify, which needs every zeroed joint
within 0.01 rad of zero. The revision is not committed: Helm on Minerva keeps it
in its operator files as `<asset>-arms-reference.json`, which every session,
the loop and Minerva Check load in place of the profile's reference. Each
revision names the one it supersedes, keeps its direction evidence, records the
zero and power-cycle readings, and clears the zeroed joints' saved limits for a
re-sweep; the direction session and manual tuning presets move to it. A release
that brings a different committed reference replaces Helm's copy, the replaced
copy kept aside (helm.operator_files).

Its first use, on 2026-11-01, is the third revision in the table above.

## Why an apparent zero may change

A reading one whole turn off has a known cause on Minerva: the drives' power-on
position range (`zero_sta`, 0x7029). At the factory setting 0 a drive reports
its power-up angle within 0…2π, so a joint resting a little below its zero at
power-up reads a turn high. All 14 drives read 0 until 2026-09-30, when
`robot_platform.control.power_on_range` set them to 1 (−π…π, centred on the
zero) and saved it. That write changes no zero; see the
[device log](../devices/MINERVA_PRODUCTION_01.md) and its
[evidence](../devices/MINERVA_POWER_ON_RANGE_20260930.json). Minerva Check reads
every drive's `zero_sta` in its stopped scan and names a reading a whole turn off
as such.

Any other apparent change has no established cause. Manufacturer documentation
also describes position bias (`add_offset`) and firmware-dependent zero behavior. A power-on turn representation change must not
be silently treated as an unchanged raw encoder reference. Manufacturer release
notes also require re-marking RS03/RS04 mechanical zero after certain upgrades.
Record firmware and power-cycle observations before blaming a particular cause.

An identified legacy bug sent all-zero data for type 22; current documentation
requires the eight-byte sequence above. The builder is corrected, but the legacy
controller remains disabled and is not the commissioning path. We have not
established which firmware accepted or ignored the previous form.

Sources: [ROBSTRIDE RS00 manual, sections 4.1.5/4.1.9/4.2](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS00/RS00User%20Manual260713.pdf)
and [V25.02.27 firmware notes](https://github.com/RobStride/Product_Information/releases/tag/V25.02.27).
