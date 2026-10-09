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
after disable, faults or a new process. See [guarded teleoperation](GUARDED_TELEOP.md).

## Minerva reference, 2026-09-24

Use the [live calibration session](JOINT_CALIBRATION.md) to check directions by
moving disabled joints and watching the CAD model follow. The guided local
Enter-to-capture mode remains available; chat is not required between joints.

The operator matched both arms to the viewer's six-joint zero pose and confirmed
that gripper zero means **inner jaw faces fully closed**. All 14 motors received
exactly one mechanical-zero command and one save attempt, with no enable/control
frames. Final repeated readback was within 0.000297 rad of zero. This is encoder
precision at a manually aligned pose, not CAD calibration accuracy.

Save responses were missing for left j3/j4/j5/j6/gripper. The other motors
returned their pinned UID. All final stops were fault-free Reset; physical CAN
links returned down. The subsequent operator-reported power cycle passed:
all 14 raw positions remained within 0.000297 rad of zero, with unchanged
UID/firmware and fault-free Reset. This verifies one power cycle; future startup
reference checks remain mandatory. See
[reference record](../config/devices/minerva-reference-20260924.json) and
[full action summary](devices/MINERVA_MECHANICAL_ZERO_20260924.json).

## Left J4 and J5 re-zeroed, 2026-10-09

The left wrist harness rework of 2026-10-08 found oxidation or ash inside an
XT30 connector, which explains left J5's repeated overcurrent trips under J3
motion. The operator replaced the connector with wires soldered to the board
and remounted left J4 and J5. At the zero pose they then read −0.620 and
−1.654 rad. With the operator confirming both at the zero pose, the stopped
maintenance session zeroed and saved those two drives only, after checking the
whole roster stopped (identity, firmware, fault-free Reset). J5 returned its
UID to the save, and J4 again gave no save response. After an operator power
cycle both read within 0.0003 rad of zero, with every drive answering with
its pinned identity and a fault-free Reset.

This is reference revision `minerva-mechanical-reference-20261009`
([record](../config/devices/minerva-reference-20261009.json),
[action summary](devices/MINERVA_MECHANICAL_ZERO_20261009.json)). The other
twelve motors keep their 2026-09-24 zero. The revision names the earlier
reference in `direction_evidence_from`, because direction checks are signs,
which a re-zero does not move. The saved limits and servo calibration were
re-bound to it unchanged, on the operator's report that the ranges are
unchanged. Helm's direction session was migrated with a backup, as on
2026-09-26.

Manual direction checks are recorded separately in
[arm direction evidence](devices/MINERVA_ARM_DIRECTIONS_20260924.json).
The first two checks established left shoulder swivel J1 sign **+1** and right
shoulder swivel J1 sign **-1**, where the sign converts a raw motor-angle change
to the CAD joint's direction. The right CAD axis points down at the reference
pose, so positive CAD J1 turns clockwise viewed from above. The demonstration
angles were not measured physical targets: these observations establish signs,
not transmission ratios or alignment accuracy. The other ten arm-joint signs
remain pending. No runtime mapping or mechanical zero was changed by these reads.

## Why an apparent zero may change

A reading one whole turn off has a known cause on Minerva: the drives' power-on
position range (`zero_sta`, 0x7029). At the factory setting 0 a drive reports
its power-up angle within 0…2π, so a joint resting a little below its zero at
power-up reads a turn high. All 14 drives read 0 until 2026-09-30, when
`robot_platform.control.power_on_range` set them to 1 (−π…π, centred on the
zero) and saved it. That write changes no zero; see the
[device log](devices/MINERVA_PRODUCTION_01.md) and its
[evidence](devices/MINERVA_POWER_ON_RANGE_20260930.json). Minerva Check reads
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
