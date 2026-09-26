# Minerva RH-02 field research — 2026-09-25 UTC

## Conclusion

There is strong precedent for **candleLight/gs_usb transmit-slot exhaustion and
USB buffer-management failures**, but no matching, resolved RH-02-specific echo
failure was found in this search. The closest reports concern UCAN, CANable, or
other STM32 boards. They justify targeted diagnostics; they do not prove that
Minerva's adapters are defective or that another firmware update will fix them.

The installed Linux module has independently verified missing fixes; see the
[driver audit](MINERVA_CAN_DRIVER_AUDIT_20260924.md). Firmware-side fixes from
2021, January 2026, and June 2026 are already included in both installed adapter
images. Adapter autosuspend is currently disabled. CAN wiring, ground reference,
USB electrical behavior, and firmware/host-controller interaction remain open.

## Identify the correct adapter

These are **classic RH-02** adapters: STM32F072, USB-A, a switched 120-ohm CAN
terminator, and a 24 MHz crystal feeding a 48 MHz system clock. They are not the
RH-02 Plus/Pro/Elite STM32G4 CAN-FD products. Firmware and fault reports for those
products must not be applied indiscriminately.
[Zephyr board support](https://docs.zephyrproject.org/latest/boards/jhoinrch/rh02/doc/index.html)

The manufacturer describes RH-02 as CANable-derived hardware with candleLight
firmware. Its compatibility and warranty claims are not an independent endurance
test. No manufacturer erratum for this exact failure was found.
[Manufacturer page](https://jhoinrch.net/index.php?a=index&aid=124&c=View&m=home)

Current inventory, after the operator restored the left short USB cable:

| Item | Left | Right |
| --- | --- | --- |
| Stable serial | `004D002C4730511520303650` | `0021002F4D53530920303937` |
| USB path at capture | `1-2.4` | `1-2.1` |
| Negotiated speed | 12 Mbps | 12 Mbps |
| Reported firmware | `fw_85e5b33_2026-09-17` | `fw_85e5b33_2026-09-17` |
| Runtime power policy | `on`, active | `on`, active |
| Runtime suspended time | 0 ms | 0 ms |

Source: `simulation_runs/rh02-leads-20260925T031025Z/usb-power.json`.
The 150 mA USB descriptor value is a declared power requirement, not a measured
current or proof of good supply voltage. A USB 3.0 cable does not change the
adapter's negotiated full-speed USB operation.

Earlier “direct USB” tests removed the external hub; short extension cables were
not then documented. The later left-cable-removal test also changed USB port
locations and failed, so it was not a cable-only comparison. Both adapters still
share the Jetson's onboard hub, `1-2`.

Later in this investigation the operator reported **60 ohms across each CAN
bus**, but clarified that **only motor power was off; adapter USB remained
connected**. These are provisional readings, not accepted unpowered measurements.
A repeat with both USB adapters unplugged was requested. Even a valid 60-ohm
result would not establish resistor locations or prove waveform/ground integrity.

## Closest reports and what actually happened

| Primary evidence | Reported behavior and outcome | Relevance to Minerva |
| --- | --- | --- |
| [candleLight #187, maintainer diagnosis](https://github.com/candle-usb/candleLight_fw/issues/187#issuecomment-3506308052) | The original UCAN report had ten USB transmit failures together. A maintainer separately reproduced sporadic transfer failures on STM32G431 with alternative firmware: lost driver slots eventually stopped transmission. Driver cleanup addressed the symptom; disabling USB double buffering worked around that G431 trigger. The maintainer distinguished the two cases. | Strong precedent for the missing host cleanup fix. Different MCU/firmware means the G431 workaround is not evidence that RH-02 needs the same change. |
| [#187 reporter's outcome](https://github.com/candle-usb/candleLight_fw/issues/187#issuecomment-3556961334) | The reporter changed to original candleLight hardware and reported no further problems, without establishing hardware versus software cause. | A known-good adapter substitution is a useful experiment. This is one report, not a demonstrated RH-02 defect or universal purchasing recommendation. |
| [candleLight #207](https://github.com/candle-usb/candleLight_fw/issues/207), [maintainer's STM32F072 result](https://github.com/candle-usb/candleLight_fw/issues/207#issuecomment-3819079838) | Heavy USB load exposed receive-buffer/list corruption. PR #223 was reported stable on STM32F072; initial throughput dropped from approximately 88% to 83% CAN load, with optimization later recovering performance. | Direct MCU-family relevance, but both January 2026 fixes are already present in our firmware. It is historical precedent, not an outstanding patch. |
| [candleLight PR #301](https://github.com/candle-usb/candleLight_fw/pull/301) | Fixes stuck device-to-host USB transfers after suspend or host reboot. | Already included. Our current failures occur during active reads; this report alone does not establish a suspend trigger. |
| [candleLight #192](https://github.com/candle-usb/candleLight_fw/issues/192) | A 27-adapter, hub-connected system stopped transmitting while receiving continued. Enlarging the driver limits did not resolve it. The public issue has no comments or confirmed resolution as retrieved. | Shows that shared USB topology/load can matter. Its load and scale differ greatly from two Minerva adapters, and failure also occurred with only Minerva's left CAN interface active. Enlarging queues is not a demonstrated fix. |
| [candleLight #58 resolution](https://github.com/candle-usb/candleLight_fw/issues/58#issuecomment-828058219) | Older reports mixed CAN faults, USB buffering, and TX-slot losses. After the 2021 USB-stack update, the maintainer could no longer reproduce the issue. | Supports instrumenting both USB and CAN instead of equating similar symptoms. The cited update is already an ancestor of our build. |

A direct RH-02 owner reported successful 250 kbit/s reception through Cangaroo on
Windows. That demonstrates ordinary receive operation, not reliable sustained
transmission or Jetson compatibility.
[RH-02 owner's test](https://github.com/andrey-pr/OpenBafangTool/issues/39)

### Verify that historical firmware fixes are already included

Local `git merge-base --is-ancestor` checks against installed source commit
`85e5b3352db7f051aa879eeb71555dfce664c44c` returned success for:

| Change | Commit | Date |
| --- | --- | --- |
| USB-stack update associated with #58 | `e6b972441b767b8952d9cd8c78cd74a43be091a9` | 2021-04-25 |
| PR #209 merge | `92bda614ce1ef0685dd3d898cde21c410199f9e1` | 2026-01-20 |
| PR #223 merge | `0d8e058cd783d64b2add9fd6efa3a94da6f7aa84` | 2026-01-30 |
| PR #301 merge | `5c6a13f2f3db6defd1b1bf4c371b760ddefdbad3` | 2026-06-11 |

Repository: `simulation_runs/candleLight_fw-rh02`. Public issue-comment snapshots
used above are saved under `simulation_runs/rh02-field-research-20260925/`.

## Chase each lead with a discriminating observation

| Lead | Evidence so far | Next observation that can distinguish it |
| --- | --- | --- |
| Host driver accounting / short USB transfers | Binary audit proves missing upstream checks and cleanup. CAN captures do not expose USB packet lengths or outstanding echo slots. | Build against matching Jetson headers, retain the original module, and instrument completion status, actual length, echo ID, and outstanding slot count. Compare a bounded stopped test before/after. Missing fixes alone are not causal proof. |
| Adapter firmware / MCU USB handling | Related upstream bugs exist; known fixes are present. Wrongly received outgoing frames and unused echo IDs remain unexplained. | Capture USB headers and correlate each anomaly with the CAN transaction. If correct-length packets contain wrong echo metadata, firmware/adapter handling becomes a stronger lead. An isolated known-good peer/host comparison can separate host dependence from adapter dependence. |
| USB cable / power / controller | Left failed with its cable removed; port paths changed too. An old `clear tt error -71` burst coincided with external hub removal, not the subsequent running tests. | Correlate new USB status codes with failure timestamps. Change only one cable/port/controller at a time, preserving serial identities. Measure adapter-side supply if electrical faults remain suspected. Do not treat general Jetson rail readings as USB-connector measurements. |
| Autosuspend / reboot lifecycle | Both adapters currently have `power/control=on` and zero runtime suspended time. The onboard hub has historical suspended time, without a matching failure timestamp. | Record power state/time before and after the same bounded run. Current evidence demotes adapter autosuspend; indiscriminately disabling host-wide power management would not establish cause. |
| CAN termination / ACK / ground / noise | A termination change altered one short trial's behavior, but errors continued. Initial 60-ohm readings had motor power off but USB adapters still powered, so unpowered resistance verification is pending. Unsupported detailed error reporting means zero software error counters do not clear the wire. | Repeat the DC resistance measurement with all bus nodes unpowered, map the two bus ends, and inspect the ground path and twisted-pair/stub layout. If needed, use a scope or independent analyzer to check ACK, retries, differential waveform, and common-mode voltage. |
| Defective individual adapter versus left-arm wiring | Failure follows the left logical branch across USB ports and exists with left-only traffic. This still couples one adapter to one CAN branch. | A controlled physical adapter-to-bus cross-test, with motor UID allowlists and temporary diagnostic mapping updated deliberately, can separate adapter from arm wiring. Do not simply cross the CAN plugs and run production commands. |

Linux documents `power/control=on` as prohibiting runtime autosuspend. The
current 2000 ms delay field does not override that policy.
[Linux USB power management](https://www.kernel.org/doc/html/latest/driver-api/usb/power-management.html)

USB `-EPROTO`/`-EILSEQ` can indicate protocol, response, or CRC failures;
controller hardware may not distinguish them. Such statuses can implicate
cables, devices, firmware, or disconnects, so a status plus timestamp is much
more useful than guessing from an echo warning alone.
[Linux USB error codes](https://www.kernel.org/doc/html/latest/driver-api/usb/error-codes.html)

For a conventional two-ended CAN bus, two 120-ohm terminators produce about
60 ohms across CAN-H/CAN-L. Extra parallel terminators increase load; a good DC
resistance alone does not verify topology or signal integrity. Each receiver
must also stay within its transceiver's common-mode range; the actual RH-02
transceiver and wiring must be checked before assuming a numerical limit.
[TI termination and ground-reference explanation](https://e2e.ti.com/support/interface-group/interface/f/interface-forum/1166126/sn65hvd233-some-questions-in-can-bus)

## Boundaries of this research

No hardware operation was performed for this document. Proposed experiments are
stopped-motor diagnostics, capped at three minutes and stopped on failure.
Firmware/source ancestry and a symptom match are not proof of physical root
cause. A passing short run does not qualify powered teleoperation.
