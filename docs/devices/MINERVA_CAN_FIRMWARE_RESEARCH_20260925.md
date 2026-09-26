# Minerva RH-02 firmware investigation — 2026-09-25 UTC

## Conclusion

There are real upstream precedents for Candlelight losing transmit confirmations,
corrupting USB receive-buffer ownership, and stopping transmission. The closest
firmware fixes are **already present** in the installed revision. This audit did
not establish a new reproducible firmware defect or justify another flash.

The immediate missing evidence is the USB frame header at each anomaly: its echo
cookie, transfer length and completion status. CAN-socket captures cannot tell
whether the adapter sent the wrong header or the host interpreted it incorrectly.
The separately verified [host-driver deficiencies](MINERVA_CAN_DRIVER_AUDIT_20260924.md)
remain a concrete lead.

This was a source/history review and primary-source research only. No Jetson
access, hardware traffic, firmware changes or production-source edits were made.

## Inspected version and hardware path

- Installed candidate: `85e5b3352db7f051aa879eeb71555dfce664c44c`, target `RH02_fw`.
- Earlier identified left firmware: `49fb00165672f046bbd05c72b709c2a23c73d2b1`.
- Local immutable checkout: `simulation_runs/candleLight_fw-rh02`.
- RH-02 uses the STM32F072/bxCAN implementation, one CAN channel, a 24 MHz HSE
  board definition, 64 shared frame objects, two reserved USB OUT buffers,
  double-buffered USB OUT and single-buffered USB IN. This excludes fixes specific
  to STM32G0/M_CAN/CAN-FD unless a shared code path is involved.
  [Board addition](https://github.com/candle-usb/candleLight_fw/commit/71f8eb74d14337ffcaba4be30d3e380387f77482),
  [USB configuration](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/src/usbd_conf.c),
  [buffer definitions](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/include/usbd_gs_can.h).

## What the observed signature establishes

In the [60-second provenance capture](MINERVA_CAN_PROVENANCE_20260924.json), eight
received frames copied outgoing commands: five left, three right. All eight had
`MSG_DONTROUTE` and `MSG_CONFIRM` clear; both application sockets reported
receive-own-messages disabled. Therefore ordinary SocketCAN local echo is not
the explanation represented by those flags. This does **not** prove the frames
physically came from another CAN node: misclassification in `gs_usb` would also
appear as ordinary receive traffic.

Earlier failures repeatedly accumulated ten such copies, then stopped producing
complete scans. TX-drop counters rose on link-down cleanup, not during the live
portion. A lost transmit-cookie accounting entry is consistent with this pattern;
exactly ten matches the historical host driver's outstanding-transfer limit. It
does not mean the firmware has only ten buffers; the inspected RH-02 firmware has
64. A larger transmit queue would not repair lost confirmations.

## Relevant upstream reports and applicability

| Primary report | Relevant mechanism | Applicability here |
| --- | --- | --- |
| [#211: wrong receive buffer](https://github.com/candle-usb/candleLight_fw/issues/211) | An interrupt between choosing a USB destination and configuring it could overwrite an already queued frame, lose messages, or manufacture false frames. | Closest ownership-corruption precedent. Fixed by `d97f265`; present in both identified revisions. |
| [#207: internal list corruption](https://github.com/candle-usb/candleLight_fw/issues/207), [PR #209](https://github.com/candle-usb/candleLight_fw/pull/209) | USB double buffering allowed a further callback after the available object pool was exhausted. Reserved receive buffers prevent that failure. | Relevant shared buffering design; fix `6f6c14e` is present in both revisions. Original reproduction deliberately exhausted buffers under severe USB load. |
| [#218: STM32F072 transmission stops](https://github.com/candle-usb/candleLight_fw/issues/218), [PR #223](https://github.com/candle-usb/candleLight_fw/pull/223) | The first buffer fix repeatedly reconfigured reception during an active transfer. Even a low-rate transmit sequence lost confirmations and stalled. | Strong symptom and chip match. Corrective `d4c9122` is already present in both revisions. |
| [#187: stops after ten USB transmit failures](https://github.com/candle-usb/candleLight_fw/issues/187) | UCAN user reported ten explicit host `usb xmit fail` messages followed by a stop. | This led to a Linux driver cleanup fix, not a firmware fix. Our recent failures lack those messages, so this is susceptibility evidence rather than a matched trigger. |
| [#192: many-adapter ENOBUFS](https://github.com/candle-usb/candleLight_fw/issues/192) | 27 CANables behind USB hubs exhausted transmit accounting; substantially larger queues did not resolve it. | Confirms other users encounter this class of problem. It remains an unresolved report with a much heavier workload; it does not diagnose Minerva. |
| [#87: spurious overflow flags](https://github.com/candle-usb/candleLight_fw/issues/87) | Uninitialized host flags were echoed as overflow indications. | Current classic-CAN transmit code explicitly clears flags and reserved fields. This is not the observed unused-cookie signature. |
| [#302: missing bxCAN RX-overflow information](https://github.com/candle-usb/candleLight_fw/issues/302) | bxCAN receive-overflow reporting is incomplete. | Further reason not to treat zero observed error frames/counters as exhaustive proof of wire integrity. Not a demonstrated explanation for malformed echo accounting. |
| [#305: STM32F072 after module reload](https://github.com/candle-usb/candleLight_fw/issues/305) | CAN initialization times out after unloading/reloading the driver on an i.MX6 host. | A restart-path comparison, not the current steady-state loss after successful identity checks. |

Commit ancestry was checked locally with `git merge-base --is-ancestor`, rather
than inferred from version dates:

| Commit | Purpose | In left `49fb001` | In installed `85e5b33` |
| --- | --- | --- | --- |
| `d97f265` | Protect USB receive preparation against interrupt reentry | Yes | Yes |
| `6f6c14e` | Reserve USB receive buffers | Yes | Yes |
| `d4c9122` | Avoid unnecessary receive reconfiguration | Yes | Yes |
| `daa48a7` | Update STM32F0 HAL | Yes | Yes |
| `20b6d44` | Handle send-buffer cleanup during suspend | No | Yes |
| `2ddbd28` | Purge queued host transmissions on CAN shutdown | No | Yes |
| `71f8eb7` | Add explicit RH-02 board definition | No | Yes |

## Source-level path review

### Transmit cookies and frame ownership

`CAN_SendFrame` removes a frame object under the interrupt guard, writes its CAN
contents into an available hardware mailbox, and queues the same object back to
USB with its original `echo_id`. `CAN_ReceiveFrame` obtains a separate free object
and explicitly sets its cookie to `0xffffffff`, the ordinary-RX marker. Error
frames use that marker too. The normal implementation does not intentionally turn
a host command's cookie into the RX marker.

The echo is queued **after writing the mailbox**, before confirmed success on the
CAN wire. Consequently an echo is not evidence of an acknowledged physical
transmission. This limitation is documented upstream and implemented in the
inspected source. [CAN queue implementation](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/src/can_common.c),
[upstream limitations](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/README.md#limitations).

### USB receive and send paths

The current receive path reserves objects before arming USB, updates the queue
under interrupt protection, and only re-arms from the main loop when allocation
changed. These are the important corrections from the historical bugs above.
The USB IN object stays reserved in `to_host_buf` until the completion callback
returns it to the free pool. List allocation and mutation have explicit interrupt
guards. No unambiguous current steady-state double release was found.

There is still reason to instrument reset/suspend boundary behavior: the send
path releases the list lock before handing data to the USB stack, while suspend
and deinitialization can purge queues. Existing cleanup changes explicitly cover
this area. However, the recent failure records do not establish a suspend/reset
at each anomaly; a possible interleaving is not a reproduced cause.
[USB class implementation](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/src/usbd_gs_can.c).

### The temporary stack buffer is not a demonstrated RH-02 defect

The optional padded USB-send path builds a stack-local packet. That deserves
scrutiny on asynchronous/DMA controllers, but this RH-02 path sends a classic
frame of 20/24 bytes or a padded 32-byte packet. Its endpoint maximum is 32 bytes;
the STM32F0 single-buffer send routine copies that entire packet synchronously
into packet memory before returning. A generic asynchronous-stack-buffer claim
would therefore be misleading for this target. No reproduction was created for
an unproven defect.
[STM32F0 USB transfer/copy code](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/libs/STM32_HAL/stm32f0xx-hal-driver/Src/stm32f0xx_ll_usb.c).

### Detailed error reporting

The inspected bxCAN firmware advertises `GS_CAN_FEATURE_BERR_REPORTING` and its
common error handler gates detailed reports on the selected feature. The actual
host rejected enabling it on both interfaces with `Operation not supported`, as
recorded in the provenance JSON. Firmware capability alone is insufficient: the
host driver must expose and request that mode. The empty error-only socket log
does not exclude intermittent CAN-wire errors while the mode remains unavailable.
[bxCAN capabilities](https://github.com/candle-usb/candleLight_fw/blob/85e5b3352db7f051aa879eeb71555dfce664c44c/src/can/bxcan.c).

## Discriminating next steps

### Internal loopback is not safe to assume silent on this driver

The firmware independently sets bxCAN's silent and loopback bits. The inspected
host baseline (`simulation_runs/gs-usb-backport-20260925/baseline/gs_usb.c`, lines
669–672) instead selects loopback **or else** listen-only. Requesting both through
netlink therefore sends only the loopback bit to firmware. Lines 708–709 also
withhold transmit-queue startup when the host listen-only flag is set. Reporting
both flags in `ip` would not prove the peripheral received both.

ST documents that ordinary loopback can expose the transmitted waveform on
CANTX, while combined silent-loopback holds CANTX recessive. Consequently a test
that depends on silence must not rely on this baseline's combined setting.
Use a physically disconnected CAN bench connection, or first audit corrected
mode forwarding and queue behavior. No loopback test was performed here.
[ST RM0091, sections 29.5.2–29.5.3](https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf),
[Linux v5.15 driver](https://github.com/torvalds/linux/blob/v5.15/drivers/net/can/usb/gs_usb.c).

### Evidence to collect

1. Record USB completion status, actual transfer length, raw cookie and active
   transmit contexts with the exact host-driver baseline. This can separate a
   missing/short/duplicate USB completion from a genuine packet with an RX cookie.
2. Preserve the CAN error-visibility gap until BERR capability is available; do
   not infer correct termination from the present zero counters. Physical
   termination/topology measurements remain independent evidence.
3. If host-side evidence shows the adapter emits complete malformed packets,
   prepare a separate firmware instrumentation build on a bench adapter: record
   frame-object ownership and cookie at USB OUT, CAN-mailbox enqueue and USB IN.
   Include queue counts and suspend/reset counters. That is a follow-on proposal,
   not a build or flash performed by this audit.
4. An isolated bench CAN pair or a known-good adapter comparison can distinguish
   an arm-bus problem from the shared RH-02 firmware/host path. Keep unrelated
   motor traffic out of any generic `cangen` reproduction.

Do not enlarge queues, repeatedly cycle links, change motor firmware, or apply a
different RH-02-family firmware as a substitute for observing the fault. None of
those actions follows from the evidence collected in this review.
