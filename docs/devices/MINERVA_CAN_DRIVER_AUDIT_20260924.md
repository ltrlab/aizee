# Minerva production CAN driver audit — 2026-09-24 local

## Result

The installed `gs_usb.ko` lacks three upstream fixes relevant to the current
transport investigation: failed USB transmit cleanup, received USB header-length
validation, and received USB payload-length validation. This conclusion comes
from the installed binary's disassembly and matching installed kernel headers,
not from the kernel version alone.

These are verified driver deficiencies, **not a verified cause of the observed
echo errors**. No driver, kernel, adapter firmware, or motor settings were changed
by this audit.

## Inspected artifact

| Field | Value |
| --- | --- |
| Device | `minerva-production-01`, `aizee-rover` |
| Kernel | `5.15.185-tegra`, aarch64 |
| Module | `/lib/modules/5.15.185-tegra/kernel/drivers/net/can/usb/gs_usb.ko` |
| SHA-256 | `8ff2f6fadea5b591a20976fa7114b2823e4c10aa048a979b347f9356acc0105e` |
| Vermagic | `5.15.185 SMP preempt mod_unload modversions aarch64` |
| Evidence | `simulation_runs/rh02-left-cable-removed-20260925T025109Z/driver-inventory.json` |
| Header | `/lib/modules/5.15.185-tegra/build/include/linux/usb.h` |

The evidence includes successful relocation-aware `objdump -dr` commands for
`gs_usb_xmit_callback` and `gs_usb_receive_bulk_callback`, plus the installed
`struct urb` declaration. The relocation entries from `-dr` verify the called
symbols. Addresses below are offsets in this module's `.text` section, not
runtime addresses. A search under `/home/ltr` to depth six found no `gs_usb.c`;
that bounded search does not establish that its source is absent everywhere.

## 1. Failed USB transmit cleanup is absent

The entire `gs_usb_xmit_callback` occupies `0x1b0`–`0x214`:

- `0x1d0` loads the completion status; `0x1d4` branches on a nonzero result.
- The failure branch calls only `netdev_info` at `0x210`, then joins the ordinary
  buffer-free path at `0x1d8`.
- That common path calls `usb_free_coherent` at `0x1e4` and returns.
- Apart from the profiling hook, those are its only external calls. The function
  has no driver-context or network-statistic writes, echo-buffer release, atomic
  decrement, or transmit-queue wake operation.

Thus it does not implement the cleanup added by upstream
[516a0cd1c03fa266bb67dd87940a209fd4e53ce7](https://github.com/torvalds/linux/commit/516a0cd1c03fa266bb67dd87940a209fd4e53ce7).
That change releases a failed transfer's echo buffer and driver slot, updates
accounting, and wakes transmission. Upstream identifies slot loss after repeated
failed USB transfers as a cause of transmission eventually stopping.

**Limit:** a missing fix proves susceptibility. It does not prove that failed
USB transmit completions occurred during our particular echo-error trials.

## 2. Received header length is not checked

The receive callback keeps its `urb` pointer in `x19` and its data-buffer pointer
in `x21`. Identification of the fields is supported by the installed header and
the buffer-free/resubmit call arguments in the disassembly:

- `urb.status` is read at offset 88.
- `urb.transfer_buffer` is read at offset 96.
- `urb.transfer_buffer_length` is at offset 128; the installed declaration places
  the next `u32`, `actual_length`, immediately after it, at offset 132.

After checking status, the callback reads the received buffer's channel byte at
`0xd50` (`[x21, #9]`). It then reads the echo ID at `0xd74` (`[x21]`). There is no
load or comparison of `urb.actual_length` before these accesses, or anywhere else
in the inspected callback. The preceding checks cover context, status, channel,
and device presence; none checks the number of received bytes.

This lacks the behavior added by upstream
[6fe9f3279f7d](https://github.com/torvalds/linux/commit/6fe9f3279f7d),
which rejects a received USB transfer shorter than the protocol header before
interpreting its fields.

**Inference, not captured evidence:** a short or empty transfer reported as
successful could leave old bytes in the reused receive buffer. Interpreting
those bytes could produce a stale echo ID. The current CAN-level logs do not
record USB transfer lengths, so they cannot establish that this happened.

## 3. Received payload length is not checked

On the normal CAN receive path, `0xfdc` reads eight bytes from buffer offset 12,
and `0xfe0` copies them into the CAN frame. The path adjusts the advertised CAN
length but never verifies that the USB transfer actually supplied the payload.

This lacks the validation added by upstream
[395d988f93861101ec89d0dd9e3b876ae9392a5b](https://github.com/torvalds/linux/commit/395d988f93861101ec89d0dd9e3b876ae9392a5b).
That fix checks the received amount against the required header/payload size and
copies only received data.

## Meaning of the existing echo warnings

In the `gs_usb` protocol an echo is a USB-side confirmation associated with an
outstanding transmit slot. The warning means the returned slot number is not
currently in use. It is not a direct measurement of CAN-wire reflections.
The protocol reserves `0xffffffff` for ordinary received CAN frames.
[Linux implementation](https://github.com/torvalds/linux/blob/v5.15/drivers/net/can/usb/gs_usb.c)

Malformed or duplicated firmware output, delayed confirmations after a reset,
driver bookkeeping errors, and incorrectly parsed short transfers remain
possible explanations. USB electrical faults could trigger some failure paths.
CAN termination can cause CAN communication faults, but the warning alone does
not diagnose termination.

## Next diagnostic step

Keep the original module and its hash as the rollback baseline. Obtain the exact
matching build inputs before preparing a replacement. A targeted backport should
preserve this kernel's APIs and coherent-buffer ownership rather than substituting
a current-mainline source file unchanged.

Useful bounded diagnostic fields are USB completion status, actual received
length, echo ID, and outstanding transmit slots at each anomaly. At the CAN
socket layer, record `recvmsg()` provenance flags and subscribe separately to
CAN error frames. Those observations can distinguish software loopback from
received traffic and expose electrical errors omitted by default RAW sockets.
[SocketCAN documentation](https://www.kernel.org/doc/html/latest/networking/can.html#returned-message-flags)

A short stopped-motor test after a verified backport could test whether these
deficiencies contribute to the present failure. Passing it would not by itself
qualify powered teleoperation or prove which physical component triggered the
original fault.
