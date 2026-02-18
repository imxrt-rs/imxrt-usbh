# Phase 2b Debugging: HID Keyboard Reports Show Zeros on Key Press

**Date**: 2026-02-17
**Symptom**: Interrupt pipe allocated correctly. Idle reports (all zeros) are received
and logged. When keyboard keys are pressed, no `key: ...` lines appear — the idle
suppression `if mods == 0 && pkt.data[2] == 0 { continue; }` always fires even
during key presses. This means `pkt.data[2]` is always 0 regardless of the key state.

## Confirmed Working

- PLL, VBUS, PHY, host controller init: ✅
- Device enumeration (VID/PID, SET_ADDRESS, SET_CONFIGURATION): ✅
- `get_configuration` + `configure` + interrupt pipe allocation: ✅
- Periodic schedule linked: qh=2 qtd=0 at addr=1 ep=1 mps=8 ✅
- ISR fires: idle reports arrive (size=8, all zeros) ✅
- Re-arm cycle works: pipe produces continuous idle reports ✅

## Root Cause Hypotheses

### H1: recv_buf in DTCM — DMA Cannot Write to It ⭐ Most Likely

**The i.MX RT 1062 Cortex-M7 has two locally-coupled RAM regions:**

| Region | Address     | CPU | DMA (EHCI) | Rust section |
|--------|-------------|-----|------------|--------------|
| ITCM   | 0x0000_0000 | ✓   | ✗          | code         |
| DTCM   | 0x2000_0000 | ✓   | ✗          | `.bss` (often!) |
| OCRAM1 | 0x2020_0000 | ✓   | ✓          | depends      |
| OCRAM2 | 0x2024_0000 | ✓   | ✓          | depends      |

DTCM is a **tightly coupled** bank accessed via a private bus. The EHCI DMA
engine uses the system bus interconnect (AXI) which **cannot reach DTCM**.
If `recv_bufs` is placed in DTCM, the EHCI controller silently fails to write
received data there. The buffer stays at its initial value (zeros).

**Why idle reports still show size=8**: The EHCI controller clears the Active
bit and reports `remaining=0` after completing the DMA transaction (from its
perspective), even though the write did not reach DTCM. The `size=8` comes
from the qTD token's bytes-to-transfer field, not from the actual buffer.

**Diagnosis**: Log the address of `statics.recv_bufs[0]`.
- `0x2000_xxxx` → DTCM → DMA cannot write → always zeros. **Fix: move to OCRAM.**
- `0x2020_xxxx` or `0x2024_xxxx` → OCRAM → DMA works → cache issue (see H2).

**Quick fix attempt**: Force `recv_bufs` to a known DMA-accessible address by
using `#[link_section = ".ocram"]` or by allocating from a fixed-address pool.

---

### H2: Cache Stale Read After poll_next Returns Ready ⭐ Second Most Likely

**Sequence in `poll_next` after first Ready:**

1. We read `recv_buf[..copy_len]` → CPU cache loads recv_buf lines (PRESENT, CLEAN).
2. We re-arm: write Active=1 to qTD, write overlay_next to QH → both become dirty.
3. `cache_clean_qtd` + `cache_clean_qh` → DCCIMVAC (write-back + invalidate) for qTD and QH.
4. **BUT recv_buf cache lines are still PRESENT AND CLEAN** (loaded in step 1, not touched).
5. Hardware DMA writes new key data to recv_buf physical memory.
6. Next `poll_next`: `cache_clean_buffer(recv_buf)` → DCCIMVAC on recv_buf.
   - If the CPU's cache lines for recv_buf were CLEAN (step 4), DCCIMVAC just invalidates them.
   - Next `copy_from_slice` fetches fresh memory → should see key data. ✓

**This path should work** — `clean_invalidate_dcache_by_address` (DCCIMVAC) does
invalidate clean lines. However, there is a subtle ordering risk:

```
poll_next call N returns Ready: reads recv_buf into CPU cache (clean lines)
Hardware writes new key data to recv_buf (behind CPU cache)
poll_next call N+1: cache_clean_buffer runs DCCIMVAC → invalidates clean recv_buf lines
copy_from_slice: cache miss → fetches from memory → correct data
```

This is correct only if `cache_clean_buffer` is called **before** `copy_from_slice`.
**Verify**: Confirm `Imxrt1062HostController::cache_clean_buffer(recv_buf.as_ptr(), recv_buf.len())`
is the call at line 1539 (not `cache_clean_qtd`). The names are confusing — both do
clean+invalidate, but only the recv_buf one clears the right address.

---

### H3: qTD Error State Mistaken for Completion

The poll_next code checks only the Active bit:
```rust
if token & ehci::QTD_TOKEN_ACTIVE != 0 { return Poll::Pending; }
// falls through as if successful
```

If the qTD is **halted** (Active=0, Halted=1 due to a transaction error like
data toggle mismatch, NAK timeout, or bus error), we treat it as a successful
completion. The result:
- `remaining = max_packet_size` (no bytes transferred in failed transaction)
- `received = max_packet_size - max_packet_size = 0`
- `copy_len = 0`, `pkt.size = 0`, `pkt.data = [0; 64]`
- Idle suppression fires → no log

**Diagnosis**: Log the full qTD token value when Active=0, including the error bits:
- Bit 6: Halted
- Bit 5: Data Buffer Error
- Bit 4: Babble
- Bit 3: Transaction Error
- Bit 2: Missed Micro-Frame

After a halt, the QH itself halts (`overlay_token[Halted]=1`). Re-arming a
halted QH without clearing the halt bit means the controller ignores the QH
permanently.

---

### H4: QH Halt Not Cleared on Re-arm

`reattach_qtd_preserve_toggle` writes `overlay_next` but does NOT touch
`overlay_token`. If the QH overlay's Halted bit (bit 6 of overlay_token) is
set after a completed transaction (shouldn't be for normal completion, but is
set on errors), re-arming by writing overlay_next alone is not enough.
EHCI requires: clear Halted bit AND clear overlay_token's Active bit before
pointing overlay_next to a new qTD.

This is only a problem if H3 is also present (qTD halted). For normal
completions (no errors), overlay_token Halted=0 and the re-arm works.

---

### H5: Interrupt Not Firing for Periodic Schedule During Key Press

The ISR masks USBINT (bit 0) and UPIE (bit 19) after servicing. `poll_next`
re-enables them before returning Pending. If there's a window where:
- USBINT fires for periodic completion
- ISR wakes pipe wakers
- `poll_next` runs, finds Active=1 (too early — hardware hasn't finished)
- Returns Pending WITHOUT re-enabling USBINTR?

**No** — `poll_next` always re-enables USBINTR when returning Pending. But there
is a race: what if poll_next checks Active before the hardware clears it?

The ISR fires on USBINT, which is raised **after** the hardware writes the
completed qTD status. So if the ISR wakes us, the qTD SHOULD have Active=0.
But the cache might still show Active=1 if the invalidation races with the ISR.

**Diagnosis**: Add logging every time poll_next returns Pending vs Ready. If
Pending is returned after an ISR-triggered wake, it means the cache invalidation
exposed stale data (H2 variant) or the ISR fired for a different reason.

---

## Diagnostic Steps

### Step 1: Print recv_buf Address (Rules out H1)

In `do_alloc_interrupt_pipe`, add immediately after computing recv_buf_ptr:

```rust
log::info!(
    "[HC] recv_buf[{}] @ 0x{:08x} (expected OCRAM: 0x2020_0000..0x202F_FFFF)",
    recv_buf_idx,
    recv_buf_ptr as u32,
);
```

**Expected**: `0x2020_xxxx` or `0x2024_xxxx` (OCRAM — DMA-accessible)
**Bad**: `0x2000_xxxx` (DTCM — DMA cannot write here → always zeros)

---

### Step 2: Log Raw Token and recv_buf Bytes on Every Completion (Rules out H2, H3)

In `poll_next`, after the `if token & QTD_TOKEN_ACTIVE != 0` check falls through,
add before copying data:

```rust
// Diagnostic: log full token and first 8 recv_buf bytes
log::info!(
    "[HC] qTD complete: token=0x{:08x} remaining={} recv=[{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}]",
    token,
    ehci::qtd_token_bytes_remaining(token),
    recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3],
    recv_buf[4], recv_buf[5], recv_buf[6], recv_buf[7],
);
```

**Expected (key press)**: token has error bits=0, remaining=0, recv bytes non-zero
**H1 symptom**: recv bytes always 0x00 even when key held
**H3 symptom**: token has Halted bit (bit 6) or error bits set, remaining=8

---

### Step 3: Check QH Overlay Token After Completion (Rules out H4)

In `poll_next` after cache-invalidating the qTD, also read the QH overlay token:

```rust
let qh = &self.statics.qh_pool[self.qh_index];
Imxrt1062HostController::cache_clean_qh(qh as *const QueueHead);
let qh_token = unsafe { (*qh_ptr).overlay_token.read() };
log::info!("[HC] qTD token=0x{:08x} QH overlay_token=0x{:08x}", token, qh_token);
```

If QH overlay token shows Halted=1 (bit 6), the clear-halt procedure is needed:
1. Clear QH overlay_token Halted bit (write 0)
2. Set overlay_next to re-armed qTD
3. Flush QH to memory

---

### Step 4: Verify qTD Buffer Pointer in Hardware (Confirms H1/H2)

Dump the actual qTD contents after allocation to confirm the buffer pointer:

```rust
let qtd = &self.statics.qtd_pool[qtd_index];
log::info!(
    "[HC] qTD[{}] @ 0x{:08x}: token=0x{:08x} buf0=0x{:08x}",
    qtd_index,
    qtd as *const TransferDescriptor as u32,
    unsafe { qtd.token.read() },
    unsafe { qtd.buffer0.read() },
);
```

`buf0` should equal `recv_buf_ptr`. If it doesn't, the qTD init is wrong.
If `buf0` is in DTCM range (0x2000_xxxx), H1 is confirmed.

---

### Step 5: Remove Idle Suppression, Watch for Non-Zero Bytes (Confirms H1)

Temporarily remove:
```rust
if mods == 0 && pkt.data[2] == 0 { continue; }
```

Instead, always log but add raw byte output:
```rust
log::info!("pkt size={} data=[{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}]",
    pkt.size, pkt.data[0], pkt.data[1], pkt.data[2], pkt.data[3],
    pkt.data[4], pkt.data[5], pkt.data[6], pkt.data[7]);
```

**While holding a key down**, watch the output. If bytes are still all zero,
the recv_buf is not receiving DMA data. If any byte goes non-zero, idle suppression
logic is the issue.

---

## Fix Procedures

### Fix for H1: Force recv_bufs into DMA-Accessible Memory

**Option A: Link section attribute** (if linker script defines an `.ocram` section):

```rust
// In UsbStatics:
#[link_section = ".ocram2"]
pub recv_bufs: [[u8; 64]; NUM_QH - 1],
```

**Option B: Allocate a static in a known OCRAM address** using a custom section
in `memory.x`. The board crate for Teensy 4.1 should have OCRAM defined.

**Option C: Move UsbStatics itself into a known-good section** by declaring:
```rust
#[link_section = ".ocram2"]
static mut STATICS: UsbStatics = UsbStatics::new();
```

Check the current linker script to find the correct section name for OCRAM
on this board crate. Typical names: `".ocram2"`, `".dmabuf"`, `".bss.dma"`.

**Verification**: After the fix, `recv_buf` address should move from
`0x2000_xxxx` to `0x2020_xxxx` or `0x2024_xxxx`.

---

### Fix for H3: Add Error Bit Checking in poll_next

After the Active bit check falls through (transfer complete), check for errors
before assuming success:

```rust
// Check for EHCI error bits in qTD token.
let error_bits = token & (QTD_TOKEN_HALTED | QTD_TOKEN_BUFFER_ERR
    | QTD_TOKEN_BABBLE | QTD_TOKEN_XACT_ERR | QTD_TOKEN_MISSED_UFRAME);
if error_bits != 0 {
    // Log and attempt recovery: clear the halt and re-arm.
    log::warn!("[HC] qTD error: token=0x{:08x}", token);
    // Clear Halted bit in QH overlay, re-arm qTD...
}
```

---

## Expected Debug Output (After Adding Step 2 Logging)

### If H1 (DTCM): recv_buf always zero even with key held
```
[INFO imxrt_usbh::host]: recv_buf[0] @ 0x20001234 (DTCM — DMA cannot write!)
[INFO imxrt_usbh::host]: qTD complete: token=0x00000000 remaining=0 recv=[00 00 00 00 00 00 00 00]
[INFO imxrt_usbh::host]: qTD complete: token=0x00000000 remaining=0 recv=[00 00 00 00 00 00 00 00]
```

### If H3 (Error/Halt): token shows error bits, remaining != 0
```
[INFO imxrt_usbh::host]: recv_buf[0] @ 0x20200120 (OCRAM — OK)
[INFO imxrt_usbh::host]: qTD complete: token=0x00000040 remaining=8 recv=[00 00 00 00 00 00 00 00]
```
(token bit 6 = Halted, remaining=8 means 0 bytes transferred)

### If Working Correctly: token=0, remaining=0, non-zero bytes for key press
```
[INFO imxrt_usbh::host]: recv_buf[0] @ 0x20200120 (OCRAM — OK)
[INFO imxrt_usbh::host]: qTD complete: token=0x00000000 remaining=0 recv=[00 00 04 00 00 00 00 00]
```
(bytes [2]=0x04 = 'A' key pressed)

---

## Action Plan

1. ✅ **Add recv_buf address log** (Step 1) → built into `host.rs` line ~1194.
2. ✅ **Add qTD completion log** (Step 2) → built into `host.rs` line ~1552.
3. **Flash and test** → observe `[HC] recv_buf[0] @ 0x...` and `[HC] qTD done: ...` lines.
4. Based on results, apply the appropriate fix (H1 → OCRAM section, H3 → error check).
5. If bytes are correct but idle suppression still fires, revisit the suppress condition.

The combination of Step 1 and Step 2 should definitively identify the root cause.

## Test Results — Round 1

```
[INFO rtic_usb_hid_keyboard::app]: Opening interrupt IN stream...
[INFO imxrt_usbh::host]: [HC] recv_buf[0] @ 0x20201340 (OCRAM=0x2020_0000..0x202F_FFFF, DTCM=0x2000_xxxx)
[INFO imxrt_usbh::host]: [HC] interrupt pipe allocated: addr=1 ep=1 mps=8 qh=2 qtd=0
[INFO imxrt_usbh::host]: [HC] qTD done: token=0x80008d00 rem=0 buf=[00 00 00 00 00 00 00 00]
[INFO imxrt_usbh::host]: [HC] qTD done: token=0x00008d00 rem=0 buf=[00 00 00 00 00 00 00 00]
```

qTD lines print periodically (~500ms) and whenever a key is pressed. Buffer is
always all zero.

### Analysis

**Token decode** (`0x80008d00` / `0x00008d00`):
- Status (bits 7:0) = 0x00 → Active=0, Halted=0, no errors ✓
- PID (bits 9:8) = 01 (IN) ✓
- CERR (bits 11:10) = 3 (no retries consumed) ✓
- IOC (bit 15) = 1 ✓
- Total bytes remaining (bits 30:16) = 0 → all 8 bytes transferred ✓
- DT (bit 31) alternates 1/0 → data toggle working correctly ✓

**Hypothesis status**:
- **H1 (DTCM) — RULED OUT**: recv_buf @ 0x20201340 is in OCRAM ✓
- **H3 (Error/Halt) — RULED OUT**: token shows no errors, remaining=0 ✓
- **H2 (Cache) — PARTIALLY**: transfer succeeds, but we read zeros

### Root Cause Identified: H6 — qTD buffer pointer advances on each re-arm

The EHCI controller writes back the modified overlay (including `buffer[0]` with
advanced current offset) to the original qTD after each completed transfer.
Our re-arm in `poll_next` only resets `qTD.token` (Active=1, fresh byte count)
but does NOT re-initialise `qTD.buffer[0]`.

**Detailed sequence**:

1. **Pipe allocation**: `qTD.init(token, recv_buf_ptr=0x20201340, 8)` → buffer[0]=0x20201340
2. **Transfer 1**: controller writes 8 bytes to overlay's buffer[0] address (0x20201340),
   then writes back overlay to qTD → `qTD.buffer[0]` = 0x20201348 (offset advanced by 8)
3. **Re-arm**: we write `qTD.token = Active|IN|8bytes` but leave buffer[0] = 0x20201348
4. **Transfer 2**: controller fetches qTD, copies into overlay → overlay buffer[0] = 0x20201348.
   Writes 8 bytes to 0x20201348. Writeback: qTD.buffer[0] = 0x20201350
5. **We read** `recv_buf[0..7]` at **0x20201340** → stale first-transfer data (all zeros from
   idle report)
6. Repeat: buffer pointer keeps advancing (0x20201358, 0x20201360, ...) while reads always
   target 0x20201340.

After 8 transfers, the DMA writes past the 64-byte `recv_bufs[0]` area, potentially
corrupting adjacent memory.

**Why the first report is also zero**: the very first transfer after pipe allocation
is typically an idle HID report (no keys pressed), so the initial data at 0x20201340
is genuinely all zeros. Subsequent key-press data is written to **advancing** addresses
that we never read.

### Fix

1. **Primary**: Re-initialise the full qTD (including buffer pointers) during re-arm
   by calling `(*qtd_ptr).init(rearm_token, recv_buf.as_ptr(), mps)` instead of only
   writing the token.
2. **Secondary**: Add `invalidate_dcache_by_address()` (DCIMVAC-only, no clean) to
   `cache.rs` and use it for DMA receive buffers. `clean_invalidate` can overwrite
   DMA data if the cache line is dirty; invalidate-only is correct for post-DMA reads.

### Expected result after fix

```
[HC] qTD done: token=0x80008d00 rem=0 buf=[00 00 00 00 00 00 00 00]   <- idle (no key)
[HC] qTD done: token=0x00008d00 rem=0 buf=[00 00 04 00 00 00 00 00]   <- 'A' pressed
key: A
```

## Test Results — Round 2 (After H6 Fix)

**Date**: 2026-02-18

```
[INFO imxrt_usbh::host]: [HC] qTD done: token=0x00008d00 rem=0 buf=[00 00 0e 00 00 00 00 00]
[INFO rtic_usb_hid_keyboard::app]: key: K
[INFO imxrt_usbh::host]: [HC] qTD done: token=0x80008d00 rem=0 buf=[00 00 00 00 00 00 00 00]
[INFO imxrt_usbh::host]: [HC] qTD done: token=0x00008d00 rem=0 buf=[00 00 00 00 00 00 00 00]
[INFO imxrt_usbh::host]: [HC] qTD done: token=0x80008d00 rem=0 buf=[00 00 0f 00 00 00 00 00]
[INFO rtic_usb_hid_keyboard::app]: key: L
```

### Analysis

**Token decode** (both `0x00008d00` and `0x80008d00`):
- Status (bits 7:0) = 0x00 → Active=0, Halted=0, no errors ✓
- Remaining (bits 30:16) = 0 → all bytes transferred ✓
- DT (bit 31) alternates 0/1 → data toggle working correctly ✓

**Buffer contents**:
- `buf=[00 00 0e 00 00 00 00 00]` → HID keycode 0x0e = 'K' at byte[2] ✓
- `buf=[00 00 0f 00 00 00 00 00]` → HID keycode 0x0f = 'L' at byte[2] ✓
- `buf=[00 00 00 00 00 00 00 00]` → idle report, suppressed correctly ✓

**Hypothesis resolution**:
- **H6 (buffer pointer advancement) — CONFIRMED AND FIXED**: Re-initialising the full qTD
  (including buffer[0]) on each re-arm restores the correct base address. Key press data
  now reaches `recv_bufs[0]` at 0x20201340 as expected.

### Conclusion: Phase 2b COMPLETE

All interrupt pipe functionality is verified working on hardware:
- ✅ Pipe allocation and periodic schedule linking
- ✅ Idle HID reports received (all zeros, correctly suppressed)
- ✅ Key press reports received with correct HID keycodes
- ✅ Re-arm cycle: buffer pointers fully reset each time
- ✅ Data toggle: alternates correctly (controller-managed, `DTC=0`)
- ✅ Zero error bits in all observed qTD tokens

**Next**: Move to Phase 2c — implement `bulk_in_transfer()` and `bulk_out_transfer()` for USB mass storage.
