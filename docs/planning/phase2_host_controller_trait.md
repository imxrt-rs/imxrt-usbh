# Phase 2: Core HostController Trait Implementation

**Estimated effort**: 5-7 days  
**Key milestones**:
- ✅ Phase 2a: `GET_DESCRIPTOR` works (3-4 days)
- ✅ Phase 2b: HID keyboard input received (2-3 days) — COMPLETE, working on hardware
- ✅ Phase 2c: USB flash drive sector read (2-3 days) — COMPLETE, working on hardware

## Sub-phase → Section Mapping

| Sub-phase | Sections | Scope |
|-----------|----------|-------|
| **2a** | 2.1, 2.2, 2.3 | Device detection, port reset, control transfers |
| **2b** | 2.4 | Interrupt pipe allocation and streaming |
| **2c** | 2.5 | Bulk IN/OUT transfers |

## 2.1 Device Detection (`device_detect()` → `Imxrt1062DeviceDetect`)

- [x] Implement `Imxrt1062DeviceDetect` as `Stream<Item = DeviceStatus>`
  - `poll_next()`: Register waker with `shared.device_waker`, then check `PORTSC1`:
    - Clear `PORTSC1[CSC]` (Connect Status Change) if set
    - Read `PORTSC1[CCS]` (Current Connect Status)
    - If connected: read `PORTSC1[PSPD]` for speed → yield `DeviceStatus::Present(UsbSpeed)`
      - `PSPD = 0b00` → Full Speed (12 Mbps) → `UsbSpeed::Full12`
      - `PSPD = 0b01` → Low Speed (1.5 Mbps) → `UsbSpeed::Low1_5`
      - `PSPD = 0b10` → High Speed (480 Mbps) → `UsbSpeed::High480` (future)
    - If not connected: yield `DeviceStatus::Absent`
  - Re-enable port change interrupt in `USBINTR` after handling (disable-on-handle pattern from RP2040)
- [x] Implement `device_detect()` method — creates and returns `Imxrt1062DeviceDetect`
- [x] ISR integration: `on_irq()` checks `USBSTS[PCI]`, clears it, disables PCI in `USBINTR`, wakes `device_waker`

### i.MX RT PORTSC1 Speed Detection Notes

- Speed is available in `PORTSC1[PSPD]` bits [27:26] after port is enabled
- The port is enabled after a successful reset, not just on connection
- During initial detection (before reset), speed may need to be inferred from line state `PORTSC1[LS]` bits [11:10]: `0b01` (J-state) = FS, `0b10` (K-state) = LS

## 2.2 Root Port Reset (`reset_root_port(rst: bool)`)

- [x] Implement `reset_root_port(rst: bool)` method
  - `rst = true`: Set `PORTSC1[PR]` (Port Reset) — begins USB reset signaling
  - `rst = false`: Clear `PORTSC1[PR]` — ends reset signaling
  - **Important**: The caller (UsbBus) handles timing — it calls with `true`, waits ≥50ms, then calls with `false`
  - After reset completes, `PORTSC1[PE]` (Port Enabled) should be set by hardware
  - Note: Writing to `PORTSC1` must preserve certain bits and write-1-to-clear others — use read-modify-write carefully, masking `W1C` bits (CSC, PEC, OCC, FPR)

## 2.3 Control Transfers (`control_transfer()`)

- [x] Implement `TransferComplete` as `Future<Output = Result<(), UsbError>>`
- [x] Implement the `control_transfer()` method
- [x] Implement `AsyncAdvanceWait` future for safe QH unlinking
- [x] Implement qTD allocation/freeing helpers
- [x] Implement async schedule link/unlink helpers
- [x] Implement EHCI error mapping (`map_qtd_error`)
- [x] Implement cache clean/invalidate wrappers

### Detailed Async State Machine

1. **Allocate resources**: Await a control pipe from `statics.control_pipes` pool (Pool of 1 — serializes control transfers)
2. **Build qTD chain** (3 qTDs for a full control transfer, 2 if no data phase):
   - **Setup qTD**: PID = SETUP (0b10), 8 bytes, data toggle = 0, buffer pointer → setup packet bytes. Set IOC = 0.
   - **Data qTD** (if `DataPhase::In` or `DataPhase::Out`): PID = IN (0b01) or OUT (0b00), length = data buffer size, data toggle = 1 initially. Buffer pointer → caller's data buffer. Set IOC = 0.
   - **Status qTD**: PID = opposite of data direction (or IN if no data phase), 0 bytes, data toggle = 1, IOC = 1 (Interrupt on Complete).
   - Link qTDs: each `next_qtd` points to the next; last qTD's `next_qtd` = T-bit (terminate).
   - Set `alt_next_qtd` = T-bit on all (don't want short-packet to skip ahead in control transfers).
3. **Configure QH for EP0**:
   - Device address, endpoint 0
   - Max packet size from `packet_size` parameter (often 8 for initial enumeration, 64 for configured FS devices)
   - Speed from current port speed
   - `DTC = 1` (data toggle from qTD), `H = 0` (not head of reclamation), `EPS` = endpoint speed
   - `current_qtd = 0`, `next_qtd` → first qTD (setup)
   - Handle `TransferExtras::WithPreamble` — set split transaction fields in QH endpoint capabilities if needed
4. **Flush caches**: Clean (flush) all qTD memory, QH memory, and outgoing data buffers from D-cache to main memory
5. **Link QH to async schedule**: Insert into the circular QH list (after sentinel QH)
6. **Enable async schedule**: Set `USBCMD[ASE]` if not already enabled
7. **Poll for completion**: Register waker, wait for ISR to signal transfer complete
   - ISR detects `USBSTS[USBINT]` (transfer completion), wakes the pipe waker
   - On wake: invalidate cache for QH overlay and qTDs, check status qTD `token[Status]` field
   - If Active bit still set → re-register waker and continue waiting
   - If error bits set → map to appropriate `UsbError` (Stall, Timeout, CrcError, etc.)
   - If success → read bytes transferred from data qTD, return `Ok(bytes_transferred)`
8. **Unlink QH from async schedule**: Remove from circular list. Use Async Advance Doorbell (`USBCMD[IAA]`) and wait for `USBSTS[AAI]` before freeing QH (ensures hardware is no longer referencing it)
9. **Free resources**: Return pipe to pool, return QH/qTDs to pools

### Error Mapping from EHCI qTD Status Bits

- Bit 6 (Halted) + Bit 5 (Data Buffer Error) → `UsbError::Overflow`
- Bit 6 (Halted) + Stall condition → `UsbError::Stall`
- Bit 4 (Babble) → `UsbError::Overflow`
- Bit 3 (Transaction Error) → `UsbError::ProtocolError` (retry up to `CERR` times, hardware handles retries)
- Missed Micro-frame → `UsbError::Timeout`

## 2.4 Interrupt Endpoint Support (`alloc_interrupt_pipe()` / `try_alloc_interrupt_pipe()`)

**Status**: ✅ COMPLETE — working on hardware (HID keyboard reports received)

- [x] Implement `Imxrt1062InterruptPipe` as `Stream<Item = InterruptPacket> + Unpin`
- [x] Implement `alloc_interrupt_pipe()` — async version (waits for pipe availability)
- [x] Implement `try_alloc_interrupt_pipe()` — sync version (returns `Err(UsbError::AllPipesInUse)` if full)

### Implementation Notes (as built)

- **QH index mapping**: `bulk_pipes` pool tokens 0,1,2 → `Pipe::new(token, 1)` → `which()` = 1,2,3 → QH index = `which+1` = 2,3,4 → recv_buf = `which-1` = 0,1,2 → waker = `which` = 1,2,3
- **Data toggle**: DTC=0 (hardware-managed) to preserve toggle across re-arms. New method `reattach_qtd_preserve_toggle` updates `overlay_next` without clearing `overlay_token`.
- **Periodic schedule**: Flat — all 32 frame list entries point to the same QH chain head. Insert at head, update all entries.
- **Drop safety**: 1ms busy-wait after frame list unlink before freeing QH/qTD resources.
- **Disable-on-handle**: ISR masks UPIE (periodic interrupt, bit 19) after periodic interrupt; `poll_next` re-enables before returning `Pending`.
- **Pool size fix**: `bulk_pipes: Pool::new((NUM_QH-1) as u8)` = 3 slots (corrected from NUM_QH=4 which would have caused OOB QH access).
- **Static recv buffers**: `recv_bufs: [[u8; 64]; NUM_QH-1]` in `UsbStatics` provides DMA-stable addresses.
- **Example**: `rtic_usb_hid_keyboard.rs` uses `UsbBus::get_configuration()` + `UsbBus::configure()` + `UsbBus::interrupt_endpoint_in()` high-level API.

### Expected hardware test output

```text
=== imxrt-usbh: HID Keyboard Example ===
USB2 PLL locked
VBUS power enabled
USB host controller initialised
USB_OTG2 ISR installed (NVIC priority 0xE0)
Entering device event loop...
DeviceEvent::Connect  addr=1  VID=045e PID=00db class=0 subclass=0
Found HID interface: iface=0 ep=1 mps=8 interval=10
Opening interrupt IN stream...
HID report [8]: 00 00 00 00 00 00 00 00
HID report [8]: 00 00 04 00 00 00 00 00
HID report [8]: 00 00 00 00 00 00 00 00
```

### Interrupt Pipe Lifecycle

1. **Allocate**: Get a pipe slot from `statics.bulk_pipes` pool. Get a QH and qTD(s) from their pools.
2. **Configure QH for interrupt endpoint**:
   - Device address, endpoint number (from parameters)
   - Max packet size from `max_packet_size`
   - Interrupt schedule mask (`S-mask`) based on `interval_ms` — controls which micro-frames to poll
   - `EPS` = endpoint speed
   - Handle `TransferExtras::WithPreamble` for split transactions
3. **Create initial qTD(s)**: PID = IN, buffer → internal receive buffer, Active bit set, IOC = 1
4. **Link QH to periodic schedule**: Add QH pointer to appropriate frame list entries based on polling interval
   - 1ms interval → every frame list entry
   - 8ms interval → every 8th entry
   - Simplified: initially use every-frame polling regardless of interval
5. **Flush caches**: Clean QH, qTD, and frame list from D-cache
6. **Return `Imxrt1062InterruptPipe`**: Owns the pipe slot (RAII — returns to pool on Drop)

### `Imxrt1062InterruptPipe::poll_next()` Implementation

1. Register waker with `shared.pipe_wakers[pipe_index]`
2. Invalidate cache for the qTD
3. Check qTD `token` — if Active bit is clear, transfer completed:
   - Copy received data into `InterruptPacket` (up to 64 bytes)
   - Re-arm qTD: set Active bit, reset bytes-to-transfer, flush cache
   - Yield `Poll::Ready(Some(packet))`
4. If still active → `Poll::Pending`
5. Re-enable relevant interrupt in `USBINTR` if needed

### `Drop` for `Imxrt1062InterruptPipe`

- Unlink QH from periodic schedule
- Invalidate/clean cache
- Return QH, qTD(s), and pipe slot to pools
- No async advance doorbell needed for periodic schedule removal (just need frame boundary)

## 2.5 Bulk Transfers (`bulk_in_transfer()` / `bulk_out_transfer()`)

✅ **COMPLETE (Phase 2c)** — bulk IN/OUT working on hardware (MSC sector read confirmed).

- [x] Add `set_overlay_toggle()` to `QueueHead` in `ehci.rs`
- [x] Add `waker_index` field to `TransferComplete` future (was hardcoded to 0 / control pipe)
- [x] Implement `do_bulk_transfer()` shared implementation
- [x] Implement `bulk_in_transfer()` — delegates to `do_bulk_transfer` with `PID_IN, is_in=true`
- [x] Implement `bulk_out_transfer()` — delegates to `do_bulk_transfer` with `PID_OUT, is_in=false`
- [x] Create `examples/rtic_usb_mass_storage.rs` — CBW/CSW READ(10) test via `UsbBus` API
- [x] Hardware validation: sector 0 read confirmed on flash drive

### Design Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Data toggle | DTC=0 in QH (hardware-managed) | Same as interrupt pipe; overlay_token[31] tracks across transfers |
| Toggle initialisation | `set_overlay_toggle()` after `attach_qtd()` | `attach_qtd()` clears overlay_token to 0; must set toggle bit before cache clean |
| Toggle readback | Read `overlay_token bit 31` after async advance doorbell | Reflects next expected toggle; update `data_toggle.set()` |
| qTD count | Single qTD per transfer (up to 5 × 4 KB = ~20 KB) | Sufficient for 512-byte sector reads; no chaining needed |
| ZLP handling | Chain a 0-byte qTD when `!is_in && VariableSize && data_len > 0 && data_len % packet_size == 0` | USB 2.0 §5.8: host must append ZLP when data exactly fills packet count |
| IN buffer cache | `cache::invalidate_dcache_by_address` (DCIMVAC) after transfer | Invalidate-only — avoids writing stale dirty lines over DMA-written data |
| OUT buffer cache | `cache_clean_buffer()` before schedule link | Flushes CPU writes to RAM before DMA reads |
| waker index | `pipe.which() as usize` (= 1, 2, or 3) | Per-pipe waker; avoids spurious wakes; matches interrupt pipe convention |
| QH pool index | `pipe.which() as usize + 1` (= 2, 3, or 4) | Index 0 = sentinel, 1 = control; bulk/interrupt share indices 2–4 |
| `TransferComplete` reuse | Yes — `waker_index` field added | Avoids duplicating polling / error-mapping logic from control transfers |

### `do_bulk_transfer` Step-by-Step

1. `Pipe::new(bulk_pipes.alloc().await, 1)` — acquires a pool slot; `qh_index = which+1`, `waker_idx = which`
2. Read `PORTSC1[PSPD]` for port speed
3. `init_endpoint(qh_characteristics(addr, ep, speed, mps, is_control=false, is_head=false))` — DTC=0
4. Compute `need_zlp = !is_in && VariableSize && data_len > 0 && mps > 0 && data_len % mps == 0`
5. `alloc_qtd()` for data qTD; if `need_zlp`, `alloc_qtd()` for ZLP qTD (free data qTD on failure)
6. `qtd_token(pid, data_len, dt=false, ioc=!need_zlp)` → `data_qtd.init(token, data, data_len)`
7. If ZLP: `qtd_token(pid, 0, dt=false, ioc=true)` → `zlp_qtd.init(..., null, 0)`; chain `data_qtd.next = zlp_qtd`
8. `qh.attach_qtd(data_qtd)` — clears `overlay_token` to 0; then `set_overlay_toggle(data_toggle.get())`
9. OUT only: `cache_clean_buffer(data, data_len)` — flush outgoing data to RAM
10. Clean qTD(s), QH, sentinel; `link_qh_to_async_schedule(qh)`; re-clean QH + sentinel; `enable_async_schedule()`
11. `TransferComplete { status_qtd, data_qtd_opt, waker_index: waker_idx }.await`
    - ZLP case: `status_qtd = zlp_qtd`, `data_qtd = Some(data_qtd)` (for early halted detection)
    - Normal case: `status_qtd = data_qtd`, `data_qtd = None`
12. IN + success: `cache::invalidate_dcache_by_address(data, data_len)`; `cache_clean_qtd(data_qtd)`; `received = data_len - qtd_token_bytes_remaining(token)`
13. OUT + success: `received = data_len`
14. `unlink_qh_from_async_schedule(qh)`; `cache_clean_qh(sentinel)`; `wait_async_advance().await`
15. `cache_clean_qh(qh)`; `new_toggle = overlay_token.read() & (1<<31) != 0`; `data_toggle.set(new_toggle)`
16. `free_qtd(data_qtd_idx)` [+ `free_qtd(zlp_qtd_idx)`]; `qh.sw_flags = 0`; pipe drops → pool slot freed
17. Return `Ok(received)` or `Err(e)`

### `TransferType` Handling

`TransferType` is passed by `UsbBus` but currently affects only OUT transfers:

- **OUT / VariableSize**: ZLP appended when `data_len % packet_size == 0` (see step 4 above).
- **OUT / FixedSize**: No ZLP — the receiver knows the transfer length out-of-band.
- **IN / VariableSize or FixedSize**: Both return `data_len - bytes_remaining` from the qTD token. EHCI will stop on a short packet (device sends fewer bytes than `total_bytes` in the qTD), so the actual received count is always correct. `TransferType` does not change IN behaviour at the EHCI level.

### Known Limitation: UsbBus Hardcodes MPS=64

`UsbBus::bulk_in_transfer` and `bulk_out_transfer` pass `packet_size=64` regardless of the device's actual bulk packet size (marked `@TODO` in cotton-usb-host source). This affects only the QH `max_packet_size` field, not data correctness — EHCI automatically issues multiple 64-byte transactions until the `qTD total_bytes` count is satisfied. For a flash drive with 512-byte sectors over a full-speed connection (64-byte max bulk packet), EHCI will issue 8 transactions of 64 bytes each per sector read, which is correct.

### Risks

- **Toggle readback timing**: `overlay_token` is read after the async advance doorbell. If the hardware has cleared the overlay between transfer completion and the doorbell acknowledgement, the toggle could be wrong. Mitigation: if toggle errors appear on hardware, read the last qTD's own DT bit (`qtd.token >> 31`) instead, which is stable after completion. To be verified on hardware.
- **ZLP on zero-length OUT**: If `data_len == 0` is passed for a VariableSize OUT, the condition `data_len > 0` in `need_zlp` prevents a spurious extra ZLP. The single data qTD sends a natural ZLP.

### Expected Hardware Test Results (`rtic_usb_mass_storage`)

Flash `rtic_usb_mass_storage.hex` with a USB flash drive connected to the USB2 host port.

```
=== imxrt-usbh: USB Mass Storage Example ===
USB2 PLL locked
VBUS power enabled
USB host controller initialised
USB_OTG2 ISR installed (NVIC priority 0xE0)
Entering device event loop...
DeviceEvent::Connect  addr=1  VID=xxxx PID=xxxx class=0
Found MSC interface: class=8 sub=6 proto=0x50 bulk_in=1 (mps=64) bulk_out=2 (mps=64)
Opening bulk endpoints...
Sending CBW READ(10) LBA=0...
CBW sent: 31 bytes
Data received: 512 bytes
Sector 0: eb 58 90 4e 54 46 53 20 20 20 20 00 02 08 00 00
CSW: status=0 (success)
```

The first bytes of sector 0 identify the filesystem: `eb 58 90 4e 54 46 53` = NTFS boot sector,
`eb 58 90 45 58 46 41 54` = exFAT, `eb 3c 90 4d 53 44 4f 53` = FAT32. A blank/unformatted drive
shows `00 00 00 ...`.

After this test passes, also re-run `rtic_usb_hid_keyboard` to confirm interrupt pipes still work
correctly — `do_bulk_transfer` shares the `bulk_pipes` pool and QH slots with interrupt pipes, so
concurrent correctness should be verified.

## Challenges Encountered and Resolutions

### ✅ Challenge: EHCI Complexity vs RP2040 Simplicity

**Problem**: EHCI uses hardware-managed linked lists of descriptors (QH/qTD) with DMA, vs RP2040's simple register-based SIE. The learning curve is steep.

**Resolution**: Built strictly incrementally — async schedule (control, Phase 2a) → periodic schedule (interrupt, Phase 2b) → bulk reuses async (Phase 2c). Used TinyUSB as a clean reference and Linux EHCI driver for edge cases. Key simplification: one QH per active transfer, no QH reuse/sharing.

### ✅ Challenge: Periodic Schedule Setup (Phase 2b)

**Problem**: Interrupt endpoints require the EHCI periodic schedule.

**Resolution**: Implemented simplified flat schedule — 32-entry frame list (all entries point to the same QH chain head), giving 1 ms effective polling for all interrupt endpoints regardless of requested interval. `FRAME_LIST_LEN = 32`, frame list size encoding `FS[2:0] = 0b101`. This is sufficient for keyboard-class devices. Proper binary-tree interval scheduling is deferred to a future phase.

### ✅ Challenge: Async Schedule QH Removal

**Problem**: Cannot directly unlink a QH — hardware may be actively reading it.

**Resolution**: Implemented `AsyncAdvanceWait` future using EHCI's Async Advance Doorbell:
1. Unlink QH (update predecessor's `horizontal_link`)
2. Set `USBCMD[IAA]`
3. Await `USBSTS[AAI]` via `async_advance_waker`
4. Safe to free QH/qTD resources

### ✅ Challenge: Cache Coherency for DMA (Cortex-M7 D-cache)

**Problem**: Cortex-M7 D-cache is write-back. CPU writes to QH/qTD/buffers are not visible to the DMA engine until cleaned; DMA writes to IN buffers are not visible to the CPU until invalidated.

**Resolution** (established in Phase 2b, reused in 2c):
- Before DMA reads (qTD, QH, OUT data buffer): `clean_invalidate_dcache_by_address` (DCCIMVAC)
- After DMA writes (IN data buffer): `invalidate_dcache_by_address` (DCIMVAC) — invalidate-only, never clean+invalidate, to avoid writing stale cache lines over DMA-written data

### ✅ Toggle Readback Reliability (Phase 2c)

**Problem**: `overlay_token bit 31` is read after the async advance doorbell to capture the next expected data toggle. If the controller has already cleared the overlay to 0, this returns the wrong toggle and subsequent transfers will use the wrong DATA0/DATA1 bit, causing `DataSeqError` STALLs.

**Resolution**: Toggle readback from `overlay_token` works correctly on hardware. The MSC sector read uses three consecutive bulk transfers (CBW OUT → Data IN → CSW IN) with data toggle tracking across all three, and all succeed. No toggle-related errors observed.

## Open Questions

1. **Q**: Should periodic schedule use full multi-level tree or simplified flat list?
   **A**: ✅ Resolved (Phase 2b) — Flat 32-entry list, 1 ms polling for all interrupt endpoints. Tree-based scheduling deferred until needed.

2. **Q**: What frame list size should we use?
   **A**: ✅ Resolved (Phase 2b) — 32 entries (`FRAME_LIST_LEN = 32`, `FS[2:0] = 0b101`). Saves memory vs. 1024 entries; 32 ms maximum scheduling granularity is acceptable.

3. **Q**: Is toggle readback from `overlay_token` reliable after async advance doorbell?
   **A**: ✅ Resolved (Phase 2c) — works correctly. MSC CBW/Data/CSW sequence uses toggle tracking across three bulk transfers with no errors.
