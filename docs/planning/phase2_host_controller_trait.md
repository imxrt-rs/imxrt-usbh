# Phase 2: Core HostController Trait Implementation

**Estimated effort**: 5-7 days  
**Key milestones**:
- ✅ Phase 2a: `GET_DESCRIPTOR` works (3-4 days)
- ✅ Phase 2b: HID keyboard input received (2-3 days) — COMPLETE, working on hardware
- Phase 2c: USB flash drive sector read (2-3 days)

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

- [ ] Implement `bulk_in_transfer()` method
- [ ] Implement `bulk_out_transfer()` method

### Bulk IN Transfer Flow

1. Allocate pipe from `statics.bulk_pipes` pool
2. Build qTD(s) for incoming data: PID = IN, buffer → caller's `data` slice, IOC = 1
   - For large transfers: chain multiple qTDs (each can reference up to 5 × 4KB = 20KB via 5 buffer pointers)
   - Set data toggle from `data_toggle: &Cell<bool>` parameter
3. Configure QH: device address, endpoint, max packet size, bulk type
4. Flush caches, link QH to async schedule
5. Wait for completion (same pattern as control transfer)
6. On success: update `data_toggle` cell, return bytes transferred
   - `TransferType::VariableSize` — a short packet (< max_packet_size) signals end of transfer, return actual bytes
   - `TransferType::FixedSize` — expect exactly `data.len()` bytes, short packet is an error
7. Unlink QH (async advance doorbell), free resources

### Bulk OUT Transfer Flow

1. Same structure as IN but: PID = OUT, buffer → caller's `data` slice
2. Handle ZLP (Zero Length Packet) if data length is a multiple of max packet size (per USB 2.0 spec)
3. Data toggle management same as IN

### Data Toggle Notes

- The `data_toggle: &Cell<bool>` is managed by the caller across transfers
- EHCI QH can track data toggle in hardware (overlay `token[DT]` bit)
- Set `DTC = 0` in QH endpoint characteristics to let QH track toggle, OR `DTC = 1` to take toggle from qTD
- For bulk: use `DTC = 0` (QH tracks toggle), initialize QH overlay toggle from `data_toggle.get()`, and read it back after transfer completes to update `data_toggle.set()`

## Challenges for This Phase

### Challenge: EHCI Complexity vs RP2040 Simplicity

**Problem**: EHCI uses hardware-managed linked lists of descriptors (QH/qTD) with DMA, vs RP2040's simple register-based SIE. The learning curve is steep.

**Solution**:
- Start with async schedule only (control transfers, phase 2a). Periodic schedule (interrupt pipes) comes next (phase 2b), then bulk (phase 2c).
- Bulk transfers reuse the async schedule infrastructure already built for control transfers.
- Reference TinyUSB's EHCI driver (`src/portable/ehci/ehci.c`) — it's a clean, minimal implementation.
- Use Linux EHCI driver (`drivers/usb/host/ehci-hcd.c`, `ehci-q.c`) as authoritative reference for edge cases.
- Key simplification: use only one QH per active transfer (no QH reuse/sharing).

### Challenge: Periodic Schedule Setup

**Problem**: Interrupt endpoints require the EHCI periodic schedule — a frame list in memory with QH pointers organized by polling interval.

**Solution**:
- **Initially**: Simplified periodic schedule — 1024-entry frame list, all entries point to same QH chain (effectively 1ms polling for all interrupt endpoints regardless of requested interval)
- **Later**: Proper interval-based scheduling with binary tree structure
- Frame list must be 4KB-aligned (1024 entries × 4 bytes)
- Alternatively, use smaller frame list (256 entries) with `USBCMD[FS]` bits to save memory
- Link interrupt QHs into the frame list; they form a linked list per frame via `horizontal_link`

### Challenge: Async Schedule QH Removal

**Problem**: You cannot simply unlink a QH from the async schedule — the hardware may be actively reading it. Premature removal causes undefined behavior.

**Solution**:
- Use EHCI's Async Advance Doorbell mechanism:
  1. Unlink QH from the circular list (point previous QH's `horizontal_link` around it)
  2. Set `USBCMD[IAA]` (Interrupt on Async Advance doorbell)
  3. Wait for `USBSTS[AAI]` interrupt — this guarantees the hardware has advanced past the removed QH
  4. Now it's safe to free/reuse the QH
- Implement an `async_advance_waker` in `UsbShared` for this purpose

## Open Questions

1. **Q**: Should periodic schedule use full multi-level tree or simplified flat list?
   **A**: Start with flat list (all interrupt QHs polled every frame = 1ms). Implement tree-based scheduling only if bandwidth becomes an issue. **Decision point**: Phase 2b.

2. **Q**: What frame list size should we use?
   **A**: Start with 256 entries (1KB, requires `USBCMD[FS] = 0b10`). Smaller than the default 1024 saves memory with minimal impact on scheduling granularity. **Decision point**: Phase 2b.
