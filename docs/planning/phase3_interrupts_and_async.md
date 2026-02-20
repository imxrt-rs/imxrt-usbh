# Phase 3: Interrupt Handling and Async Support

**Estimated effort**: 2-3 days  
**Key milestone**: Reliable operation, no corruption
**Status**: ✅ COMPLETE — all items implemented during phases 1–2c

## 3.1 Interrupt Handler (`UsbShared::on_irq()`) — ✅ COMPLETE

Implemented during phases 1–2 as part of getting transfers working.

Following the RP2040's disable-on-handle pattern to prevent IRQ storms:

```rust
pub fn on_irq(&self) {
    // 1. Read USBSTS (active interrupt flags)
    let status = read_reg!(usb, USB2, USBSTS);
    
    // 2. Clear only the bits we handle (W1C register)
    write_reg!(usb, USB2, USBSTS, status & HANDLED_MASK);
    
    // 3. Disable handled interrupts (re-enabled by poll() callers)
    modify_reg!(usb, USB2, USBINTR, &= !status);
    
    // 4. Wake appropriate wakers based on which interrupts fired:
    if status & PCI_BIT != 0 {
        // Port Change Interrupt → device connect/disconnect
        self.device_waker.wake();
    }
    if status & (USBINT_BIT | USBERRINT_BIT) != 0 {
        // Transfer completion or error → wake all active pipe wakers
        // (Could be smarter and only wake the relevant pipe, but EHCI
        //  doesn't tell you which QH completed — need to check each)
        for waker in &self.pipe_wakers {
            waker.wake();
        }
    }
    if status & AAI_BIT != 0 {
        // Async Advance Interrupt → QH safely unlinked, wake waiters
        // (Used during QH removal from async schedule)
        self.async_advance_waker.wake();
    }
}
```

### Important Design Note

EHCI `USBSTS[USBINT]` fires on *any* qTD completion with IOC set, but doesn't identify *which* QH/qTD completed. The simplest correct approach is to wake all pipe wakers and let each Future re-check its own QH/qTD status. This matches how Linux's EHCI driver scans all active QHs on completion interrupts.

### Checklist

- [x] Implement `on_irq()` — `UsbShared::on_irq()` in `src/host.rs`
- [x] Add `async_advance_waker: CriticalSectionWakerRegistration` to `UsbShared`
- [x] Bind to NVIC IRQ: `USB_OTG2` (IRQ #112) — all examples manually install ISR at priority 0xE0
- [x] Re-enable-on-poll helpers: `reenable_interrupt()`, `reenable_transfer_interrupts()`, `reenable_async_advance_interrupt()`
- [x] Public API: `on_usb_irq(usb_base: *const ())` avoids exposing RAL types to app code

Note: Examples use manual ISR installation (vector table patching) rather than
RTIC `#[task(binds = USB_OTG2)]` because RTIC's dispatcher uses a different
interrupt than USB_OTG2. The manual approach is functionally equivalent.

## 3.2 Waker Registration Pattern — ✅ COMPLETE

Implemented during phases 2a–2c. All futures and streams follow the
register-waker → re-enable-interrupt → check-status pattern.

- [x] In each `Future::poll()` / `Stream::poll_next()`:
  1. Register waker: `shared.pipe_wakers[my_index].register(cx.waker())`
  2. Re-enable relevant interrupt bits in `USBINTR` (counterpart to disable-on-handle in ISR)
  3. Check for completion (cache-invalidate first, then read QH/qTD status)
  4. If not done → `Poll::Pending`
  5. If done → clean up and `Poll::Ready(result)`
- [x] Ensure waker registration happens *before* re-enabling interrupts to avoid race conditions
  - `TransferComplete::poll()`: registers waker at line 1577, re-enables interrupts at line 1603
  - `Imxrt1062DeviceDetect::poll_next()`: registers waker at line 1762, re-enables at `reenable_interrupt()`
  - `Imxrt1062InterruptPipe::poll_next()`: registers waker at line 1877, re-enables at line 1892
  - `AsyncAdvanceWait::poll()`: registers waker at line 1653, re-enables at `reenable_async_advance_interrupt()`
- [x] Device detection waker: `shared.device_waker`
- [x] Per-pipe wakers: `shared.pipe_wakers[pipe_index]`
- [x] Async advance waker: `shared.async_advance_waker`

## 3.3 DMA and Cache Coherency — ✅ COMPLETE

Implemented during phases 1–2c. Uses Option A (per-operation cache management).

- [x] Implement cache management wrapper functions in `src/cache.rs`:
  - `invalidate_dcache_by_address()` — invalidate-only (DCIMVAC), for DMA receive buffers
  - `clean_invalidate_dcache_by_address()` — clean+invalidate (DCCIMVAC), for QH/qTD/OUT buffers
  - Both include DSB barriers before and after, plus ISB
- [x] Higher-level wrappers in `Imxrt1062HostController`:
  - `cache_clean_qh()` — clean+invalidate a QueueHead (64B, 64B-aligned)
  - `cache_clean_qtd()` — clean+invalidate a TransferDescriptor (32B, 32B-aligned)
  - `cache_clean_buffer()` — clean+invalidate an arbitrary data buffer
- [x] `cortex_m::asm::dsb()` barriers embedded inside cache functions
- [x] Cache operation call sites:
  - **Before linking QH/qTD to schedule**: `cache_clean_qh()` + `cache_clean_qtd()` + `cache_clean_buffer()` in `do_control_transfer()` and `do_bulk_transfer()`
  - **Before reading QH/qTD status in poll()**: `cache_clean_qtd()` + `cache_clean_qh()` in `TransferComplete::poll()` and `Imxrt1062InterruptPipe::poll_next()`
  - **Before reading IN data**: `invalidate_dcache_by_address()` on receive buffer in `Imxrt1062InterruptPipe::poll_next()` and bulk IN completion
  - **After re-arming interrupt qTD**: `cache_clean_invalidate_dcache_by_address()` in interrupt pipe re-arm
- [x] Cache-line aliasing prevention:
  - `QueueHead` is `#[repr(C, align(64))]` — occupies exactly one 64B-aligned region (2 cache lines)
  - `TransferDescriptor` is `#[repr(C, align(32))]` — occupies exactly one cache line
  - `RecvBuf` is `#[repr(C, align(32))]` with 64B size — no aliasing with adjacent data
  - `FrameList` is `#[repr(C, align(4096))]`
- [x] **Alternative approach (DTCM)**: Not pursued. Per-operation cache management is working correctly.

## Challenges for This Phase — All Resolved

### Challenge: Cache Coherency (Cortex-M7 specific) — ✅ Resolved

**Problem**: The Cortex-M7's 32KB L1 write-back D-cache means DMA structures in SRAM are not automatically visible to the USB controller (and vice versa). This causes silent data corruption.

**Solution implemented**: Option A — per-operation cache management.
- `clean_invalidate_dcache_by_address()` before hardware reads (CPU → DMA)
- `invalidate_dcache_by_address()` before CPU reads (DMA → CPU)
- All DMA structures (QH, qTD, RecvBuf) are cache-line-aligned to prevent aliasing
- Working correctly with both HID keyboard and mass storage transfers

### Challenge: EHCI Transfer Completion Identification — ✅ Resolved

**Problem**: When `USBSTS[USBINT]` fires, EHCI doesn't tell you *which* QH/qTD completed. You must scan all active QHs.

**Solution implemented**:
- Wake all pipe wakers on any completion interrupt (simple, correct, slightly wasteful)
- Each pipe's `poll()` checks its own QH/qTD status
- This matches the RP2040 pattern where completion interrupts wake potentially-affected wakers
