# Phase 3: Interrupt Handling and Async Support

**Estimated effort**: 2-3 days  
**Key milestone**: Reliable operation, no corruption

## 3.1 Interrupt Handler (`UsbShared::on_irq()`)

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

- [ ] Implement `on_irq()` as shown above
- [ ] Add `async_advance_waker: CriticalSectionWakerRegistration` to `UsbShared`
- [ ] Bind to NVIC IRQ: `USB_OTG2` (IRQ #112)
- [ ] RTIC task binding example:
  ```rust
  #[task(binds = USB_OTG2, shared = [&usb_shared], priority = 2)]
  fn usb_host_irq(cx: usb_host_irq::Context) {
      cx.shared.usb_shared.on_irq();
  }
  ```

## 3.2 Waker Registration Pattern

Following the RP2040's pattern with `CriticalSectionWakerRegistration` (from `rtic-common`):

- [ ] In each `Future::poll()` / `Stream::poll_next()`:
  1. Register waker: `shared.pipe_wakers[my_index].register(cx.waker())`
  2. Re-enable relevant interrupt bits in `USBINTR` (counterpart to disable-on-handle in ISR)
  3. Check for completion (cache-invalidate first, then read QH/qTD status)
  4. If not done → `Poll::Pending`
  5. If done → clean up and `Poll::Ready(result)`
- [ ] Ensure waker registration happens *before* re-enabling interrupts to avoid race conditions
- [ ] Device detection waker: `shared.device_waker`
- [ ] Per-pipe wakers: `shared.pipe_wakers[pipe_index]`
- [ ] Async advance waker: `shared.async_advance_waker`

## 3.3 DMA and Cache Coherency

See [CACHE_COHERENCY.md](../CACHE_COHERENCY.md) for full details. Summary of required operations:

- [ ] Implement cache management wrapper functions:

  ```rust
  /// Clean (flush) a memory range from D-cache to main memory.
  /// Call BEFORE the USB controller needs to READ this memory.
  /// (CPU wrote data → hardware needs to see it)
  fn cache_clean(addr: *const u8, size: usize);
  
  /// Invalidate D-cache for a memory range.
  /// Call BEFORE the CPU needs to READ data that hardware WROTE.
  /// (Hardware wrote data → CPU needs to see it)
  fn cache_invalidate(addr: *const u8, size: usize);
  
  /// Clean and invalidate — used when both CPU and hardware may have modified memory.
  fn cache_clean_invalidate(addr: *const u8, size: usize);
  ```

- [ ] Use `cortex_m::asm::dsb()` and `cortex_m::asm::dmb()` barriers around DMA operations
- [ ] Cache operation call sites:
  - **Before linking QH/qTD to schedule**: `cache_clean()` on QH, all qTDs, and any OUT data buffers
  - **Before reading QH/qTD status in poll()**: `cache_invalidate()` on the QH overlay and qTDs
  - **Before reading IN data**: `cache_invalidate()` on the receive data buffer
  - **After re-arming interrupt qTD**: `cache_clean()` on the qTD
- [ ] Avoid cache-line aliasing: ensure no non-DMA data shares a cache line (32 bytes) with DMA structures
- [ ] **Alternative approach**: Consider placing QH/qTD pools in DTCM (addresses 0x2000_0000 – 0x2007_FFFF) which is not cached. This eliminates cache coherency concerns for descriptors entirely, at the cost of using limited DTCM space. Data buffers would still need cache management.

## Challenges for This Phase

### Challenge: Cache Coherency (Cortex-M7 specific)

**Problem**: The Cortex-M7's 32KB L1 write-back D-cache means DMA structures in SRAM are not automatically visible to the USB controller (and vice versa). This causes silent data corruption.

**Solution** (choose one, or combine):
- **Option A: Per-operation cache management** (recommended for initial implementation)
  - `cache_clean()` before hardware reads (CPU → DMA)
  - `cache_invalidate()` before CPU reads (DMA → CPU)
  - Safer but more code, small performance cost
- **Option B: Place DMA structures in DTCM** (0x2000_0000 region, not cached)
  - Eliminates cache concerns for QH/qTD pools
  - Requires linker script changes to place structures in DTCM
  - Data buffers (caller-provided) still need cache management
  - Limited DTCM space (512KB shared with stack)
- **Option C: Configure MPU to mark DMA regions as non-cacheable**
  - Clean separation, no per-operation overhead
  - More complex setup, inflexible region sizes
- Add cache-line alignment padding to prevent false sharing between DMA and non-DMA data

### Challenge: EHCI Transfer Completion Identification

**Problem**: When `USBSTS[USBINT]` fires, EHCI doesn't tell you *which* QH/qTD completed. You must scan all active QHs.

**Solution**:
- Wake all pipe wakers on any completion interrupt (simple, correct, slightly wasteful)
- Each pipe's `poll()` checks its own QH/qTD status
- This matches the RP2040 pattern where completion interrupts wake potentially-affected wakers
- Future optimization: maintain a "dirty" bitmap of active pipes, scan only those
