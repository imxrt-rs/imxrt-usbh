# Cache Coherency in i.MX RT USB Host Implementation

## Overview

Cache coherency is one of the most critical and challenging aspects of implementing USB host functionality on the i.MX RT 1060/1062 microcontroller. Unlike the RP2040 (which has no data cache), the Cortex-M7 core in i.MX RT chips has a 32KB L1 data cache that can cause subtle and hard-to-debug data corruption issues if not properly managed.

This document explains what cache coherency is, why it matters for USB DMA operations, and how to properly handle it in the `imxrt-usbh` implementation.

## Table of Contents

1. [What is Cache Coherency?](#what-is-cache-coherency)
2. [Why Cache Coherency Matters for USB](#why-cache-coherency-matters-for-usb)
3. [The Cortex-M7 Cache Architecture](#the-cortex-m7-cache-architecture)
4. [USB DMA Data Structures Affected](#usb-dma-data-structures-affected)
5. [Cache Coherency Problems and Symptoms](#cache-coherency-problems-and-symptoms)
6. [Solutions and Best Practices](#solutions-and-best-practices)
7. [Implementation Guidelines](#implementation-guidelines)
8. [Testing and Debugging](#testing-and-debugging)
9. [References](#references)

---

## What is Cache Coherency?

**Cache coherency** is the consistency of shared data that exists in multiple caches. In embedded systems with DMA (Direct Memory Access), the problem arises because there are effectively two entities that can read/write memory:

1. **The CPU** - reads/writes through its cache
2. **DMA peripherals** (like USB controller) - read/write directly to/from main memory, **bypassing the cache**

When these two views of memory diverge, you have a cache coherency problem.

### Simple Example

```
Initial state: Memory[0x1000] = 0x00

CPU writes:
  - CPU writes 0x42 to address 0x1000
  - Value goes into cache, marked as "dirty"
  - Main memory still contains 0x00 (write hasn't flushed yet)

USB DMA reads:
  - USB controller reads from address 0x1000
  - Reads DIRECTLY from main memory (bypasses cache)
  - Gets 0x00 instead of 0x42 ❌ WRONG!

Result: USB controller sees stale data!
```

## Why Cache Coherency Matters for USB

The EHCI USB host controller uses DMA extensively:

### DMA Read Operations (Device → CPU)
The USB controller writes data it receives from USB devices directly into memory:
- **Received data buffers** - incoming packets from USB devices
- **Queue Transfer Descriptor (qTD) status** - completion status, error codes, bytes transferred
- **Queue Head (QH) overlay area** - current transfer state

### DMA Write Operations (CPU → Device)
The USB controller reads data directly from memory to send to USB devices:
- **Transmit data buffers** - outgoing packets to USB devices
- **Queue Transfer Descriptor (qTD)** - transfer parameters, buffer pointers
- **Queue Head (QH)** - endpoint configuration, qTD queue pointers

If the CPU's cache view doesn't match main memory, the USB controller will:
- Send **wrong data** to USB devices
- **Miss** updates the USB controller made to descriptors
- Experience **random failures** that are hard to reproduce

## The Cortex-M7 Cache Architecture

### Cache Specifications (i.MX RT 1060/1062)

- **L1 Data Cache**: 32 KB, 4-way set associative
- **Cache Line Size**: 32 bytes
- **Write Policy**: Write-back (not write-through)
- **Cache is enabled by default** after reset (in typical startup code)

### Write-Back Cache Behavior

The Cortex-M7 uses a **write-back** cache policy:

1. When CPU writes to memory, data goes into cache
2. Cache line is marked as "dirty"
3. Data is NOT immediately written to main memory
4. Dirty line is flushed to memory only when:
   - Cache line is evicted (replaced by another address)
   - Explicit cache flush operation
   - Clean & Invalidate operation

This means **CPU writes are NOT immediately visible to DMA hardware**.

### Cache Line Boundaries

The 32-byte cache line size has important implications:

```c
// Structure layout in memory:
struct Example {
    uint32_t field_a;     // Offset 0x00
    uint32_t field_b;     // Offset 0x04
    // ... more fields ...
    uint32_t field_h;     // Offset 0x1C
    uint32_t next_field;  // Offset 0x20 (starts NEW cache line)
};
```

If a structure spans multiple cache lines, you must manage each affected cache line separately.

### False Sharing Problem

**False sharing** occurs when two logically separate pieces of data share a cache line:

```
Memory Layout:
[0x1000-0x101F]: Cache Line 1
  0x1000-0x1003: CPU variable (frequently written by CPU)
  0x1004-0x101F: DMA buffer (written by USB controller)

Problem:
- USB controller writes new DMA data to 0x1004+ in main memory
- CPU writes to its variable at 0x1000 → entire cache line (0x1000-0x101F) marked dirty
- Cache line is flushed (evicted or explicit clean) → CPU's stale copy of
  0x1004-0x101F overwrites the USB controller's DMA data in main memory!

Alternatively:
- CPU invalidates the cache line to see DMA data → CPU's dirty write to
  0x1000 is discarded (never flushed to main memory)!
```

Either way, one side loses data. The root cause is that cache operations work on
entire 32-byte cache lines — you cannot clean or invalidate a partial line.

**Solution**: Ensure DMA structures are cache-line aligned and don't share cache lines with non-DMA data.

## USB DMA Data Structures Affected

### 1. Queue Head (QH)

The QH is a 48-byte structure (EHCI spec section 3.6):

```rust
#[repr(C, align(64))]
struct QueueHead {
    horizontal_link: u32,           // 0x00
    endpoint_chars: u32,            // 0x04
    endpoint_caps: u32,             // 0x08
    current_qtd: u32,               // 0x0C
    // Overlay area (matches qTD format):
    next_qtd: u32,                  // 0x10
    alt_next_qtd: u32,              // 0x14
    token: u32,                     // 0x18 - USB controller updates this!
    buffer_ptrs: [u32; 5],          // 0x1C-0x2C
    // Total: 48 bytes of EHCI data + 16 bytes padding = 64 bytes
    _padding: [u8; 16],
}
```

**Cache implications**:
- **CPU writes**: QH configuration (endpoint characteristics, qTD pointers)
- **USB controller writes**: Overlay area (token, status) during transfer execution
- **EHCI alignment**: 64-byte aligned (EHCI spec requirement — hardware ignores low 6 address bits)
- **Size**: 48 bytes of EHCI data, padded to 64 bytes = exactly 2 cache lines (0–31, 32–63)
- **Padding**: 16 bytes of padding prevents false sharing with adjacent structures
- **Must**: Flush before activating, invalidate before reading overlay status

### 2. Queue Transfer Descriptor (qTD)

The qTD is a 32-byte structure (EHCI spec section 3.5):

```rust
#[repr(C, align(32))]
struct QueueTransferDescriptor {
    next_qtd: u32,                  // 0x00
    alt_next_qtd: u32,              // 0x04
    token: u32,                     // 0x08 - USB controller updates this!
    buffer_ptrs: [u32; 5],          // 0x0C-0x1F
}
```

**Cache implications**:
- **CPU writes**: All fields when setting up transfer
- **USB controller writes**: Token field (status, bytes transferred)
- **Size**: Exactly 32 bytes = 1 cache line
- **Must**: Align to 32-byte boundary, flush before linking to QH, invalidate before reading status

### 3. Data Buffers

Data buffers for USB transfers:

```rust
// Receive buffer (DMA writes)
let mut rx_buffer = [0u8; 512];

// Transmit buffer (DMA reads)
let tx_buffer = [0x01, 0x02, 0x03, /* ... */];
```

**Cache implications**:
- **RX buffers**: USB controller writes, CPU reads
  - Must invalidate before CPU reads received data
  - Must be cache-line aligned (or waste space with padding)
- **TX buffers**: CPU writes, USB controller reads
  - Must flush before starting DMA transfer
  - Can be any alignment (but alignment helps)

### 4. Periodic Frame List

For interrupt endpoint support:

```rust
#[repr(C, align(4096))]
struct PeriodicFrameList {
    frames: [u32; 1024],  // 4KB, one entry per microframe
}
```

**Cache implications**:
- **CPU writes**: Frame list pointers
- **USB controller reads**: Pointers during periodic schedule traversal
- **Must**: Flush after CPU modifies frame list entries

## Cache Coherency Problems and Symptoms

### Problem 1: Stale Data Sent to USB Device

**Scenario**: CPU prepares transmit buffer, but doesn't flush cache.

```rust
// CPU code:
let mut tx_buffer = [0u8; 64];
tx_buffer[0] = 0x55;  // Goes into cache, not main memory
// ... setup qTD with buffer pointer ...
// USB controller starts transfer
// USB reads main memory → gets 0x00 instead of 0x55 ❌
```

**Symptoms**:
- USB device receives incorrect data
- Transfers appear to complete successfully (no errors reported)
- Data corruption is silent and consistent
- Problem disappears if you add delays or print statements (lucky cache eviction)

### Problem 2: CPU Reads Stale Transfer Status

**Scenario**: USB controller updates qTD status, but CPU reads cached value.

```rust
// USB controller writes to qTD.token in main memory
// CPU reads qTD.token from cache (stale value)
if qtd.token.active() {
    // CPU thinks transfer is still active
    // Actually completed! ❌
}
```

**Symptoms**:
- Transfers appear to never complete (infinite timeouts)
- Status bits show transfer is active when it actually finished
- Polling loops never exit
- Adding cache invalidate "magically fixes" the problem

### Problem 3: CPU Writes Overwrite USB Controller Updates

**Scenario**: False sharing causes CPU write to evict cache line with USB updates.

```rust
// qTD is in cache (clean)
// USB controller updates qTD.token in main memory
// CPU modifies adjacent field in same cache line
// Cache line marked dirty, later flushed
// USB controller's update to token is overwritten! ❌
```

**Symptoms**:
- Intermittent data corruption
- Transfer status mysteriously reset
- Errors appear random and timing-dependent
- Very hard to debug!

### Problem 4: Partial Cache Line Corruption

**Scenario**: DMA structure not aligned, spans partial cache lines.

```rust
// Buffer starts at 0x20001004 (not cache-aligned)
// CPU writes to 0x20001000-0x20001003 (same cache line)
// USB controller writes to 0x20001004+
// Cache operations cause partial corruption
```

**Symptoms**:
- First few bytes of buffer corrupted
- Data corruption at structure boundaries
- Alignment-dependent failures

## Solutions and Best Practices

### Solution 1: Cache Management Operations

The `cortex-m` crate provides cache maintenance methods on `cortex_m::peripheral::SCB`.
All of these methods include DSB and ISB barriers internally — **no manual barrier calls needed**.

#### Clean (Flush)
Writes dirty cache lines back to main memory. **Safe** — cleaning/writing-back is non-destructive.
```rust
// Use before DMA reads from memory (CPU → Device)
scb.clean_dcache_by_address(addr, size);
// Barriers already included — no manual DSB needed
```

#### Invalidate
Marks cache lines as invalid, forcing next read from main memory.
**`unsafe`** — discards any dirty (unflushed) data in the invalidated cache lines!
```rust
// Use before CPU reads DMA-written data (Device → CPU)
// SAFETY: The caller must ensure no dirty data will be lost.
//         This is safe when the region is exclusively DMA-written.
unsafe { scb.invalidate_dcache_by_address(addr, size); }
```

#### Clean & Invalidate
Flushes dirty data to main memory, then invalidates cache lines. **Safe.**
```rust
// Use for bidirectional structures (like QH overlay)
scb.clean_invalidate_dcache_by_address(addr, size);
```

> **Note**: The `cortex-m` SCB methods require `&mut SCB`. In an RTIC application,
> pass `SCB` ownership to the task that manages USB, or use `cortex_m::peripheral::SCB::steal()`
> in `unsafe` contexts (e.g., within the ISR).

### Solution 2: Proper Alignment

All DMA structures must be cache-line aligned:

```rust
#[repr(C, align(64))]  // 64-byte alignment (EHCI requirement + 2 full cache lines)
struct QueueHead {
    // ... fields ...
    _padding: [u8; 16],  // Pad 48-byte QH to 64 bytes
}

#[repr(C, align(32))]  // 32-byte alignment (EHCI requirement = 1 cache line)
struct QueueTransferDescriptor {
    // ... fields ...
}

// For buffers, use alignment or ensure exclusive cache lines
#[repr(align(32))]
struct AlignedBuffer {
    data: [u8; 512],
}
```

### Solution 3: Size Padding

Ensure structures don't partially fill cache lines:

```rust
#[repr(C, align(64))]
struct QueueHead {
    // Actual EHCI fields: 48 bytes (spans 2 cache lines)
    // Pad to 64 bytes (exactly 2 full cache lines)
    // This prevents false sharing AND satisfies the EHCI 64-byte alignment requirement.
    _padding: [u8; 16],
}
```

### Solution 4: Non-Cacheable Memory Regions

i.MX RT allows configuring memory regions as non-cacheable via MPU:

**Advantages**:
- No cache management needed
- Simpler code
- Eliminates coherency bugs

**Disadvantages**:
- Slower CPU access (every access hits main memory)
- Requires MPU configuration
- Reduces available cacheable RAM

**Recommendation**: Use for critical DMA structures (QH, qTD), keep buffers cacheable with proper management.

### Solution 5: Use DTCM (Data Tightly-Coupled Memory)

The i.MX RT 1060 has 1MB of FlexRAM that can be partitioned between:
- **ITCM** (Instruction TCM) — at `0x0000_0000`, for code
- **DTCM** (Data TCM) — at `0x2000_0000`, for data
- **OCRAM** — at `0x2020_0000`, general-purpose (cached by L1 D-cache)

Default Teensy 4.1 partition: 512KB ITCM + 512KB DTCM (0KB OCRAM).
Some configurations use 256KB ITCM + 512KB DTCM + 256KB OCRAM.

Key DTCM properties:
- **Not cached** — single-cycle deterministic access, no coherency issues
- CPU accesses are fast (no cache miss penalty)
- DMA peripherals can access DTCM directly
- Shared with stack and static variables — space is limited

**Recommendation**: Place QH/qTD pools in DTCM (via linker section) if space permits.
This eliminates cache coherency concerns for descriptors entirely. Data buffers
(caller-provided, potentially large) would remain in OCRAM with cache management.

> **Linker script example** (place in a `.dtcm_dma` section):
> ```rust
> #[link_section = ".dtcm_dma"]
> static QH_POOL: ConstStaticCell<[QueueHead; 4]> = ConstStaticCell::new(...);
> ```

## Implementation Guidelines

### Guideline 1: Cache Management Wrapper Functions

Create utility functions for all cache operations:

```rust
/// Clean (flush) cache for DMA read (CPU write → Device read)
#[inline]
unsafe fn cache_clean(addr: *const u8, size: usize) {
    cortex_m::asm::dsb();
    
    let start = (addr as usize) & !0x1F;  // Align down to cache line
    let end = ((addr as usize) + size + 31) & !0x1F;  // Align up
    
    for line_addr in (start..end).step_by(32) {
        // Use SCB cache maintenance registers
        // SCB_DCCMVAC: Data Cache Clean by MVA to PoC
        core::ptr::write_volatile(
            0xE000EF6C as *mut u32,
            line_addr as u32
        );
    }
    
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Invalidate cache for CPU read (Device write → CPU read)
#[inline]
unsafe fn cache_invalidate(addr: *const u8, size: usize) {
    cortex_m::asm::dsb();
    
    let start = (addr as usize) & !0x1F;
    let end = ((addr as usize) + size + 31) & !0x1F;
    
    for line_addr in (start..end).step_by(32) {
        // SCB_DCIMVAC: Data Cache Invalidate by MVA to PoC
        core::ptr::write_volatile(
            0xE000EF5C as *mut u32,
            line_addr as u32
        );
    }
    
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}
```

### Guideline 2: Cache Management at Transfer Boundaries

**Before starting a transfer**:
```rust
// Setup qTD
qtd.next = ...;
qtd.token = ...;
qtd.buffer_ptrs[0] = tx_buffer.as_ptr() as u32;

// Flush TX buffer (CPU wrote to it)
cache_clean(tx_buffer.as_ptr(), tx_buffer.len());

// Flush qTD (CPU wrote to it)
cache_clean(&qtd as *const _ as *const u8, size_of::<QTD>());

// Link qTD to QH
qh.next_qtd = &qtd as *const _ as u32;

// Flush QH (CPU wrote to it)
cache_clean(&qh as *const _ as *const u8, size_of::<QH>());

// Now USB controller can safely read all structures
```

**After transfer completes**:
```rust
// SAFETY: qTD is exclusively DMA-written at this point (CPU is not writing to it),
// so invalidation won't discard any dirty CPU data.
unsafe { cache_invalidate(&qtd as *const _ as *const u8, size_of::<QTD>()); }

// Check status
if qtd.token.active() {
    // Still running
} else {
    // Completed — invalidate RX buffer to read received data.
    // SAFETY: RX buffer was not written by CPU since the transfer started.
    unsafe { cache_invalidate(rx_buffer.as_ptr(), rx_buffer.len()); }
    
    // Now safe to read received data
    let received_data = &rx_buffer[..bytes_transferred];
}
```

### Guideline 3: Defensive Cache Operations

When in doubt, add more cache operations:
- **Cost**: ~10-50 CPU cycles per cache line
- **Benefit**: Eliminates subtle bugs
- **Optimize later**: Profile first, optimize only if necessary

```rust
// Defensive approach for bidirectional structures.
// Use clean_invalidate instead of bare invalidate for the overlay area,
// because the CPU may have written to other fields in the same cache lines.
unsafe fn update_qh_overlay(qh: &mut QueueHead) {
    // Before reading: clean+invalidate to flush any CPU writes AND
    // pick up the USB controller's latest updates
    cache_clean_invalidate(
        qh as *const _ as *const u8,
        size_of::<QueueHead>()
    );
    
    // Read status
    let status = qh.overlay.token;
    
    // ... process status ...
    
    // Before writing new values
    qh.overlay.next_qtd = new_qtd_ptr;
    
    // After writing: clean to push CPU writes to main memory
    cache_clean(
        qh as *const _ as *const u8,
        size_of::<QueueHead>()
    );
}
```

### Guideline 4: Document Cache Requirements

Every DMA structure should document cache requirements:

```rust
/// Queue Head for EHCI transfers.
///
/// # Cache Coherency Requirements
///
/// - Must be 64-byte aligned (EHCI hardware requirement)
/// - Padded to 64 bytes (2 full cache lines) to prevent false sharing
/// - CPU must CLEAN after modifying endpoint characteristics or qTD pointers
/// - CPU must CLEAN+INVALIDATE before reading overlay area status
///   (use clean+invalidate rather than bare invalidate because the CPU
///   may have dirty data in the same cache lines as the overlay)
/// - USB controller updates overlay area (token, buffer pointers) during transfers
#[repr(C, align(64))]
pub struct QueueHead {
    // ... 48 bytes of EHCI fields ...
    _padding: [u8; 16],  // Pad to 64 bytes
}
```

## Testing and Debugging

### Testing Strategy

1. **Test with cache disabled**: Verify functionality without cache coherency issues
   ```rust
   // In startup code (testing only!)
   let mut cp = cortex_m::Peripherals::take().unwrap();
   cp.SCB.disable_dcache(&mut cp.CPUID);
   ```

2. **Test with cache enabled, no management**: Verify problems appear without cache management

3. **Test with cache enabled, proper management**: Verify problems are fixed

4. **Stress testing**: Rapid transfers, concurrent endpoints, hot-plug/unplug

### Debugging Techniques

#### Check if cache coherency is the issue:

```rust
// Add explicit cache operations and see if problem disappears
cache_clean_invalidate_all();  // Nuclear option
cortex_m::asm::dsb();
```

#### Verify alignment:

```rust
let addr = &my_qh as *const _ as usize;
assert_eq!(addr & 0x1F, 0, "QH not cache-line aligned!");
```

#### Add memory barriers:

```rust
// DMB orders CPU memory accesses but does NOT flush/invalidate cache.
// For DMA-updated data, you need cache_invalidate(), not just DMB.
// DMB is useful for ordering: e.g., write descriptor THEN write doorbell.
cortex_m::asm::dmb();  // Data Memory Barrier
let status = qtd.token;  // Still reads from cache! Use cache_invalidate first.
```

#### Dump cache statistics:

```rust
// Read cache performance counters (if available)
// Check for excessive cache misses
```

### Common Debug Pitfalls

1. **Adding delays "fixes" the problem**: Probably cache issue (delay causes eviction)
2. **Adding prints "fixes" the problem**: Print causes cache flush/memory access
3. **Works on first try, fails on subsequent tries**: Cache lines still valid from first try
4. **Works with optimizations off, fails with optimizations on**: Optimizer keeps data in registers/cache

## Performance Considerations

### Cache Operation Costs

- **Clean single cache line**: ~10 cycles
- **Invalidate single cache line**: ~10 cycles
- **Clean entire 32KB cache**: ~10,000 cycles
- **Clean by address range**: ~10 cycles × (size / 32)

### Optimization Strategies

1. **Batch cache operations**: Clean entire qTD chain at once, not one at a time
2. **Align to cache boundaries**: Reduces partial cache line operations
3. **Use TCM for hot paths**: Zero cache management overhead
4. **Profile before optimizing**: Measure actual impact

### When NOT to Optimize

- During initial implementation (correctness first!)
- When cache operations are <1% of transfer time
- When optimization makes code significantly more complex

## References

### Official Documentation

1. **ARM Cortex-M7 Technical Reference Manual**
   - Chapter 6: Cache and TCM
   - Section 6.2: Cache maintenance operations

2. **i.MX RT 1060 Reference Manual**
   - Chapter 3: Cortex-M7 Memory System
   - Cache configuration and MPU

3. **ARM Cache Maintenance Operations**
   - ARM®v7-M Architecture Reference Manual
   - Section B3.12: System Control Block

### Code Examples

1. **`cortex-m` crate**: `SCB::clean_dcache_by_address()`, `SCB::invalidate_dcache_by_address()`, etc.
   - Repository: https://github.com/rust-embedded/cortex-m
   - These are the primary cache maintenance functions we should use.
2. **Zephyr RTOS**: `arch/arm/core/cortex_m/mpu/arm_mpu.c` (MPU-based non-cacheable regions)
3. **NuttX**: `arch/arm/src/armv7-m/arm_cache.c` (C reference for cache line operations)
4. **Teensyduino USBHost_t36**: https://github.com/PaulStoffregen/USBHost_t36
   - C++ USB host driver for the same i.MX RT 1062 hardware; see how it handles cache

---

## Quick Reference Card

| Operation | When to Use | Cache Function |
|-----------|-------------|----------------|
| **Clean** | Before USB DMA reads from memory (CPU → Device) | `cache_clean()` |
| **Invalidate** | Before CPU reads USB-written data (Device → CPU) | `unsafe { cache_invalidate() }` |
| **Clean + Invalidate** | Bidirectional structures (QH overlay) | `cache_clean_invalidate()` |
| **DSB** | Included automatically by `cortex-m` SCB methods | `cortex_m::asm::dsb()` |
| **DMB** | Ordering between CPU memory accesses (not for DMA coherency) | `cortex_m::asm::dmb()` |

> ⚠️ **DMB vs cache operations**: A DMB (Data Memory Barrier) only orders CPU memory
> accesses relative to each other. It does **not** flush or invalidate the cache.
> For DMA coherency, you must use explicit cache clean/invalidate operations.
> DMB is useful for ensuring the CPU sees its own writes in order (e.g., writing a
> descriptor before writing the doorbell register), but it cannot make DMA-written
> data visible to the CPU.

## Summary Checklist

For every DMA structure in your USB host implementation:

- [ ] Structure meets EHCI alignment (`#[repr(C, align(64))]` for QH, `#[repr(C, align(32))]` for qTD)
- [ ] Structure size is padded to a multiple of cache line size (prevents false sharing)
- [ ] CPU cleans cache after writing structure (before DMA reads it)
- [ ] CPU invalidates cache before reading USB-updated fields (after DMA writes it)
- [ ] Invalidate calls are `unsafe` with documented safety rationale
- [ ] No false sharing with non-DMA data
- [ ] Barriers handled by `cortex-m` SCB methods (no manual DSB/ISB needed)
- [ ] Documented cache coherency requirements in doc comments
- [ ] Tested with cache both enabled and disabled

**Remember**: Cache coherency bugs are subtle, timing-dependent, and hard to reproduce. Defensive cache management is worth the small performance cost!

---

**Document Version**: 1.1  
**Date**: 2025-10-06  
**Last Updated**: 2026-02-06
