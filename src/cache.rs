//! Cache maintenance operations.
//!
//! On `target_os = "none"` (ARM embedded), these perform real D-cache
//! clean/invalidate via Cortex-M CBP registers. On host targets (for
//! testing), they are no-ops since there is no DMA engine.
//!
//! The ARM implementation was adapted from the cortex-m (0.7.1) crate.
//! cortex-m only lets you access these functions when you have the SCB
//! in `cortex_m::Peripherals`. We duplicate the routines we need so the
//! driver doesn't need to own or steal the peripheral collection.
//!
//! See <https://github.com/rust-embedded/cortex-m/issues/304> and
//! <https://github.com/rust-embedded/cortex-m/pull/320>.

// ---------------------------------------------------------------------------
// Host target: no-op stubs (no DMA engine)
// ---------------------------------------------------------------------------

#[cfg(not(target_os = "none"))]
pub fn invalidate_dcache_by_address(_addr: usize, _size: usize) {}

#[cfg(not(target_os = "none"))]
pub fn clean_invalidate_dcache_by_address(_addr: usize, _size: usize) {}

// ---------------------------------------------------------------------------
// ARM embedded target: real cache maintenance
// ---------------------------------------------------------------------------

/// Invalidates D-cache by address (discard only, no write-back).
///
/// * `addr`: The address to invalidate.
/// * `size`: The number of bytes to invalidate.
///
/// Invalidates D-cache starting from the first cache line containing `addr`,
/// finishing once at least `size` bytes have been invalidated.
///
/// **Warning**: Any dirty data in the invalidated cache lines is **discarded**
/// without being written back to main memory. This is correct for DMA receive
/// buffers (where RAM contents written by hardware are authoritative), but
/// must NOT be used for data the CPU has modified and not yet flushed.
///
/// It is recommended that `addr` is aligned to the cache line size and `size`
/// is a multiple of the cache line size, otherwise surrounding data will also
/// be invalidated.
#[cfg(target_os = "none")]
pub fn invalidate_dcache_by_address(addr: usize, size: usize) {
    // No-op zero sized operations
    if size == 0 {
        return;
    }

    // Safety: write-only registers, pointer to static memory
    let cbp = unsafe { &*cortex_m::peripheral::CBP::PTR };

    cortex_m::asm::dsb();

    // Cache lines are fixed to 32 bytes on Cortex-M7
    const LINESIZE: usize = 32;
    let num_lines = ((size - 1) / LINESIZE) + 1;

    let mut addr = addr & 0xFFFF_FFE0;

    for _ in 0..num_lines {
        // Safety: write to Cortex-M write-only register — DCIMVAC (invalidate only)
        unsafe { cbp.dcimvac.write(addr as u32) };
        addr += LINESIZE;
    }

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Cleans and invalidates D-cache by address.
///
/// * `addr`: The address to clean and invalidate.
/// * `size`: The number of bytes to clean and invalidate.
///
/// Cleans and invalidates D-cache starting from the first cache line containing `addr`,
/// finishing once at least `size` bytes have been cleaned and invalidated.
///
/// It is recommended that `addr` is aligned to the cache line size and `size` is a multiple of
/// the cache line size, otherwise surrounding data will also be cleaned.
///
/// Cleaning and invalidating causes data in the D-cache to be written back to main memory,
/// and then marks that data in the D-cache as invalid, causing future reads to first fetch
/// from main memory.
#[cfg(target_os = "none")]
pub fn clean_invalidate_dcache_by_address(addr: usize, size: usize) {
    // No-op zero sized operations
    if size == 0 {
        return;
    }

    // Safety: write-only registers, pointer to static memory
    let cbp = unsafe { &*cortex_m::peripheral::CBP::PTR };

    cortex_m::asm::dsb();

    // Cache lines are fixed to 32 bit on Cortex-M7 and not present in earlier Cortex-M
    const LINESIZE: usize = 32;
    let num_lines = ((size - 1) / LINESIZE) + 1;

    let mut addr = addr & 0xFFFF_FFE0;

    for _ in 0..num_lines {
        // Safety: write to Cortex-M write-only register
        unsafe { cbp.dccimvac.write(addr as u32) };
        addr += LINESIZE;
    }

    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}
