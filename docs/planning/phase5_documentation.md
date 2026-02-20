# Phase 5: Documentation and Polish

**Estimated effort**: 1-2 days  
**Key milestone**: Complete docs and examples

## Checklist

- [ ] Add comprehensive `///` doc comments to all public types and methods
- [ ] Document the required initialization order (clocks → PHY → controller → mode → schedules → interrupts)
- [ ] Document the `PERIODICLISTBASE`/`DEVICEADDR` register alias workaround
- [ ] Document cache coherency requirements (link to [CACHE_COHERENCY.md](../CACHE_COHERENCY.md))
- [ ] Document DMA buffer alignment requirements
- [ ] Add `# Safety` sections for any `unsafe` code blocks (raw pointer DMA, cache operations, register aliasing)
- [ ] Update README.md with working usage example and hardware setup
- [ ] Add `# Panics` documentation for any functions that can panic
- [ ] Consider adding `defmt` feature flag for debug logging (already in Cargo.toml dependencies)
- [ ] Add another example that shows how to process multiple devices (say a mouse and keyboard) through a hub.
- [ ] Document limitation around hubs and HS devices
