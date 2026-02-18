//! Optional logging.
//!
//! When the `log` feature is enabled, the `info!`, `warn!`, `debug!`, and
//! `trace!` macros expand to `log::info!` etc. When `defmt-03` is enabled
//! (and `log` is not), they expand to `defmt::info!` etc. When neither
//! feature is enabled, they expand to nothing.

#![allow(unused)]

// --- log crate backend ---

#[cfg(feature = "log")]
macro_rules! trace {
    ($($args:tt)*) => { log::trace!($($args)*) };
}

#[cfg(feature = "log")]
macro_rules! debug {
    ($($args:tt)*) => { log::debug!($($args)*) };
}

#[cfg(feature = "log")]
macro_rules! info {
    ($($args:tt)*) => { log::info!($($args)*) };
}

#[cfg(feature = "log")]
macro_rules! warn {
    ($($args:tt)*) => { log::warn!($($args)*) };
}

// --- defmt backend (only if log is not enabled) ---

#[cfg(all(feature = "defmt-03", not(feature = "log")))]
macro_rules! trace {
    ($($args:tt)*) => { { use defmt_03 as defmt; defmt::trace!($($args)*) } };
}

#[cfg(all(feature = "defmt-03", not(feature = "log")))]
macro_rules! debug {
    ($($args:tt)*) => { { use defmt_03 as defmt; defmt::debug!($($args)*) } };
}

#[cfg(all(feature = "defmt-03", not(feature = "log")))]
macro_rules! info {
    ($($args:tt)*) => { { use defmt_03 as defmt; defmt::info!($($args)*) } };
}

#[cfg(all(feature = "defmt-03", not(feature = "log")))]
macro_rules! warn {
    ($($args:tt)*) => { { use defmt_03 as defmt; defmt::warn!($($args)*) } };
}

// --- no-op (neither feature enabled) ---

#[cfg(not(any(feature = "log", feature = "defmt-03")))]
macro_rules! trace {
    ($($args:tt)*) => {};
}

#[cfg(not(any(feature = "log", feature = "defmt-03")))]
macro_rules! debug {
    ($($args:tt)*) => {};
}

#[cfg(not(any(feature = "log", feature = "defmt-03")))]
macro_rules! info {
    ($($args:tt)*) => {};
}

#[cfg(not(any(feature = "log", feature = "defmt-03")))]
macro_rules! warn {
    ($($args:tt)*) => {};
}
