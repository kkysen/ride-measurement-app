//! Rust exposed to the app through `uniffi`. Kotlin bindings are generated from this library at build time.

uniffi::setup_scaffolding!();

/// The version of this crate, to check the app is running the Rust it was built with.
#[uniffi::export]
pub fn core_version() -> String {
    env!("CARGO_PKG_VERSION").into()
}
