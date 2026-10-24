#!/usr/bin/env bash
# Sets the app's version (`versionName`, and the next `versionCode`) for a new build on the phone,
# and the Rust core's (`crates/ffi/Cargo.toml`) to the same, if any Rust changed since the last bump:
# the core's version is the app version that last changed it.
set -euo pipefail

new="${1:?usage: bump-version.sh <version, e.g. 0.1.12>}"
cd "$(dirname "$0")/.."
gradle=android/app/build.gradle.kts
ffi=crates/ffi/Cargo.toml

last_bump=$(git log -1 --format=%H -G 'versionName = ' -- "$gradle")
code=$(sed -nE 's/^ *versionCode = ([0-9]+)$/\1/p' "$gradle")
sed -i -E "s/^( *versionCode = )[0-9]+$/\1$((code + 1))/; s/^( *versionName = )\".*\"$/\1\"$new\"/" "$gradle"
echo "app: $new (versionCode $((code + 1)))"

# What the core is built from, as `crates/ffi/build.rs` hashes it, plus the bindings themselves.
if git diff --quiet "$last_bump" -- crates proto Cargo.lock; then
    echo "Rust core: unchanged since $(git log -1 --format=%h "$last_bump"), stays $(sed -nE 's/^version = "(.*)"$/\1/p' "$ffi")"
else
    sed -i -E "0,/^version = \".*\"$/s//version = \"$new\"/" "$ffi"
    cargo metadata --format-version 1 > /dev/null # updates `Cargo.lock`
    echo "Rust core: $new"
fi
