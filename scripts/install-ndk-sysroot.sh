#!/usr/bin/env bash
# Installs just the target-side parts of the Android NDK into `$ANDROID_HOME/ndk-sysroot/<release>/`:
# the sysroot (headers and library stubs) and the ARM64 compiler runtime (`libclang_rt.builtins`, `libunwind`).
#
# Google ships the NDK's compilers for x86-64 Linux only, so on ARM64 Linux, Rust and C code for Android
# is built with the host's own `clang` and `lld` against these, which are plain files and work on any host.
set -euo pipefail

ndk_release=r30

: "${ANDROID_HOME:?set ANDROID_HOME (see mise.toml)}"
dest="$ANDROID_HOME/ndk-sysroot/$ndk_release"
if [[ -d "$dest/sysroot" && -d "$dest/runtime" ]]; then
    echo "already installed: $dest"
    exit 0
fi

download_dir="$ANDROID_HOME/.download"
mkdir -p "$download_dir"
zip="$download_dir/android-ndk-$ndk_release-linux.zip"
if [[ ! -f "$zip" ]]; then
    curl -fL -o "$zip.part" "https://dl.google.com/android/repository/android-ndk-$ndk_release-linux.zip"
    mv "$zip.part" "$zip"
fi

extract="$download_dir/ndk-extract"
rm -rf "$extract"
mkdir -p "$extract"
prefix="android-ndk-$ndk_release"
toolchain="$prefix/toolchains/llvm/prebuilt/linux-x86_64"
unzip -q "$zip" "$toolchain/sysroot/*" "$toolchain/lib/clang/*/lib/linux/*" "$prefix/source.properties" -d "$extract"
rm -rf "$dest"
mkdir -p "$dest/runtime"
mv "$extract/$toolchain/sysroot" "$dest/sysroot"
# `libclang_rt.builtins-aarch64-android.a` and friends, and `aarch64/libunwind.a`.
mv "$extract/$toolchain"/lib/clang/*/lib/linux/*aarch64* "$dest/runtime/"
mv "$extract/$prefix/source.properties" "$dest/"
rm -rf "$extract" "$zip"
echo "installed: $dest"
