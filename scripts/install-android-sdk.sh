#!/usr/bin/env bash
# Installs the Android SDK command-line tools and the packages the app needs into `$ANDROID_HOME`.
set -euo pipefail

cmdline_tools_build=16111833
platform=android-37
build_tools=36.0.0

: "${ANDROID_HOME:?set ANDROID_HOME (see mise.toml)}"
sdkmanager="$ANDROID_HOME/cmdline-tools/latest/bin/sdkmanager"

if [[ ! -x "$sdkmanager" ]]; then
    download_dir="$ANDROID_HOME/.download"
    mkdir -p "$download_dir" "$ANDROID_HOME/cmdline-tools"
    zip="$download_dir/cmdline-tools.zip"
    curl -fL -o "$zip" "https://dl.google.com/android/repository/commandlinetools-linux-${cmdline_tools_build}_latest.zip"
    rm -rf "$download_dir/cmdline-tools"
    unzip -q "$zip" -d "$download_dir"
    rm -rf "$ANDROID_HOME/cmdline-tools/latest"
    mv "$download_dir/cmdline-tools" "$ANDROID_HOME/cmdline-tools/latest"
    rm -rf "$download_dir"
fi

"$sdkmanager" --install "platform-tools" "platforms;$platform" "build-tools;$build_tools"
