set shell := ["mise", "exec", "--", "bash", "-euo", "pipefail", "-c"]

fixture := justfile_directory() / "android/app/build/fixtures/kotlin-session"
apk := justfile_directory() / "android/app/build/outputs/apk/debug/app-debug.apk"

# Build everything and run every test, including the Kotlin-to-Rust/Python format cross-checks.
test: test-rust test-android test-cross

test-rust:
    cargo fmt --check
    cargo clippy --all-targets -- -D warnings
    cargo test

# Android unit tests; also writes the Kotlin fixture session.
test-android:
    cd android && ./gradlew :app:testDebugUnitTest

test-cross: test-android python-env
    RIDE_KOTLIN_FIXTURE={{fixture}} cargo test --test kotlin_fixture
    RIDE_KOTLIN_FIXTURE={{fixture}} python/.venv/bin/pytest -q python

python-env:
    cd python && uv venv -q --allow-existing .venv && uv pip install -q --python .venv maturin pytest && uv pip install -q --python .venv -e .

apk:
    cd android && ./gradlew :app:assembleDebug

# Install over wireless debugging (`adb pair` and `adb connect` first; see README).
install: apk
    adb install -r {{apk}}

# Copy sessions off the phone into `data/sessions/`; never deletes anything on the phone.
pull:
    mkdir -p data
    adb pull /sdcard/RideLogger/sessions data/

summary session:
    cargo run -q --bin ride -- summary {{session}}

plot session stream:
    python/.venv/bin/python -m ride.plot {{session}} {{stream}}

# Export a session's audio to `<session>/audio.wav` for listening.
wav session:
    cargo run -q --bin ride -- wav {{session}} {{session}}/audio.wav
