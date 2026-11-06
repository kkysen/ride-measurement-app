set shell := ["mise", "exec", "--", "bash", "-euo", "pipefail", "-c"]

fixture := justfile_directory() / "android/app/build/fixtures/kotlin-session"
apk := justfile_directory() / "android/app/build/outputs/apk/debug/app-debug.apk"
# A drive from Google's Smartphone Decimeter Challenge 2023 with a Samsung phone, and that day's broadcast orbits.
sdc_drive := "2021-04-08-21-28-us-ca-mtv-k/sm-g988b"
sdc := justfile_directory() / "data/sdc" / sdc_drive
sdc_orbits := "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2021/098/BRDM00DLR_S_20210980000_01D_MN.rnx.gz"

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

# Solve a real drive with ground truth, per constellation (downloads it first; see `fetch-sdc`).
test-sdc: fetch-sdc
    RIDE_SDC_DRIVE={{sdc}} cargo test -p ride-gnss --test sdc -- --nocapture

# The position filter on the same drive with GNSS cut out, from its IMU (see `crates/fusion/tests/sdc_position.rs`).
test-sdc-position: fetch-sdc
    RIDE_SDC_DRIVE={{sdc}} cargo test --release -p ride-fusion --test sdc_position -- --nocapture

# Download the drive into `data/sdc/` (not committed: Kaggle's rules).
# Needs a Kaggle API token in `~/.kaggle/access_token` and the competition's rules accepted on its Data tab.
fetch-sdc:
    mkdir -p {{sdc}}
    cd {{sdc}} && for f in supplemental/gnss_log.txt ground_truth.csv device_imu.csv; do \
        [ -e "$(basename $f)" ] || uvx --from kaggle kaggle competitions download -q \
            -c smartphone-decimeter-2023 -f sdc2023/train/{{sdc_drive}}/$f -p .; \
    done
    cd {{sdc}} && if [ -e gnss_log.txt.zip ]; then unzip -oq gnss_log.txt.zip && rm gnss_log.txt.zip; fi
    cd {{sdc}} && [ -e orbits.rnx ] || curl -sfL {{sdc_orbits}} | gunzip > orbits.rnx

python-env:
    cd python && uv venv -q --allow-existing .venv && uv pip install -q --python .venv maturin pytest && uv pip install -q --python .venv -e '.[experiments]'

apk:
    cd android && ./gradlew :app:assembleDebug

# Start the `adb` server, on localhost only: under WSL's mirrored networking `adb` hangs instead of starting it.
adb-server:
    @if timeout 3 adb devices >/dev/null 2>&1; then echo "adb server already running"; \
    else (nohup adb nodaemon server start > data/work/adb-server.log 2>&1 &) ; sleep 1; timeout 3 adb devices; fi

# Set the app's version for a new build on the phone, and the Rust core's to it if the Rust changed since.
bump version:
    scripts/bump-version.sh {{version}}

# Install over wireless debugging (`just adb-server`, `adb pair` and `adb connect` first; see README).
install: apk
    @unzip -l {{apk}} | rg -q 'lib/arm64-v8a/libride_ffi.so' || (echo "the APK has no Rust core (libride_ffi.so); not installing" && exit 1)
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

# Write `<session>/map.html`: the fused path (joined to the track underground) colored by speed, the GPS fixes as dots, with your manual events marked.
map session:
    cargo run -q --bin ride -- map {{session}}

# Download broadcast orbits for every constellation into `<session>/orbits/`, following the same plan as the app
# (`ride orbit-plan`): per UTC day, the complete merged file if it's out (the day after), else BKG's 15-minute one.
orbits session:
    cargo run -q --bin ride -- orbit-plan {{session}} | while read -r -a pairs; do \
        for ((i = 0; i < ${#pairs[@]}; i += 2)); do \
            f=${pairs[i]}; mkdir -p "$(dirname "$f")"; \
            if curl -sfL "${pairs[i+1]}" -o "$f.part"; then mv "$f.part" "$f"; break; fi; \
            rm -f "$f.part"; \
        done; \
    done

# How well raw GNSS solves for velocity (navigation messages, ephemerides, residuals, own vs phone speed).
gnss session *flags:
    cargo run -q --bin ride -- gnss {{session}} {{flags}}

# Run an analysis example on every session in `data/sessions/` but those in `scripts/sweep-skip.txt`,
# `$SWEEP_JOBS` at a time (default 4), each capped at `$SWEEP_CAP` (default 1200M; see `scripts/sweep.sh`).
# A session that goes over the cap is killed and reported, and the sweep goes on.
sweep example *args:
    cargo build -q --release -p ride-cli --example {{example}}
    scripts/sweep.sh {{example}} {{args}}

# Every score (see `crates/cli/examples/score/main.rs`) on every session: each session's in `data/work/score/`.
score *args:
    just sweep score {{args}}
