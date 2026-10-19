# Ride measurement app (Android, personal use)

## Context
Record rides (train, bus, etc.) with the highest-fidelity sensor data possible
(GNSS position, Doppler speed, IMU, audio, optionally video),
then fuse offline for an accurate trajectory/speed profile and mode classification.
Empty repo; greenfield.

## Key decisions
- **Native Android app (Kotlin), not a PWA.**
  A PWA gets only fused `Geolocation` (~1 Hz, no raw GNSS, no Doppler, no timestamps you can trust),
  `DeviceMotion` at browser-chosen rates, and dies when the screen locks/backgrounds.
  Native gives: `GnssMeasurement` (raw pseudorange, pseudorange-rate = Doppler, carrier phase, per-satellite C/N0),
  `GnssNavigationMessage`, `Location` with speed/bearing accuracy,
  `SensorManager` at `SENSOR_DELAY_FASTEST` / `HIGH_SAMPLING_RATE_SENSORS` (~200-500 Hz accel, gyro, mag, baro),
  all timestamped on `elapsedRealtimeNanos`, plus a foreground service so it runs screen-off for hours.
  Turn on Developer options > "Force full GNSS measurements" to disable duty cycling.
- **Split: dumb, lossless logger on phone; fusion offline in Python.**
  Real-time fusion is optional later; offline lets us run a *smoother* (forward + backward),
  which beats any real-time filter, and lets us re-run algorithms on old rides.
- **Future iPhone support**: keep the session format platform-neutral and the fusion pipeline phone-agnostic,
  so an iOS logger later is just a thin Swift app writing the same format.
  iOS has no raw GNSS (only CoreLocation position/speed/course with accuracies), so iPhone rides will be less accurate;
  the pipeline must treat raw GNSS as optional. Not worth Kotlin Multiplatform/Flutter for a logger this thin.

## Phase 1: Logger app
- Foreground service, wake lock, single monotonic clock (`elapsedRealtimeNanos`) for every stream;
  log GNSS clock offset to map to UTC.
- Streams: raw GNSS measurements + nav messages, `Location` (GPS provider and fused),
  accel / gyro / mag / baro / uncalibrated variants, audio (Opus or 16 kHz PCM; low cost, great for mode/events),
  video optional (CameraX, low res/fps, frame timestamps logged) since it is the battery/storage hog.
- Format: one session dir per ride; append-only binary or protobuf-delimited per stream, audio/video in standard containers.
  Optionally also emit the GnssLogger-compatible text format so Google tools (GNSS Analysis, RTKLIB via `gnsslogger_to_rnx`) work directly.
- Calibration: record 10-30 s stationary at start (IMU bias, gravity direction); note phone mounting (pocket vs. fixed changes everything).
- Export via share sheet / adb pull.

## Phase 2: Offline fusion (Python, `numpy`/`scipy`, maybe `gtsam`)
1. **GNSS processing**: raw measurements -> own WLS/PPP-lite or RTKLIB with a nearby CORS base station (free RINEX from NOAA/EUREF/IGS) for sub-meter;
   Doppler -> velocity solution independent of position (typically ~0.1 m/s, better than differencing positions).
   Carrier-phase time-differencing gives cm-level *relative* displacement when phone tracks ADR (many do poorly; check).
2. **Error-state EKF/UKF + RTS smoother**, or factor graph (GTSAM) which handles outages and loop constraints cleanly.
   State: position, velocity, attitude, accel/gyro bias. IMU drives the prediction at high rate;
   GNSS position, Doppler velocity, and baro altitude are measurement updates, weighted by their reported/estimated uncertainty, with outlier gating (multipath in stations/urban canyons).
3. **Vehicle constraints** (the big win for rail): non-holonomic constraint (no lateral/vertical velocity in vehicle frame) after estimating phone-to-vehicle rotation;
   zero-velocity updates at stops (detected from IMU variance + audio).
   For trains, **map-matching to the track graph** from OpenStreetMap (`railway=rail`) reduces the problem to 1-D along-track position; that alone dominates accuracy in tunnels.
   Same idea for buses with road graph + known route.
4. **Tunnels/underground**: pure IMU dead-reckoning drifts quickly (phone MEMS, ~minutes = hundreds of m);
   rely on track constraint + ZUPTs at stations + known station positions + audio/vibration signatures.
5. **Mode classification**: audio spectrogram + IMU vibration spectra (rail joints give periodic clacks whose rate = speed / rail length, an independent speed cue).
   Start with handcrafted features + small classifier; label your own rides.
6. **Video (later)**: visual odometry (e.g. ORB-SLAM3, or simple optical flow for speed) as another velocity factor;
   TrackSight-style localization needs a reference map of features per line, so only worth it for routes you repeat.

## Phase 3 (optional)
Live display of speed/position on phone; ride viewer (map + speed plot) as a local web page.

## Verification
- Logger: compare recorded rates/gaps per stream; check screen-off 1 h ride has no dropouts.
- GNSS: verify Doppler speed vs. position-derived speed agree on open sky; stationary test for noise floor.
- Fusion: drop GNSS for synthetic outages on good data and measure error vs. held-out truth;
  compare against known track geometry and station-to-station distances / timetable.
