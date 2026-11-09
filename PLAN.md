# Ride measurement app (Android first, personal use)

## Context
Record rides (train, bus, etc.) with the highest-fidelity sensor data possible,
primarily for offline analysis, with some lightweight live analysis on the phone.
Offline results are authoritative: anything computed live can be recomputed better later (smoother, more context).
Audio and video are deferred to later improvements.
Many rides start in or just after tunnels, so the design must not depend on an initial GNSS fix,
and must not trust network (cell/WiFi) location, which can report the wrong underground station.

## Platform
- **Native Android app (Kotlin), not a PWA.** A PWA gets only fused, low-rate `Geolocation` (no raw GNSS, no Doppler),
  throttled `DeviceMotion`, no barometer/cell/WiFi data, and is killed when the screen locks.
- **Install by sideloading**, no Play Store review: enable developer mode, install without a cable via wireless debugging (`adb pair` / `adb connect`, Android 11+),
  or by downloading the APK from a GitHub release; USB only as a fallback.
  (Google is rolling out developer identity verification for sideloaded apps on certified devices;
  `adb`/Android Studio installs to your own phone stay available, and friends would need either that or a free hobbyist registration.)
- **iPhone later**: keep the session format and fusion pipeline phone-agnostic so an iOS logger is a thin Swift app.
  iOS has no raw GNSS and no cell/WiFi scan APIs, so iPhone rides are less accurate; raw GNSS is optional in the pipeline.
  Install: free Apple ID signing (re-sign every 7 days) for yourself, or $99/yr developer account + TestFlight for friends.

## Rust core for performance-sensitive and shared code
- Platform shells stay thin (Kotlin on Android, Swift on iOS later): sensor callbacks, UI, foreground service.
- Shared Rust crate for heavy or reused algorithms: live EKF, rail-joint detection, map-matching.
  Built for Android with `cargo-ndk`, bindings to Kotlin and Swift via `uniffi`,
  and to Python via `pyo3`/`maturin`, so live and offline processing run the exact same code.
- The logger stays in Kotlin: it is mostly Android API calls and file appends,
  and crossing into Rust per sensor sample would cost more than it saves.
- The session format is defined by a `.proto` schema, not a Rust library, so every language reads it natively.
- Rust from the start for everything that must be correct and identical everywhere:
  the session reader, format validation, lossless round-trip tests, and the summary/report CLI, exposed to Python via `pyo3`.
- Python (on top of the Rust reader) for exploration, plots, and first drafts of fusion algorithms, where fast iteration matters.
  Proven algorithms are ported to Rust so phone and offline share them; slow long-running ones (smoothing hours of IMU) move early.

## Phase 1: Logger app (main focus)
Built incrementally: a barebones end-to-end pipeline first, then filled in one stream/feature at a time,
each milestone installable and usable on a real ride.
1. End-to-end skeleton, every stage present but minimal:
   - App: Kotlin + Jetpack Compose, start/stop button, foreground service + wake lock, one stream (accelerometer).
   - Format: minimal `.proto` schema and chunked `zstd` writer, one session directory per recording.
   - Transfer: upload finished sessions to OneDrive (see Storage), no cable.
   - Offline: Rust crate that reads sessions, with a CLI printing a per-stream rate/gap summary;
     thin Python package using it via `pyo3` to plot a stream.
   - Tests: round-trip test that writer output decodes bit-exactly; build + tests runnable from one script.
2. Uncalibrated gyro and accel, batched, with gap detection.
3. GNSS: raw measurements, clock, nav messages, status, location fixes.
4. Barometer, magnetometer, light, proximity, battery/temperature.
5. Cell info, service state, WiFi scans (only without GNSS).
6. Manual event buttons (boarded, at station, alighted).
7. Post-ride verified recompress, export, fuller per-stream report.

Foreground service + wake lock, runs screen-off for hours.
One clock for everything: `elapsedRealtimeNanos`; also log GNSS clock to map to UTC.

Streams:
- **GNSS raw** (`GnssMeasurement`): pseudorange, pseudorange-rate (Doppler, gives velocity directly, ~0.1 m/s),
  ADR (accumulated delta range = carrier phase, cm-level relative displacement when the chip tracks it),
  per-satellite C/N0; plus `GnssNavigationMessage`, `GnssStatus`, `GnssClock`.
  Enable Developer options > "Force full GNSS measurements" to disable **duty cycling**
  (the chip otherwise powers down between fixes to save battery, causing gaps and resetting carrier phase).
- **Location fixes**: GPS provider (with speed/bearing accuracy) and fused provider, logged but not trusted blindly.
- **IMU** at max rate (`HIGH_SAMPLING_RATE_SENSORS`, ~400 Hz): uncalibrated accel, gyro, mag only (they include the bias estimates);
  calibrated variants and rotation vector are derivable, so they are not stored.
- **Barometer**: altitude changes (1 hPa is about 8 m; phone sensors resolve a few cm to ~1 m of relative height over minutes),
  so underground platforms 10-30 m deep are 1-4 hPa below street level and each station's depth is a fingerprint.
  Tunnel entry/exit pressure waves scale with speed squared, so they are strong on HSR and weaker but often visible on subways,
  along with piston-effect swings as trains move through tunnels and into station boxes.
  Confounders: weather drift (slow, ~1 hPa/h at worst), car HVAC pressurization, doors opening. To validate on real subway rides.
- **Cell info** (`CellInfo`: cell IDs, signal strength) and **WiFi scans** (BSSIDs, RSSI):
  recorded as raw fingerprints, never as a position. We build our own station fingerprint map from labeled past rides.
  Cell info is passive: `requestCellInfoUpdate` returns what the modem already sees (serving + neighbor cells),
  so poll it every few seconds; it does not make the modem search and costs little.
  Also log `ServiceState` and `SignalStrength` changes (in/out of service, RAT, registered cell),
  which measures time-to-reconnect at each station, a useful dataset in itself.
  Normal apps cannot trigger a network search (`requestNetworkScan` needs carrier/system privileges) or toggle airplane mode.
- **Other cheap signals**: light sensor (tunnel detection), proximity (in pocket), battery/temperature.
  Step detection is derivable from accel, so it is not stored.
- **Manual event button**: tap "boarded", "at station X" (picked from nearby OSM stations), "alighted".
  Cheap and the most reliable underground ground truth.
- Optional toggle: audio, deferred by default, see rail joints note.
  Mono 16 kHz lossless FLAC is ~50-70 MB/h, roughly doubling the session size;
  Opus at ~24 kbps is ~10 MB/h but lossy, which conflicts with never losing data, so FLAC if recorded at all.
  Audio records conversations nearby, so it stays local and off by default.

Format: one directory per session, one file per stream, layout described in the Storage section.
GnssLogger-compatible text (for Google's tools and RTKLIB) is generated by an export tool, not stored.
Calibration: 10-30 s stationary at start when possible (IMU bias, gravity). Export by cloud upload (see Storage).

## Storage
Rule: store only what cannot be recomputed later; everything derived is regenerated offline.
- Stored: uncalibrated IMU, raw GNSS + nav messages + clock, chip GPS fixes and fused fixes
  (both use inputs we lack, and are tiny), barometer, cell/WiFi scans, light, proximity, battery/temperature, manual events.
- Not stored: calibrated IMU, rotation vector, step detector, GnssLogger text, anything the live EKF computes.
- Encoding for high-rate streams: columnar chunks of a few seconds each.
  Every encoding must be bit-exact lossless: decoding returns the exact `float` bits Android delivered.
  Values: XOR with the previous value's bits (Gorilla-style) or byte-shuffle, then compress;
  timestamps: delta of delta, zigzag, then compressed.
  Integer quantization to `Sensor.getResolution` is only an optimization, never trusted:
  phones often misreport resolution and calibration rescales values off clean steps,
  so a chunk is quantized only if every value round-trips bit-exactly, otherwise it is stored as raw bits.
  Low-rate streams: length-delimited protobuf messages, compressed per chunk.
- Compression on the fly with `zstd` at a low level, one independent frame per chunk.
  At ~30 KB/s raw this costs negligible CPU, avoids a post-ride pass and the peak disk usage of uncompressed files,
  and a crash loses at most the current chunk. The downside is a slightly worse ratio than one big high-level pass.
- Post-ride recompress on the phone (while charging, or right after the ride): rewrite each stream as one file at a high `zstd` level.
  Never delete data before its replacement is verified: write the new file, `fsync`, decode it fully,
  compare every record against the original chunks (or a content hash of the decoded records), and only then delete the chunks.
  Any failure keeps the chunks and flags the session; sessions are also deleted from the phone only after the copy off it is verified the same way.
  Benchmark `zstd` vs `lz4` vs domain codecs on real rides before freezing the format; the `.proto` schema versions it.
- Rough size: ~80-100 MB/h before encoding, est. ~20-40 MB/h after; about 1 GB/month at 1 h/day.
- Transfer: the app uploads finished sessions to cloud storage behind a small backend interface
  (upload file resumably, get the remote hash), so providers are swappable.
  First backend: OneDrive via Microsoft Graph (MSAL sign-in, `Files.ReadWrite.AppFolder` scope so it only touches its own app folder,
  upload sessions for large files; needs a free Azure app registration). Google Drive or an S3-style bucket can be added later.
  Runs in the background via `WorkManager`, on unmetered WiFi by default, retried until done.
  The phone deletes a session only after every file's remote hash (`quickXorHash` on OneDrive) matches one computed locally.
  The PC pulls from OneDrive (or its synced folder) for analysis.

## Battery
- Main costs: GNSS with duty cycling disabled, and the CPU waking for every 400 Hz sample.
- Use sensor batching (`maxReportLatencyUs` of a few seconds) so the hardware FIFO buffers samples and the CPU sleeps between batches;
  this is likely the biggest single saving. Write to disk in chunks, never per sample.
  Batching is lossless: same samples, same hardware timestamps, just delivered later.
  The one risk is FIFO overflow (dropped samples), so keep latency well under `Sensor.getFifoMaxEventCount` / rate
  (the FIFO is often shared between sensors), and detect gaps from timestamps; on any gap, shorten the latency automatically.
- WiFi scans are throttled by Android (4 per 2 min in the foreground); scan only while GNSS is lost,
  prioritizing when stopped, since a stop underground is almost always a station.
- Station detection, live: a stop (low IMU variance for several seconds) near an OSM station on the matched line;
  underground, a stop plus dwell length plus the along-track distance since the last known station picks the candidate.
  Offline, the HMM in Phase 3 makes the final call.
- Pass-through stations (express trains): predicted from the dead-reckoned along-track position against OSM stations,
  so the WiFi scan is scheduled to fire just before the predicted pass, widening the window as position uncertainty grows.
  Confirmed by signatures of passing through a station box: pressure step from the larger cross-section,
  light sensor brightening, a short burst of new WiFi BSSIDs, and switches/crossovers in the gyro just outside many stations.
  Each pass becomes a position fix offline, the same as a stop but weaker.
- Screen off during recording; live display is opt-in.
- Expect single-digit percent per hour; measure with `dumpsys batterystats` on real rides and record battery in the session.

## Phase 2: Live analysis (light)
Forward-only error-state EKF on the phone: IMU prediction, Doppler velocity and GPS position updates, ZUPTs.
Shows speed, along-track distance, current/next station. Results logged, but overwritten by offline processing.

## Phase 3: Offline fusion (prototype in Python on the Rust reader, proven parts in Rust)
1. **GNSS**: own WLS from raw measurements, or RTKLIB with a nearby CORS base station (free RINEX) for sub-meter;
   Doppler velocity; time-differenced ADR where available.
2. **Smoother**: error-state EKF + RTS backward pass, or factor graph. The backward pass is what solves tunnel starts:
   the first good fix after exiting propagates back through the tunnel segment.
3. **Vehicle constraints**: estimate phone-to-vehicle rotation, then non-holonomic constraint (no sideways/vertical velocity);
   **ZUPTs** (zero-velocity updates: when stopped, assert velocity = 0, which resets IMU drift) at stops detected from IMU variance.
4. **Map-matching to OSM track graph** (`railway=*`): reduces the train problem to 1-D along-track position;
   bus uses road graph. Station identity resolved by an HMM over the graph using inter-station distance (from integrated speed),
   dwell timing, barometer, own WiFi/cell fingerprints, and manual taps;
   network location enters only as a weak, outlier-gated hint, so a misconfigured station cell cannot override the rest.
   OSM / OpenRailwayMap geometry is only roughly accurate (worse underground), so it is a prior with uncertainty, not a hard constraint:
   the graph topology (which tracks connect, station order) is trusted far more than coordinates.
   Repeated rides refine our own track map: jointly estimate track geometry (curvature from gyro, length from speed)
   and station positions across all rides of a line, SLAM-style, with OSM as the starting point.
5. **Rail joints**: periodic vertical jolts at rate = speed / rail length (often 12.5-25 m) give a GNSS-free speed estimate.
   Accel first: it measures the car body's jolt directly at 400 Hz, free since we record it anyway;
   it is damped when the phone is in a pocket or hand. Audio catches the higher-frequency clack and is less sensitive to how the phone is held,
   but also hears everything else. Expect accel to be enough on jointed track; add audio if the jolts get lost in damping.
   Caveat: continuously welded rail has no joints, but switches/expansion joints still give occasional landmarks.
6. **Curves and switches**: all derived from stored IMU + fused speed, nothing extra to record.
   With phone-to-car rotation known (from straight track), speed `v`, yaw rate `w` (gyro), and car-frame accel `a_lat`, `a_vert`:
   centripetal accel `v*w`, radius `v/w` (cross-checked against OSM geometry),
   car roll `phi = atan2(v*w, g) - atan2(a_lat, a_vert)`, cant `= track gauge (~1500 mm between rail centers) * sin(phi)`,
   and unbalanced lateral accel / cant deficiency = the lateral accel felt in the car.
   Caveat: the car body leans outward on its suspension in proportion to unbalanced accel, which biases cant.
   Separate them with repeat rides through the same curve at different speeds (cant is fixed, deficiency scales with `v^2`),
   or directly when stopped on a curve (`phi` is then pure cant plus a tiny lean).
   Speeds through a given curve are often similar, which makes a per-curve fit ill-conditioned,
   so fit jointly across all curves: one suspension roll coefficient per train type shared by every curve, plus one cant per curve.
   The spread of deficiency across many curves pins the coefficient; each curve's cant then needs only a few passes.
   Also note the speed variation within each pass (accelerating/braking through the curve), which adds leverage.
   Switches: short yaw-rate S-curves and lateral jerk; record diverging vs straight route and speed.
   Build a per-curve database (radius, cant, max/typical speed, deficiency) from repeated rides.
7. **Door events**: only checked while stopped.
   Barometer: a small pressure step or transient when the car's HVAC-pressurized air meets the platform.
   Audio (if recorded): door chime, door motors, and the HVAC sound changing as outside air and noise come in.
   Also a small car sway in the IMU as passengers board. Gives dwell start/end and confirms a real station stop vs a signal hold.
8. **Mode classification** (later): IMU vibration spectra, then audio.
9. **Video** (later): visual odometry as a velocity factor; TrackSight-style localization only for routes repeated often.

## Findings from the first subway rides (2026-10-20) and what to build from them
Rides: `151449` (3 express, 7 under the East River), `154951` (elevated express 7), `184134` (7, R, M, F via 53 St),
`214427` (2 express from 42 St, 1 local). Prototypes in `python/experiments/`, on CSVs from `crates/cli/examples/`.

Done in the live speed (`crates/fusion`):
- Gravity follows tilt (30 s with GNSS, subtracting the fused acceleration; 600 s without), travel axis from GNSS
  speed changes (precision-weighted, only above 3 m/s), turned only by the phone's tilting, re-initialization from the median
  of rejected GNSS; whole-ride average speed in the app.
- Subway top speed (55 mph) as a projection that leaves the covariance alone (`Fusion::set_max_speed`).
  Not yet set live, and it should end at "Alighted".

Signals, what they showed, and how to use them:
- **Wi-Fi and cell underground**: MTA Wi-Fi (BoldynWiFi, OFMTA1XWIFI, NYCTA, Passpoint) and a usable cell signal
  appear only at stations. Every underground station showed up, including express pass-throughs (0 to 15 s windows)
  and stops the user didn't mark; stops last 35 s and more. A delayed pass can be slow, so a stop needs a door opening,
  not just a long window. Access points identify a station across rides (52 shared between two rides at 96 St).
  Cell between stations exists in some tunnels: 14 St (L), Joralemon (4, 5; AT&T and Verizon), G Court Sq to Hoyt-Schermerhorn (AT&T),
  North River and East River (Amtrak, LIRR, NJT), 63 St lower level and Atlantic Av (LIRR), Lexington north of 59 St soon,
  and every future CBTC section; there, use Wi-Fi, not cell. Read the carrier from the recorded cells.
- **Naming stations without labels**: the last GNSS fix before going underground gives the entrance station,
  then the sequence of stops and passes, their timing, and the chime type pin the line, direction, and express/local
  against the line's station list. Each named station records its access points, so later rides recognize it directly.
  A sudden jump of several stations (e.g. backward) is a mislabeled station: reject it, report it, and correct the access point.
- **Door chime**: a ~36 dB tone 3 s before each "doors closed" press, 441/555 Hz on the 7, 2, F, R, M;
  645/516 Hz on the 1, so it identifies at least some car types. More precise than the button.
- **Lurch before a stop**: braking ends with a step of 0.5 m/s^2 and more, then a flat dwell; clear at some stops,
  messy in crowds. A rule found 5 of 13 stops with 1 false; not ported. Combine with the chime, radio, and power.
- **Vibration (accel 1-25 Hz, audio 20-300 Hz)**: AUC 0.98 for stopped vs moving on the elevated 7, but useless underground with the phone in hand,
  and `151449`'s audio came from a headphone mic. Recheck with the phone mic (AC turns up at stations on new trains).
- **Magnetometer**: traction current shows as a field change roughly proportional to motoring effort
  (median 109 uT at +0.5 m/s^2 and over, 33 uT coasting); braking looks like coasting.
  Not yet a usable acceleration or binary powering signal in the phone frame (precision 0.4 to 0.6):
  rotate into the world frame with the gyro, learn the gain while GNSS is available, then use it underground as
  "not motoring, so not accelerating" and as a phase signal. Balises can't be seen (passive, MHz, read only by the train).
- **Inverter (VVVF) sound**: fixed lines at 385, 1418, 1924 Hz are auxiliary or HVAC; a 1 kHz line at 5 to 20 km/h on the 7 may be
  the traction inverter's fixed carrier. To do: spectrogram ridge tracking for lines whose pitch follows speed, a GNSS-free speed.
- **OMNY taps**: the reader's beep is a pure tone near 2.85 to 3.0 kHz, about 0.25 s, at a stair transition (pressure).
  All three Google Wallet taps on 2026-10-20 match: Flushing 18:56:03 (2852 Hz, `184134`), Grand Av 19:16:02 (2945 Hz, `184134`),
  Times Sq 21:45:06.8 (2984 Hz, `214427`; fainter same-pitch beeps after it are other riders).
  A station-entry signal for the fusion, with the station's Wi-Fi APs; to do: a detector, checked on more taps.
- **Train beacons**: a BLE beacon family (manufacturer `02FD`, ID prefix `e2c56db5-dffb-48d2-b060-d0f5a710`), one per car,
  on CBTC-equipped or CBTC-ready cars: R188 (7), R160 (F, M, R), R211 (C; also a few R179s there); none on
  R62, R62A, R142 (1, 2, 3; the R142 is an NTT, from 2000, without them). Gives exact boarding/alighting and the car;
  short sightings are other trains. Decode the varying bytes. Piper's BLE beacons (B Division, not the L) power countdown clocks,
  on trains or stations; not yet identified. Phones and trackers (Apple, Find My Device `0x40` Eddystone) are riders.
- **Crowding**: BLE device counts (300 to 600 on platforms), but the unfiltered scan pauses with the screen off, so normalize for that;
  the user's Aranet4 (in the BLE data) gives CO2 (640 to 1180 ppm on a crowded 2), temperature, humidity, and a second barometer.
  Decoded by `ride_fusion::aranet` (`crates/cli/examples/co2.rs` per reading). Over 694 readings with the screen on, CO2 follows
  near devices (≥ -80 dBm, 30 s): Spearman 0.55, median 570 ppm under 50 devices, 970 at 150 to 200, 1025 over 300.
  Cars differ more than crowds explain (ventilation): 2050 ppm on a 2 (Oct 21, 6:28 pm) and 1720 on a 7 at ~190 devices,
  700 on the C at 80. Transfer passages: 1230 to 1600 ppm at 14 St to the L (Oct 23, 7:47 pm) with only 76 devices.
  To do: fetch the device's stored history over GATT, for screen-off stretches (the filtered scan drops its advertisements).
- **Pressure**: 0.6 m noise; stair descents are clear; tunnel pressure waves are large (a "-95 m" dip under the East River),
  so a grade term needs them rejected. Battery temperature is useless (phone heat, 1 minute steps).
- **Operation**: detect CBTC (beacon, line, smooth spacing) and ATO vs ATPM (ATO's repeatable jerk-limited profile and stopping point;
  ATPM's notched, hunting speed). Out of Flushing on `184134`, speed hunted 56 to 60 km/h, unlike the smooth peaks later:
  consistent with the first stop out of a terminal being ATPM. Needs labeled rides.

Next, in order:
1. Stations into the fusion: stops as zero speed (chime, radio, lurch, power), station-to-station distances from the
   line's geometry, entrance station from the last fix. Wi-Fi, cell, and needed audio features go into Rust so live matches replay.
2. Line and station identification from the stop/pass sequence; an access point database built from rides.
3. Subway mode, detected from the sensors, through the settle queue; the 1.4 m/s^2 bound only if it helps
   (the runaway came from 0.2 to 0.4 m/s^2 of bias).
   Not from the marks: they're labels for calibrating and checking, never inputs (user, 2026-10-21;
   a version driven by them was built and reverted). Detect the subway from the sensors instead:
   position against the subway map, station Wi-Fi, train and station Bluetooth, cell service patterns,
   pressure, sound (traction, chimes), vibration. The marks then score the detection.
   Built (2026-10-21): `SubwayDetector`, from the subway's Wi-Fi or no fix among few networks;
   the speed is held to 55 mph while in it, live and in replay, and every ride's smoothed top speed is now 55 mph.
   It misses elevated lines (GNSS keeps the speed in check there); adding position on the subway map would catch them.
   More signals scored (`python/experiments/subway_signals.py`, AUC train vs off): the magnetometer's spread is the best
   (0.76 to 0.91 on every ride; median 13 to 31 uT on trains, 3 to 6 on the street); the train beacon (0.78 to 0.98 on
   CBTC and C/D/E rides, absent on the 1, 2, 3); a fix within 25 m of a track while moving catches the elevated 7.
   With those OR'd in, trains are caught 99 to 100% of the time (elevated 7: 61 to 99%), but it fires in 20 to 50% of
   off-train windows with a fix, some of which are platforms; the labels can't tell platforms from the street.
   Chimes and OMNY beeps are events, not window features (`chime_omny.py`, 32 ms): chimes precede 28 of 31
   "doors closed" presses on phone-mic rides, pitches found per car type; all 4 known OMNY taps found within 0.3 s
   (2026-10-21's at 110 St, 13:57:06.1, between two stair descents in the pressure; Google Wallet says 1:57 pm), with 3 to 8 other beeps per ride
   (other riders, an elevator): pair a beep with the stair pattern to tell the rider's own.
   Weak per window: pressure. Hunting came out backwards:
   walking's ~2 Hz steps swamp it; separate lateral sway from steps' vertical bounce.
   A logistic regression did worse than the rules, trained on labels that call stations "off".
   The same applies to walking (runaways up to 187 mph on transfers underground)
   and to trains faster than the subway (LIRR, Amtrak).
4. Magnetometer in the world frame, gain learned with GNSS; inverter pitch tracking.
5. Beacon decoding; Piper; CBTC and ATO/ATPM classification.
6. Pressure grade term, robust to tunnel waves (grades up to 4.6% on the 7 under the East River, 5.5% on the Manhattan Bridge).
7. Recording: the input device in the session, and the ambient temperature sensor if the phone has one.
   Ask for "train stopped/started" taps on rides, the most precise labels.
   The ride of 2026-10-21 (`20261021-135509`) has them: every "train stopped/started" tap is precise enough to tune on,
   except the "train stopped" at 14:17:29, a mistap 0.9 s before the "train started" (it predates the Taps screen).
   It also has an elevator up 34 floors, plain in the pressure: 1018.6 to 1005.0 hPa (about 113 m)
   from 14:29:35 to 14:29:59, cruising at 0.70 hPa/s (about 5.8 m/s);
   a check for detecting vertical motion, with the accelerometer's push at the start and end.
   The "Alighted" at 14:17:09 should have been "Boarded": off the D at 7 Av/53 St (14:14:06),
   then onto the E there (its doors closed 17 s later, then "7 Av/53 St (E)").
   Against its 9 stop and 9 start taps (`python/experiments/startstop.py`): the start surge is sharp
   (detections a median 0.1 to 0.3 s after the tap) but handling makes false ones; stops don't show in the
   acceleration's size (braking fades; stopped-in-hand looks like moving); the replayed speed estimator registers
   only 3 of 9 stops and 3 of 9 starts.
   In the gyro-tracked frame (not a gravity average, which tilting in hand leaks into), the final lurch is clear:
   a collapse of sustained braking 0.5 to 0.9 s before all 9 stop taps (the rider's reaction, median 0.78 s);
   braking for half the 10 s before plus a still phone after keeps 5 of 9 with no false ones on the ride.
   The rider often moves to exit right after a stop, so confirm stops with the train, not the phone being still.
   Doors in the sound (`doors.py`): above 200 Hz, +4 to 7 dB as they open, -2 to 4 dB as they close (chime 2 s before);
   a door cycle (step up, step down 8 to 90 s later with a chime) finds 25 of 39 openings and 26 of 31 closings
   at 22 an hour elsewhere. Next: combine lurch, door cycle, and start surge into one stop sequence; the magnetometer
   in the world frame for starts; then port to Rust and the speed estimator's zero-velocity handling underground.
   Express passes: each station's Wi-Fi is seen in one scan per pass (every ~5 s), cell signal peaks at stations
   (sampled every 5 s): speed between stations from pass times and distances, not within one. Poll cell every 1 s,
   look for fixed station BLE beacons, and check `is_80211mc_responder` (now recorded) for Wi-Fi ranging.

Carried over from earlier sessions, still open:
- Combined estimator stage 1: a horizontal position layer (own GNSS position and velocity, phone fixes without an own solution,
  IMU along-track speed and gyro heading between), smoothed backward like speed; gated on `just test-sdc` with GNSS cut out.
  Built (2026-10-21): `crates/fusion/src/position.rs`, fed by `Fusion`, in `Replay::positions`; not in the app yet.
  On the 2026-10-20 rides it's a median 1.2 to 1.9 m from the fixes, but our own GNSS solutions are left out
  (`USE_OWN_SOLUTIONS_FOR_POSITION`): they made it far worse, hundreds of meters off with tight covariances.
  The gate is `just test-sdc-position`, and it fails on the speed estimator, not the position filter:
  in 120 s outages the speed runs away (95 m/s against 15) or sticks at zero while moving, with an untruthful sigma.
  Through the 32-minute tunnel from Jackson Heights to Herald Square, gyro heading alone wanders by kilometers:
  that needs the track geometry (map matching).
  Position, velocity, and acceleration each from the best fusion available, and one failing never takes down the others.
  The ride's average speed by position as a cross-check, over the rough path (small position changes ignored).
- GNSS: all five constellations decode (GPS, QZSS, Galileo, BeiDou, GLONASS) but bit positions are unverified on a real S26+ log;
  check both frequencies (L1/E1/B1 and L5/E5a) are measured and used; ionosphere (Klobuchar) and troposphere models (~12 m up bias).
- Ground truth from the trip itself: SubwayNow / GTFS-realtime trip pages (e.g. `data/trips/7X-*`) give the train's
  stop and pass times and trip ID; save them during or right after a ride, before they expire, to check station detection and timing.
- Live and offline run the same Rust code; any difference or duplication is justified first.
- Testing without the phone: `just test-sdc` (Kaggle Smartphone Decimeter Challenge; redistribution terms to check), the Android emulator,
  and CI (set up, never run for Rust). Live map, live speed, the Rust writer, and `Health` are still untested on the phone.
- Inverter sound references: VVVF-Simulator (`JOTAN-0655/VVVF-Simulator`) and its shared train files (`VvvfGeeks/YAML-Files`,
  31 trains, mostly Japanese, none from NYC); NYC traction is Bombardier IGBT on the R188, Alstom ONIX or Siemens SITRAC on the R160,
  Alstom ONIX on the R142. Measure NYC signatures ourselves from rides with GNSS, by car type.
- Slow cell reconnection at stations: measured from `ServiceState` (no app can force a network search).

## Session 2026-10-21: decisions, app changes, and what's open

Decisions (the user's):
- The rider's marks (taps) are labels for calibrating and scoring, never inputs to the speed or anything live;
  context (subway, walking, other trains) is detected from the sensors. A mark-driven speed bound was built and reverted.
- Always analyze the IMU and magnetometer in a gyro-tracked world frame, never with a moving-average gravity.
- The rider often moves to exit right after a stop: confirm stops from the train (lurch, doors, chime), not stillness.
- Recorder and sensor changes come first: data not recorded can't be recovered, analysis can be redone.
- Buttons never move under a finger (fixed panes); every button gives haptic feedback.
- Taps are corrected losslessly (appended `KIND_DELETE` / changed marks); whole rides aren't deleted from the app.
- Bump the version in its own commit before every install on the phone.

App, 0.1.1 to 0.1.3 (0.1.3 on the phone, installed 2026-10-21 from `26fccd6`):
haptic buttons; Start/Map/Past rides/Taps on top (wrapping), ride buttons in a fixed pane; the Taps screen (this ride's
taps, deletable); past rides' taps viewable, deletable, and changeable (`ride_session::marks`, appended records);
back gesture returns to the main page; screens clear the status bar; offline orbit downloads and a stop mid-download
aren't reported as problems; the average speed wraps; subway detection holds the live speed to 55 mph in the subway;
cell polled every 1 s, Wi-Fi scans requested every 4 s always, BLE low latency, under a "Radio scanning" level
(Frequent, Normal, Battery saver); `is_80211mc_responder` recorded per access point.

0.1.4 (on the phone, installed 2026-10-21 from `af669b8`): post-ride recompression stores audio as FLAC,
bit-exact (`53aa61a`), 23% smaller; the 17 sessions pulled to the PC were re-encoded with `ride recompress`
(575 MB to 443 MB of audio, every WAV export unchanged). The phone's copies were then replaced, each checked first
(same audio decoded, and the copy's SHA-256 on the phone).
OneDrive's 6 (2026-10-22): re-encoded in a copy, each WAV export identical, then swapped in by renaming;
the originals deleted at the rider's request (267 MB to 211 MB).

Gotchas learned:
- Android records SSIDs quoted (`"BoldynWiFi"`); `BoldynPasspoint` is a street network, `PasspointBoldyn` the subway's.
- `ride summary` gives tap times from the first tap, not the session start; give clock times instead.
- `wifi.csv` from `crates/cli/examples/radio.rs` has each network's last-seen time, not the scan's.
- The app can't upload to OneDrive (no client ID in the build); the user uploads through the website to
  `Documents/RideLogger/sessions`, which syncs to `/mnt/c/Users/kkyse/OneDrive/Khyber/Documents/RideLogger/sessions`.
- Wireless debugging's address changes on every reconnect; `just adb-server` first after a WSL restart.

Open, roughly in order:
1. Done: Wi-Fi scan throttling is off since the 2026-10-21 evening rides (`182135`, `215922`).
2. Done (`crates/cli/examples/radio_stats.rs`): with throttling off, fresh Wi-Fi scans come every 5.0 s (median),
   not 4 s: a scan takes longer, so requests during one merge into it. No subway access point is an 802.11mc
   responder (only 15 home routers of 2638 to 3331 seen), so ranging is out. Cell is polled every 1 s,
   but the modem re-measures the registered cell every 2.0 s.
3. Stops (`python/experiments/stops.py`, `8a1159b`): door cycles up to 60 s (sound step up, chime, step down)
   find 27 of 39 stops on four rides with none at speed aboard;
   with the 2026-10-21 evening rides, 38 of 57 tapped stops on seven, still none at speed (`e779e91`); the lurch alone finds 37 of 39 but 42 at speed,
   so it only times a confirmed stop; start surges find 8 of 9 starts on `135509` within 1 s.
   Next: port door cycles (with the lurch for timing) to Rust and give the speed estimator zero-velocity updates
   from doors open to doors closed (it registers only 3 of 9 stops on `20261021-135509`); magnetometer for starts.
4. Subway detection in Rust: add the magnetometer, train beacon, and fix-on-track signals (scored in
   `subway_signals.py`), and walking detection for runaways off the train; position on the subway map for elevated lines.
5. Hunting, as lateral sway told from walking's vertical bounce; OMNY tap detection paired with the stair pattern.
6. The speed estimator's runaway and sigma (the position gate, `just test-sdc-position`, fails on it),
   and its start-up runaway in the first minute (221 mph on `184134`).
   2026-10-21: the position filter now sets off from a stop in its last direction (`ce084a5`; median 182 m to 147 m).
   On the phone's rides it lets the speed's errors move the path: fix vs position, 95% worse on four of five rides,
   max 20 m to 331 m on `214427` (`crates/cli/examples/positions.rs`); the user decides whether it stays.
   The runaway is mostly gyro bias tilting gravity (`crates/fusion/examples/sdc_speed.rs`; the bias at rest, removed,
   takes the first outage's end from 226 m/s to 45 m/s), and no stop ever registers in the car.
   A bias learner is on the local branch `gyro-bias-learner` (`3bacc5c`): better on `154951` and `184134`,
   worse on `214427`; the user decides. `CUT_GNSS` on `export.rs` scores such changes on real rides.
7. Stations from the registered cell (`python/experiments/station_cell.py`): a station keeps its cell ID across rides
   (110 St `29646293`, Chambers St `28852869` both ways); Wi-Fi radios recur too weakly to use alone.
   Next: cell ID to station names (label by stop order along a known line), then a lurch or door cycle there
   means a stop at that station, and its position pins the path (with the stop heading fix, `ce084a5`).
   Trip paths (`python/experiments/trip_paths.py`): 12 of 16 legs placed on routes, awaiting the user's check;
   at speed the cells shift a station (tunnel handovers), so use only the cells at stops.
   Also: stations from pass times and distances on express runs; station BLE beacons if any.
   Express passes to check: the 2 past 59 St and 66 St (`214427`), and the F express in Queens on `184134`,
   which passed 65 St very fast and 36 St more slowly (per the rider).

7a. The 2026-10-23 ride (1 to the L at 14 St, then the L to Brooklyn), once uploaded: on the L platform at 6 Av,
   the live speed read 25 mph (IMU drift standing still), and the status 1 train and R62, then Q at Union Sq and
   3 Av (a one-stop GTFS-RT match; the boarding at 6 Av likely not a stop), and R62 despite a steady car beacon
   (the 1's chime kept 15 min). Also check for cell service under the East River on the L and which cells serve it.
   Car beacons on the L (live: car 9dd8 on the 6 Av platform, 9d94 aboard): confirm they're the same family.
   If so, the family isn't Piper's (the rider: the L's CBTC predates Piper), unless Piper's were added to the L too.
   Approaching Lorimer St, the live dwell read stopped 0:48 just as the train arrived: a lurch found before the
   platform (a hold outside it, or braking taken for a stop), and the arrival's lurch not restarting it.
   Taps: at Graham Av, a train start tapped by mistake for the stop, the stop tapped 1 s after it (a bit late);
   drop the start if the rider hasn't deleted it in the app.
   Departures on the L detected poorly (per the rider): likely ATO's gentle starts under the lurch threshold
   tuned on manually driven lines; check the L's departures against the car beacons' own accelerometers.
   On the curve approaching Jefferson St, the live speed went from 40 to 55 mph (the cap), likely a curve,
   not speeding up: centripetal acceleration (v^2/r, about 1.5 m/s^2 at 40 mph) leaking into the travel axis,
   and underground the axis learned from the horizontal spread, which a long curve turns sideways.
   Idea (untested): subtract v times the gyro's yaw rate, sideways to the axis, before projecting and before
   learning the axis, and hold the axis while turning. Test on underground curves (the speed hitting the cap)
   and on elevated curves with GNSS withheld against GNSS.
   Back at 14 St, waiting for the 1 after walking from the L: a passing 2's chime was taken as the rider's
   (NTT cars), the live speed ran to 55 mph standing on the platform, and the station read 14 St (L),
   from the L just ridden. While between trains: ignore chimes, hold the speed, name the station by all its lines.
   The average speed seems to leave out underground travel (the rider). Between two stops at known stations,
   the track distance (GTFS shapes) over the time between them is that run's average speed, and the speed
   integrated from the departure to the stop must cover that distance: a measurement for the smoother, which would
   end runaway IMU speeds between known stations. Needs the stations right (tunnel holds, cells shifted at speed).
   86 St on the uptown 1 wasn't named live (the rider): it has no tagged cell yet; add it from this ride's tap,
   or from the stop between 79 St and 96 St.
   Held in the tunnel between 86 St and 96 St on the uptown 1 for a couple of minutes: live it said stopped
   at first, then not, then stopped 2:28 once moving again. Likely the lurch's likely stop dropped for want of
   doors, and the quiet phone's stop rejected by the gate (`rejected as outliers: stop 4482`, samples); a quiet
   phone should keep a stop alive, ungated underground like door cycles. Then at 96 St, NTT cars on the 1
   (R62A): a 2 or 3 heard across the platform, or the chime misread; the line should rule out its models.

8. The home elevator (`python/experiments/elevator.py`): 12 m in 25 s with the phone still is the elevator,
   with steps (4.9 to 6.4 m/s^2 std) the stairs; marks the start and end of a trip at home.

9. 2026-10-22, from the sensors alone (committed, not yet on the phone):
   - Door cycles in Rust (`crates/fusion/src/doors.rs`, `e9cb7d8`), fed live from each microphone read (`a390bac`):
     closings match Python's exactly; none GNSS could check came while a train moved, and all 39 aboard have the
     stop's lurch before (36) or a start surge after (33), underground too; openings aren't sure (several steps up
     per closing; where GNSS could check, 4 of 4 earliest openings aboard were moving), so only closings are used.
     A closing holds the speed at zero (ungated underground), takes the drift since the last measurement out as a
     ramp, forward and backward, and the 8 s before it are zero for the smoother and the position filter (`b1427ca`).
     With walking and the gravity fix too: median forward speed at tapped stops 23 to 11, 16 to 2, 32 to 3, 52 to 13,
     55 to 20, 49 to 14 mph; smoothed under 1 m/s at 19 of 58 stops, from 6; fix vs position max on `214427`
     331 m to 61 m. Runaways between stations still reach 55 mph. The app's 100 ms reads give the same closings.
   - Each ride's door-opening sound, learned from its own cycles (`door_self_template.py`): 28 of the 31 openings
     a candidate could find, against 7 for the earliest; not yet trusted for zero speed (one of two GNSS-checked
     picks was at 11 m/s). Next: port it once more rides check it.
   - Stations from the cell at each heard stop (`station_stops.py`): 62 cells, 6 recurring; a site (ID / 256) spans
     a few stations. Pinning positions waits for more rides and the rider's corrected trips.
   - Gravity re-learned in 2 s while the phone is handled before any measurement (`f1a7add`): the start-up runaway
     on `184134` 221 to 14 mph, `215922` 55 to 11.
   - Walking (`crates/fusion/src/walking.rs`, `11f06ec`): two 4 s windows of steps hold the speed near 1.4 m/s.
     The 102 mph stretch on `154951` remains: quiet, not walking, not seen as the subway.
   - The home elevator stays offline: anywhere in a ride, 26 other 25 s height changes of 10 m or more barely
     shake (escalators, grades, tunnel pressure); only at a ride's end does it stand out.
   - Live map: redraws only what smoothing changed, pans only near the edge (`b5299a7`); tiles kept a month (`b2e2ac0`).
   - Express passes, from the door cycles: none heard on `214427`'s 2 from 42 St to 72 St (it ran express,
     passing 50 St, 59 St, and 66 St, per the rider; at 72 St two closings 15 s apart, the doors closing, reopening,
     closing), and none between Jackson Hts and Queens Plaza on `184134`'s F (correct, per the rider).
   - Learned door openings: the one GNSS put at 25 mph was the 7 pulling into 61 St at 4:01:27 pm (GNSS likely
     right; the rider tapped doors open 18 s later); a clip is in `data/work/clips/`, for the rider to listen to.

10. 2026-10-22 night, 0.1.5 on the phone (installed 2026-10-22 3:55 am):
   - The bus out of Port Authority (`20261022-002914`): a moving fix now ends the subway Wi-Fi's hold, and no fix
     counts as underground only with a traction-current burst (|B| std over 10 uT, without a fix, in 90 s);
     elevated track (`open_track.py`, GTFS shapes between above-ground stations) brings elevated lines back.
     Underground train time stays all subway; the bus and the Lincoln Tunnel are no longer subway.
   - Walking needs 24 s of steps (the rider sometimes walks a few cars on a moving train); running holds 3 m/s.
   - Stations from the registered cell (`station_cells.py`, from the rider's "at station" tags; 52 cells,
     37 stations). Tags made while passing may be a station off: the rider is reviewing the table.
   - The main screen shows the mode (walking, running, still, subway under or above ground, other vehicle), the
     station, and the car type from the chime; a notification after 10 minutes staying put out of the subway.
   - OMNY: a beep with stairs within 45 s keeps all 4 known taps and drops 38 beeps to 21 (`omny_stairs.py`).
   - Stairs or riding up (`elevator.py`'s `moves`): steps during are stairs, none during but walking just before is
     riding; an escalator can't yet be told from an elevator (both about 0.5 m/s, both walked onto).
   - Hunting (`hunting.py`): sideways over vertical power tells a train from walking (AUC 0.86), not motion.
   - Oct 20, 4:27 pm: the 102 mph was the rider indoors in Flushing for 4 minutes (fixes before and after within
     10 m); next: a gap's displacement as a constraint for the smoother.
   Next: the rider's corrections to the station table; pin positions at stations; the elevator's jolts.

11. 0.1.6 (installed 2026-10-22, early morning):
   - Stations from Android's own fused location (Google's database), already recorded: station by station
     underground on `182135` and `135509`; the tagged-cell table is the fallback. BeaconDB sends cell IDs to its server.
   - OpenCelliD doesn't work for us (checked 2026-10-23 with the rider's key, AT&T 310/410):
     of the 52 tagged station cells, 38 are unknown and 14 have one sample each, ranges 0.9 to 6.9 km;
     only Flushing-Main St is within 100 m (85 m), four more within 330 m, the rest 400 m to 12 km off.
     Subway cells get few GPS-tagged reports, placed from wherever GPS last was. Not built; the key stays
     in `~/.gradle/gradle.properties` (`ride.opencellidKey`), unused. Recheck coverage before reconsidering.
   - Car types as the rider named them: R62 (1's R62A and 3's R62 sound the same) and NTT (2 R142, 7 R188,
     F/M/R R160), with the model from the station's lines.
   - OMNY: the rider's own taps are the loud beeps (38 to 54 dB, others 33 or less), all 7 confirmed;
     Google Wallet's notifications are recorded once the rider grants access (they can lag; the beep times them).
   - Bus or car: the 158 (NJ Transit, 12:56 to 1:15 am) made 10 stops in 20 min and shook 0.85 m/s^2 (median);
     the Uber from Edgewater (2:48 to 3:04 am) 6 in 14 min at 0.67. Too alike with one of each, and shaking varies
     by bus (high floor shakes less, per the rider); next: bus stops' dwell and door sounds, against lights.
   - Train beacons (`02FD`, see Ideas): ride with each train on CBTC lines, unused so far (see 12).
   - Automatic OneDrive upload once they register an app
     (client ID; device code sign-in, so no package name or signature hash; steps in `CHECKLIST.md`).
   - Missed from earlier sessions, now listed:
     - Train and line identification (asked 2026-10-21): line from route geometry, stations, and car type;
       the trainset from the beacons; the trip from GTFS realtime (SubwayNow) by station times.
     - Speed from fixed radios: a station's Wi-Fi or cell signal rising and falling as an express passes,
       and Wi-Fi ranging (`is_80211mc_responder`) if any subway AP supports it.
     - Updating the app away from the computer (asked 2026-10-21): CI-signed APKs, installed from GitHub Releases
       (e.g. Obtainium), since wireless debugging needs the same network.
     - The tunnels with cell service are per carrier (the rider has AT&T, may switch to T-Mobile): keep the carrier
       with each cell, and don't read a tunnel's coverage on one carrier as true of another.

12. After 0.1.6 (2026-10-22, not yet on the phone):
   - Gyro bias learner retested on today's `main` (branch `gyro-bias-retest`, `8b04c78`), GNSS cut 30 s in 90
     (outage rms / at outage end, km/h): `154951` 3.33/6.39 to 3.03/5.19, `184134` 6.84/10.98 to 7.46/11.20,
     `214427` 2.99/0.61 to 2.27/5.48. Still mixed, so still off `main`.
   - Elevator or escalator by the car's jolts (`elevator_jolts.py`): no separation with the phone in hand;
     next, the jolt along gravity only and the escalator's tread vibration.
   - Smoother: across a gap between good fixes, speeds cover at most twice the straight distance plus both accuracies
     and 1.5 m/s (`e232062`); Flushing indoors (4:25 pm, 257 s) went from 33 to 10 mph.
   - Position: a door cycle at an identified station pins the position there, 90 m accurate (`abc9f5e`).
     Not yet judged on a real ride: needs a ride with stations to check the path against.
   - Door openings learned per ride (`door_self_template.py`), all 9 subway rides: within 3 s of 30 of 60 open taps,
     of the 33 with any candidate; the 1 to 4 kHz step up misses the rest. Not ported; next, a better step-up finder.
   - Untagged cells (`untagged_cells_check.py`): 106 placed by Google, 2 flagged, swapping between 61 St-Woodside and
     69 St on the elevated 7 (Oct 20, 4:02 pm); no jumps.
   - Train beacons (`train_beacons.py`): one per car (2 to 5 steady at once, the same IDs across rides),
     each sending its car's acceleration (64 counts per g); the 7's lie flat, another fleet's stand on edge.
     Next: boarding and alighting from them, and the car's own acceleration against the phone's.

13. Later on 2026-10-22 (0.1.7 built, not installed):
   - Car beacons in Rust (`car_beacons.rs`, `b32b06d`): decoded live and in replay; the rider's car is the steady
     beacon heard loudest (5 dB to switch). The main screen shows it ("car a199"), and a "Car number" tag
     (`KIND_CAR_NUMBER`, a label) records the number painted on the car, to tie beacons to cars (`5784d1c`).
     On `184134` the cars rank `a199` (-74 dBm, the rider's), `a1d2` -76, `a138` -78, `a139` -85, `a1f6` -91.
   - The car's accelerometer as a measurement of the phone's bias (`LiveSpeedEstimator::on_car_accel`): its forward
     axis and sign are learned (correlation 0.7, no lag), but it's noisy (0.46 m/s^2 left of a 0.61 signal) and
     changed little (GNSS cut: `154951` 3.33 to 3.41 km/h, `184134` 6.84 either way; underground stop taps,
     `184134`'s 90th percentile forward speed 46 to 38 mph, others unchanged), so off (`USE_CAR_ACCEL`).
   - Beacon prediction to check: present on the L (R143, R160; the oldest CBTC, Siemens) and on the A's R211s,
     absent on the 4, 5, 6 (R142, R142A; a no-CBTC control, whose cars will be replaced before any CBTC).
     The rider will ride the J/Z (R160s not fitted for CBTC, R179s CBTC-ready, the line not CBTC):
     beacons on both means they follow car models, on only the R179s means CBTC equipment, on neither means
     they need a CBTC line.
   - On foot in a gap (`5d6a052`): the walking allowance counts only detected steps, and out of a vehicle each speed
     is held to a run (4.5 m/s), walk (2.6), or standing (0.5): Flushing's McDonald's (4:25 pm, 257 s) from 10 mph
     to 1.5 at most. A run in a gap stays a run (tested).
   - Door openings searched 1 to 10 s after the lurch (`door_lurch_openings.py`): 32 of 60 taps, against 30;
     not ported. The taps follow the lurch steadily (median 3.3 s), but a cycle's last lurch is often not the
     stopping one; next, find the stopping lurch itself.
   - The stopping lurch (`stopping_lurch.py`, ported as `crates/fusion/src/stops.rs`): the lurch after which the
     car is still or settles back along the braking direction (1 to 5 s after), scored by the braking before it,
     the best within 30 s either side, known about 40 s later. 21 of 28 precise "train stopped" taps within 1 s,
     22 within 2 s (any lurch at all: 23 and 24, among 746).
     In the subway, once a door closing or a departure confirms it, it holds the speed at zero for 4 s:
     2 s after the stop taps, smoothed 6.2 mph median and 39.6 p90 before, 0.0 and 11.2 now; GNSS-cut scores
     unchanged (unconfirmed, two lurches at speed on `154951` made them worse).
   - Per-stop records (`Fusion::stop_records`, `stops.csv` from `export.rs`): stopping lurch and score, station,
     doors' opening and closing, departure, dwell, and doors-open time; offline from recorded sensors, nothing new
     recorded. Openings: heard 1 to 10 s after the stop, else the stop plus 3.2 s (flagged): "plus 3.2 s" puts 43 of
     60 "doors open" taps within 3 s, the learned sound searched after the stopping lurch 37, before 32.
     Next: confirm stops whose doors weren't heard and that no surge ends (most of the 11.2 mph p90); dwell at
     stations against the rider's taps; show the last dwell live if wanted.
   - Elevator or escalator by the climb's shape and the walking around it (`elevator_shape.py`): no separation, and
     the PABT escalators aren't among the climbs found; needs the rider's times for escalator rides.
   - The C's cars are R211s (a few R179s) in `stations.rs`.
   - Uploads: the rider will register the Azure app later (a free Azure account, for the tenant); nothing until then.

14. Tried on 2026-10-22, after the car's accelerometer went on (no recent GNSS only):
   - Gyro bias learner, gated (`gyro-bias-retest`, `7abdfaf`, `GB_MODE`/`GB_TAU` experiment knobs), GNSS cut 30 s in 90
     (outage rms / at outage end, km/h): `main` 3.41/6.54, 6.84/10.97, 2.99/0.61 (`154951`, `184134`, `214427`).
     Learning only while still: 120 s 2.95/5.66, 4.61/6.75, 3.23/1.03; 600 s 3.27/6.28, 5.63/9.36, 3.04/0.50.
     Applying only after 10 or 20 s without GNSS, or not learning while handled, changed little.
     Still-only at 600 s is better on two rides and within 0.05 km/h on the third, but the time constant's effect
     isn't monotonic (240 s: 3.11, 3.83, 3.28/3.53): not merged; needs more rides with GNSS to tune.
   - Speed from fixed radios (`radio_pass_speed.py`): not usable. 129 of 171 passes are one 4 s Wi-Fi scan, and
     how long a station is heard barely follows the speed (correlation 0.20); the 1 s registered cell is no better
     (-0.14), since handovers follow the modem's hysteresis. Would need faster scans or RTT ranging.
   - Traction sound on the 7's R188s (`vvvf_gnss.py`, 7227 frames with GNSS): a line locked to speed at 32.5 Hz per m/s
     (harmonics near 65, 98), strongest braking and motoring, but too weak per 0.5 s to read speed from
     (7 to 13 m/s off; near a 3 m/s-off prior, 1 to 4.5 m/s off); band levels give motoring AUC 0.57, braking 0.62.
     Next: longer FFTs, other car types.
   - Car position (`car_position.py`, ready for "Car number" taps): consist and the rider's car, tags mapped to beacons,
     offset behind the stop marker by car length (A Division 15.7 m, B Division 18.4 m).
     Ahead or behind needs direction: the train pulling in past a still rider (front cars peak first),
     the walking order against the phone's heading, each beacon's braking sign (car facing), consecutive car numbers,
     platform-end Wi-Fi. Range is 2 to 3 cars, so without direction only the train's ends resolve.
     On `184134` and `135509` every other car's peak at boarding came while walking: an order only so far.

15. Stops and transfers (2026-10-22, after 14):
   - Long "dwells" were mostly the rider changing trains or waiting on the platform (59 St and 7 Av on `135509`,
     Times Sq on `151449` and `155919`, Grand Av and Jackson Hts on `184134`); 86 St on `135509` (104 s) and
     Elmhurst Av on `184134` (78 s) were real holds. 110 St is the rider's usual station: long records there
     are usually platform waits, a check, not a rule.
   - From the step rhythm, stop records now have `alighted_ns` (walking off), `boarded_ns` (the last steps before
     another train's departure), and `on_platform` (walking up to a stop heard from the platform); no dwell for these.
     Off and back on within 30 s is stepping out to let others off: dwell kept.
     The sensors can't see the other train's events at a transfer; a closing heard within 60 s is still the train left
     (7 Av's 21.5 s: the D's, as the rider guessed, and 22.4 s by the rider's taps).
   - A departure's fading pull read as a stop (Grand Central, `151449`, 3:44 pm, and 3 more): braking that began 0 to
     8 s after a door closing and lasted 7 s or more is now the departure (its start surge was missed), not a stop.
   - `stops_tapped.csv` (offline): stops as the taps tell them, transfers marked, filled from the sensors.
     23 of 28 tapped stops matched, the sensors 0.8 s early, dwells 1.2 s long, all within 3 s;
     6 of 9 tapped alightings seen, the rest at stops never found.
   - Walking off now confirms a stop for zero speed: smoothed speed 2 s after "train stopped" taps, p90 11.2 to
     2.5 mph (median 0); GNSS-cut scores unchanged. Left: 2 stops the lurch detector misses
     (`182135` 6:27 pm, `215922` 10:37 pm).
   - The gated gyro bias learner (branch `gyro-bias-retest`) waits for the rider's next outdoor ride to tune on.
16. Lines, transfers, and every car's accelerometer (2026-10-22, after 15):
   - Which train (`python/experiments/train_line.py`): the stations seen in order against every GTFS stopping
     pattern (with the stations each skips), split at the stop records' transfers, then the scheduled trip.
     44 of the rider's 62 line-naming tags right (49 among near-equal lines). Left: 2 against 3 (same stops;
     the chime would settle it, but the replayed status's chime family is R62 on a third of the 7's R188 rows),
     C against B on Central Park West (a C's R211 beacon, if the R68s have none), and tags on a transfer's
     platform naming the next train. Scheduled trips fit within 2 minutes. Not live until it's in Rust and better.
   - Transfers (`python/experiments/transfers.py`): `transfers.csv` (sensors) and `transfers_tapped.csv` (taps and
     tags) per ride: station, lines, chime family, car beacon, alight and board times, walking and standing,
     height change, passage or same platform. Times Sq 7 to 3 (`155919`, 4:08 to 4:09 pm, +9 m), Jackson Hts M to F
     (`184134`, 7:20 to 7:24 pm, same platform), Grand Av, and 7 Av D to E (`135509`) found by the sensors;
     59 St C to D only by taps; Times Sq 3 to 7 (`151449`) boarded 93 s early (walking onto the platform).
   - Every steady car's accelerometer, averaged (`crates/fusion/src/live_speed.rs`): two cars agree with each other
     0.86, with GNSS 0.81 and 0.82, so their noise is their own; outage rms (at end) `154951` 3.41 (6.54) to
     3.25 (6.22) km/h, `184134` 6.84 (10.97) to 6.39 (9.82), `214427` unchanged. Gain against GNSS 0.6 to 0.75.
17. Chimes, boarding, car changes, and the cars' scale (2026-10-22, after 16):
   - The chime's car type (`crates/fusion/src/doors.rs`): each tone's family is the one leading most of its windows,
     and it counts only once a closing follows within 8 s; the status shows it only in the subway mode.
     The status had kept the last chime-like tone for 15 minutes (the 7 read R62 after walking off at Flushing).
     Each confirmed chime is exported (`chimes.csv`), and line identification counts them:
     48 of 62 tags right, from 44 (the 3 on `151449` 3:30 pm, `155919` 4:13 pm, `215922` 10:14 pm; the 2 stays 2).
     The B runs R68s and R211s (the rider), so beacons don't separate the B from the C either.
   - Boarding (`python/experiments/boarding.py`, `stepped_in` in `crates/fusion/src/fusion.rs`): a step onto a train
     is often too short for a step window; a car beacon first heard after the last walking that rises 8 dB is the
     boarding. Times Sq 3 to 7 (`151449`): 3:40:34 pm against the tap at 3:40:30, from 3:38:57; the other 4 tapped
     boardings unchanged, all within 10 s. Waiting for the boarded train's door cycle failed when the rider
     boarded early (+156 s on `155919`).
   - Changes of car on the same train: each stop record keeps the strongest car beacon before stepping off and
     after boarding; a step out (under 30 s) into another car is `car_change` (82 St, `184134`, 7:07 pm,
     `a199` to `a138`), not a transfer, in `stops.csv` and `transfers.py`.
   - The cars' scale (`python/experiments/car_scale.py`): the 0.6 to 0.75 gain is the readings' noise diluting the
     fit; dividing by it makes the error worse (0.38 to 0.50 m/s^2 on `a14e`), and the 3D direction adds little
     (correlation 0.65 against 0.63), so the raw single axis stays and no per-car table is kept.
   - Recording GTFS-RT and subwaydata.nyc's archives: the rider said yes to both; see 18.
18. Which train actually ran (2026-10-22, in 0.1.7, not yet on the phone):
   - The app records the MTA's subway GTFS-realtime feeds (public, no key) every 30 s into a `transit` stream,
     each response unchanged (`GtfsRealtimeFeed`); Rust picks the feeds for lines within 1.5 km, nearest first
     (`ride_fusion::gtfs_rt`), every feed without a position. Underground, a failed poll retries the moment the
     network is validated again, with 5 s timeouts, so a few seconds of signal in a station are used.
     Size: 7 to 185 KB per feed, about 4.5 times less over the air (gzip); a typical 2 to 4 feeds is roughly
     10 to 20 MB of mobile data per hour, and stored zstd-compressed (xz gets 167 KB to 21 KB).
   - Reading it: `ride_session::transit::polls`, `crates/cli/examples/transit.rs` (`vehicles.csv`,
     `trip_updates.csv`, with the MTA's train ID and tracks).
   - Past rides: `python/experiments/actual_trips.py` against subwaydata.nyc's daily archive. Clear: the 7 express
     on 2026-10-20 is train `07 1536+` in both sessions; the 2 (not 3) from 96 St on 2026-10-21 at 6:27 pm; the C
     (not B) from 110 St; the F from Queens Plaza. Short legs, lines sharing stops and minutes, platform waits,
     and legs without a detected walk between trains are still unsure or wrong.
   - Mobile data is fine for now (the rider has unlimited data).
   - Cross-platform transfers (2026-10-22): `215922`'s 3 to 1 at 96 St (10:37 pm) had no stop found at all;
     a departure after 24 s or more of walking that began as a walk off, with a ride before it, now infers the stop
     6 s before the walk (`ccc2a24`; within 11 s of the taps, no stop added on 7 other rides). With few steps,
     `train_change` marks a stop whose cars heard before and after are all different (`7b646c6`): exactly R to M
     and M to F on `184134`, nothing ridden through. Without beacons and steps: the chime family can tell the 1
     (R62A) from the 2 (R142), not from the 3 (R62); the recorded GTFS-RT feed is the general answer.

## Session 2026-10-23 to 24: app changes, and what's open

App 0.1.9 (on the phone, installed 2026-10-24 from `120d5ae`), reported live by the rider on the 2026-10-23 rides
and built the same night:
- Walking ends the live hold at a likely stop (`5a8177c`): underground it read 0 mph walking after getting off.
- The "< v + sigma" bound is capped at the subway's 55 mph (`266c174`).
- `transfer m:ss` from walking off a train until on another, instead of the stop's time (`4b3efcb`).
  It ends on a departure with no steps around it, or a car heard as the rider's for 3 minutes
  (trains at the platform are heard steadily too, at most 70 s at Jackson Hts on `184134`).
  On `184134`: right at Grand Av, Jackson Hts, and leaving at 34 St; briefly wrong at 90 St (walking along the car)
  and 82 St (stepping out); missed at 74 St, where the walk off isn't found at all.
- The graph: each stop shaded from its stop to its departure, the doors' opening darker, labeled with the station,
  dwell, and doors' time (`e2e870c`); labels fit the room to the next stop, on two lines, else a short name
  (`Jackson Hts`), else none until zoomed (`dac7bc4`); pinch zooms time only (`e2e870c`);
  a strip across the top of each leg's line and walking (`ff0017b`, `6ad93e1`), from `Fusion::timeline`,
  each leg's line from all its stops.
- Speeds under 10 show a decimal (`6dce4e9`).
- Stairs, and elevators or escalators (one kind: the barometer can't tell them apart), from the barometer and
  the steps (`3c5a22c`, `crates/fusion/src/vertical.rs`), in the status and the strip. Finds the home elevator
  and stairs; unchecked: up and down pairs after leaving 34 St on `184134`, rides up or down at 2 to 3 am on `002914`.
- Past rides show start and end with the date, and the length as h:mm:ss (`56d2a1e`); the end is the last
  sensor sample, since the files' modification times move when recompression or tap edits rewrite them.

App 0.1.14 (on the phone, installed 2026-10-24 from `88596dc`, Rust core 0.1.14; 0.1.13 added the replay's progress bar, 0.1.14 items 3 and 12). From 0.1.12: past rides' map and graph
(item 14 below; 0.1.11 fixed 0.1.10's crash opening a long ride's graph, `851fbaf`), replays 8 times faster
(`ae5a245`) and cached until the Rust changes (`dd63d1f`), and items 3 and 4 below.
The Rust core's version is now the app version that last changed it (`just bump`, `6cb7b89`).

Sessions `20261022-142111` to `20261024-003357` (7) pulled to `data/sessions/` on 2026-10-24, each file's size
checked against the phone. The L ride is likely `20261023-193658` (7:36 pm), the uptown 1 with the tunnel hold
`20261024-003357` (12:33 am).

Open, from the rider's reports on those rides (details in item 7a of the 2026-10-21 section):
1. Done (`69b80ca`, `d28d26e`, `44c806a`; scored with `crates/cli/examples/holds.rs`, `ec8afa9`, over the 11 rides
   with taps, `just sweep holds`, results in `data/work/cmp/holds-*.txt`): the live speed underground with nothing
   holding it. The main cause: on the L every stop's settling made a start surge about 1 s after the lurch, taken
   for a departure, which ended the live hold and dwell at once (standing at Bedford Av, 55 mph); surges within
   `DEPART_AFTER_STOP_S` of the lurch are now skipped, as `stop_records` already did. The quiet phone's zero is
   ungated without recent GNSS (it now holds the tunnel hold before 96 St at 0 to 4 mph; the phone is rarely still
   enough in hand to move the scores). Between trains (`transfer_s`) the live speed shows 0 without steps, else at
   most 3 m/s (display only, like the likely stop).
   Live speed under 2 mph through tapped dwells: 1623 of 3570 s (45%) to 2233 (63%); `193658` 95 to 261 of 442,
   `003357` 219 to 336 of 420. Over 6 mph from an "alighted" tap to the next "boarded": 3275 to 2742 of 11540 s;
   the rest is where no walk off was found (item 9). Shown stopped while moving between taps: 1942 to 2258 of
   9683 s, 187 of the rise the 14 St transfer on `003357` (the metric's span holds it) and the rest false stops
   now held longer. Still open: 73 of 107 tapped stops reach over 10 mph some second of their dwell
   (the hold only starts with the likely stop, about 5 s after the lurch, and ends on walking).
2. Done (`e76cc1b`): the yaw rate times the speed, sideways to the axis, taken out of the IMU's horizontal
   acceleration while turning slower than 0.1 rad/s (faster, the rider turning the phone, it ran a stop in a
   GNSS cut on `011209` to 35 mph). Scored with the untracked scratch `crates/cli/examples/curves.rs`
   (`just sweep curves`; results in `data/work/cmp/curves-*.txt`): seconds at 50 mph or more with no fix within
   10 s, 1459 to 1083 in all (`003357` 58 to 38, its Jefferson St stretch 45 to 13 s; `193658` 63 to 48);
   pooled RMS in simulated 30 s GNSS cuts 5.9 to 5.9 mph, but `160303` 3.6 to 7.4, `155919` 5.7 to 7.5,
   `193658` 4.0 to 4.6. Holding the spread's axis while turning, alone or with it, regressed both; ungated,
   the cut RMS on `011209` went 5.5 to 13.5 mph.
   Also tried: naming each stop's station by the lines its train turned out to be on (`leg_station`, patch in
   `data/work/cmp/leg-station.patch`): no stop's station changed on `193658` or six other subway rides
   (14 St-Union Sq already reads the L), so not committed.
3. Done (`b7011be`, `51aa0fb`, `0d40f12`): the route and stop count only since the last walk off, none between
   trains, where the station's lines are its complex's; a feed trip only matches its own line's platform.
   Boarding from the platform (a transfer's end) counts as a 30 s stop at the station the radio named then.
   On `193658`: the L from 7:52:58 pm, just after boarding at 6 Av.
   Was: the line shown: 6 Av (1 train), Q at Union Sq and 3 Av, 14 St (L) on the 1's platform.
   Record the boarding stop (a train stopping while on the platform, then leaving with the rider) for the feeds'
   match; no line when two tie on one stop's timing; never a line that doesn't serve the station shown;
   between trains, name the station by all its lines.
4. Done (`fe726f0`, `b7011be`, `fbf5648`): chimes only since boarding, none between trains; the L's R143s and
   R160s; a chime the line's fleet doesn't make dropped; `R62-family chime` when that's all that's known.
   On `193658` and `003357`: NTT on the L, nothing at the 14 St transfer, R62A at 96 St.
   Was: the cars shown: R62 on the L (the 1's chime kept 15 minutes, and the L's fleet not in `car_models`, so the
   chime's family was shown as if a model; say `R62-family chime` then), despite a steady car beacon, which R62s
   don't have; NTT on the 1 at 96 St; a passing 2's chime taken as the rider's at 14 St.
   Ignore chimes between trains, and only count chimes since boarding; add the L's R143s and R160s.
5. Done (checked, no code change; scored with the untracked scratch `crates/cli/examples/departs.rs`, `just sweep departs`,
   results in `data/work/cmp/departs-before.txt`): departures on the L are found well since `69b80ca`.
   Against the "train started" taps, a start surge within 2 s of 100 of 114 taps over 11 rides, all 14 on `193658`
   (median under 0.5 s); the stop records' departure within 2 s of 81, 10 of 14 on `193658`, the 4 missed for want of
   a stop before them (boarding at 7:52:49 pm, 7:55:30, 7:58:15, and the hold at 8:04:45), not of a departure.
   The car beacons' accelerometers can't do better: the rider's car is heard only on 4 tapped rides (`135509`,
   `155919`, `181802`, `193658`; `003357`'s L stretch had no steady car), about 0.4 readings a second on the L
   with gaps up to 5 s. The start shows (the long axis from about 0.4 to 1.2 to 1.5 m/s^2 within 1 to 3 s of the tap),
   but a detector (steady over the 4 s before, then two readings off by over 0.6 m/s^2) found 3 of the L's 10 taps
   with the car heard, each 1 to 1.5 s late; over 8 s and 2 readings, 4 of 10, 3 of 6 on `135509`, with 10 starts
   no tap was near. Not committed.
6. Partly done (`69b80ca`): Lorimer St: stopped 0:48 on arriving. A lurch at 8:02:19 pm, 20 s after leaving
   Bedford Av with no tap (braking taken for a stop, or a slowdown), started the live dwell; the arrival's own
   lurch at 8:03:06 didn't restart it because the L's settling surge 1 s later read as a departure. Now the
   dwell restarts at 8:03:06 (10 s at 8:03:18 in the replay's statuses), but the early one still shows from
   8:02:28: its cell (29256324) is another sector of Bedford Av's site (29256316), so item 7's cue misses it.
   The travel axis's sign braking into Bedford Av (checked, no code change; scored with the untracked scratch
   `crates/cli/examples/signs.rs`, `just sweep signs`, results in `data/work/cmp/signs-*.txt`):
   the live speed should fall over the 10 s before each "train stopped" tap. Over all tapped rides it falls before 43,
   rises before 25, and stays within 1 mph before 40. Of the 25, 7 rise from under 5 mph, which any axis would.
   At Bedford Av on `193658` (8:01:11 pm, 33 to 55 mph), two causes stack:
   a door cycle heard mid-river at 8:00:10 pm zeroed the speed at 47 mph (a false `Doors`, worth its own look);
   and while cruising, the spread axis wanders through sideways and round to backward within seconds,
   because the covariance is centered, so a departure's steady push goes into the mean and only sway is left.
   From the zero, that sideways axis drove the speed negative, and braking along the axis (back to forward by then)
   took it to the limit. Tried, all worse or no better, not committed (patches in `data/work/cmp/`):
   holding the sign by an anchor carried from when it was learned (`sign-anchor.patch`), braking falls 37, rises 30;
   that plus signing again at each departure, on `193658` 4 and 5 (was 5 and 3);
   freezing the spread axis once its sign is known, on `193658`, `011209` and `003357` 19 and 14 (was 24 and 13);
   updating the axis only when one direction holds 3 times the rest of the spread (`axis-gate.patch`), 19 and 13.
   Starts can't score the sign: from rest, the speed rises either way.
   Learning the axis from the push itself, also tried, none better, not committed (patches in `data/work/axis/`;
   baseline at `HEAD` braking falls 43, rises 25, flat 40; starts right 93, wrong 10, flat 11):
   the axis with its sign from the horizontal acceleration summed over the first 10 s after each stop ends
   and leaky-summed (8 s) up to each stop, kept only when at least 2 m/s and 60% of the summed magnitude,
   the spread no longer turning it after (`push.patch`), braking 43, 27, 38: the phone is rarely still long enough
   to mark a stop (9 pushes on `193658`); the same also starting a push at each door closing (`push2.patch`),
   39, 24, 45; at each start surge and stopping lurch from `crate::stops` instead (`surge.patch`), 34, 27, 47;
   the spread's covariance uncentered, 28, 30, 50.
   The false door cycle mid-river on `193658` (opening 7:59:22, closing 8:00:05 pm, found 8:00:10) has no lurch
   or start surge around it, like several real ones; scored with the scratch `crates/cli/examples/doorscore.rs`
   (`DUMP=1 just sweep doorscore`, `data/work/axis/doors-base.txt`): of 251 cycles on all rides, 137 near a tap.
   The quietest 5 s of the cycle (median of 1 s stds of `|a|`) was 0.37 m/s^2 there, but real cycles reach 0.38
   to 0.98 (rider handling), so vibration doesn't separate it; no change made.
   Cycles whose quietest 5 s is over 1.0 (25, none near a tap, mostly the rider walking) could be dropped by it.
7. Done (`d56a86b`): stops found between stations. A stop with no closing heard and the same registered cell
   as the stop before (the rider aboard) is `StopRecord::between_stations`: zero speed kept, no dwell, the live
   dwell not started there, labeled "between stations" on the graph. Against the "train stopped" taps
   (an "at station" tap is within 60 s before 15 found stops, none of them labeled):
   the same cell and no closing, 0 of 87 tapped stops and
   9 of 47 untapped (5 labeled); the same cell alone, 1 and 13; a station named by the cell, 79 of 87 and 39
   of 47, no cue; the same site (ID / 256), 73 of 87 tapped, shared by neighboring stations; the time since the
   stop before overlaps. Untapped stops include real ones the rider didn't tap, so the 47 aren't all false.
   Next: the time since the departure (Lorimer's was 20 s after it), once more taps check it.
8. Partly done (`13ead04`, `dcb8c74`, `ec85c9f`, `ba8a343`, `9610696`, `0df034c`, `3dea235`, `66e043e`, `43aa82f`): the track between stations, from the MTA's GTFS
   shapes (`python/experiments/subway_track.py` to `crates/fusion/data/subway_track.bin`, 182 KB; `crates/fusion/src/track.rs`).
   Legs (`track::legs`): consecutive stops at named stations underground on the same train, no good fix between,
   a shape joining them, and no faster than a train can go (3 mph/s up and down, 55 mph between): 126 pairs
   over all sessions, 8 too fast (likely misnamed), 14 with a fix, none without a shape.
   - The map path: each leg's samples placed along the track by the share of the smoothed speeds' distance, each
     stop's dwell at its station. `stop_positions` (`just sweep stop_positions`, `data/work/track/`): 166 stops,
     dead reckoned a median 482 m off (90th 2902 m), joined 0 m (353 m) by construction; the honest score, each
     stop left out with its neighbors joined across it (88 stops): 180 m (737 m), against 830 m (3189 m)
     dead reckoned on the same stops.
   - The smoothed speed held to the track (`track::hold_to_track`, after `bound_gaps`): underground the speeds
     covered a median 0.46 of the track (90 legs; a tenth under 0.14), so they're scaled to it, at most 55 mph.
     Scored by `crates/cli/examples/legs_gnss.rs` (GNSS cut throughout, against the ride as recorded) on the 13
     legs with GNSS through them (3 rides): each leg's distance a median 462 m off to 82 m (the track 51 m);
     the per-fix speed RMS unchanged, 12.3 to 12.5 mph: the shape stays the IMU's. Forward and live speeds
     are untouched (`curves`, `holds` identical on `193658`).
   - The average speed from the last station (`StopRecord::from_last_station`): a median 0.8 mph off on those
     legs, against 5.0 from the raw speeds; a median 21.5 mph over all legs.
   - The maps draw the joined path (`Replay::map_path`, one point a second): `just map` as a colored line with
     the GPS fixes as a dot layer (a layer control hides either), and the app's past-ride map through
     `RideReplay.positions`, with the fixes as a thin dark line (not yet seen on the phone). On `193658` and
     `20261024-003357`, the fused point within 1.5 s of a fix is a median 1 and 2 m from it (90th 2 and 7 m).
     The path jumps up to 5.5 km in a second (at least 8 and 5 jumps over 700 m on those rides), where a sample placed at a
     station sits next to ones left dead reckoned: `track::join` leaves samples outside legs where they were.
   - Done (`66e043e`, `43aa82f`): the jumps. `track::blend` shifts each stretch `track::place` left alone by an
     offset blended over time between its ends' shifts (none at a good fix). `jumps` (`just sweep jumps`, one point
     a second): the largest step 5509 to 688 m on `193658`, 4443 to 333 m on `003357`, 7019 to 285 m on
     `151449`; what's left is the dead reckoning's own jumps (`135509` keeps 3305 m). Steps over 55 mph the dead
     reckoning doesn't make, all sessions, 523 to 636: a few km of offset spread over minutes reads as a smooth
     56 to 85 mph (82 more on `135509`); a blend by distance or speed might fix that. At good fixes unchanged.
     `stop_positions`: the 88 left out stops placed on the track unchanged; all 166 joined, 90th 353 to 40 m.
   - Tried, not committed: blending by distance instead of time. By the distance the unshifted path travels
     (time under 1 m), steps over 55 mph 1624 to 1551 (those the dead reckoning doesn't make 636 to 635),
     `135509` 115 to 32 and its largest step 3305 to 1121 m; but the largest step grows on six rides
     (`182135` 358 to 505 m, `011209` 153 to 220 m, `160303` 119 to 189 m, `214427` 1009 to 1071 m)
     where the dead reckoning's own jumps take the shift. Capping each step at 55 mph and adding 0.5 m/s
     for dwells undoes most of that but gets worse overall: 1663 steps (740 not the dead reckoning's).
     Good fixes and left out stops unchanged in both. Next: weight by the smoothed speed, or fix the
     dead reckoning's jumps first.
   Left: the live map still draws GPS fixes; the hold is a scaling after smoothing, not a distance measurement inside the
   filter's passes (that would fix the speeds' shape too); the live (forward) speed isn't held; the average isn't
   shown in the app; no licensing note for the MTA's data exists in the repo, for this or `open_track.bin`;
   Lorimer St on `193658` (681 m in 47 s, 32 mph) unchecked; legs need both ends' stations right, and a
   cell a station off within the time bound still passes.
9. Done (`80c1aa5`, `374cb64`, `37fc39e`; scored by `crates/cli/examples/walkoffs.rs`): walk offs found within
   20 s before to 45 s after an "alighted" tap, 27 to 29 of 33; false ones (no tap near) 5 to 4.
   90 St dropped (a short walk, then carried to the next stop felt, no departure between); 74 St found from the
   next stop met on the platform with no departure from this one (27 s after the tap, when the steps began);
   `181802` 6:34 pm found from a missed stop tried at each step window, not only at departures.
   Live speed over 6 mph between "alighted" and "boarded" taps 2742 to 2587 of 11540 s; dwell seconds held
   2233 to 2354 of 3570; `154951` gets 2 more untapped stops (589 to 594 s over 6 mph).
   Left: `184134` 7:24 pm (one step window, no stop found); `135509` 2:17 pm (walked before the tap, the
   train's stop missed); `182135` 6:27 pm is a step out (boarded 2 s later), not a miss; `002914` 1:15 am
   found 26 s before the tap (walking to the doors). The 4 false ones have no taps near (likely untapped ends
   of rides), and `002914` 3:04 am. The car beacon fading wasn't tried.
10. Done (`fee65f5`): 86 St on the 1 tagged with cell 29639550, from the 2026-10-24 ride (`003357`): no
    "at station" tap there, so the stop between 79 St and 96 St (tapped stopped 1:15:07 am; `UNTAPPED` in
    `station_cells.py`); the cell held from 11 s before that stop to 38 s after. On `214427` and `002914` it
    shows about 70 s before 96 St on the 2 (passing 86 St), and no other station has it.
11. Done (checked, no code change): LTE service the whole way under the East River on both crossings, no gaps.
    Brooklyn-bound (`193658`): 29205636/29205628 (PCI 246) from 7:58:36 pm, 29448828 (PCI 372) from 7:59:53 pm;
    Manhattan-bound (`003357`): 29448837 (PCI 373) from 12:45:12 am, 29205629 (PCI 247) from 12:46:09 am:
    the same sites, other sectors or carriers each way, all TAC 2434. None is in `station_cells.csv`, so no
    station is named mid-river; nor are 1 Av (28855173, 28855165; PCI 496) and Bedford Av (29256316, 29256324;
    PCI 501), which could be tagged. The one gap was at Bedford Av, 12:43:46 to 12:44:05 am (0:19).
12. Done (`1ff3f9d`): the L's car beacons are the same family (header, UUID, the 100 byte, 1 m power `c5`, 1 g
    at rest), under company ID `0x02fd`, not Apple's; `BleRecorder.kt` no longer calls them Piper's.
13. Taps: drop the mistaken train start at Graham Av if not deleted in the app. Checked: not deleted; on
    `20261023-193658` the "train started" at 8:04:45 pm, 0.9 s before the "train stopped".
    Dropping it means appending a deletion to the session's `events` (`delete_mark`), which agents don't do:
    left for the rider to delete in the app (Past rides). The only repo-side fixes are `subway.py`'s own `CORRECTIONS`, not read by the Rust.
14. Done (`4b14848`, `e300d7a`; 42 s to 5.3 s per replay on the laptop with `ae5a245` and `9a0d57c`): Past rides' "Ride" button replays the ride (`replay_session`), smoothed over
    all of it, on the live map and graph; cached per app build in the app's files. Not while recording, since
    a replay takes 483 MB at its peak on `20261024-003357` (1.4 GB before `9d2f900` boxed the large inputs)
    and 40 s on the laptop; unmeasured on the phone. Since `18d424b` the replay streams its inputs:
    on the 3:14:27 `20261022-002914`, 0.85 GB at its peak instead of 1.6 GB, most of the rest fusion's own.
    While recording (`8a3fe94`, `4de4cb1`): a cached replay shows as usual (`cached_replay`, no replay);
    an uncached ride up to 1:00:00 gets a "Replay now" button (about 300 MB by the laptop's peaks), a longer one still says to stop first.
    Unchecked on the phone.
    Faster scoring (2026-10-25, `f3032eb`, `e25983d`, `9e68286`, `ad48d6c`, `710b438`, `31296ca`):
    - Why the laptop was slow: `Replay::statuses` (on unless `no_statuses`, which the app sets) re-smoothed the
      whole ride at every status, through `vertical_moves`: 87% of the time. Now it smooths only while a recent
      elevator or escalator move needs it (`9e68286`): `20261021-215922` 6.1 s instead of 37 s, all sessions 2:22
      instead of 11:39 with statuses; without them, the 3 h ride took 19.5 s and the 51 min ride 4.7 s.
      Live, the status no longer slows as a recording grows.
    - Memory (`ad48d6c`): `Input` is 16 bytes (`f32` IMU samples, exact; rare inputs boxed), the buffer is
      reserved up front, and smoothing makes no sorted copy: the 3 h ride 0.53 GB at its peak instead of 0.81 GB.
      `position_inputs`, `forward`, and `dead_reckoned` are each under 4 MB even on it: not worth shrinking.
    - Still parts (`f3032eb`): `Options::skip_still` leaves out where GPS stays within 100 m for 10 min, but each
      end's minute; it finds exactly the four known stretches. Accidental sessions are in `scripts/sweep-skip.txt`.
    - `just score` (`710b438`, `31296ca`): every score of eight examples, five replays a session, 4 sessions at once
      capped at 1200M each: 2:12 for all 18 sessions (7:12 one at a time), each session's in `data/work/score/`.
    - Open: the backward pass carries state across a whole ride. Cutting any minute (even one near the end of
      `20261024-220437`) changes smoothed speeds 45 min earlier by up to 29 mph, while the forward pass is unchanged
      before the cut. Fixes should anchor the backward pass; something (bias, heading?) doesn't reset at them.
    Constant cost per status (2026-10-25, `9b1a626` to `b1d8e7b`): `Replay::status_cost_ns` times each status
    (`crates/cli/examples/statuscost.rs`, scratch). On `20261022-002914` (194 min) a status took 1.2 ms in its
    first 10 minutes and 48.6 ms in its last; now 8 us and 20 us, and feeding the inputs no longer slows.
    Each step's output hash (statuses, timeline, vertical moves included) is the same on all 18 sessions,
    and the stop records and timeline were checked equal to a full rebuild at every status.
    - `ee2efc9`: the elevator check uses the live speed kept at each sample (`Fusion::live_speeds`), not smoothing: 48.6 to 1.9 ms.
    - `a04e59f`: stop records settled once the next stop is 15 min old (`RECORDS_SETTLE_S`); only the rest
      recomputed, once a status (was three times, and at every live estimate): 2.2 to 0.46 ms.
    - `ffffc06` (`riding_lines`), `b514416` (`vertical::moves_since`, the status's moves from recent heights: 0.13 ms),
      `864d35e` (each pair of stations' track remembered: 25 us), `036a175` (dwell, route, fades by binary search).
    - `b1d8e7b`: `Fusion::timeline` (asked at every smoothing tick) keeps its settled segments: 4 to 74 us before,
      6 to 17 us now (the copy of what's kept).
    - Done (`bad96b9`, `1f11f16`, `51f946b`, `282afe6`, `28f997f`): the smoothing tick's FFI sends only what changed.
      `stops_since` and `timeline_since` take the count the app's copy has settled (`SettledList`) and return the rest:
      settled records never change, and the timeline's last kept segment still grows, so it isn't counted settled.
      On `20261022-002914` a tick sends 4.7 items in the first 10 minutes and 4.9 in the last (94.7 before);
      the getters take 3.3 and 16.9 us there (6.7 and 26.0 us for the whole lists), most of it the open tail's recompute.
      `crates/cli/examples/deltas.rs` (scratch, `replay_with_hook`) applied the deltas at every status (and every 7th)
      on all 18 sessions: equal to the whole lists at all 5,662 ticks; `deltas_rebuild_the_lists` does it on a synthetic ride.
      `smoothed()` already sends only the 10-minute window live (flat); `LiveTrack.applySmoothed` now looks only there.
      `Fusion::aboard_at` and the elevator check's departures by binary search and newest first: statuscost hash the same on all 18.
      Unchecked on the phone. Still per tick in Kotlin: copying the lists of references for the `StateFlow`s (one per fix too).
      Done (`da9ff08`): `missed_stop` by binary searches (stops by `recent`): 2.2 and 5.4 us a call in the first and last
      10 minutes of `20261022-002914` before, 1.4 and 0.9 us now (only 102 calls a ride). `f144c62`: `on_train_stop`'s
      door cycles and departures since the stop by binary search.
      `607ed02`: the live smoothing tick's track hold built every record, good fix, and leg of the ride;
      now from a dwell before the window: 4 and 547 us a tick there before, 2 and 21 us now.
      `statuscost` misses it (replay is whole-ride, one smoothing): measured by replaying with `Fusion::new`.
      statuscost hash the same on all 18, whole-ride and live.
      Done (`8447edf`): `subway_periods` keeps its periods and reads only the switches since its last call;
      every replay hash in `just score` the same on all 18.
      Permanent check (`f16e582`): `just score` (`ONLY=cost`) replays live (`Fusion::new`) with every status and
      smoothing tick (`fusion.recent_smoothed`, `stops_since`, `timeline_since`, as `LiveSpeed.kt`), reports the
      first and last 10 minutes, flags growth over 3x, and hashes every replay.
      On `20261022-002914` (194 min), first and last 10 minutes: status 6 and 15 us, tick 8 and 32 us
      (7 and 48 us before, timing `smoothed()`, which the FFI doesn't send), feeding 58 and 259 ms per 10 s.
      Open, the rider's call: live feeding is 95% the windowed backward smoothing pass (`smooth_window`),
      which re-runs every second with new GNSS over the whole window: up to 10 min after a measurement,
      or 1 h without one (`MAX_UNMEASURED_NS`). Bounded, not by ride length, but its last 10 minutes feed at
      1.27 s per 10 s on `20261021-215922` (12 ms in its first), 0.64 s on `20261021-155919`, 0.74 s on `20261024-003357`.
      - Re-sorting: once one input arrives late, every re-run sorts a copy of the window (10% of live time).
        Inserting it in place, as over a whole ride, changes the live output on 8 of 18 (it leaves the window sooner); not committed.
      - Prototype (not committed, `data/work/livewin/prototype.diff`, scratch `crates/cli/examples/livewin.rs`):
        redo only the last N min each re-run, older values keeping their last smoothing.
        Feeding in the last 10 min, base / N=2 / 5 / 10 min (ms per 10 s): `20261022-002914` 252 / 72 / 135 / 242;
        `20261021-215922` 1272 / 227 / 288 / 396; `20261021-155919` 638 / 129 / 236 / 431; `20261024-003357` 739 / 174 / 205 / 256.
        The rest still grows: the prototype still walks (and sorts) the whole window to skip the old part.
        The live estimate (`signs`'s input) is unchanged, as are all `just score` numbers (those replays are whole-ride).
        The final live smoothed speeds change badly: median 0 to 0.05 mph but p99 13 to 43 mph, max 20 to 106 mph,
        12 to 39% of values over 1 mph: a fix after a tunnel can no longer correct the tunnel before the last N min,
        which is what the window is for. Not worth it as is.
      - Prototype (not committed, `data/work/livewin/correction-only.diff`): re-run only after a correction
        (a fix, a stop anchor, walking), extending with the forward estimate between.
        Output nearly the same (final live smoothed speeds: p99 0 to 0.45 mph, max 0 to 19.5 mph; right after a fix
        p99 0.01 to 0.32 mph, max 9.1 mph), but little saved: the costly re-runs are the ones a fix makes.
        Feeding in the last 10 min, base / correction-only (ms per 10 s): `20261022-002914` 305 / 202;
        `20261021-215922` 1242 / 1198; `20261021-155919` 601 / 596; `20261024-003357` 702 / 800.
      - Prototype (not committed, `data/work/livewin/gap-only.diff`): the same, each re-run reaching back only to the
        previous correction. Feeding (first / last 10 min, ms per 10 s): `20261022-002914` 28 / 33;
        `20261021-215922` 16 / 259; `20261021-155919` 21 / 95; `20261024-003357` 19 / 197.
        But the output changes badly: final live smoothed speeds p99 up to 40 mph, max 291 mph on `20261022-002914`,
        17 to 38% over 1 mph on two rides; right after a fix p99 5 to 19 mph, max 6 to 120 mph.
        The backward pass started at the last fix lacks what the full window gave it (the raw GNSS speeds, the stop
        ramps before the previous correction), so it isn't pinned there as assumed. Not usable as is.
        Rider-agreed next step (2026-10-25): fold this into the speed estimate work after the Saturday fixes.
        The smoother reaching 45 min back (likely the accelerometer bias state) is the same non-locality;
        once that is fixed, retry gap-only. Until then the live smoothing stays as it is.

Laptop memory (2026-10-24): parallel replays over all sessions (about 1.5 GB each on the longest) crashed WSL.
`just sweep <example>` runs one over every session in turn, each capped at 3 GB,
and Claude Code runs in a `systemd` slice capped at 8 GB (`claude()` in the dotfiles).
15. Done (`a96ba38`): `just test-rust` passes again.
16. Done (`ee2efc9`, `7074047`, `2b458ae`, `93565b5`, `b816883`): elevators and escalators.
    The rider saw "elevator or escalator down" on trains. Against the taps ("boarded" to the next "alighted"),
    statuses saying one aboard 5 to 1, rides aboard 4 to 0: none while aboard (`Fusion::aboard_at`, a stop or departure
    felt with no steps around it since the last walk off), and a ride faster than 5 m/s by the live speed is a train.
    Only live speeds near GNSS (no GNSS, not judged) let 21 statuses aboard through: just boarded,
    no stop or departure of that train is felt yet. The one left is 55 s after boarding on `20261022-181802`.
    The rider labeled all 21 rides found (`crates/cli/examples/score/vertical_labels.csv`, `just score`'s
    `vertical:`): 11 elevators (7 at home, 2 in Edgewater, 1 in Flushing), 5 station exit stairs, 6 pressure noise;
    no elevators in the subway, and only one or two escalators in all. A ride under 40 s is stairs now
    (`VERTICAL_MIN_RIDE_S`): elevators 11/11, rides found 11 (all elevators), exit stairs as stairs 5/5.
    Left: the 6 noise moves (Edgewater, `002914`, 2:06 to 2:49 am) show as stairs; a rule that a ride in the
    subway can only be an escalator (the rider's idea) is untried.
    Open, from notes: false stairs up on the 180 St platform (Tue Oct 27, 10:04 pm, at 10:40 pm).
    Labels: "escalator" at Borough Hall's exit (Sat Oct 24, 8:51 am, at 9:31 am);
    "96 St I took the stairs, there are no escalators", run down two at a time (Fri Oct 30, 2:49 pm, at 2:50 pm);
    "boarded elevator" (Sat Oct 31, 4:58 pm, at 5:38 pm).
17. Which entrance the rider used (`70ef373`, prototype; waiting on the rider's check).
    Data: data.ny.gov "MTA Subway Entrances and Exits: 2024" (`i9wp-a4ja`, data of 2025-12-05, downloaded 2026-10-25)
    in `data/mta/entrances/` (2,120 entrances: type, entry/exit allowed, location; no street or corner names).
    Loader `python/experiments/subway_entrances.py`. Scratch: `crates/cli/examples/entrance_export.rs` (export per session)
    and `data/work/entrances/match_entrances.py`, report in `data/work/entrances/report.txt`.
    Method, sensors only: a GPS gap of 90 s or more while the replay was in the subway;
    entry is the last fix within 15 m before it, exit the first after it; nearest entrance allowing entry or exit.
    `subway_changes` alone lags losing GPS by about 30 s and flickers, so its ends were 70 to 120 m off.
    On `20261024-085154` the exit is Borough Hall's IRT stair (`232`, NE) 8 m away at 9:32:42 am, beside the app's stairs (+5.7 m).
    Open: entries and exits were found on 13 of 18 sessions; most are 2 to 30 m from the nearest entrance,
    with the next one often within 15 m, so the corner is unsure; to put in Rust once the rider confirms.
18. The speed bound is wrong (rider, 2026-11-08). On CBTC, a train can't use its motors to accelerate past 50 mph.
    Trains generally don't exceed 55 mph, except on long downgrades without signal timers to hold them,
    like the 60 St tunnel, where the rider expects about 60 to 62 mph
    (notes on Tue Nov 3, 6:01 pm and Sat Nov 7, 5:59 pm).
    So: no powered acceleration above 50 mph on CBTC lines; 55 mph as the usual cap, not a hard one;
    above 55 only coasting down a long grade without timers. Keep the bound per line or segment.
    Also from notes: "55 mph ± 28 mph is meaningless since it's supposed to be a cap",
    braking at 55 mph doesn't bring the speed down (Tue Oct 27, 10:04 pm),
    and the speed jumps from 0 to 55 mph right after leaving a station (Sun Nov 1, 1:50 am; Tue Oct 27, 10:04 pm).
    For the grade term: "accelerating on a downgrade" (Tue Oct 27, 10:04 pm, at 11:17 pm, south from Woodlawn on the 4).
    Done in part (2026-11-08, `bd29c7b`), not on the phone: a leg underground whose riding lines are all CBTC
    end to end (the 7, the L: `live_speed::CBTC_LINES`) is capped at 50 mph, not 55.
    Fast sweep (203 legs, `no_live_smoothing`): 5 legs changed, 4 on Fri Oct 23, 7:36 pm (the L),
    one 1.22 to 1.05 (peak 55 to 50 mph); counts unchanged (30 under 0.5, 91 within 0.7 to 1.3, median 0.95, 47 over 1.3).
    Done (2026-11-08, `1a4ea9c`), not on the phone: CBTC per track segment, not per line (the rider):
    `track::CBTC_TRACK` by line and GTFS stop IDs (the 7, the L, Queens Boulevard 50 St to Kew Gardens,
    Culver Church Av to W 8 St; sources in the code), and the bound on the motors, not the speed:
    on a CBTC stretch, the reach from a stand and the acceleration ceiling's rise stop at 50 mph,
    a faster speed known from GNSS isn't pulled down. `CBTC_LINES` is gone.
    Fast sweep (204 legs, against `HEAD` the same way): counts unchanged (30, 91, 48), median 0.96 to 0.95;
    3 legs changed, all Tue Oct 20, 6:41 pm on Queens Boulevard (one 1.00 to 0.89, peak 55 to 50 mph).
    Uncertain: Culver's date in service (no source confirms it); the 63 St line and Queens Boulevard east of
    Kew Gardens are left out until a source shows them in service.
    Open: off a leg (no station known, or no track ahead), the cap and the acceleration ceiling are still 55 mph;
    no grade term yet, so the 60 St tunnel's 60 to 62 mph coasting is still cut at 55 without GNSS;
    the 0 to 55 jump after a station and braking at 55 are untouched.
19. Braking into a station often shows as accelerating (Thu Oct 29, 1:23 am).
    Done in part (2026-11-08, `e5b11a3`), not on the phone: the replay's acceleration was the two passes' rates
    combined by the speed's weights, which change along the way, before the speeds are bounded and held to the track;
    at 18 St it read up to +2.2 m/s² while the shown speed fell from 18 to 4 mph.
    Now it's the shown speed's own slope over about a second each side (`accel_from_speeds`); speeds unchanged.
    Checked with `crates/cli/examples/brakesign.rs` (scratch, `skip_still`), the 20 s before each stop on 7 rides
    (Thu Oct 29, 11:07 pm to Sat Nov 7, 5:59 pm, 136 stops): samples where the replay's speed falls but its acceleration
    reads positive went from 74 to 38.
    Open: the live acceleration's sign was already right (the rate of the live speed shown); what's wrong is the live
    speed itself, which rises into many stops: it falls over those 20 s at only 79 of 147 stops (8 of 11 on Thu Oct 29).
    Underground, braking integrates past zero and the shown magnitude climbs again (18 St, 96 St on Thu Oct 29),
    or the live speed is stuck low then rises (item 27).
    Clamping the signed speed at zero once the axis sign is known was tried and dropped: on those 7 rides,
    stops where the live speed falls went from 71 to 55.
    The replay also reads positive where its speed itself rises before a stop (Sat Nov 7, 2:29 pm).
    Looked into (2026-11-08), nothing kept: `crates/cli/examples/brakecause.rs` (scratch, `no_live_smoothing`,
    same counts as `brakesign`) buckets the 68 of 147 stops where the live speed doesn't fall:
    30 where the replay itself speeds up in the window or the live speed runs away with no braking seen,
    16 held at 0 by a likely stop or transfer then released (some moving: Bowery at 44 mph on Thu Nov 5, 10:06 pm;
    item 27's hold), 9 with no approach (the replay under 3 mph), 8 crossing zero then rising, 5 pinned at the cap.
    The signed speed below zero isn't braking past the stop: often the whole leg runs negative with the right shape
    (DeKalb Av, Sun Nov 1, 1:50 am), so showing it as 0 made it 51 of 136 (from 71);
    re-checking the axis sign at each departure without GNSS, 69.
    The cap: on Thu Nov 5 the bias reaches -1 to -2 m/s^2 underground (a gravity tilt leak, not the accelerometer)
    and pins the speed at 55 mph through braking into 50 St and 14 St.
    Keeping the caps from moving the bias: 79 to 73; resetting the bias at each stillness without GNSS: 79 to 75.
    Not a gravity tilt leak (checked 2026-11-08, stash "item19: gravity tilt leak probe"):
    at each stop 2 s still, the accelerometer's mean against gravity as carried into the stop, along the axis.
    On Thu Nov 5, 10:06 pm, at the three underground stops 24 to 36 min from GNSS,
    the live gravity leaked -0.09, -0.10 and -0.34 m/s^2 while the bias read -1.59, -0.67 and -0.29;
    gravity carried by the gyro alone since the last stop leaked -1.49, -3.96 and -0.36 (the gyro's drift),
    so the 600 s time constant helps and dropping it would make it worse.
    Where the bias comes from (2026-11-08, stash "item19: bias source probe"; per-second bias changes by source):
    on Thu Nov 5, 10:06 pm, the caps move it (55 mph cap, leg cap, acceleration ceiling), but they follow
    the input: the along-axis acceleration itself averages -0.7 to -2.4 m/s^2 over 30 s for 15 min underground.
    Two causes: the first-order turn `v - (omega x v) dt` lengthens gravity and the travel axis each sample,
    so with the phone in hand the axis grew to 1.3 to 1.8 long and gravity about 2 m/s^2 too strong,
    scaling the along-track acceleration up to 1.8x; and gravity's direction drifting with the gyro.
    An exact rotation keeps both at length (axis 1.00, peak bias -2.05 to -1.36) but scored worse:
    falls 81 to 78 of 147, `liveratio` under 0.5 30 to 41, within 91 to 78, median 0.95 to 0.81,
    so the speed's scale underground had been leaning on the lengthened axis; not kept.
    Also not kept: the caps updating the covariance like an exact measurement (bias unchanged, peak -2.15),
    and no cap bias moves while a likely stop holds (only delays it).
    During the Canal St dwell (shown 0), the estimator's own speed ran to -55 mph and the caps pulled the bias down.
    Open: the exact rotation with the speed's scale found elsewhere (the leg cap, anchors), gravity's gyro drift,
    and item 27's holds.
    With the exact rotation in, what makes the speed low (2026-11-08, stash "item19: exact rotation (test passes)
    with scratch flags", `data/work/item19bias/`; nothing kept, none reached the baseline):
    not the axis: re-measured at each departure from the mean acceleration since the stop,
    it already pointed within cos 0.95 to 1.00 on Sat Nov 7, 5:59 pm;
    not the caps: with the leg cap, the acceleration ceiling or the anchors off, under 0.5 stays 39 to 49.
    It's the bias learned underground: with no bias without GNSS, most of the 15 legs that newly fell under 0.5
    recover (Sat Nov 7, 5:59 pm, 27:59 in: 0.53 to 0.97), but others need it (under 0.5 37, within 76).
    The ungated stop and door zeros and the caps swing it by 0.2 to 0.6 m/s^2 at a time
    (+0.65 to -0.29 in 4 min on Sat Nov 7, 5:59 pm), far more than an accelerometer bias moves,
    and a wrong one drags the speed (-0.15 m/s^2 is 10 m/s over a 75 s leg).
    Scores (`liveratio` under 0.5 / within / median, `brakecause` falls; baseline 30 / 91 / 0.95, 81):
    exact rotation 41 / 78 / 0.81, 78; bias frozen at its last GNSS value 40 / 72 / 0.88;
    bias walk x4 48 / 90 / 0.87; x0.25 35 / 82 / 0.88, 74; accel noise x2 43 / 73 / 0.85, 78;
    walk x0.25 and noise x2 38 / 82 / 0.90, 68; bias held within 0.2 m/s^2 43 / 86 / 0.84, 78;
    stops not moving the bias 47 / 67 / 0.82, 77; nor the caps 37 / 80 / 0.96, 73.
    A bias that only measurements of it move (GNSS, the cars), with the exact rotation (2026-11-08,
    stash "item19: exact rotation with only measurements moving the bias"; `liveratio` only, nothing kept):
    stops, doors and caps correcting only the speed (a Schmidt update: the bias and its variance kept),
    and the anchors too, 42 / 74 / 0.89; anchors still moving it, with the bias walk x4, 37 / 71 / 0.96;
    x0.25, 36 / 77 / 0.95 (over 1.3: 55, 67, 61, against 47).
    So the stop zeros were absorbing a real along-axis error that a frozen bias leaves in the speed.
    Open: what that error is (gravity's gyro drift, the axis between departures), rather than tuning the bias.
20. Done, unchecked on the phone (`34cfce3`, `6b7d63f`, `10fdd80`, `00681c8`): the live map underground, noted on
    four rides, the latest Sat Nov 7, 5:59 pm. `Fusion::recent_track_path`, drawn under the GPS path:
    each leg between stations once the train stops at its far station (placed as a past ride's by `track::Along`);
    into a tunnel from the last good fix to the first station (`track::to_station`; Queensboro Plaza to 5 Av/59 St
    on Sat Nov 7), and out of one; the leg in progress through the next 4 stations on the riding line
    (`track::ahead`), by the live speeds' distance times the median of the window's legs' track over it
    (a stopgap: see item 27; the live speed underground has no steady factor).
    The map follows the newest point of either path. `crates/cli/examples/livetrack.rs` (`skip_still`):
    legs within 4 and 7 m of the past-ride path; the newest point at each tick against the whole-ride replay's place:
    Sat Nov 7, 5:59 pm median 0 m, 90th 54 m, max 635 m; Thu Nov 5, 10:06 pm 0, 653, 1033 m. 173 to 225 us a tick.
    Left: the leg in progress lags or leads by up to 1 km (live speeds underground); a station stop the app
    misses makes a leg past it; walking into a station (no train path from the street) isn't drawn, by design.
    The Queensboro Plaza walk off at 6:11 pm on Sat Nov 7 (then a departure at 6:13 pm) is the app's guess, not the rider's:
    the rider doesn't remember changing trains there; check it against the steps.
    Notes that the map wasn't drawn underground: Mon Oct 26, 8:08 pm (at 8:48 pm), Tue Oct 27, 10:04 pm (at 11:51 pm),
    Sat Nov 7, 5:59 pm (at 6:28 pm).
21. Wrong lines and stations in the status:
    "6 train doesn't go anywhere near Astoria Blvd" (Sat Nov 7, 2:29 pm);
    "E at 59th but the E doesn't go there" (Sat Oct 31, 4:58 pm);
    103 St on the 3, which doesn't stop there (Thu Oct 29, 11:07 pm);
    14 St while at 18 St, corrected later (Thu Oct 29, 1:23 am).
    A line must serve the station it's named at.
    Done in part (2026-11-08), not on the phone: a route that doesn't stop at the station's complex
    (`stations::serves`) isn't named, and the riding lines narrow to those that do (`be81507`);
    a stop since the last door cycle names the station, not that cycle's (`169868d`).
    Replayed (`skip_still`): no route at Astoria Blvd until the N matches, none at 59 St (R211 cars named),
    none at 103 St; 18 St from 1:25:38 am instead of 1:26:08 am.
    Left: the station itself can be wrong, from the cell at speed (103 St on the 3; the rider was on the 3,
    and once the 1 is matched there); a route stale from before a walk (the 6) still needs the station to show.
    Stale line, done in part (2026-11-08), not on the phone: the 7 on Tue Nov 3 was matched at Queensboro Plaza,
    named at its 7 platform (the MTA lists the 7 and N/W ones at one spot), with the W's stops no longer in the feeds.
    Platforms at one spot count as one, and only a line serving every stop's complex matches (`83660f7`);
    the riding lines narrow by complex, so that stop doesn't start them over as the 7 (`018956c`).
    Replayed (`skip_still`): the W at Queensboro Plaza, then no route to Times Sq instead of the 7;
    the N, not the 7, there on Sat Nov 7, 5:59 pm; no 6 at 125 St on the bus on Sat Nov 7, 2:29 pm;
    Sat Oct 31, 4:58 pm names the C for the A from 125 St (R211 cars either way);
    Thu Oct 29 (1:23 am, 4:29 pm, 11:07 pm) and Fri Oct 23, 7:36 pm unchanged.
    Tried and dropped: no route after walking since its newest stop (2 step windows, as `walked`):
    steps in the car dropped the right 3 for minutes on Thu Oct 29, 11:07 pm.
    Still left: the 3 kept after the walk across 96 St to the 1 (Thu Oct 29, 11:19 pm) until 103 St is named.
    `just test-rust` fails on the untracked `crates/cli/examples/entrance_export.rs` (no `no_live_smoothing`).
    A or C on Sat Oct 31, 4:58 pm, done (2026-11-08, `41c2dd4`), not on the phone: the A, from the recorded GTFS-RT.
    That weekend the A ran local below 59 St (track A2), then express from 59 St to 125 St (A4),
    and the C ran only from 59 St north.
    The A's trip `097250_A..N` lists 42 St-Port Authority, 50 St, 59 St, 125 St and 145 St
    within a minute or two of the rider's stops (left 42 St 5:12:28 pm, 125 St at 5:24:28 pm);
    the C left 59 St at 5:12:52 pm, before the rider got there.
    The status dropped the A because the MTA lists 50 St for the C and E only, and the E stopping there won ("says E at 59th").
    A trip the feeds list at a stop's own platform now counts as serving it.
    Replayed (`skip_still`): the A from 50 St on, through 59 St and 125 St to 168 St, but for one C at 125 St (5:25:59 pm);
    no change on Thu Oct 29, 4:29 pm, Tue Nov 3, 6:01 pm, or Sat Nov 7 (2:29 pm, 5:59 pm).
    No line from Queensboro Plaza to Times Sq on Tue Nov 3, done (2026-11-08, `1d4f5bf`), not on the phone: the W kept.
    The cell names those stops a station early (Broadway at 30 Av, Queensboro Plaza at 39 Av),
    and once the feeds settled the W's Queensboro Plaza at 6:12:10 pm, 1:27 after the stop named there,
    nothing matched the newest stop, and the stops after, through the 60 St tunnel, have no station.
    A trip matched at each of the older stops is now kept while no newer stop rules it out
    (a platform its line doesn't stop at, or a stop before it reached only after the rider left);
    never one from the newer stops' times alone (an N a station ahead lists them within seconds).
    Replayed (`skip_still`): the W from Broadway through Queensboro Plaza, Lexington Av/59 St, 5 Av/59 St and 57 St-7 Av,
    but for 10 s walking blips (6:12:43, 6:13:33, 6:19:03 pm), and still the W at Times Sq on the Q (item 22);
    Thu Oct 29, 4:29 pm, Sat Oct 31, 4:58 pm and Sat Nov 7 (2:29 pm, 5:59 pm) unchanged.
22. Missed transfers and switches: 3 to 1 (Thu Oct 29, 4:29 pm); "switched to Q" (Tue Nov 3, 6:01 pm; Sat Nov 7, 5:59 pm).
    Looked into (2026-11-08, `skip_still`), nothing fixed: all three are changes across the platform
    with almost no steps (7 s, 17 s; at 96 St the rider stood 1:42), so no walk off is found
    and no car beacons were heard (`train_change` from beacons can't fire).
    96 St: the chime went from NTT (the 3) to R62 at the 1's closing, 10 s before leaving.
    Tried a train change from the chime before the stop against the closing's (stash "item22: chime train change"):
    it splits Thu Oct 29's leg at 96 St (the 1 after), but over the 37 sessions with legs
    it flagged 13 stops not walked off at, only 3 of them real changes (also Wed Oct 21, 6:27 pm, 96 St);
    the chime family is too noisy for it alone.
    57 St-7 Av, N/W to Q: the same chime, and the feeds can't tell: the Q's trip lists no 57 St stop,
    and the N the rider left reached Times Sq within the slack of the Q, so the N, matched at 5 Av/59 St too, wins.
    Left: a sensor sign of a change across the platform; Tue Nov 3 also shows the 7 from Queensboro Plaza
    to Times Sq (the stops between have no station; item 21's stale line).
    The rider's ground truth for Tue Nov 3, 6:01 pm: the W from 30 Av to 57 St-7 Av, then the Q to Times Sq
    ("switched to Q" at 6:19 pm, right after tagging the Q's car 5048).
    Checked again (2026-11-08) with the recorded GTFS-RT: the W `108200_N..S72R` and the Q `109100_Q..S36R`
    are both at 57 St-7 Av at 6:19:30 pm, the rider's dwell (6:19:17 to 6:19:32 pm).
    Settled, the W reached 49 St at 6:22:26 pm, after the rider stopped at Times Sq (6:21:49 pm): not the W.
    But the Q's last Times Sq time is 6:24:06 pm, 2:17 late, so the feeds don't confirm the Q either,
    and live, at Times Sq, the W was still predicted there at 6:22:25 pm, so the status shows the W until the rider walks off.
    Left: the change, live; the express passing 49 St (named by the cell at 6:20:33 pm) is a sign the W stops there.
    The rider (2026-11-08): the GTFS-RT is often off by a little, so its times can't settle it,
    but the train didn't stop at 49 St, so it was a Q on the express track.
    Next: use the stop pattern itself (a station passed without stopping) to rule out the locals that stop there.
    Done (2026-11-08, `b8f5187`, scored by `crates/cli/examples/passes.rs`, `a1d1cfd`), not on the phone:
    a station passed (`Fusion::passes`) is one the radio named in the subway with no lurch (kept or not),
    stop, departure or door cycle within 20 s, the live speed at least 3 m/s while named,
    no stop or lurch at its complex within 2 min, and stops felt before and after at other stations;
    decided 15 s after. The riding lines and the feeds' trips drop the lines stopping at its platform;
    with none left, the rider changed at the stop before (57 St-7 Av), to its lines that passing leaves.
    The status's lines shown with no route are the riding lines aboard.
    Replayed (`skip_still`), Tue Nov 3: 49 St passed (6:20:25 to 6:21:22 pm), the W still shown to 6:21:43 pm
    (the pass is decided only then), the Q at Times Sq from 6:22:03 pm until walking off.
    Over the 37 rides with legs, 33 passes: express 2/3 past the 1's stations, the 4 past Spring St and Canal St,
    the F past 36 St; the 1 to the 2/3 after 96 St (Mon Oct 26, 11:22 am; Sat Oct 24, 9:09 am)
    and from 72 St (Mon Oct 26, 8:39 pm); none on the A/C on Sat Oct 31.
    Without the stops felt around it, missed stops read as passes: the 1 from 137 St to 96 St
    (Sat Oct 31, 10:13 to 10:20 pm, no stop felt or tapped at four stations) dropped its route.
    Left, likely wrong: 79 St at 3:13 am (Thu Oct 22, 12:29 am), when every train is local there;
    the live speed underground isn't evidence (it ran to the 55 mph cap through the Q's pass);
    the change is shown only 35 s after the station's last naming.
    Done (2026-11-08, `cb3d2db`, `03f018d`), not on the phone: 79 St at 3:13 am on Thu Oct 22 was a real pass.
    That ride has no feeds, but the MTA's schedule has an overnight 3, express there
    (Times Sq 3:08:30 am, 72 St 3:13 am, 96 St 3:16 am); the train left Times Sq at 3:08:18 am and 72 St at 3:12:10 am,
    stopped at 96 St at 3:15:03 am, with nothing of a stop at 79 St or 86 St; the next 1 left Times Sq at 3:12:30 am.
    Only the chime at 72 St, heard as an NTT car's (the 2's), says otherwise.
    A pass now counts against the lines stopping there only if no trip the feeds listed rules it out:
    one at the stop before's station, from then until the pass, heading toward it, that doesn't stop there
    (`Pass::skipped_by_a_trip`); without the feeds polled from the stop on, it counts as before.
    It's decided once a lurch after its window is scored, else 8 s after (was 15 s);
    the stop felt after isn't needed when such a trip skips it (an express can run over 2 min to its next stop).
    Replayed (`skip_still`): the Q on Tue Nov 3 from 6:21:53 pm (was 6:22:03 pm), before the doors opened.
    Over the 37 rides with legs, 42 passes (was 33): 27 with a trip skipping them, 14 with no feeds,
    and Franklin St (Mon Oct 26, 11:36 am) not counted, on the 3 after a false stop at Houston St.
    The 9 new ones are express 2/3 runs past the 1's stations, 49 St on Fri Oct 30, 4:04 pm,
    and 49 St's BMT platform named from the 2 at 50 St; no stop was tapped during any.
    Left: passes of another line's platform at a complex (59 St's A/C/B/D one from the 2) rule out lines the rider isn't on;
    the 20 s after the last naming stays, for a cell naming the next station while the train is still stopping.
    The rider (2026-11-09) confirms the 3 at 79 St on Thu Oct 22: Times Sq to 96 St in about 6 min, getting off there,
    so it ran express and the pass is real (an earlier doubt, 2026-11-08, is withdrawn).
    Still check whether the other passes with no feeds (14, several at 79 St at 15 to 42 mph) are stops missed.
    Houston St (Mon Oct 26, 11:36 am) should be a stop, the rider says;
    a false stop may be just before or after it, which may be what hid the Franklin St pass.
23. Done, not on the phone (`4affdf5`, `a27cacf`, `e1d4f91`): graph, map and replay (Mon Oct 26, 8:08 pm).
    The graph's time axis shows round 12-hour New York times, as many as fit (`clock_ticks`).
    The live map pins each tap where the rider was (`positions_at`), underground once its leg is drawn.
    A replayed ride has a "Taps" button: its taps as pins on the map and lines on the graph, labeled where there's room;
    the replay's buttons are on their own row with fixed widths, so none moves.
    The tap lists show 12-hour times too.
24. Done, not on the phone (`d6f8ffe`): "says SMEE chime on NTT 5 train" (Tue Oct 27, 10:04 pm).
    Arriving at 3 Av-149 St, a 0.29 s tone at 523 Hz alone confirmed a closing as the SMEE chime,
    and the 2's and 5's fleets weren't in `car_models`, so it was shown. Now the 4 and 5 are R142 and R142A,
    the 6 R62A and R142A: a SMEE chime on the 2 or 5 says nothing, and the 6 names its model.
    Over the 37 rides with legs, the only other change is the 6 named. The classifier itself is still noisy:
    requiring both of a family's pitches lost half the confirmed chimes (the 1's are often 641 Hz alone),
    and one-pitch SMEE tones are as often wrong as right (523 Hz alone on the N and W's R160s, 0.8 s long).
    Left: the N, Q, R and W fleets; a better chime feature than the pitch votes.
25. Audio. Recording now takes every microphone at its native rate (`4ec4350`, `6000ec9`), untested on the phone:
    check a ride has 48 kHz, 3 channels, no sameness or missing-channel `Health` reports, and the microphone info.
    Checked (2026-11-09) on 0.1.18's two rides (Sun Nov 8, 2:40 pm; Sun Nov 8, 9:29 pm): 48 kHz, 4 channels by index,
    measured 48000.00 Hz, no discontinuities, but only one microphone's sound.
    Channel 1 is an exact copy of channel 0 in 226 of 228 10 s windows (Sun Nov 8, 9:29 pm), channels 2 and 3 are all zeros;
    channel 0 is -25 dBFS median, little clipping, real content up to 16 kHz.
    The source fell back to `VOICE_RECOGNITION` (no `UNPROCESSED` on this phone), which is likely mono;
    the session lists the microphones `bottom` and `back`.
    The `Health` check reported only the copy, in the first 10 s: also report silent channels, and check over the whole ride.
    Next, untested: try `CAMCORDER` or `MIC` with `CHANNEL_IN_STEREO` (bottom and back), or index masks on those sources,
    then drop channels that are copies or silent rather than storing them (FLAC hides it, but it's not more data).
    The rider (2026-11-09) finds the 48 kHz recordings "massively better than before".
    0.1.19's `MicProbe` (`adb shell am start -n dev.khyber.ride/.MainActivity --ez probe_mics true`, about 7 min)
    tries 7 sources x default/bottom/back x 7 layouts x 16-bit/float into `mic-probe.tsv`.
    First run (`data/work/mic-probe/`): only the first 7 rows have sound, the rest silent once the app left the screen
    (Android silences a background app's microphone), so rerun with the app open.
    So far `DEFAULT` with index masks of 2 to 4 channels gives the bottom and back microphones as 2 distinct channels,
    a 3rd or 4th silent. The phone's audio policy lists 2 built-in microphones (`bottom`, `back`; a "2 Mic" device),
    channel masks up to 4 (0x60000c: left, right, top left, top right), and 24-bit (`PCM_8_24`) at 48 kHz: try float capture for 24 bits.
    Second run (0.1.21, in a foreground service, `data/work/mic-probe/mic-probe-2.tsv`), all 294 opened, none refused:
    - `VOICE_RECOGNITION` (what rides use now) and `VOICE_COMMUNICATION` are one microphone, the bottom:
      every 2-channel layout gives an exact copy, and only the bottom is active. So every ride so far is mono.
    - Both microphones active and 2 different channels: `DEFAULT`, `MIC` and `CAMCORDER`, by index mask (2 channels) or stereo,
      on the default or bottom input (asking for the back one often gives two equal channels).
      `CAMCORDER` most reliably (about -36 and -14 dBFS: bottom and back differ a lot).
    - `UNPROCESSED` opens although the phone says it isn't supported, but only the bottom is active,
      its second channel 40 dB quieter (not the back microphone), and mono near full scale (-2 dBFS): suspect.
    - A 3rd or 4th channel is always silent or a copy. Float opens everywhere; whether it carries 24 bits is unchecked.
    Left: levels alone can't tell a second microphone from a processed copy of the first; next probe, after asking:
    correlation between channels, peaks and clipping, and a few seconds of each candidate saved to listen to.
    Then record with the best (likely `CAMCORDER` or `MIC`, 2 channels, float), only after the rider hears it.
    Third run (0.1.22, 3 s clips, stopped by the rider after 27 of 72; `data/work/mic-probe/clips-3/`):
    asking for the bottom or back input gives one microphone twice (correlation 0.98 to 1.00 at lag 0);
    only the default input with 2 channels by index gives 2 microphones (`DEFAULT` 0.85, `MIC` 0.41, `CAMCORDER` 0.08,
    best at lags of 1 to 41 samples, under the 0.4 ms the 12 cm between them takes).
    Float by index carries more than 16 bits; float by stereo mask doesn't. So: default input, index mask 2, float.
    Left: which of `DEFAULT`, `MIC`, `CAMCORDER` (the rider listens to the 3 clips), and which channel is which.
    Fourth run (0.1.23, rider talking, `data/work/mic-probe/clips-4/`): all three give 2 microphones,
    the second channel consistently 10 to 13 samples later (0.2 to 0.3 ms) and 5 to 9 dB louder.
    In float the second channel reached full scale (1.0) with `MIC` and `CAMCORDER`, not with `DEFAULT` (peak 0.61).
    Done, on the phone (0.1.24, installed 2026-11-09 overnight; `1777c94`, `b055932`, `b7c41ff`, `f344f75`), untested on a ride:
    rides record `DEFAULT` on the default input (the rider heard little difference between the three), 2 channels, float
    (`pcm_f32le`; 16-bit only if refused); recompression makes 24-bit FLAC only when every sample is exactly k/2^23,
    else keeps the floats under `zstd`. Check on the first ride: 2 different channels, no `Health` copy/silence report,
    and whether the chunks became FLAC (if not, the floats aren't plain 24-bit and storage is about 2x).
    Installed in 0.1.23 (`54c1e3c`): the probe's notification has a Stop button, and `probe_clips` is only those 6 tries (about 25 s).
    Then benchmark lossless codecs on it (FLAC, WavPack, MPEG-4 ALS, OptimFROG),
    and try lossless on OneDrive with Opus on the phone, dropped on the phone only after the upload is verified.
    Opus, measured on Thu Nov 5, 10:06 pm (16 kHz mono, 1:03:16): FLAC 80 MB, Opus 64 kbps 32 MB, 32 kbps 15 MB.
    Estimated for 48 kHz and 3 microphones: FLAC 550 to 700 MB an hour, Opus at 48 kbps a microphone about 65 MB.
    Uncoupled channels (mapping family 255), so the microphones stay separate; check announcements and chimes survive.
    Find car 2239's train, the only one with the 1's new automated announcements.
26. Labels from notes to learn from: the W parked on the center track at 30 Av (Sat Nov 7, 5:59 pm),
    the M4 bus (Thu Nov 5, 4:30 pm), the Grand St to W 4 St shuttle bus (Thu Oct 29, 10:22 pm), car numbers.
    All notes are listed by `ride summary`; a copy of them is in `data/work/notes.txt`.
    More, all as notes: R179s on the A (Tue Oct 27, 10:04 pm, at 11:44 pm, car 3011, the second car)
    and on the C (Thu Oct 29, 5:20 pm, at 5:23 pm, car 3146); on that ride "transferred to A" at 5:33 pm (car 4322)
    and "transferred back to the same C" at 5:44 pm (car 3149);
    "door l8" on the F (Thu Oct 22, 6:18 pm, at 6:28 pm, car 8909, second car from the back);
    car 1241 as a note (Mon Nov 2, 6:04 pm, at 6:27 pm, after tagging 1242);
    "first car" as a note (Fri Oct 30, 4:00 pm, at 4:04 pm, car 6785).
    Regenerated on 2026-11-08 from all 60 sessions, with clock times and the tags around each note:
    63 notes on 21 rides.
27. The live speed underground (rider, 2026-11-08: "just fix the live speed"). Mostly done (2026-11-08), not on the phone.
    Scored by `crates/cli/examples/liveratio.rs` (live distance over track length per leg; `skip_still`,
    `Options::no_live_smoothing` since `534a8bc`, byte-identical and 6x faster), over the 37 sessions with legs
    (`data/work/sessions-with-legs.txt`) at 5 jobs: about 1 minute. Compare with `data/work/cmp_liveratio.py`
    (legs matched by session and start). On 203 matched legs, baseline (`d0d7a43` plus the grid) to now (`f85f5cd`):
    within 0.7 to 1.3 of the track 63 to 82, runaways over 1.3 61 to 43, 90th percentile 2.05 to 1.66,
    under 0.5 45 to 46, median 0.85 to 0.85.
    Causes found and fixed:
    - Held at 0 after a stop (`4fec3d9`): the departure surge must hold 0.5 m/s^2 for 3 s; leaving Fulton St (2/3,
      Mon Oct 26, 8:08 pm) it stepped 1.0 then settled to 0.24 to 0.4, so the likely stop held the speed at 0 the
      whole leg. A weaker start (`train_stops::Event::Started`) now ends a likely stop only; counted as departures,
      weak starts added 42 untapped departures at 17 to 29 mph (false; `missed_stop` would add stops).
      Scored against the rider's 354 "train started" taps by `crates/cli/examples/departs2.rs`: 314 found.
    - Axis left behind by a phone turned in a hand (`8497e55`): once GNSS sets the axis, only tilting turned it;
      58 deg at 1.2 rad/s at 50 St (Sat Nov 7, 5:59 pm) left it 60 to 80 deg off. Faster than 0.25 rad/s about the
      vertical (no train turns over 0.18) now turns the axis with the phone.
    - Caps from known inputs only (rider: never the estimate itself; `988599e`): what a train reaches in the time
      since the departure, and the fastest any train goes on a leg to the next station its line stops at
      (`track::stations_ahead_m`, the farthest where lines disagree). Rider wanted an average-speed cap; with only
      accurate inputs, the leg's own average isn't known live.
    - IRT recorded peaks (`0c5a27a`, `f85f5cd`): the IRT Capacity Study (STV, July 2020, archive.org
      `irt-capacity-study-final-reportt-redacted`, in `data/downloads/speed-capacity-docs-20261108/`) charts event
      recorder speeds per stretch; `python/experiments/irt_digitize.py` and `irt_peaks.py` read 328 stretches from
      26 of 28 charts into `crates/fusion/data/irt_recorded_peaks.csv`; the cap is at most the peak plus the more of
      5 mph and 15% (speed limits changed since 2018 "somewhat, not massively").
    - Anchors (`19fb8a2`, `2074d2c`): a stop at a named station, or a station the radio names on the way (half
      weight), is a known place; the estimate's distance against it corrects the bias and speed.
    - GNSS is never capped (`01c19ce`): the 55 mph cap, the acceleration ceiling and the leg cap stand aside with
      GNSS in the last 5 s.
    - Stuck low (`29984be`): of the 46 legs under 0.5, 31 were held at 0 by a likely stop, 15 integrated too slowly
      with no hold, 1 by a transfer. 16 of the 31 held by a lurch 6 to 20 s after the live departure, no closing
      heard (motoring cut off); a likely stop within 20 s of a departure no longer holds (live only: the stop
      records never kept them). Under 0.5 46 to 30, within 0.7 to 1.3 82 to 91, median 0.85 to 0.95,
      over 1.3 43 to 47; 19 legs nearer the track by over 0.15, 3 further (4:13 on Mon Oct 26, 8:08 pm, 0.88 to 1.63).
    - Held at 0 then released (2026-11-08, `brakecause`, 147 stops): of the 16, 9 by a likely stop and 7 by a transfer.
      A transfer clamped a GNSS speed: boarding at 30 Av missed (Tue Nov 3, 6:01 pm), GNSS read 26 to 30 mph
      shown as 0; with GNSS in the last 5 s the clamp stands aside (`4178971`).
      Bowery at 44 mph (Thu Nov 5, 10:06 pm): a lurch 13 s after leaving, before the Williamsburg Bridge,
      was kept as a stop, and a walk along the M 245 s later counted as walking off at it;
      a stop within 20 s after a departure takes no walk off now (`59b151b`, offline too).
      Falls 79 to 81 of 147, held 16 to 13; `liveratio` unchanged (30 under 0.5, 91 within, median 0.95, 47 over).
      Open: likely stops with the departure missed, held the whole next leg: Times Sq to 50 St
      (Sun Nov 1, 1:50 am, 2 min dwell, then a slow pull at 0.2 to 0.4 m/s^2, under the surge's 0.5 and the weak start);
      the rest held under 8 mph by the replay (walking, slow pulls into stations).
    Open: stuck-low legs (30 under 0.5): held from a stop with no departure found, offline or live
      (the leg starts at the stop: Wed Oct 21, 6:21 pm, 11:28, the phone in hand); lurches mid-leg 40 to 80 s
      after the departure; slow integration without a hold (15);
    runaways (47): more mid-leg anchors (more tagged cells); the 3's charts (F.2-15, F.2-16) don't digitize;
    use the study's average peak speeds (B-8) and measured stop braking (2.11 and 1.83 mph/s, cars 8116, 8112).
    The MTA's Speed and Capacity Review (mta.info/document/13046) blocks scripted downloads: the rider is to save it.
    The rider's note on what works: with the phone very still, leaving a station from a correct stop,
    the numbers look accurate by eye (Mon Oct 26, 8:08 pm, at 8:18 pm).
28. Learn the trains' acceleration curves (speeding up, cruise, braking) from the above-ground rides,
    per line and car model, to bound and shape the speed between stations (rider, 2026-11-08). Done (2026-11-08).
    Open track grid (`b7317bc`, `0377dfb`, `6f988ba`): plus OpenStreetMap's subway ways not tagged as tunnels
    (portals, ramps, open cuts, bridges, viaducts), but not untagged ways in New Jersey (PATH under streets);
    the rider's M60 on Broadway at 116 to 120 St over the 1 (Sat Nov 7, 2:29 pm) stays 177 m or more off it.
    Found: the Manhattan Bridge, the 1's 125 St viaduct (three rides), and the Williamsburg Bridge on the M
    (Thu Nov 5, 10:06 pm: the "J across the bridge" was the M).
    Curves (`09b1b89`): `crates/cli/examples/accelcurves.rs` over 14 runs on 9 rides (speeds over 6 to 8 s),
    fit by `data/work/track/fit_curves.py` (constant force, 1/v, 1/v^2) to each 2 mph bin's 90th percentile:
    fleet `min(1.00, 6.5/v, 75/v^2)`, R188 `min(1.24, 9.5/v, 85/v^2)` (rms 10 to 15%); braking flat about 0.76 to
    0.8 m/s^2 to 30 mph. All models ridden are rated 2.5 mph/s motoring, 3.0 full service, 3.2 emergency
    (Wikipedia); only the R188s went over motoring beyond GNSS noise (1.4 to 1.56 m/s^2 at 10 to 18 mph, 7 of 13
    departures). Car models: 6458, 6303, 1170 R142; 5546 R46 (the W; the rider's tag, correcting the app's: item 36); 8582 R160; 5078 R68A; 2718 R68;
    2387 R62A; the 7 R188. `subway_accel_limits` is the R188 fit with 15% margin, braking 3.0 mph/s.
    Open: the CBTC stretches not ridden yet (southern L, lower Culver F) to tell CBTC from the R188;
    check the R188 starts over the rated rate against the gyro-tracked accelerometer.
29. Done, not on the phone (`9ad588a`): the NYCT feeds switched to GTFS-RT v2 (rider, 2026-11-08);
    v1 lacked the 2's reroute past a fire (Tue Oct 27, 10:04 pm, at 10:48 pm: express from 180 St to Gun Hill Rd). v2 is `nyct-v2%2Fgtfs...`, the same line groups,
    from the MTA's developer group post (for daylight saving time ending 2026-11-01; v1 retires in 2027).
    Checked on 2026-11-08: all eight answer with no key, header version 2.0. Against v1 on the wire:
    past stops (1629 stop updates against 1111 on the B/D/F/M), stop sequences, `direction_id`,
    a vehicle descriptor (its ID only the trip ID plus `_V`: no car or train), `is_assigned` on every trip,
    B Division delays as alerts, some entity IDs ending in `_`,
    and vehicles mostly `IN_TRANSIT_TO` where v1 said `STOPPED_AT`. Fusion reads only trip IDs (as keys between polls),
    routes, stop IDs and times (`trips::trip_stops`), so past stops only add the times behind the train;
    nothing reads the status, track labels or trip ID suffixes, and the raw responses are stored unchanged.
    Gzipped, the numbered lines' feed is 24 KB (v1 13 KB), well within the fetcher's 5 s timeouts.
30. False stops between stations, from notes (item 7 finds stops between stations; these are wrong ones):
    a station stop in the middle of the Joralemon tunnel, though the train likely didn't stop
    and surely moved on (Mon Oct 26, 1:38 pm, at 1:43 pm);
    on Mon Oct 26, 8:08 pm: after Borough Hall (8:16 pm), right before Wall St during the curve (8:20 pm),
    and a stop right before Canal St with false walking after it, on the express 2 (8:27 pm);
    on Tue Oct 27, 10:04 pm: after leaving 138 St (10:17 pm), then a true stop in the tunnel,
    and before West Farms Sq (10:36 pm).
    Check each against the lurch, the closing and the cell; the curve one may be the yaw rate (item 2).
    Partly done (`crates/fusion/src/stops.rs` `CURVE_RAD_S`; scored by the untracked scratch
    `crates/cli/examples/falsestops.rs`, `skip_still`, over the 37 rides with legs). The six, classified:
    - A curve's pull read as braking: before Wall St (8:19:49 pm), turning 0.13 rad/s at about 7 m/s,
      the lurch's 0.92 m/s^2. Fixed: a lurch turning 0.08 to 0.25 rad/s is a stop only with a closing heard
      (live: no likely hold). 109 to 99 stops with no tap and no closing, 356 to 354 backed
      (the 2 lost had only an "at station" tap: Tue Oct 20, 6:41 pm, at 7:26 pm; Tue Oct 20, 9:44 pm, at 9:55 pm);
      tapped stops found 239 of 303 and 82 missed, both unchanged; live ratio 31 legs under 0.5, 93 within
      0.7 to 1.3 (91), median 0.97 (0.95), 49 over 1.3 (48), over 205 legs (204).
    - Slowing mid-leg, a lurch with no closing, no turn: before Canal (8:26:49 pm), after 138 St (10:16:58 pm),
      after Borough Hall (8:16:18 pm, 34 s after a hold at 8:15:44 pm). Open: nothing found that tells them from
      real stops with no closing heard (tapped ones at 1:52:30 pm, 11:41:09 pm look the same: departures missed,
      the live speed on, the braking as short), nor from item 7's holds (Lorimer's has no start before the station).
    - A closing heard at speed: Joralemon (1:41:51 pm, 39 mph, cell 29950588 in the tube), before West Farms Sq
      (10:35:43 pm, 17 mph). Open: the door detector's false cycles.
    Item 31 has another cause: no lurch found at all at 10:29 and 10:32 pm, and the subway flag drops at each
    station there (10:26 to 10:45 pm), so no likely hold.
31. Stops on the 5 above ground not counted, or at least no dwell time shown and not 0 mph
    (Tue Oct 27, 10:04 pm, at 10:29 and 10:32 pm).
32. Transfer held wrongly: "false resumption of transfer" once the train moved south from Woodlawn
    (Tue Oct 27, 10:04 pm, at 11:15 pm); the D from 161 St to 155 St marked transfer the whole way,
    then 55 mph as soon as it moved (11:36 pm; the jump is item 18's).
33. Walking speed wrong: "walking 55 mph" (Mon Oct 26, 8:08 pm, at 8:41 pm);
    6.7 mph walking at a normal pace with GNSS Doppler (Tue Oct 27, 5:23 pm, at 5:33 pm).
    Live done, not on the phone (2026-11-08, replayed with `skip_still`): both were walking off a train underground.
    A likely stop held the shown speed at 0 while the IMU alone ran on (38 mph by 8:41:10 pm);
    one step window since the stop released the hold and showed the runaway, 39 to 55 mph with the mode "walking"
    (8:41:17 pm), and 43 to 47 mph from 5:32:37 pm; walking is measured only after 6 windows (24 s),
    and between trains the cap was 3 m/s, the 6.7 mph shown at 5:33:10 pm, before GNSS came back at 5:33:38 pm
    (with GNSS the walk read 2 to 4 mph). Now, without recent GNSS, after a likely stop with steps since it
    or between trains, the live speed shows at most the rider's fastest pace on foot:
    `WALK_MAX_MPS` 2.3 m/s (5 mph) walking, `RUN_MAX_MPS` 6.7 m/s (15 mph) running,
    the one pair of on-foot limits, the offline gap holds included (were 2.6 and 4.5 m/s): 5.1 mph in both cases.
    `liveratio` at these caps (205 legs): unchanged, 31 under 0.5, 93 within 0.7 to 1.3, median 0.97, 49 over 1.3.
    At the first caps (1.4 m/s walking, 3 m/s running):
    `liveratio` (`no_live_smoothing`, 204 legs): 9 legs shorter, none longer; under 0.5 30 to 31
    (a 236 s leg averaging 5.4 mph, 0.67 to 0.34, peak 13.6 to 7.4 mph), within 0.7 to 1.3 91, median 0.95, over 1.3 48.
    Open: offline, the smoothed speed walking off at 8:41 pm still runs 14 to 30 mph (8:40:54 to 8:42:20 pm);
    on Tue Oct 27 it reads 4 to 6 mph. Offline has no between-trains hold; adding one (or measuring walking sooner)
    is for the rider to approve, as a live and offline difference.
34. The 2 from GTFS-RT keeps being lost (Mon Oct 26, 8:08 pm, at 8:19 pm). Check against v2 (item 29).
35. Station cells missing (the registered cell, item 7 of 2026-10-21):
    none at 138 St and 149 St-Grand Concourse on the 5 (Tue Oct 27, 10:04 pm, at 10:16 and 10:21 pm);
    the southbound 4 missed 110 St, 103 St, 86 St and 59 St (Mon Nov 2, 6:04 pm, at 6:22 and 6:24 pm);
    on the next 4 (Mon Nov 2, 6:31 pm) none from 125 St south, in other apps too, after restarting the phone
    and moving to the next car, until Brooklyn Bridge-City Hall (6:38 pm).
    So the cell can be gone for a whole run; the station must come from the rest then.
36. Wrong car number shown (Tue Nov 3, 6:01 pm, at 6:14 pm): the app's car number was wrong;
    the rider's tags are right: 5546 on the W (6:04 pm) and 5048 on the Q (6:19 pm).
    Why (checked 2026-11-08): the app shows the car beacon's ID as "car" (`Status::car`, `MainActivity.kt`),
    not the car's number: at 6:14 pm it was `9a72` (39538), and it went between that and `9776` over the ride.
    Left: show the car's number, from the rider's tags tying beacons to cars, or not call the ID a car.
37. Lines from service patterns, from notes (Sun Nov 8, 2:40 pm, at 2:52 pm): it was the A,
    "since it's running local this weekend, not B/C; also B never runs weekends".
    Rule out lines not running then (the B on weekends and late nights) and use the feeds' or the schedule's local/express runs.
    Done (2026-11-09, `77ac818`): a station's lines are those the feeds list at its platform then,
    plus its listed lines that run by the schedule (`data/service_hours.csv`, from the static GTFS) and the feeds don't list elsewhere instead;
    a route the feeds list at the complex is no longer dropped at a station the MTA doesn't list it for.
    Replayed (`skip_still`): route A from 110 St on, at every station (before: only between stations, "B/C" at them);
    lines "A/C/D" there, as the feeds listed a D too.
    Over all 62 sessions, statuses with a route 619 to 649; line changes checked are part-time lines ruled out
    (no 3 in Brooklyn or B/C late at night, no W/Z on weekends) or the late-night 2 local on the 1.
    Left: without feeds, the schedule only rules lines out; it doesn't add a line running local off its pattern (the A here).
38. Line names in order (Sun Nov 8, 2:40 pm, at 2:54 pm): "B/C", not "C/B"; sort as the MTA does (letters, numbers in order).
    Done (2026-11-09, `87e1b3c`): numbers ascending, then letters alphabetically, then the S and the SIR,
    last as in the MTA's `route_sort_order` (which otherwise groups letters by trunk: A, C, E, B, D).
39. False stops up again (Sun Nov 8, 2:40 pm, at 3:27 pm): "seems to be detecting way more false stops now", on 0.1.18.
    Score this ride's stops against its taps with `falsestops`, against 0.1.17's rules, before blaming items 22 or 27.
    Checked (2026-11-09): no regression found. Items 22's pass rules (`cb3d2db`, `03f018d`) and item 30's curve rule
    came after 0.1.18, so weren't on the phone. Replayed (`skip_still`) at 0.1.16 (`9a6ea7e`), 0.1.17 (`91e5506`)
    and 0.1.18 (`40e18ab`), worktrees in `data/work/wt39-*`:
    - Stop records (`falsestops`): 0.1.17 and 0.1.18 identical, 16 backed and 10 with no tap or closing (2:40 pm),
      8 and 5 (9:29 pm); 0.1.16 13 and 13, 8 and 8. So 0.1.16 had more.
    - Live, as shown (`data/work/livestops.rs`: runs of the speed held at 0, and of the dwell shown):
      2:40 pm held 14 tapped and 9 untapped (0.1.17) to 13 and 11 (0.1.18); dwells 19 and 7 both; 9:29 pm unchanged.
      The 2 more are `4fec3d9`'s weak start ending a false hold: at 3:01:51, 3:10:57 and 3:19:18 pm a lurch at
      34 to 47 mph held 0 for 105 to 138 s through to the next true stop (counted as one tapped stop);
      now it holds 27 to 93 s, the train is shown moving, then the true stop again: more stops seen, less time wrong.
    This ride's own rate is high: 10 of 26 unbacked (item 30: 99 of 453 over 37 rides). Its unbacked lurches are
    item 30's open kind, mid-leg at 34 to 47 mph live (3:01:51, 3:15:41, 3:19:18, 3:26:02, 3:27:17 pm, the note's time),
    no closing; tapped stops at 3:14:47, 3:25:03 and 3:30:17 pm come at 30 to 42 mph live too, so the live speed
    before doesn't tell them apart. Open with item 30: what tells a lurch at speed from a stop.
40. A held train is a delay, not a stop (Sun Nov 8, 9:29 pm): the A held from 9:35 pm to 9:46 pm,
    "delay on A at Nostrand, unsanitary condition on A train in front of us".
    A label for item 26; check the app showed it as one hold, and the dwell isn't counted as a normal stop's.
    The feeds have it (checked 2026-11-09): the rider was on trip `127600_A..N20X002` (stopped at Hoyt-Schermerhorn 9:50:53 pm,
    the rider's next stop 9:50:55 pm). It was shown "in transit to" Hoyt from 9:35:29 pm while still at Nostrand,
    and its Hoyt arrival slid 10 s each poll from 9:40 pm on (9:40 to 9:51 pm) until it moved at 9:46 pm.
    The A ahead (`124500_A..N55R`) sat at Hoyt from 9:34:44 to 9:45:45 pm: the train with the unsanitary condition.
    So a hold is visible live: our trip's next arrival sliding with the clock, and the trip ahead stopped for minutes.
    Service alerts (the "delay" text) aren't polled; add the subway alerts feed.
    Recording done (`8606bcf`, in 0.1.25): the subway alerts feed is recorded every 60 s while any subway feed is polled;
    replay skips it. Left: check it on the next ride's recording, then show alerts for the rider's line live.
41. Live speed inputs late: both of Sun Nov 8's rides reported "inputs arrived over 1500 ms late" from the start,
    356 in the first 21:45 of Sun Nov 8, 9:29 pm, and 344 on Sat Nov 7, 5:59 pm; find what's late (which sensor or the feeds), and why.
    Found (2026-11-09, scratch `lateness` example: each record's own time against its `received`): two streams, a few at a time,
    no sensor or audio stalls (the count climbs by 1 to 3 per report).
    - Fused fixes: their own time is when Android computed them, sometimes cached (up to 30 s old);
      120 to 186 a ride over 1.5 s. A false alarm in effect, they only name stations:
      fixed in `c3f38c5`, live and replay feed them at their recorded time, as Wi-Fi scans and cells.
    - Subway car beacons: really late, BLE's 2 s report delay (`BleRecorder.kt`) plus scanning,
      median 1.7 s and up to 5.5 s behind (409 of 766 on Sun Nov 8, 9:29 pm, over 1.5 s).
      They reach the speed (`on_car_accel`), so live applies them out of order and differs from a replay there.
      Not fixed: needs a choice, either `SETTLE_NS` at 6 s (every settled output 4.5 s later live,
      and `estimate` steps 4 times the pending inputs each 500 ms: measure it first),
      or a shorter report delay (more wakeups; helps only if batching, not the scan's duty cycle, is most of it).
42. Better compression (rider, 2026-11-09), all bit-exact lossless, benchmarked on real rides first:
    - Audio is 95% of a ride (Sun Nov 8, 2:40 pm: 312 of 328 MB). First stop storing copied and silent channels (item 25),
      then compare FLAC at its highest settings, WavPack (`-hhx6`), MPEG-4 ALS and OptimFROG on the 2-microphone audio;
      stereo decorrelation should help most, since the microphones hear the same train.
    - Sensor streams are still protobuf plus `zstd` (accel 1.58M samples in 8.7 MB, 5.5 bytes a sample):
      the columnar encoding planned above (XOR or byte-shuffle values, delta-of-delta timestamps) was never built.
      Try it, and `zstd --long` with a trained dictionary, and `xz`, on accel, gyro, mag and `radio` and `transit`.
    - The feeds (`transit`) repeat almost everything poll to poll: store each poll as a diff to the previous one.
    - Measure the phone's CPU and battery for each, since recompression runs there.
    Non-audio measured (2026-11-09, scratch `crates/session/examples/compbench.rs`, log `data/work/compbench.log`)
    on Sun Nov 8, 2:40 pm, Sun Nov 8, 9:29 pm and Sat Nov 7, 5:59 pm;
    every candidate decoded back to the stored chunks byte for byte. Speeds are on the PC, single thread.
    - The planned XOR and byte-shuffle values lose: worse than today (0.75x to 0.84x on accel).
      Values sit on a calibration-shifted grid (about 10k distinct values per axis a ride, but almost none exactly `k` times the step),
      so splitting bytes breaks what ties the low bytes to the high ones, and XOR of nearby floats isn't small.
      Plain per-axis integer differences of the bits help (1.9x), but the winner is a value table:
      per axis, the ride's sorted distinct values once (a few KB), then each sample as the difference of its rank in that table,
      zigzag varint; timestamps delta of delta, zigzag varint (5% under plain deltas). Lossless by construction, no resolution guess.
    - Rank table plus `zstd -19`: accel 9.6 to 4.5 MB/h (2.1x), gyro 8.9 to 3.6 (2.3x to 2.6x), mag 2.6 to 0.78 (3.3x),
      pressure 1.4x, light 1.5x; encodes at 25 to 45 MB/s, 6 times faster than today's `zstd -19` on protobuf, decodes at ~850 MB/s.
      `xz -6e` on top saves 4% to 7% more but decodes 4 times slower; `zstd -22` and `--long` change nothing (under 1%).
    - `transit`: one `zstd` frame over the whole stream already uses earlier polls (37 to 52 MB to ~0.8 MB, about 1 MB/h).
      Each poll as a `zstd` patch to the previous one from its URL is 10% to 13% worse,
      and polls compressed alone with a dictionary trained on another ride are 5 to 6 times worse. Keep it; `xz -6e` saves 13% to 16% (0.13 MB/h).
    - `radio`, `gnss`, `location`: `zstd -22 --long` under 1%, `xz -6e` 6% to 9%; not worth a format change without a columnar layout per record type.
    - Recommendation: in post-ride recompression, write accel, gyro, mag, pressure and light as the rank table encoding plus `zstd -19`
      (one table per stream per ride, so only post-ride; the live chunks stay as they are), verified as now before deleting the chunks.
      Saves about 12.5 MB/h (sensors 21.5 to 9 MB/h; all non-audio 25.5 to 13 MB/h), small next to audio (~330 MB/h).
      Not yet tried: predicting the rank from the two samples before; timing on the phone itself.
    Sensor columns done (2026-11-09, `a31cb6a`): post-ride recompression writes every sensor stream as the rank table plus `zstd -19`
    (`crates/session/src/sensor_columns.rs`, a skippable frame tagged `ride.sensor-columns.v1` then the columns),
    replaced only once it decodes to the original chunks byte for byte; readers decode it transparently, live chunks and old files read as before.
    A stream protobuf wouldn't re-encode identically, or with mixed sample shapes, stays plain `zstd` (none seen yet; `proximity` stays plain as it's not smaller).
    On copies of Sun Nov 8, 2:40 pm and 9:29 pm (`data/work/columns-check/`): accel 8.74 to 4.11 MB and 6.03 to 2.88, gyro 8.17 to 3.50 and 5.63 to 2.13,
    mag 2.40 to 0.73 and 1.64 to 0.49, pressure 0.18 to 0.13, light 0.08 to 0.05, as benchmarked;
    `ride summary`, `ride csv` of each stream, and whole and live replays are identical to the originals'.
    On the phone in 0.1.25 (installed 2026-11-09, 9:24 am, from `f63ff3f`). Left: phone timing; already-recompressed rides get it only by running `ride recompress` again.

## Future: flights
- Same logger, in airplane mode: GNSS is receive-only and usually keeps working; needs a window seat, expect weak signal and gaps.
  Airliner speed/altitude are within consumer GNSS limits (~1900 km/h, 18 km).
- Barometer measures cabin pressure (pressurized to ~1800-2400 m equivalent), not aircraft altitude:
  gives the climb/descent and pressurization profile, with GNSS altitude for the real one.
- Fusion needs an aircraft motion model: free 3D, sideslip/crab allowed, no track graph or non-holonomic constraint;
  ZUPTs only on the ground, runway/taxiway geometry from OSM as a weak prior at takeoff and landing.
- Turns are coordinated, so the curve math gives bank angle with near-zero "cant deficiency"; flag uncoordinated turns and turbulence.
- Ground truth: ADS-B tracks (OpenSky Network, ADS-B Exchange) give the aircraft's own GPS position, altitude, and speed about every second;
  compare our fused trajectory against them (check each source's access terms for historical data).
- Battery: flights are long, so power bank or lower GNSS/IMU rates if needed.

## Verification
- Logger: per-stream rate/gap report; 1 h screen-off ride without dropouts.
- GNSS: Doppler speed vs position-derived speed agree on open sky; stationary noise floor.
- Fusion: synthetic GNSS outages on good data, error vs held-out truth;
  station-to-station distances vs OSM; correct station identity on rides starting underground.
- Stops, dwells and doors: the rules were tuned and scored on the same 9 rides and about 28 tapped stops,
  some from a single example (the stop 6 s before walking off, from 96 St).
  Score the next few tapped rides as a held-out check, before any retuning on them,
  and before trusting dwell and door times for analyses.
