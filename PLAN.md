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
- **Train beacons**: a BLE beacon family (manufacturer `02FD`, ID prefix `e2c56db5-dffb-48d2-b060-d0f5a710`) rides with each train
  on the 7, R, M, F (CBTC lines) and was absent on the 1, 2, 3. Gives exact boarding/alighting and the trainset;
  short sightings are other trains. Decode the varying bytes. Piper's BLE beacons (B Division, not the L) power countdown clocks,
  on trains or stations; not yet identified. Phones and trackers (Apple, Find My Device `0x40` Eddystone) are riders.
- **Crowding**: BLE device counts (300 to 600 on platforms), but the unfiltered scan pauses with the screen off, so normalize for that;
  the user's Aranet4 (in the BLE data) gives CO2 (640 to 1180 ppm on a crowded 2), temperature, humidity, and a second barometer.
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
7. Stations from pass times and distances on express runs; station BLE beacons if any.
   Express passes to check: the 2 past 59 St and 66 St (`214427`), and the F express in Queens on `184134`,
   which passed 65 St very fast and 36 St more slowly (per the rider).

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
