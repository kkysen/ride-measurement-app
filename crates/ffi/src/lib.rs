//! Rust exposed to the app through `uniffi`. Kotlin bindings are generated from this library at build time.
//!
//! Only thin wrappers live here: the logic is in `ride-fusion` and `ride-session`, shared with offline tools.

use std::sync::{Arc, Mutex};

use ride_fusion::live_speed;
use ride_fusion::speed_colors::{self, SpeedScale as Scale};

uniffi::setup_scaffolding!();

/// The version of this crate, to check the app is running the Rust it was built with.
#[uniffi::export]
pub fn core_version() -> String {
    env!("CARGO_PKG_VERSION").into()
}

/// A fix more than this long after the previous one starts a new path segment (a tunnel).
#[uniffi::export]
pub fn track_max_gap_ns() -> i64 {
    ride_session::track::MAX_GAP_NS
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum Sensor {
    Imu,
    GnssDoppler,
    GnssPosition,
    Stop,
    Doors,
    Walking,
    SpeedLimit,
    Car,
}

impl From<live_speed::Sensor> for Sensor {
    fn from(s: live_speed::Sensor) -> Self {
        match s {
            live_speed::Sensor::Imu => Sensor::Imu,
            live_speed::Sensor::GnssDoppler => Sensor::GnssDoppler,
            live_speed::Sensor::GnssPosition => Sensor::GnssPosition,
            live_speed::Sensor::Stop => Sensor::Stop,
            live_speed::Sensor::Doors => Sensor::Doors,
            live_speed::Sensor::Walking => Sensor::Walking,
            live_speed::Sensor::SpeedLimit => Sensor::SpeedLimit,
            live_speed::Sensor::Car => Sensor::Car,
        }
    }
}

#[derive(Clone, Debug, uniffi::Record)]
pub struct SensorCount {
    pub sensor: Sensor,
    pub count: u32,
}

#[derive(Clone, Debug, uniffi::Record)]
pub struct RideStatus {
    pub mode: String,
    /// The station the registered cell names, if it's one the rider has tagged; with the lines tagged there.
    pub station: Option<String>,
    pub lines: Vec<String>,
    /// Stopped there: a door cycle was just heard.
    pub at_station: bool,
    /// The train's route, from its stops and the GTFS-realtime feeds.
    pub route: Option<String>,
    /// The car type, from the latest door chime and the cars' beacons, narrowed by the route or the lines.
    pub cars: Option<String>,
    /// How long (s) the rider has stayed within 100 m, out of the subway.
    pub staying_s: f64,
    /// The rider's car, by its beacon's ID in hex (e.g. `a199`), if a steady one is heard.
    pub car: Option<String>,
    /// At a stop, until the train departs: since it stopped (s), since its doors opened (s) until they close,
    /// then how long they were open (s).
    pub stopped_s: Option<f64>,
    pub doors_open_s: Option<f64>,
    pub doors_were_open_s: Option<f64>,
}

#[derive(Clone, Debug, uniffi::Record)]
pub struct Estimate {
    pub speed_mps: f64,
    pub sigma_mps: f64,
    /// Sensors that contributed recently, in a fixed order.
    pub sensors: Vec<Sensor>,
    pub stopped: bool,
    pub since_gnss_s: Option<f64>,
    /// Measurements rejected as outliers so far, per sensor.
    pub rejected: Vec<SensorCount>,
    /// Acceleration in the direction of travel, negative when braking.
    pub accel_mps2: f64,
    /// The whole ride's average speed so far, stops included.
    pub average_mps: Option<f64>,
    /// The fastest the vehicle can go now (55 mph on the subway), if limited.
    pub max_speed_mps: Option<f64>,
}

/// A smoothed speed at a fix or a sample time (`ride_fusion::fusion::SmoothedSpeed`).
#[derive(Clone, Debug, uniffi::Record)]
pub struct SmoothedSpeed {
    pub elapsed_realtime_ns: i64,
    pub speed_mps: f64,
    pub sigma_mps: f64,
    pub accel_mps2: f64,
}

/// A train stop so far, for the live graph's labels (see `ride_fusion::fusion::StopRecord`).
#[derive(Clone, Debug, uniffi::Record)]
pub struct LiveStop {
    pub stopped_ns: i64,
    pub departed_ns: Option<i64>,
    pub station: Option<String>,
    pub in_subway: bool,
}

/// Raw GNSS progress, for the screen.
#[derive(Clone, Debug, uniffi::Record)]
pub struct GnssStats {
    pub subframes_ok: u64,
    pub subframes_bad: u64,
    pub ephemerides: u64,
    pub epochs: u64,
    pub epochs_solved: u64,
}

/// [`ride_fusion::fusion::Fusion`], callable from any thread: the live speed, from the same code offline replays.
#[derive(uniffi::Object)]
pub struct Fusion(Mutex<ride_fusion::fusion::Fusion>);

#[uniffi::export]
impl Fusion {
    /// Smooths the last 10 minutes of input every second, as offline replay does.
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self(Mutex::new(ride_fusion::fusion::Fusion::new())))
    }

    /// A batch of Android sensor samples as delivered (`Sensor.getType()`, each `SensorEvent.timestamp`, and their
    /// `SensorEvent.values` back to back, `values_per_sample` each). Batched, since each call into Rust costs tens
    /// of microseconds.
    pub fn on_sensor_samples(
        &self,
        sensor_type: i32,
        timestamps_ns: Vec<i64>,
        values_per_sample: u32,
        values: Vec<f32>,
    ) {
        let n = values_per_sample as usize;
        let mut fusion = self.lock();
        for (t, v) in timestamps_ns.iter().zip(values.chunks_exact(n.max(1))) {
            fusion.on_sensor(sensor_type, *t, v);
        }
    }

    /// A GPS-provider fix.
    #[allow(
        clippy::too_many_arguments,
        reason = "mirrors `android.location.Location`'s fields"
    )]
    pub fn on_fix(
        &self,
        t_ns: i64,
        lat_deg: f64,
        lon_deg: f64,
        altitude_m: Option<f64>,
        horizontal_accuracy_m: Option<f64>,
        speed_mps: Option<f64>,
        speed_accuracy_mps: Option<f64>,
    ) {
        self.lock().on_fix(ride_fusion::fusion::Fix {
            elapsed_realtime_ns: t_ns,
            lat: lat_deg,
            lon: lon_deg,
            altitude: altitude_m,
            accuracy: horizontal_accuracy_m,
            speed: speed_mps,
            speed_sigma: speed_accuracy_mps,
        });
    }

    /// An encoded `GnssMeasurementsEvent` (as recorded), received at `received_ns`.
    pub fn on_gnss_measurements(
        &self,
        received_ns: i64,
        encoded: Vec<u8>,
    ) -> Result<(), RideError> {
        use prost::Message;
        let event = ride_session::proto::GnssMeasurementsEvent::decode(encoded.as_slice())
            .map_err(anyhow::Error::from)?;
        self.lock().on_gnss_measurements(received_ns, &event);
        Ok(())
    }

    /// A Wi-Fi scan's SSIDs, at the time it was recorded, for telling when the phone is in the subway.
    pub fn on_wifi_scan(&self, t_ns: i64, ssids: Vec<String>) {
        self.lock().on_wifi_scan(t_ns, ssids);
    }

    /// One microphone read as recorded (mono 16-bit little-endian PCM from stream frame `first_frame`), at `t_ns`,
    /// when its last frame was recorded, with the recorder's timestamps (frame positions and elapsed realtime ns)
    /// from it: door cycles in the sound hold the speed at zero.
    pub fn on_audio(
        &self,
        t_ns: i64,
        sample_rate_hz: u32,
        first_frame: i64,
        pcm_s16le: Vec<u8>,
        timestamp_frames: Vec<i64>,
        timestamp_ns: Vec<i64>,
    ) {
        let samples = pcm_s16le
            .as_chunks::<2>()
            .0
            .iter()
            .map(|&b| i16::from_le_bytes(b))
            .collect();
        self.lock().on_audio(
            t_ns,
            sample_rate_hz,
            ride_fusion::fusion::AudioSamples {
                first_frame,
                samples,
                timestamps: timestamp_frames.into_iter().zip(timestamp_ns).collect(),
            },
        );
    }

    /// A downloaded orbit file (from [`orbit_fetch_plan`]); returns how many ephemerides it held.
    pub fn add_orbit_file(&self, path: String) -> Result<u32, RideError> {
        let ephemerides = ride_gnss::orbits::read_orbit_file(std::path::Path::new(&path))
            .map_err(anyhow::Error::from)?;
        Ok(self.add_ephemerides(ephemerides))
    }

    /// Every orbit file already in a session's `orbits/` (e.g. after a restart); returns how many ephemerides.
    pub fn add_session_orbits(&self, session_dir: String) -> Result<u32, RideError> {
        let ephemerides =
            ride_gnss::orbits::read_session_orbits(std::path::Path::new(&session_dir))
                .map_err(anyhow::Error::from)?;
        Ok(self.add_ephemerides(ephemerides))
    }

    /// An encoded `GnssNavigationMessage` (as recorded).
    pub fn on_navigation_message(&self, encoded: Vec<u8>) -> Result<(), RideError> {
        use prost::Message;
        let msg = ride_session::proto::GnssNavigationMessage::decode(encoded.as_slice())
            .map_err(anyhow::Error::from)?;
        self.lock().on_navigation_message(&msg);
        Ok(())
    }

    /// Android's own (fused) location: underground, it names the station from Google's database.
    pub fn on_network_fix(
        &self,
        t_ns: i64,
        lat_deg: f64,
        lon_deg: f64,
        horizontal_accuracy_m: Option<f64>,
    ) {
        self.lock()
            .on_network_fix(t_ns, lat_deg, lon_deg, horizontal_accuracy_m);
    }

    /// A Bluetooth advertisement (`ScanResult`'s bytes and RSSI) at its own time: a subway car's beacon names the car.
    pub fn on_ble(&self, t_ns: i64, data: Vec<u8>, rssi: i32) {
        self.lock().on_ble(t_ns, &data, rssi);
    }

    /// A GTFS-realtime feed's response body (HTTP 200 only), when it arrived, for a request at `request_unix_ms`:
    /// gives the train's route.
    pub fn on_transit(&self, t_ns: i64, request_unix_ms: i64, body: Vec<u8>) {
        self.lock().on_transit(t_ns, request_unix_ms, &body);
    }

    /// The registered LTE cell's ID, if any, at the time it was recorded: names the station.
    pub fn on_cell(&self, t_ns: i64, cell_id: Option<i64>) {
        self.lock().on_cell(t_ns, cell_id);
    }

    /// The train's stops so far, oldest first.
    pub fn stops(&self) -> Vec<LiveStop> {
        self.lock()
            .stop_records()
            .into_iter()
            .map(|r| LiveStop {
                stopped_ns: r.stopped_ns,
                departed_ns: r.departed_ns,
                station: r.station.map(|s| s.name.clone()),
                in_subway: r.in_subway,
            })
            .collect()
    }

    /// What the rider is on and where, for the main screen, and how long they've stayed put.
    pub fn status(&self) -> RideStatus {
        use ride_fusion::fusion::Mode;
        let s = self.lock().status();
        let lines = s
            .station
            .as_ref()
            .map(|st| st.lines.clone())
            .unwrap_or_default();
        RideStatus {
            mode: match s.mode {
                Mode::Unknown => "unknown",
                Mode::Still => "still",
                Mode::Walking => "walking",
                Mode::Running => "running",
                Mode::Subway { underground: true } => "subway, underground",
                Mode::Subway { underground: false } => "subway, above ground",
                Mode::Vehicle => "bus, car, or other train",
            }
            .to_string(),
            station: s.station.as_ref().map(|st| st.name.clone()),
            lines,
            at_station: s.at_station,
            route: s.route.clone(),
            // The chime's family, or the model when what's heard and the lines leave one.
            cars: s.car_type.and_then(|c| match (c.ntt, c.model) {
                (Some(true), Some(m)) => Some(format!("NTT ({m})")),
                (_, Some(m)) => Some(m.to_string()),
                (Some(true), None) => Some("NTT".to_string()),
                (Some(false), None) => Some("R62".to_string()),
                (None, None) => None,
            }),
            staying_s: s.staying_s,
            car: s.car.map(|c| format!("{c:04x}")),
            stopped_s: s.dwell.as_ref().map(|d| d.stopped_s),
            doors_open_s: s.dwell.as_ref().and_then(|d| d.doors_open_s),
            doors_were_open_s: s.dwell.as_ref().and_then(|d| d.doors_were_open_s),
        }
    }

    pub fn estimate(&self, now_ns: i64) -> Estimate {
        let fusion = self.lock();
        let e = fusion.estimate(now_ns);
        Estimate {
            average_mps: fusion.average_speed_mps(),
            speed_mps: e.speed_mps,
            sigma_mps: e.sigma_mps,
            sensors: e.sensors.into_iter().map(Sensor::from).collect(),
            stopped: e.stopped,
            since_gnss_s: e.since_gnss_s,
            max_speed_mps: e.max_speed_mps,
            rejected: e
                .rejected
                .into_iter()
                .map(|(s, count)| SensorCount {
                    sensor: s.into(),
                    count,
                })
                .collect(),
            accel_mps2: e.accel_mps2,
        }
    }

    /// The window's latest smoothed speeds (forward and backward passes combined), re-run every
    /// [`smooth_every_ns`]; older values are frozen and don't change any more.
    pub fn smoothed(&self) -> Vec<SmoothedSpeed> {
        self.lock()
            .recent_smoothed()
            .into_iter()
            .map(|s| SmoothedSpeed {
                elapsed_realtime_ns: s.elapsed_realtime_ns,
                speed_mps: s.speed_mps,
                sigma_mps: s.sigma_mps,
                accel_mps2: s.accel_mps2,
            })
            .collect()
    }

    /// Inputs that arrived after newer ones had settled ([`settle_ns`]), so went into the forward pass out of order.
    pub fn late_inputs(&self) -> u64 {
        self.lock().late_inputs
    }

    pub fn gnss_stats(&self) -> GnssStats {
        let s = self.lock().navigator().stats.clone();
        GnssStats {
            subframes_ok: s.subframes_ok,
            subframes_bad: s.subframes_bad,
            ephemerides: s.ephemerides,
            epochs: s.epochs,
            epochs_solved: s.epochs_solved,
        }
    }
}

impl Fusion {
    fn add_ephemerides(&self, ephemerides: Vec<ride_gnss::ephemeris::Ephemeris>) -> u32 {
        let n = ephemerides.len() as u32;
        let mut fusion = self.lock();
        for e in ephemerides {
            fusion.navigator_mut().add_ephemeris(e);
        }
        n
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, ride_fusion::fusion::Fusion> {
        // A panic while holding the lock leaves the state usable; keep going rather than poison the app.
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, uniffi::Enum)]
pub enum SpeedScale {
    Rail,
    HighSpeedRail,
    Flight,
}

impl From<SpeedScale> for Scale {
    fn from(s: SpeedScale) -> Self {
        match s {
            SpeedScale::Rail => Scale::Rail,
            SpeedScale::HighSpeedRail => Scale::HighSpeedRail,
            SpeedScale::Flight => Scale::Flight,
        }
    }
}

impl From<Scale> for SpeedScale {
    fn from(s: Scale) -> Self {
        match s {
            Scale::Rail => SpeedScale::Rail,
            Scale::HighSpeedRail => SpeedScale::HighSpeedRail,
            Scale::Flight => SpeedScale::Flight,
        }
    }
}

/// All scales, in order from slowest.
#[uniffi::export]
pub fn speed_scales() -> Vec<SpeedScale> {
    Scale::ALL.into_iter().map(SpeedScale::from).collect()
}

#[uniffi::export]
pub fn speed_scale_name(scale: SpeedScale) -> String {
    Scale::from(scale).name().into()
}

#[uniffi::export]
pub fn speed_scale_bands_kmh(scale: SpeedScale) -> Vec<u32> {
    Scale::from(scale).bands_kmh().into()
}

#[uniffi::export]
pub fn speed_scale_for_top_speed(top_speed_mps: Option<f64>) -> SpeedScale {
    Scale::for_top_speed(top_speed_mps).into()
}

/// ARGB color for the path between two consecutive fixes with these speeds (`None` for unknown).
#[uniffi::export]
pub fn speed_edge_color_argb(scale: SpeedScale, a_mps: Option<f64>, b_mps: Option<f64>) -> u32 {
    Scale::from(scale).color_argb(speed_colors::edge_speed(a_mps, b_mps))
}

/// Boundaries between the acceleration color bands, m/s^2.
#[uniffi::export]
pub fn accel_bounds_mps2() -> Vec<f64> {
    speed_colors::ACCEL_BOUNDS_MPS2.into()
}

/// ARGB color for the path between two consecutive fixes with these accelerations (`None` for unknown).
#[uniffi::export]
pub fn accel_edge_color_argb(a_mps2: Option<f64>, b_mps2: Option<f64>) -> u32 {
    speed_colors::accel_color_argb(speed_colors::edge_accel(a_mps2, b_mps2))
}

/// One orbit file to try: download `url` to `path` (via `path` + `.part`, then a rename).
#[derive(Clone, Debug, uniffi::Record)]
pub struct OrbitFile {
    pub url: String,
    pub path: String,
}

/// The orbit files the live fusion needs now (`ride_gnss::orbits::fetch_plan` over the last few hours),
/// per UTC day the files to try in order until one downloads; `just orbits` follows the same plan offline.
#[uniffi::export]
pub fn orbit_fetch_plan(session_dir: String, now_unix_ms: i64) -> Vec<Vec<OrbitFile>> {
    ride_gnss::orbits::fetch_plan(
        std::path::Path::new(&session_dir),
        now_unix_ms - ride_gnss::orbits::LIVE_LOOKBACK_MS,
        now_unix_ms,
    )
    .into_iter()
    .map(|day| {
        day.into_iter()
            .map(|f| OrbitFile {
                url: f.url,
                path: f.path.display().to_string(),
            })
            .collect()
    })
    .collect()
}

/// How often to follow [`orbit_fetch_plan`] again while recording (ms): BKG updates its file every 15 minutes.
#[uniffi::export]
pub fn settle_ns() -> i64 {
    ride_fusion::fusion::SETTLE_NS
}

/// How often smoothing re-runs over the window (ns of settled input).
#[uniffi::export]
pub fn smooth_every_ns() -> i64 {
    ride_fusion::fusion::SMOOTH_EVERY_NS
}

/// The MTA GTFS-realtime feed URLs to record now (`ride_fusion::gtfs_rt::feeds_near`): the lines near the last
/// known position, or every feed without one.
#[uniffi::export]
pub fn gtfs_rt_feeds(lat: Option<f64>, lon: Option<f64>) -> Vec<String> {
    ride_fusion::gtfs_rt::feeds_near(lat.zip(lon))
}

/// How often to poll [`gtfs_rt_feeds`] while recording (ms).
#[uniffi::export]
pub fn gtfs_rt_poll_ms() -> i64 {
    ride_fusion::gtfs_rt::POLL_MS
}

#[uniffi::export]
pub fn orbit_refresh_ms() -> i64 {
    ride_gnss::orbits::LIVE_REFRESH_MS
}

/// A failure of a whole operation, with its full cause chain.
#[derive(Debug, uniffi::Error)]
pub enum RideError {
    Failed { reason: String },
}

impl std::fmt::Display for RideError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RideError::Failed { reason } => f.write_str(reason),
        }
    }
}

impl std::error::Error for RideError {}

impl From<anyhow::Error> for RideError {
    fn from(e: anyhow::Error) -> Self {
        RideError::Failed {
            reason: format!("{e:#}"),
        }
    }
}

/// What happened to one stream file.
#[derive(Clone, Debug, uniffi::Record)]
pub struct StreamRecompression {
    pub file_name: String,
    pub replaced: bool,
    /// Human-readable: the sizes if replaced, else why the original was kept.
    pub summary: String,
}

/// The file a session gets once post-ride recompression handled every stream; uploads wait for it.
/// A tap in a past ride, as the rider means it now (see `ride_session::marks`).
#[derive(uniffi::Record)]
pub struct SessionMark {
    /// What deleting or changing it names it by.
    pub received_ns: i64,
    pub unix_ms: i64,
    /// `ManualEvent.Kind`'s number.
    pub kind: i32,
    pub text: String,
    pub deleted: bool,
    pub edited: bool,
}

/// A finished session's taps in time order, deleted ones flagged; never the recording in progress,
/// whose `events` stream the app is still writing.
#[uniffi::export]
pub fn session_marks(session_dir: String) -> Result<Vec<SessionMark>, RideError> {
    let session = ride_session::read::Session::open(&session_dir)?;
    Ok(ride_session::marks::marks(&session)?
        .into_iter()
        .map(|m| SessionMark {
            received_ns: m.received_ns,
            unix_ms: m.unix_ms,
            kind: m.kind.into(),
            text: m.text,
            deleted: m.deleted,
            edited: m.edited,
        })
        .collect())
}

/// Deletes a finished session's tap, or with `kind`, changes it to that kind and `text`;
/// either way by appending records, so nothing is lost. `now_ns` and `now_unix_ms`: the phone's clocks now.
#[uniffi::export]
pub fn edit_session_mark(
    session_dir: String,
    received_ns: i64,
    kind: Option<i32>,
    text: String,
    now_ns: i64,
    now_unix_ms: i64,
) -> Result<(), RideError> {
    use ride_session::proto::manual_event::Kind;
    let session = ride_session::read::Session::open(&session_dir)?;
    let marks = ride_session::marks::marks(&session)?;
    let Some(mark) = marks
        .iter()
        .find(|m| m.received_ns == received_ns && !m.deleted)
    else {
        return Err(anyhow::anyhow!("no such tap in {session_dir}").into());
    };
    match kind {
        None => ride_session::marks::delete_mark(&session, mark, now_ns, now_unix_ms)?,
        Some(k) => {
            let k = Kind::try_from(k).map_err(|_| anyhow::anyhow!("unknown tap kind {k}"))?;
            ride_session::marks::change_mark(&session, mark, k, &text, now_ns, now_unix_ms)?
        }
    }
    Ok(())
}

#[uniffi::export]
pub fn recompressed_marker() -> String {
    ride_session::recompress::MARKER.into()
}

/// Post-ride recompression of the session in `dir` (`ride_session::recompress`).
#[uniffi::export]
pub fn recompress_session(dir: String) -> Result<Vec<StreamRecompression>, RideError> {
    use ride_session::recompress::{self, Outcome};
    Ok(
        recompress::recompress_session(std::path::Path::new(&dir), recompress::LEVEL)?
            .into_iter()
            .map(|(file_name, outcome)| StreamRecompression {
                file_name,
                replaced: matches!(outcome, Outcome::Replaced { .. }),
                summary: outcome.to_string(),
            })
            .collect(),
    )
}

/// Writes `session.pb` in `dir` from an encoded `SessionInfo` (`ride_session::write::write_session_info`).
#[uniffi::export]
pub fn write_session_info(dir: String, encoded_info: Vec<u8>) -> Result<(), RideError> {
    use prost::Message;
    let info =
        ride_session::SessionInfo::decode(encoded_info.as_slice()).map_err(anyhow::Error::from)?;
    Ok(ride_session::write::write_session_info(
        std::path::Path::new(&dir),
        &info,
    )?)
}

/// A stream file being recorded (`ride_session::write::StreamWriter`): one checksummed zstd frame per chunk,
/// synced after each.
#[derive(uniffi::Object)]
pub struct StreamWriter(Mutex<Option<ride_session::write::StreamWriter>>);

#[uniffi::export]
impl StreamWriter {
    /// Creates `<dir>/<name>.zst`; fails if it exists, so nothing is ever overwritten.
    #[uniffi::constructor]
    pub fn create(dir: String, name: String) -> Result<Arc<Self>, RideError> {
        let w = ride_session::write::StreamWriter::create(std::path::Path::new(&dir), &name)?;
        Ok(Arc::new(Self(Mutex::new(Some(w)))))
    }

    /// The file's path.
    pub fn path(&self) -> String {
        self.with(|w| Ok(w.path().display().to_string()))
            .unwrap_or_default()
    }

    /// Appends a chunk encoded as a protobuf message (records, audio).
    pub fn append_encoded(&self, chunk: Vec<u8>) -> Result<(), RideError> {
        self.with(|w| w.append_encoded(&chunk))
    }

    /// Builds a sensor chunk from a chunk's samples and appends it: one call per chunk, since each call into Rust
    /// costs tens of microseconds. `value_bits` are each value's IEEE 754 bits (`Float.floatToRawIntBits`),
    /// so they arrive unaltered.
    pub fn append_sensor_samples(
        &self,
        timestamps_ns: Vec<i64>,
        values_per_sample: u32,
        value_bits: Vec<u32>,
        accuracy: Vec<i32>,
        written_elapsed_realtime_ns: i64,
    ) -> Result<(), RideError> {
        if timestamps_ns.is_empty() {
            return Ok(());
        }
        let chunk = ride_session::SensorChunk::from_sample_bits(
            &timestamps_ns,
            values_per_sample,
            value_bits,
            &accuracy,
            written_elapsed_realtime_ns,
        )?;
        self.with(|w| w.append(&chunk))
    }

    /// Writes any chunks left pending by earlier failures, then closes the file; later appends fail.
    /// The file is closed even if the pending chunks still can't be written (that's the error).
    /// (`close()` in Kotlin frees this object.)
    pub fn finish(&self) -> Result<(), RideError> {
        let w = self.0.lock().unwrap_or_else(|e| e.into_inner()).take();
        match w {
            Some(mut w) => Ok(w.write_pending()?),
            None => Ok(()),
        }
    }
}

impl StreamWriter {
    fn with<T>(
        &self,
        f: impl FnOnce(&mut ride_session::write::StreamWriter) -> anyhow::Result<T>,
    ) -> Result<T, RideError> {
        let mut guard = self.0.lock().unwrap_or_else(|e| e.into_inner());
        let w = guard
            .as_mut()
            .ok_or_else(|| anyhow::anyhow!("stream writer is closed"))?;
        Ok(f(w)?)
    }
}

/// The session format version this build writes.
#[uniffi::export]
pub fn format_version() -> u32 {
    ride_session::FORMAT_VERSION
}
