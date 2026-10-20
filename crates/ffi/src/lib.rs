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
}

impl From<live_speed::Sensor> for Sensor {
    fn from(s: live_speed::Sensor) -> Self {
        match s {
            live_speed::Sensor::Imu => Sensor::Imu,
            live_speed::Sensor::GnssDoppler => Sensor::GnssDoppler,
            live_speed::Sensor::GnssPosition => Sensor::GnssPosition,
            live_speed::Sensor::Stop => Sensor::Stop,
        }
    }
}

#[derive(Clone, Debug, uniffi::Record)]
pub struct SensorCount {
    pub sensor: Sensor,
    pub count: u32,
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
}

/// A smoothed speed at a fix or a sample time (`ride_fusion::fusion::SmoothedSpeed`).
#[derive(Clone, Debug, uniffi::Record)]
pub struct SmoothedSpeed {
    pub elapsed_realtime_ns: i64,
    pub speed_mps: f64,
    pub sigma_mps: f64,
    pub accel_mps2: f64,
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
    /// Keeps `window_s` seconds of input for smoothing.
    #[uniffi::constructor]
    pub fn new(window_s: u32) -> Arc<Self> {
        let window = i64::from(window_s) * 1_000_000_000;
        Arc::new(Self(Mutex::new(ride_fusion::fusion::Fusion::new(Some(
            window,
        )))))
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

    /// An encoded `GnssNavigationMessage` (as recorded).
    pub fn on_navigation_message(&self, encoded: Vec<u8>) -> Result<(), RideError> {
        use prost::Message;
        let msg = ride_session::proto::GnssNavigationMessage::decode(encoded.as_slice())
            .map_err(anyhow::Error::from)?;
        self.lock().on_navigation_message(&msg);
        Ok(())
    }

    pub fn estimate(&self, now_ns: i64) -> Estimate {
        let e = self.lock().estimate(now_ns);
        Estimate {
            speed_mps: e.speed_mps,
            sigma_mps: e.sigma_mps,
            sensors: e.sensors.into_iter().map(Sensor::from).collect(),
            stopped: e.stopped,
            since_gnss_s: e.since_gnss_s,
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

    /// Smoothed speeds over the window (forward and backward passes combined).
    pub fn smoothed(&self) -> Vec<SmoothedSpeed> {
        self.lock()
            .smoothed()
            .into_iter()
            .map(|s| SmoothedSpeed {
                elapsed_realtime_ns: s.elapsed_realtime_ns,
                speed_mps: s.speed_mps,
                sigma_mps: s.sigma_mps,
                accel_mps2: s.accel_mps2,
            })
            .collect()
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
