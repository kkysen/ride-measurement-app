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
}

/// [`live_speed::LiveSpeedEstimator`], callable from any thread.
#[derive(uniffi::Object)]
pub struct LiveSpeedEstimator(Mutex<live_speed::LiveSpeedEstimator>);

#[uniffi::export]
impl LiveSpeedEstimator {
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self(Mutex::new(live_speed::LiveSpeedEstimator::new())))
    }

    pub fn on_gyro(&self, wx: f64, wy: f64, wz: f64) {
        self.lock().on_gyro(wx, wy, wz);
    }

    pub fn on_accel(&self, t_ns: i64, ax: f64, ay: f64, az: f64) {
        self.lock().on_accel(t_ns, ax, ay, az);
    }

    pub fn on_gnss_fix(
        &self,
        t_ns: i64,
        lat_deg: f64,
        lon_deg: f64,
        horizontal_accuracy_m: Option<f64>,
        speed_mps: Option<f64>,
        speed_accuracy_mps: Option<f64>,
    ) {
        self.lock().on_gnss_fix(
            t_ns,
            lat_deg,
            lon_deg,
            horizontal_accuracy_m,
            speed_mps,
            speed_accuracy_mps,
        );
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
        }
    }
}

impl LiveSpeedEstimator {
    fn lock(&self) -> std::sync::MutexGuard<'_, live_speed::LiveSpeedEstimator> {
        // A panic while holding the lock leaves the estimator usable; keep going rather than poison the app.
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

    /// Builds the sensor chunk accumulated in `builder` and appends it, if it has any samples.
    pub fn append_sensor_chunk(
        &self,
        builder: Arc<SensorChunkBuilder>,
        written_elapsed_realtime_ns: i64,
    ) -> Result<(), RideError> {
        let chunk = {
            let mut b = builder.lock();
            if b.is_empty() {
                return Ok(());
            }
            b.build(written_elapsed_realtime_ns)
        };
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

/// Sensor samples for the next chunk (`ride_session::write::ChunkBuilder`).
#[derive(uniffi::Object)]
pub struct SensorChunkBuilder(Mutex<ride_session::write::ChunkBuilder>);

#[uniffi::export]
impl SensorChunkBuilder {
    #[uniffi::constructor]
    pub fn new() -> Arc<Self> {
        Arc::new(Self(Mutex::new(Default::default())))
    }

    pub fn sample_count(&self) -> u32 {
        self.lock().len() as u32
    }

    /// Whether a sample with `value_count` values can go in this chunk.
    pub fn fits(&self, value_count: u32) -> bool {
        self.lock().fits(value_count as usize)
    }

    /// `value_bits` are each value's IEEE 754 bits (`Float.floatToRawIntBits`), so they arrive unaltered.
    pub fn add(
        &self,
        timestamp_ns: i64,
        value_bits: Vec<u32>,
        accuracy: i32,
    ) -> Result<(), RideError> {
        Ok(self.lock().add(timestamp_ns, &value_bits, accuracy)?)
    }
}

impl SensorChunkBuilder {
    fn lock(&self) -> std::sync::MutexGuard<'_, ride_session::write::ChunkBuilder> {
        self.0.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// The session format version this build writes.
#[uniffi::export]
pub fn format_version() -> u32 {
    ride_session::FORMAT_VERSION
}
