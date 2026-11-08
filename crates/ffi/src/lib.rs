//! Rust exposed to the app through `uniffi`. Kotlin bindings are generated from this library at build time.
//!
//! Only thin wrappers live here: the logic is in `ride-fusion` and `ride-session`, shared with offline tools.

use std::sync::{Arc, Mutex};

mod display;

use ride_fusion::live_speed;
use ride_fusion::speed_colors::{self, SpeedScale as Scale};

uniffi::setup_scaffolding!();

/// The app version that last changed the Rust core (`scripts/bump-version.sh` sets it),
/// to check the app is running the Rust it was built with.
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
    Anchor,
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
            live_speed::Sensor::Anchor => Sensor::Anchor,
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
    /// Since the rider walked off a train, in the subway, until they board another.
    pub transfer_s: Option<f64>,
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
/// The `prost` tags are only for [`replay_session`]'s cache.
#[derive(Clone, PartialEq, uniffi::Record, prost::Message)]
pub struct SmoothedSpeed {
    #[prost(int64, tag = "1")]
    pub elapsed_realtime_ns: i64,
    #[prost(double, tag = "2")]
    pub speed_mps: f64,
    #[prost(double, tag = "3")]
    pub sigma_mps: f64,
    #[prost(double, tag = "4")]
    pub accel_mps2: f64,
}

impl From<ride_fusion::fusion::SmoothedSpeed> for SmoothedSpeed {
    fn from(s: ride_fusion::fusion::SmoothedSpeed) -> Self {
        SmoothedSpeed {
            elapsed_realtime_ns: s.elapsed_realtime_ns,
            speed_mps: s.speed_mps,
            sigma_mps: s.sigma_mps,
            accel_mps2: s.accel_mps2,
        }
    }
}

/// A train stop so far, for the live graph's labels (see `ride_fusion::fusion::StopRecord`).
#[derive(Clone, PartialEq, uniffi::Record, prost::Message)]
pub struct LiveStop {
    #[prost(int64, tag = "1")]
    pub stopped_ns: i64,
    #[prost(int64, optional, tag = "2")]
    pub departed_ns: Option<i64>,
    #[prost(string, optional, tag = "3")]
    pub station: Option<String>,
    #[prost(bool, tag = "4")]
    pub in_subway: bool,
    #[prost(int64, optional, tag = "5")]
    pub doors_open_ns: Option<i64>,
    #[prost(int64, optional, tag = "6")]
    pub doors_closed_ns: Option<i64>,
    /// When the rider walked off, if they changed trains or got off here (not stepping out and back on).
    #[prost(int64, optional, tag = "7")]
    pub left_ns: Option<i64>,
}

impl From<ride_fusion::fusion::StopRecord> for LiveStop {
    fn from(r: ride_fusion::fusion::StopRecord) -> Self {
        LiveStop {
            stopped_ns: r.stopped_ns,
            departed_ns: r.departed_ns,
            station: if r.between_stations {
                Some("between stations".into())
            } else {
                r.station.map(|s| s.name.clone())
            },
            in_subway: r.in_subway,
            doors_open_ns: r.doors_open_ns,
            doors_closed_ns: r.doors_closed_ns,
            left_ns: r.alighted_ns.filter(|_| r.rider_left()),
        }
    }
}

/// What changed in the train's stops since a caller's copy (see [`Fusion::stops_since`]):
/// keep the first `from`, replace the rest with `items`, and ask next time from `settled`.
#[derive(uniffi::Record)]
pub struct StopsDelta {
    pub from: u32,
    pub settled: u32,
    pub items: Vec<LiveStop>,
}

/// The live map's path underground (see [`Fusion::track_path`]): keep the legs starting before `from`,
/// replace the rest with `legs`.
#[derive(uniffi::Record)]
pub struct TrackPath {
    pub from: i64,
    pub legs: Vec<TrackLeg>,
}

/// A train's run between two stations along the track: its points, each with the smoothed speed there.
#[derive(uniffi::Record)]
pub struct TrackLeg {
    pub from_ns: i64,
    pub points: Vec<TrackPoint>,
}

#[derive(uniffi::Record)]
pub struct TrackPoint {
    pub elapsed_realtime_ns: i64,
    pub lat: f64,
    pub lon: f64,
    pub speed_mps: f64,
    pub accel_mps2: f64,
}

/// What changed in the timeline since a caller's copy (see [`Fusion::timeline_since`]), as [`StopsDelta`].
#[derive(uniffi::Record)]
pub struct TimelineDelta {
    pub from: u32,
    pub settled: u32,
    pub items: Vec<TimelineSegment>,
}

/// A stretch of what the rider did, for the live graph's strip (see `ride_fusion::fusion::Fusion::timeline`).
#[derive(Clone, PartialEq, uniffi::Record, prost::Message)]
pub struct TimelineSegment {
    #[prost(int64, tag = "1")]
    pub start_ns: i64,
    #[prost(int64, tag = "2")]
    pub end_ns: i64,
    /// `walking`, `stairs up`, `elevator or escalator down`, a line (`7`), or `subway` when the line isn't known.
    #[prost(string, tag = "3")]
    pub label: String,
    /// Walking, stairs, or an elevator or escalator, rather than a subway leg.
    #[prost(bool, tag = "4")]
    pub walking: bool,
}

impl From<ride_fusion::fusion::Segment> for TimelineSegment {
    fn from(s: ride_fusion::fusion::Segment) -> Self {
        use ride_fusion::fusion::Doing;
        TimelineSegment {
            start_ns: s.start_ns,
            end_ns: s.end_ns,
            walking: !matches!(s.doing, Doing::Subway { .. }),
            label: match s.doing {
                Doing::Walking => "walking".to_string(),
                Doing::Stairs { up: true } => "stairs up".to_string(),
                Doing::Stairs { up: false } => "stairs down".to_string(),
                Doing::ElevatorOrEscalator { up: true } => "elevator or escalator up".to_string(),
                Doing::ElevatorOrEscalator { up: false } => {
                    "elevator or escalator down".to_string()
                }
                Doing::Subway { line: Some(l) } => l,
                Doing::Subway { line: None } => "subway".to_string(),
            },
        }
    }
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

    /// One microphone read as recorded (16-bit little-endian PCM, `channels` interleaved, from stream frame
    /// `first_frame`), at `t_ns`, when its last frame was recorded, with the recorder's timestamps
    /// (frame positions and elapsed realtime ns) from it: door cycles in the sound hold the speed at zero.
    #[allow(clippy::too_many_arguments)]
    pub fn on_audio(
        &self,
        t_ns: i64,
        sample_rate_hz: u32,
        channels: u32,
        first_frame: i64,
        pcm_s16le: Vec<u8>,
        timestamp_frames: Vec<i64>,
        timestamp_ns: Vec<i64>,
    ) {
        self.lock().on_audio(
            t_ns,
            sample_rate_hz,
            ride_fusion::fusion::AudioSamples::from_pcm(
                first_frame,
                &pcm_s16le,
                channels as usize,
                timestamp_frames.into_iter().zip(timestamp_ns).collect(),
            ),
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

    /// The train's stops so far, oldest first, from the caller's copy settled up to `settled` (0 at first):
    /// settled ones never change, so only the rest crosses at each tick, however long the ride.
    pub fn stops_since(&self, settled: u32) -> StopsDelta {
        let (from, settled, items) = self.lock().stop_records_since(settled as usize);
        StopsDelta {
            from: from as u32,
            settled: settled as u32,
            items: items.into_iter().map(LiveStop::from).collect(),
        }
    }

    /// The path along the track underground between stations, for the live map: the legs that start in the
    /// smoothing window, each once the train stops at its far station.
    pub fn track_path(&self) -> TrackPath {
        let (from, legs) = self.lock().recent_track_path();
        TrackPath {
            from,
            legs: legs
                .into_iter()
                .map(|(from_ns, points)| TrackLeg {
                    from_ns,
                    points: points
                        .into_iter()
                        .map(|(s, (lat, lon))| TrackPoint {
                            elapsed_realtime_ns: s.elapsed_realtime_ns,
                            lat,
                            lon,
                            speed_mps: s.speed_mps,
                            accel_mps2: s.accel_mps2,
                        })
                        .collect(),
                })
                .collect(),
        }
    }

    /// What the rider did over the ride so far, in time order, from the caller's copy as [`Self::stops_since`].
    pub fn timeline_since(&self, settled: u32) -> TimelineDelta {
        let (from, settled, items) = self.lock().timeline_since(settled as usize);
        TimelineDelta {
            from: from as u32,
            settled: settled as u32,
            items: items.into_iter().map(TimelineSegment::from).collect(),
        }
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
                Mode::Stairs { up: true } => "stairs up",
                Mode::Stairs { up: false } => "stairs down",
                Mode::ElevatorOrEscalator { up: true } => "elevator or escalator up",
                Mode::ElevatorOrEscalator { up: false } => "elevator or escalator down",
                Mode::Subway { underground: true } => "subway, underground",
                Mode::Subway { underground: false } => "subway, above ground",
                Mode::Vehicle => "bus, car, or other train",
            }
            .to_string(),
            station: s.station.as_ref().map(|st| st.name.clone()),
            lines,
            at_station: s.at_station,
            route: s.route.clone(),
            // The chime's family, or the model when what's heard and the lines leave one:
            // the SMEE chime (R46, R62, R62A, R68, R68A) alone isn't one model.
            cars: s.car_type.and_then(|c| match (c.ntt, c.model) {
                (Some(true), Some(m)) => Some(format!("NTT ({m})")),
                (_, Some(m)) => Some(m.to_string()),
                (Some(true), None) => Some("NTT".to_string()),
                (Some(false), None) => Some("SMEE chime".to_string()),
                (None, None) => None,
            }),
            staying_s: s.staying_s,
            car: s.car.map(|c| format!("{c:04x}")),
            stopped_s: s.dwell.as_ref().map(|d| d.stopped_s),
            doors_open_s: s.dwell.as_ref().and_then(|d| d.doors_open_s),
            doors_were_open_s: s.dwell.as_ref().and_then(|d| d.doors_were_open_s),
            transfer_s: s.transfer_s,
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
            .map(SmoothedSpeed::from)
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

/// Round clock times (unix ms) to label the graph's time axis with, at most `max_ticks`; see [`display::clock_ticks`].
#[uniffi::export]
pub fn clock_ticks(
    start_unix_ms: i64,
    end_unix_ms: i64,
    utc_offset_ms: i64,
    max_ticks: u32,
) -> Vec<i64> {
    display::clock_ticks(start_unix_ms, end_unix_ms, utc_offset_ms, max_ticks)
}

/// A point on the map.
#[derive(Clone, Debug, PartialEq, uniffi::Record)]
pub struct LatLon {
    pub lat: f64,
    pub lon: f64,
}

/// Where a path (`path_ns`, `path_lat`, `path_lon`, in time order) was at each of `times_ns`, such as the rider's taps;
/// `None` off the path or in a gap over `max_gap_ns`. See [`display::positions_at`].
#[uniffi::export]
pub fn positions_at(
    times_ns: Vec<i64>,
    path_ns: Vec<i64>,
    path_lat: Vec<f64>,
    path_lon: Vec<f64>,
    max_gap_ns: i64,
) -> Vec<Option<LatLon>> {
    let path: Vec<_> = path_ns
        .into_iter()
        .zip(path_lat)
        .zip(path_lon)
        .map(|((t, lat), lon)| (t, lat, lon))
        .collect();
    display::positions_at(&times_ns, &path, max_gap_ns)
        .into_iter()
        .map(|p| p.map(|(lat, lon)| LatLon { lat, lon }))
        .collect()
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

/// When a past ride started and ended (unix ms), from its data (see `Session::end_unix_ms`).
#[derive(Clone, Debug, uniffi::Record)]
pub struct SessionSpan {
    pub start_unix_ms: i64,
    pub end_unix_ms: Option<i64>,
}

#[uniffi::export]
pub fn session_span(session_dir: String) -> Result<SessionSpan, RideError> {
    let session = ride_session::read::Session::open(&session_dir)?;
    Ok(SessionSpan {
        start_unix_ms: session.info.start_unix_ms,
        end_unix_ms: session.end_unix_ms()?,
    })
}

/// A GPS fix of a past ride, with the replay's smoothed speed there, for the map (see `LiveTrack.Point`).
#[derive(Clone, PartialEq, uniffi::Record, prost::Message)]
pub struct ReplayFix {
    #[prost(int64, tag = "1")]
    pub elapsed_realtime_ns: i64,
    #[prost(int64, tag = "2")]
    pub time_ms: i64,
    #[prost(double, tag = "3")]
    pub lat: f64,
    #[prost(double, tag = "4")]
    pub lon: f64,
    #[prost(double, optional, tag = "5")]
    pub gps_speed_mps: Option<f64>,
    #[prost(double, optional, tag = "6")]
    pub gps_speed_accuracy_mps: Option<f64>,
    #[prost(double, optional, tag = "7")]
    pub accuracy_m: Option<f64>,
    /// True if this fix starts a new segment after a gap (a tunnel).
    #[prost(bool, tag = "8")]
    pub starts_segment: bool,
    #[prost(double, optional, tag = "9")]
    pub speed_mps: Option<f64>,
    #[prost(double, optional, tag = "10")]
    pub accel_mps2: Option<f64>,
}

/// A point of a past ride's fused path (`ride_fusion::replay::Replay::map_path`):
/// GNSS and dead reckoning, joined to the subway track underground.
#[derive(Clone, PartialEq, uniffi::Record, prost::Message)]
pub struct ReplayPosition {
    #[prost(int64, tag = "1")]
    pub elapsed_realtime_ns: i64,
    #[prost(double, tag = "2")]
    pub lat: f64,
    #[prost(double, tag = "3")]
    pub lon: f64,
    /// One sigma (m).
    #[prost(double, tag = "4")]
    pub sigma_m: f64,
    #[prost(double, optional, tag = "5")]
    pub speed_mps: Option<f64>,
    #[prost(double, optional, tag = "6")]
    pub accel_mps2: Option<f64>,
}

/// A past ride replayed and smoothed over all of it (`ride_fusion::replay`), for its map and graph.
#[derive(Clone, PartialEq, uniffi::Record, prost::Message)]
pub struct RideReplay {
    #[prost(message, repeated, tag = "1")]
    pub smoothed: Vec<SmoothedSpeed>,
    #[prost(message, repeated, tag = "2")]
    pub track: Vec<ReplayFix>,
    #[prost(message, repeated, tag = "3")]
    pub stops: Vec<LiveStop>,
    #[prost(message, repeated, tag = "4")]
    pub timeline: Vec<TimelineSegment>,
    /// The core the cache was made by ([`core_hash`]); not for the app.
    #[prost(string, tag = "5")]
    pub build: String,
    /// Unix ms minus elapsed-realtime ms over the ride, from its start, to show times on the clock.
    #[prost(int64, tag = "6")]
    pub clock_offset_ms: i64,
    /// The fused path, one point a second; [`Self::track`] is the GPS fixes alone.
    #[prost(message, repeated, tag = "7")]
    pub positions: Vec<ReplayPosition>,
}

/// A hash of the Rust that decides what a replay gives (see `build.rs`): app builds that don't change it
/// keep their cached replays.
#[uniffi::export]
pub fn core_hash() -> String {
    env!("RIDE_CORE_HASH").into()
}

/// Told how far a replay has got (see `ride_fusion::replay::replay_with_progress`).
#[uniffi::export(with_foreign)]
pub trait ReplayProgress: Send + Sync {
    /// The fraction of the ride's inputs fed, every 1%; 0 once the session is read, 1 before smoothing it all.
    fn on_progress(&self, fraction: f64);
}

/// Where [`replay_session`] caches a session's replay: by the session directory's name.
fn replay_cache_path(session_dir: &str, cache_dir: &str) -> std::path::PathBuf {
    let id = std::path::Path::new(session_dir)
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default();
    std::path::Path::new(cache_dir).join(format!("{id}.pb"))
}

/// The replay cached at `cache`, if this core ([`core_hash`]) made it.
fn read_cached_replay(cache: &std::path::Path) -> Option<RideReplay> {
    use prost::Message;
    std::fs::read(cache)
        .ok()
        .and_then(|b| RideReplay::decode(&b[..]).ok())
        .filter(|r| r.build == core_hash())
}

/// The replay [`replay_session`] cached for this session, without replaying it if there's none:
/// cheap enough to show a past ride while recording.
#[uniffi::export]
pub fn cached_replay(session_dir: String, cache_dir: String) -> Option<RideReplay> {
    read_cached_replay(&replay_cache_path(&session_dir, &cache_dir))
}

/// Replays a past ride, or reads the replay cached in `cache_dir` if this core ([`core_hash`]) made it.
/// The cache is per session and never in the session's directory, which is uploaded;
/// tap edits and recompression don't change what a replay gives, so neither invalidates it.
#[uniffi::export]
pub fn replay_session(
    session_dir: String,
    cache_dir: String,
    progress: Arc<dyn ReplayProgress>,
) -> Result<RideReplay, RideError> {
    let build = core_hash();
    use prost::Message;
    let session = ride_session::read::Session::open(&session_dir)?;
    let cache = replay_cache_path(&session_dir, &cache_dir);
    if let Some(cached) = read_cached_replay(&cache) {
        return Ok(cached);
    }
    let options = ride_fusion::replay::Options {
        no_statuses: true,
        ..Default::default()
    };
    let r = ride_fusion::replay::replay_with_progress(&session, &options, &mut |f| {
        progress.on_progress(f)
    })?;
    let fixes = if session.stream_path("location").exists() {
        ride_session::track::gps_segments(&session.read_records("location")?.chunks)
    } else {
        Vec::new()
    };
    let track = fixes
        .iter()
        .flat_map(|segment| {
            segment.iter().enumerate().map(|(i, p)| {
                let s = r.at(p.elapsed_realtime_ns);
                ReplayFix {
                    elapsed_realtime_ns: p.elapsed_realtime_ns,
                    time_ms: p.time_ms,
                    lat: p.latitude_degrees,
                    lon: p.longitude_degrees,
                    gps_speed_mps: p.speed_meters_per_second.map(f64::from),
                    gps_speed_accuracy_mps: p.speed_accuracy_meters_per_second.map(f64::from),
                    accuracy_m: p.horizontal_accuracy_meters.map(f64::from),
                    starts_segment: i == 0,
                    speed_mps: s.map(|s| s.speed_mps),
                    accel_mps2: s.map(|s| s.accel_mps2),
                }
            })
        })
        .collect();
    let positions = r
        .map_path(1_000_000_000)
        .into_iter()
        .map(|(p, s)| ReplayPosition {
            elapsed_realtime_ns: p.elapsed_realtime_ns,
            lat: p.lat,
            lon: p.lon,
            sigma_m: p.sigma_m,
            speed_mps: s.map(|s| s.speed_mps),
            accel_mps2: s.map(|s| s.accel_mps2),
        })
        .collect();
    let replay = RideReplay {
        positions,
        smoothed: r.smoothed.into_iter().map(SmoothedSpeed::from).collect(),
        track,
        stops: r.stops.into_iter().map(LiveStop::from).collect(),
        timeline: r.timeline.into_iter().map(TimelineSegment::from).collect(),
        build,
        clock_offset_ms: session.info.start_unix_ms
            - session.info.start_elapsed_realtime_ns / 1_000_000,
    };
    // A failed write only means replaying again next time.
    let _ = std::fs::create_dir_all(&cache_dir).and_then(|()| {
        let partial = cache.with_extension("pb.partial");
        std::fs::write(&partial, replay.encode_to_vec())?;
        std::fs::rename(&partial, &cache)
    });
    Ok(replay)
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
