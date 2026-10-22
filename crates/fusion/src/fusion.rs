//! Everything the app and offline replay feed: IMU, GNSS fixes, raw GNSS, and navigation messages,
//! fused into a live speed and, over a window of recent inputs, a smoothed one.
//!
//! The one entry point for both, so live and offline can't differ:
//! - Raw GNSS measurements are solved for velocity (`ride_gnss`), which feeds the estimator with its real uncertainty;
//!   the phone's own fix speed is used only when there's no such solution, since both come from the same Doppler.
//! - Inputs settle into the forward pass in time order after [`SETTLE_NS`], whatever order they arrive in,
//!   and the live display gets a provisional estimate through the newest ones.
//! - Smoothing runs the same estimator backward in time over the buffered inputs (gyro negated, as time reversal
//!   runs rotation backward) and combines both directions at each fix and second by their uncertainties.
//!   Offline, it runs once over the whole ride. Live, it re-runs every [`SMOOTH_EVERY_NS`] of settled input
//!   that brought a new GNSS measurement, and a value freezes once smoothing has included [`SMOOTH_WINDOW_NS`]
//!   of GNSS after it: so the fixes after a tunnel, however long, correct all of it, up to [`MAX_UNMEASURED_NS`].
//!   The backward pass carries the first good fix after a tunnel back through it, which a forward filter can't.
//!   It starts from the forward pass's frame (gravity, travel axis, bias), which constant speed can't teach it.
//! - Whether the phone is in the subway ([`SubwayDetector`], from Wi-Fi and GNSS, never the rider's marks)
//!   switches the speed's upper bound to the subway's top speed while it is. Its inputs settle like any other,
//!   so live and offline switch at the same point.
//! - Position (see [`crate::position`]) is filtered from the same settled inputs: our own GNSS solutions,
//!   the fixes, the gyro's turns about up, and the forward speed at each sample; its inputs are light,
//!   so they're kept for the whole ride and smoothed when asked.

use std::collections::{BTreeMap, VecDeque};

use ride_gnss::solve::{Navigator, geodetic_to_ecef};
use ride_session::proto::{GnssMeasurementsEvent, GnssNavigationMessage};

use crate::car_beacons::{self, CarReading, CarTracker};
use crate::doors::ChimeCars;
use crate::doors::{DoorCycle, DoorDetector};
use crate::live_speed::{Estimate, LiveSpeedEstimator, SUBWAY_MAX_SPEED_MPS};
use crate::position::{PositionEstimate, PositionInput, smooth};
use crate::stations::{self, Station};
use crate::subway::SubwayDetector;
use crate::walking::{Gait, WalkingDetector};

/// Whether our own GNSS position and velocity solutions update the position filter.
/// Not yet: on the first rides (2026-10-20) they made it much worse even when cross-checked,
/// hundreds of meters off with tight covariances (once 31 km), while the phone's fixes alone left it
/// a median 1 to 2 m from them; the decoders are still unverified on this phone. Turn on once they are.
const USE_OWN_SOLUTIONS_FOR_POSITION: bool = false;

/// A fix without an accuracy is taken to be this good (m, 68%): about a GPS fix in a city.
const DEFAULT_FIX_ACCURACY_M: f64 = 20.0;

/// Own velocity solutions this recent suppress the phone's fix speed (which comes from the same Doppler).
const OWN_SOLUTION_HOLD_NS: i64 = 3_000_000_000;

/// One buffered input, for re-running the estimator.
#[derive(Clone, Debug)]
enum Input {
    Accel([f64; 3]),
    Gyro([f64; 3]),
    /// A fix, with its speed only if it was used. Boxed: fixes are rare, and IMU samples (most of the window,
    /// which on the phone is about half a million inputs) stay small.
    Fix(Box<Fix>),
    /// Our own velocity solution's speed and standard deviation.
    Speed(f64, f64),
    /// Entering or leaving the subway changed the speed's upper bound from this,
    /// which the backward pass goes back to.
    MaxSpeedBefore(Option<f64>),
    /// At a door cycle's closing: the train stood still from this time to here (see [`DWELL_SURE_S`]).
    StandingSince(i64),
    /// The rider has been walking, or running if true (see [`crate::walking`]).
    Walking(bool),
    /// The rider's car's accelerometer reading (see [`LiveSpeedEstimator::on_car_accel`]).
    Car(u16, [f64; 3]),
}

/// How long before a door cycle's closing the train surely stood (s): the chime comes in the 8 s before it,
/// and openings, found less surely, aren't used.
const DWELL_SURE_S: f64 = 8.0;
const DOOR_STOP_SIGMA_MPS: f64 = 0.2;

/// Across a gap between two fixes at least this good (m, 68%), the smoothed speeds may cover at most
/// [`GAP_PATH_RATIO`] times the straight distance between them, plus both accuracies and [`GAP_WANDER_MPS`]
/// over the gap; beyond that they're scaled down to it. Indoors at Flushing (`20261020-154951`, 4:27 pm) the speed
/// ran to 102 mph between fixes a few meters apart. A subway's track is rarely half again the straight line.
const GAP_FIX_M: f64 = 50.0;
/// A stop at an identified station pins the position to the station's point, this accurate (m, 68%):
/// the rider is anywhere along a platform of up to 180 m (ten 60 ft cars), and the MTA's point is about its middle.
const STATION_PIN_M: f64 = 90.0;
const GAP_MIN_NS: i64 = 10_000_000_000;
const GAP_PATH_RATIO: f64 = 2.0;
/// Walking around (a store, a platform) between the fixes, which the straight distance doesn't see.
const GAP_WANDER_MPS: f64 = 1.5;

/// Inputs settle into the forward pass, in time order, once input this much newer has arrived: Android delivers
/// sensor samples up to 1 s late (`SensorStreamRecorder.kt`'s batching), and GNSS a little after its epoch.
/// Live and offline feed the same buffer, so both see exactly the same order; until then, the live display
/// gets a provisional estimate through the unsettled inputs (see [`Fusion::estimate`]).
pub const SETTLE_NS: i64 = 1_500_000_000;

/// An input waiting to settle.
#[derive(Clone, Debug)]
enum Pending {
    Accel([f64; 3]),
    Gyro([f64; 3]),
    Fix(Fix),
    Measurements(Box<GnssMeasurementsEvent>),
    /// A Wi-Fi scan's SSIDs.
    WifiScan(Vec<String>),
    /// Microphone samples from a stream frame, with the recorder's (frame, elapsed realtime ns) timestamps.
    Audio(Box<AudioSamples>),
    /// The magnetic field's magnitude (uT).
    Mag(f64),
    /// The registered LTE cell's ID, if any.
    Cell(Option<i64>),
    /// Android's own location (fused, from Google's cell and Wi-Fi database underground): lat, lon, accuracy (m).
    NetworkFix(f64, f64, f64),
    /// A subway car's beacon (see [`crate::car_beacons`]).
    Car(CarReading),
}

#[derive(Clone, Debug)]
pub struct AudioSamples {
    pub first_frame: i64,
    pub samples: Vec<i16>,
    pub timestamps: Vec<(i64, i64)>,
}

impl Pending {
    /// Order among inputs at the same time: IMU first, as offline used to sort them.
    fn rank(&self) -> u8 {
        match self {
            Self::Accel(_) => 0,
            Self::Gyro(_) => 1,
            Self::Fix(_) => 2,
            Self::Measurements(_) => 3,
            Self::WifiScan(_) => 4,
            Self::Audio(_) => 5,
            Self::Mag(_) => 6,
            Self::Cell(_) => 7,
            Self::NetworkFix(..) => 8,
            Self::Car(_) => 9,
        }
    }
}

/// What sets the speed's upper bound: the vehicle's top speed if known, else the subway's while in it.
#[derive(Clone, Debug, Default)]
struct Context {
    subway: SubwayDetector,
    in_subway: bool,
    /// See [`Fusion::set_max_speed`].
    base_max_speed_mps: Option<f64>,
}

impl Context {
    fn max_speed_mps(&self) -> Option<f64> {
        self.base_max_speed_mps
            .or(self.in_subway.then_some(SUBWAY_MAX_SPEED_MPS))
    }
}

/// Feeds one settled (or provisional) input to the estimator; returns what smoothing keeps of it,
/// and the bound before it if it changed the speed's upper bound.
/// `own` is the speed and standard deviation of the velocity solved from `Measurements`, if any.
fn step(
    estimator: &mut LiveSpeedEstimator,
    last_own_solution_ns: &mut Option<i64>,
    context: &mut Context,
    t_ns: i64,
    input: &Pending,
    own: Option<(f64, f64)>,
) -> (Option<Input>, Option<Option<f64>>) {
    match input {
        Pending::WifiScan(ssids) => context
            .subway
            .on_wifi_scan(t_ns, ssids.iter().map(String::as_str)),
        Pending::Fix(fix) => context
            .subway
            .on_fix(t_ns, fix.lat, fix.lon, fix.accuracy, fix.speed),
        Pending::Mag(b) => context.subway.on_magnetometer(t_ns, *b),
        _ => {}
    }
    let in_subway = context.subway.in_subway(t_ns);
    let bound_before = (in_subway != context.in_subway).then(|| {
        let before = context.max_speed_mps();
        context.in_subway = in_subway;
        estimator.set_max_speed(context.max_speed_mps());
        before
    });
    (
        step_estimator(estimator, last_own_solution_ns, t_ns, input, own),
        bound_before,
    )
}

fn step_estimator(
    estimator: &mut LiveSpeedEstimator,
    last_own_solution_ns: &mut Option<i64>,
    t_ns: i64,
    input: &Pending,
    own: Option<(f64, f64)>,
) -> Option<Input> {
    match input {
        Pending::WifiScan(_)
        | Pending::Audio(_)
        | Pending::Mag(_)
        | Pending::Cell(_)
        | Pending::NetworkFix(..)
        | Pending::Car(_) => None,
        Pending::Accel(a) => {
            estimator.on_accel(t_ns, a[0], a[1], a[2]);
            Some(Input::Accel(*a))
        }
        Pending::Gyro(w) => {
            estimator.on_gyro(w[0], w[1], w[2]);
            Some(Input::Gyro(*w))
        }
        Pending::Fix(fix) => {
            let own_recent =
                last_own_solution_ns.is_some_and(|t| (t_ns - t).abs() <= OWN_SOLUTION_HOLD_NS);
            let (speed, speed_sigma) = if own_recent {
                (None, None)
            } else {
                (fix.speed, fix.speed_sigma)
            };
            estimator.on_gnss_fix(t_ns, fix.lat, fix.lon, fix.accuracy, speed, speed_sigma);
            Some(Input::Fix(Box::new(Fix {
                speed,
                speed_sigma,
                ..*fix
            })))
        }
        Pending::Measurements(_) => {
            let (speed, sigma) = own?;
            estimator.on_gnss_speed(t_ns, speed, sigma);
            *last_own_solution_ns = Some(t_ns);
            Some(Input::Speed(speed, sigma))
        }
    }
}

/// Android's location names a station when it's this sure (m) and the station is within this (m), or its accuracy.
const NETWORK_FIX_M: f64 = 150.0;
const NETWORK_STATION_M: f64 = 100.0;
/// Faster than this (m/s), not walking and not in the subway, is some other vehicle.
const VEHICLE_MPS: f64 = 4.0;
/// Staying put: fixes this good (m), all within this of the first (m), and none moving this fast (m/s).
const STAY_FIX_M: f64 = 50.0;
const STAY_RADIUS_M: f64 = 100.0;
const STAY_MOVING_MPS: f64 = 3.0;

/// What the rider is on, as well as the sensors tell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Unknown,
    Still,
    Walking,
    Running,
    /// The subway, under or above ground; on a train or in a station.
    Subway {
        underground: bool,
    },
    /// Moving faster than a run, not in the subway: a bus, car, or other train.
    Vehicle,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Status {
    pub mode: Mode,
    /// The station the registered cell names, in the subway; at it, if a door cycle there was just heard.
    pub station: Option<Station>,
    pub at_station: bool,
    /// The car type from the latest door chime, within 15 minutes.
    pub cars: Option<ChimeCars>,
    /// How long (s) the rider has stayed within 100 m, out of the subway; the app reminds them to stop recording.
    pub staying_s: f64,
    /// The rider's car: its beacon's ID (see [`crate::car_beacons`]).
    pub car: Option<u16>,
}

/// Forward estimates are kept at every fix and at least this often, so smoothing covers gaps without fixes.
const SAMPLE_EVERY_NS: i64 = 1_000_000_000;

/// Live smoothing re-runs at most this often, when settled input crosses a multiple of it with a new GNSS
/// measurement, so the map's recent path is recolored about as soon as a fix after a tunnel arrives.
/// Without one it wouldn't change: the backward pass has no weight until a measurement initializes it.
pub const SMOOTH_EVERY_NS: i64 = 1_000_000_000;
/// How much GNSS after a value live smoothing includes before freezing it, so how far back it reaches while GNSS
/// is available: on the phone, about 25 MB of IMU samples, and about 12 ms per re-run on a desktop.
pub const SMOOTH_WINDOW_NS: i64 = 600_000_000_000;
/// Live, values without a GNSS measurement after them freeze anyway once this old,
/// which bounds the phone's memory (about 150 MB of IMU samples) and the re-run on leaving a tunnel.
pub const MAX_UNMEASURED_NS: i64 = 3_600_000_000_000;

/// `android.hardware.Sensor` types.
pub const TYPE_ACCELEROMETER: i32 = 1;
pub const TYPE_MAGNETIC_FIELD: i32 = 2;
pub const TYPE_MAGNETIC_FIELD_UNCALIBRATED: i32 = 14;
pub const TYPE_GYROSCOPE: i32 = 4;
pub const TYPE_GYROSCOPE_UNCALIBRATED: i32 = 16;
pub const TYPE_ACCELEROMETER_UNCALIBRATED: i32 = 35;

pub enum ImuSample {
    Accel([f64; 3]),
    Gyro([f64; 3]),
    Mag([f64; 3]),
}

/// An accelerometer, gyro, or magnetometer sample from Android's values: uncalibrated sensors carry their bias estimate in
/// `values[3..6]`, which is subtracted (in `f32`, as Android computes the calibrated values).
pub fn imu_sample(sensor_type: i32, v: &[f32]) -> Option<ImuSample> {
    let uncalibrated = matches!(
        sensor_type,
        TYPE_ACCELEROMETER_UNCALIBRATED
            | TYPE_GYROSCOPE_UNCALIBRATED
            | TYPE_MAGNETIC_FIELD_UNCALIBRATED
    );
    if v.len() < 3 || (uncalibrated && v.len() < 6) {
        return None;
    }
    let xyz = std::array::from_fn(|k| f64::from(if uncalibrated { v[k] - v[k + 3] } else { v[k] }));
    match sensor_type {
        TYPE_ACCELEROMETER | TYPE_ACCELEROMETER_UNCALIBRATED => Some(ImuSample::Accel(xyz)),
        TYPE_GYROSCOPE | TYPE_GYROSCOPE_UNCALIBRATED => Some(ImuSample::Gyro(xyz)),
        TYPE_MAGNETIC_FIELD | TYPE_MAGNETIC_FIELD_UNCALIBRATED => Some(ImuSample::Mag(xyz)),
        _ => None,
    }
}

/// A GPS fix, as `android.location.Location` reports it (degrees, meters, m/s).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Fix {
    pub elapsed_realtime_ns: i64,
    pub lat: f64,
    pub lon: f64,
    pub altitude: Option<f64>,
    pub accuracy: Option<f64>,
    pub speed: Option<f64>,
    pub speed_sigma: Option<f64>,
}

/// A smoothed speed at a fix or a sample time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoothedSpeed {
    pub elapsed_realtime_ns: i64,
    pub speed_mps: f64,
    pub sigma_mps: f64,
    pub forward_mps: f64,
    pub backward_mps: f64,
    /// Acceleration in the direction of travel (m/s^2): the passes' low-passed rates of change of the fused speed,
    /// weighted like the speed. Their lags run opposite ways in time, so the combination is roughly centered.
    pub accel_mps2: f64,
}

pub struct Fusion {
    estimator: LiveSpeedEstimator,
    navigator: Navigator,
    last_own_solution_ns: Option<i64>,
    context: Context,
    /// Inputs not yet settled, by time, rank, and arrival.
    pending: BTreeMap<(i64, u8, u64), Pending>,
    arrivals: u64,
    /// The newest input time seen, and the newest settled.
    latest_ns: Option<i64>,
    settled_ns: Option<i64>,
    /// Inputs that arrived after newer ones had already settled, so went into the forward pass out of order.
    pub late_inputs: u64,
    /// Settled inputs with their times, and the forward estimate at each fix and sample time, within `window_ns`.
    inputs: VecDeque<(i64, Input)>,
    forward: VecDeque<(i64, Estimate)>,
    window_ns: Option<i64>,
    /// With a window: the latest smoothing of it, the values frozen as they left it,
    /// and the settled time at or after which smoothing re-runs.
    recent: VecDeque<SmoothedSpeed>,
    frozen: Vec<SmoothedSpeed>,
    next_smooth_ns: Option<i64>,
    /// The times of settled GNSS measurements (fixes and own velocities) in the window,
    /// and the newest one the latest smoothing included.
    measured: VecDeque<i64>,
    measured_ns: Option<i64>,
    smoothed_measured_ns: Option<i64>,
    /// The distance (m) covered by the frozen values (trapezoidally), from the first one's time
    /// to the last one's, whose time and speed the window's distance continues from.
    frozen_distance: Distance,
    /// The position filter's inputs for the whole ride, in settled order.
    position_inputs: Vec<(i64, PositionInput)>,
    /// The turn about up (rad) since the last sample, and the last gyro sample's time.
    turn: f64,
    last_gyro_ns: Option<i64>,
    /// Door cycles from the microphone, if it's fed; and those found so far.
    doors: Option<DoorDetector>,
    door_cycles: Vec<DoorCycle>,
    walking: WalkingDetector,
    cars: CarTracker,
    /// When the subway detector switched, and to what.
    subway_changes: Vec<(i64, bool)>,
    /// The registered LTE cell and when it was last reported.
    cell: Option<(i64, i64)>,
    /// The station nearest Android's own latest location, if close and sure enough, and when.
    network_station: Option<(i64, &'static Station)>,
    /// The latest door cycle's closing and the station its cell names, if one.
    last_stop: Option<(i64, Option<&'static Station>)>,
    /// Staying put: where the rider has been since when (a good fix's position, or none indoors).
    stay: Option<(i64, Option<(f64, f64)>)>,
    /// The newest settled input's time.
    now_ns: i64,
}

impl Default for Fusion {
    fn default() -> Self {
        Self::new()
    }
}

impl Fusion {
    /// Live: smoothing re-runs as GNSS measurements arrive, over [`SMOOTH_WINDOW_NS`] or since GNSS was lost.
    pub fn new() -> Self {
        Self::with_window(Some(SMOOTH_WINDOW_NS))
    }

    /// Offline: everything is kept and smoothed at once, when asked, so every tunnel gets the fix after it.
    pub fn whole_ride() -> Self {
        Self::with_window(None)
    }

    /// `window_ns`: how long values stay unfrozen once a GNSS measurement after them has been smoothed;
    /// `None` keeps everything and smooths it all at once.
    pub fn with_window(window_ns: Option<i64>) -> Self {
        Self {
            estimator: LiveSpeedEstimator::new(),
            navigator: Navigator::default(),
            last_own_solution_ns: None,
            context: Context::default(),
            pending: BTreeMap::new(),
            arrivals: 0,
            latest_ns: None,
            settled_ns: None,
            late_inputs: 0,
            inputs: VecDeque::new(),
            forward: VecDeque::new(),
            window_ns,
            recent: VecDeque::new(),
            frozen: Vec::new(),
            next_smooth_ns: None,
            measured: VecDeque::new(),
            measured_ns: None,
            smoothed_measured_ns: None,
            frozen_distance: Distance::default(),
            position_inputs: Vec::new(),
            turn: 0.0,
            last_gyro_ns: None,
            doors: None,
            door_cycles: Vec::new(),
            walking: WalkingDetector::default(),
            cars: CarTracker::default(),
            subway_changes: Vec::new(),
            cell: None,
            network_station: None,
            last_stop: None,
            stay: None,
            now_ns: 0,
        }
    }

    /// The fastest the vehicle can go, for the whole ride (see [`LiveSpeedEstimator::set_max_speed`]);
    /// without one, the subway's top speed applies while in the subway.
    pub fn set_max_speed(&mut self, max_speed_mps: Option<f64>) {
        self.context.base_max_speed_mps = max_speed_mps;
        self.estimator.set_max_speed(self.context.max_speed_mps());
    }

    /// Whether the phone is in the subway as of the settled inputs.
    pub fn in_subway(&self) -> bool {
        self.context.in_subway
    }

    /// A Wi-Fi scan's SSIDs, at the time it was recorded.
    pub fn on_wifi_scan(&mut self, t_ns: i64, ssids: Vec<String>) {
        self.arrive(t_ns, Pending::WifiScan(ssids));
    }

    /// Microphone samples (mono, `rate_hz`) as they're read, at `t_ns`, when they arrived;
    /// door cycles in them hold the speed at zero (see [`crate::doors`]).
    pub fn on_audio(&mut self, t_ns: i64, rate_hz: u32, audio: AudioSamples) {
        self.doors.get_or_insert_with(|| DoorDetector::new(rate_hz));
        self.arrive(t_ns, Pending::Audio(Box::new(audio)));
    }

    /// When the subway detector switched on (true) or off, in order.
    pub fn subway_changes(&self) -> &[(i64, bool)] {
        &self.subway_changes
    }

    /// Android's own location (the fused provider's, as recorded): underground, Google's database places the cell and
    /// Wi-Fi at station level (on `182135`, 110 St to 14 St station by station, mostly within 50 m).
    pub fn on_network_fix(&mut self, t_ns: i64, lat: f64, lon: f64, accuracy_m: Option<f64>) {
        self.arrive(
            t_ns,
            Pending::NetworkFix(lat, lon, accuracy_m.unwrap_or(f64::INFINITY)),
        );
    }

    /// The registered LTE cell (its ID, if any), at the time it was recorded.
    pub fn on_cell(&mut self, t_ns: i64, cell_id: Option<i64>) {
        self.arrive(t_ns, Pending::Cell(cell_id));
    }

    /// What the rider is doing and where, as of the settled input, for the app to show.
    pub fn status(&self) -> Status {
        let t = self.now_ns;
        let recent = |last: i64, s: f64| (t - last) as f64 * 1e-9 <= s;
        let gait = self
            .walking
            .last_gait()
            .filter(|&(at, _)| recent(at, 6.0))
            .and_then(|(_, g)| g);
        let estimate = self.estimator.estimate(t);
        let underground = estimate.since_gnss_s.is_none_or(|s| s > 10.0);
        let mode = match gait {
            Some(Gait::Running) => Mode::Running,
            Some(Gait::Walking) => Mode::Walking,
            None if self.context.in_subway => Mode::Subway { underground },
            None if estimate.initialized && estimate.speed_mps > VEHICLE_MPS => Mode::Vehicle,
            None if estimate.stopped => Mode::Still,
            None => Mode::Unknown,
        };
        // Android's own location first (Google's database), then the rider's tagged cells.
        let cell_station = self
            .network_station
            .filter(|&(at, _)| recent(at, 30.0))
            .map(|(_, s)| s)
            .or_else(|| {
                self.cell
                    .filter(|&(at, _)| recent(at, 30.0))
                    .and_then(|(_, c)| stations::of_cell(c))
            });
        let stopped_at = self
            .last_stop
            .filter(|&(at, _)| recent(at, 60.0))
            .and_then(|(_, s)| s);
        Status {
            mode,
            station: stopped_at
                .or(cell_station.filter(|_| self.context.in_subway))
                .cloned(),
            at_station: stopped_at.is_some(),
            cars: self
                .doors
                .as_ref()
                .and_then(DoorDetector::last_chime)
                .filter(|&(at, _)| recent(at, 900.0))
                .map(|(_, c)| c),
            staying_s: self
                .stay
                .map_or(0.0, |(since, _)| (t - since) as f64 * 1e-9),
            car: self.cars.current_at(t),
        }
    }

    /// Keeps track of how long the rider has stayed in one place out of the subway (see [`Status::staying_s`]).
    fn track_stay(&mut self, t_ns: i64, input: &Pending) {
        if self.context.in_subway {
            self.stay = Some((t_ns, None));
            return;
        }
        let (since, at) = *self.stay.get_or_insert((t_ns, None));
        if let Pending::Fix(f) = input
            && f.accuracy.is_some_and(|a| a <= STAY_FIX_M)
        {
            let moved = at.is_none_or(|p| stations::meters(p, (f.lat, f.lon)) > STAY_RADIUS_M);
            let moving = f.speed.is_some_and(|v| v >= STAY_MOVING_MPS);
            if moving || moved {
                self.stay = Some((t_ns, Some((f.lat, f.lon))));
            } else {
                self.stay = Some((since, at.or(Some((f.lat, f.lon)))));
            }
        }
    }

    /// The door cycles found so far.
    pub fn door_cycles(&self) -> &[DoorCycle] {
        &self.door_cycles
    }

    pub fn navigator(&self) -> &Navigator {
        &self.navigator
    }

    pub fn navigator_mut(&mut self) -> &mut Navigator {
        &mut self.navigator
    }

    /// An Android sensor sample, as delivered (`SensorEvent.values`); IMU samples are used, others ignored.
    pub fn on_sensor(&mut self, sensor_type: i32, t_ns: i64, values: &[f32]) {
        match imu_sample(sensor_type, values) {
            Some(ImuSample::Accel(a)) => self.on_accel(t_ns, a),
            Some(ImuSample::Gyro(w)) => self.on_gyro(t_ns, w),
            Some(ImuSample::Mag(b)) => self.on_magnetometer(t_ns, b),
            None => {}
        }
    }

    /// A Bluetooth advertisement (`ScanResult`'s data and signal strength), at the time it was received;
    /// a subway car's beacon gives the car's own acceleration.
    pub fn on_ble(&mut self, t_ns: i64, data: &[u8], rssi: i32) {
        if let Some(r) = car_beacons::decode(data, rssi) {
            self.arrive(t_ns, Pending::Car(r));
        }
    }

    /// A magnetometer sample (uT), for telling the subway's traction current.
    pub fn on_magnetometer(&mut self, t_ns: i64, b: [f64; 3]) {
        self.arrive(
            t_ns,
            Pending::Mag((b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt()),
        );
    }

    pub fn on_accel(&mut self, t_ns: i64, a: [f64; 3]) {
        self.arrive(t_ns, Pending::Accel(a));
    }

    pub fn on_gyro(&mut self, t_ns: i64, w: [f64; 3]) {
        self.arrive(t_ns, Pending::Gyro(w));
    }

    /// A GPS-provider fix (never fused or network ones, which can be the wrong station underground).
    pub fn on_fix(&mut self, fix: Fix) {
        self.arrive(fix.elapsed_realtime_ns, Pending::Fix(fix));
    }

    /// Navigation messages only add orbits, so they take effect on arrival, live and offline alike.
    pub fn on_navigation_message(&mut self, msg: &GnssNavigationMessage) {
        self.navigator.on_navigation_message(msg);
    }

    /// Raw measurements received at `received_ns`, solved once they settle.
    pub fn on_gnss_measurements(&mut self, received_ns: i64, event: &GnssMeasurementsEvent) {
        let t = event
            .clock
            .as_ref()
            .and_then(|c| c.elapsed_realtime_nanos)
            .unwrap_or(received_ns);
        self.arrive(t, Pending::Measurements(Box::new(event.clone())));
    }

    /// Settles everything still pending, as at the end of a recording, and smooths the window once more.
    pub fn finish(&mut self) {
        while let Some(((t, _, _), input)) = self.pending.pop_first() {
            self.settle(t, input);
        }
        if self.window_ns.is_some() {
            self.recent = self.smooth_window().into();
            self.smoothed_measured_ns = self.measured_ns;
        }
    }

    fn arrive(&mut self, t_ns: i64, input: Pending) {
        if self.settled_ns.is_some_and(|s| t_ns < s) {
            // Newer inputs already went forward; apply it now rather than lose it, and count it.
            self.late_inputs += 1;
            self.settle(t_ns, input);
            return;
        }
        self.arrivals += 1;
        self.pending
            .insert((t_ns, input.rank(), self.arrivals), input);
        let latest = self.latest_ns.map_or(t_ns, |l| l.max(t_ns));
        self.latest_ns = Some(latest);
        while let Some(entry) = self.pending.first_entry() {
            if entry.key().0 > latest - SETTLE_NS {
                break;
            }
            let ((t, _, _), input) = entry.remove_entry();
            self.settle(t, input);
        }
    }

    fn settle(&mut self, t_ns: i64, input: Pending) {
        self.settled_ns = Some(self.settled_ns.map_or(t_ns, |s| s.max(t_ns)));
        self.now_ns = self.now_ns.max(t_ns);
        if let Pending::Cell(c) = input {
            self.cell = c.map(|c| (t_ns, c));
            return;
        }
        if let Pending::NetworkFix(lat, lon, accuracy) = input {
            self.network_station = stations::nearest(lat, lon)
                .filter(|&(_, d)| accuracy <= NETWORK_FIX_M && d <= NETWORK_STATION_M.max(accuracy))
                .map(|(s, _)| (t_ns, s));
            return;
        }
        if let Pending::Car(r) = input {
            if self.cars.on_reading(t_ns, &r) {
                self.estimator.on_car_accel(t_ns, r.car, r.accel);
                self.buffer(t_ns, Input::Car(r.car, r.accel));
            }
            return;
        }
        if let (Pending::Audio(a), Some(doors)) = (&input, &mut self.doors) {
            for cycle in doors.on_audio(a.first_frame, &a.samples, &a.timestamps) {
                self.on_door_cycle(t_ns, cycle);
            }
            return;
        }
        let solution = match &input {
            Pending::Measurements(event) => self.navigator.on_measurements(event).ok(),
            Pending::Fix(f) => {
                self.navigator.set_approximate_position(geodetic_to_ecef(
                    f.lat,
                    f.lon,
                    f.altitude.unwrap_or(0.0),
                ));
                None
            }
            _ => None,
        };
        self.position_input(t_ns, &input, solution.as_ref());
        let own = solution
            .and_then(|sol| sol.velocity)
            .map(|v| (v.speed(), v.speed_sigma()));
        let (kept, bound_before) = step(
            &mut self.estimator,
            &mut self.last_own_solution_ns,
            &mut self.context,
            t_ns,
            &input,
            own,
        );
        if let Some(before) = bound_before {
            self.buffer(t_ns, Input::MaxSpeedBefore(before));
            self.subway_changes.push((t_ns, self.context.in_subway));
        }
        self.track_stay(t_ns, &input);
        if let Pending::Accel(a) = &input
            && self
                .walking
                .on_accel(t_ns, (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
                == Some(true)
        {
            let running = matches!(self.walking.last_gait(), Some((_, Some(Gait::Running))));
            self.estimator.on_walking(t_ns, running);
            self.buffer(t_ns, Input::Walking(running));
        }
        let Some(kept) = kept else {
            return;
        };
        let sample = match kept {
            Input::Fix(_) => true,
            Input::Accel(_) => self
                .forward
                .back()
                .is_none_or(|&(t, _)| t_ns - t >= SAMPLE_EVERY_NS),
            _ => false,
        };
        if matches!(kept, Input::Fix(_) | Input::Speed(..)) {
            self.measured_ns = Some(self.measured_ns.map_or(t_ns, |m| m.max(t_ns)));
            self.measured.push_back(t_ns);
        }
        self.buffer(t_ns, kept);
        if sample {
            let estimate = self.estimator.estimate(t_ns);
            if estimate.initialized {
                if self.turn != 0.0 {
                    self.position_inputs
                        .push((t_ns, PositionInput::Turn(std::mem::take(&mut self.turn))));
                }
                self.position_inputs.push((
                    t_ns,
                    PositionInput::Speed(estimate.speed_mps, estimate.sigma_mps),
                ));
            }
            self.position_inputs.push((t_ns, PositionInput::Sample));
            self.forward.push_back((t_ns, estimate));
        }
        if self.window_ns.is_some() {
            let next = *self.next_smooth_ns.get_or_insert(t_ns);
            if t_ns >= next && self.measured_ns != self.smoothed_measured_ns {
                self.recent = self.smooth_window().into();
                self.smoothed_measured_ns = self.measured_ns;
                self.next_smooth_ns =
                    Some(t_ns - t_ns.rem_euclid(SMOOTH_EVERY_NS) + SMOOTH_EVERY_NS);
            }
        }
    }

    /// A door cycle found at `t_ns`: the train stood from [`DWELL_SURE_S`] before its closing to the closing.
    fn on_door_cycle(&mut self, t_ns: i64, cycle: DoorCycle) {
        self.door_cycles.push(cycle);
        let network = self
            .network_station
            .filter(|&(at, _)| (t_ns - at) as f64 * 1e-9 <= 30.0)
            .map(|(_, s)| s);
        let station = network.or_else(|| self.cell.and_then(|(_, c)| stations::of_cell(c)));
        self.last_stop = Some((t_ns, station));
        // Stopped at a known station: underground, the only fix there is, and what keeps the heading fix
        // (the position's direction of travel, `ce084a5`) from carrying a drift on to the next station.
        if self.context.in_subway
            && let Some(s) = station
        {
            self.position_inputs.push((
                cycle.close_ns - (DWELL_SURE_S * 1e9) as i64 / 2,
                PositionInput::Fix {
                    lat: s.lat,
                    lon: s.lon,
                    accuracy_m: STATION_PIN_M,
                },
            ));
        }
        // Since the speed was last measured, the IMU alone carried it, and its error there is mostly a constant
        // leak (gravity through a small tilt, or bias), which grows linearly: take it out of the forward samples
        // as a ramp from nothing then to all of it at the closing, and all of it after.
        if let Some(anchor) = self
            .estimator
            .last_measured_ns()
            .filter(|&a| a < cycle.close_ns)
            && let Some(error) = self
                .forward
                .iter()
                .rev()
                .find(|(t, _)| *t <= cycle.close_ns)
                .map(|(_, e)| e.speed_mps)
        {
            let ramp = |t: i64| {
                if t <= anchor {
                    0.0
                } else if t >= cycle.close_ns {
                    error
                } else {
                    error * (t - anchor) as f64 / (cycle.close_ns - anchor) as f64
                }
            };
            for (t, e) in self.forward.iter_mut().rev() {
                if *t <= anchor {
                    break;
                }
                e.speed_mps = (e.speed_mps - ramp(*t)).max(0.0);
            }
            for (t, input) in self.position_inputs.iter_mut().rev() {
                if *t <= anchor {
                    break;
                }
                if let PositionInput::Speed(v, _) = input {
                    *v = (*v - ramp(*t)).max(0.0);
                }
            }
        }
        self.estimator.on_stopped_at(cycle.close_ns, t_ns);
        let from = cycle.close_ns - (DWELL_SURE_S * 1e9) as i64;
        self.buffer(cycle.close_ns, Input::StandingSince(from));
        let mut t = from;
        while t <= cycle.close_ns {
            self.position_inputs
                .push((t, PositionInput::Speed(0.0, DOOR_STOP_SIGMA_MPS)));
            t += SAMPLE_EVERY_NS;
        }
    }

    /// Keeps what the position filter needs of a settled input.
    fn position_input(
        &mut self,
        t_ns: i64,
        input: &Pending,
        solution: Option<&ride_gnss::solve::Solution>,
    ) {
        match input {
            Pending::Gyro(w) => {
                // The rate about up turns the direction of travel, whichever way the phone is held.
                if let (Some(last), Some(up)) = (self.last_gyro_ns, self.estimator.up()) {
                    let dt = (t_ns - last) as f64 * 1e-9;
                    if dt > 0.0 && dt < 1.0 {
                        self.turn += (w[0] * up[0] + w[1] * up[1] + w[2] * up[2]) * dt;
                    }
                }
                self.last_gyro_ns = Some(t_ns);
            }
            Pending::Fix(f) => self.position_inputs.push((
                t_ns,
                PositionInput::Fix {
                    lat: f.lat,
                    lon: f.lon,
                    accuracy_m: f.accuracy.unwrap_or(DEFAULT_FIX_ACCURACY_M),
                },
            )),
            Pending::Measurements(_) => {
                let Some(sol) = solution else { return };
                if !USE_OWN_SOLUTIONS_FOR_POSITION {
                    return;
                }
                if let Some(p) = &sol.position {
                    self.position_inputs.push((
                        t_ns,
                        PositionInput::OwnPosition {
                            ecef: p.position,
                            covariance: p.position_covariance,
                        },
                    ));
                }
                if let Some(v) = &sol.velocity {
                    self.position_inputs.push((
                        t_ns,
                        PositionInput::OwnVelocity {
                            ecef: v.velocity,
                            covariance: v.velocity_covariance,
                        },
                    ));
                }
            }
            Pending::Accel(_)
            | Pending::WifiScan(_)
            | Pending::Audio(_)
            | Pending::Mag(_)
            | Pending::Cell(_)
            | Pending::NetworkFix(..)
            | Pending::Car(_) => {}
        }
    }

    /// Positions at every fix and sample time so far, smoothed over the whole ride.
    pub fn positions(&self) -> Vec<PositionEstimate> {
        // Settled inputs are in time order, except any that arrived late.
        let mut inputs = self.position_inputs.clone();
        inputs.sort_by_key(|(t, _)| *t);
        smooth(&inputs)
    }

    /// The estimate at `now_ns`: the settled forward pass carried on, in a copy, through the inputs still
    /// settling, with measurements solved without changing the navigator. Nothing of it is kept.
    pub fn estimate(&self, now_ns: i64) -> Estimate {
        if self.pending.is_empty() {
            return self.estimator.estimate(now_ns);
        }
        let mut estimator = self.estimator.clone();
        let mut last_own = self.last_own_solution_ns;
        let mut context = self.context.clone();
        for (&(t, _, _), input) in &self.pending {
            let own = match input {
                Pending::Measurements(event) => self
                    .navigator
                    .solve(event)
                    .ok()
                    .and_then(|sol| sol.velocity)
                    .map(|v| (v.speed(), v.speed_sigma())),
                _ => None,
            };
            step(&mut estimator, &mut last_own, &mut context, t, input, own);
        }
        estimator.estimate(now_ns)
    }

    fn buffer(&mut self, t_ns: i64, input: Input) {
        self.inputs.push_back((t_ns, input));
        if let Some(window) = self.window_ns {
            // Smoothed with a window's worth of GNSS after it, or too old to wait any longer.
            let measured = &self.measured;
            let smoothed = self.smoothed_measured_ns;
            let done = |t: i64| {
                t_ns - t > MAX_UNMEASURED_NS
                    || measured
                        .iter()
                        .find(|&&m| m > t)
                        .is_some_and(|&first| smoothed.is_some_and(|s| s - first >= window))
            };
            while self.inputs.front().is_some_and(|(t, _)| done(*t)) {
                self.inputs.pop_front();
            }
            while let Some(&(t, ref f)) = self.forward.front() {
                if !done(t) {
                    break;
                }
                // Freeze its latest smoothing, or the forward estimate alone if smoothing never reached it.
                while self
                    .recent
                    .front()
                    .is_some_and(|s| s.elapsed_realtime_ns < t)
                {
                    self.recent.pop_front();
                }
                let value = match self.recent.front() {
                    Some(s) if s.elapsed_realtime_ns == t => self.recent.pop_front().unwrap(),
                    _ => combine(
                        t,
                        f,
                        &Estimate {
                            initialized: false,
                            ..f.clone()
                        },
                    ),
                };
                self.frozen_distance.add(&value);
                self.frozen.push(value);
                self.forward.pop_front();
            }
            let oldest = self.inputs.front().map_or(t_ns, |&(t, _)| t);
            while self.measured.len() > 1 && self.measured[1] <= oldest {
                self.measured.pop_front();
            }
        }
    }

    /// The whole ride's average smoothed speed (m/s), stops included, once it covers any time:
    /// live, the window's part changes as smoothing re-runs, so a tunnel's is corrected by the fix after it.
    pub fn average_speed_mps(&self) -> Option<f64> {
        let mut distance = self.frozen_distance;
        if self.window_ns.is_none() {
            self.smooth_window().iter().for_each(|s| distance.add(s));
        } else {
            self.recent.iter().for_each(|s| distance.add(s));
        }
        distance.average_mps()
    }

    /// Smoothed speeds at every fix and sample time so far: those frozen as they left the window, then the
    /// window's latest smoothing (or, without a window, everything smoothed at once).
    pub fn smoothed(&self) -> Vec<SmoothedSpeed> {
        if self.window_ns.is_none() {
            return self.smooth_window();
        }
        self.frozen.iter().chain(&self.recent).copied().collect()
    }

    /// The window's latest smoothing, which the live map recolors its recent path with.
    pub fn recent_smoothed(&self) -> Vec<SmoothedSpeed> {
        if self.window_ns.is_none() {
            return self.smooth_window();
        }
        self.recent.iter().copied().collect()
    }

    /// Smoothed speeds at every fix and sample time in the window: the forward estimate there combined with
    /// a backward pass over the window, each weighted by its variance. The backward pass gets no weight until a
    /// measurement initializes it, since it starts from nothing at the window's end.
    fn smooth_window(&self) -> Vec<SmoothedSpeed> {
        let Some(&(end, _)) = self.inputs.back() else {
            return Vec::new();
        };
        // Settled inputs are in time order, except any that arrived late; sort a copy for those.
        let mut inputs: Vec<&(i64, Input)> = self.inputs.iter().collect();
        inputs.sort_by_key(|(t, _)| *t);
        let mut backward = self.estimator.reversed();
        // Forward samples from the latest back, each paired with the backward estimate once the pass reaches it.
        let mut pending = self.forward.iter().rev().peekable();
        let mut pairs: Vec<(i64, &Estimate, Estimate)> = Vec::with_capacity(self.forward.len());
        // Going back through a door cycle's dwell: the time it began.
        let mut standing_since: Option<i64> = None;
        // The last (earliest so far) time a measurement set the backward speed, and the first pair after it.
        let mut anchor: Option<(i64, usize)> = None;
        for &&(t, ref input) in inputs.iter().rev() {
            // Reversed time runs forward from the end; rotation changes sign, acceleration doesn't
            // (see `LiveSpeedEstimator::reversed`).
            let tr = end - t + 1;
            match *input {
                Input::Accel(a) => {
                    backward.on_accel(tr, a[0], a[1], a[2]);
                    match standing_since {
                        Some(from) if t >= from => backward.on_known_stop(tr),
                        Some(_) => {
                            standing_since = None;
                            anchor = Some((t, pairs.len()));
                        }
                        None => {}
                    }
                }
                Input::Walking(running) => {
                    backward.on_walking(tr, running);
                    anchor = Some((t, pairs.len()));
                }
                Input::StandingSince(from) => {
                    // As in the forward pass (`on_door_cycle`): the drift since the last measurement, as a ramp.
                    if let Some((ta, first)) = anchor
                        && ta > t
                    {
                        let error = backward.estimate(tr).speed_mps;
                        for (ts, _, b) in &mut pairs[first..] {
                            let ramp = error * (ta - *ts) as f64 / (ta - t) as f64;
                            b.speed_mps = (b.speed_mps - ramp.clamp(0.0, error)).max(0.0);
                        }
                    }
                    anchor = None;
                    standing_since = Some(from);
                }
                Input::Gyro(w) => backward.on_gyro(-w[0], -w[1], -w[2]),
                Input::Car(car, a) => backward.on_car_accel(tr, car, a),
                Input::Speed(speed, sigma) => {
                    backward.on_gnss_speed(tr, speed, sigma);
                    anchor = Some((t, pairs.len()));
                }
                // Going back past a switch, the bound is the one before it.
                Input::MaxSpeedBefore(before) => backward.set_max_speed(before),
                Input::Fix(ref f) => {
                    backward.on_gnss_fix(tr, f.lat, f.lon, f.accuracy, f.speed, f.speed_sigma);
                    if f.speed.is_some() {
                        anchor = Some((t, pairs.len()));
                    }
                }
            }
            while let Some(&&(ts, ref f)) = pending.peek() {
                if ts < t {
                    break;
                }
                pairs.push((ts, f, backward.estimate(end - ts + 1)));
                pending.next();
            }
        }
        let mut out: Vec<SmoothedSpeed> =
            pairs.iter().map(|(ts, f, b)| combine(*ts, f, b)).collect();
        out.reverse();
        let fixes: Vec<&Fix> = inputs
            .iter()
            .filter_map(|(_, input)| match input {
                Input::Fix(f) if f.accuracy.is_some_and(|a| a <= GAP_FIX_M) => Some(&**f),
                _ => None,
            })
            .collect();
        bound_gaps(&mut out, &fixes);
        out
    }
}

/// Scales down the smoothed speeds across each gap between good fixes to the distance the fixes allow
/// (see [`GAP_FIX_M`]). Only a bound: a path longer than the straight line is fine up to it.
fn bound_gaps(out: &mut [SmoothedSpeed], fixes: &[&Fix]) {
    for pair in fixes.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let (ta, tb) = (a.elapsed_realtime_ns, b.elapsed_realtime_ns);
        if tb - ta < GAP_MIN_NS {
            continue;
        }
        let gap_s = (tb - ta) as f64 * 1e-9;
        let allowed = GAP_PATH_RATIO * stations::meters((a.lat, a.lon), (b.lat, b.lon))
            + a.accuracy.unwrap_or(0.0)
            + b.accuracy.unwrap_or(0.0)
            + GAP_WANDER_MPS * gap_s;
        let from = out.partition_point(|s| s.elapsed_realtime_ns <= ta);
        let to = out.partition_point(|s| s.elapsed_realtime_ns < tb);
        if to <= from {
            continue;
        }
        let mut distance = Distance::default();
        out[from.saturating_sub(1)..(to + 1).min(out.len())]
            .iter()
            .for_each(|s| distance.add(s));
        if distance.meters > allowed {
            let k = allowed / distance.meters;
            for s in &mut out[from..to] {
                s.speed_mps *= k;
                s.accel_mps2 *= k;
            }
        }
    }
}

/// Distance (m) along smoothed speeds, trapezoidally between their times; values out of time order are skipped.
#[derive(Clone, Copy, Debug, Default)]
struct Distance {
    meters: f64,
    first_ns: Option<i64>,
    last: Option<(i64, f64)>,
}

impl Distance {
    fn add(&mut self, s: &SmoothedSpeed) {
        let t = s.elapsed_realtime_ns;
        let v = s.speed_mps.max(0.0);
        match self.last {
            Some((last_t, _)) if t <= last_t => return,
            Some((last_t, last_v)) => {
                self.meters += (last_v + v) / 2.0 * (t - last_t) as f64 * 1e-9
            }
            None => self.first_ns = Some(t),
        }
        self.last = Some((t, v));
    }

    fn average_mps(&self) -> Option<f64> {
        let first = self.first_ns?;
        let (last, _) = self.last?;
        (last > first).then(|| self.meters / ((last - first) as f64 * 1e-9))
    }
}

fn combine(t: i64, f: &Estimate, b: &Estimate) -> SmoothedSpeed {
    let wf = 1.0 / f.sigma_mps.max(1e-3).powi(2);
    let wb = if b.initialized {
        1.0 / b.sigma_mps.max(1e-3).powi(2)
    } else {
        0.0
    };
    SmoothedSpeed {
        elapsed_realtime_ns: t,
        speed_mps: (f.speed_mps * wf + b.speed_mps * wb) / (wf + wb),
        sigma_mps: (1.0 / (wf + wb)).sqrt(),
        forward_mps: f.speed_mps,
        backward_mps: b.speed_mps,
        // In reversed time, speeding up is slowing down.
        accel_mps2: (f.accel_mps2 * wf - b.accel_mps2 * wb) / (wf + wb),
    }
}
