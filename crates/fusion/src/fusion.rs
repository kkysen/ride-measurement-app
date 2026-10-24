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

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use ride_gnss::solve::{Navigator, geodetic_to_ecef};
use ride_session::proto::{GnssMeasurementsEvent, GnssNavigationMessage};

use crate::car_beacons::{self, CarReading, CarTracker};
use crate::doors::ChimeCars;
use crate::doors::{DoorCycle, DoorDetector};
use crate::live_speed::{Estimate, LiveSpeedEstimator, SUBWAY_MAX_SPEED_MPS};
use crate::position::{PositionEstimate, PositionInput, smooth};
use crate::stations::{self, CarType, Station};
use crate::stops::{self as train_stops, StopDetector};
use crate::subway::SubwayDetector;
use crate::trips;
use crate::walking::{Gait, WINDOW_S as WALK_WINDOW_S, WalkingDetector};

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
/// A stop's record: its doors' cycle closes from [`DOORS_AFTER_STOP_S`]'s start on, and before the next stop
/// or [`RECORD_MAX_DWELL_S`]; the opening is the cycle's own if heard within [`DOORS_AFTER_STOP_S`] of the stop,
/// else unknown: conductors open the doors when they choose, so no fixed time after the stop stands in for it
/// (the rider's taps ranged 1.9 to 5.2 s after the lurch, 10th to 90th percentile).
/// The departure is the first start surge after the closing (or [`DEPART_AFTER_STOP_S`] after the stop).
const DOORS_AFTER_STOP_S: (f64, f64) = (1.0, 10.0);
const DEPART_AFTER_STOP_S: f64 = 5.0;
const RECORD_MAX_DWELL_S: f64 = 300.0;
/// A transfer (see [`Fusion::transfer_s`]) is shown for at most this long (s): a late night wait for a train.
const TRANSFER_MAX_S: f64 = 1_800.0;
/// Between trains (see [`Fusion::transfer_s`]) the live speed shows at most a run (m/s).
const BETWEEN_TRAINS_MAX_MPS: f64 = 3.0;
/// A departure is the rider's new train if no step window ends within this before and after it (s).
const TRANSFER_STILL_S: (f64, f64) = (10.0, 5.0);
/// A car heard as the rider's this long (s) is their train's, not one standing at the platform:
/// at Jackson Hts on `184134`, 7:21 pm, the trains waited at most 70 s.
const TRANSFER_CAR_S: f64 = 180.0;
/// A boarding from the platform stands for a stop this long (s), before its departure or after the new car is
/// first heard: well within the feeds' [`trips::SLACK_S`].
const BOARDING_STOP_S: f64 = 30.0;
/// Step windows ([`WALK_WINDOW_S`] each) between stops that start the rider's line over (see [`Fusion::riding_lines`]).
const LINE_RESET_WINDOWS: usize = 2;
/// A GNSS speed over this after a likely stop means the train is moving after all.
const LIKELY_STOP_GNSS_MPS: f64 = 1.0;
/// By when (s after the lurch) the stop detector has decided on it: [`train_stops::SEP_S`] and the 10 s it waits more.
const LIKELY_DECIDED_BY_S: f64 = train_stops::SEP_S + 12.0;
/// A "stop" whose braking began within [`FADE_AFTER_CLOSE_S`] after a door closing, and lasted at least
/// [`FADE_MIN_S`], is the departure's pull fading (the train sped away, then cruised): at Grand Central on
/// `20261020-151449` (3:44 pm), and three more, the pull began 0 to 8 s after the closing and lasted 11 to 17 s.
/// A false closing heard while pulling in (`20261020-184134`, 7:00 pm) came 5 s before a real stop.
const FADE_AFTER_CLOSE_S: (f64, f64) = (-0.5, 8.0);
const FADE_MIN_S: f64 = 7.0;
/// The rider walks off at a stop when [`ALIGHT_WINDOWS`] of the next [`ALIGHT_OF`] step-rhythm windows have
/// steps (12 s or more of the 16), before the departure: a step out of the doors to let others off is shorter.
/// [`PLATFORM_WINDOWS`] in the [`PLATFORM_BEFORE_S`] before a stop, still walking in the [`PLATFORM_LAST_S`]
/// before it, mean the rider was on the platform, not riding: what's heard is another train's
/// (or the rider's own, later boarded). Walking up the car to the doors ends as the train stops (Spring St,
/// `20261021-215922`, 10:07 pm).
const ALIGHT_WINDOWS: usize = 3;
const ALIGHT_OF: usize = 4;
const PLATFORM_WINDOWS: usize = 3;
const PLATFORM_BEFORE_S: f64 = 24.0;
const PLATFORM_LAST_S: f64 = 8.0;
/// A stop the lurch detector missed, found from the rider walking off: in the subway, a departure after at least
/// [`MISSED_STOP_WALK_WINDOWS`] step windows (24 s, more than the rider walks along a moving train), starting with a
/// walk off ([`ALIGHT_WINDOWS`] of [`ALIGHT_OF`]), riding before it (a departure, with no stop found from it, nor from
/// [`MISSED_STOP_LOOKBACK_S`] before the walk, to the departure). Without the riding, walks into a station to board
/// (`20261021-155919`, 4:00 pm; `20261020-214427`, 9:44 pm) and a wait on a platform (`20261021-215922`, 10:14 pm)
/// came out as stops. The train stopped [`ALIGHT_AFTER_STOP_S`] before the walk began: at 96 St on
/// `20261021-215922` (10:37 pm, a 3 to 1 transfer across the platform), tapped stopped 10:37:17 and off 10:37:23,
/// with no lurch found, the speed ran on at 40 mph while the rider walked the platform.
/// Its record's score is [`MISSED_STOP_SCORE`].
const MISSED_STOP_WALK_WINDOWS: usize = 6;
const MISSED_STOP_LOOKBACK_S: f64 = 60.0;
const ALIGHT_AFTER_STOP_S: f64 = 6.0;
pub const MISSED_STOP_SCORE: f64 = 0.0;

/// One stop of the train, for later analyses: dwell, and the doors' times. A stop with no door cycle is a
/// tunnel stop, or one whose doors weren't heard.
#[derive(Clone, Debug, PartialEq)]
pub struct StopRecord {
    /// The stopping lurch (see [`crate::stops`]), and its score (higher is surer).
    pub stopped_ns: i64,
    pub score: f64,
    pub in_subway: bool,
    /// The MTA's name for the station Android's location or the tagged cell named, if any.
    pub station: Option<&'static Station>,
    pub doors_open_ns: Option<i64>,
    /// Whether the opening was heard, rather than estimated from the stop.
    pub doors_open_heard: bool,
    pub doors_closed_ns: Option<i64>,
    pub departed_ns: Option<i64>,
    /// The rider was walking on the platform before this stop, so it's another train's, or the one they board.
    pub on_platform: bool,
    /// When the rider walked off (the start of the steps), if they did before the train left: the record's train
    /// leaves unseen, and a closing is kept only if heard within [`HEARD_FROM_PLATFORM_S`] of the stop.
    pub alighted_ns: Option<i64>,
    /// When the rider boarded (the end of the steps) after walking on the platform, and before
    /// [`StopRecord::departed_ns`], which is then the boarded train's.
    pub boarded_ns: Option<i64>,
    /// The car the rider was in (the strongest car beacon) in the [`CAR_SPAN_S`] before stepping off,
    /// and after boarding: different around a step out is a change of car on the same train.
    pub car_before: Option<u16>,
    pub car_after: Option<u16>,
    /// Every car heard steadily in the [`TRAIN_SPAN_S`] before the stop differs from every car heard after the
    /// departure: another train, boarded across the platform with too few steps to see (see [`trains_differ`]).
    pub train_change: bool,
    /// A hold between stations, not a platform stop (see [`Fusion::between_stations`]):
    /// the train stood, but the dwell starts at the platform.
    pub between_stations: bool,
}

/// A walk off was along the car when there are at most [`CARRIED_STEP_WINDOWS`] step windows from
/// [`CARRIED_WALK_S`] after it to the next stop (see `carried_on` in [`stop_records`]).
const CARRIED_WALK_S: f64 = 24.0;
const CARRIED_STEP_WINDOWS: usize = 2;
/// A closing heard after the rider walked off is still their train's if this soon after the stop (s).
const HEARD_FROM_PLATFORM_S: f64 = 60.0;
/// Off and back on within this (s) is stepping out of the doors (`20261020-184134`, 7:07 pm, 16 s), not a transfer.
const STEP_OUT_S: f64 = 30.0;

impl StopRecord {
    /// From the stop to pulling away (s), when the rider rode through it.
    pub fn dwell_s(&self) -> Option<f64> {
        if self.between_stations
            || self.on_platform
            || (self.alighted_ns.is_some() && !self.stepped_out())
        {
            return None;
        }
        self.departed_ns
            .map(|d| (d - self.stopped_ns) as f64 * 1e-9)
    }

    /// Whether the rider changed trains or got off here (not just stepping out to let others off).
    pub fn rider_left(&self) -> bool {
        self.alighted_ns.is_some() && !self.stepped_out()
    }

    /// Whether the rider stepped off and back on within [`STEP_OUT_S`]: the same train, still ridden.
    pub fn stepped_out(&self) -> bool {
        self.alighted_ns
            .zip(self.boarded_ns)
            .is_some_and(|(a, b)| ((b - a) as f64 * 1e-9) < STEP_OUT_S)
    }

    /// Whether the rider stepped out and back into another car of the same train
    /// (82 St, `20261020-184134`, 7:07 pm).
    pub fn car_change(&self) -> bool {
        self.stepped_out()
            && self
                .car_before
                .zip(self.car_after)
                .is_some_and(|(a, b)| a != b)
    }

    /// How long the doors stood open (s).
    pub fn doors_open_s(&self) -> Option<f64> {
        Some((self.doors_closed_ns? - self.doors_open_ns?) as f64 * 1e-9)
    }
}

/// Whether the train's stopping lurch (see [`crate::stops`]) holds the speed at zero, in the subway,
/// for the [`STOP_STANDING_S`] after the braking's collapse that showed the car still.
const USE_STOPPING_LURCH: bool = true;
const STOP_STANDING_S: f64 = 4.0;

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
/// Walking around (a store, a platform) between the fixes, which the straight distance doesn't see,
/// for the time steps were detected (each detection covers the step window before it); and running.
const GAP_WANDER_MPS: f64 = 1.5;
const GAP_RUN_MPS: f64 = 4.5;
const GAP_STEP_WINDOW_NS: i64 = 4_000_000_000;
/// Out of a vehicle in a gap (out of the subway throughout, and the fixes no farther apart than a walk covers),
/// each speed is held to what the rider's feet say at the time: a run, a walk (1.4 m/s and two sigma),
/// or, with no steps, standing (shuffling in a line). At the McDonald's in Flushing (`20261020-154951`,
/// 4:25 pm, 257 s) the bound above still left 10 mph.
const GAP_HOLD_WALK_MPS: f64 = 2.6;
const GAP_HOLD_STAND_MPS: f64 = 0.5;

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
    /// A GTFS-realtime poll's trip stops, and when it was requested (unix ms).
    Transit(Box<[trips::TripStop]>, i64),
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
            Self::Transit(..) => 10,
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
        | Pending::Car(_)
        | Pending::Transit(..) => None,
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

/// What the rider was doing over a stretch of the ride (see [`Fusion::timeline`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub start_ns: i64,
    pub end_ns: i64,
    pub doing: Doing,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Doing {
    Walking,
    /// In the subway, on a train or waiting for one, on this line if known.
    Subway {
        line: Option<String>,
    },
    /// Climbing or descending stairs (see [`crate::vertical`]).
    Stairs {
        up: bool,
    },
    /// Standing in an elevator or on an escalator, which the barometer can't tell apart.
    ElevatorOrEscalator {
        up: bool,
    },
}

/// Steps over less than this (s) aren't a walk on the timeline: shifting in a seat, a step to the doors.
const TIMELINE_WALK_S: f64 = 8.0;
/// Riding up or down isn't an elevator or escalator with a departure within this (s) before it or during it,
/// or at an average speed over this (m/s; an escalator moves the rider about 0.5 m/s).
const VERTICAL_DEPARTED_S: f64 = 60.0;
const VERTICAL_VEHICLE_MPS: f64 = 2.5;
/// The status shows a move that ended within this (s): about half its 25 s span, the lag in finding it.
const VERTICAL_RECENT_S: f64 = 15.0;
/// The subway detector off for at most this (s) is the same subway ride on the timeline.
const TIMELINE_SUBWAY_GAP_S: f64 = 60.0;

/// What the rider is on, as well as the sensors tell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Unknown,
    Still,
    Walking,
    Running,
    /// Going up or down (see [`crate::vertical`]), found [`VERTICAL_RECENT_S`] late at most.
    Stairs {
        up: bool,
    },
    ElevatorOrEscalator {
        up: bool,
    },
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
    /// The train's route, from its recent stops and the GTFS-realtime feeds (see [`crate::trips`]).
    pub route: Option<String>,
    /// The car type from what's heard (the chime, the cars' beacons), narrowed by the route,
    /// else the station's lines (see [`stations::car_type`]).
    pub car_type: Option<CarType>,
    /// How long (s) the rider has stayed within 100 m, out of the subway; the app reminds them to stop recording.
    pub staying_s: f64,
    /// The rider's car: its beacon's ID (see [`crate::car_beacons`]).
    pub car: Option<u16>,
    /// The train's current stop, until it departs.
    pub dwell: Option<Dwell>,
    /// Since the rider walked off a train (s), in the subway, until they're on another (see [`Fusion::transfer_s`]).
    pub transfer_s: Option<f64>,
}

/// The train's current stop, live: how long since it stopped, and its doors (see [`Fusion::stop_records`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Dwell {
    /// Since the stopping lurch (s); it's found a few seconds after, so this starts there, not at 0.
    pub stopped_s: f64,
    /// Since the doors opened (s), once heard, until they close.
    pub doors_open_s: Option<f64>,
    /// How long they were open (s), once their closing is heard.
    pub doors_were_open_s: Option<f64>,
}

/// The train's route is matched from its stops in the subway within this (s), at most this many,
/// the newest within [`ROUTE_STOP_S`] (the chime's 15 minutes).
const ROUTE_STOPS_S: f64 = 1_800.0;
const ROUTE_STOPS: usize = 4;
const ROUTE_STOP_S: f64 = 900.0;

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
pub const TYPE_PRESSURE: i32 = 6;

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
    /// Every car beacon reading (time, car, RSSI dBm): for when the rider steps into a car.
    car_readings: Vec<(i64, u16, i32)>,
    /// Each confirmed door chime: its time and pitch family.
    chimes: Vec<(i64, ChimeCars)>,
    /// The train's stops from its motion (see [`crate::stops`]), with the station named when each was found,
    /// and its departures.
    stop_detector: StopDetector,
    train_stops: Vec<(train_stops::Stop, Option<&'static Station>, bool)>,
    departures: Vec<i64>,
    /// The latest lurch the car settled after, shown live as the stop before it's decided (see [`Fusion::estimate`]).
    likely_stop: Option<train_stops::Stop>,
    /// Stops in the subway waiting for a door closing or a departure to confirm them.
    unconfirmed_stops: Vec<train_stops::Stop>,
    walking: WalkingDetector,
    /// Each step-rhythm window's end, and whether it had steps: for when the rider leaves or boards a train.
    step_windows: Vec<(i64, bool)>,
    cars: CarTracker,
    /// The rider's car (see [`CarTracker::current`]) and since when it has been.
    car_since: Option<(u16, i64)>,
    altimeter: crate::vertical::Altimeter,
    /// Every trip the GTFS-realtime feeds have listed.
    trips: trips::Trips,
    /// When the subway detector switched, and to what.
    subway_changes: Vec<(i64, bool)>,
    /// The registered LTE cell and when it was last reported.
    cell: Option<(i64, i64)>,
    /// Each change of the registered LTE cell (none when it's lost), for the cell at a stop.
    cells: Vec<(i64, Option<i64>)>,
    /// The station nearest Android's own latest location, if close and sure enough, and when.
    network_station: Option<(i64, &'static Station)>,
    /// Each station the cell or Android's location named, first and last when, in turn: where the rider was
    /// when (see [`Fusion::boarding`]).
    station_history: Vec<(i64, i64, &'static Station)>,
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
            car_readings: Vec::new(),
            chimes: Vec::new(),
            stop_detector: StopDetector::new(),
            train_stops: Vec::new(),
            departures: Vec::new(),
            likely_stop: None,
            unconfirmed_stops: Vec::new(),
            walking: WalkingDetector::default(),
            step_windows: Vec::new(),
            cars: CarTracker::default(),
            car_since: None,
            altimeter: Default::default(),
            trips: trips::Trips::default(),
            subway_changes: Vec::new(),
            cell: None,
            cells: Vec::new(),
            network_station: None,
            station_history: Vec::new(),
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

    /// A GTFS-realtime feed's response body (see [`crate::gtfs_rt`]), received at `t_ns` for a request at
    /// `request_unix_ms`; only each trip's stops are kept.
    pub fn on_transit(&mut self, t_ns: i64, request_unix_ms: i64, body: &[u8]) {
        if let Some(feed) = ride_session::transit::decode(body) {
            let stops = trips::trip_stops(&feed);
            self.arrive(
                t_ns,
                Pending::Transit(stops.into_boxed_slice(), request_unix_ms),
            );
        }
    }

    /// The train's route: its last few named stops in the subway since the rider last walked off a train
    /// (`boarded_after`), the newest within [`ROUTE_STOP_S`], matched to the feeds' trips.
    /// Boarding from the platform counts as a stop there (see [`Self::boarding`]): no stop is felt aboard,
    /// so otherwise the boarding station is left out (6 Av on the L, `20261023-193658`).
    fn route(&self, t: i64, boarded_after: i64) -> Option<String> {
        let ago = |at: i64| (t - at) as f64 * 1e-9;
        let records = self.stop_records();
        let boarding = self
            .boarding(t, &records)
            .and_then(|(_, b)| b)
            .and_then(|(stopped, departed)| {
                Some(trips::RiderStop {
                    stopped_ns: stopped,
                    departed_ns: Some(departed),
                    station: self.station_at(stopped)?,
                })
            })
            .filter(|b| ago(b.stopped_ns) <= ROUTE_STOPS_S);
        let stops: Vec<trips::RiderStop> = boarding
            .into_iter()
            .chain(
                self.train_stops
                    .iter()
                    .filter(|(stop, _, in_subway)| {
                        *in_subway && ago(stop.t_ns) <= ROUTE_STOPS_S && stop.t_ns > boarded_after
                    })
                    .filter_map(|(stop, station, _)| {
                        Some(trips::RiderStop {
                            stopped_ns: stop.t_ns,
                            departed_ns: self.departed_after(stop.t_ns),
                            station: (*station)?,
                        })
                    }),
            )
            .collect();
        let recent = &stops[stops.len().saturating_sub(ROUTE_STOPS)..];
        if ago(recent.last()?.stopped_ns) > ROUTE_STOP_S {
            return None;
        }
        self.trips.route(recent, t)
    }

    /// The lines the rider's train can be, from the stations it stopped at: each stop narrows them to the lines
    /// serving it too, so a 2 at 59 St stays a 2, not the A/B/C/D there. They start over from a stop
    /// none of them serve (a transfer across the platform), or after [`LINE_RESET_WINDOWS`] step windows since the
    /// stop before: walking to another platform needn't look like walking off (74 St to Jackson Hts on `184134`,
    /// 7:09 pm, by stairs), and steps within a car at a stop don't change the line anyway.
    fn riding_lines(&self, t: i64) -> Vec<String> {
        let mut lines: Vec<String> = Vec::new();
        let mut prev = i64::MIN;
        for (stop, station, in_subway) in &self.train_stops {
            let Some(station) = station.filter(|_| *in_subway && stop.t_ns <= t) else {
                continue;
            };
            let walked = self
                .step_windows
                .iter()
                .filter(|&&(at, steps)| steps && at > prev && at <= stop.t_ns)
                .count()
                >= LINE_RESET_WINDOWS;
            let both: Vec<String> = lines
                .iter()
                .filter(|l| station.lines.contains(l))
                .cloned()
                .collect();
            lines = if both.is_empty() || walked {
                station.lines.clone()
            } else {
                both
            };
            prev = stop.t_ns;
        }
        // Walking since the last stop: the next may be another line's.
        let walked_since = self
            .step_windows
            .iter()
            .filter(|&&(at, steps)| steps && at > prev && at <= t)
            .count()
            >= LINE_RESET_WINDOWS;
        if walked_since { Vec::new() } else { lines }
    }

    /// The station the radio names now: Android's own location first (Google's database), then the rider's
    /// tagged cells, each within the last 30 s.
    fn radio_station(&self, t: i64) -> Option<&'static Station> {
        let recent = |at: i64| (t - at) as f64 * 1e-9 <= 30.0;
        self.network_station
            .filter(|&(at, _)| recent(at))
            .map(|(_, s)| s)
            .or_else(|| {
                self.cell
                    .filter(|&(at, _)| recent(at))
                    .and_then(|(_, c)| stations::of_cell(c))
            })
    }

    fn note_station(&mut self, t: i64) {
        let Some(s) = self.radio_station(t) else {
            return;
        };
        match self.station_history.last_mut() {
            Some((_, last, named)) if *named == s => *last = t,
            _ => self.station_history.push((t, t, s)),
        }
    }

    /// The station the radio named at `t` (within 30 s of being named).
    fn station_at(&self, t: i64) -> Option<&'static Station> {
        let i = self
            .station_history
            .partition_point(|&(first, ..)| first <= t);
        let &(_, last, s) = self.station_history.get(i.checked_sub(1)?)?;
        ((t - last) as f64 * 1e-9 <= 30.0).then_some(s)
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
        // A move is found half its span late (see `vertical::moves`), so one ending just now is still going on.
        let vertical = self
            .vertical_moves()
            .into_iter()
            .rfind(|m| recent(m.end_ns, VERTICAL_RECENT_S));
        let mode = match (vertical, gait) {
            (Some(m), _) if m.kind == crate::vertical::Kind::Stairs => Mode::Stairs {
                up: m.change_m > 0.0,
            },
            (Some(m), _) => Mode::ElevatorOrEscalator {
                up: m.change_m > 0.0,
            },
            (None, gait) => match gait {
                Some(Gait::Running) => Mode::Running,
                Some(Gait::Walking) => Mode::Walking,
                None if self.context.in_subway => Mode::Subway { underground },
                None if estimate.initialized && estimate.speed_mps > VEHICLE_MPS => Mode::Vehicle,
                None if estimate.stopped => Mode::Still,
                None => Mode::Unknown,
            },
        };
        let cell_station = self.radio_station(t);
        // What's known of the rider's train is only from since they last walked off one,
        // and nothing while between trains: chimes and stops heard then are other trains'.
        let transfer_s = self.transfer_s(t);
        let boarded_after = self
            .stop_records()
            .into_iter()
            .rfind(|r| r.rider_left())
            .and_then(|r| r.alighted_ns)
            .unwrap_or(i64::MIN);
        let stopped_at = self
            .last_stop
            .filter(|&(at, _)| recent(at, 60.0) && at > boarded_after)
            .and_then(|(_, s)| s);
        let riding = self.riding_lines(t);
        let station = stopped_at
            .or(cell_station.filter(|_| self.context.in_subway))
            .map(|s| stations::on_lines(s, &riding))
            .cloned();
        let aboard = matches!(mode, Mode::Subway { .. }) && transfer_s.is_none();
        let cars = self
            .doors
            .as_ref()
            .and_then(DoorDetector::last_chime)
            .filter(|&(at, _)| recent(at, 900.0) && at > boarded_after)
            // Only aboard: once walking off, the last train's chime no longer says what the rider is on.
            .filter(|_| aboard)
            .map(|(_, c)| c);
        let car = self.cars.current_at(t);
        let route = self.route(t, boarded_after).filter(|_| aboard);
        // The route if the feeds give it, else the lines the train has been on, else the station's:
        // between trains, all the lines of its complex, any of which the rider may take next.
        let lines = match &route {
            Some(r) => vec![r.clone()],
            None if !riding.is_empty() && aboard => riding,
            None if transfer_s.is_some() => station
                .as_ref()
                .map(stations::complex_lines)
                .unwrap_or_default(),
            None => station
                .as_ref()
                .map(|s| s.lines.clone())
                .unwrap_or_default(),
        };
        Status {
            mode,
            route,
            car_type: stations::car_type(
                cars.map(|c| c == ChimeCars::Ntt),
                aboard && car.is_some(),
                &lines,
            ),
            station,
            at_station: stopped_at.is_some(),
            cars,
            staying_s: self
                .stay
                .map_or(0.0, |(since, _)| (t - since) as f64 * 1e-9),
            car,
            dwell: transfer_s.map_or_else(|| self.dwell(t), |_| None),
            transfer_s,
        }
    }

    /// The train's stop now, if it hasn't departed since; as in [`Self::stop_records`], without walking off.
    fn dwell(&self, t: i64) -> Option<Dwell> {
        let decided = self
            .train_stops
            .iter()
            .rev()
            .find(|(stop, _, in_subway)| *in_subway && !self.departure_fade(stop))
            .map(|(stop, ..)| stop);
        // A likely stop shows first; once decided, the decided one (the same lurch, or a stronger one after it).
        let stop = match (decided, self.likely_stop_now(t)) {
            (Some(d), Some(l)) if l.t_ns > d.t_ns => &l.clone(),
            (Some(d), _) => d,
            (None, Some(l)) => &l.clone(),
            (None, None) => return None,
        };
        let s_ns = |s: f64| (s * 1e9) as i64;
        // A hold between stations: the train stands (the speed shows it), but the dwell starts at the platform.
        let prev = self
            .train_stops
            .iter()
            .filter(|(s, _, in_subway)| *in_subway && s.t_ns < stop.t_ns && !self.departure_fade(s))
            .map(|(s, ..)| s.t_ns)
            .max();
        let closed = self
            .door_cycles
            .iter()
            .any(|c| c.close_ns > stop.t_ns + s_ns(DOORS_AFTER_STOP_S.0));
        if !closed && prev.is_some_and(|p| self.between_stations(p, stop.t_ns)) {
            return None;
        }
        let since = |from: i64| (t - from) as f64 * 1e-9;
        let stopped_s = since(stop.t_ns);
        if self.departed_after(stop.t_ns).is_some() || stopped_s > RECORD_MAX_DWELL_S {
            return None;
        }
        let cycle = self
            .door_cycles
            .iter()
            .find(|c| c.close_ns > stop.t_ns + s_ns(DOORS_AFTER_STOP_S.0));
        // A cycle is heard whole at its closing, so live the opening is known only then.
        let opened = cycle
            .map(|c| c.open_ns)
            .filter(|&o| o >= stop.t_ns && o <= stop.t_ns + s_ns(DOORS_AFTER_STOP_S.1));
        Some(Dwell {
            stopped_s,
            doors_open_s: None,
            doors_were_open_s: cycle
                .zip(opened)
                .map(|(c, o)| (c.close_ns - o) as f64 * 1e-9),
        })
    }

    /// What the rider did over the ride so far, in time order: walking, and the subway, a leg per train,
    /// which each walk off a train ends. Each leg's line is from all its stops: the feeds' trip matching the most
    /// of them (see [`trips::Trips::route`]), else the one line serving every one of them, so a stop later in a
    /// leg names the line for the whole leg. Nothing is shown out of the subway when not walking.
    pub fn timeline(&self) -> Vec<Segment> {
        let t = self.now_ns;
        let s_ns = |s: f64| (s * 1e9) as i64;
        let mut walks: Vec<(i64, i64)> = Vec::new();
        for &(at, steps) in &self.step_windows {
            if !steps {
                continue;
            }
            let start = at - s_ns(WALK_WINDOW_S);
            match walks.last_mut() {
                Some(w) if start <= w.1 + s_ns(WALK_WINDOW_S) => w.1 = at,
                _ => walks.push((start, at)),
            }
        }
        walks.retain(|&(a, b)| b - a >= s_ns(TIMELINE_WALK_S));
        let mut periods: Vec<(i64, i64)> = Vec::new();
        let mut since = None;
        for &(at, on) in &self.subway_changes {
            match (on, since) {
                // Back within a minute: the detector flickering (or an elevated stretch), the same ride.
                (true, None)
                    if periods
                        .last()
                        .is_some_and(|p| at - p.1 <= s_ns(TIMELINE_SUBWAY_GAP_S)) =>
                {
                    since = periods.pop().map(|p| p.0);
                }
                (true, None) => since = Some(at),
                (false, Some(s)) => {
                    periods.push((s, at));
                    since = None;
                }
                _ => {}
            }
        }
        if let Some(s) = since {
            periods.push((s, t));
        }
        let records = self.stop_records();
        let mut segments: Vec<Segment> = Vec::new();
        for (from, to) in periods {
            let mut cuts: Vec<i64> = records
                .iter()
                .filter(|r| r.rider_left())
                .filter_map(|r| r.alighted_ns)
                .filter(|&a| a > from && a < to)
                .collect();
            cuts.push(to);
            let mut start = from;
            for end in cuts {
                let line = self.leg_line(start, end);
                // The leg, less the walks in it.
                let mut at = start;
                for &(a, b) in walks.iter().filter(|&&(a, b)| b > start && a < end) {
                    if a > at {
                        segments.push(Segment {
                            start_ns: at,
                            end_ns: a,
                            doing: Doing::Subway { line: line.clone() },
                        });
                    }
                    at = at.max(b);
                }
                if at < end {
                    segments.push(Segment {
                        start_ns: at,
                        end_ns: end,
                        doing: Doing::Subway { line },
                    });
                }
                start = end;
            }
        }
        segments.extend(walks.into_iter().map(|(a, b)| Segment {
            start_ns: a,
            end_ns: b,
            doing: Doing::Walking,
        }));
        // Going up or down replaces what was there then (walking, mostly).
        for m in self.vertical_moves() {
            let mut cut = Vec::with_capacity(segments.len() + 2);
            for s in segments {
                if s.end_ns <= m.start_ns || s.start_ns >= m.end_ns {
                    cut.push(s);
                    continue;
                }
                if s.start_ns < m.start_ns {
                    cut.push(Segment {
                        end_ns: m.start_ns,
                        ..s.clone()
                    });
                }
                if s.end_ns > m.end_ns {
                    cut.push(Segment {
                        start_ns: m.end_ns,
                        ..s
                    });
                }
            }
            let up = m.change_m > 0.0;
            cut.push(Segment {
                start_ns: m.start_ns,
                end_ns: m.end_ns,
                doing: match m.kind {
                    crate::vertical::Kind::Stairs => Doing::Stairs { up },
                    crate::vertical::Kind::Riding => Doing::ElevatorOrEscalator { up },
                },
            });
            segments = cut;
        }
        segments.sort_by_key(|s| s.start_ns);
        // Pieces left by a walk too short to show, or a flicker, join up.
        let mut joined: Vec<Segment> = Vec::new();
        for s in segments {
            match joined.last_mut() {
                Some(last)
                    if last.doing == s.doing
                        && s.start_ns - last.end_ns <= s_ns(TIMELINE_SUBWAY_GAP_S) =>
                {
                    last.end_ns = last.end_ns.max(s.end_ns);
                }
                _ => joined.push(s),
            }
        }
        joined
    }

    /// The line of a subway leg from its stops (see [`Self::timeline`]).
    fn leg_line(&self, from: i64, to: i64) -> Option<String> {
        let stops: Vec<trips::RiderStop> = self
            .train_stops
            .iter()
            .filter(|(stop, _, in_subway)| *in_subway && stop.t_ns >= from && stop.t_ns <= to)
            .filter_map(|(stop, station, _)| {
                Some(trips::RiderStop {
                    stopped_ns: stop.t_ns,
                    departed_ns: self.departed_after(stop.t_ns),
                    station: (*station)?,
                })
            })
            .collect();
        if stops.is_empty() {
            return None;
        }
        if let Some(route) = self.trips.route(&stops, to) {
            return Some(route);
        }
        let mut lines = stops[0].station.lines.clone();
        for s in &stops[1..] {
            lines.retain(|l| s.station.lines.contains(l));
        }
        match &lines[..] {
            [line] => Some(line.clone()),
            _ => None,
        }
    }

    /// Since the rider walked off their train (s), while still in the subway: a transfer, or leaving the station.
    /// It ends on boarding another train: a departure with no steps around it (the lurch detectors fire on walking
    /// too), or a car beacon heard as the rider's for [`TRANSFER_CAR_S`], other than the train's they left
    /// (departures are missed on the L). Trains at the platform are heard steadily too, but leave sooner.
    /// Stops found meanwhile don't end it: walking through a station makes lurches.
    fn transfer_s(&self, t: i64) -> Option<f64> {
        if !self.context.in_subway {
            return None;
        }
        let records = self.stop_records();
        let (alighted, boarded) = self.boarding(t, &records)?;
        let since_s = (t - alighted) as f64 * 1e-9;
        (boarded.is_none() && since_s <= TRANSFER_MAX_S).then_some(since_s)
    }

    /// When the rider last walked off a train, and how they boarded the next, if they have:
    /// the train's stop at the platform (stopped, departed), as near as known.
    /// The first departure since with no steps around it, the train leaving with the rider,
    /// or else the new car heard as theirs for [`TRANSFER_CAR_S`], from when it was first: on the L, whose
    /// departures are missed, it's heard just as the rider boards, from the platform.
    fn boarding(&self, t: i64, records: &[StopRecord]) -> Option<(i64, Option<(i64, i64)>)> {
        let left = records.iter().rfind(|r| r.rider_left())?;
        let alighted = left.alighted_ns?;
        let s_ns = |s: f64| (s * 1e9) as i64;
        // The departures found, and those the stop records give (a stop stepped out at, then ridden on from).
        let departed = records.iter().filter_map(|r| r.departed_ns);
        let departure = self
            .departures
            .iter()
            .copied()
            .chain(departed)
            .filter(|&d| {
                d > alighted
                    && !self.step_windows.iter().any(|&(at, steps)| {
                        steps
                            && at > d - s_ns(TRANSFER_STILL_S.0)
                            && at < d + s_ns(TRANSFER_STILL_S.1)
                    })
            })
            .min();
        let new_car = self
            .car_since
            .filter(|&(c, since)| {
                self.cars.current_at(t) == Some(c)
                    && left.car_before != Some(c)
                    && since > alighted
                    && (t - since) as f64 * 1e-9 >= TRANSFER_CAR_S
            })
            .map(|(_, since)| since);
        let platform = s_ns(BOARDING_STOP_S);
        let boarded = match (departure, new_car) {
            (Some(d), Some(c)) if c < d => Some((c, c + platform)),
            (Some(d), _) => Some((d - platform, d)),
            (None, Some(c)) => Some((c, c + platform)),
            (None, None) => None,
        };
        Some((alighted, boarded))
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

    /// The door chimes a closing confirmed so far, with their pitch family.
    pub fn chimes(&self) -> &[(i64, ChimeCars)] {
        &self.chimes
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
        if sensor_type == TYPE_PRESSURE {
            if let Some(&hpa) = values.first() {
                self.on_pressure(t_ns, f64::from(hpa));
            }
            return;
        }
        match imu_sample(sensor_type, values) {
            Some(ImuSample::Accel(a)) => self.on_accel(t_ns, a),
            Some(ImuSample::Gyro(w)) => self.on_gyro(t_ns, w),
            Some(ImuSample::Mag(b)) => self.on_magnetometer(t_ns, b),
            None => {}
        }
    }

    /// Going up and down so far: stairs, and elevators or escalators (see [`crate::vertical`]).
    /// Riding up or down with a train departing around it, or at a vehicle's speed, was the train (boarding is
    /// walking, then its tunnel pressure waves pass for a ride): on `214427` and `182135`, aboard the 2.
    pub fn vertical_moves(&self) -> Vec<crate::vertical::Move> {
        let s_ns = |s: f64| (s * 1e9) as i64;
        // Smoothed only if needed: over a whole ride, that's smoothing it all again.
        let smoothed = std::cell::OnceCell::new();
        crate::vertical::moves(self.altimeter.heights(), &self.step_windows, WALK_WINDOW_S)
            .into_iter()
            .filter(|m| {
                if m.kind == crate::vertical::Kind::Stairs {
                    return true;
                }
                let departed = self
                    .departures
                    .iter()
                    .any(|&d| d >= m.start_ns - s_ns(VERTICAL_DEPARTED_S) && d <= m.end_ns);
                if departed {
                    return false;
                }
                let speeds: Vec<f64> = smoothed
                    .get_or_init(|| self.smoothed())
                    .iter()
                    .filter(|s| {
                        s.elapsed_realtime_ns >= m.start_ns && s.elapsed_realtime_ns <= m.end_ns
                    })
                    .map(|s| s.speed_mps)
                    .collect();
                let fast = !speeds.is_empty()
                    && speeds.iter().sum::<f64>() / speeds.len() as f64 > VERTICAL_VEHICLE_MPS;
                !fast
            })
            .collect()
    }

    /// A barometer reading (hPa), for going up and down (see [`crate::vertical`]).
    pub fn on_pressure(&mut self, t_ns: i64, hpa: f64) {
        self.altimeter.on_pressure(t_ns, hpa);
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
        if let Pending::Transit(stops, request_unix_ms) = input {
            self.trips.on_poll(t_ns, request_unix_ms, stops.into_vec());
            return;
        }
        if let Pending::Cell(c) = input {
            self.cell = c.map(|c| (t_ns, c));
            if self.cells.last().is_none_or(|&(_, last)| last != c) {
                self.cells.push((t_ns, c));
            }
            self.note_station(t_ns);
            return;
        }
        if let Pending::NetworkFix(lat, lon, accuracy) = input {
            self.network_station = stations::nearest(lat, lon)
                .filter(|&(_, d)| accuracy <= NETWORK_FIX_M && d <= NETWORK_STATION_M.max(accuracy))
                .map(|(s, _)| (t_ns, s));
            self.note_station(t_ns);
            return;
        }
        if let Pending::Car(r) = input {
            self.cars.on_reading(t_ns, &r);
            let current = self.cars.current();
            if current != self.car_since.map(|(c, _)| c) {
                self.car_since = current.map(|c| (c, t_ns));
            }
            self.car_readings.push((t_ns, r.car, r.rssi));
            if self.cars.steady(t_ns, r.car) {
                self.estimator.on_car_accel(t_ns, r.car, r.accel);
                self.buffer(t_ns, Input::Car(r.car, r.accel));
            }
            return;
        }
        if let (Pending::Audio(a), Some(doors)) = (&input, &mut self.doors) {
            let cycles = doors.on_audio(a.first_frame, &a.samples, &a.timestamps);
            if let Some(c) = doors.last_chime()
                && self.chimes.last() != Some(&c)
            {
                self.chimes.push(c);
            }
            for cycle in cycles {
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
            && let Some(walking) = self
                .walking
                .on_accel(t_ns, (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt())
        {
            let gait = self.walking.last_gait().and_then(|(_, g)| g);
            self.step_windows.push((t_ns, gait.is_some()));
            self.confirm_by_walking_off(t_ns);
            if walking {
                let running = gait == Some(Gait::Running);
                self.estimator.on_walking(t_ns, running);
                self.buffer(t_ns, Input::Walking(running));
            }
        }
        let stop_events = match &input {
            Pending::Accel(a) => self.stop_detector.on_accel(t_ns, *a),
            Pending::Gyro(w) => self.stop_detector.on_gyro(t_ns, *w),
            _ => Vec::new(),
        };
        for event in stop_events {
            match event {
                train_stops::Event::Stopped(stop) => self.on_train_stop(t_ns, stop),
                train_stops::Event::Likely(stop) => {
                    if self.context.in_subway && !self.departure_fade(&stop) {
                        self.likely_stop = Some(stop);
                    }
                }
                train_stops::Event::Departed(at) => {
                    if self.context.in_subway
                        && let Some(stop) =
                            missed_stop(&self.step_windows, &self.train_stops, &self.departures, at)
                    {
                        let station = self.station_now(stop.t_ns);
                        self.train_stops.push((stop, station, true));
                        let from = stop.standing_ns();
                        self.hold_standing(t_ns, from, from + (STOP_STANDING_S * 1e9) as i64);
                    }
                    self.departures.push(at);
                    self.confirm_stops(t_ns, at);
                }
            }
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
        self.confirm_stops(t_ns, cycle.close_ns);
        let station = self.station_now(t_ns);
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
        self.hold_standing(
            t_ns,
            cycle.close_ns - (DWELL_SURE_S * 1e9) as i64,
            cycle.close_ns,
        );
    }

    /// The train's start surges found so far (see [`crate::stops`]).
    pub fn departures(&self) -> &[i64] {
        &self.departures
    }

    /// Each step-rhythm window's end, and whether it had steps.
    pub fn step_windows(&self) -> &[(i64, bool)] {
        &self.step_windows
    }

    /// Each stop of the train found so far, with its doors and departure (see [`StopRecord`]).
    pub fn stop_records(&self) -> Vec<StopRecord> {
        let mut records = stop_records(
            &self.train_stops,
            &self.door_cycles,
            &self.departures,
            &self.step_windows,
        );
        let span = (CAR_SPAN_S * 1e9) as i64;
        for r in &mut records {
            if let (Some(b), Some(d), false) = (r.boarded_ns, r.departed_ns, r.stepped_out()) {
                r.boarded_ns = Some(stepped_in(&self.car_readings, b, d).unwrap_or(b));
            }
            if let (Some(a), Some(b)) = (r.alighted_ns, r.boarded_ns) {
                r.car_before = strongest_car(&self.car_readings, a - span, a);
                r.car_after = strongest_car(&self.car_readings, b, b + span);
            }
            if let Some(d) = r.departed_ns {
                r.train_change = trains_differ(&self.car_readings, r.stopped_ns, d);
            }
        }
        for k in 1..records.len() {
            let (before, after) = records.split_at_mut(k);
            let (prev, r) = (&before[k - 1], &mut after[0]);
            r.between_stations = r.in_subway
                && prev.in_subway
                && !prev.rider_left()
                && r.doors_closed_ns.is_none()
                && self.between_stations(prev.stopped_ns, r.stopped_ns);
        }
        records
    }

    /// The registered LTE cell at `t_ns`, if any.
    fn cell_at(&self, t_ns: i64) -> Option<i64> {
        let i = self.cells.partition_point(|&(t, _)| t <= t_ns);
        i.checked_sub(1).and_then(|i| self.cells[i].1)
    }

    /// Whether a stop at `t_ns` with no closing heard, after one at `prev_ns`, is a hold between stations:
    /// the registered cell the same at both, so the train hasn't reached the next station's.
    /// Against the "train stopped" taps over 11 subway rides (`crates/cli/examples/holds.rs`),
    /// 0 of 87 tapped stops had the same cell as the stop before and no closing, and 9 of 47 untapped stops did
    /// (the hold before Lorimer St on `20261023-193658`, 8:02 pm, whose lurch started the live dwell 48 s early).
    /// The same cell alone was 1 of the 87 tapped and 13 of the 47 untapped; a station named by the cell,
    /// 79 tapped and 39 untapped, tells nothing.
    fn between_stations(&self, prev_ns: i64, t_ns: i64) -> bool {
        self.cell_at(t_ns)
            .is_some_and(|c| self.cell_at(prev_ns) == Some(c))
    }

    /// Whether a found stop is the departure's pull fading (see [`FADE_AFTER_CLOSE_S`]).
    fn departure_fade(&self, stop: &train_stops::Stop) -> bool {
        departure_fade(stop, &self.door_cycles)
    }

    /// The train's first departure after the stop whose lurch was at `stopped_ns`, as [`stop_records`] finds it:
    /// a start surge within [`DEPART_AFTER_STOP_S`] of the lurch is the stop's own settling, not a departure.
    /// On the L, every stop's settling made one about 1 s after it (`20261023-193658`), which ended the live
    /// hold at once: standing at Bedford Av, the speed ran to the 55 mph cap.
    fn departed_after(&self, stopped_ns: i64) -> Option<i64> {
        let after = stopped_ns + (DEPART_AFTER_STOP_S * 1e9) as i64;
        self.departures.iter().copied().find(|&d| d > after)
    }

    /// The station Android's own location or the registered cell names as of `t_ns`, if any.
    fn station_now(&self, t_ns: i64) -> Option<&'static Station> {
        let network = self
            .network_station
            .filter(|&(at, _)| (t_ns - at) as f64 * 1e-9 <= 30.0)
            .map(|(_, s)| s);
        network
            .or_else(|| self.cell.and_then(|(_, c)| stations::of_cell(c)))
            .map(|s| stations::on_lines(s, &self.riding_lines(t_ns)))
    }

    /// The train's stopping lurch, found at `t_ns` ([`train_stops::SEP_S`] and a bit after it): in the subway,
    /// the train stood from the braking's collapse through the seconds that showed it still.
    fn on_train_stop(&mut self, t_ns: i64, stop: train_stops::Stop) {
        let in_subway = self.context.in_subway;
        let station = self
            .network_station
            .filter(|&(at, _)| (stop.t_ns - at) as f64 * 1e-9 <= 60.0)
            .map(|(_, s)| s)
            .or_else(|| self.cell.and_then(|(_, c)| stations::of_cell(c)))
            .map(|s| stations::on_lines(s, &self.riding_lines(stop.t_ns)));
        self.train_stops.push((stop, station, in_subway));
        if USE_STOPPING_LURCH && in_subway && !self.departure_fade(&stop) {
            self.unconfirmed_stops.push(stop);
            // What came while the stop was being decided.
            let since: Vec<i64> = self
                .door_cycles
                .iter()
                .map(|c| c.close_ns)
                .chain(self.departures.iter().copied())
                .filter(|&x| x > stop.t_ns)
                .collect();
            for x in since {
                self.confirm_stops(t_ns, x);
            }
        }
    }

    /// A door closing or a departure at `at`, found at `t_ns`, confirms the unconfirmed stops it follows:
    /// a train that stopped either opens its doors or pulls away. A lurch with neither after it, found at speed on
    /// the elevated 7 (`154951`, 3:55 and 4:10 pm), was braking that eased; with GNSS cut 30 s in 90 they made
    /// the outages worse (3.41 to 4.49 km/h rms).
    fn confirm_stops(&mut self, t_ns: i64, at: i64) {
        let (confirmed, waiting): (Vec<_>, Vec<_>) =
            self.unconfirmed_stops.drain(..).partition(|s| {
                let dt = (at - s.t_ns) as f64 * 1e-9;
                dt > DEPART_AFTER_STOP_S && dt < RECORD_MAX_DWELL_S
            });
        self.unconfirmed_stops = waiting
            .into_iter()
            .filter(|s| (t_ns - s.t_ns) as f64 * 1e-9 < RECORD_MAX_DWELL_S)
            .collect();
        for stop in confirmed {
            let from = stop.standing_ns();
            self.hold_standing(t_ns, from, from + (STOP_STANDING_S * 1e9) as i64);
        }
    }

    /// Stops waiting for confirmation that the rider walked off from (see [`ALIGHT_WINDOWS`]): a train that
    /// stopped, at the end of a ride or a transfer, often with no door cycle heard and, the rider gone,
    /// no departure (`20261021-135509`, 2:20 pm; `20261021-182135`, 6:44 pm).
    fn confirm_by_walking_off(&mut self, t_ns: i64) {
        let window_ns = (WALK_WINDOW_S * 1e9) as i64;
        let walked_off: Vec<train_stops::Stop> = self
            .unconfirmed_stops
            .iter()
            .copied()
            .filter(|s| {
                let after: Vec<bool> = self
                    .step_windows
                    .iter()
                    .filter(|&&(t, _)| t - window_ns >= s.t_ns)
                    .take(ALIGHT_OF)
                    .map(|&(_, steps)| steps)
                    .collect();
                after.len() == ALIGHT_OF && after.iter().filter(|&&x| x).count() >= ALIGHT_WINDOWS
            })
            .collect();
        for stop in walked_off {
            self.unconfirmed_stops.retain(|s| s.t_ns != stop.t_ns);
            let from = stop.standing_ns();
            self.hold_standing(t_ns, from, from + (STOP_STANDING_S * 1e9) as i64);
        }
    }

    /// The train stood still from `from` to `to`, found at `t_ns`: the speed is zero there, measured.
    fn hold_standing(&mut self, t_ns: i64, from: i64, to: i64) {
        // Since the speed was last measured, the IMU alone carried it, and its error there is mostly a constant
        // leak (gravity through a small tilt, or bias), which grows linearly: take it out of the forward samples
        // as a ramp from nothing then to all of it at the closing, and all of it after.
        if let Some(anchor) = self.estimator.last_measured_ns().filter(|&a| a < to)
            && let Some(error) = self
                .forward
                .iter()
                .rev()
                .find(|(t, _)| *t <= to)
                .map(|(_, e)| e.speed_mps)
        {
            let ramp = |t: i64| {
                if t <= anchor {
                    0.0
                } else if t >= to {
                    error
                } else {
                    error * (t - anchor) as f64 / (to - anchor) as f64
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
        self.estimator.on_stopped_at(to, t_ns);
        self.buffer(to, Input::StandingSince(from));
        let mut t = from;
        while t <= to {
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
            | Pending::Car(_)
            | Pending::Transit(..) => {}
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
    /// The live estimate. While a likely stop holds (see [`Self::likely_stop_now`]) it shows standing still,
    /// without changing what fusion computes: the stop is only measured once a closing, departure, or walking off
    /// confirms it, as eased braking at speed also settles (`154951`, the elevated 7).
    pub fn estimate(&self, now_ns: i64) -> Estimate {
        let mut e = self.estimate_unheld(now_ns);
        // Between trains the rider is on foot: standing on a platform, or walking at most at a run.
        // The IMU alone ran to the 55 mph cap standing on the platforms at 6 Av and 14 St (`20261023-193658`).
        if self.transfer_s(now_ns).is_some() {
            let walking = self.step_windows.last().is_some_and(|&(_, steps)| steps);
            e.speed_mps = if walking {
                e.speed_mps.min(BETWEEN_TRAINS_MAX_MPS)
            } else {
                0.0
            };
            e.accel_mps2 = 0.0;
            e.stopped = !walking;
            return e;
        }
        if self.likely_stop_now(now_ns).is_some() {
            e.speed_mps = 0.0;
            e.accel_mps2 = 0.0;
            e.stopped = true;
        }
        e
    }

    /// The likely stop the train is standing at now, if any: not departed since, no GNSS speed since says it's
    /// moving, the rider not walking now, and under [`RECORD_MAX_DWELL_S`] ago. A later lurch replaces it.
    fn likely_stop_now(&self, now_ns: i64) -> Option<train_stops::Stop> {
        let stop = self.likely_stop?;
        let moving_gnss = self
            .estimator
            .last_gnss_speed()
            .is_some_and(|(t, v)| t > stop.t_ns && v > LIKELY_STOP_GNSS_MPS);
        let departed = self.departed_after(stop.t_ns).is_some();
        let since_s = (now_ns - stop.t_ns) as f64 * 1e-9;
        // Once the decision is due, it must have kept this lurch (on `135509` and `215922`, 3 of 34 weren't).
        let dropped = since_s > LIKELY_DECIDED_BY_S
            && !self.train_stops.iter().any(|(s, ..)| s.t_ns == stop.t_ns);
        let old = since_s > RECORD_MAX_DWELL_S;
        // Walking since the stop: the rider is moving (off the train, or along it), so the walking pace shows.
        let walking = self
            .step_windows
            .last()
            .is_some_and(|&(t, steps)| steps && t > stop.t_ns);
        (self.context.in_subway && !moving_gnss && !departed && !dropped && !old && !walking)
            .then_some(stop)
    }

    fn estimate_unheld(&self, now_ns: i64) -> Estimate {
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
        let steps: Vec<(i64, bool)> = inputs
            .iter()
            .filter_map(|&&(t, ref input)| match *input {
                Input::Walking(running) => Some((t, running)),
                _ => None,
            })
            .collect();
        bound_gaps(&mut out, &fixes, &steps, &self.subway_changes);
        out
    }
}

/// Scales down the smoothed speeds across each gap between good fixes to the distance the fixes allow
/// (see [`GAP_FIX_M`]). Only a bound: a path longer than the straight line is fine up to it.
/// Out of a vehicle, each speed in the gap is also held to the rider's feet (see [`GAP_HOLD_WALK_MPS`]).
/// `steps`: when steps were detected, and whether running; `subway`: when the subway detector switched, and to what.
fn bound_gaps(
    out: &mut [SmoothedSpeed],
    fixes: &[&Fix],
    steps: &[(i64, bool)],
    subway: &[(i64, bool)],
) {
    let in_subway_at = |t: i64| {
        subway
            .iter()
            .take_while(|&&(at, _)| at <= t)
            .last()
            .is_some_and(|&(_, on)| on)
    };
    // The gait during the step window before a detection (running wins over walking).
    let feet_at = |t: i64| {
        let from = steps.partition_point(|&(at, _)| at < t);
        let to = steps.partition_point(|&(at, _)| at <= t + GAP_STEP_WINDOW_NS);
        steps[from..to].iter().map(|&(_, running)| running).max()
    };
    for pair in fixes.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let (ta, tb) = (a.elapsed_realtime_ns, b.elapsed_realtime_ns);
        if tb - ta < GAP_MIN_NS {
            continue;
        }
        let gap_s = (tb - ta) as f64 * 1e-9;
        let (mut walk_s, mut run_s) = (0.0, 0.0);
        let mut last = ta;
        for &(t, running) in steps.iter().filter(|&&(t, _)| t > ta && t <= tb) {
            let covered = (t - last.max(t - GAP_STEP_WINDOW_NS)).max(0) as f64 * 1e-9;
            last = t;
            if running {
                run_s += covered;
            } else {
                walk_s += covered;
            }
        }
        let straight = stations::meters((a.lat, a.lon), (b.lat, b.lon));
        let allowed = GAP_PATH_RATIO * straight
            + a.accuracy.unwrap_or(0.0)
            + b.accuracy.unwrap_or(0.0)
            + GAP_WANDER_MPS * walk_s
            + GAP_RUN_MPS * run_s;
        let from = out.partition_point(|s| s.elapsed_realtime_ns <= ta);
        let to = out.partition_point(|s| s.elapsed_realtime_ns < tb);
        if to <= from {
            continue;
        }
        let on_foot = straight / gap_s < VEHICLE_MPS
            && !in_subway_at(ta)
            && !subway.iter().any(|&(t, on)| on && t > ta && t < tb);
        if on_foot {
            for s in &mut out[from..to] {
                let hold = match feet_at(s.elapsed_realtime_ns) {
                    Some(true) => GAP_RUN_MPS,
                    Some(false) => GAP_HOLD_WALK_MPS,
                    None => GAP_HOLD_STAND_MPS,
                };
                if s.speed_mps > hold {
                    s.accel_mps2 *= hold / s.speed_mps;
                    s.speed_mps = hold;
                }
            }
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

/// The stops' records from what fusion found: stops (with the station named then, and whether in the subway),
/// door cycles, start surges, and step-rhythm windows (see [`StopRecord`]).
fn stop_records(
    train_stops: &[(train_stops::Stop, Option<&'static Station>, bool)],
    door_cycles: &[DoorCycle],
    surges: &[i64],
    step_windows: &[(i64, bool)],
) -> Vec<StopRecord> {
    let s_ns = |s: f64| (s * 1e9) as i64;
    type Found = (train_stops::Stop, Option<&'static Station>, bool);
    let (fades, mut stops): (Vec<&Found>, Vec<&Found>) = train_stops
        .iter()
        .partition(|(stop, ..)| departure_fade(stop, door_cycles));
    // A missed stop is found at the departure after it, so it can come after later stops.
    stops.sort_by_key(|(stop, ..)| stop.t_ns);
    // A departure's fading pull began as it pulled away, when its start surge was missed (Grand Central,
    // `20261020-151449`, 3:44 pm).
    let mut departures = surges.to_vec();
    departures.extend(fades.iter().map(|(stop, ..)| stop.braking_since_ns));
    departures.sort_unstable();
    let window_ns = s_ns(WALK_WINDOW_S);
    let mut out = Vec::new();
    for (k, &&(stop, station, in_subway)) in stops.iter().enumerate() {
        let next = stops.get(k + 1).map_or(i64::MAX, |(s, ..)| s.t_ns);
        let limit = next.min(stop.t_ns + s_ns(RECORD_MAX_DWELL_S));
        let walked_before = |at: i64, within: f64| {
            step_windows
                .iter()
                .filter(|&&(t, steps)| steps && t > at - s_ns(within) && t <= at)
                .count()
        };
        let on_platform_at = |at: i64| {
            walked_before(at, PLATFORM_BEFORE_S) >= PLATFORM_WINDOWS
                && walked_before(at, PLATFORM_LAST_S) > 0
        };
        let on_platform = on_platform_at(stop.t_ns);
        let after: Vec<(i64, bool)> = step_windows
            .iter()
            .copied()
            .filter(|&(t, _)| t - window_ns >= stop.t_ns && t <= limit)
            .collect();
        let first_departure = |from: i64| {
            departures
                .iter()
                .copied()
                .find(|&d| d > from && d > stop.t_ns + s_ns(DEPART_AFTER_STOP_S) && d < limit)
        };
        // Walking off: enough steps in a row, starting before the train's departure.
        let alighted = (!on_platform)
            .then(|| {
                after.windows(ALIGHT_OF).find_map(|w| {
                    (w.iter().filter(|(_, steps)| *steps).count() >= ALIGHT_WINDOWS)
                        .then(|| w.iter().find(|(_, steps)| *steps).unwrap().0 - window_ns)
                })
            })
            .flatten();
        let cycle = door_cycles.iter().find(|c| {
            c.close_ns > stop.t_ns + s_ns(DOORS_AFTER_STOP_S.0)
                && c.close_ns < limit
                && alighted.is_none_or(|_| c.close_ns < stop.t_ns + s_ns(HEARD_FROM_PLATFORM_S))
        });
        let ridden_departure = first_departure(cycle.map_or(stop.t_ns, |c| c.close_ns));
        // Still aboard: a short walk, then standing until the next stop was felt (the rider not walking on the
        // platform before it), with no departure since the walk: the train carried the rider on, and the walk was
        // along the car (90 St, `20261020-184134`, 7:06 pm).
        let carried_on = |a: i64| {
            stops.get(k + 1).is_some_and(|(next, ..)| {
                let walked_on = step_windows
                    .iter()
                    .filter(|&&(t, steps)| steps && t > a + s_ns(CARRIED_WALK_S) && t <= next.t_ns)
                    .count();
                next.t_ns > a + s_ns(CARRIED_WALK_S)
                    && walked_on <= CARRIED_STEP_WINDOWS
                    && !on_platform_at(next.t_ns)
                    && !departures.iter().any(|&d| d > a && d < next.t_ns)
            })
        };
        let alighted = alighted
            .filter(|&a| ridden_departure.is_none_or(|d| a < d))
            .filter(|&a| !carried_on(a));
        // Boarding: the last steps before a departure, after walking on the platform or off a train.
        let (boarded, departed) = if on_platform || alighted.is_some() {
            let from = alighted.unwrap_or(stop.t_ns);
            let walked: Vec<i64> = after
                .iter()
                .filter(|&&(t, steps)| steps && t > from)
                .map(|&(t, _)| t)
                .collect();
            match walked.last() {
                Some(&last) => match first_departure(last) {
                    Some(d) => (Some(last), Some(d)),
                    None => (None, None),
                },
                None if on_platform => (None, ridden_departure),
                None => (None, None),
            }
        } else {
            (None, ridden_departure)
        };
        let doors_open = cycle.and_then(|c| {
            let heard = (c.open_ns - stop.t_ns) as f64 * 1e-9;
            (heard >= DOORS_AFTER_STOP_S.0 && heard <= DOORS_AFTER_STOP_S.1)
                .then_some((c.open_ns, true))
        });
        out.push(StopRecord {
            stopped_ns: stop.t_ns,
            score: stop.score,
            in_subway,
            station,
            doors_open_ns: doors_open.map(|d| d.0),
            doors_open_heard: doors_open.is_some_and(|d| d.1),
            doors_closed_ns: cycle.map(|c| c.close_ns),
            departed_ns: departed,
            on_platform,
            alighted_ns: alighted,
            boarded_ns: boarded,
            car_before: None,
            car_after: None,
            train_change: false,
            between_stations: false,
        });
    }
    out
}

/// The stop before the departure at `departure` that the lurch detector missed, found from the rider walking off
/// (see [`MISSED_STOP_WALK_WINDOWS`]), if any.
fn missed_stop(
    step_windows: &[(i64, bool)],
    train_stops: &[(train_stops::Stop, Option<&'static Station>, bool)],
    departures: &[i64],
    departure: i64,
) -> Option<train_stops::Stop> {
    let s_ns = |s: f64| (s * 1e9) as i64;
    let window_ns = s_ns(WALK_WINDOW_S);
    let since = departure - s_ns(RECORD_MAX_DWELL_S);
    let windows: Vec<(i64, bool)> = step_windows
        .iter()
        .copied()
        .filter(|&(t, _)| t > since && t <= departure)
        .collect();
    let walk_start = windows.windows(ALIGHT_OF).find_map(|w| {
        (w.iter().filter(|(_, steps)| *steps).count() >= ALIGHT_WINDOWS)
            .then(|| w.iter().find(|(_, steps)| *steps).unwrap().0 - window_ns)
    })?;
    let walked = windows
        .iter()
        .filter(|&&(t, steps)| steps && t > walk_start)
        .count();
    let stop_ns = walk_start - s_ns(ALIGHT_AFTER_STOP_S);
    // Riding up to the walk: a departure before it, with no stop since (not walking into the station to board).
    let rode_from = departures
        .iter()
        .copied()
        .filter(|&d| d > walk_start - s_ns(RECORD_MAX_DWELL_S) && d < walk_start)
        .max()?;
    let found = train_stops.iter().any(|(s, ..)| {
        s.t_ns > rode_from.min(walk_start - s_ns(MISSED_STOP_LOOKBACK_S)) && s.t_ns < departure
    });
    let departed_between = departures.iter().any(|&d| d > stop_ns && d < departure);
    (walked >= MISSED_STOP_WALK_WINDOWS && !found && !departed_between).then_some(
        train_stops::Stop {
            t_ns: stop_ns,
            score: MISSED_STOP_SCORE,
            braking_since_ns: stop_ns,
        },
    )
}

/// The rider's car is the strongest car beacon over this long (s) before stepping off or after boarding.
const CAR_SPAN_S: f64 = 20.0;

/// How long (s) before a stop and after its departure the cars heard name the train.
const TRAIN_SPAN_S: f64 = 60.0;

/// The car beacons heard at least 3 times between `from` and `until`.
fn cars_heard(readings: &[(i64, u16, i32)], from: i64, until: i64) -> HashSet<u16> {
    let mut counts: HashMap<u16, usize> = HashMap::new();
    for &(_, car, _) in readings.iter().filter(|&&(t, ..)| t >= from && t <= until) {
        *counts.entry(car).or_default() += 1;
    }
    counts
        .into_iter()
        .filter(|&(_, n)| n >= 3)
        .map(|(car, _)| car)
        .collect()
}

/// Whether the train before the stop at `stopped` and after its departure at `departed` are different ones:
/// cars heard on both sides, none of them the same. A beacon is heard 2 to 3 cars away, so the rider's own train
/// always shares a car across a stop it stays on.
fn trains_differ(readings: &[(i64, u16, i32)], stopped: i64, departed: i64) -> bool {
    let span = (TRAIN_SPAN_S * 1e9) as i64;
    let before = cars_heard(readings, stopped - span, stopped);
    let after = cars_heard(readings, departed, departed + span);
    !before.is_empty() && !after.is_empty() && before.is_disjoint(&after)
}

/// The car beacon with the highest mean RSSI between `from` and `until`, heard at least 3 times.
fn strongest_car(readings: &[(i64, u16, i32)], from: i64, until: i64) -> Option<u16> {
    let mut sums: HashMap<u16, (f64, usize)> = HashMap::new();
    for &(_, car, rssi) in readings.iter().filter(|&&(t, ..)| t >= from && t <= until) {
        let e = sums.entry(car).or_default();
        e.0 += f64::from(rssi);
        e.1 += 1;
    }
    sums.into_iter()
        .filter(|(_, (_, n))| *n >= 3)
        .max_by(|a, b| (a.1.0 / a.1.1 as f64).total_cmp(&(b.1.0 / b.1.1 as f64)))
        .map(|(car, _)| car)
}

/// A car's beacon rises this much (dB) over its first readings once the rider is inside the car.
const STEPPED_IN_DB: f64 = 8.0;
const STEPPED_IN_BASE: usize = 3;

/// When the rider stepped into a car, if a car beacon first heard after `from` (the last walking found)
/// rises [`STEPPED_IN_DB`] over its first readings before `until` (the departure).
/// A step onto a train is often too short for a step-rhythm window, so the last window can be the walk
/// along the platform: Times Sq, 3 to 7 (`20261020-151449`), 3:38:57 pm, and the 7's car `a14e`
/// rose from -86 to -74 dBm as the rider stepped in, 3:40:34 against the tap at 3:40:30
/// (`python/experiments/boarding.py`; no change at the other 4 tapped boardings).
fn stepped_in(readings: &[(i64, u16, i32)], from: i64, until: i64) -> Option<i64> {
    let mut first: HashMap<u16, (i64, Vec<i32>)> = HashMap::new();
    readings
        .iter()
        .filter(|&&(t, ..)| t > from - 60_000_000_000 && t <= until)
        .find_map(|&(t, car, rssi)| {
            let (t0, base) = first.entry(car).or_insert((t, Vec::new()));
            if *t0 < from {
                return None;
            }
            if base.len() < STEPPED_IN_BASE {
                base.push(rssi);
                return None;
            }
            let mean = base.iter().map(|&r| f64::from(r)).sum::<f64>() / base.len() as f64;
            (f64::from(rssi) >= mean + STEPPED_IN_DB).then_some(t)
        })
}

/// Whether a found stop is the departure's pull fading (see [`FADE_AFTER_CLOSE_S`]).
fn departure_fade(stop: &train_stops::Stop, door_cycles: &[DoorCycle]) -> bool {
    let braking_s = (stop.t_ns - stop.braking_since_ns) as f64 * 1e-9;
    braking_s >= FADE_MIN_S
        && door_cycles.iter().any(|c| {
            let after = (stop.braking_since_ns - c.close_ns) as f64 * 1e-9;
            after >= FADE_AFTER_CLOSE_S.0 && after <= FADE_AFTER_CLOSE_S.1
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_from_the_feeds() {
        use prost::Message;
        use ride_session::transit_realtime::{
            FeedEntity, FeedHeader, FeedMessage, TripDescriptor, TripUpdate,
            trip_update::{StopTimeEvent, StopTimeUpdate},
        };
        let unix = 1_793_000_000i64;
        let t0 = 5_000_000_000_000i64;
        let update = |trip: &str, route: &str, at: i64| FeedEntity {
            id: trip.into(),
            trip_update: Some(TripUpdate {
                trip: TripDescriptor {
                    trip_id: Some(trip.into()),
                    route_id: Some(route.into()),
                    ..Default::default()
                },
                stop_time_update: vec![StopTimeUpdate {
                    stop_id: Some("120S".into()),
                    arrival: Some(StopTimeEvent {
                        time: Some(at),
                        ..Default::default()
                    }),
                    departure: Some(StopTimeEvent {
                        time: Some(at + 30),
                        ..Default::default()
                    }),
                    ..Default::default()
                }],
                ..Default::default()
            }),
            vehicle: None,
        };
        let body = FeedMessage {
            header: FeedHeader {
                gtfs_realtime_version: "1.0".into(),
                timestamp: Some(unix as u64),
            },
            entity: vec![update("a", "2", unix + 10), update("b", "3", unix + 300)],
        }
        .encode_to_vec();
        let s96 = stations::all()
            .iter()
            .find(|s| s.name == "96 St" && s.lines.iter().any(|l| l == "2"))
            .unwrap();
        let mut f = Fusion::whole_ride();
        f.context.in_subway = true;
        let stop = train_stops::Stop {
            t_ns: t0 + 12_000_000_000,
            score: 1.0,
            braking_since_ns: t0,
        };
        f.train_stops.push((stop, Some(s96), true));
        assert_eq!(f.status().route, None);
        f.on_transit(t0, unix * 1000, &body);
        f.on_cell(t0 + 30_000_000_000, None);
        let s = f.status();
        assert_eq!(s.route.as_deref(), Some("2"));
        // No chime or beacons heard: no car type.
        assert_eq!(s.car_type, None);
        // Undecodable bodies are dropped.
        f.on_transit(t0 + 31_000_000_000, unix * 1000 + 31_000, b"not a feed");
    }

    const S: i64 = 1_000_000_000;

    fn stop(t_s: i64) -> (train_stops::Stop, Option<&'static Station>, bool) {
        let stop = train_stops::Stop {
            t_ns: t_s * S,
            score: 1.0,
            braking_since_ns: (t_s - 15) * S,
        };
        (stop, None, true)
    }

    /// Step-rhythm windows every 4 s over `from_s..to_s`, with steps over `steps_s`.
    fn windows(from_s: i64, to_s: i64, steps_s: std::ops::Range<i64>) -> Vec<(i64, bool)> {
        (from_s / 4..to_s / 4)
            .map(|k| (k * 4 * S, steps_s.contains(&(k * 4))))
            .collect()
    }

    fn cycle(open_s: i64, close_s: i64) -> DoorCycle {
        DoorCycle {
            open_ns: open_s * S,
            close_ns: close_s * S,
        }
    }

    #[test]
    fn a_long_hold_ridden_through_is_one_dwell() {
        // 86 St on the C (`20261021-135509`, 2:04 pm): 104 s standing, doors open 85 s, no walking.
        let r = stop_records(
            &[stop(100), stop(300)],
            &[cycle(103, 188)],
            &[204 * S],
            &windows(0, 400, 0..0),
        );
        assert_eq!(r.len(), 2);
        assert!(!r[0].on_platform && !r[0].rider_left());
        assert_eq!(r[0].dwell_s(), Some(104.0));
        assert_eq!(r[0].doors_open_s(), Some(85.0));
    }

    #[test]
    fn walking_off_ends_the_record() {
        // Off from 100 s (the first step window, 100 to 104 s) for 40 s of walking; the train leaves unseen,
        // and a later surge is no departure of it.
        let r = stop_records(
            &[stop(100)],
            &[cycle(103, 125)],
            &[260 * S],
            &windows(0, 300, 104..144),
        );
        assert!(r[0].rider_left());
        assert_eq!(r[0].alighted_ns, Some(100 * S));
        assert_eq!(r[0].dwell_s(), None);
        // The closing, heard from the platform within a minute, is still the train's.
        assert_eq!(r[0].doors_closed_ns, Some(125 * S));
    }

    #[test]
    fn stepping_into_a_car_is_when_its_beacon_rises() {
        // Walked to 100 s; a new car first heard at 150 s around -86 dBm, then -74 from 180 s; away at 200 s.
        let mut readings = vec![(40 * S, 7, -80)];
        readings.extend((150..180).step_by(3).map(|t| (t * S, 9, -86)));
        readings.extend((180..200).step_by(3).map(|t| (t * S, 9, -74)));
        assert_eq!(stepped_in(&readings, 100 * S, 200 * S), Some(180 * S));
        // A car heard since before the walk ended doesn't count.
        let mut early = vec![(90 * S, 9, -86)];
        early.extend(readings.iter().copied());
        assert_eq!(stepped_in(&early, 100 * S, 200 * S), None);
    }

    #[test]
    fn a_transfer_boards_the_next_train() {
        // Off at 104 s, walking to 144 s, standing, boarding with steps ending at 200 s, away at 210 s.
        let mut w = windows(0, 300, 104..144);
        w.iter_mut()
            .filter(|(t, _)| *t == 196 * S || *t == 200 * S)
            .for_each(|(_, s)| *s = true);
        let r = stop_records(&[stop(100)], &[], &[210 * S], &w);
        assert_eq!(r[0].boarded_ns, Some(200 * S));
        assert_eq!(r[0].departed_ns, Some(210 * S));
        assert!(r[0].rider_left());
        assert_eq!(r[0].dwell_s(), None);
    }

    #[test]
    fn stepping_out_to_let_others_off_keeps_the_dwell() {
        // Off and back on within 16 s (`20261020-184134`, 7:07 pm), the same train away at 130 s.
        let r = stop_records(
            &[stop(100)],
            &[cycle(103, 125)],
            &[130 * S],
            &windows(0, 300, 104..120),
        );
        assert!(!r[0].rider_left());
        assert!((r[0].dwell_s().unwrap() - 30.0).abs() < 1e-6);
    }

    #[test]
    fn a_missed_stop_is_found_from_walking_off() {
        // 96 St, `20261021-215922`: off at 100 s, 40 s of walking, standing, on with steps to 200 s, away at 210 s.
        let mut w = windows(0, 300, 104..144);
        w.iter_mut()
            .filter(|(t, _)| *t == 196 * S || *t == 200 * S)
            .for_each(|(_, s)| *s = true);
        let found = missed_stop(&w, &[], &[30 * S], 210 * S).unwrap();
        assert_eq!(found.t_ns, 94 * S);
        assert_eq!(found.score, MISSED_STOP_SCORE);
        let r = stop_records(&[(found, None, true)], &[], &[210 * S], &w);
        assert_eq!(r[0].alighted_ns, Some(100 * S));
        assert_eq!(r[0].boarded_ns, Some(200 * S));
        assert!(r[0].rider_left());
        // Not when the stop was found, nor for a short walk along the train.
        assert_eq!(missed_stop(&w, &[stop(90)], &[30 * S], 210 * S), None);
        // Nor walking into the station to board: no ride before the walk, or a stop after the last departure.
        assert_eq!(missed_stop(&w, &[], &[], 210 * S), None);
        assert_eq!(missed_stop(&w, &[stop(50)], &[30 * S], 210 * S), None);
        assert_eq!(
            missed_stop(&windows(0, 300, 104..120), &[], &[30 * S], 210 * S),
            None
        );
    }

    #[test]
    fn other_cars_after_a_stop_are_another_train() {
        let heard = |cars: &[u16], from_s: i64, to_s: i64| -> Vec<(i64, u16, i32)> {
            (from_s..to_s)
                .step_by(5)
                .flat_map(|t| cars.iter().map(move |&c| (t * S, c, -80)))
                .collect()
        };
        let mut swapped = heard(&[1, 2, 3], 40, 100);
        swapped.extend(heard(&[7, 8], 160, 220));
        assert!(trains_differ(&swapped, 100 * S, 160 * S));
        let mut stayed = heard(&[1, 2, 3], 40, 100);
        stayed.extend(heard(&[3, 4], 160, 220));
        assert!(!trains_differ(&stayed, 100 * S, 160 * S));
        // Nothing heard on one side says nothing.
        assert!(!trains_differ(&heard(&[1], 40, 100), 100 * S, 160 * S));
    }

    #[test]
    fn walking_on_the_platform_is_no_dwell() {
        let r = stop_records(&[stop(100)], &[], &[130 * S], &windows(0, 300, 60..104));
        assert!(r[0].on_platform);
        assert_eq!(r[0].dwell_s(), None);
    }

    #[test]
    fn a_departures_fading_pull_is_no_stop_but_its_departure() {
        // Grand Central (`20261020-151449`, 3:44 pm): doors closed at 150 s, the pull from 151 s faded at 164 s,
        // and no start surge was found.
        let mut fade = stop(164);
        fade.0.braking_since_ns = 151 * S;
        let r = stop_records(
            &[stop(100), fade, stop(250)],
            &[cycle(103, 150)],
            &[],
            &windows(0, 300, 0..0),
        );
        assert_eq!(r.len(), 2);
        assert_eq!(r[0].departed_ns, Some(151 * S));
        assert_eq!(r[0].dwell_s(), Some(51.0));
    }

    #[test]
    fn a_false_closing_while_pulling_in_keeps_a_short_braking_stop() {
        // `20261020-184134`, 7:00 pm: a closing heard 5 s before a stop whose braking lasted under 7 s.
        let mut s = stop(100);
        s.0.braking_since_ns = 95 * S;
        assert!(!departure_fade(&s.0, &[cycle(60, 95)]));
    }
}
