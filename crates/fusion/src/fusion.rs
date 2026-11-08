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
/// Kept small: the IMU samples are most of them, every one of a whole ride's for the replay.
/// They're `f32` as the sensors report them (see [`imu_sample`]), so that's exact, and the rare inputs are boxed.
#[derive(Clone, Debug)]
enum Input {
    Accel([f32; 3]),
    Gyro([f32; 3]),
    /// A fix, with its speed only if it was used.
    Fix(Box<Fix>),
    /// Our own velocity solution's speed and standard deviation.
    Speed(Box<(f64, f64)>),
    /// Entering or leaving the subway changed the speed's upper bound from this,
    /// which the backward pass goes back to.
    MaxSpeedBefore(Box<Option<f64>>),
    /// At a door cycle's closing: the train stood still from this time to here (see [`DWELL_SURE_S`]).
    StandingSince(i64),
    /// The rider has been walking, or running if true (see [`crate::walking`]).
    Walking(bool),
    /// The rider's car's accelerometer reading (see [`LiveSpeedEstimator::on_car_accel`]).
    Car(Box<(u16, [f64; 3])>),
}

const _: () = assert!(std::mem::size_of::<Input>() == 16);

/// An IMU sample as kept: the `f32` it came from, exactly.
fn imu_f32(v: [f64; 3]) -> [f32; 3] {
    v.map(|x| x as f32)
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
/// How many of the stations named before a departure [`Fusion::before_next_station`] looks back through.
const PREV_STATIONS: usize = 8;
/// How soon after a door cycle's closing a departure must come for [`Fusion::before_next_station`] to time from it.
const DEPART_AFTER_DOORS_S: f64 = 20.0;
/// A door cycle opening this soon (s) after the last one closed is the doors reopening at the same stop.
const REOPEN_S: f64 = 15.0;
/// See [`Fusion::moved_on`].
const MOVED_ON_AFTER_S: f64 = 20.0;
/// How long the radio must keep naming the other station: a cell of a neighboring station flickers in at times.
const MOVED_ON_HELD_S: f64 = 10.0;
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
/// How much a station the radio names on a leg counts as a known place (see [`Fusion::anchor_by_radio`]).
const RADIO_ANCHOR_WEIGHT: f64 = 0.5;
/// By when (s after the lurch) the stop detector has decided on it: [`train_stops::SEP_S`] and the 10 s it waits more.
const LIKELY_DECIDED_BY_S: f64 = train_stops::SEP_S + 12.0;
/// A "stop" whose braking began within [`FADE_AFTER_CLOSE_S`] after a door closing, and lasted at least
/// [`FADE_MIN_S`], is the departure's pull fading (the train sped away, then cruised): at Grand Central on
/// `20261020-151449` (3:44 pm), and three more, the pull began 0 to 8 s after the closing and lasted 11 to 17 s.
/// A false closing heard while pulling in (`20261020-184134`, 7:00 pm) came 5 s before a real stop.
const FADE_AFTER_CLOSE_S: (f64, f64) = (-0.5, 8.0);
const FADE_MIN_S: f64 = 7.0;
/// A likely stop whose lurch comes within this (s) after a departure found live is the departure's pull ending
/// (motoring cut off, then cruising), not a stop: it doesn't hold the live speed at 0. Over the 37 sessions with
/// legs, 16 of the 31 legs held under half their length by a likely stop were held by a lurch 6 to 20 s after
/// the departure, with no closing heard just before (so [`FADE_AFTER_CLOSE_S`] missed them); none of them split
/// the legs between stop records, so the stop detector dropped them once it had decided ([`LIKELY_DECIDED_BY_S`]).
/// At 20 s: under 0.5 of the track 46 to 30 legs, within 0.7 to 1.3 82 to 91 (at 15 s: 36 and 85).
const LIKELY_AFTER_DEPARTURE_S: f64 = 20.0;
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
/// How far (s) a stop's time can be before a stop found earlier: a missed stop is found up to
/// [`RECORD_MAX_DWELL_S`] after the walk off began, [`ALIGHT_AFTER_STOP_S`] after the stop.
const MISSED_STOP_DISORDER_S: f64 = RECORD_MAX_DWELL_S + ALIGHT_AFTER_STOP_S + 60.0;
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
    /// The track (m) from the station before, on the same train, and the average speed over it (m/s):
    /// from the departure there to this stop (see [`crate::track::legs`]).
    pub from_last_station: Option<(f64, f64)>,
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

/// Audio as door detection takes it: mono, one sample per frame.
#[derive(Clone, Debug)]
pub struct AudioSamples {
    pub first_frame: i64,
    pub samples: Vec<i16>,
    pub timestamps: Vec<(i64, i64)>,
}

impl AudioSamples {
    /// From 16-bit little-endian PCM as recorded, `channels` interleaved (one per microphone):
    /// each frame is the mean of its channels, live and replayed alike.
    pub fn from_pcm(
        first_frame: i64,
        pcm_s16le: &[u8],
        channels: usize,
        timestamps: Vec<(i64, i64)>,
    ) -> Self {
        let channels = channels.max(1);
        let samples = pcm_s16le
            .as_chunks::<2>()
            .0
            .chunks_exact(channels)
            .map(|frame| {
                let sum: i32 = frame
                    .iter()
                    .map(|&b| i32::from(i16::from_le_bytes(b)))
                    .sum();
                (sum / channels as i32) as i16
            })
            .collect();
        Self {
            first_frame,
            samples,
            timestamps,
        }
    }
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
            Some(Input::Accel(imu_f32(*a)))
        }
        Pending::Gyro(w) => {
            estimator.on_gyro(w[0], w[1], w[2]);
            Some(Input::Gyro(imu_f32(*w)))
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
            Some(Input::Speed(Box::new((speed, sigma))))
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
/// The live speed drifts to 3 m/s over a ride off the train underground (`20261024-003357`, 12.6 m up after
/// alighting), and a train just boarded, before any of its stops or departures is felt, rides at 7 m/s or more.
const VERTICAL_VEHICLE_MPS: f64 = 5.0;
/// Riding up or down (an elevator or escalator) takes at least this (s) from start to end, the span included:
/// the rider's elevators took 53 s to 1:02 (`crates/cli/examples/score/vertical_labels.csv`), and the
/// station exit stairs found with few steps 24 to 28 s (as did pressure noise).
const VERTICAL_MIN_RIDE_S: f64 = 40.0;
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

/// The leg in progress (see [`Fusion::recent_track_path`]): from the departure at `left`, along `track`,
/// with `next` of [`Fusion::live_speeds`] summed into `m` (meters) so far.
/// A good fix's time, a station's address, and the riding lines (see `Fusion::fix_tracks`).
type FixTrackKey = (i64, usize, Vec<String>);

struct AheadPath {
    left: i64,
    /// The track's length over the live speeds' distance on the leg before (see [`Fusion::ahead_scale`]).
    scale: f64,
    track: crate::track::Track,
    next: usize,
    m: f64,
    prev: Option<(i64, f32)>,
    points: Vec<(SmoothedSpeed, (f64, f64))>,
}

/// The bounds of [`Fusion::ahead_scale`].
const AHEAD_SCALE: (f64, f64) = (0.5, 4.0);

/// How often [`Fusion::recent_track_path`] gives a point along the track (ns).
const TRACK_PATH_EVERY_NS: i64 = 1_000_000_000;

/// A smoothed speed at a fix or a sample time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SmoothedSpeed {
    pub elapsed_realtime_ns: i64,
    pub speed_mps: f64,
    pub sigma_mps: f64,
    pub forward_mps: f64,
    pub backward_mps: f64,
    /// Acceleration in the direction of travel (m/s^2): the slope of the smoothed speeds themselves, centered
    /// (see [`accel_from_speeds`]).
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
    /// The forward (live) speed (m/s) at each sample time, for the whole ride: what an elevator or escalator
    /// check needs of the speed, without smoothing.
    live_speeds: Vec<(i64, f32)>,
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
    /// Weaker starts ([`train_stops::Event::Started`]), only for ending a likely stop.
    starts: Vec<i64>,
    /// The departure the speed's leg cap counts from (see [`LiveSpeedEstimator::start_leg`]), if on one,
    /// the station it left, and the last station the radio named on the way (see [`Fusion::anchor_by_radio`]).
    leg_since_ns: Option<i64>,
    leg_from: Option<&'static Station>,
    leg_radio: Option<&'static Station>,
    /// The timeline's segments kept, and the walks so far (see [`Fusion::timeline`]).
    timeline_kept: std::cell::RefCell<TimelineKept>,
    walks: std::cell::RefCell<Walks>,
    subway_periods: std::cell::RefCell<SubwayPeriods>,
    /// The lines ridden as of the old enough stops (see [`Fusion::riding_lines`]).
    riding_lines: std::cell::RefCell<RidingLines>,
    /// The track between each pair of stations a leg was looked up for (see [`crate::track::between`]),
    /// by their addresses: a status looks up the same recent legs again.
    tracks: std::cell::RefCell<HashMap<(usize, usize), Option<crate::track::Track>>>,
    /// The track between a good fix and a station (see [`crate::track::to_station`]), by the fix's time,
    /// the station's address, and the riding lines: each tick asks for the same recent ones again.
    fix_tracks: std::cell::RefCell<HashMap<FixTrackKey, Option<crate::track::Track>>>,
    /// The leg in progress so far (see [`Fusion::recent_track_path`]): its start, the live speeds summed,
    /// and its points, so each tick adds only the speeds since the last.
    ahead_path: std::cell::RefCell<Option<AheadPath>>,
    /// The stop records settled so far (see [`Fusion::records`]), so a status recomputes only the recent ones.
    settled_records: std::cell::RefCell<SettledRecords>,
    /// The times of good fixes (see [`Fusion::good_fix_ns`]), in time order, and where each was.
    good_fixes: Vec<i64>,
    good_fix_points: Vec<(f64, f64)>,
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
    /// Whether the smoothed speeds are held to the track between stations (see [`crate::track::hold_to_track`]).
    track_hold: bool,
    /// For scoring only: no smoothing of the window as inputs arrive (see [`Fusion::set_live_smoothing`]).
    no_live_smoothing: bool,
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

    /// Over a whole ride, room for this many more IMU samples (accelerometer and gyro) and other inputs:
    /// growing the buffer as they come briefly holds it twice, and leaves it up to twice as big as needed.
    pub fn reserve_inputs(&mut self, additional: usize) {
        if self.window_ns.is_none() {
            self.inputs.reserve_exact(additional);
        }
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
            live_speeds: Vec::new(),
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
            starts: Vec::new(),
            leg_since_ns: None,
            leg_from: None,
            leg_radio: None,
            settled_records: Default::default(),
            tracks: Default::default(),
            fix_tracks: Default::default(),
            ahead_path: Default::default(),
            timeline_kept: std::cell::RefCell::new(TimelineKept {
                joined: Vec::new(),
                until: i64::MIN,
            }),
            walks: Default::default(),
            subway_periods: Default::default(),
            riding_lines: std::cell::RefCell::new(RidingLines {
                lines: Vec::new(),
                prev: i64::MIN,
                max_t: i64::MIN,
                upto: 0,
            }),
            good_fixes: Vec::new(),
            good_fix_points: Vec::new(),
            likely_stop: None,
            unconfirmed_stops: Vec::new(),
            walking: WalkingDetector::default(),
            step_windows: Vec::new(),
            cars: CarTracker::default(),
            car_since: None,
            altimeter: Default::default(),
            trips: trips::Trips::default(),
            subway_changes: Vec::new(),
            track_hold: true,
            no_live_smoothing: false,
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

    /// For scoring only: leave the smoothed speeds unheld to the track (see [`crate::track::hold_to_track`]).
    pub fn set_track_hold(&mut self, on: bool) {
        self.track_hold = on;
    }

    /// For scoring only: `false` skips smoothing the window as inputs arrive, which is most of a live replay's time,
    /// for what reads only the forward (live) estimate and the stops: the smoothed speeds are then the forward
    /// estimate's, until [`Fusion::finish`] smooths the last window.
    pub fn set_live_smoothing(&mut self, on: bool) {
        self.no_live_smoothing = !on;
    }

    /// Whether the phone is in the subway as of the settled inputs.
    pub fn in_subway(&self) -> bool {
        self.context.in_subway
    }

    /// A Wi-Fi scan's SSIDs, at the time it was recorded.
    pub fn on_wifi_scan(&mut self, t_ns: i64, ssids: Vec<String>) {
        self.arrive(t_ns, Pending::WifiScan(ssids));
    }

    /// Microphone samples (at `rate_hz`, channels already mixed) as they're read, at `t_ns`, when they arrived;
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
    fn route(&self, t: i64, boarded_after: i64, records: &Records) -> Option<String> {
        let ago = |at: i64| (t - at) as f64 * 1e-9;
        let boarding = self
            .boarding(t, records)
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
                recent(
                    &self.train_stops,
                    |x| x.0.t_ns,
                    t - (ROUTE_STOPS_S * 1e9) as i64 - 1,
                    (RECORDS_SETTLE_S * 1e9) as i64,
                )
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
    /// The stops before [`RidingLines::upto`] are kept as they left the lines, once old enough (see
    /// [`RECORDS_SETTLE_S`]), so only the newer are gone through again: for `t` at or after them all.
    fn riding_lines(&self, t: i64) -> Vec<String> {
        let lag = (RECORDS_SETTLE_S * 1e9) as i64;
        let now = self.settled_ns.unwrap_or(i64::MIN);
        let mut kept = self.riding_lines.borrow_mut();
        // Keep the old enough ones, for any later `t`.
        while let Some(&(stop, station, in_subway)) = self.train_stops.get(kept.upto) {
            if stop.t_ns >= now.saturating_sub(lag) {
                break;
            }
            let (lines, prev) = self.ride_on(&kept.lines, kept.prev, stop.t_ns, station, in_subway);
            kept.lines = lines;
            kept.prev = prev;
            kept.max_t = kept.max_t.max(stop.t_ns);
            kept.upto += 1;
        }
        let (mut lines, mut prev, from) = if t >= kept.max_t {
            (kept.lines.clone(), kept.prev, kept.upto)
        } else {
            (Vec::new(), i64::MIN, 0)
        };
        drop(kept);
        for &(stop, station, in_subway) in &self.train_stops[from..] {
            if stop.t_ns <= t {
                (lines, prev) = self.ride_on(&lines, prev, stop.t_ns, station, in_subway);
            }
        }
        // Walking since the last stop: the next may be another line's.
        if self.walked(prev, t) {
            Vec::new()
        } else {
            lines
        }
    }

    /// The lines after a stop at `at` (see [`Self::riding_lines`]), and the stop before the next.
    fn ride_on(
        &self,
        lines: &[String],
        prev: i64,
        at: i64,
        station: Option<&'static Station>,
        in_subway: bool,
    ) -> (Vec<String>, i64) {
        let Some(station) = station.filter(|_| in_subway) else {
            return (lines.to_vec(), prev);
        };
        let walked = self.walked(prev, at);
        // By complex: a stop named at another of its platforms (Queensboro Plaza's 7 one, from the N/W,
        // Tue Nov 3, 6:10 pm) is still a stop of the lines the train was on.
        let both: Vec<String> = lines
            .iter()
            .filter(|l| stations::serves(station, l))
            .cloned()
            .collect();
        let lines = if both.is_empty() || walked {
            station.lines.clone()
        } else {
            both
        };
        (lines, at)
    }

    /// Whether there were [`LINE_RESET_WINDOWS`] step windows with steps after `from`, up to `to`.
    fn walked(&self, from: i64, to: i64) -> bool {
        let i = self.step_windows.partition_point(|&(at, _)| at <= from);
        self.step_windows[i..]
            .iter()
            .take_while(|&&(at, _)| at <= to)
            .filter(|&&(_, steps)| steps)
            .take(LINE_RESET_WINDOWS)
            .count()
            >= LINE_RESET_WINDOWS
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

    /// Whether the radio has named a station outside the complex of the one it named at `stopped_ns`, since then
    /// up to `t`: the train has moved on, though no departure was felt. On `20261024-085154` the 3's start from
    /// 96 St was taken for a stop, and its live hold and dwell ran past 86 St and 79 St to 72 St (1:38 at 79 St).
    /// Only the stations named since the stop are looked at, and callers ask only within [`RECORD_MAX_DWELL_S`].
    /// The radio often names the station only some seconds into the dwell (the cell before it lingers),
    /// so the stop's station is the one named [`MOVED_ON_AFTER_S`] after it, and only stations named after that,
    /// and for [`MOVED_ON_HELD_S`], count.
    fn moved_on(&self, stopped_ns: i64, t: i64) -> bool {
        let settled = stopped_ns + (MOVED_ON_AFTER_S * 1e9) as i64;
        if t <= settled {
            return false;
        }
        let Some(at) = self.station_at(settled) else {
            return false;
        };
        let from = self
            .station_history
            .partition_point(|&(first, ..)| first <= settled);
        self.station_history[from..]
            .iter()
            .take_while(|&&(first, ..)| first <= t)
            .any(|&(first, last, s)| {
                !stations::same_complex(at, s) && (last - first) as f64 * 1e-9 >= MOVED_ON_HELD_S
            })
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
        let records = self.records();
        let vertical = self
            .vertical_moves_where(
                &records,
                t - ((VERTICAL_RECENT_S + 1.0) * 1e9) as i64,
                |m| recent(m.end_ns, VERTICAL_RECENT_S),
            )
            .pop();
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
        let transfer_s = self.transfer_s(t, &records);
        let boarded_after = records
            .last_left()
            .and_then(|r| r.alighted_ns)
            .unwrap_or(i64::MIN);
        // The train's newest stop in the subway, when it's after the last door cycle: the train has moved on
        // from that cycle's station (14 St still shown at 18 St, Thu Oct 29, 1:25 am), so the newer stop names it.
        let last_door = self
            .last_stop
            .filter(|&(at, _)| recent(at, 60.0) && at > boarded_after);
        let stopped_since = last_door.and_then(|(at, _)| {
            self.train_stops
                .iter()
                .rev()
                // Found in about time order: the last few cover a stop found out of order.
                .take(4)
                .find(|(stop, _, in_subway)| *in_subway && stop.t_ns > at)
        });
        // A stop not decided yet (a likely one) after the cycle: the cell names its station, if any.
        let likely_since = last_door.is_some_and(|(at, _)| {
            self.likely_stop_now(t)
                .is_some_and(|l| l.t_ns > at && self.context.in_subway)
        });
        let stopped_at = match stopped_since {
            Some(&(_, station, _)) => station,
            None if likely_since => self.station_now(t),
            None => last_door.and_then(|(_, s)| s),
        };
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
        // A line is only named at a station it serves: one matched from earlier stops is stale at a station
        // of another line (the 6 from 125 St still shown at Astoria Blvd, Sat Nov 7, 2:57 pm).
        let route = self
            .route(t, boarded_after, &records)
            .filter(|_| aboard)
            .filter(|r| station.as_ref().is_none_or(|s| stations::serves(s, r)));
        // The route if the feeds give it, else the lines the train has been on, else the station's:
        // between trains, all the lines of its complex, any of which the rider may take next.
        let lines = match &route {
            Some(r) => vec![r.clone()],
            None if !riding.is_empty() && aboard => {
                let at: Vec<String> = riding
                    .iter()
                    .filter(|l| station.as_ref().is_none_or(|s| stations::serves(s, l)))
                    .cloned()
                    .collect();
                if at.is_empty() { riding } else { at }
            }
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
        // The stop before, from the recent stops (found in about time order) if there's one, else from all.
        let prev_in = |stops: &[(train_stops::Stop, Option<&'static Station>, bool)]| {
            stops
                .iter()
                .filter(|(s, _, in_subway)| {
                    *in_subway && s.t_ns < stop.t_ns && !self.departure_fade(s)
                })
                .map(|(s, ..)| s.t_ns)
                .max()
        };
        let lag = s_ns(RECORDS_SETTLE_S);
        let prev = prev_in(recent(&self.train_stops, |x| x.0.t_ns, stop.t_ns, lag))
            .or_else(|| prev_in(&self.train_stops));
        // Door cycles are found at their closings, in time order.
        let cycle = self.door_cycles.get(
            self.door_cycles
                .partition_point(|c| c.close_ns <= stop.t_ns + s_ns(DOORS_AFTER_STOP_S.0)),
        );
        let closed = cycle.is_some();
        if !closed && prev.is_some_and(|p| self.between_stations(p, stop.t_ns)) {
            return None;
        }
        let since = |from: i64| (t - from) as f64 * 1e-9;
        let stopped_s = since(stop.t_ns);
        if self.departed_after(stop.t_ns).is_some()
            || stopped_s > RECORD_MAX_DWELL_S
            || self.moved_on(stop.t_ns, t)
        {
            return None;
        }
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
    /// The segments up to a settled time are kept (see [`TimelineKept`]), so only those since are made again.
    pub fn timeline(&self) -> Vec<Segment> {
        self.timeline_since(0).2
    }

    /// [`Self::timeline`] from segment `from` on, clamped to the settled ones, with that index and how many are
    /// settled: those never change, so a caller holding the first `from` keeps them and replaces the rest.
    /// The app asks this at every smoothing tick, so it sends only the open end (see `crates/ffi`).
    pub fn timeline_since(&self, from: usize) -> (usize, usize, Vec<Segment>) {
        let t = self.now_ns;
        let lag = (RECORDS_SETTLE_S * 1e9) as i64;
        let records = self.records();
        let mut kept = self.timeline_kept.borrow_mut();
        // Keep up to a settled time, or the start of the leg it's in: a leg's line is from all its stops.
        let settled = self.settled_ns.unwrap_or(i64::MIN).saturating_sub(lag);
        // Only the periods ending after both are used below.
        let periods = self.subway_periods(t, kept.until.min(settled));
        let until = periods
            .iter()
            .find(|&&(from, to)| from < settled && settled < to)
            .map_or(settled, |&(from, _)| {
                self.cut_before(&records, from, settled).unwrap_or(from)
            });
        if until > kept.until {
            let pieces = self.timeline_between(&records, &periods, kept.until, until);
            join_segments(&mut kept.joined, pieces);
            kept.until = until;
        }
        // The last kept segment still grows when what follows joins it.
        let settled = kept.joined.len().saturating_sub(1);
        let from = from.min(settled);
        let mut out = kept.joined[from..].to_vec();
        let until = kept.until;
        drop(kept);
        join_segments(
            &mut out,
            self.timeline_between(&records, &periods, until, i64::MAX),
        );
        (from, settled, out)
    }

    /// The subway's periods (see [`Self::timeline`]) ending after `after`: from the detector's switches,
    /// back within a minute the same, the last still open at `t`.
    /// Only the switches since the last call are read, onto the periods kept from them.
    fn subway_periods(&self, t: i64, after: i64) -> Vec<(i64, i64)> {
        let s_ns = |s: f64| (s * 1e9) as i64;
        let mut kept = self.subway_periods.borrow_mut();
        let SubwayPeriods {
            closed: periods,
            since,
            upto,
        } = &mut *kept;
        for &(at, on) in &self.subway_changes[*upto..] {
            match (on, *since) {
                // Back within a minute: the detector flickering (or an elevated stretch), the same ride.
                (true, None)
                    if periods
                        .last()
                        .is_some_and(|p| at - p.1 <= s_ns(TIMELINE_SUBWAY_GAP_S)) =>
                {
                    *since = periods.pop().map(|p| p.0);
                }
                (true, None) => *since = Some(at),
                (false, Some(s)) => {
                    periods.push((s, at));
                    *since = None;
                }
                _ => {}
            }
        }
        *upto = self.subway_changes.len();
        let first = periods.partition_point(|p| p.1 <= after);
        let mut out = periods[first..].to_vec();
        if let Some(s) = *since {
            out.push((s, t));
        }
        out
    }

    /// The last walk off a train in the subway period from `from`, at or before `at`.
    fn cut_before(&self, records: &Records, from: i64, at: i64) -> Option<i64> {
        // A walk off is within `RECORD_MAX_DWELL_S` after its stop.
        let dwell = (RECORD_MAX_DWELL_S * 1e9) as i64;
        records
            .before(at)
            .take_while(|r| r.stopped_ns > from - dwell)
            .filter(|r| r.rider_left())
            .filter_map(|r| r.alighted_ns)
            .find(|&a| a > from && a <= at)
    }

    /// The timeline's segments (see [`Self::timeline`]) cut to after `lo`, up to `hi`, in time order, before
    /// those next to each other are joined. Its segments don't overlap, so cut at any time and joined again
    /// they're the same.
    fn timeline_between(
        &self,
        records: &Records,
        periods: &[(i64, i64)],
        lo: i64,
        hi: i64,
    ) -> Vec<Segment> {
        let s_ns = |s: f64| (s * 1e9) as i64;
        let overlaps = |a: i64, b: i64| b > lo && a < hi;
        let walks: Vec<(i64, i64)> = {
            let mut kept = self.walks.borrow_mut();
            for &(at, steps) in &self.step_windows[kept.upto..] {
                if !steps {
                    continue;
                }
                let start = at - s_ns(WALK_WINDOW_S);
                match kept.walks.last_mut() {
                    Some(w) if start <= w.1 + s_ns(WALK_WINDOW_S) => w.1 = at,
                    _ => kept.walks.push((start, at)),
                }
            }
            kept.upto = self.step_windows.len();
            let first = kept.walks.partition_point(|w| w.1 <= lo);
            kept.walks[first..]
                .iter()
                .copied()
                .take_while(|&(a, _)| a < hi)
                .filter(|&(a, b)| b - a >= s_ns(TIMELINE_WALK_S))
                .collect()
        };
        let mut segments: Vec<Segment> = Vec::new();
        for &(from, to) in periods.iter().filter(|&&(a, b)| overlaps(a, b)) {
            // The legs, which each walk off a train ends: from the last before `lo`.
            let first = self.cut_before(records, from, lo).unwrap_or(from).max(from);
            let dwell = s_ns(RECORD_MAX_DWELL_S);
            let mut cuts: Vec<i64> = records
                .since(first - dwell)
                .filter(|r| r.rider_left())
                .filter_map(|r| r.alighted_ns)
                .filter(|&a| a > first && a < to)
                .collect();
            cuts.push(to);
            let mut start = first;
            for end in cuts {
                if start >= hi {
                    break;
                }
                if end <= lo {
                    start = end;
                    continue;
                }
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
        for m in self.vertical_moves_where(records, lo, |m| m.start_ns < hi) {
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
        let mut out: Vec<Segment> = segments
            .into_iter()
            .filter(|s| overlaps(s.start_ns, s.end_ns))
            .map(|s| Segment {
                start_ns: s.start_ns.max(lo),
                end_ns: s.end_ns.min(hi),
                ..s
            })
            .collect();
        out.sort_by_key(|s| s.start_ns);
        out
    }

    /// The line of a subway leg from its stops (see [`Self::timeline`]).
    fn leg_line(&self, from: i64, to: i64) -> Option<String> {
        let lag = (RECORDS_SETTLE_S * 1e9) as i64;
        let stops: Vec<trips::RiderStop> = recent(&self.train_stops, |x| x.0.t_ns, from - 1, lag)
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
    fn transfer_s(&self, t: i64, records: &Records) -> Option<f64> {
        if !self.context.in_subway {
            return None;
        }
        let (alighted, boarded) = self.boarding(t, records)?;
        let since_s = (t - alighted) as f64 * 1e-9;
        (boarded.is_none() && since_s <= TRANSFER_MAX_S).then_some(since_s)
    }

    /// When the rider last walked off a train, and how they boarded the next, if they have:
    /// the train's stop at the platform (stopped, departed), as near as known.
    /// The first departure since with no steps around it, the train leaving with the rider,
    /// or else the new car heard as theirs for [`TRANSFER_CAR_S`], from when it was first: on the L, whose
    /// departures are missed, it's heard just as the rider boards, from the platform.
    fn boarding(&self, t: i64, records: &Records) -> Option<(i64, Option<(i64, i64)>)> {
        let left = records.last_left()?;
        let alighted = left.alighted_ns?;
        let s_ns = |s: f64| (s * 1e9) as i64;
        let still = |d: i64| {
            let from = self
                .step_windows
                .partition_point(|&(at, _)| at <= d - s_ns(TRANSFER_STILL_S.0));
            !self.step_windows[from..]
                .iter()
                .take_while(|&&(at, _)| at < d + s_ns(TRANSFER_STILL_S.1))
                .any(|&(_, steps)| steps)
        };
        // The departures found, and those the stop records give (a stop stepped out at, then ridden on from):
        // a record's departure is within `RECORD_MAX_DWELL_S` of its stop.
        let departed = records
            .since(alighted - s_ns(RECORD_MAX_DWELL_S))
            .filter_map(|r| r.departed_ns)
            .filter(|&d| d > alighted && still(d))
            .min();
        // The first, of departures found in about time order: none found later is more than a lag earlier.
        let lag = s_ns(RECORDS_SETTLE_S);
        let start = recent_index(&self.departures, |&d| d, alighted, lag);
        let mut found: Option<i64> = None;
        for &d in &self.departures[start..] {
            if found.is_some_and(|f| d > f + lag) {
                break;
            }
            if d > alighted && still(d) {
                found = Some(found.map_or(d, |f| f.min(d)));
            }
        }
        let departure = found.into_iter().chain(departed).min();
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
        self.vertical_moves_where(&self.records(), i64::MIN, |_| true)
    }

    /// [`Self::vertical_moves`] ending after `since_ns` of those `keep` keeps: the status keeps only recent ones,
    /// found from the recent heights (see [`crate::vertical::moves_since`]).
    /// A ride is at a vehicle's speed by the live speed over its span (see [`Self::live_speeds`]),
    /// a slice of it found by time, so the check costs the same however long the ride.
    fn vertical_moves_where(
        &self,
        records: &Records,
        since_ns: i64,
        keep: impl Fn(&crate::vertical::Move) -> bool,
    ) -> Vec<crate::vertical::Move> {
        let s_ns = |s: f64| (s * 1e9) as i64;
        crate::vertical::moves_since(
            self.altimeter.heights(),
            &self.step_windows,
            WALK_WINDOW_S,
            since_ns,
        )
        .into_iter()
        .filter(|m| keep(m))
        .filter(|m| {
            if m.kind == crate::vertical::Kind::Stairs {
                return true;
            }
            // Departures are found in time order.
            let from = self
                .departures
                .partition_point(|&d| d < m.start_ns - s_ns(VERTICAL_DEPARTED_S));
            let departed = self.departures.get(from).is_some_and(|&d| d <= m.end_ns);
            if departed {
                return false;
            }
            if self.aboard_at(m.start_ns, records) {
                return false;
            }
            let from = self.live_speeds.partition_point(|&(t, _)| t < m.start_ns);
            let to = self.live_speeds.partition_point(|&(t, _)| t <= m.end_ns);
            let speeds = &self.live_speeds[from..to.max(from)];
            let fast = !speeds.is_empty()
                && speeds.iter().map(|&(_, v)| f64::from(v)).sum::<f64>() / speeds.len() as f64
                    > VERTICAL_VEHICLE_MPS;
            !fast
        })
        // A short ride is stairs, its steps missed: an elevator's ride takes longer.
        .map(|mut m| {
            if m.kind == crate::vertical::Kind::Riding
                && m.end_ns - m.start_ns < s_ns(VERTICAL_MIN_RIDE_S)
            {
                m.kind = crate::vertical::Kind::Stairs;
            }
            m
        })
        .collect()
    }

    /// Whether the rider was aboard a train at `t`: it departed or stopped with them (no steps around it, as in
    /// [`Self::boarding`]) since they last walked off one. Aboard, riding up or down is the train: on
    /// `20261021-215922` and `20261024-085154` the status said "elevator or escalator down" between stations.
    fn aboard_at(&self, t: i64, records: &Records) -> bool {
        let s_ns = |s: f64| (s * 1e9) as i64;
        // A record's alighting and departure come in its own dwell, before the next stop, so the records
        // newest first give them newest first; settled ones after the last left aren't left at.
        let settled_left = records.settled.last_left.map_or(0, |i| i + 1);
        let left = records
            .tail
            .iter()
            .rev()
            .chain(records.settled.records[..settled_left].iter().rev())
            .filter(|r| r.rider_left())
            .filter_map(|r| r.alighted_ns)
            .find(|&a| a <= t)
            .unwrap_or(i64::MIN);
        let still_around = |d: i64| {
            let from = self
                .step_windows
                .partition_point(|&(at, _)| at <= d - s_ns(TRANSFER_STILL_S.0));
            !self.step_windows[from..]
                .iter()
                .take_while(|&&(at, _)| at < d + s_ns(TRANSFER_STILL_S.1))
                .any(|&(_, steps)| steps)
        };
        let within = |d: i64| d > left && d <= t && still_around(d);
        // Departures are found in time order.
        let departures = &self.departures[self.departures.partition_point(|&d| d <= left)
            ..self.departures.partition_point(|&d| d <= t)];
        if departures.iter().rev().any(|&d| within(d)) {
            return true;
        }
        // The records' departures: those of stops up to `t`, back to the stop in whose dwell `left` is.
        for r in records.before(t) {
            if r.departed_ns.is_some_and(within) {
                return true;
            }
            if r.stopped_ns <= left {
                break;
            }
        }
        // Departures are missed (none on the C/E into Spring St, `20261021-215922`), so a stop felt counts too.
        // Stops are found in about time order (a missed one at the departure after it).
        recent(
            &self.train_stops,
            |x| x.0.t_ns,
            left,
            s_ns(RECORDS_SETTLE_S),
        )
        .iter()
        .rev()
        .filter(|(stop, _, in_subway)| *in_subway && !self.departure_fade(stop))
        .any(|(stop, ..)| within(stop.t_ns))
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
            self.anchor_by_radio(t_ns);
            return;
        }
        if let Pending::NetworkFix(lat, lon, accuracy) = input {
            self.network_station = stations::nearest(lat, lon)
                .filter(|&(_, d)| accuracy <= NETWORK_FIX_M && d <= NETWORK_STATION_M.max(accuracy))
                .map(|(s, _)| (t_ns, s));
            self.note_station(t_ns);
            self.anchor_by_radio(t_ns);
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
                self.buffer(t_ns, Input::Car(Box::new((r.car, r.accel))));
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
                if !self.before_next_station(cycle) {
                    self.on_door_cycle(t_ns, cycle);
                }
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
            self.buffer(t_ns, Input::MaxSpeedBefore(Box::new(before)));
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
            // A stop missed at the end of a ride, with no departure after it to find it from: found from the walk
            // off itself (`20261022-181802`, 6:34 pm).
            if self.context.in_subway
                && gait.is_some()
                && let Some(stop) = missed_stop(
                    &self.step_windows,
                    &self.train_stops,
                    &self.departures,
                    t_ns,
                )
            {
                let station = self.station_now(stop.t_ns);
                self.train_stops.push((stop, station, true));
                let from = stop.standing_ns();
                self.hold_standing(t_ns, from, from + (STOP_STANDING_S * 1e9) as i64);
            }
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
                train_stops::Event::Stopped(stop) => {
                    // Found [`train_stops::SEP_S`] late: it ends the leg only if it came after the leg began.
                    if self.leg_since_ns.is_some_and(|l| stop.t_ns > l) {
                        self.anchor_at_stop(&stop);
                    }
                    self.on_train_stop(t_ns, stop)
                }
                train_stops::Event::Likely(stop) => self.on_likely_stop(stop),
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
                    self.start_leg(at);
                }
                train_stops::Event::Started(at) => {
                    self.starts.push(at);
                    if self.likely_stop.is_some_and(|s| s.t_ns < at) {
                        self.start_leg(at);
                    }
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
            self.live_speeds.push((t_ns, estimate.speed_mps as f32));
            self.forward.push_back((t_ns, estimate));
        }
        if self.window_ns.is_some() && !self.no_live_smoothing {
            let next = *self.next_smooth_ns.get_or_insert(t_ns);
            if t_ns >= next && self.measured_ns != self.smoothed_measured_ns {
                self.recent = self.smooth_window().into();
                self.smoothed_measured_ns = self.measured_ns;
                self.next_smooth_ns =
                    Some(t_ns - t_ns.rem_euclid(SMOOTH_EVERY_NS) + SMOOTH_EVERY_NS);
            }
        }
    }

    /// Whether the train can't yet be at a station by a door cycle's closing (its opening is often heard early),
    /// by the track: underground, the doors not just reopened, and a departure within [`DEPART_AFTER_DOORS_S`] of
    /// the last cycle's closing, at a station the radio named, less than the least time a train takes from there
    /// to the next station ago, timed from that closing (away from the station named before, if one was;
    /// see [`crate::track::next_station_m`]).
    /// A door cycle heard then is another sound: mid-river on the L on `20261023-193658`, 7:59:22 pm,
    /// 67 s after leaving 1 Av, 2 km from Bedford Av.
    fn before_next_station(&self, cycle: DoorCycle) -> bool {
        let t_ns = cycle.close_ns;
        // Doors opening again soon after closing, before the train left: the same stop.
        let reopened = self
            .door_cycles
            .last()
            .is_some_and(|c| (cycle.open_ns - c.close_ns) as f64 * 1e-9 <= REOPEN_S);
        if !self.context.in_subway || reopened {
            return false;
        }
        // Departures are found in about time order.
        let Some(&departed) = self.departures.get(
            self.departures
                .partition_point(|&d| d <= t_ns)
                .wrapping_sub(1),
        ) else {
            return false;
        };
        // Only a departure right after the doors closed (surges are also found at stops and mid-run),
        // timed from the closing: a surge is found up to some seconds into the start.
        let Some(departed) = self.door_cycles.last().map(|c| c.close_ns).filter(|&c| {
            let s = (departed - c) as f64 * 1e-9;
            (0.0..=DEPART_AFTER_DOORS_S).contains(&s)
        }) else {
            return false;
        };
        let Some(from) = self.station_at(departed) else {
            return false;
        };
        // The station named before it, among the last few.
        let i = self
            .station_history
            .partition_point(|&(first, ..)| first <= departed);
        let prev = self.station_history[i.saturating_sub(PREV_STATIONS)..i]
            .iter()
            .rev()
            .map(|&(_, _, s)| s)
            .find(|s| !stations::same_complex(from, s));
        let lines = self.riding_lines(departed);
        crate::track::next_station_m(from, prev, &lines)
            .is_some_and(|m| ((t_ns - departed) as f64 * 1e-9) < crate::track::least_s(m))
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
        self.stop_records_since(0).2
    }

    /// [`Self::stop_records`] from record `from` on, clamped to the settled ones, with that index and how many
    /// are settled: those never change, so a caller holding the first `from` keeps them and replaces the rest.
    /// The app asks this at every smoothing tick, so it sends only the unsettled end (see `crates/ffi`).
    pub fn stop_records_since(&self, from: usize) -> (usize, usize, Vec<StopRecord>) {
        let records = self.records();
        let settled = records.settled.records.len();
        let from = from.min(settled);
        let items = records.settled.records[from..]
            .iter()
            .chain(&records.tail)
            .cloned()
            .collect();
        (from, settled, items)
    }

    /// The stop records (see [`Self::stop_records`]): those settled, kept, and the rest recomputed from the inputs
    /// since, a slice of each found from its end, so a status costs the same however long the ride.
    /// A record is settled once the stop after it is [`RECORDS_SETTLE_S`] old: everything it's made from (the next
    /// stop, the steps, door cycles, departures, and car beacons up to it) is found by then.
    fn records(&self) -> Records<'_> {
        let s_ns = |s: f64| (s * 1e9) as i64;
        let lag = s_ns(RECORDS_SETTLE_S);
        let mut settled = self.settled_records.borrow_mut();
        let cut = settled.records.last().map_or(i64::MIN, |r| r.stopped_ns);
        // Inputs from well before the first stop recomputed: its record looks back at the steps before it.
        let from = cut.saturating_sub(lag);
        let stops: Vec<_> = self.train_stops[settled.stops_from..]
            .iter()
            .filter(|(stop, ..)| stop.t_ns > cut)
            .copied()
            .collect();
        let mut tail = stop_records(
            &stops,
            recent(&self.door_cycles, |c| c.close_ns, from, lag),
            recent(&self.departures, |&d| d, from, lag),
            recent(&self.step_windows, |&(t, _)| t, from, lag),
        );
        let readings = recent(&self.car_readings, |&(t, ..)| t, from, lag);
        let span = (CAR_SPAN_S * 1e9) as i64;
        for r in &mut tail {
            if let (Some(b), Some(d), false) = (r.boarded_ns, r.departed_ns, r.stepped_out()) {
                r.boarded_ns = Some(stepped_in(readings, b, d).unwrap_or(b));
            }
            if let (Some(a), Some(b)) = (r.alighted_ns, r.boarded_ns) {
                r.car_before = strongest_car(readings, a - span, a);
                r.car_after = strongest_car(readings, b, b + span);
            }
            if let Some(d) = r.departed_ns {
                r.train_change = trains_differ(readings, r.stopped_ns, d);
            }
        }
        for k in 0..tail.len() {
            let (before, after) = tail.split_at_mut(k);
            let Some(prev) = before.last().or(settled.records.last()) else {
                continue;
            };
            let r = &mut after[0];
            r.between_stations = r.in_subway
                && prev.in_subway
                && !prev.rider_left()
                && r.doors_closed_ns.is_none()
                && self.between_stations(prev.stopped_ns, r.stopped_ns);
        }
        // The legs to the recomputed stops, from the last settled stop a leg can start at, if any.
        let seed = settled.leg_from.clone();
        let fixes_from = seed
            .as_ref()
            .map_or(cut, |a| a.departed_ns.unwrap_or(a.stopped_ns))
            .min(cut);
        let fixes = &self.good_fixes[self.good_fixes.partition_point(|&t| t <= fixes_from)..];
        let seeded: Vec<StopRecord> = seed.into_iter().chain(tail.iter().cloned()).collect();
        let mut tracks = self.tracks.borrow_mut();
        let (legs, _) = crate::track::legs_by(&seeded, fixes, None, |a, b| {
            tracks
                .entry((
                    std::ptr::from_ref(a) as usize,
                    std::ptr::from_ref(b) as usize,
                ))
                .or_insert_with(|| crate::track::between(a, b))
                .clone()
        });
        drop(tracks);
        for leg in legs {
            if let Some(r) = tail.iter_mut().find(|r| r.stopped_ns == leg.to_ns) {
                let length = leg.track.length_m();
                let s = (leg.to_ns - leg.from_ns) as f64 * 1e-9;
                r.from_last_station = Some((length, length / s));
            }
        }
        // Settle those whose next stop is old enough.
        let now = self.settled_ns.unwrap_or(i64::MIN);
        let n = (0..tail.len())
            .take_while(|&k| {
                tail.get(k + 1)
                    .is_some_and(|next| next.stopped_ns < now - lag)
            })
            .count();
        if n > 0 {
            for r in tail.drain(..n) {
                // As `track::legs` goes through them.
                if r.on_platform {
                    settled.leg_from = None;
                }
                if crate::track::anchor(&r) {
                    settled.leg_from = Some(r.clone());
                }
                if r.alighted_ns.is_some() {
                    settled.leg_from = None;
                }
                if r.rider_left() {
                    settled.last_left = Some(settled.records.len());
                }
                settled.records.push(r);
            }
            let cut = settled.records.last().map_or(i64::MIN, |r| r.stopped_ns);
            let mut i = self.train_stops.len();
            while i > settled.stops_from && self.train_stops[i - 1].0.t_ns > cut - lag {
                i -= 1;
            }
            settled.stops_from = i;
        }
        drop(settled);
        Records {
            settled: self.settled_records.borrow(),
            tail,
        }
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

    /// A likely stop found live (see [`train_stops::Event::Likely`]): it holds the live speed at 0 in the subway,
    /// unless it's a departure's pull fading or ending.
    fn on_likely_stop(&mut self, stop: train_stops::Stop) {
        if self.context.in_subway && !self.departure_fade(&stop) && !self.just_departed(stop.t_ns) {
            self.likely_stop = Some(stop);
            self.anchor_at_stop(&stop);
        }
    }

    /// Whether a lurch at `t_ns` comes within [`LIKELY_AFTER_DEPARTURE_S`] after the last departure found.
    fn just_departed(&self, t_ns: i64) -> bool {
        self.departures.last().is_some_and(|&d| {
            let after_s = (t_ns - d) as f64 * 1e-9;
            after_s > 0.0 && after_s <= LIKELY_AFTER_DEPARTURE_S
        })
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
        // Departures are found in time order.
        self.departures
            .get(self.departures.partition_point(|&d| d <= after))
            .copied()
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

    /// A departure at `at` underground from the station named now: the speed is capped along the way to the next
    /// stations on the riding line (see [`LiveSpeedEstimator::start_leg`]), if the way is known.
    fn start_leg(&mut self, at: i64) {
        let Some(station) = self
            .context
            .in_subway
            .then(|| self.station_now(at))
            .flatten()
        else {
            self.end_leg();
            return;
        };
        let prev = self
            .train_stops
            .iter()
            .rev()
            .filter_map(|x| x.1)
            .find(|s| !stations::same_complex(station, s));
        let lines = self.riding_lines(at);
        match crate::track::stations_ahead_m(station, prev, &lines) {
            Some(ahead) => {
                let recorded =
                    crate::track::recorded_peak_mph(station, prev, &lines).map(|mph| mph * 0.44704);
                self.estimator.start_leg(at, &ahead, recorded);
                self.leg_since_ns = Some(at);
                self.leg_from = Some(station);
                self.leg_radio = None;
            }
            None => self.end_leg(),
        }
    }

    fn end_leg(&mut self) {
        self.estimator.end_leg();
        self.leg_since_ns = None;
        self.leg_from = None;
        self.leg_radio = None;
    }

    /// The train stands at `stop` on a leg: if a station other than the departure's is named, it stands that
    /// station's track from the departure's (see [`LiveSpeedEstimator::anchor`]); then the leg ends.
    fn anchor_at_stop(&mut self, stop: &train_stops::Stop) {
        if let (Some(from), Some(at)) = (self.leg_from, self.station_now(stop.t_ns))
            && !stations::same_complex(from, at)
            && let Some(track) = crate::track::between(from, at)
        {
            self.estimator.anchor(stop.t_ns, track.length_m(), 1.0);
        }
        self.end_leg();
    }

    /// On a leg, the radio names a station on the way (a tagged cell, or a network fix at one): the train is
    /// passing it, so that station's track from the departure's is a known place, at half weight for the radio's
    /// lag (cells and network fixes come late or early by up to a station's length). Only once per station, and
    /// only one a train could have reached by now ([`subway_reach_by_time_m`]).
    fn anchor_by_radio(&mut self, t_ns: i64) {
        let (Some(from), Some(since)) = (self.leg_from, self.leg_since_ns) else {
            return;
        };
        let Some(at) = self.station_now(t_ns) else {
            return;
        };
        if stations::same_complex(from, at)
            || self
                .leg_radio
                .is_some_and(|r| stations::same_complex(r, at))
        {
            return;
        }
        self.leg_radio = Some(at);
        let Some(track) = crate::track::between(from, at) else {
            return;
        };
        let since_s = (t_ns - since) as f64 * 1e-9;
        if track.length_m() <= crate::live_speed::subway_reach_by_time_m(since_s) {
            self.estimator
                .anchor(t_ns, track.length_m(), RADIO_ANCHOR_WEIGHT);
        }
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
            // What came while the stop was being decided: door cycles (by closing) and departures are in time
            // order, so found by binary search, not a scan of the ride.
            let cycles = &self.door_cycles[self
                .door_cycles
                .partition_point(|c| c.close_ns <= stop.t_ns)..];
            let departures =
                &self.departures[self.departures.partition_point(|&d| d <= stop.t_ns)..];
            let since: Vec<i64> = cycles
                .iter()
                .map(|c| c.close_ns)
                .chain(departures.iter().copied())
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
            Pending::Fix(f) => {
                let accuracy_m = f.accuracy.unwrap_or(DEFAULT_FIX_ACCURACY_M);
                if accuracy_m <= GAP_FIX_M {
                    self.good_fixes.push(t_ns);
                    self.good_fix_points.push((f.lat, f.lon));
                }
                self.position_inputs.push((
                    t_ns,
                    PositionInput::Fix {
                        lat: f.lat,
                        lon: f.lon,
                        accuracy_m,
                    },
                ))
            }
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

    /// Positions at every fix and sample time so far, smoothed over the whole ride,
    /// and underground joined along the track between stations (see [`crate::track::join`]).
    pub fn positions(&self) -> Vec<PositionEstimate> {
        let mut positions = self.dead_reckoned();
        crate::track::join(
            &mut positions,
            &self.smoothed(),
            &self.stop_records(),
            &self.good_fix_ns(),
            None,
        );
        positions
    }

    /// [`Self::positions`] before joining stations along the track: GNSS and dead reckoning alone.
    pub fn dead_reckoned(&self) -> Vec<PositionEstimate> {
        // Settled inputs are in time order, except any that arrived late.
        let mut inputs = self.position_inputs.clone();
        inputs.sort_by_key(|(t, _)| *t);
        smooth(&inputs)
    }

    /// The times of the phone's fixes within [`GAP_FIX_M`]: where the path is GNSS's own.
    pub fn good_fix_ns(&self) -> Vec<i64> {
        self.position_inputs
            .iter()
            .filter(|(_, i)| {
                matches!(i, PositionInput::Fix { accuracy_m, .. } if *accuracy_m <= GAP_FIX_M)
            })
            .map(|&(t, _)| t)
            .collect()
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
        // That bound is for the IMU alone: with recent GNSS, its speed stands, as for the subway's caps
        // (boarded at 30 Av, `20261103-180153`, the boarding missed, GNSS read 26 to 30 mph held at 0).
        if !self.estimator.gnss_recent(now_ns) && self.transfer_s(now_ns, &self.records()).is_some()
        {
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
        let after = stop.t_ns + (DEPART_AFTER_STOP_S * 1e9) as i64;
        let departed = self.departed_after(stop.t_ns).is_some()
            || self.starts.last().is_some_and(|&s| s > after);
        let since_s = (now_ns - stop.t_ns) as f64 * 1e-9;
        // Once the decision is due, it must have kept this lurch (on `135509` and `215922`, 3 of 34 weren't).
        let lag = (RECORDS_SETTLE_S * 1e9) as i64;
        let dropped = since_s > LIKELY_DECIDED_BY_S
            && !recent(&self.train_stops, |x| x.0.t_ns, stop.t_ns, lag)
                .iter()
                .any(|(s, ..)| s.t_ns == stop.t_ns);
        let old = since_s > RECORD_MAX_DWELL_S;
        // Walking since the stop: the rider is moving (off the train, or along it), so the walking pace shows.
        let walking = self
            .step_windows
            .last()
            .is_some_and(|&(t, steps)| steps && t > stop.t_ns);
        (self.context.in_subway
            && !moving_gnss
            && !departed
            && !dropped
            && !old
            && !walking
            && !self.moved_on(stop.t_ns, now_ns))
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
        // Over a whole ride, a late input goes in its place, after any at the same time, as a stable sort would
        // put it: smoothing then needs no sorted copy of every input.
        if self.window_ns.is_none() && self.inputs.back().is_some_and(|&(t, _)| t > t_ns) {
            let i = self.inputs.partition_point(|&(t, _)| t <= t_ns);
            self.inputs.insert(i, (t_ns, input));
            return;
        }
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

    /// The live map's path underground, as [`crate::track::join`] places a past ride's (without
    /// [`crate::track::blend`]: the live map draws GPS fixes, not dead reckoning), from what starts in the window:
    /// - each leg between stations (see [`crate::track::legs`]), once the train stops at its far station;
    /// - into a tunnel, from the last good fix to the first station stopped at, and out of one, from the
    ///   station to the first good fix, along the track the fix is on (see [`crate::track::to_station`]);
    /// - the leg in progress, toward the next station on the riding line (see [`crate::track::ahead`]),
    ///   by the smoothed speeds' distance since leaving: not held to the track until the train gets there,
    ///   so often short underground; it ends at the station at most.
    ///
    /// The window's start, and each part not yet settled, by its start, with its points in time order:
    /// a caller replaces its parts with these and drops any others starting in the window;
    /// a part that ended before the window never changes again, so isn't given again.
    #[allow(clippy::type_complexity)]
    pub fn recent_track_path(&self) -> (i64, Vec<(i64, Vec<(SmoothedSpeed, (f64, f64))>)>) {
        use crate::track::{Along, Leg, ahead, anchor, to_station};
        let Some(first) = self
            .recent_smoothed()
            .first()
            .map(|s| s.elapsed_realtime_ns)
        else {
            return (i64::MIN, Vec::new());
        };
        // As the hold in `smooth_window`: a leg starts within a dwell of its first stop.
        let from = first - (RECORD_MAX_DWELL_S * 1e9) as i64 - 1;
        // The smoothed speeds from there: frozen, then the window's. Smoothing reaches only the latest measurement,
        // so a part still changes until the window has passed its end.
        let speeds: Vec<SmoothedSpeed> = if self.window_ns.is_some() {
            let i = self
                .frozen
                .partition_point(|s| s.elapsed_realtime_ns < from);
            self.frozen[i..]
                .iter()
                .chain(&self.recent)
                .copied()
                .collect()
        } else {
            self.smooth_window()
        };
        let records: Vec<StopRecord> = self.records().since(from).cloned().collect();
        let k0 = self.good_fixes.partition_point(|&t| t <= from);
        let fixes = &self.good_fixes[k0..];
        let points = &self.good_fix_points[k0..];
        let (legs, _) = crate::track::legs(&records, fixes, None);
        let mut parts: Vec<Leg> = Vec::new();
        let mut ahead_of: Option<(i64, crate::track::Track)> = None;
        // Whether the rider stayed on one train over records `range`: no platform walk before any of them,
        // and none they got off at without boarding a train there (its departure is then the boarded train's).
        let on_board = |range: std::ops::Range<usize>| {
            records[range].iter().all(|x| {
                !x.on_platform
                    && x.alighted_ns
                        .is_none_or(|a| x.departed_ns.is_some_and(|d| d > a))
            })
        };
        let lost_after = |j: usize| {
            fixes
                .get(j + 1)
                .is_none_or(|&n| n - fixes[j] > ride_session::track::MAX_GAP_NS)
        };
        let fix_track = |j: usize, station: &'static stations::Station, lines: Vec<String>| {
            let key = (fixes[j], std::ptr::from_ref(station) as usize, lines);
            let mut cache = self.fix_tracks.borrow_mut();
            if let Some(t) = cache.get(&key) {
                return t.clone();
            }
            let t = to_station(points[j], station, &key.2);
            cache.insert(key, t.clone());
            t
        };
        for (k, r) in records.iter().enumerate() {
            if !anchor(r) {
                continue;
            }
            let station = r.station.unwrap();
            // Into the tunnel: from the last good fix, where GPS was lost (as the map's dashed gaps),
            // if the rider stayed on the train from there.
            let j = fixes.partition_point(|&t| t < r.stopped_ns);
            if !legs.iter().any(|l| l.to_ns == r.stopped_ns)
                && let Some(j) = j.checked_sub(1).filter(|&j| lost_after(j))
            {
                let i = records.partition_point(|x| x.stopped_ns <= fixes[j]);
                // The stop the train stood at when GPS was lost, if it had stopped and not left yet.
                let i = i
                    .checked_sub(1)
                    .filter(|&i| records[i].departed_ns.is_some_and(|d| d > fixes[j]))
                    .unwrap_or(i);
                // Only to the first station after it.
                if on_board(i..k + 1)
                    && !records[i..k]
                        .iter()
                        .any(|x| anchor(x) && x.stopped_ns > fixes[j])
                    && let Some(track) = fix_track(j, station, self.riding_lines(r.stopped_ns))
                {
                    parts.push(Leg {
                        from_ns: fixes[j],
                        to_ns: r.stopped_ns,
                        track,
                    });
                }
            }
            // Out of it, to the first good fix after GPS was lost, or on toward the next station.
            let Some(left) = r.departed_ns else {
                continue;
            };
            if legs.iter().any(|l| l.from_ns == left) {
                continue;
            }
            let j = fixes.partition_point(|&t| t <= left);
            let lines = self.riding_lines(left);
            match fixes.get(j) {
                Some(&t) => {
                    let lost = j.checked_sub(1).is_none_or(lost_after);
                    let until = records.partition_point(|x| x.stopped_ns < t);
                    if lost
                        && on_board(k..until)
                        && let Some(track) = fix_track(j, station, lines.clone())
                    {
                        parts.push(Leg {
                            from_ns: left,
                            to_ns: t,
                            track: track.reversed(),
                        });
                    }
                }
                None if k + 1 == records.len() && on_board(k..k + 1) => {
                    let before = records[..k]
                        .iter()
                        .rev()
                        .filter_map(|x| x.station)
                        .find(|s| !stations::same_complex(station, s));
                    ahead_of = ahead(station, before, &lines).map(|t| (left, t));
                }
                None => {}
            }
        }
        let sample = |from_ns: i64, at: &dyn Fn(&SmoothedSpeed) -> (f64, f64)| {
            // One a second, as `Replay::map_path`, and the last.
            let mut out: Vec<(SmoothedSpeed, (f64, f64))> = Vec::new();
            let mut end = None;
            for s in speeds
                .iter()
                .skip_while(|s| s.elapsed_realtime_ns < from_ns)
            {
                let t = s.elapsed_realtime_ns;
                if out
                    .last()
                    .is_none_or(|(l, _)| t - l.elapsed_realtime_ns >= TRACK_PATH_EVERY_NS)
                {
                    out.push((*s, at(s)));
                    end = None;
                } else {
                    end = Some(*s);
                }
            }
            out.extend(end.map(|s| (s, at(&s))));
            out
        };
        let mut out: Vec<(i64, Vec<(SmoothedSpeed, (f64, f64))>)> = Vec::new();
        for leg in legs.iter().chain(&parts).filter(|l| l.to_ns >= first) {
            let along = Along::new(leg, &speeds);
            let points: Vec<_> = sample(leg.from_ns, &|s| along.at(s.elapsed_realtime_ns))
                .into_iter()
                .take_while(|(s, _)| s.elapsed_realtime_ns <= leg.to_ns)
                .collect();
            out.push((leg.from_ns, points));
        }
        // The leg in progress, by the live speed, which the smoothing hasn't reached yet underground.
        let mut cached = self.ahead_path.borrow_mut();
        match ahead_of {
            None => *cached = None,
            Some((left, track)) => {
                let a = match cached.take() {
                    Some(a) if a.left == left && a.track == track => a,
                    _ => AheadPath {
                        left,
                        next: self.live_speeds.partition_point(|&(t, _)| t < left),
                        scale: self.ahead_scale(&legs, left),
                        track,
                        m: 0.0,
                        prev: None,
                        points: Vec::new(),
                    },
                };
                let a = cached.insert(a);
                // The newest point is the latest speed's, kept only if a second after the one before.
                if a.points.len() > 1
                    && a.points[a.points.len() - 1].0.elapsed_realtime_ns
                        - a.points[a.points.len() - 2].0.elapsed_realtime_ns
                        < TRACK_PATH_EVERY_NS
                {
                    a.points.pop();
                }
                let n = self.live_speeds.len();
                for k in a.next..n {
                    let (t, v) = self.live_speeds[k];
                    if let Some((pt, pv)) = a.prev {
                        a.m += f64::from(pv + v) / 2.0 * (t - pt) as f64 * 1e-9;
                    }
                    a.prev = Some((t, v));
                    let due = a
                        .points
                        .last()
                        .is_none_or(|(l, _)| t - l.elapsed_realtime_ns >= TRACK_PATH_EVERY_NS);
                    if due || k + 1 == n {
                        // The acceleration since the point before, for coloring by it.
                        let accel_mps2 = a.points.last().map_or(0.0, |(l, _)| {
                            (f64::from(v) - l.speed_mps)
                                / ((t - l.elapsed_realtime_ns) as f64 * 1e-9).max(1e-3)
                        });
                        let s = SmoothedSpeed {
                            elapsed_realtime_ns: t,
                            speed_mps: f64::from(v),
                            sigma_mps: f64::NAN,
                            forward_mps: f64::from(v),
                            backward_mps: f64::NAN,
                            accel_mps2,
                        };
                        a.points.push((s, a.track.at(a.m * a.scale)));
                    }
                }
                a.next = n;
                out.retain(|(t, _)| *t != left);
                out.push((left, a.points.clone()));
            }
        }
        out.sort_by_key(|(t, _)| *t);
        (first, out)
    }

    /// How much farther than the live speeds say the train goes: the median over the legs in the window that
    /// ended by `left` of each's track over their distance, within [`AHEAD_SCALE`]; 1 without one.
    /// A stopgap: underground the live speeds' distance is 0 to 2.4 times the track (median 0.76 over 43 legs on
    /// 8 rides, `crates/cli/examples/liveratio.rs`), with no steady factor; to go once the live speed is fixed.
    fn ahead_scale(&self, legs: &[crate::track::Leg], left: i64) -> f64 {
        let mut ratios: Vec<f64> = legs
            .iter()
            .filter(|l| l.to_ns <= left)
            .filter_map(|leg| {
                let i = self.live_speeds.partition_point(|&(t, _)| t < leg.from_ns);
                let j = self.live_speeds.partition_point(|&(t, _)| t <= leg.to_ns);
                let m: f64 = self.live_speeds[i..j]
                    .windows(2)
                    .map(|w| f64::from(w[0].1 + w[1].1) / 2.0 * (w[1].0 - w[0].0) as f64 * 1e-9)
                    .sum();
                (m > 0.0).then(|| leg.track.length_m() / m)
            })
            .collect();
        if ratios.is_empty() {
            return 1.0;
        }
        ratios.sort_by(f64::total_cmp);
        ratios[ratios.len() / 2].clamp(AHEAD_SCALE.0, AHEAD_SCALE.1)
    }

    /// Smoothed speeds at every fix and sample time in the window: the forward estimate there combined with
    /// a backward pass over the window, each weighted by its variance. The backward pass gets no weight until a
    /// measurement initializes it, since it starts from nothing at the window's end.
    fn smooth_window(&self) -> Vec<SmoothedSpeed> {
        let Some(&(end, _)) = self.inputs.back() else {
            return Vec::new();
        };
        // Settled inputs are in time order, except any that arrived late; sort a copy for those.
        // Copied only then: over a whole ride, a copy is a pointer per IMU sample.
        let sorted: Option<Vec<&(i64, Input)>> =
            (!self.inputs.iter().is_sorted_by_key(|(t, _)| *t)).then(|| {
                let mut inputs: Vec<_> = self.inputs.iter().collect();
                inputs.sort_by_key(|(t, _)| *t);
                inputs
            });
        let inputs = || -> Box<dyn DoubleEndedIterator<Item = &(i64, Input)> + '_> {
            match &sorted {
                Some(sorted) => Box::new(sorted.iter().copied()),
                None => Box::new(self.inputs.iter()),
            }
        };
        let mut backward = self.estimator.reversed();
        // Forward samples from the latest back, each paired with the backward estimate once the pass reaches it.
        let mut pending = self.forward.iter().rev().peekable();
        let mut pairs: Vec<(i64, &Estimate, Estimate)> = Vec::with_capacity(self.forward.len());
        // Going back through a door cycle's dwell: the time it began.
        let mut standing_since: Option<i64> = None;
        // The last (earliest so far) time a measurement set the backward speed, and the first pair after it.
        let mut anchor: Option<(i64, usize)> = None;
        for &(t, ref input) in inputs().rev() {
            // Reversed time runs forward from the end; rotation changes sign, acceleration doesn't
            // (see `LiveSpeedEstimator::reversed`).
            let tr = end - t + 1;
            match *input {
                Input::Accel(a) => {
                    let a = a.map(f64::from);
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
                Input::Gyro(w) => {
                    let w = w.map(f64::from);
                    backward.on_gyro(-w[0], -w[1], -w[2])
                }
                Input::Car(ref c) => backward.on_car_accel(tr, c.0, c.1),
                Input::Speed(ref v) => {
                    backward.on_gnss_speed(tr, v.0, v.1);
                    anchor = Some((t, pairs.len()));
                }
                // Going back past a switch, the bound is the one before it.
                Input::MaxSpeedBefore(ref before) => backward.set_max_speed(**before),
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
        let fixes: Vec<&Fix> = inputs()
            .filter_map(|(_, input)| match input {
                Input::Fix(f) if f.accuracy.is_some_and(|a| a <= GAP_FIX_M) => Some(&**f),
                _ => None,
            })
            .collect();
        let steps: Vec<(i64, bool)> = inputs()
            .filter_map(|&(t, ref input)| match *input {
                Input::Walking(running) => Some((t, running)),
                _ => None,
            })
            .collect();
        bound_gaps(&mut out, &fixes, &steps, &self.subway_changes);
        if self.track_hold {
            // Only legs within the window are held, and a leg starts within a dwell of its first stop: the records
            // and good fixes from a dwell before the window, so a tick costs the same however long the ride.
            let from = out.first().map_or(i64::MAX, |s| s.elapsed_realtime_ns)
                - (RECORD_MAX_DWELL_S * 1e9) as i64
                - 1;
            let records: Vec<StopRecord> = self.records().since(from).cloned().collect();
            let fixes = &self.good_fixes[self.good_fixes.partition_point(|&t| t <= from)..];
            let (legs, _) = crate::track::legs(&records, fixes, None);
            crate::track::hold_to_track(&mut out, &legs);
        }
        accel_from_speeds(&mut out);
        out
    }
}

/// Half the span (ns) each smoothed acceleration is the shown speed's slope over (see [`accel_from_speeds`]):
/// over a second, as the samples are about a second apart (a little over: 1.001 s), to reach one each side.
const ACCEL_HALF_SPAN_NS: i64 = 1_500_000_000;

/// Sets each smoothed speed's acceleration to the slope of the speeds themselves, as shown, from
/// [`ACCEL_HALF_SPAN_NS`] before to as long after (or as far as the speeds go).
/// Not the passes' own rates combined: their weights change along the way, and the speeds are then bounded
/// and held to the track, so that combination rose while the shown speed fell braking into a station
/// (item 19, 18 St on `20261029-012337`: up to +2.2 m/s^2 while the speed fell from 18 to 4 mph).
fn accel_from_speeds(out: &mut [SmoothedSpeed]) {
    let speeds: Vec<(i64, f64)> = out
        .iter()
        .map(|s| (s.elapsed_realtime_ns, s.speed_mps))
        .collect();
    let (mut lo, mut hi) = (0, 0);
    for (i, s) in out.iter_mut().enumerate() {
        let t = speeds[i].0;
        while speeds[lo].0 < t - ACCEL_HALF_SPAN_NS {
            lo += 1;
        }
        hi = hi.max(i);
        while hi + 1 < speeds.len() && speeds[hi + 1].0 <= t + ACCEL_HALF_SPAN_NS {
            hi += 1;
        }
        let ((ta, va), (tb, vb)) = (speeds[lo], speeds[hi]);
        s.accel_mps2 = if tb > ta {
            (vb - va) / ((tb - ta) as f64 * 1e-9)
        } else {
            0.0
        };
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

/// How old (s) the stop after a stop record must be for the record to be settled (see [`Fusion::records`]):
/// longer than anything it's made from takes to be found (a missed stop is found at the departure after it,
/// within [`RECORD_MAX_DWELL_S`]).
const RECORDS_SETTLE_S: f64 = 900.0;

/// The stop records settled so far, and the bookmarks into the inputs for the rest (see [`Fusion::records`]).
#[derive(Default)]
struct SettledRecords {
    records: Vec<StopRecord>,
    /// The index of the last of them the rider left the train at.
    last_left: Option<usize>,
    /// The last of them a leg between stations can start at, if one can (see [`crate::track::legs`]).
    leg_from: Option<StopRecord>,
    /// The train stops before this index are all of settled records' stops, or fades long before them.
    stops_from: usize,
}

/// The timeline (see [`Fusion::timeline`]) up to `until`, its segments joined: settled, but for the last,
/// which those after may join.
struct TimelineKept {
    joined: Vec<Segment>,
    until: i64,
}

/// The subway's periods (see [`Fusion::subway_periods`]) from the detector's switches before `upto`:
/// those closed, and the start of the open one. The last closed one reopens when the detector is back within a minute.
#[derive(Default)]
struct SubwayPeriods {
    closed: Vec<(i64, i64)>,
    since: Option<i64>,
    upto: usize,
}

/// The walks (start, end) from the step windows before `upto` (see [`Fusion::timeline`]).
#[derive(Default)]
struct Walks {
    walks: Vec<(i64, i64)>,
    upto: usize,
}

/// Joins `pieces`, in time order, onto `joined` (see [`Fusion::timeline`]): pieces left by a walk too short to
/// show, or a flicker, join up.
fn join_segments(joined: &mut Vec<Segment>, pieces: Vec<Segment>) {
    let gap = (TIMELINE_SUBWAY_GAP_S * 1e9) as i64;
    for s in pieces {
        match joined.last_mut() {
            Some(last) if last.doing == s.doing && s.start_ns - last.end_ns <= gap => {
                last.end_ns = last.end_ns.max(s.end_ns);
            }
            _ => joined.push(s),
        }
    }
}

/// The lines ridden (see [`Fusion::riding_lines`]) as of the train stops before `upto`, the last of them
/// with a station at `prev`, and the latest of their times.
struct RidingLines {
    lines: Vec<String>,
    prev: i64,
    max_t: i64,
    upto: usize,
}

/// The stop records (see [`Fusion::records`]): those settled, then the rest.
struct Records<'a> {
    settled: std::cell::Ref<'a, SettledRecords>,
    tail: Vec<StopRecord>,
}

impl Records<'_> {
    /// The last record the rider left the train at.
    fn last_left(&self) -> Option<&StopRecord> {
        self.tail
            .iter()
            .rfind(|r| r.rider_left())
            .or_else(|| self.settled.last_left.map(|i| &self.settled.records[i]))
    }

    /// The records of stops at or before `t_ns`, newest first.
    fn before(&self, t_ns: i64) -> impl Iterator<Item = &StopRecord> {
        let i = self
            .settled
            .records
            .partition_point(|r| r.stopped_ns <= t_ns);
        self.tail
            .iter()
            .rev()
            .filter(move |r| r.stopped_ns <= t_ns)
            .chain(self.settled.records[..i].iter().rev())
    }

    /// The records of stops after `t_ns`.
    fn since(&self, t_ns: i64) -> impl Iterator<Item = &StopRecord> {
        let i = self
            .settled
            .records
            .partition_point(|r| r.stopped_ns <= t_ns);
        self.settled.records[i..]
            .iter()
            .chain(self.tail.iter().filter(move |r| r.stopped_ns > t_ns))
    }
}

/// The index in `v`, in about time order by `time` (none more than `lag` out), from which on are all those after
/// `from`: found from the end, so it costs what's after it.
fn recent_index<T>(v: &[T], time: impl Fn(&T) -> i64, from: i64, lag: i64) -> usize {
    let mut i = v.len();
    while i > 0 && time(&v[i - 1]) > from.saturating_sub(lag) {
        i -= 1;
    }
    i
}

/// Those of `v` from [`recent_index`] on.
fn recent<T>(v: &[T], time: impl Fn(&T) -> i64, from: i64, lag: i64) -> &[T] {
    &v[recent_index(v, time, from, lag)..]
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
        // A lurch just after a departure is its pull ending (see [`LIKELY_AFTER_DEPARTURE_S`]), not a stop to walk
        // off at: a walk along the M across the Williamsburg Bridge 245 s after one (`20261105-220621`, 10:25 pm)
        // was taken for walking off, and held the live speed at 0 as a transfer to Bowery at 44 mph.
        let pull_ending = departures.iter().any(|&d| {
            let after_s = (stop.t_ns - d) as f64 * 1e-9;
            after_s > 0.0 && after_s <= LIKELY_AFTER_DEPARTURE_S
        });
        // Walking off: enough steps in a row, starting before the train's departure.
        let alighted = (!on_platform && !pull_ending)
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
            .filter(|&a| !carried_on(a))
            .or_else(|| {
                // Left without a walk off seen (few steps, in a crowd): the next stop is another train's, met on
                // the platform, and this one left unfelt; off at the first steps (74 St, `20261020-184134`, 7:09 pm).
                let (next, ..) = stops.get(k + 1)?;
                let first = after.iter().find(|&&(t, steps)| steps && t < next.t_ns)?.0 - window_ns;
                (!on_platform
                    && !pull_ending
                    && ridden_departure.is_none()
                    && on_platform_at(next.t_ns)
                    && !departures.iter().any(|&d| d > stop.t_ns && d < next.t_ns))
                .then_some(first)
            });
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
            from_last_station: None,
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
    // Step windows and departures are in time order: binary searches, so a call costs the same late in a ride.
    let windows = &step_windows[step_windows.partition_point(|&(t, _)| t <= since)
        ..step_windows.partition_point(|&(t, _)| t <= departure)];
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
    let rode_from = departures[..departures.partition_point(|&d| d < walk_start)]
        .last()
        .copied()
        .filter(|&d| d > walk_start - s_ns(RECORD_MAX_DWELL_S))?;
    // Stops are in the order found, a missed one at most a dwell and a walk after its time: those found since.
    let from = rode_from.min(walk_start - s_ns(MISSED_STOP_LOOKBACK_S));
    let found = recent(
        train_stops,
        |x| x.0.t_ns,
        from,
        s_ns(MISSED_STOP_DISORDER_S),
    )
    .iter()
    .any(|(s, ..)| s.t_ns > from && s.t_ns < departure);
    let departed_between = departures[departures.partition_point(|&d| d <= stop_ns)..]
        .first()
        .is_some_and(|&d| d < departure);
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
    // Door cycles are found at their closings, in time order: those closing just before the braking.
    let s_ns = |s: f64| (s * 1e9) as i64;
    let from = stop.braking_since_ns - s_ns(FADE_AFTER_CLOSE_S.1) - 1;
    let until = stop.braking_since_ns - s_ns(FADE_AFTER_CLOSE_S.0) + 1;
    braking_s >= FADE_MIN_S
        && door_cycles[door_cycles.partition_point(|c| c.close_ns < from)..]
            .iter()
            .take_while(|c| c.close_ns <= until)
            .any(|c| {
                let after = (stop.braking_since_ns - c.close_ns) as f64 * 1e-9;
                after >= FADE_AFTER_CLOSE_S.0 && after <= FADE_AFTER_CLOSE_S.1
            })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn braking_reads_negative_whatever_the_passes_rates() {
        // Item 19: the shown speed falls into a station while the passes' rates said speeding up.
        let mut out: Vec<SmoothedSpeed> = (0..=20)
            .map(|i| SmoothedSpeed {
                elapsed_realtime_ns: i * 1_000_000_000,
                speed_mps: 20.0 - i as f64,
                sigma_mps: 1.0,
                forward_mps: 20.0,
                backward_mps: 0.0,
                accel_mps2: 2.0,
            })
            .collect();
        accel_from_speeds(&mut out);
        for s in &out {
            assert!((s.accel_mps2 + 1.0).abs() < 1e-9, "{s:?}");
        }
    }

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

    #[test]
    fn no_likely_stop_just_after_departing() {
        let mut f = Fusion::whole_ride();
        f.context.in_subway = true;
        f.departures.push(100 * S);
        let lurch = |t_s: i64| train_stops::Stop {
            t_ns: t_s * S,
            score: 1.0,
            braking_since_ns: (t_s - 4) * S,
        };
        // Motoring cut off 12 s after leaving, no closing heard: not a stop, so the live speed isn't held.
        f.on_likely_stop(lurch(112));
        assert_eq!(f.likely_stop_now(113 * S), None);
        // Later in the leg, a lurch still holds it.
        f.on_likely_stop(lurch(150));
        assert_eq!(f.likely_stop_now(151 * S), Some(lurch(150)));
        assert!(f.estimate(151 * S).stopped);
    }

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

    /// A caller's copy of a list, kept by the deltas of [`Fusion::stop_records_since`] or [`Fusion::timeline_since`].
    struct Mirror<T> {
        items: Vec<T>,
        settled: usize,
    }

    impl<T: Clone + PartialEq + std::fmt::Debug> Mirror<T> {
        fn apply(&mut self, (from, settled, items): (usize, usize, Vec<T>)) {
            self.items.truncate(from);
            self.items.extend(items);
            self.settled = settled;
        }
    }

    #[test]
    fn deltas_rebuild_the_lists() {
        // Two hours: rides of ten stops two minutes apart, then a walk of five minutes, again and again;
        // a tick every 10 s, as the app's smoothing tick asks, and a caller that misses ticks.
        let mut f = Fusion::whole_ride();
        let ride_s = 10 * 120 + 300;
        let mut every = (
            Mirror {
                items: Vec::new(),
                settled: 0,
            },
            Mirror {
                items: Vec::new(),
                settled: 0,
            },
        );
        let mut sparse = (
            Mirror {
                items: Vec::new(),
                settled: 0,
            },
            Mirror {
                items: Vec::new(),
                settled: 0,
            },
        );
        for tick in 1..=720i64 {
            let now = tick * 10;
            for t in (now - 10..now).filter(|t| t % 4 == 0) {
                let walking = t % ride_s >= 10 * 120;
                f.step_windows.push((t * S, walking));
                if t % ride_s == 0 {
                    f.subway_changes.push((t * S, true));
                }
                if t % ride_s == 10 * 120 {
                    f.subway_changes.push((t * S, false));
                }
            }
            for t in now - 10..now {
                let phase = t % ride_s;
                if phase < 10 * 120 && phase % 120 == 60 {
                    f.train_stops.push(stop(t));
                }
                if phase < 10 * 120 && phase % 120 == 100 {
                    f.departures.push(t * S);
                }
            }
            f.now_ns = now * S;
            f.settled_ns = Some(now * S);
            let stops = f.stop_records();
            let timeline = f.timeline();
            every.0.apply(f.stop_records_since(every.0.settled));
            every.1.apply(f.timeline_since(every.1.settled));
            assert_eq!(every.0.items, stops, "stops at {now} s");
            assert_eq!(every.1.items, timeline, "timeline at {now} s");
            if tick % 7 == 0 {
                sparse.0.apply(f.stop_records_since(sparse.0.settled));
                sparse.1.apply(f.timeline_since(sparse.1.settled));
                assert_eq!(sparse.0.items, stops, "stops at {now} s, every 7th tick");
                assert_eq!(
                    sparse.1.items, timeline,
                    "timeline at {now} s, every 7th tick"
                );
            }
        }
        // Most of both settled, so the deltas were short.
        assert!(every.0.settled > 20, "{} stops settled", every.0.settled);
        assert!(every.1.settled > 4, "{} segments settled", every.1.settled);
        assert!(f.stop_records_since(every.0.settled).2.len() < 10);
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
    fn no_walking_off_at_a_departures_pull_ending() {
        // The M over the Williamsburg Bridge (`20261105-220621`, 10:25 pm): a lurch 13 s after leaving, then a
        // walk along the train from 245 s later; the next stop is 570 s on. Not a walk off, so no transfer.
        let r = stop_records(
            &[stop(100), stop(670)],
            &[],
            &[87 * S],
            &windows(0, 700, 345..470),
        );
        assert!(!r[0].rider_left());
        assert_eq!(r[0].alighted_ns, None);
        // The same walk after a stop well into the leg is walking off.
        let r = stop_records(
            &[stop(100), stop(670)],
            &[],
            &[60 * S],
            &windows(0, 700, 345..470),
        );
        assert!(r[0].rider_left());
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
