//! Replays a recorded session through [`Fusion`], fed exactly as the app feeds it live,
//! then smooths over the whole ride: live smooths only recent input, as it arrives, which can only be less accurate.

use ride_gnss::solve::Stats;
use ride_session::proto::record::Payload;
use ride_session::proto::{AudioChunk, SensorChunk};
use ride_session::read::{Session, StreamContents, read_stream_file};
use ride_session::track::GPS_PROVIDER;

use crate::doors::DoorCycle;
use crate::fusion::{AudioSamples, Fix, Fusion, ImuSample, SmoothedSpeed, imu_sample};

/// What a replay produced.
pub struct Replay {
    /// Smoothed speed at every GPS fix and every second.
    pub smoothed: Vec<SmoothedSpeed>,
    /// Navigation messages, ephemerides, and epochs solved from raw GNSS.
    pub gnss: Stats,
    /// Smoothed position at every sample (see [`crate::position`]).
    pub positions: Vec<crate::position::PositionEstimate>,
    /// The same before joining stations along the track (see [`Fusion::dead_reckoned`]), for scoring.
    pub dead_reckoned: Vec<crate::position::PositionEstimate>,
    /// The times of good fixes (see [`Fusion::good_fix_ns`]).
    pub good_fix_ns: Vec<i64>,
    /// Door cycles heard, if the session has audio.
    pub door_cycles: Vec<DoorCycle>,
    /// Door chimes a closing confirmed, with their pitch family.
    pub chimes: Vec<(i64, crate::doors::ChimeCars)>,
    /// The train's stops, with dwell and the doors' times (see [`crate::fusion::StopRecord`]).
    pub stops: Vec<crate::fusion::StopRecord>,
    /// The train's start surges (see [`crate::stops`]).
    pub departures: Vec<i64>,
    /// Each step-rhythm window's end, and whether it had steps.
    pub step_windows: Vec<(i64, bool)>,
    /// When the subway detector switched on (true) or off.
    pub subway_changes: Vec<(i64, bool)>,
    /// What the app would have shown, every [`STATUS_EVERY_NS`].
    pub statuses: Vec<(i64, crate::fusion::Status)>,
    /// How long each of [`Self::statuses`] took to make (ns), for checking its cost stays the same over a ride.
    pub status_cost_ns: Vec<u64>,
    /// How long feeding the inputs since the status before each of [`Self::statuses`] took (ns),
    /// smoothing included: for checking the cost per input stays the same over a ride.
    pub feed_cost_ns: Vec<u64>,
    /// Each smoothing tick's time and how long it took (ns), every [`Options::tick_every_ns`]:
    /// what the app asks then (see `LiveSpeed.kt`), holding the settled counts as it does.
    pub tick_cost_ns: Vec<(i64, u64)>,
    /// What the rider did over the ride (see [`Fusion::timeline`]), as of its end.
    pub timeline: Vec<crate::fusion::Segment>,
    /// Stairs, elevators, and escalators (see [`crate::vertical`]).
    pub vertical: Vec<crate::vertical::Move>,
    /// The live estimate the app would have shown (see [`Fusion::estimate`]), every [`Options::live_every_ns`].
    pub live: Vec<(i64, crate::live_speed::Estimate)>,
}

pub const STATUS_EVERY_NS: i64 = 10_000_000_000;

impl Replay {
    /// The smoothed speed at a fix's time, if the replay has one.
    pub fn at(&self, elapsed_realtime_ns: i64) -> Option<&SmoothedSpeed> {
        let i = self
            .smoothed
            .partition_point(|s| s.elapsed_realtime_ns < elapsed_realtime_ns);
        self.smoothed
            .get(i)
            .filter(|s| s.elapsed_realtime_ns == elapsed_realtime_ns)
    }

    /// The smoothed speed nearest in time, if within 2 s.
    pub fn nearest(&self, elapsed_realtime_ns: i64) -> Option<&SmoothedSpeed> {
        let i = self
            .smoothed
            .partition_point(|s| s.elapsed_realtime_ns < elapsed_realtime_ns);
        [i.checked_sub(1), Some(i)]
            .into_iter()
            .flatten()
            .filter_map(|i| self.smoothed.get(i))
            .min_by_key(|s| (s.elapsed_realtime_ns - elapsed_realtime_ns).abs())
            .filter(|s| (s.elapsed_realtime_ns - elapsed_realtime_ns).abs() <= 2_000_000_000)
    }

    /// [`Self::positions`] for a map: at most one every `every_ns` (and the last),
    /// each with the smoothed speed nearest it (see [`Self::nearest`]).
    pub fn map_path(
        &self,
        every_ns: i64,
    ) -> Vec<(&crate::position::PositionEstimate, Option<&SmoothedSpeed>)> {
        let mut out = Vec::new();
        let mut last_ns: Option<i64> = None;
        for (i, p) in self.positions.iter().enumerate() {
            let t = p.elapsed_realtime_ns;
            if last_ns.is_some_and(|l| t - l < every_ns) && i + 1 < self.positions.len() {
                continue;
            }
            last_ns = Some(t);
            out.push((p, self.nearest(t)));
        }
        out
    }
}

enum Event {
    Accel([f64; 3]),
    Gyro([f64; 3]),
    Mag([f64; 3]),
    Pressure(f64),
    Fix(Box<Fix>),
    Measurements(Box<ride_session::proto::GnssMeasurementsEvent>),
    Navigation(Box<ride_session::proto::GnssNavigationMessage>),
    WifiScan(Vec<String>),
    Cell(Option<i64>),
    NetworkFix(Box<(f64, f64, Option<f64>)>),
    Audio(u32, Box<AudioSamples>),
    Ble(Box<(Vec<u8>, i32)>),
    /// A GTFS-realtime response body that came back, and when it was requested (unix ms).
    Transit(Box<(Vec<u8>, i64)>),
}

/// The app feeds audio to fusion as it reads it, this many frames at a time (`AudioStreamRecorder.kt`).
const AUDIO_READ_FRAMES: usize = 1600;

/// Uses the uncalibrated accelerometer and gyro if recorded, else the calibrated ones, like the app.
/// Every input is merged by time; live, they arrive slightly out of order, so the replay can differ a little
/// from what the phone showed, never by design.
pub fn replay(session: &Session) -> anyhow::Result<Replay> {
    replay_with(session, &Options::default())
}

/// What the app is told about a ride, beyond what it records.
#[derive(Clone, Debug, Default)]
pub struct Options {
    /// See [`Fusion::set_max_speed`].
    pub max_speed_mps: Option<f64>,
    /// For testing only: GNSS (fixes and raw measurements) cut out as in a tunnel, see [`GnssOutages`].
    pub gnss_outages: Option<GnssOutages>,
    /// For comparison only: leave the audio out, as before door cycles were heard.
    pub ignore_audio: bool,
    /// For scoring only: leave the smoothed speeds unheld to the track (see [`Fusion::set_track_hold`]).
    pub no_track_hold: bool,
    /// Leave out [`Replay::statuses`], for what doesn't show them: about a fifth of a replay's time.
    pub no_statuses: bool,
    /// For scoring: keep [`Replay::live`] this often (ns), if set.
    pub live_every_ns: Option<i64>,
    /// For scoring: leave out where the phone sat still (see [`still_cuts`]), as a gap in the recording.
    pub skip_still: bool,
    /// Replay as the phone does ([`Fusion::new`]), smoothing only a recent window as input arrives,
    /// instead of the whole ride at once.
    pub live: bool,
    /// For checking costs: ask what the app asks at each smoothing tick this often (ns), timing each.
    pub tick_every_ns: Option<i64>,
}

/// How long the GPS fixes must stay within [`STILL_RADIUS_M`] of a stretch's first for it to be still.
pub const STILL_MIN_NS: i64 = 600_000_000_000;
pub const STILL_RADIUS_M: f64 = 100.0;
/// How much of each end of a still stretch is kept.
pub const STILL_KEEP_NS: i64 = 60_000_000_000;

/// Where the phone sat still: stretches of [`STILL_MIN_NS`] or more whose GPS fixes all stay within
/// [`STILL_RADIUS_M`] of the stretch's first, each less its first and last [`STILL_KEEP_NS`], as (start, end) ns.
/// Nothing happens there, and leaving it out speeds up scoring: an hour and a half on `20261022-002914`.
pub fn still_cuts(session: &Session) -> anyhow::Result<Vec<(i64, i64)>> {
    if !session.stream_path("location").exists() {
        return Ok(Vec::new());
    }
    let fixes: Vec<_> =
        ride_session::track::gps_segments(&session.read_records("location")?.chunks)
            .into_iter()
            .flatten()
            .collect();
    let mut cuts = Vec::new();
    let mut i = 0;
    while i < fixes.len() {
        let a = &fixes[i];
        let mut j = i;
        while j + 1 < fixes.len() {
            let b = &fixes[j + 1];
            let m_per_deg = 111_320.0;
            let dx = (b.longitude_degrees - a.longitude_degrees)
                * m_per_deg
                * a.latitude_degrees.to_radians().cos();
            let dy = (b.latitude_degrees - a.latitude_degrees) * m_per_deg;
            if dx.hypot(dy) > STILL_RADIUS_M {
                break;
            }
            j += 1;
        }
        let (start, end) = (a.elapsed_realtime_ns, fixes[j].elapsed_realtime_ns);
        if end - start >= STILL_MIN_NS {
            cuts.push((start + STILL_KEEP_NS, end - STILL_KEEP_NS));
        }
        i = j + 1;
    }
    Ok(cuts)
}

/// Whether `t_ns` falls in one of `cuts` (sorted, disjoint).
fn in_cuts(cuts: &[(i64, i64)], t_ns: i64) -> bool {
    let i = cuts.partition_point(|c| c.1 < t_ns);
    cuts.get(i).is_some_and(|c| c.0 <= t_ns)
}

/// GNSS cut out for the last `outage_s` of every `period_s` from `start_ns`, to score dead reckoning on rides
/// that had it throughout.
#[derive(Clone, Copy, Debug)]
pub struct GnssOutages {
    pub start_ns: i64,
    pub outage_s: f64,
    pub period_s: f64,
}

impl GnssOutages {
    fn covers(&self, t_ns: i64) -> bool {
        let s = (t_ns - self.start_ns) as f64 * 1e-9;
        s >= 0.0 && s.rem_euclid(self.period_s) >= self.period_s - self.outage_s
    }
}

pub fn replay_with(session: &Session, options: &Options) -> anyhow::Result<Replay> {
    replay_with_progress(session, options, &mut |_| {})
}

/// [`replay_with`], telling `progress` the fraction of the ride's inputs fed so far, every 1%:
/// reading the session comes before the first call, smoothing the whole ride after the last.
pub fn replay_with_progress(
    session: &Session,
    options: &Options,
    progress: &mut dyn FnMut(f64),
) -> anyhow::Result<Replay> {
    replay_with_hook(session, options, progress, &mut |_, _| {})
}

/// [`replay_with_progress`], also handing `on_status` the fusion at each status's time (with statuses on),
/// right after the status: for checking what the app asks at its ticks.
pub fn replay_with_hook(
    session: &Session,
    options: &Options,
    progress: &mut dyn FnMut(f64),
    on_status: &mut dyn FnMut(i64, &Fusion),
) -> anyhow::Result<Replay> {
    // Every input is merged by time, at equal times in this order: the IMU, pressure, records, audio.
    // The big streams are read a chunk at a time as they're fed:
    // reading a long ride's whole streams and collecting their events first took over a gigabyte.
    // The recorded streams the app feeds live, preferring uncalibrated (see `RecorderService.kt`).
    let mut sensors: Vec<(Option<i32>, &str)> = Vec::new();
    for (name, fallback) in [
        ("accel_uncal", "accel"),
        ("gyro_uncal", "gyro"),
        ("mag_uncal", "mag"),
    ] {
        let info = |n: &str| session.info.streams.iter().find(|s| s.name == n);
        let Some(stream) = info(name).or_else(|| info(fallback)) else {
            continue;
        };
        let Some(sensor_type) = stream.sensor.as_ref().map(|s| s.r#type) else {
            continue;
        };
        sensors.push((Some(sensor_type), &stream.name));
    }
    if session.info.streams.iter().any(|s| s.name == "pressure") {
        sensors.push((None, "pressure"));
    }
    let mut records: Vec<(i64, Event)> = Vec::new();
    for stream in ["location", "gnss", "radio", ride_session::transit::STREAM] {
        if !session.stream_path(stream).exists() {
            continue;
        }
        for r in session
            .read_records(stream)?
            .chunks
            .into_iter()
            .flat_map(|c| c.records)
        {
            let received = r.received_elapsed_realtime_ns;
            match r.payload {
                Some(Payload::Location(l)) if l.provider == GPS_PROVIDER && !l.is_mock => {
                    let fix = Fix {
                        elapsed_realtime_ns: l.elapsed_realtime_nanos,
                        lat: l.latitude_degrees,
                        lon: l.longitude_degrees,
                        altitude: l.altitude_meters,
                        accuracy: l.horizontal_accuracy_meters.map(f64::from),
                        speed: l.speed_meters_per_second.map(f64::from),
                        speed_sigma: l.speed_accuracy_meters_per_second.map(f64::from),
                    };
                    records.push((fix.elapsed_realtime_ns, Event::Fix(Box::new(fix))));
                }
                // Android's own location, as the app passes it on.
                Some(Payload::Location(l)) if l.provider == "fused" && !l.is_mock => {
                    records.push((
                        l.elapsed_realtime_nanos,
                        Event::NetworkFix(Box::new((
                            l.latitude_degrees,
                            l.longitude_degrees,
                            l.horizontal_accuracy_meters.map(f64::from),
                        ))),
                    ))
                }
                Some(Payload::GnssMeasurements(m)) => {
                    let t = m
                        .clock
                        .as_ref()
                        .and_then(|c| c.elapsed_realtime_nanos)
                        .unwrap_or(received);
                    records.push((t, Event::Measurements(Box::new(m))));
                }
                Some(Payload::GnssNavigationMessage(m)) => {
                    records.push((received, Event::Navigation(Box::new(m))))
                }
                // At the time the app recorded it, which is also when it fed it live.
                Some(Payload::WifiScan(w)) => records.push((
                    received,
                    Event::WifiScan(w.results.into_iter().map(|r| r.ssid).collect()),
                )),
                // Bluetooth advertisements, at each result's own time; the subway cars' beacons are used.
                Some(Payload::BleScan(b)) => {
                    for r in b.results {
                        records.push((
                            r.timestamp_elapsed_realtime_ns,
                            Event::Ble(Box::new((r.advertising_data, r.rssi))),
                        ));
                    }
                }
                // The MTA's feeds, when each response arrived, as the app passes them on.
                Some(Payload::GtfsRealtime(f)) if f.http_status == 200 => records.push((
                    received,
                    Event::Transit(Box::new((f.body, f.request_unix_ms))),
                )),
                // The registered LTE cell, as the app passes it on.
                Some(Payload::CellInfo(c)) => records.push((
                    received,
                    Event::Cell(
                        c.cells
                            .iter()
                            .find(|x| x.registered && x.kind == "lte")
                            .and_then(|x| x.cell_id),
                    ),
                )),
                _ => {}
            }
        }
    }
    // Audio as the app reads it: each read arrives when its last frame was recorded.
    let audio_rate = session
        .info
        .streams
        .iter()
        .find(|s| s.name == "audio")
        .and_then(|s| s.audio.as_ref())
        .map(|a| a.sample_rate_hz as u32);
    // Stable, like the merge: records at equal times stay in the order read.
    records.sort_by_key(|(t, _)| *t);
    let cuts = if options.skip_still {
        still_cuts(session)?
    } else {
        Vec::new()
    };
    records.retain(|(t, _)| !in_cuts(&cuts, *t));
    if let Some(cut) = options.gnss_outages {
        records.retain(|(t, e)| {
            !(matches!(e, Event::Fix(_) | Event::Measurements(_)) && cut.covers(*t))
        });
    }

    let mut sources: Vec<Source> = Vec::new();
    for &(sensor_type, name) in &sensors {
        sources.push(in_order(
            || {
                Ok(Box::new(
                    sensor_events(sensor_type, session.stream_chunks(name)?)
                        .filter(|e| e.as_ref().map_or(true, |(t, _)| !in_cuts(&cuts, *t))),
                ))
            },
            || {
                sensor_events(sensor_type, chunks_of(session.read_stream(name)?))
                    .filter(|e| e.as_ref().map_or(true, |(t, _)| !in_cuts(&cuts, *t)))
                    .collect()
            },
        )?);
    }
    let records_len = records.len();
    sources.push((records_len, Box::new(records.into_iter())));
    if let Some(rate) = audio_rate.filter(|_| !options.ignore_audio)
        && session.stream_path("audio").exists()
    {
        // Still FLAC where recompressed: each chunk is decoded only as it's fed.
        let path = session.stream_path("audio");
        sources.push(in_order(
            || {
                Ok(Box::new(audio_events(
                    rate,
                    session.stream_chunks("audio")?,
                    &cuts,
                )))
            },
            || audio_events(rate, chunks_of(read_stream_file(&path)?), &cuts).collect(),
        )?);
    }
    let total = sources.iter().map(|s| s.0).sum::<usize>().max(1);
    // Fusion keeps every accelerometer and gyro sample (the first sources), and some records.
    let buffered: usize = sources
        .iter()
        .zip(&sensors)
        .filter(|(_, (sensor_type, _))| {
            matches!(
                sensor_type.and_then(|t| imu_sample(t, &[0.0; 6])),
                Some(ImuSample::Accel(_) | ImuSample::Gyro(_))
            )
        })
        .map(|(s, _)| s.0)
        .sum::<usize>()
        + records_len;
    let events = merge(sources.into_iter().map(|s| s.1).collect());

    let mut fusion = if options.live {
        Fusion::new()
    } else {
        Fusion::whole_ride()
    };
    fusion.reserve_inputs(buffered);
    fusion.set_max_speed(options.max_speed_mps);
    fusion.set_track_hold(!options.no_track_hold);
    // Downloaded orbits (`just orbits`), for constellations whose navigation messages the phone missed.
    for eph in ride_gnss::orbits::read_session_orbits(&session.dir)? {
        fusion.navigator_mut().add_ephemeris(eph);
    }
    let mut statuses = Vec::new();
    let mut status_cost_ns = Vec::new();
    let mut feed_cost_ns = Vec::new();
    let mut tick_cost_ns = Vec::new();
    let mut live = Vec::new();
    let mut events = events.peekable();
    let mut next_status = events.peek().map_or(0, |e| e.0);
    let mut next_live = next_status;
    let mut next_tick = next_status;
    // The app's copies' settled counts (see [`Fusion::stop_records_since`], [`Fusion::timeline_since`]).
    let mut stops_settled = 0;
    let mut timeline_settled = 0;
    let mut fed = std::time::Instant::now();
    let every = total.div_ceil(100);
    progress(0.0);
    for (i, (t, e)) in events.enumerate() {
        if i % every == 0 && i > 0 {
            progress(i as f64 / total as f64);
        }
        if t >= next_status && !options.no_statuses {
            feed_cost_ns.push(fed.elapsed().as_nanos() as u64);
            let start = std::time::Instant::now();
            statuses.push((t, fusion.status()));
            status_cost_ns.push(start.elapsed().as_nanos() as u64);
            on_status(t, &fusion);
            next_status = t + STATUS_EVERY_NS;
            fed = std::time::Instant::now();
        }
        if let Some(every) = options.tick_every_ns
            && t >= next_tick
        {
            let start = std::time::Instant::now();
            std::hint::black_box(fusion.recent_smoothed());
            stops_settled = fusion.stop_records_since(stops_settled).1;
            timeline_settled = fusion.timeline_since(timeline_settled).1;
            tick_cost_ns.push((t, start.elapsed().as_nanos() as u64));
            next_tick = t + every;
            fed = fed.checked_add(start.elapsed()).unwrap_or(fed);
        }
        if let Some(every) = options.live_every_ns
            && t >= next_live
        {
            live.push((t, fusion.estimate(t)));
            next_live = t + every;
        }
        match e {
            Event::Accel(a) => fusion.on_accel(t, a),
            Event::Gyro(w) => fusion.on_gyro(t, w),
            Event::Mag(b) => fusion.on_magnetometer(t, b),
            Event::Pressure(p) => fusion.on_pressure(t, p),
            Event::Fix(fix) => fusion.on_fix(*fix),
            Event::Measurements(m) => {
                fusion.on_gnss_measurements(t, &m);
            }
            Event::Navigation(m) => fusion.on_navigation_message(&m),
            Event::WifiScan(ssids) => fusion.on_wifi_scan(t, ssids),
            Event::Cell(c) => fusion.on_cell(t, c),
            Event::NetworkFix(f) => fusion.on_network_fix(t, f.0, f.1, f.2),
            Event::Audio(rate, audio) => fusion.on_audio(t, rate, *audio),
            Event::Ble(b) => fusion.on_ble(t, &b.0, b.1),
            Event::Transit(b) => fusion.on_transit(t, b.1, &b.0),
        }
    }
    progress(1.0);
    fusion.finish();
    // `Fusion::positions`, keeping the path before joining for scoring.
    let dead_reckoned = fusion.dead_reckoned();
    let good_fix_ns = fusion.good_fix_ns();
    let stops = fusion.stop_records();
    let smoothed = fusion.smoothed();
    let mut positions = dead_reckoned.clone();
    crate::track::join(&mut positions, &smoothed, &stops, &good_fix_ns, None);
    Ok(Replay {
        smoothed,
        gnss: fusion.navigator().stats.clone(),
        positions,
        dead_reckoned,
        good_fix_ns,
        door_cycles: fusion.door_cycles().to_vec(),
        chimes: fusion.chimes().to_vec(),
        stops,
        departures: fusion.departures().to_vec(),
        step_windows: fusion.step_windows().to_vec(),
        subway_changes: fusion.subway_changes().to_vec(),
        timeline: fusion.timeline(),
        vertical: fusion.vertical_moves(),
        statuses,
        status_cost_ns,
        feed_cost_ns,
        tick_cost_ns,
        live,
    })
}

/// One input's events in time order, or the error that ended its stream.
type Events<'a> = Box<dyn Iterator<Item = anyhow::Result<(i64, Event)>> + 'a>;

/// One input's events in time order, and how many there are.
type Source<'a> = (usize, Box<dyn Iterator<Item = (i64, Event)> + 'a>);

fn chunks_of<C>(contents: StreamContents<C>) -> impl Iterator<Item = anyhow::Result<C>> {
    contents.chunks.into_iter().map(Ok)
}

/// The events of a stream read a chunk at a time by `stream`, read twice: once to count them and check their order.
/// Collected and sorted (stably, so equal times keep their order) if they're out of order,
/// and from `read`, the whole file, if the stream is damaged: it reads past damage, where `stream` stops.
fn in_order<'a>(
    stream: impl Fn() -> anyhow::Result<Events<'a>>,
    read: impl FnOnce() -> anyhow::Result<Vec<(i64, Event)>>,
) -> anyhow::Result<Source<'a>> {
    let mut len = 0;
    let mut last = i64::MIN;
    let mut sorted = true;
    for event in stream()? {
        let Ok((t, _)) = event else {
            let mut events = read()?;
            events.sort_by_key(|(t, _)| *t);
            return Ok((events.len(), Box::new(events.into_iter())));
        };
        len += 1;
        sorted &= t >= last;
        last = t;
    }
    let events = stream()?.map_while(Result::ok);
    if sorted {
        Ok((len, Box::new(events)))
    } else {
        let mut events: Vec<_> = events.collect();
        events.sort_by_key(|(t, _)| *t);
        Ok((len, Box::new(events.into_iter())))
    }
}

/// Merges sources by time; at equal times, earlier sources come first,
/// the same order as a stable sort of all of them one after another.
fn merge<'a>(
    sources: Vec<Box<dyn Iterator<Item = (i64, Event)> + 'a>>,
) -> impl Iterator<Item = (i64, Event)> + 'a {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;
    let mut sources: Vec<_> = sources.into_iter().map(Iterator::peekable).collect();
    let mut heads: BinaryHeap<Reverse<(i64, usize)>> = sources
        .iter_mut()
        .enumerate()
        .filter_map(|(i, s)| s.peek().map(|e| Reverse((e.0, i))))
        .collect();
    std::iter::from_fn(move || {
        let Reverse((_, i)) = heads.pop()?;
        let event = sources[i].next()?;
        if let Some(e) = sources[i].peek() {
            heads.push(Reverse((e.0, i)));
        }
        Some(event)
    })
}

/// An IMU stream's events (`Some` of its sensor type), or the barometer's.
fn sensor_events<'a>(
    sensor_type: Option<i32>,
    chunks: impl Iterator<Item = anyhow::Result<SensorChunk>> + 'a,
) -> impl Iterator<Item = anyhow::Result<(i64, Event)>> + 'a {
    chunks.flat_map(move |chunk| {
        let chunk = match chunk {
            Ok(chunk) => chunk,
            Err(e) => return vec![Err(e)],
        };
        let values: Vec<f32> = chunk.values().collect();
        let timestamps = chunk.timestamps_ns().into_iter().enumerate();
        match sensor_type {
            Some(sensor_type) => {
                let n = chunk.values_per_sample as usize;
                timestamps
                    .filter_map(|(i, t)| {
                        match imu_sample(sensor_type, &values[i * n..(i + 1) * n]) {
                            Some(ImuSample::Accel(a)) => Some(Ok((t, Event::Accel(a)))),
                            Some(ImuSample::Gyro(w)) => Some(Ok((t, Event::Gyro(w)))),
                            Some(ImuSample::Mag(b)) => Some(Ok((t, Event::Mag(b)))),
                            None => None,
                        }
                    })
                    .collect()
            }
            None => {
                let n = (chunk.values_per_sample as usize).max(1);
                timestamps
                    .map(|(i, t)| Ok((t, Event::Pressure(f64::from(values[i * n])))))
                    .collect()
            }
        }
    })
}

/// Audio as the app reads it: each read arrives when its last frame was recorded.
/// A chunk whose FLAC doesn't decode is left out, as when reading the stream.
fn audio_events<'a>(
    rate: u32,
    chunks: impl Iterator<Item = anyhow::Result<AudioChunk>> + 'a,
    cuts: &'a [(i64, i64)],
) -> impl Iterator<Item = anyhow::Result<(i64, Event)>> + 'a {
    let mut clock: Option<(i64, i64)> = None;
    chunks.flat_map(move |chunk| {
        let mut chunk = match chunk {
            Ok(chunk) => chunk,
            Err(e) => return vec![Err(e)],
        };
        let mut events = Vec::new();
        // A chunk wholly in a cut isn't decoded: its timestamps say so (a few seconds each).
        let times = || chunk.timestamps.iter().map(|t| t.boottime_ns);
        if times().next().is_some() && times().all(|t| in_cuts(cuts, t)) {
            clock = chunk
                .timestamps
                .last()
                .map(|t| (t.frame_position, t.boottime_ns))
                .or(clock);
            return events;
        }
        if !ride_session::audio_flac::restore(&mut chunk) {
            return events;
        }
        let samples: Vec<i16> = chunk
            .pcm_s16le
            .as_chunks::<2>()
            .0
            .iter()
            .map(|&b| i16::from_le_bytes(b))
            .collect();
        let timestamps: Vec<(i64, i64)> = chunk
            .timestamps
            .iter()
            .map(|t| (t.frame_position, t.boottime_ns))
            .collect();
        clock = timestamps.last().copied().or(clock);
        let Some((f0, t0)) = clock else {
            return events;
        };
        for (i, read) in samples.chunks(AUDIO_READ_FRAMES).enumerate() {
            let first = chunk.first_frame + (i * AUDIO_READ_FRAMES) as i64;
            let last = first + read.len() as i64;
            let arrived = t0 + ((last - f0) as f64 / f64::from(rate) * 1e9) as i64;
            if in_cuts(cuts, arrived) {
                continue;
            }
            events.push(Ok((
                arrived,
                Event::Audio(
                    rate,
                    Box::new(AudioSamples {
                        first_frame: first,
                        samples: read.to_vec(),
                        // The app takes one timestamp per read.
                        timestamps: timestamps.get(i).copied().into_iter().collect(),
                    }),
                ),
            )));
        }
        events
    })
}
