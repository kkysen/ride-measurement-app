//! Replays a recorded session through [`Fusion`], fed exactly as the app feeds it live,
//! then smooths over the whole ride: live smooths only recent input, as it arrives, which can only be less accurate.

use ride_gnss::solve::Stats;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
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
    /// What the rider did over the ride (see [`Fusion::timeline`]), as of its end.
    pub timeline: Vec<crate::fusion::Segment>,
    /// Stairs, elevators, and escalators (see [`crate::vertical`]).
    pub vertical: Vec<crate::vertical::Move>,
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
    /// Leave out [`Replay::statuses`], for what doesn't show them: about a fifth of a replay's time.
    pub no_statuses: bool,
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
    let mut events: Vec<(i64, Event)> = Vec::new();
    // The recorded streams the app feeds live, preferring uncalibrated (see `RecorderService.kt`).
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
        for chunk in &session.read_stream(&stream.name)?.chunks {
            let n = chunk.values_per_sample as usize;
            let values: Vec<f32> = chunk.values().collect();
            for (i, t) in chunk.timestamps_ns().into_iter().enumerate() {
                match imu_sample(sensor_type, &values[i * n..(i + 1) * n]) {
                    Some(ImuSample::Accel(a)) => events.push((t, Event::Accel(a))),
                    Some(ImuSample::Gyro(w)) => events.push((t, Event::Gyro(w))),
                    Some(ImuSample::Mag(b)) => events.push((t, Event::Mag(b))),
                    None => {}
                }
            }
        }
    }
    if session.info.streams.iter().any(|s| s.name == "pressure") {
        for chunk in &session.read_stream("pressure")?.chunks {
            let n = (chunk.values_per_sample as usize).max(1);
            let values: Vec<f32> = chunk.values().collect();
            for (i, t) in chunk.timestamps_ns().into_iter().enumerate() {
                events.push((t, Event::Pressure(f64::from(values[i * n]))));
            }
        }
    }
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
                    events.push((fix.elapsed_realtime_ns, Event::Fix(Box::new(fix))));
                }
                // Android's own location, as the app passes it on.
                Some(Payload::Location(l)) if l.provider == "fused" && !l.is_mock => events.push((
                    l.elapsed_realtime_nanos,
                    Event::NetworkFix(Box::new((
                        l.latitude_degrees,
                        l.longitude_degrees,
                        l.horizontal_accuracy_meters.map(f64::from),
                    ))),
                )),
                Some(Payload::GnssMeasurements(m)) => {
                    let t = m
                        .clock
                        .as_ref()
                        .and_then(|c| c.elapsed_realtime_nanos)
                        .unwrap_or(received);
                    events.push((t, Event::Measurements(Box::new(m))));
                }
                Some(Payload::GnssNavigationMessage(m)) => {
                    events.push((received, Event::Navigation(Box::new(m))))
                }
                // At the time the app recorded it, which is also when it fed it live.
                Some(Payload::WifiScan(w)) => events.push((
                    received,
                    Event::WifiScan(w.results.into_iter().map(|r| r.ssid).collect()),
                )),
                // Bluetooth advertisements, at each result's own time; the subway cars' beacons are used.
                Some(Payload::BleScan(b)) => {
                    for r in b.results {
                        events.push((
                            r.timestamp_elapsed_realtime_ns,
                            Event::Ble(Box::new((r.advertising_data, r.rssi))),
                        ));
                    }
                }
                // The MTA's feeds, when each response arrived, as the app passes them on.
                Some(Payload::GtfsRealtime(f)) if f.http_status == 200 => events.push((
                    received,
                    Event::Transit(Box::new((f.body, f.request_unix_ms))),
                )),
                // The registered LTE cell, as the app passes it on.
                Some(Payload::CellInfo(c)) => events.push((
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
    if let Some(rate) = audio_rate.filter(|_| !options.ignore_audio)
        && session.stream_path("audio").exists()
    {
        let mut clock: Option<(i64, i64)> = None;
        for chunk in session.read_audio("audio")?.chunks {
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
            let Some((f0, t0)) = clock else { continue };
            for (i, read) in samples.chunks(AUDIO_READ_FRAMES).enumerate() {
                let first = chunk.first_frame + (i * AUDIO_READ_FRAMES) as i64;
                let last = first + read.len() as i64;
                let arrived = t0 + ((last - f0) as f64 / f64::from(rate) * 1e9) as i64;
                events.push((
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
                ));
            }
        }
    }
    // Stable, so at equal times sensors (pushed first) come before GNSS.
    events.sort_by_key(|(t, _)| *t);
    if let Some(cut) = options.gnss_outages {
        events.retain(|(t, e)| {
            !(matches!(e, Event::Fix(_) | Event::Measurements(_)) && cut.covers(*t))
        });
    }

    let mut fusion = Fusion::whole_ride();
    fusion.set_max_speed(options.max_speed_mps);
    // Downloaded orbits (`just orbits`), for constellations whose navigation messages the phone missed.
    for eph in ride_gnss::orbits::read_session_orbits(&session.dir)? {
        fusion.navigator_mut().add_ephemeris(eph);
    }
    let mut statuses = Vec::new();
    let mut next_status = events.first().map_or(0, |e| e.0);
    for (t, e) in events {
        if t >= next_status && !options.no_statuses {
            statuses.push((t, fusion.status()));
            next_status = t + STATUS_EVERY_NS;
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
    fusion.finish();
    Ok(Replay {
        smoothed: fusion.smoothed(),
        gnss: fusion.navigator().stats.clone(),
        positions: fusion.positions(),
        door_cycles: fusion.door_cycles().to_vec(),
        chimes: fusion.chimes().to_vec(),
        stops: fusion.stop_records(),
        departures: fusion.departures().to_vec(),
        step_windows: fusion.step_windows().to_vec(),
        subway_changes: fusion.subway_changes().to_vec(),
        timeline: fusion.timeline(),
        vertical: fusion.vertical_moves(),
        statuses,
    })
}
