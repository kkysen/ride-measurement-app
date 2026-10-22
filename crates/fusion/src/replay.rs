//! Replays a recorded session through [`Fusion`], fed exactly as the app feeds it live,
//! then smooths over the whole ride: live smooths only recent input, as it arrives, which can only be less accurate.

use ride_gnss::solve::Stats;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::GPS_PROVIDER;

use crate::fusion::{Fix, Fusion, ImuSample, SmoothedSpeed, imu_sample};

/// What a replay produced.
pub struct Replay {
    /// Smoothed speed at every GPS fix and every second.
    pub smoothed: Vec<SmoothedSpeed>,
    /// Navigation messages, ephemerides, and epochs solved from raw GNSS.
    pub gnss: Stats,
    /// Smoothed position at every sample (see [`crate::position`]).
    pub positions: Vec<crate::position::PositionEstimate>,
}

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
    Fix(Fix),
    Measurements(ride_session::proto::GnssMeasurementsEvent),
    Navigation(ride_session::proto::GnssNavigationMessage),
    WifiScan(Vec<String>),
}

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
    for (name, fallback) in [("accel_uncal", "accel"), ("gyro_uncal", "gyro")] {
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
                    None => {}
                }
            }
        }
    }
    for stream in ["location", "gnss", "radio"] {
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
                    events.push((fix.elapsed_realtime_ns, Event::Fix(fix)));
                }
                Some(Payload::GnssMeasurements(m)) => {
                    let t = m
                        .clock
                        .as_ref()
                        .and_then(|c| c.elapsed_realtime_nanos)
                        .unwrap_or(received);
                    events.push((t, Event::Measurements(m)));
                }
                Some(Payload::GnssNavigationMessage(m)) => {
                    events.push((received, Event::Navigation(m)))
                }
                // At the time the app recorded it, which is also when it fed it live.
                Some(Payload::WifiScan(w)) => events.push((
                    received,
                    Event::WifiScan(w.results.into_iter().map(|r| r.ssid).collect()),
                )),
                _ => {}
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
    for (t, e) in events {
        match e {
            Event::Accel(a) => fusion.on_accel(t, a),
            Event::Gyro(w) => fusion.on_gyro(t, w),
            Event::Fix(fix) => fusion.on_fix(fix),
            Event::Measurements(m) => {
                fusion.on_gnss_measurements(t, &m);
            }
            Event::Navigation(m) => fusion.on_navigation_message(&m),
            Event::WifiScan(ssids) => fusion.on_wifi_scan(t, ssids),
        }
    }
    fusion.finish();
    Ok(Replay {
        smoothed: fusion.smoothed(),
        gnss: fusion.navigator().stats.clone(),
        positions: fusion.positions(),
    })
}
