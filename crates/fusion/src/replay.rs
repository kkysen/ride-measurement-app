//! Replays a recorded session through [`LiveSpeedEstimator`], fed the same way the app feeds it live.

use ride_session::read::Session;
use ride_session::track::TrackPoint;

use crate::live_speed::{Estimate, LiveSpeedEstimator};

enum Event<'a> {
    Accel(i64, [f64; 3]),
    Gyro(i64, [f64; 3]),
    Fix(&'a TrackPoint),
}

impl Event<'_> {
    fn t_ns(&self) -> i64 {
        match self {
            Event::Accel(t, _) | Event::Gyro(t, _) => *t,
            Event::Fix(p) => p.elapsed_realtime_ns,
        }
    }
}

/// The estimate just after each fix in `fixes` (GPS fixes, as from [`ride_session::track::gps_segments`]).
///
/// Uses the uncalibrated accelerometer and gyro streams if recorded, else the calibrated ones, like the app.
/// Sensor samples and fixes are merged by timestamp; live, they arrive slightly out of order,
/// so the replay can differ a little from what the phone showed, never by design.
pub fn speed_at_fixes(session: &Session, fixes: &[&TrackPoint]) -> anyhow::Result<Vec<Estimate>> {
    let mut events: Vec<Event> = Vec::new();
    for (name, fallback, gyro) in [
        ("accel_uncal", "accel", false),
        ("gyro_uncal", "gyro", true),
    ] {
        let name = [name, fallback]
            .into_iter()
            .find(|n| session.info.streams.iter().any(|s| s.name == *n));
        let Some(name) = name else { continue };
        let uncalibrated = name.ends_with("_uncal");
        let contents = session.read_stream(name)?;
        for chunk in &contents.chunks {
            let n = chunk.values_per_sample as usize;
            let values: Vec<f32> = chunk.values().collect();
            for (i, t) in chunk.timestamps_ns().into_iter().enumerate() {
                let v = &values[i * n..(i + 1) * n];
                if v.len() < 3 || (uncalibrated && v.len() < 6) {
                    continue;
                }
                // Uncalibrated sensors carry the bias estimate in values[3..5]; subtract it, in `f32` like the app.
                let xyz: [f64; 3] = std::array::from_fn(|k| {
                    f64::from(if uncalibrated { v[k] - v[k + 3] } else { v[k] })
                });
                events.push(if gyro {
                    Event::Gyro(t, xyz)
                } else {
                    Event::Accel(t, xyz)
                });
            }
        }
    }
    events.extend(fixes.iter().map(|p| Event::Fix(p)));
    // Stable, so at equal times sensors (pushed first) come before fixes.
    events.sort_by_key(Event::t_ns);

    let mut est = LiveSpeedEstimator::new();
    let mut out = Vec::with_capacity(fixes.len());
    for e in &events {
        match e {
            Event::Accel(t, [x, y, z]) => est.on_accel(*t, *x, *y, *z),
            Event::Gyro(_, [x, y, z]) => est.on_gyro(*x, *y, *z),
            Event::Fix(p) => {
                est.on_gnss_fix(
                    p.elapsed_realtime_ns,
                    p.latitude_degrees,
                    p.longitude_degrees,
                    p.horizontal_accuracy_meters.map(f64::from),
                    p.speed_meters_per_second.map(f64::from),
                    p.speed_accuracy_meters_per_second.map(f64::from),
                );
                out.push(est.estimate(p.elapsed_realtime_ns));
            }
        }
    }
    Ok(out)
}
