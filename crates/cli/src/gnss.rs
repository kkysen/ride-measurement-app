//! `ride gnss`: how well raw GNSS solved, to check the solver on a phone's real data.

use ride_gnss::solve::{Navigator, NoSolution, geodetic_to_ecef};
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::GPS_PROVIDER;

pub fn gnss(session: &Session) -> anyhow::Result<()> {
    let mut records: Vec<_> = Vec::new();
    for stream in ["gnss", "location"] {
        if session.stream_path(stream).exists() {
            records.extend(
                session
                    .read_records(stream)?
                    .chunks
                    .into_iter()
                    .flat_map(|c| c.records),
            );
        }
    }
    records.sort_by_key(|r| r.received_elapsed_realtime_ns);
    let mut nav = Navigator::default();
    let mut no_solution = std::collections::BTreeMap::<String, u64>::new();
    // (own speed, residual RMS, satellites, and the phone's speed at about the same time)
    let mut solved: Vec<(f64, f64, usize, Option<f32>)> = Vec::new();
    let mut last_phone_speed: Option<(i64, f32)> = None;
    for r in &records {
        match &r.payload {
            Some(Payload::GnssNavigationMessage(m)) => nav.on_navigation_message(m),
            Some(Payload::Location(l)) if l.provider == GPS_PROVIDER => {
                nav.set_approximate_position(geodetic_to_ecef(
                    l.latitude_degrees,
                    l.longitude_degrees,
                    l.altitude_meters.unwrap_or(0.0),
                ));
                if let Some(s) = l.speed_meters_per_second {
                    last_phone_speed = Some((r.received_elapsed_realtime_ns, s));
                }
            }
            Some(Payload::GnssMeasurements(m)) => match nav.on_measurements(m) {
                Ok(sol) => {
                    let phone = last_phone_speed
                        .filter(|(t, _)| (r.received_elapsed_realtime_ns - t).abs() < 1_500_000_000)
                        .map(|(_, s)| s);
                    solved.push((sol.speed(), sol.residual_rms, sol.satellites_used, phone));
                }
                Err(e) => {
                    let key = match e {
                        NoSolution::TooFewSatellites { .. } => {
                            "too few usable GPS satellites".into()
                        }
                        other => format!("{other:?}"),
                    };
                    *no_solution.entry(key).or_default() += 1;
                }
            },
            _ => {}
        }
    }
    let s = &nav.stats;
    println!(
        "navigation subframes: {} passed parity, {} failed; {} ephemerides",
        s.subframes_ok, s.subframes_bad, s.ephemerides
    );
    println!("epochs: {} of {} solved", s.epochs_solved, s.epochs);
    for (why, n) in &no_solution {
        println!("  {n} unsolved: {why}");
    }
    if solved.is_empty() {
        return Ok(());
    }
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v[v.len() / 2]
    };
    println!(
        "Doppler residual RMS: median {:.3} m/s; satellites used: median {}",
        median(solved.iter().map(|x| x.1).collect()),
        median(solved.iter().map(|x| x.2 as f64).collect())
    );
    println!(
        "own speed: median {:.2} m/s, max {:.2} m/s",
        median(solved.iter().map(|x| x.0).collect()),
        solved.iter().map(|x| x.0).fold(0.0, f64::max)
    );
    let diffs: Vec<f64> = solved
        .iter()
        .filter_map(|x| x.3.map(|p| x.0 - f64::from(p)))
        .collect();
    if !diffs.is_empty() {
        println!(
            "own minus phone speed: median {:.3} m/s, median absolute {:.3} m/s, over {} epochs",
            median(diffs.clone()),
            median(diffs.iter().map(|d| d.abs()).collect()),
            diffs.len()
        );
    }
    Ok(())
}
