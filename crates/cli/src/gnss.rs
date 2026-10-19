//! `ride gnss`: how well raw GNSS solved, to check the solver on a phone's real data.

use ride_gnss::solve::{Navigator, NoSolution, geodetic_to_ecef};
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::GPS_PROVIDER;

/// The phone's own fix values count as "at the same time" within this.
const SAME_TIME_NS: i64 = 1_500_000_000;

/// One solved epoch, with the phone's own fix at about the same time for comparison.
struct Epoch {
    speed: f64,
    doppler_residual_rms: f64,
    pseudorange_residual_rms: f64,
    satellites: usize,
    phone_speed: Option<f32>,
    /// Distance from the solved position to the phone's fix (m).
    phone_distance: Option<f64>,
}

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
    let mut solved: Vec<Epoch> = Vec::new();
    let mut phone_speed: Option<(i64, f32)> = None;
    let mut phone_position: Option<(i64, [f64; 3])> = None;
    for r in &records {
        let t = r.received_elapsed_realtime_ns;
        match &r.payload {
            Some(Payload::GnssNavigationMessage(m)) => nav.on_navigation_message(m),
            Some(Payload::Location(l)) if l.provider == GPS_PROVIDER => {
                let ecef = geodetic_to_ecef(
                    l.latitude_degrees,
                    l.longitude_degrees,
                    l.altitude_meters.unwrap_or(0.0),
                );
                nav.set_approximate_position(ecef);
                phone_position = Some((t, ecef));
                if let Some(s) = l.speed_meters_per_second {
                    phone_speed = Some((t, s));
                }
            }
            Some(Payload::GnssMeasurements(m)) => match nav.on_measurements(m) {
                Ok(sol) => {
                    let recent = |at: i64| (t - at).abs() < SAME_TIME_NS;
                    solved.push(Epoch {
                        speed: sol.speed(),
                        doppler_residual_rms: sol.residual_rms,
                        pseudorange_residual_rms: sol.position_residual_rms,
                        satellites: sol.satellites_used,
                        phone_speed: phone_speed.filter(|(at, _)| recent(*at)).map(|(_, s)| s),
                        phone_distance: phone_position.filter(|(at, _)| recent(*at)).map(
                            |(_, p)| {
                                let d: [f64; 3] = std::array::from_fn(|k| sol.position[k] - p[k]);
                                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
                            },
                        ),
                    });
                }
                Err(e) => {
                    let why = match e {
                        NoSolution::TooFewSatellites { .. } => {
                            "too few usable GPS satellites".into()
                        }
                        other => format!("{other:?}"),
                    };
                    *no_solution.entry(why).or_default() += 1;
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
    let all = |f: fn(&Epoch) -> f64| solved.iter().map(f).collect::<Vec<_>>();
    println!(
        "satellites used: median {}; residual RMS: Doppler median {:.3} m/s, pseudorange median {:.1} m",
        median(all(|e| e.satellites as f64)),
        median(all(|e| e.doppler_residual_rms)),
        median(all(|e| e.pseudorange_residual_rms))
    );
    let distances: Vec<f64> = solved.iter().filter_map(|e| e.phone_distance).collect();
    if !distances.is_empty() {
        println!(
            "own position from the phone's fix: median {:.1} m, max {:.1} m",
            median(distances.clone()),
            distances.iter().copied().fold(0.0, f64::max)
        );
    }
    println!(
        "own speed: median {:.2} m/s, max {:.2} m/s",
        median(all(|e| e.speed)),
        all(|e| e.speed).into_iter().fold(0.0, f64::max)
    );
    let diffs: Vec<f64> = solved
        .iter()
        .filter_map(|e| e.phone_speed.map(|p| e.speed - f64::from(p)))
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
