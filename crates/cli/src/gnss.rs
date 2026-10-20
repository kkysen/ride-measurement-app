//! `ride gnss`: how well raw GNSS solved, to check the solver on a phone's real data.

use std::collections::BTreeMap;

use ride_gnss::orbits::{SESSION_ORBITS_DIR, fetch_plan, read_session_orbits};
use ride_gnss::solve::{Navigator, NoSolution, geodetic_to_ecef};
use ride_gnss::system::System;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::GPS_PROVIDER;

/// The phone's own fix values count as "at the same time" within this.
const SAME_TIME_NS: i64 = 1_500_000_000;

/// One epoch's velocity, with the phone's own speed at about the same time for comparison.
struct Velocity {
    speed: f64,
    residual_rms: f64,
    satellites: usize,
    by_system: Vec<(System, usize)>,
    with_this_position: bool,
    phone_speed: Option<f32>,
}

/// One epoch's position, with its distance from the phone's own fix at about the same time (m).
struct Position {
    residual_rms: f64,
    checked: bool,
    satellites: usize,
    phone_distance: Option<f64>,
}

pub fn gnss(session: &Session, decoded_only: bool) -> anyhow::Result<()> {
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
    nav.decoded_only = decoded_only;
    let orbits = read_session_orbits(&session.dir)?;
    if decoded_only {
        println!("ignoring downloaded orbits: only the phone's navigation messages");
    } else if orbits.is_empty() {
        println!(
            "no downloaded orbits in {SESSION_ORBITS_DIR}/ (`just orbits`): only the phone's navigation messages"
        );
    } else {
        println!("{} ephemerides from {SESSION_ORBITS_DIR}/", orbits.len());
    }
    for eph in orbits {
        nav.add_ephemeris(eph);
    }
    let mut no_solution = std::collections::BTreeMap::<String, u64>::new();
    let mut velocities: Vec<Velocity> = Vec::new();
    let mut positions: Vec<Position> = Vec::new();
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
                    if let Some(v) = &sol.velocity {
                        velocities.push(Velocity {
                            speed: v.speed(),
                            residual_rms: v.residual_rms,
                            satellites: v.satellites_used,
                            by_system: v.satellites_by_system.clone(),
                            with_this_position: v.with_this_position,
                            phone_speed: phone_speed.filter(|(at, _)| recent(*at)).map(|(_, s)| s),
                        });
                    }
                    if let Some(p) = &sol.position {
                        positions.push(Position {
                            residual_rms: p.residual_rms,
                            checked: p.checked,
                            satellites: p.satellites_used,
                            phone_distance: phone_position.filter(|(at, _)| recent(*at)).map(
                                |(_, fix)| {
                                    let d: [f64; 3] =
                                        std::array::from_fn(|k| p.position[k] - fix[k]);
                                    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
                                },
                            ),
                        });
                    }
                }
                Err(e) => {
                    let why = match e {
                        NoSolution::TooFewSatellites { .. } => "too few usable satellites".into(),
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
        "navigation subframes: {} passed parity, {} failed; {} ephemerides ({} decoded from them)",
        s.subframes_ok, s.subframes_bad, s.ephemerides, s.ephemerides_decoded
    );
    for (system, d) in &s.decoding {
        println!(
            "  {}: {} messages passed their checks, {} failed; {} ephemerides decoded",
            system.name(),
            d.ok,
            d.bad,
            d.ephemerides
        );
    }
    if s.decoding.contains_key(&System::Beidou) {
        println!(
            "  BeiDou words arrive {}",
            match nav.beidou_interleaved() {
                Some(true) => "interleaved, as transmitted",
                Some(false) => "de-interleaved",
                None => "either way so far (no subframe has settled which)",
            }
        );
    }
    println!(
        "epochs: {} of {} solved: {} with a position, {} with a velocity ({} from an earlier position)",
        s.epochs_solved,
        s.epochs,
        s.positions_solved,
        s.velocities_solved,
        velocities.iter().filter(|v| !v.with_this_position).count()
    );
    for (why, n) in &no_solution {
        println!("  {n} unsolved: {why}");
    }
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(f64::NAN)
    };
    if !positions.is_empty() {
        let checked: Vec<&Position> = positions.iter().filter(|p| p.checked).collect();
        println!(
            "position: satellites median {}, pseudorange residual RMS median {:.1} m (of {} checked), \
             {} unchecked (no satellite to spare)",
            median(positions.iter().map(|p| p.satellites as f64).collect()),
            median(checked.iter().map(|p| p.residual_rms).collect()),
            checked.len(),
            positions.len() - checked.len()
        );
        let distances: Vec<f64> = positions.iter().filter_map(|p| p.phone_distance).collect();
        if !distances.is_empty() {
            println!(
                "own position from the phone's fix: median {:.1} m, max {:.1} m",
                median(distances.clone()),
                distances.iter().copied().fold(0.0, f64::max)
            );
        }
    }
    if velocities.is_empty() {
        return Ok(());
    }
    let mut per_system = BTreeMap::<System, Vec<f64>>::new();
    for v in &velocities {
        for &(sys, n) in &v.by_system {
            per_system.entry(sys).or_default().push(n as f64);
        }
    }
    let per_system: Vec<String> = per_system
        .into_iter()
        .map(|(sys, mut n)| {
            // Epochs without the constellation count as none.
            n.resize(velocities.len(), 0.0);
            format!("{} {}", sys.name(), median(n))
        })
        .collect();
    println!(
        "velocity: satellites median {} ({}), Doppler residual RMS median {:.3} m/s",
        median(velocities.iter().map(|v| v.satellites as f64).collect()),
        per_system.join(", "),
        median(velocities.iter().map(|v| v.residual_rms).collect())
    );
    let speeds: Vec<f64> = velocities.iter().map(|v| v.speed).collect();
    println!(
        "own speed: median {:.2} m/s, max {:.2} m/s",
        median(speeds.clone()),
        speeds.iter().copied().fold(0.0, f64::max)
    );
    let diffs: Vec<f64> = velocities
        .iter()
        .filter_map(|v| v.phone_speed.map(|p| v.speed - f64::from(p)))
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

/// The orbit files to download for a session (`ride_gnss::orbits::fetch_plan` over the session's GNSS),
/// one UTC day per line as `path url` pairs to try in order until one downloads; `just orbits` follows it.
pub fn orbit_plan(session: &Session) -> anyhow::Result<()> {
    let start_ms = session.info.start_unix_ms;
    let mut last_ns = session.info.start_elapsed_realtime_ns;
    if session.stream_path("gnss").exists() {
        for chunk in session.read_records("gnss")?.chunks {
            for r in chunk.records {
                last_ns = last_ns.max(r.received_elapsed_realtime_ns);
            }
        }
    }
    let end_ms = start_ms + (last_ns - session.info.start_elapsed_realtime_ns) / 1_000_000;
    for day in fetch_plan(&session.dir, start_ms, end_ms) {
        let pairs: Vec<String> = day
            .iter()
            .map(|f| format!("{} {}", f.path.display(), f.url))
            .collect();
        println!("{}", pairs.join(" "));
    }
    Ok(())
}
