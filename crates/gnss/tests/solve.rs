//! Position and velocity from synthetic Android measurements made with real orbits and a known receiver motion.

use ride_gnss::ephemeris::{C, Ephemeris, OMEGA_E};
use ride_gnss::solve::{Navigator, geodetic_to_ecef};
use ride_gnss::system::System;
use ride_session::proto::{GnssClock, GnssMeasurement, GnssMeasurementsEvent};

const WEEK: i64 = 1590;
/// 01:00 on 2010-07-01, in GPS seconds of week.
const TOW: f64 = 4.0 * 86_400.0 + 3600.0;

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

/// East, north, up unit vectors at a latitude and longitude.
fn enu(lat: f64, lon: f64) -> [[f64; 3]; 3] {
    let (sl, cl) = lat.to_radians().sin_cos();
    let (so, co) = lon.to_radians().sin_cos();
    [
        [-so, co, 0.0],
        [-sl * co, -sl * so, cl],
        [cl * co, cl * so, sl],
    ]
}

/// What Android would report for a receiver at `rx` moving at `v`, with clock drift `drift` (m/s),
/// for every satellite above 10 degrees.
fn measurements(
    ephs: &[Ephemeris],
    rx: [f64; 3],
    v: [f64; 3],
    drift: f64,
    up: [f64; 3],
) -> GnssMeasurementsEvent {
    measurements_at(WEEK, TOW, ephs, rx, v, drift, up)
}

/// The same at GPS week `week`, seconds of week `tow`, for GPS and Galileo (whose times count like GPS's).
fn measurements_at(
    week: i64,
    tow: f64,
    ephs: &[Ephemeris],
    rx: [f64; 3],
    v: [f64; 3],
    drift: f64,
    up: [f64; 3],
) -> GnssMeasurementsEvent {
    let rx_gps_ns = week * 604_800_000_000_000 + (tow * 1e9) as i64;
    let time_nanos = 1_000_000_000_000;
    let mut ms = Vec::new();
    for e in ephs {
        // Solve the transmit time: the signal leaves at t_tx and arrives at TOW.
        let mut t_tx = tow - 0.075;
        let mut state = e.state_at(t_tx);
        let mut pos = state.position;
        for _ in 0..5 {
            state = e.state_at(t_tx);
            let range = norm(sub(state.position, rx));
            let th = OMEGA_E * range / C;
            pos = [
                th.cos() * state.position[0] + th.sin() * state.position[1],
                -th.sin() * state.position[0] + th.cos() * state.position[1],
                state.position[2],
            ];
            t_tx = tow - norm(sub(pos, rx)) / C;
        }
        let th = OMEGA_E * norm(sub(pos, rx)) / C;
        let vel = [
            th.cos() * state.velocity[0] + th.sin() * state.velocity[1],
            -th.sin() * state.velocity[0] + th.cos() * state.velocity[1],
            state.velocity[2],
        ];
        let los = sub(pos, rx).map(|x| x / norm(sub(pos, rx)));
        if dot(los, up) < 10f64.to_radians().sin() {
            continue;
        }
        // Transmit time on the satellite's clock (with group delay), and Doppler with both clocks' drift.
        let t_sv = t_tx + state.clock_bias - e.tgd();
        // BeiDou counts its own time, 14 s behind GPS.
        let t_sv = if e.sat().system == System::Beidou {
            t_sv - 14.0
        } else {
            t_sv
        };
        let prr = dot(sub(vel, v), los) + drift - C * state.clock_drift;
        ms.push(GnssMeasurement {
            svid: i32::from(e.sat().svid),
            constellation_type: match e.sat().system {
                System::Galileo => 6,
                System::Beidou => 5,
                _ => 1,
            },
            state: 0x1 | 0x8,
            received_sv_time_nanos: (t_sv * 1e9).round() as i64,
            cn0_dbhz: 40.0,
            pseudorange_rate_meters_per_second: prr,
            pseudorange_rate_uncertainty_meters_per_second: 0.05,
            ..Default::default()
        });
    }
    GnssMeasurementsEvent {
        clock: Some(GnssClock {
            time_nanos,
            full_bias_nanos: Some(time_nanos - rx_gps_ns),
            bias_nanos: Some(0.0),
            elapsed_realtime_nanos: Some(123),
            ..Default::default()
        }),
        measurements: ms,
        ..Default::default()
    }
}

fn setup() -> (Navigator, Vec<Ephemeris>, [f64; 3], [f64; 3], [f64; 3]) {
    let ephs = ride_gnss::rinex::read_nav(include_str!("data/brdc1820.10n"));
    let mut nav = Navigator::default();
    for e in &ephs {
        nav.add_ephemeris(e.clone());
    }
    // Under the fixture satellites' average position, so most are in view (only 3 are above New York then).
    let mean: [f64; 3] = std::array::from_fn(|k| {
        ephs.iter()
            .map(|e| e.state_at(TOW).position[k])
            .sum::<f64>()
    });
    let (lat, lon) = (
        (mean[2] / norm(mean)).asin().to_degrees(),
        mean[1].atan2(mean[0]).to_degrees(),
    );
    let rx = geodetic_to_ecef(lat, lon, 10.0);
    let [east, north, up] = enu(lat, lon);
    // 20 m/s east, 5 m/s north, climbing 0.5 m/s.
    let v = std::array::from_fn(|k| 20.0 * east[k] + 5.0 * north[k] + 0.5 * up[k]);
    (nav, ephs, rx, v, up)
}

#[test]
fn recovers_position_and_velocity() {
    let (mut nav, ephs, rx, v, up) = setup();
    let event = measurements(&ephs, rx, v, 50.0, up);
    let n = event.measurements.len();
    assert!(n >= 6, "only {n} satellites visible");
    let sol = nav.on_measurements(&event).unwrap();
    let (pos, vel) = (sol.position.clone().unwrap(), sol.velocity.clone().unwrap());
    assert!(
        norm(sub(pos.position, rx)) < 1.0,
        "position off by {} m",
        norm(sub(pos.position, rx))
    );
    assert!(
        norm(sub(vel.velocity, v)) < 0.01,
        "velocity off by {} m/s",
        norm(sub(vel.velocity, v))
    );
    assert!((vel.clock_drift - 50.0).abs() < 0.01);
    assert!((vel.speed() - norm(v)).abs() < 0.01);
    assert!(
        vel.speed_sigma() > 0.0 && vel.speed_sigma() < 0.2,
        "sigma {}",
        vel.speed_sigma()
    );
    assert_eq!((vel.satellites_used, vel.satellites_rejected), (n, 0));
    assert_eq!(sol.elapsed_realtime_ns, Some(123));
}

#[test]
fn rejects_a_bad_satellite() {
    let (mut nav, ephs, rx, v, up) = setup();
    let mut event = measurements(&ephs, rx, v, 50.0, up);
    let n = event.measurements.len();
    // Multipath or a tracking glitch: one satellite's Doppler 3 m/s off.
    event.measurements[0].pseudorange_rate_meters_per_second += 3.0;
    let sol = nav.on_measurements(&event).unwrap();
    let vel = sol.velocity.clone().unwrap();
    assert_eq!((vel.satellites_used, vel.satellites_rejected), (n - 1, 1));
    assert!(
        norm(sub(vel.velocity, v)) < 0.01,
        "velocity off by {} m/s",
        norm(sub(vel.velocity, v))
    );
}

#[test]
fn survives_a_millisecond_pseudorange_error() {
    let (mut nav, ephs, rx, v, up) = setup();
    let mut event = measurements(&ephs, rx, v, 50.0, up);
    // A transmit time off by a whole millisecond (300 km), as an ambiguous measurement can be.
    event.measurements[0].received_sv_time_nanos += 1_000_000;
    let sol = nav.on_measurements(&event).unwrap();
    let (pos, vel) = (sol.position.clone().unwrap(), sol.velocity.clone().unwrap());
    assert!(
        norm(sub(pos.position, rx)) < 1.0,
        "position off by {} m",
        norm(sub(pos.position, rx))
    );
    assert!(
        norm(sub(vel.velocity, v)) < 0.01,
        "velocity off by {} m/s",
        norm(sub(vel.velocity, v))
    );
    assert!(pos.residual_rms < 1.0, "residual {}", pos.residual_rms);

    // Flagged as ambiguous, it isn't used at all.
    let mut event = measurements(&ephs, rx, v, 50.0, up);
    let n = event.measurements.len();
    event.measurements[0].state |= 0x10;
    let sol = nav.on_measurements(&event).unwrap();
    let vel = sol.velocity.clone().unwrap();
    assert_eq!(vel.satellites_used + vel.satellites_rejected, n - 1);
}

/// A navigator 500 m off, with measurements at 2021-04-08 21:30 GPS time (GPS week 2152, Thursday)
/// from at most `counts` satellites of each constellation (Android's constellation type, count).
fn sparse_sky(counts: &[(i32, usize)]) -> (Navigator, GnssMeasurementsEvent, [f64; 3], [f64; 3]) {
    let (week, tow) = (2152, 4.0 * 86_400.0 + 21.5 * 3600.0);
    let ephs: Vec<Ephemeris> =
        ride_gnss::rinex::read_nav(include_str!("data/BRDM00DLR_S_20210980000-21h.rnx"))
            .into_iter()
            .filter(|e| {
                e.healthy()
                    && matches!(
                        e.sat().system,
                        System::Gps | System::Galileo | System::Beidou
                    )
            })
            .collect();
    let (lat, lon) = (40.75, -73.99);
    let rx = geodetic_to_ecef(lat, lon, 10.0);
    let [east, north, up] = enu(lat, lon);
    let v = std::array::from_fn(|k| 15.0 * east[k] - 8.0 * north[k]);
    let mut event = measurements_at(week, tow, &ephs, rx, v, 30.0, up);
    // One measurement per satellite (the excerpt repeats ephemerides), then at most `counts` of each.
    let mut seen = std::collections::BTreeSet::new();
    event
        .measurements
        .retain(|m| seen.insert((m.constellation_type, m.svid)));
    let mut taken = std::collections::BTreeMap::new();
    event.measurements.retain(|m| {
        let limit = counts
            .iter()
            .find(|(c, _)| *c == m.constellation_type)
            .map_or(0, |(_, n)| *n);
        let count = taken.entry(m.constellation_type).or_insert(0);
        *count += 1;
        *count <= limit
    });
    for &(c, n) in counts {
        assert!(
            taken.get(&c).copied().unwrap_or(0) >= n,
            "too few visible of {c}"
        );
    }

    let mut nav = Navigator::default();
    for e in &ephs {
        nav.add_ephemeris(e.clone());
    }
    nav.set_approximate_position(std::array::from_fn(|k| rx[k] + 500.0 * east[k]));
    (nav, event, rx, v)
}

/// Five satellites from two constellations: exactly as many as position's unknowns (3 coordinates and
/// 2 clock biases), so the position solves but nothing checks it, and it comes with a wide uncertainty.
#[test]
fn exactly_determined_position_is_wide() {
    let (mut nav, event, rx, _) = sparse_sky(&[(1, 1), (6, 4)]);
    let sol = nav.on_measurements(&event).unwrap();
    let pos = sol.position.unwrap();
    assert!(!pos.checked);
    assert_eq!(pos.satellites_used, 5);
    assert!(
        norm(sub(pos.position, rx)) < 5.0,
        "position off by {} m",
        norm(sub(pos.position, rx))
    );
    for k in [0, 4, 8] {
        assert!(
            pos.position_covariance[k] >= 60.0 * 60.0,
            "variance {}",
            pos.position_covariance[k]
        );
    }
    assert!(sol.velocity.unwrap().with_this_position);
    assert_eq!(nav.stats.positions_solved, 1);
}

/// Five satellites from three constellations: too few for position (3 coordinates, 3 clock biases),
/// enough for velocity (3 components, 1 clock drift, and one to check), which uses the last position instead;
/// 500 m off, each direction to a satellite tilts by about 25 microradians, a few cm/s of its relative motion,
/// which so few satellites amplify a little.
#[test]
fn velocity_solves_when_position_cannot() {
    let (mut nav, event, _, v) = sparse_sky(&[(1, 1), (5, 1), (6, 3)]);
    let sol = nav.on_measurements(&event).unwrap();
    assert!(sol.position.is_none());
    let vel = sol.velocity.unwrap();
    assert!(!vel.with_this_position);
    assert!(
        norm(sub(vel.velocity, v)) < 0.1,
        "velocity off by {} m/s",
        norm(sub(vel.velocity, v))
    );
    assert_eq!(nav.stats.positions_solved, 0);
    assert_eq!(nav.stats.velocities_solved, 1);
}

/// BeiDou B1C measurements are left out: the broadcast clock's group delay is B1I's.
#[test]
fn ignores_beidou_b1c() {
    let (mut nav, mut event, _, _) = sparse_sky(&[(1, 1), (5, 3), (6, 4)]);
    let with_b1i = nav.on_measurements(&event).unwrap().velocity.unwrap();
    for m in event
        .measurements
        .iter_mut()
        .filter(|m| m.constellation_type == 5)
    {
        m.carrier_frequency_hz = Some(1.575_42e9);
    }
    let with_b1c = nav.on_measurements(&event).unwrap().velocity.unwrap();
    let count = |v: &ride_gnss::solve::VelocitySolution| {
        v.satellites_by_system
            .iter()
            .find(|(s, _)| *s == System::Beidou)
            .map_or(0, |(_, n)| *n)
    };
    assert_eq!((count(&with_b1i), count(&with_b1c)), (3, 0));
}
