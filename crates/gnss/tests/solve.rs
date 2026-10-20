//! Position and velocity from synthetic Android measurements made with real orbits and a known receiver motion.

use ride_gnss::ephemeris::{C, KeplerEphemeris, OMEGA_E};
use ride_gnss::solve::{Navigator, geodetic_to_ecef};
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
    ephs: &[KeplerEphemeris],
    rx: [f64; 3],
    v: [f64; 3],
    drift: f64,
    up: [f64; 3],
) -> GnssMeasurementsEvent {
    let rx_gps_ns = WEEK * 604_800_000_000_000 + (TOW * 1e9) as i64;
    let time_nanos = 1_000_000_000_000;
    let mut ms = Vec::new();
    for e in ephs {
        // Solve the transmit time: the signal leaves at t_tx and arrives at TOW.
        let mut t_tx = TOW - 0.075;
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
            t_tx = TOW - norm(sub(pos, rx)) / C;
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
        let t_sv = t_tx + state.clock_bias - e.tgd;
        let prr = dot(sub(vel, v), los) + drift - C * state.clock_drift;
        ms.push(GnssMeasurement {
            svid: i32::from(e.sat.svid),
            constellation_type: 1,
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

fn setup() -> (
    Navigator,
    Vec<KeplerEphemeris>,
    [f64; 3],
    [f64; 3],
    [f64; 3],
) {
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
    assert!(
        norm(sub(sol.position, rx)) < 1.0,
        "position off by {} m",
        norm(sub(sol.position, rx))
    );
    assert!(
        norm(sub(sol.velocity, v)) < 0.01,
        "velocity off by {} m/s",
        norm(sub(sol.velocity, v))
    );
    assert!((sol.clock_drift - 50.0).abs() < 0.01);
    assert!((sol.speed() - norm(v)).abs() < 0.01);
    assert!(
        sol.speed_sigma() > 0.0 && sol.speed_sigma() < 0.2,
        "sigma {}",
        sol.speed_sigma()
    );
    assert_eq!((sol.satellites_used, sol.satellites_rejected), (n, 0));
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
    assert_eq!((sol.satellites_used, sol.satellites_rejected), (n - 1, 1));
    assert!(
        norm(sub(sol.velocity, v)) < 0.01,
        "velocity off by {} m/s",
        norm(sub(sol.velocity, v))
    );
}

#[test]
fn survives_a_millisecond_pseudorange_error() {
    let (mut nav, ephs, rx, v, up) = setup();
    let mut event = measurements(&ephs, rx, v, 50.0, up);
    // A transmit time off by a whole millisecond (300 km), as an ambiguous measurement can be.
    event.measurements[0].received_sv_time_nanos += 1_000_000;
    let sol = nav.on_measurements(&event).unwrap();
    assert!(
        norm(sub(sol.position, rx)) < 1.0,
        "position off by {} m",
        norm(sub(sol.position, rx))
    );
    assert!(
        norm(sub(sol.velocity, v)) < 0.01,
        "velocity off by {} m/s",
        norm(sub(sol.velocity, v))
    );
    assert!(
        sol.position_residual_rms < 1.0,
        "residual {}",
        sol.position_residual_rms
    );

    // Flagged as ambiguous, it isn't used at all.
    let mut event = measurements(&ephs, rx, v, 50.0, up);
    let n = event.measurements.len();
    event.measurements[0].state |= 0x10;
    let sol = nav.on_measurements(&event).unwrap();
    assert_eq!(sol.satellites_used + sol.satellites_rejected, n - 1);
}
