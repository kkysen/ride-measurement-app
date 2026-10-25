// Scratch: the speed between stations held to the track, scored where GNSS knows the truth:
// a ride replayed with GNSS cut out throughout (as underground), its legs between stations
// (`ride_fusion::track::legs`), and in each the smoothed speed against the phone's confident fix speeds
// from the ride as recorded: RMS, and the distance each covers over the leg.
use ride_fusion::replay::{GnssOutages, Replay};
use ride_fusion::track::{covered_m, legs};
use ride_session::read::Session;
use ride_session::track::gps_segments;
pub fn run(s: &Session, r: &Replay, uncut: &Replay, out: &mut String) -> anyhow::Result<()> {
    if !s.stream_path("location").exists() {
        return Ok(());
    }
    let fixes: Vec<_> = gps_segments(&s.read_records("location")?.chunks)
        .into_iter()
        .flatten()
        .collect();
    let (legs, stats) = legs(&r.stops, &r.good_fix_ns, None);
    let mph = |v: f64| v / 0.44704;
    let (mut sq, mut n) = (0.0, 0);
    for l in &legs {
        let inside: Vec<_> = fixes
            .iter()
            .filter(|f| f.elapsed_realtime_ns > l.from_ns && f.elapsed_realtime_ns < l.to_ns)
            .filter(|f| f.speed_accuracy_meters_per_second.is_some_and(|a| a <= 0.5))
            .collect();
        let mut leg_sq = 0.0;
        let mut gnss_m = 0.0;
        for w in inside.windows(2) {
            let dt = (w[1].elapsed_realtime_ns - w[0].elapsed_realtime_ns) as f64 * 1e-9;
            if dt < 3.0 {
                gnss_m += f64::from(
                    w[0].speed_meters_per_second.unwrap() + w[1].speed_meters_per_second.unwrap(),
                ) / 2.0
                    * dt;
            }
        }
        for f in &inside {
            let i = r
                .smoothed
                .partition_point(|x| x.elapsed_realtime_ns < f.elapsed_realtime_ns);
            if let Some(x) = r.smoothed.get(i) {
                leg_sq += mph(x.speed_mps - f64::from(f.speed_meters_per_second.unwrap())).powi(2);
            }
        }
        sq += leg_sq;
        n += inside.len();
        let fixed = uncut
            .good_fix_ns
            .iter()
            .filter(|&&t| t > l.from_ns && t < l.to_ns)
            .count();
        let secs = (l.to_ns - l.from_ns) as f64 * 1e-9;
        let truth = if fixed as f64 >= secs / 5.0 {
            format!("{:.0}", covered_m(&uncut.smoothed, l.from_ns, l.to_ns))
        } else {
            "-".into()
        };
        out!(
            out,
            "  leg {:.0} s: track {:.0} m, smoothed {:.0} m, truth {truth} m, GNSS {:.0} m over {} fixes, rms {:.1} mph",
            secs,
            l.track.length_m(),
            covered_m(&r.smoothed, l.from_ns, l.to_ns),
            gnss_m,
            inside.len(),
            (leg_sq / inside.len().max(1) as f64).sqrt()
        );
    }
    out!(
        out,
        "legs_gnss: {}: {} legs, {stats:?}; rms {:.1} mph over {n} fixes",
        s.dir.file_name().unwrap().to_string_lossy(),
        legs.len(),
        (sq / n.max(1) as f64).sqrt()
    );
    Ok(())
}

/// The GNSS cut `r` is replayed with: all of it, as underground.
pub fn outages() -> GnssOutages {
    GnssOutages {
        start_ns: 0,
        outage_s: 1e12,
        period_s: 1e12,
    }
}
