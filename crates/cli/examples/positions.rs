// Scratch: smoothed positions against the GPS fixes, and through the gaps between them.
use ride_fusion::live_speed::SUBWAY_MAX_SPEED_MPS;
use ride_fusion::replay::{Options, replay_with};
use ride_session::read::Session;
use ride_session::track::gps_segments;

fn meters(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dn = (a.0 - b.0).to_radians() * 6_371_000.0;
    let de = (a.1 - b.1).to_radians() * 6_371_000.0 * a.0.to_radians().cos();
    dn.hypot(de)
}

fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    // `SUBWAY=1`: the subway's top speed for the whole ride.
    let options = Options {
        max_speed_mps: std::env::var_os("SUBWAY").map(|_| SUBWAY_MAX_SPEED_MPS),
        ..Options::default()
    };
    let r = replay_with(&s, &options)?;
    let ps = &r.positions;
    println!("{} positions", ps.len());
    let Some(first) = ps.first() else {
        return Ok(());
    };
    let t0 = first.elapsed_realtime_ns;
    let loc = s.read_records("location")?;
    let fixes: Vec<_> = gps_segments(&loc.chunks).into_iter().flatten().collect();
    // Each fix against the position at its time (not independent: the fix is one of the inputs).
    let mut d: Vec<f64> = fixes
        .iter()
        .filter_map(|f| {
            let i = ps.partition_point(|p| p.elapsed_realtime_ns < f.elapsed_realtime_ns);
            let p = ps.get(i)?;
            (p.elapsed_realtime_ns == f.elapsed_realtime_ns)
                .then(|| meters((p.lat, p.lon), (f.latitude_degrees, f.longitude_degrees)))
        })
        .collect();
    d.sort_by(f64::total_cmp);
    if !d.is_empty() {
        println!(
            "fix vs position: {} fixes, median {:.1} m, 95% {:.1} m, max {:.1} m",
            d.len(),
            d[d.len() / 2],
            d[d.len() * 95 / 100],
            d[d.len() - 1]
        );
    }
    // Every 30 s: position, sigma, speed, and the nearest fix in time.
    let mut next = 0.0;
    for p in ps {
        let t = (p.elapsed_realtime_ns - t0) as f64 * 1e-9;
        if t < next {
            continue;
        }
        next += 30.0;
        let near = fixes
            .iter()
            .min_by_key(|f| (f.elapsed_realtime_ns - p.elapsed_realtime_ns).abs());
        let fix = near.map_or(String::new(), |f| {
            format!(
                "  fix {:+5.0} s away, {:6.0} m off",
                (f.elapsed_realtime_ns - p.elapsed_realtime_ns) as f64 * 1e-9,
                meters((p.lat, p.lon), (f.latitude_degrees, f.longitude_degrees))
            )
        });
        println!(
            "{t:6.0} s ({:.1})  {:.5},{:.5}  ±{:6.1} m  {:5.1} km/h{fix}",
            p.elapsed_realtime_ns as f64 * 1e-9,
            p.lat,
            p.lon,
            p.sigma_m,
            p.velocity[0].hypot(p.velocity[1]) * 3.6
        );
    }
    Ok(())
}
