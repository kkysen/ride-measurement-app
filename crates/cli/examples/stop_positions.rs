// Scratch: at each subway stop at a named station, how far the position is from the station (m):
// dead reckoned (GNSS, the IMU, and the station pins), joined along the track (`ride_fusion::track::join`),
// which puts a joined stop on its station by construction, and joined with that stop left out as an anchor
// (its neighbors joined across it, its position from the speeds along the track): the honest score.
use ride_fusion::position::PositionEstimate;
use ride_fusion::replay::replay;
use ride_fusion::stations::meters;
use ride_fusion::track::join;
use ride_session::read::Session;

fn at(positions: &[PositionEstimate], t: i64) -> Option<&PositionEstimate> {
    positions
        .iter()
        .min_by_key(|p| (p.elapsed_realtime_ns - t).abs())
}

fn summary(name: &str, mut e: Vec<f64>) -> String {
    e.sort_by(f64::total_cmp);
    if e.is_empty() {
        return format!("{name} none");
    }
    format!(
        "{name} median {:.0} m, 90th {:.0} m",
        e[e.len() / 2],
        e[e.len() * 9 / 10]
    )
}

fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let r = replay(&Session::open(&dir)?)?;
        let stats = join(
            &mut r.dead_reckoned.clone(),
            &r.smoothed,
            &r.stops,
            &r.good_fix_ns,
            None,
        );
        let (mut dr, mut joined, mut loo, mut loo_dr) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for (k, s) in r.stops.iter().enumerate() {
            if !s.in_subway {
                continue;
            }
            let Some(station) = s.station else { continue };
            let (Some(p), Some(q)) = (
                at(&r.dead_reckoned, s.stopped_ns),
                at(&r.positions, s.stopped_ns),
            ) else {
                continue;
            };
            let mut left_out = r.dead_reckoned.clone();
            join(
                &mut left_out,
                &r.smoothed,
                &r.stops,
                &r.good_fix_ns,
                Some(k),
            );
            let o = at(&left_out, s.stopped_ns).unwrap();
            let d = |p: &PositionEstimate| meters((p.lat, p.lon), (station.lat, station.lon));
            let (a, b, c) = (d(p), d(q), d(o));
            println!(
                "{dir} {:<28} dead reckoned {a:6.0} m  joined {b:6.0} m  left out {c:6.0} m  sigma {:6.0} m",
                station.name, p.sigma_m
            );
            dr.push(a);
            joined.push(b);
            // Placed along the track with it left out: its neighbors were joined across it.
            if (o.lat, o.lon) != (p.lat, p.lon) {
                loo.push(c);
                loo_dr.push(a);
            }
        }
        if !dr.is_empty() {
            println!(
                "{dir}: {} stops; {}; {}; {} placed left out: {}, {}; {stats:?}",
                dr.len(),
                summary("dead reckoned", dr),
                summary("joined", joined),
                loo.len(),
                summary("left out", loo),
                summary("dead reckoned", loo_dr),
            );
        }
    }
    Ok(())
}
