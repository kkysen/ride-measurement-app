// Scratch: whole-ride average speed by smoothed speed vs. by fix path at several spacings.
use ride_fusion::replay::replay;
use ride_session::read::Session;
use ride_session::track::gps_segments;

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    let r = 6_371_000.0;
    let (la1, lo1, la2, lo2) = (
        a.0.to_radians(),
        a.1.to_radians(),
        b.0.to_radians(),
        b.1.to_radians(),
    );
    let h = ((la2 - la1) / 2.0).sin().powi(2)
        + la1.cos() * la2.cos() * ((lo2 - lo1) / 2.0).sin().powi(2);
    2.0 * r * h.sqrt().asin()
}

fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let session = Session::open(&dir)?;
        let r = replay(&session)?;
        let s = &r.smoothed;
        let (Some(first), Some(last)) = (s.first(), s.last()) else {
            println!("{dir}: no samples");
            continue;
        };
        let span = (last.elapsed_realtime_ns - first.elapsed_realtime_ns) as f64 * 1e-9;
        let mut d = 0.0;
        let mut fwd = 0.0;
        for w in s.windows(2) {
            let dt = (w[1].elapsed_realtime_ns - w[0].elapsed_realtime_ns) as f64 * 1e-9;
            d += (w[0].speed_mps + w[1].speed_mps) / 2.0 * dt;
            fwd += (w[0].forward_mps + w[1].forward_mps) / 2.0 * dt;
        }
        let kmh = |m: f64, t: f64| m / t * 3.6;
        println!(
            "{dir}: {:.0} s, smoothed {:.0} m ({:.1} km/h), forward {:.0} m ({:.1} km/h)",
            span,
            d,
            kmh(d, span),
            fwd,
            kmh(fwd, span)
        );
        let loc = session.read_records("location")?;
        let segs = gps_segments(&loc.chunks);
        let pts: Vec<_> = segs.iter().flatten().collect();
        let covered: f64 = segs
            .iter()
            .filter_map(|g| {
                Some((g.last()?.elapsed_realtime_ns - g.first()?.elapsed_realtime_ns) as f64 * 1e-9)
            })
            .sum();
        println!(
            "  fixes {} in {} segments covering {:.0} s",
            pts.len(),
            segs.len(),
            covered
        );
        for spacing in [0.0, 25.0, 50.0, 100.0, 200.0, 400.0] {
            // Path through fixes, keeping a fix once it's `spacing` from the last kept one; gaps between segments bridged straight.
            let mut total = 0.0;
            let mut kept: Option<(f64, f64)> = None;
            for p in &pts {
                let q = (p.latitude_degrees, p.longitude_degrees);
                match kept {
                    None => kept = Some(q),
                    Some(k) => {
                        let x = dist(k, q);
                        if x >= spacing {
                            total += x;
                            kept = Some(q);
                        }
                    }
                }
            }
            let t = (pts.last().unwrap().elapsed_realtime_ns - pts[0].elapsed_realtime_ns) as f64
                * 1e-9;
            println!(
                "  path @{spacing:>3} m: {:.0} m over {:.0} s ({:.1} km/h)",
                total,
                t,
                kmh(total, t)
            );
        }
    }
    Ok(())
}
