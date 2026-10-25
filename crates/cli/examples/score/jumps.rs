// Scratch: how continuous the joined path (`Replay::positions`, as the map draws it, one point a second) is:
// the largest step between consecutive points, the count of steps implying over 55 mph,
// how many of those the dead-reckoned path doesn't make (seams the joining adds),
// and how far the joined path is from the dead-reckoned one at the good fixes (where the path is GNSS's own).
use ride_fusion::replay::Replay;
use ride_fusion::stations::meters;

const MAX_MPS: f64 = 55.0 * 0.44704;

pub fn run(dir: &str, r: &Replay, out: &mut String) -> anyhow::Result<()> {
    {
        let path = r.map_path(1_000_000_000);
        let (mut max_m, mut max_at, mut over, mut seams) = (0.0f64, 0i64, 0usize, 0usize);
        let t0 = r.positions.first().map_or(0, |p| p.elapsed_realtime_ns);
        for w in path.windows(2) {
            let (a, b) = (w[0].0, w[1].0);
            let d = meters((a.lat, a.lon), (b.lat, b.lon));
            let dt = (b.elapsed_realtime_ns - a.elapsed_realtime_ns) as f64 * 1e-9;
            if d > max_m {
                (max_m, max_at) = (d, a.elapsed_realtime_ns);
            }
            // Steps under 20 m are GNSS noise between close samples, not motion.
            if d > 20.0 && d / dt.max(1.0) > MAX_MPS {
                over += 1;
                // The dead-reckoned path between the same samples: a step it doesn't make is a seam.
                let i = r
                    .positions
                    .partition_point(|p| p.elapsed_realtime_ns < a.elapsed_realtime_ns);
                let j = r
                    .positions
                    .partition_point(|p| p.elapsed_realtime_ns < b.elapsed_realtime_ns);
                let (da, db) = (&r.dead_reckoned[i], &r.dead_reckoned[j]);
                if meters((da.lat, da.lon), (db.lat, db.lon)) / dt.max(1.0) <= MAX_MPS {
                    seams += 1;
                    if std::env::var("JUMPS_LIST").is_ok() {
                        let s = (a.elapsed_realtime_ns - t0) / 1_000_000_000;
                        out!(out, "  seam {}:{:02} {d:.0} m", s / 60, s % 60);
                    }
                }
            }
        }
        let mut off: Vec<f64> = r
            .good_fix_ns
            .iter()
            .filter_map(|&t| {
                let i = r
                    .positions
                    .iter()
                    .position(|p| p.elapsed_realtime_ns == t)?;
                let (p, q) = (&r.positions[i], &r.dead_reckoned.get(i)?);
                Some(meters((p.lat, p.lon), (q.lat, q.lon)))
            })
            .collect();
        off.sort_by(f64::total_cmp);
        let q = |f: f64| {
            off.get(((off.len() as f64 * f) as usize).min(off.len().saturating_sub(1)))
                .copied()
                .unwrap_or(0.0)
        };
        let s = (max_at - t0) / 1_000_000_000;
        out!(
            out,
            "jumps: {dir}: {} points; max step {max_m:.0} m at {}:{:02}; {over} steps over 55 mph ({seams} where the dead-reckoned path isn't); at {} good fixes off the dead-reckoned path median {:.0} m, 90th {:.0} m, max {:.0} m",
            path.len(),
            s / 60,
            s % 60,
            off.len(),
            q(0.5),
            q(0.9),
            q(1.0),
        );
    }
    Ok(())
}
