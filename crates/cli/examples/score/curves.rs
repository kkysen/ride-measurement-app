// Scratch: scores the forward (live) speed of a ride, for changes to the IMU's projection (curves):
// how long it's at 50 mph or more with no GNSS fix within 10 s (runaways underground; trains cruise 40 to 45),
// the median at the rider's "train stopped" taps (should be 0),
// and with GNSS cut 30 s of every 90 s, the RMS error inside the cuts against the phone's confident fix speeds.
use ride_fusion::replay::{GnssOutages, Replay};
use ride_session::marks::marks;
use ride_session::proto::manual_event::Kind;
use ride_session::read::Session;
use ride_session::track::gps_segments;
pub fn run(s: &Session, r: &Replay, rc: &Replay, out: &mut String) -> anyhow::Result<()> {
    let fixes: Vec<_> = if s.stream_path("location").exists() {
        gps_segments(&s.read_records("location")?.chunks)
            .into_iter()
            .flatten()
            .collect()
    } else {
        Vec::new()
    };
    let mph = |v: f64| v / 0.44704;
    let near_fix = |t: i64| {
        let i = fixes.partition_point(|f| f.elapsed_realtime_ns < t);
        [i.checked_sub(1), Some(i)]
            .into_iter()
            .flatten()
            .filter_map(|i| fixes.get(i))
            .any(|f| (f.elapsed_realtime_ns - t).abs() < 10_000_000_000)
    };
    let mut fast_s = 0.0;
    let dump = std::env::var("DUMP").is_ok();
    let t0 = r.smoothed.first().map_or(0, |x| x.elapsed_realtime_ns);
    let mut run: Option<(i64, f64)> = None;
    for w in r.smoothed.windows(2) {
        let v = mph(w[0].forward_mps.abs());
        if v >= 50.0 && !near_fix(w[0].elapsed_realtime_ns) {
            fast_s += (w[1].elapsed_realtime_ns - w[0].elapsed_realtime_ns) as f64 * 1e-9;
            let e = run.get_or_insert((w[0].elapsed_realtime_ns, 0.0));
            e.1 = e.1.max(v);
        } else if let Some((st, pk)) = run.take()
            && dump
        {
            out!(
                out,
                "  fast at +{:.0} s for {:.0} s peak {:.0} mph",
                (st - t0) as f64 * 1e-9,
                (w[0].elapsed_realtime_ns - st) as f64 * 1e-9,
                pk
            );
        }
    }
    let at = |t: i64| {
        let i = r.smoothed.partition_point(|x| x.elapsed_realtime_ns < t);
        r.smoothed.get(i).map(|x| mph(x.forward_mps.abs()))
    };
    if dump {
        for m in marks(s)?.iter().filter(|m| !m.deleted) {
            out!(
                out,
                "  mark +{:.0} s {:?} {:?}",
                (m.t_ns - t0) as f64 * 1e-9,
                m.kind,
                m
            );
        }
    }
    let mut stopped: Vec<f64> = marks(s)?
        .iter()
        .filter(|m| !m.deleted && m.kind == Kind::TrainStopped)
        .filter_map(|m| at(m.t_ns + 3_000_000_000))
        .collect();
    stopped.sort_by(f64::total_cmp);
    let median = stopped.get(stopped.len() / 2).copied().unwrap_or(f64::NAN);

    let first = outages_start(r);
    let mut sq = 0.0;
    let mut n = 0;
    for f in &fixes {
        let (Some(v), Some(acc)) = (
            f.speed_meters_per_second,
            f.speed_accuracy_meters_per_second,
        ) else {
            continue;
        };
        let phase = ((f.elapsed_realtime_ns - first) as f64 * 1e-9).rem_euclid(90.0);
        if acc > 0.5 || phase < 60.0 {
            continue;
        }
        let i = rc
            .smoothed
            .partition_point(|x| x.elapsed_realtime_ns < f.elapsed_realtime_ns);
        if let Some(x) = rc.smoothed.get(i) {
            let e = mph(x.forward_mps.abs() - f64::from(v));
            if dump && e.abs() > 10.0 {
                out!(
                    out,
                    "  cut err at +{:.0} s: {:+.0} mph (fix {:.0})",
                    (f.elapsed_realtime_ns - first) as f64 * 1e-9,
                    e,
                    mph(f64::from(v))
                );
            }
            sq += e.powi(2);
            n += 1;
        }
    }
    out!(
        out,
        "curves: {:>16}  >=50 mph unaided {:6.0} s  at stopped taps median {:5.1} mph (n {:2})  outage rms {:5.1} mph (n {})",
        s.dir.file_name().unwrap().to_string_lossy(),
        fast_s,
        median,
        stopped.len(),
        (sq / n.max(1) as f64).sqrt(),
        n
    );
    Ok(())
}

/// Where `rc`'s outages start: the first smoothed speed of the uncut replay, as `curves` did.
pub fn outages_start(r: &Replay) -> i64 {
    r.smoothed.first().map_or(0, |x| x.elapsed_realtime_ns)
}

/// The GNSS cut `rc` is replayed with: 30 s of every 90 s.
pub fn outages(r: &Replay) -> GnssOutages {
    GnssOutages {
        start_ns: outages_start(r),
        outage_s: 30.0,
        period_s: 90.0,
    }
}
