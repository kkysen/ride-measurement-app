// Scratch: the live map's path underground (`Fusion::recent_track_path`), kept as `LiveTrack.kt` keeps it
// at each smoothing tick, against the same legs placed after a whole-ride replay (`track::Along`).
use ride_fusion::fusion::SMOOTH_EVERY_NS;
use ride_fusion::replay::{Options, replay_with, replay_with_hook};
use ride_fusion::stations::meters;
use ride_fusion::track::{Along, legs};
use ride_session::read::Session;

type Leg = (i64, Vec<(i64, (f64, f64))>);

fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let s = Session::open(&dir)?;
        let mut kept: Vec<Leg> = Vec::new();
        let mut next_tick = i64::MIN;
        let mut ticks = 0;
        let mut tick_ns = 0u128;
        // Each tick's newest point of each part: where the map showed the train then.
        let mut newest: Vec<(i64, i64, (f64, f64))> = Vec::new();
        replay_with_hook(
            &s,
            &Options {
                live: true,
                skip_still: true,
                ..Options::default()
            },
            &mut |_| {},
            &mut |t, f| {
                if t < next_tick {
                    return;
                }
                next_tick = t + SMOOTH_EVERY_NS;
                let start = std::time::Instant::now();
                let (from, legs) = f.recent_track_path();
                tick_ns += start.elapsed().as_nanos();
                ticks += 1;
                for (l, p) in &legs {
                    if let Some((s, q)) = p.last() {
                        newest.push((*l, s.elapsed_realtime_ns, *q));
                    }
                }
                kept.retain(|(l, _)| *l < from && !legs.iter().any(|(n, _)| n == l));
                kept.extend(legs.into_iter().map(|(l, p)| {
                    (
                        l,
                        p.into_iter()
                            .map(|(s, q)| (s.elapsed_realtime_ns, q))
                            .collect(),
                    )
                }));
            },
        )?;
        let r = replay_with(
            &s,
            &Options {
                skip_still: true,
                ..Options::default()
            },
        )?;
        let (offline, _) = legs(&r.stops, &r.good_fix_ns, None);
        let t0 = r.smoothed.first().map_or(0, |x| x.elapsed_realtime_ns);
        let mut d: Vec<f64> = Vec::new();
        let mut matched = 0;
        for (from, points) in &kept {
            let Some(leg) = offline
                .iter()
                .find(|l| l.from_ns == *from && points.last().is_some_and(|p| p.0 <= l.to_ns))
            else {
                // Into or out of a tunnel, or the leg in progress at the end.
                if let (Some(a), Some(b)) = (points.first(), points.last()) {
                    let m: f64 = points.windows(2).map(|w| meters(w[0].1, w[1].1)).sum();
                    println!(
                        "  part at {}: {:.0} s, {m:.0} m, {:.5},{:.5} to {:.5},{:.5}",
                        mmss(a.0 - t0),
                        (b.0 - a.0) as f64 * 1e-9,
                        a.1.0,
                        a.1.1,
                        b.1.0,
                        b.1.1
                    );
                }
                continue;
            };
            matched += 1;
            let along = Along::new(leg, &r.smoothed);
            for &(t, p) in points {
                d.push(meters(p, along.at(t.clamp(leg.from_ns, leg.to_ns))));
            }
        }
        // Where the map showed the train at each tick, against where the whole-ride replay puts it then,
        // on the legs it later joins (the leg in progress, then the leg once the train got there).
        let mut shown: Vec<f64> = Vec::new();
        for &(l, t, q) in &newest {
            if let Some(leg) = offline.iter().find(|x| x.from_ns == l && t <= x.to_ns) {
                shown.push(meters(q, Along::new(leg, &r.smoothed).at(t)));
            }
        }
        for leg in &offline {
            let e: Vec<f64> = newest
                .iter()
                .filter(|&&(l, t, _)| l == leg.from_ns && t <= leg.to_ns)
                .map(|&(_, t, q)| meters(q, Along::new(leg, &r.smoothed).at(t)))
                .collect();
            let worst = e.iter().copied().fold(0.0, f64::max);
            println!(
                "  leg at {} ({:.0} s, {:.0} m): {} ticks, worst {worst:.0} m",
                mmss(leg.from_ns - t0),
                (leg.to_ns - leg.from_ns) as f64 * 1e-9,
                leg.track.length_m(),
                e.len()
            );
        }
        shown.sort_by(f64::total_cmp);
        let at = |v: &[f64], f: f64| {
            v.get(((v.len() as f64 - 1.0) * f) as usize)
                .copied()
                .unwrap_or(f64::NAN)
        };
        println!(
            "  newest point at each tick vs whole-ride: {} ticks, median {:.0} m, p90 {:.0} m, max {:.0} m",
            shown.len(),
            at(&shown, 0.5),
            at(&shown, 0.9),
            at(&shown, 1.0)
        );
        d.sort_by(f64::total_cmp);
        let q = |f: f64| {
            d.get(((d.len() as f64 - 1.0) * f) as usize)
                .copied()
                .unwrap_or(f64::NAN)
        };
        println!(
            "{dir}: live legs {} (points {}), offline legs {}, matched {matched}; \
             live vs offline point m: median {:.0} p90 {:.0} max {:.0}; tick {:.0} us mean over {ticks}",
            kept.len(),
            kept.iter().map(|(_, p)| p.len()).sum::<usize>(),
            offline.len(),
            q(0.5),
            q(0.9),
            q(1.0),
            tick_ns as f64 / ticks.max(1) as f64 / 1e3,
        );
    }
    Ok(())
}

fn mmss(ns: i64) -> String {
    let s = ns / 1_000_000_000;
    format!("{}:{:02}", s / 60, s % 60)
}
