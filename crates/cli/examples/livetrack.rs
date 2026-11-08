// Scratch: the live map's path underground (`Fusion::recent_track_path`), kept as `LiveTrack.kt` keeps it
// at each smoothing tick, against the same legs placed after a whole-ride replay (`track::Along`).
use ride_fusion::fusion::SMOOTH_EVERY_NS;
use ride_fusion::replay::{Options, replay, replay_with_hook};
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
        replay_with_hook(
            &s,
            &Options {
                live: true,
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
                kept.retain(|(l, _)| *l < from);
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
        let r = replay(&s)?;
        let (offline, _) = legs(&r.stops, &r.good_fix_ns, None);
        let mut d: Vec<f64> = Vec::new();
        let mut matched = 0;
        for (from, points) in &kept {
            let Some(leg) = offline
                .iter()
                .find(|l| (l.from_ns - from).abs() < 5_000_000_000)
            else {
                continue;
            };
            matched += 1;
            let along = Along::new(leg, &r.smoothed);
            for &(t, p) in points {
                d.push(meters(p, along.at(t.clamp(leg.from_ns, leg.to_ns))));
            }
        }
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
