// Scratch: each leg between stations, the live (forward) speed's distance against the track's length,
// to see how far off the live speed is underground and whether by a steady factor.
use ride_fusion::replay::{Options, replay_with};
use ride_fusion::track::legs;
use ride_session::read::Session;

fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let r = replay_with(
            &Session::open(&dir)?,
            &Options {
                live: true,
                skip_still: true,
                live_every_ns: Some(200_000_000),
                ..Options::default()
            },
        )?;
        let (legs, _) = legs(&r.stops, &r.good_fix_ns, None);
        let t0 = r.smoothed.first().map_or(0, |x| x.elapsed_realtime_ns);
        let mut ratios = Vec::new();
        for l in &legs {
            let i = r.live.partition_point(|(t, _)| *t < l.from_ns);
            let j = r.live.partition_point(|(t, _)| *t <= l.to_ns);
            let w = &r.live[i..j];
            let m: f64 = w
                .windows(2)
                .map(|p| (p[0].1.speed_mps + p[1].1.speed_mps) / 2.0 * (p[1].0 - p[0].0) as f64 * 1e-9)
                .sum();
            let peak = w.iter().map(|p| p.1.speed_mps).fold(0.0, f64::max);
            let s = (l.to_ns - l.from_ns) as f64 * 1e-9;
            let ratio = m / l.track.length_m();
            ratios.push(ratio);
            println!(
                "  {:>5}  {:>4.0} s  track {:>5.0} m  live {:>5.0} m  ratio {ratio:4.2}  avg {:4.1} mph  live peak {:4.1} mph",
                {
                    let s = (l.from_ns - t0) / 1_000_000_000;
                    format!("{}:{:02}", s / 60, s % 60)
                },
                s,
                l.track.length_m(),
                m,
                l.track.length_m() / s / 0.44704,
                peak / 0.44704
            );
        }
        ratios.sort_by(f64::total_cmp);
        println!(
            "{dir}: {} legs, live/track median {:.2}",
            ratios.len(),
            ratios.get(ratios.len() / 2).copied().unwrap_or(f64::NAN)
        );
    }
    Ok(())
}
