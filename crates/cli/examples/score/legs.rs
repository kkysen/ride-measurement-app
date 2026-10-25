// Scratch: each leg between stations joined along the track (`ride_fusion::track::legs`):
// its time, the track's length, the smoothed speeds' distance, and their ratio.
use ride_fusion::replay::Replay;
use ride_fusion::track::{covered_m, legs};
pub fn run(dir: &str, r: &Replay, out: &mut String) -> anyhow::Result<()> {
    {
        let (legs, stats) = legs(&r.stops, &r.good_fix_ns, None);
        for l in &legs {
            let d = covered_m(&r.smoothed, l.from_ns, l.to_ns);
            let s = (l.to_ns - l.from_ns) as f64 * 1e-9;
            out!(
                out,
                "leg {:>3}:{:02} track {:5.0} m speeds {:5.0} m ratio {:4.2} avg {:4.1} mph",
                (s / 60.0) as i64,
                s as i64 % 60,
                l.track.length_m(),
                d,
                d / l.track.length_m(),
                l.track.length_m() / s / 0.44704
            );
        }
        for st in &r.stops {
            if let (Some(s), Some((m, v))) = (st.station, st.from_last_station) {
                out!(
                    out,
                    "  at {:<24} {m:5.0} m from the last station, average {:4.1} mph",
                    s.name,
                    v / 0.44704
                );
            }
        }
        out!(out, "legs: {dir}: {} legs; {stats:?}", legs.len());
    }
    Ok(())
}
