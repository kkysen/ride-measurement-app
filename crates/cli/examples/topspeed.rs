// Scratch: the replay's top smoothed and forward speeds, and how long the smoothed speed is over 55 mph.
use ride_fusion::replay::replay;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let r = replay(&Session::open(std::env::args().nth(1).unwrap())?)?;
    let top = r.smoothed.iter().map(|s| s.speed_mps).fold(0.0, f64::max);
    let fwd = r.smoothed.iter().map(|s| s.forward_mps).fold(0.0, f64::max);
    let over = r
        .smoothed
        .iter()
        .filter(|s| s.speed_mps > 55.0 * 0.44704)
        .count();
    println!(
        "top {:.0} mph, forward top {:.0} mph, {over} samples over 55 mph",
        top / 0.44704,
        fwd / 0.44704
    );
    // Stretches where the forward pass (the live display) is over 55 mph.
    let t0 = r.smoothed[0].elapsed_realtime_ns;
    let mut start: Option<(f64, f64)> = None;
    for s in &r.smoothed {
        let t = (s.elapsed_realtime_ns - t0) as f64 * 1e-9;
        match (s.forward_mps > 55.0 * 0.44704 + 0.5, start) {
            (true, None) => start = Some((t, s.forward_mps)),
            (true, Some((a, m))) => start = Some((a, m.max(s.forward_mps))),
            (false, Some((a, m))) => {
                println!(
                    "  forward over 55 mph {a:.0} to {t:.0} s, top {:.0} mph",
                    m / 0.44704
                );
                start = None;
            }
            _ => {}
        }
    }
    Ok(())
}
