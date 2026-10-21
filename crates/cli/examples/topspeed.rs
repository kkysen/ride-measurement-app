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
    Ok(())
}
