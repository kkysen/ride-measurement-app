// Scratch: smoothed speed every 60 s, plus forward/backward, sigma.
use ride_fusion::replay::replay;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let dir = std::env::args().nth(1).unwrap();
    let r = replay(&Session::open(&dir)?)?;
    let t0 = r.smoothed[0].elapsed_realtime_ns;
    let mut next = 0.0;
    for s in &r.smoothed {
        let t = (s.elapsed_realtime_ns - t0) as f64 * 1e-9;
        if t >= next {
            println!(
                "{t:6.0} s  {:7.1} km/h ± {:6.1}  fwd {:7.1}  bwd {:7.1}",
                s.speed_mps * 3.6,
                s.sigma_mps * 3.6,
                s.forward_mps * 3.6,
                s.backward_mps * 3.6
            );
            next += 60.0;
        }
    }
    Ok(())
}
