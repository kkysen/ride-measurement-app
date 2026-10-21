// Scratch: fixes each minute (local time), and fix gaps.
use ride_session::read::Session;
use ride_session::track::gps_segments;
fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    let loc = s.read_records("location")?;
    let segs = gps_segments(&loc.chunks);
    let pts: Vec<_> = segs.iter().flatten().collect();
    let t0 = pts[0].elapsed_realtime_ns;
    let mut next = 0.0;
    let mut prev = t0;
    for p in &pts {
        let t = (p.elapsed_realtime_ns - t0) as f64 * 1e-9;
        let gap = (p.elapsed_realtime_ns - prev) as f64 * 1e-9;
        let local = (p.time_ms / 1000 - 4 * 3600).rem_euclid(86400);
        if t >= next || gap > 5.0 {
            println!(
                "{t:6.0} s {:02}:{:02}:{:02}  {:.5},{:.5}  {:5.1} km/h ±{:?} m{}",
                local / 3600,
                local / 60 % 60,
                local % 60,
                p.latitude_degrees,
                p.longitude_degrees,
                p.speed_meters_per_second.unwrap_or(f32::NAN) * 3.6,
                p.horizontal_accuracy_meters,
                if gap > 5.0 {
                    format!("  (after {gap:.0} s gap)")
                } else {
                    String::new()
                }
            );
            if t >= next {
                next += 60.0;
            }
        }
        prev = p.elapsed_realtime_ns;
    }
    Ok(())
}
