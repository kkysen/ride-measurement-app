// Scratch: per fix in [from, to] s: phone speed and accuracy vs. forward and smoothed fusion.
use ride_fusion::replay::replay;
use ride_session::read::Session;
use ride_session::track::gps_segments;
fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let s = Session::open(&a[1])?;
    let (from, to): (f64, f64) = (a[2].parse()?, a[3].parse()?);
    let r = replay(&s)?;
    let loc = s.read_records("location")?;
    let segs = gps_segments(&loc.chunks);
    let pts: Vec<_> = segs.iter().flatten().collect();
    let t0 = pts[0].elapsed_realtime_ns;
    for p in &pts {
        let t = (p.elapsed_realtime_ns - t0) as f64 * 1e-9;
        if t < from || t > to {
            continue;
        }
        let f = r.at(p.elapsed_realtime_ns);
        println!(
            "{t:5.0} s  phone {:5.1} ±{:4.1} km/h   fwd {:5.1}  smoothed {:5.1} ±{:4.1}",
            p.speed_meters_per_second.unwrap_or(f32::NAN) * 3.6,
            p.speed_accuracy_meters_per_second.unwrap_or(f32::NAN) * 3.6,
            f.map_or(f64::NAN, |f| f.forward_mps * 3.6),
            f.map_or(f64::NAN, |f| f.speed_mps * 3.6),
            f.map_or(f64::NAN, |f| f.sigma_mps * 3.6)
        );
    }
    println!("gnss: {:?}", r.gnss);
    Ok(())
}
