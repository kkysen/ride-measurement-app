// Scratch: the marks, and each stretch where the smoothed speed is over 55 mph (s from the first sample).
use ride_fusion::replay::replay;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    let r = replay(&s)?;
    let t0 = r.smoothed[0].elapsed_realtime_ns;
    let sec = |t: i64| (t - t0) as f64 * 1e-9;
    for rec in s
        .read_records("events")?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
    {
        if let Some(Payload::ManualEvent(e)) = &rec.payload {
            println!(
                "{:7.0} s  {}",
                sec(rec.received_elapsed_realtime_ns),
                e.kind().as_str_name()
            );
        }
    }
    let mut start: Option<(f64, f64)> = None;
    for sm in &r.smoothed {
        let t = sec(sm.elapsed_realtime_ns);
        let over = sm.speed_mps > 55.0 * 0.44704;
        match (over, start) {
            (true, None) => start = Some((t, sm.speed_mps)),
            (true, Some((t1, top))) => start = Some((t1, top.max(sm.speed_mps))),
            (false, Some((t1, top))) => {
                println!(
                    "{t1:7.0} s to {t:7.0} s over 55 mph, top {:.0} mph",
                    top / 0.44704
                );
                start = None;
            }
            _ => {}
        }
    }
    Ok(())
}
