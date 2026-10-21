// Scratch: each Wi-Fi scan between two times (s from the first radio record): networks, how many seen in the
// last 15 s (results can be cached from earlier scans), subway networks among those, and the latest fix's age.
use ride_fusion::subway::is_subway_ssid;
use ride_session::proto::record::Payload;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let mut a = std::env::args().skip(1);
    let s = Session::open(a.next().unwrap())?;
    let from: f64 = a.next().unwrap().parse()?;
    let to: f64 = a.next().unwrap().parse()?;
    let mut t0 = None;
    for r in s
        .read_records("radio")?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
    {
        let t = r.received_elapsed_realtime_ns;
        let t0 = *t0.get_or_insert(t);
        let Some(Payload::WifiScan(w)) = r.payload else {
            continue;
        };
        let ts = (t - t0) as f64 * 1e-9;
        if !(from..=to).contains(&ts) {
            continue;
        }
        let fresh: Vec<_> = w
            .results
            .iter()
            .filter(|x| t / 1000 - x.timestamp_us <= 15_000_000)
            .collect();
        let sub: Vec<_> = fresh
            .iter()
            .filter(|x| is_subway_ssid(&x.ssid))
            .map(|x| x.ssid.trim())
            .collect();
        println!(
            "{ts:7.1} s  {:3} results, {:3} fresh, subway {:?}  requested {}",
            w.results.len(),
            fresh.len(),
            sub,
            w.requested
        );
    }
    Ok(())
}
