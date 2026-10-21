// Scratch: when the subway detector switches, with the Wi-Fi and fixes around it (s from the first record).
use ride_fusion::subway::{SubwayDetector, is_subway_ssid};
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::GPS_PROVIDER;
fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    let mut ev: Vec<(i64, Payload)> = Vec::new();
    for stream in ["location", "radio"] {
        for r in s
            .read_records(stream)?
            .chunks
            .into_iter()
            .flat_map(|c| c.records)
        {
            if let Some(p) = r.payload {
                ev.push((r.received_elapsed_realtime_ns, p));
            }
        }
    }
    ev.sort_by_key(|e| e.0);
    let t0 = ev[0].0;
    let mut d = SubwayDetector::default();
    let mut was = false;
    for (t, p) in ev {
        let note = match &p {
            Payload::Location(l) if l.provider == GPS_PROVIDER => {
                d.on_fix(t, l.horizontal_accuracy_meters.map(f64::from));
                None
            }
            Payload::WifiScan(w) => {
                d.on_wifi_scan(t, w.results.iter().map(|r| r.ssid.as_str()));
                let sub = w.results.iter().filter(|r| is_subway_ssid(&r.ssid)).count();
                Some(format!("scan: {} networks, {sub} subway", w.results.len()))
            }
            _ => None,
        };
        let now = d.in_subway(t);
        if now != was {
            println!(
                "{:7.1} s  {}  ({})",
                (t - t0) as f64 * 1e-9,
                if now { "IN" } else { "out" },
                note.unwrap_or_default()
            );
            was = now;
        }
    }
    Ok(())
}
