// Scratch: how often Wi-Fi scans and cell readings really refresh, and which networks offer ranging (802.11mc).
// `cargo run --release -p ride-cli --example radio_stats -- <session>...`
use std::collections::{BTreeMap, BTreeSet};

use ride_session::proto::record::Payload;
use ride_session::read::Session;

fn quantiles(mut x: Vec<f64>) -> String {
    if x.is_empty() {
        return "none".into();
    }
    x.sort_by(f64::total_cmp);
    let q = |p: f64| x[((x.len() - 1) as f64 * p) as usize];
    format!(
        "n {}, median {:.1}, 10% {:.1}, 90% {:.1}",
        x.len(),
        q(0.5),
        q(0.1),
        q(0.9)
    )
}

fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let s = Session::open(&dir)?;
        let mut scans = Vec::new(); // (received ns, requested, fresh results, all results)
        let mut responders: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        let mut networks: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        let mut snapshots = Vec::new(); // received ns
        // Registered cell's own timestamp, to see when Android actually re-measured it.
        let mut cell_ts = Vec::new();
        for r in s
            .read_records("radio")?
            .chunks
            .into_iter()
            .flat_map(|c| c.records)
        {
            let t = r.received_elapsed_realtime_ns;
            match r.payload {
                Some(Payload::WifiScan(w)) => {
                    // A result is from this scan if seen within 5 s before it was delivered.
                    let fresh = w
                        .results
                        .iter()
                        .filter(|x| (t / 1000 - x.timestamp_us).abs() < 5_000_000)
                        .count();
                    for x in &w.results {
                        networks
                            .entry(x.ssid.clone())
                            .or_default()
                            .insert(x.bssid.clone());
                        if x.is_80211mc_responder {
                            responders
                                .entry(x.ssid.clone())
                                .or_default()
                                .insert(x.bssid.clone());
                        }
                    }
                    scans.push((t, w.requested, fresh, w.results.len()));
                }
                Some(Payload::CellInfo(c)) => {
                    snapshots.push(t);
                    if let Some(reg) = c.cells.iter().find(|c| c.registered) {
                        cell_ts.push(reg.timestamp_ms);
                    }
                }
                _ => {}
            }
        }
        println!("{dir}");
        let new_scans: Vec<_> = scans.iter().filter(|s| s.2 > 0).collect();
        let gaps = |t: Vec<i64>| {
            t.windows(2)
                .map(|w| (w[1] - w[0]) as f64 * 1e-9)
                .collect::<Vec<_>>()
        };
        println!(
            "  Wi-Fi: {} deliveries ({} requested), {} with fresh results; s between fresh ones: {}",
            scans.len(),
            scans.iter().filter(|s| s.1).count(),
            new_scans.len(),
            quantiles(gaps(new_scans.iter().map(|s| s.0).collect()))
        );
        println!(
            "  Wi-Fi fresh share of results per delivery: {}",
            quantiles(
                scans
                    .iter()
                    .map(|s| s.2 as f64 / s.3.max(1) as f64)
                    .collect()
            )
        );
        let mut ts: Vec<i64> = cell_ts.clone();
        ts.dedup();
        println!(
            "  cell: {} snapshots, s between: {}; registered cell re-measured {} times, s between: {}",
            snapshots.len(),
            quantiles(gaps(snapshots)),
            ts.len(),
            quantiles(ts.windows(2).map(|w| (w[1] - w[0]) as f64 * 1e-3).collect())
        );
        let total: usize = networks.values().map(|b| b.len()).sum();
        let resp: usize = responders.values().map(|b| b.len()).sum();
        println!("  802.11mc responders: {resp} of {total} access points");
        for (ssid, b) in &responders {
            println!("    {ssid}: {} of {}", b.len(), networks[ssid].len());
        }
    }
    Ok(())
}
