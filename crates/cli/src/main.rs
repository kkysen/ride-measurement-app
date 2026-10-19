use std::path::PathBuf;

use anyhow::Context;
use clap::{Parser, Subcommand};
use ride_session::proto::record::Payload;
use ride_session::proto::{StreamKind, manual_event};
use ride_session::read::Session;

#[derive(Parser)]
#[command(about = "Inspect ride sessions")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Per-stream sample counts, rates, and gaps.
    Summary { session: PathBuf },
    /// Print one stream as CSV: timestamp_ns, accuracy, values...
    Csv { session: PathBuf, stream: String },
}

fn main() -> anyhow::Result<()> {
    match Cli::parse().command {
        Command::Summary { session } => summary(&Session::open(session)?),
        Command::Csv { session, stream } => csv(&Session::open(session)?, &stream),
    }
}

fn summary(session: &Session) -> anyhow::Result<()> {
    let info = &session.info;
    println!(
        "session {} (format v{})",
        info.session_id, info.format_version
    );
    if !info.continues_session_id.is_empty() {
        println!(
            "continues session {} (Android killed recording and it restarted)",
            info.continues_session_id
        );
    }
    if let Some(d) = &info.device {
        println!("device {} {} (SDK {})", d.manufacturer, d.model, d.sdk_int);
    }
    for stream in &info.streams {
        if stream.kind() == StreamKind::Records {
            record_summary(session, &stream.name)?;
            continue;
        }
        let contents = session
            .read_stream(&stream.name)
            .with_context(|| format!("stream {}", stream.name))?;
        let timestamps: Vec<i64> = contents
            .chunks
            .iter()
            .flat_map(|c| c.timestamps_ns())
            .collect();
        print!(
            "{}: {} chunks, {} samples",
            stream.name,
            contents.chunks.len(),
            timestamps.len()
        );
        if contents.truncated {
            print!(", TRUNCATED tail");
        }
        println!();
        if timestamps.len() < 2 {
            continue;
        }
        let mut dts: Vec<i64> = timestamps.windows(2).map(|w| w[1] - w[0]).collect();
        let non_monotonic = dts.iter().filter(|&&dt| dt <= 0).count();
        let duration_s = (timestamps[timestamps.len() - 1] - timestamps[0]) as f64 * 1e-9;
        dts.sort_unstable();
        let median = dts[dts.len() / 2];
        let max = dts[dts.len() - 1];
        let gaps = dts.iter().filter(|&&dt| dt > 2 * median).count();
        println!(
            "  {duration_s:.1} s, mean {:.1} Hz, median interval {:.3} ms, max {:.3} ms, {gaps} gaps (> 2x median), {non_monotonic} non-increasing",
            (timestamps.len() - 1) as f64 / duration_s,
            median as f64 * 1e-6,
            max as f64 * 1e-6,
        );
    }
    Ok(())
}

fn record_summary(session: &Session, name: &str) -> anyhow::Result<()> {
    let contents = session
        .read_records(name)
        .with_context(|| format!("stream {name}"))?;
    let records: Vec<_> = contents.chunks.iter().flat_map(|c| &c.records).collect();
    print!(
        "{name}: {} chunks, {} records",
        contents.chunks.len(),
        records.len()
    );
    if contents.truncated {
        print!(", TRUNCATED tail");
    }
    println!();
    let (Some(first), Some(last)) = (records.first(), records.last()) else {
        return Ok(());
    };
    let start = first.received_elapsed_realtime_ns;
    println!(
        "  {:.1} s",
        (last.received_elapsed_realtime_ns - start) as f64 * 1e-9
    );
    let mut counts = std::collections::BTreeMap::<&str, usize>::new();
    for r in &records {
        *counts.entry(payload_name(r.payload.as_ref())).or_default() += 1;
        if let Some(Payload::ManualEvent(e)) = &r.payload {
            let kind = manual_event::Kind::try_from(e.kind).map_or("unknown", |k| k.as_str_name());
            println!(
                "  at {:.1} s: {kind} {}",
                (r.received_elapsed_realtime_ns - start) as f64 * 1e-9,
                e.text
            );
        }
    }
    for (kind, n) in counts {
        println!("  {n} {kind}");
    }
    Ok(())
}

fn payload_name(payload: Option<&Payload>) -> &'static str {
    match payload {
        None => "empty",
        Some(Payload::GnssMeasurements(_)) => "gnss_measurements",
        Some(Payload::GnssStatus(_)) => "gnss_status",
        Some(Payload::GnssNavigationMessage(_)) => "gnss_navigation_message",
        Some(Payload::GnssMeasurementsStatus(_)) => "gnss_measurements_status",
        Some(Payload::Location(_)) => "location",
        Some(Payload::CellInfo(_)) => "cell_info",
        Some(Payload::ServiceState(_)) => "service_state",
        Some(Payload::SignalStrength(_)) => "signal_strength",
        Some(Payload::WifiScan(_)) => "wifi_scan",
        Some(Payload::ManualEvent(_)) => "manual_event",
        Some(Payload::Battery(_)) => "battery",
    }
}

fn csv(session: &Session, stream: &str) -> anyhow::Result<()> {
    let contents = session.read_stream(stream)?;
    for chunk in &contents.chunks {
        let n = chunk.values_per_sample as usize;
        let values: Vec<f32> = chunk.values().collect();
        for (i, t) in chunk.timestamps_ns().into_iter().enumerate() {
            print!("{t},{}", chunk.accuracy[i]);
            for v in &values[i * n..(i + 1) * n] {
                // `{:?}` prints the shortest representation that parses back to the same `f32`.
                print!(",{v:?}");
            }
            println!();
        }
    }
    if contents.truncated {
        eprintln!("warning: {stream} has a truncated tail");
    }
    Ok(())
}
