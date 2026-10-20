//! `ride import-gnsslogger`: a GnssLogger log as a session, to try the GNSS tools on public logs.

use std::path::Path;

use anyhow::{Context, bail};
use ride_gnss::gnsslogger;
use ride_session::proto::record::Payload;
use ride_session::proto::{Record, RecordChunk, StreamInfo, StreamKind};
use ride_session::write::{StreamWriter, write_session_info};
use ride_session::{FORMAT_VERSION, SessionInfo};

/// Records per chunk, about what the app writes in a few seconds of GNSS.
const RECORDS_PER_CHUNK: usize = 100;

pub fn import(log_path: &Path, dir: &Path) -> anyhow::Result<()> {
    if dir.exists() && dir.read_dir()?.next().is_some() {
        bail!("{} exists and isn't empty; not touching it", dir.display());
    }
    let text = std::fs::read_to_string(log_path)
        .with_context(|| format!("reading {}", log_path.display()))?;
    let log = gnsslogger::read(&text);
    let Some(first) = log.epochs.first() else {
        bail!("no raw measurements in {}", log_path.display());
    };
    let start_ns = first
        .clock
        .as_ref()
        .unwrap()
        .elapsed_realtime_nanos
        .unwrap();

    let mut gnss: Vec<Record> = Vec::new();
    for e in &log.epochs {
        let t = e.clock.as_ref().unwrap().elapsed_realtime_nanos.unwrap();
        gnss.push(record(t, Payload::GnssMeasurements(e.clone())));
    }
    for (t, m) in &log.navigation {
        gnss.push(record(*t, Payload::GnssNavigationMessage(m.clone())));
    }
    gnss.sort_by_key(|r| r.received_elapsed_realtime_ns);
    let location: Vec<Record> = log
        .fixes
        .iter()
        .map(|l| record(l.elapsed_realtime_nanos, Payload::Location(l.clone())))
        .collect();

    let streams = [("gnss", &gnss), ("location", &location)];
    let info = SessionInfo {
        format_version: FORMAT_VERSION,
        session_id: format!(
            "gnsslogger-{}",
            log_path.file_stem().unwrap_or_default().display()
        ),
        // The session's clock is UTC itself in newer logs; older ones have no UTC for it.
        start_unix_ms: if log.utc_times {
            start_ns / 1_000_000
        } else {
            0
        },
        start_elapsed_realtime_ns: start_ns,
        streams: streams
            .iter()
            .filter(|(_, records)| !records.is_empty())
            .map(|(name, _)| StreamInfo {
                name: name.to_string(),
                kind: StreamKind::Records.into(),
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    };
    write_session_info(dir, &info)?;
    for (name, records) in streams {
        if records.is_empty() {
            continue;
        }
        let mut w = StreamWriter::create(dir, name)?;
        for chunk in records.chunks(RECORDS_PER_CHUNK) {
            w.append(&RecordChunk {
                records: chunk.to_vec(),
                written_elapsed_realtime_ns: chunk.last().unwrap().received_elapsed_realtime_ns,
            })?;
        }
    }
    println!(
        "{}: {} epochs, {} fixes, {} navigation messages",
        dir.display(),
        log.epochs.len(),
        log.fixes.len(),
        log.navigation.len()
    );
    Ok(())
}

fn record(t: i64, payload: Payload) -> Record {
    Record {
        received_elapsed_realtime_ns: t,
        payload: Some(payload),
    }
}
