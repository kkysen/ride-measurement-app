//! The rider's marks (taps) in a session's `events` stream, with later corrections applied, and appending
//! corrections to a finished session. Nothing is ever removed: deleting a mark records a `KIND_DELETE` naming it,
//! and changing one deletes it and records the new one at the old one's moment (`at_elapsed_realtime_ns`).

use std::fs::OpenOptions;
use std::io::Write;

use anyhow::bail;
use prost::Message;

use crate::proto::manual_event::Kind;
use crate::proto::record::Payload;
use crate::proto::{ManualEvent, Record};
use crate::read::Session;
use crate::write::LIVE_ZSTD_LEVEL;
use crate::{RecordChunk, STREAM_EXTENSION};

pub const EVENTS_STREAM: &str = "events";

/// A mark as the rider means it now.
#[derive(Clone, Debug, PartialEq)]
pub struct Mark {
    /// Its record's time: what a deletion names it by.
    pub received_ns: i64,
    /// The moment it marks, on the recording's clock.
    pub t_ns: i64,
    /// Wall-clock time of that moment, from the session's start.
    pub unix_ms: i64,
    pub kind: Kind,
    pub text: String,
    pub deleted: bool,
    /// Whether it was recorded after the fact, as a correction.
    pub edited: bool,
}

/// The session's marks in time order, deletions applied as flags; `KIND_DELETE` records themselves are left out.
pub fn marks(session: &Session) -> anyhow::Result<Vec<Mark>> {
    if !session.stream_path(EVENTS_STREAM).exists() {
        return Ok(Vec::new());
    }
    let contents = session.read_records(EVENTS_STREAM)?;
    let events: Vec<(i64, ManualEvent)> = contents
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
        .filter_map(|r| match r.payload {
            Some(Payload::ManualEvent(e)) => Some((r.received_elapsed_realtime_ns, e)),
            _ => None,
        })
        .collect();
    let deleted: std::collections::HashSet<i64> = events
        .iter()
        .filter(|(_, e)| e.kind() == Kind::Delete)
        .map(|(_, e)| e.deletes_received_elapsed_realtime_ns)
        .collect();
    let info = &session.info;
    let mut out: Vec<Mark> = events
        .into_iter()
        .filter(|(_, e)| e.kind() != Kind::Delete)
        .map(|(received, e)| {
            let t = if e.at_elapsed_realtime_ns != 0 {
                e.at_elapsed_realtime_ns
            } else {
                received
            };
            Mark {
                received_ns: received,
                t_ns: t,
                unix_ms: info.start_unix_ms + (t - info.start_elapsed_realtime_ns) / 1_000_000,
                kind: e.kind(),
                text: e.text,
                deleted: deleted.contains(&received),
                edited: e.edited_unix_ms != 0,
            }
        })
        .collect();
    out.sort_by_key(|m| m.t_ns);
    Ok(out)
}

/// Records that a mark was wrong. `now_ns`: a time for the new record that no other record has
/// (it names this record, should it ever be deleted in turn).
pub fn delete_mark(
    session: &Session,
    mark: &Mark,
    now_ns: i64,
    now_unix_ms: i64,
) -> anyhow::Result<()> {
    append(
        session,
        vec![record(
            now_ns,
            ManualEvent {
                kind: Kind::Delete.into(),
                deletes_received_elapsed_realtime_ns: mark.received_ns,
                edited_unix_ms: now_unix_ms,
                ..Default::default()
            },
        )],
    )
}

/// Changes a mark's kind or text: deletes it and records the new one at its moment.
pub fn change_mark(
    session: &Session,
    mark: &Mark,
    kind: Kind,
    text: &str,
    now_ns: i64,
    now_unix_ms: i64,
) -> anyhow::Result<()> {
    append(
        session,
        vec![
            record(
                now_ns,
                ManualEvent {
                    kind: Kind::Delete.into(),
                    deletes_received_elapsed_realtime_ns: mark.received_ns,
                    edited_unix_ms: now_unix_ms,
                    ..Default::default()
                },
            ),
            // A different time from the deletion's, so each can be named on its own.
            record(
                now_ns + 1,
                ManualEvent {
                    kind: kind.into(),
                    text: text.into(),
                    at_elapsed_realtime_ns: mark.t_ns,
                    edited_unix_ms: now_unix_ms,
                    ..Default::default()
                },
            ),
        ],
    )
}

fn record(t_ns: i64, e: ManualEvent) -> Record {
    Record {
        received_elapsed_realtime_ns: t_ns,
        payload: Some(Payload::ManualEvent(e)),
    }
}

/// Appends one chunk to a finished session's `events` stream: one more independent zstd frame,
/// so what's there is never rewritten.
fn append(session: &Session, records: Vec<Record>) -> anyhow::Result<()> {
    let path = session.stream_path(EVENTS_STREAM);
    if !path.exists() {
        bail!(
            "{} has no {EVENTS_STREAM}.{STREAM_EXTENSION}",
            session.dir.display()
        );
    }
    let written = records.last().map_or(0, |r| r.received_elapsed_realtime_ns);
    let chunk = RecordChunk {
        records,
        written_elapsed_realtime_ns: written,
    }
    .encode_to_vec();
    let mut delimited = Vec::with_capacity(chunk.len() + 10);
    prost::encoding::encode_varint(chunk.len() as u64, &mut delimited);
    delimited.extend_from_slice(&chunk);
    let mut encoder = zstd::stream::Encoder::new(Vec::new(), LIVE_ZSTD_LEVEL)?;
    encoder.include_checksum(true)?;
    encoder.write_all(&delimited)?;
    let frame = encoder.finish()?;
    let mut file = OpenOptions::new().append(true).open(&path)?;
    file.write_all(&frame)?;
    file.sync_all()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{StreamInfo, StreamKind};
    use crate::write::{StreamWriter, write_session_info};
    use crate::{FORMAT_VERSION, SessionInfo};

    fn session_with(marks: &[(i64, Kind, &str)]) -> (tempfile::TempDir, Session) {
        let dir = tempfile::tempdir().unwrap();
        write_session_info(
            dir.path(),
            &SessionInfo {
                format_version: FORMAT_VERSION,
                start_unix_ms: 1_000_000,
                start_elapsed_realtime_ns: 5_000_000_000,
                streams: vec![StreamInfo {
                    name: EVENTS_STREAM.into(),
                    kind: StreamKind::Records.into(),
                    ..Default::default()
                }],
                ..Default::default()
            },
        )
        .unwrap();
        let mut w = StreamWriter::create(dir.path(), EVENTS_STREAM).unwrap();
        for &(t, kind, text) in marks {
            w.append(&RecordChunk {
                records: vec![record(
                    t,
                    ManualEvent {
                        kind: kind.into(),
                        text: text.into(),
                        ..Default::default()
                    },
                )],
                written_elapsed_realtime_ns: t,
            })
            .unwrap();
        }
        let s = Session::open(dir.path()).unwrap();
        (dir, s)
    }

    #[test]
    fn deletes_and_changes_keep_everything() {
        let (_dir, s) = session_with(&[
            (6_000_000_000, Kind::Boarded, ""),
            (7_000_000_000, Kind::Alighted, ""),
            (8_000_000_000, Kind::AtStation, "103 St"),
        ]);
        let before = std::fs::metadata(s.stream_path(EVENTS_STREAM))
            .unwrap()
            .len();
        let m = marks(&s).unwrap();
        assert_eq!(m.len(), 3);
        assert_eq!(m[0].unix_ms, 1_001_000);
        // After a reboot: a clock far below the recording's.
        change_mark(&s, &m[1], Kind::Boarded, "", 100, 2_000_000).unwrap();
        delete_mark(&s, &m[2], 200, 2_000_001).unwrap();
        let after = marks(&s).unwrap();
        let live: Vec<_> = after.iter().filter(|m| !m.deleted).collect();
        assert_eq!(live.len(), 2);
        assert_eq!(live[1].kind, Kind::Boarded);
        assert_eq!(live[1].t_ns, 7_000_000_000);
        assert!(live[1].edited);
        // The originals are all still there, flagged.
        assert_eq!(after.len(), 4);
        assert_eq!(after.iter().filter(|m| m.deleted).count(), 2);
        // Only appended to.
        assert!(
            std::fs::metadata(s.stream_path(EVENTS_STREAM))
                .unwrap()
                .len()
                > before
        );
    }
}
