//! The `transit` stream: the MTA's GTFS-realtime feeds as recorded during a ride (`GtfsRealtimeFeed`),
//! decoded into what each train reported: its trip, the MTA's train ID, and where it was or expected to be.

use prost::Message;

use crate::proto::record::Payload;
use crate::read::Session;
use crate::transit_realtime::FeedMessage;

/// The stream's name.
pub const STREAM: &str = "transit";

/// One poll's response, decoded.
pub struct Poll {
    /// When the response finished arriving (`SystemClock.elapsedRealtimeNanos`).
    pub received_ns: i64,
    pub url: String,
    pub request_unix_ms: i64,
    pub http_status: i32,
    /// None if the request failed or the body doesn't decode (its bytes stay in the stream either way).
    pub feed: Option<FeedMessage>,
}

/// A feed body, if it decodes.
pub fn decode(body: &[u8]) -> Option<FeedMessage> {
    FeedMessage::decode(body).ok()
}

/// Every recorded poll, in order. A session from before this stream existed has none.
pub fn polls(session: &Session) -> anyhow::Result<Vec<Poll>> {
    if !session.stream_path(STREAM).exists() {
        return Ok(Vec::new());
    }
    Ok(session
        .read_records(STREAM)?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
        .filter_map(|r| match r.payload {
            Some(Payload::GtfsRealtime(f)) => Some(Poll {
                received_ns: r.received_elapsed_realtime_ns,
                feed: (f.http_status == 200).then(|| decode(&f.body)).flatten(),
                url: f.url,
                request_unix_ms: f.request_unix_ms,
                http_status: f.http_status,
            }),
            _ => None,
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{GtfsRealtimeFeed, Record, RecordChunk};
    use crate::transit_realtime::{
        FeedEntity, FeedHeader, NyctTripDescriptor, TripDescriptor, TripUpdate, VehiclePosition,
        trip_update::{StopTimeEvent, StopTimeUpdate},
    };
    use crate::write::StreamWriter;

    fn feed() -> FeedMessage {
        let trip = TripDescriptor {
            trip_id: Some("082600_1..S03R".into()),
            route_id: Some("1".into()),
            start_date: Some("20261022".into()),
            nyct: Some(NyctTripDescriptor {
                train_id: Some("01 1346 242/SFT".into()),
                is_assigned: Some(true),
                direction: Some(3),
            }),
            ..Default::default()
        };
        FeedMessage {
            header: FeedHeader {
                gtfs_realtime_version: "1.0".into(),
                timestamp: Some(1_792_686_000),
            },
            entity: vec![
                FeedEntity {
                    id: "1".into(),
                    trip_update: Some(TripUpdate {
                        trip: trip.clone(),
                        stop_time_update: vec![StopTimeUpdate {
                            stop_id: Some("118S".into()),
                            arrival: Some(StopTimeEvent {
                                time: Some(1_792_686_060),
                                ..Default::default()
                            }),
                            ..Default::default()
                        }],
                        ..Default::default()
                    }),
                    vehicle: None,
                },
                FeedEntity {
                    id: "2".into(),
                    trip_update: None,
                    vehicle: Some(VehiclePosition {
                        trip: Some(trip),
                        stop_id: Some("118S".into()),
                        current_status: Some(1),
                        ..Default::default()
                    }),
                },
            ],
        }
    }

    #[test]
    fn recorded_polls_decode() {
        let dir = tempfile::tempdir().unwrap();
        crate::write::write_session_info(
            dir.path(),
            &crate::SessionInfo {
                format_version: crate::FORMAT_VERSION,
                ..Default::default()
            },
        )
        .unwrap();
        let mut w = StreamWriter::create(dir.path(), STREAM).unwrap();
        let record = |t, status, body: Vec<u8>| Record {
            received_elapsed_realtime_ns: t,
            payload: Some(Payload::GtfsRealtime(GtfsRealtimeFeed {
                url: "https://example/nyct%2Fgtfs".into(),
                request_unix_ms: 5,
                http_status: status,
                body,
            })),
        };
        w.append(&RecordChunk {
            records: vec![
                record(10, 200, feed().encode_to_vec()),
                record(20, 503, b"busy".to_vec()),
            ],
            written_elapsed_realtime_ns: 30,
        })
        .unwrap();
        drop(w);
        let polls = polls(&Session::open(dir.path()).unwrap()).unwrap();
        assert_eq!(polls.len(), 2);
        assert_eq!(polls[0].feed.as_ref(), Some(&feed()));
        let train = polls[0].feed.as_ref().unwrap().entity[1]
            .vehicle
            .as_ref()
            .unwrap();
        assert_eq!(
            train
                .trip
                .as_ref()
                .unwrap()
                .nyct
                .as_ref()
                .unwrap()
                .train_id
                .as_deref(),
            Some("01 1346 242/SFT")
        );
        assert_eq!(polls[1].http_status, 503);
        assert!(polls[1].feed.is_none());
    }
}
