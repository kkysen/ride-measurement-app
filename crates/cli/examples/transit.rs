// The `transit` stream (the MTA's GTFS-realtime feeds recorded during a ride) as CSVs in `out`, for Python:
// `vehicles.csv`, each train's reported position per poll, and `trip_updates.csv`, each predicted or reported stop.
// `cargo run --example transit -- <session> <out>`; a raw feed body (`.pb`) in place of the session also works.
use std::io::Write;

use prost::Message;
use ride_session::read::Session;
use ride_session::transit::{Poll, polls};
use ride_session::transit_realtime::{FeedMessage, TripDescriptor};

fn trip_columns(trip: Option<&TripDescriptor>) -> String {
    let t = trip.cloned().unwrap_or_default();
    let nyct = t.nyct.unwrap_or_default();
    format!(
        "{},{},{},{},{},{}",
        t.trip_id.unwrap_or_default(),
        t.route_id.unwrap_or_default(),
        t.start_date.unwrap_or_default(),
        nyct.train_id.unwrap_or_default().replace(',', " "),
        nyct.is_assigned.map_or(String::new(), |a| a.to_string()),
        nyct.direction.map_or(String::new(), |d| d.to_string()),
    )
}

fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let input = std::path::Path::new(&a[1]);
    let out = std::path::Path::new(&a[2]);
    let polls = if input.is_file() {
        vec![Poll {
            received_ns: 0,
            url: input.display().to_string(),
            request_unix_ms: 0,
            http_status: 200,
            feed: Some(FeedMessage::decode(&std::fs::read(input)?[..])?),
        }]
    } else {
        polls(&Session::open(input)?)?
    };
    std::fs::create_dir_all(out)?;
    let mut v = std::fs::File::create(out.join("vehicles.csv"))?;
    writeln!(
        v,
        "received_ns,feed,feed_time,trip_id,route,start_date,train_id,assigned,direction,stop_id,status,stop_sequence,time"
    )?;
    let mut u = std::fs::File::create(out.join("trip_updates.csv"))?;
    writeln!(
        u,
        "received_ns,feed,feed_time,trip_id,route,start_date,train_id,assigned,direction,stop_id,arrival,departure,scheduled_track,actual_track"
    )?;
    let (mut failed, mut ok) = (0, 0);
    for p in &polls {
        let Some(feed) = &p.feed else {
            failed += 1;
            continue;
        };
        ok += 1;
        let name = p.url.rsplit("gtfs").next().unwrap_or_default();
        let head = format!(
            "{},{name},{}",
            p.received_ns,
            feed.header.timestamp.unwrap_or_default()
        );
        for e in &feed.entity {
            if let Some(x) = &e.vehicle {
                writeln!(
                    v,
                    "{head},{},{},{},{},{}",
                    trip_columns(x.trip.as_ref()),
                    x.stop_id.clone().unwrap_or_default(),
                    x.current_status.unwrap_or(2),
                    x.current_stop_sequence
                        .map_or(String::new(), |s| s.to_string()),
                    x.timestamp.map_or(String::new(), |t| t.to_string()),
                )?;
            }
            if let Some(t) = &e.trip_update {
                let trip = trip_columns(Some(&t.trip));
                for s in &t.stop_time_update {
                    let time =
                        |e: &Option<ride_session::transit_realtime::trip_update::StopTimeEvent>| {
                            e.as_ref()
                                .and_then(|e| e.time)
                                .map_or(String::new(), |t| t.to_string())
                        };
                    let nyct = s.nyct.clone().unwrap_or_default();
                    writeln!(
                        u,
                        "{head},{trip},{},{},{},{},{}",
                        s.stop_id.clone().unwrap_or_default(),
                        time(&s.arrival),
                        time(&s.departure),
                        nyct.scheduled_track.unwrap_or_default(),
                        nyct.actual_track.unwrap_or_default(),
                    )?;
                }
            }
        }
    }
    eprintln!("{ok} polls decoded, {failed} failed or undecodable");
    Ok(())
}
