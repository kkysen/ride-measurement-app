//! `ride map`: the ride's path on a map, colored by speed, as one HTML file.

use std::path::Path;

use anyhow::Context;
use ride_fusion::fusion::SmoothedSpeed;
use ride_fusion::replay::replay;
use ride_fusion::speed_colors::{RAMP_ARGB, SpeedScale, UNKNOWN_ARGB, edge_speed};
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::{TrackPoint, gps_segments};
use serde_json::{Value, json};

const TEMPLATE: &str = include_str!("map.html");
const DATA_PLACEHOLDER: &str = "/*RIDE_DATA*/null";

/// `scale` preselects a color scale; without one, the page picks it from the top fused speed.
pub fn map(session: &Session, output: &Path, scale: Option<SpeedScale>) -> anyhow::Result<()> {
    let locations = session.read_records("location")?;
    warn_unclean("location", locations.truncated, locations.damage.len());
    let segments = gps_segments(&locations.chunks);
    let start_ns = segments
        .first()
        .and_then(|s| s.first())
        .map(|p| p.elapsed_realtime_ns)
        .context("no GPS fixes in this session")?;
    let fixes: Vec<&TrackPoint> = segments.iter().flatten().collect();
    let points = fixes.len();
    // The app's fusion replayed over the recorded IMU, fixes, and raw GNSS, then smoothed over the whole ride.
    let replayed = replay(session)?;
    let g = &replayed.gnss;
    eprintln!(
        "raw GNSS: {} of {} epochs solved; {} ephemerides from {} navigation subframes ({} failed parity)",
        g.epochs_solved,
        g.epochs,
        g.ephemerides,
        g.subframes_ok + g.subframes_bad,
        g.subframes_bad
    );
    let fused: Vec<Option<&SmoothedSpeed>> = fixes
        .iter()
        .map(|p| replayed.at(p.elapsed_realtime_ns))
        .collect();
    let top_speed = fused.iter().flatten().map(|e| e.speed_mps).reduce(f64::max);
    let mut fused = fused.into_iter();

    // Manual events, pinned at the nearest-in-time fix, since they have no position of their own.
    let mut events = Vec::new();
    if session.stream_path("events").exists() {
        let contents = session.read_records("events")?;
        warn_unclean("events", contents.truncated, contents.damage.len());
        for r in contents.chunks.iter().flat_map(|c| &c.records) {
            let Some(Payload::ManualEvent(e)) = &r.payload else {
                continue;
            };
            let t = r.received_elapsed_realtime_ns;
            let Some(p) = segments
                .iter()
                .flatten()
                .min_by_key(|p| (p.elapsed_realtime_ns - t).abs())
            else {
                continue;
            };
            let kind = e
                .kind()
                .as_str_name()
                .trim_start_matches("KIND_")
                .to_lowercase()
                .replace('_', " ");
            events.push(json!({
                "lat": p.latitude_degrees,
                "lon": p.longitude_degrees,
                "t": (t - start_ns) as f64 * 1e-9,
                "offsetS": (p.elapsed_realtime_ns - t) as f64 * 1e-9,
                "label": if e.text.is_empty() { kind } else { format!("{kind}: {}", e.text) },
            }));
        }
    }

    let data = json!({
        "session": session.info.session_id,
        "segments": segments
            .iter()
            .map(|s| {
                let mut prev: Option<Option<f64>> = None;
                s.iter()
                    .map(|p| {
                        let e = fused.next().flatten();
                        let speed = e.map(|e| e.speed_mps);
                        // The first fix of a segment has no path into it; color it by its own speed.
                        let edge = prev.map_or(speed, |prev| edge_speed(prev, speed));
                        prev = Some(speed);
                        point_json(p, e, edge, start_ns)
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>(),
        "events": events,
        // Colors and scales come from `ride_fusion::speed_colors`, the same as the app's live map;
        // the page only looks them up.
        "ramp": RAMP_ARGB.map(css_color),
        "unknownColor": css_color(UNKNOWN_ARGB),
        "scales": SpeedScale::ALL.map(|s| json!({"id": s.id(), "name": s.name(), "bandsKmh": s.bands_kmh()})),
        "autoScale": SpeedScale::for_top_speed(top_speed).id(),
        "scale": scale.map(SpeedScale::id),
    });
    // Embedded in a `<script>`, so a `</script>` in an event note mustn't end it.
    let data = serde_json::to_string(&data)?.replace("</", "<\\/");
    let html = TEMPLATE.replacen(DATA_PLACEHOLDER, &data, 1);
    std::fs::write(output, html).with_context(|| format!("writing {}", output.display()))?;
    eprintln!(
        "{}: {points} GPS fixes in {} segments, {} events",
        output.display(),
        segments.len(),
        events.len()
    );
    Ok(())
}

/// `[lat, lon, seconds since the first fix, UTC ms, GPS speed, GPS speed accuracy, horizontal accuracy,
/// smoothed fused speed, its sigma, [color band of the path into this fix, per scale]]`, compact for long rides.
fn point_json(
    p: &TrackPoint,
    fused: Option<&SmoothedSpeed>,
    edge_speed: Option<f64>,
    start_ns: i64,
) -> Value {
    json!([
        p.latitude_degrees,
        p.longitude_degrees,
        (p.elapsed_realtime_ns - start_ns) as f64 * 1e-9,
        p.time_ms,
        p.speed_meters_per_second,
        p.speed_accuracy_meters_per_second,
        p.horizontal_accuracy_meters,
        fused.map(|e| e.speed_mps),
        fused.map(|e| e.sigma_mps),
        SpeedScale::ALL.map(|s| s.band(edge_speed)),
    ])
}

fn css_color(argb: u32) -> String {
    format!("#{:06x}", argb & 0xFF_FFFF)
}

fn warn_unclean(stream: &str, truncated: bool, damaged: usize) {
    if truncated || damaged > 0 {
        eprintln!("warning: {stream}: truncated: {truncated}, {damaged} damaged ranges skipped");
    }
}

#[cfg(test)]
mod tests {
    use ride_session::proto::{
        Location, ManualEvent, Record, RecordChunk, StreamInfo, StreamKind, manual_event,
    };
    use ride_session::read::Session;
    use ride_session::write::{StreamWriter, write_session_info};
    use ride_session::{FORMAT_VERSION, SessionInfo};

    /// A synthetic ride: 2 minutes heading north at up to 20 m/s, with a 30 s tunnel in the middle.
    pub fn write_demo_session(dir: &std::path::Path) {
        let streams = ["location", "events"].map(|name| StreamInfo {
            name: name.into(),
            kind: StreamKind::Records.into(),
            ..Default::default()
        });
        let info = SessionInfo {
            format_version: FORMAT_VERSION,
            session_id: "demo".into(),
            streams: streams.into(),
            ..Default::default()
        };
        write_session_info(dir, &info).unwrap();
        let mut records = Vec::new();
        let mut lat = 40.75;
        for t in 0..120i64 {
            let speed = 20.0 * (std::f64::consts::PI * t as f64 / 120.0).sin();
            lat += speed / 111_000.0;
            if (50..80).contains(&t) {
                continue;
            }
            for provider in ["gps", "fused"] {
                records.push(Record {
                    received_elapsed_realtime_ns: t * 1_000_000_000,
                    payload: Some(ride_session::proto::record::Payload::Location(Location {
                        provider: provider.into(),
                        time_ms: 1_790_000_000_000 + t * 1000,
                        elapsed_realtime_nanos: t * 1_000_000_000,
                        latitude_degrees: lat,
                        longitude_degrees: if provider == "gps" { -73.99 } else { -73.0 },
                        speed_meters_per_second: Some(speed as f32),
                        speed_accuracy_meters_per_second: Some(0.3),
                        horizontal_accuracy_meters: Some(4.0),
                        ..Default::default()
                    })),
                });
            }
        }
        let mut w = StreamWriter::create(dir, "location").unwrap();
        w.append(&RecordChunk {
            records,
            ..Default::default()
        })
        .unwrap();
        let mut w = StreamWriter::create(dir, "events").unwrap();
        let event = |t: i64, kind: manual_event::Kind, text: &str| Record {
            received_elapsed_realtime_ns: t * 1_000_000_000,
            payload: Some(ride_session::proto::record::Payload::ManualEvent(
                ManualEvent {
                    kind: kind.into(),
                    text: text.into(),
                },
            )),
        };
        w.append(&RecordChunk {
            records: vec![
                event(1, manual_event::Kind::Boarded, ""),
                event(119, manual_event::Kind::Note, "</script><b>not html</b>"),
            ],
            ..Default::default()
        })
        .unwrap();
    }

    #[test]
    fn map_embeds_gps_segments_and_events() {
        let dir = tempfile::tempdir().unwrap();
        write_demo_session(dir.path());
        let out = dir.path().join("map.html");
        super::map(&Session::open(dir.path()).unwrap(), &out, None).unwrap();
        let html = std::fs::read_to_string(&out).unwrap();
        // To look at the demo map: `RIDE_MAP_DEMO=$PWD/target/demo-map.html cargo test -p ride-cli map`.
        if let Some(demo) = std::env::var_os("RIDE_MAP_DEMO") {
            std::fs::copy(&out, demo).unwrap();
        }
        let start = html.find("const data = ").unwrap() + "const data = ".len();
        let end = start + html[start..].find(";\n").unwrap();
        let json = &html[start..end];
        // An event note can't close the `<script>` early.
        assert!(!json.contains("</"));
        let data: serde_json::Value = serde_json::from_str(&json.replace("<\\/", "</")).unwrap();
        let segments = data["segments"].as_array().unwrap();
        assert_eq!(
            segments
                .iter()
                .map(|s| s.as_array().unwrap().len())
                .collect::<Vec<_>>(),
            [50, 40]
        );
        // Every fix carries the replayed fused speed. With no IMU in this session it can only follow GPS,
        // lagging a little since nothing predicts between fixes.
        for p in segments.iter().flat_map(|s| s.as_array().unwrap()) {
            assert!((p[7].as_f64().unwrap() - p[4].as_f64().unwrap()).abs() < 1.5);
        }
        // Fused fixes are left out.
        assert!(
            segments
                .iter()
                .flat_map(|s| s.as_array().unwrap())
                .all(|p| p[1] == -73.99)
        );
        // Colors and scales come from `ride_fusion::speed_colors`; the demo tops out around 70 km/h.
        assert_eq!(data["autoScale"], "rail");
        assert_eq!(data["ramp"].as_array().unwrap().len(), 10);
        assert!(
            segments[0][1][9]
                .as_array()
                .unwrap()
                .iter()
                .all(|b| b.is_u64())
        );
        assert_eq!(data["events"][0]["label"], "boarded");
        assert_eq!(data["events"][1]["label"], "note: </script><b>not html</b>");
    }
}
