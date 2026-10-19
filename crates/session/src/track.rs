//! The ride's path from GPS fixes, for maps.
//!
//! Only GPS-provider fixes are used: fused and network fixes can come from cell or WiFi positions,
//! which can be the wrong station underground.

use crate::RecordChunk;
use crate::proto::record::Payload;

/// The provider name Android gives fixes computed by the GNSS chip.
pub const GPS_PROVIDER: &str = "gps";

/// A fix more than this long after the previous one starts a new segment,
/// so a tunnel is shown as a gap rather than a straight line drawn through it.
pub const MAX_GAP_NS: i64 = 5_000_000_000;

#[derive(Clone, Debug, PartialEq)]
pub struct TrackPoint {
    pub elapsed_realtime_ns: i64,
    /// UTC milliseconds since the epoch, from the fix.
    pub time_ms: i64,
    pub latitude_degrees: f64,
    pub longitude_degrees: f64,
    pub speed_meters_per_second: Option<f32>,
    pub speed_accuracy_meters_per_second: Option<f32>,
    pub horizontal_accuracy_meters: Option<f32>,
}

/// GPS fixes in time order, split into segments at gaps longer than [`MAX_GAP_NS`].
pub fn gps_segments<'a>(chunks: impl IntoIterator<Item = &'a RecordChunk>) -> Vec<Vec<TrackPoint>> {
    let mut points: Vec<TrackPoint> = chunks
        .into_iter()
        .flat_map(|c| &c.records)
        .filter_map(|r| match &r.payload {
            Some(Payload::Location(l)) if l.provider == GPS_PROVIDER && !l.is_mock => {
                Some(TrackPoint {
                    elapsed_realtime_ns: l.elapsed_realtime_nanos,
                    time_ms: l.time_ms,
                    latitude_degrees: l.latitude_degrees,
                    longitude_degrees: l.longitude_degrees,
                    speed_meters_per_second: l.speed_meters_per_second,
                    speed_accuracy_meters_per_second: l.speed_accuracy_meters_per_second,
                    horizontal_accuracy_meters: l.horizontal_accuracy_meters,
                })
            }
            _ => None,
        })
        .collect();
    points.sort_by_key(|p| p.elapsed_realtime_ns);
    points.dedup_by_key(|p| p.elapsed_realtime_ns);

    let mut segments: Vec<Vec<TrackPoint>> = Vec::new();
    for p in points {
        match segments.last_mut() {
            Some(s)
                if p.elapsed_realtime_ns - s.last().unwrap().elapsed_realtime_ns <= MAX_GAP_NS =>
            {
                s.push(p)
            }
            _ => segments.push(vec![p]),
        }
    }
    segments
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{Location, Record};

    fn fix(provider: &str, t_s: i64, lat: f64) -> Record {
        Record {
            received_elapsed_realtime_ns: t_s * 1_000_000_000,
            payload: Some(Payload::Location(Location {
                provider: provider.into(),
                elapsed_realtime_nanos: t_s * 1_000_000_000,
                latitude_degrees: lat,
                speed_meters_per_second: Some(t_s as f32),
                ..Default::default()
            })),
        }
    }

    #[test]
    fn gps_only_sorted_and_split_at_gaps() {
        let chunks = [
            RecordChunk {
                records: vec![
                    fix("gps", 1, 1.0),
                    fix("fused", 2, 99.0),
                    fix("gps", 3, 3.0),
                ],
                ..Default::default()
            },
            // Out of order across chunks, a duplicate, and a tunnel-length gap.
            RecordChunk {
                records: vec![fix("gps", 2, 2.0), fix("gps", 3, 3.0), fix("gps", 20, 20.0)],
                ..Default::default()
            },
        ];
        let lats: Vec<Vec<f64>> = gps_segments(&chunks)
            .iter()
            .map(|s| s.iter().map(|p| p.latitude_degrees).collect())
            .collect();
        assert_eq!(lats, vec![vec![1.0, 2.0, 3.0], vec![20.0]]);
    }
}
