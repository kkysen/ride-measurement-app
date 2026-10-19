use std::fs;

use ride_session::proto::StreamInfo;
use ride_session::read::Session;
use ride_session::write::{StreamWriter, write_session_info};
use ride_session::{FORMAT_VERSION, SensorChunk, SessionInfo};

/// Values that a lossy or careless encoding would mangle.
fn tricky_values() -> Vec<f32> {
    vec![
        0.0,
        -0.0,
        f32::MIN_POSITIVE,
        f32::from_bits(1), // smallest subnormal
        f32::MAX,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::from_bits(0x7fc0_1234), // NaN with a payload
        9.806_65,
        -1.234_567_9e-7,
        123_456.79,
        0.1,
    ]
}

fn chunk(start_ns: i64, n: usize) -> SensorChunk {
    let tricky = tricky_values();
    let timestamps: Vec<i64> = (0..n as i64)
        .map(|i| start_ns + i * 2_500_000 + i % 3)
        .collect();
    let values: Vec<f32> = (0..n * 3)
        .map(|i| tricky[i % tricky.len()] * if i % 5 == 0 { 1.0 } else { -1.0 })
        .collect();
    let accuracy: Vec<i32> = (0..n as i32).map(|i| i % 4).collect();
    SensorChunk::from_samples(&timestamps, 3, &values, &accuracy, start_ns + 1)
}

fn write_session(dir: &std::path::Path, chunks: &[SensorChunk]) {
    let info = SessionInfo {
        format_version: FORMAT_VERSION,
        session_id: "test".into(),
        streams: vec![StreamInfo {
            name: "accel".into(),
            ..Default::default()
        }],
        ..Default::default()
    };
    write_session_info(dir, &info).unwrap();
    let mut w = StreamWriter::create(dir, "accel").unwrap();
    for c in chunks {
        w.append(c).unwrap();
    }
}

#[test]
fn round_trip_is_bit_exact() {
    let dir = tempfile::tempdir().unwrap();
    let chunks: Vec<_> = (0..5)
        .map(|i| chunk(1_000_000_000 * (i + 1), 400))
        .collect();
    write_session(dir.path(), &chunks);

    let session = Session::open(dir.path()).unwrap();
    assert_eq!(session.info.session_id, "test");
    let read = session.read_stream("accel").unwrap();
    assert!(!read.truncated);
    assert_eq!(read.chunks, chunks);
    for (r, w) in read.chunks.iter().zip(&chunks) {
        let r_bits: Vec<u32> = r.values().map(f32::to_bits).collect();
        assert_eq!(r_bits, w.value_bits);
        assert_eq!(r.timestamps_ns(), w.timestamps_ns());
    }
}

#[test]
fn delta_encoding_round_trips() {
    let timestamps = [5, 7, 7, 3, i64::MAX / 2, -10];
    let c = SensorChunk::from_samples(&timestamps, 0, &[], &[0; 6], 0);
    assert_eq!(c.timestamps_ns(), timestamps);
}

#[test]
fn truncated_tail_keeps_complete_chunks() {
    let dir = tempfile::tempdir().unwrap();
    let chunks: Vec<_> = (0..3)
        .map(|i| chunk(1_000_000_000 * (i + 1), 400))
        .collect();
    write_session(dir.path(), &chunks);

    let path = dir.path().join("accel.zst");
    let len = fs::metadata(&path).unwrap().len();
    let file = fs::OpenOptions::new().write(true).open(&path).unwrap();
    file.set_len(len - 10).unwrap();

    let read = Session::open(dir.path())
        .unwrap()
        .read_stream("accel")
        .unwrap();
    assert!(read.truncated);
    assert_eq!(read.chunks, chunks[..2]);
}

#[test]
fn record_stream_round_trips() {
    use ride_session::RecordChunk;
    use ride_session::proto::record::Payload;
    use ride_session::proto::{Location, ManualEvent, Record, manual_event};

    let dir = tempfile::tempdir().unwrap();
    write_session(dir.path(), &[]);
    let chunks = vec![RecordChunk {
        records: vec![
            Record {
                received_elapsed_realtime_ns: 5,
                payload: Some(Payload::Location(Location {
                    provider: "gps".into(),
                    latitude_degrees: f64::from_bits(0x4044_5a1c_ac08_3127),
                    longitude_degrees: -73.985_130_1,
                    speed_meters_per_second: Some(-0.0),
                    ..Default::default()
                })),
            },
            Record {
                received_elapsed_realtime_ns: 6,
                payload: Some(Payload::ManualEvent(ManualEvent {
                    kind: manual_event::Kind::AtStation.into(),
                    text: "14 St".into(),
                })),
            },
        ],
        written_elapsed_realtime_ns: 7,
    }];
    let mut w = StreamWriter::create(dir.path(), "events").unwrap();
    for c in &chunks {
        w.append(c).unwrap();
    }
    let read = Session::open(dir.path())
        .unwrap()
        .read_records("events")
        .unwrap();
    assert!(!read.truncated);
    assert_eq!(read.chunks, chunks);
}

#[test]
fn audio_stream_round_trips() {
    use ride_session::AudioChunk;
    use ride_session::proto::AudioTimestamp;

    let dir = tempfile::tempdir().unwrap();
    write_session(dir.path(), &[]);
    let pcm: Vec<u8> = (0..32_000u32)
        .flat_map(|i| ((i * 7919) as i16).to_le_bytes())
        .collect();
    let chunks: Vec<_> = (0..3)
        .map(|c| AudioChunk {
            first_frame: c * 32_000,
            pcm_s16le: pcm.clone(),
            timestamps: vec![AudioTimestamp {
                frame_position: c * 32_000,
                boottime_ns: 1_000_000_000 * (c + 1),
            }],
            written_elapsed_realtime_ns: c,
        })
        .collect();
    let mut w = StreamWriter::create(dir.path(), "audio").unwrap();
    for c in &chunks {
        w.append(c).unwrap();
    }
    let read = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap();
    assert!(!read.truncated);
    assert_eq!(read.chunks, chunks);
}
