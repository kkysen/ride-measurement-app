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

mod damage {
    use super::*;
    use ride_session::read::{decode_stream_bytes, read_stream_file};

    /// A live-format stream: one frame per chunk. Returns the bytes and each frame's start offset.
    fn live_stream(chunks: &[SensorChunk]) -> (Vec<u8>, Vec<usize>) {
        let dir = tempfile::tempdir().unwrap();
        let mut offsets = Vec::new();
        let mut w = StreamWriter::create(dir.path(), "s").unwrap();
        for c in chunks {
            offsets.push(fs::metadata(w.path()).unwrap().len() as usize);
            w.append(c).unwrap();
        }
        (fs::read(w.path()).unwrap(), offsets)
    }

    fn chunks(n: i64) -> Vec<SensorChunk> {
        (0..n)
            .map(|i| chunk(1_000_000_000 * (i + 1), 400))
            .collect()
    }

    #[test]
    fn corrupt_middle_frame_loses_only_that_chunk() {
        let chunks = chunks(5);
        let (mut bytes, offsets) = live_stream(&chunks);
        // Flip a byte in the middle of frame 2's compressed data; the frame checksum catches it.
        let mid = (offsets[2] + offsets[3]) / 2;
        bytes[mid] ^= 0xff;
        let read = decode_stream_bytes::<SensorChunk>(&bytes);
        assert!(!read.truncated);
        assert_eq!(read.damage.len(), 1, "{:?}", read.damage);
        assert_eq!(read.chunks, [&chunks[..2], &chunks[3..]].concat());
    }

    #[test]
    fn garbage_between_frames_is_skipped() {
        let chunks = chunks(3);
        let (bytes, offsets) = live_stream(&chunks);
        let mut damaged = bytes[..offsets[1]].to_vec();
        damaged.extend_from_slice(b"not zstd at all");
        damaged.extend_from_slice(&bytes[offsets[1]..]);
        let read = decode_stream_bytes::<SensorChunk>(&damaged);
        assert_eq!(read.damage.len(), 1);
        assert_eq!(read.damage[0].offset, offsets[1]);
        assert_eq!(read.chunks, chunks);
    }

    #[test]
    fn trailing_garbage_is_reported() {
        let chunks = chunks(2);
        let (mut bytes, _) = live_stream(&chunks);
        bytes.extend_from_slice(&[0; 7]);
        let read = decode_stream_bytes::<SensorChunk>(&bytes);
        assert!(read.truncated);
        assert_eq!(read.damage.len(), 1);
        assert_eq!(read.chunks, chunks);
    }

    #[test]
    fn truncated_single_frame_salvages_complete_chunks() {
        // Like a recompressed file cut off by a crash: every chunk in one frame.
        let chunks = chunks(20);
        let mut encoder = zstd::stream::Encoder::new(Vec::new(), 19).unwrap();
        for c in &chunks {
            use prost::Message;
            std::io::Write::write_all(&mut encoder, &c.encode_length_delimited_to_vec()).unwrap();
        }
        let bytes = encoder.finish().unwrap();
        // Cut into the last block only; zstd emits a block's output only once the whole block is in.
        let read = decode_stream_bytes::<SensorChunk>(&bytes[..bytes.len() - 10]);
        assert!(read.truncated);
        assert!(read.damage.is_empty());
        assert!(!read.chunks.is_empty() && read.chunks.len() < chunks.len());
        assert_eq!(read.chunks, chunks[..read.chunks.len()]);
    }

    #[test]
    fn huge_length_prefix_is_damage_not_an_allocation() {
        // A valid frame whose content claims a 1 TiB message.
        let content = [0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x10];
        let bytes = zstd::encode_all(&content[..], 3).unwrap();
        let read = decode_stream_bytes::<SensorChunk>(&bytes);
        assert_eq!(read.damage.len(), 1);
        assert!(read.chunks.is_empty());
    }

    #[test]
    fn empty_file_is_clean() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("empty.zst");
        fs::write(&path, b"").unwrap();
        let read = read_stream_file::<SensorChunk>(&path).unwrap();
        assert!(read.is_clean() && read.chunks.is_empty());
    }
}

#[test]
fn chunk_builder_keeps_exact_bits_and_matches_from_samples() {
    use ride_session::write::ChunkBuilder;
    let tricky = tricky_values();
    let mut b = ChunkBuilder::default();
    let timestamps: Vec<i64> = (0..4)
        .map(|i| 1_000_000_000 + i * 2_500_000 + i % 3)
        .collect();
    let mut values = Vec::new();
    for (i, &t) in timestamps.iter().enumerate() {
        let v: Vec<f32> = (0..3).map(|k| tricky[(i * 3 + k) % tricky.len()]).collect();
        b.add(
            t,
            &v.iter().map(|x| x.to_bits()).collect::<Vec<_>>(),
            i as i32,
        )
        .unwrap();
        values.extend(v);
    }
    assert!(!b.fits(4));
    assert!(b.add(0, &[0; 4], 0).is_err());
    let built = b.build(7);
    assert!(b.is_empty() && b.fits(4));
    assert_eq!(
        built,
        SensorChunk::from_samples(&timestamps, 3, &values, &[0, 1, 2, 3], 7)
    );
}
