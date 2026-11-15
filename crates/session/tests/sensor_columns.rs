use std::fs;
use std::path::Path;

use prost::Message;
use ride_session::proto::StreamInfo;
use ride_session::read::{Session, decode_stream_bytes};
use ride_session::recompress::{
    Encoding, LEVEL, MARKER, Options, Outcome, content_sha256, recompress_sensor_stream,
    recompress_session, recompress_stream_checked, recompress_stream_with,
};
use ride_session::sensor_columns::{
    content_len, decode_file, encode_chunks_file_in_blocks, encode_file, is_columnar,
};
use ride_session::write::{StreamWriter, write_session_info};
use ride_session::{FORMAT_VERSION, SensorChunk, SessionInfo};

fn content(chunks: &[SensorChunk]) -> Vec<u8> {
    let mut out = Vec::new();
    for c in chunks {
        c.encode_length_delimited(&mut out).unwrap();
    }
    out
}

fn roundtrip(chunks: &[SensorChunk]) {
    let bodies: Vec<Vec<u8>> = chunks.iter().map(|c| c.encode_to_vec()).collect();
    let file = encode_file(&bodies, LEVEL).unwrap();
    assert!(is_columnar(&file));
    assert_eq!(decode_file(&file).unwrap(), content(chunks));
    let read = decode_stream_bytes::<SensorChunk>(&file);
    assert!(read.is_clean());
    assert_eq!(read.chunks, chunks);
}

fn chunk(timestamps: &[i64], vps: u32, bits: Vec<u32>, written: i64) -> SensorChunk {
    let n = timestamps.len();
    let accuracy: Vec<i32> = (0..n as i32).map(|i| i % 4 - 1).collect();
    SensorChunk::from_sample_bits(timestamps, vps, bits, &accuracy, written).unwrap()
}

#[test]
fn empty_stream() {
    roundtrip(&[]);
}

#[test]
fn empty_chunks() {
    roundtrip(&[SensorChunk::default()]);
    roundtrip(&[SensorChunk::default(), SensorChunk::default()]);
    roundtrip(&[
        chunk(&[], 3, vec![], 5),
        chunk(&[1], 3, vec![1, 2, 3], 6),
        chunk(&[], 3, vec![], 7),
    ]);
}

#[test]
fn single_sample() {
    roundtrip(&[chunk(&[123_456_789], 1, vec![0x447a_0000], 99)]);
}

#[test]
fn nan_payloads_negative_zero_and_extremes() {
    let bits = vec![
        0x8000_0000,
        0,
        0x7fc0_0000,
        0xffc0_0001,
        0x7f80_0001,
        0x7f80_0000,
        0xff80_0000,
        0x0000_0001,
        0x8000_0001,
        u32::MAX,
        0x7fff_ffff,
        0x3f80_0000,
    ];
    let ts: Vec<i64> = (0..4).map(|i| i * 10).collect();
    roundtrip(&[chunk(&ts, 3, bits, -1)]);
}

#[test]
fn non_monotonic_and_extreme_timestamps() {
    // Raw deltas, as a chunk holds them, so the extremes wrap.
    let deltas = vec![i64::MAX, 5, i64::MIN, -3, 0, 0, 7, i64::MAX - 1, -1];
    let raw = |deltas: Vec<i64>, written| SensorChunk {
        value_bits: (0..deltas.len() as u32)
            .map(|i| i.wrapping_mul(0x9e37_79b9))
            .collect(),
        accuracy: vec![3; deltas.len()],
        timestamp_deltas_ns: deltas,
        values_per_sample: 1,
        written_elapsed_realtime_ns: written,
    };
    roundtrip(&[
        raw(deltas.clone(), i64::MIN),
        raw(deltas[..3].to_vec(), i64::MAX),
    ]);
    // Going back in time between and within chunks.
    roundtrip(&[
        chunk(&[100, 90, 95], 1, vec![1, 2, 3], 0),
        chunk(&[50, 50, 49], 1, vec![3, 2, 1], 0),
    ]);
}

#[test]
fn many_chunks_like_a_ride() {
    let mut chunks = Vec::new();
    for c in 0..40i64 {
        let ts: Vec<i64> = (0..200)
            .map(|i| 1_000_000_000 + (c * 200 + i) * 2_500_000 + i % 3)
            .collect();
        let bits = (0..600)
            .map(|i| (9.81f32 + ((c * 600 + i) % 37) as f32 * 0.0012).to_bits())
            .collect();
        chunks.push(chunk(&ts, 3, bits, c * 2_000_000_000));
    }
    roundtrip(&chunks);
}

#[test]
fn refuses_what_it_cant_give_back() {
    // Values that aren't whole samples.
    let odd = SensorChunk {
        timestamp_deltas_ns: vec![1, 2],
        values_per_sample: 3,
        value_bits: vec![1, 2, 3, 4],
        accuracy: vec![0, 0],
        written_elapsed_realtime_ns: 0,
    };
    assert!(encode_file(&[odd.encode_to_vec()], LEVEL).is_err());
    // Bytes protobuf wouldn't write this way (a field repeated).
    let mut body = SensorChunk::default().encode_to_vec();
    body.extend_from_slice(&[0x10, 1, 0x10, 1]);
    assert!(encode_file(&[body], LEVEL).is_err());
}

fn ride(dir: &Path, name: &str) -> Vec<SensorChunk> {
    let info = SessionInfo {
        format_version: FORMAT_VERSION,
        streams: vec![StreamInfo {
            name: name.into(),
            ..Default::default()
        }],
        ..Default::default()
    };
    write_session_info(dir, &info).unwrap();
    let mut w = StreamWriter::create(dir, name).unwrap();
    let mut chunks = Vec::new();
    for c in 0..30i64 {
        let ts: Vec<i64> = (0..200)
            .map(|i| 5_000_000 + (c * 200 + i) * 2_500_000)
            .collect();
        let bits = (0..600)
            .map(|i| (0.3f32 + ((c * 600 + i) % 101) as f32 * 0.0007).to_bits())
            .collect();
        let ch = chunk(&ts, 3, bits, c);
        w.append(&ch).unwrap();
        chunks.push(ch);
    }
    chunks
}

#[test]
fn session_recompresses_sensors_to_columns_and_reads_the_same() {
    let dir = tempfile::tempdir().unwrap();
    let chunks = ride(dir.path(), "accel_uncal");
    let results = recompress_session(dir.path(), LEVEL).unwrap();
    assert!(
        matches!(results[0].1, Outcome::Replaced { .. }),
        "{results:?}"
    );
    let session = Session::open(dir.path()).unwrap();
    assert!(is_columnar(
        &fs::read(session.stream_path("accel_uncal")).unwrap()
    ));
    let read = session.read_stream("accel_uncal").unwrap();
    assert!(read.is_clean());
    assert_eq!(read.chunks, chunks);
    let streamed: Vec<SensorChunk> = session
        .stream_chunks("accel_uncal")
        .unwrap()
        .collect::<anyhow::Result<_>>()
        .unwrap();
    assert_eq!(streamed, chunks);
    // Again: already columns, nothing to gain, kept.
    let again = recompress_sensor_stream(&session.stream_path("accel_uncal"), LEVEL);
    assert!(matches!(again, Outcome::KeptOriginal { .. }), "{again}");
    assert_eq!(session.read_stream("accel_uncal").unwrap().chunks, chunks);
}

#[test]
fn keeps_original_when_the_columns_file_is_damaged() {
    let dir = tempfile::tempdir().unwrap();
    ride(dir.path(), "gyro_uncal");
    let path = dir.path().join("gyro_uncal.zst");
    let original = fs::read(&path).unwrap();
    let outcome = recompress_stream_checked(&path, LEVEL, Encoding::Sensor, |tmp| {
        let mut bytes = fs::read(tmp).unwrap();
        let mid = bytes.len() / 2;
        bytes[mid] ^= 0x40;
        fs::write(tmp, bytes).unwrap();
    });
    assert!(matches!(outcome, Outcome::KeptOriginal { .. }), "{outcome}");
    assert_eq!(fs::read(&path).unwrap(), original);
}

#[test]
fn damaged_columns_read_as_damage() {
    let dir = tempfile::tempdir().unwrap();
    ride(dir.path(), "mag_uncal");
    let path = dir.path().join("mag_uncal.zst");
    recompress_sensor_stream(&path, LEVEL);
    let bytes = fs::read(&path).unwrap();
    let read = decode_stream_bytes::<SensorChunk>(&bytes[..bytes.len() - 7]);
    assert!(read.chunks.is_empty());
    assert_eq!(read.damage.len(), 1);
}

#[test]
fn unencodable_sensor_stream_falls_back_to_plain() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("light.zst");
    let mut w = StreamWriter::create(dir.path(), "light").unwrap();
    let mixed = [
        chunk(&[1, 2], 1, vec![5, 6], 0),
        chunk(&[3, 4], 2, vec![5, 6, 7, 8], 0),
    ];
    for _ in 0..50 {
        for c in &mixed {
            w.append(c).unwrap();
        }
    }
    let outcome = recompress_sensor_stream(&path, LEVEL);
    assert!(matches!(outcome, Outcome::Replaced { .. }), "{outcome}");
    let bytes = fs::read(&path).unwrap();
    assert!(!is_columnar(&bytes));
    assert_eq!(decode_stream_bytes::<SensorChunk>(&bytes).chunks.len(), 100);
}

#[test]
fn blocks_round_trip_and_every_stream_gets_columns() {
    let dir = tempfile::tempdir().unwrap();
    let chunks = ride(dir.path(), "accel_uncal");
    // Single block is the old form: the same bytes `encode_file` gives.
    let bodies: Vec<Vec<u8>> = chunks.iter().map(|c| c.encode_to_vec()).collect();
    let old = encode_file(&bodies, LEVEL).unwrap();
    assert_eq!(
        encode_chunks_file_in_blocks(&chunks, LEVEL, u64::MAX).unwrap(),
        old
    );
    assert_eq!(decode_file(&old).unwrap(), content(&chunks));
    let read = decode_stream_bytes::<SensorChunk>(&old);
    assert!(read.is_clean());
    assert_eq!(read.chunks, chunks);
    // Blocks of a few chunks, and of one chunk each (a block always has a chunk).
    for block in [3 * content_len(&chunks[0]), 1] {
        let blocked = encode_chunks_file_in_blocks(&chunks, LEVEL, block).unwrap();
        assert!(is_columnar(&blocked));
        assert_ne!(blocked, old);
        assert_eq!(decode_file(&blocked).unwrap(), content(&chunks));
        let read = decode_stream_bytes::<SensorChunk>(&blocked);
        assert!(read.is_clean());
        assert_eq!(read.chunks, chunks);
    }
    // No stream is too long for columns: through the recompressor, in blocks, with the same content.
    let path = dir.path().join("accel_uncal.zst");
    let original = content_sha256(&path, Encoding::Sensor).unwrap();
    let options = Options {
        block_bytes: 4 * content_len(&chunks[0]),
        ..Options::default()
    };
    let outcome = recompress_stream_with(&path, LEVEL, Encoding::Sensor, options);
    let Outcome::Replaced { sha256, .. } = outcome else {
        panic!("{outcome}")
    };
    assert_eq!(sha256, original);
    let bytes = fs::read(&path).unwrap();
    assert!(is_columnar(&bytes));
    assert_eq!(decode_file(&bytes).unwrap(), content(&chunks));
    assert_eq!(content_sha256(&path, Encoding::Sensor).unwrap(), original);
}

#[test]
fn session_info_gets_the_content_hash_and_refuses_a_mismatch() {
    let dir = tempfile::tempdir().unwrap();
    let chunks = ride(dir.path(), "gyro_uncal");
    let path = dir.path().join("gyro_uncal.zst");
    let original = content_sha256(&path, Encoding::Sensor).unwrap();
    // Old session: no hash. Recompression writes it.
    assert!(
        Session::open(dir.path()).unwrap().info.streams[0]
            .content_sha256
            .is_empty()
    );
    recompress_session(dir.path(), LEVEL).unwrap();
    let info = Session::open(dir.path()).unwrap().info;
    assert_eq!(info.streams[0].content_sha256, original.to_vec());
    assert!(is_columnar(&fs::read(&path).unwrap()));
    assert_eq!(
        decode_file(&fs::read(&path).unwrap()).unwrap(),
        content(&chunks)
    );

    // A different hash recorded: the stream is kept as it is, and the hash stays.
    let dir = tempfile::tempdir().unwrap();
    ride(dir.path(), "gyro_uncal");
    let path = dir.path().join("gyro_uncal.zst");
    let before = fs::read(&path).unwrap();
    let mut info = Session::open(dir.path()).unwrap().info;
    info.streams[0].content_sha256 = vec![7; 32];
    write_session_info(dir.path(), &info).unwrap();
    let outcomes = recompress_session(dir.path(), LEVEL).unwrap();
    let Outcome::KeptOriginal { reason, .. } = &outcomes[0].1 else {
        panic!("{}", outcomes[0].1)
    };
    assert!(reason.contains("differs"), "{reason}");
    assert_eq!(fs::read(&path).unwrap(), before);
    let info = Session::open(dir.path()).unwrap().info;
    assert_eq!(info.streams[0].content_sha256, vec![7; 32]);

    // The right hash recorded: replaced as usual.
    let dir = tempfile::tempdir().unwrap();
    ride(dir.path(), "gyro_uncal");
    let mut info = Session::open(dir.path()).unwrap().info;
    info.streams[0].content_sha256 = original.to_vec();
    write_session_info(dir.path(), &info).unwrap();
    let outcomes = recompress_session(dir.path(), LEVEL).unwrap();
    assert!(matches!(outcomes[0].1, Outcome::Replaced { .. }));
}

#[test]
fn marker_names_each_stream_content_sha256_before_and_after() {
    let dir = tempfile::tempdir().unwrap();
    ride(dir.path(), "gyro_uncal");
    let path = dir.path().join("gyro_uncal.zst");
    let hex = |d: [u8; 32]| d.iter().map(|b| format!("{b:02x}")).collect::<String>();
    let before = hex(content_sha256(&path, Encoding::Sensor).unwrap());
    recompress_session(dir.path(), LEVEL).unwrap();
    assert!(is_columnar(&fs::read(&path).unwrap()));
    assert_eq!(
        hex(content_sha256(&path, Encoding::Sensor).unwrap()),
        before
    );
    let marker = fs::read_to_string(dir.path().join(MARKER)).unwrap();
    assert!(marker.starts_with("gyro_uncal.zst: replaced, "), "{marker}");
    assert!(
        marker.contains(&format!("content SHA-256 {before}\n")),
        "{marker}"
    );
    // Again: the marker says it's done, so nothing is redone and it still names the same content.
    assert!(recompress_session(dir.path(), LEVEL).unwrap().is_empty());
    assert_eq!(fs::read_to_string(dir.path().join(MARKER)).unwrap(), marker);
}
