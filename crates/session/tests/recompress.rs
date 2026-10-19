use std::fs;
use std::path::{Path, PathBuf};

use ride_session::proto::StreamInfo;
use ride_session::read::Session;
use ride_session::recompress::{
    LEVEL, MARKER, Outcome, recompress_session, recompress_stream, recompress_stream_checked,
};
use ride_session::write::{StreamWriter, write_session_info};
use ride_session::{FORMAT_VERSION, SensorChunk, SessionInfo};

/// A stream of `chunks` chunks in the live format: one zstd frame each.
fn write_stream(dir: &Path, name: &str, chunks: usize) -> PathBuf {
    let mut w = StreamWriter::create(dir, name).unwrap();
    for c in 0..chunks {
        let n = 200;
        let timestamps: Vec<i64> = (0..n)
            .map(|i| 1_000_000_000 + ((c * n + i) as i64) * 2_500_000)
            .collect();
        let values: Vec<f32> = (0..n * 3)
            .map(|i| f32::from_bits(0x411c_e80a ^ (c * n * 3 + i) as u32))
            .collect();
        w.append(&SensorChunk::from_samples(
            &timestamps,
            3,
            &values,
            &vec![3; n],
            c as i64,
        ))
        .unwrap();
    }
    w.path().to_owned()
}

fn decoded(path: &Path) -> Vec<u8> {
    zstd::decode_all(fs::File::open(path).unwrap()).unwrap()
}

fn listing(dir: &Path) -> Vec<String> {
    let mut names: Vec<_> = fs::read_dir(dir)
        .unwrap()
        .map(|e| e.unwrap().file_name().into_string().unwrap())
        .collect();
    names.sort();
    names
}

#[test]
fn replaces_with_smaller_file_with_identical_content() {
    let dir = tempfile::tempdir().unwrap();
    let path = write_stream(dir.path(), "accel", 50);
    let before = decoded(&path);
    let outcome = recompress_stream(&path, LEVEL);
    let Outcome::Replaced {
        before: b,
        after: a,
    } = outcome
    else {
        panic!("{outcome}")
    };
    assert!(a < b);
    assert_eq!(decoded(&path), before);
    assert_eq!(listing(dir.path()), ["accel.zst"]);
}

#[test]
fn keeps_original_when_verification_fails() {
    let dir = tempfile::tempdir().unwrap();
    let path = write_stream(dir.path(), "accel", 50);
    let original = fs::read(&path).unwrap();
    let outcome = recompress_stream_checked(&path, LEVEL, |tmp| {
        let mut bytes = fs::read(tmp).unwrap();
        let mid = bytes.len() / 2;
        bytes[mid] ^= 0xff;
        fs::write(tmp, bytes).unwrap();
    });
    assert!(matches!(outcome, Outcome::KeptOriginal { .. }), "{outcome}");
    assert_eq!(fs::read(&path).unwrap(), original);
    assert_eq!(listing(dir.path()), ["accel.zst"]);
}

#[test]
fn keeps_crash_truncated_original() {
    let dir = tempfile::tempdir().unwrap();
    let path = write_stream(dir.path(), "accel", 5);
    let mut bytes = fs::read(&path).unwrap();
    bytes.truncate(bytes.len() - 10);
    fs::write(&path, &bytes).unwrap();
    let outcome = recompress_stream(&path, LEVEL);
    assert!(matches!(outcome, Outcome::KeptOriginal { .. }), "{outcome}");
    assert_eq!(fs::read(&path).unwrap(), bytes);
}

#[test]
fn session_reads_the_same_and_info_is_untouched() {
    let dir = tempfile::tempdir().unwrap();
    let info = SessionInfo {
        format_version: FORMAT_VERSION,
        streams: vec![StreamInfo {
            name: "accel".into(),
            ..Default::default()
        }],
        ..Default::default()
    };
    write_session_info(dir.path(), &info).unwrap();
    write_stream(dir.path(), "accel", 3);
    let info_bytes = fs::read(dir.path().join("session.pb")).unwrap();
    let before = Session::open(dir.path())
        .unwrap()
        .read_stream("accel")
        .unwrap()
        .chunks;
    let results = recompress_session(dir.path(), LEVEL).unwrap();
    assert!(
        matches!(results[0].1, Outcome::Replaced { .. }),
        "{results:?}"
    );
    assert_eq!(fs::read(dir.path().join("session.pb")).unwrap(), info_bytes);
    assert!(dir.path().join(MARKER).exists());
    let contents = Session::open(dir.path())
        .unwrap()
        .read_stream("accel")
        .unwrap();
    assert!(contents.is_clean());
    assert_eq!(contents.chunks, before);
}

#[test]
fn keeps_original_cut_at_a_frame_boundary_mid_message() {
    // Complete zstd frames, but the decoded content ends inside a message.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("accel.zst");
    let mut encoder = zstd::stream::Encoder::new(Vec::new(), 3).unwrap();
    std::io::Write::write_all(&mut encoder, &[10, 1, 2, 3]).unwrap(); // claims 10 bytes, has 3
    let bytes = encoder.finish().unwrap();
    fs::write(&path, &bytes).unwrap();
    let outcome = recompress_stream(&path, LEVEL);
    assert!(matches!(outcome, Outcome::KeptOriginal { .. }), "{outcome}");
    assert_eq!(fs::read(&path).unwrap(), bytes);
}
