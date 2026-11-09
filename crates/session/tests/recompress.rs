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
    let outcome = recompress_stream_checked(&path, LEVEL, None, |tmp| {
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

/// How `write_audio` records the tone.
#[derive(Clone, Copy)]
enum Format {
    S16,
    /// Float, each sample exactly a 24-bit integer over 2^23, as a 24-bit input gives it.
    F32From24,
    /// Float with more precision than 24 bits.
    F32Fine,
}

/// An audio stream of `chunks` 2 s chunks of a noisy tone, in the live format.
fn write_audio(dir: &Path, chunks: usize) -> PathBuf {
    write_audio_as(dir, chunks, Format::S16)
}

fn write_audio_as(dir: &Path, chunks: usize, format: Format) -> PathBuf {
    let mut w = StreamWriter::create(dir, "audio").unwrap();
    let mut x: u32 = 7;
    for c in 0..chunks {
        let samples: Vec<f64> = (0..32_000)
            .map(|i| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let t = (c * 32_000 + i) as f64;
                (t * 0.07).sin() * 3000.0 + f64::from(x >> 26)
            })
            .collect();
        let bytes = |f: &dyn Fn(f64) -> [u8; 4]| samples.iter().flat_map(|&s| f(s)).collect();
        let (pcm_s16le, pcm_f32le) = match format {
            Format::S16 => (
                samples
                    .iter()
                    .flat_map(|&s| (s as i16).to_le_bytes())
                    .collect(),
                Vec::new(),
            ),
            // 256 times the 16-bit tone, so it uses 24 bits.
            Format::F32From24 => (
                Vec::new(),
                bytes(&|s| ((s * 256.0).round() as f32 / 8_388_608.0).to_le_bytes()),
            ),
            Format::F32Fine => (
                Vec::new(),
                bytes(&|s| ((s / 32_768.0) as f32).to_le_bytes()),
            ),
        };
        w.append(&ride_session::AudioChunk {
            first_frame: (c * 32_000) as i64,
            pcm_s16le,
            pcm_f32le,
            timestamps: Vec::new(),
            written_elapsed_realtime_ns: c as i64,
            flac: Vec::new(),
        })
        .unwrap();
    }
    w.path().to_owned()
}

fn audio_session(dir: &Path) {
    write_session_info(
        dir,
        &SessionInfo {
            format_version: FORMAT_VERSION,
            streams: vec![StreamInfo {
                name: "audio".into(),
                kind: ride_session::proto::StreamKind::Audio as i32,
                audio: Some(ride_session::proto::AudioInfo {
                    sample_rate_hz: 16_000,
                    channels: 1,
                    ..Default::default()
                }),
                ..Default::default()
            }],
            ..Default::default()
        },
    )
    .unwrap();
}

#[test]
fn audio_becomes_flac_and_reads_back_as_recorded() {
    let dir = tempfile::tempdir().unwrap();
    audio_session(dir.path());
    let path = write_audio(dir.path(), 5);
    let before = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap()
        .chunks;
    // The same stream recompressed by zstd alone, to beat.
    let plain = tempfile::tempdir().unwrap();
    let plain_path = write_audio(plain.path(), 5);
    recompress_stream(&plain_path, LEVEL);
    let zstd_only = fs::metadata(&plain_path).unwrap().len();
    let results = recompress_session(dir.path(), LEVEL).unwrap();
    assert!(
        matches!(results[0].1, Outcome::Replaced { .. }),
        "{results:?}"
    );
    let flac = fs::metadata(&path).unwrap().len();
    assert!(flac < zstd_only, "{flac} >= {zstd_only}");
    let raw = ride_session::read::read_stream_file::<ride_session::AudioChunk>(&path).unwrap();
    assert!(
        raw.chunks
            .iter()
            .all(|c| c.pcm_s16le.is_empty() && !c.flac.is_empty())
    );
    let after = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap();
    assert!(after.is_clean());
    assert_eq!(after.chunks, before);
    // Again: already FLAC, nothing to gain, and still the same.
    let again = recompress_session(dir.path(), LEVEL).unwrap();
    assert!(
        matches!(again[0].1, Outcome::KeptOriginal { .. }),
        "{again:?}"
    );
    assert_eq!(
        Session::open(dir.path())
            .unwrap()
            .read_audio("audio")
            .unwrap()
            .chunks,
        before
    );
}

#[test]
fn keeps_audio_whose_flac_is_damaged() {
    let dir = tempfile::tempdir().unwrap();
    let path = write_audio(dir.path(), 3);
    let original = fs::read(&path).unwrap();
    let audio = ride_session::recompress::Audio {
        channels: 1,
        sample_rate_hz: 16_000,
    };
    let outcome = recompress_stream_checked(&path, LEVEL, Some(audio), |tmp| {
        // Rewrite the new file with one FLAC byte flipped, so it still decodes as zstd.
        let mut bytes = decoded(tmp);
        let n = bytes.len();
        bytes[n / 2] ^= 0x10;
        fs::write(tmp, zstd::encode_all(bytes.as_slice(), 3).unwrap()).unwrap();
    });
    assert!(
        matches!(outcome, Outcome::KeptOriginal { .. }),
        "{outcome:?}"
    );
    assert_eq!(fs::read(&path).unwrap(), original);
}

#[test]
fn float_audio_of_24_bit_samples_becomes_flac_and_reads_back_as_recorded() {
    let dir = tempfile::tempdir().unwrap();
    audio_session(dir.path());
    let path = write_audio_as(dir.path(), 3, Format::F32From24);
    let before = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap()
        .chunks;
    let plain = tempfile::tempdir().unwrap();
    let plain_path = write_audio_as(plain.path(), 3, Format::F32From24);
    recompress_stream(&plain_path, LEVEL);
    let zstd_only = fs::metadata(&plain_path).unwrap().len();
    let results = recompress_session(dir.path(), LEVEL).unwrap();
    assert!(
        matches!(results[0].1, Outcome::Replaced { .. }),
        "{results:?}"
    );
    let flac = fs::metadata(&path).unwrap().len();
    assert!(flac < zstd_only, "{flac} >= {zstd_only}");
    let raw = ride_session::read::read_stream_file::<ride_session::AudioChunk>(&path).unwrap();
    assert!(
        raw.chunks
            .iter()
            .all(|c| c.pcm_f32le.is_empty() && c.pcm_s16le.is_empty() && !c.flac.is_empty())
    );
    let after = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap();
    assert!(after.is_clean());
    assert_eq!(after.chunks, before);
    assert!(after.chunks.iter().all(|c| !c.pcm_f32le.is_empty()));
}

#[test]
fn float_audio_finer_than_24_bits_stays_float_under_zstd() {
    let dir = tempfile::tempdir().unwrap();
    audio_session(dir.path());
    let path = write_audio_as(dir.path(), 3, Format::F32Fine);
    let before = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap()
        .chunks;
    recompress_session(dir.path(), LEVEL).unwrap();
    let raw = ride_session::read::read_stream_file::<ride_session::AudioChunk>(&path).unwrap();
    assert!(
        raw.chunks
            .iter()
            .all(|c| !c.pcm_f32le.is_empty() && c.flac.is_empty())
    );
    let after = Session::open(dir.path())
        .unwrap()
        .read_audio("audio")
        .unwrap();
    assert!(after.is_clean());
    assert_eq!(after.chunks, before);
}
