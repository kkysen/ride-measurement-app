//! Checks a session written by the Kotlin writer (`android/app/src/test/.../FixtureTest.kt`) decodes bit-exactly.
//! Run with `RIDE_KOTLIN_FIXTURE=<dir>`; skipped otherwise.

use ride_session::read::Session;

// Keep in sync with `FixtureTest.kt`.
const CHUNKS: usize = 3;
const SAMPLES_PER_CHUNK: usize = 100;
const TRICKY_BITS: [u32; 10] = [
    0x0000_0000,
    0x8000_0000,
    0x0080_0000,
    0x0000_0001,
    0x7f7f_ffff,
    0x7f80_0000,
    0xff80_0000,
    0x7fc0_1234,
    0x411c_e80a,
    0xb4a5_b4b2,
];

fn fixture_bits(i: usize) -> u32 {
    TRICKY_BITS[i % TRICKY_BITS.len()] ^ i as u32
}

fn fixture_timestamp(sample: usize) -> i64 {
    1_000_000_000 + sample as i64 * 2_500_000 + (sample % 3) as i64
}

#[test]
fn kotlin_fixture_decodes_bit_exactly() {
    let Ok(dir) = std::env::var("RIDE_KOTLIN_FIXTURE") else {
        eprintln!("RIDE_KOTLIN_FIXTURE not set, skipping");
        return;
    };
    let session = Session::open(dir).unwrap();
    assert_eq!(session.info.session_id, "kotlin-fixture");
    let read = session.read_stream("accel").unwrap();
    assert!(!read.truncated);
    assert_eq!(read.chunks.len(), CHUNKS);
    for (c, chunk) in read.chunks.iter().enumerate() {
        assert_eq!(chunk.values_per_sample, 3);
        assert_eq!(chunk.written_elapsed_realtime_ns, c as i64);
        let timestamps = chunk.timestamps_ns();
        for i in 0..SAMPLES_PER_CHUNK {
            let sample = c * SAMPLES_PER_CHUNK + i;
            assert_eq!(timestamps[i], fixture_timestamp(sample));
            assert_eq!(chunk.accuracy[i], (sample % 4) as i32);
            for k in 0..3 {
                assert_eq!(
                    chunk.value_bits[i * 3 + k],
                    fixture_bits(sample * 3 + k),
                    "sample {sample} value {k}"
                );
            }
        }
    }
}
