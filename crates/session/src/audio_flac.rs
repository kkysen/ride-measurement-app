//! FLAC for audio chunks: post-ride recompression moves each chunk's PCM into a FLAC stream, and readers move it back.
//!
//! A chunk is converted only if converting it back gives the original message byte for byte,
//! so a chunk that can't be (an odd byte count, an encoder failure, a field the round trip would drop,
//! a float that isn't exactly a 24-bit sample) stays as PCM, under the stream's zstd.

use flacenc::component::BitRepr;
use flacenc::error::Verify;
use prost::Message;

use crate::{AudioChunk, Pcm};

/// The scale of 24-bit samples given as float: Android gives a 24-bit input's sample k as k / 2^23.
const F32_SCALE: f32 = 8_388_608.0;

/// The 24-bit integer a float sample is exactly, if it is one: k / 2^23 for k in [-2^23, 2^23),
/// and not negative zero, which the integer can't tell from zero.
fn f32_as_i24(v: f32) -> Option<i32> {
    let k = v * F32_SCALE;
    (k.fract() == 0.0
        && (-F32_SCALE..F32_SCALE).contains(&k)
        && !(v == 0.0 && v.is_sign_negative()))
    .then_some(k as i32)
}

/// The chunk's samples as a FLAC stream, if it has PCM and isn't already converted:
/// 16-bit PCM as 16-bit FLAC, float as 24-bit FLAC if every sample is exactly a 24-bit integer.
fn encode(pcm: Pcm<'_>, channels: usize, sample_rate_hz: usize) -> Option<Vec<u8>> {
    let bytes = pcm.bytes();
    if bytes.is_empty()
        || channels == 0
        || !bytes
            .len()
            .is_multiple_of(pcm.bytes_per_sample() * channels)
    {
        return None;
    }
    let (samples, bits): (Vec<i32>, usize) = match pcm {
        Pcm::S16(b) => (
            b.as_chunks::<2>()
                .0
                .iter()
                .map(|&b| i32::from(i16::from_le_bytes(b)))
                .collect(),
            16,
        ),
        Pcm::F32(b) => (
            b.as_chunks::<4>()
                .0
                .iter()
                .map(|&b| f32_as_i24(f32::from_le_bytes(b)))
                .collect::<Option<_>>()?,
            24,
        ),
    };
    let config = flacenc::config::Encoder::default().into_verified().ok()?;
    let source = flacenc::source::MemSource::from_samples(&samples, channels, bits, sample_rate_hz);
    let stream = flacenc::encode_with_fixed_block_size(&config, source, config.block_size).ok()?;
    let mut sink = flacenc::bitsink::ByteSink::new();
    stream.write(&mut sink).ok()?;
    Some(sink.as_slice().to_vec())
}

/// The PCM a FLAC stream decodes to: 16-bit little-endian from 16-bit FLAC (`false`),
/// 32-bit little-endian float from 24-bit FLAC (`true`).
fn decode(flac: &[u8]) -> Option<(Vec<u8>, bool)> {
    let mut reader = claxon::FlacReader::new(std::io::Cursor::new(flac)).ok()?;
    let float = match reader.streaminfo().bits_per_sample {
        16 => false,
        24 => true,
        _ => return None,
    };
    let mut pcm = Vec::new();
    for s in reader.samples() {
        let s = s.ok()?;
        if float {
            pcm.extend_from_slice(&(s as f32 / F32_SCALE).to_le_bytes());
        } else {
            pcm.extend_from_slice(&i16::try_from(s).ok()?.to_le_bytes());
        }
    }
    Some((pcm, float))
}

/// The encoded chunk with its PCM as FLAC, or `None` to keep it as it is.
pub fn to_flac(message: &[u8], channels: usize, sample_rate_hz: usize) -> Option<Vec<u8>> {
    let mut chunk = AudioChunk::decode(message).ok()?;
    if !chunk.flac.is_empty() || (!chunk.pcm_s16le.is_empty() && !chunk.pcm_f32le.is_empty()) {
        return None;
    }
    chunk.flac = encode(chunk.pcm(), channels, sample_rate_hz)?;
    chunk.pcm_s16le.clear();
    chunk.pcm_f32le.clear();
    let converted = chunk.encode_to_vec();
    (converted.len() < message.len() && to_pcm(&converted).as_deref() == Some(message))
        .then_some(converted)
}

/// The encoded chunk with FLAC moved back into PCM, as recorded; `None` if it has no FLAC or it doesn't decode.
pub fn to_pcm(message: &[u8]) -> Option<Vec<u8>> {
    let mut chunk = AudioChunk::decode(message).ok()?;
    if chunk.flac.is_empty() {
        return None;
    }
    restore(&mut chunk).then(|| chunk.encode_to_vec())
}

/// Moves a decoded chunk's FLAC back into `pcm_s16le` or `pcm_f32le`; false if it doesn't decode.
pub fn restore(chunk: &mut AudioChunk) -> bool {
    if chunk.flac.is_empty() {
        return true;
    }
    match decode(&chunk.flac) {
        Some((pcm, float)) => {
            if float {
                chunk.pcm_f32le = pcm;
            } else {
                chunk.pcm_s16le = pcm;
            }
            chunk.flac.clear();
            true
        }
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::AudioTimestamp;

    fn chunk(pcm: Vec<u8>) -> Vec<u8> {
        AudioChunk {
            first_frame: 32_000,
            pcm_s16le: pcm,
            pcm_f32le: Vec::new(),
            timestamps: vec![AudioTimestamp {
                frame_position: 32_100,
                boottime_ns: 123_456_789,
            }],
            written_elapsed_realtime_ns: 987_654_321,
            flac: Vec::new(),
        }
        .encode_to_vec()
    }

    #[test]
    fn round_trips_bit_exact_and_smaller() {
        // A tone with noise and the extremes, 2 s at 16 kHz.
        let mut x: u32 = 1;
        let pcm: Vec<u8> = (0..32_000)
            .flat_map(|i| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = (x >> 24) as f64 - 128.0;
                let s = match i {
                    100 => i16::MIN,
                    101 => i16::MAX,
                    _ => ((i as f64 * 0.2).sin() * 8000.0 + noise) as i16,
                };
                s.to_le_bytes()
            })
            .collect();
        let original = chunk(pcm);
        let flac = to_flac(&original, 1, 16_000).expect("converted");
        assert!(
            flac.len() < original.len() * 3 / 4,
            "{} of {}",
            flac.len(),
            original.len()
        );
        assert_eq!(to_pcm(&flac).unwrap(), original);
    }

    #[test]
    fn keeps_what_it_cant_convert() {
        // An odd byte count isn't whole samples.
        assert_eq!(to_flac(&chunk(vec![1, 2, 3]), 1, 16_000), None);
        assert_eq!(to_flac(&chunk(Vec::new()), 1, 16_000), None);
    }

    fn float_chunk(samples: impl IntoIterator<Item = f32>) -> Vec<u8> {
        let mut c = AudioChunk::decode(chunk(Vec::new()).as_slice()).unwrap();
        c.pcm_f32le = samples.into_iter().flat_map(f32::to_le_bytes).collect();
        c.encode_to_vec()
    }

    #[test]
    fn round_trips_24_bit_floats() {
        let mut x: u32 = 1;
        let samples: Vec<f32> = (0..64_000)
            .map(|i| {
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let k = match i {
                    100 => -(1 << 23),
                    101 => (1 << 23) - 1,
                    _ => ((i as f64 * 0.2).sin() * 2e6) as i32 + (x >> 20) as i32 - 2048,
                };
                k as f32 / F32_SCALE
            })
            .collect();
        let original = float_chunk(samples);
        let flac = to_flac(&original, 2, 48_000).expect("converted");
        assert!(
            flac.len() < original.len() * 3 / 4,
            "{} of {}",
            flac.len(),
            original.len()
        );
        assert_eq!(to_pcm(&flac).unwrap(), original);
        let mut c = AudioChunk::decode(flac.as_slice()).unwrap();
        assert!(restore(&mut c));
        assert!(c.pcm_s16le.is_empty() && !c.pcm_f32le.is_empty());
    }

    #[test]
    fn keeps_floats_finer_than_24_bits() {
        let finer = (0..4000).map(|i| (i as f32 * 0.01).sin() * 0.3);
        assert_eq!(to_flac(&float_chunk(finer), 2, 48_000), None);
        // Negative zero, out of range, and NaN aren't 24-bit samples either.
        for odd in [-0.0, 1.0, -1.5, f32::NAN] {
            let samples = (0..4000).map(|i| if i == 7 { odd } else { 0.25 });
            assert_eq!(to_flac(&float_chunk(samples), 2, 48_000), None, "{odd}");
        }
    }
}
