//! FLAC for audio chunks: post-ride recompression moves each chunk's PCM into a FLAC stream, and readers move it back.
//!
//! A chunk is converted only if converting it back gives the original message byte for byte,
//! so a chunk that can't be (an odd byte count, an encoder failure, a field the round trip would drop) stays as PCM.

use flacenc::component::BitRepr;
use flacenc::error::Verify;
use prost::Message;

use crate::AudioChunk;

/// The chunk's samples as a FLAC stream, if it has PCM and isn't already converted.
fn encode(pcm: &[u8], channels: usize, sample_rate_hz: usize) -> Option<Vec<u8>> {
    if pcm.is_empty() || channels == 0 || !pcm.len().is_multiple_of(2 * channels) {
        return None;
    }
    let samples: Vec<i32> = pcm
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&b| i32::from(i16::from_le_bytes(b)))
        .collect();
    let config = flacenc::config::Encoder::default().into_verified().ok()?;
    let source = flacenc::source::MemSource::from_samples(&samples, channels, 16, sample_rate_hz);
    let stream = flacenc::encode_with_fixed_block_size(&config, source, config.block_size).ok()?;
    let mut sink = flacenc::bitsink::ByteSink::new();
    stream.write(&mut sink).ok()?;
    Some(sink.as_slice().to_vec())
}

/// The PCM a FLAC stream decodes to, as 16-bit little-endian.
fn decode(flac: &[u8]) -> Option<Vec<u8>> {
    let mut reader = claxon::FlacReader::new(std::io::Cursor::new(flac)).ok()?;
    if reader.streaminfo().bits_per_sample != 16 {
        return None;
    }
    let mut pcm = Vec::new();
    for s in reader.samples() {
        let s = i16::try_from(s.ok()?).ok()?;
        pcm.extend_from_slice(&s.to_le_bytes());
    }
    Some(pcm)
}

/// The encoded chunk with its PCM as FLAC, or `None` to keep it as it is.
pub fn to_flac(message: &[u8], channels: usize, sample_rate_hz: usize) -> Option<Vec<u8>> {
    let mut chunk = AudioChunk::decode(message).ok()?;
    if !chunk.flac.is_empty() {
        return None;
    }
    chunk.flac = encode(&chunk.pcm_s16le, channels, sample_rate_hz)?;
    chunk.pcm_s16le.clear();
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

/// Moves a decoded chunk's FLAC back into `pcm_s16le`; false if it doesn't decode.
pub fn restore(chunk: &mut AudioChunk) -> bool {
    if chunk.flac.is_empty() {
        return true;
    }
    match decode(&chunk.flac) {
        Some(pcm) => {
            chunk.pcm_s16le = pcm;
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
}
