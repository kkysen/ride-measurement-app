//! Columnar encoding of a sensor stream, for post-ride recompression (PLAN.md item 42).
//!
//! A file in this encoding is a zstd skippable frame holding [`TAG`], then one zstd frame (with checksum)
//! of the columns below. It decodes back to the stream's content, the sequence of length-delimited
//! `SensorChunk`s, byte for byte; [`encode`] refuses a stream it couldn't give back exactly.
//!
//! The columns, every integer a varint, signed ones zigzagged:
//! - the chunk count and `values_per_sample`, then per chunk its sample count and the change in
//!   `written_elapsed_realtime_ns` from the chunk before;
//! - each sample's timestamp as the change in its delta from the sample before (delta of delta, across chunks);
//! - each accuracy as the change from the one before;
//! - per axis, the stream's distinct values sorted (as differences of their order keys),
//!   then each sample's value as the change in its rank in that table from the sample before.
//!
//! All arithmetic wraps, so any timestamps (non-monotonic, huge jumps) and any bit patterns (NaN payloads, -0.0)
//! come back exactly.

use std::io::{self, Read};

use prost::Message;

use crate::SensorChunk;

/// The skippable frame's payload: names the encoding and its version.
pub const TAG: &[u8] = b"ride.sensor-columns.v1";
/// Magic of the zstd skippable frame that carries [`TAG`] (any of `0x184D2A50..=0x184D2A5F` is skippable).
const SKIPPABLE_MAGIC: u32 = 0x184D_2A5C;

/// The header every file in this encoding starts with.
fn header() -> Vec<u8> {
    let mut h = Vec::with_capacity(8 + TAG.len());
    h.extend_from_slice(&SKIPPABLE_MAGIC.to_le_bytes());
    h.extend_from_slice(&(TAG.len() as u32).to_le_bytes());
    h.extend_from_slice(TAG);
    h
}

/// The length of the header every file in this encoding starts with.
pub fn header_len() -> usize {
    8 + TAG.len()
}

/// Whether a file's bytes are in this encoding.
pub fn is_columnar(bytes: &[u8]) -> bool {
    bytes.starts_with(&header())
}

fn invalid(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, msg.into())
}

fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push(v as u8 | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}

fn zz(v: i64) -> u64 {
    ((v << 1) ^ (v >> 63)) as u64
}

fn unzz(v: u64) -> i64 {
    ((v >> 1) as i64) ^ -((v & 1) as i64)
}

/// An order key for an `f32`'s bits, increasing with `f32::total_cmp`, one to one with the bits.
fn key(bits: u32) -> u32 {
    if bits & 0x8000_0000 != 0 {
        !bits
    } else {
        bits | 0x8000_0000
    }
}

fn unkey(k: u32) -> u32 {
    if k & 0x8000_0000 != 0 {
        k & 0x7fff_ffff
    } else {
        !k
    }
}

struct Cursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn varint(&mut self) -> io::Result<u64> {
        let mut v = 0u64;
        for i in 0..10 {
            let byte = *self
                .bytes
                .get(self.pos)
                .ok_or_else(|| invalid("columns end early"))?;
            self.pos += 1;
            v |= u64::from(byte & 0x7f) << (7 * i);
            if byte & 0x80 == 0 {
                return Ok(v);
            }
        }
        Err(invalid("varint too long"))
    }

    fn count(&mut self) -> io::Result<usize> {
        let v = self.varint()?;
        // Each counted item takes at least a byte, so a count past the input is corrupt.
        if v > self.bytes.len() as u64 * 8 {
            return Err(invalid(format!("count {v} is past the input")));
        }
        Ok(v as usize)
    }

    fn section(&mut self) -> io::Result<Cursor<'a>> {
        let len = self.count()?;
        let end = self
            .pos
            .checked_add(len)
            .filter(|&e| e <= self.bytes.len())
            .ok_or_else(|| invalid("section past the end"))?;
        let s = Cursor {
            bytes: &self.bytes[self.pos..end],
            pos: 0,
        };
        self.pos = end;
        Ok(s)
    }
}

fn put_section(out: &mut Vec<u8>, section: &[u8]) {
    put_varint(out, section.len() as u64);
    out.extend_from_slice(section);
}

/// The columns for a stream's chunk bodies, or why they can't give the bodies back exactly
/// (a body protobuf wouldn't encode the same way, or chunks that aren't whole samples of one shape).
pub fn encode_columns(bodies: &[Vec<u8>]) -> Result<Vec<u8>, String> {
    let chunks = bodies
        .iter()
        .enumerate()
        .map(|(i, body)| column_chunk(body).map_err(|e| format!("chunk {i} {e}")))
        .collect::<Result<Vec<_>, _>>()?;
    encode_chunk_columns(&chunks)
}

/// A chunk body decoded for [`encode_chunk_columns`], or why the columns couldn't give it back exactly;
/// its vectors hold no spare capacity, so a whole stream of them takes little more than its content.
pub fn column_chunk(body: &[u8]) -> Result<SensorChunk, String> {
    let mut c = SensorChunk::decode(body).map_err(|e| format!("doesn't decode: {e}"))?;
    if c.encode_to_vec() != body {
        return Err("doesn't re-encode to the same bytes".into());
    }
    c.timestamp_deltas_ns.shrink_to_fit();
    c.value_bits.shrink_to_fit();
    c.accuracy.shrink_to_fit();
    Ok(c)
}

/// The columns for a stream's chunks, each from [`column_chunk`],
/// or why they can't give them back exactly (chunks that aren't whole samples of one shape).
pub fn encode_chunk_columns(chunks: &[SensorChunk]) -> Result<Vec<u8>, String> {
    let vps = chunks.first().map_or(1, |c| c.values_per_sample);
    if vps == 0 && chunks.iter().any(|c| !c.timestamp_deltas_ns.is_empty()) {
        return Err("samples without values".into());
    }
    for (i, c) in chunks.iter().enumerate() {
        let n = c.timestamp_deltas_ns.len();
        if c.values_per_sample != vps
            || c.value_bits.len() != n * vps as usize
            || c.accuracy.len() != n
        {
            return Err(format!("chunk {i} isn't whole samples of {vps} values"));
        }
    }
    let vps = vps as usize;

    let mut head = Vec::new();
    put_varint(&mut head, chunks.len() as u64);
    put_varint(&mut head, vps as u64);
    let mut ts = Vec::new();
    let mut acc = Vec::new();
    let mut prev_w = 0i64;
    let mut prev_t = 0i64;
    let mut prev_d = 0i64;
    let mut prev_a = 0i32;
    for c in chunks {
        put_varint(&mut head, c.timestamp_deltas_ns.len() as u64);
        put_varint(
            &mut head,
            zz(c.written_elapsed_realtime_ns.wrapping_sub(prev_w)),
        );
        prev_w = c.written_elapsed_realtime_ns;
        let mut t = 0i64;
        for &d in &c.timestamp_deltas_ns {
            // The first delta of a chunk is absolute, so this is the absolute time in every case.
            t = t.wrapping_add(d);
            let delta = t.wrapping_sub(prev_t);
            put_varint(&mut ts, zz(delta.wrapping_sub(prev_d)));
            prev_d = delta;
            prev_t = t;
        }
        for &a in &c.accuracy {
            put_varint(&mut acc, zz(i64::from(a.wrapping_sub(prev_a))));
            prev_a = a;
        }
    }

    let mut values = Vec::new();
    for axis in 0..vps {
        let column = || {
            chunks
                .iter()
                .flat_map(|c| c.value_bits.iter().skip(axis).step_by(vps))
                .map(|&b| key(b))
        };
        let mut table: Vec<u32> = column().collect();
        table.sort_unstable();
        table.dedup();
        put_varint(&mut values, table.len() as u64);
        let mut p = 0u32;
        for &k in &table {
            put_varint(&mut values, u64::from(k.wrapping_sub(p)));
            p = k;
        }
        let mut prev_rank = 0i64;
        for k in column() {
            let rank = table
                .binary_search(&k)
                .expect("every value is in its table") as i64;
            put_varint(&mut values, zz(rank - prev_rank));
            prev_rank = rank;
        }
    }

    let mut out = Vec::new();
    put_section(&mut out, &head);
    put_section(&mut out, &ts);
    put_section(&mut out, &acc);
    put_section(&mut out, &values);
    Ok(out)
}

/// The chunks the columns hold.
pub fn decode_columns(bytes: &[u8]) -> io::Result<Vec<SensorChunk>> {
    let mut all = Cursor { bytes, pos: 0 };
    let mut head = all.section()?;
    let mut ts = all.section()?;
    let mut acc = all.section()?;
    let mut values = all.section()?;
    if all.pos != bytes.len() {
        return Err(invalid("bytes after the columns"));
    }
    let n_chunks = head.count()?;
    let vps = u32::try_from(head.varint()?).map_err(|_| invalid("values per sample too large"))?;
    let mut chunks = Vec::with_capacity(n_chunks);
    let mut prev_w = 0i64;
    let mut prev_t = 0i64;
    let mut prev_d = 0i64;
    let mut prev_a = 0i32;
    let mut total = 0usize;
    for _ in 0..n_chunks {
        let n = usize::try_from(head.varint()?).map_err(|_| invalid("sample count too large"))?;
        let w = prev_w.wrapping_add(unzz(head.varint()?));
        prev_w = w;
        let mut deltas = Vec::with_capacity(n.min(1 << 20));
        let mut t = 0i64;
        for _ in 0..n {
            let delta = prev_d.wrapping_add(unzz(ts.varint()?));
            let abs = prev_t.wrapping_add(delta);
            deltas.push(abs.wrapping_sub(t));
            t = abs;
            prev_d = delta;
            prev_t = abs;
        }
        let mut accuracy = Vec::with_capacity(n.min(1 << 20));
        for _ in 0..n {
            let a = prev_a.wrapping_add(unzz(acc.varint()?) as i32);
            accuracy.push(a);
            prev_a = a;
        }
        total += n;
        chunks.push(SensorChunk {
            timestamp_deltas_ns: deltas,
            values_per_sample: vps,
            value_bits: Vec::new(),
            accuracy,
            written_elapsed_realtime_ns: w,
        });
    }
    if head.pos != head.bytes.len() || ts.pos != ts.bytes.len() || acc.pos != acc.bytes.len() {
        return Err(invalid("columns longer than their samples"));
    }
    let vps = vps as usize;
    if total > 0 && vps == 0 {
        return Err(invalid("samples without values"));
    }
    let mut column = vec![0u32; total];
    for axis in 0..vps {
        let len = values.count()?;
        let mut table = Vec::with_capacity(len);
        let mut p = 0u32;
        for _ in 0..len {
            p = p.wrapping_add(values.varint()? as u32);
            table.push(unkey(p));
        }
        let mut rank = 0i64;
        for v in column.iter_mut() {
            rank = rank.wrapping_add(unzz(values.varint()?));
            *v = *usize::try_from(rank)
                .ok()
                .and_then(|r| table.get(r))
                .ok_or_else(|| invalid("rank past its table"))?;
        }
        let mut s = 0;
        for c in &mut chunks {
            let n = c.timestamp_deltas_ns.len();
            if axis == 0 {
                c.value_bits.resize(n * vps, 0);
            }
            for i in 0..n {
                c.value_bits[i * vps + axis] = column[s + i];
            }
            s += n;
        }
    }
    if values.pos != values.bytes.len() {
        return Err(invalid("values longer than their samples"));
    }
    Ok(chunks)
}

/// A whole file in this encoding for a stream's chunk bodies, compressed at `level`.
pub fn encode_file(bodies: &[Vec<u8>], level: i32) -> Result<Vec<u8>, String> {
    file_of(&encode_columns(bodies)?, level)
}

/// Like [`encode_file`], for chunks from [`column_chunk`].
pub fn encode_chunks_file(chunks: &[SensorChunk], level: i32) -> Result<Vec<u8>, String> {
    file_of(&encode_chunk_columns(chunks)?, level)
}

fn file_of(columns: &[u8], level: i32) -> Result<Vec<u8>, String> {
    let mut out = header();
    let mut encoder =
        zstd::stream::write::Encoder::new(&mut out, level).map_err(|e| e.to_string())?;
    encoder.include_checksum(true).map_err(|e| e.to_string())?;
    io::Write::write_all(&mut encoder, columns).map_err(|e| e.to_string())?;
    encoder.finish().map_err(|e| e.to_string())?;
    Ok(out)
}

/// The stream content (length-delimited chunks, as the live format decodes to) of a whole file in this encoding.
pub fn decode_file(bytes: &[u8]) -> io::Result<Vec<u8>> {
    let mut content = Vec::new();
    decode_file_with(bytes, |body| {
        put_varint(&mut content, body.len() as u64);
        content.extend_from_slice(body);
    })?;
    Ok(content)
}

/// Like [`decode_file`], handing each chunk's body to `each` in order instead of collecting the content.
pub fn decode_file_with(bytes: &[u8], mut each: impl FnMut(&[u8])) -> io::Result<()> {
    let header = header();
    if !bytes.starts_with(&header) {
        return Err(invalid("not a sensor columns file"));
    }
    let frame = &bytes[header.len()..];
    let size = zstd::zstd_safe::find_frame_compressed_size(frame)
        .map_err(|_| invalid("incomplete zstd frame"))?;
    if size != frame.len() {
        return Err(invalid("bytes after the columns frame"));
    }
    let mut columns = Vec::new();
    zstd::stream::read::Decoder::with_buffer(frame)?
        .single_frame()
        .read_to_end(&mut columns)?;
    let chunks = decode_columns(&columns)?;
    drop(columns);
    for c in chunks {
        each(&c.encode_to_vec());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_orders_like_total_cmp_and_inverts() {
        let bits = [
            0u32,
            0x8000_0000,
            1,
            0x7f80_0001,
            0xffc0_1234,
            0x3f80_0000,
            0xbf80_0000,
            u32::MAX,
        ];
        for &a in &bits {
            assert_eq!(unkey(key(a)), a);
            for &b in &bits {
                assert_eq!(
                    key(a).cmp(&key(b)),
                    f32::from_bits(a).total_cmp(&f32::from_bits(b))
                );
            }
        }
    }
}
