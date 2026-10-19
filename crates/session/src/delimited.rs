use std::io::{self, Read};

/// Reads one varint-length-prefixed message body.
/// Returns `Ok(None)` on a clean end of input before any length byte.
pub fn read_message(r: &mut impl Read) -> io::Result<Option<Vec<u8>>> {
    let mut len: u64 = 0;
    for i in 0..10 {
        let mut byte = [0u8];
        if r.read(&mut byte)? == 0 {
            if i == 0 {
                return Ok(None);
            }
            return Err(io::ErrorKind::UnexpectedEof.into());
        }
        len |= u64::from(byte[0] & 0x7f) << (7 * i);
        if byte[0] & 0x80 == 0 {
            let mut buf = vec![0; usize::try_from(len).map_err(io::Error::other)?];
            r.read_exact(&mut buf)?;
            return Ok(Some(buf));
        }
    }
    Err(io::Error::new(
        io::ErrorKind::InvalidData,
        "varint too long",
    ))
}
