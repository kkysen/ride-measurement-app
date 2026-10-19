use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail};
use prost::Message;

use crate::delimited::read_message;
use crate::{
    AudioChunk, FORMAT_VERSION, RecordChunk, SESSION_INFO_FILE, STREAM_EXTENSION, SensorChunk,
    SessionInfo,
};

pub struct Session {
    pub dir: PathBuf,
    pub info: SessionInfo,
}

impl Session {
    pub fn open(dir: impl Into<PathBuf>) -> anyhow::Result<Self> {
        let dir = dir.into();
        let path = dir.join(SESSION_INFO_FILE);
        let mut file = BufReader::new(
            File::open(&path).with_context(|| format!("opening {}", path.display()))?,
        );
        let Some(bytes) = read_message(&mut file)? else {
            bail!("{} is empty", path.display());
        };
        let info = SessionInfo::decode(bytes.as_slice())?;
        if info.format_version != FORMAT_VERSION {
            bail!("unsupported format version {}", info.format_version);
        }
        Ok(Self { dir, info })
    }

    pub fn stream_path(&self, name: &str) -> PathBuf {
        self.dir.join(format!("{name}.{STREAM_EXTENSION}"))
    }

    /// Reads a sensor stream.
    pub fn read_stream(&self, name: &str) -> anyhow::Result<StreamContents> {
        read_stream_file(&self.stream_path(name))
    }

    /// Reads an audio stream.
    pub fn read_audio(&self, name: &str) -> anyhow::Result<StreamContents<AudioChunk>> {
        read_stream_file(&self.stream_path(name))
    }

    /// Reads a record stream.
    pub fn read_records(&self, name: &str) -> anyhow::Result<StreamContents<RecordChunk>> {
        read_stream_file(&self.stream_path(name))
    }
}

pub struct StreamContents<C = SensorChunk> {
    pub chunks: Vec<C>,
    /// The file ends in an incomplete frame or message, as after a crash.
    /// Every complete chunk before the cut, including ones inside the incomplete frame, is still in `chunks`.
    pub truncated: bool,
    /// Byte ranges of the file that couldn't be decoded; decoding resumed at the next frame after each.
    pub damage: Vec<Damage>,
}

impl<C> StreamContents<C> {
    /// No truncation and no damage.
    pub fn is_clean(&self) -> bool {
        !self.truncated && self.damage.is_empty()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Damage {
    pub offset: usize,
    pub len: usize,
    pub reason: String,
}

const ZSTD_MAGIC: [u8; 4] = [0x28, 0xb5, 0x2f, 0xfd];

/// Reads a stream file frame by frame, so damage in one frame doesn't lose the others.
///
/// Live files have one chunk per frame; recompressed files have every chunk in one frame;
/// both decode the same way, as a sequence of length-delimited chunks per frame.
pub fn read_stream_file<C: Message + Default>(path: &Path) -> anyhow::Result<StreamContents<C>> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    Ok(decode_stream_bytes(&bytes))
}

pub fn decode_stream_bytes<C: Message + Default>(bytes: &[u8]) -> StreamContents<C> {
    let mut contents = StreamContents {
        chunks: Vec::new(),
        truncated: false,
        damage: Vec::new(),
    };
    let mut offset = 0;
    while offset < bytes.len() {
        let rest = &bytes[offset..];
        match zstd::zstd_safe::find_frame_compressed_size(rest) {
            Ok(size) if size > 0 && size <= rest.len() => {
                if let Err(reason) = decode_frame(&rest[..size], &mut contents.chunks, false) {
                    contents.damage.push(Damage {
                        offset,
                        len: size,
                        reason,
                    });
                }
                offset += size;
            }
            _ => match next_frame_start(bytes, offset + 1) {
                // Garbage or a broken frame in the middle: skip to the next frame.
                Some(next) => {
                    contents.damage.push(Damage {
                        offset,
                        len: next - offset,
                        reason: "not a complete zstd frame".into(),
                    });
                    offset = next;
                }
                // The last frame is cut off, as after a crash: salvage the complete chunks in it.
                None => {
                    contents.truncated = true;
                    if rest.starts_with(&ZSTD_MAGIC) {
                        let _ = decode_frame(rest, &mut contents.chunks, true);
                    } else {
                        contents.damage.push(Damage {
                            offset,
                            len: rest.len(),
                            reason: "trailing bytes that aren't a zstd frame".into(),
                        });
                    }
                    break;
                }
            },
        }
    }
    contents
}

/// Decodes every length-delimited chunk in one frame. With `partial`, stops quietly at the cut.
fn decode_frame<C: Message + Default>(
    frame: &[u8],
    out: &mut Vec<C>,
    partial: bool,
) -> Result<(), String> {
    let mut decoder = zstd::stream::read::Decoder::with_buffer(frame)
        .map_err(|e| e.to_string())?
        .single_frame();
    // Chunks are only kept once the whole frame decodes (or, for a cut-off frame, up to the cut),
    // so a corrupt frame contributes nothing rather than possibly-wrong chunks.
    let mut decoded = Vec::new();
    loop {
        match read_message(&mut decoder) {
            Ok(Some(bytes)) => match C::decode(bytes.as_slice()) {
                Ok(c) => decoded.push(c),
                Err(e) => return Err(format!("chunk doesn't decode: {e}")),
            },
            Ok(None) => break,
            Err(e) if partial => {
                let _ = e;
                break;
            }
            Err(e) => return Err(e.to_string()),
        }
    }
    out.extend(decoded);
    Ok(())
}

fn next_frame_start(bytes: &[u8], from: usize) -> Option<usize> {
    bytes
        .get(from..)?
        .windows(ZSTD_MAGIC.len())
        .position(|w| w == ZSTD_MAGIC)
        .map(|p| from + p)
}
