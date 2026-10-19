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
    /// The file ends in an incomplete frame or message, as after a crash; everything before it is in `chunks`.
    pub truncated: bool,
}

pub fn read_stream_file<C: Message + Default>(path: &Path) -> anyhow::Result<StreamContents<C>> {
    let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let mut decoder = zstd::stream::read::Decoder::new(file)?;
    let mut chunks = Vec::new();
    loop {
        match read_message(&mut decoder) {
            Ok(Some(bytes)) => chunks.push(C::decode(bytes.as_slice())?),
            Ok(None) => {
                return Ok(StreamContents {
                    chunks,
                    truncated: false,
                });
            }
            Err(e) if is_truncation(&e) => {
                return Ok(StreamContents {
                    chunks,
                    truncated: true,
                });
            }
            Err(e) => return Err(e).with_context(|| format!("reading {}", path.display())),
        }
    }
}

fn is_truncation(e: &std::io::Error) -> bool {
    e.kind() == std::io::ErrorKind::UnexpectedEof
}
