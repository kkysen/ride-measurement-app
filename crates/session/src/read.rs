use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail};
use prost::Message;

use crate::delimited::read_message;
use crate::{FORMAT_VERSION, SESSION_INFO_FILE, STREAM_EXTENSION, SensorChunk, SessionInfo};

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

    pub fn read_stream(&self, name: &str) -> anyhow::Result<StreamContents> {
        read_stream_file(&self.stream_path(name))
    }
}

pub struct StreamContents {
    pub chunks: Vec<SensorChunk>,
    /// The file ends in an incomplete frame or message, as after a crash; everything before it is in `chunks`.
    pub truncated: bool,
}

pub fn read_stream_file(path: &Path) -> anyhow::Result<StreamContents> {
    let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let mut decoder = zstd::stream::read::Decoder::new(file)?;
    let mut chunks = Vec::new();
    loop {
        match read_message(&mut decoder) {
            Ok(Some(bytes)) => chunks.push(SensorChunk::decode(bytes.as_slice())?),
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
