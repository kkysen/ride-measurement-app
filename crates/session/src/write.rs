use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

use prost::Message;

use crate::{SESSION_INFO_FILE, STREAM_EXTENSION, SessionInfo};

/// Level used while recording; cheap, since post-ride recompression squeezes further.
pub const LIVE_ZSTD_LEVEL: i32 = 3;

pub fn write_session_info(dir: &Path, info: &SessionInfo) -> anyhow::Result<()> {
    fs::create_dir_all(dir)?;
    let path = dir.join(SESSION_INFO_FILE);
    let mut file = File::create(path)?;
    file.write_all(&info.encode_length_delimited_to_vec())?;
    file.sync_all()?;
    Ok(())
}

/// Appends chunks to `<dir>/<name>.zst`, one independent zstd frame per chunk, synced after each.
pub struct StreamWriter {
    path: PathBuf,
    file: File,
}

impl StreamWriter {
    pub fn create(dir: &Path, name: &str) -> anyhow::Result<Self> {
        let path = dir.join(format!("{name}.{STREAM_EXTENSION}"));
        let file = OpenOptions::new()
            .create_new(true)
            .append(true)
            .open(&path)?;
        Ok(Self { path, file })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn append(&mut self, chunk: &impl Message) -> anyhow::Result<()> {
        let frame = zstd::encode_all(
            chunk.encode_length_delimited_to_vec().as_slice(),
            LIVE_ZSTD_LEVEL,
        )?;
        self.file.write_all(&frame)?;
        self.file.sync_data()?;
        Ok(())
    }
}
