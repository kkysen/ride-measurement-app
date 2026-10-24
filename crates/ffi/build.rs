//! Hashes the Rust that decides what a replay gives (the core crates' sources and data, and the record format),
//! so cached replays are kept across app builds that don't change it (see `replay_session`).

use std::hash::{Hash, Hasher};
use std::path::{Path, PathBuf};

fn files(dir: &Path, out: &mut Vec<PathBuf>) {
    if dir.is_file() {
        out.push(dir.to_path_buf());
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            files(&p, out);
        } else {
            out.push(p);
        }
    }
}

fn main() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut all = Vec::new();
    for dir in [
        "crates/fusion/src",
        "crates/fusion/data",
        "crates/session/src",
        "crates/session/proto",
        "crates/gnss/src",
        "proto",
        "Cargo.lock",
    ] {
        let dir = root.join(dir);
        println!("cargo:rerun-if-changed={}", dir.display());
        files(&dir, &mut all);
    }
    all.sort();
    // `DefaultHasher` may change with the toolchain: then rides are only replayed again.
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for p in &all {
        p.strip_prefix(&root).unwrap_or(p).hash(&mut h);
        std::fs::read(p).unwrap_or_default().hash(&mut h);
    }
    println!("cargo:rustc-env=RIDE_CORE_HASH={:016x}", h.finish());
}
