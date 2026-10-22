//! The door cycles `ride_fusion::doors` finds in a session's audio, fed chunk by chunk as the app records it,
//! one `open_ns,close_ns` per line, to check against `python/experiments/stops.py`.
//! `cargo run --release -p ride-fusion --example door_cycles -- <session dir>`

use ride_fusion::doors::DoorDetector;
use ride_session::read::Session;

fn main() -> anyhow::Result<()> {
    let session = Session::open(std::path::Path::new(
        &std::env::args().nth(1).expect("session dir"),
    ))?;
    let rate = session
        .info
        .streams
        .iter()
        .find_map(|s| s.audio.as_ref().map(|a| a.sample_rate_hz))
        .expect("an audio stream");
    let mut detector = DoorDetector::new(rate as u32);
    println!("open_ns,close_ns");
    for chunk in session.read_audio("audio")?.chunks {
        let samples: Vec<i16> = chunk
            .pcm_s16le
            .as_chunks::<2>()
            .0
            .iter()
            .map(|&b| i16::from_le_bytes(b))
            .collect();
        let timestamps: Vec<(i64, i64)> = chunk
            .timestamps
            .iter()
            .map(|t| (t.frame_position, t.boottime_ns))
            .collect();
        for c in detector.on_audio(chunk.first_frame, &samples, &timestamps) {
            println!("{},{}", c.open_ns, c.close_ns);
        }
    }
    Ok(())
}
