// Scratch: each session's start and end (unix ms) from its data.
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    for dir in std::env::args().skip(1) {
        let s = Session::open(&dir)?;
        let end = s.end_unix_ms()?;
        println!(
            "{dir} {} {:?} {:?} min",
            s.info.start_unix_ms,
            end,
            end.map(|e| (e - s.info.start_unix_ms) / 60_000)
        );
    }
    Ok(())
}
