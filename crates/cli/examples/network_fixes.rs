// Scratch: every non-GPS fix (network, fused) as CSV, to see where Android's own location puts the phone underground.
use ride_session::proto::record::Payload;
use ride_session::read::Session;
fn main() -> anyhow::Result<()> {
    let s = Session::open(std::env::args().nth(1).unwrap())?;
    println!("t_ns,provider,lat,lon,h_acc");
    for r in s
        .read_records("location")?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
    {
        if let Some(Payload::Location(l)) = r.payload
            && l.provider != "gps"
        {
            println!(
                "{},{},{},{},{}",
                l.elapsed_realtime_nanos,
                l.provider,
                l.latitude_degrees,
                l.longitude_degrees,
                l.horizontal_accuracy_meters.unwrap_or(f32::NAN)
            );
        }
    }
    Ok(())
}
