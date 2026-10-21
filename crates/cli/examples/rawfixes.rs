// Scratch: every GPS-provider fix between two times (s from the first), as fusion gets them.
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use ride_session::track::GPS_PROVIDER;
fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let s = Session::open(args.next().unwrap())?;
    let from: f64 = args.next().unwrap().parse()?;
    let to: f64 = args.next().unwrap().parse()?;
    let mut t0 = None;
    for r in s
        .read_records("location")?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
    {
        let Some(Payload::Location(l)) = r.payload else {
            continue;
        };
        if l.provider != GPS_PROVIDER || l.is_mock {
            continue;
        }
        let t =
            (l.elapsed_realtime_nanos - *t0.get_or_insert(l.elapsed_realtime_nanos)) as f64 * 1e-9;
        if (from..=to).contains(&t) {
            println!(
                "{t:7.1} s  {:.5},{:.5}  ±{:?} m  {:?} m/s",
                l.latitude_degrees,
                l.longitude_degrees,
                l.horizontal_accuracy_meters,
                l.speed_meters_per_second
            );
        }
    }
    Ok(())
}
