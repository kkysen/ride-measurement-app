// Scratch: the radio stream as CSVs in `out` (ble.csv, wifi.csv, cell.csv, service.csv, errors.csv), for Python.
use ride_session::proto::record::Payload;
use ride_session::read::Session;
use std::io::Write;
fn main() -> anyhow::Result<()> {
    let a: Vec<String> = std::env::args().collect();
    let s = Session::open(&a[1])?;
    let out = std::path::Path::new(&a[2]);
    std::fs::create_dir_all(out)?;
    let mut ble = std::fs::File::create(out.join("ble.csv"))?;
    writeln!(ble, "t_ns,address,rssi,filtered,data")?;
    let mut wifi = std::fs::File::create(out.join("wifi.csv"))?;
    writeln!(wifi, "t_ns,bssid,ssid,mhz,dbm")?;
    let mut cell = std::fs::File::create(out.join("cell.csv"))?;
    writeln!(cell, "t_ns,kind,registered,cell_id,pci,tac,arfcn,dbm")?;
    let mut svc = std::fs::File::create(out.join("service.csv"))?;
    writeln!(svc, "t_ns,state")?;
    let mut bat = std::fs::File::create(out.join("battery.csv"))?;
    writeln!(bat, "t_ns,temp_c")?;
    for r in s
        .read_records("radio")?
        .chunks
        .into_iter()
        .flat_map(|c| c.records)
    {
        let t = r.received_elapsed_realtime_ns;
        match r.payload {
            Some(Payload::BleScan(b)) => {
                for x in b.results {
                    let hex: String = x
                        .advertising_data
                        .iter()
                        .map(|b| format!("{b:02x}"))
                        .collect();
                    writeln!(
                        ble,
                        "{},{},{},{},{}",
                        x.timestamp_elapsed_realtime_ns, x.address, x.rssi, b.filtered, hex
                    )?;
                }
            }
            Some(Payload::WifiScan(w)) => {
                for x in w.results {
                    let ssid = x.ssid.replace([',', '\n', '"'], " ");
                    writeln!(
                        wifi,
                        "{},{},{},{},{}",
                        x.timestamp_us * 1000,
                        x.bssid,
                        ssid,
                        x.frequency_mhz,
                        x.level_dbm
                    )?;
                }
            }
            Some(Payload::CellInfo(c)) => {
                for x in c.cells {
                    let o = |v: Option<i64>| v.map(|v| v.to_string()).unwrap_or_default();
                    writeln!(
                        cell,
                        "{t},{},{},{},{},{},{},{}",
                        x.kind,
                        x.registered,
                        o(x.cell_id),
                        o(x.pci.map(i64::from)),
                        o(x.tac.map(i64::from)),
                        o(x.arfcn.map(i64::from)),
                        x.dbm
                    )?;
                }
            }
            Some(Payload::Battery(b)) => {
                writeln!(bat, "{t},{}", b.temperature_decidegrees_c as f64 / 10.0)?
            }
            Some(Payload::ServiceState(v)) => writeln!(svc, "{t},{}", v.state)?,
            Some(Payload::RecorderError(e)) => {
                eprintln!("{t} error {}: {} (x{})", e.source, e.message, e.count)
            }
            _ => {}
        }
    }
    Ok(())
}
