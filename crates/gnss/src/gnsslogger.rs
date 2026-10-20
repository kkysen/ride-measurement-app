//! Logs from Google's GnssLogger app (as in Google's GPS Measurement Tools and Smartphone Decimeter Challenge):
//! raw measurements, the phone's fixes, and navigation messages, as this project's records.
//!
//! Columns are found by the names in the log's `# Raw,...` and `# Fix,...` header lines, which differ by version.
//! Times go on one clock: newer logs stamp measurements with UTC (`utcTimeMillis`), and then everything uses
//! UTC in ns as its "elapsed realtime"; older ones use elapsed realtime, and their fixes, stamped only with UTC,
//! are left out.

use std::collections::HashMap;

use ride_session::proto::{
    GnssClock, GnssMeasurement, GnssMeasurementsEvent, GnssNavigationMessage, Location,
};

#[derive(Default)]
pub struct Log {
    /// One event per epoch, in the log's order.
    pub epochs: Vec<GnssMeasurementsEvent>,
    pub fixes: Vec<Location>,
    /// With the time of the epoch before each (navigation lines carry no time of their own).
    pub navigation: Vec<(i64, GnssNavigationMessage)>,
    /// Whether the times are UTC (ns since 1970) rather than the phone's elapsed realtime.
    pub utc_times: bool,
}

struct Columns(HashMap<String, usize>);

impl Columns {
    fn new(header: &str) -> Self {
        Self(
            header
                .split(',')
                .enumerate()
                .map(|(i, name)| (name.trim().to_string(), i))
                .collect(),
        )
    }

    fn get<'a>(&self, f: &[&'a str], name: &str) -> Option<&'a str> {
        let v = f.get(*self.0.get(name)?)?.trim();
        (!v.is_empty()).then_some(v)
    }

    fn f64(&self, f: &[&str], name: &str) -> Option<f64> {
        self.get(f, name)?.parse().ok()
    }

    fn i64(&self, f: &[&str], name: &str) -> Option<i64> {
        self.get(f, name)?.parse().ok()
    }
}

/// Reads a log; lines that don't parse are skipped.
pub fn read(text: &str) -> Log {
    let mut log = Log::default();
    let mut raw: Option<Columns> = None;
    let mut fix: Option<Columns> = None;
    let mut last_time = 0;
    for line in text.lines() {
        if let Some(header) = line.strip_prefix("# ") {
            if header.starts_with("Raw,") {
                let c = Columns::new(header);
                log.utc_times = c.0.contains_key("utcTimeMillis");
                raw = Some(c);
            } else if header.starts_with("Fix,") {
                fix = Some(Columns::new(header));
            }
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        match f[0] {
            "Raw" => {
                if let Some((t, clock, m)) = raw.as_ref().and_then(|c| raw_line(c, &f)) {
                    last_time = t;
                    match log.epochs.last_mut() {
                        Some(e) if e.clock.as_ref().unwrap().time_nanos == clock.time_nanos => {
                            e.measurements.push(m);
                        }
                        _ => log.epochs.push(GnssMeasurementsEvent {
                            clock: Some(clock),
                            measurements: vec![m],
                            ..Default::default()
                        }),
                    }
                }
            }
            "Fix" if log.utc_times => {
                if let Some(l) = fix.as_ref().and_then(|c| fix_line(c, &f)) {
                    log.fixes.push(l);
                }
            }
            "Nav" => {
                if let Some(m) = nav_line(&f) {
                    log.navigation.push((last_time, m));
                }
            }
            _ => {}
        }
    }
    log
}

fn raw_line(c: &Columns, f: &[&str]) -> Option<(i64, GnssClock, GnssMeasurement)> {
    let t = c
        .i64(f, "utcTimeMillis")
        .or_else(|| c.i64(f, "ElapsedRealtimeMillis"))?
        * 1_000_000;
    let clock = GnssClock {
        time_nanos: c.i64(f, "TimeNanos")?,
        leap_second: c.i64(f, "LeapSecond").map(|s| s as i32),
        time_uncertainty_nanos: c.f64(f, "TimeUncertaintyNanos"),
        full_bias_nanos: c.i64(f, "FullBiasNanos"),
        bias_nanos: c.f64(f, "BiasNanos"),
        bias_uncertainty_nanos: c.f64(f, "BiasUncertaintyNanos"),
        drift_nanos_per_second: c.f64(f, "DriftNanosPerSecond"),
        drift_uncertainty_nanos_per_second: c.f64(f, "DriftUncertaintyNanosPerSecond"),
        hardware_clock_discontinuity_count: c.i64(f, "HardwareClockDiscontinuityCount").unwrap_or(0)
            as i32,
        elapsed_realtime_nanos: Some(t),
        ..Default::default()
    };
    let m = GnssMeasurement {
        svid: c.i64(f, "Svid")? as i32,
        constellation_type: c.i64(f, "ConstellationType")? as i32,
        time_offset_nanos: c.f64(f, "TimeOffsetNanos").unwrap_or(0.0),
        state: c.i64(f, "State")? as i32,
        received_sv_time_nanos: c.i64(f, "ReceivedSvTimeNanos")?,
        received_sv_time_uncertainty_nanos: c.i64(f, "ReceivedSvTimeUncertaintyNanos")?,
        cn0_dbhz: c.f64(f, "Cn0DbHz")?,
        pseudorange_rate_meters_per_second: c.f64(f, "PseudorangeRateMetersPerSecond")?,
        pseudorange_rate_uncertainty_meters_per_second: c
            .f64(f, "PseudorangeRateUncertaintyMetersPerSecond")?,
        accumulated_delta_range_state: c.i64(f, "AccumulatedDeltaRangeState").unwrap_or(0) as i32,
        accumulated_delta_range_meters: c.f64(f, "AccumulatedDeltaRangeMeters").unwrap_or(0.0),
        accumulated_delta_range_uncertainty_meters: c
            .f64(f, "AccumulatedDeltaRangeUncertaintyMeters")
            .unwrap_or(0.0),
        carrier_frequency_hz: c.f64(f, "CarrierFrequencyHz").map(|v| v as f32),
        multipath_indicator: c.i64(f, "MultipathIndicator").unwrap_or(0) as i32,
        snr_in_db: c.f64(f, "SnrInDb"),
        automatic_gain_control_level_db: c.f64(f, "AgcDb"),
        baseband_cn0_dbhz: c.f64(f, "BasebandCn0DbHz"),
        code_type: c.get(f, "CodeType").map(str::to_string),
        full_inter_signal_bias_nanos: c.f64(f, "FullInterSignalBiasNanos"),
        full_inter_signal_bias_uncertainty_nanos: c.f64(f, "FullInterSignalBiasUncertaintyNanos"),
        satellite_inter_signal_bias_nanos: c.f64(f, "SatelliteInterSignalBiasNanos"),
        satellite_inter_signal_bias_uncertainty_nanos: c
            .f64(f, "SatelliteInterSignalBiasUncertaintyNanos"),
    };
    Some((t, clock, m))
}

fn fix_line(c: &Columns, f: &[&str]) -> Option<Location> {
    let time_ms = c.i64(f, "UnixTimeMillis")?;
    let float = |name: &str| c.f64(f, name).map(|v| v as f32);
    Some(Location {
        provider: c.get(f, "Provider")?.to_lowercase(),
        time_ms,
        elapsed_realtime_nanos: time_ms * 1_000_000,
        latitude_degrees: c.f64(f, "LatitudeDegrees")?,
        longitude_degrees: c.f64(f, "LongitudeDegrees")?,
        altitude_meters: c.f64(f, "AltitudeMeters"),
        speed_meters_per_second: float("SpeedMps"),
        bearing_degrees: float("BearingDegrees"),
        horizontal_accuracy_meters: float("AccuracyMeters"),
        vertical_accuracy_meters: float("VerticalAccuracyMeters"),
        speed_accuracy_meters_per_second: float("SpeedAccuracyMps"),
        bearing_accuracy_degrees: float("BearingAccuracyDegrees"),
        is_mock: c.i64(f, "MockLocation").unwrap_or(0) != 0,
        ..Default::default()
    })
}

/// `Nav,Svid,Type,Status,MessageId,Sub-messageId,Data(Bytes)`: the data as signed bytes.
fn nav_line(f: &[&str]) -> Option<GnssNavigationMessage> {
    let int = |k: usize| f.get(k)?.trim().parse::<i32>().ok();
    Some(GnssNavigationMessage {
        svid: int(1)?,
        r#type: int(2)?,
        status: int(3)?,
        message_id: int(4)?,
        submessage_id: int(5)?,
        data: f
            .get(6..)?
            .iter()
            .map(|b| b.trim().parse::<i8>().map(|b| b as u8))
            .collect::<Result<_, _>>()
            .ok()?,
    })
}
