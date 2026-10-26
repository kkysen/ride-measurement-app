//! The rider's Aranet4 air quality monitor, from its Bluetooth advertisements: CO2, temperature, humidity,
//! and pressure. It advertises its latest reading only with "Smart Home integrations" on (flag bit 5),
//! as manufacturer data under SAF Tehnika's company ID `0x0702`: an 8 byte header (flags, firmware version),
//! then little-endian CO2 (ppm), temperature (1/20 °C), pressure (1/10 hPa), humidity (%), battery (%),
//! status (1 green, 2 yellow, 3 red), the measurement interval (s), the reading's age (s), and a counter.
//! Checked against `20261021-135509`'s advertisements (544 ppm, 22.2 °C, 1016.6 hPa, 37 %, 60 s).
//!
//! The device updates its reading once per interval, so most advertisements repeat one reading;
//! [`Readings`] keeps each once, at the time it was measured (received minus its age).

/// SAF Tehnika's Bluetooth company ID.
const COMPANY_ID: [u8; 2] = [0x02, 0x07];
const INTEGRATIONS_FLAG: u8 = 1 << 5;
const HEADER_LEN: usize = 8;
const READING_LEN: usize = 14;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
    pub co2_ppm: u16,
    pub temperature_c: f64,
    pub pressure_hpa: f64,
    pub humidity_pct: u8,
    pub battery_pct: u8,
    /// 1 green, 2 yellow, 3 red, by the device's own CO2 thresholds.
    pub status: u8,
    pub interval_s: u16,
    /// How long before the advertisement the reading was measured.
    pub age_s: u16,
}

/// The Aranet4 reading in an advertisement (the whole advertising data), if it's one with integrations on.
pub fn decode(data: &[u8]) -> Option<Reading> {
    let m = manufacturer_data(data)?;
    if m.len() < HEADER_LEN + READING_LEN || m[0] & INTEGRATIONS_FLAG == 0 {
        return None;
    }
    let r = &m[HEADER_LEN..];
    let u16_at = |i: usize| u16::from_le_bytes([r[i], r[i + 1]]);
    Some(Reading {
        co2_ppm: u16_at(0),
        temperature_c: f64::from(u16_at(2)) / 20.0,
        pressure_hpa: f64::from(u16_at(4)) / 10.0,
        humidity_pct: r[6],
        battery_pct: r[7],
        status: r[8],
        interval_s: u16_at(9),
        age_s: u16_at(11),
    })
}

/// The manufacturer-specific data structure under SAF's company ID, without the ID.
fn manufacturer_data(mut data: &[u8]) -> Option<&[u8]> {
    while let [len, rest @ ..] = data {
        let len = usize::from(*len);
        if len == 0 || rest.len() < len {
            return None;
        }
        let (field, next) = rest.split_at(len);
        if let [0xff, id0, id1, m @ ..] = field
            && [*id0, *id1] == COMPANY_ID
        {
            return Some(m);
        }
        data = next;
    }
    None
}

/// Each distinct reading once, at its measurement time.
#[derive(Clone, Debug, Default)]
pub struct Readings {
    last_measured_ns: Option<i64>,
}

impl Readings {
    /// Takes an advertisement received at `t_ns`; returns its reading with its measurement time
    /// if it's a new one.
    pub fn on_ble(&mut self, t_ns: i64, data: &[u8]) -> Option<(i64, Reading)> {
        let r = decode(data)?;
        let measured = t_ns - i64::from(r.age_s) * 1_000_000_000;
        // Ages are whole seconds, so one reading's measurement time wanders by up to a second.
        if self
            .last_measured_ns
            .is_some_and(|last| measured - last < i64::from(r.interval_s.max(2)) * 500_000_000)
        {
            return None;
        }
        self.last_measured_ns = Some(measured);
        Some((measured, r))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// From `20261021-135509`, with the name ("Aranet4 23293") after the manufacturer data.
    const ADV: &str = "02010619ff020722130401000e0f012002bc01b6272554013c001d00b00302e0fc0e094172616e657434203233323933";

    #[test]
    fn decodes_a_recorded_advertisement() {
        let r = decode(&hex(ADV)).unwrap();
        assert_eq!(r.co2_ppm, 544);
        assert!((r.temperature_c - 22.2).abs() < 1e-9);
        assert!((r.pressure_hpa - 1016.6).abs() < 1e-9);
        assert_eq!((r.humidity_pct, r.battery_pct, r.status), (37, 84, 1));
        assert_eq!((r.interval_s, r.age_s), (60, 29));
    }

    #[test]
    fn ignores_other_and_truncated_advertisements() {
        assert_eq!(decode(&hex("02010619ff4c00")), None);
        assert_eq!(decode(&hex(&ADV[..40])), None);
        let mut off = hex(ADV);
        off[7] &= !INTEGRATIONS_FLAG;
        assert_eq!(decode(&off), None);
    }

    #[test]
    fn keeps_each_reading_once() {
        let adv = hex(ADV);
        let mut rs = Readings::default();
        let s = 1_000_000_000;
        assert_eq!(rs.on_ble(100 * s, &adv).unwrap().0, 71 * s);
        // The same reading, re-advertised later with an age that drifted by a second.
        let mut later = adv.clone();
        later[26] = 29 + 10;
        assert!(rs.on_ble(111 * s, &later).is_none());
        later[26] = 0;
        assert_eq!(rs.on_ble(131 * s, &later).unwrap().0, 131 * s);
    }
}
