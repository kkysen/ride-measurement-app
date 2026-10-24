//! The subway cars' own Bluetooth beacons: iBeacon-shaped manufacturer data, but under company ID `0x02fd`,
//! not Apple's, with the UUID `e2c56db5-dffb-48d2-b060-d0f5a710xxxx`, one per car (the last two bytes name
//! the car), on the 7's R188s, B Division cars, and the L's alike: the same header, the same 100 byte and 1 m power
//! (`c5`), and 1 g at rest on `20261020-184134` (7, F) and `20261023-193658` (L). The L's CBTC predates Piper's,
//! so they aren't Piper's.
//! Their "major" and "minor" fields carry the car's own accelerometer, 64 counts per g, about twice a second:
//! major's two bytes and minor's first are x, y, z (signed); minor's second is 100 and the last byte the 1 m power.
//! (`python/experiments/train_beacons.py`; on `184134` the x axis follows GNSS's acceleration, correlation 0.81.)
//!
//! The rider's car is the steady beacon heard loudest: steady, so a train passing the other way isn't it.

use std::collections::HashMap;

const PREFIX: [u8; 14] = [
    0xe2, 0xc5, 0x6d, 0xb5, 0xdf, 0xfb, 0x48, 0xd2, 0xb0, 0x60, 0xd0, 0xf5, 0xa7, 0x10,
];
/// m/s^2 per count: 1 g is 64.
pub const COUNT_MPS2: f64 = 9.80665 / 64.0;
/// A car counts as steady once heard this many times within [`STEADY_S`].
const STEADY_READINGS: usize = 10;
const STEADY_S: f64 = 30.0;
/// The signal strength's low-pass (s), and how much louder (dB) another steady car must be to become the rider's.
const RSSI_TAU_S: f64 = 10.0;
const SWITCH_DB: f64 = 5.0;

/// One car's reading: its ID and its accelerometer (m/s^2, in the beacon's frame, gravity included).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CarReading {
    pub car: u16,
    pub rssi: i32,
    pub accel: [f64; 3],
}

/// The car beacon in an advertisement, if it's one.
pub fn decode(data: &[u8], rssi: i32) -> Option<CarReading> {
    let i = data.windows(PREFIX.len()).position(|w| w == PREFIX)? + PREFIX.len();
    let b = data.get(i..i + 5)?;
    let signed = |x: u8| f64::from(x as i8) * COUNT_MPS2;
    Some(CarReading {
        car: u16::from_be_bytes([b[0], b[1]]),
        rssi,
        accel: [signed(b[2]), signed(b[3]), signed(b[4])],
    })
}

#[derive(Clone, Debug, Default)]
struct Heard {
    times: std::collections::VecDeque<i64>,
    rssi: f64,
    last_ns: i64,
}

/// Which car the rider is in, from the readings heard.
#[derive(Clone, Debug, Default)]
pub struct CarTracker {
    heard: HashMap<u16, Heard>,
    current: Option<u16>,
}

impl CarTracker {
    /// Takes a reading; returns whether it's from the rider's car.
    pub fn on_reading(&mut self, t_ns: i64, r: &CarReading) -> bool {
        let h = self.heard.entry(r.car).or_default();
        let dt = if h.last_ns == 0 {
            f64::INFINITY
        } else {
            (t_ns - h.last_ns) as f64 * 1e-9
        };
        let k = 1.0 - (-dt / RSSI_TAU_S).exp();
        h.rssi += k * (f64::from(r.rssi) - h.rssi);
        h.last_ns = t_ns;
        h.times.push_back(t_ns);
        while h
            .times
            .front()
            .is_some_and(|&t| (t_ns - t) as f64 * 1e-9 > STEADY_S)
        {
            h.times.pop_front();
        }
        let steady = |h: &Heard| {
            h.times.len() >= STEADY_READINGS && (t_ns - h.last_ns) as f64 * 1e-9 <= STEADY_S
        };
        let current = self.current.and_then(|c| {
            self.heard
                .get(&c)
                .filter(|h| steady(h))
                .map(|h| (c, h.rssi))
        });
        let best = self
            .heard
            .iter()
            .filter(|(_, h)| steady(h))
            .max_by(|a, b| a.1.rssi.total_cmp(&b.1.rssi))
            .map(|(&c, h)| (c, h.rssi));
        self.current = match (current, best) {
            (Some((c, rc)), Some((b, rb))) if b != c && rb < rc + SWITCH_DB => Some(c),
            (_, best) => best.map(|(c, _)| c),
        };
        self.current == Some(r.car)
    }

    /// Whether `car` is heard steadily as of `t_ns`: a car of the rider's train, not one passing.
    pub fn steady(&self, t_ns: i64, car: u16) -> bool {
        self.heard.get(&car).is_some_and(|h| {
            h.times.len() >= STEADY_READINGS && (t_ns - h.last_ns) as f64 * 1e-9 <= STEADY_S
        })
    }

    /// The rider's car, if a steady one is heard.
    pub fn current(&self) -> Option<u16> {
        self.current
    }

    /// The rider's car as of `t_ns`: only if it was heard within [`STEADY_S`], so it goes once they get off.
    pub fn current_at(&self, t_ns: i64) -> Option<u16> {
        self.current.filter(|c| {
            self.heard
                .get(c)
                .is_some_and(|h| (t_ns - h.last_ns) as f64 * 1e-9 <= STEADY_S)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ad(car: u16, x: i8, y: i8, z: i8) -> Vec<u8> {
        let mut v = vec![0x02, 0x01, 0x06, 0x1a, 0xff, 0x4c, 0x00, 0x02, 0x15];
        v.extend(PREFIX);
        v.extend(car.to_be_bytes());
        v.extend([x as u8, y as u8, z as u8, 100, (-59i8) as u8]);
        v
    }

    #[test]
    fn decodes_the_cars_accelerometer() {
        let r = decode(&ad(0xa199, -1, 0, -64), -70).unwrap();
        assert_eq!(r.car, 0xa199);
        assert!((r.accel[2] + 9.80665).abs() < 1e-9);
        assert!((r.accel[0] + COUNT_MPS2).abs() < 1e-9);
        assert!(decode(&[1, 2, 3], -70).is_none());
    }

    #[test]
    fn the_loudest_steady_car_is_the_riders() {
        let mut t = CarTracker::default();
        let s = 1_000_000_000;
        // A passing train's car, loud but brief, never becomes the rider's.
        t.on_reading(0, &decode(&ad(0x9888, 0, 0, 64), -50).unwrap());
        for i in 1..30 {
            t.on_reading(i * s, &decode(&ad(0xa199, 0, 0, -64), -60).unwrap());
            t.on_reading(i * s + 1, &decode(&ad(0xa138, 0, 0, -64), -75).unwrap());
        }
        assert_eq!(t.current(), Some(0xa199));
    }
}
