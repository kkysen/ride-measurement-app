//! Speed colors for maps: the one definition both the app's live map and `ride map` use.
//!
//! A light-to-dark blue ramp in fixed bands, narrow at low speeds and wide at high ones,
//! so a color means the same speed on every ride and every map.
//! Each scale covers its kind of ride's top speed; a ride uses the first whose limit its top speed is within.

/// Sequential blue ramp, as ARGB: light is slow, dark is fast.
pub const RAMP_ARGB: [u32; 10] = [
    0xFF86B6EF, 0xFF6DA7EC, 0xFF5598E7, 0xFF3987E5, 0xFF2A78D6, 0xFF256ABF, 0xFF1C5CAB, 0xFF184F95,
    0xFF104281, 0xFF0D366B,
];

/// For a point with no fused speed.
pub const UNKNOWN_ARGB: u32 = 0xFF8A8984;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedScale {
    /// Subways and commuter rail, up to 200 km/h.
    Rail,
    /// High-speed rail, up to 400 km/h (the CR450).
    HighSpeedRail,
    /// Flights, up to Mach 1.
    Flight,
}

impl SpeedScale {
    pub const ALL: [SpeedScale; 3] = [
        SpeedScale::Rail,
        SpeedScale::HighSpeedRail,
        SpeedScale::Flight,
    ];

    /// For command lines and data files.
    pub fn id(self) -> &'static str {
        match self {
            SpeedScale::Rail => "rail",
            SpeedScale::HighSpeedRail => "high-speed-rail",
            SpeedScale::Flight => "flight",
        }
    }

    pub fn from_id(id: &str) -> Option<SpeedScale> {
        Self::ALL.into_iter().find(|s| s.id() == id)
    }

    /// For people.
    pub fn name(self) -> &'static str {
        match self {
            SpeedScale::Rail => "rail",
            SpeedScale::HighSpeedRail => "high-speed rail",
            SpeedScale::Flight => "flight",
        }
    }

    /// The fastest a ride on this scale goes; `None` for no limit.
    pub fn max_kmh(self) -> Option<f64> {
        match self {
            SpeedScale::Rail => Some(200.0),
            SpeedScale::HighSpeedRail => Some(400.0),
            SpeedScale::Flight => None,
        }
    }

    /// Lower edge of each band, km/h, one per [`RAMP_ARGB`] color; the last band is open-ended.
    pub fn bands_kmh(self) -> [u32; 10] {
        match self {
            SpeedScale::Rail => [0, 10, 20, 30, 40, 60, 80, 100, 130, 160],
            SpeedScale::HighSpeedRail => [0, 25, 50, 100, 150, 200, 250, 300, 340, 380],
            SpeedScale::Flight => [0, 50, 100, 200, 300, 400, 500, 650, 800, 950],
        }
    }

    /// The scale for a ride whose top speed so far is `top_speed_mps` (`None` if there's no speed yet).
    pub fn for_top_speed(top_speed_mps: Option<f64>) -> SpeedScale {
        let kmh = top_speed_mps.unwrap_or(0.0) * 3.6;
        Self::ALL
            .into_iter()
            .find(|s| s.max_kmh().is_none_or(|max| kmh <= max))
            .unwrap_or(SpeedScale::Flight)
    }

    /// The band (index into [`RAMP_ARGB`]) for a speed, or `None` without one.
    pub fn band(self, speed_mps: Option<f64>) -> Option<usize> {
        let kmh = speed_mps? * 3.6;
        Some(
            self.bands_kmh()
                .iter()
                .rposition(|&edge| kmh >= f64::from(edge))
                .unwrap_or(0),
        )
    }

    /// The color for a speed, or [`UNKNOWN_ARGB`] without one.
    pub fn color_argb(self, speed_mps: Option<f64>) -> u32 {
        self.band(speed_mps).map_or(UNKNOWN_ARGB, |b| RAMP_ARGB[b])
    }
}

/// The speed a map draws the path between two consecutive fixes with: their mean, if both have one.
pub fn edge_speed(a_mps: Option<f64>, b_mps: Option<f64>) -> Option<f64> {
    Some((a_mps? + b_mps?) / 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kmh(v: f64) -> Option<f64> {
        Some(v / 3.6)
    }

    #[test]
    fn fixed_bands() {
        let rail = SpeedScale::Rail;
        assert_eq!(rail.color_argb(Some(0.0)), RAMP_ARGB[0]);
        assert_eq!(rail.color_argb(kmh(9.9)), RAMP_ARGB[0]);
        assert_eq!(rail.color_argb(kmh(10.0)), RAMP_ARGB[1]);
        assert_eq!(rail.color_argb(kmh(120.0)), RAMP_ARGB[7]);
        assert_eq!(rail.color_argb(kmh(190.0)), RAMP_ARGB[9]);
        assert_eq!(
            SpeedScale::HighSpeedRail.color_argb(kmh(390.0)),
            RAMP_ARGB[9]
        );
        assert_eq!(SpeedScale::Flight.color_argb(kmh(1200.0)), RAMP_ARGB[9]);
        assert_eq!(rail.color_argb(None), UNKNOWN_ARGB);
    }

    #[test]
    fn scale_follows_the_top_speed() {
        assert_eq!(SpeedScale::for_top_speed(None), SpeedScale::Rail);
        assert_eq!(SpeedScale::for_top_speed(kmh(200.0)), SpeedScale::Rail);
        assert_eq!(
            SpeedScale::for_top_speed(kmh(201.0)),
            SpeedScale::HighSpeedRail
        );
        assert_eq!(
            SpeedScale::for_top_speed(kmh(400.0)),
            SpeedScale::HighSpeedRail
        );
        assert_eq!(SpeedScale::for_top_speed(kmh(401.0)), SpeedScale::Flight);
    }
}
