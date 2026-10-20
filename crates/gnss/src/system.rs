//! The GNSS constellations Android reports, and how each one counts the time in `received_sv_time_nanos`.

pub const NANOS_PER_WEEK: i64 = 604_800_000_000_000;
pub const NANOS_PER_DAY: i64 = 86_400_000_000_000;
/// BeiDou time (BDT) is behind GPS time by this, constantly (BDS-SIS-ICD section 5.1).
pub const BDT_BEHIND_GPS_NS: i64 = 14_000_000_000;
/// GLONASS time is UTC(SU) plus 3 hours (GLONASS ICD section 3.3.3).
const GLONASS_AHEAD_OF_UTC_NS: i64 = 3 * 3_600_000_000_000;
/// GPS minus UTC since 2017, for clocks that don't report leap seconds.
pub const DEFAULT_LEAP_SECONDS: i32 = 18;

/// `GnssMeasurement.STATE_*` flags.
pub const STATE_CODE_LOCK: i32 = 1 << 0;
pub const STATE_TOW_DECODED: i32 = 1 << 3;
/// The transmit time may be off by whole milliseconds (300 km each).
pub const STATE_MSEC_AMBIGUOUS: i32 = 1 << 4;
pub const STATE_GLO_TOD_DECODED: i32 = 1 << 7;
pub const STATE_GAL_E1BC_CODE_LOCK: i32 = 1 << 10;
pub const STATE_TOW_KNOWN: i32 = 1 << 14;
pub const STATE_GLO_TOD_KNOWN: i32 = 1 << 15;

/// A constellation, as `GnssStatus.CONSTELLATION_*` numbers them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum System {
    Gps,
    Glonass,
    Qzss,
    Beidou,
    Galileo,
}

impl System {
    pub fn from_android(constellation_type: i32) -> Option<Self> {
        match constellation_type {
            1 => Some(Self::Gps),
            3 => Some(Self::Glonass),
            4 => Some(Self::Qzss),
            5 => Some(Self::Beidou),
            6 => Some(Self::Galileo),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Gps => "GPS",
            Self::Glonass => "GLONASS",
            Self::Qzss => "QZSS",
            Self::Beidou => "BeiDou",
            Self::Galileo => "Galileo",
        }
    }

    /// Whether `state` says the code is locked and `received_sv_time_nanos` is the full time of week
    /// (of day for GLONASS), not just within a code or bit period.
    pub fn time_known(self, state: i32) -> bool {
        let code_lock = match self {
            // E1B/C reports its own code lock flag (and may not set STATE_CODE_LOCK).
            Self::Galileo => STATE_CODE_LOCK | STATE_GAL_E1BC_CODE_LOCK,
            _ => STATE_CODE_LOCK,
        };
        let time = match self {
            Self::Glonass => STATE_GLO_TOD_DECODED | STATE_GLO_TOD_KNOWN,
            _ => STATE_TOW_DECODED | STATE_TOW_KNOWN,
        };
        state & code_lock != 0 && state & time != 0 && state & STATE_MSEC_AMBIGUOUS == 0
    }

    /// The period `received_sv_time_nanos` counts within (ns).
    pub fn period_ns(self) -> i64 {
        match self {
            Self::Glonass => NANOS_PER_DAY,
            _ => NANOS_PER_WEEK,
        }
    }

    /// GPS time (ns since the GPS epoch) as this system counts `received_sv_time_nanos`: ns into its week
    /// (Galileo's and QZSS's weeks start with GPS's), or into its day for GLONASS.
    pub fn time_of_period_ns(self, gps_ns: i64, leap_seconds: i32) -> i64 {
        let t = match self {
            Self::Gps | Self::Qzss | Self::Galileo => gps_ns,
            Self::Beidou => gps_ns - BDT_BEHIND_GPS_NS,
            Self::Glonass => {
                gps_ns - i64::from(leap_seconds) * 1_000_000_000 + GLONASS_AHEAD_OF_UTC_NS
            }
        };
        t.rem_euclid(self.period_ns())
    }
}

/// One satellite: its constellation and Android's `svid` in it (PRN, or slot number for GLONASS).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SatId {
    pub system: System,
    pub svid: u8,
}

impl std::fmt::Display for SatId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let letter = match self.system {
            System::Gps => 'G',
            System::Glonass => 'R',
            System::Qzss => 'J',
            System::Beidou => 'C',
            System::Galileo => 'E',
        };
        write!(f, "{letter}{:02}", self.svid)
    }
}
