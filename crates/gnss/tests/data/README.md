# GNSS test data

Excerpts of real orbit files from RTKLIB's test data (`test/data` at tag `rtklib_2.4.3`),
originally from the IGS via CDDIS, for 2010-07-01 (GPS week 1590, day 4):

- `brdc1820.10n`: the broadcast ephemeris (RINEX 2 navigation file) header and each satellite's 00:00 record.
- `igs15904-first-2h.sp3`: IGS final precise orbits (SP3) for the same satellites, 00:00 to 02:00 every 15 min,
  epoch and position lines only.

Broadcast orbits agree with precise ones to a few meters, which checks the orbit math independently.

From Google's GPS Measurement Tools (`opensource/demoFiles` in `google/gps-measurement-tools`, Apache 2.0),
a real phone at rest on 2016-06-30 around 21:26 UTC:

- `gnsslogger_2016_06_30_21_26_07.txt`: the GnssLogger log `pseudoranges_log_2016_06_30_21_26_07.txt`,
  `Raw` and `Fix` lines only (GPS only, about 4 minutes).
- `hour1820-20h-22h.16n`: the matching hourly broadcast ephemeris `hour1820.16n`, 20:00 to 22:00 records only.

- `BRDM00DLR_S_20210980000-21h.rnx`: the IGS daily merged broadcast ephemeris for 2021-04-08 (RINEX 3,
  from BKG's mirror, `igs.bkg.bund.de/root_ftp/IGS/BRDC/2021/098/`), header and the 21:00 to 21:59 records
  of GPS, GLONASS, Galileo, BeiDou, and QZSS.
