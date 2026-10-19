# GNSS test data

Excerpts of real orbit files from RTKLIB's test data (`test/data` at tag `rtklib_2.4.3`),
originally from the IGS via CDDIS, for 2010-07-01 (GPS week 1590, day 4):

- `brdc1820.10n`: the broadcast ephemeris (RINEX 2 navigation file) header and each satellite's 00:00 record.
- `igs15904-first-2h.sp3`: IGS final precise orbits (SP3) for the same satellites, 00:00 to 02:00 every 15 min,
  epoch and position lines only.

Broadcast orbits agree with precise ones to a few meters, which checks the orbit math independently.
