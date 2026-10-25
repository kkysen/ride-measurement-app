"""The MTA's subway entrances and exits (data.ny.gov "MTA Subway Entrances and Exits: 2024", `i9wp-a4ja`),
for finding which entrance the rider used to go down into or come up from the subway.
Download into `data/mta/entrances/entrances.csv`:
`curl -L -o data/mta/entrances/entrances.csv 'https://data.ny.gov/api/views/i9wp-a4ja/rows.csv?accessType=DOWNLOAD'`.
Each row is one entrance of one station (`GTFS Stop ID`, `Complex ID`), with its type
(Stair, Elevator, Escalator, Easement, Station House, ...), whether entry and exit are allowed, and its location.
The dataset has no street or corner names, so `corner` describes an entrance by its bearing from its station's middle."""

import csv
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CSV = ROOT / "data/mta/entrances/entrances.csv"


@dataclass(frozen=True)
class Entrance:
    stop_name: str
    station: str  # constituent station name
    gtfs_stop_id: str
    complex_id: str
    routes: str
    kind: str  # `Entrance Type`
    entry: bool
    exit: bool
    lat: float
    lon: float


def meters(a, b):
    """Flat distance (m) between two (lat, lon) points."""
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def load(path=CSV):
    with open(path) as f:
        return [
            Entrance(
                stop_name=r["Stop Name"],
                station=r["Constituent Station Name"],
                gtfs_stop_id=r["GTFS Stop ID"],
                complex_id=r["Complex ID"],
                routes=r["Daytime Routes"],
                kind=r["Entrance Type"],
                entry=r["Entry Allowed"] == "YES",
                exit=r["Exit Allowed"] == "YES",
                lat=float(r["Entrance Latitude"]),
                lon=float(r["Entrance Longitude"]),
            )
            for r in csv.DictReader(f)
        ]


class Index:
    """Entrances bucketed on a ~100 m grid, for nearest lookups without scanning them all."""

    CELL = 0.001

    def __init__(self, entrances):
        self.entrances = entrances
        self.grid = defaultdict(list)
        for e in entrances:
            self.grid[self._cell(e.lat, e.lon)].append(e)
        by_stop = defaultdict(list)
        for e in entrances:
            by_stop[e.gtfs_stop_id].append(e)
        self.middle = {
            s: (sum(e.lat for e in es) / len(es), sum(e.lon for e in es) / len(es)) for s, es in by_stop.items()
        }

    def _cell(self, lat, lon):
        return (math.floor(lat / self.CELL), math.floor(lon / self.CELL))

    def near(self, lat, lon, radius_m=300.0):
        """Entrances within `radius_m` of the point, nearest first, as (meters, entrance)."""
        n = math.ceil(radius_m / 80) + 1
        ci, cj = self._cell(lat, lon)
        out = []
        for i in range(ci - n, ci + n + 1):
            for j in range(cj - n, cj + n + 1):
                for e in self.grid.get((i, j), ()):
                    d = meters((lat, lon), (e.lat, e.lon))
                    if d <= radius_m:
                        out.append((d, e))
        return sorted(out, key=lambda x: x[0])

    def corner(self, e):
        """The entrance's bearing from its station's middle, e.g. "NE, 60 m"."""
        m = self.middle[e.gtfs_stop_id]
        d = meters(m, (e.lat, e.lon))
        if d < 15:
            return "middle"
        dn = e.lat - m[0]
        de = (e.lon - m[1]) * math.cos(math.radians(m[0]))
        names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        return f"{names[round(math.degrees(math.atan2(de, dn)) / 45) % 8]}, {d:.0f} m"
