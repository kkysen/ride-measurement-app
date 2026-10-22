"""Which actual train the rider was on: each ride's stop records matched against what the MTA's own trains reported
that day (subwaydata.nyc's daily archive, built from the GTFS-realtime feeds: every trip's actual arrival and
departure per stop, and the train's ID). Unlike the schedule, it tells apart lines that share stops (2 and 3,
B and C) by when each train actually stopped, and gives the trip the rider's own `transit` stream will record live.

Inputs: `stops.csv` and `fixes.csv` in `target/analysis/stops2-<ride>` (stop records, sensors only, with walking off
and on), the MTA's station list (`crates/fusion/data/stations.csv`) and GTFS stops (`data/mta/gtfs/stops.txt`),
and the archive unpacked under `target/` (`ARCHIVE`, default the newest `target/dl-subwaydata-*/x`), downloaded with
`curl -L https://subwaydata.nyc/data/subwaydatanyc_YYYY-MM-DD_csv.tar.xz` per day.
Each ride is split into legs where the rider walked off or on; within a leg, every stop of every trip is a
candidate when the trip was at a stop of that station within `SLACK_S` of the rider's stop (its arrival before
the rider's departure and its departure after the rider's stop, both with slack). The trip matching most stops
wins, then the smaller time error. Same-named stations (several 96 Sts, 7 Avs) are narrowed to the one nearest
Android's own fix at the time (`target/analysis/trainline/net-<ride>.csv`). Labels (taps and tags) aren't used.

2026-10-22, 8 subway rides, 17 legs (archive for 2026-10-20 and 21, 1.4 MB each):
- Clear: the 7 express to Flushing on 2026-10-20 is train `07 1536+ 34H/MST` (trip `093650_7X..N`) in both of
  that afternoon's sessions (`151449`, 3/3 stops, and `154951`, 6/8); the 2 south from 96 St at 6:27 pm on
  2026-10-21 (`182135`) is `02 1740  241/FLA`, 5/6 within 11 s, not a 3; the C from 110 St (`135509`) is
  `1C 1348+ 168/EUC`, 5/8 (the 86 St hold makes its mean error 101 s), not a B; the F at 7:29 pm (`184134`)
  `1F 1903 179/STL`, 5/8.
- Unsure (flagged): legs of 2 or 3 stops, and lines sharing stops and minutes (the D or the 1 from 59 St).
- Wrong or merged: legs still include a platform wait's other trains, and a leg without a walking off between
  two trains (`215922`: the 3 from Chambers St, then the 1 to 110 St) matches its longer part only.
The recorded `transit` stream (from app 0.1.7 on) gives the same trips live, with the train ID."""

import csv
import math
import os
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
ANALYSIS = ROOT / "target/analysis"
NY = ZoneInfo("America/New_York")
RIDES = ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922"]
SLACK_S = 45.0
NEAR_M = 250.0
NET_S = 90.0
PICK_M = 1_000.0


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def meters(a, b):
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def archive():
    if "ARCHIVE" in os.environ:
        return Path(os.environ["ARCHIVE"])
    return max((ROOT / "target").glob("dl-subwaydata-*/x"))


def load_archive(path):
    """{parent stop_id: [(arrival or departure s, departure or arrival s, trip_uid)]}, {trip_uid: trip row}."""
    at = defaultdict(list)
    trips = {}
    for f in sorted(path.glob("*_trips.csv")):
        trips |= {r["trip_uid"]: r for r in read(f)}
    for f in sorted(path.glob("*_stop_times.csv")):
        for r in read(f):
            a = r["arrival_time"] or r["departure_time"]
            d = r["departure_time"] or r["arrival_time"]
            if a:
                at[r["stop_id"][:-1]].append((int(a), int(d), r["trip_uid"]))
    return at, trips


def station_stops():
    """{station name: [(position, GTFS parent stop_ids within NEAR_M of it)]}, one entry per MTA station of that
    name (there are several 96 Sts, 7 Avs, and so on)."""
    parents = [r for r in read(ROOT / "data/mta/gtfs/stops.txt") if r["location_type"] == "1"]
    out = defaultdict(list)
    for s in read(ROOT / "crates/fusion/data/stations.csv"):
        p = (float(s["lat"]), float(s["lon"]))
        ids = {g["stop_id"] for g in parents if meters(p, (float(g["stop_lat"]), float(g["stop_lon"]))) <= NEAR_M}
        out[s["station"]].append((p, ids))
    return out


def candidates(name, t, net, names):
    """The GTFS stops a stop record named `name` at `t` can be: of the stations so named, those within PICK_M of
    Android's own fix nearest in time (within NET_S), or all of them without one."""
    entries = names.get(name, [])
    near = [f for f in net if abs(f[0] - t) <= NET_S]
    if near:
        fix = min(near, key=lambda f: abs(f[0] - t))[1:]
        close = [ids for p, ids in entries if meters(p, fix) <= PICK_M]
        if close:
            return set().union(*close)
    return set().union(*(ids for _, ids in entries)) if entries else set()


def legs(ride):
    """The ride's stops in the subway as legs: [[(name, stopped s, departed s or None)]], split at walking off/on,
    and Android's own fixes [(s, lat, lon)] (`target/analysis/trainline/net-<ride>.csv`, if exported)."""
    d = ANALYSIS / f"stops2-{ride}"
    fx = read(d / "fixes.csv")[0]
    off = int(fx["time_ms"]) * 1_000_000 - int(fx["t_ns"])
    unix = lambda ns: (int(ns) + off) / 1e9 if ns else None  # noqa: E731
    net_path = ANALYSIS / "trainline" / f"net-{ride}.csv"
    net = [(unix(r["t_ns"]), float(r["lat"]), float(r["lon"])) for r in read(net_path)] if net_path.exists() else []
    out, leg = [], []
    for s in read(d / "stops.csv"):
        if s["in_subway"] != "true" or not s["station"]:
            continue
        if s["boarded_ns"] and leg:
            out.append(leg)
            leg = []
        leg.append((s["station"], unix(s["stopped_ns"]), unix(s["departed_ns"])))
        if s["alighted_ns"]:
            out.append(leg)
            leg = []
    if leg:
        out.append(leg)
    return out, net


def match(leg, at, names, net):
    """[(matched stops, mean |error| s, trip_uid)], best first."""
    hits = defaultdict(list)
    for name, stopped, departed in leg:
        seen = set()
        for sid in candidates(name, stopped, net, names):
            for a, d, uid in at.get(sid, ()):
                if uid in seen:
                    continue
                if a <= (departed or stopped) + SLACK_S and d >= stopped - SLACK_S:
                    seen.add(uid)
                    hits[uid].append(abs(a - stopped))
    ranked = [(len(e), sum(e) / len(e), uid) for uid, e in hits.items()]
    ranked.sort(key=lambda x: (-x[0], x[1]))
    return ranked


def clock(t):
    return datetime.fromtimestamp(t, NY).strftime("%-I:%M:%S %p").lower()


def main(rides):
    at, trips = load_archive(archive())
    names = station_stops()
    for ride in rides:
        print(f"== {ride}")
        ride_legs, net = legs(ride)
        for leg in ride_legs:
            if len(leg) < 2:
                continue
            ranked = match(leg, at, names, net)
            first, last = leg[0], leg[-1]
            span = f"{clock(first[1])} {first[0]} to {clock(last[1])} {last[0]}, {len(leg)} stops"
            if not ranked:
                print(f"  {span}: no trip")
                continue
            n, err, uid = ranked[0]
            t = trips[uid]
            # The best trip of any other route, to show how clearly the line is decided.
            other = next((r for r in ranked[1:] if trips[r[2]]["route_id"] != t["route_id"]), None)
            vs = f"; next other line {trips[other[2]]['route_id']} {other[0]}/{len(leg)}" if other else ""
            if n < 2 or (other and other[0] == n):
                vs += " (UNSURE)"
            print(
                f"  {span}: {t['route_id']} trip {t['trip_id']} train '{t['vehicle_id']}', "
                f"{n}/{len(leg)} stops, mean error {err:.0f} s{vs}"
            )


if __name__ == "__main__":
    main(sys.argv[1:] or RIDES)
