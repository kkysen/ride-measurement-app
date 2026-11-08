"""The subway's track above ground, for telling a fix on an elevated train (`crates/fusion/src/subway.rs`):
from the MTA's GTFS (`data/mta/gtfs`, stop orders and shapes) and station list (`data/mta/stations.csv`, from
data.ny.gov, with each station's structure). A stretch of a route's shape counts when the stations at both ends
are above ground (elevated, viaduct, embankment, at grade, or open cut); underground track runs under streets,
where a car or bus would look like a train. Also every OpenStreetMap subway way tagged as a bridge
(`OSM`, from Overpass: `way["railway"="subway"]` over the city with `out tags geom`, 2026-11-08), for bridges and
viaducts between underground stations (the Manhattan and Williamsburg Bridges, the 1's 125 St viaduct),
which the station rule misses. Only ways tagged as bridges: about 160 km of untagged OSM subway track
wasn't on the station rule's open track (yards, PATH partly underground in Jersey City, and 7 km at NYCT
portals, partly in the tunnel mouth; `data/work/track/check_untagged.txt`), where a bus could look like a train.
Points every 10 m, deduplicated to 5 m, written as little-endian
i32 pairs (latitude, longitude, in 1e-5 degrees) to `crates/fusion/data/open_track.bin`."""

import csv
import json
import math
import struct
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GTFS = ROOT / "data/mta/gtfs"
OSM = ROOT / "data/downloads/osm-subway-20261108c/subway_ways.json"
OUT = ROOT / "crates/fusion/data/open_track.bin"
STEP_M = 10.0


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def meters(a, b):
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def add_line(points, p, q):
    """Points every `STEP_M` from p to q (not q), at 5e-5 degrees, and the length (m)."""
    d = meters(p, q)
    n = max(1, int(d // STEP_M))
    for k in range(n):
        la = p[0] + (q[0] - p[0]) * k / n
        lo = p[1] + (q[1] - p[1]) * k / n
        points.add((round(la * 2e4) * 5, round(lo * 2e4) * 5))
    return d


def main():
    structure = {s["GTFS Stop ID"]: s["Structure"] for s in read(ROOT / "data/mta/stations.csv")}
    stops = {s["stop_id"]: (float(s["stop_lat"]), float(s["stop_lon"])) for s in read(GTFS / "stops.txt")}
    shapes = defaultdict(list)
    for r in read(GTFS / "shapes.txt"):
        shapes[r["shape_id"]].append((int(r["shape_pt_sequence"]), float(r["shape_pt_lat"]), float(r["shape_pt_lon"])))
    trip_of = {}
    for t in read(GTFS / "trips.txt"):
        if t["shape_id"] and t["shape_id"] not in trip_of:
            trip_of[t["shape_id"]] = t["trip_id"]
    wanted = set(trip_of.values())
    seq = defaultdict(list)
    with open(GTFS / "stop_times.txt") as f:
        for r in csv.DictReader(f):
            if r["trip_id"] in wanted:
                seq[r["trip_id"]].append((int(r["stop_sequence"]), r["stop_id"][:-1]))
    points = set()
    open_m = 0.0
    for shape_id, pts in shapes.items():
        if shape_id not in trip_of:
            continue
        pts = [(la, lo) for _, la, lo in sorted(pts)]
        order = [s for _, s in sorted(seq[trip_of[shape_id]])]
        # Each stop's nearest shape point, in order along the shape.
        idx, at = [], 0
        for s in order:
            if s not in stops:
                continue
            best = min(range(at, len(pts)), key=lambda i: meters(pts[i], stops[s]))
            idx.append((best, s))
            at = best
        for (i, a), (j, b) in zip(idx, idx[1:]):
            if structure.get(a, "Subway") == "Subway" or structure.get(b, "Subway") == "Subway":
                continue
            for p, q in zip(pts[i:j], pts[i + 1 : j + 1]):
                open_m += add_line(points, p, q)
    osm_m = 0.0
    for way in json.loads(OSM.read_text())["elements"]:
        if way["tags"].get("bridge", "no") == "no" or way["tags"].get("tunnel", "no") != "no":
            continue
        g = [(n["lat"], n["lon"]) for n in way["geometry"]]
        for p, q in zip(g, g[1:]):
            osm_m += add_line(points, p, q)
    OUT.write_bytes(b"".join(struct.pack("<ii", la, lo) for la, lo in sorted(points)))
    print(f"{len(points)} points, {open_m / 1000:.0f} km of shapes above ground, {osm_m / 1000:.0f} km of OSM bridges, {OUT.stat().st_size} bytes")


if __name__ == "__main__":
    main()
