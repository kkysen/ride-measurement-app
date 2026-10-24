"""The subway's track between stations, for joining stops along it underground (`crates/fusion/src/track.rs`):
from the MTA's GTFS (`data/mta/gtfs`, stop orders and shapes), as in `open_track.py`.
Each shape that some trip follows, with its stops in order, deduplicated by its stop order (either direction),
simplified to within 2 m (Douglas-Peucker), written to `crates/fusion/data/subway_track.bin`, little-endian:
per shape, its route (u8 length, ASCII), its points (u16 count, then i32 latitude and longitude in 1e-5 degrees),
and its stops (u16 count, then each parent stop ID (u8 length, ASCII) and its nearest point's index (u16))."""

import csv
import math
import struct
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GTFS = ROOT / "data/mta/gtfs"
OUT = ROOT / "crates/fusion/data/subway_track.bin"
TOLERANCE_M = 2.0


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def meters(a, b):
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def off_line_m(p, a, b):
    """Distance (m) from `p` to the segment `a`-`b`, flat."""
    k = math.cos(math.radians(a[0]))
    ax, ay = 0.0, 0.0
    bx, by = (b[1] - a[1]) * k, b[0] - a[0]
    px, py = (p[1] - a[1]) * k, p[0] - a[0]
    l2 = bx * bx + by * by
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, (px * bx + py * by) / l2))
    return math.hypot(px - t * bx, py - t * by) * math.radians(1) * 6_371_000


def simplify(pts, keep):
    """Douglas-Peucker, keeping the indices in `keep` (the stops' points); returns the kept indices."""
    out = {0, len(pts) - 1} | set(keep)
    stack = [(0, len(pts) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        inner = [k for k in sorted(out) if i < k < j]
        if inner:
            bounds = [i, *inner, j]
            stack.extend(zip(bounds, bounds[1:]))
            continue
        k, d = max(((k, off_line_m(pts[k], pts[i], pts[j])) for k in range(i + 1, j)), key=lambda x: x[1])
        if d > TOLERANCE_M:
            out.add(k)
            stack.extend([(i, k), (k, j)])
    return sorted(out)


def main():
    stops = {s["stop_id"]: (float(s["stop_lat"]), float(s["stop_lon"])) for s in read(GTFS / "stops.txt")}
    shapes = defaultdict(list)
    for r in read(GTFS / "shapes.txt"):
        shapes[r["shape_id"]].append((int(r["shape_pt_sequence"]), float(r["shape_pt_lat"]), float(r["shape_pt_lon"])))
    trips_of = defaultdict(list)
    route_of = {}
    for t in read(GTFS / "trips.txt"):
        if t["shape_id"]:
            trips_of[t["shape_id"]].append(t["trip_id"])
            route_of[t["shape_id"]] = t["route_id"]
    # Every trip's stop order, since trips sharing a shape can skip different stops; one trip per order.
    wanted = {t: s for s, ts in trips_of.items() for t in ts}
    seq = defaultdict(list)
    with open(GTFS / "stop_times.txt") as f:
        for r in csv.DictReader(f):
            if r["trip_id"] in wanted:
                seq[r["trip_id"]].append((int(r["stop_sequence"]), r["stop_id"][:-1]))
    seen = set()
    out = []
    total_m = 0.0
    for trip, shape_id in sorted(wanted.items()):
        order = tuple(s for _, s in sorted(seq[trip]) if s in stops)
        if len(order) < 2 or order in seen or order[::-1] in seen or shape_id not in shapes:
            continue
        seen.add(order)
        pts = [(la, lo) for _, la, lo in sorted(shapes[shape_id])]
        idx, at = [], 0
        for s in order:
            best = min(range(at, len(pts)), key=lambda i: meters(pts[i], stops[s]))
            idx.append(best)
            at = best
        kept = simplify(pts, idx)
        where = {k: n for n, k in enumerate(kept)}
        total_m += sum(meters(p, q) for p, q in zip(pts, pts[1:]))
        b = bytearray()
        route = route_of[shape_id].encode()
        b += struct.pack("<B", len(route)) + route
        b += struct.pack("<H", len(kept))
        for k in kept:
            b += struct.pack("<ii", round(pts[k][0] * 1e5), round(pts[k][1] * 1e5))
        b += struct.pack("<H", len(order))
        for s, k in zip(order, idx):
            e = s.encode()
            b += struct.pack("<B", len(e)) + e + struct.pack("<H", where[k])
        out.append(bytes(b))
    OUT.write_bytes(b"".join(out))
    print(f"{len(out)} stop orders, {total_m / 1000:.0f} km of shapes, {OUT.stat().st_size} bytes")


if __name__ == "__main__":
    main()
