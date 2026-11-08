"""The IRT Capacity Study's recorded peak speeds per stretch between stations.
For each of Figures F.2-1 to F.2-28 (`charts/f-000.png` to `f-027.png`, in order), `irt_digitize.py` finds where the
simulated line comes down to 0 (stops) and the highest event recorder speed in each column. The charts' x is
distance along the track, so the stations come from the MTA's GTFS (`data/mta/gtfs`): of the route's stopping
patterns in the figure's direction, from its first station to its last, with each stop's distance along its shape
scaled so the end stations fall on the end stops, the one with the most stations whose every station falls on a
stop (within `TOL_PX`); stops with no station (at a signal) are passed over.
Writes `data/work/irt/irt_peaks.csv` (route, direction, from and to parent stop IDs and names, recorded and simulated
peak, mph) and, the highest recorded per route and stretch, `crates/fusion/data/irt_recorded_peaks.csv`.
The charts: `pdfimages -f 295 -l 322 -png` of the study (archive.org, `irt-capacity-study-final-reportt-redacted`)."""
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import irt_digitize as digitize  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
CHARTS = ROOT / "data/downloads/speed-capacity-docs-20261108/charts"
GTFS = ROOT / "data/mta/gtfs"
OUT = ROOT / "data/work/irt/irt_peaks.csv"
TOL_PX = 20

# The figures' titles (F.2-1 to F.2-28): route, N or S, first and last station as the study names them.
FIGURES = [
    ("1", "N", "South Ferry", "96 Street"),
    ("1", "N", "96 Street", "Van Cortlandt Park - 242 Street"),
    ("1", "S", "Van Cortlandt Park - 242 Street", "96 Street"),
    ("1", "S", "96 Street", "South Ferry"),
    ("2", "N", "Flatbush Avenue - Brooklyn College", "Nevins Street"),
    ("2", "N", "Nevins Street", "14 Street"),
    ("2", "N", "14 Street", "125 Street"),
    ("2", "N", "125 Street", "East 180 Street"),
    ("2", "N", "East 180 Street", "Wakefield - 241 Street"),
    ("2", "S", "Wakefield - 241 Street", "East 180 Street"),
    ("2", "S", "East 180 Street", "125 Street"),
    ("2", "S", "125 Street", "14 Street"),
    ("2", "S", "14 Street", "Nevins Street"),
    ("2", "S", "Nevins Street", "Flatbush Avenue - Brooklyn College"),
    ("3", "N", "New Lots Avenue", "Crown Heights - Utica Avenue"),
    ("3", "S", "Crown Heights - Utica Avenue", "New Lots Avenue"),
    ("4", "N", "Crown Heights - Utica Avenue", "138 Street - Grand Concourse"),
    ("4", "N", "138 Street - Grand Concourse", "Burnside Avenue"),
    ("4", "N", "Burnside Avenue", "Woodlawn"),
    ("4", "S", "Woodlawn", "Burnside Avenue"),
    ("4", "S", "Burnside Avenue", "138 Street - Grand Concourse"),
    ("4", "S", "138 Street - Grand Concourse", "Crown Heights - Utica Avenue"),
    ("5", "N", "138 Street - Grand Concourse", "Eastchester - Dyre Avenue"),
    ("5", "S", "Eastchester - Dyre Avenue", "138 Street - Grand Concourse"),
    ("6", "N", "Brooklyn Bridge - City Hall", "125 Street"),
    ("6", "N", "125 Street", "Pelham Bay Park"),
    ("6", "S", "Pelham Bay Park", "125 Street"),
    ("6", "S", "125 Street", "Brooklyn Bridge - City Hall"),
]


def norm(name):
    n = name.lower().replace("–", "-")
    for a, b in [("street", "st"), ("avenue", "av"), ("east ", "e "), ("heights", "hts"), ("crown hts", "crown hts")]:
        n = n.replace(a, b)
    return re.sub(r"[^a-z0-9]", "", n)


def meters(a, b):
    import math
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def patterns(route, stops, shapes):
    """The route's distinct stopping patterns: (direction letter, [parent stop IDs], [m along the shape])."""
    trips = {}
    with open(GTFS / "trips.txt") as f:
        for t in csv.DictReader(f):
            if t["route_id"] == route:
                trips[t["trip_id"]] = t["shape_id"]
    seqs = defaultdict(list)
    with open(GTFS / "stop_times.txt") as f:
        for r in csv.DictReader(f):
            if r["trip_id"] in trips:
                seqs[r["trip_id"]].append((int(r["stop_sequence"]), r["stop_id"]))
    seen = {}
    for trip, sq in seqs.items():
        sq.sort()
        ids = tuple(x[1] for x in sq)
        key = (ids[0][-1], tuple(i[:-1] for i in ids))
        if key in seen or trips[trip] not in shapes:
            continue
        pts, along = shapes[trips[trip]]
        at, out = 0, []
        for i in key[1]:
            k = min(range(at, len(pts)), key=lambda j: meters(pts[j], stops[i]))
            out.append(along[k])
            at = k
        seen[key] = out
    return [(d, ids, al) for (d, ids), al in seen.items()]


def load_shapes():
    raw = defaultdict(list)
    with open(GTFS / "shapes.txt") as f:
        for r in csv.DictReader(f):
            raw[r["shape_id"]].append((int(r["shape_pt_sequence"]), float(r["shape_pt_lat"]), float(r["shape_pt_lon"])))
    out = {}
    for sid, pts in raw.items():
        pts = [(la, lo) for _, la, lo in sorted(pts)]
        along = [0.0]
        for p, q in zip(pts, pts[1:]):
            along.append(along[-1] + meters(p, q))
        out[sid] = (pts, along)
    return out


def main():
    names, stops = {}, {}
    with open(GTFS / "stops.txt") as f:
        for s in csv.DictReader(f):
            names[s["stop_id"]] = s["stop_name"]
            stops[s["stop_id"]] = (float(s["stop_lat"]), float(s["stop_lon"]))
    shapes = load_shapes()
    cache = {}
    rows = []
    for k, (route, direction, first, last) in enumerate(FIGURES):
        path = CHARTS / f"f-{k:03d}.png"
        dips, rec, sim = digitize.read(str(path))
        if route not in cache:
            cache[route] = patterns(route, stops, shapes)
        best = None
        for d, ids, along in cache[route]:
            if d != direction or len(dips) < 2:
                continue
            nids = [norm(names[i]) for i in ids]
            hit = lambda n, want: n.startswith(norm(want)) or norm(want).startswith(n)
            a = next((i for i, n in enumerate(nids) if hit(n, first)), None)
            b = next((i for i, n in enumerate(nids) if a is not None and i > a and hit(n, last)), None)
            if a is None or b is None:
                continue
            span = along[b] - along[a]
            if span <= 0:
                continue
            # The end stations on any two stops (an extra stop at an edge or a signal shouldn't throw the scale).
            for i0 in range(len(dips)):
                for i1 in range(i0 + 1, len(dips)):
                    xs = [dips[i0] + (along[i] - along[a]) / span * (dips[i1] - dips[i0]) for i in range(a, b + 1)]
                    matched = []
                    for x in xs:
                        near = min(dips, key=lambda d_: abs(d_ - x))
                        # One station per stop, in order.
                        if abs(near - x) > TOL_PX or (matched and near <= matched[-1]):
                            break
                        matched.append(near)
                    else:
                        if best is None or len(matched) > len(best[1]):
                            best = (ids[a : b + 1], matched)
        if best is None:
            print(f"F.2-{k + 1} {route}{direction} {first} to {last}: {len(dips)} stops, no GTFS pattern fits",
                  file=sys.stderr)
            continue
        ids, xs = best
        for (x, y), (xa, xb) in zip(zip(ids, ids[1:]), zip(xs, xs[1:])):
            rows.append((route, direction, x, y, names[x], names[y], digitize.peak(rec, xa, xb), digitize.peak(sim, xa, xb)))
        print(f"F.2-{k + 1} {route}{direction} {first} to {last}: {len(dips)} stops, {len(ids)} stations", file=sys.stderr)
    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["route", "direction", "from_stop", "to_stop", "from_name", "to_name", "recorded_mph", "simulated_mph"])
        w.writerows(rows)
    print(f"{len(rows)} stretches to {OUT}", file=sys.stderr)
    best = {}
    for route, _, a, b, _, _, rec, _ in rows:
        if rec is not None:
            best[(route, a, b)] = max(best.get((route, a, b), 0.0), rec)
    with open(ROOT / "crates/fusion/data/irt_recorded_peaks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["route", "from_stop", "to_stop", "recorded_mph"])
        for (route, a, b), v in sorted(best.items()):
            w.writerow([route, a, b, v])


if __name__ == "__main__":
    main()
