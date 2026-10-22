"""Trip paths: each leg (a "Boarded" tap to the next "Alighted") as stations on a route, from where GNSS last had
the rider before boarding and first had them after alighting, and the registered cell's runs between (a station
has its own cell, `station_cell.py`), which also names each cell. Uses the MTA's schedule data
(`data/mta/gtfs`, from http://web.mta.info/developers/data/nyct/subway/google_transit.zip) for stop orders.

Prints each leg's best routes, with the stations it passed and the cells seen there, and the cell-to-station table
with any cell that two legs put at different stations."""

import csv
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402
from subway import on_train_intervals  # noqa: E402

GTFS = Path(__file__).resolve().parents[2] / "data/mta/gtfs"
RIDES = ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922"]
# A station within this of the rider's fix (m) can be where they boarded or alighted.
NEAR_M = 500.0
# Fixes this good (m) and this close in time (s) to the tap place the rider.
FIX_ACC_M = 50.0
FIX_WITHIN_S = 900.0
# A registered cell held this briefly (s) is a blip at a handover, not a station.
MIN_CELL_S = 8.0


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def meters(a, b):
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def gtfs():
    """Stations {parent id: (name, lat, lon)} and patterns [(route, headsign, [parent ids in order])]."""
    stations = {}
    for s in read(GTFS / "stops.txt"):
        if s["location_type"] == "1":
            stations[s["stop_id"]] = (s["stop_name"], float(s["stop_lat"]), float(s["stop_lon"]))
    trips = {t["trip_id"]: (t["route_id"], t["trip_headsign"]) for t in read(GTFS / "trips.txt")}
    seqs = defaultdict(list)
    with open(GTFS / "stop_times.txt") as f:
        for r in csv.DictReader(f):
            seqs[r["trip_id"]].append((int(r["stop_sequence"]), r["stop_id"][:-1]))
    patterns = Counter()
    for trip, seq in seqs.items():
        route, head = trips[trip]
        patterns[(route, head, tuple(s for _, s in sorted(seq)))] += 1
    return stations, [(r, h, list(p)) for (r, h, p), n in patterns.items() if n >= 5]


def fixes(ride):
    out = []
    for f in read(ANALYSIS / ride / "fixes.csv"):
        if f["h_acc"] and float(f["h_acc"]) <= FIX_ACC_M:
            out.append((int(f["t_ns"]), float(f["lat"]), float(f["lon"])))
    return out


def place(fx, t_ns, before):
    """The last good fix before `t_ns` (or the first after), within `FIX_WITHIN_S`."""
    side = [f for f in fx if (f[0] <= t_ns if before else f[0] >= t_ns)]
    if not side:
        return None
    f = side[-1] if before else side[0]
    return f if abs(f[0] - t_ns) <= FIX_WITHIN_S * 1e9 else None


def near(stations, f):
    """Stations within `NEAR_M` of the fix, with their distances (m)."""
    out = {}
    for s, (_, la, lo) in stations.items():
        d = meters((f[1], f[2]), (la, lo))
        if d <= NEAR_M:
            out[s] = d
    return out


def cell_runs(ride, a, b):
    """Registered cell runs from `a` to `b` ns: [(start, end, cell)], blips under `MIN_CELL_S` dropped."""
    rows = [(int(c["t_ns"]), c["cell_id"]) for c in read(ANALYSIS / ride / "cell.csv") if c["registered"] == "true"]
    runs = []
    for t, c in rows:
        if not a <= t <= b:
            continue
        if runs and runs[-1][2] == c:
            runs[-1][1] = t
        else:
            runs.append([t, t, c])
    runs = [r for r in runs if (r[1] - r[0]) * 1e-9 >= MIN_CELL_S]
    merged = []
    for r in runs:
        if merged and merged[-1][2] == r[2]:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    return merged


def stop_taps(ride, a, b):
    kinds = {"KIND_DOORS_OPEN", "KIND_TRAIN_STOPPED", "KIND_AT_STATION"}
    return [int(e["t_ns"]) for e in read(ANALYSIS / ride / "events.csv") if e["kind"] in kinds and a <= int(e["t_ns"]) <= b]


def stop_times(ride, a, b):
    """The rider's stop taps ("doors open", "train stopped"), one per stop (taps within 60 s are one)."""
    kinds = {"KIND_DOORS_OPEN", "KIND_TRAIN_STOPPED"}
    ts = sorted(int(e["t_ns"]) for e in read(ANALYSIS / ride / "events.csv") if e["kind"] in kinds and a <= int(e["t_ns"]) <= b)
    out = []
    for t in ts:
        if not out or t - out[-1] > 60e9:
            out.append(t)
    return out


def tracks(patterns, start, ends):
    """Stations the track passes from `start`, to each possible end (any later station if `ends` is empty):
    the most stations any route has between the two, so express runs still list the locals they pass."""
    out = {}
    for route, head, p in patterns:
        if start not in p:
            continue
        i = p.index(start)
        for k in range(i + 1, len(p)):
            if ends and p[k] not in ends:
                continue
            seg = p[i : k + 1]
            if len(seg) > len(out.get(p[k], ())):
                out[p[k]] = seg
    return out


def align(runs, track, table):
    """Cell runs onto the track's stations in order, each station taking 0 to 2 runs: (cost, station index per run).
    Known cells cost nothing at their station and a lot elsewhere; a station passed without its own run costs a
    little (an express can hold one cell across a local)."""
    n, m = len(runs), len(track)
    inf = float("inf")
    # best[i][j]: runs[:i] placed, the last on station j-1 or before (j stations used).
    best = [[inf] * (m + 1) for _ in range(n + 1)]
    back = {}
    best[0][0] = 0.0

    def cost(c, s):
        if not table[c]:
            return 0.5
        top, k = table[c].most_common(1)[0]
        return 0.0 if top == s else (0.3 if table[c][s] else 3.0)

    for j in range(m):
        for i in range(n + 1):
            if best[i][j] == inf:
                continue
            base = best[i][j]
            # Station j with no run.
            if base + 1.0 < best[i][j + 1]:
                best[i][j + 1] = base + 1.0
                back[(i, j + 1)] = (i, j, 0)
            # With one or two runs.
            for take in (1, 2):
                if i + take > n:
                    break
                c = base + sum(cost(runs[i + q][2], track[j]) for q in range(take)) + (0.7 if take == 2 else 0)
                if c < best[i + take][j + 1]:
                    best[i + take][j + 1] = c
                    back[(i + take, j + 1)] = (i, j, take)
    if best[n][m] == inf:
        return inf, None
    where = [None] * n
    i, j = n, m
    while (i, j) != (0, 0):
        pi, pj, take = back[(i, j)]
        for q in range(take):
            where[pi + q] = pj
        i, j = pi, pj
    return best[n][m], where


def legs():
    stations, patterns = gtfs()
    out = []
    for ride in RIDES:
        fx = fixes(ride)
        for a, b in on_train_intervals(ride):
            fa, fb = place(fx, a, True), place(fx, b, False)
            fresh = fa is not None and (a - fa[0]) * 1e-9 <= 120
            fresh_end = fb is not None and (fb[0] - b) * 1e-9 <= 300
            out.append(dict(ride=ride, a=a, b=b,
                            starts=near(stations, fa) if fresh else {},
                            stale_starts=near(stations, fa) if fa and not fresh else {},
                            ends=near(stations, fb) if fresh_end else {},
                            runs=cell_runs(ride, a, b), stops=stop_times(ride, a, b)))
    return stations, patterns, out


def solve(patterns, ls, rounds=6):
    """Each leg's track and alignment, re-solved with the cell table from every other leg until it settles."""
    table = defaultdict(Counter)
    chosen = {}
    for _ in range(rounds):
        new = defaultdict(Counter)
        prev = None
        for k, leg in enumerate(ls):
            starts = leg["starts"]
            if not starts and prev and prev[0] == leg["ride"]:
                starts = {prev[1]: 0.0}
            starts = starts or leg["stale_starts"]
            best = None
            for st, d0 in starts.items():
                for end, track in tracks(patterns, st, leg["ends"]).items():
                    c, where = align(leg["runs"], track, table)
                    if where is None:
                        continue
                    # Nearer stations to the fixes are likelier; a station is 1 per 200 m.
                    c += d0 / 200 + leg["ends"].get(end, 0.0) / 200
                    # Unknown ends: a longer track has more free stations to absorb runs; pay for each.
                    c += 0.0 if leg["ends"] else 0.2 * len(track)
                    # Every tapped stop, and the end, must be a stop of some route along the track.
                    stopped = {track[-1]} | {track[where[i]] for t in leg["stops"] if (i := run_at(leg, t)) is not None}
                    if not routes(patterns, track, stopped):
                        c += 3.0
                    if where is not None and (best is None or c < best[0]):
                        best = (c, track, where)
            if best is None:
                prev = None
                continue
            chosen[k] = best
            for run, j in zip(leg["runs"], best[2]):
                new[run[2]][best[1][j]] += 1
            prev = (leg["ride"], best[1][-1])
        table = new
    return table, chosen


def run_at(leg, t):
    for i, r in enumerate(leg["runs"]):
        if r[0] - 5e9 <= t <= r[1] + 5e9:
            return i
    return None


def routes(patterns, track, stopped):
    """Routes whose stops along this track include every station it stopped at, with the fewest stops."""
    out = []
    for route, head, p in patterns:
        if track[0] in p and track[-1] in p and p.index(track[0]) < p.index(track[-1]):
            seg = p[p.index(track[0]) : p.index(track[-1]) + 1]
            if all(s in seg for s in stopped):
                out.append((len(seg), route))
    best = min((n for n, _ in out), default=None)
    return sorted({r for n, r in out if n == best})


def report():
    stations, patterns, ls = legs()
    table, chosen = solve(patterns, ls)
    name = lambda s: stations[s][0]
    for k, leg in enumerate(ls):
        print(f"\n{leg['ride']} leg {k}, {(leg['b'] - leg['a']) * 1e-9 / 60:.0f} min, {len(leg['runs'])} cell runs, "
              f"{len(leg['stops'])} tapped stops")
        if k not in chosen:
            print("  not placed")
            continue
        cost, track, where = chosen[k]
        stopped = set()
        for t in leg["stops"]:
            i = run_at(leg, t)
            if i is not None:
                stopped.add(track[where[i]])
        # The rider got off at the end, so the train stopped there.
        stopped.add(track[-1])
        rs = routes(patterns, track, stopped)
        print(f"  {name(track[0])} to {name(track[-1])}, {len(track)} stations, cost {cost:.1f}; route {'/'.join(rs) or '?'}")
        for j, s in enumerate(track):
            cells = [leg["runs"][i][2] for i, w in enumerate(where) if w == j]
            print(f"    {name(s):32s} {' '.join(cells):22s} {'stop' if s in stopped else ''}")
    clashes = {c: n for c, n in table.items() if len(n) > 1}
    print(f"\n{len(table)} cells placed, {len(clashes)} at more than one station:")
    for c, n in clashes.items():
        print(f"  {c}: " + ", ".join(f"{name(s)} x{k}" for s, k in n.items()))


if __name__ == "__main__":
    report()
