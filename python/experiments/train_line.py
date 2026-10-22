"""Which train the rider was on: the line, direction, and scheduled trip, from the stations seen in order.

Observations: each run of one station name in the replayed status (`status.csv` from `crates/cli/examples/export.rs`,
Android's fused location or a tagged cell), placed by Android's own fixes during it (`net-<ride>.csv` from
`crates/cli/examples/network_fixes.rs`), and marked stopped when a stop record (`stops.csv`) falls in it.
Candidates: every stopping pattern in the MTA's GTFS (`data/mta/gtfs`), with the stations on its track
(within 80 m of its shape) that it skips. A Viterbi pass over (pattern, station) picks the cheapest path:
a station must come later along the same pattern, a stop at a skipped station or a pass of a served one costs,
a station far from Android's fix costs, a pattern with no train there within 15 minutes costs, and changing
pattern (a transfer) costs. Then, per segment, the scheduled trip closest to the stop times, and its lateness.
Scored against the rider's "at station" tags that name a line, e.g. "96 St (C)" (labels only).

2026-10-22, 8 subway rides, 62 tags: the line is right at 44 (49 counting near-equal lines), from 35 with one pattern
per run of stations; the stop records' walking off and on (`stops2-<ride>`) split transfers, and a station passed at
speed (Android's fix lagging) or a hold in a skipped station no longer rules a line out. Left wrong:
- 2 against 3: same stops and schedule; the chime (R142 NTT against R62) would settle it, but the replayed status
  shows its last chime, R62 on a third of the 7's (all R188) rows, so it can't vote yet.
- C against B on Central Park West: same stops and schedule; a car beacon (R211) would point to the C if the R68s
  have none (the CBTC guess), untested.
- Tags on a transfer's platform, which name the next train before the sensors see it.
Scheduled trips fit with spreads under 2 minutes; lateness is the nearest trip's, so a train late by a headway
matches the next one. Not in the live status: too often wrong between same-stop lines."""

import csv
import math
import sys
from bisect import bisect_left
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[2]
GTFS = ROOT / "data/mta/gtfs"
ANALYSIS = ROOT / "target/analysis"
NY = ZoneInfo("America/New_York")
RIDES = {r: f"stops2-{r}" for r in ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922"]}
ON_TRACK_M = 80.0
SWITCH = float(__import__("os").environ.get("SWITCH", 2.0))
PASS_SERVED = float(__import__("os").environ.get("PASS_SERVED", 0.7))
# Where the stop records saw the rider walk off or on (`alighted_ns`, `boarded_ns`), a change of train is cheap
# and staying on the same one costs.
SWITCH_AT_TRANSFER = 0.5
STAY_AT_TRANSFER = 1.5
SKIP_OBS = 2.0
WINDOW_S = 15 * 60


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def meters(a, b):
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def seconds(hms):
    h, m, s = map(int, hms.split(":"))
    return h * 3600 + m * 60 + s


def load_gtfs():
    names = {r["GTFS Stop ID"]: r["Stop Name"] for r in read(ROOT / "data/mta/stations.csv")}
    pos = {r["stop_id"]: (float(r["stop_lat"]), float(r["stop_lon"])) for r in read(GTFS / "stops.txt") if not r["parent_station"]}
    trips = {r["trip_id"]: r for r in read(GTFS / "trips.txt")}
    seq = defaultdict(list)
    with open(GTFS / "stop_times.txt") as f:
        for r in csv.DictReader(f):
            seq[r["trip_id"]].append((int(r["stop_sequence"]), r["stop_id"][:-1], seconds(r["departure_time"])))
    shapes = defaultdict(list)
    for r in read(GTFS / "shapes.txt"):
        shapes[r["shape_id"]].append((int(r["shape_pt_sequence"]), float(r["shape_pt_lat"]), float(r["shape_pt_lon"])))
    patterns = {}
    for tid, st in seq.items():
        st.sort()
        t = trips[tid]
        key = (t["route_id"], t["direction_id"], tuple(s for _, s, _ in st))
        p = patterns.setdefault(key, {"route": t["route_id"], "dir": t["direction_id"], "stops": key[2],
                                      "shape": t["shape_id"], "trips": []})
        p["trips"].append((t["service_id"], {s: x for _, s, x in st}, t["trip_headsign"], tid))
    # Stations on each pattern's track, in order along its shape.
    shape_cache = {}
    out = []
    for p in patterns.values():
        sid = p["shape"] or p["trips"][0][3].split("_")[-1]
        if sid not in shape_cache:
            pts = [(la, lo) for _, la, lo in sorted(shapes.get(sid, []))]
            on = []
            if pts:
                lat0, lat1 = min(x for x, _ in pts) - 0.002, max(x for x, _ in pts) + 0.002
                lon0, lon1 = min(y for _, y in pts) - 0.002, max(y for _, y in pts) + 0.002
                for stop, xy in pos.items():
                    if not (lat0 <= xy[0] <= lat1 and lon0 <= xy[1] <= lon1):
                        continue
                    best = min(range(len(pts)), key=lambda i: (pts[i][0] - xy[0]) ** 2 + (pts[i][1] - xy[1]) ** 2)
                    if meters(pts[best], xy) < ON_TRACK_M:
                        on.append((best, stop))
            shape_cache[sid] = [s for _, s in sorted(on)]
        track = shape_cache[sid]
        served = set(p["stops"])
        # Keep the track between the pattern's own ends, and make sure every stop is in it, in order.
        if track and all(s in track for s in p["stops"]):
            i, j = sorted((track.index(p["stops"][0]), track.index(p["stops"][-1])))
            track = track[i : j + 1]
            if track and track[0] != p["stops"][0]:
                track = track[::-1]
        else:
            track = list(p["stops"])
        p["track"] = track
        p["served"] = served
        p["names"] = [names.get(s, s) for s in track]
        by_stop = defaultdict(list)
        for service, times, _, _ in p["trips"]:
            for s, x in times.items():
                by_stop[(service, s)].append(x)
        p["times"] = {k: sorted(v) for k, v in by_stop.items()}
        out.append(p)
    return out, pos, names


def services(day):
    wd = day.weekday()
    return ["Weekday"] if wd < 5 else ["Saturday", "Saturday-H-20260908-20261031"] if wd == 5 else ["Sunday", "Sunday-H-20260908-20261031"]


def clock_of(ride):
    fx = read(ANALYSIS / RIDES[ride] / "fixes.csv")[0]
    off = int(fx["time_ms"]) * 1_000_000 - int(fx["t_ns"])
    return lambda t: datetime.fromtimestamp((t + off) / 1e9, NY)


def observations(ride):
    d = ANALYSIS / RIDES[ride]
    status = read(d / "status.csv")
    stops = [s for s in read(d / "stops.csv") if s["in_subway"] == "true"]
    net = [(int(r["t_ns"]), float(r["lat"]), float(r["lon"])) for r in read(ANALYSIS / "trainline" / f"net-{ride}.csv")
           if r["h_acc"] not in ("", "NaN") and float(r["h_acc"]) <= 150]
    runs = []
    for r in status:
        if not r["station"]:
            continue
        t = int(r["t_ns"])
        if runs and runs[-1]["name"] == r["station"] and t - runs[-1]["t1"] <= 30e9:
            runs[-1]["t1"] = t
        else:
            runs.append({"name": r["station"], "t0": t, "t1": t})
    for o in runs:
        near = [(la, lo) for t, la, lo in net if o["t0"] - 10e9 <= t <= o["t1"] + 10e9]
        o["pos"] = (sorted(x[0] for x in near)[len(near) // 2], sorted(x[1] for x in near)[len(near) // 2]) if near else None
        st = [s for s in stops if o["t0"] - 15e9 <= int(s["stopped_ns"]) <= o["t1"] + 10e9]
        o["stopped"] = int(st[0]["stopped_ns"]) if st else None
        # Doors heard: a station stop, not a hold in a station the train skips.
        o["doors"] = any(s["doors_closed_ns"] for s in st)
    moves = [int(s[k]) for s in stops for k in ("alighted_ns", "boarded_ns") if s.get(k)]
    for i, o in enumerate(runs):
        o["transfer"] = i > 0 and any(runs[i - 1]["t0"] - 30e9 <= m <= o["t0"] for m in moves)
    return runs


def time_cost(p, stop, t_day, svc):
    for service in svc:
        xs = p["times"].get((service, stop))
        if xs:
            for base in (t_day, t_day + 86400):
                i = bisect_left(xs, base - WINDOW_S)
                if i < len(xs) and xs[i] <= base + WINDOW_S:
                    return 0.0
    return 2.0


def emission(o, p, k, pos, t_day, svc):
    stop = p["track"][k]
    c = 0.0
    if o["pos"] is not None and stop in pos:
        # Capped: passing at speed, Android's fix lags a station or two behind.
        c += min(1.5, max(0.0, (meters(o["pos"], pos[stop]) - 150) / 150))
    served = stop in p["served"]
    c += (0.0 if served else 3.0 if o["doors"] else 1.0) if o["stopped"] else (PASS_SERVED if served else 0.0)
    return c + (time_cost(p, stop, t_day, svc) if served else 0.0)


def day_seconds(clock, t):
    ct = clock(t)
    return ct.hour * 3600 + ct.minute * 60 + ct.second


# The door chime's family by route, where the fleet is one family (the rider's: 1 R62A, 3 R62, 2 R142, 7 R188).
CHIME = {"1": "R62", "3": "R62", "2": "Ntt", "4": "Ntt", "5": "Ntt", "6": "Ntt", "6X": "Ntt", "7": "Ntt", "7X": "Ntt"}


def chime_votes(ride, t0, t1):
    rows = read(ANALYSIS / RIDES[ride] / "status.csv")
    votes = defaultdict(int)
    prev = ""
    for r in rows:
        t = int(r["t_ns"])
        # Count each chime once: a change of the shown family, or the first one in the span.
        if t0 <= t <= t1 and r["cars"] and r["cars"] != prev:
            votes[r["cars"]] += 1
        prev = r["cars"]
    return votes


def alternatives(seg, patterns, pos, clock, svc, ride):
    """Every pattern that explains the segment's stations nearly as well, with the chime's vote, best first."""
    t_days = [day_seconds(clock, o["t0"]) for o in seg["obs"]]
    votes = chime_votes(ride, seg["obs"][0]["t0"], seg["obs"][-1]["t1"])
    out = []
    for qi, q in enumerate(patterns):
        ks, last = [], -1
        for o in seg["obs"]:
            k = next((k for k in range(last + 1, len(q["names"])) if q["names"][k] == o["name"]), None)
            if k is None:
                break
            ks.append(k)
            last = k
        if len(ks) != len(seg["obs"]):
            continue
        c = sum(emission(o, q, k, pos, td, svc) for o, k, td in zip(seg["obs"], ks, t_days))
        c += sum(sum(0.3 if s in q["served"] else 0.1 for s in q["track"][a + 1 : b]) for a, b in zip(ks, ks[1:]))
        fam = CHIME.get(q["route"])
        n = sum(votes.values())
        if fam and n >= 2:
            c += 2.0 * (1 - votes[fam] / n)
        out.append((c, qi, ks))
    out.sort(key=lambda x: x[0])
    return [x for x in out if x[0] <= out[0][0] + 0.5] if out else []


def viterbi(obs, patterns, pos, clock):
    day = clock(obs[0]["t0"])
    svc = services(day if day.hour >= 4 else day.replace(day=day.day - 1))
    states = []  # per obs: {(pi, k): (cost, back)}
    for i, o in enumerate(obs):
        ct = clock(o["t0"])
        t_day = ct.hour * 3600 + ct.minute * 60 + ct.second
        em = {}
        for pi, p in enumerate(patterns):
            for k, n in enumerate(p["names"]):
                if n != o["name"]:
                    continue
                em[(pi, k)] = emission(o, p, k, pos, t_day, svc)
        prev = states[-1] if states else {}
        best_prev = min(prev.values(), key=lambda v: v[0])[0] if prev else 0.0
        best_key = min(prev, key=lambda s: prev[s][0]) if prev else None
        cur = {}
        for (pi, k), c in em.items():
            switch, stay = (SWITCH_AT_TRANSFER, STAY_AT_TRANSFER) if o["transfer"] else (SWITCH, 0.0)
            options = [(best_prev + switch + c, best_key)] if prev else [(c, None)]
            for (qj, kk), (pc, _) in prev.items():
                if qj == pi and kk < k:
                    skipped = sum(0.3 if s in patterns[pi]["served"] else 0.1 for s in patterns[pi]["track"][kk + 1 : k])
                    options.append((pc + skipped + stay + c, (qj, kk)))
            cur[(pi, k)] = min(options, key=lambda x: x[0])
        # An observation may be left unexplained (a mislabeled station, a walk through a transfer).
        for key, (pc, _) in prev.items():
            if key not in cur or cur[key][0] > pc + SKIP_OBS:
                cur[key] = (pc + SKIP_OBS, ("skip", key))
        states.append(cur)
    # Trace back.
    key = min(states[-1], key=lambda s: states[-1][s][0])
    path = []
    for i in range(len(obs) - 1, -1, -1):
        cost, back = states[i][key]
        if isinstance(back, tuple) and back and back[0] == "skip":
            path.append(None)
            key = back[1]
        else:
            path.append(key)
            if back is None:
                path.extend([None] * i)
                break
            key = back
    return path[::-1], svc


def segments(obs, path, patterns):
    segs = []
    for o, s in zip(obs, path):
        if s is None:
            continue
        pi, k = s
        if segs and segs[-1]["pi"] == pi and segs[-1]["ks"][-1] < k:
            segs[-1]["obs"].append(o)
            segs[-1]["ks"].append(k)
        else:
            segs.append({"pi": pi, "obs": [o], "ks": [k]})
    return segs


def best_trip(seg, p, clock, svc):
    best = None
    for service, times, headsign, tid in p["trips"]:
        if service not in svc:
            continue
        diffs = []
        for o, k in zip(seg["obs"], seg["ks"]):
            stop = p["track"][k]
            if stop not in times or not o["stopped"]:
                continue
            ct = clock(o["stopped"])
            t_day = ct.hour * 3600 + ct.minute * 60 + ct.second
            if ct.hour < 4:
                t_day += 86400
            diffs.append(t_day - times[stop])
        if not diffs:
            continue
        diffs.sort()
        med = diffs[len(diffs) // 2]
        spread = sum(abs(d - med) for d in diffs) / len(diffs)
        score = abs(med) + 2 * spread
        if best is None or score < best[0]:
            best = (score, med, spread, len(diffs), tid, headsign)
    return best


def tags(ride):
    out = []
    for e in read(ANALYSIS / RIDES[ride] / "events.csv"):
        if e["kind"] != "KIND_AT_STATION":
            continue
        txt = e["text"].strip()
        if "(" in txt and txt.endswith(")"):
            out.append((int(e["t_ns"]), txt, txt[txt.rindex("(") + 1 : -1].strip().upper()))
    return out


def identify(ride, patterns, pos):
    """The ride's train segments: [{t0, t1, route, dir, routes (near-equal), trip, first, last, passed}]."""
    clock = clock_of(ride)
    obs = observations(ride)
    path, svc = viterbi(obs, patterns, pos, clock)
    out = []
    for s in segments(obs, path, patterns):
        alts = alternatives(s, patterns, pos, clock, svc, ride) or [(0.0, s["pi"], s["ks"])]
        routes = []
        for _, qi, _ in alts:
            if patterns[qi]["route"] not in routes:
                routes.append(patterns[qi]["route"])
        # Among near-equal patterns, the one whose schedule fits the stop times best.
        fits = []
        for c, qi, ks in alts:
            trip = best_trip({"obs": s["obs"], "ks": ks}, patterns[qi], clock, svc)
            fits.append(((trip[2] if trip else 1e9), c, qi, ks, trip))
        _, _, qi, ks, trip = min(fits, key=lambda x: (x[0], x[1]))
        p = patterns[qi]
        out.append({"t0": s["obs"][0]["t0"], "t1": s["obs"][-1]["t1"], "route": p["route"], "dir": p["dir"],
                    "routes": routes, "trip": trip, "first": s["obs"][0]["name"], "last": s["obs"][-1]["name"],
                    "n": len(s["obs"]), "passed": [o["name"] for o, k in zip(s["obs"], ks) if p["track"][k] not in p["served"]]})
    return out


def main(rides):
    patterns, pos, names = load_gtfs()
    print(f"{len(patterns)} stopping patterns", file=sys.stderr)
    total = right = in_set = 0
    for ride in rides:
        clock = clock_of(ride)
        print(f"== {ride}")
        spans = []
        for g in identify(ride, patterns, pos):
            trip = g["trip"]
            spans.append((g["t0"], g["t1"], g["route"], set(g["routes"])))
            tr = f", trip {trip[4]} to {trip[5]}, {trip[1] / 60:+.1f} min late (spread {trip[2] / 60:.1f}, {trip[3]} stops)" if trip else ""
            others = [r for r in g["routes"] if r != g["route"]]
            alt = f" (or {'/'.join(others)})" if others else ""
            print(f"  {clock(g['t0']):%-I:%M %p}-{clock(g['t1']):%-I:%M %p} {g['route']}{alt} dir {g['dir']}: "
                  f"{g['first']} to {g['last']}, {g['n']} stations"
                  f"{', passed ' + ', '.join(g['passed']) if g['passed'] else ''}{tr}".replace("AM", "am").replace("PM", "pm"))
        for t, txt, line in tags(ride):
            hit = [(r, rs) for a, b, r, rs in spans if a - 30e9 <= t <= b + 30e9]
            cands = {r for r, _ in hit} | {r + "X" for r, _ in hit}
            sets = set().union(*(rs for _, rs in hit)) if hit else set()
            got = "/".join(r for r, _ in hit) or "-"
            ok = line in cands or line + "X" in cands
            total += 1
            right += ok
            in_set += ok or line in sets
            if not ok:
                print(f"    miss: {clock(t):%-I:%M:%S %p} tag {txt!r}, got {got}".replace("AM", "am").replace("PM", "pm"))
    print(f"line right at {right} of {total} tags; among the near-equal lines at {in_set}")


if __name__ == "__main__":
    main(sys.argv[1:] or list(RIDES))
