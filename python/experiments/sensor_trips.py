"""Trips from the sensors alone, no taps, for the rider to correct: where each went wrong shows what to fix.

- Underground: a GNSS gap over 2 minutes with the subway's own Wi-Fi in it. It starts at the station nearest
  the last fix before the gap and ends at the one nearest the first fix after.
- Legs: inside the gap, train rides are the stretches without walking (a step rhythm in the accelerometer);
  walking between them is a transfer. A transfer across a platform has no walking, so it's missed.
- Stops: door cycles in the sound (`stops.door_cycles`), which fire at no stop at speed but miss about a third.
- Route: for each leg in turn, the route and end station whose stops after the start best match its door cycles,
  with the last leg ending at the exit station.
- Above ground: stations within 150 m of the GNSS track in order, and whether the train stopped there.

Prints each trip's legs with start and end times (the ride's local zone, am/pm), stations, and the stops heard."""

import csv
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ride  # noqa: E402
import stops as S  # noqa: E402
import trip_paths as T  # noqa: E402
from common import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = {
    "151449": "20261020-151449",
    "154951": "20261020-154951",
    "184134": "20261020-184134",
    "214427": "20261020-214427",
    "135509": "20261021-135509",
    "155919": "20261021-155919",
    "182135": "20261021-182135",
    "215922": "20261021-215922",
}
GAP_S = 120.0
WALK_WINDOW_S = 4.0
MIN_LEG_S = 60.0


def clock(ride_name):
    """Elapsed realtime ns to the ride's own local time, from a fix's UTC time."""
    fx = [f for f in T.read(ANALYSIS / ride_name / "fixes.csv") if f["time_ms"]]
    f = fx[len(fx) // 2]
    off_ms = int(f["time_ms"]) - int(f["t_ns"]) // 1_000_000
    session = ROOT / "data/sessions" / RIDES[ride_name]
    return lambda t: ride.local_clock(session, int(t) // 1_000_000 + off_ms)


def walking(session):
    """Start times (ns) of `WALK_WINDOW_S` windows with steps: |a| shaking over 1 m/s^2, mostly at 1.4 to 2.6 Hz."""
    a = ride.read_stream(session, "accel_uncal")
    t = a.timestamps_ns
    x = np.linalg.norm(a.values[:, :3].astype(float) - a.values[:, 3:6], axis=1)
    k = np.arange(t[0], t[-1], WALK_WINDOW_S * 1e9)
    idx = np.searchsorted(t, k)
    out = []
    for i in range(len(k) - 1):
        seg = x[idx[i] : idx[i + 1]]
        if len(seg) < 100:
            continue
        f = np.fft.rfftfreq(len(seg), WALK_WINDOW_S / len(seg))
        p = np.abs(np.fft.rfft(seg - seg.mean())) ** 2
        if seg.std() > 1.0 and p[(f > 1.4) & (f < 2.6)].sum() > 0.35 * p[f > 0.3].sum():
            out.append(int(k[i]))
    return np.array(out)


def underground(ride_name):
    """(last fix before, first fix after) for each GNSS gap over `GAP_S` with subway Wi-Fi in it."""
    fx = T.fixes(ride_name)
    wifi = [t for t, _, _ in __import__("station_wifi").sightings(ride_name)]
    out = []
    for f0, f1 in zip(fx, fx[1:]):
        if (f1[0] - f0[0]) * 1e-9 > GAP_S and any(f0[0] < t < f1[0] for t in wifi):
            out.append((f0, f1))
    return out


def legs_in(a, b, walks):
    """Stretches of at least `MIN_LEG_S` from `a` to `b` without walking."""
    w = [x for x in walks if a <= x <= b]
    edges = [a] + w + [b]
    out = []
    for s, e in zip(edges, edges[1:]):
        s2 = s + (WALK_WINDOW_S * 1e9 if s in w else 0)
        if (e - s2) * 1e-9 >= MIN_LEG_S:
            out.append((s2, e))
    return out


def plan(patterns, start, exit_stations, legs_stops):
    """Each leg's (route, stations): its end after as many stops as door cycles heard, the last at an exit station.
    A depth-first search over routes, cheapest by how far each leg's stop count is off."""
    best = [None]

    def go(k, at, cost, path):
        if best[0] and cost >= best[0][0]:
            return
        if k == len(legs_stops):
            if at in exit_stations:
                best[0] = (cost + exit_stations[at] / 200, path)
            return
        heard = legs_stops[k]
        seen = set()
        for route, head, p in patterns:
            if at not in p:
                continue
            i = p.index(at)
            for n in range(1, min(len(p) - i, 25)):
                key = (route, p[i + n])
                if key in seen:
                    continue
                seen.add(key)
                c = abs(n - heard) if heard else 0.3 * n
                if c > 3:
                    continue
                go(k + 1, p[i + n], cost + c, path + [(route, p[i : i + n + 1])])

    go(0, start, 0.0, [])
    return best[0]


def report():
    stations, patterns = T.gtfs()
    name = lambda s: stations[s][0]
    for ride_name, sess in RIDES.items():
        session = ROOT / "data/sessions" / sess
        at = clock(ride_name)
        print(f"\n{ride_name}")
        gaps = underground(ride_name)
        if not gaps:
            # Above ground: stations the track passes, in order, and whether it stopped (speed under 1 m/s).
            own = {int(r["t_ns"]): float(r["speed"]) for r in T.read(ANALYSIS / ride_name / "own.csv")}
            ot = np.array(sorted(own))
            last = None
            for t, la, lo in T.fixes(ride_name):
                near = [(T.meters((la, lo), (sl, sn)), s) for s, (_, sl, sn) in stations.items()]
                d, s = min(near)
                if d > 150 or s == last:
                    continue
                j = min(np.searchsorted(ot, t), len(ot) - 1)
                print(f"  {at(t)}  {name(s)}{'  (stopped)' if own[ot[j]] < 1 else ''}")
                last = s
            continue
        walks = list(walking(session))
        try:
            cycles = S.door_cycles(ride_name)
        except FileNotFoundError:
            cycles = np.zeros((0, 2))
        for f0, f1 in gaps:
            starts = T.near(stations, f0)
            exits = T.near(stations, f1)
            ls = legs_in(f0[0], f1[0], walks)
            heard = [[c for c in cycles[:, 0] if a + 30e9 < c < b + 10e9] for a, b in ls]
            print(f"  underground {at(f0[0])} to {at(f1[0])}: entered near "
                  f"{', '.join(map(name, starts)) or '?'}, came out near {', '.join(map(name, exits)) or '?'}; "
                  f"{len(ls)} legs")
            best = None
            for st, d in starts.items():
                p = plan(patterns, st, exits, [len(h) for h in heard])
                if p and (best is None or p[0] + d / 200 < best[0]):
                    best = (p[0] + d / 200, p[1])
            for k, (a, b) in enumerate(ls):
                line = f"    leg {k + 1}: {at(a)} to {at(b)}, {len(heard[k])} stops heard"
                if best:
                    route, st = best[1][k]
                    line += f": {route} from {name(st[0])} to {name(st[-1])} ({len(st) - 1} stops on)"
                print(line)
            if not best:
                print("    no route fits")


if __name__ == "__main__":
    report()
