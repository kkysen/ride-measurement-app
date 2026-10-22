"""Which station: the subway's own Wi-Fi access points (`BoldynWiFi` and `PasspointBoldyn`, the same radios),
seen from the train as it pulls in or passes. Sightings within `GAP_S` of each other make one visit; visits on
any ride sharing at least `SHARED` radios are one station. Prints each ride's visits in order, with the station
each is, whether a "doors open" or "train stopped" tap falls in it (which only scores), and how often the
same station comes back across rides.

A radio is its BSSID's first five bytes: each one broadcasts both networks on neighboring addresses."""

import csv
import sys
from collections import defaultdict

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

RIDES = ["151449", "184134", "214427", "135509", "155919", "182135", "215922"]
SSIDS = {"BoldynWiFi", "PasspointBoldyn"}
GAP_S = 20.0
SHARED = 2


def sightings(ride):
    out = []
    with open(ANALYSIS / ride / "wifi.csv") as f:
        for r in csv.DictReader(f):
            if r["ssid"].strip().strip('"') in SSIDS:
                out.append((int(r["t_ns"]), r["bssid"][:14], int(r["dbm"])))
    return sorted(out)


def visits(ride):
    """(start ns, end ns, {radio: strongest dBm})."""
    out = []
    for t, radio, dbm in sightings(ride):
        if out and t - out[-1][1] <= GAP_S * 1e9:
            v = out[-1]
            v[1] = t
            v[2][radio] = max(v[2].get(radio, -999), dbm)
        else:
            out.append([t, t, {radio: dbm}])
    return out


def taps(ride, kinds):
    with open(ANALYSIS / ride / "events.csv") as f:
        return [int(r["t_ns"]) for r in csv.DictReader(f) if r["kind"].removeprefix("KIND_") in kinds]


def scans(ride, bin_s=4.0, min_dbm=-80):
    """Sightings binned into scans (results of one land within a few seconds): (t ns, {radio: dBm})."""
    out = {}
    for t, radio, dbm in sightings(ride):
        if dbm < min_dbm:
            continue
        k = int(t // (bin_s * 1e9))
        d = out.setdefault(k, (t, {}))[1]
        d[radio] = max(d.get(radio, -999), dbm)
    return [out[k] for k in sorted(out)]


def radio_stations(min_jaccard=0.3, strongest=3):
    """Radios grouped by how often they're among the strongest in the same scans, on every ride:
    a station's radios are heard together; a neighbor's only at the edges. {radio: station}."""
    seen = defaultdict(int)
    both = defaultdict(int)
    for ride in RIDES:
        for _, d in scans(ride):
            top = sorted(d, key=d.get, reverse=True)[:strongest]
            for a in top:
                seen[a] += 1
            for i, a in enumerate(top):
                for b in top[i + 1 :]:
                    both[tuple(sorted((a, b)))] += 1
    parent = {r: r for r in seen}

    def find(r):
        while parent[r] != r:
            parent[r] = parent[parent[r]]
            r = parent[r]
        return r

    for (a, b), n in both.items():
        if n / (seen[a] + seen[b] - n) >= min_jaccard and n >= 2:
            parent[find(a)] = find(b)
    roots = {}
    return {r: roots.setdefault(find(r), len(roots)) for r in seen}


def sequence(ride, of):
    """Runs of scans whose strongest radio is the same station: (start ns, end ns, station, scans)."""
    out = []
    for t, d in scans(ride):
        top = max(d, key=d.get)
        if top not in of:
            continue
        s = of[top]
        if out and out[-1][2] == s and t - out[-1][1] <= GAP_S * 1e9:
            out[-1][1] = t
            out[-1][3] += 1
        else:
            out.append([t, t, s, 1])
    return out


def main_strongest():
    of = radio_stations()
    rides_of = defaultdict(set)
    seqs = {ride: sequence(ride, of) for ride in RIDES}
    for ride, sq in seqs.items():
        for v in sq:
            rides_of[v[2]].add(ride)
    for ride, sq in seqs.items():
        stops = taps(ride, {"DOORS_OPEN", "TRAIN_STOPPED"})
        print(f"\n{ride}: start s, length s, scans, station, rides with it, stop tap in it")
        t0 = sq[0][0] if sq else 0
        for a, b, s, n in sq:
            tapped = any(a - 10e9 <= x <= b + 10e9 for x in stops)
            print(f"  {(a - t0) * 1e-9:7.0f} {(b - a) * 1e-9:5.0f} {n:3d}  S{s:<3d} {len(rides_of[s])} {'stop' if tapped else ''}")
    stations = set(of.values())
    multi = sum(1 for rs in rides_of.values() if len(rs) > 1)
    print(f"\n{len(of)} radios, {len(stations)} stations, {multi} seen on more than one ride")


def main():
    all_visits = [(ride, v) for ride in RIDES for v in visits(ride)]
    # Union-find over visits sharing radios.
    parent = list(range(len(all_visits)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    by_radio = defaultdict(list)
    for i, (_, v) in enumerate(all_visits):
        for radio in v[2]:
            by_radio[radio].append(i)
    for i, (_, v) in enumerate(all_visits):
        counts = defaultdict(int)
        for radio in v[2]:
            for j in by_radio[radio]:
                if j != i:
                    counts[j] += 1
        for j, n in counts.items():
            if n >= SHARED:
                parent[find(i)] = find(j)
    roots = {}
    station = [roots.setdefault(find(i), len(roots)) for i in range(len(all_visits))]
    rides_of = defaultdict(set)
    for (ride, _), s in zip(all_visits, station):
        rides_of[s].add(ride)

    for ride in RIDES:
        stops = taps(ride, {"DOORS_OPEN", "TRAIN_STOPPED"})
        print(f"\n{ride}: visit start (s from first), length s, radios, station, rides with it, stop tap in it")
        t0 = None
        for (r, v), s in zip(all_visits, station):
            if r != ride:
                continue
            t0 = t0 or v[0]
            tapped = any(v[0] - 10e9 <= a <= v[1] + 10e9 for a in stops)
            print(f"  {(v[0] - t0) * 1e-9:7.0f} {(v[1] - v[0]) * 1e-9:5.0f} {len(v[2]):3d}  S{s:<3d} "
                  f"{len(rides_of[s])} {'stop' if tapped else ''}")
    multi = sum(1 for rs in rides_of.values() if len(rs) > 1)
    print(f"\n{len(all_visits)} visits, {len(roots)} stations, {multi} seen on more than one ride")


if __name__ == "__main__":
    {"strongest": main_strongest}.get(sys.argv[1] if sys.argv[1:] else "", main)()
