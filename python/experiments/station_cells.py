"""The cell-to-station table: the LTE cell registered as the rider tagged each "at station" (pulling in or
passing; taps name the station and line, e.g. "96 St (1)"), from 2 s before the tap to 15 s after, across rides.
The rider asked for exactly this, so the tags are its labels; nothing live reads a tap.
Lines are dropped from the name (a station's platforms for several lines share a cell, e.g. Jackson Heights on the
F, M, and R) but kept as the lines seen there. A cell tagged as two different stations is left out (90 St and
Junction Blvd on the 7, tagged while passing). Each station's position is the MTA's (`data/mta/stations.csv`,
matched by name and line).
Writes `crates/fusion/data/station_cells.csv`: cell_id, station, lines (space-separated), lat, lon."""

import csv
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922", "002914"]
OUT = ROOT / "crates/fusion/data/station_cells.csv"


def split(tag):
    m = re.match(r"(.*?)\s*\(([^)]*)\)\s*$", tag)
    return (m.group(1), m.group(2)) if m else (tag.strip(), "")


def main():
    names = defaultdict(Counter)
    lines = defaultdict(set)
    for r in RIDES:
        ev = [(int(e["t_ns"]), e["text"]) for e in csv.DictReader(open(ANALYSIS / r / "events.csv"))
              if e["kind"] == "KIND_AT_STATION" and e["text"].strip()]
        cells = [(int(c["t_ns"]), c["cell_id"]) for c in csv.DictReader(open(ANALYSIS / r / "cell.csv"))
                 if c["registered"] == "true" and c["kind"] == "lte" and c["cell_id"]]
        for t, tag in ev:
            seen = Counter(c for tc, c in cells if t - 2e9 <= tc <= t + 15e9)
            if not seen:
                continue
            cell = seen.most_common(1)[0][0]
            name, line = split(tag)
            names[cell][name] += 1
            if line:
                lines[cell].add(line)
    # The MTA's station list has each station's daytime routes, to tell same-named stations apart (33 St on the 7
    # is in Queens, not Manhattan).
    stations = [(s["Stop Name"].lower(), s["Daytime Routes"].split(), float(s["GTFS Latitude"]), float(s["GTFS Longitude"]))
                for s in csv.DictReader(open(ROOT / "data/mta/stations.csv"))]
    alias = {"jackson heights": "jackson hts", "times square": "times sq", "47-50 st's": "47-50 sts", "7 av/53 st": "7 av"}

    # Trunks: an express passing a local station (the 2 at 59 St) shares the local's track.
    trunks = ["1 2 3", "4 5 6", "A C E", "B D F M", "N Q R W", "7", "G", "J Z", "L", "S"]
    trunk = {r: set(t.split()) for t in trunks for r in t.split()}

    def where(name, lines):
        key = name.lower()
        key = alias.get(key, key)
        near = set().union(*(trunk.get(line.upper(), {line.upper()}) for line in lines)) if lines else set()
        cands = [s for s in stations if key in s[0] and (not near or near & set(s[1]))]
        if not cands:
            return None
        # One the tagged line stops at (110 St on the 1 is Cathedral Pkwy, on the 2 and 3 Central Park North),
        # then the closest name: exact, a prefix, then anywhere in it ("110 st" in "cathedral pkwy (110 st)").
        own = {line.upper() for line in lines}
        best = min(cands, key=lambda s: (not own & set(s[1]), s[0] != key, not s[0].startswith(key), len(s[0])))
        return best[2:]

    rows, dropped, unplaced = [], [], []
    for cell, n in sorted(names.items()):
        if len(n) > 1:
            dropped.append((cell, dict(n)))
            continue
        name = next(iter(n))
        pos = where(name, lines[cell])
        if pos is None:
            unplaced.append(name)
            continue
        rows.append((cell, name, " ".join(sorted(lines[cell])), f"{pos[0]:.6f}", f"{pos[1]:.6f}"))
    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cell_id", "station", "lines", "lat", "lon"])
        w.writerows(rows)
    print(f"{len(rows)} cells, {len({r[1] for r in rows})} stations; left out {dropped}; no position for {unplaced}")


if __name__ == "__main__":
    main()
