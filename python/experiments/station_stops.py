"""Stations from the sensors alone: at each door cycle's closing (Rust, `data/work/doors/`), the registered LTE cell
in the 8 s before it, while the train surely stood. Stops on any ride sharing that cell are one station.
Prints each cell's stops across rides with the stations `trip_paths.py` put it at (from GNSS at the ends of each
leg and the MTA's stop orders, before the rider's corrections), and how often each cell recurs.

On seven rides: 93 stops heard, 90 with an LTE cell, in 62 cells; only 6 cells recur on another ride.
A cell ID is its site (eNodeB, the ID over 256) and a sector; 38 sites, 9 recurring, with 40 of the stops.
A site covers a few stations (`115805`: 103 St to 116 St on the 1), so the cell, not the site, names a station,
and most stations so far were heard stopped at once. Too few to pin positions yet: that needs more rides and
the rider's corrected trips to name the cells."""

import csv
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["135509", "154951", "184134", "214427", "155919", "182135", "215922"]
SURE_S = 8.0


def cells(ride):
    with open(ANALYSIS / ride / "cell.csv") as f:
        return [(int(r["t_ns"]), r["cell_id"]) for r in csv.DictReader(f) if r["registered"] == "true" and r["kind"] == "lte"]


def stops():
    """[(ride, close ns, cell)] with the cell most often registered while standing, if any."""
    out = []
    for ride in RIDES:
        cyc = np.loadtxt(ROOT / "data/work/doors" / f"{ride}.csv", delimiter=",", skiprows=1, dtype=np.int64).reshape(-1, 2)
        cs = cells(ride)
        for _, close in cyc:
            seen = Counter(c for t, c in cs if close - SURE_S * 1e9 <= t <= close)
            out.append((ride, int(close), seen.most_common(1)[0][0] if seen else None))
    return out


def main():
    import trip_paths as T

    stations, patterns, ls = T.legs()
    table, _ = T.solve(patterns, ls)
    name = lambda s: stations[s][0]
    by_cell = defaultdict(list)
    st = stops()
    for ride, close, cell in st:
        by_cell[cell].append(ride)
    none = len(by_cell.pop(None, []))
    multi = {c: r for c, r in by_cell.items() if len(set(r)) > 1}
    print(f"{len(st)} stops heard; {none} with no LTE cell; {len(by_cell)} cells, {len(multi)} on more than one ride")
    for c, rides in sorted(by_cell.items(), key=lambda x: -len(x[1])):
        guess = ", ".join(f"{name(s)} x{k}" for s, k in table.get(c, Counter()).most_common(2)) or "?"
        print(f"  {c:>10s}: {len(rides)} stops on {', '.join(sorted(set(rides)))}; trip_paths: {guess}")


if __name__ == "__main__":
    main()
