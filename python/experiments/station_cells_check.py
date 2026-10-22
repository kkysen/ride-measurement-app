"""Checks the tagged cell-to-station table (`station_cells.py`) against Android's own location (Google's database):
for every registered LTE cell, the stations nearest the fused fixes while the phone was on it (accuracy 150 m or
better, nearest station within 100 m), across rides. Flags a cell whose tagged station Google never put the rider
near, and cells that a ride's sequence visits, leaves, and returns to (back and forth along the line).
Needs `network.csv` per ride (`crates/cli/examples/network_fixes.rs`).

On nine rides, Google agrees with 49 of the 52 cells. Flagged: `28163244`, tagged 69 St on the 7 while passing,
where Google put the rider at 74 St-Broadway all 6 times (likely the 74 St cell, the tag a station early);
`29254526` (Jackson Hts, where Google mostly says 74 St-Broadway, the same complex); `29639549` (96 St, back and
forth on `182135`, the transfer between the 1 and 2/3 platforms). `29254524` (Grand Av-Newtown) had no fix."""

import csv
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
RIDES = ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922", "002914"]


def meters(a, b):
    dn = math.radians(a[0] - b[0]) * 6_371_000
    de = math.radians(a[1] - b[1]) * 6_371_000 * math.cos(math.radians(a[0]))
    return math.hypot(dn, de)


def main():
    mta = [(r["station"], float(r["lat"]), float(r["lon"])) for r in csv.DictReader(open(ROOT / "crates/fusion/data/stations.csv"))]
    table = {r["cell_id"]: r for r in csv.DictReader(open(ROOT / "crates/fusion/data/station_cells.csv"))}

    def nearest(la, lo):
        return min(((meters((la, lo), (m[1], m[2])), m[0]) for m in mta))

    google = defaultdict(Counter)
    back_and_forth = defaultdict(list)
    for ride in RIDES:
        cells = sorted((int(c["t_ns"]), c["cell_id"]) for c in csv.DictReader(open(ANALYSIS / ride / "cell.csv"))
                       if c["registered"] == "true" and c["kind"] == "lte" and c["cell_id"])
        net = sorted((int(r["t_ns"]), float(r["lat"]), float(r["lon"]), float(r["h_acc"] or "nan"))
                     for r in csv.DictReader(open(ANALYSIS / ride / "network.csv")))
        j = 0
        for t, la, lo, acc in net:
            while j + 1 < len(cells) and cells[j + 1][0] <= t:
                j += 1
            if not cells or cells[j][0] > t or t - cells[j][0] > 5e9 or not acc <= 150:
                continue
            d, name = nearest(la, lo)
            if d <= max(100, acc):
                google[cells[j][1]][name] += 1
        # The table's stations in the ride's order: a cell left and come back to within 3 minutes.
        seq = []
        for t, c in cells:
            if c in table and (not seq or seq[-1][1] != c):
                seq.append((t, c))
        for k in range(2, len(seq)):
            if seq[k][1] == seq[k - 2][1] and seq[k][0] - seq[k - 2][0] < 180e9:
                back_and_forth[seq[k - 1][1]].append(ride)
    print("| Cell | Tagged as | Google, while on it | Flag |\n|---|---|---|---|")
    for c, r in sorted(table.items(), key=lambda kv: kv[1]["station"]):
        tagged = nearest(float(r["lat"]), float(r["lon"]))[1]
        g = google.get(c, Counter())
        top = ", ".join(f"{n} {k}" for n, k in g.most_common(3)) or "no fix"
        flags = []
        if g and tagged not in g:
            flags.append("Google never put you there")
        elif g and g.most_common(1)[0][0] != tagged:
            flags.append("Google mostly elsewhere")
        if back_and_forth.get(c):
            flags.append(f"back and forth on {', '.join(sorted(set(back_and_forth[c])))}")
        print(f"| {c} | {tagged} | {top} | {'; '.join(flags)} |")


if __name__ == "__main__":
    main()
