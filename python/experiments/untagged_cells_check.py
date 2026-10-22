"""The cells the rider never tagged, placed by Android's own location (Google's database) as in
`station_cells_check.py`: each registered LTE cell's station is the one Google most often put the rider near while
on it. Suspicious, per the rider: a ride's cells naming stations out of order, or jumping away and back.
Flags, with clock times:
- split: Google put the rider at stations over 800 m apart while on the same cell (2 or more fixes each);
- jump: a cell whose station is over 1.5 km from both the station before it and after it in the ride,
  while those two are within 1 km of each other, all within 5 minutes;
- back and forth: stations A, B, A in a row within 3 minutes, B being the flagged cell's.
Needs `cell.csv`, `network.csv`, and `fixes.csv` per ride in `target/analysis`.

On nine rides (2026-10-22): 106 untagged cells placed, 2 flagged, both on the elevated 7 on Oct 20 at 4:02 pm:
`28347409` (61 St-Woodside) and `28357391` (69 St) swapping back and forth between the two stations, a minute before
the rider's 69 St tag; above ground, with GPS, so harmless. No jumps, and no cell split between distant stations."""

import csv
import sys
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402
from station_cells_check import RIDES, meters  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NY = ZoneInfo("America/New_York")


def clock_of(ride):
    """Boot-time ns to a New York clock time, from the ride's first GPS fix."""
    r = next(csv.DictReader(open(ANALYSIS / ride / "fixes.csv")))
    off = int(r["time_ms"]) * 1_000_000 - int(r["t_ns"])
    return lambda t: datetime.fromtimestamp((t + off) / 1e9, NY).strftime("%b %-d %-I:%M %p").replace("AM", "am").replace("PM", "pm")


def main():
    mta = {}
    for r in csv.DictReader(open(ROOT / "crates/fusion/data/stations.csv")):
        mta.setdefault(r["station"], (float(r["lat"]), float(r["lon"])))
    tagged = {r["cell_id"] for r in csv.DictReader(open(ROOT / "crates/fusion/data/station_cells.csv"))}

    def nearest(la, lo):
        return min((meters((la, lo), p), n) for n, p in mta.items())

    google = defaultdict(Counter)
    rides = {}
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
        rides[ride] = cells

    station = {c: g.most_common(1)[0][0] for c, g in google.items()}
    flags = defaultdict(list)
    for c, g in google.items():
        common = [(n, k) for n, k in g.most_common() if k >= 2]
        if len(common) >= 2 and meters(mta[common[0][0]], mta[common[1][0]]) > 800:
            flags[c].append(f"split: {common[0][0]} {common[0][1]}, {common[1][0]} {common[1][1]}")
    for ride, cells in rides.items():
        clock = clock_of(ride)
        seq = []
        for t, c in cells:
            if c in station and (not seq or station[seq[-1][1]] != station[c]):
                seq.append((t, c))
        for k in range(1, len(seq) - 1):
            (ta, a), (tb, b), (tc, cc) = seq[k - 1], seq[k], seq[k + 1]
            pa, pb, pc = mta[station[a]], mta[station[b]], mta[station[cc]]
            if tc - ta < 300e9 and meters(pa, pb) > 1500 and meters(pb, pc) > 1500 and meters(pa, pc) < 1000:
                flags[b].append(f"jump on {ride} at {clock(tb)}: {station[a]}, {station[b]}, {station[cc]}")
            if station[a] == station[cc] and tc - ta < 180e9:
                flags[b].append(f"back and forth on {ride} at {clock(tb)}: {station[a]}, {station[b]}, {station[cc]}")
    untagged = [c for c in station if c not in tagged]
    print(f"{len(untagged)} untagged cells placed by Google, {sum(1 for c in untagged if flags.get(c))} flagged\n")
    print("| Cell | Google's station (fixes) | Flags |\n|---|---|---|")
    for c in sorted(untagged, key=lambda c: station[c]):
        if flags.get(c):
            top = ", ".join(f"{n} {k}" for n, k in google[c].most_common(2))
            print(f"| {c} | {top} | {'; '.join(flags[c])} |")


if __name__ == "__main__":
    main()
