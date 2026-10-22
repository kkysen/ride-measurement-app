"""Which station, from the registered cell: the cell ID the phone is connected to around each "doors open" or
"train stopped" tap (which only place the stops), per ride, to see whether a station keeps its cell across rides.
On `182135` (110 St to Chambers St on the 1/2/3) and `215922` (back), the stops they share have the same cells
in reverse order: 110 St `29646293`, Chambers St `28852869`, and `29638278` and `29639557` between."""

import csv
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402

RIDES = sys.argv[1:] or ["182135", "215922"]


def rows(ride, name):
    with open(ANALYSIS / ride / name) as f:
        return list(csv.DictReader(f))


for ride in RIDES:
    stops = sorted({int(e["t_ns"]) for e in rows(ride, "events.csv") if e["kind"] in ("KIND_DOORS_OPEN", "KIND_TRAIN_STOPPED")})
    cells = [(int(c["t_ns"]), c["kind"], c["cell_id"]) for c in rows(ride, "cell.csv") if c["registered"] == "true"]
    print(ride)
    last = None
    for a in stops:
        near = sorted({f"{k} {i}" for t, k, i in cells if abs(t - a) < 10e9})
        if near != last:
            print(f"  {(a - stops[0]) * 1e-9:6.0f} s  {', '.join(near)}")
        last = near
