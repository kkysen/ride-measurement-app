"""The rider's transfers: each walk off a train paired with the next walk onto one, from the stop records
(`stops.csv`, `alighted_ns` and `boarded_ns` from walking, in `data/work/analysis/stops2-<ride>` from
`crates/cli/examples/export.rs`), with the lines on either side from `train_line.py`.

Per transfer: the station, the train left (line, direction, chime family, the rider's car beacon) and the one
boarded, when, how much of it was walking (`steps.csv`) and how much standing, and whether it was across a platform
or through a passage (the barometer's height change, `press-<ride>.csv` from `ride csv <session> pressure`,
over 2.5 m, or a minute or more of walking). Writes `transfers.csv` (sensors only) and `transfers_tapped.csv`
(boundaries from the rider's boarded and alighted taps, and lines from "at station" tags naming one: labels only)
next to the stop records. An alighting with no boarding within `MAX_TRANSFER_S` ends a trip, not a transfer.

2026-10-22, against the known ones (sensors / taps):
- `155919` Times Sq 7 to 3: 4:08:15 to 4:09:27 pm / 4:08:16 to 4:09:22 pm, 48 s walking, +9 m (a passage up).
  The line after is "2" from the sensors (2 and 3 share stops; see `train_line.py`), "3" from the tags.
- `151449` Times Sq 3 to 7: off at 3:38:01 pm both ways, but on at 3:38:57 pm by the sensors against 3:40:30 pm
  tapped: the walk on was taken from walking onto the 7's platform, not the train.
- `184134` Jackson Hts M to F: 7:20:38 to 7:24:30 pm, 16 s walking, same platform (across it). The R to M at
  Grand Av (7:15 pm) is found too; the 7 to R at 74 St-Broadway (-17 m) only by taps.
- `135509` 59 St C to D: only by taps (62 s, 60 walking); 7 Av D to E: 2:14:05 to 2:17:17 pm by the sensors only.
- Two false ones: a 16 s walk at 82 St (`184134`, the rider moved along the 7, tapped as off and on).
Not in the live status: the lines come from Python only (`train_line.py`); per the single source rule, that
waits for a Rust port of the line matcher."""

import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import train_line as tl  # noqa: E402

MAX_TRANSFER_S = 20 * 60
STEP_WINDOW_S = 4.0
PASSAGE_M = 2.5
PASSAGE_WALK_S = 60.0
M_PER_HPA = 8.3
PREFIX = "e2c56db5dffb48d2b060d0f5a710"
COLUMNS = ["station", "from_line", "from_dir", "from_cars", "from_car", "to_line", "to_dir", "to_cars", "to_car",
           "alighted", "boarded", "alighted_ns", "boarded_ns", "total_s", "walking_s", "standing_s", "height_m", "kind"]


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def nearest_status(status, t):
    i = np.searchsorted(status["t"], t)
    return status["rows"][max(0, min(i, len(status["rows"]) - 1))]


def car_beacon(ride, a, b):
    """The strongest car beacon heard steadily between a and b, or ""."""
    path = tl.ANALYSIS / ride / "ble.csv"
    if not path.exists():
        return ""
    seen = {}
    for r in read(path):
        t = int(r["t_ns"])
        if not a <= t <= b:
            continue
        i = r["data"].find(PREFIX)
        if i < 0 or int(r["rssi"]) == 127:
            continue
        seen.setdefault(r["data"][i + len(PREFIX) : i + len(PREFIX) + 4], []).append(int(r["rssi"]))
    steady = {k: np.median(v) for k, v in seen.items() if len(v) >= 5}
    return max(steady, key=steady.get) if steady else ""


def height_m(press, a, b):
    t, hpa = press
    before = hpa[(t >= a - 10e9) & (t <= a)]
    after = hpa[(t >= b) & (t <= b + 10e9)]
    if not len(before) or not len(after):
        return float("nan")
    return float(-M_PER_HPA * (np.median(after) - np.median(before)))


def line_at(segs, t, side):
    """The train segment just before (side "from") or just after (side "to") time t."""
    if side == "from":
        c = [g for g in segs if g["t0"] < t - 30e9]
        return max(c, key=lambda g: g["t0"]) if c else None
    c = [g for g in segs if g["t0"] >= t - 60e9]
    return min(c, key=lambda g: g["t0"]) if c else None


def tagged_line(tags, t, side):
    if side == "from":
        c = [x for x in tags if x[0] <= t + 10e9]
        return c[-1][2] if c else ""
    c = [x for x in tags if x[0] >= t - 5e9]
    return c[0][2] if c else ""


STEP_OUT_S = 30.0


def build(ride, pairs, segs, tags, use_tags):
    d = tl.ANALYSIS / tl.RIDES[ride]
    clock = tl.clock_of(ride)
    rows = read(d / "status.csv")
    status = {"t": np.array([int(r["t_ns"]) for r in rows]), "rows": rows}
    walks = np.array([int(r["t_ns"]) for r in read(d / "steps.csv") if r["steps"] == "true"])
    press_rows = np.loadtxt(tl.ANALYSIS / "trainline" / f"press-{ride}.csv", delimiter=",", usecols=(0, 2))
    press = (press_rows[:, 0], press_rows[:, 1])
    out = []
    for a, b, station in pairs:
        walking = float(((walks >= a) & (walks < b)).sum() * STEP_WINDOW_S)
        total = (b - a) / 1e9
        h = height_m(press, a, b)
        fs, ts = line_at(segs, a, "from"), line_at(segs, b, "to")
        fl = tagged_line(tags, a, "from") if use_tags else ""
        tl_ = tagged_line(tags, b, "to") if use_tags else ""
        fmt = lambda t: clock(t).strftime("%-I:%M:%S %p").lower()  # noqa: E731
        out.append({
            "station": station or nearest_status(status, a)["station"],
            "from_line": fl or (fs["route"] if fs else ""), "from_dir": fs["dir"] if fs else "",
            "from_cars": nearest_status(status, a - 5e9)["cars"], "from_car": car_beacon(ride, a - 120e9, a),
            "to_line": tl_ or (ts["route"] if ts else ""), "to_dir": ts["dir"] if ts else "",
            "to_cars": nearest_status(status, b + 60e9)["cars"], "to_car": car_beacon(ride, b, b + 120e9),
            "alighted": fmt(a), "boarded": fmt(b), "alighted_ns": a, "boarded_ns": b,
            "total_s": round(total, 1), "walking_s": round(min(walking, total), 1),
            "standing_s": round(max(0.0, total - walking), 1),
            "height_m": round(h, 1),
            "kind": "passage" if (abs(h) >= PASSAGE_M or walking >= PASSAGE_WALK_S) else "same platform",
        })
        # Off and back on within `STEP_OUT_S` is the same train (`StopRecord::stepped_out` in Rust):
        # into another car (82 St, `184134`, 7:07 pm, `a199` to `a138`), or the same one, not a transfer.
        if total < STEP_OUT_S:
            row = out[-1]
            row["to_line"], row["to_dir"] = row["from_line"], row["from_dir"]
            cars = row["from_car"], row["to_car"]
            row["kind"] = "car change" if all(cars) and cars[0] != cars[1] else "stepped out"
    return out


def pairs_from(records, alight_key, board_key):
    """(alighted, boarded, station) for each walk off followed by a walk on within `MAX_TRANSFER_S`."""
    moves = []
    for r in records:
        if r.get(alight_key):
            moves.append((int(r[alight_key]), "off", r.get("station", "")))
        if r.get(board_key):
            moves.append((int(r[board_key]), "on", r.get("station", "")))
    moves.sort()
    out = []
    for (t, kind, st), nxt in zip(moves, moves[1:]):
        if kind == "off" and nxt[1] == "on" and nxt[0] - t <= MAX_TRANSFER_S * 1e9:
            out.append((t, nxt[0], st or nxt[2]))
    return out


def write(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, COLUMNS)
        w.writeheader()
        w.writerows(rows)


def main(rides):
    patterns, pos, _ = tl.load_gtfs()
    for ride in rides:
        d = tl.ANALYSIS / tl.RIDES[ride]
        segs = tl.identify(ride, patterns, pos)
        tags = [(t, txt, line) for t, txt, line in tl.tags(ride)]
        sensed = build(ride, pairs_from(read(d / "stops.csv"), "alighted_ns", "boarded_ns"), segs, tags, False)
        # The rider's own boarded and alighted taps set the boundaries.
        taps = [{"alighted_ns": e["t_ns"]} if e["kind"] == "KIND_ALIGHTED" else {"boarded_ns": e["t_ns"]}
                for e in read(d / "events.csv") if e["kind"] in ("KIND_ALIGHTED", "KIND_BOARDED")]
        tapped = build(ride, pairs_from(taps, "alighted_ns", "boarded_ns"), segs, tags, True)
        write(d / "transfers.csv", sensed)
        write(d / "transfers_tapped.csv", tapped)
        print(f"== {ride}")
        for name, rows in (("sensors", sensed), ("taps", tapped)):
            for r in rows:
                print(f"  {name:7s} {r['station'] or '?':24.24s} {r['from_line'] or '?'} ({r['from_cars'] or '?'}, car {r['from_car'] or '-'})"
                      f" -> {r['to_line'] or '?'} ({r['to_cars'] or '?'}, car {r['to_car'] or '-'}): {r['alighted']} to {r['boarded']},"
                      f" {r['total_s']:.0f} s ({r['walking_s']:.0f} walking), {r['height_m']:+.1f} m, {r['kind']}")


if __name__ == "__main__":
    main(sys.argv[1:] or list(tl.RIDES))
