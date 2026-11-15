"""When the rider boards, against walking on the platform: scored against the rider's "boarded" taps (labels only).

The stop records (`crates/fusion/src/fusion.rs`) take the last step-rhythm window before the boarded train's
departure. A step onto the train is a few steps, often under a 4 s window, so on `151449` (Times Sq 3 to 7)
it fell on the last walk along the platform, 3:38:57 pm, against the tap at 3:40:30.
Cues after the last walking, before the departure:
- the boarded train's door cycle (arrival and doors opening while the rider stands), boarding inside it;
- the boarded car's beacon (`e2c56db5...a710xxxx`) rising: inside the car its signal jumps about 10 dB;
- step windows in the cycle.
Rules scored: `last_steps` (now), `cycle` (any step in the cycle, else the cycle's middle),
`beacon` (the first reading of a beacon first heard after the last steps that is 8 dB over its first readings,
inside the cycle if one), and `combined` (steps in the cycle, else beacon, else the cycle's middle).

Needs `data/work/analysis/stops3-<ride>/{stops,steps,door_cycles,events,fixes}.csv` and `data/work/analysis/<ride>/ble.csv`."""

import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import clock_from_fixes, session_dir  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
ANALYSIS = ROOT / "data/work/analysis"
RIDES = ["151449", "154951", "184134", "214427", "135509", "155919", "182135", "215922"]
PREFIX = "e2c56db5dffb48d2b060d0f5a710"
RISE_DB = 8.0


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def ns(v):
    return int(v) if v else None


def beacon_rise(ble, t0, t1):
    """The first time a car beacon first heard after t0 rises RISE_DB over its first three readings, before t1."""
    first, best = {}, None
    for t, car, rssi in ble:
        if t < t0 - 60e9 or t > t1:
            continue
        if car not in first:
            first[car] = (t, [])
        t_first, base = first[car]
        if t_first < t0:
            continue
        if len(base) < 3:
            base.append(rssi)
            continue
        if rssi >= np.mean(base) + RISE_DB:
            best = t if best is None else min(best, t)
            continue
    return best


def main():
    errors = {k: [] for k in ("last_steps", "cycle", "beacon", "combined")}
    for ride in RIDES:
        d = ANALYSIS / f"stops3-{ride}"
        clock = clock_from_fixes(d / "fixes.csv", session_dir(ride))
        taps = [int(e["t_ns"]) for e in read(d / "events.csv") if e["kind"].endswith("BOARDED")]
        steps = [(int(r["t_ns"]), r["steps"] == "true") for r in read(d / "steps.csv")]
        cycles = [(int(r["open_ns"]), int(r["close_ns"])) for r in read(d / "door_cycles.csv")]
        ble = []
        for r in read(ANALYSIS / ride / "ble.csv"):
            i = r["data"].find(PREFIX)
            if i >= 0:
                ble.append((int(r["t_ns"]), r["data"][i + 28 : i + 32], int(r["rssi"])))
        ble.sort()
        for s in read(d / "stops.csv"):
            boarded, departed = ns(s["boarded_ns"]), ns(s["departed_ns"])
            if boarded is None or departed is None:
                continue
            # The boarded train's door cycle: the last closing before its departure, after the last steps' start.
            cyc = [c for c in cycles if c[1] <= departed + 2e9 and c[1] > boarded - 4e9]
            cyc = cyc[-1] if cyc else None
            in_cycle = [t for t, st in steps if st and cyc and cyc[0] <= t <= cyc[1] + 2e9]
            by_cycle = in_cycle[-1] if in_cycle else ((cyc[0] + cyc[1]) // 2 if cyc else boarded)
            rise = beacon_rise(ble, boarded, cyc[1] if cyc else departed)
            if rise is not None and cyc and not cyc[0] - 5e9 <= rise <= cyc[1]:
                rise = None
            by_beacon = rise if rise is not None else boarded
            combined = in_cycle[-1] if in_cycle else (rise if rise is not None else by_cycle)
            near = [t for t in taps if boarded - 120e9 <= t <= departed + 10e9]
            if not near:
                print(f"{ride} {clock(boarded)}: no tap; cycle {clock(cyc[0]) if cyc else '-'}, beacon {clock(rise) if rise else '-'}")
                continue
            tap = min(near, key=lambda t: abs(t - by_cycle))
            row = {"last_steps": boarded, "cycle": by_cycle, "beacon": by_beacon, "combined": combined}
            for k, v in row.items():
                errors[k].append((v - tap) / 1e9)
            print(f"{ride} tap {clock(tap)}: " + ", ".join(f"{k} {(v - tap) / 1e9:+.0f} s" for k, v in row.items()))
    for k, e in errors.items():
        e = np.abs(e)
        print(f"{k:10s} median {np.median(e):5.1f} s, within 10 s {np.sum(e <= 10)} of {len(e)}, worst {e.max():.0f} s")


if __name__ == "__main__":
    main()
