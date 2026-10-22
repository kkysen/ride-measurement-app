"""Speed from fixed radios: how long a station's own Wi-Fi (`station_wifi.radio_stations`) stays the strongest
heard, against the replayed fusion's forward speed then. At a constant coverage length L, a pass at speed v
lasts L / v, so if L is about the same at every station, the duration gives the speed.
Each run is a stop (a "doors open" or "train stopped" tap within 10 s, or the fused speed under 1 m/s) or a pass.
Prints each run's length, its scans, the fused speed, and the implied L = v * length, then L's spread on passes.

On the 7 rides (2026-10-22), not usable: of 171 passes with a sane fused speed (1 to 25 m/s), 129 are one 4 s scan;
L spreads 31 to 92 m (IQR), and 1 / length barely follows the speed (correlation 0.20 with 2+ scans, 8 to 9 m/s rms).
The registered LTE cell, polled every 1 s, is no better: 31 passes, a 6 s median stay, correlation -0.14, 11 m/s rms;
handovers follow the modem's hysteresis, not the platform's edges. It needs faster Wi-Fi scans or RTT ranging.

Needs `wifi.csv`, `events.csv`, and `fused.csv` per ride in `target/analysis`."""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS  # noqa: E402
from station_wifi import RIDES, radio_stations, sequence, taps  # noqa: E402


def fused(ride):
    with open(ANALYSIS / ride / "fused.csv") as f:
        rows = [(int(r["t_ns"]), float(r["forward"]) - float(r["backward"])) for r in csv.DictReader(f)]
    a = np.array(rows, float)
    return a[:, 0], np.abs(a[:, 1])


def main():
    of = radio_stations()
    passes, stops = [], []
    for ride in RIDES:
        ft, fv = fused(ride)
        stop_taps = taps(ride, {"DOORS_OPEN", "TRAIN_STOPPED"})
        print(f"\n{ride}: length s, scans, mean fused m/s, min, kind, implied L m")
        for a, b, s, n in sequence(ride, of):
            # A run's true span is about a scan (4 s) longer than first to last sighting.
            length = (b - a) * 1e-9 + 4.0
            m = (ft >= a - 2e9) & (ft <= b + 2e9)
            if not m.any():
                continue
            v, vmin = fv[m].mean(), fv[m].min()
            stop = vmin < 1.0 or any(a - 10e9 <= x <= b + 10e9 for x in stop_taps)
            implied = v * length
            (stops if stop else passes).append((ride, length, n, v, implied))
            print(f"  {length:5.0f} {n:3d} {v:5.1f} {vmin:5.1f} {'stop' if stop else 'pass'} {implied:6.0f}")
    if passes:
        p = np.array([x[1:] for x in passes])
        good = p[:, 1] >= 2
        print(f"\npasses: {len(p)}, with 2+ scans {good.sum()}")
        for name, q in (("all", p), ("2+ scans", p[good])):
            if len(q):
                L = q[:, 3]
                print(f"  {name}: implied L median {np.median(L):.0f} m, IQR {np.percentile(L, 25):.0f} to {np.percentile(L, 75):.0f} m,"
                      f" speed from median L vs fused: rms {np.sqrt(np.mean((np.median(L) / q[:, 0] - q[:, 2]) ** 2)):.1f} m/s"
                      f" (fused mean {q[:, 2].mean():.1f})")


if __name__ == "__main__":
    main()
