"""Stations from radio: windows where MTA Wi-Fi (Boldyn, OFMTA, NYCTA, Passpoint) is seen above -85 dBm, or the
registered cell is above -100 dBm, while GNSS has no fix (underground). Each window's length tells a stop
(tens of seconds) from a pass (a few). Listed against the manual events nearby."""

import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, events, imu  # noqa: E402

MTA = ("Boldyn", "OFMTA", "NYCTA", "Passpoint")


def windows(name, gap=8):
    _, _, _, t0 = imu(name, 1.0)
    rd = lambda f: list(csv.DictReader(open(ANALYSIS / name / f)))
    T = lambda r: (int(r["t_ns"]) - t0) * 1e-9
    fixes = np.array(sorted(T(r) for r in rd("fixes.csv")))
    underground = lambda t: not len(fixes) or np.min(np.abs(fixes - t)) > 10
    hits = [(T(r), "w") for r in rd("wifi.csv") if any(m in r["ssid"] for m in MTA) and int(r["dbm"]) > -85]
    hits += [(T(r), "c") for r in rd("cell.csv") if r["registered"] == "true" and -100 < int(r["dbm"]) < 0]
    hits = sorted((t, k) for t, k in hits if underground(t))
    out = []
    for t, k in hits:
        if out and t - out[-1][1] <= gap:
            out[-1][1] = t
            out[-1][2].add(k)
        else:
            out.append([t, t, {k}])
    return out, events(name, t0), t0


if __name__ == "__main__":
    for name in sys.argv[1:]:
        ws, ev, _ = windows(name)
        print(f"== {name}: {len(ws)} underground radio windows")
        used = set()
        for a, b, kinds in ws:
            near = [(t, k, x) for t, k, x in ev if a - 20 <= t <= b + 15]
            used |= {t for t, _, _ in near}
            mk = "; ".join(f"{t - a:+.0f} {k.lower()} {x}".strip() for t, k, x in near)
            print(f"  {a:6.0f} s  {b - a:4.0f} s {''.join(sorted(kinds)):2s} {'STOP' if b - a >= 15 else 'pass':4s}  {mk}")
        missed = [(round(t), x) for t, k, x in ev if k == "AT_STATION" and t not in used]
        print("  station presses with no window:", missed)
