"""The train beacons (iBeacon, UUID `e2c56db5-dffb-48d2-b060-d0f5a710xxxx`): per ride, each device's ID (the UUID's last
two bytes, one per address), how often and how long it was seen, and its "major" and "minor" fields decoded as they
look: major two signed bytes and minor's first byte near (0, 0, -64), an accelerometer at 64 counts per g lying flat;
minor's second byte is 0x64 (100, a battery percent?); the last byte is the 1 m power (-59 dBm).
`96e0` is the UUID's well-known default (Apple's AirLocate sample), some other beacon.

On `184134` (2026-10-22): 2 to 5 IDs are seen steadily at once for the same span (`a199`, `a138`, `a1f6` for
820 to 830 s from 6:56 pm, with `a139`, `a1d2` for about 145 s; then `9888`, `99ae`, `99ee` for 347 s from 7:10 pm):
one per car, nearer cars stronger, not one per train. IDs recur across rides (`a199`, `a1f6` at 4:11 pm on
`154951` and again at 6:56 pm on `184134`; `a14e` on `151449` and `154951`), so each names a physical car.
Decoded, the axes are 1 g long (64 counts): the `a1xx` devices (the 7's R188s) lie flat (z about -64); the `97xx` to
`9exx` ones (from 7:10 pm, another fleet) stand on edge (x about +-64), the sign flipping between cars facing
opposite ways. Each car's own accelerometer: in principle the car's motion, at the beacon's rate (about 1 Hz here).
Needs `ble.csv` and `fixes.csv` per ride in `data/work/analysis`."""

import csv
import sys
from collections import defaultdict

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, clock_from_fixes, session_dir  # noqa: E402

PREFIX = "e2c56db5dffb48d2b060d0f5a710"


def signed(b):
    return b - 256 if b >= 128 else b


def main(rides):
    for ride in rides:
        clock = clock_from_fixes(ANALYSIS / ride / "fixes.csv", session_dir(ride))
        seen = defaultdict(dict)
        for r in csv.DictReader(open(ANALYSIS / ride / "ble.csv")):
            i = r["data"].find(PREFIX)
            if i < 0:
                continue
            p = bytes.fromhex(r["data"][i + len(PREFIX) : i + len(PREFIX) + 14])
            if len(p) < 7:
                continue
            seen[(p[:2].hex(), r["address"])][int(r["t_ns"])] = (signed(p[2]), signed(p[3]), signed(p[4]), p[5], signed(p[6]))
        print(f"== {ride}")
        for (dev, addr), v in sorted(seen.items(), key=lambda kv: min(kv[1])):
            if len(v) < 20:
                continue
            ts = sorted(v)
            axes = [sum(x[k] for x in v.values()) / len(v) for k in range(3)]
            print(f"  {dev} {addr}: {len(v):4d} from {clock(ts[0])} for {(ts[-1] - ts[0]) / 1e9:4.0f} s, "
                  f"mean (x, y, z) ({axes[0]:+.1f}, {axes[1]:+.1f}, {axes[2]:+.1f}), "
                  f"byte {sorted({x[3] for x in v.values()})[:3]}, power {sorted({x[4] for x in v.values()})[:2]}")


if __name__ == "__main__":
    main(sys.argv[1:] or ["151449", "154951", "184134", "135509", "155919"])
