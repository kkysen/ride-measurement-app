"""Radio around the underground ride: per 10 s, BLE devices seen (and the commonest manufacturer IDs / service
UUIDs), Wi-Fi networks seen, and the registered cell's signal and service state, against the manual events."""

import collections
import csv
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from common import ANALYSIS, events, imu  # noqa: E402

name = sys.argv[1] if len(sys.argv) > 1 else "151449"
t_from = float(sys.argv[2]) if len(sys.argv) > 2 else 950
_, _, _, t0 = imu(name, 1.0)
rd = lambda f: list(csv.DictReader(open(ANALYSIS / name / f)))
ble, wifi, cell, svc = rd("ble.csv"), rd("wifi.csv"), rd("cell.csv"), rd("service.csv")
T = lambda r: (int(r["t_ns"]) - t0) * 1e-9


def kinds(hexdata):
    """Manufacturer company IDs and 16-bit service UUIDs in an advertisement."""
    b = bytes.fromhex(hexdata)
    out, i = [], 0
    while i < len(b) and b[i]:
        n, t = b[i], b[i + 1] if i + 1 < len(b) else 0
        d = b[i + 2 : i + 1 + n]
        if t == 0xFF and len(d) >= 2:
            out.append(f"m{d[0] | d[1] << 8:04x}")
        if t in (0x16,) and len(d) >= 2:
            out.append(f"s{d[0] | d[1] << 8:04x}")
        if t in (0x02, 0x03) and len(d) >= 2:
            out.append(f"u{d[0] | d[1] << 8:04x}")
        i += n + 1
    return out


ev = events(name, t0)
allk = collections.Counter(k for r in ble for k in set(kinds(r["data"])))
print("BLE advertisement kinds over the ride:", allk.most_common(15))
ssids = collections.Counter(r["ssid"] for r in wifi if T(r) > t_from)
print("Wi-Fi SSIDs:", ssids.most_common(15))
end = max(T(r) for r in ble)
for w in np.arange(t_from, end, 10):
    bl = [r for r in ble if w <= T(r) < w + 10]
    wf = [r for r in wifi if w <= T(r) < w + 10]
    cl = [r for r in cell if w <= T(r) < w + 10 and r["registered"] == "true"]
    sv = [r["state"] for r in svc if w <= T(r) < w + 10]
    devs = len({r["address"] for r in bl})
    top = collections.Counter(k for r in bl for k in set(kinds(r["data"]))).most_common(2)
    nets = {r["ssid"] for r in wf}
    best = max((int(r["dbm"]) for r in cl), default=None)
    mk = "; ".join(f"{k.lower()} {x}".strip() for te, k, x in ev if w <= te < w + 10)
    print(f"{w:6.0f} ble {devs:3d} {str(top)[:34]:34s} wifi {len(wf):3d} {str(sorted(nets))[:40]:40s} cell {best} svc {','.join(sorted(set(sv)))}  {mk}")
