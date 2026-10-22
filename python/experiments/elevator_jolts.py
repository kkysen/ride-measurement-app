"""Elevator or escalator, for every "riding" height change (`elevator.moves`): an elevator car accelerates and brakes
vertically, a pulse in |a| low-passed at 1 Hz (about 0.3 to 1 m/s^2 for a second) at the start of the climb and the
opposite one at its end; an escalator has none, just a steady climb at its 0.5 m/s along a 30 degree incline
(0.25 m/s up, unless walked). Prints, per change, the vertical speed from the barometer and the low-passed |a|'s
peak deviation from its median in the 6 s around the start and the end of the steepest 10 m.

Result (2026-10-22, 10 changes): no separation. The PABT escalators to Gate 301 (`002914`) show 0.5 to 1.5 m/s^2
and the home elevator at rides' ends 0.5 to 1.9: a handheld phone's own motion swamps the car's pulse.
Vertical speed overlaps too (escalators 0.15 to 0.24 m/s, the elevator 0.24 to 0.47). Next: the jolt along gravity
only (gyro-tracked), and the escalator's steady step-tread vibration."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ride  # noqa: E402
from elevator import moves  # noqa: E402
from sensor_trips import walking  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def altitude(session):
    p = ride.read_stream(session, "pressure")
    t, v = p.timestamps_ns, p.values[:, 0].astype(float)
    sec = np.arange(t[0], t[-1], 0.5e9)
    idx = np.searchsorted(t, sec)
    med = np.array([np.median(v[a:b]) if b > a else np.nan for a, b in zip(idx, np.append(idx[1:], len(t)))])
    return sec, -8.3 * (med - np.nanmedian(med))


def lowpass_norm(session):
    a = ride.read_stream(session, "accel_uncal")
    t = a.timestamps_ns
    x = np.linalg.norm(a.values[:, :3].astype(float) - a.values[:, 3:6], axis=1)
    dt = np.median(np.diff(t)) * 1e-9
    k = max(1, int(round(1.0 / dt)))
    return t, np.convolve(x, np.ones(k) / k, mode="same")


def ends(sec, alt, t0, dz):
    """Start and end of the climb: where the height leaves 10% and reaches 90% of the change around t0."""
    m = (sec >= t0 - 30e9) & (sec <= t0 + 60e9)
    s, z = sec[m], alt[m]
    z0 = np.nanmedian(z[:10])
    rel = (z - z0) / dz
    a = s[np.argmax(rel > 0.1)]
    b = s[np.argmax(rel > 0.9)]
    return a, b


def jolt(t, x, at):
    m = (t >= at - 3e9) & (t <= at + 3e9)
    ref = (t >= at - 12e9) & (t <= at + 12e9)
    if m.sum() < 10:
        return np.nan
    return np.max(np.abs(x[m] - np.median(x[ref])))


if __name__ == "__main__":
    for s in sorted((ROOT / "data/sessions").iterdir()):
        try:
            walks = walking(s)
            mv = [m for m in moves(s, walks) if m[3] == "riding"]
        except Exception as e:  # noqa: BLE001
            print(f"{s.name}: {e}")
            continue
        if not mv:
            continue
        sec, alt = altitude(s)
        t, x = lowpass_norm(s)
        for t0, dz, peak, _ in mv:
            a, b = ends(sec, alt, t0, dz)
            vz = abs(dz) / max((b - a) * 1e-9, 1e-3) * 0.8
            print(f"{s.name} {ride.clock(t0) if hasattr(ride, 'clock') else t0} {dz:+5.1f} m, {vz:4.2f} m/s up/down, "
                  f"jolt start {jolt(t, x, a):4.2f} end {jolt(t, x, b):4.2f} m/s^2")
