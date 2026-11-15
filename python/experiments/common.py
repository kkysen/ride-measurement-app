"""Shared loading for offline experiments: IMU at a fixed rate, fixes, events, and the replayed fusion."""

import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ride  # noqa: E402

SESSIONS = Path("/mnt/c/Users/kkyse/OneDrive/Khyber/Documents/RideLogger/sessions")
ANALYSIS = Path(__file__).resolve().parents[2] / "data/work/analysis"


LOCAL = Path(__file__).resolve().parents[2] / "data/sessions"


def session_dir(name):
    """The session pulled into `data/sessions/` (`just pull`), else the OneDrive copy."""
    for root in (LOCAL, SESSIONS):
        found = sorted(root.glob(f"2026*-{name}")) if root.exists() else []
        if found:
            return found[0]
    return LOCAL / f"20261020-{name}"


def imu(name, rate=50.0):
    """Accelerometer (m/s^2) and gyro (rad/s), offsets removed, averaged into bins of 1/rate s: (t_s, acc, gyro, t0_ns)."""
    d = session_dir(name)
    a = ride.read_stream(d, "accel_uncal")
    g = ride.read_stream(d, "gyro_uncal")
    t0 = a.timestamps_ns[0]
    acc = a.values[:, :3].astype(float) - a.values[:, 3:6]
    gyr = g.values[:, :3].astype(float) - g.values[:, 3:6]
    n = int((a.timestamps_ns[-1] - t0) * 1e-9 * rate)
    def binned(ts, v):
        i = ((ts - t0) * 1e-9 * rate).astype(np.int64)
        ok = (i >= 0) & (i < n)
        s = np.zeros((n, 3))
        c = np.zeros(n)
        np.add.at(s, i[ok], v[ok])
        np.add.at(c, i[ok], 1)
        out = s / np.maximum(c, 1)[:, None]
        # Fill empty bins from the previous one.
        for k in np.nonzero(c == 0)[0]:
            if k:
                out[k] = out[k - 1]
        return out
    return (np.arange(n) + 0.5) / rate, binned(a.timestamps_ns, acc), binned(g.timestamps_ns, gyr), t0


def read_csv(name, file):
    with open(ANALYSIS / name / file) as f:
        return list(csv.DictReader(f))


def fixes(name, t0):
    rows = read_csv(name, "fixes.csv")
    num = lambda r, k: float(r[k]) if r[k] else np.nan
    return {k: np.array([num(r, k) for r in rows]) for k in ["lat", "lon", "speed", "speed_acc", "h_acc"]} | {
        "t": np.array([(int(r["t_ns"]) - t0) * 1e-9 for r in rows])
    }


def events(name, t0):
    return [((int(r["t_ns"]) - t0) * 1e-9, r["kind"].removeprefix("KIND_"), r["text"]) for r in read_csv(name, "events.csv")]


def fused(name, t0):
    rows = read_csv(name, "fused.csv")
    return {k: np.array([float(r[k]) for r in rows]) for k in ["speed", "sigma", "forward", "backward", "accel"]} | {
        "t": np.array([(int(r["t_ns"]) - t0) * 1e-9 for r in rows])
    }


def clock_from_fixes(fixes_csv, session, pattern="%-I:%M:%S %P"):
    """Boot-time ns to the ride's own local clock text (Rust's `SessionClock`, never a fixed zone).

    The boot-to-Unix offset comes from the first fix with a wall time in `fixes_csv`; `session` is the session directory."""
    row = next(r for r in csv.DictReader(open(fixes_csv)) if r["time_ms"])
    off_ms = int(row["time_ms"]) - int(row["t_ns"]) // 1_000_000
    return lambda t_ns: ride.local_clock(session, int(t_ns) // 1_000_000 + off_ms, pattern)
