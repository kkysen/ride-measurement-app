"""Applies the stopped/moving model trained on the elevated 7 (`154951`) to the underground ride (`151449`),
and shows it against the manual events."""

import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import stopclass as sc  # noqa: E402  (fits nothing at import beyond its own report; reuses its helpers)
from common import ANALYSIS, events, imu  # noqa: E402

COLS = [3, 4, 5, 9, 10]
train = np.load(ANALYSIS / "154951" / "vibration.npy")
sp, ph = train[:, 1], train[:, 2]
lab = ((ph == 0) & (sp < 5)) | (sp > 15)
Xtr = sc.smoothed(train[:, COLS], 5, True)
model = sc.fit(Xtr[lab], (sp[lab] > 15).astype(float))

test = np.load(ANALYSIS / "151449" / "vibration.npy")
p = model(sc.smoothed(test[:, COLS], 5, True))
t = test[:, 0]
_, _, _, t0 = imu("151449", 1.0)
ev = events("151449", t0)
line = []
for i in range(len(t)):
    if t[i] < 950:
        continue
    marks = [f"{k.lower()} {x}".strip() for (te, k, x) in ev if t[i] <= te < t[i] + 1]
    line.append(("S" if p[i] < 0.5 else "."))
    if marks or len(line) == 60:
        print(f"{t[i] - len(line) + 1:6.0f} s {''.join(line):60s} {'; '.join(marks)}")
        line = []
print("S = called stopped (P(moving) < 0.5), . = moving, one character per second")
