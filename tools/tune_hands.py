"""Summarise a hand log from `demo.py --log hands.csv` (Issue 2: real hands vs the thresholds).

    python tools/tune_hands.py hands.csv

For each gesture you labelled: how often the arm saw that gesture, what it saw instead, and the measured
finger ratios / thumb gap / openness, next to the thresholds in handpose.py and gestures.py they are
compared against. Reading it: a labelled "point" whose r_middle often sits above DOWN (1.30) is why point
reads as palm; a labelled "open" whose openness sits under open_openness (0.80) is why arming fails.
"""
from __future__ import annotations

import csv
import statistics
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from james_gestures import handpose as HP                 # noqa: E402
from james_gestures.gestures import GestureConfig          # noqa: E402

COLS = ["r_index", "r_middle", "r_ring", "r_pinky", "gap", "openness"]


def summarise(rows: list[dict]) -> str:
    c = GestureConfig()
    out = [f"thresholds: finger up > {HP.UP_ON} (stays up > {HP.UP_OFF}), down < {HP.DOWN}, "
           f"pinch gap < {c.pinch_ratio}, open palm openness > {c.open_openness}, "
           f"fist openness < {c.fist_openness} and every finger <= {c.fist_max_ratio}", ""]
    by = {}
    for r in rows:
        if r["label"]:
            by.setdefault(r["label"], []).append(r)
    if not by:
        return "\n".join(out + ["no labelled frames: hold a gesture and press 1-5 while holding it"])
    for label in ("open", "point", "peace", "pinch", "fist"):
        rs = by.get(label)
        if not rs:
            continue
        seen = Counter(r["detected"] for r in rs)
        hit = seen.get(label, 0) / len(rs)
        instead = ", ".join(f"{k} {v / len(rs):.0%}" for k, v in seen.most_common() if k != label) or "-"
        out.append(f"{label:6s} {len(rs):4d} frames   recognised {hit:4.0%}   instead: {instead}")
        for col in COLS:
            v = [float(r[col]) for r in rs]
            q = statistics.quantiles(v, n=10) if len(v) >= 10 else [min(v)] * 9
            out.append(f"         {col:9s} median {statistics.median(v):5.2f}   10%..90% {q[0]:5.2f}..{q[-1]:5.2f}"
                       f"   min {min(v):5.2f}  max {max(v):5.2f}")
        palm = [float(r["palm_px"]) / max(float(r["frame_h"]), 1) for r in rs]
        out.append(f"         hand size  median {statistics.median(palm):.3f} of frame height")
        out.append("")
    return "\n".join(out)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: python tools/tune_hands.py hands.csv")
    with open(sys.argv[1], newline="") as f:
        print(summarise(list(csv.DictReader(f))))


if __name__ == "__main__":
    main()
