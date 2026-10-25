"""Record what the classifier sees on a real webcam, with the gesture you meant, for tuning (Issue 2).

    python demo.py --log hands.csv        then hold a gesture and press its key while holding it:
        1 open palm   2 point   3 peace   4 pinch   5 fist   0 stop labelling

Every frame with a hand writes one row: time, your label, what was detected, the four finger ratios,
the thumb-index gap, openness and hand size. tools/tune_hands.py turns the file into per-gesture
accuracy and the measured ranges next to the current thresholds.
"""
from __future__ import annotations

import csv

LABEL_KEYS = {ord("1"): "open", ord("2"): "point", ord("3"): "peace", ord("4"): "pinch", ord("5"): "fist",
              ord("0"): ""}
FIELDS = ["t", "label", "detected", "raw", "r_index", "r_middle", "r_ring", "r_pinky", "gap", "openness",
          "palm_px", "frame_h"]


def detected(g) -> str:
    """The one gesture name the arm acts on for this frame (same vocabulary as the labels)."""
    for name, flag in (("fist", g.fist), ("pinch", g.pinch), ("peace", g.peace), ("point", g.point),
                       ("open", g.open_palm)):
        if flag:
            return name
    return "other"


class HandLog:
    def __init__(self, path: str):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(FIELDS)
        self.label = ""
        self.rows = 0

    def key(self, k: int) -> bool:
        """Handle a key press; True if it was a label key."""
        if k in LABEL_KEYS:
            self.label = LABEL_KEYS[k]
            return True
        return False

    def write(self, t: float, g, raw: str | None, frame_h: int) -> None:
        if not g.hand or not g.ratios:
            return
        self._w.writerow([f"{t:.3f}", self.label, detected(g), raw or "", *(f"{r:.3f}" for r in g.ratios),
                          f"{g.pinch_ratio:.3f}", f"{g.openness:.3f}", f"{g.palm_px:.1f}", frame_h])
        self.rows += 1

    def close(self) -> None:
        self._f.close()
