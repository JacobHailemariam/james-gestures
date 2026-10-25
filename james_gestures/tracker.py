"""MediaPipe Hand Landmarker wrapper: camera frame in, the driving hand's 21 points out.

Only ONE hand drives the arm: the largest one in view (the closest to the camera). Every other
hand is returned in `others` so a HUD can draw it faded, but it never produces commands.
This is the same rule JAMES uses when its owner key is switched off.

Needs: pip install mediapipe opencv-python, and the model file hand_landmarker.task
(it ships in the repo root; pass another path with --model).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

Point = tuple[float, float]


@dataclass
class TrackResult:
    hand: list[Point] | None = None
    handedness: str = ""                          # "Left" / "Right" as MediaPipe reports it
    others: list[list[Point]] = field(default_factory=list)


def largest(hands: list[list[Point]]) -> int | None:
    """Index of the biggest hand (wrist to middle MCP), i.e. the one nearest the camera."""
    if not hands:
        return None
    return max(range(len(hands)), key=lambda i: math.dist(hands[i][0], hands[i][9]))


class HandTracker:
    def __init__(self, model_path: str | Path = "hand_landmarker.task", max_hands: int = 2,
                 det_conf: float = 0.5, track_conf: float = 0.5):
        import mediapipe as mp
        from mediapipe.tasks import python as mpt
        from mediapipe.tasks.python import vision

        model_path = Path(model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"{model_path} not found: run from the repo root, or pass --model <path>")
        self._mp = mp
        opts = vision.HandLandmarkerOptions(
            base_options=mpt.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO, num_hands=max_hands,
            min_hand_detection_confidence=det_conf, min_tracking_confidence=track_conf)
        self._lm = vision.HandLandmarker.create_from_options(opts)
        self._t0 = time.monotonic()
        self._last_ms = -1

    def process(self, frame_bgr) -> TrackResult:
        import cv2
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        ts = int((time.monotonic() - self._t0) * 1000)
        ts = max(ts, self._last_ms + 1)            # VIDEO mode needs strictly increasing timestamps
        self._last_ms = ts
        out = self._lm.detect_for_video(self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb), ts)
        hands = [[(p.x * w, p.y * h) for p in lm] for lm in (out.hand_landmarks or [])]
        handed = [(hd[0].category_name if hd else "") for hd in (out.handedness or [])]
        i = largest(hands)
        if i is None:
            return TrackResult()
        return TrackResult(hands[i], handed[i] if i < len(handed) else "",
                           [hd for j, hd in enumerate(hands) if j != i])

    def close(self) -> None:
        try:
            self._lm.close()
        except Exception:
            pass
