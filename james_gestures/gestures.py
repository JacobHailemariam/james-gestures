"""Gesture reading from 21 MediaPipe hand landmarks, built on the JAMES gesture stack.

Pipeline per frame (all geometry, no model, no network):
  1. motion.OneEuro       smooths the 21 points (steady at rest, no lag when moving)
  2. pinch ratio          thumb-index gap / palm size, with hysteresis (from JAMES LENS)
  3. handpose.PoseTracker palm | point | pinch | peace, per-finger hysteresis + short confirm times,
                          and "a pinch that began in a fist is not a pinch" (so opening a fist never
                          fires the claw)
  4. handpose.openness    0 fist .. 1 open hand; fist = openness below FIST_OPENNESS
  5. motion.Swipe         left / right / up / down flicks, once each

Input:  a hand as 21 (x, y) pixel points in MediaPipe order (0 wrist, 4 thumb tip, 8 index tip, ...)
Output: a GestureFrame. Every threshold is relative to palm size, so distance to the camera doesn't matter.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from . import handpose as HP
from .motion import OneEuro, Swipe, Trail

WRIST, THUMB_TIP, INDEX_MCP, INDEX_TIP, MID_MCP = 0, 4, 5, 8, 9
Point = tuple[float, float]


@dataclass
class GestureConfig:
    pinch_ratio: float = 0.33       # thumb-index gap / palm size below this = pinch (raise a bit for gloves)
    pinch_release: float = 1.4      # hysteresis: open again only above pinch_ratio * this
    fist_openness: float = 0.15     # pose palm with openness below this = fist ...
    fist_max_ratio: float = 1.10    # ... AND every finger fully folded (tip/knuckle ratio at or below this).
                                    # A pinch with the other fingers curled has a low openness too, but its index
                                    # is only half bent (~1.2+), so it no longer counts as a fist / STOP.
                                    # Real fists measure ~0.8-1.0. Watch the HUD numbers and tune if needed.
    open_openness: float = 0.80     # openness above this (and pose "palm") = open palm ...
    open_release: float = 0.30      # ... and it stays open until openness drops below this
    smooth_min_cutoff: float = 1.0  # 1-euro: lower = steadier at rest
    smooth_beta: float = 0.01       # 1-euro: higher = less lag on fast moves
    swipes: bool = True


@dataclass
class GestureFrame:
    hand: list[Point] | None = None   # smoothed points
    fingertip: Point | None = None
    norm: Point | None = None         # fingertip in 0..1 frame coordinates (x right, y down)
    palm_px: float = 0.0              # wrist to middle knuckle: a stand-in for hand size / distance
    openness: float = 0.0             # 0 fist .. 1 open
    ratios: list[float] | None = None # per finger tip/knuckle ratio, index..pinky (~0.8 folded .. ~1.9 straight)
    pose: str | None = None           # palm | point | pinch | peace (confirmed)
    pinch_ratio: float | None = None
    pinch: bool = False
    point: bool = False
    open_palm: bool = False
    fist: bool = False
    peace: bool = False
    swipe: str | None = None          # left | right | up | down, on the frame it happens
    direction: Point | None = None    # unit vector index knuckle -> tip (image coords)
    angle_deg: float | None = None    # 0 = right, 90 = up

    @property
    def name(self) -> str:
        if self.swipe:
            return f"swipe {self.swipe}"
        for n in ("pinch", "fist", "peace", "point", "open_palm"):
            if getattr(self, n):
                return n
        return "hand" if self.hand else "none"


class GestureReader:
    def __init__(self, cfg: GestureConfig | None = None, clock=time.monotonic):
        self.cfg = cfg or GestureConfig()
        self.clock = clock
        self.poses = HP.PoseTracker()
        self.smooth = OneEuro(self.cfg.smooth_min_cutoff, self.cfg.smooth_beta)
        self.trail, self.swiper = Trail(), Swipe()
        self._pinched = False
        self._open = False

    def reset(self) -> None:
        self._pinched = self._open = False
        self.poses.reset()
        self.smooth.reset()
        self.trail.push(0.0, None)

    def read(self, hand: list[Point] | None, frame_w: int = 1, frame_h: int = 1, t: float | None = None) -> GestureFrame:
        t = self.clock() if t is None else t
        g = GestureFrame()
        if not HP.valid(hand):
            self.reset()
            return g
        c = self.cfg
        sm = self.smooth([(float(x), float(y)) for x, y in hand[:21]], t)
        pts = [(float(x), float(y)) for x, y in sm]
        g.hand, g.fingertip = pts, pts[INDEX_TIP]
        g.norm = (pts[INDEX_TIP][0] / max(frame_w, 1), pts[INDEX_TIP][1] / max(frame_h, 1))
        palm = HP.palm_size(pts)
        g.palm_px = palm
        g.openness = HP.openness(pts)
        g.ratios = HP.ratios(pts)

        g.pinch_ratio = math.dist(pts[THUMB_TIP], pts[INDEX_TIP]) / palm
        self._pinched = g.pinch_ratio < c.pinch_ratio * (c.pinch_release if self._pinched else 1.0)

        g.pose = self.poses.update(pts, self._pinched, t)
        # PoseTracker reports a fist as "palm" (closed); openness tells them apart. A pinch that began in a
        # fist is latched off inside PoseTracker, so curling the hand never closes the claw by accident.
        g.fist = (g.pose == "palm" and g.openness < c.fist_openness
                  and max(g.ratios) <= c.fist_max_ratio)
        g.pinch = g.pose == "pinch"
        g.point = g.pose == "point"
        g.peace = g.pose == "peace"
        self._open = g.pose == "palm" and g.openness > (c.open_release if self._open else c.open_openness)
        g.open_palm = self._open

        dx, dy = pts[INDEX_TIP][0] - pts[INDEX_MCP][0], pts[INDEX_TIP][1] - pts[INDEX_MCP][1]
        n = math.hypot(dx, dy)
        if n > 1e-6:
            g.direction = (dx / n, dy / n)
            g.angle_deg = math.degrees(math.atan2(-dy, dx)) % 360

        if c.swipes:
            self.trail.push(t, HP.palm_centre(pts))
            g.swipe = self.swiper(self.trail, t, palm)
        return g
