"""Draw the README figures from the real code and the real webcam recording.

    python tools/make_readme_figures.py            # writes media/fig-*.png

Every number comes from the repo: hands from handpose.synth, ratios from handpose.ratios, the filter from
motion.OneEuro, thresholds from handpose / GestureConfig, the measured hands from
docs/data/hands_webcam.csv (recorded with `demo.py --log` on a laptop webcam, Oct 25 2026), and the voice
numbers from the measurement in james_gestures/voice.py's docstring.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from james_gestures import handpose as HP  # noqa: E402
from james_gestures.gestures import GestureConfig  # noqa: E402
from james_gestures.motion import OneEuro  # noqa: E402

OUT = ROOT / "media"
DATA = ROOT / "docs" / "data" / "hands_webcam.csv"

# Atlas palette (matches media/hero.jpg)
BG, INK, MUTED, GRID = "#f3ecdd", "#17130f", "#6f665a", "#e2d9c6"
RED, VIOLET, YELLOW, PINK = "#e8382b", "#5b58a6", "#f4b42a", "#ff4f7b"
FINGER_COLORS = [RED, VIOLET, YELLOW, PINK]
BONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]

plt.rcParams.update({
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "axes.edgecolor": INK, "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": INK, "ytick.color": INK, "font.size": 11, "axes.titleweight": "bold",
    "axes.spines.top": False, "axes.spines.right": False,
})


def save(fig, name):
    path = OUT / name
    fig.savefig(path, dpi=160, bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)
    print("wrote", path.relative_to(ROOT))


# ---------------------------------------------------------------- 1. landmarks and the finger ratio
def fig_landmarks():
    poses = [("open", "open palm", "arm / idle"), ("point", "point", "base + shoulder"),
             ("peace", "peace", "elbow"), ("pinch", "pinch", "claw"), ("fist", "fist", "STOP")]
    fig, axes = plt.subplots(1, 5, figsize=(13, 3.9))
    for ax, (pose, title, does) in zip(axes, poses):
        pts = np.array(HP.synth(pose, (0, 0), 120), dtype=float)
        pts[:, 1] *= -1                                       # image y points down
        r = HP.ratios(HP.synth(pose, (0, 0), 120))
        for a, b in BONES:
            ax.plot(*pts[[a, b]].T, color=INK, lw=2, solid_capstyle="round", zorder=1)
        ax.scatter(*pts.T, s=26, color=BG, edgecolor=INK, lw=1.4, zorder=2)
        for (k, _, t), col in zip(HP.FINGERS, FINGER_COLORS):  # tip-to-wrist line per finger
            ax.plot(*pts[[0, t]].T, color=col, lw=1.2, ls=(0, (3, 2)), zorder=0)
            ax.scatter(*pts[t], s=60, color=col, zorder=3)
        ax.scatter(*pts[0], s=70, color=INK, zorder=3)
        ax.set_title(title, fontsize=13, pad=4)
        ax.text(0.5, -0.06, does, transform=ax.transAxes, ha="center", va="top", color=MUTED, fontsize=10)
        txt = "  ".join(f"{v:.1f}" for v in r)
        ax.text(0.5, -0.19, txt, transform=ax.transAxes, ha="center", va="top", fontsize=10,
                family="monospace")
        ax.set_aspect("equal")
        ax.axis("off")
    fig.suptitle("21 MediaPipe landmarks  →  four finger ratios (index, middle, ring, pinky)", fontsize=14,
                 fontweight="bold", y=1.02)
    fig.text(0.5, -0.12, "ratio = |wrist → fingertip| / |wrist → knuckle|.  Straight ≈ 1.9, folded ≈ 0.8.  "
             "Scale-free: the same at any distance from the camera.", ha="center", color=MUTED, fontsize=10.5)
    save(fig, "fig-landmarks.png")


# ---------------------------------------------------------------- 2. real webcam ratios vs thresholds
def fig_real_ratios():
    rows = [r for r in csv.DictReader(open(DATA, newline="")) if r["label"]]
    order = ["open", "point", "peace", "pinch", "fist"]
    names = ["index", "middle", "ring", "pinky"]
    fig, ax = plt.subplots(figsize=(12, 4.6))
    ax.axhspan(HP.UP_ON, 2.2, color=VIOLET, alpha=0.10, lw=0)
    ax.axhspan(0.4, HP.DOWN, color=RED, alpha=0.08, lw=0)
    ax.axhspan(HP.DOWN, HP.UP_ON, color=GRID, alpha=0.9, lw=0)
    ax.axhline(HP.UP_ON, color=VIOLET, lw=1.2)
    ax.axhline(HP.UP_OFF, color=VIOLET, lw=1, ls=":")
    ax.axhline(HP.DOWN, color=RED, lw=1.2)
    ax.text(len(order) - 0.45, HP.UP_ON + 0.03, f"finger UP  > {HP.UP_ON}", color=VIOLET, ha="right", fontsize=10)
    ax.text(len(order) - 0.45, HP.UP_OFF - 0.075, f"stays up until < {HP.UP_OFF} (hysteresis)", color=VIOLET,
            ha="right", fontsize=9)
    ax.text(len(order) - 0.45, HP.DOWN - 0.09, f"finger DOWN  < {HP.DOWN}", color=RED, ha="right", fontsize=10)
    w = 0.17
    for gi, g in enumerate(order):
        rs = [r for r in rows if r["label"] == g]
        hit = sum(r["detected"] == g for r in rs) / len(rs)
        for fi, (n, col) in enumerate(zip(names, FINGER_COLORS)):
            v = np.array([float(r[f"r_{n}"]) for r in rs])
            x = gi + (fi - 1.5) * w
            ax.plot([x, x], [v.min(), v.max()], color=col, lw=6, solid_capstyle="round", alpha=0.55)
            ax.scatter([x], [np.median(v)], color=col, edgecolor=INK, s=46, zorder=3, lw=0.8)
        ax.text(gi, 2.16, f"{len(rs)} frames · {hit:.0%}", ha="center", fontsize=9.5, color=INK)
    ax.set_xticks(range(len(order)), [o if o != "open" else "open palm" for o in order], fontsize=12)
    ax.set_ylim(0.45, 2.25)
    ax.set_ylabel("finger ratio")
    ax.set_xlim(-0.6, len(order) - 0.4)
    for n, col in zip(names, FINGER_COLORS):
        ax.scatter([], [], color=col, label=n, s=40)
    ax.legend(loc="lower left", ncol=4, frameon=False, fontsize=10, bbox_to_anchor=(0, -0.3))
    ax.set_title("Real webcam, real hand: every finger lands clear of the thresholds", loc="left", pad=22)
    save(fig, "fig-real-ratios.png")


# ---------------------------------------------------------------- 3. the 1-euro filter
def fig_one_euro():
    rng = np.random.default_rng(3)
    fps, T = 30, 4.0
    t = np.arange(0, T, 1 / fps)
    true = np.where(t < 1.5, 200.0, np.where(t < 2.3, 200 + (t - 1.5) / 0.8 * 180, 380.0))
    noisy = true + rng.normal(0, 4.0, size=t.size)
    f = OneEuro(*(GestureConfig().smooth_min_cutoff, GestureConfig().smooth_beta))
    out = np.array([f(np.array([[x, 0.0]]), ti)[0][0] for x, ti in zip(noisy, t)])
    fig, ax = plt.subplots(figsize=(12, 3.8))
    ax.plot(t, noisy, color=MUTED, lw=1, alpha=0.8, label="raw MediaPipe fingertip (±4 px jitter)")
    ax.plot(t, out, color=RED, lw=2.6, label="1-euro filtered")
    ax.plot(t, true, color=INK, lw=1, ls="--", label="where the hand really is")
    ax.axvspan(0, 1.5, color=VIOLET, alpha=0.07, lw=0)
    ax.axvspan(2.3, T, color=VIOLET, alpha=0.07, lw=0)
    ax.text(0.75, 222, "still hand: heavy smoothing,\nthe arm holds steady", ha="center", fontsize=10, color=VIOLET)
    ax.text(1.9, 330, "fast move:\nsmoothing falls away,\nno lag", ha="center", fontsize=10, color=RED)
    still = slice(int(0.5 * fps), int(1.5 * fps))
    ax.text(3.15, 300, f"jitter at rest\nraw  {noisy[still].std():.1f} px\nfiltered  {out[still].std():.1f} px",
            fontsize=10, family="monospace", ha="center")
    ax.set_xlabel("seconds")
    ax.set_ylabel("x position (px)")
    ax.legend(loc="upper left", frameon=False, fontsize=10)
    ax.set_title("1-euro filter (Casiez et al., CHI 2012): steady when still, instant when moving", loc="left")
    save(fig, "fig-one-euro.png")


# ---------------------------------------------------------------- 4. wake word
def fig_wake_word():
    fig, ax = plt.subplots(figsize=(8, 2.6))
    labels = ["bare command words", 'wake word "james" first']
    vals = [6.6, 0.0]
    ax.barh(labels, vals, color=[RED, VIOLET], height=0.55)
    ax.text(6.6 + 0.12, 0, "6.6 false commands / min", va="center", fontsize=11)
    ax.text(0.12, 1, "0 in 100 s", va="center", fontsize=11, color=VIOLET, fontweight="bold")
    ax.set_xlim(0, 9)
    ax.invert_yaxis()
    ax.set_xlabel("false voice commands per minute, Cal Hacks hall, nobody talking to the laptop")
    ax.tick_params(axis="y", length=0, labelsize=11)
    ax.set_title("Offline voice in a 2,000-person hall", loc="left")
    save(fig, "fig-wake-word.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    fig_landmarks()
    fig_real_ratios()
    fig_one_euro()
    fig_wake_word()
