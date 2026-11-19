"""Draw the PPO figures for the README from the logged training runs and the evaluation.

    python tools/make_ppo_figures.py          # reads docs/data/ppo_train_seed*.csv and ppo_eval.json, writes media/fig-ppo.png
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import make_readme_figures as F  # noqa: E402  (same palette and style as the other README figures)

DATA = ROOT / "docs" / "data"


def smooth(y: np.ndarray, k: int = 15) -> np.ndarray:
    k = max(1, min(k, len(y)))
    return np.convolve(np.pad(y, (k - 1, 0), mode="edge"), np.ones(k) / k, mode="valid")


def main() -> int:
    runs = []
    for p in sorted(DATA.glob("ppo_train_seed*.csv")):
        rows = list(csv.DictReader(open(p, newline="")))
        runs.append({k: np.array([float(r[k]) for r in rows]) for k in ("env_steps", "success_rate", "collision_rate")})
    ev = json.loads((DATA / "ppo_eval.json").read_text(encoding="utf-8"))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.4), gridspec_kw={"width_ratios": [1.25, 1]})
    # 1. learning curves, one line per seed (training episodes, stochastic policy)
    for i, r in enumerate(runs):
        x = r["env_steps"] / 1e6
        ax1.plot(x, 100 * smooth(r["success_rate"]), color=F.VIOLET, lw=2, alpha=0.9,
                 label="reached the target" if i == 0 else None)
        ax1.plot(x, 100 * smooth(r["collision_rate"]), color=F.RED, lw=1.6, alpha=0.9,
                 label="touched the wearer or bag" if i == 0 else None)
    ax1.set_xlabel("environment steps (millions)")
    ax1.set_ylabel("% of training episodes")
    ax1.set_ylim(0, 100)
    ax1.legend(frameon=False, loc="center right", fontsize=10)
    ax1.set_title(f"Training, {len(runs)} seeds", loc="left")

    # 2. evaluation on fixed tasks: success and collisions, PPO vs IK vs random
    sets = [("open", "open space"), ("near", "near the wearer")]
    labels, succ, err, coll = [], [], [], []
    for key, name in sets:
        e = ev[key]
        for ctl, ctl_name in (("ppo", "PPO"), ("ik", "IK"), ("random", "random")):
            labels.append(f"{ctl_name}\n{name}")
            if ctl == "ppo":
                succ.append(100 * e["ppo"]["success_rate"]["mean"]); err.append(100 * e["ppo"]["success_rate"]["std"])
                coll.append(100 * e["ppo"]["collision_rate"]["mean"])
            else:
                succ.append(100 * e[ctl]["success_rate"]); err.append(0); coll.append(100 * e[ctl]["collision_rate"])
    x = np.array([0, 1, 2, 3.6, 4.6, 5.6])
    colors = [F.VIOLET, F.MUTED, F.GRID] * 2
    ax2.bar(x, succ, width=0.75, color=colors, edgecolor=F.INK, lw=0.8, yerr=err, capsize=3,
            error_kw={"ecolor": F.INK, "lw": 1})
    for xi, s, c in zip(x, succ, coll):
        ax2.text(xi, s + 3, f"{s:.0f}%", ha="center", fontsize=10, fontweight="bold")
        ax2.text(xi, -17, f"{c:.2f}%" if c < 1 else f"{c:.1f}%", ha="center", fontsize=9.5, color=F.RED)
    ax2.text(-0.6, -17, "touched\nwearer/bag", ha="right", va="center", fontsize=8.5, color=F.RED)
    ax2.set_xticks(x, ["PPO", "IK", "random"] * 2, fontsize=10)
    for xc, name in ((1, "open space"), (4.6, "near the wearer")):
        ax2.text(xc, 106, name, ha="center", fontsize=11, color=F.INK)
    ax2.set_ylim(0, 112)
    ax2.set_ylabel("% of tasks reached within 3 cm")
    n = ev["meta"]["tasks_per_set"]
    ax2.set_title(f"Evaluation, {n} fixed tasks per set", loc="left", pad=22)
    ax2.tick_params(axis="x", length=0, pad=4)
    F.save(fig, "fig-ppo.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
