"""Train the PPO reaching policy on the MuJoCo model of Atlas.

    python tools/train_ppo.py                        # ~1.5M steps, writes rl/reach_policy.pt and docs/data/ppo_train.csv
    python tools/train_ppo.py --steps 200000 --out /tmp/p.pt --log /tmp/p.csv    # a quick run

CPU only; the environment is kinematic (no contact physics), so most of the time goes to Python stepping.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rl.env import ReachEnv, VecReach  # noqa: E402
from rl.ppo import PPO, PPOConfig  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=1_500_000, help="total environment steps")
    ap.add_argument("--envs", type=int, default=16)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--out", default=str(ROOT / "rl" / "reach_policy.pt"))
    ap.add_argument("--log", default=str(ROOT / "docs" / "data" / "ppo_train.csv"))
    a = ap.parse_args()

    torch.set_num_threads(a.threads)
    np.random.seed(a.seed)
    env = VecReach(a.envs, seed=a.seed * 1000)
    agent = PPO(ReachEnv.obs_dim, ReachEnv.act_dim, PPOConfig(), seed=a.seed)
    per_iter = agent.cfg.steps_per_env * a.envs
    iters = a.steps // per_iter
    obs = env.reset()
    t0, done_steps = time.time(), 0
    Path(a.log).parent.mkdir(parents=True, exist_ok=True)
    with open(a.log, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["iter", "env_steps", "episodes", "success_rate", "collision_rate", "timeout_rate",
                    "final_dist_mm", "ep_len", "pi_loss", "v_loss", "approx_kl", "clip_frac", "action_std", "sec"])
        for it in range(1, iters + 1):
            frac = 1.0 - (it - 1) / iters                   # linear learning-rate decay to zero
            for g in agent.opt.param_groups:
                g["lr"] = agent.cfg.lr * frac
            batch, obs, eps = agent.collect(env, obs)
            st = agent.update(batch)
            done_steps += per_iter
            n = max(len(eps), 1)
            succ = sum(e["success"] for e in eps) / n
            coll = sum(e["collided"] for e in eps) / n
            tout = sum(e["timeout"] for e in eps) / n
            dist = 1000 * float(np.mean([e["dist"] for e in eps])) if eps else float("nan")
            eplen = float(np.mean([e["steps"] for e in eps])) if eps else float("nan")
            std = float(agent.net.log_std.detach().exp().mean())
            w.writerow([it, done_steps, len(eps), f"{succ:.4f}", f"{coll:.4f}", f"{tout:.4f}", f"{dist:.1f}",
                        f"{eplen:.1f}", f"{st['pi_loss']:.4f}", f"{st['v_loss']:.4f}", f"{st['approx_kl']:.5f}",
                        f"{st['clip_frac']:.3f}", f"{std:.3f}", f"{time.time() - t0:.0f}"])
            f.flush()
            if it % 20 == 0 or it == iters:
                print(f"it {it:4d}/{iters}  steps {done_steps:>8,}  success {succ:5.1%}  collide {coll:5.1%}  "
                      f"final {dist:6.1f} mm  kl {st['approx_kl']:.4f}  std {std:.2f}  {time.time() - t0:5.0f}s",
                      flush=True)
    agent.save(a.out, extra={"env_steps": done_steps, "seed": a.seed})
    print("saved", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
