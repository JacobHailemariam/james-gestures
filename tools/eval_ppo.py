"""Evaluate the PPO reaching policy against two baselines on the same fixed set of reaching tasks.

    python tools/eval_ppo.py                 # every seed in rl/runs/, 1000 tasks per set -> docs/data/ppo_eval.json
    python tools/eval_ppo.py --n 200 --policies rl/reach_policy.pt          # quicker, one policy

Controllers, all with the same joint limits, 10 Hz control and per-joint speed caps:
  ppo     the trained policy (rl/reach_policy.pt), deterministic
  ik      damped-least-squares inverse kinematics: steps straight down the fingertip error, no knowledge of the
          keep-out zones (the textbook way to reach a point)
  random  uniform random actions
A task is a reachable target (sampled from a pose that is itself clear of every keep-out zone) and a start pose
(home in 30% of tasks, else a random safe pose). Two fixed task sets: "open" (any reachable target) and "near"
(targets whose pose passes within 6 cm of the wearer or the bag). Success = fingertip within 3 cm without
touching a zone, within 8 s.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rl.env import ReachEnv  # noqa: E402


def tasks(n: int, near_body: bool, seed: int = 2026):
    env = ReachEnv(seed=seed + int(near_body))
    out = []
    for _ in range(n):
        target = env.sample_target(near_body=near_body)
        start = env.kin.home.copy() if env.rng.random() < env.cfg.home_start else env.random_safe_pose()
        out.append((target, start))
    return out


def ik_controller(env: ReachEnv, damping: float = 0.05):
    kin = env.kin

    def act(obs):
        q = env.q
        tip, _ = kin.fk(q)
        J = np.zeros((3, 4))
        for i in range(4):                       # numerical Jacobian, metres per radian
            dq = np.zeros(4); dq[i] = 0.5
            J[:, i] = (kin.fk(q + dq)[0] - kin.fk(q - dq)[0]) / np.radians(1.0)
        kin.fk(q)
        err = env.target - tip
        step = np.degrees(J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), err))
        return np.clip(step / (kin.speed * env.cfg.dt), -1, 1)
    return act


def run(controller_factory, task_list, env: ReachEnv) -> dict:
    res = {"success": 0, "collided": 0, "timeout": 0, "final_mm": [], "time_s": [], "zones": {}}
    act = controller_factory(env)
    for target, start in task_list:
        obs = env.reset(target=target, start=start)
        while True:
            obs, r, done, info = env.step(act(obs))
            if done:
                break
        res["success"] += info["success"]; res["collided"] += info["collided"]; res["timeout"] += info["timeout"]
        res["final_mm"].append(info["dist"] * 1000)
        if info["success"]:
            res["time_s"].append(info["steps"] * env.cfg.dt)
        if info["collided"]:
            res["zones"][info["zone"]] = res["zones"].get(info["zone"], 0) + 1
    n = len(task_list)
    return {"success_rate": res["success"] / n, "collision_rate": res["collided"] / n, "timeout_rate": res["timeout"] / n,
            "median_final_mm": float(np.median(res["final_mm"])),
            "mean_time_to_reach_s": float(np.mean(res["time_s"])) if res["time_s"] else None,
            "collisions_by_zone": res["zones"], "n": n}


def summarize(runs: list[dict]) -> dict:
    keys = ("success_rate", "collision_rate", "timeout_rate", "median_final_mm")
    return {k: {"mean": float(np.mean([r[k] for r in runs])), "std": float(np.std([r[k] for r in runs]))} for k in keys}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--policies", nargs="+", default=sorted(str(p) for p in (ROOT / "rl" / "runs").glob("seed*.pt")))
    ap.add_argument("--out", default=str(ROOT / "docs" / "data" / "ppo_eval.json"))
    a = ap.parse_args()

    from rl.ppo import Policy
    env = ReachEnv(seed=1)
    out = {"meta": {"tasks_per_set": a.n, "task_seed": 2026, "success_radius_mm": env.cfg.success_m * 1000,
                    "episode_limit_s": env.cfg.max_steps * env.cfg.dt,
                    "policies": [Path(p).name for p in a.policies]}}
    for name, near in (("open", False), ("near", True)):
        task_list = tasks(a.n, near)
        rng = np.random.default_rng(7)
        per_seed = {}
        for path in a.policies:
            pol = Policy(path)
            per_seed[Path(path).stem] = run(lambda e, pol=pol: pol, task_list, env)
        res = {"ppo_seeds": per_seed, "ppo": summarize(list(per_seed.values())),
               "ik": run(ik_controller, task_list, env),
               "random": run(lambda e: (lambda o: rng.uniform(-1, 1, 4)), task_list, env)}
        out[name] = res
        print(f"\n== {name} ({a.n} tasks)")
        print(f"{'':10} {'success':>8} {'collision':>10} {'timeout':>8} {'median final':>13} {'time to reach':>14}")
        rows = list(per_seed.items()) + [("ik", res["ik"]), ("random", res["random"])]
        for k, r in rows:
            t = f"{r['mean_time_to_reach_s']:.2f} s" if r["mean_time_to_reach_s"] else "-"
            print(f"{k:10} {r['success_rate']:8.1%} {r['collision_rate']:10.1%} {r['timeout_rate']:8.1%} "
                  f"{r['median_final_mm']:10.1f} mm {t:>14}   {r['collisions_by_zone']}")
        m = res["ppo"]
        print(f"{'ppo mean':10} {m['success_rate']['mean']:8.1%} +- {m['success_rate']['std']:.1%}   "
              f"collision {m['collision_rate']['mean']:.1%}")
    Path(a.out).write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print("\nwrote", a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
