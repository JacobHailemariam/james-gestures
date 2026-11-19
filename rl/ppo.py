"""PPO (Schulman et al., 2017) for continuous actions, written out in PyTorch so every piece is visible.

Gaussian policy with a state-independent log-std, separate value network, generalised advantage estimation
(Schulman et al., 2016), clipped surrogate objective, value clipping, advantage normalisation, running
observation normalisation, and gradient-norm clipping.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
import torch
import torch.nn as nn


@dataclass
class PPOConfig:
    steps_per_env: int = 128      # rollout length per environment
    gamma: float = 0.99
    lam: float = 0.95             # GAE lambda
    clip: float = 0.2
    epochs: int = 10
    minibatch: int = 512
    lr: float = 3e-4
    vf_coef: float = 0.5
    ent_coef: float = 0.0
    max_grad_norm: float = 0.5
    hidden: int = 128
    init_log_std: float = -0.5


class RunningNorm:
    """Running mean/variance of observations (parallel Welford), so inputs stay near unit scale."""

    def __init__(self, dim: int):
        self.mean = np.zeros(dim, np.float64)
        self.var = np.ones(dim, np.float64)
        self.count = 1e-4

    def update(self, x: np.ndarray) -> None:
        bm, bv, bc = x.mean(0), x.var(0), x.shape[0]
        delta, tot = bm - self.mean, self.count + bc
        self.mean = self.mean + delta * bc / tot
        self.var = (self.var * self.count + bv * bc + delta**2 * self.count * bc / tot) / tot
        self.count = tot

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8), -10, 10).astype(np.float32)

    def state(self) -> dict:
        return {"mean": self.mean.tolist(), "var": self.var.tolist(), "count": self.count}

    @classmethod
    def from_state(cls, s: dict) -> "RunningNorm":
        n = cls(len(s["mean"]))
        n.mean, n.var, n.count = np.array(s["mean"]), np.array(s["var"]), s["count"]
        return n


def mlp(i: int, h: int, o: int, out_gain: float) -> nn.Sequential:
    net = nn.Sequential(nn.Linear(i, h), nn.Tanh(), nn.Linear(h, h), nn.Tanh(), nn.Linear(h, o))
    for m in net:
        if isinstance(m, nn.Linear):
            nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
            nn.init.zeros_(m.bias)
    nn.init.orthogonal_(net[-1].weight, gain=out_gain)
    return net


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int, hidden: int = 128, init_log_std: float = -0.5):
        super().__init__()
        self.pi = mlp(obs_dim, hidden, act_dim, out_gain=0.01)    # small initial actions
        self.v = mlp(obs_dim, hidden, 1, out_gain=1.0)
        self.log_std = nn.Parameter(torch.full((act_dim,), init_log_std))

    def dist(self, obs: torch.Tensor) -> torch.distributions.Normal:
        return torch.distributions.Normal(self.pi(obs), self.log_std.exp())

    def value(self, obs: torch.Tensor) -> torch.Tensor:
        return self.v(obs).squeeze(-1)

    @torch.no_grad()
    def act(self, obs: np.ndarray, deterministic: bool = False) -> np.ndarray:
        o = torch.as_tensor(obs, dtype=torch.float32)
        mu = self.pi(o)
        a = mu if deterministic else torch.distributions.Normal(mu, self.log_std.exp()).sample()
        return a.numpy()


def gae(rew: np.ndarray, val: np.ndarray, done: np.ndarray, last_val: np.ndarray, gamma: float, lam: float):
    """Advantages and returns for a [T, N] rollout. done[t] = the episode ended after step t."""
    T = rew.shape[0]
    adv = np.zeros_like(rew)
    last = np.zeros_like(last_val)
    for t in reversed(range(T)):
        nxt = last_val if t == T - 1 else val[t + 1]
        nonterm = 1.0 - done[t]
        delta = rew[t] + gamma * nxt * nonterm - val[t]
        last = delta + gamma * lam * nonterm * last
        adv[t] = last
    return adv, adv + val


class PPO:
    def __init__(self, obs_dim: int, act_dim: int, cfg: PPOConfig | None = None, seed: int = 0):
        torch.manual_seed(seed)
        self.cfg = cfg or PPOConfig()
        self.net = ActorCritic(obs_dim, act_dim, self.cfg.hidden, self.cfg.init_log_std)
        self.opt = torch.optim.Adam(self.net.parameters(), lr=self.cfg.lr, eps=1e-5)
        self.norm = RunningNorm(obs_dim)

    def collect(self, env, obs: np.ndarray):
        """Run the current policy for steps_per_env steps in every env. Returns the batch, final obs, episode infos."""
        c, T, N = self.cfg, self.cfg.steps_per_env, env.n
        buf = {k: [] for k in ("obs", "act", "logp", "val", "rew", "done")}
        finished = []
        for _ in range(T):
            self.norm.update(obs)
            o = torch.as_tensor(self.norm(obs))
            with torch.no_grad():
                d = self.net.dist(o)
                a = d.sample()
                buf["logp"].append(d.log_prob(a).sum(-1).numpy())
                buf["val"].append(self.net.value(o).numpy())
            buf["obs"].append(o.numpy()); buf["act"].append(a.numpy())
            obs, r, done, infos = env.step(a.numpy())          # the env clips to [-1, 1]
            buf["rew"].append(r); buf["done"].append(done.astype(np.float32))
            finished += [i for i, dn in zip(infos, done) if dn]
        with torch.no_grad():
            last_val = self.net.value(torch.as_tensor(self.norm(obs))).numpy()
        b = {k: np.asarray(v) for k, v in buf.items()}
        adv, ret = gae(b["rew"], b["val"], b["done"], last_val, c.gamma, c.lam)
        batch = {"obs": b["obs"].reshape(T * N, -1), "act": b["act"].reshape(T * N, -1),
                 "logp": b["logp"].reshape(-1), "val": b["val"].reshape(-1),
                 "adv": adv.reshape(-1), "ret": ret.reshape(-1)}
        return batch, obs, finished

    def update(self, batch: dict) -> dict:
        c = self.cfg
        t = {k: torch.as_tensor(v, dtype=torch.float32) for k, v in batch.items()}
        n = len(t["adv"])
        stats = {"pi_loss": [], "v_loss": [], "entropy": [], "approx_kl": [], "clip_frac": []}
        for _ in range(c.epochs):
            for idx in torch.randperm(n).split(c.minibatch):
                adv = t["adv"][idx]
                adv = (adv - adv.mean()) / (adv.std() + 1e-8)
                d = self.net.dist(t["obs"][idx])
                logp = d.log_prob(t["act"][idx]).sum(-1)
                ratio = (logp - t["logp"][idx]).exp()
                pi_loss = -torch.min(ratio * adv, ratio.clamp(1 - c.clip, 1 + c.clip) * adv).mean()
                v = self.net.value(t["obs"][idx])
                v_clip = t["val"][idx] + (v - t["val"][idx]).clamp(-c.clip, c.clip)
                v_loss = 0.5 * torch.max((v - t["ret"][idx]) ** 2, (v_clip - t["ret"][idx]) ** 2).mean()
                ent = d.entropy().sum(-1).mean()
                loss = pi_loss + c.vf_coef * v_loss - c.ent_coef * ent
                self.opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.net.parameters(), c.max_grad_norm)
                self.opt.step()
                with torch.no_grad():
                    stats["pi_loss"].append(pi_loss.item()); stats["v_loss"].append(v_loss.item())
                    stats["entropy"].append(ent.item())
                    stats["approx_kl"].append(((ratio - 1) - (logp - t["logp"][idx])).mean().item())
                    stats["clip_frac"].append(((ratio - 1).abs() > c.clip).float().mean().item())
        return {k: float(np.mean(v)) for k, v in stats.items()}

    def save(self, path, extra: dict | None = None) -> None:
        torch.save({"net": self.net.state_dict(), "norm": self.norm.state(), "cfg": asdict(self.cfg),
                    **(extra or {})}, path)


class Policy:
    """A trained policy for inference: normalise the observation, return the mean action (no exploration noise)."""

    def __init__(self, path):
        ck = torch.load(path, map_location="cpu", weights_only=False)
        cfg = PPOConfig(**ck["cfg"])
        sd = ck["net"]
        obs_dim, act_dim = sd["pi.0.weight"].shape[1], sd["pi.4.weight"].shape[0]
        self.net = ActorCritic(obs_dim, act_dim, cfg.hidden, cfg.init_log_std)
        self.net.load_state_dict(sd)
        self.net.eval()
        self.norm = RunningNorm.from_state(ck["norm"])
        self.meta = {k: v for k, v in ck.items() if k not in ("net", "norm")}

    def __call__(self, obs: np.ndarray) -> np.ndarray:
        return np.clip(self.net.act(self.norm(np.asarray(obs)), deterministic=True), -1.0, 1.0)
