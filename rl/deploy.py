"""Run the trained reaching policy on the arm, through the same path every other command takes.

    policy -> joint targets -> ArmController.goto() (joint limits, deg/s rate caps, 10 Hz per joint)
           -> "J <joint> <deg>" lines -> SimLink, or SerialLink to the board (60 deg/s slew in the firmware)

A runtime shield sits between the policy and the controller: every proposed pose is checked against the
keep-out zones before it is sent, and a pose that would enter one is never sent; the arm holds where it is.
"""
from __future__ import annotations

import time

import numpy as np

from .env import JOINTS, ArmKinematics, ReachConfig, ReachEnv, to_degrees_dict


def run_reach(arm, policy, target, kin: ArmKinematics | None = None, wait=None, on_step=None,
              max_s: float = 8.0, cfg: ReachConfig | None = None) -> dict:
    """Drive an armed ArmController to put the fingertip on `target` (metres, arm.xml frame).

    wait(dt) must let `dt` seconds pass for the controller (real time: sleep and tick; tests: advance a fake
    clock and tick). Returns {"reached", "shielded", "steps", "dist_mm", "min_clear_mm"}.
    """
    cfg = cfg or ReachConfig()
    env = ReachEnv(cfg, kin=kin)
    kin = env.kin
    wait = wait or (lambda dt: _real_wait(arm, dt))
    min_clear, shielded, k = np.inf, False, 0
    for k in range(int(round(max_s / cfg.dt))):
        q = np.array([arm.pos[n] for n in JOINTS])
        obs = env.observe(q, target)
        min_clear = min(min_clear, env.clear)
        if env.dist < cfg.success_m:
            break
        a = np.clip(policy(obs), -1, 1)
        q_next = np.clip(q + a * kin.speed * cfg.dt, kin.lo, kin.hi)
        _, pts = kin.fk(q_next)
        if kin.clearance(pts)[0] < 0:                 # shield: never send a pose inside a keep-out zone
            shielded = True
            break
        if not arm.goto(to_degrees_dict(q_next)):
            break                                      # disarmed (STOP, fist, board lost): stop driving
        wait(cfg.dt)
        if on_step:
            on_step(k, env)
    q = np.array([arm.pos[n] for n in JOINTS])
    env.observe(q, target)
    return {"reached": env.dist < cfg.success_m, "shielded": shielded, "steps": k,
            "dist_mm": env.dist * 1000, "min_clear_mm": min(min_clear, env.clear) * 1000}


def _real_wait(arm, dt: float, rate_hz: float = 60.0) -> None:
    end = time.monotonic() + dt
    while time.monotonic() < end:
        arm.tick()
        time.sleep(1.0 / rate_hz)
