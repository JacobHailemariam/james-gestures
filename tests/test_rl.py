"""PPO reaching policy: the environment's constraints, GAE, one PPO update, save/load, and deployment through
the real ArmController with the keep-out shield. Skips when torch or mujoco is missing (or blocked)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("torch")
pytest.importorskip("mujoco", exc_type=ImportError)

from james_gestures import ArmConfig, ArmController, PrintLink, default_joints  # noqa: E402
from rl.deploy import run_reach  # noqa: E402
from rl.env import JOINTS, ArmKinematics, ReachEnv, VecReach  # noqa: E402
from rl.ppo import PPO, Policy, PPOConfig, gae  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def kin():
    return ArmKinematics()


@pytest.fixture(scope="module")
def bad_pose(kin):
    """A pose inside a keep-out zone (bag or wearer), found by a seeded random search."""
    rng = np.random.default_rng(3)
    for _ in range(5000):
        q = rng.uniform(kin.lo, kin.hi)
        if kin.clearance(kin.fk(q)[1])[0] < -0.02:
            return q
    pytest.fail("no keep-out pose found")


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def armed_controller():
    clock, link = Clock(), PrintLink(quiet=True)
    arm = ArmController(link, cfg=ArmConfig(mirror=False, poses_file=None), clock=clock)
    assert arm.voice("home") and arm.armed                # a voice command arms the arm by itself

    def wait(dt):
        for _ in range(int(round(dt * 60))):
            clock.t += 1 / 60
            arm.tick()
    return arm, link, wait


# -- environment ----------------------------------------------------------------------------------------------
def test_reset_and_step_shapes(kin):
    env = ReachEnv(seed=0, kin=kin)
    obs = env.reset()
    assert obs.shape == (ReachEnv.obs_dim,) == (14,) and obs.dtype == np.float32
    obs, r, done, info = env.step(np.zeros(ReachEnv.act_dim))
    assert obs.shape == (14,) and np.isfinite(r) and isinstance(done, bool)
    assert {"success", "collided", "dist", "steps"} <= set(info)


def test_one_step_respects_speed_caps_and_joint_limits(kin):
    env = ReachEnv(seed=1, kin=kin)
    j = default_joints()
    cap = np.array([j[n].max_speed for n in JOINTS]) * env.cfg.dt
    rng = np.random.default_rng(0)
    env.reset()
    for _ in range(200):
        q0 = env.q.copy()
        _, _, done, _ = env.step(rng.uniform(-3, 3, 4))     # out-of-range actions are clipped to [-1, 1]
        assert np.all(np.abs(env.q - q0) <= cap + 1e-9)
        assert np.all(env.q >= kin.lo) and np.all(env.q <= kin.hi)
        if done:
            env.reset()


def test_sampled_targets_are_reachable(kin):
    env = ReachEnv(seed=2, kin=kin)
    for _ in range(10):
        q = env.random_safe_pose(min_clear=0.01)
        tip, _ = kin.fk(q)
        env.reset(target=tip, start=q)
        assert env.dist == pytest.approx(0.0, abs=1e-9)


def test_near_body_targets_come_from_poses_close_to_the_body(kin):
    a, b = ReachEnv(seed=4, kin=kin), ReachEnv(seed=4, kin=kin)
    target = a.sample_target(near_body=True)
    while True:                                           # replay the same draws to recover the sampling pose
        q = b.random_safe_pose(min_clear=0.01)
        tip, pts = kin.fk(q)
        if kin.clearance(pts)[0] < 0.06:
            break
    assert np.allclose(tip, target)
    assert 0.01 < kin.clearance(pts)[0] < 0.06


def test_keep_out_pose_has_negative_clearance_and_ends_the_episode(kin, bad_pose):
    clear, zone = kin.clearance(kin.fk(bad_pose)[1])
    assert clear < 0 and zone
    env = ReachEnv(seed=5, kin=kin)
    env.reset(target=kin.fk(kin.home)[0], start=kin.home)
    for _ in range(env.cfg.max_steps):                    # drive straight at the bad pose at full speed
        a = np.clip((bad_pose - env.q) / (kin.speed * env.cfg.dt), -1, 1)
        _, r, done, info = env.step(a)
        if done:
            break
    assert done and info["collided"] and not info["success"]
    assert r < 0


# -- PPO ------------------------------------------------------------------------------------------------------
def test_gae_hand_checked():
    rew = np.array([[1.0], [2.0], [3.0]], np.float32)
    val = np.array([[0.5], [0.6], [0.7]], np.float32)
    done = np.array([[0.0], [1.0], [0.0]], np.float32)    # the episode ends after step 1
    last = np.array([0.8], np.float32)
    g, lam = 0.9, 0.8
    d2 = 3 + g * 0.8 - 0.7                                # bootstraps from last_val
    d1 = 2 - 0.6                                          # terminal: no bootstrap, no carry
    d0 = 1 + g * 0.6 - 0.5
    want = np.array([[d0 + g * lam * d1], [d1], [d2]])
    adv, ret = gae(rew, val, done, last, g, lam)
    assert np.allclose(adv, want, atol=1e-6)
    assert np.allclose(ret, want + val, atol=1e-6)


def test_ppo_one_update_and_policy_round_trip(tmp_path):
    env = VecReach(2, seed=0)
    agent = PPO(ReachEnv.obs_dim, ReachEnv.act_dim, PPOConfig(steps_per_env=8, minibatch=8, epochs=1), seed=0)
    batch, obs, _ = agent.collect(env, env.reset())
    assert batch["obs"].shape == (16, 14) and batch["act"].shape == (16, 4)
    stats = agent.update(batch)
    assert all(math.isfinite(v) for v in stats.values())
    path = tmp_path / "p.pt"
    agent.save(path, extra={"env_steps": 16})
    pol = Policy(path)
    o = np.random.default_rng(0).normal(size=14).astype(np.float32)
    want = np.clip(agent.net.act(agent.norm(o), deterministic=True), -1, 1)
    assert np.allclose(pol(o), want, atol=1e-6)
    assert pol.meta["env_steps"] == 16


# -- ArmController.goto ---------------------------------------------------------------------------------------
def test_goto_needs_arming_and_clamps_to_limits():
    arm = ArmController(PrintLink(quiet=True), cfg=ArmConfig(mirror=False, poses_file=None), clock=Clock())
    assert not arm.goto({"shoulder": 100})
    assert arm.target["shoulder"] == 90                  # untouched while disarmed
    arm2, _, _ = armed_controller()
    assert arm2.goto({"shoulder": 500, "elbow": -40, "base": 300})
    j = default_joints()
    assert arm2.target["shoulder"] == j["shoulder"].hi
    assert arm2.target["elbow"] == j["elbow"].lo
    assert arm2.target["base"] == j["base"].hi
    assert not arm2.steering


def test_goto_preset_still_moves_the_arm():
    arm, link, wait = armed_controller()
    assert arm.goto_preset("raise")
    wait(4.0)
    assert arm.pos["shoulder"] == 150 and arm.pos["elbow"] == 120
    assert any(line.startswith("J shoulder") for line in link.log)


# -- deployment -----------------------------------------------------------------------------------------------
def test_run_reach_zero_policy_neither_reaches_nor_shields(kin):
    arm, link, wait = armed_controller()
    target = kin.fk(np.array([180.0, 120.0, 60.0, 90.0]))[0]
    res = run_reach(arm, lambda o: np.zeros(4), target, kin=kin, wait=wait, max_s=1.0)
    assert not res["reached"] and not res["shielded"]
    home = {n: default_joints()[n].home for n in JOINTS}
    for line in link.log:                                 # only the controller's first sync of each joint at home
        p = line.split()
        if p[0] == "J":
            assert float(p[2]) == pytest.approx(home[p[1]]), line


def test_shield_never_sends_a_keep_out_pose(kin, bad_pose):
    arm, link, wait = armed_controller()
    speed, dt = kin.speed, 0.1

    def toward_bad(obs):
        q = np.array([arm.pos[n] for n in JOINTS])
        return np.clip((bad_pose - q) / (speed * dt), -1, 1)

    res = run_reach(arm, toward_bad, kin.fk(bad_pose)[0], kin=kin, wait=wait, max_s=10.0)
    assert res["shielded"] and not res["reached"]
    sent = {n: default_joints()[n].home for n in JOINTS}  # replay every J line the board would have received
    for line in link.log:
        p = line.split()
        if p[0] == "J":
            sent[p[1]] = float(p[2])
            q = np.array([sent[n] for n in JOINTS])
            assert kin.clearance(kin.fk(q)[1])[0] >= 0, line


# -- the trained policy ---------------------------------------------------------------------------------------
@pytest.mark.skipif(not (ROOT / "rl" / "reach_policy.pt").exists(), reason="no trained policy in rl/")
def test_trained_policy_reaches_open_space_targets():
    sys.path.insert(0, str(ROOT / "tools"))
    from eval_ppo import run, tasks
    pol = Policy(ROOT / "rl" / "reach_policy.pt")
    res = run(lambda e: pol, tasks(30, near_body=False), ReachEnv(seed=1))
    assert res["success_rate"] >= 0.6, res
