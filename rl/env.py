"""Reaching task for PPO on the MuJoCo model of Atlas (sim/arm.xml), with the real arm's constraints.

The agent moves the four joints to put the fingertip on a target point near the wearer. Everything the agent can
do is something the real arm can do:

- **Same control rate and speed caps.** One action every 0.1 s (the controller sends each joint at most 10 times
  a second). An action in [-1, 1] per joint is a fraction of that joint's max speed from `default_joints()`
  (base 90, shoulder 45, elbow 60, wrist 90 deg/s), so a step can never ask for more than the arm can do.
- **Same joint limits**, in the firmware's degrees (base 0..270, shoulder/elbow 20..160, wrist 0..180).
- **Keep-out zones.** The backpack (the box the pose solver avoids, plus 2 cm) and the wearer's head, neck,
  shoulders and torso, read from the wearer geoms in arm.xml. The head gets an extra 5 cm. Points sampled along
  every link are checked each step; touching a zone ends the episode with a penalty.

The kinematics come from MuJoCo (mj_kinematics on the same model the simulator and the pose solver use).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from james_gestures.arm import default_joints
from james_gestures.sim import HOME, MODEL

JOINTS = ("base", "shoulder", "elbow", "wrist")
BAG = dict(x=(0.0, 0.245), y=(-0.050, 0.020), z=(0.0, 0.420))   # bag + plate + turntable (tools/solve_poses.py)
BAG_MARGIN = 0.02
HEAD_MARGIN = 0.05                     # extra room around the head
BODY_MARGIN = 0.02
LINK_RADIUS = 0.03                     # the printed links are ~50-60 mm across
KEEP_OUT = ("head", "neck", "shoulders", "torso", "arm_upper_l", "arm_upper_r")
MOUNT = np.array([0.155, 0.020, 0.268])   # turntable centre: observations are relative to it
SCALE = 0.8                               # metres -> roughly unit range


@dataclass
class ReachConfig:
    dt: float = 0.1               # seconds per action (10 Hz, as the controller sends)
    max_steps: int = 80           # 8 s per episode
    success_m: float = 0.03       # fingertip within 3 cm = reached
    progress_w: float = 10.0      # reward per metre of distance closed
    effort_w: float = 0.002       # penalty on squared action
    time_w: float = 0.01          # small penalty per step: reach quickly
    caution_m: float = 0.04       # inside this clearance a growing penalty starts
    caution_w: float = 3.0
    collide_penalty: float = 2.0
    success_bonus: float = 2.0
    home_start: float = 0.3       # fraction of episodes that start from the home pose, the rest from a random safe pose
    near_body: float = 0.5        # fraction of targets that hug the wearer or the bag (pose clearance under 6 cm)


class ArmKinematics:
    """MuJoCo forward kinematics plus clearance to the keep-out zones. Angles are the firmware's degrees."""

    def __init__(self, model_path=MODEL):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.adr = np.array([self.model.joint(n).qposadr[0] for n in JOINTS])
        self.bodies = [self.model.body(n).id for n in ("upper", "fore", "hand")]
        self.tip = self.model.site("tip").id
        j = default_joints()
        self.lo = np.array([j[n].lo for n in JOINTS], float)
        self.hi = np.array([j[n].hi for n in JOINTS], float)
        self.speed = np.array([j[n].max_speed for n in JOINTS], float)
        self.home = np.array([HOME[n] for n in JOINTS], float)
        mujoco.mj_kinematics(self.model, self.data)
        self.zones = []                                    # (kind, centre, axis/rotation, size, margin)
        for name in KEEP_OUT:
            g = self.model.geom(name)
            gid = g.id
            pos = self.data.geom_xpos[gid].copy()
            mat = self.data.geom_xmat[gid].reshape(3, 3).copy()
            size = self.model.geom_size[gid].copy()
            margin = HEAD_MARGIN if name == "head" else BODY_MARGIN
            kind = {mujoco.mjtGeom.mjGEOM_SPHERE: "sphere", mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
                    mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid"}[int(self.model.geom_type[gid])]
            self.zones.append((name, kind, pos, mat, size, margin))

    def fk(self, q_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Fingertip position and points sampled along the links (shoulder pivot -> elbow -> wrist -> tip)."""
        self.data.qpos[self.adr] = np.radians(q_deg - self.home)
        self.mj.mj_kinematics(self.model, self.data)
        joints = [self.data.xpos[b] for b in self.bodies] + [self.data.site_xpos[self.tip]]
        s = np.linspace(0.0, 1.0, 6)[:, None]
        pts = np.concatenate([p0 + s * (p1 - p0) for p0, p1 in zip(joints[:-1], joints[1:])])
        return joints[-1].copy(), pts

    def clearance(self, pts: np.ndarray) -> tuple[float, str]:
        """Smallest gap (m) between the link surfaces and any keep-out zone, negative = inside one."""
        best, who = np.inf, ""
        lo = np.array([BAG["x"][0], BAG["y"][0], BAG["z"][0]]) - BAG_MARGIN
        hi = np.array([BAG["x"][1], BAG["y"][1], BAG["z"][1]]) + BAG_MARGIN
        out = np.maximum(lo - pts, 0) + np.maximum(pts - hi, 0)
        outside = np.linalg.norm(out, axis=1)
        inside = np.min(np.minimum(pts - lo, hi - pts), axis=1)
        d_bag = np.where(outside > 0, outside, -inside) - LINK_RADIUS
        if d_bag.min() < best:
            best, who = float(d_bag.min()), "bag"
        for name, kind, c, mat, size, margin in self.zones:
            local = (pts - c) @ mat                           # into the geom's frame
            if kind == "sphere":
                d = np.linalg.norm(local, axis=1) - size[0]
            elif kind == "capsule":
                z = np.clip(local[:, 2], -size[1], size[1])
                d = np.linalg.norm(local - np.stack([0 * z, 0 * z, z], 1), axis=1) - size[0]
            else:                                              # ellipsoid: scaled distance, good near the surface
                r = np.linalg.norm(local / size[:3], axis=1)
                d = (r - 1.0) * size[:3].min()
            d = d - margin - LINK_RADIUS
            if d.min() < best:
                best, who = float(d.min()), name
        return best, who


class ReachEnv:
    """One reaching episode at a time. reset() -> obs; step(action) -> obs, reward, done, info."""

    obs_dim, act_dim = 14, 4

    def __init__(self, cfg: ReachConfig | None = None, seed: int | None = None, kin: ArmKinematics | None = None):
        self.cfg = cfg or ReachConfig()
        self.kin = kin or ArmKinematics()
        self.rng = np.random.default_rng(seed)
        self.q = self.kin.home.copy()
        self.target = np.zeros(3)

    # -- sampling -------------------------------------------------------------------------------------------
    def random_safe_pose(self, min_clear: float = 0.0) -> np.ndarray:
        while True:
            q = self.rng.uniform(self.kin.lo, self.kin.hi)
            _, pts = self.kin.fk(q)
            if self.kin.clearance(pts)[0] > min_clear:
                return q

    def sample_target(self, near_body: bool | None = None) -> np.ndarray:
        """A point the fingertip can actually reach without touching a keep-out zone. Near-body targets come
        from poses that pass within 6 cm of the wearer or the bag: the cases where steering straight at the
        target is most likely to clip something."""
        if near_body is None:
            near_body = self.rng.random() < self.cfg.near_body
        while True:
            q = self.random_safe_pose(min_clear=0.01)
            tip, pts = self.kin.fk(q)
            if not near_body or self.kin.clearance(pts)[0] < 0.06:
                return tip

    # -- episode ----------------------------------------------------------------------------------------------
    def reset(self, target=None, start=None) -> np.ndarray:
        self.target = np.asarray(target, float) if target is not None else self.sample_target()
        if start is not None:
            self.q = np.asarray(start, float).copy()
        elif self.rng.random() < self.cfg.home_start:
            self.q = self.kin.home.copy()
        else:
            self.q = self.random_safe_pose()
        self.t = 0
        self.tip, pts = self.kin.fk(self.q)
        self.clear = self.kin.clearance(pts)[0]
        self.dist = float(np.linalg.norm(self.tip - self.target))
        return self._obs()

    def observe(self, q, target) -> np.ndarray:
        """Observation for joint angles `q` (firmware degrees) and a target point, without stepping (for deployment)."""
        self.q = np.asarray(q, float).copy()
        self.target = np.asarray(target, float)
        self.tip, pts = self.kin.fk(self.q)
        self.clear = self.kin.clearance(pts)[0]
        self.dist = float(np.linalg.norm(self.tip - self.target))
        return self._obs()

    def _obs(self) -> np.ndarray:
        qn = 2 * (self.q - self.kin.lo) / (self.kin.hi - self.kin.lo) - 1
        tip = (self.tip - MOUNT) / SCALE
        tgt = (self.target - MOUNT) / SCALE
        return np.concatenate([qn, tip, tgt, tgt - tip, [min(self.clear, 0.2) / 0.2]]).astype(np.float32)

    def step(self, action):
        c = self.cfg
        a = np.clip(np.asarray(action, float), -1.0, 1.0)
        self.q = np.clip(self.q + a * self.kin.speed * c.dt, self.kin.lo, self.kin.hi)
        self.t += 1
        self.tip, pts = self.kin.fk(self.q)
        self.clear, zone = self.kin.clearance(pts)
        dist = float(np.linalg.norm(self.tip - self.target))
        r = c.progress_w * (self.dist - dist) - c.effort_w * float(a @ a) - c.time_w
        if self.clear < c.caution_m:
            r -= c.caution_w * (c.caution_m - max(self.clear, 0.0)) / c.caution_m * c.dt
        self.dist = dist
        collided = self.clear < 0.0
        success = dist < c.success_m and not collided
        if collided:
            r -= c.collide_penalty
        if success:
            r += c.success_bonus
        timeout = self.t >= c.max_steps
        done = collided or success or timeout
        info = {"success": success, "collided": collided, "zone": zone if collided else "", "dist": dist,
                "steps": self.t, "timeout": timeout and not (collided or success)}
        return self._obs(), r, done, info


class VecReach:
    """N independent ReachEnvs stepped together, each resetting itself when its episode ends."""

    def __init__(self, n: int, cfg: ReachConfig | None = None, seed: int = 0):
        self.envs = [ReachEnv(cfg, seed=seed + i) for i in range(n)]
        self.n = n

    def reset(self) -> np.ndarray:
        return np.stack([e.reset() for e in self.envs])

    def step(self, actions: np.ndarray):
        obs, rew, done, infos = [], [], [], []
        for e, a in zip(self.envs, actions):
            o, r, d, i = e.step(a)
            if d:
                o = e.reset()
            obs.append(o); rew.append(r); done.append(d); infos.append(i)
        return np.stack(obs), np.array(rew, np.float32), np.array(done), infos


def to_degrees_dict(q: np.ndarray) -> dict[str, float]:
    return {n: float(v) for n, v in zip(JOINTS, q)}

