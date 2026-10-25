"""Solve the named poses (left/right shoulder, left/right hip) from the backpack geometry in sim/arm.xml.

  .venv/Scripts/python.exe tools/solve_poses.py            # print the joint angles for each target
  .venv/Scripts/python.exe tools/solve_poses.py --write    # also write them into poses.json

For each target point (metres, same frame as arm.xml: x across the back seen from behind, y out of the bag,
z up) it searches base/shoulder/elbow/wrist within the joint limits for the pose that puts the fingertip on
the target without any part of the arm passing through the bag. Pure kinematics via MuJoCo's forward pass.

"Left" below means the WEARER's left. Seen from behind the bag that is the +x side (x near 0.245).
Flip LEFT_IS_PLUS_X if the sim shows the opposite of what the team calls left.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from james_gestures.sim import HOME, MODEL  # noqa: E402

LEFT_IS_PLUS_X = True
BAG = dict(x=(0.0, 0.245), y=(-0.050, 0.020), z=(0.0, 0.420))      # bag body plus plate/turntable, metres
MARGIN = 0.020                                                      # keep every link this far from the bag

# Where the fingertip should end up. Reach check first: the shoulder pivot stands 127 mm proud of the back
# face and the shoulder and elbow fold only +-70 deg each, so the fingertip can come at most ~70 mm in front
# of the back face, and only with the arm fully folded, i.e. ~375 mm out from the base axis. Over the
# shoulder therefore means high (about 600 mm up), and around the hip means well out to the side.
def targets() -> dict[str, np.ndarray]:
    lx, rx = (0.215, 0.030) if LEFT_IS_PLUS_X else (0.030, 0.215)
    lhx, rhx = (0.430, -0.180) if LEFT_IS_PLUS_X else (-0.180, 0.430)
    return {
        "left shoulder": np.array([lx, -0.060, 0.600]),     # above the shoulder, just in front of the back face
        "right shoulder": np.array([rx, -0.060, 0.600]),
        "left hip": np.array([lhx, -0.050, 0.150]),         # beside the hip, 185 mm outside the bag's edge
        "right hip": np.array([rhx, -0.050, 0.150]),
    }


def main() -> int:
    import mujoco

    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="write the result into poses.json (merged)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    jid = {n: model.joint(n).qposadr[0] for n in ("base", "shoulder", "elbow", "wrist")}
    lo = np.array([model.joint(n).range[0] for n in jid])
    hi = np.array([model.joint(n).range[1] for n in jid])
    names = list(jid)
    bodies = [model.body(n).id for n in ("upper", "fore", "hand")]
    tip = model.site("tip").id

    def fk(q_rad: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:     # joint ranges/q are radians
        for n, v in zip(names, q_rad):
            data.qpos[jid[n]] = v
        mujoco.mj_forward(model, data)
        pts = [data.xpos[b].copy() for b in bodies] + [data.site_xpos[tip].copy()]
        return pts[-1], pts

    def inside_bag(p: np.ndarray) -> float:
        dx = max(BAG["x"][0] - MARGIN - p[0], p[0] - BAG["x"][1] - MARGIN, 0)
        dy = max(BAG["y"][0] - MARGIN - p[1], p[1] - BAG["y"][1] - MARGIN, 0)
        dz = max(BAG["z"][0] - MARGIN - p[2], p[2] - BAG["z"][1] - MARGIN, 0)
        return 1.0 if (dx == 0 and dy == 0 and dz == 0) else 0.0

    def cost(q: np.ndarray, target: np.ndarray) -> float:
        t, pts = fk(q)
        err = float(np.linalg.norm(t - target))
        pen = 0.0
        segs = list(zip(pts[:-1], pts[1:]))
        for p0, p1 in segs:
            for s in np.linspace(0, 1, 6):
                pen += inside_bag(p0 + s * (p1 - p0))
        return err + 0.05 * pen + 0.0005 * abs(q[3]) / 90     # tiny preference for a straight wrist

    rng = np.random.default_rng(a.seed)
    out = {}
    print(f"{'pose':15} {'base':>6} {'shoulder':>9} {'elbow':>6} {'wrist':>6}   tip error mm   bag hits")
    for name, tgt in targets().items():
        best, best_c = None, 1e9
        for _ in range(60):                                   # random restarts
            q = rng.uniform(lo, hi)
            step = (hi - lo) / 8
            c = cost(q, tgt)
            for _ in range(400):                              # coordinate descent with shrinking steps
                improved = False
                for i in range(4):
                    for d in (+1, -1):
                        q2 = q.copy(); q2[i] = float(np.clip(q2[i] + d * step[i], lo[i], hi[i]))
                        c2 = cost(q2, tgt)
                        if c2 < c:
                            q, c, improved = q2, c2, True
                if not improved:
                    step *= 0.5
                    if step.max() < 0.0005:
                        break
            if c < best_c:
                best, best_c = q.copy(), c
        t, pts = fk(best)
        hits = sum(inside_bag(p0 + s * (p1 - p0)) for p0, p1 in zip(pts[:-1], pts[1:]) for s in np.linspace(0, 1, 6))
        proto = {n: round(HOME[n] + math.degrees(float(v)), 1) for n, v in zip(names, best)}
        out[name] = proto
        err_mm = float(np.linalg.norm(t - tgt)) * 1000
        print(f"{name:15} {proto['base']:6.1f} {proto['shoulder']:9.1f} {proto['elbow']:6.1f} {proto['wrist']:6.1f}"
              f"   {err_mm:8.1f}      {int(hits)}")
        print(f"{'':15} tip at x={t[0]*1000:.0f} y={t[1]*1000:.0f} z={t[2]*1000:.0f} mm"
              f"  (target {tgt[0]*1000:.0f} {tgt[1]*1000:.0f} {tgt[2]*1000:.0f})")
    if a.write:
        p = ROOT / "poses.json"
        saved = json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}
        saved.update(out)
        p.write_text(json.dumps(saved, indent=2) + "\n", encoding="utf-8")
        print("written to", p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
