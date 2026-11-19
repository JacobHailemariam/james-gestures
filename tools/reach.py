"""Reach a point with the trained PPO policy, on the simulator, the real arm, or both.

    python tools/reach.py --sim --near 3                 # three targets near the wearer, in the MuJoCo viewer
    python tools/reach.py --sim --target 200 -60 520     # one target, mm in the arm.xml frame (x across the back,
                                                         # y out of the bag, z up from the bag bottom)
    python tools/reach.py --port COM3 --target 200 -60 520         # the real arm (and add --sim to watch both)

Every pose the policy proposes is checked against the keep-out zones first, then sent through ArmController
(joint limits, deg/s caps, 10 Hz per joint) as ordinary "J <joint> <deg>" lines; the firmware slews on top.
Ctrl+C sends STOP.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from james_gestures import ArmConfig, ArmController, PrintLink, SerialLink  # noqa: E402
from rl.deploy import run_reach  # noqa: E402
from rl.env import ReachEnv  # noqa: E402
from rl.ppo import Policy  # noqa: E402


class Tee:
    def __init__(self, *links):
        self.links = links

    def send(self, line):
        for l in self.links:
            l.send(line)

    def close(self):
        for l in self.links:
            if hasattr(l, "close"):
                l.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"), help="target point, mm")
    ap.add_argument("--near", type=int, default=0, help="reach this many random targets near the wearer")
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--sim", action="store_true", help="show the MuJoCo viewer")
    ap.add_argument("--port", help="serial port of the arm board")
    ap.add_argument("--policy", default=str(ROOT / "rl" / "reach_policy.pt"))
    a = ap.parse_args()
    if not a.target and not a.near:
        ap.error("give --target X Y Z or --near N")

    links = []
    if a.sim:
        from james_gestures.sim import SimLink
        links.append(SimLink())
    if a.port:
        links.append(SerialLink(a.port))
    link = Tee(*links) if links else PrintLink()
    arm = ArmController(link, cfg=ArmConfig(mirror=False, poses_file=None))
    policy = Policy(a.policy)
    sampler = ReachEnv(seed=a.seed)
    targets = [[v / 1000 for v in a.target]] if a.target else [sampler.sample_target(near_body=True)
                                                                for _ in range(a.near)]
    try:
        arm.voice("home")                               # arm the controller (voice-style), starting from home
        for i, tgt in enumerate(targets, 1):
            print(f"target {i}: ({tgt[0]*1000:.0f}, {tgt[1]*1000:.0f}, {tgt[2]*1000:.0f}) mm")
            res = run_reach(arm, policy, tgt)
            print(f"  reached={res['reached']}  off by {res['dist_mm']:.0f} mm  closest to a keep-out zone "
                  f"{res['min_clear_mm']:.0f} mm  shield={'stopped it' if res['shielded'] else 'not needed'}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        link.send("STOP")
        if hasattr(link, "close"):
            link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
