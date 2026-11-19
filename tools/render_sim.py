"""Render the MuJoCo arm in each voice pose to PNGs for the README (offscreen: no window, no hardware).

    python tools/render_sim.py                       # writes media/sim-<pose>.png and media/sim-poses.png
    python tools/render_sim.py --gif                 # also media/sim-voice.gif: the voice poses, in motion
    python tools/render_sim.py --azimuth -110 --elevation -12 --distance 2.4    # move the camera

Each still is reached the way the real board is told to reach it: ARM 1, then one `J <joint> <deg>` per joint
from PRESETS, sent through SimLink. The GIF goes further: each voice command goes through the real
ArmController (same rate limits, same 10 Hz-per-joint sends) on a simulated clock, so the arm moves at its
real speeds. The GIF needs Pillow.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from james_gestures.arm import PRESETS  # noqa: E402
from james_gestures.sim import HOME, SimLink  # noqa: E402

POSES = ["home", "left shoulder", "right shoulder", "left hip", "right hip"]
OUT = ROOT / "media"


def pose(link: SimLink, name: str) -> None:
    link.send("ARM 1")
    target = {**HOME, **PRESETS[name]}
    for joint, deg in target.items():
        link.send(f"J {joint} {deg:.1f}")


def caption(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    h = max(40, img.shape[0] // 13)
    cv2.rectangle(out, (0, 0), (img.shape[1], h), (23, 19, 15), -1)
    scale = h / 60
    cv2.putText(out, text, (int(h * 0.3), int(h * 0.68)), cv2.FONT_HERSHEY_SIMPLEX, scale, (243, 236, 221),
                max(1, round(scale * 2)), cv2.LINE_AA)
    return out


def make_gif(renderer, cam, path: Path, fps: int = 12, speed: float = 1.5, hold_s: float = 1.2) -> None:
    """Voice commands through the real ArmController on a fake clock, one frame every 1/fps of playback."""
    from PIL import Image

    from james_gestures.arm import ArmConfig, ArmController

    class Clock:
        t = 0.0

        def __call__(self):
            return self.t

    link, clock = SimLink(show=False), Clock()
    arm = ArmController(link, cfg=ArmConfig(mirror=False, poses_file=None), clock=clock)
    frames, dt, frame_every = [], 1 / 120, speed / fps
    next_frame = 0.0

    def run(seconds: float, text: str) -> None:
        nonlocal next_frame
        end = clock.t + seconds
        while clock.t < end:
            clock.t += dt
            arm.tick()
            if clock.t >= next_frame:
                next_frame += frame_every
                renderer.update_scene(link.data, camera=cam)
                frames.append(caption(renderer.render(), text))

    run(0.8, 'say "james ..."')
    for name in ["left shoulder", "right shoulder", "left hip", "right hip", "home"]:
        assert arm.voice(name), name
        far = max(abs(arm.target[j] - arm.pos[j]) / arm.j[j].max_speed for j in arm.target)
        run(far + 0.3 + hold_s, f'"james {name}"')
    imgs = [Image.fromarray(f).quantize(colors=128, method=Image.Quantize.MEDIANCUT) for f in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=round(1000 / fps), loop=0, optimize=True)
    print(f"wrote {path.relative_to(ROOT)} ({len(frames)} frames, {path.stat().st_size / 1e6:.1f} MB)")
    link.close()


def make_ppo_gif(renderer, cam, path: Path, policy_path: str, n_targets: int = 5, fps: int = 12,
                 speed: float = 1.5, seed: int = 11) -> None:
    """The trained policy driving the arm through the real ArmController to targets near the wearer.

    The targets are the first `n_targets` near-body targets from a fixed seed (not picked by hand); the caption
    says whether each was reached. A yellow ball marks the target."""
    import mujoco
    from PIL import Image

    from james_gestures.arm import ArmConfig, ArmController
    from rl.deploy import run_reach
    from rl.env import ReachEnv
    from rl.ppo import Policy

    class Clock:
        t = 0.0

        def __call__(self):
            return self.t

    policy, clock = Policy(policy_path), Clock()
    link = SimLink(show=False)
    arm = ArmController(link, cfg=ArmConfig(mirror=False, poses_file=None), clock=clock)
    arm.voice("home")                                  # arms the controller, already at home
    sampler = ReachEnv(seed=seed)
    targets = [sampler.sample_target(near_body=True) for _ in range(n_targets)]
    frames, state = [], {"next": 0.0, "text": ""}
    frame_every, sub = speed / fps, 1 / 60

    def draw(target):
        renderer.update_scene(link.data, camera=cam)
        scn = renderer.scene
        if scn.ngeom < scn.maxgeom:
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]),
                                np.asarray(target, float), np.eye(3).flatten(), np.array([1.0, 0.8, 0.1, 1.0]))
            scn.ngeom += 1
        frames.append(caption(renderer.render(), state["text"]))

    def wait_for(target):
        def wait(dt):
            end = clock.t + dt
            while clock.t < end - 1e-9:
                clock.t += sub
                arm.tick()
                if clock.t >= state["next"]:
                    state["next"] += frame_every
                    draw(target)
        return wait

    for i, tgt in enumerate(targets, 1):
        state["text"] = f"PPO policy: target {i} of {n_targets}"
        res = run_reach(arm, policy, tgt, wait=wait_for(tgt))
        state["text"] = f"target {i}: " + ("reached" if res["reached"] else f"missed by {res['dist_mm']:.0f} mm")
        wait_for(tgt)(0.8)
        print(f"target {i}: reached={res['reached']} dist={res['dist_mm']:.0f} mm "
              f"min clearance={res['min_clear_mm']:.0f} mm shielded={res['shielded']}")
    imgs = [Image.fromarray(f).quantize(colors=128, method=Image.Quantize.MEDIANCUT) for f in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:], duration=round(1000 / fps), loop=0, optimize=True)
    print(f"wrote {path.relative_to(ROOT)} ({len(frames)} frames, {path.stat().st_size / 1e6:.1f} MB)")
    link.close()


def main() -> int:
    import mujoco

    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--azimuth", type=float, default=-120.0, help="degrees; -90 = straight behind the wearer")
    ap.add_argument("--elevation", type=float, default=-14.0, help="degrees; negative looks down")
    ap.add_argument("--distance", type=float, default=2.0, help="metres from the look-at point")
    ap.add_argument("--lookat", type=float, nargs=3, default=[0.12, -0.08, 0.15], metavar=("X", "Y", "Z"))
    ap.add_argument("--gif", action="store_true", help="also write media/sim-voice.gif (needs Pillow)")
    ap.add_argument("--gif-width", type=int, default=480)
    ap.add_argument("--ppo-gif", action="store_true",
                    help="only write media/ppo-reach.gif: the trained PPO policy reaching targets near the wearer")
    ap.add_argument("--policy", default=str(ROOT / "rl" / "reach_policy.pt"))
    a = ap.parse_args()

    link = SimLink(show=False)
    renderer = mujoco.Renderer(link.model, height=a.height, width=a.width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = a.lookat
    cam.azimuth, cam.elevation, cam.distance = a.azimuth, a.elevation, a.distance

    if a.ppo_gif:
        gw = a.gif_width
        gif_renderer = mujoco.Renderer(link.model, height=round(gw * a.height / a.width), width=gw)
        make_ppo_gif(gif_renderer, cam, OUT / "ppo-reach.gif", a.policy)
        gif_renderer.close()
        return 0

    OUT.mkdir(exist_ok=True)
    panels = []
    for name in POSES:
        pose(link, name)
        renderer.update_scene(link.data, camera=cam)
        img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        path = OUT / f"sim-{name.replace(' ', '-')}.png"
        cv2.imwrite(str(path), img)
        print("wrote", path.relative_to(ROOT))
        label = img.copy()
        cv2.rectangle(label, (0, 0), (a.width, 56), (23, 19, 15), -1)
        cv2.putText(label, f'"james {name}"', (16, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (243, 236, 221), 2,
                    cv2.LINE_AA)
        panels.append(label)
    strip = np.hstack(panels)
    strip = cv2.resize(strip, (strip.shape[1] // 2, strip.shape[0] // 2), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(OUT / "sim-poses.png"), strip)
    print("wrote", (OUT / "sim-poses.png").relative_to(ROOT))
    if a.gif:
        gw = a.gif_width
        gif_renderer = mujoco.Renderer(link.model, height=round(gw * a.height / a.width), width=gw)
        make_gif(gif_renderer, cam, OUT / "sim-voice.gif")
        gif_renderer.close()
    renderer.close()
    link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
