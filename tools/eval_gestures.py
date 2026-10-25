"""Synthetic evaluation of the gesture stack: GestureReader + ArmController, no camera, no MediaPipe.

    .venv/Scripts/python.exe tools/eval_gestures.py            # full sweep, compact tables
    .venv/Scripts/python.exe tools/eval_gestures.py --quick    # fewer hand placements
    .venv/Scripts/python.exe tools/eval_gestures.py --seed 3 --fps 30 --hold 3.0

What it does
  * builds clean 21-point hands with handpose.synth for open / point / pinch / peace / fist / half (neutral),
    at several sizes, frame positions and in-plane rotations
  * adds Gaussian jitter to every landmark on every frame, sigma = level x palm size (wrist to middle knuckle),
    for several levels (0 .. 8 %). We do NOT know MediaPipe's real error on this rig: treat the levels as a sweep.
  * plays a 30 fps sequence of poses (3 s each, 200 ms linear morph between poses) through GestureReader and
    ArmController with a list-collecting link, and reports per jitter level and pose:
        acc      fraction of frames after the first correct frame that still report the right gesture
        lat      confirm latency, frames / ms from the first fully-formed frame of the pose to the first
                 correct frame (median over placements; also the worst case)
        miss     fraction of holds that never reported the right gesture
        flips    gesture-name changes per 1000 frames while holding (after confirm)
  * safety checks per jitter level: fist-from-open -> STOP latency, pinch inside a fist is NOT a pinch and does
    not close the claw, ARM happens only after the 1 s open-palm hold (and never from point / half / fist)
  * joystick check: POINT held and hand moved left / right / up / down -> base / shoulder move the right way,
    stay inside limits and inside max_speed * dt per frame

Every number printed comes from this run. Nothing in the package is changed.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from james_gestures import ArmConfig, ArmController, GestureReader  # noqa: E402
from james_gestures import handpose as HP  # noqa: E402

W, H = 640, 480
EXPECT = {"open": "open_palm", "point": "point", "pinch": "pinch", "peace": "peace", "fist": "fist", "half": "hand"}
SEQUENCE = ["half", "open", "point", "half", "peace", "pinch", "open", "fist", "half"]
LEVELS = (0.0, 0.005, 0.01, 0.02, 0.04, 0.08)


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class ListLink:
    """Collects (time, line) so latencies can be measured."""

    def __init__(self, clock):
        self.clock, self.log = clock, []

    def send(self, line):
        self.log.append((self.clock.t, line))

    def lines(self):
        return [l for _, l in self.log]


def placements(quick: bool):
    sizes = (70, 120, 200) if not quick else (120,)
    positions = ((320, 400), (120, 160), (540, 300)) if not quick else ((320, 400),)
    angles = (-25, 0, 25) if not quick else (0,)
    return [(s, p, a) for s in sizes for p in positions for a in angles]


def clean(pose, size, pos, angle):
    return np.asarray(HP.synth(pose, pos, size, angle), float)


class Jitter:
    """Per-landmark Gaussian noise, sigma = level x palm size. tau = 0: white (independent every frame).
    tau > 0: an Ornstein-Uhlenbeck random walk with that time constant in seconds, i.e. slow drift of each
    point that a smoothing filter cannot average away (closer to how a landmark model actually errs)."""

    def __init__(self, level, size, rng, dt, tau=0.0):
        self.sigma, self.rng, self.tau = level * size, rng, tau
        self.rho = math.exp(-dt / tau) if tau > 0 else 0.0
        self.off = None

    def __call__(self, pts):
        if self.sigma <= 0:
            return [tuple(p) for p in pts]
        white = self.rng.normal(0.0, self.sigma, pts.shape)
        if self.tau <= 0:
            return [tuple(p) for p in pts + white]
        if self.off is None:
            self.off = white
        else:
            self.off = self.rho * self.off + math.sqrt(1 - self.rho ** 2) * white
        return [tuple(p) for p in pts + self.off]


TAU = 0.0   # set from --tau in main()


def jitter(pts, level, size, rng):
    """Kept for the per-call sites: one Jitter per (rig) so a correlated walk is continuous within a run."""
    key = id(rng)
    j = _JIT.get(key)
    if j is None or j.sigma != level * size:
        j = _JIT[key] = Jitter(level, size, rng, 1.0 / _FPS[0], TAU)
    return j(pts)


_JIT: dict = {}
_FPS = [30.0]


class Rig:
    def __init__(self, fps):
        self.clock = Clock()
        self.link = ListLink(self.clock)
        self.reader = GestureReader(clock=self.clock)
        self.arm = ArmController(self.link, cfg=ArmConfig(mirror=False), clock=self.clock)
        self.dt = 1.0 / fps

    def step(self, pts):
        self.clock.t += self.dt
        g = self.reader.read(pts, W, H, t=self.clock.t)
        self.arm.update(g)
        return g


# ---------------------------------------------------------------- pose holds

def run_sequence(level, size, pos, angle, fps, hold_s, morph_s, rng):
    """One placement, the full pose sequence. Returns per-pose stats and the arm log."""
    rig = Rig(fps)
    n_hold, n_morph = int(round(hold_s * fps)), int(round(morph_s * fps))
    out = []
    prev = None
    for pose in SEQUENCE:
        tgt = clean(pose, size, pos, angle)
        if prev is not None:
            for k in range(1, n_morph + 1):
                a = k / (n_morph + 1)
                rig.step(jitter(prev * (1 - a) + tgt * a, level, size, rng))
        names = []
        t_start = rig.clock.t
        for _ in range(n_hold):
            g = rig.step(jitter(tgt, level, size, rng))
            names.append(g.name)
        want = EXPECT[pose]
        first = next((i for i, n in enumerate(names) if n == want), None)
        if first is None:
            out.append((pose, dict(miss=1, acc=0.0, lat=None, flips=None, frames=len(names))))
        else:
            after = names[first:]
            acc = sum(n == want for n in after) / len(after)
            flips = sum(1 for a, b in zip(after, after[1:]) if a != b)
            out.append((pose, dict(miss=0, acc=acc, lat=first, flips=flips, frames=len(after), t_start=t_start)))
        prev = tgt
    return out, rig


def eval_holds(level, places, fps, hold_s, morph_s, rng):
    per = {p: dict(acc=[], lat=[], miss=0, flips=0, frames=0, n=0) for p in EXPECT}
    arm_ok, arm_total = 0, 0
    for size, pos, angle in places:
        res, rig = run_sequence(level, size, pos, angle, fps, hold_s, morph_s, rng)
        for pose, r in res:
            d = per[pose]
            d["n"] += 1
            if r["miss"]:
                d["miss"] += 1
                continue
            d["acc"].append(r["acc"])
            d["lat"].append(r["lat"])
            d["flips"] += r["flips"]
            d["frames"] += r["frames"]
    return per


# ---------------------------------------------------------------- safety checks

def arm_with_open(rig, level, size, pos, angle, rng, seconds=1.6):
    t0 = rig.clock.t
    tgt = clean("open", size, pos, angle)
    for _ in range(int(round(seconds / rig.dt))):
        rig.step(jitter(tgt, level, size, rng))
    arms = [t for t, l in rig.link.log if l == "ARM 1" and t > t0]
    return (arms[0] - t0) if arms else None


def check_arm_hold(level, places, fps, rng):
    """ARM only after >= arm_hold_s of open palm; never from point/half/fist; latency stats."""
    lat, early, never, false_arm = [], 0, 0, 0
    for size, pos, angle in places:
        rig = Rig(fps)
        for pose in ("half", "point", "fist", "peace"):
            tgt = clean(pose, size, pos, angle)
            for _ in range(int(round(1.5 * fps))):
                rig.step(jitter(tgt, level, size, rng))
        if any(l == "ARM 1" for l in rig.link.lines()):
            false_arm += 1
        t = arm_with_open(rig, level, size, pos, angle, rng)
        if t is None:
            never += 1
        else:
            lat.append(t)
            if t < rig.arm.cfg.arm_hold_s - 1e-9:
                early += 1
    return dict(lat=lat, early=early, never=never, false_arm=false_arm, n=len(places))


def check_fist_stop(level, places, fps, rng):
    """Armed with an open palm, then fist: time from the first fist frame to STOP."""
    lat, never = [], 0
    for size, pos, angle in places:
        rig = Rig(fps)
        arm_with_open(rig, level, size, pos, angle, rng)
        if not rig.arm.armed:
            never += 1
            continue
        t0 = rig.clock.t
        tgt = clean("fist", size, pos, angle)
        for _ in range(int(round(1.0 * fps))):
            rig.step(jitter(tgt, level, size, rng))
        stops = [t for t, l in rig.link.log if l == "STOP" and t > t0]
        if stops:
            lat.append(stops[0] - t0)
        else:
            never += 1
    return dict(lat=lat, never=never, n=len(places))


def check_pinch_in_fist(level, places, fps, rng):
    """Armed, then a fist with the thumb tip on the index tip (pinch ratio ~0): must not be a pinch, must not
    send CLAW 1, and the fist must still STOP."""
    bad_pinch, claw, no_stop = 0, 0, 0
    for size, pos, angle in places:
        rig = Rig(fps)
        arm_with_open(rig, level, size, pos, angle, rng)
        n_claw0 = rig.link.lines().count("CLAW 1")
        fist = clean("fist", size, pos, angle)
        for _ in range(int(round(0.5 * fps))):
            rig.step(jitter(fist, level, size, rng))
        pf = fist.copy()
        pf[4] = pf[8]
        saw_pinch = False
        for _ in range(int(round(1.5 * fps))):
            g = rig.step(jitter(pf, level, size, rng))
            saw_pinch |= g.pinch
        bad_pinch += saw_pinch
        claw += rig.link.lines().count("CLAW 1") > n_claw0
        no_stop += "STOP" not in rig.link.lines()
    return dict(bad_pinch=bad_pinch, claw=claw, no_stop=no_stop, n=len(places))


def check_half_open_pinch(level, places, fps, rng):
    """Control: a pinch made from an open hand IS a pinch and closes the claw (so the fist rule did not kill it)."""
    ok = 0
    for size, pos, angle in places:
        rig = Rig(fps)
        arm_with_open(rig, level, size, pos, angle, rng)
        tgt = clean("pinch", size, pos, angle)
        for _ in range(int(round(0.6 * fps))):
            rig.step(jitter(tgt, level, size, rng))
        ok += "CLAW 1" in rig.link.lines()
    return dict(ok=ok, n=len(places))


# ---------------------------------------------------------------- joystick

def check_joystick(level, fps, rng, size=120, seconds=1.0):
    """POINT held at centre (zero), then moved 30 % of the frame left / right / up / down.
    mirror=False here (demo.py flips the image itself and also uses mirror=False)."""
    results = {}
    moves = {"right": (0.3 * W, 0, "base", +1), "left": (-0.3 * W, 0, "base", -1),
             "up": (0, -0.3 * H, "shoulder", +1), "down": (0, 0.3 * H, "shoulder", -1)}
    for name, (dx, dy, joint, sign) in moves.items():
        rig = Rig(fps)
        arm_with_open(rig, level, size, (320, 300), 0, rng)
        armed = rig.arm.armed
        tgt = clean("point", size, (320, 300), 0)
        for _ in range(int(round(0.5 * fps))):
            rig.step(jitter(tgt, level, size, rng))
        start = dict(rig.arm.pos)
        moved = clean("point", size, (320 + dx, 300 + dy), 0)
        prev = dict(rig.arm.pos)
        rate_ok, limit_ok, others_still = True, True, True
        for _ in range(int(round(seconds * fps))):
            rig.step(jitter(moved, level, size, rng))
            for n, jt in rig.arm.j.items():
                step = abs(rig.arm.pos[n] - prev[n])
                if step > jt.max_speed * rig.dt + 1e-6:
                    rate_ok = False
                if not (jt.lo <= rig.arm.pos[n] <= jt.hi):
                    limit_ok = False
                if n != joint and abs(rig.arm.pos[n] - start[n]) > 1e-9:
                    others_still = False
            prev = dict(rig.arm.pos)
        delta = rig.arm.pos[joint] - start[joint]
        results[name] = dict(armed=armed, joint=rig.arm.active_joint, want=joint, delta=delta,
                             dir_ok=(delta * sign > 0), rate_ok=rate_ok, limit_ok=limit_ok,
                             others_still=others_still, drive=rig.arm.drive)
    return results


# ---------------------------------------------------------------- margins

def _synth_curls(curls, spread, thumb, size=120, pos=(320, 400)):
    saved = HP.POSES.get("_tmp")
    HP.POSES["_tmp"] = (curls, spread, thumb)
    try:
        return clean("_tmp", size, pos, 0)
    finally:
        if saved is None:
            HP.POSES.pop("_tmp", None)
        else:
            HP.POSES["_tmp"] = saved


def steady_name(pts, fps, frames=20):
    rig = Rig(fps)
    g = None
    for _ in range(frames):
        g = rig.step([tuple(p) for p in pts])
    return g


def report_margins(fps):
    """Where the pose boundaries sit, in the synth hand's curl parameter (0 straight .. 1 folded) and in the
    finger ratio the code thresholds on. No jitter: this is about how precisely a human has to shape the hand."""
    print("MARGINS: steady-state reading of a clean hand while its shape varies (no jitter)")
    print("  thresholds: FIST_RATIO 1.05 OPEN_RATIO 1.75 UP_ON/OFF 1.50/1.38 DOWN 1.30 PINCH_MIN_INDEX 1.12 | "
          "fist_openness 0.15 fist_max_ratio 1.10 open_openness 0.80 (release 0.30) pinch_ratio 0.33 (x1.4 release)")
    print("  (a) all four fingers curl together, thumb out (open palm -> neutral -> fist):")
    row = []
    for c in [i / 20 for i in range(21)]:
        g = steady_name(_synth_curls((c, c, c, c), 1.0, "out"), fps)
        row.append(f"{c:.2f}:{g.name}(op {g.openness:.2f}, r {max(g.ratios):.2f})")
    for i in range(0, len(row), 3):
        print("     " + " | ".join(row[i:i + 3]))
    print("  (b) POINT: index straight, the other three at curl c, thumb across:")
    row = []
    for c in [0.3 + i * 0.05 for i in range(15)]:
        g = steady_name(_synth_curls((0, c, c, c), 0.8, "across"), fps)
        row.append(f"{c:.2f}:{g.name}(mid r {g.ratios[1]:.2f})")
    for i in range(0, len(row), 4):
        print("     " + " | ".join(row[i:i + 4]))
    print("  (c) PEACE: index+middle at curl c, ring+pinky folded, thumb across:")
    row = []
    for c in [i * 0.05 for i in range(11)]:
        g = steady_name(_synth_curls((c, c, 1, 1), 2.2, "across"), fps)
        row.append(f"{c:.2f}:{g.name}(idx r {g.ratios[0]:.2f})")
    for i in range(0, len(row), 4):
        print("     " + " | ".join(row[i:i + 4]))
    print("  (d) PINCH gap: open-ish hand (curls .45/.55/.65/.75), thumb tip at gap g x palm from the index tip:")
    row = []
    base = clean("pinch", 120, (320, 400), 0)
    palm = HP.palm_size(base)
    for gidx in [0.0, 0.1, 0.2, 0.3, 0.33, 0.36, 0.4, 0.46, 0.5, 0.6]:
        pts = base.copy()
        pts[4] = pts[8] + np.array([-1.0, 0.0]) * gidx * palm
        g = steady_name(pts, fps)
        row.append(f"{gidx:.2f}:{g.name}")
    print("     " + " | ".join(row))
    print("  (e) synth pose ratios (index..pinky) and openness, the numbers the HUD shows:")
    for pose in ("open", "half", "fist", "point", "peace", "pinch"):
        g = steady_name(clean(pose, 120, (320, 400), 0), fps)
        print(f"     {pose:>6}: {g.name:<9} ratios {' '.join(f'{r:.2f}' for r in g.ratios)}  openness {g.openness:.2f}"
              f"  pinch_ratio {g.pinch_ratio:.2f}")
    print()


# ---------------------------------------------------------------- report

def fmt_lat(frames, fps):
    if not frames:
        return "   -"
    return f"{statistics.median(frames):4.0f}f/{1000 * statistics.median(frames) / fps:4.0f}ms (max {max(frames):.0f}f)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--hold", type=float, default=3.0, help="seconds per pose")
    ap.add_argument("--morph", type=float, default=0.2, help="seconds of linear morph between poses")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--levels", type=float, nargs="*", default=list(LEVELS), help="jitter sigma / palm size")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--tau", type=float, default=0.0,
                    help="jitter time constant in s (0 = white noise; e.g. 0.3 = slow per-landmark drift)")
    ap.add_argument("--no-margins", action="store_true", help="skip the pose-boundary sweep")
    a = ap.parse_args()
    global TAU
    TAU = a.tau
    _FPS[0] = a.fps
    rng = np.random.default_rng(a.seed)
    places = placements(a.quick)
    fps = a.fps
    print(f"synthetic eval: {len(places)} placements (size x position x rotation), {fps:g} fps, "
          f"{a.hold:g} s per pose, {a.morph:g} s morph, seed {a.seed}, "
          f"jitter {'white' if a.tau <= 0 else f'correlated tau={a.tau:g}s'}")
    if not a.no_margins:
        report_margins(fps)
    print(f"sequence: {' -> '.join(SEQUENCE)}")
    print()

    # ---- pose holds
    print("POSE HOLDS   acc = frames correct after confirm | lat = median confirm latency from first full frame "
          "(max) | miss = holds never recognised | flips/1k = gesture changes per 1000 held frames")
    hdr = f"{'jitter':>7} {'pose':>6} {'acc':>7} {'lat':>24} {'miss':>6} {'flips/1k':>9}"
    print(hdr)
    summary = {}
    for lvl in a.levels:
        per = eval_holds(lvl, places, fps, a.hold, a.morph, rng)
        summary[lvl] = per
        for pose in ("open", "point", "pinch", "peace", "fist", "half"):
            d = per[pose]
            acc = statistics.mean(d["acc"]) if d["acc"] else 0.0
            flips = 1000.0 * d["flips"] / d["frames"] if d["frames"] else float("nan")
            print(f"{100 * lvl:6.1f}% {pose:>6} {100 * acc:6.1f}% {fmt_lat(d['lat'], fps):>24} "
                  f"{d['miss']:>3}/{d['n']:<2} {flips:9.1f}")
        print()

    # ---- safety
    print("SAFETY (per jitter level, over the same placements)")
    print(f"{'jitter':>7} {'ARM lat med/max s':>18} {'early':>6} {'noARM':>6} {'falseARM':>9} "
          f"{'STOP lat med/max s':>19} {'noSTOP':>7} {'fist-pinch=pinch':>17} {'CLAW1':>6} {'noSTOP':>7} {'openpinch claw':>15}")
    for lvl in a.levels:
        ar = check_arm_hold(lvl, places, fps, rng)
        st = check_fist_stop(lvl, places, fps, rng)
        pf = check_pinch_in_fist(lvl, places, fps, rng)
        op = check_half_open_pinch(lvl, places, fps, rng)
        arl = f"{statistics.median(ar['lat']):.2f}/{max(ar['lat']):.2f}" if ar["lat"] else "-"
        stl = f"{statistics.median(st['lat']):.2f}/{max(st['lat']):.2f}" if st["lat"] else "-"
        print(f"{100 * lvl:6.1f}% {arl:>18} {ar['early']:>6} {ar['never']:>6} {ar['false_arm']:>9} "
              f"{stl:>19} {st['never']:>7} {pf['bad_pinch']:>14}/{pf['n']:<2} {pf['claw']:>6} {pf['no_stop']:>7} "
              f"{op['ok']:>12}/{op['n']:<2}")
    print("  ARM lat: seconds of open palm before 'ARM 1' (config arm_hold_s=1.0; early = before that).")
    print("  STOP lat: seconds from first fist frame to 'STOP' (config stop_hold_s=0.3 + pose confirm + smoothing).")
    print("  fist-pinch: thumb tip placed on the index tip of a fist; should read as fist (STOP), never pinch/CLAW 1.")
    print("  openpinch claw: control, a pinch made from an open hand must still send CLAW 1.")
    print()

    # ---- joystick
    print("JOYSTICK (POINT held, hand moved 30 % of the frame; mirror=False as in demo.py)")
    print(f"{'jitter':>7} {'move':>6} {'joint':>9} {'delta deg':>10} {'dir':>4} {'rate':>5} {'limits':>7} {'others':>7} {'drive':>6}")
    for lvl in a.levels:
        js = check_joystick(lvl, fps, rng)
        for mv, r in js.items():
            print(f"{100 * lvl:6.1f}% {mv:>6} {str(r['joint']):>9} {r['delta']:+10.1f} "
                  f"{'ok' if r['dir_ok'] else 'BAD':>4} {'ok' if r['rate_ok'] else 'BAD':>5} "
                  f"{'ok' if r['limit_ok'] else 'BAD':>7} {'ok' if r['others_still'] else 'BAD':>7} {r['drive']:+6.2f}")
    print("  dir: sign of the joint change matches the move. rate: every frame <= max_speed*dt. others: other joints untouched.")


if __name__ == "__main__":
    main()
