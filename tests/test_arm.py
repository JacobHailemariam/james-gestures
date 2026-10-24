"""Tests for the gesture reader and the arm controller, with synthetic hands (no camera, no MediaPipe)."""
import pytest

from james_gestures import ArmConfig, ArmController, GestureReader, PrintLink
from james_gestures import handpose as HP
from james_gestures.gestures import GestureFrame

W, H = 640, 480


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def hand(pose, x=320, y=400, size=120):
    return HP.synth(pose, (x, y), size)


def run(reader, arm, clock, pose, seconds, dt=1 / 30, **kw):
    g = None
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        g = reader.read(hand(pose, **kw) if pose else None, W, H, t=clock.t)
        arm.update(g)
    return g


@pytest.fixture
def rig():
    clock = Clock()
    link = PrintLink(quiet=True)
    return GestureReader(clock=clock), ArmController(link, cfg=ArmConfig(mirror=False), clock=clock), link, clock


# ---------- gestures ----------

@pytest.mark.parametrize("pose,name", [("open", "open_palm"), ("fist", "fist"), ("point", "point"),
                                       ("peace", "peace"), ("pinch", "pinch")])
def test_poses(pose, name):
    r, t = GestureReader(), 0.0
    for _ in range(15):
        t += 1 / 30
        g = r.read(hand(pose), W, H, t=t)
    assert g.name == name


def test_one_noisy_frame_changes_nothing():
    r, t = GestureReader(), 0.0
    for _ in range(15):
        t += 1 / 30
        g = r.read(hand("open"), W, H, t=t)
    t += 1 / 30
    g = r.read(hand("point"), W, H, t=t)          # a single frame of another pose
    assert g.open_palm


def test_pinch_that_starts_in_a_fist_is_not_a_pinch():
    r, t = GestureReader(), 0.0
    for _ in range(15):
        t += 1 / 30
        r.read(hand("fist"), W, H, t=t)
    # fist with thumb on the index: force the pinch ratio low while the fingers are still folded
    pts = hand("fist")
    pts[4] = pts[8]
    for _ in range(10):
        t += 1 / 30
        g = r.read(pts, W, H, t=t)
    assert not g.pinch


def test_no_hand_is_empty():
    g = GestureReader().read(None, W, H, t=1.0)
    assert g.hand is None and g.name == "none"


# ---------- arm safety ----------

def test_starts_disarmed_and_ignores_motion(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "point", 1.0, x=50)
    assert not arm.armed
    assert not any(l.startswith("J ") for l in link.log)


def test_open_palm_arms_after_hold(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 0.5)
    assert not arm.armed
    run(r, arm, clock, "open", 0.8)
    assert arm.armed and "ARM 1" in link.log


def test_fist_stops(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "fist", 0.6)
    assert not arm.armed and arm.state == "STOPPED" and "STOP" in link.log


def test_pinch_closes_claw_and_release_opens(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "pinch", 0.4)
    assert "CLAW 1" in link.log
    run(r, arm, clock, "open", 0.4)
    assert link.log[-1] != "CLAW 1" and "CLAW 0" in link.log


def test_joint_speed_is_limited(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    start = arm.pos["base"]
    run(r, arm, clock, "open", 0.5, x=630)               # jump the hand to the far edge
    moved = abs(arm.pos["base"] - start)
    assert moved <= arm.j["base"].max_speed * 0.5 + 1e-6
    assert arm.j["base"].lo <= arm.pos["base"] <= arm.j["base"].hi


def test_hand_lost_holds_then_disarms(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, None, 0.5)
    assert arm.armed and arm.state == "HOLD"
    run(r, arm, clock, None, 2.0)
    assert not arm.armed


def test_presets_only_when_armed(rig):
    r, arm, link, clock = rig
    assert not arm.goto_preset("raise")
    run(r, arm, clock, "open", 1.5)
    assert arm.goto_preset("raise")
    assert arm.target["shoulder"] == 150


def test_targets_always_clamped():
    clock = Clock()
    arm = ArmController(PrintLink(quiet=True), clock=clock)
    arm.armed = True
    for _ in range(200):
        clock.t += 0.05
        arm.update(GestureFrame(hand=[(0, 0)] * 21, norm=(5.0, -5.0), palm_px=100))
    for n, j in arm.j.items():
        assert j.lo <= arm.pos[n] <= j.hi
