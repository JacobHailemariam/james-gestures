"""The point/peace joystick: one joint at a time, from the offset to where the pose began."""
from tests.test_arm import rig, run  # noqa: F401  (fixture)


def test_open_palm_is_idle(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    before = dict(arm.pos)
    run(r, arm, clock, "open", 1.0, x=600, y=100)             # wave the open hand around
    assert arm.pos == before and arm.active_joint is None


def test_point_right_turns_base_only(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, x=320)                   # clutch in at the centre: zero
    start = dict(arm.pos)
    run(r, arm, clock, "point", 1.0, x=320 + 0.3 * 640)       # hand well right of the zero
    assert arm.active_joint == "base" and arm.drive == 1.0
    assert arm.pos["base"] > start["base"]
    assert all(arm.pos[k] == start[k] for k in ("shoulder", "elbow", "wrist"))


def test_point_up_lifts_shoulder(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, y=400)
    s0 = arm.pos["shoulder"]
    run(r, arm, clock, "point", 1.0, y=400 - 0.3 * 480)       # hand up (image y shrinks)
    assert arm.active_joint == "shoulder" and arm.pos["shoulder"] > s0


def test_peace_left_right_is_dead_while_wrist_locked(rig):
    r, arm, link, clock = rig                                  # default: lock_wrist=True
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "peace", 0.5, x=320)
    before = dict(arm.pos)
    run(r, arm, clock, "peace", 1.0, x=320 - 0.3 * 640)
    assert arm.active_joint is None and arm.pos == before
    run(r, arm, clock, "peace", 1.0, x=320, y=400 - 0.3 * 480)   # up/down still drives the elbow
    assert arm.active_joint == "elbow"


def test_peace_drives_wrist_when_unlocked():
    from james_gestures import ArmConfig, ArmController, GestureReader, PrintLink
    from tests.test_arm import Clock
    clock, link = Clock(), PrintLink(quiet=True)
    r = GestureReader(clock=clock)
    arm = ArmController(link, cfg=ArmConfig(mirror=False, lock_wrist=False), clock=clock)
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "peace", 0.5, x=320)
    w0 = arm.pos["wrist"]
    run(r, arm, clock, "peace", 1.0, x=320 - 0.3 * 640)
    assert arm.active_joint == "wrist" and arm.pos["wrist"] < w0


def test_small_offset_is_dead_zone(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, x=320)
    start = dict(arm.pos)
    run(r, arm, clock, "point", 1.0, x=320 + 0.02 * 640)      # 2% of the frame: jitter
    assert arm.pos == start and arm.drive == 0.0


def test_unpoint_stops_and_rezeroes(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 1.0, x=320 + 0.3 * 640)
    run(r, arm, clock, "open", 0.5, x=320 + 0.3 * 640)        # let go, hand stays where it is
    held = dict(arm.pos)
    assert arm.active_joint is None
    run(r, arm, clock, "point", 0.5, x=320 + 0.3 * 640)       # point again from here: new zero, no motion
    assert arm.pos == held
