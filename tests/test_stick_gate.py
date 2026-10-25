"""Axis lock and the move timer: a twitch can't switch joints mid-move, and a move ends after stick_max_s
until an open palm is shown."""
from tests.test_arm import rig, run  # noqa: F401  (fixture)

RIGHT = 320 + 0.3 * 640
UP = 400 - 0.3 * 480


def test_twitch_on_the_other_axis_does_not_switch_joints(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, x=320, y=400)                 # clutch in
    run(r, arm, clock, "point", 0.5, x=RIGHT, y=400)               # moving the base
    assert arm.active_joint == "base"
    s0 = arm.pos["shoulder"]
    run(r, arm, clock, "point", 1.0, x=320 + 0.1 * 640, y=UP)      # now mostly "up": would pick shoulder before
    assert arm.active_joint == "base" and arm.pos["shoulder"] == s0


def test_unpoint_frees_the_axis(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, x=320, y=400)
    run(r, arm, clock, "point", 0.5, x=RIGHT, y=400)
    run(r, arm, clock, "open", 0.5)                                # un-point
    run(r, arm, clock, "point", 0.5, x=320, y=400)
    run(r, arm, clock, "point", 0.5, x=320, y=UP)
    assert arm.active_joint == "shoulder"


def test_move_ends_after_stick_max_s_until_open_palm(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, x=320, y=400)
    run(r, arm, clock, "point", 2.5, x=RIGHT, y=400)               # inside the 3 s window
    assert arm.active_joint == "base" and not arm.stick_gated
    run(r, arm, clock, "point", 1.0, x=RIGHT, y=400)               # crosses 3 s
    assert arm.stick_gated and arm.active_joint is None
    b = arm.pos["base"]
    run(r, arm, clock, "point", 2.0, x=RIGHT, y=400)               # keep pushing: nothing
    assert arm.pos["base"] == b and arm.stick_gated
    run(r, arm, clock, "open", 0.5)                                # palm: stick lives again
    assert not arm.stick_gated
    run(r, arm, clock, "point", 0.5, x=320, y=400)
    run(r, arm, clock, "point", 1.0, x=320 - 0.3 * 640, y=400)     # back the other way (it hit 270 above)
    assert arm.active_joint == "base" and arm.pos["base"] < b


def test_stop_and_rearm_clear_the_gate(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, "open", 1.5)
    run(r, arm, clock, "point", 0.5, x=320, y=400)
    run(r, arm, clock, "point", 3.5, x=RIGHT, y=400)
    assert arm.stick_gated
    run(r, arm, clock, "fist", 0.6)
    assert not arm.armed and not arm.stick_gated
