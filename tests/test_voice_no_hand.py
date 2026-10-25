"""Voice must work with no hand in the picture: it arms by itself and the hand-lost disarm waits for a hand."""
from tests.test_arm import rig, run  # noqa: F401  (fixture)


def test_voice_preset_completes_with_no_hand_ever(rig):
    r, arm, link, clock = rig
    run(r, arm, clock, None, 1.0)                     # empty room
    assert not arm.armed
    assert arm.voice("raise") and arm.armed
    run(r, arm, clock, None, 5.0)                     # still no hand: must not disarm, must keep moving
    assert arm.armed and arm.state in ("PRESET", "ARMED")
    assert abs(arm.pos["shoulder"] - 150) < 1 and abs(arm.pos["elbow"] - 120) < 1
    assert "ARM 0" not in link.log


def test_hand_lost_disarm_applies_once_a_hand_has_driven(rig):
    r, arm, link, clock = rig
    assert arm.voice("grab") and arm.armed             # grab, not a preset: a preset keeps playing hand or no hand
    run(r, arm, clock, None, 3.0)
    assert arm.armed                                  # no hand yet: nothing to lose
    run(r, arm, clock, "open", 0.5)                   # a hand shows up and is used
    run(r, arm, clock, None, 2.5)                     # ... and leaves for longer than lost_disarm_s
    assert not arm.armed and "ARM 0" in link.log


def test_voice_claw_with_no_hand(rig):
    r, arm, link, clock = rig
    assert arm.voice("grab") and link.log == ["ARM 1", "CLAW 1"]
    run(r, arm, clock, None, 1.0)
    assert arm.voice("release") and link.log[-1] == "CLAW 0" and arm.armed
