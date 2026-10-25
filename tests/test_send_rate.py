"""Joint updates go out at most send_period_s apart per joint, and the final position is always sent."""
from james_gestures import ArmConfig, ArmController, PrintLink


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def test_updates_are_rate_limited_and_end_exactly_on_target():
    clock, link = Clock(), PrintLink(quiet=True)
    arm = ArmController(link, cfg=ArmConfig(send_period_s=0.1), clock=clock)
    assert arm.voice("raise")                                   # arms, shoulder 90 -> 150 at 45 deg/s
    link.log.clear()
    for _ in range(90):                                         # 3 s at 30 fps
        clock.t += 1 / 30
        arm.tick()
    sh = [l for l in link.log if l.startswith("J shoulder")]
    assert 10 <= len(sh) <= 16, sh                              # ~1.33 s of motion at 10 Hz, not 40 messages
    assert sh[-1] == "J shoulder 150.0"                         # final value sent even if not 'due'


def test_default_period_keeps_old_behaviour_when_zero():
    clock, link = Clock(), PrintLink(quiet=True)
    arm = ArmController(link, cfg=ArmConfig(send_period_s=0.0), clock=clock)
    assert arm.voice("raise")
    link.log.clear()
    for _ in range(45):
        clock.t += 1 / 30
        arm.tick()
    sh = [l for l in link.log if l.startswith("J shoulder")]
    assert len(sh) >= 35                                        # every frame while moving
