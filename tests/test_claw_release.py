"""A held pinch that flickers for a frame must not open the claw (that would drop the object)."""
from james_gestures import ArmConfig, ArmController, PrintLink
from james_gestures import handpose as HP
from james_gestures.gestures import GestureFrame


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def frame(pinch: bool, gap: float, open_palm=False):
    pts = HP.synth("pinch" if pinch else "open", (320, 400), 120)
    return GestureFrame(hand=pts, palm_px=120, pinch=pinch, pinch_ratio=gap, open_palm=open_palm)


def armed_rig():
    clock, link = Clock(), PrintLink(quiet=True)
    arm = ArmController(link, cfg=ArmConfig(mirror=False), clock=clock)
    for _ in range(40):                       # 1.3 s of open palm -> ARM 1
        clock.t += 1 / 30
        arm.update(frame(False, 0.8, open_palm=True))
    assert arm.armed and "ARM 1" in link.log
    link.log.clear()
    return arm, link, clock


def claws(link):
    return [l for l in link.log if l.startswith("CLAW")]


def test_noisy_frame_during_pinch_keeps_the_claw_closed():
    arm, link, clock = armed_rig()
    clock.t += 1 / 30; arm.update(frame(True, 0.10))          # real pinch
    assert claws(link) == ["CLAW 1"]
    clock.t += 1 / 30; arm.update(frame(False, 0.20))         # pinch un-confirmed by noise, thumb still close
    clock.t += 1 / 30; arm.update(frame(True, 0.10))          # pinch confirmed again
    assert claws(link) == ["CLAW 1"], "no CLAW 0 and no duplicate CLAW 1"


def test_real_release_opens_the_claw():
    arm, link, clock = armed_rig()
    clock.t += 1 / 30; arm.update(frame(True, 0.10))
    clock.t += 1 / 30; arm.update(frame(False, 0.20))         # flicker
    clock.t += 1 / 30; arm.update(frame(False, 0.60))         # thumb really left the finger
    assert claws(link) == ["CLAW 1", "CLAW 0"]
    clock.t += 1 / 30; arm.update(frame(True, 0.10))          # pinch again -> closes again
    assert claws(link) == ["CLAW 1", "CLAW 0", "CLAW 1"]
