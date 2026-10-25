"""Voice command parsing, voice -> arm behaviour, and the pinch-vs-fist fix. No mic, no camera."""
import math

import numpy as np

from james_gestures import ArmConfig, ArmController, GestureReader, PrintLink
from james_gestures import handpose as HP
from james_gestures.voice import WAKE_WORDS, after_wake, parse, parse_result

W, H = 640, 480


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def hand(pose, x=320, y=400, size=120):
    return HP.synth(pose, (x, y), size)


def curled_pinch(index_ratio=1.18, gap=0.03):
    """Middle/ring/pinky folded, index bent to `index_ratio` (tip/knuckle distance), thumb tip `gap` palm-sizes
    from the index tip. This is the shape that used to read as a fist and send STOP."""
    pts = [np.array(p, float) for p in hand("fist")]
    w, k = pts[0], pts[5]
    d = pts[8] - w
    pts[8] = w + d / np.linalg.norm(d) * math.dist(w, k) * index_ratio
    pts[7] = (pts[8] + pts[6]) / 2
    palm = math.dist(pts[0], pts[9])
    pts[4] = pts[8] + np.array([-1.0, 0.0]) * gap * palm
    return [(float(x), float(y)) for x, y in pts]


def feed(reader, arm, clock, pts_or_pose, seconds, dt=1 / 30):
    g = None
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        pts = hand(pts_or_pose) if isinstance(pts_or_pose, str) else pts_or_pose
        g = reader.read(pts, W, H, t=clock.t)
        arm.update(g)
    return g


def rig(**cfg):
    clock = Clock()
    link = PrintLink(quiet=True)
    return GestureReader(clock=clock), ArmController(link, cfg=ArmConfig(mirror=False, **cfg), clock=clock), link, clock


# ---------- voice parsing ----------

def test_parse_words():
    assert parse("grab") == ["grab"]
    assert parse("please let go") == ["release"]
    assert parse("raise [unk] release") == ["raise", "release"]
    assert parse("hello there") == []
    assert parse("") == []


def test_stop_wins_over_everything():
    assert parse("grab stop raise") == ["stop"]
    assert parse("FREEZE") == ["stop"]


def vres(*pairs):
    """('word', conf) pairs -> a Vosk final result dict."""
    return {"text": " ".join(w for w, _ in pairs), "result": [{"word": w, "conf": c} for w, c in pairs]}


def test_low_confidence_dropped_but_stop_kept():
    assert parse_result(vres(("james", 1.0), ("grab", 0.3), ("stop", 0.2))) == ["stop"]
    assert parse_result(vres(("james", 1.0), ("grab", 0.3))) == []
    assert parse_result(vres(("james", 1.0), ("grab", 0.95))) == ["grab"]
    # the old bare-word behaviour is still there with wake=None
    assert parse_result(vres(("grab", 0.3), ("stop", 0.2)), wake=None) == ["stop"]
    assert parse_result(vres(("grab", 0.95)), wake=None) == ["grab"]


def test_no_wake_word_no_command():
    # what the hall noise produced, verbatim from tools/eval_voice.py: must all be ignored
    assert parse_result(vres(("let", 0.91), ("go", 0.91))) == []
    assert parse_result(vres(("open", 0.44), ("release", 0.81), ("open", 0.62), ("go", 1.0))) == []
    assert parse_result(vres(("[unk]", 0.85), ("open", 0.55), ("[unk]", 1.0), ("open", 0.8), ("up", 0.86))) == []
    assert parse_result(vres(("[unk]", 1.0), ("home", 0.9), ("[unk]", 1.0))) == []
    assert parse_result(vres(("stop", 1.0))) == []                        # even stop needs the wake word
    assert parse_result({"text": "home"}) == []                           # text-only fallback too
    assert parse_result({"text": "james home"}) == ["home"]


def test_wake_word_must_be_confident():
    assert parse_result(vres(("james", 0.5), ("grab", 1.0))) == []
    assert parse_result(vres(("james", 0.6), ("grab", 1.0))) == ["grab"]
    assert parse_result(vres(("[unk]", 1.0), ("james", 0.9), ("grab", 1.0), ("[unk]", 1.0))) == ["grab"]


def test_command_must_follow_wake_word_closely():
    assert WAKE_WORDS == ("james",)
    assert parse_result(vres(("james", 1.0), ("let", 1.0), ("go", 1.0))) == ["release"]
    assert parse_result(vres(("james", 1.0), ("[unk]", 1.0), ("grab", 1.0))) == ["grab"]
    # a command 4+ words later is someone else's sentence, not an order
    assert parse_result(vres(("james", 1.0), ("[unk]", 1.0), ("[unk]", 1.0), ("[unk]", 1.0), ("grab", 1.0))) == []
    # the command before the wake word does not count either
    assert parse_result(vres(("grab", 1.0), ("james", 1.0))) == []
    # two commands in one breath still work when they fit in the window
    assert parse_result(vres(("james", 1.0), ("grab", 1.0), ("release", 1.0))) == ["grab", "release"]


def test_after_wake_on_partial_text():
    # mirrors VoiceListener._partial_stop: the early stop only fires after the wake word
    assert after_wake("stop".split(), WAKE_WORDS) is None
    assert after_wake("[unk] open halt".split(), WAKE_WORDS) is None
    assert after_wake("james stop".split(), WAKE_WORDS) == ["stop"]
    assert after_wake("[unk] james [unk] stop".split(), WAKE_WORDS) == ["[unk]", "stop"]
    assert after_wake("stop".split(), None) == ["stop"]


# ---------- voice -> arm ----------

def test_voice_arms_by_itself_and_can_stop():
    r, arm, link, clock = rig()
    assert arm.voice("raise") and arm.armed and link.log[0] == "ARM 1"      # locked -> "james raise" arms, then goes
    assert arm.voice("grab") and link.log.count("ARM 1") == 1 and "CLAW 1" in link.log
    assert arm.voice("stop") and "STOP" in link.log and not arm.armed
    assert not arm.voice("follow") and not arm.armed                        # follow alone never arms


def test_voice_grab_is_not_undone_by_the_next_frame():
    r, arm, link, clock = rig()
    feed(r, arm, clock, "open", 1.5)
    assert arm.voice("grab")
    feed(r, arm, clock, "open", 0.5)                      # no pinch in view
    assert link.log.count("CLAW 1") == 1 and "CLAW 0" not in link.log
    arm.voice("release")
    assert link.log[-1] == "CLAW 0" or "CLAW 0" in link.log


def test_preset_pauses_steering_until_follow():
    r, arm, link, clock = rig()
    feed(r, arm, clock, "open", 1.5)
    assert arm.voice("raise")
    feed(r, arm, clock, hand("point", x=600, y=100), 3.0)  # pointing far away: must not drag the arm
    assert arm.state == "PRESET"
    assert abs(arm.pos["shoulder"] - 150) < 1e-6           # reached the preset
    assert arm.voice("follow") and arm.steering


def test_open_palm_takes_steering_back_after_a_preset():
    r, arm, link, clock = rig()
    assert arm.voice("home") and not arm.steering           # voice-armed, preset holds the arm
    feed(r, arm, clock, "open", 1.5)                        # palm held 1 s: the hand steers again
    assert arm.steering and arm.armed


def test_preset_keeps_playing_without_a_hand():
    r, arm, link, clock = rig()
    feed(r, arm, clock, "open", 1.5)
    arm.voice("raise")
    feed(r, arm, clock, None, 4.0)
    assert arm.armed and abs(arm.pos["shoulder"] - 150) < 1e-6


def test_rearming_never_opens_the_claw():
    r, arm, link, clock = rig()
    feed(r, arm, clock, "open", 1.5)
    arm.voice("grab")
    arm.voice("stop")
    feed(r, arm, clock, "open", 1.5)                      # re-arm
    assert arm.armed and "CLAW 0" not in link.log


def test_pinch_claw_can_be_turned_off():
    r, arm, link, clock = rig(pinch_claw=False)
    feed(r, arm, clock, "open", 1.5)
    feed(r, arm, clock, "pinch", 0.5)
    assert "CLAW 1" not in link.log


# ---------- pinch vs fist ----------

def test_thumb_near_index_is_not_a_fist():
    # thumb close to the index but not touching (not a pinch yet), other fingers curled
    for ratio in (1.15, 1.25, 1.4):
        for gap in (0.36, 0.45):
            r, t = GestureReader(), 0.0
            for _ in range(20):
                t += 1 / 30
                g = r.read(curled_pinch(ratio, gap), W, H, t=t)
            assert not g.fist, (ratio, gap)


def test_pinch_started_with_index_curled_is_not_a_fist():
    # the pinch begins with the index nearly folded (JAMES then ignores it as a pinch), then the index opens a bit
    r, t = GestureReader(), 0.0
    for ratio in [1.05] * 6 + [1.12, 1.18, 1.22, 1.25] + [1.25] * 20:
        t += 1 / 30
        g = r.read(curled_pinch(ratio, 0.03), W, H, t=t)
    assert not g.fist


def test_curled_pinch_does_not_stop_the_arm():
    r, arm, link, clock = rig()
    feed(r, arm, clock, "open", 1.5)
    feed(r, arm, clock, curled_pinch(1.05), 0.2)          # pinch starts with the index curled ...
    feed(r, arm, clock, curled_pinch(1.25), 1.5)          # ... then the index opens up a bit
    assert arm.armed and "STOP" not in link.log


def test_real_fist_still_stops_even_with_thumb_on_index():
    r, arm, link, clock = rig()
    feed(r, arm, clock, "open", 1.5)
    pts = hand("fist")
    pts[4] = pts[8]
    feed(r, arm, clock, pts, 0.6)
    assert not arm.armed and "STOP" in link.log
