"""Issue 2 software side: background hands can't drive, and the tuning log round-trips. No camera."""
import csv
import importlib.util
from pathlib import Path

from james_gestures import GestureReader
from james_gestures import handpose as HP
from james_gestures.handlog import HandLog, detected
from james_gestures.tracker import pick_driver

H = 480


def test_near_hand_drives():
    near = HP.synth("open", (320, 400), 90)           # ~0.19 of the frame height: someone at the laptop
    assert pick_driver([near], H) == 0


def test_background_hand_alone_does_not_drive():
    far = HP.synth("open", (100, 200), 15)            # ~0.03: someone across the hall
    assert pick_driver([far], H) is None


def test_largest_near_hand_still_wins_over_background():
    far, near = HP.synth("open", (100, 200), 15), HP.synth("point", (320, 400), 90)
    assert pick_driver([far, near], H) == 1


def test_min_hand_zero_turns_the_filter_off():
    far = HP.synth("open", (100, 200), 15)
    assert pick_driver([far], H, min_palm_frac=0) == 0


def _load_tuner():
    p = Path(__file__).resolve().parents[1] / "tools" / "tune_hands.py"
    spec = importlib.util.spec_from_file_location("tune_hands", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_log_round_trip_and_summary(tmp_path):
    path = tmp_path / "hands.csv"
    log, reader, t = HandLog(str(path)), GestureReader(), 0.0
    for key, pose in ((ord("1"), "open"), (ord("2"), "point")):
        assert log.key(key)
        for _ in range(30):
            t += 1 / 30
            g = reader.read(HP.synth(pose, (320, 400), 120), 640, H, t=t)
            log.write(t, g, reader.poses.raw, H)
    log.close()
    rows = list(csv.DictReader(open(path, newline="")))
    assert len(rows) == 60 and {r["label"] for r in rows} == {"open", "point"}
    late_point = [r for r in rows if r["label"] == "point"][-10:]
    assert all(r["detected"] == "point" for r in late_point)
    text = _load_tuner().summarise(rows)
    assert "open" in text and "point" in text and "recognised" in text


def test_detected_names_match_the_labels():
    reader, t = GestureReader(), 0.0
    for _ in range(40):
        t += 1 / 30
        g = reader.read(HP.synth("peace", (320, 400), 120), 640, H, t=t)
    assert detected(g) == "peace"
